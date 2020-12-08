unit tctypeparser;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, pastree, pscanner, pparser,
  tcbaseparser, testregistry;

type
  { TBaseTestTypeParser }

  TBaseTestTypeParser= Class(TTestParser)
  private
    FAddComment: Boolean;
    FType : TPasType;
    FHint : string;
    FErrorSource : String;
  Protected
    Function ParseType(ASource : String; ATypeClass : TClass;Const AHint : String = '') : TPasType; virtual; overload;
    Procedure AssertParseTypeError(ASource : String);
    Procedure AssertComment;
    procedure SetUp; override;
    Procedure TearDown; override;
    Property TheType : TPasType Read FType Write FType;
    Property Hint : string Read FHint Write FHint;
    Property AddComment : Boolean Read FAddComment Write FAddComment;
  end;

  { TTestTypeParser }

  TTestTypeParser = Class(TBaseTestTypeParser)
  private
  Protected
    procedure StartTypeHelper(ForType: String; AParent: String);
    Procedure DoTestAliasType(Const AnAliasType : String; Const AHint : String);
    procedure DoTestStringType(const AnAliasType: String; const AHint: String);
    procedure DoTypeError(Const AMsg,ASource : string);
    Procedure DoParseError;
    Procedure DoParsePointer(Const ASource : String; Const AHint : String; ADestType : TClass = Nil);
    Procedure DoParseArray(Const ASource : String; Const AHint : String; ADestType : TClass = Nil);
    Procedure DoParseEnumerated(Const ASource : String; Const AHint : String; ACount : integer);
    Procedure DoTestFileType(Const AType : String; Const AHint : String; ADestType : TClass = Nil);
    Procedure DoTestRangeType(Const AStart,AStop,AHint : String);
    Procedure DoParseSimpleSet(Const ASource : String; Const AHint : String; IsPacked : Boolean = False);
    Procedure DoParseComplexSet(Const ASource : String; Const AHint : String);
    procedure DoParseRangeSet(const ASource: String; const AHint: String);
    Procedure DoTestComplexSet;
    Procedure DoTestClassOf(Const AHint : string);
  Published
    Procedure TestAliasType;
    procedure TestAbsoluteAliasType;
    Procedure TestCrossUnitAliasType;
    Procedure TestAliasTypeDeprecated;
    Procedure TestAliasTypePlatform;
    Procedure TestSimpleTypeByte;
    Procedure TestSimpleTypeByteComment;
    Procedure TestSimpleTypeByteDeprecated;
    Procedure TestSimpleTypeBytePlatform;
    Procedure TestSimpleTypeBoolean;
    Procedure TestSimpleTypeBooleanDeprecated;
    Procedure TestSimpleTypeBooleanPlatform;
    Procedure TestSimpleTypeChar;
    Procedure TestSimpleTypeCharDeprecated;
    Procedure TestSimpleTypeCharPlatform;
    Procedure TestSimpleTypeInteger;
    Procedure TestSimpleTypeIntegerDeprecated;
    Procedure TestSimpleTypeIntegerPlatform;
    Procedure TestSimpleTypeInt64;
    Procedure TestSimpleTypeInt64Deprecated;
    Procedure TestSimpleTypeInt64Platform;
    Procedure TestSimpleTypeLongInt;
    Procedure TestSimpleTypeLongIntDeprecated;
    Procedure TestSimpleTypeLongIntPlatform;
    Procedure TestSimpleTypeLongWord;
    Procedure TestSimpleTypeLongWordDeprecated;
    Procedure TestSimpleTypeLongWordPlatform;
    Procedure TestSimpleTypeDouble;
    Procedure TestSimpleTypeDoubleDeprecated;
    Procedure TestSimpleTypeDoublePlatform;
    Procedure TestSimpleTypeShortInt;
    Procedure TestSimpleTypeShortIntDeprecated;
    Procedure TestSimpleTypeShortIntPlatform;
    Procedure TestSimpleTypeSmallInt;
    Procedure TestSimpleTypeSmallIntDeprecated;
    Procedure TestSimpleTypeSmallIntPlatform;
    Procedure TestSimpleTypeString;
    Procedure TestSimpleTypeStringDeprecated;
    Procedure TestSimpleTypeStringPlatform;
    Procedure TestSimpleTypeStringSize;
    Procedure TestSimpleTypeStringSizeIncomplete;
    Procedure TestSimpleTypeStringSizeWrong;
    Procedure TestSimpleTypeStringSizeDeprecated;
    Procedure TestSimpleTypeStringSizePlatform;
    Procedure TestSimpleTypeWord;
    Procedure TestSimpleTypeWordDeprecated;
    Procedure TestSimpleTypeWordPlatform;
    Procedure TestSimpleTypeQWord;
    Procedure TestSimpleTypeQWordDeprecated;
    Procedure TestSimpleTypeQWordPlatform;
    Procedure TestSimpleTypeCardinal;
    Procedure TestSimpleTypeCardinalDeprecated;
    Procedure TestSimpleTypeCardinalPlatform;
    Procedure TestSimpleTypeWideChar;
    Procedure TestSimpleTypeWideCharDeprecated;
    Procedure TestSimpleTypeWideCharPlatform;
    Procedure TestPointerSimple;
    procedure TestPointerSimpleDeprecated;
    procedure TestPointerSimplePlatform;
    Procedure TestStaticArray;
    Procedure TestStaticArrayComment;
    procedure TestStaticArrayDeprecated;
    procedure TestStaticArrayPlatform;
    Procedure TestStaticArrayPacked;
    Procedure TestStaticArrayTypedIndex;
    Procedure TestStaticArrayOfMethod;
    procedure TestStaticArrayOfProcedure;
    Procedure TestDynamicArray;
    Procedure TestDynamicArrayComment;
    procedure TestDynamicArrayOfMethod;
    procedure TestDynamicArrayOfProcedure;
    Procedure TestGenericArray;
    Procedure TestSimpleEnumerated;
    Procedure TestSimpleEnumeratedComment;
    Procedure TestSimpleEnumeratedComment2;
    Procedure TestSimpleEnumeratedDeprecated;
    Procedure TestSimpleEnumeratedPlatform;
    Procedure TestAssignedEnumerated;
    Procedure TestAssignedEnumeratedDeprecated;
    Procedure TestAssignedEnumeratedPlatform;
    Procedure TestFileType;
    Procedure TestFileTypeDeprecated;
    Procedure TestFileTypePlatform;
    Procedure TestRangeType;
    Procedure TestCharRangeType;
    Procedure TestCharRangeType2;
    Procedure TestRangeTypeDeprecated;
    Procedure TestRangeTypePlatform;
    Procedure TestIdentifierRangeType;
    Procedure TestIdentifierRangeTypeDeprecated;
    Procedure TestIdentifierRangeTypePlatform;
    Procedure TestNegativeIdentifierRangeType;
    Procedure TestSimpleSet;
    Procedure TestPackedSet;
    Procedure TestSimpleSetDeprecated;
    Procedure TestSimpleSetPlatform;
    Procedure TestComplexSet;
    Procedure TestComplexSetDeprecated;
    Procedure TestComplexSetPlatform;
    procedure TestRangeLowHigh;
    Procedure TestRangeSet;
    Procedure TestSubRangeSet;
    Procedure TestRangeSetDeprecated;
    Procedure TestRangeSetPlatform;
    Procedure TestNegativeRangeType;
    Procedure TestClassOf;
    Procedure TestClassOfComment;
    Procedure TestClassOfDeprecated;
    Procedure TestClassOfPlatform;
    Procedure TestReferenceAlias;
    Procedure TestReferenceSet;
    Procedure TestReferenceClassOf;
    Procedure TestReferenceFile;
    Procedure TestReferenceArray;
    Procedure TestReferencePointer;
    Procedure TestInvalidColon;
    Procedure TestTypeHelper;
    Procedure TestTypeHelperWithParent;
    procedure TestPointerReference;
    Procedure TestPointerKeyWord;
    Procedure TestPointerFile;
  end;

  { TTestRecordTypeParser }

  TTestRecordTypeParser = Class(TBaseTestTypeParser)
  private
    FDecl : TStrings;
    FAdvanced,
    FEnded,
    FStarted: boolean;
    FRecord: TPasRecordType;
    FMember1: TPasElement;
    function GetC(AIndex: Integer): TPasConst;
    Function GetField(AIndex : Integer; R : TPasRecordType) : TPasVariable;
    Function GetField(AIndex : Integer; R : TPasVariant) : TPasVariable;
    function GetF(AIndex: Integer): TPasVariable;
    function GetM(AIndex : Integer): TPasElement;
    Function GetVariant(AIndex : Integer; R : TPasRecordType) : TPasVariant;
    function GetV(AIndex: Integer): TPasVariant;
  Protected
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure StartRecord(Advanced: boolean = false);
    Procedure EndRecord(AEnd : String = 'end');
    Procedure AddMember(S : String);
    Procedure ParseRecord;
    Procedure ParseRecordFail(Msg: string; MsgNumber: integer);
    Procedure DoParseRecord;
    Procedure TestFields(Const Fields : Array of string; AHint : String; HaveVariant : Boolean = False);
    procedure AssertVariantSelector(AName, AType: string);
    procedure AssertConst1(Hints: TPasMemberHints; Index: integer = 1);
    procedure AssertField1(Hints: TPasMemberHints);
    procedure AssertField2(Hints: TPasMemberHints);
    procedure AssertMethod2(Hints: TPasMemberHints; isClass : Boolean = False);
    procedure AssertConstructor2(Hints: TPasMemberHints; isClass : Boolean = False);
    procedure AssertOperatorMethod2(Hints: TPasMemberHints; isClass : Boolean = False);
    procedure AssertVariant1(Hints: TPasMemberHints);
    procedure AssertVariant1(Hints: TPasMemberHints; VariantLabels : Array of string);
    procedure AssertVariant2(Hints: TPasMemberHints);
    procedure AssertVariant2(Hints: TPasMemberHints; VariantLabels : Array of string);
    procedure AssertOneIntegerField(Hints: TPasMemberHints);
    procedure AssertTwoIntegerFields(Hints1, Hints2: TPasMemberHints);
    procedure AssertIntegerFieldAndMethod(Hints1, Hints2: TPasMemberHints);
    procedure AssertIntegerFieldAndConstructor(Hints1, Hints2: TPasMemberHints);
    procedure AssertRecordField(AIndex: Integer;Hints: TPasMemberHints);
    procedure AssertRecordVariant(AIndex: Integer;Hints: TPasMemberHints; VariantLabels : Array of string);
    Procedure AssertRecordVariantVariant(AIndex: Integer;Const AFieldName,ATypeName: string;Hints: TPasMemberHints; VariantLabels : Array of string);
    Procedure DoTestEmpty(Const AHint : String);
    procedure DoTestDeprecatedVariantNoStorage(Const AHint : string);
    procedure DoTestDeprecatedVariantStorage(Const AHint : string);
    procedure DoTestVariantNoStorage(Const AHint : string);
    procedure DoTestVariantStorage(Const AHint : string);
    procedure DoTestTwoVariantsNoStorage(Const AHint : string);
    procedure DoTestTwoVariantsStorage(Const AHint : string);
    procedure DoTestTwoVariantsFirstDeprecatedStorage(Const AHint : string);
    procedure DoTestTwoVariantsSecondDeprecatedStorage(Const AHint : string);
    Procedure DoTestVariantTwoLabels(Const AHint : string);
    Procedure DoTestTwoVariantsTwoLabels(Const AHint : string);
    procedure DoTestVariantNestedRecord(Const AHint : string);
    procedure DoTestVariantNestedVariant(Const AHint : string);
    procedure DoTestVariantNestedVariantFirstDeprecated(Const AHint : string);
    procedure DoTestVariantNestedVariantSecondDeprecated(const AHint: string);
    procedure DoTestVariantNestedVariantBothDeprecated(const AHint: string);
    Property TheRecord : TPasRecordType Read FRecord;
    Property Advanced: boolean read FAdvanced;
    Property Const1 : TPasConst Index 0 Read GetC;
    Property Field1 : TPasVariable Index 0 Read GetF;
    Property Field2 : TPasVariable Index 1 Read GetF;
    Property Variant1 : TPasVariant Index 0 Read GetV;
    Property Variant2 : TPasVariant Index 1 Read GetV;
    Property Members[AIndex : Integer] : TPasElement Read GetM;
    Property Member1 : TPasElement Read FMember1;
  Published
    Procedure TestEmpty;
    Procedure TestEmptyComment;
    Procedure TestEmptyDeprecated;
    Procedure TestEmptyPlatform;
    Procedure TestOneField;
    Procedure TestOneFieldComment;
    Procedure TestOneFieldDeprecated;
    Procedure TestOneFieldPlatform;
    Procedure TestOneFieldSemicolon;
    Procedure TestOneFieldSemicolonDeprecated;
    Procedure TestOneFieldSemicolonPlatform;
    Procedure TestOneDeprecatedField;
    Procedure TestOneDeprecatedFieldDeprecated;
    Procedure TestOneDeprecatedFieldPlatform;
    Procedure TestOnePlatformField;
    Procedure TestOnePlatformFieldDeprecated;
    Procedure TestOnePlatformFieldPlatform;
    Procedure TestOneGenericField;
    Procedure TestTwoFields;
    procedure TestTwoFieldProtected;
    procedure TestTwoFieldStrictPrivate;
    procedure TestTwoFieldPrivateNoDelphi;
    Procedure TestTwoFieldPrivate;
    Procedure TestTwoFieldDeprecated;
    Procedure TestTwoFieldPlatform;
    Procedure TestTwoFieldsFirstDeprecated;
    Procedure TestTwoFieldsFirstDeprecatedDeprecated;
    Procedure TestTwoFieldsFirstDeprecatedPlatform;
    Procedure TestTwoFieldsSecondDeprecated;
    Procedure TestTwoFieldsSecondDeprecatedDeprecated;
    Procedure TestTwoFieldsSecondDeprecatedPlatform;
    Procedure TestTwoFieldsBothDeprecated;
    Procedure TestTwoFieldsBothDeprecatedDeprecated;
    Procedure TestTwoFieldsBothDeprecatedPlatform;
    Procedure TestTwoFieldsCombined;
    Procedure TestTwoFieldsCombinedDeprecated;
    Procedure TestTwoFieldsCombinedPlatform;
    Procedure TestTwoDeprecatedFieldsCombined;
    Procedure TestTwoDeprecatedFieldsCombinedDeprecated;
    Procedure TestTwoDeprecatedFieldsCombinedPlatform;
    procedure TestFieldAndConstructor;
    Procedure TestFieldAndMethod;
    Procedure TestFieldAnd2Methods;
    Procedure TestFieldAndProperty;
    Procedure TestFieldAndClassMethod;
    Procedure TestFieldAndClassOperator;
    Procedure TestFieldAndClassVar;
    Procedure TestFieldAndVar;
    Procedure TestNested;
    Procedure TestNestedDeprecated;
    Procedure TestNestedPlatform;
    procedure TestNestedSemicolon;
    procedure TestNestedSemicolonDeprecated;
    procedure TestNestedSemicolonPlatform;
    procedure TestNestedFirst;
    procedure TestNestedFirstDeprecated;
    procedure TestNestedFirstPlatform;
    Procedure TestDeprecatedNested;
    Procedure TestDeprecatedNestedDeprecated;
    Procedure TestDeprecatedNestedPlatform;
    procedure TestDeprecatedNestedFirst;
    procedure TestDeprecatedNestedFirstDeprecated;
    procedure TestDeprecatedNestedFirstPlatform;
    Procedure TestVariantNoStorage;
    procedure TestVariantNoStorageDeprecated;
    procedure TestVariantNoStoragePlatform;
    Procedure TestVariantStorage;
    procedure TestVariantStorageDeprecated;
    procedure TestVariantStoragePlatform;
    Procedure TestDeprecatedVariantNoStorage;
    procedure TestDeprecatedVariantNoStorageDeprecated;
    procedure TestDeprecatedVariantNoStoragePlatform;
    Procedure TestDeprecatedVariantStorage;
    procedure TestDeprecatedVariantStorageDeprecated;
    procedure TestDeprecatedVariantStoragePlatform;
    Procedure TestTwoVariantsNoStorage;
    procedure TestTwoVariantsNoStorageDeprecated;
    procedure TestTwoVariantsNoStoragePlatform;
    Procedure TestTwoVariantsStorage;
    procedure TestTwoVariantsStorageDeprecated;
    procedure TestTwoVariantsStoragePlatform;
    Procedure TestTwoVariantsFirstDeprecatedStorage;
    procedure TestTwoVariantsFirstDeprecatedStorageDeprecated;
    procedure TestTwoVariantsFirstDeprecatedStoragePlatform;
    Procedure TestTwoVariantsSecondDeprecatedStorage;
    procedure TestTwoVariantsSecondDeprecatedStorageDeprecated;
    procedure TestTwoVariantsSecondDeprecatedStoragePlatform;
    Procedure TestVariantTwoLabels;
    Procedure TestVariantTwoLabelsDeprecated;
    Procedure TestVariantTwoLabelsPlatform;
    Procedure TestTwoVariantsTwoLabels;
    Procedure TestTwoVariantsTwoLabelsDeprecated;
    Procedure TestTwoVariantsTwoLabelsPlatform;
    Procedure TestVariantNestedRecord;
    Procedure TestVariantNestedRecordDeprecated;
    Procedure TestVariantNestedRecordPlatform;
    Procedure TestVariantNestedVariant;
    Procedure TestVariantNestedVariantDeprecated;
    Procedure TestVariantNestedVariantPlatForm;
    Procedure TestVariantNestedVariantFirstDeprecated;
    Procedure TestVariantNestedVariantFirstDeprecatedDeprecated;
    Procedure TestVariantNestedVariantFirstDeprecatedPlatform;
    Procedure TestVariantNestedVariantSecondDeprecated;
    Procedure TestVariantNestedVariantSecondDeprecatedDeprecated;
    Procedure TestVariantNestedVariantSecondDeprecatedPlatform;
    Procedure TestVariantNestedVariantBothDeprecated;
    Procedure TestVariantNestedVariantBothDeprecatedDeprecated;
    Procedure TestVariantNestedVariantBothDeprecatedPlatform;
    Procedure TestOperatorField;
    Procedure TestPropertyFail;
    Procedure TestAdvRec_TwoConst;
    Procedure TestAdvRec_Property;
    Procedure TestAdvRec_PropertyImplementsFail;
    Procedure TestAdvRec_PropertyNoTypeFail;
    Procedure TestAdvRec_ForwardFail;
    Procedure TestAdvRec_PublishedFail;
    Procedure TestAdvRec_ProcVirtualFail;
    Procedure TestAdvRec_ProcOverrideFail;
    Procedure TestAdvRec_ProcMessageFail;
    Procedure TestAdvRec_DestructorFail;
    Procedure TestAdvRec_CaseInVar;
    Procedure TestAdvRec_EmptySections;
    Procedure TestAdvRecordInFunction;
    Procedure TestAdvRecordInAnonFunction;
    Procedure TestAdvRecordClassOperator;
    Procedure TestAdvRecordInitOperator;
    Procedure TestAdvRecordGenericFunction;
  end;

  { TTestProcedureTypeParser }
  TCallingConventionTest = Procedure (CC : TCallingConvention;Const AHint : String) of object;

  TTestProcedureTypeParser = Class(TBaseTestTypeParser)
  Private
    FProc : TPasProcedureType;
    procedure CheckArrayOfConstArgument(Aindex: Integer; Ac: TArgumentAccess);
  Protected
    procedure DoTestFunction(CC: TCallingConvention; const AHint: String);
    procedure DoTestFunctionOfObject(CC: TCallingConvention; const AHint: String);
    procedure DoTestFunctionOneArg(CC: TCallingConvention; const AHint: String);
    procedure DoTestFunctionOneArgOfObject(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOfObject(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOfObjectOneArg(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureIsNested(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureIsNestedOneArg(CC: TCallingConvention; const AHint: String);
    procedure CheckOpenArrayArgument(Ac: TArgumentAccess);
    procedure DoTestProcedureArrayOfConst(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOpenArray(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureConstOpenArray(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureVarOpenArray(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOutOpenArray(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOneArgDefault(CC: TCallingConvention;const AHint: String);
    procedure DoTestProcedureOneArgDefaultExpr(CC: TCallingConvention;const AHint: String);
    procedure DoTestProcedureOneArgDefaultSet(CC: TCallingConvention;const AHint: String);
    procedure DoTestProcedureOneConstArgDefault(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOneVarArgDefault(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureOneOutArgDefault(CC: TCallingConvention; const AHint: String);
    function CheckArgument(AIndex : Integer; Const AName,ATypeName : String; AAccess : TArgumentAccess) : TPasArgument;
    Function ParseType(ASource : String; CC : TCallingConvention; ATypeClass : TClass;Const AHint : String = '') : TPasProcedureType; virtual; overload;
    Procedure DoTestProcedureDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureOneArgDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureOneVarArgDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureOneConstArgDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureOneOutArgDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoVarArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoConstArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoOutArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoCombinedArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoCombinedVarArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoCombinedConstArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureTwoCombinedOutArgsDecl(CC : TCallingConvention; Const AHint : String);
    Procedure DoTestProcedureDefaultConstArgsDecl(CC : TCallingConvention; Const AHint : String);
    procedure DoTestProcedureUntypedArgDecl(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureUntypedConstArgDecl(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureUntypedOutArgDecl(CC: TCallingConvention; const AHint: String);
    procedure DoTestProcedureUntypedDefArg;
    Procedure TestCallingConventions(Proc : TCallingConventionTest; Const AHint : String);
    Procedure TestCallingConventions(Proc : TCallingConventionTest);
    Function FuncProc : TPasFunctionType;
    Property Proc : TPasProcedureType Read FProc;
  Published
    Procedure TestProcedure;
    Procedure TestProcedureComment;
    Procedure TestProcedureOneArg;
    Procedure TestProcedureOneVarArg;
    Procedure TestProcedureOneConstArg;
    Procedure TestProcedureOneOutArg;
    Procedure TestProcedureTwoArgs;
    Procedure TestProcedureTwoVarArgs;
    Procedure TestProcedureTwoConstArgs;
    Procedure TestProcedureTwoOutArgs;
    Procedure TestProcedureTwoCombinedArgs;
    Procedure TestProcedureTwoCombinedVarArgs;
    Procedure TestProcedureTwoCombinedConstArgs;
    Procedure TestProcedureTwoCombinedOutArgs;
    Procedure TestProcedureDefaultConstArgs;
    Procedure TestProcedureUntypedArg;
    Procedure TestProcedureUntypedConstArg;
    Procedure TestProcedureUntypedOutArg;
    Procedure TestProcedureUntypedDefArg;
    Procedure TestProcedureOneArgDefault;
    Procedure TestProcedureOneArgDefaultExpr;
    Procedure TestProcedureOneArgDefaultSet;
    Procedure TestProcedureOneVarArgDefault;
    Procedure TestProcedureOneConstArgDefault;
    Procedure TestProcedureOneOutArgDefault;
    Procedure TestProcedureNoMultiArgDefaults;
    Procedure TestProcedureOpenArray;
    Procedure TestProcedureConstOpenArray;
    Procedure TestProcedureOutOpenArray;
    Procedure TestProcedureVarOpenArray;
    Procedure TestProcedureArrayOfConst;
    Procedure TestProcedureReference;
    Procedure TestProcedureOfObject;
    Procedure TestProcedureOfObjectOneArg;
    Procedure TestProcedureIsNested;
    Procedure TestProcedureIsNesteOneArg;
    Procedure TestFunction;
    Procedure TestFunctionOneArg;
    Procedure TestFunctionOfObject;
    Procedure TestFunctionOneArgOfObject;
    Procedure TestCBlock;
    Procedure TestMacPasoutArg;
    Procedure TestMacPasPropertyArg;
    Procedure TestMacPasPropertyVarArg;
    Procedure TestMacPasClassArg;
  end;



implementation

uses typinfo;



{ TTestProcedureTypeParser }

procedure TTestProcedureTypeParser.DoTestProcedureUntypedArgDecl(
  CC: TCallingConvention; const AHint: String);

Var
  A : TPasArgument;

begin
  ParseType('procedure(var A)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','',argVar);
  AssertNull('No argument type', A.ArgType)
end;

procedure TTestProcedureTypeParser.DoTestProcedureUntypedConstArgDecl(
  CC: TCallingConvention; const AHint: String);

Var
  A : TPasArgument;

begin
  ParseType('procedure(const A)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','',argConst);
  AssertNull('No argument type', A.ArgType)
end;

procedure TTestProcedureTypeParser.DoTestProcedureUntypedOutArgDecl(
  CC: TCallingConvention; const AHint: String);
Var
  A : TPasArgument;

begin
  ParseType('procedure(out A)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','',argOut);
  AssertNull('No argument type', A.ArgType)
end;

procedure TTestProcedureTypeParser.DoTestProcedureUntypedDefArg;
begin
  ParseType('procedure(A)',ccdefault,TPasProcedureType,'');
end;

procedure TTestProcedureTypeParser.DoTestProcedureOneVarArgDefault(
  CC: TCallingConvention; const AHint: String);
Var
  A : TPasArgument;

begin
  ParseType('procedure(var A : Integer = 1)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','Integer',argVar);
  AssertNotNull('have default argument type', A.Value);
  AssertEquals('argument expr type', TPrimitiveExpr, A.ValueExpr.ClassType);
  AssertEquals('argument expr type', '1', TPrimitiveExpr(A.ValueExpr).Value);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOneOutArgDefault(
  CC: TCallingConvention; const AHint: String);
Var
  A : TPasArgument;

begin
  ParseType('procedure(out A : Integer = 1)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','Integer',argOut);
  AssertNotNull('have default argument type', A.Value);
  AssertEquals('argument expr type', TPrimitiveExpr, A.ValueExpr.ClassType);
  AssertEquals('argument expr type', '1', TPrimitiveExpr(A.ValueExpr).Value);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOneConstArgDefault(
  CC: TCallingConvention; const AHint: String);
Var
  A : TPasArgument;

begin
  ParseType('procedure(const A : Integer = 1)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','Integer',argConst);
  AssertNotNull('have default argument type', A.Value);
  AssertEquals('argument expr type', TPrimitiveExpr, A.ValueExpr.ClassType);
  AssertEquals('argument expr type', '1', TPrimitiveExpr(A.ValueExpr).Value);
end;

procedure TTestProcedureTypeParser.DoTestProcedureArrayOfConst(
  CC: TCallingConvention; const AHint: String);

begin
  ParseType('procedure(A : Array of const)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckArrayOfConstArgument(0,argDefault);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOfObject(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure of Object',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',0,Proc.Args.Count);
  AssertEquals('Is OF Object',True,Proc.IsOfObject);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOfObjectOneArg(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure (A : integer)of Object',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  AssertEquals('Is OF Object',True,Proc.IsOfObject);
  CheckArgument(0,'A','Integer',argDefault);
end;

procedure TTestProcedureTypeParser.DoTestProcedureIsNested(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure is nested',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',0,Proc.Args.Count);
  AssertEquals('Is nested',True,Proc.IsNested);
end;

procedure TTestProcedureTypeParser.DoTestProcedureIsNestedOneArg(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure (A : integer) is nested',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  AssertEquals('Is nested',True,Proc.IsNested);
  CheckArgument(0,'A','Integer',argDefault);
end;


procedure TTestProcedureTypeParser.CheckArrayOfConstArgument(Aindex : Integer; Ac : TArgumentAccess);
Var
  A : TPasArgument;
  T : TPasArrayType;

begin
  A:=CheckArgument(Aindex,'A','',ac);
  AssertEquals('ArrayType',TPasArrayType,A.ArgType.ClassType);
  T:=A.ArgType as TPasArrayType;
  AssertNull('Have Element type',T.ElType);
end;

procedure TTestProcedureTypeParser.DoTestFunction(CC: TCallingConvention;
  const AHint: String);
begin
  ParseType('function : integer',CC,TPasFunctionType,AHint);
  AssertEquals('Argument count',0,Proc.Args.Count);
  AssertEquals('Is OF Object',False,Proc.IsOfObject);
  AssertNotNull('Have result',FuncProc.ResultEl);
  AssertEquals('Result type class',TPasResultElement,FuncProc.ResultEl.ClassType);
  AssertNotNull('Have result',FuncProc.ResultEl.ResultType);
  AssertEquals('Result type element class ',TPasUnresolvedTypeRef,FuncProc.ResultEl.ResultType.ClassType);
  AssertEquals('Result type element name','Integer',FuncProc.ResultEl.ResultType.Name);
end;

procedure TTestProcedureTypeParser.DoTestFunctionOfObject(CC: TCallingConvention;
  const AHint: String);
begin
  ParseType('function : integer of object',CC,TPasFunctionType,AHint);
  AssertEquals('Argument count',0,Proc.Args.Count);
  AssertEquals('Is OF Object',True,Proc.IsOfObject);
  AssertNotNull('Have result',FuncProc.ResultEl);
  AssertEquals('Result type class',TPasResultElement,FuncProc.ResultEl.ClassType);
  AssertNotNull('Have result',FuncProc.ResultEl.ResultType);
  AssertEquals('Result type element class ',TPasUnresolvedTypeRef,FuncProc.ResultEl.ResultType.ClassType);
  AssertEquals('Result type element name','Integer',FuncProc.ResultEl.ResultType.Name);
end;

procedure TTestProcedureTypeParser.DoTestFunctionOneArg(CC: TCallingConvention;
  const AHint: String);
begin
  ParseType('function (A : Integer) : Integer',CC,TPasFunctionType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argDefault);
  AssertNotNull('Have result',FuncProc.ResultEl);
  AssertEquals('Result type class',TPasResultElement,FuncProc.ResultEl.ClassType);
  AssertNotNull('Have result',FuncProc.ResultEl.ResultType);
  AssertEquals('Result type element class ',TPasUnresolvedTypeRef,FuncProc.ResultEl.ResultType.ClassType);
  AssertEquals('Result type element name','Integer',FuncProc.ResultEl.ResultType.Name);
end;

procedure TTestProcedureTypeParser.DoTestFunctionOneArgOfObject(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('function (A : Integer) : Integer of object',CC,TPasFunctionType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  AssertEquals('Is OF Object',True,Proc.IsOfObject);
  CheckArgument(0,'A','Integer',argDefault);
  AssertNotNull('Have result',FuncProc.ResultEl);
  AssertEquals('Result type class',TPasResultElement,FuncProc.ResultEl.ClassType);
  AssertNotNull('Have result',FuncProc.ResultEl.ResultType);
  AssertEquals('Result type element class ',TPasUnresolvedTypeRef,FuncProc.ResultEl.ResultType.ClassType);
  AssertEquals('Result type element name','Integer',FuncProc.ResultEl.ResultType.Name);
end;

procedure TTestProcedureTypeParser.CheckOpenArrayArgument(Ac : TArgumentAccess);
Var
  A : TPasArgument;
  T : TPasArrayType;

begin
  A:=CheckArgument(0,'A','',ac);
  AssertEquals('ArrayType',TPasArrayType,A.ArgType.ClassType);
  T:=A.ArgType as TPasArrayType;
  AssertNotNull('Have Element type',T.ElType);
  AssertEquals('Element type',TPasUnresolvedTypeRef,T.ElType.ClassType);
  AssertEquals('Element type name','Integer',TPasUnresolvedTypeRef(T.ElType).Name);
  AssertEquals('No boundaries','',T.IndexRange);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOpenArray(
  CC: TCallingConvention; const AHint: String);

begin
  ParseType('procedure(A : Array of integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckOpenArrayArgument(argDefault);
end;

procedure TTestProcedureTypeParser.DoTestProcedureConstOpenArray(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure(const A : Array of integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckOpenArrayArgument(argConst);
end;

procedure TTestProcedureTypeParser.DoTestProcedureVarOpenArray(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure(var A : Array of integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckOpenArrayArgument(argVar);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOutOpenArray(
  CC: TCallingConvention; const AHint: String);
begin
  ParseType('procedure(out A : Array of integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckOpenArrayArgument(argOut);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOneArgDefault(
  CC: TCallingConvention; const AHint: String);
Var
  A : TPasArgument;

begin
  ParseType('procedure(A : Integer = 1)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','Integer',argDefault);
  AssertNotNull('have default argument type', A.ValueExpr);
  AssertEquals('argument expr type', TPrimitiveExpr, A.ValueExpr.ClassType);
  AssertEquals('argument expr value', '1', TPrimitiveExpr(A.ValueExpr).Value);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOneArgDefaultExpr(
  CC: TCallingConvention; const AHint: String);

Var
  A : TPasArgument;
  B : TBinaryExpr;

begin
  ParseType('procedure(A : Integer = 1+2)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','Integer',argDefault);
  AssertNotNull('have default argument type', A.ValueExpr);
  AssertEquals('argument expr type', TBinaryExpr, A.ValueExpr.ClassType);
  B:=TBinaryExpr(A.ValueExpr);
  AssertNotNull('have left expr', B.Left);
  AssertEquals('argument left expr type', TPrimitiveExpr, B.left.ClassType);
  AssertEquals('argument left expr value', '1', TPrimitiveExpr(B.Left).Value);
  AssertNotNull('have right expr', B.Right);
  AssertEquals('argument right expr type', TPrimitiveExpr, B.right.ClassType);
  AssertEquals('argument right expr value', '2', TPrimitiveExpr(B.right).Value);
  TAssert.AssertSame('B.left.parent=B',B,B.left.Parent);
  TAssert.AssertSame('B.right.parent=B',B,B.right.Parent);
end;

procedure TTestProcedureTypeParser.DoTestProcedureOneArgDefaultSet(
  CC: TCallingConvention; const AHint: String);
Var
  A : TPasArgument;
  B : TParamsExpr;

begin
  ParseType('procedure(A : TB = [])',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  A:=CheckArgument(0,'A','TB',argDefault);
  AssertNotNull('have default argument type', A.ValueExpr);
  AssertEquals('argument expr type', TParamsExpr, A.ValueExpr.ClassType);
  B:=TParamsExpr(A.ValueExpr);
  AssertEquals('No params',0,Length(B.Params));
end;

function TTestProcedureTypeParser.CheckArgument(AIndex: Integer; Const AName,
  ATypeName: String; AAccess: TArgumentAccess): TPasArgument;
Var
  A : TPasArgument;
  C : String;
begin
  C:='Argument '+IntToStr(AIndex)+' : ';
  AssertNotNull(C+'assigned',Proc.Args[AIndex]);
  AssertEquals(C+'class',TPasArgument,TObject(Proc.Args[AIndex]).ClassType);
  A:=TPasArgument(Proc.Args[AIndex]);
  AssertEquals(C+'Access',AAccess,A.Access);
  AssertEquals(C+'name',AName,A.Name);
  if (ATypeName<>'') then
    begin
    AssertNotNull(C+'type assigned',A.ArgType);
    if (ATypeName[1]='[') then
      AssertEquals(C+'type classname',LowerCase(Copy(ATypeName,2,Length(ATypeName)-2)),LowerCase(A.ArgType.ClassName))
    else
      AssertEquals(C+'type name',ATypeName,A.ArgType.Name);
    end;
  Result:=A;
end;

Function TTestProcedureTypeParser.ParseType(ASource: String;
  CC: TCallingConvention; ATypeClass: TClass; Const AHint: String
  ): TPasProcedureType;

Var
  CCS : String;

begin
  if CC=ccdefault then
    Result:=TPasProcedureType(ParseType(ASource,ATypeClass,AHint))
  else
    begin
    CCS:=cCallingConventions[CC];
    if (AHint<>'') then
      Result:=TPasProcedureType(ParseType(ASource+';' +CCS+';',ATypeClass,AHint))
    else
      Result:=TPasProcedureType(ParseType(ASource+';' +CCS,ATypeClass,AHint));
    end;
  FProc:=Result;
  AssertEquals('Correct calling convention for procedural type',cc,Result.CallingConvention);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureDecl(CC: TCallingConvention;
  Const AHint: String);

begin
  ParseType('procedure',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',0,Proc.Args.Count);
  if AddComment then
    AssertComment;
end;

Procedure TTestProcedureTypeParser.DoTestProcedureOneArgDecl(
  CC: TCallingConvention; Const AHint: String);

begin
  ParseType('procedure(A : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argDefault);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureOneVarArgDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(var A : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argVar);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureOneConstArgDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(const A : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argConst);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureOneOutArgDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(out A : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',1,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argOut);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(A : Integer;B : String)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argDefault);
  CheckArgument(1,'B','[TPasAliasType]',argDefault);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoVarArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(Var A : Integer;Var B : String)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argVar);
  CheckArgument(1,'B','[TPasAliasType]',argVar);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoConstArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(const A : Integer;Const B : String)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argConst);
  CheckArgument(1,'B','[TPasAliasType]',argConst);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoOutArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(out A : Integer;Out B : String)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argOut);
  CheckArgument(1,'B','[TPasAliasType]',argOut);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoCombinedArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(A,B : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argDefault);
  CheckArgument(1,'B','Integer',argDefault);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoCombinedVarArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(Var A,B : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argVar);
  CheckArgument(1,'B','Integer',argVar);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoCombinedConstArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(Const A,B : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argConst);
  CheckArgument(1,'B','Integer',argConst);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureTwoCombinedOutArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(Out A,B : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argOut);
  CheckArgument(1,'B','Integer',argOut);
end;

Procedure TTestProcedureTypeParser.DoTestProcedureDefaultConstArgsDecl(
  CC: TCallingConvention; Const AHint: String);
begin
  ParseType('procedure(A : Integer; Const B : Integer)',CC,TPasProcedureType,AHint);
  AssertEquals('Argument count',2,Proc.Args.Count);
  CheckArgument(0,'A','Integer',argDefault);
  CheckArgument(1,'B','Integer',argConst);
end;

Procedure TTestProcedureTypeParser.TestCallingConventions(
  Proc: TCallingConventionTest; Const AHint: String);

Var
  CC : TCallingConvention;

begin
  For cc:=ccDefault to High(TCallingConvention) do
    begin
    if CC<>ccDefault then
      Setup;
    try
      Proc(cc,AHint);
    finally
      tearDown;
    end;
    end;
end;

Procedure TTestProcedureTypeParser.TestCallingConventions(
  Proc: TCallingConventionTest);
begin
  TestCallingConventions(Proc,'');
  Setup;
  TestCallingConventions(Proc,'deprecated');
  Setup;
  TestCallingConventions(Proc,'platform');
end;

Function TTestProcedureTypeParser.FuncProc: TPasFunctionType;
begin
  Result:=Proc as TPasFunctionType;
end;

Procedure TTestProcedureTypeParser.TestProcedure;
begin
  TestCallingConventions(@DoTestProcedureDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureComment;
begin
  AddComment:=True;
  TestCallingConventions(@DoTestProcedureDecl);

end;

Procedure TTestProcedureTypeParser.TestProcedureOneArg;
begin
  TestCallingConventions(@DoTestProcedureOneArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneVarArg;
begin
  TestCallingConventions(@DoTestProcedureOneVarArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneConstArg;
begin
  TestCallingConventions(@DoTestProcedureOneConstArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneOutArg;
begin
  TestCallingConventions(@DoTestProcedureOneOutArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoVarArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoVarArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoConstArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoConstArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoOutArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoOutArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoCombinedArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoCombinedArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoCombinedVarArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoCombinedVarArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoCombinedConstArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoCombinedConstArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureTwoCombinedOutArgs;
begin
  TestCallingConventions(@DoTestProcedureTwoCombinedOutArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureDefaultConstArgs;
begin
  TestCallingConventions(@DoTestProcedureDefaultConstArgsDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureUntypedArg;
begin
  TestCallingConventions(@DoTestProcedureUntypedArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureUntypedConstArg;
begin
  TestCallingConventions(@DoTestProcedureUntypedConstArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureUntypedOutArg;
begin
  TestCallingConventions(@DoTestProcedureUntypedOutArgDecl);
end;

Procedure TTestProcedureTypeParser.TestProcedureUntypedDefArg;
begin
  AssertException('No untyped arg by value',EParserError,@DoTestProcedureUntypedDefArg)
end;

Procedure TTestProcedureTypeParser.TestProcedureOneArgDefault;
begin
  TestCallingConventions(@DoTestProcedureOneArgDefault);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneArgDefaultExpr;
begin
  TestCallingConventions(@DoTestProcedureOneArgDefaultExpr);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneArgDefaultSet;
begin
  TestCallingConventions(@DoTestProcedureOneArgDefaultSet);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneVarArgDefault;
begin
  TestCallingConventions(@DoTestProcedureOneVarArgDefault);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneConstArgDefault;
begin
  TestCallingConventions(@DoTestProcedureOneConstArgDefault);
end;

Procedure TTestProcedureTypeParser.TestProcedureOneOutArgDefault;
begin
  TestCallingConventions(@DoTestProcedureOneOutArgDefault);
end;

Procedure TTestProcedureTypeParser.TestProcedureNoMultiArgDefaults;
begin
  AssertParseTypeError('procedure (A,B : Integer = 1)');
end;

Procedure TTestProcedureTypeParser.TestProcedureOpenArray;
begin
  TestCallingConventions(@DoTestProcedureOpenArray);
end;

Procedure TTestProcedureTypeParser.TestProcedureConstOpenArray;
begin
  TestCallingConventions(@DoTestProcedureConstOpenArray);
end;

Procedure TTestProcedureTypeParser.TestProcedureOutOpenArray;
begin
  TestCallingConventions(@DoTestProcedureVarOpenArray);
end;

Procedure TTestProcedureTypeParser.TestProcedureVarOpenArray;
begin
  TestCallingConventions(@DoTestProcedureOutOpenArray);
end;

Procedure TTestProcedureTypeParser.TestProcedureArrayOfConst;
begin
  TestCallingConventions(@DoTestProcedureArrayOfConst);
end;

procedure TTestProcedureTypeParser.TestProcedureReference;
begin
  ParseType('reference to procedure',ccDefault,TPasProcedureType);
  AssertEquals('Argument count',0,Proc.Args.Count);
  AssertEquals('Is Reference to',True,Proc.IsReferenceTo);
end;

Procedure TTestProcedureTypeParser.TestProcedureOfObject;
begin
  TestCallingConventions(@DoTestProcedureOfObject);
end;

Procedure TTestProcedureTypeParser.TestProcedureOfObjectOneArg;
begin
  TestCallingConventions(@DoTestProcedureOfObjectOneArg);
end;

Procedure TTestProcedureTypeParser.TestProcedureIsNested;
begin
  TestCallingConventions(@DoTestProcedureIsNested);
end;

Procedure TTestProcedureTypeParser.TestProcedureIsNesteOneArg;
begin
  TestCallingConventions(@DoTestProcedureIsNestedOneArg);
end;

Procedure TTestProcedureTypeParser.TestFunction;
begin
  TestCallingConventions(@DoTestFunction);
end;

Procedure TTestProcedureTypeParser.TestFunctionOneArg;
begin
  TestCallingConventions(@DoTestFunctionOneArg);
end;

Procedure TTestProcedureTypeParser.TestFunctionOfObject;
begin
  TestCallingConventions(@DoTestFunctionOfObject);
end;

Procedure TTestProcedureTypeParser.TestFunctionOneArgOfObject;
begin
  TestCallingConventions(@DoTestFunctionOneArgOfObject);

end;

procedure TTestProcedureTypeParser.TestCBlock;


begin
  ParseType('reference to procedure (a: integer); cblock;',TPasProcedureType,'');
  FProc:=Definition as TPasProcedureType;
  AssertEquals('Argument count',1,Proc.Args.Count);
  AssertEquals('Is cblock',True,ptmCblock in Proc.Modifiers);
end;

procedure TTestProcedureTypeParser.TestMacPasoutArg;
begin
  Parser.CurrentModeswitches:=[msMac];
  ParseType('procedure (out: integer); ',TPasProcedureType,'');
  FProc:=Definition as TPasProcedureType;
  AssertEquals('Argument count',1,Proc.Args.Count);
end;

procedure TTestProcedureTypeParser.TestMacPasPropertyArg;
begin
  Parser.CurrentModeswitches:=[msMac];
  ParseType('procedure (property : integer); ',TPasProcedureType,'');
  FProc:=Definition as TPasProcedureType;
  AssertEquals('Argument count',1,Proc.Args.Count);
end;

procedure TTestProcedureTypeParser.TestMacPasPropertyVarArg;
begin
  Parser.CurrentModeswitches:=[msMac];
  ParseType('procedure (var property : integer); ',TPasProcedureType,'');
  FProc:=Definition as TPasProcedureType;
  AssertEquals('Argument count',1,Proc.Args.Count);
end;

procedure TTestProcedureTypeParser.TestMacPasClassArg;
begin
  Parser.CurrentModeswitches:=[msMac];
  ParseType('procedure (class : integer); ',TPasProcedureType,'');
  FProc:=Definition as TPasProcedureType;
  AssertEquals('Argument count',1,Proc.Args.Count);
end;

{ TTestRecordTypeParser }

function TTestRecordTypeParser.GetC(AIndex: Integer): TPasConst;
begin
  Result:=TObject(TheRecord.Members[AIndex]) as TPasConst;
end;

function TTestRecordTypeParser.GetField(AIndex: Integer; R: TPasRecordType
  ): TPasVariable;
begin
  AssertNotNull(R);
  AssertNotNull(R.Members);
  AssertTrue('Have AIndex elements',R.Members.Count>AIndex);
  AssertEquals('Correct class in member',TPasVariable,TObject(R.Members[AIndex]).ClassType);
  Result:=TPasVariable(R.Members[AIndex]);
end;

function TTestRecordTypeParser.GetField(AIndex: Integer; R: TPasVariant
  ): TPasVariable;
begin
  AssertNotNull(R);
  AssertNotNull('Have variant members', R.Members);
  AssertNotNull('Have variant members member list',R.Members.Members);
  AssertTrue('Have AIndex elements',R.Members.Members.Count>AIndex);
  AssertEquals('Correct class in member',TPasVariable,TObject(R.Members.members[AIndex]).ClassType);
  Result:=TPasVariable(R.Members.Members[AIndex]);
end;

function TTestRecordTypeParser.GetF(AIndex: Integer): TPasVariable;
begin
  Result:=GetField(AIndex,TheRecord);
end;

function TTestRecordTypeParser.GetM(AIndex : Integer): TPasElement;
begin
  AssertNotNull('Have Record',TheRecord);
  if (AIndex>=TheRecord.Members.Count) then
    Fail('No member '+IntToStr(AIndex));
  AssertNotNull('Have member'+IntToStr(AIndex),TheRecord.Members[AIndex]);
  If Not (TObject(TheRecord.Members[AIndex]) is TPasElement) then
    Fail('Member '+IntTostr(AIndex)+' is not a TPasElement');
  Result:=TPasElement(TheRecord.Members[AIndex])
end;

function TTestRecordTypeParser.GetVariant(AIndex: Integer; R: TPasRecordType
  ): TPasVariant;
begin
  AssertNotNull(R);
  AssertNotNull(R.Variants);
  AssertTrue('Have AIndex variant elements',R.Variants.Count>AIndex);
  AssertEquals('Correct class in variant',TPasVariant,TObject(R.Variants[AIndex]).ClassType);
  Result:=TPasVariant(R.Variants[AIndex]);
end;

function TTestRecordTypeParser.GetV(AIndex: Integer): TPasVariant;
begin
  Result:=GetVariant(AIndex,TheRecord);
end;

procedure TTestRecordTypeParser.SetUp;
begin
  inherited SetUp;
  FDecl:=TStringList.Create;
  FStarted:=false;
  FEnded:=false;
end;

procedure TTestRecordTypeParser.TearDown;
begin
  FreeAndNil(FDecl);
  inherited TearDown;
end;

procedure TTestRecordTypeParser.StartRecord(Advanced: boolean);
var
  S: String;
begin
  if FStarted then
    Fail('TTestRecordTypeParser.StartRecord already started');
  FStarted:=True;
  S:='TMyRecord = record';
  if Advanced then
    S:='{$modeswitch advancedrecords}'+sLineBreak+S;
  FDecl.Add(S);
end;

procedure TTestRecordTypeParser.EndRecord(AEnd: String);
begin
  if FEnded then exit;
  if not FStarted then
    StartRecord;
  FEnded:=True;
  if (AEnd<>'') then
    FDecl.Add('  '+AEnd);
end;

procedure TTestRecordTypeParser.AddMember(S: String);
begin
  if Not FStarted then
    StartRecord;
  FDecl.Add('    '+S);
end;

procedure TTestRecordTypeParser.ParseRecord;
begin
  DoParseRecord;
end;

procedure TTestRecordTypeParser.ParseRecordFail(Msg: string; MsgNumber: integer
  );
var
  ok: Boolean;
begin
  ok:=false;
  try
    ParseRecord;
  except
    on E: EParserError do
      begin
      AssertEquals('Expected {'+Msg+'} '+IntToStr(MsgNumber)+', but got msg {'+Parser.LastMsg+'} '+IntToStr(Parser.LastMsgNumber),MsgNumber,Parser.LastMsgNumber);
      AssertEquals('Expected {'+Msg+'}, but got msg {'+Parser.LastMsg+'}',Msg,Parser.LastMsg);
      ok:=true;
      end;
  end;
  AssertEquals('Missing parser error {'+Msg+'} ('+IntToStr(MsgNumber)+')',true,ok);
end;

procedure TTestRecordTypeParser.DoParseRecord;
begin
  EndRecord;
  Add('Type');
  if AddComment then
    begin
    Add('// A comment');
    Engine.NeedComments:=True;
    end;
  Add('  '+TrimRight(FDecl.Text)+';');
  ParseDeclarations;
  AssertEquals('One record type definition',1,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasRecordType,TObject(Declarations.Types[0]).ClassType);
  FRecord:=TObject(Declarations.Types[0]) as TPasRecordType;
  TheType:=FRecord; // needed by AssertComment
  Definition:=TheType; // needed by CheckHint
  if TheRecord.Members.Count>0 then
    FMember1:=TObject(TheRecord.Members[0]) as TPasElement;
end;

procedure TTestRecordTypeParser.TestFields(const Fields: array of string;
  AHint: String; HaveVariant: Boolean);

Var
  S : String;
  I : integer;

begin
  StartRecord;
  For I:=Low(Fields) to High(Fields) do
    AddMember(Fields[i]);
  S:='end';
  if AHint<>'' then
    S:=S+' '+AHint;
  EndRecord(S);
  ParseRecord;
  if HaveVariant then
    begin
    AssertNotNull('Have variants',TheRecord.Variants);
    AssertNotNull('Have variant type',TheRecord.VariantEl);
    end
  else
    begin
    AssertNull('No variants',TheRecord.Variants);
    AssertNull('No variant element',TheRecord.VariantEl);
    end;
  if AddComment then
    AssertComment;
  if (AHint<>'') then
    CheckHint(TPasMemberHint(GetEnumValue(TypeInfo(TPasMemberHint),'h'+AHint)));
end;

procedure TTestRecordTypeParser.AssertVariantSelector(AName,AType : string);

var
  V: TPasVariable;
begin
  AssertNotNull('Have variant element',TheRecord.VariantEl);
  if AName<>'' then
    begin
    AssertEquals('Have variant variable',TPasVariable,TheRecord.VariantEl.ClassType);
    V:=TPasVariable(TheRecord.VariantEl);
    AssertEquals('Have variant variable name',AName,V.Name);
    AssertNotNull('Have variant var type',V.VarType);
    AssertEquals('Have variant selector type',TPasUnresolvedTypeRef,V.VarType.ClassType);
    AssertEquals('Have variant selector type name',lowercase(AType),lowercase(V.VarType.Name));
    end else begin
    AssertEquals('Have variant selector type',TPasUnresolvedTypeRef,TheRecord.VariantEl.ClassType);
    AssertEquals('Have variant selector type name',lowercase(AType),lowercase(TheRecord.VariantEl.Name));
    end;
end;

procedure TTestRecordTypeParser.AssertConst1(Hints: TPasMemberHints;
  Index: integer);
begin
  if Hints=[] then ;
  AssertEquals('Member '+IntToStr(Index+1)+' type',TPasConst,TObject(TheRecord.Members[Index]).ClassType);
  AssertEquals('Const '+IntToStr(Index+1)+' name','x',Const1.Name);
  AssertNotNull('Have '+IntToStr(Index+1)+' const expr',Const1.Expr);
end;

procedure TTestRecordTypeParser.DoTestEmpty(const AHint: String);
begin
  TestFields([],AHint);
  AssertNotNull('Have members array',TheRecord.Members);
  AssertEquals('Zero members in array',0,TheRecord.Members.Count);
end;

procedure TTestRecordTypeParser.AssertVariant1(Hints: TPasMemberHints);
begin
  AssertVariant1(Hints,['0']);
end;

procedure TTestRecordTypeParser.AssertVariant1(Hints: TPasMemberHints;
  VariantLabels: array of string);

Var
  I : Integer;

begin
  AssertNotNull('Have variant 1',Variant1);
  AssertNotNull('Variant 1 has Values ',Variant1.Values);
  if Length(VariantLabels)=0 then
    begin
    AssertEquals('Have 1 value',1,Variant1.Values.Count);
    AssertNotNull('Assigned value',Variant1.Values[0]);
    AssertEquals('Expression',TPrimitiveExpr,TObject(Variant1.Values[0]).CLassType);
    AssertExpression('First value is 0',TPasExpr(Variant1.Values[0]),pekNumber,'0');
    end
  else
    begin
    AssertEquals('Have correct number of values',Length(VariantLabels),Variant1.Values.Count);
    For I:=0 to Length(VariantLabels)-1 do
      begin
      AssertEquals(Format('Expression for variant %d',[I]),TPrimitiveExpr,TObject(Variant1.Values[0]).CLassType);
      AssertExpression(Format('Value %d is %s',[i,VariantLabels[i]]),TPasExpr(Variant1.Values[I]),pekNumber,VariantLabels[i]);
      end;
    end;
  AssertNotNull('Have members',Variant1.Members);
  AssertNotNull('Have member members',Variant1.Members.Members);
  AssertNotNull('member 0 not null',Variant1.Members.Members[0]);
  AssertEquals('Member 0 has correct name',TPasVariable,TObject(Variant1.Members.Members[0]).ClassType);
  AssertEquals('Member 0 has correct name','y',TPasVariable(Variant1.Members.Members[0]).Name);
  AssertNotNull('member 0 has not null type',TPasVariable(Variant1.Members.Members[0]).VarType);
  AssertEquals('member 0 has correct type',TPasUnresolvedTypeRef,TPasVariable(Variant1.Members.Members[0]).VarType.ClassType);
  AssertEquals('member 0 has correct type name','Integer',TPasVariable(Variant1.Members.Members[0]).VarType.Name);
  AssertTrue('Field 1 hints match',TPasVariable(Variant1.Members.Members[0]).Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertVariant2(Hints: TPasMemberHints);
begin
  AssertVariant2(Hints,['1']);
end;

procedure TTestRecordTypeParser.AssertVariant2(Hints: TPasMemberHints;
  VariantLabels: array of string);

Var
  I : Integer;

begin
  AssertNotNull('Have variant 2',Variant2);
  AssertNotNull('Variant 2 has Values ',Variant2.Values);
  if Length(VariantLabels)=0 then
    begin
    AssertEquals('Variant 2 has 1 value',2,Variant2.Values.Count);
    AssertEquals('Expression',TPrimitiveExpr,TObject(Variant2.Values[0]).CLassType);
    AssertExpression('First value is 1',TPasExpr(Variant2.Values[0]),pekNumber,'1');
    end
  else
    begin
    AssertEquals('Variant 2 Has correct number of values',Length(VariantLabels),Variant2.Values.Count);
    For I:=0 to Length(VariantLabels)-1 do
      begin
      AssertEquals(Format('Expression for variant %d',[I]),TPrimitiveExpr,TObject(Variant2.Values[I]).CLassType);
      AssertExpression(Format('Value %d is %s',[i,VariantLabels[i]]),TPasExpr(Variant2.Values[I]),pekNumber,VariantLabels[i]);
//      AssertEquals(Format('Variant 2, Value %d is %s',[i,VariantLabels[i]]),VariantLabels[i],Variant2.Values[I]);
      end;
    end;
  AssertNotNull('Have members',Variant2.Members);
  AssertNotNull('Have member members',Variant2.Members.Members);
  AssertNotNull('member 1 not null',Variant2.Members.Members[0]);
  AssertEquals('Member 1 has correct name',TPasVariable,TObject(Variant2.Members.Members[0]).ClassType);
  AssertEquals('Member 1 has correct name','z',TPasVariable(Variant2.Members.Members[0]).Name);
  AssertNotNull('member 1 has not null type',TPasVariable(Variant2.Members.Members[0]).VarType);
  AssertEquals('member 1 has correct type',TPasUnresolvedTypeRef,TPasVariable(Variant2.Members.Members[0]).VarType.ClassType);
  AssertEquals('member 1 has correct type name','Integer',TPasVariable(Variant2.Members.Members[0]).VarType.Name);
  AssertTrue('Field 1 hints match',TPasVariable(Variant2.Members.Members[0]).Hints=Hints)
end;

procedure TTestRecordTypeParser.DoTestVariantNoStorage(const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : (y : integer;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertVariant1([]);
end;

procedure TTestRecordTypeParser.DoTestDeprecatedVariantNoStorage(
  const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : (y : integer deprecated;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertVariant1([hDeprecated]);
end;

procedure TTestRecordTypeParser.DoTestDeprecatedVariantStorage(
  const AHint: string);
begin
  TestFields(['x : integer;','case s : integer of','0 : (y : integer deprecated;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('s','integer');
  AssertVariant1([hDeprecated]);
end;

procedure TTestRecordTypeParser.DoTestVariantStorage(const AHint: string);
begin
  TestFields(['x : integer;','case s : integer of','0 : (y : integer;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('s','integer');
  AssertVariant1([]);
end;

procedure TTestRecordTypeParser.DoTestTwoVariantsNoStorage(const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : (y : integer;);','1 : (z : integer;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertVariant1([]);
  AssertVariant2([]);
end;

procedure TTestRecordTypeParser.DoTestTwoVariantsStorage(const AHint: string);
begin
  TestFields(['x : integer;','case s : integer of','0 : (y : integer;);','1 : (z : integer;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('s','integer');
  AssertVariant1([]);
  AssertVariant2([]);
end;

procedure TTestRecordTypeParser.DoTestTwoVariantsFirstDeprecatedStorage(
  const AHint: string);
begin
  TestFields(['x : integer;','case s : integer of','0 : (y : integer deprecated;);','1 : (z : integer;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('s','integer');
  AssertVariant1([hdeprecated]);
  AssertVariant2([]);
end;

procedure TTestRecordTypeParser.DoTestTwoVariantsSecondDeprecatedStorage(
  const AHint: string);
begin
  TestFields(['x : integer;','case s : integer of','0 : (y : integer ;);','1 : (z : integer deprecated;)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('s','integer');
  AssertVariant1([]);
  AssertVariant2([hdeprecated]);
end;

procedure TTestRecordTypeParser.DoTestVariantTwoLabels(const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0,1 : (y : integer)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertVariant1([],['0','1']);
end;

procedure TTestRecordTypeParser.DoTestTwoVariantsTwoLabels(const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0,1 : (y : integer);','2,3 : (z : integer);'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertVariant1([],['0','1']);
  AssertVariant2([],['2','3']);
end;

procedure TTestRecordTypeParser.DoTestVariantNestedRecord(const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : ( y : record','  z : integer;','end)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertRecordVariant(0,[],['0']);
end;

procedure TTestRecordTypeParser.DoTestVariantNestedVariant(const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : ( y : record','  z : integer;','  case byte of ','    1 : (i : integer);','    2 : ( j :  byte)', 'end)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertRecordVariant(0,[],['0']);
  AssertRecordVariantVariant(0,'i','Integer',[],['1']);
  AssertRecordVariantVariant(1,'j','Byte',[],['2'])
end;

procedure TTestRecordTypeParser.DoTestVariantNestedVariantFirstDeprecated(
  const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : ( y : record','  z : integer;','  case byte of ','    1 : (i : integer deprecated);','    2 : ( j :  byte)', 'end)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertRecordVariant(0,[],['0']);
  AssertRecordVariantVariant(0,'i','Integer',[hDeprecated],['1']);
  AssertRecordVariantVariant(1,'j','Byte',[],['2'])
end;

procedure TTestRecordTypeParser.DoTestVariantNestedVariantSecondDeprecated(
  const AHint: string);
begin
  TestFields(['x : integer;','case integer of','0 : ( y : record','  z : integer;','  case byte of ','    1 : (i : integer );','    2 : ( j :  byte deprecated)', 'end)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertRecordVariant(0,[],['0']);
  AssertRecordVariantVariant(0,'i','Integer',[],['1']);
  AssertRecordVariantVariant(1,'j','Byte',[hDeprecated],['2'])
end;

procedure TTestRecordTypeParser.DoTestVariantNestedVariantBothDeprecated(const AHint: string);

begin
  TestFields(['x : integer;','case integer of','0 : ( y : record','  z : integer;','  case byte of ','    1 : (i : integer deprecated );','    2 : ( j :  byte deprecated)', 'end)'],AHint,True);
  AssertField1([]);
  AssertVariantSelector('','integer');
  AssertRecordVariant(0,[],['0']);
  AssertRecordVariantVariant(0,'i','Integer',[hdeprecated],['1']);
  AssertRecordVariantVariant(1,'j','Byte',[hDeprecated],['2'])
end;

procedure TTestRecordTypeParser.TestEmpty;
begin
  DoTestEmpty('')
end;

procedure TTestRecordTypeParser.TestEmptyComment;
begin
  AddComment:=True;
  TestEmpty;
end;

procedure TTestRecordTypeParser.TestEmptyDeprecated;
begin
  DoTestEmpty('Deprecated')
end;

procedure TTestRecordTypeParser.TestEmptyPlatform;
begin
  DoTestEmpty('Platform')
end;

procedure TTestRecordTypeParser.AssertField1(Hints : TPasMemberHints);

begin
  AssertEquals('Member 1 field type',TPasVariable,TObject(TheRecord.Members[0]).ClassType);
  AssertEquals('Field 1 name','x',Field1.Name);
  AssertNotNull('Have 1 Field type',Field1.VarType);
  AssertEquals('Field 1 type',TPasUnresolvedTypeRef,Field1.VarType.ClassType);
  AssertEquals('Field 1 type name','Integer',Field1.VarType.Name);
  AssertTrue('Field 1 hints match',Field1.Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertField2(Hints : TPasMemberHints);

begin
  AssertEquals('Member 2 field type',TPasVariable,TObject(TheRecord.Members[1]).ClassType);
  AssertEquals('Field 2 name','y',Field2.Name);
  AssertNotNull('Have 2 Field type',Field2.VarType);
  AssertEquals('Field 2 type',TPasUnresolvedTypeRef,Field2.VarType.ClassType);
  AssertEquals('Field 2 type name','Integer',Field2.VarType.Name);
  AssertTrue('Field 2 hints match',Field2.Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertMethod2(Hints: TPasMemberHints; isClass : Boolean = False);

Var
  P : TPasProcedure;

begin
  if IsClass then
    AssertEquals('Member 2 type',TPasClassProcedure,TObject(TheRecord.Members[1]).ClassType)
  else
    AssertEquals('Member 2 type',TPasProcedure,TObject(TheRecord.Members[1]).ClassType);
  P:=TPasProcedure(TheRecord.Members[1]);
  AssertEquals('Method name','dosomething2',P.Name);
  AssertTrue('Method hints match',P.Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertConstructor2(Hints: TPasMemberHints;
  isClass: Boolean);
Var
  P : TPasProcedure;

begin
  if IsClass then
    AssertEquals('Member 2 type',TPasClassConstructor,TObject(TheRecord.Members[1]).ClassType)
  else
    AssertEquals('Member 2 type',TPasConstructor,TObject(TheRecord.Members[1]).ClassType);
  P:=TPasProcedure(TheRecord.Members[1]);
  AssertEquals('Constructor name','create',P.Name);
  AssertTrue('Constructor hints match',P.Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertOperatorMethod2(Hints: TPasMemberHints;
  isClass: Boolean);
Var
  P : TPasOperator;

begin
  if IsClass then
    AssertEquals('Member 2 type',TPasClassOperator,TObject(TheRecord.Members[1]).ClassType)
  else
    AssertEquals('Member 2 type',TPasOperator,TObject(TheRecord.Members[1]).ClassType);
  P:=TPasOperator(TheRecord.Members[1]);
  AssertEquals('Method name','assign(ta,Cardinal):Boolean',P.Name);

  AssertTrue('Method hints match',P.Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertOneIntegerField(Hints : TPasMemberHints);

begin
  AssertEquals('One field',1,TheRecord.Members.Count);
  AssertField1(Hints);
end;

procedure TTestRecordTypeParser.AssertTwoIntegerFields(Hints1,Hints2: TPasMemberHints);

begin
  AssertEquals('Two field',2,TheRecord.Members.Count);
  AssertField1(Hints1);
  AssertField2(Hints2);
end;

procedure TTestRecordTypeParser.AssertIntegerFieldAndMethod(Hints1,
  Hints2: TPasMemberHints);
begin
  AssertEquals('Two members',2,TheRecord.Members.Count);
  AssertField1(Hints1);
  AssertMethod2(Hints2);
end;

procedure TTestRecordTypeParser.AssertIntegerFieldAndConstructor(Hints1,
  Hints2: TPasMemberHints);
begin
  AssertEquals('Two members',2,TheRecord.Members.Count);
  AssertField1(Hints1);
  AssertConstructor2(Hints2);
end;

procedure TTestRecordTypeParser.AssertRecordField(AIndex: Integer;
  Hints: TPasMemberHints);

Var
  F : TPasVariable;
  R : TPasRecordtype;

begin
  AssertEquals('Member 2 field type',TPasVariable,TObject(TheRecord.Members[AIndex]).ClassType);
  F:=GetF(AIndex);
  if AIndex=1 then
    AssertEquals('Field 2 name','y',F.Name)
  else
    AssertEquals('Field 1 name','x',F.Name);
  AssertNotNull('Have 2 Field type',F.VarType);
  AssertEquals('Field 2 type',TPasRecordType,F.VarType.ClassType);
  R:=F.VarType as TPasRecordType;
  AssertNotNull('Record field has members',R.Members);
  AssertEquals('Record field has 1 member',1,R.Members.Count);
  AssertTrue('Record field hints match',F.Hints=Hints)
end;

procedure TTestRecordTypeParser.AssertRecordVariant(AIndex: Integer;
  Hints: TPasMemberHints; VariantLabels: array of string);

Var
  F : TPasVariant;
  V : TPasVariable;
  R : TPasRecordtype;
  I : Integer;
  MN : String;

begin
  F:=GetV(AIndex);
  MN:='Variant '+IntToStr(AIndex)+' ';
  AssertNotNull('Have variant 1',F);
  AssertEquals('Have correct number of values',Length(VariantLabels),F.Values.Count);
  For I:=0 to Length(VariantLabels)-1 do
    begin
    AssertEquals(Format('Expression for variant %d',[I]),TPrimitiveExpr,TObject(Variant1.Values[i]).CLassType);
    AssertExpression(Format('Value %d is %s',[i,VariantLabels[i]]),TPasExpr(Variant1.Values[I]),pekNumber,VariantLabels[i]);
    end;
  V:=GetField(0,F);
  AssertEquals(MN+'has correct name','y',V.Name);
  AssertNotNull(MN+'has not null type',V.VarType);
  AssertEquals(MN+'has correct type',TPasRecordType,V.VarType.ClassType);
  AssertTrue(MN+'hints match',V.Hints=Hints);
  R:=TPasVariable(F.Members.Members[0]).VarType as TPasRecordType;
  V:=GetField(0,R);
  AssertEquals('Field 1 has correct name','z',V.Name);
  AssertNotNull('Record field has members',R.Members);
  AssertEquals('Record field has 1 member',1,R.Members.Count);

end;

procedure TTestRecordTypeParser.AssertRecordVariantVariant(AIndex: Integer;
  const AFieldName, ATypeName: string; Hints: TPasMemberHints;
  VariantLabels: array of string);

Var
  F : TPasVariant;
  V : TPasVariable;
  R : TPasRecordtype;
  I : Integer;
  MN : String;

begin
  F:=GetV(0);
  MN:='Nested Variant '+IntToStr(AIndex)+' ';
  AssertNotNull('Have variant 1',F);
  AssertEquals('Have correct number of values',1,F.Values.Count);
  AssertEquals('Expression',TPrimitiveExpr,TObject(F.Values[0]).CLassType);
  AssertExpression('First value is 0',TPasExpr(F.Values[0]),pekNumber,'0');
  // First variant, Y, record
  V:=GetField(0,F);
  AssertEquals(MN+'has correct name','y',V.Name);
  AssertNotNull(MN+'has not null type',V.VarType);
  AssertEquals(MN+'has correct type',TPasRecordType,V.VarType.ClassType);
  R:=TPasVariable(F.Members.Members[0]).VarType as TPasRecordType;
  AssertNotNull('Record field has members',R.Members);
  AssertEquals('Record field has 2 members',1,R.Members.Count);
  // First variant
  F:=GetVariant(Aindex,R);
  // First field of first variant, i
  AssertEquals('Have correct number of values',Length(VariantLabels),F.Values.Count);
  For I:=0 to Length(VariantLabels)-1 do
    begin
    AssertEquals(Format('Expression for variant %d',[I]),TPrimitiveExpr,TObject(F.Values[i]).CLassType);
    AssertExpression(Format('Value %d is %s',[i,VariantLabels[i]]),TPasExpr(F.Values[I]),pekNumber,VariantLabels[i]);
    end;
  V:=GetField(0,F);
  AssertEquals('Nested Variant 0 has correct name',AFieldName,V.Name);
  AssertEquals('Nested variant 0 has correct type',TPasUnresolvedTypeRef,V.VarType.ClassType);
  AssertEquals('Nested variant 0 has correct type name',ATypeName,V.VarType.Name);
  AssertTrue(MN+'hints match',V.Hints=Hints);
end;

procedure TTestRecordTypeParser.TestOneField;
begin
  TestFields(['x : integer'],'',False);
  AssertOneIntegerField([]);
end;

procedure TTestRecordTypeParser.TestOneFieldComment;
begin
  AddComment:=True;
  TestFields(['{a} x : integer'],'',False);
  AssertOneIntegerField([]);
  AssertEquals('Member 1 comment','a'+sLineBreak,TPAsElement(TheRecord.Members[0]).DocComment);
end;

procedure TTestRecordTypeParser.TestOneFieldDeprecated;
begin
  TestFields(['x : integer'],'deprecated',False);
  AssertOneIntegerField([]);
end;

procedure TTestRecordTypeParser.TestOneFieldPlatform;
begin
  TestFields(['x : integer'],'platform',False);
  AssertOneIntegerField([]);
end;

procedure TTestRecordTypeParser.TestOneFieldSemicolon;
begin
  TestFields(['x : integer;'],'',False);
  AssertOneIntegerField([]);
end;

procedure TTestRecordTypeParser.TestOneFieldSemicolonDeprecated;
begin
  TestFields(['x : integer;'],'deprecated',False);
  AssertOneIntegerField([]);

end;

procedure TTestRecordTypeParser.TestOneFieldSemicolonPlatform;
begin
  TestFields(['x : integer;'],'platform',False);
  AssertOneIntegerField([]);
end;

procedure TTestRecordTypeParser.TestOneDeprecatedField;
begin
  TestFields(['x : integer deprecated;'],'',False);
  AssertOneIntegerField([hDeprecated]);
end;

procedure TTestRecordTypeParser.TestOneDeprecatedFieldDeprecated;
begin
  TestFields(['x : integer deprecated;'],'deprecated',False);
  AssertOneIntegerField([hDeprecated]);
end;

procedure TTestRecordTypeParser.TestOneDeprecatedFieldPlatform;
begin
  TestFields(['x : integer deprecated;'],'platform',False);
  AssertOneIntegerField([hDeprecated]);
end;

procedure TTestRecordTypeParser.TestOnePlatformField;
begin
  TestFields(['x : integer platform;'],'',False);
  AssertOneIntegerField([hplatform]);
end;

procedure TTestRecordTypeParser.TestOnePlatformFieldDeprecated;
begin
  TestFields(['x : integer platform;'],'Deprecated',False);
  AssertOneIntegerField([hplatform]);
end;

procedure TTestRecordTypeParser.TestOnePlatformFieldPlatform;
begin
  TestFields(['x : integer platform;'],'Platform',False);
  AssertOneIntegerField([hplatform]);
end;

procedure TTestRecordTypeParser.TestOneGenericField;
begin
  TestFields(['Generic : Integer;'],'',False);
  AssertEquals('Member 1 field type',TPasVariable,TObject(TheRecord.Members[0]).ClassType);
  AssertEquals('Field 1 name','Generic',Field1.Name);
  AssertNotNull('Have 1 Field type',Field1.VarType);
  AssertEquals('Field 1 type',TPasUnresolvedTypeRef,Field1.VarType.ClassType);
  AssertEquals('Field 1 type name','Integer',Field1.VarType.Name);
end;

procedure TTestRecordTypeParser.TestTwoFields;
begin
  TestFields(['x : integer;','y : integer'],'',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldPrivateNoDelphi;
Var
  EC : TClass;
begin
  EC:=nil;
  try
    TestFields(['private','x : integer'],'',False);
    Fail('Need po_Delphi for visibility specifier');
  except
    on EA : EAssertionFailedError do
      Raise;
    on E : Exception do
      EC:=E.ClassType;
  end;
  AssertEquals('Exception class',EParserError,EC);
end;

procedure TTestRecordTypeParser.TestTwoFieldProtected;
Var
  B : Boolean;
  EName: String;
begin
  B:=false;
  EName:='';
  try
    TestFields(['protected','x : integer'],'',False);
    Fail('Protected not allowed as record visibility specifier')
  except
    on E : Exception do
      begin
      EName:=E.ClassName;
      B:=E is EParserError;
      end;
  end;
  If not B then
    Fail('Wrong exception class "'+EName+'".');
end;

procedure TTestRecordTypeParser.TestTwoFieldPrivate;
begin
  Scanner.Options:=[po_Delphi];
  TestFields(['private','x,y : integer'],'',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldStrictPrivate;
begin
  Scanner.Options:=[po_Delphi];
  TestFields(['strict private','x,y : integer'],'',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldDeprecated;
begin
  TestFields(['x : integer;','y : integer'],'deprecated',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldPlatform;
begin
  TestFields(['x : integer;','y : integer'],'platform',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsFirstDeprecated;
begin
  TestFields(['x : integer deprecated;','y : integer'],'',False);
  AssertTwoIntegerFields([hdeprecated],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsFirstDeprecatedDeprecated;
begin
  TestFields(['x : integer deprecated;','y : integer'],'deprecated',False);
  AssertTwoIntegerFields([hdeprecated],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsFirstDeprecatedPlatform;
begin
  TestFields(['x : integer deprecated;','y : integer'],'platform',False);
  AssertTwoIntegerFields([hdeprecated],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsSecondDeprecated;
begin
  TestFields(['x : integer;','y : integer deprecated;'],'',False);
  AssertTwoIntegerFields([],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsSecondDeprecatedDeprecated;
begin
  TestFields(['x : integer;','y : integer deprecated;'],'deprecated',False);
  AssertTwoIntegerFields([],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsSecondDeprecatedPlatform;
begin
  TestFields(['x : integer;','y : integer deprecated;'],'platform',False);
  AssertTwoIntegerFields([],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsBothDeprecated;
begin
  TestFields(['x : integer deprecated;','y : integer deprecated;'],'',False);
  AssertTwoIntegerFields([hdeprecated],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsBothDeprecatedDeprecated;
begin
  TestFields(['x : integer deprecated;','y : integer deprecated;'],'deprecated',False);
  AssertTwoIntegerFields([hdeprecated],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsBothDeprecatedPlatform;
begin
  TestFields(['x : integer deprecated;','y : integer deprecated;'],'platform',False);
  AssertTwoIntegerFields([hdeprecated],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsCombined;
begin
  TestFields(['x,y : integer;'],'',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsCombinedDeprecated;
begin
  TestFields(['x,y : integer;'],'deprecated',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoFieldsCombinedPlatform;
begin
  TestFields(['x,y : integer;'],'platform',False);
  AssertTwoIntegerFields([],[]);
end;

procedure TTestRecordTypeParser.TestTwoDeprecatedFieldsCombined;
begin
  TestFields(['x,y : integer deprecated;'],'',False);
  AssertTwoIntegerFields([hdeprecated],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoDeprecatedFieldsCombinedDeprecated;
begin
  TestFields(['x,y : integer deprecated;'],'deprecated',False);
  AssertTwoIntegerFields([hdeprecated],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestTwoDeprecatedFieldsCombinedPlatform;
begin
  TestFields(['x,y : integer deprecated;'],'platform',False);
  AssertTwoIntegerFields([hdeprecated],[hdeprecated]);
end;

procedure TTestRecordTypeParser.TestFieldAndConstructor;

begin
  Parser.Options:=[po_delphi];
  TestFields(['x : integer;','constructor create;'],'',False);
  AssertIntegerFieldAndConstructor([],[]);
end;

procedure TTestRecordTypeParser.TestFieldAndMethod;
begin
  Parser.Options:=[po_delphi];
  TestFields(['x : integer;','procedure dosomething2;'],'',False);
  AssertIntegerFieldAndMethod([],[]);
end;

procedure TTestRecordTypeParser.TestFieldAnd2Methods;
Var
  P : TPasFunction;

begin
  Parser.Options:=[po_delphi];
  TestFields(['x : integer;','procedure dosomething2;','function dosomething3 : Integer;'],'',False);
  AssertEquals('Member count',3,TheRecord.Members.Count);
  AssertField1([]);
  AssertMethod2([]);
  AssertEquals('Member 3 type',TPasFunction,TObject(TheRecord.Members[2]).ClassType);
  P:=TPasFunction(TheRecord.Members[2]);
  AssertEquals('Method 2 name','dosomething3',P.Name);
  AssertTrue('Method 2 hints match',[]=P.Hints);
  // Standard type
  AssertEquals('Method 2 result type','Integer', P.FuncType.ResultEl.ResultType.Name);
end;

procedure TTestRecordTypeParser.TestFieldAndProperty;

Var
  P : TPasProperty;
begin
  Parser.Options:=[po_delphi];
  TestFields(['private','x : integer;','public','property MyX : Integer read X write X'],'',False);
  AssertEquals('Member count',2,TheRecord.Members.Count);
  AssertField1([]);
  AssertEquals('Member 2 type',TPasProperty,TObject(TheRecord.Members[1]).ClassType);
  P:=TPasProperty(TheRecord.Members[1]);
  AssertEquals('Property name','MyX',P.Name);
  AssertNotNull('Method 2 type',P.ResolvedType);
  AssertEquals('Method 2 type','Integer',P.ResolvedType.Name);
  AssertEquals('Method 2 read','X', P.ReadAccessorName);
  AssertEquals('Method 2 Write','X', P.WriteAccessorName);
end;

procedure TTestRecordTypeParser.TestFieldAndClassMethod;

Var
  P : TPasFunction;

begin
  Parser.Options:=[po_delphi];
  TestFields(['x : integer;','class procedure dosomething2;','function dosomething3 : Integer;'],'',False);
  AssertEquals('Member count',3,TheRecord.Members.Count);
  AssertField1([]);
  AssertMethod2([],True);
  AssertEquals('Class procedure',TPasClassProcedure,TObject(TheRecord.Members[1]).ClassType);
  AssertEquals('Member 3 type',TPasFunction,TObject(TheRecord.Members[2]).ClassType);
  P:=TPasFunction(TheRecord.Members[2]);
  AssertEquals('Method 2 name','dosomething3',P.Name);
  AssertTrue('Method 2 hints match',[]=P.Hints);
  // Standard type
  AssertEquals('Method 2 result type','Integer', P.FuncType.ResultEl.ResultType.Name);
end;

procedure TTestRecordTypeParser.TestFieldAndClassOperator;

Var
  P : TPasFunction;

begin
  Scanner.CurrentModeSwitches:=Scanner.CurrentModeSwitches+[msAdvancedRecords];
  TestFields(['x : integer;','class operator assign(a : ta; b : Cardinal) : boolean;','function dosomething3 : Integer;'],'',False);
  AssertEquals('Member count',3,TheRecord.Members.Count);
  AssertField1([]);
  AssertOperatorMethod2([],True);
  AssertEquals('Member 3 type',TPasFunction,TObject(TheRecord.Members[2]).ClassType);
  P:=TPasFunction(TheRecord.Members[2]);
  AssertEquals('Method 2 name','dosomething3',P.Name);
  AssertTrue('Method 2 hints match',[]=P.Hints);
  // Standard type
  AssertEquals('Method 2 result type','Integer', P.FuncType.ResultEl.ResultType.Name);
end;

procedure TTestRecordTypeParser.TestFieldAndClassVar;
begin
  Scanner.CurrentModeSwitches:=Scanner.CurrentModeSwitches+[msAdvancedRecords];
  TestFields(['x : integer;','class var y : integer;'],'',False);
  AssertField1([]);
  AssertTrue('Second field is class var',vmClass in Field2.VarModifiers);
end;

procedure TTestRecordTypeParser.TestFieldAndVar;
begin
  Scanner.CurrentModeSwitches:=Scanner.CurrentModeSwitches+[msAdvancedRecords];
  TestFields(['x : integer;','var y : integer;'],'',False);
  AssertField1([]);
  AssertTrue('Second field is regular var',not (vmClass in Field2.VarModifiers));
end;

procedure TTestRecordTypeParser.TestNested;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end'],'',False);
  AssertField1([]);
  AssertRecordField(1,[])
end;

procedure TTestRecordTypeParser.TestNestedSemicolon;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end;'],'',False);
  AssertField1([]);
  AssertRecordField(1,[])
end;

procedure TTestRecordTypeParser.TestNestedSemicolonDeprecated;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end;'],'deprecated',False);
  AssertField1([]);
  AssertRecordField(1,[])
end;

procedure TTestRecordTypeParser.TestNestedSemicolonPlatform;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end;'],'platform',False);
  AssertField1([]);
  AssertRecordField(1,[])
end;

procedure TTestRecordTypeParser.TestNestedDeprecated;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end'],'deprecated',False);
  AssertField1([]);
  AssertRecordField(1,[])
end;

procedure TTestRecordTypeParser.TestNestedPlatform;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end'],'platform',False);
  AssertField1([]);
  AssertRecordField(1,[])
end;

procedure TTestRecordTypeParser.TestNestedFirst;
begin
  TestFields(['x : record','  z : integer;','end;','y : integer;'],'',False);
  AssertField2([]);
  AssertRecordField(0,[])
end;

procedure TTestRecordTypeParser.TestNestedFirstDeprecated;
begin
  TestFields(['x : record','  z : integer;','end;','y : integer;'],'deprecated',False);
  AssertField2([]);
  AssertRecordField(0,[])
end;

procedure TTestRecordTypeParser.TestNestedFirstPlatform;
begin
  TestFields(['x : record','  z : integer;','end;','y : integer;'],'platform',False);
  AssertField2([]);
  AssertRecordField(0,[])
end;

procedure TTestRecordTypeParser.TestDeprecatedNested;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end deprecated;'],'',False);
  AssertField1([]);
  AssertRecordField(1,[hdeprecated])
end;

procedure TTestRecordTypeParser.TestDeprecatedNestedDeprecated;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end deprecated;'],'deprecated',False);
  AssertField1([]);
  AssertRecordField(1,[hdeprecated])
end;

procedure TTestRecordTypeParser.TestDeprecatedNestedPlatform;
begin
  TestFields(['x : integer;','y : record','  z : integer;','end deprecated;'],'platform',False);
  AssertField1([]);
  AssertRecordField(1,[hdeprecated])
end;

procedure TTestRecordTypeParser.TestDeprecatedNestedFirst;
begin
  TestFields(['x : record','  z : integer;','end deprecated;','y : integer;'],'',False);
  AssertField2([]);
  AssertRecordField(0,[hdeprecated])
end;

procedure TTestRecordTypeParser.TestDeprecatedNestedFirstDeprecated;
begin
  TestFields(['x : record','  z : integer;','end deprecated;','y : integer;'],'deprecated',False);
  AssertField2([]);
  AssertRecordField(0,[hdeprecated])
end;

procedure TTestRecordTypeParser.TestDeprecatedNestedFirstPlatform;
begin
  TestFields(['x : record','  z : integer;','end deprecated;','y : integer;'],'platform',False);
  AssertField2([]);
  AssertRecordField(0,[hdeprecated])
end;

procedure TTestRecordTypeParser.TestVariantNoStorage;
begin
  DoTestVariantNoStorage('');
end;

procedure TTestRecordTypeParser.TestVariantNoStorageDeprecated;

begin
  DoTestVariantNoStorage('deprecated');
end;

procedure TTestRecordTypeParser.TestVariantNoStoragePlatform;

begin
  DoTestVariantNoStorage('platform');
end;

procedure TTestRecordTypeParser.TestVariantStorage;
begin
  DoTestVariantStorage('');
end;

procedure TTestRecordTypeParser.TestVariantStorageDeprecated;
begin
  DoTestVariantStorage('deprecated');

end;

procedure TTestRecordTypeParser.TestVariantStoragePlatform;
begin
  DoTestVariantStorage('platform');
end;

procedure TTestRecordTypeParser.TestDeprecatedVariantNoStorage;
begin
  DoTestDeprecatedVariantNoStorage('');
end;

procedure TTestRecordTypeParser.TestDeprecatedVariantNoStorageDeprecated;
begin
  DoTestDeprecatedVariantNoStorage('Deprecated');
end;

procedure TTestRecordTypeParser.TestDeprecatedVariantNoStoragePlatform;
begin
  DoTestDeprecatedVariantNoStorage('Platform');
end;

procedure TTestRecordTypeParser.TestDeprecatedVariantStorage;
begin
  DoTestDeprecatedVariantStorage('');
end;

procedure TTestRecordTypeParser.TestDeprecatedVariantStorageDeprecated;
begin
  DoTestDeprecatedVariantStorage('Deprecated');
end;

procedure TTestRecordTypeParser.TestDeprecatedVariantStoragePlatform;
begin
  DoTestDeprecatedVariantStorage('Platform');
end;

procedure TTestRecordTypeParser.TestTwoVariantsNoStorage;
begin
  DoTestTwoVariantsNoStorage('');
end;

procedure TTestRecordTypeParser.TestTwoVariantsNoStorageDeprecated;
begin
  DoTestTwoVariantsNoStorage('deprecated');
end;

procedure TTestRecordTypeParser.TestTwoVariantsNoStoragePlatform;
begin
  DoTestTwoVariantsNoStorage('platform');
end;

procedure TTestRecordTypeParser.TestTwoVariantsStorage;
begin
  DoTestTwoVariantsStorage('');
end;

procedure TTestRecordTypeParser.TestTwoVariantsStorageDeprecated;
begin
  DoTestTwoVariantsStorage('deprecated');
end;

procedure TTestRecordTypeParser.TestTwoVariantsStoragePlatform;
begin
  DoTestTwoVariantsStorage('platform');
end;

procedure TTestRecordTypeParser.TestTwoVariantsFirstDeprecatedStorage;
begin
  DoTestTwoVariantsFirstDeprecatedStorage('');
end;

procedure TTestRecordTypeParser.TestTwoVariantsFirstDeprecatedStorageDeprecated;
begin
  DoTestTwoVariantsFirstDeprecatedStorage('deprecated');
end;

procedure TTestRecordTypeParser.TestTwoVariantsFirstDeprecatedStoragePlatform;
begin
  DoTestTwoVariantsFirstDeprecatedStorage('platform');
end;

procedure TTestRecordTypeParser.TestTwoVariantsSecondDeprecatedStorage;
begin
  DoTestTwoVariantsSecondDeprecatedStorage('');
end;

procedure TTestRecordTypeParser.TestTwoVariantsSecondDeprecatedStorageDeprecated;
begin
  DoTestTwoVariantsSecondDeprecatedStorage('deprecated');
end;

procedure TTestRecordTypeParser.TestTwoVariantsSecondDeprecatedStoragePlatform;
begin
  DoTestTwoVariantsSecondDeprecatedStorage('platform');
end;

procedure TTestRecordTypeParser.TestVariantTwoLabels;
begin
  DoTestVariantTwoLabels('');
end;

procedure TTestRecordTypeParser.TestVariantTwoLabelsDeprecated;
begin
  DoTestVariantTwoLabels('Deprecated');
end;

procedure TTestRecordTypeParser.TestVariantTwoLabelsPlatform;
begin
  DoTestVariantTwoLabels('Platform');
end;

procedure TTestRecordTypeParser.TestTwoVariantsTwoLabels;
begin
  DoTestTwoVariantsTwoLabels('');
end;

procedure TTestRecordTypeParser.TestTwoVariantsTwoLabelsDeprecated;
begin
  DoTestTwoVariantsTwoLabels('Deprecated');
end;

procedure TTestRecordTypeParser.TestTwoVariantsTwoLabelsPlatform;
begin
  DoTestTwoVariantsTwoLabels('Platform');
end;

procedure TTestRecordTypeParser.TestVariantNestedRecord;
begin
  DoTestVariantNestedRecord('');
end;

procedure TTestRecordTypeParser.TestVariantNestedRecordDeprecated;
begin
  DoTestVariantNestedRecord('Deprecated');
end;

procedure TTestRecordTypeParser.TestVariantNestedRecordPlatform;
begin
  DoTestVariantNestedRecord('Platform');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariant;
begin
  DoTestVariantNestedVariant('');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantDeprecated;
begin
  DoTestVariantNestedVariant('deprecated');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantPlatForm;
begin
  DoTestVariantNestedVariant('Platform');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantFirstDeprecated;
begin
  DoTestVariantNestedVariantFirstDeprecated('');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantFirstDeprecatedDeprecated;
begin
  DoTestVariantNestedVariantFirstDeprecated('deprecated');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantFirstDeprecatedPlatform;
begin
  DoTestVariantNestedVariantFirstDeprecated('platform');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantSecondDeprecated;
begin
  DoTestVariantNestedVariantSecondDeprecated('');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantSecondDeprecatedDeprecated;
begin
  DoTestVariantNestedVariantSecondDeprecated('deprecated');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantSecondDeprecatedPlatform;
begin
  DoTestVariantNestedVariantSecondDeprecated('platform');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantBothDeprecated;
begin
  DoTestVariantNestedVariantBothDeprecated('');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantBothDeprecatedDeprecated;
begin
  DoTestVariantNestedVariantBothDeprecated('deprecated');
end;

procedure TTestRecordTypeParser.TestVariantNestedVariantBothDeprecatedPlatform;
begin
  DoTestVariantNestedVariantBothDeprecated('platform');
end;

procedure TTestRecordTypeParser.TestOperatorField;
begin
  TestFields(['operator : integer;'],'',False);
  AssertEquals('Field 1 name','operator',Field1.Name);
end;

procedure TTestRecordTypeParser.TestPropertyFail;
begin
  AddMember('Property Something');
  ParseRecordFail(SErrRecordPropertiesNotAllowed,nErrRecordPropertiesNotAllowed);
end;

procedure TTestRecordTypeParser.TestAdvRec_TwoConst;
var
  aConst: TPasConst;
begin
  Scanner.Options:=[po_Delphi];
  TestFields(['public','Const x =123;','y : integer = 456'],'',False);
  AssertEquals('Two Const',2,TheRecord.Members.Count);
  AssertConst1([]);
  AssertEquals('Correct visibility',visPublic,TPasConst(TheRecord.Members[0]).Visibility);
  AssertEquals('Member 2 type',TPasConst,TObject(TheRecord.Members[1]).ClassType);
  aConst:=TPasConst(TheRecord.Members[1]);
  AssertEquals('Const 2 name','y',aConst.Name);
  AssertNotNull('Have 2 const expr',aConst.Expr);
end;

procedure TTestRecordTypeParser.TestAdvRec_Property;
begin
  StartRecord(true);
  AddMember('Property Something: word');
  ParseRecord;
end;

procedure TTestRecordTypeParser.TestAdvRec_PropertyImplementsFail;
begin
  StartRecord(true);
  AddMember('Property Something: word implements ISome;');
  ParseRecordFail('Expected ";"',nParserExpectTokenError);
end;

procedure TTestRecordTypeParser.TestAdvRec_PropertyNoTypeFail;
begin
  StartRecord(true);
  AddMember('Property Something;');
  ParseRecordFail('Expected ":"',nParserExpectTokenError);
end;

procedure TTestRecordTypeParser.TestAdvRec_ForwardFail;
begin
  StartRecord(true);
  FDecl.Add(';TMyRecord = record');
  ParseRecordFail('Syntax error in type',nParserTypeSyntaxError);
end;

procedure TTestRecordTypeParser.TestAdvRec_PublishedFail;
begin
  StartRecord(true);
  AddMember('published');
  AddMember('A: word;');
  ParseRecordFail(SParserInvalidRecordVisibility,nParserInvalidRecordVisibility);
end;

procedure TTestRecordTypeParser.TestAdvRec_ProcVirtualFail;
begin
  StartRecord(true);
  AddMember('procedure DoIt; virtual;');
  ParseRecordFail(SParserExpectedCommaColon,nParserExpectedCommaColon);
end;

procedure TTestRecordTypeParser.TestAdvRec_ProcOverrideFail;
begin
  StartRecord(true);
  AddMember('procedure DoIt; override;');
  ParseRecordFail(SParserExpectedCommaColon,nParserExpectedCommaColon);
end;

procedure TTestRecordTypeParser.TestAdvRec_ProcMessageFail;
begin
  StartRecord(true);
  AddMember('procedure DoIt; message 2;');
  ParseRecordFail(SParserExpectedCommaColon,nParserExpectedCommaColon);
end;

procedure TTestRecordTypeParser.TestAdvRec_DestructorFail;
begin
  StartRecord(true);
  AddMember('destructor Free;');
  ParseRecordFail(SParserNoConstructorAllowed,nParserNoConstructorAllowed);
end;

procedure TTestRecordTypeParser.TestAdvRec_CaseInVar;

// Found in System.UITypes.pas

begin
  StartRecord(true);
  AddMember('var');
  AddMember('Case Integer of');
  AddMember('  1 : (x: integer);');
  AddMember('  2 : (y,z: integer)');
  ParseRecord;
end;

procedure TTestRecordTypeParser.TestAdvRec_EmptySections;
begin
  StartRecord(true);
  AddMember('const');
  AddMember('type');
  AddMember('var');
  AddMember('  x: integer;');
  ParseRecord;
end;

procedure TTestRecordTypeParser.TestAdvRecordInFunction;

// Src from bug report 36179

Const
   Src =
    '{$mode objfpc}'+sLineBreak+
    '{$modeswitch advancedrecords}'+sLineBreak+
    'program afile;'+sLineBreak+
    '  procedure DoThis;'+sLineBreak+
    '  type'+sLineBreak+
    '    TMyRecord = record'+sLineBreak+
    '      private'+sLineBreak+
    '        x, y, z: integer;'+sLineBreak+
    '    end;'+sLineBreak+
    '  begin'+sLineBreak+
    '  end;'+sLineBreak+
    'begin'+sLineBreak+
    'end.';

begin
  Source.Text:=Src;
  ParseModule; // We're just interested in that it parses.
end;

procedure TTestRecordTypeParser.TestAdvRecordInAnonFunction;

// Src from bug report 36179, modified to put record in anonymous function
//  Delphi 10.3.2 allows this

Const
   Src =
    '{$mode objfpc}'+sLineBreak+
    '{$modeswitch advancedrecords}'+sLineBreak+
    'program afile;'+sLineBreak+
    'var a : Procedure;'+sLineBreak+
    'begin'+sLineBreak+
    ' a := '+sLineBreak+
    '  procedure '+sLineBreak+
    '  type'+sLineBreak+
    '    TMyRecord = record'+sLineBreak+
    '      private'+sLineBreak+
    '        x, y, z: integer;'+sLineBreak+
    '    end;'+sLineBreak+
    '  begin'+sLineBreak+
    '  end;'+sLineBreak+
    'end.';

begin
  Source.Text:=Src;
  ParseModule; // We're just interested in that it parses.
end;

procedure TTestRecordTypeParser.TestAdvRecordClassOperator;

// Source from bug id 36180

Const
   SRC =
    '{$mode objfpc}'+sLineBreak+
    '{$modeswitch advancedrecords}'+sLineBreak+
    'program afile;'+sLineBreak+
    'type'+sLineBreak+
    '  TMyRecord = record'+sLineBreak+
    '    class operator = (a, b: TMyRecord): boolean;'+sLineBreak+
    '  end;'+sLineBreak+
    'class operator TMyRecord.= (a, b: TMyRecord): boolean;'+sLineBreak+
    'begin'+sLineBreak+
    '  result := (@a = @b);'+sLineBreak+
    'end;'+sLineBreak+
    'begin'+sLineBreak+
    'end.';

begin
  Source.Text:=Src;
  ParseModule;   // We're just interested in that it parses.
end;

procedure TTestRecordTypeParser.TestAdvRecordInitOperator;
// Source from bug id 36180

Const
   SRC =
    '{$mode objfpc}'+sLineBreak+
    '{$modeswitch advancedrecords}'+sLineBreak+
    'program afile;'+sLineBreak+
    'type'+sLineBreak+
    '  TMyRecord = record'+sLineBreak+
    '    class operator initialize (var self: TMyRecord);'+sLineBreak+
    '  end;'+sLineBreak+
    'class operator TMyRecord.initialize (a, b: TMyRecord);'+sLineBreak+
    'begin'+sLineBreak+
    '  result := (@a = @b);'+sLineBreak+
    'end;'+sLineBreak+
    'begin'+sLineBreak+
    'end.';

begin
  Source.Text:=Src;
  ParseModule;   // We're just interested in that it parses.
end;

procedure TTestRecordTypeParser.TestAdvRecordGenericFunction;

Const
   SRC =
    '{$mode objfpc}'+sLineBreak+
    '{$modeswitch advancedrecords}'+sLineBreak+
    'program afile;'+sLineBreak+
    'type'+sLineBreak+
    '  TMyRecord = record'+sLineBreak+
    '    generic class procedure doit<T> (a: T);'+sLineBreak+
    '  end;'+sLineBreak+
    'generic class procedure TMyRecord.DoIt<T>(a: T);'+sLineBreak+
    'begin'+sLineBreak+
    'end;'+sLineBreak+
    'begin'+sLineBreak+
    'end.';
begin
  Source.Text:=Src;
  ParseModule;   // We're just interested in that it parses.
end;

{ TBaseTestTypeParser }

Function TBaseTestTypeParser.ParseType(ASource: String; ATypeClass: TClass;
  Const AHint: String): TPasType;

Var
  D : String;

begin
  Hint:=AHint;
  Add('Type');
  If AddComment then
    begin
    Add('// A comment');
    Parser.Engine.NeedComments:=True;
    end;
  D:='A = '+ASource;
  If Hint<>'' then
    D:=D+' '+Hint;
  Add('  '+D+';');
//  Writeln(source.text);
  ParseDeclarations;
  if ATypeClass.InHeritsFrom(TPasClassType) then
    AssertEquals('One type definition',1,Declarations.Classes.Count)
  else
    AssertEquals('One type definition',1,Declarations.Types.Count);
  If ATypeClass<>Nil then
    begin
    if ATypeClass.InheritsFrom(TPasClassType) then
      Result:=TPasType(Declarations.Classes[0])
    else
      Result:=TPasType(Declarations.Types[0]);
    AssertEquals('First declaration is type definition.',ATypeClass,Result.ClassType);
    end;
  AssertEquals('First declaration has correct name.','A',Result.Name);
  FType:=Result;
  Definition:=Result;
  if (Hint<>'') then
    CheckHint(TPasMemberHint(GetEnumValue(TypeInfo(TPasMemberHint),'h'+Hint)));
end;

Procedure TBaseTestTypeParser.AssertParseTypeError(ASource: String);
begin
  try
    ParseType(ASource,Nil,'');
    Fail('Expected parser error');
  except
    // all OK.
  end;
end;

Procedure TBaseTestTypeParser.AssertComment;
begin
  AssertNotNull('Have type',TheType);
  AssertEquals('Type comment',' A comment'+sLineBreak,TheTYpe.DocComment);
end;

procedure TBaseTestTypeParser.SetUp;
begin
  Inherited;
  FErrorSource:='';
  FHint:='';
  FType:=Nil;
  Parser.CurrentModeswitches:=[msObjfpc];
end;

Procedure TBaseTestTypeParser.TearDown;
begin
  inherited TearDown;
  FType:=Nil;
end;

{ TTestTypeParser }

procedure TTestTypeParser.DoTestAliasType(const AnAliasType: String;
  const AHint: String);
begin
  ParseType(AnAliasType,TPasAliasType,AHint);
  AssertEquals('Unresolved type',TPasUnresolvedTypeRef,TPasAliasType(TheType).DestType.ClassType);
end;

procedure TTestTypeParser.DoTestStringType(const AnAliasType: String;
  const AHint: String);
begin
  ParseType(AnAliasType,TPasAliasType,AHint);
  AssertEquals('String type',TPasStringType,TPasAliasType(TheType).DestType.ClassType);
end;

procedure TTestTypeParser.DoTypeError(const AMsg, ASource: string);

begin
  FErrorSource:=ASource;
  AssertException(AMsg,EParserError,@DoParseError);
end;

procedure TTestTypeParser.DoParseError;
begin
  ParseType(FErrorSource,Nil);
end;

procedure TTestTypeParser.DoParsePointer(const ASource: String;
  const AHint: String; ADestType: TClass);

begin
  ParseType('^'+ASource,TPasPointerType,AHint);
  if ADestType = Nil then
    ADestType:=TPasUnresolvedTypeRef;
  AssertEquals('Destination type '+ADestType.ClassName,ADestType,TPasPointerType(TheType).DestType.ClassType);
end;

procedure TTestTypeParser.DoParseArray(const ASource: String;
  const AHint: String; ADestType: TClass);
begin
  ParseType(ASource,TPasArrayType,AHint);
  if ADestType = Nil then
    ADestType:=TPasUnresolvedTypeRef;
  AssertEquals('Destination type '+ADestType.ClassName,ADestType,TPasArrayType(TheType).ElType.ClassType);
end;

procedure TTestTypeParser.DoParseEnumerated(const ASource: String;
  const AHint: String; ACount: integer);

Var
  I : Integer;

begin
  ParseType(ASource,TPasEnumType,AHint);
  AssertNotNull('Have values',TPasEnumType(TheType).Values);
  AssertEquals('Value count',ACount,TPasEnumType(TheType).Values.Count);
  For I:=0 to TPasEnumType(TheType).Values.Count-1 do
    AssertEquals('Enum value typed element '+IntToStr(I),TPasEnumValue,TObject(TPasEnumType(TheType).Values[i]).ClassType);
end;

procedure TTestTypeParser.DoTestFileType(const AType: String;
  const AHint: String; ADestType: TClass);
begin
  ParseType('File of '+AType,TPasFileType,AHint);
  AssertNotNull('Have element type',TPasFileType(TheType).ElType);
  if ADestType = Nil then
    ADestType:=TPasUnresolvedTypeRef;
  AssertEquals('Element type '+ADestType.ClassName,ADestType,TPasFileType(TheType).ElType.ClassType);
end;

procedure TTestTypeParser.DoTestRangeType(const AStart, AStop, AHint: String);
begin
  ParseType(AStart+'..'+AStop,TPasRangeType,AHint);
  AssertEquals('Range start',AStart,Stringreplace(TPasRangeType(TheType).RangeStart,' ','',[rfReplaceAll]));
  AssertEquals('Range start',AStop,Stringreplace(TPasRangeType(TheType).RangeEnd,' ','',[rfReplaceAll]));
end;

procedure TTestTypeParser.DoParseSimpleSet(const ASource: String; const AHint: String; IsPacked: Boolean);
begin
  if IsPacked then
    ParseType('Packed Set of '+ASource,TPasSetType,AHint)
  else
    ParseType('Set of '+ASource,TPasSetType,AHint);
  AssertNotNull('Have enumtype',TPasSetType(TheType).EnumType);
  AssertEquals('Element type ',TPasUnresolvedTypeRef,TPasSetType(TheType).EnumType.ClassType);
  AssertEquals('IsPacked is correct',isPacked,TPasSetType(TheType).IsPacked);
end;

procedure TTestTypeParser.DoParseComplexSet(const ASource: String;
  const AHint: String);

begin
  ParseType('Set of '+ASource,TPasSetType,AHint);
  AssertNotNull('Have enumtype',TPasSetType(TheType).EnumType);
  AssertEquals('Element type ',TPasEnumType,TPasSetType(TheType).EnumType.ClassType);
end;

procedure TTestTypeParser.DoParseRangeSet(const ASource: String;
  const AHint: String);

begin
  ParseType('Set of '+ASource,TPasSetType,AHint);
  AssertNotNull('Have enumtype',TPasSetType(TheType).EnumType);
  AssertEquals('Element type ',TPasRangeType,TPasSetType(TheType).EnumType.ClassType);
end;

procedure TTestTypeParser.DoTestComplexSet;

Var
  I : integer;

begin
  AssertNotNull('Have values',TPasEnumType(TPasSetType(TheType).EnumType).Values);
  AssertEquals('Have 3 values',3, TPasEnumType(TPasSetType(TheType).EnumType).Values.Count);
  For I:=0 to TPasEnumType(TPasSetType(TheType).EnumType).Values.Count-1 do
    AssertEquals('Enum value typed element '+IntToStr(I),TPasEnumValue,TObject(TPasEnumType(TPasSetType(TheType).EnumType).Values[i]).ClassType);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TPasSetType(TheType).EnumType).Values[0]).Name);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TPasSetType(TheType).EnumType).Values[1]).Name);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TPasSetType(TheType).EnumType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TPasSetType(TheType).EnumType).Values[0]).AssignedValue);
  AssertEquals('Assigned value second enumerated empty','',TPasEnumValue(TPasEnumType(TPasSetType(TheType).EnumType).Values[1]).AssignedValue);
  AssertEquals('Assigned value third enumerated empty','',TPasEnumValue(TPasEnumType(TPasSetType(TheType).EnumType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.DoTestClassOf(const AHint: string);

begin
  ParseType('Class of TSomeClass',TPasClassOfType,AHint);
  AssertNotNull('Have class type',TPasClassOfType(TheType).DestType);
  AssertEquals('Element type ',TPasUnresolvedTypeRef,TPasClassOfType(TheType).DestType.ClassType);
end;

procedure TTestTypeParser.TestAliasType;

begin
  DoTestAliasType('othertype','');
  AssertEquals('Unresolved type name ','othertype',TPasUnresolvedTypeRef(TPasAliasType(TheType).DestType).name);
end;

procedure TTestTypeParser.TestAbsoluteAliasType;
begin
  Add('Type');
  Add('  Absolute = Integer;');
  ParseDeclarations;
  AssertEquals('First declaration is type definition.',TPasAliasType,TPasElement(Declarations.Types[0]).ClassType);
  AssertEquals('First declaration has correct name.','Absolute',TPasElement(Declarations.Types[0]).Name);
end;

procedure TTestTypeParser.TestCrossUnitAliasType;
begin
  DoTestAliasType('otherunit.othertype','');
end;

procedure TTestTypeParser.TestAliasTypeDeprecated;
begin
  DoTestALiasType('othertype','deprecated');
end;

procedure TTestTypeParser.TestAliasTypePlatform;
begin
  DoTestALiasType('othertype','platform');
end;

procedure TTestTypeParser.TestSimpleTypeByte;
begin
  DoTestAliasType('BYTE','');
end;

procedure TTestTypeParser.TestSimpleTypeByteComment;
begin
  AddComment:=True;
  DoTestAliasType('BYTE','');
  AssertComment;
end;

procedure TTestTypeParser.TestSimpleTypeByteDeprecated;
begin
  DoTestAliasType('BYTE','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeBytePlatform;
begin
  DoTestAliasType('BYTE','platform');
end;

procedure TTestTypeParser.TestSimpleTypeBoolean;
begin
  DoTestAliasType('BOOLEAN','');
end;

procedure TTestTypeParser.TestSimpleTypeBooleanDeprecated;
begin
  DoTestAliasType('BOOLEAN','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeBooleanPlatform;
begin
  DoTestAliasType('BOOLEAN','platform');
end;

procedure TTestTypeParser.TestSimpleTypeChar;
begin
  DoTestAliasType('CHAR','');
end;

procedure TTestTypeParser.TestSimpleTypeCharDeprecated;
begin
  DoTestAliasType('CHAR','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeCharPlatform;
begin
  DoTestAliasType('CHAR','platform');
end;

procedure TTestTypeParser.TestSimpleTypeInteger;
begin
  DoTestAliasType('INTEGER','');
end;

procedure TTestTypeParser.TestSimpleTypeIntegerDeprecated;
begin
  DoTestAliasType('INTEGER','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeIntegerPlatform;
begin
  DoTestAliasType('INTEGER','platform');
end;

procedure TTestTypeParser.TestSimpleTypeInt64;
begin
  DoTestAliasType('INT64','');
end;

procedure TTestTypeParser.TestSimpleTypeInt64Deprecated;
begin
  DoTestAliasType('INT64','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeInt64Platform;
begin
  DoTestAliasType('INT64','platform');
end;

procedure TTestTypeParser.TestSimpleTypeLongInt;
begin
  DoTestAliasType('LONGINT','');
end;

procedure TTestTypeParser.TestSimpleTypeLongIntDeprecated;
begin
  DoTestAliasType('LONGINT','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeLongIntPlatform;
begin
  DoTestAliasType('LONGINT','platform');
end;

procedure TTestTypeParser.TestSimpleTypeLongWord;
begin
  DoTestAliasType('LONGWORD','');
end;

procedure TTestTypeParser.TestSimpleTypeLongWordDeprecated;
begin
  DoTestAliasType('LONGWORD','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeLongWordPlatform;
begin
  DoTestAliasType('LONGWORD','platform');
end;

procedure TTestTypeParser.TestSimpleTypeDouble;
begin
  DoTestAliasType('Double','');
end;

procedure TTestTypeParser.TestSimpleTypeDoubleDeprecated;
begin
  DoTestAliasType('Double','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeDoublePlatform;
begin
  DoTestAliasType('Double','platform');
end;

procedure TTestTypeParser.TestSimpleTypeShortInt;
begin
  DoTestAliasType('SHORTINT','');
end;

procedure TTestTypeParser.TestSimpleTypeShortIntDeprecated;
begin
  DoTestAliasType('SHORTINT','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeShortIntPlatform;
begin
  DoTestAliasType('SHORTINT','platform');
end;

procedure TTestTypeParser.TestSimpleTypeSmallInt;
begin
  DoTestAliasType('SMALLINT','');
end;

procedure TTestTypeParser.TestSimpleTypeSmallIntDeprecated;
begin
  DoTestAliasType('SMALLINT','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeSmallIntPlatform;
begin
  DoTestAliasType('SMALLINT','platform');
end;

procedure TTestTypeParser.TestSimpleTypeString;
begin
  DoTestAliasType('STRING','');
end;

procedure TTestTypeParser.TestSimpleTypeStringDeprecated;
begin
  DoTestAliasType('STRING','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeStringPlatform;
begin
  DoTestAliasType('STRING','platform');
end;

procedure TTestTypeParser.TestSimpleTypeStringSize;
begin
  DoTestStringType('String[10]','');
end;

procedure TTestTypeParser.TestSimpleTypeStringSizeIncomplete;
begin
  DoTypeError('Incomplete string: missing ]','string[10');
end;

procedure TTestTypeParser.TestSimpleTypeStringSizeWrong;
begin
  DoTypeError('Incomplete string, ) instead of ]','string[10)');
end;

procedure TTestTypeParser.TestSimpleTypeStringSizeDeprecated;
begin
  DoTestStringType('String[10]','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeStringSizePlatform;
begin
  DoTestStringType('String[10]','Platform');
end;

procedure TTestTypeParser.TestSimpleTypeWord;
BEGIN
  DoTestAliasType('WORD','');
end;

procedure TTestTypeParser.TestSimpleTypeWordDeprecated;
begin
  DoTestAliasType('WORD','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeWordPlatform;
begin
  DoTestAliasType('WORD','platform');
end;

procedure TTestTypeParser.TestSimpleTypeQWord;
BEGIN
  DoTestAliasType('QWORD','');
end;

procedure TTestTypeParser.TestSimpleTypeQWordDeprecated;
begin
  DoTestAliasType('QWORD','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeQWordPlatform;
begin
  DoTestAliasType('QWORD','platform');
end;

procedure TTestTypeParser.TestSimpleTypeCardinal;
begin
  DoTestAliasType('CARDINAL','');
end;

procedure TTestTypeParser.TestSimpleTypeCardinalDeprecated;
begin
  DoTestAliasType('CARDINAL','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeCardinalPlatform;
begin
  DoTestAliasType('CARDINAL','platform');
end;

procedure TTestTypeParser.TestSimpleTypeWideChar;
begin
  DoTestAliasType('WIDECHAR','');
end;

procedure TTestTypeParser.TestSimpleTypeWideCharDeprecated;
begin
  DoTestAliasType('WIDECHAR','deprecated');
end;

procedure TTestTypeParser.TestSimpleTypeWideCharPlatform;
begin
  DoTestAliasType('WIDECHAR','platform');
end;

procedure TTestTypeParser.TestPointerSimple;
begin
  DoParsePointer('integer','');
end;

procedure TTestTypeParser.TestPointerSimpleDeprecated;
begin
  DoParsePointer('integer','deprecated');
end;

procedure TTestTypeParser.TestPointerSimplePlatform;
begin
  DoParsePointer('integer','platform');
end;

procedure TTestTypeParser.TestStaticArray;
begin
  DoParseArray('array [0..2] of integer','',Nil);
  AssertEquals('Array type','0..2',TPasArrayType(TheType).IndexRange);
end;

procedure TTestTypeParser.TestStaticArrayComment;
begin
  AddComment:=True;
  TestStaticArray;
  AssertComment;
end;

procedure TTestTypeParser.TestStaticArrayDeprecated;
begin
  DoParseArray('array [0..2] of integer','deprecated',Nil);
  AssertEquals('Array type','0..2',TPasArrayType(TheType).IndexRange);
end;

procedure TTestTypeParser.TestStaticArrayPlatform;
begin
  DoParseArray('array [0..2] of integer','platform',Nil);
  AssertEquals('Array type','0..2',TPasArrayType(TheType).IndexRange);
end;

procedure TTestTypeParser.TestStaticArrayPacked;
begin
  DoParseArray('packed array [0..2] of integer','',Nil);
  AssertEquals('Array type','0..2',TPasArrayType(TheType).IndexRange);
  AssertEquals('Packed',True,TPasArrayType(TheType).IsPacked);
end;

procedure TTestTypeParser.TestStaticArrayTypedIndex;
begin
  DoParseArray('array [Boolean] of integer','',Nil);
  AssertEquals('Array type','Boolean',TPasArrayType(TheType).IndexRange);
end;

procedure TTestTypeParser.TestStaticArrayOfMethod;
begin
  DoParseArray('array[0..127] of procedure of object','',TPasProcedureType);
  AssertEquals('Array element type',TPasProcedureType,TPasArrayType(TheType).ElType.ClassType);
end;

procedure TTestTypeParser.TestStaticArrayOfProcedure;
begin
  DoParseArray('array[0..127] of procedure','',TPasProcedureType);
  AssertEquals('Array element type',TPasProcedureType,TPasArrayType(TheType).ElType.ClassType);
end;

procedure TTestTypeParser.TestDynamicArrayOfMethod;
begin
  DoParseArray('array of procedure of object','',TPasProcedureType);
  AssertEquals('Array element type',TPasProcedureType,TPasArrayType(TheType).ElType.ClassType);
end;

procedure TTestTypeParser.TestDynamicArrayOfProcedure;
begin
  DoParseArray('array of procedure ','',TPasProcedureType);
  AssertEquals('Array element type',TPasProcedureType,TPasArrayType(TheType).ElType.ClassType);
end;

procedure TTestTypeParser.TestDynamicArray;
begin
  DoParseArray('array of integer','',Nil);
  AssertEquals('Array type','',TPasArrayType(TheType).IndexRange);
end;

procedure TTestTypeParser.TestDynamicArrayComment;
begin
  AddComment:=True;
  DoParseArray('array of integer','',Nil);
  AssertEquals('Array type','',TPasArrayType(TheType).IndexRange);
  AssertComment;
end;

procedure TTestTypeParser.TestGenericArray;
begin
  Add('Type');
  Add('generic TArray<T> = array of T;');
//  Writeln(source.text);
  ParseDeclarations;
  AssertEquals('One type definition',1,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasArrayType,TObject(Declarations.Types[0]).ClassType);
  FType:=TPasType(Declarations.Types[0]);
  AssertEquals('First declaration has correct name.','TArray',FType.Name);
  AssertEquals('Array type','',TPasArrayType(TheType).IndexRange);
  AssertEquals('Generic Array type',True,TPasArrayType(TheType).IsGenericArray);
end;

procedure TTestTypeParser.TestSimpleEnumerated;

begin
  DoParseEnumerated('(one,two,three)','',3);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TheType).Values[0]).Name);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TheType).Values[1]).Name);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TheType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[0]).AssignedValue);
  AssertEquals('Assigned value second enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[1]).AssignedValue);
  AssertEquals('Assigned value third enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.TestSimpleEnumeratedComment;
begin
  AddComment:=True;
  TestSimpleEnumerated;
  AssertComment;
  AssertEquals('No comment on enum 0','',TPasEnumValue(TPasEnumType(TheType).Values[0]).DocComment);
  AssertEquals('No comment on enum 1','',TPasEnumValue(TPasEnumType(TheType).Values[1]).DocComment);
  AssertEquals('No comment on enum 2','',TPasEnumValue(TPasEnumType(TheType).Values[2]).DocComment);
end;

procedure TTestTypeParser.TestSimpleEnumeratedComment2;
begin
  AddComment:=True;
  DoParseEnumerated('( {a} one, {b} two, {c} three)','',3);
  AssertEquals('comment on enum 0','a'+sLineBreak,TPasEnumValue(TPasEnumType(TheType).Values[0]).DocComment);
  AssertEquals('comment on enum 1','b'+sLineBreak,TPasEnumValue(TPasEnumType(TheType).Values[1]).DocComment);
  AssertEquals('comment on enum 2','c'+sLineBreak,TPasEnumValue(TPasEnumType(TheType).Values[2]).DocComment);
end;

procedure TTestTypeParser.TestSimpleEnumeratedDeprecated;
begin
  DoParseEnumerated('(one,two,three)','deprecated',3);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TheType).Values[0]).Name);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TheType).Values[1]).Name);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TheType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[0]).AssignedValue);
  AssertEquals('Assigned value second enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[1]).AssignedValue);
  AssertEquals('Assigned value third enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.TestSimpleEnumeratedPlatform;
begin
  DoParseEnumerated('(one,two,three)','platform',3);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TheType).Values[0]).Name);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TheType).Values[1]).Name);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TheType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[0]).AssignedValue);
  AssertEquals('Assigned value second enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[1]).AssignedValue);
  AssertEquals('Assigned value third enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.TestAssignedEnumerated;
begin
  DoParseEnumerated('(one,two:=2,three)','',3);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TheType).Values[0]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[0]).AssignedValue);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TheType).Values[1]).Name);
  AssertEquals('Assigned value enumerated','2',TPasEnumValue(TPasEnumType(TheType).Values[1]).AssignedValue);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TheType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.TestAssignedEnumeratedDeprecated;
begin
  DoParseEnumerated('(one,two:=2,three)','',3);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TheType).Values[0]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[0]).AssignedValue);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TheType).Values[1]).Name);
  AssertEquals('Assigned value enumerated','2',TPasEnumValue(TPasEnumType(TheType).Values[1]).AssignedValue);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TheType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.TestAssignedEnumeratedPlatform;
begin
  DoParseEnumerated('(one,two:=2,three)','',3);
  AssertEquals('First enumerated value','one',TPasEnumValue(TPasEnumType(TheType).Values[0]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[0]).AssignedValue);
  AssertEquals('Second enumerated value','two',TPasEnumValue(TPasEnumType(TheType).Values[1]).Name);
  AssertEquals('Assigned value enumerated','2',TPasEnumValue(TPasEnumType(TheType).Values[1]).AssignedValue);
  AssertEquals('Third enumerated value','three',TPasEnumValue(TPasEnumType(TheType).Values[2]).Name);
  AssertEquals('Assigned value first enumerated empty','',TPasEnumValue(TPasEnumType(TheType).Values[2]).AssignedValue);
end;

procedure TTestTypeParser.TestFileType;
begin
  DoTestFileType('integer','');
end;

procedure TTestTypeParser.TestFileTypeDeprecated;
begin
  DoTestFileType('integer','deprecated');
end;

procedure TTestTypeParser.TestFileTypePlatform;
begin
  DoTestFileType('integer','platform');
end;

procedure TTestTypeParser.TestRangeType;
begin
  DoTestRangeType('1','4','');
end;

procedure TTestTypeParser.TestCharRangeType;
begin
  DoTestRangeType('#1','#4','');
end;

procedure TTestTypeParser.TestCharRangeType2;
begin
  DoTestRangeType('''A''','''B''','');
end;

procedure TTestTypeParser.TestRangeTypeDeprecated;
begin
  DoTestRangeType('1','4','deprecated');
end;

procedure TTestTypeParser.TestRangeTypePlatform;
begin
  DoTestRangeType('1','4','platform');
end;

procedure TTestTypeParser.TestIdentifierRangeType;
begin
  DoTestRangeType('tkFirst','tkLast','');
end;

procedure TTestTypeParser.TestIdentifierRangeTypeDeprecated;
begin
  DoTestRangeType('tkFirst','tkLast','deprecated');
end;

procedure TTestTypeParser.TestIdentifierRangeTypePlatform;
begin
  DoTestRangeType('tkFirst','tkLast','platform');
end;

procedure TTestTypeParser.TestNegativeIdentifierRangeType;
begin
  DoTestRangeType('-tkLast','tkLast','');
end;

procedure TTestTypeParser.TestSimpleSet;
begin
  DoParseSimpleSet('Byte','');
end;

procedure TTestTypeParser.TestSimpleSetDeprecated;
begin
  DoParseSimpleSet('Byte','deprecated');
end;

procedure TTestTypeParser.TestSimpleSetPlatform;
begin
  DoParseSimpleSet('Byte','platform');
end;


procedure TTestTypeParser.TestComplexSet;


begin
  DoParseComplexSet('(one, two, three)','');
  DoTestComplexSet;
end;

procedure TTestTypeParser.TestComplexSetDeprecated;

begin
  DoParseComplexSet('(one, two, three)','deprecated');
  DoTestComplexSet;
end;

procedure TTestTypeParser.TestComplexSetPlatform;

begin
  DoParseComplexSet('(one, two, three)','platform');
  DoTestComplexSet;
end;

procedure TTestTypeParser.TestPackedSet;
begin
  DoParseSimpleSet('Byte','',True);
end;

procedure TTestTypeParser.TestRangeLowHigh;

begin
  DoParseRangeSet('low(TRange)..high(TRange)','');
end;


procedure TTestTypeParser.TestRangeSet;
begin
  // TRange = (rLow, rMiddle, rHigh);
  DoParseRangeSet('rMiddle..high(TRange)','');
end;

procedure TTestTypeParser.TestSubRangeSet;
begin
  DoParseRangeSet('0..SizeOf(Integer)*8-1','');
end;

procedure TTestTypeParser.TestRangeSetDeprecated;
begin
  DoParseRangeSet('0..SizeOf(Integer)*8-1','deprecated');
end;

procedure TTestTypeParser.TestRangeSetPlatform;
begin
  DoParseRangeSet('0..SizeOf(Integer)*8-1','platform');
end;

procedure TTestTypeParser.TestNegativeRangeType;
begin
  DoTestRangeType('2-1','3','');
end;

procedure TTestTypeParser.TestClassOf;
begin
  DoTestClassOf('');
end;

procedure TTestTypeParser.TestClassOfComment;
begin
  AddComment:=True;
  DoTestClassOf('');
  AssertComment;
end;

procedure TTestTypeParser.TestClassOfDeprecated;
begin
  DoTestClassOf('deprecated');
end;

procedure TTestTypeParser.TestClassOfPlatform;
begin
  DoTestClassOf('Platform');
end;

procedure TTestTypeParser.TestReferenceAlias;
begin
  Add('Type');
  Add(' Type1 = Integer;');
  Add(' Type2 = Type1;');
  Add('end.');
  ParseDeclarations;
  AssertEquals('Two type definitions',2,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasAliasType,TObject(Declarations.Types[0]).ClassType);
  AssertEquals('Second declaration is type definition.',TPasAliasType,TObject(Declarations.Types[1]).ClassType);
  AssertEquals('First declaration has correct name.','Type1',TPasType(Declarations.Types[0]).Name);
  AssertEquals('Second declaration has correct name.','Type2',TPasType(Declarations.Types[1]).Name);
  AssertSame('Second declaration references first.',Declarations.Types[0],TPasAliasType(Declarations.Types[1]).DestType);
end;

procedure TTestTypeParser.TestReferenceSet;

begin
  Add('Type');
  Add(' Type1 = (a,b,c);');
  Add(' Type2 = set of Type1;');
  Add('end.');
  ParseDeclarations;
  AssertEquals('Two type definitions',2,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasEnumType,TObject(Declarations.Types[0]).ClassType);
  AssertEquals('Second declaration is type definition.',TPasSetType,TObject(Declarations.Types[1]).ClassType);
  AssertEquals('First declaration has correct name.','Type1',TPasType(Declarations.Types[0]).Name);
  AssertEquals('Second declaration has correct name.','Type2',TPasType(Declarations.Types[1]).Name);
  AssertSame('Second declaration references first.',Declarations.Types[0],TPasSetType(Declarations.Types[1]).EnumType);
end;

procedure TTestTypeParser.TestReferenceClassOf;
begin
  Add('Type');
  Add(' Type1 = Class(TObject);');
  Add(' Type2 = Class of Type1;');
  Add('end.');
  ParseDeclarations;
  AssertEquals('1 type definitions',1,Declarations.Types.Count);
  AssertEquals('1 class definitions',1,Declarations.Classes.Count);
  AssertEquals('First declaration is class definition.',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  AssertEquals('Second declaration is type definition.',TPasClassOfType,TObject(Declarations.Types[0]).ClassType);
  AssertEquals('First declaration has correct name.','Type2',TPasType(Declarations.Types[0]).Name);
  AssertEquals('Second declaration has correct name.','Type1',TPasType(Declarations.Classes[0]).Name);
  AssertSame('Second declaration references first.',Declarations.Classes[0],TPasClassOfType(Declarations.Types[0]).DestType);
end;

procedure TTestTypeParser.TestReferenceFile;
begin
  Add('Type');
  Add(' Type1 = (a,b,c);');
  Add(' Type2 = File of Type1;');
  Add('end.');
  ParseDeclarations;
  AssertEquals('Two type definitions',2,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasEnumType,TObject(Declarations.Types[0]).ClassType);
  AssertEquals('Second declaration is type definition.',TPasFileType,TObject(Declarations.Types[1]).ClassType);
  AssertEquals('First declaration has correct name.','Type1',TPasType(Declarations.Types[0]).Name);
  AssertEquals('Second declaration has correct name.','Type2',TPasType(Declarations.Types[1]).Name);
  AssertSame('Second declaration references first.',Declarations.Types[0],TPasFileType(Declarations.Types[1]).elType);
end;

procedure TTestTypeParser.TestReferenceArray;
begin
  Add('Type');
  Add(' Type1 = (a,b,c);');
  Add(' Type2 = Array of Type1;');
  Add('end.');
  ParseDeclarations;
  AssertEquals('Two type definitions',2,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasEnumType,TObject(Declarations.Types[0]).ClassType);
  AssertEquals('Second declaration is type definition.',TPasArrayType,TObject(Declarations.Types[1]).ClassType);
  AssertEquals('First declaration has correct name.','Type1',TPasType(Declarations.Types[0]).Name);
  AssertEquals('Second declaration has correct name.','Type2',TPasType(Declarations.Types[1]).Name);
  AssertSame('Second declaration references first.',Declarations.Types[0],TPasArrayType(Declarations.Types[1]).elType);
end;

procedure TTestTypeParser.TestReferencePointer;
begin
  Add('Type');
  Add(' Type1 = (a,b,c);');
  Add(' Type2 = ^Type1;');
  Add('end.');
  ParseDeclarations;
  AssertEquals('Two type definitions',2,Declarations.Types.Count);
  AssertEquals('First declaration is type definition.',TPasEnumType,TObject(Declarations.Types[0]).ClassType);
  AssertEquals('Second declaration is type definition.',TPasPointerType,TObject(Declarations.Types[1]).ClassType);
  AssertEquals('First declaration has correct name.','Type1',TPasType(Declarations.Types[0]).Name);
  AssertEquals('Second declaration has correct name.','Type2',TPasType(Declarations.Types[1]).Name);
  AssertSame('Second declaration references first.',Declarations.Types[0],TPasPointerType(Declarations.Types[1]).DestType);
end;

procedure TTestTypeParser.TestInvalidColon;
var
  ok: Boolean;
begin
  ok:=false;
  try
    ParseType(':1..2',TPasSetType);
  except
    on E: EParserError do
      ok:=true;
  end;
  AssertEquals('wrong colon in type raised an error',true,ok);
end;


procedure TTestTypeParser.StartTypeHelper(ForType: String; AParent: String);
Var
  S : String;
begin

  S:='TMyClass = Type Helper';
  if (AParent<>'') then
    begin
    S:=S+'('+AParent;
    S:=S+')';
    end;
  S:=S+' for '+ForType;
  Add(S);

end;

procedure TTestTypeParser.TestTypeHelper;
begin
  Scanner.CurrentModeSwitches:=Scanner.CurrentModeSwitches+[msTypeHelpers];
  ParseType('Type Helper for AnsiString end',TPasClassType,'');
end;

procedure TTestTypeParser.TestTypeHelperWithParent;
begin
  Scanner.CurrentModeSwitches:=Scanner.CurrentModeSwitches+[msTypeHelpers];
  ParseType('Type Helper(TOtherHelper) for AnsiString end',TPasClassType,'');
end;

procedure TTestTypeParser.TestPointerReference;
begin
  Add('Type');
  Add('  pReference = ^Reference;');
  Add('  Reference = object');
  Add('  end;');
  ParseDeclarations;
  AssertEquals('type definition count',1,Declarations.Types.Count);
  AssertEquals('object definition count',1,Declarations.Classes.Count);
end;

procedure TTestTypeParser.TestPointerKeyWord;
begin
  Add('type');
  Add('  &file = object');
  Add('  end;');
  ParseDeclarations;
  AssertEquals('object definition count',1,Declarations.Classes.Count);
end;

procedure TTestTypeParser.TestPointerFile;
begin
  Add('type');
  Add('  pfile = ^file;');
  ParseDeclarations;
  AssertEquals('object definition count',1,Declarations.Types.Count);
end;



initialization
  RegisterTests([TTestTypeParser,TTestRecordTypeParser,TTestProcedureTypeParser]);
end.

