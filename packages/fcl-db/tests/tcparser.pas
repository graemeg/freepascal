{
    This file is part of the Free Component Library
    Copyright (c) 2010-2014 by the Free Pascal development team

    SQL source syntax parser test suite

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit tcparser;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, fpsqltree, fpsqlscanner, fpsqlparser, testregistry;

type

  { TTestParser }

  TTestParser = Class(TSQLParser)
  public
    Procedure ParseStringDef(Out DT : TSQLDataType; Out Len : Integer; Out ACharset : TSQLStringtype);
    Function  ParseType(Flags : TParseTypeFlags) : TSQLTypeDefinition;
    Function  ParseConstraint : TSQLExpression;
    Function  ParseProcedureStatements : TSQLStatement;
  end;

  { TTestSQLParser }

  TTestSQLParser = class(TTestCase)
  Private
    FParserOptions: TParserOptions;
    FSource : TStringStream;
    FParser : TTestParser;
    FToFree : TSQLElement; //will be freed by test teardown
    FErrSource : string;
    function GetParserOptions: TParserOptions;
  protected
    procedure AssertTypeDefaults(TD: TSQLTypeDefinition; Len: Integer=0);
    procedure TestStringDef(ASource: String; ExpectDT: TSQLDataType; ExpectLen: Integer; ExpectCharset : TSQLStringType='');
    function TestType(ASource : string; AFlags : TParseTypeFlags; AExpectedType : TSQLDataType) : TSQLTypeDefinition;
    function TestCheck(ASource : string; AExpectedConstraint : TSQLElementClass) : TSQLExpression;
    procedure CreateParser(Const ASource : string; aOptions : TParserOptions = []);
    function CheckClass(E : TSQLElement; C : TSQLElementClass) : TSQLElement;
    procedure TestDropStatement(Const ASource : string;C : TSQLElementClass);
    function TestCreateStatement(Const ASource,AName : string;C: TSQLElementClass) : TSQLCreateOrAlterStatement;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLToken); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLBinaryoperation); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLUnaryoperation); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLternaryoperation); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLDataType); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TForeignKeyAction); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLJoinType); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLAggregateFunction); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLAggregateOption); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TSQLOrderDirection); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TPlanJoinType); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TTriggerMoment); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TTriggerState); overload;
    procedure AssertEquals(const AMessage: String; Expected, Actual: TTriggerOperations); overload;
    function AssertLiteralExpr(Const AMessage : String; Element : TSQLExpression; ALiteralClass : TSQLElementClass) : TSQLLiteral;
    procedure AssertIdentifierName(Const AMessage : String; Const AExpected : String; Element : TSQLElement);
    procedure AssertField(AField : TSQLElement; Const AName : String; Const AAlias : String = '');
    procedure AssertAggregate(AField : TSQLElement; AAgregate : TSQLAggregateFunction; Const AFieldName : String; AOption : TSQLAggregateOption; Const AAlias : String = '');
    procedure AssertAggregateExpression(E : TSQLElement; AAgregate : TSQLAggregateFunction; Const AFieldName : String; AOption : TSQLAggregateOption);
    procedure AssertTable(ATable : TSQLElement; Const AName : String; Const AAlias : String = '');
    function AssertJoin(AJoin : TSQLElement; Const AFirst,ASecond : String; Const aJoinType : TSQLJoinType) : TSQLJoinTableReference;
    function AssertJoinOn(AJoin : TSQLExpression; Const AFirst,ASecond : String; Const AOperation : TSQLBinaryOperation) : TSQLBinaryExpression;
    function AssertOrderBy(AOrderBy : TSQLElement; Const AField : String; Const ANumber : Integer; Const AOrdering : TSQLOrderDirection) : TSQLOrderByElement;
    function AssertSecondaryFile(ASecondaryFile : TSQLElement; Const AFile : String; Const ALength,AStart : Integer) : TSQLDatabaseFileInfo;
    procedure TestTypeError;
    procedure TestStringError;
    procedure TestCheckError;
    procedure TestParseError;
    procedure SetUp; override;
    procedure TearDown; override;
    property Parser : TTestParser Read FParser;
    property ParserOptions : TParserOptions Read GetParserOptions Write FParserOptions;
    property ToFree : TSQLElement Read FToFree Write FTofree;
  end;

  { TTestDropParser }

  TTestDropParser = Class(TTestSQLParser)
  published
    procedure TestDropDatabase;
    procedure TestDropDomain;
    procedure TestDropException;
    procedure TestDropGenerator;
    procedure TestDropIndex;
    procedure TestDropProcedure;
    procedure TestDropRole;
    procedure TestDropTable;
    procedure TestDropTrigger;
    procedure TestDropView;
    procedure TestDropShadow;
    procedure TestDropExternalFunction;
  end;

  { TTestGeneratorParser }

  TTestGeneratorParser = Class(TTestSQLParser)
  Published
    procedure TestCreateGenerator;
    procedure TestCreateSequence;
    procedure TestAlterSequence;
    procedure TestSetGenerator;
  end;

  { TTestSetTermParser }

  TTestSetTermParser = Class(TTestSQLParser)
  Published
    procedure TestSetTermNoOption;
    procedure TestSetTermOption;
  end;


  { TTestRoleParser }

  TTestRoleParser = Class(TTestSQLParser)
  Published
    procedure TestCreateRole;
    procedure TestAlterRole;
  end;

  { TTestTypeParser }

  TTestTypeParser = Class(TTestSQLParser)
  private
  Published
    procedure TestStringType1;
    procedure TestStringType2;
    procedure TestStringType3;
    procedure TestStringType4;
    procedure TestStringType5;
    procedure TestStringType6;
    procedure TestStringType7;
    procedure TestStringType8;
    procedure TestStringType9;
    procedure TestStringType10;
    procedure TestStringType11;
    procedure TestStringType12;
    procedure TestStringType13;
    procedure TestStringType14;
    procedure TestStringType15;
    procedure TestStringType16;
    procedure TestStringType17;
    procedure TestStringType18;
    procedure TestStringType19;
    procedure TestStringTypeErrors1;
    procedure TestStringTypeErrors2;
    procedure TestStringTypeErrors3;
    procedure TestTypeInt1;
    procedure TestTypeInt2;
    procedure TestTypeInt3;
    procedure TestTypeInt4;
    procedure TestTypeInt5;
    procedure TestNumerical1;
    procedure TestNumerical2;
    procedure TestNumerical3;
    procedure TestNumericalError1;
    procedure TestNumericalError2;
    procedure TestNumericalError3;
    procedure TestNumericalError4;
    procedure TestNumericalError5;
    procedure TestNumericalError6;
    procedure TestNumericalError7;
    procedure TestBlob1;
    procedure TestBlob2;
    procedure TestBlob3;
    procedure TestBlob4;
    procedure TestBlob5;
    procedure TestBlob6;
    procedure TestBlob7;
    procedure TestBlob8;
    procedure TestBlobError1;
    procedure TestBlobError2;
    procedure TestBlobError3;
    procedure TestBlobError4;
    procedure TestBlobError5;
    procedure TestBlobError6;
    procedure TestBlobError7;
    procedure TestSmallInt;
    procedure TestFloat;
    procedure TestDoublePrecision;
    procedure TestDoublePrecisionDefault;
  end;

  { TTestCheckParser }

  TTestCheckParser = Class (TTestSQLParser)
  private
  published
    procedure TestCheckNull;
    procedure TestCheckNotNull;
    procedure TestCheckBraces;
    procedure TestCheckBracesError;
    procedure TestCheckParamError;
    procedure TestCheckIdentifierError;
    procedure TestIsEqual;
    procedure TestIsNotEqual1;
    procedure TestIsNotEqual2;
    procedure TestGreaterThan;
    procedure TestGreaterThanEqual1;
    procedure TestGreaterThanEqual2;
    procedure TestLessThan;
    procedure TestLessThanEqual1;
    procedure TestLessThanEqual2;
    procedure TestLike;
    procedure TestNotLike;
    procedure TestContaining;
    procedure TestNotContaining;
    procedure TestStarting;
    procedure TestNotStarting;
    procedure TestStartingWith;
    procedure TestNotStartingWith;
    procedure TestBetween;
    procedure TestNotBetween;
    procedure TestLikeEscape;
    procedure TestNotLikeEscape;
    procedure TestAnd;
    procedure TestOr;
    procedure TestNotOr;
  end;

  { TTestDomainParser }

  // Most relevant tests are in type definition testing.
  TTestDomainParser = Class(TTestSQLParser)
  private
  Published
    procedure TestSimpleDomain;
    procedure TestSimpleDomainAs;
    procedure TestNotNullDomain;
    procedure TestDefaultNotNullDomain;
    procedure TestCheckDomain;
    procedure TestDefaultCheckNotNullDomain;
    procedure TestAlterDomainDropDefault;
    procedure TestAlterDomainDropCheck;
    procedure TestAlterDomainDropCheckError;
    procedure TestAlterDomainAddCheck;
    procedure TestAlterDomainAddConstraintCheck;
    procedure TestAlterDomainAddConstraintError;
    procedure TestAlterDomainSetDefault;
    procedure TestAlterDomainRename;
    procedure TestAlterDomainNewType;
    procedure TestAlterDomainNewTypeError1;
    procedure TestAlterDomainNewTypeError2;
  end;

  { TTestExceptionParser }

  TTestExceptionParser = Class(TTestSQLParser)
  Published
    procedure TestException;
    procedure TestAlterException;
    procedure TestExceptionError1;
    procedure TestExceptionError2;
  end;

  { TTestIndexParser }

  TTestIndexParser = Class(TTestSQLParser)
  private
  Published
    procedure TestAlterindexActive;
    procedure TestAlterindexInactive;
    procedure TestCreateIndexSimple;
    procedure TestIndexIndexDouble;
    procedure TestCreateIndexAscending;
    procedure TestCreateIndexDescending;
    procedure TestCreateIndexUnique;
    procedure TestCreateIndexUniqueAscending;
    procedure TestCreateIndexUniqueDescending;
    procedure TestIndexError1;
    procedure TestIndexError2;
    procedure TestIndexError3;
    procedure TestIndexError4;
    procedure TestIndexError5;
    procedure TestIndexError6;
  end;

  { TTestTableParser }

  TTestTableParser = Class(TTestSQLParser)
  private
    procedure DoTestCreateReferencesField(Const ASource : String; AOnUpdate,AOnDelete : TForeignKeyAction);
  Published
    procedure TestCreateOneSimpleField;
    procedure TestCreateTwoSimpleFields;
    procedure TestCreateOnePrimaryField;
    procedure TestCreateOneNamedPrimaryField;
    procedure TestCreateOneUniqueField;
    procedure TestCreateOneNamedUniqueField;
    procedure TestCreateNotNullPrimaryField;
    procedure TestCreateNotNullDefaultPrimaryField;
    procedure TestCreateComputedByField;
    procedure TestCreateCheckField;
    procedure TestCreateNamedCheckField;
    procedure TestCreateReferencesField;
    procedure TestCreateReferencesOnUpdateCascadeField;
    procedure TestCreateReferencesOnUpdateNoActionField;
    procedure TestCreateReferencesOnUpdateSetDefaultField;
    procedure TestCreateReferencesOnUpdateSetNullField;
    procedure TestCreateReferencesOnDeleteCascadeField;
    procedure TestCreateReferencesOnDeleteNoActionField;
    procedure TestCreateReferencesOnDeleteSetDefaultField;
    procedure TestCreateReferencesOnDeleteSetNullField;
    procedure TestCreateReferencesOnUpdateAndDeleteSetNullField;
    procedure TestCreateNamedReferencesField;
    procedure TestCreatePrimaryKeyConstraint;
    procedure TestCreateNamedPrimaryKeyConstraint;
    procedure TestCreateForeignKeyConstraint;
    procedure TestCreateNamedForeignKeyConstraint;
    procedure TestCreateUniqueConstraint;
    procedure TestCreateNamedUniqueConstraint;
    procedure TestCreateCheckConstraint;
    procedure TestCreateNamedCheckConstraint;
    procedure TestAlterDropField;
    procedure TestAlterDropFields;
    procedure TestAlterDropConstraint;
    procedure TestAlterDropConstraints;
    procedure TestAlterRenameField;
    procedure TestAlterRenameColumnField;
    procedure TestAlterFieldType;
    procedure TestAlterFieldPosition;
    procedure TestAlterAddField;
    procedure TestAlterAddFields;
    procedure TestAlterAddPrimarykey;
    procedure TestAlterAddNamedPrimarykey;
    procedure TestAlterAddCheckConstraint;
    procedure TestAlterAddNamedCheckConstraint;
    procedure TestAlterAddForeignkey;
    procedure TestAlterAddNamedForeignkey;
  end;

  { TTestDeleteParser }

  TTestDeleteParser = Class(TTestSQLParser)
  Private
    function TestDelete(Const ASource , ATable: String) : TSQLDeleteStatement;
  Published
    procedure TestSimpleDelete;
    procedure TestSimpleDeleteAlias;
    procedure TestDeleteWhereNull;
  end;

  { TTestUpdateParser }

  TTestUpdateParser = Class(TTestSQLParser)
  Private
    function TestUpdate(Const ASource , ATable: String) : TSQLUpdateStatement;
  Published
    procedure TestUpdateOneField;
    procedure TestUpdateOneFieldFull;
    procedure TestUpdateTwoFields;
    procedure TestUpdateOneFieldWhereIsNull;
  end;

  { TTestInsertParser }

  TTestInsertParser = Class(TTestSQLParser)
  Private
    function TestInsert(Const ASource , ATable: String) : TSQLInsertStatement;
  Published
    procedure TestInsertOneField;
    procedure TestInsertTwoFields;
    procedure TestInsertOneValue;
    procedure TestInsertTwoValues;
  end;

  { TTestSelectParser }

  TTestSelectParser = Class(TTestSQLParser)
  Private
    FSelect : TSQLSelectStatement;
    function TestSelect(Const ASource : String) : TSQLSelectStatement;
    procedure TestSelectError(Const ASource : String);
    procedure DoExtractSimple(Expected : TSQLExtractElement);
    property Select : TSQLSelectStatement Read FSelect;
  Published
    procedure TestSelectOneFieldOneTable;
    procedure TestSelectOneFieldOneTableTransaction;
    procedure TestSelectOneArrayFieldOneTable;
    procedure TestSelectTwoFieldsOneTable;
    procedure TestSelectOneFieldAliasOneTable;
    procedure TestSelectTwoFieldAliasesOneTable;
    procedure TestSelectOneTableFieldOneTable;
    procedure TestSelectOneDistinctFieldOneTable;
    procedure TestSelectOneAllFieldOneTable;
    procedure TestSelectAsteriskOneTable;
    procedure TestSelectDistinctAsteriskOneTable;
    procedure TestSelectOneFieldOneTableAlias;
    procedure TestSelectOneFieldOneTableAsAlias;
    procedure TestSelectTwoFieldsTwoTables;
    procedure TestSelectTwoFieldsTwoTablesJoin;
    procedure TestSelectTwoFieldsTwoInnerTablesJoin;
    procedure TestSelectTwoFieldsTwoLeftTablesJoin;
    procedure TestSelectTwoFieldsTwoFullOuterTablesJoin;
    procedure TestSelectTwoFieldsTwoFullTablesJoin;
    procedure TestSelectTwoFieldsTwoRightTablesJoin;
    procedure TestSelectTwoFieldsThreeTablesJoin;
    procedure TestSelectTwoFieldsBracketThreeTablesJoin;
    procedure TestSelectTwoFieldsThreeBracketTablesJoin;
    procedure TestAggregateCount;
    procedure TestAggregateCountAsterisk;
    procedure TestAggregateCountAll;
    procedure TestAggregateCountDistinct;
    procedure TestAggregateMax;
    procedure TestAggregateMaxAll;
    procedure TestAggregateMaxAsterisk;
    procedure TestAggregateMaxDistinct;
    procedure TestAggregateMin;
    procedure TestAggregateMinAll;
    procedure TestAggregateMinAsterisk;
    procedure TestAggregateMinDistinct;
    procedure TestAggregateSum;
    procedure TestAggregateSumAll;
    procedure TestAggregateSumAsterisk;
    procedure TestAggregateSumDistinct;
    procedure TestAggregateAvg;
    procedure TestAggregateAvgAll;
    procedure TestAggregateAvgAsterisk;
    procedure TestAggregateAvgDistinct;
    procedure TestUpperConst;
    procedure TestUpperError;
    procedure TestGenID;
    procedure TestGenIDError1;
    procedure TestGenIDError2;
    procedure TestCastSimple;
    procedure TestExtractSimple;
    procedure TestOrderByOneField;
    procedure TestOrderByTwoFields;
    procedure TestOrderByThreeFields;
    procedure TestOrderByOneDescField;
    procedure TestOrderByTwoDescFields;
    procedure TestOrderByThreeDescFields;
    procedure TestOrderByOneTableField;
    procedure TestOrderByOneColumn;
    procedure TestOrderByTwoColumns;
    procedure TestOrderByTwoColumnsDesc;
    procedure TestOrderByCollate;
    procedure TestOrderByCollateDesc;
    procedure TestOrderByCollateDescTwoFields;
    procedure TestGroupByOne;
    procedure TestGroupByTwo;
    procedure TestHavingOne;
    procedure TestUnionSimple;
    procedure TestUnionSimpleAll;
    procedure TestUnionSimpleOrderBy;
    procedure TestUnionDouble;
    procedure TestUnionError1;
    procedure TestUnionError2;
    procedure TestPlanOrderNatural;
    procedure TestPlanOrderOrder;
    procedure TestPlanOrderIndex1;
    procedure TestPlanOrderIndex2;
    procedure TestPlanJoinNatural;
    procedure TestPlanDefaultNatural;
    procedure TestPlanMergeNatural;
    procedure TestPlanMergeNested;
    procedure TestSubSelect;
    procedure TestWhereExists;
    procedure TestWhereSingular;
    procedure TestWhereAll;
    procedure TestWhereAny;
    procedure TestWhereSome;
    procedure TestParam;
    procedure TestParamExpr;
  end;

  { TTestRollBackParser }

  TTestRollBackParser = Class(TTestSQLParser)
  Private
    FRollback : TSQLRollbackStatement;
    function TestRollback(Const ASource : String) : TSQLRollbackStatement;
    procedure TestRollbackError(Const ASource : String);
    property Rollback : TSQLRollbackStatement Read FRollback;
  Published
    procedure TestRollback;
    procedure TestRollbackWork;
    procedure TestRollbackRelease;
    procedure TestRollbackWorkRelease;
    procedure TestRollbackTransaction;
    procedure TestRollbackTransactionWork;
    procedure TestRollbackTransactionRelease;
    procedure TestRollbackTransactionWorkRelease;
  end;

  { TTestCommitParser }

  TTestCommitParser = Class(TTestSQLParser)
  Private
    FCommit : TSQLCommitStatement;
    function TestCommit(Const ASource : String) : TSQLCommitStatement;
    procedure TestCommitError(Const ASource : String);
    property Commit : TSQLCommitStatement Read FCommit;
  Published
    procedure TestCommit;
    procedure TestCommitWork;
    procedure TestCommitRelease;
    procedure TestCommitWorkRelease;
    procedure TestCommitTransaction;
    procedure TestCommitTransactionWork;
    procedure TestCommitTransactionRelease;
    procedure TestCommitTransactionWorkRelease;
    procedure TestCommitRetain;
    procedure TestCommitWorkRetain;
    procedure TestCommitReleaseRetain;
    procedure TestCommitWorkReleaseRetain;
    procedure TestCommitTransactionRetain;
    procedure TestCommitTransactionWorkRetain;
    procedure TestCommitTransactionReleaseRetain;
    procedure TestCommitTransactionWorkReleaseRetain;
    procedure TestCommitRetainSnapShot;
  end;

  { TTestExecuteProcedureParser }

  TTestExecuteProcedureParser = Class(TTestSQLParser)
  Private
    FExecute : TSQLExecuteProcedureStatement;
    function TestExecute(Const ASource : String) : TSQLExecuteProcedureStatement;
    procedure TestExecuteError(Const ASource : String);
    property Execute: TSQLExecuteProcedureStatement Read FExecute;
  Published
    procedure TestExecuteSimple;
    procedure TestExecuteSimpleTransaction;
    procedure TestExecuteSimpleReturningValues;
    procedure TestExecuteSimpleReturning2Values;
    procedure TestExecuteOneArg;
    procedure TestExecuteOneArgNB;
    procedure TestExecuteTwoArgs;
    procedure TestExecuteTwoArgsNB;
    procedure TestExecuteOneArgSelect;
    procedure TestExecuteOneArgSelectNB;
    procedure TestExecuteTwoArgsSelect;
    procedure TestExecuteTwoArgsSelectNB;
    procedure TestExecuteOneArgSelectErr;
    procedure TestExecuteOneArgSelectErr2;
    procedure TestExecuteOneArgSelectErr3;
    procedure TestExecuteOneArgSelectErr4;
  end;

  { TTestConnectParser }

  TTestConnectParser = Class(TTestSQLParser)
  Private
    FConnect : TSQLConnectStatement;
    function TestConnect(Const ASource : String) : TSQLConnectStatement;
    procedure TestConnectError(Const ASource : String);
    property Connect: TSQLConnectStatement Read FConnect;
  Published
    procedure TestConnectSimple;
    procedure TestConnectUser;
    procedure TestConnectPassword;
    procedure TestConnectUserPassword;
    procedure TestConnectUserPasswordRole;
    procedure TestConnectUserPasswordRoleCache;
    procedure TestConnectSimpleCache;
  end;

  { TTestCreateDatabaseParser }

  TTestCreateDatabaseParser = Class(TTestSQLParser)
  Private
    FCreateDB : TSQLCreateDatabaseStatement;
    function TestCreate(Const ASource : String) : TSQLCreateDatabaseStatement;
    procedure TestCreateError(Const ASource : String);
    property CreateDB : TSQLCreateDatabaseStatement Read FCreateDB;
  published
    procedure TestSimple;
    procedure TestSimpleSchema;
    procedure TestSimpleUSer;
    procedure TestSimpleUSerPassword;
    procedure TestSimplePassword;
    procedure TestPageSize;
    procedure TestPageSize2;
    procedure TestPageSizeLength;
    procedure TestPageSizeLength2;
    procedure TestPageSizeLength3;
    procedure TestPageSizeLength4;
    procedure TestCharset;
    procedure TestSecondaryFile1;
    procedure TestSecondaryFile2;
    procedure TestSecondaryFile3;
    procedure TestSecondaryFile4;
    procedure TestSecondaryFile5;
    procedure TestSecondaryFile6;
    procedure TestSecondaryFile7;
    procedure TestSecondaryFile8;
    procedure TestSecondaryFile9;
    procedure TestSecondaryFile10;
    procedure TestSecondaryFileS;
    procedure TestSecondaryFileError1;
    procedure TestSecondaryFileError2;
    procedure TestSecondaryFileError3;
  end;

  { TTestAlterDatabaseParser }

  TTestAlterDatabaseParser = Class(TTestSQLParser)
  Private
    FAlterDB : TSQLAlterDatabaseStatement;
    function TestAlter(Const ASource : String) : TSQLAlterDatabaseStatement;
    procedure TestAlterError(Const ASource : String);
    property AlterDB : TSQLAlterDatabaseStatement Read FAlterDB;
  published
    procedure TestSimple;
    procedure TestLength;
    procedure TestStarting;
    procedure TestStartingLength;
    procedure TestFiles;
    procedure TestFiles2;
    procedure TestError;
    procedure TestFilesError;
  end;

  { TTestCreateViewParser }

  TTestCreateViewParser = Class(TTestSQLParser)
  Private
    FView : TSQLCreateViewStatement;
    function TestCreate(Const ASource : String) : TSQLCreateViewStatement;
    procedure TestCreateError(Const ASource : String);
    property View : TSQLCreateViewStatement Read FView;
  Published
    procedure TestSimple;
    procedure TestFieldList;
    procedure TestFieldList2;
    procedure TestSimpleWithCheckoption;
  end;

  { TTestCreateShadowParser }

  TTestCreateShadowParser = Class(TTestSQLParser)
  Private
    FShadow : TSQLCreateShadowStatement;
    function TestCreate(Const ASource : String) : TSQLCreateShadowStatement;
    procedure TestCreateError(Const ASource : String);
    property Shadow : TSQLCreateShadowStatement Read FShadow;
  published
    procedure TestSimple;
    procedure TestLength;
    procedure TestLength2;
    procedure TestLength3;
    procedure TestLength4;
    procedure TestSecondaryFile1;
    procedure TestSecondaryFile2;
    procedure TestSecondaryFile3;
    procedure TestSecondaryFile4;
    procedure TestSecondaryFile5;
    procedure TestSecondaryFile6;
    procedure TestSecondaryFile7;
    procedure TestSecondaryFile8;
    procedure TestSecondaryFileS;
  end;

  { TTestProcedureStatement }

  TTestProcedureStatement = Class(TTestSQLParser)
  Private
    FStatement : TSQLStatement;
    procedure TestParseStatementError;
    function TestStatement(Const ASource : String) : TSQLStatement;
    procedure TestStatementError(Const ASource : String);
  Public
    property Statement : TSQLStatement Read FStatement;
  Published
    procedure TestException;
    procedure TestExceptionError;
    procedure TestExit;
    procedure TestSuspend;
    procedure TestEmptyBlock;
    procedure TestExitBlock;
    procedure TestExitBlockError;
    procedure TestPostEvent;
    procedure TestPostEventColName;
    procedure TestPostError;
    procedure TestAssignSimple;
    procedure TestAssignSimpleNew;
    procedure TestAssignSelect;
    procedure TestBlockAssignSimple;
    procedure TestIf;
    procedure TestIfBlock;
    procedure TestIfElse;
    procedure TestIfBlockElse;
    procedure TestIfElseError;
    procedure TestIfBlockElseBlock;
    procedure TestIfErrorBracketLeft;
    procedure TestIfErrorBracketRight;
    procedure TestIfErrorNoThen;
    procedure TestIfErrorSemicolonElse;
    procedure TestWhile;
    procedure TestWhileBlock;
    procedure TestWhileErrorBracketLeft;
    procedure TestWhileErrorBracketRight;
    procedure TestWhileErrorNoDo;
    procedure TestWhenAny;
    procedure TestWhenSQLCode;
    procedure TestWhenGDSCode;
    procedure TestWhenException;
    procedure TestWhenExceptionGDS;
    procedure TestWhenAnyBlock;
    procedure TestWhenErrorAny;
    procedure TestWhenErrorNoDo;
    procedure TestWhenErrorExceptionInt;
    procedure TestWhenErrorExceptionString;
    procedure TestWhenErrorSqlCode;
    procedure TestWhenErrorGDSCode;
    procedure TestExecuteStatement;
    procedure TestExecuteStatementReturningValues;
    procedure TestExecuteStatementReturningValuesColon;
    procedure TestExecuteStatementReturningValuesBrackets;
    procedure TestForSimple;
    procedure TestForSimpleNoColon;
    procedure TestForSimple2fields;
    procedure TestForBlock;
  end;

  { TTestCreateProcedureParser }

  TTestCreateProcedureParser = Class(TTestSQLParser)
  Private
    FStatement : TSQLCreateProcedureStatement;
    function TestCreate(Const ASource : String) : TSQLCreateProcedureStatement;
    procedure TestCreateError(Const ASource : String);
    property Statement : TSQLCreateProcedureStatement Read FStatement;
  Published
    procedure TestEmptyProcedure;
    procedure TestExitProcedure;
    procedure TestProcedureOneArgument;
    procedure TestProcedureTwoArguments;
    procedure TestProcedureOneReturnValue;
    procedure TestProcedureTwoReturnValues;
    procedure TestProcedureOneLocalVariable;
    procedure TestProcedureTwoLocalVariable;
    procedure TestProcedureInputOutputLocal;
  end;

  { TTestCreateTriggerParser }

  TTestCreateTriggerParser = Class(TTestSQLParser)
  Private
    FStatement : TSQLAlterCreateTriggerStatement;
    function TestCreate(Const ASource : String) : TSQLCreateTriggerStatement;
    function TestAlter(Const ASource : String) : TSQLAlterTriggerStatement;
    procedure TestCreateError(Const ASource : String);
    property Statement : TSQLAlterCreateTriggerStatement Read FStatement;
  Published
    procedure TestEmptyTrigger;
    procedure TestExitTrigger;
    procedure TestEmptyTriggerAfterUpdate;
    procedure TestEmptyTriggerBeforeDelete;
    procedure TestEmptyTriggerBeforeInsert;
    procedure TestEmptyTriggerBeforeInsertPosition1;
    procedure TestEmptyTriggerBeforeInsertPosition1inActive;
    procedure TestEmptyTriggerBeforeInsertPosition1Active;
    procedure TestTriggerOneLocalVariable;
    procedure TestTriggerTwoLocalVariables;
    procedure TestAlterTrigger;
  end;

  { TTestDeclareExternalFunctionParser }

  TTestDeclareExternalFunctionParser = Class(TTestSQLParser)
  Private
    FStatement : TSQLDeclareExternalFunctionStatement;
    function TestCreate(Const ASource : String) : TSQLDeclareExternalFunctionStatement;
    procedure TestCreateError(Const ASource : String);
    property Statement : TSQLDeclareExternalFunctionStatement Read FStatement;
  Published
    procedure TestEmptyfunction;
    procedure TestEmptyfunctionByValue;
    procedure TestCStringfunction;
    procedure TestCStringFreeItfunction;
    procedure TestOneArgumentFunction;
    procedure TestTwoArgumentsFunction;
  end;

  { TTestGrantParser }

  TTestGrantParser = Class(TTestSQLParser)
  Private
    FStatement : TSQLGrantStatement;
    function TestGrant(Const ASource : String) : TSQLGrantStatement;
    procedure TestGrantError(Const ASource : String);
    property Statement : TSQLGrantStatement Read FStatement;
  Published
    procedure TestSimple;
    procedure Test2Operations;
    procedure TestDeletePrivilege;
    procedure TestUpdatePrivilege;
    procedure TestInsertPrivilege;
    procedure TestReferencePrivilege;
    procedure TestAllPrivileges;
    procedure TestAllPrivileges2;
    procedure TestUpdateColPrivilege;
    procedure TestUpdate2ColsPrivilege;
    procedure TestReferenceColPrivilege;
    procedure TestReference2ColsPrivilege;
    procedure TestUserPrivilege;
    procedure TestUserPrivilegeWithGrant;
    procedure TestGroupPrivilege;
    procedure TestProcedurePrivilege;
    procedure TestViewPrivilege;
    procedure TestTriggerPrivilege;
    procedure TestPublicPrivilege;
    procedure TestExecuteToUser;
    procedure TestExecuteToProcedure;
    procedure TestRoleToUser;
    procedure TestRoleToUserWithAdmin;
    procedure TestRoleToPublic;
    procedure Test2RolesToUser;
  end;

  { TTestRevokeParser }

  TTestRevokeParser = Class(TTestSQLParser)
  Private
    FStatement : TSQLRevokeStatement;
    function TestRevoke(Const ASource : String) : TSQLRevokeStatement;
    procedure TestRevokeError(Const ASource : String);
    property Statement : TSQLRevokeStatement Read FStatement;
  Published
    procedure TestSimple;
    procedure Test2Operations;
    procedure TestDeletePrivilege;
    procedure TestUpdatePrivilege;
    procedure TestInsertPrivilege;
    procedure TestReferencePrivilege;
    procedure TestAllPrivileges;
    procedure TestAllPrivileges2;
    procedure TestUpdateColPrivilege;
    procedure TestUpdate2ColsPrivilege;
    procedure TestReferenceColPrivilege;
    procedure TestReference2ColsPrivilege;
    procedure TestUserPrivilege;
    procedure TestUserPrivilegeWithRevoke;
    procedure TestGroupPrivilege;
    procedure TestProcedurePrivilege;
    procedure TestViewPrivilege;
    procedure TestTriggerPrivilege;
    procedure TestPublicPrivilege;
    procedure TestExecuteToUser;
    procedure TestExecuteToProcedure;
    procedure TestRoleToUser;
    procedure TestRoleToPublic;
    procedure Test2RolesToUser;
  end;

  { TTestGlobalParser }

  TTestGlobalParser = Class(TTestSQLParser)
  published
    procedure TestEmpty;
  end;

implementation

uses typinfo;

{ TTestSetTermParser }

procedure TTestSetTermParser.TestSetTermNoOption;
begin
  FErrSource:='SET TERM ^ ;';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestSetTermParser.TestSetTermOption;
begin
  CreateParser('SET TERM ^ ;');
  FToFree:=Parser.Parse([poAllowSetTerm]);
  AssertEquals('Terminator set','^',Parser.Scanner.AlternateTerminator);
end;

{ TTestGlobalParser }

procedure TTestGlobalParser.TestEmpty;
begin
  CreateParser('');
  AssertNull('Empty statement returns nil',Parser.Parse);
end;

{ --------------------------------------------------------------------
  TTestParser
  --------------------------------------------------------------------}

procedure TTestParser.ParseStringDef(Out DT: TSQLDataType; Out Len: Integer; Out ACharset : TSQLStringtype);
begin
  ParseCharTypeDefinition(DT,Len,ACharset);
end;

function TTestParser.ParseType(Flags: TParseTypeFlags): TSQLTypeDefinition;
begin
  Result:=ParseTypeDefinition(Nil,Flags);
end;

function TTestParser.ParseConstraint: TSQLExpression;
begin
//  GetNextToken;
  Result:=ParseCheckConstraint(Nil);
end;

function TTestParser.ParseProcedureStatements: TSQLStatement;
begin
  Result:=Self.ParseProcedureStatement(Nil);
end;

{ --------------------------------------------------------------------
  TTestSQLParser
  --------------------------------------------------------------------}

procedure TTestSQLParser.SetUp;
begin
  // nothing yet
end;

procedure TTestSQLParser.TearDown;
begin
  FreeAndNil(FParser);
  FreeAndNil(FSource);
  FreeAndNil(FToFree);
end;

procedure TTestSQLParser.CreateParser(const ASource: string; aOptions: TParserOptions = []);
begin
  FSource:=TStringStream.Create(ASource);
  FParserOptions:=aOptions;
  FParser:=TTestParser.Create(FSource);
end;


Function TTestSQLParser.CheckClass(E: TSQLElement; C: TSQLElementClass) : TSQLElement;
begin
  AssertEquals(C,E.ClassType);
  Result:=E;
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected, Actual: TSQLToken);

Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLToken),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLToken),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLBinaryOperation);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLBinaryOperation),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLBinaryOperation),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLUnaryoperation);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLUnaryOperation),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLUnaryOperation),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLternaryoperation);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLTernaryOperation),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLTernaryOperation),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected, Actual: TSQLDataType);

Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLDataType),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLDataType),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TForeignKeyAction);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TForeignKeyAction),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TForeignKeyAction),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLJoinType);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLJoinType),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLJoinType),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLAggregateFunction);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLAggregateFunction),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLAggregateFunction),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLAggregateOption);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLAggregateOption),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLAggregateOption),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TSQLOrderDirection);

Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TSQLOrderDirection),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TSQLOrderDirection),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TPlanJoinType);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TPlanJoinType),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TPlanJoinType),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TTriggerMoment);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TTriggerMoment),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TTriggerMoment),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TTriggerState);
Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TTriggerState),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TTriggerState),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

procedure TTestSQLParser.AssertEquals(const AMessage: String; Expected,
  Actual: TTriggerOperations);
begin
  If Expected<>Actual then
    Fail(Amessage)
end;

Function TTestSQLParser.AssertLiteralExpr(const AMessage: String;
  Element: TSQLExpression; ALiteralClass: TSQLElementClass) : TSQLLiteral;
begin
  CheckClass(Element,TSQLLiteralExpression);
  Result:=TSQLLiteral(Checkclass(TSQLLiteralExpression(Element).Literal,ALiteralClass));
end;

procedure TTestSQLParser.AssertIdentifierName(const AMessage : String;
  const AExpected: String; Element: TSQLElement);
begin
  AssertNotNull(AMessage+': Have identifier ',Element);
  CheckClass(Element,TSQLidentifierName);
  AssertEquals(AMessage+': Correct identifier name',AExpected,TSQLidentifierName(Element).Name);
end;

procedure TTestSQLParser.AssertField(AField: TSQLElement; const AName: String;
  const AAlias: String);

Var
  F : TSQLSelectField;
  E : TSQLidentifierExpression;

begin
  AssertNotNull('Have field',AField);
  F:=TSQLSelectField(CheckClass(AField,TSQLSelectField));
  AssertNotNull('Have field expresssion,',F.Expression);
  E:=TSQLidentifierExpression(CheckClass(F.Expression,TSQLidentifierExpression));
  AssertIdentifierName('Correct field name',AName,E.Identifier);
  If (AAlias<>'') then
    AssertIdentifierName('Correct alias',AALias,F.AliasName);
end;

procedure TTestSQLParser.AssertAggregate(AField: TSQLElement;
  AAgregate: TSQLAggregateFunction; const AFieldName: String;
  AOption: TSQLAggregateOption; const AAlias: String);
Var
  F : TSQLSelectField;

begin
  AssertNotNull('Have field',AField);
  F:=TSQLSelectField(CheckClass(AField,TSQLSelectField));
  AssertNotNull('Have field expresssion,',F.Expression);
  AssertAggregateExpression(F.Expression,AAgregate,AFieldName,AOption);
  If (AAlias<>'') then
    AssertIdentifierName('Correct alias',AALias,F.AliasName);
end;

procedure TTestSQLParser.AssertAggregateExpression(E: TSQLElement;
  AAgregate: TSQLAggregateFunction; const AFieldName: String;
  AOption: TSQLAggregateOption);

Var
  AF : TSQLAggregateFunctionExpression;
  I : TSQLIdentifierExpression;

begin
  AF:=TSQLAggregateFunctionExpression(CheckClass(E,TSQLAggregateFunctionExpression));
  AssertEquals('Correct function',AAgregate,AF.Aggregate);
  AssertEquals('Correct function',AOption,AF.Option);
  If (AFieldName<>'') then
    begin
    I:=TSQLIdentifierExpression(CheckClass(AF.Expression, TSQLIdentifierExpression));
    AssertIdentifierName('Correct field name',AFieldName,I.Identifier);
    end;
end;

procedure TTestSQLParser.AssertTable(ATable: TSQLElement; const AName: String;
  const AAlias: String);
Var
  T : TSQLSimpleTablereference;

begin
  AssertNotNull('Have table',ATable);
  T:=TSQLSimpleTablereference(CheckClass(ATable,TSQLSimpleTablereference));
  AssertIdentifierName('Correct table name',AName,T.ObjectName);
  If (AAlias<>'') then
    AssertIdentifierName('Correct alias',AALias,T.AliasName);
end;

function TTestSQLParser.AssertJoin(AJoin: TSQLElement; const AFirst,
  ASecond: String; const ajointype: TSQLJoinType):TSQLJoinTableReference;
Var
  J : TSQLJoinTableReference;

begin
  AssertNotNull('Have join',AJoin);
  J:=TSQLJoinTableReference(CheckClass(AJoin,TSQLJoinTableReference));
  if (AFirst<>'') then
    AssertTable(J.Left,AFirst,'');
  if (ASecond<>'') then
    AssertTable(J.Right,ASecond,'');
  AssertEquals('Correct join type',AJoinType,J.JoinType);
  Result:=J;
end;

function TTestSQLParser.AssertJoinOn(AJoin: TSQLExpression; const AFirst,
  ASecond: String; const AOperation: TSQLBinaryOperation): TSQLBinaryExpression;

Var
  I : TSQLIdentifierExpression;

begin
  Result:=TSQLBinaryExpression(CheckClass(AJoin,TSQLBinaryExpression));
  AssertEquals('Correct ON operation',AOperation,Result.Operation);
  I:=TSQLIdentifierExpression(CheckClass(Result.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Left field name',AFirst,I.Identifier);
  I:=TSQLIdentifierExpression(CheckClass(Result.Right,TSQLIdentifierExpression));
  AssertIdentifierName('Right field name',ASecond,I.Identifier);
end;

function TTestSQLParser.AssertOrderBy(AOrderBy: TSQLElement;
  const AField: String; const ANumber: Integer; const AOrdering: TSQLOrderDirection
  ): TSQLOrderByElement;

Var
  I : TSQLIntegerLiteral;

begin
  Result:=TSQLOrderByElement(CheckClass(AorderBy,TSQLOrderByElement));
  If (AField<>'') then
    AssertIdentifierName('Correct order by field',AField,Result.Field)
  else if (ANumber>0) then
    begin
    I:=TSQLIntegerLiteral(CheckClass(Result.Field,TSQLIntegerLiteral));
    AssertEquals('Correct order by column number',ANumber,I.Value);
    end;
  AssertEquals('Correct ordering',AOrdering,Result.OrderBy);
end;

function TTestSQLParser.AssertSecondaryFile(ASecondaryFile: TSQLElement;
  const AFile: String; const ALength, AStart: Integer): TSQLDatabaseFileInfo;
begin
  Result:=TSQLDatabaseFileInfo(CheckClass(ASecondaryFile,TSQLDatabaseFileInfo));
  AssertEquals('Secondary file name',AFile,Result.FileName);
  AssertEquals('Secondary file length',ALength,Result.Length);
  AssertEquals('Secondary file start',AStart,Result.StartPage);
end;

procedure TTestSQLParser.TestTypeError;

begin
  TestType(FErrSource,[],sdtInteger);
end;

procedure TTestSQLParser.TestStringError;

begin
  TestStringDef(FErrSource,sdtchar,0);
end;

procedure TTestSQLParser.TestCheckError;
begin
  TestCheck(FErrSource,TSQLExpression);
end;

procedure TTestSQLParser.TestParseError;
begin
  CreateParser(FErrSource);
  FToFree:=Parser.Parse;
end;

procedure TTestSQLParser.TestStringDef(ASource : String; ExpectDT : TSQLDataType; ExpectLen : Integer; ExpectCharset : TSQLStringType='');

Var
  Dt : TSQLDataType;
  L : integer;
  cs : TSQLStringType;
begin
  CreateParser(ASOURCE);
  Parser.GetNextToken;
  Parser.ParseStringDef(dt,l,cs);
  AssertEquals('Datatype is CHAR',ExpectDT,Dt);
  AssertEquals('Length is 1',ExpectLen,l);
  AssertEquals('End of Stream reached',tsqlEOF,Parser.CurrentToken);
  AssertEquals('Correct character set',ExpectCharset,CS);
end;


Function TTestSQLParser.TestType(ASource : string; AFlags : TParseTypeFlags; AExpectedType : TSQLDataType) : TSQLTypeDefinition;

begin
  CreateParser(ASource);
  FToFree:=Parser.ParseType(AFlags);
  AssertNotNull('ParseType returns result',FToFree);
  CheckClass(FTofree,TSQLTypeDefinition);
  Result:=TSQLTypeDefinition(FToFree);
  AssertEquals('Type definition has correct data type',AExpectedType,Result.Datatype);
end;

function TTestSQLParser.TestCheck(ASource: string; AExpectedConstraint: TSQLElementClass
  ): TSQLExpression;
begin
  CreateParser('('+ASource+')');
  FToFree:=Parser.ParseConstraint();
  AssertNotNull('ParseType returns result',FToFree);
  CheckClass(FTofree,AExpectedConstraint);
  Result:=TSQLExpression(FToFree);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

function TTestSQLParser.GetParserOptions: TParserOptions;
begin
  if Assigned(FParser) then
    Result:=FParser.Options
  else
    Result:=FParserOptions;
end;

procedure TTestSQLParser.AssertTypeDefaults(TD : TSQLTypeDefinition;Len : Integer = 0);

begin
  AssertNull(TD.DefaultValue);
  AssertNull(TD.Check);
  AssertNull(TD.Collation);
  AssertEquals('Array dim 0',0,Length(TD.ArrayDims));
  AssertEquals('Blob type 0',0,TD.BlobType);
  AssertEquals('Not required',False,TD.NotNull);
  AssertEquals('Length',Len,TD.Len);
end;

procedure TTestSQLParser.TestDropStatement(const ASource: string;
  C: TSQLElementClass);

Var
  D : TSQLDropStatement;
begin
  If ASOURCE='SHADOW' then
    CreateParser('DROP '+ASource+' 1')
  else
    CreateParser('DROP '+ASource+' A');
  FToFree:=Parser.Parse;
  AssertNotNull('Parse returns result',FTofree);
  If Not FToFree.InheritsFrom(TSQLDropStatement) then
    Fail('Drop statement is not of type TSQLDropStatement');
  CheckClass(FToFree ,C);
  D:=TSQLDropStatement(FToFree);
  If ASOURCE='SHADOW' then
    AssertIdentifierName('object name','1',D.ObjectName)
  else
    AssertIdentifierName('object name','A',D.ObjectName);
end;

function TTestSQLParser.TestCreateStatement(const ASource,AName: string;
  C: TSQLElementClass): TSQLCreateOrAlterStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  AssertNotNull('Parse returns result',FTofree);
  If Not FToFree.InheritsFrom(TSQLCreateOrAlterStatement) then
    Fail('create statement is not of type TSQLCreateOrAlterStatement');
  CheckClass(FToFree ,C);
  Result:=TSQLCreateOrAlterStatement(FToFree);
  AssertIdentifierName('Correct identifier',AName,Result.ObjectName);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;



{ --------------------------------------------------------------------
  TTestDropParser
  --------------------------------------------------------------------}

procedure TTestDropParser.TestDropDatabase;

begin
  TestDropStatement('DATABASE',TSQLDropDatabaseStatement);
end;

procedure TTestDropParser.TestDropDomain;

begin
  TestDropStatement('DOMAIN',TSQLDropDomainStatement);
end;

procedure TTestDropParser.TestDropException;
begin
  TestDropStatement('EXCEPTION',TSQLDropExceptionStatement);
end;

procedure TTestDropParser.TestDropGenerator;

begin
  TestDropStatement('GENERATOR',TSQLDropGeneratorStatement);
end;

procedure TTestDropParser.TestDropIndex;

begin
  TestDropStatement('INDEX',TSQLDropIndexStatement);
end;

procedure TTestDropParser.TestDropProcedure;

begin
  TestDropStatement('PROCEDURE',TSQLDropProcedureStatement);
end;

procedure TTestDropParser.TestDropRole;
begin
  TestDropStatement('ROLE',TSQLDropRoleStatement);
end;

procedure TTestDropParser.TestDropTable;

begin
  TestDropStatement('TABLE',TSQLDropTableStatement);
end;

procedure TTestDropParser.TestDropTrigger;

begin
  TestDropStatement('TRIGGER',TSQLDropTriggerStatement);
end;

procedure TTestDropParser.TestDropView;

begin
  TestDropStatement('VIEW',TSQLDropViewStatement);
end;

procedure TTestDropParser.TestDropShadow;
begin
  TestDropStatement('SHADOW',TSQLDropShadowStatement);
end;

procedure TTestDropParser.TestDropExternalFunction;
begin
  TestDropStatement('EXTERNAL FUNCTION',TSQLDropExternalFunctionStatement);

end;

{ --------------------------------------------------------------------
  TTestGeneratorParser
  --------------------------------------------------------------------}


procedure TTestGeneratorParser.TestCreateGenerator;

begin
  TestCreateStatement('CREATE GENERATOR A','A',TSQLCreateGeneratorStatement);
end;

procedure TTestGeneratorParser.TestCreateSequence;

Var
  C : TSQLCreateOrAlterStatement;
begin
  C:=TestCreateStatement('CREATE SEQUENCE A','A',TSQLCreateGeneratorStatement);
  AssertEquals('Sequence detected',True,TSQLCreateGeneratorStatement(c).IsSequence);
end;

procedure TTestGeneratorParser.TestAlterSequence;
Var
  C : TSQLCreateOrAlterStatement;
  D : TSQLAlterGeneratorStatement absolute C;
begin
  C:=TestCreateStatement('ALTER SEQUENCE A RESTART WITH 100','A',TSQLAlterGeneratorStatement);
  AssertEquals('Sequence detected',True,D.IsSequence);
  AssertEquals('Sequence restart ',True,D.HasRestart);
  AssertEquals('Sequence restart value',100,D.Restart);
end;

procedure TTestGeneratorParser.TestSetGenerator;

Var
  S : TSQLSetGeneratorStatement;

begin
  CreateParser('SET GENERATOR A TO 1');
  FToFree:=Parser.Parse;
  S:=TSQLSetGeneratorStatement(CheckClass(FToFree,TSQLSetGeneratorStatement));
  AssertIdentifierName('Correct generator name','A',S.Objectname);
  AssertEquals('New value',1,S.NewValue);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

{ --------------------------------------------------------------------
  TTestTypeParser
  --------------------------------------------------------------------}

procedure TTestTypeParser.TestStringType1;

begin
  TestStringDef('CHAR(1)',sdtChar,1);
end;

procedure TTestTypeParser.TestStringType2;

begin
  TestStringDef('CHAR',sdtChar,0);
end;

procedure TTestTypeParser.TestStringType3;

begin
  TestStringDef('CHARACTER',sdtChar,0);
end;

procedure TTestTypeParser.TestStringType4;

begin
  TestStringDef('CHARACTER VARYING',sdtVarChar,0);
end;

procedure TTestTypeParser.TestStringType5;

begin
  TestStringDef('VARCHAR',sdtVarChar,0);
end;

procedure TTestTypeParser.TestStringType6;

begin
  TestStringDef('VARCHAR(2)',sdtVarChar,2);
end;

procedure TTestTypeParser.TestStringType7;

begin
  TestStringDef('CHARACTER VARYING (2)',sdtVarChar,2);
end;

procedure TTestTypeParser.TestStringType8;

begin
  TestStringDef('NATIONAL CHARACTER VARYING (2)',sdtNVarChar,2);
end;

procedure TTestTypeParser.TestStringType9;

begin
  TestStringDef('NATIONAL CHARACTER (2)',sdtNChar,2);
end;

procedure TTestTypeParser.TestStringType10;

begin
  TestStringDef('NATIONAL CHARACTER',sdtNChar,0);
end;

procedure TTestTypeParser.TestStringType11;

begin
  TestStringDef('NATIONAL CHARACTER VARYING',sdtNVarChar,0);
end;

procedure TTestTypeParser.TestStringType12;

begin
  TestStringDef('NCHAR',sdtNChar,0);
end;

procedure TTestTypeParser.TestStringType13;

begin
  TestStringDef('NCHAR(2)',sdtNChar,2);
end;


procedure TTestTypeParser.TestStringType14;

begin
  TestStringDef('NCHAR VARYING(2)',sdtNVarChar,2);
end;

procedure TTestTypeParser.TestStringType15;
begin
  TestStringDef('CHAR (15) CHARACTER SET UTF8',sdtChar,15,'UTF8');
end;

procedure TTestTypeParser.TestStringType16;
begin
  TestStringDef('CHAR VARYING (15) CHARACTER SET UTF8',sdtVarChar,15,'UTF8');
end;

procedure TTestTypeParser.TestStringType17;
begin
  TestStringDef('CHAR VARYING CHARACTER SET UTF8',sdtVarChar,0,'UTF8');
end;

procedure TTestTypeParser.TestStringType18;
begin
  TestStringDef('CHARACTER CHARACTER SET UTF8',sdtChar,0,'UTF8');
end;

procedure TTestTypeParser.TestStringType19;

Var
  T : TSQLTypeDefinition;

begin
  T:=TestType('CHAR(10) COLLATE UTF8',[],sdtChar);
  AssertNotNull('Have collation',T.Collation);
  AssertEquals('Correct collation','UTF8',T.Collation.Name);
end;

procedure TTestTypeParser.TestStringTypeErrors1;
begin
  FErrSource:='VARCHAR VARYING';
  AssertException(ESQLParser,@TestStringError);
end;

procedure TTestTypeParser.TestStringTypeErrors2;
begin
  FErrSource:='CHAR(A)';
  AssertException(ESQLParser,@TestStringError);
end;

procedure TTestTypeParser.TestStringTypeErrors3;
begin
  FErrSource:='CHAR(1]';
  AssertException(ESQLParser,@TestStringError);
end;


procedure TTestTypeParser.TestTypeInt1;

Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('INT',[],sdtInteger);
  AssertTypeDefaults(TD);
end;

procedure TTestTypeParser.TestTypeInt2;

Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('INT DEFAULT NULL',[],sdtInteger);
  AssertNotNull('Have Default value',TD.DefaultValue);
  CheckClass(TD.DefaultValue,TSQLNullLiteral);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestTypeParser.TestTypeInt3;

Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('INT DEFAULT 1',[],sdtInteger);
  AssertNotNull('Have Default value',TD.DefaultValue);
  CheckClass(TD.DefaultValue,TSQLIntegerLiteral);
  AssertEquals('Correct default value ',1,TSQLIntegerLiteral(TD.DefaultValue).Value);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestTypeParser.TestTypeInt4;

Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('INT NOT NULL',[],sdtInteger);
  AssertNull('No Default value',TD.DefaultValue);
  AssertEquals('Required field',True,TD.NotNull);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestTypeParser.TestTypeInt5;

Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('INT [3]',[],sdtInteger);
  AssertEquals('Array of length',1,Length(TD.ArrayDims));
  AssertEquals('Upper bound',3,TD.ArrayDims[0][2]);
  AssertEquals('Lower bound',1,TD.ArrayDims[0][1]);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestTypeParser.TestNumerical1;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('NUMERIC (10)',[],sdtNumeric);
  AssertEquals('Correct length',10,TD.Len);
end;

procedure TTestTypeParser.TestNumerical2;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('NUMERIC (10,3)',[],sdtNumeric);
  AssertEquals('Correct length',10,TD.Len);
  AssertEquals('Correct scale',3,TD.Scale);
end;

procedure TTestTypeParser.TestNumerical3;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('NUMERIC',[],sdtNumeric);
  AssertEquals('Correct length',0,TD.Len);
  AssertEquals('Correct scale',0,TD.Scale);
end;

procedure TTestTypeParser.TestNumericalError1;

begin
  FErrSource:='NUMERIC ()';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestNumericalError2;

begin
  FErrSource:='NUMERIC (A)';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestNumericalError3;

begin
  FErrSource:='NUMERIC (,1)';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestNumericalError4;

begin
  FErrSource:='NUMERIC (1,)';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestNumericalError5;
begin
  FErrSource:='NUMERIC (1';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestNumericalError6;
begin
  FErrSource:='NUMERIC (1,';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestNumericalError7;
begin
  FErrSource:='NUMERIC (1 NOT';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestBlob1;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB sub_type 1 SEGMENT SIZE 80 CHARACTER SET UTF8',[],sdtBlob);
  AssertEquals('Blob type 1',1,TD.BlobType);
  AssertEquals('Blob segment size',80,TD.Len);
  AssertEquals('Character set','UTF8',TD.Charset);

end;

procedure TTestTypeParser.TestBlob2;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB (80,1) CHARACTER SET UTF8',[],sdtBlob);
  AssertEquals('Blob type 1',1,TD.BlobType);
  AssertEquals('Blob segment size',80,TD.Len);
  AssertEquals('Character set','UTF8',TD.Charset);
end;

procedure TTestTypeParser.TestBlob3;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB SEGMENT SIZE 80',[],sdtBlob);
  AssertEquals('Blob type 0',0,TD.BlobType);
  AssertEquals('Blob segment size',80,TD.Len);
  AssertEquals('Character set','',TD.Charset);
end;

procedure TTestTypeParser.TestBlob4;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB SUB_TYPE 1',[],sdtBlob);
  AssertEquals('Blob type 1',1,TD.BlobType);
  AssertEquals('Blob segment size',0,TD.Len);
  AssertEquals('Character set','',TD.Charset);
end;

procedure TTestTypeParser.TestBlob5;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB (80)',[],sdtBlob);
  AssertEquals('Blob type 0',0,TD.BlobType);
  AssertEquals('Blob segment size',80,TD.Len);
  AssertEquals('Character set','',TD.Charset);
end;

procedure TTestTypeParser.TestBlob6;
Var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB',[],sdtBlob);
  AssertEquals('Blob type 0',0,TD.BlobType);
  AssertEquals('Blob segment size',0,TD.Len);
  AssertEquals('Character set','',TD.Charset);
end;

procedure TTestTypeParser.TestBlob7;
var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB SUB_TYPE BINARY',[],sdtBlob);
  AssertEquals('Blob type 0',0,TD.BlobType);
  AssertEquals('Blob segment size',0,TD.Len);
  AssertEquals('Character set','',TD.Charset);
end;

procedure TTestTypeParser.TestBlob8;
var
  TD : TSQLTypeDefinition;

begin
  TD:=TestType('BLOB SUB_TYPE TEXT',[],sdtBlob);
  AssertEquals('Blob type 1',1,TD.BlobType);
  AssertEquals('Blob segment size',0,TD.Len);
  AssertEquals('Character set','',TD.Charset);
end;

procedure TTestTypeParser.TestSmallInt;

begin
  TestType('SMALLINT',[],sdtSmallint);
end;

procedure TTestTypeParser.TestFloat;

begin
  TestType('FLOAT',[],sdtFloat);
end;

procedure TTestTypeParser.TestDoublePrecision;

begin
  TestType('DOUBLE PRECISION',[],sdtDoublePrecision);
end;

procedure TTestTypeParser.TestDoublePrecisionDefault;

begin
  TestType('DOUBLE PRECISION DEFAULT 0',[],sdtDoublePrecision);
end;

procedure TTestTypeParser.TestBlobError1;
begin
  FerrSource:='BLOB (1,)';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestBlobError2;
begin
  FerrSource:='BLOB 1,)';
  // EAssertionfailed, due to not EOF
  AssertException(EAssertionFailedError,@TestTypeError);
end;

procedure TTestTypeParser.TestBlobError3;
begin
  FerrSource:='BLOB (80) SUB_TYPE 3';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestBlobError4;
begin
  FerrSource:='BLOB CHARACTER UTF8';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestBlobError5;
begin
  FerrSource:='BLOB (80) SEGMENT SIZE 80';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestBlobError6;
begin
  FerrSource:='BLOB (A)';
  AssertException(ESQLParser,@TestTypeError);
end;

procedure TTestTypeParser.TestBlobError7;
begin
  FerrSource:='BLOB (1';
  AssertException(ESQLParser,@TestTypeError);
end;


{ --------------------------------------------------------------------
  TTestCheckParser
  --------------------------------------------------------------------}

procedure TTestCheckParser.TestCheckNotNull;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE IS NOT NULL',TSQLBinaryExpression));
  AssertEquals('IS NOT operator,',boISNot,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is null',B.Right,TSQLNullLiteral);
end;

procedure TTestCheckParser.TestCheckNull;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE IS NULL',TSQLBinaryExpression));
  AssertEquals('IS operator,',boIS,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is null',B.Right,TSQLNullLiteral);
end;

procedure TTestCheckParser.TestCheckBraces;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('(VALUE IS NULL)',TSQLBinaryExpression));
  AssertEquals('IS operator,',boIS,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is null',B.Right,TSQLNullLiteral);
end;

procedure TTestCheckParser.TestCheckBracesError;

begin
  FErrSource:='(VALUE IS NOT NULL ME )';
  AssertException('Error in braces.', ESQLParser,@TestCheckError);
end;

procedure TTestCheckParser.TestCheckParamError;
begin
  FErrSource:='VALUE <> :P';
  AssertException('Parameter.', ESQLParser,@TestCheckError);
end;

procedure TTestCheckParser.TestCheckIdentifierError;

begin
  FErrSource:='(X IS NOT NULL)';
  AssertException('Error in check: identifier.', ESQLParser,@TestCheckError);
end;

procedure TTestCheckParser.TestIsEqual;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE = 3',TSQLBinaryExpression));
  AssertEquals('Equal operator',boEq,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestIsNotEqual1;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE <> 3',TSQLBinaryExpression));
  AssertEquals('Not Equal operator',boNE,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestIsNotEqual2;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE != 3',TSQLBinaryExpression));
  AssertEquals('ENot qual operator',boNE,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestGreaterThan;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE > 3',TSQLBinaryExpression));
  AssertEquals('Greater than operator',boGT,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestGreaterThanEqual1;
Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE >= 3',TSQLBinaryExpression));
  AssertEquals('Greater or Equal operator',boGE,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestGreaterThanEqual2;
Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE !< 3',TSQLBinaryExpression));
  AssertEquals('Greater or Equal operator',boGE,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestLessThan;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE < 3',TSQLBinaryExpression));
  AssertEquals('Less than operator',boLT,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestLessThanEqual1;
Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE <= 3',TSQLBinaryExpression));
  AssertEquals('Less or Equal operator',boLE,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestLessThanEqual2;
Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE !> 3',TSQLBinaryExpression));
  AssertEquals('Less or Equal operator',boLE,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is integer',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestLike;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE LIKE ''%3''',TSQLBinaryExpression));
  AssertEquals('Like operator',boLike,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestNotLike;

Var
  B : TSQLBinaryExpression;
  U : TSQLUnaryExpression;
begin
  U:=TSQLUnaryExpression(TestCheck('VALUE NOT LIKE ''%3''',TSQLUnaryExpression));
  AssertEquals('Like operator',uoNot,U.Operation);
  CheckClass(U.Operand,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(U.Operand);
  AssertEquals('Like operator',boLike,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestContaining;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE CONTAINING ''3''',TSQLBinaryExpression));
  AssertEquals('Like operator',boContaining,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestNotContaining;

Var
  B : TSQLBinaryExpression;
  U : TSQLUnaryExpression;
begin
  U:=TSQLUnaryExpression(TestCheck('VALUE NOT CONTAINING ''3''',TSQLUnaryExpression));
  AssertEquals('Like operator',uoNot,U.Operation);
  CheckClass(U.Operand,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(U.Operand);
  AssertEquals('Like operator',boContaining,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestStarting;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE STARTING ''3''',TSQLBinaryExpression));
  AssertEquals('Starting operator',boStarting,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestNotStarting;

Var
  B : TSQLBinaryExpression;
  U : TSQLUnaryExpression;
begin
  U:=TSQLUnaryExpression(TestCheck('VALUE NOT STARTING ''3''',TSQLUnaryExpression));
  AssertEquals('Not operator',uoNot,U.Operation);
  CheckClass(U.Operand,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(U.Operand);
  AssertEquals('Starting operator',boStarting,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestStartingWith;

Var
  B : TSQLBinaryExpression;

begin
  B:=TSQLBinaryExpression(TestCheck('VALUE STARTING WITH ''3''',TSQLBinaryExpression));
  AssertEquals('Starting operator',boStarting,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestNotStartingWith;

Var
  B : TSQLBinaryExpression;
  U : TSQLUnaryExpression;
begin
  U:=TSQLUnaryExpression(TestCheck('VALUE NOT STARTING WITH ''3''',TSQLUnaryExpression));
  AssertEquals('Not operator',uoNot,U.Operation);
  CheckClass(U.Operand,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(U.Operand);
  AssertEquals('Starting operator',boStarting,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertLiteralExpr('Right is string',B.Right,TSQLStringLiteral);
end;


procedure TTestCheckParser.TestBetween;

Var
  T : TSQLTernaryExpression;

begin
  T:=TSQLTernaryExpression(TestCheck('VALUE BETWEEN 1 AND 5',TSQLTernaryExpression));
  AssertEquals('Like operator',tobetween,T.Operation);
  AssertLiteralExpr('Left is value',T.Left,TSQLValueLiteral);
  AssertLiteralExpr('Middle is integer',T.Middle,TSQLIntegerLiteral);
  AssertLiteralExpr('Right is integer',T.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestNotBetween;

Var
  U : TSQLUnaryExpression;
  T : TSQLTernaryExpression;

begin
  U:=TSQLUnaryExpression(TestCheck('VALUE NOT BETWEEN 1 AND 5',TSQLUnaryExpression));
  AssertEquals('Not operator',uoNot,U.Operation);
  CheckClass(U.Operand,TSQLTernaryExpression);
  T:=TSQLTernaryExpression(U.Operand);
  AssertEquals('Like operator',tobetween,T.Operation);
  AssertLiteralExpr('Left is value',T.Left,TSQLValueLiteral);
  AssertLiteralExpr('Middle is integer',T.Middle,TSQLIntegerLiteral);
  AssertLiteralExpr('Right is integer',T.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestLikeEscape;

Var
  T : TSQLTernaryExpression;

begin
  T:=TSQLTernaryExpression(TestCheck('VALUE LIKE ''%2'' ESCAPE ''3''',TSQLTernaryExpression));
  AssertEquals('Like operator',toLikeEscape,T.Operation);
  AssertLiteralExpr('Left is value',T.Left,TSQLValueLiteral);
  AssertLiteralExpr('Middle is string',T.Middle,TSQLStringLiteral);
  AssertLiteralExpr('Right is string',T.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestNotLikeEscape;
Var
  U : TSQLUnaryExpression;
  T : TSQLTernaryExpression;

begin
  U:=TSQLUnaryExpression(TestCheck('VALUE NOT LIKE ''%2'' ESCAPE ''3''',TSQLUnaryExpression));
  AssertEquals('Not operator',uoNot,U.Operation);
  CheckClass(U.Operand,TSQLTernaryExpression);
  T:=TSQLTernaryExpression(U.Operand);
  AssertEquals('Like operator',toLikeEscape,T.Operation);
  AssertLiteralExpr('Left is value',T.Left,TSQLValueLiteral);
  AssertLiteralExpr('Middle is string',T.Middle,TSQLStringLiteral);
  AssertLiteralExpr('Right is string',T.Right,TSQLStringLiteral);
end;

procedure TTestCheckParser.TestAnd;

Var
  T,B : TSQLBinaryExpression;

begin
  T:=TSQLBinaryExpression(TestCheck('VALUE > 4 AND Value < 11',TSQLBinaryExpression));
  AssertEquals('And operator',boand,T.Operation);
  CheckClass(T.Left,TSQLBinaryExpression);
  CheckClass(T.Right,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(T.Left);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Less than operator',boGT,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLIntegerLiteral);
  B:=TSQLBinaryExpression(T.Right);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Less than operator',boLT,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestOr;

Var
  T,B : TSQLBinaryExpression;

begin
  T:=TSQLBinaryExpression(TestCheck('VALUE < 4 or Value > 11',TSQLBinaryExpression));
  AssertEquals('And operator',boor,T.Operation);
  CheckClass(T.Left,TSQLBinaryExpression);
  CheckClass(T.Right,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(T.Left);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Less than operator',boLT,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLIntegerLiteral);
  B:=TSQLBinaryExpression(T.Right);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Less than operator',boGT,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLIntegerLiteral);
end;

procedure TTestCheckParser.TestNotOr;

Var
  T,B : TSQLBinaryExpression;

begin
  T:=TSQLBinaryExpression(TestCheck('VALUE IS NOT NULL or Value > 11',TSQLBinaryExpression));
  AssertEquals('And operator',boor,T.Operation);
  CheckClass(T.Left,TSQLBinaryExpression);
  CheckClass(T.Right,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(T.Left);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Is not null operator',boisNot,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLNullLiteral);
  B:=TSQLBinaryExpression(T.Right);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Less than operator',boGT,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLIntegerLiteral);
end;

{ TTestDomainParser }

procedure TTestDomainParser.TestSimpleDomain;

Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLCreateDomainStatement;
  T : TSQLTypeDefinition;

begin
  P:=TestCreateStatement('CREATE DOMAIN A INT','A',TSQLCreateDomainStatement);
  CheckClass(P,TSQLCreateDomainStatement);
  D:=TSQLCreateDomainStatement(P);
  AssertNotNull('Have type Definition',D.TypeDefinition);
  T:=D.TypeDefinition;
  AssertTypeDefaults(T);
  AssertEquals('Integer data type',sdtInteger,T.DataType);
end;

procedure TTestDomainParser.TestSimpleDomainAs;
Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLCreateDomainStatement;
  T : TSQLTypeDefinition;

begin
  P:=TestCreateStatement('CREATE DOMAIN A AS INT','A',TSQLCreateDomainStatement);
  CheckClass(P,TSQLCreateDomainStatement);
  D:=TSQLCreateDomainStatement(P);
  AssertNotNull('Have type Definition',D.TypeDefinition);
  T:=D.TypeDefinition;
  AssertTypeDefaults(T);
  AssertEquals('Integer data type',sdtInteger,T.DataType);
end;

procedure TTestDomainParser.TestNotNullDomain;
Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLCreateDomainStatement;
  T : TSQLTypeDefinition;

begin
  P:=TestCreateStatement('CREATE DOMAIN A INT NOT NULL','A',TSQLCreateDomainStatement);
  CheckClass(P,TSQLCreateDomainStatement);
  D:=TSQLCreateDomainStatement(P);
  AssertNotNull('Have type Definition',D.TypeDefinition);
  T:=D.TypeDefinition;
  AssertEquals('Integer data type',sdtInteger,T.DataType);
  AssertEquals('Not null',True,T.NotNull);
end;

procedure TTestDomainParser.TestDefaultNotNullDomain;
Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLCreateDomainStatement;
  T : TSQLTypeDefinition;

begin
  P:=TestCreateStatement('CREATE DOMAIN A INT DEFAULT 2 NOT NULL','A',TSQLCreateDomainStatement);
  CheckClass(P,TSQLCreateDomainStatement);
  D:=TSQLCreateDomainStatement(P);
  AssertNotNull('Have type Definition',D.TypeDefinition);
  T:=D.TypeDefinition;
  AssertNotNull('Have default value',T.DefaultValue);
  CheckClass(T.DefaultValue,TSQLINtegerLiteral);
  AssertEquals('Integer data type',sdtInteger,T.DataType);
  AssertEquals('Not null',True,T.NotNull);
end;

procedure TTestDomainParser.TestCheckDomain;
var
  P : TSQLCreateOrAlterStatement;
  D : TSQLCreateDomainStatement;
  T : TSQLTypeDefinition;

begin
  P:=TestCreateStatement('CREATE DOMAIN A AS CHAR(8) CHECK (VALUE STARTING WITH ''V'')','A',TSQLCreateDomainStatement);
  CheckClass(P,TSQLCreateDomainStatement);
  D:=TSQLCreateDomainStatement(P);
  AssertNotNull('Have type Definition',D.TypeDefinition);
  T:=D.TypeDefinition;
  AssertNull('No default value',T.DefaultValue);
  AssertEquals('Character data type',sdtChar,T.DataType);
  AssertEquals('Not null must be allowed',False,T.NotNull);
end;

procedure TTestDomainParser.TestDefaultCheckNotNullDomain;
var
  P : TSQLCreateOrAlterStatement;
  D : TSQLCreateDomainStatement;
  T : TSQLTypeDefinition;
begin
  P:=TestCreateStatement(
    'CREATE DOMAIN DEFCHECKNOTN AS VARCHAR(1) DEFAULT ''s'' CHECK (VALUE IN (''s'',''h'',''A'')) NOT NULL',
    'DEFCHECKNOTN',TSQLCreateDomainStatement);
  CheckClass(P,TSQLCreateDomainStatement);
  D:=TSQLCreateDomainStatement(P);
  AssertNotNull('Have type Definition',D.TypeDefinition);
  T:=D.TypeDefinition;
  AssertNotNull('Have default value',T.DefaultValue);
  AssertEquals('Varchar data type',sdtVarChar,T.DataType);
  AssertEquals('Not null',True,T.NotNull);
end;

procedure TTestDomainParser.TestAlterDomainDropDefault;
begin
  TestCreateStatement('ALTER DOMAIN A DROP DEFAULT','A',TSQLAlterDomainDropDefaultStatement);
end;

procedure TTestDomainParser.TestAlterDomainDropCheck;
begin
  TestCreateStatement('ALTER DOMAIN A DROP CONSTRAINT','A',TSQLAlterDomainDropCheckStatement);
end;

procedure TTestDomainParser.TestAlterDomainAddCheck;

Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLAlterDomainAddCheckStatement;
  B : TSQLBinaryExpression;

begin
  P:=TestCreateStatement('ALTER DOMAIN A ADD CHECK (VALUE IS NOT NULL)','A',TSQLAlterDomainAddCheckStatement);
  D:=TSQLAlterDomainAddCheckStatement(P);
  AssertNotNull('Have check',D.Check);
  CheckClass(D.Check,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(D.Check);
  AssertEquals('Is not null operator',boIsNot,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Is not null operator',boisNot,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLNullLiteral);
end;
procedure TTestDomainParser.TestAlterDomainAddConstraintCheck;

Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLAlterDomainAddCheckStatement;
  B : TSQLBinaryExpression;

begin
  P:=TestCreateStatement('ALTER DOMAIN A ADD CONSTRAINT CHECK (VALUE IS NOT NULL)','A',TSQLAlterDomainAddCheckStatement);
  D:=TSQLAlterDomainAddCheckStatement(P);
  AssertNotNull('Have check',D.Check);
  CheckClass(D.Check,TSQLBinaryExpression);
  B:=TSQLBinaryExpression(D.Check);
  AssertEquals('Is not null operation',boIsNot,B.Operation);
  AssertLiteralExpr('Left is value',B.Left,TSQLValueLiteral);
  AssertEquals('Is not null operator',boisNot,B.Operation);
  AssertLiteralExpr('Right is value',B.Right,TSQLNullLiteral);
end;

procedure TTestDomainParser.TestAlterDomainAddConstraintError;
begin
  FErrSource:='ALTER DOMAIN A ADD CONSTRAINT (VALUE IS NOT NULL)';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestDomainParser.TestAlterDomainSetDefault;
Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLAlterDomainSetDefaultStatement;

begin
  P:=TestCreateStatement('ALTER DOMAIN A SET DEFAULT NULL','A',TSQLAlterDomainSetDefaultStatement);
  D:=TSQLAlterDomainSetDefaultStatement(P);
  AssertNotNull('Have default',D.DefaultValue);
  CheckClass(D.DefaultValue,TSQLNullLiteral);
end;

procedure TTestDomainParser.TestAlterDomainRename;
Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLAlterDomainRenameStatement;

begin
  P:=TestCreateStatement('ALTER DOMAIN A B','A',TSQLAlterDomainRenameStatement);
  D:=TSQLAlterDomainRenameStatement(P);
  AssertIdentifierName('New name','B',D.NewName);
end;

procedure TTestDomainParser.TestAlterDomainNewType;
Var
  P : TSQLCreateOrAlterStatement;
  D : TSQLAlterDomainTypeStatement;

begin
  P:=TestCreateStatement('ALTER DOMAIN A TYPE CHAR(10)','A',TSQLAlterDomainTypeStatement);
  D:=TSQLAlterDomainTypeStatement(P);
  AssertNotNull('Have type definition',D.NewType);
  AssertEquals('Char type',sdtChar,D.NewType.DataType);
  AssertEquals('Char type of len 10',10,D.NewType.Len);
end;

procedure TTestDomainParser.TestAlterDomainNewTypeError1;

begin
  FErrSource:='ALTER DOMAIN A TYPE INT NOT NULL';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestDomainParser.TestAlterDomainNewTypeError2;
begin
  FErrSource:='ALTER DOMAIN A TYPE INT DEFAULT 1';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestDomainParser.TestAlterDomainDropCheckError;
begin
  FErrSource:='ALTER DOMAIN A DROP CHECK';
  AssertException(ESQLParser,@TestParseError);
end;


{ TTestExceptionParser }

procedure TTestExceptionParser.TestException;
Var
  P : TSQLCreateOrAlterStatement;
  E : TSQLCreateExceptionStatement;

begin
  P:=TestCreateStatement('CREATE EXCEPTION A ''A message''','A',TSQLCreateExceptionStatement);
  E:=TSQLCreateExceptionStatement(P);
  AssertNotNull('Have message',E.ExceptionMessage);
  AssertEquals('Message','A message',E.ExceptionMessage.Value)
end;

procedure TTestExceptionParser.TestAlterException;
Var
  P : TSQLCreateOrAlterStatement;
  E : TSQLCreateExceptionStatement;

begin
  P:=TestCreateStatement('ALTER EXCEPTION A ''A massage''','A',TSQLAlterExceptionStatement);
  E:=TSQLCreateExceptionStatement(P);
  AssertNotNull('Have message',E.ExceptionMessage);
  AssertEquals('Message','A massage',E.ExceptionMessage.Value)
end;

procedure TTestExceptionParser.TestExceptionError1;
begin
  FErrSource:='CREATE EXCEPTION NOT';
  ASsertException(ESQLParser,@TestParseError);
end;

procedure TTestExceptionParser.TestExceptionError2;
begin
  FErrSource:='CREATE EXCEPTION A NOT';
  ASsertException(ESQLParser,@TestParseError);
end;

{ TTestRoleParser }

procedure TTestRoleParser.TestCreateRole;

begin
  TestCreateStatement('CREATE ROLE A','A',TSQLCreateROLEStatement);
end;

procedure TTestRoleParser.TestAlterRole;
begin
  FErrSource:='ALTER ROLE A';
  ASsertException(ESQLParser,@TestParseError);
end;

{ TTestIndexParser }

procedure TTestIndexParser.TestAlterindexActive;

Var
  A : TSQLAlterIndexStatement;

begin
  A:=TSQLAlterIndexStatement(TestCreateStatement('ALTER INDEX A ACTIVE','A',TSQLAlterIndexStatement));
  AssertEquals('Active',False,A.Inactive);
end;

procedure TTestIndexParser.TestAlterindexInactive;

Var
  A : TSQLAlterIndexStatement;

begin
  A:=TSQLAlterIndexStatement(TestCreateStatement('ALTER INDEX A INACTIVE','A',TSQLAlterIndexStatement));
  AssertEquals('Inactive',True,A.Inactive);
end;

procedure TTestIndexParser.TestCreateIndexSimple;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE INDEX A ON B (C)','A',TSQLCreateIndexStatement));
  If Not (C.Options=[]) then
    Fail('Options empty');
  AssertIdentifiername('Correct table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',1,C.FieldNames.Count);
  AssertIdentifiername('Field name','C',C.FieldNames[0]);
end;

procedure TTestIndexParser.TestIndexIndexDouble;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE INDEX A ON B (C,D)','A',TSQLCreateIndexStatement));
  If Not (C.Options=[]) then
    Fail('Options empty');
  AssertIdentifiername('Correct table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',2,C.FieldNames.Count);
  AssertIdentifiername('Field name 1','C',C.FieldNames[0]);
  AssertIdentifiername('Field name 2','D',C.FieldNames[1]);
end;

procedure TTestIndexParser.TestIndexError1;
begin
  FErrSource:='ALTER UNIQUE INDEX A ACTIVE';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestIndexParser.TestIndexError2;
begin
  FErrSource:='ALTER ASCENDING INDEX A ACTIVE';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestIndexParser.TestIndexError3;
begin
  FErrSource:='ALTER DESCENDING INDEX A ACTIVE';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestIndexParser.TestIndexError4;
begin
  FErrSource:='CREATE INDEX A ON B';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestIndexParser.TestIndexError5;
begin
  FErrSource:='CREATE INDEX A ON B ()';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestIndexParser.TestIndexError6;
begin
  FErrSource:='CREATE INDEX A ON B (A,)';
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestIndexParser.TestCreateIndexUnique;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE UNIQUE INDEX A ON B (C)','A',TSQLCreateIndexStatement));
  If not ([ioUnique]=C.Options) then
    Fail('Not Unique index');
  AssertIdentifierName('Have table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',1,C.FieldNames.Count);
  AssertIdentifierName('Correct field name','C',C.FieldNames[0]);
end;

procedure TTestIndexParser.TestCreateIndexUniqueAscending;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE UNIQUE ASCENDING INDEX A ON B (C)','A',TSQLCreateIndexStatement));
  If not ([ioUnique,ioAscending ]=C.Options) then
    Fail('Not Unique ascending index');
  AssertIdentifierName('Have table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',1,C.FieldNames.Count);
  AssertIdentifierName('Correct field name','C',C.FieldNames[0]);
end;

procedure TTestIndexParser.TestCreateIndexUniqueDescending;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE UNIQUE DESCENDING INDEX A ON B (C)','A',TSQLCreateIndexStatement));
  If not ([ioUnique,ioDescending]=C.Options) then
    Fail('Not Unique descending index');
  AssertIdentifierName('Have table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',1,C.FieldNames.Count);
  AssertIdentifierName('Correct field name','C',C.FieldNames[0]);
end;

procedure TTestIndexParser.TestCreateIndexAscending;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE ASCENDING INDEX A ON B (C)','A',TSQLCreateIndexStatement));
  If not ([ioAscending]=C.Options) then
    Fail('Not ascending index');
  AssertIdentifierName('Have table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',1,C.FieldNames.Count);
  AssertIdentifierName('Correct field name','C',C.FieldNames[0]);
end;

procedure TTestIndexParser.TestCreateIndexDescending;

Var
  C : TSQLCreateIndexStatement;

begin
  C:=TSQLCreateIndexStatement(TestCreateStatement('CREATE DESCENDING INDEX A ON B (C)','A',TSQLCreateIndexStatement));
  If not ([ioDescending] = C.Options) then
    Fail('Not descending index');
  AssertIdentifierName('Table name','B',C.TableName);
  AssertNotNull('Have fieldlist',C.FieldNames);
  AssertEquals('Number of fields',1,C.FieldNames.Count);
  AssertIdentifierName('Correct field name','C',C.FieldNames[0]);
end;

{ TTestTableParser }

procedure TTestTableParser.DoTestCreateReferencesField(const ASource: String;
  AOnUpdate, AOnDelete: TForeignKeyAction);
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  D : TSQLForeignKeyFieldConstraint;

begin

  C:=TSQLCreateTableStatement(TestCreateStatement(ASource,'A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertEquals('Field can be NULL',false,F.FieldType.NotNull);
  AssertNull('Have default',F.FieldType.DefaultValue);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  D:=TSQLForeignKeyFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLForeignKeyFieldConstraint));
  AssertNull('No constraint name',D.ConstraintName);
  AssertIdentifierName('Correct table name','C',D.Definition.TableName);
  AssertEquals('Correct field list count',1,D.Definition.FieldList.Count);
  AssertIdentifierName('Correct field name','D',D.Definition.FieldList[0]);
  AssertEquals('No on update action',AOnUpdate,D.Definition.OnUpdate);
  AssertEquals('No on delete action',AOnDelete,D.Definition.OnDelete);
end;

procedure TTestTableParser.TestCreateOneSimpleField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
end;

procedure TTestTableParser.TestCreateTwoSimpleFields;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, C CHAR(5))','A',TSQLCreateTableStatement));
  AssertEquals('Two fields',2,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[1],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','C',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtChar,F.FieldType.DataType);
end;

procedure TTestTableParser.TestCreateOnePrimaryField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P : TSQLPrimaryKeyFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT PRIMARY KEY)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  P:=TSQLPrimaryKeyFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLPrimaryKeyFieldConstraint));
  AssertNull('No constraint name',P.ConstraintName);
end;

procedure TTestTableParser.TestCreateOneNamedPrimaryField;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P : TSQLPrimaryKeyFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT CONSTRAINT C PRIMARY KEY)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  P:=TSQLPrimaryKeyFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLPrimaryKeyFieldConstraint));
  AssertIdentifierName('Constraint name','C',P.ConstraintName);
end;

procedure TTestTableParser.TestCreateOneUniqueField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  U : TSQLUniqueFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT UNIQUE)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  U:=TSQLUniqueFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLUniqueFieldConstraint));
  AssertNull('No constraint name',U.ConstraintName);
end;

procedure TTestTableParser.TestCreateOneNamedUniqueField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  U : TSQLUniqueFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT CONSTRAINT C UNIQUE)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  U:=TSQLUniqueFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLUniqueFieldConstraint));
  AssertIdentifierName('Constraint name','C',U.ConstraintName);
end;

procedure TTestTableParser.TestCreateNotNullPrimaryField;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT NOT NULL PRIMARY KEY)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertEquals('Field is not NULL',true,F.FieldType.NotNull);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  CheckClass(F.FieldType.Constraint,TSQLPrimaryKeyFieldConstraint);
end;

procedure TTestTableParser.TestCreateNotNullDefaultPrimaryField;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT DEFAULT 0 NOT NULL PRIMARY KEY)','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertEquals('Field is not NULL',true,F.FieldType.NotNull);
  AssertNotNull('Have default',F.FieldType.DefaultValue);
  CheckClass(F.FieldType.DefaultValue,TSQLIntegerLiteral);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  CheckClass(F.FieldType.Constraint,TSQLPrimaryKeyFieldConstraint);
end;

procedure TTestTableParser.TestCreateCheckField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  B : TSQLBinaryExpression;
  CC : TSQLCheckFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT CHECK (B<>0))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertNull('Have no default',F.FieldType.DefaultValue);
  AssertNull('Fieldtype has no check',F.FieldType.Check);
  AssertNotNull('Field has constraint check',F.FieldType.Constraint);
  CC:=TSQLCheckFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLCheckFieldConstraint));
  AssertNull('No constraint name',CC.ConstraintName);
  B:=TSQLBinaryExpression(CheckClass(CC.Expression,TSQLBinaryExpression));
  AssertEquals('Unequal check',boNE,B.Operation);
end;

procedure TTestTableParser.TestCreateNamedCheckField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  B : TSQLBinaryExpression;
  CC : TSQLCheckFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT CONSTRAINT C CHECK (B<>0))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertNull('Have no default',F.FieldType.DefaultValue);
  AssertNull('Fieldtype has no check',F.FieldType.Check);
  AssertNotNull('Field has constraint check',F.FieldType.Constraint);
  CC:=TSQLCheckFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLCheckFieldConstraint));
  AssertidentifierName('Constraint name','C',CC.ConstraintName);
  B:=TSQLBinaryExpression(CheckClass(CC.Expression,TSQLBinaryExpression));
  AssertEquals('Unequal check',boNE,B.Operation);
end;

procedure TTestTableParser.TestCreateReferencesField;

begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D))',fkaNone,fkaNone);
end;

procedure TTestTableParser.TestCreateReferencesOnUpdateCascadeField;

begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON UPDATE CASCADE)',fkaCascade,fkaNone);
end;

procedure TTestTableParser.TestCreateReferencesOnUpdateNoActionField;

begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON UPDATE NO ACTION)',fkaNoAction,fkaNone);
end;

procedure TTestTableParser.TestCreateReferencesOnUpdateSetDefaultField;
begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON UPDATE SET DEFAULT)',fkaSetDefault,fkaNone);
end;

procedure TTestTableParser.TestCreateReferencesOnUpdateSetNullField;
begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON UPDATE SET NULL)',fkaSetNull,fkaNone);
end;

procedure TTestTableParser.TestCreateReferencesOnDeleteCascadeField;

begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON DELETE CASCADE)',fkaNone,fkaCascade);
end;

procedure TTestTableParser.TestCreateReferencesOnDeleteNoActionField;

begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON DELETE NO ACTION)',fkaNone,fkaNoAction);
end;

procedure TTestTableParser.TestCreateReferencesOnDeleteSetDefaultField;
begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON DELETE SET DEFAULT)',fkaNone,fkaSetDefault);
end;

procedure TTestTableParser.TestCreateReferencesOnDeleteSetNullField;
begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON DELETE SET NULL)',fkaNone,fkaSetNull);
end;

procedure TTestTableParser.TestCreateReferencesOnUpdateAndDeleteSetNullField;
begin
  DoTestCreateReferencesField('CREATE TABLE A (B INT REFERENCES C(D) ON UPDATE SET NULL ON DELETE SET NULL)',fkaSetNull,fkaSetNull);
end;

procedure TTestTableParser.TestCreateNamedReferencesField;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  D : TSQLForeignKeyFieldConstraint;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT CONSTRAINT FK REFERENCES C(D))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  AssertNotNull('Have field type',F.FieldType);
  AssertEquals('Correct field type',sdtInteger,F.FieldType.DataType);
  AssertEquals('Field can be NULL',false,F.FieldType.NotNull);
  AssertNull('Have default',F.FieldType.DefaultValue);
  AssertNotNull('Have constraint',F.FieldType.Constraint);
  D:=TSQLForeignKeyFieldConstraint(CheckClass(F.FieldType.Constraint,TSQLForeignKeyFieldConstraint));
  AssertIdentifierName('Correct constraint name','FK',D.ConstraintName);
  AssertIdentifierName('Correct table name','C',D.Definition.TableName);
  AssertEquals('Correct field list count',1,D.Definition.FieldList.Count);
  AssertIdentifierName('Correct field name','D',D.Definition.FieldList[0]);
end;

procedure TTestTableParser.TestCreateComputedByField;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  B : TSQLBinaryExpression;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, C INT, D COMPUTED BY (B+C))','A',TSQLCreateTableStatement));
  AssertEquals('Three fields',3,C.FieldDefs.Count);
  AssertEquals('No constraints',0,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[2],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','D',F.FieldName);
  AssertNull('No field type',F.FieldType);
  AssertNotNull('Have computed by expression',F.ComputedBy);
  B:=TSQLBinaryExpression(CheckClass(F.ComputedBy,TSQLBinaryExpression));
  AssertEquals('Add operation',boAdd,B.Operation);
  CheckClass(B.Left,TSQLIdentifierExpression);
  AssertIdentifierName('Correct identifier','B',TSQLIdentifierExpression(B.Left).Identifier);
  CheckClass(B.Right,TSQLIdentifierExpression);
  AssertIdentifierName('Correct identifier','C',TSQLIdentifierExpression(B.Right).Identifier);
end;

procedure TTestTableParser.TestCreatePrimaryKeyConstraint;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P: TSQLTablePrimaryKeyConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, PRIMARY KEY (B))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTablePrimaryKeyConstraintDef(CheckClass(C.Constraints[0],TSQLTablePrimaryKeyConstraintDef));
  AssertNotNull('Fieldlist assigned',P.FieldList);
  AssertNull('Constraint name empty',P.ConstraintName);
  AssertEquals('One field in primary key',1,P.FieldList.Count);
  AssertIdentifierName('fieldname','B',P.FieldList[0]);
end;

procedure TTestTableParser.TestCreateNamedPrimaryKeyConstraint;

Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P: TSQLTablePrimaryKeyConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, CONSTRAINT A_PK PRIMARY KEY (B))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTablePrimaryKeyConstraintDef(CheckClass(C.Constraints[0],TSQLTablePrimaryKeyConstraintDef));
  AssertNotNull('Fieldlist assigned',P.FieldList);
  AssertIdentifierName('fieldname','A_PK',P.ConstraintName);
  AssertEquals('One field in primary key',1,P.FieldList.Count);
  AssertIdentifierName('fieldname','B',P.FieldList[0]);
end;

procedure TTestTableParser.TestCreateForeignKeyConstraint;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P: TSQLTableForeignKeyConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, FOREIGN KEY (B) REFERENCES C(D))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTableForeignKeyConstraintDef(CheckClass(C.Constraints[0],TSQLTableForeignKeyConstraintDef));
  AssertNotNull('Fieldlist assigned',P.FieldList);
  AssertNull('Constraint name',P.ConstraintName);
  AssertEquals('One field in foreign key',1,P.FieldList.Count);
  AssertIdentifierName('fieldname','B',P.FieldList[0]);
  AssertIdentifierName('Target table name','C',P.Definition.TableName);
  AssertEquals('One field in primary key target',1,P.Definition.FieldList.Count);
  AssertIdentifierName('target fieldname','D',P.Definition.FieldList[0]);
end;

procedure TTestTableParser.TestCreateNamedForeignKeyConstraint;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P: TSQLTableForeignKeyConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, CONSTRAINT A_FK FOREIGN KEY (B) REFERENCES C(D))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTableForeignKeyConstraintDef(CheckClass(C.Constraints[0],TSQLTableForeignKeyConstraintDef));
  AssertNotNull('Fieldlist assigned',P.FieldList);
  AssertIdentifierName('fieldname','A_FK',P.ConstraintName);
  AssertEquals('One field in foreign key',1,P.FieldList.Count);
  AssertIdentifierName('fieldname','B',P.FieldList[0]);
  AssertIdentifierName('Target table name','C',P.Definition.TableName);
  AssertEquals('One field in primary key target',1,P.Definition.FieldList.Count);
  AssertIdentifierName('target fieldname','D',P.Definition.FieldList[0]);
end;

procedure TTestTableParser.TestCreateUniqueConstraint;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P: TSQLTableUniqueConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, UNIQUE (B))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTableUniqueConstraintDef(CheckClass(C.Constraints[0],TSQLTableUniqueConstraintDef));
  AssertNotNull('Fieldlist assigned',P.FieldList);
  AssertNull('Constraint name empty',P.ConstraintName);
  AssertEquals('One field in primary key',1,P.FieldList.Count);
  AssertIdentifierName('Name is correct','B',P.FieldList[0]);
end;

procedure TTestTableParser.TestCreateNamedUniqueConstraint;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  P: TSQLTableUniqueConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, CONSTRAINT U_A UNIQUE (B))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTableUniqueConstraintDef(CheckClass(C.Constraints[0],TSQLTableUniqueConstraintDef));
  AssertNotNull('Fieldlist assigned',P.FieldList);
  AssertIdentifierName('fieldname','U_A',P.ConstraintName);
  AssertEquals('One field in primary key',1,P.FieldList.Count);
  AssertIdentifierName('Name is correct','B',P.FieldList[0]);
end;

procedure TTestTableParser.TestCreateCheckConstraint;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  B : TSQLBinaryExpression;
  P: TSQLTableCheckConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, CHECK (B<>0))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTableCheckConstraintDef(CheckClass(C.Constraints[0],TSQLTableCheckConstraintDef));
  AssertNull('Constraint name empty',P.ConstraintName);
  AssertNotNull('Check expression assigned',P.Check);
  B:=TSQLBinaryExpression(CheckClass(P.Check,TSQLBinaryExpression));
  AssertEquals('Unequal',boNE,B.Operation);
end;

procedure TTestTableParser.TestCreateNamedCheckConstraint;
Var
  C : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  B : TSQLBinaryExpression;
  P: TSQLTableCheckConstraintDef;

begin
  C:=TSQLCreateTableStatement(TestCreateStatement('CREATE TABLE A (B INT, CONSTRAINT C_A CHECK (B<>0))','A',TSQLCreateTableStatement));
  AssertEquals('One field',1,C.FieldDefs.Count);
  AssertEquals('One constraints',1,C.Constraints.Count);
  F:=TSQLTableFieldDef(CheckClass(C.FieldDefs[0],TSQLTableFieldDef));
  AssertIdentifierName('fieldname','B',F.FieldName);
  P:=TSQLTableCheckConstraintDef(CheckClass(C.Constraints[0],TSQLTableCheckConstraintDef));
  AssertIdentifierName('Constainrname','C_A',P.ConstraintName);
  AssertNotNull('Check expression assigned',P.Check);
  B:=TSQLBinaryExpression(CheckClass(P.Check,TSQLBinaryExpression));
  AssertEquals('Not equal operation',boNE,B.Operation);
end;

procedure TTestTableParser.TestAlterDropField;
Var
  A : TSQLAlterTableStatement;
  D : TSQLDropTableFieldOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A DROP B','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  D:=TSQLDropTableFieldOperation(CheckClass(A.Operations[0],TSQLDropTableFieldOperation));
  AssertidentifierName('Drop field name','B',D.ObjectName);
end;

procedure TTestTableParser.TestAlterDropFields;
Var
  A : TSQLAlterTableStatement;
  D : TSQLDropTableFieldOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A DROP B, DROP C','A',TSQLAlterTableStatement));
  AssertEquals('Two operations',2,A.Operations.Count);
  D:=TSQLDropTableFieldOperation(CheckClass(A.Operations[0],TSQLDropTableFieldOperation));
  AssertidentifierName('Drop field name','B',D.ObjectName);
  D:=TSQLDropTableFieldOperation(CheckClass(A.Operations[1],TSQLDropTableFieldOperation));
  AssertidentifierName('Drop field name','C',D.ObjectName);
end;

procedure TTestTableParser.TestAlterDropConstraint;
Var
  A : TSQLAlterTableStatement;
  D : TSQLDropTableConstraintOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A DROP CONSTRAINT B','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  D:=TSQLDropTableConstraintOperation(CheckClass(A.Operations[0],TSQLDropTableConstraintOperation));
  AssertidentifierName('Drop field name','B',D.ObjectName);
end;

procedure TTestTableParser.TestAlterDropConstraints;
Var
  A : TSQLAlterTableStatement;
  D : TSQLDropTableConstraintOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A DROP CONSTRAINT B, DROP CONSTRAINT C','A',TSQLAlterTableStatement));
  AssertEquals('Two operations',2,A.Operations.Count);
  D:=TSQLDropTableConstraintOperation(CheckClass(A.Operations[0],TSQLDropTableConstraintOperation));
  AssertidentifierName('Drop Constraint name','B',D.ObjectName);
  D:=TSQLDropTableConstraintOperation(CheckClass(A.Operations[1],TSQLDropTableConstraintOperation));
  AssertidentifierName('Drop field name','C',D.ObjectName);
end;

procedure TTestTableParser.TestAlterRenameField;
Var
  A : TSQLAlterTableStatement;
  R : TSQLAlterTableFieldNameOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ALTER B TO C','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  R:=TSQLAlterTableFieldNameOperation(CheckClass(A.Operations[0],TSQLAlterTableFieldNameOperation));
  AssertidentifierName('Old field name','B',R.ObjectName);
  AssertidentifierName('New field name','C',R.NewName);
end;
procedure TTestTableParser.TestAlterRenameColumnField;
Var
  A : TSQLAlterTableStatement;
  R : TSQLAlterTableFieldNameOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ALTER COLUMN B TO C','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  R:=TSQLAlterTableFieldNameOperation(CheckClass(A.Operations[0],TSQLAlterTableFieldNameOperation));
  AssertidentifierName('Old field name','B',R.ObjectName);
  AssertidentifierName('New field name','C',R.NewName);
end;

procedure TTestTableParser.TestAlterFieldType;
Var
  A : TSQLAlterTableStatement;
  R : TSQLAlterTableFieldTypeOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ALTER COLUMN B TYPE INT','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  R:=TSQLAlterTableFieldTypeOperation(CheckClass(A.Operations[0],TSQLAlterTableFieldTypeOperation));
  AssertidentifierName('Old field name','B',R.ObjectName);
  AssertNotNull('Have field type',R.NewType);
  Checkclass(R.NewType,TSQLTypeDefinition);
  AssertEquals('Correct data type',sdtInteger,R.NewType.DataType);
end;

procedure TTestTableParser.TestAlterFieldPosition;
Var
  A : TSQLAlterTableStatement;
  R : TSQLAlterTableFieldPositionOperation;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ALTER COLUMN B POSITION 3','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  R:=TSQLAlterTableFieldPositionOperation(CheckClass(A.Operations[0],TSQLAlterTableFieldPositionOperation));
  AssertidentifierName('Old field name','B',R.ObjectName);
  AssertEquals('Correct position',3,R.NewPosition);
end;

procedure TTestTableParser.TestAlterAddField;
Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddFieldOperation;
  D : TSQLTableFieldDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD B INT','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddFieldOperation(CheckClass(A.Operations[0],TSQLAlterTableAddFieldOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableFieldDef(CheckClass(F.Element,TSQLTableFieldDef));
  AssertIdentifierName('New field name','B',D.FieldName);
  AssertNotNull('Have fielddef',D.FieldType);
  AssertEquals('Correct field type',sdtINteger,D.FieldType.DataType);
end;

procedure TTestTableParser.TestAlterAddFields;

Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddFieldOperation;
  D : TSQLTableFieldDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD B INT, ADD C CHAR(50)','A',TSQLAlterTableStatement));
  AssertEquals('Two operations',2,A.Operations.Count);
  F:=TSQLAlterTableAddFieldOperation(CheckClass(A.Operations[0],TSQLAlterTableAddFieldOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableFieldDef(CheckClass(F.Element,TSQLTableFieldDef));
  AssertIdentifierName('New field name','B',D.FieldName);
  AssertNotNull('Have fielddef',D.FieldType);
  AssertEquals('Correct field type',sdtINteger,D.FieldType.DataType);
  F:=TSQLAlterTableAddFieldOperation(CheckClass(A.Operations[1],TSQLAlterTableAddFieldOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableFieldDef(CheckClass(F.Element,TSQLTableFieldDef));
  AssertIdentifierName('New field name','C',D.FieldName);
  AssertNotNull('Have fielddef',D.FieldType);
  AssertEquals('Correct field type',sdtChar,D.FieldType.DataType);
  AssertEquals('Correct field lengthe',50,D.FieldType.Len);
end;

procedure TTestTableParser.TestAlterAddPrimarykey;
Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddConstraintOperation;
  D : TSQLTablePrimaryKeyConstraintDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD PRIMARY KEY (B)','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddConstraintOperation(CheckClass(A.Operations[0],TSQLAlterTableAddConstraintOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTablePrimaryKeyConstraintDef(CheckClass(F.Element,TSQLTablePrimaryKeyConstraintDef));
  AssertNull('No constraint name',D.ConstraintName);
  AssertEquals('Have 1 field',1,D.FieldList.Count);
  AssertIdentifierName('fieldname','B',D.FieldList[0]);
end;

procedure TTestTableParser.TestAlterAddNamedPrimarykey;
Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddConstraintOperation;
  D : TSQLTablePrimaryKeyConstraintDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD CONSTRAINT U_K PRIMARY KEY (B)','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddConstraintOperation(CheckClass(A.Operations[0],TSQLAlterTableAddConstraintOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTablePrimaryKeyConstraintDef(CheckClass(F.Element,TSQLTablePrimaryKeyConstraintDef));
  AssertIdentifierName('No constraint name','U_K',D.ConstraintName);
  AssertEquals('Have 1 field',1,D.FieldList.Count);
  AssertIdentifierName('fieldname','B',D.FieldList[0]);
end;

procedure TTestTableParser.TestAlterAddCheckConstraint;

Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddConstraintOperation;
  D : TSQLTableCheckConstraintDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD CHECK (B<>0)','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddConstraintOperation(CheckClass(A.Operations[0],TSQLAlterTableAddConstraintOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableCheckConstraintDef(CheckClass(F.Element,TSQLTableCheckConstraintDef));
  AssertNull('Constaintname',D.ConstraintName);
  AssertNotNull('Check expression assigned',D.Check);
  CheckClass(D.Check,TSQLBinaryExpression);
end;

procedure TTestTableParser.TestAlterAddNamedCheckConstraint;

Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddConstraintOperation;
  D : TSQLTableCheckConstraintDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD CONSTRAINT C_A CHECK (B<>0)','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddConstraintOperation(CheckClass(A.Operations[0],TSQLAlterTableAddConstraintOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableCheckConstraintDef(CheckClass(F.Element,TSQLTableCheckConstraintDef));
  AssertIdentifierName('Constaintname','C_A',D.ConstraintName);
  AssertNotNull('Check expression assigned',D.Check);
  CheckClass(D.Check,TSQLBinaryExpression);
end;

procedure TTestTableParser.TestAlterAddForeignkey;

Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddConstraintOperation;
  D : TSQLTableForeignKeyConstraintDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD FOREIGN KEY (B) REFERENCES C(D)','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddConstraintOperation(CheckClass(A.Operations[0],TSQLAlterTableAddConstraintOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableForeignKeyConstraintDef(CheckClass(F.Element,TSQLTableForeignKeyConstraintDef));
  AssertNull('No constraint name',D.ConstraintName);
  AssertEquals('Have 1 field',1,D.FieldList.Count);
  AssertIdentifierName('fieldname','B',D.FieldList[0]);
  AssertIdentifierName('Target table name','C',D.Definition.TableName);
  AssertEquals('One field in primary key target',1,D.Definition.FieldList.Count);
  AssertIdentifierName('target fieldname','D',D.Definition.FieldList[0]);
end;

procedure TTestTableParser.TestAlterAddNamedForeignkey;
Var
  A : TSQLAlterTableStatement;
  F : TSQLAlterTableAddConstraintOperation;
  D : TSQLTableForeignKeyConstraintDef;

begin
  A:=TSQLAlterTableStatement(TestCreateStatement('ALTER TABLE A ADD CONSTRAINT F_A FOREIGN KEY (B) REFERENCES C(D)','A',TSQLAlterTableStatement));
  AssertEquals('One operation',1,A.Operations.Count);
  F:=TSQLAlterTableAddConstraintOperation(CheckClass(A.Operations[0],TSQLAlterTableAddConstraintOperation));
  AssertNotNull('Have element',F.Element);
  D:=TSQLTableForeignKeyConstraintDef(CheckClass(F.Element,TSQLTableForeignKeyConstraintDef));
  AssertIdentifierName('constraint name','F_A',D.ConstraintName);
  AssertEquals('Have 1 field',1,D.FieldList.Count);
  AssertIdentifierName('fieldname','B',D.FieldList[0]);
  AssertIdentifierName('Target table name','C',D.Definition.TableName);
  AssertEquals('One field in primary key target',1,D.Definition.FieldList.Count);
  AssertIdentifierName('target fieldname','D',D.Definition.FieldList[0]);
end;

{ TTestDeleteParser }

function TTestDeleteParser.TestDelete(const ASource,ATable: String
  ): TSQLDeleteStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLDeleteStatement(CheckClass(FToFree,TSQLDeleteStatement));
  AssertIdentifierName('Correct table name',ATable,Result.TableName);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestDeleteParser.TestSimpleDelete;

Var
  D : TSQLDeleteStatement;

begin
  D:=TestDelete('DELETE FROM A','A');
  AssertNull('No where',D.WhereClause);
end;

procedure TTestDeleteParser.TestSimpleDeleteAlias;
Var
  D : TSQLDeleteStatement;
begin
  D:=TestDelete('DELETE FROM A B','A');
  AssertIdentifierName('Alias name','B',D.AliasName);
  AssertNull('No where',D.WhereClause);
end;

procedure TTestDeleteParser.TestDeleteWhereNull;
Var
  D : TSQLDeleteStatement;
  B : TSQLBinaryExpression;
  I : TSQLIdentifierExpression;
  L : TSQLLiteralExpression;

begin
  D:=TestDelete('DELETE FROM A WHERE B IS NULL','A');
  AssertNotNull('No where',D.WhereClause);
  B:=TSQLBinaryExpression(CheckClass(D.WhereClause,TSQLBinaryExpression));
  AssertEquals('Is null operation',boIs,B.Operation);
  I:=TSQLIdentifierExpression(CheckClass(B.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Correct field name','B',I.Identifier);
  L:=TSQLLiteralExpression(CheckClass(B.Right,TSQLLiteralExpression));
  CheckClass(L.Literal,TSQLNullLiteral);
end;

{ TTestUpdateParser }

function TTestUpdateParser.TestUpdate(const ASource, ATable: String
  ): TSQLUpdateStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLUpdateStatement(CheckClass(FToFree,TSQLUpdateStatement));
  AssertIdentifierName('Correct table name',ATable,Result.TableName);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestUpdateParser.TestUpdateOneField;

Var
  U : TSQLUpdateStatement;
  P : TSQLUpdatePair;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;

begin
  U:=TestUpdate('UPDATE A SET B=1','A');
  AssertEquals('One field updated',1,U.Values.Count);
  P:=TSQLUpdatePair(CheckClass(U.Values[0],TSQLUpdatePair));
  AssertIdentifierName('Correct field name','B',P.FieldName);
  E:=TSQLLiteralExpression(CheckClass(P.Value,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Value 1',1,I.Value);
  AssertNull('No where clause',U.WhereClause);
end;

procedure TTestUpdateParser.TestUpdateOneFieldFull;
Var
  U : TSQLUpdateStatement;
  P : TSQLUpdatePair;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;

begin
  U:=TestUpdate('UPDATE A SET A.B=1','A');
  AssertEquals('One field updated',1,U.Values.Count);
  P:=TSQLUpdatePair(CheckClass(U.Values[0],TSQLUpdatePair));
  AssertIdentifierName('Correct field name','A.B',P.FieldName);
  E:=TSQLLiteralExpression(CheckClass(P.Value,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Value 1',1,I.Value);
  AssertNull('No where clause',U.WhereClause);
end;

procedure TTestUpdateParser.TestUpdateTwoFields;
Var
  U : TSQLUpdateStatement;
  P : TSQLUpdatePair;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;

begin
  U:=TestUpdate('UPDATE A SET B=1, C=2','A');
  AssertEquals('One field updated',2,U.Values.Count);
  P:=TSQLUpdatePair(CheckClass(U.Values[0],TSQLUpdatePair));
  AssertIdentifierName('Correct field name','B',P.FieldName);
  E:=TSQLLiteralExpression(CheckClass(P.Value,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Value 1',1,I.Value);
  P:=TSQLUpdatePair(CheckClass(U.Values[1],TSQLUpdatePair));
  AssertIdentifierName('Correct field name','C',P.FieldName);
  E:=TSQLLiteralExpression(CheckClass(P.Value,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Value 2',2,I.Value);
  AssertNull('No where clause',U.WhereClause);
end;

procedure TTestUpdateParser.TestUpdateOneFieldWhereIsNull;
Var
  U : TSQLUpdateStatement;
  P : TSQLUpdatePair;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;
  B : TSQLBinaryExpression;
  IE : TSQLIdentifierExpression;
  L : TSQLLiteralExpression;

begin
  U:=TestUpdate('UPDATE A SET B=1 WHERE B IS NULL','A');
  AssertEquals('One field updated',1,U.Values.Count);
  P:=TSQLUpdatePair(CheckClass(U.Values[0],TSQLUpdatePair));
  AssertIdentifierName('Correct field name','B',P.FieldName);
  E:=TSQLLiteralExpression(CheckClass(P.Value,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Value 1',1,I.Value);
  AssertNotNull('where clause',U.WhereClause);
  B:=TSQLBinaryExpression(CheckClass(U.WhereClause,TSQLBinaryExpression));
  AssertEquals('Is null operation',boIs,B.Operation);
  IE:=TSQLIdentifierExpression(CheckClass(B.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Correct field name','B',IE.Identifier);
  L:=TSQLLiteralExpression(CheckClass(B.Right,TSQLLiteralExpression));
  CheckClass(L.Literal,TSQLNullLiteral);
end;

{ TTestInsertParser }

function TTestInsertParser.TestInsert(const ASource, ATable: String
  ): TSQLInsertStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLInsertStatement(CheckClass(FToFree,TSQLInsertStatement));
  AssertIdentifierName('Correct table name',ATable,Result.TableName);
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestInsertParser.TestInsertOneField;

Var
  I : TSQLInsertStatement;
  E : TSQLLiteralExpression;
  L : TSQLIntegerLiteral;

begin
  I:=TestInsert('INSERT INTO A (B) VALUES (1)','A');
  AssertNotNull('Have fields',I.Fields);
  AssertEquals('1 field',1,I.Fields.Count);
  AssertIdentifierName('Correct field name','B',I.Fields[0]);
  AssertNotNull('Have values',I.Values);
  AssertEquals('Have 1 value',1,I.Values.Count);
  E:=TSQLLiteralExpression(CheckClass(I.Values[0],TSQLLiteralExpression));
  L:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,L.Value);
end;

procedure TTestInsertParser.TestInsertTwoFields;

Var
  I : TSQLInsertStatement;
  E : TSQLLiteralExpression;
  L : TSQLIntegerLiteral;

begin
  I:=TestInsert('INSERT INTO A (B,C) VALUES (1,2)','A');
  AssertNotNull('Have fields',I.Fields);
  AssertEquals('2 fields',2,I.Fields.Count);
  AssertIdentifierName('Correct field 1 name','B',I.Fields[0]);
  AssertIdentifierName('Correct field 2 name','C',I.Fields[1]);
  AssertNotNull('Have values',I.Values);
  AssertEquals('Have 2 values',2,I.Values.Count);
  E:=TSQLLiteralExpression(CheckClass(I.Values[0],TSQLLiteralExpression));
  L:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,L.Value);
  E:=TSQLLiteralExpression(CheckClass(I.Values[1],TSQLLiteralExpression));
  L:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',2,L.Value);
end;

procedure TTestInsertParser.TestInsertOneValue;
Var
  I : TSQLInsertStatement;
  E : TSQLLiteralExpression;
  L : TSQLIntegerLiteral;

begin
  I:=TestInsert('INSERT INTO A VALUES (1)','A');
  AssertNull('Have no fields',I.Fields);
  AssertNotNull('Have values',I.Values);
  AssertEquals('Have 1 value',1,I.Values.Count);
  E:=TSQLLiteralExpression(CheckClass(I.Values[0],TSQLLiteralExpression));
  L:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,L.Value);
end;

procedure TTestInsertParser.TestInsertTwoValues;
Var
  I : TSQLInsertStatement;
  E : TSQLLiteralExpression;
  L : TSQLIntegerLiteral;

begin
  I:=TestInsert('INSERT INTO A VALUES (1,2)','A');
  AssertNull('Have no fields',I.Fields);
  AssertNotNull('Have values',I.Values);
  AssertEquals('Have 2 values',2,I.Values.Count);
  E:=TSQLLiteralExpression(CheckClass(I.Values[0],TSQLLiteralExpression));
  L:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,L.Value);
  E:=TSQLLiteralExpression(CheckClass(I.Values[1],TSQLLiteralExpression));
  L:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',2,L.Value);
end;

{ TTestSelectParser }

function TTestSelectParser.TestSelect(const ASource : String): TSQLSelectStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLSelectStatement(CheckClass(FToFree,TSQLSelectStatement));
  FSelect:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestSelectParser.TestSelectError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestSelectParser.TestSelectOneFieldOneTable;
begin
  TestSelect('SELECT B FROM A');
  AssertNull('No transaction name',Select.TransactionName);
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneFieldOneTableTransaction;
begin
  TestSelect('SELECT TRANSACTION C B FROM A');
  AssertIdentifierName('Correct transaction name','C',Select.TransactionName);
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneArrayFieldOneTable;
Var
  E : TSQLIdentifierExpression;
begin
  TestSelect('SELECT B[1] FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  E:=TSQLIdentifierExpression(CheckClass(TSQLSelectField(Select.Fields[0]).Expression,TSQLIdentifierExpression));
  AssertEquals('Element 1 in array ',1,E.ElementIndex);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectTwoFieldsOneTable;
begin
  TestSelect('SELECT B,C FROM A');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneFieldAliasOneTable;

begin
  TestSelect('SELECT B AS C FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B','C');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectTwoFieldAliasesOneTable;
begin
  TestSelect('SELECT B AS D,C AS E FROM A');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B','D');
  AssertField(Select.Fields[1],'C','E');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneTableFieldOneTable;

begin
  TestSelect('SELECT A.B FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  // Field does not support linking/refering to a table, so the field name is
  // assigned as A.B (instead of B with a <link to table A>)
  AssertField(Select.Fields[0],'A.B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneDistinctFieldOneTable;
begin
  TestSelect('SELECT DISTINCT B FROM A');
  AssertEquals('DISTINCT Query',True,Select.Distinct);
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneAllFieldOneTable;
begin
  TestSelect('SELECT ALL B FROM A');
  AssertEquals('ALL Query',True,Select.All);
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectAsteriskOneTable;
begin
  TestSelect('SELECT * FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  CheckClass(Select.Fields[0],TSQLSelectAsterisk);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectDistinctAsteriskOneTable;
begin
  TestSelect('SELECT DISTINCT * FROM A');
  AssertEquals('DISTINCT Query',True,Select.Distinct);
  AssertEquals('One field',1,Select.Fields.Count);
  CheckClass(Select.Fields[0],TSQLSelectAsterisk);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneFieldOneTableAlias;
begin
  TestSelect('SELECT C.B FROM A C');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'C.B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectOneFieldOneTableAsAlias;
begin
  TestSelect('SELECT C.B FROM A AS C');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'C.B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoTables;
begin
  TestSelect('SELECT B,C FROM A,D');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('Two table',2,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertTable(Select.Tables[1],'D');
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoTablesJoin;

Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A JOIN D ON E=F');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  J:=AssertJoin(Select.Tables[0],'A','D',jtNone);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoInnerTablesJoin;
Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A INNER JOIN D ON E=F');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  J:=AssertJoin(Select.Tables[0],'A','D',jtInner);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoFullOuterTablesJoin;
Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A FULL OUTER JOIN D ON E=F');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  J:=AssertJoin(Select.Tables[0],'A','D',jtFullOuter);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoFullTablesJoin;
Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A FULL JOIN D ON E=F');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  J:=AssertJoin(Select.Tables[0],'A','D',jtFullOuter);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoLeftTablesJoin;
Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A LEFT JOIN D ON E=F');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  J:=AssertJoin(Select.Tables[0],'A','D',jtLeft);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsTwoRightTablesJoin;
Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A RIGHT JOIN D ON E=F');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  J:=AssertJoin(Select.Tables[0],'A','D',jtRight);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsThreeTablesJoin;

Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A JOIN D ON E=F JOIN G ON (H=I)');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  j:=AssertJoin(Select.Tables[0],'','G',jtNone);
  AssertJoinOn(J.JoinClause,'H','I',boEq);
  J:=AssertJoin(J.Left,'A','D',jtNone);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsBracketThreeTablesJoin;

Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM (A JOIN D ON E=F) JOIN G ON (H=I)');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  j:=AssertJoin(Select.Tables[0],'','G',jtNone);
  AssertJoinOn(J.JoinClause,'H','I',boEq);
  J:=AssertJoin(J.Left,'A','D',jtNone);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestSelectTwoFieldsThreeBracketTablesJoin;
Var
  J : TSQLJoinTableReference;

begin
  TestSelect('SELECT B,C FROM A JOIN (D JOIN G ON E=F)  ON (H=I)');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertEquals('One table',1,Select.Tables.Count);
  j:=AssertJoin(Select.Tables[0],'A','',jtNone);
  AssertJoinOn(J.JoinClause,'H','I',boEq);
  j:=AssertJoin(J.Right,'D','G',jtNone);
  AssertJoinOn(J.JoinClause,'E','F',boEq);
end;

procedure TTestSelectParser.TestAggregateCount;

begin
  TestSelect('SELECT COUNT(B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afCount,'B',aoNone,'');
end;

procedure TTestSelectParser.TestAggregateCountAsterisk;

begin
  TestSelect('SELECT COUNT(*) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afCount,'',aoAsterisk,'');
end;

procedure TTestSelectParser.TestAggregateCountAll;
begin
  TestSelect('SELECT COUNT(ALL B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afCount,'B',aoAll,'');
end;

procedure TTestSelectParser.TestAggregateCountDistinct;
begin
  TestSelect('SELECT COUNT(DISTINCT B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afCount,'B',aoDistinct,'');
end;

procedure TTestSelectParser.TestAggregateMax;

begin
  TestSelect('SELECT MAX(B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afMax,'B',aoNone,'');
end;

procedure TTestSelectParser.TestAggregateMaxAsterisk;

begin
  TestSelectError('SELECT Max(*) FROM A');
end;

procedure TTestSelectParser.TestAggregateMaxAll;
begin
  TestSelect('SELECT MAX(ALL B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afMax,'B',aoAll,'');
end;

procedure TTestSelectParser.TestAggregateMaxDistinct;
begin
  TestSelect('SELECT MAX(DISTINCT B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afMax,'B',aoDistinct,'');
end;

procedure TTestSelectParser.TestAggregateMin;

begin
  TestSelect('SELECT Min(B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afMin,'B',aoNone,'');
end;

procedure TTestSelectParser.TestAggregateMinAsterisk;

begin
  TestSelectError('SELECT Min(*) FROM A');
end;

procedure TTestSelectParser.TestAggregateMinAll;
begin
  TestSelect('SELECT Min(ALL B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afMin,'B',aoAll,'');
end;

procedure TTestSelectParser.TestAggregateMinDistinct;
begin
  TestSelect('SELECT Min(DISTINCT B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afMin,'B',aoDistinct,'');
end;

procedure TTestSelectParser.TestAggregateSum;

begin
  TestSelect('SELECT Sum(B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afSum,'B',aoNone,'');
end;

procedure TTestSelectParser.TestAggregateSumAsterisk;

begin
  TestSelectError('SELECT Sum(*) FROM A');
end;

procedure TTestSelectParser.TestAggregateSumAll;
begin
  TestSelect('SELECT Sum(ALL B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afSum,'B',aoAll,'');
end;

procedure TTestSelectParser.TestAggregateSumDistinct;
begin
  TestSelect('SELECT Sum(DISTINCT B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afSum,'B',aoDistinct,'');
end;

procedure TTestSelectParser.TestAggregateAvg;

begin
  TestSelect('SELECT Avg(B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afAvg,'B',aoNone,'');
end;

procedure TTestSelectParser.TestAggregateAvgAsterisk;

begin
  TestSelectError('SELECT Avg(*) FROM A');
end;

procedure TTestSelectParser.TestAggregateAvgAll;
begin
  TestSelect('SELECT Avg(ALL B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afAvg,'B',aoAll,'');
end;

procedure TTestSelectParser.TestAggregateAvgDistinct;
begin
  TestSelect('SELECT Avg(DISTINCT B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertAggregate(Select.Fields[0],afAvg,'B',aoDistinct,'');
end;

procedure TTestSelectParser.TestUpperConst;

Var
  E : TSQLFunctionCallExpression;
  L : TSQLLiteralExpression;
  S : TSQLStringLiteral;

begin
  TestSelect('SELECT UPPER(''a'') FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  CheckClass(Select.Fields[0],TSQLSelectField);
  E:=TSQLFunctionCallExpression(CheckClass(TSQLSelectField(Select.Fields[0]).Expression,TSQLFunctionCallExpression));
  AssertEquals('UPPER function name','UPPER',E.Identifier);
  AssertEquals('One function element',1,E.Arguments.Count);
  L:=TSQLLiteralExpression(CheckClass(E.Arguments[0],TSQLLiteralExpression));
  S:=TSQLStringLiteral(CheckClass(L.Literal,TSQLStringLiteral));
  AssertEquals('Correct constant','a',S.Value);
end;

procedure TTestSelectParser.TestUpperError;

begin
  TestSelectError('SELECT UPPER(''A'',''B'') FROM C');
end;

procedure TTestSelectParser.TestGenID;
Var
  E : TSQLGenIDExpression;
  L : TSQLLiteralExpression;
  S : TSQLIntegerLiteral;

begin
  TestSelect('SELECT GEN_ID(GEN_B,1) FROM RDB$DATABASE');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'RDB$DATABASE');
  CheckClass(Select.Fields[0],TSQLSelectField);
  E:=TSQLGenIDExpression(CheckClass(TSQLSelectField(Select.Fields[0]).Expression,TSQLGenIDExpression));
  AssertIdentifierName('GenID generator function name','GEN_B',E.Generator);
  L:=TSQLLiteralExpression(CheckClass(E.Value,TSQLLiteralExpression));
  S:=TSQLIntegerLiteral(CheckClass(L.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct constant',1,S.Value);
end;

procedure TTestSelectParser.TestGenIDError1;
begin
  TestSelectError('SELECT GEN_ID(''GEN_B'',1) FROM RDB$DATABASE');
end;

procedure TTestSelectParser.TestGenIDError2;
begin
  TestSelectError('SELECT GEN_ID(''GEN_B'') FROM RDB$DATABASE');
end;

procedure TTestSelectParser.TestCastSimple;
var
  C : TSQLCastExpression;
  L : TSQLLiteralExpression;
  S : TSQLIntegerLiteral;

begin
  TestSelect('SELECT CAST(1 AS VARCHAR(5)) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  CheckClass(Select.Fields[0],TSQLSelectField);
  C:=TSQLCastExpression(CheckClass(TSQLSelectField(Select.Fields[0]).Expression,TSQLCastExpression));
  L:=TSQLLiteralExpression(CheckClass(C.Value,TSQLLiteralExpression));
  S:=TSQLIntegerLiteral(CheckClass(L.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct constant',1,S.Value);
  AssertTypeDefaults(C.NewType,5);
  AssertEquals('Correct type',sdtVarChar,C.NewType.DataType);
end;

procedure TTestSelectParser.DoExtractSimple(Expected: TSQLExtractElement);

var
  E : TSQLExtractExpression;
  I : TSQLIdentifierExpression;

begin
  TestSelect('SELECT EXTRACT('+ExtractElementNames[Expected]+' FROM B) FROM A');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  CheckClass(Select.Fields[0],TSQLSelectField);
  E:=TSQLExtractExpression(CheckClass(TSQLSelectField(Select.Fields[0]).Expression,TSQLExtractExpression));
  I:=TSQLIdentifierExpression(CheckClass(E.Value,TSQLIdentifierExpression));
  AssertIdentifierName('Correct field','B',I.Identifier);
  FreeAndNil(FParser);
  FreeAndNil(FSource);
  FreeAndNil(FToFree);
end;


procedure TTestSelectParser.TestExtractSimple;
Var
  E : TSQLExtractElement;
begin
  For E:=Low(TSQLExtractElement) to High(TSQLExtractElement) do
    DoExtractSimple(E);
end;

procedure TTestSelectParser.TestOrderByOneField;

begin
  TestSelect('SELECT B FROM A ORDER BY C');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('One order by field',1,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'C',0,obAscending);
end;

procedure TTestSelectParser.TestOrderByTwoFields;

begin
  TestSelect('SELECT B FROM A ORDER BY C,D');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Two order by fields',2,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'C',0,obAscending);
  AssertOrderBy(Select.OrderBy[1],'D',0,obAscending);
end;

procedure TTestSelectParser.TestOrderByThreeFields;
begin
  TestSelect('SELECT B FROM A ORDER BY C,D,E');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Three order by fields',3,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'C',0,obAscending);
  AssertOrderBy(Select.OrderBy[1],'D',0,obAscending);
  AssertOrderBy(Select.OrderBy[2],'E',0,obAscending);
end;

procedure TTestSelectParser.TestOrderByOneDescField;

begin
  TestSelect('SELECT B FROM A ORDER BY C DESC');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('One order by field',1,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'C',0,obDescending);
end;

procedure TTestSelectParser.TestOrderByTwoDescFields;

begin
  TestSelect('SELECT B FROM A ORDER BY C DESC, D DESCENDING');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Two order by fields',2,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'C',0,obDescending);
  AssertOrderBy(Select.OrderBy[1],'D',0,obDescending);
end;

procedure TTestSelectParser.TestOrderByThreeDescFields;
begin
  TestSelect('SELECT B FROM A ORDER BY C DESC,D DESCENDING, E DESC');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Three order by fields',3,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'C',0,obDescending);
  AssertOrderBy(Select.OrderBy[1],'D',0,obDescending);
  AssertOrderBy(Select.OrderBy[2],'E',0,obDescending);
end;

procedure TTestSelectParser.TestOrderByOneTableField;

begin
  TestSelect('SELECT B FROM A ORDER BY C.D');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('One order by field',1,Select.Orderby.Count);
  // Field does not support linking/refering to a table, so the field name is
  // assigned as C.D (instead of D with a <link to table C>)
  AssertOrderBy(Select.OrderBy[0],'C.D',0,obAscending);
end;


procedure TTestSelectParser.TestOrderByOneColumn;
begin
  TestSelect('SELECT B FROM A ORDER BY 1');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('One order by field',1,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'',1,obAscending);
end;

procedure TTestSelectParser.TestOrderByTwoColumns;

begin
  TestSelect('SELECT B,C FROM A ORDER BY 1,2');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Two order by fields',2,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'',1,obAscending);
  AssertOrderBy(Select.OrderBy[1],'',2,obAscending);
end;

procedure TTestSelectParser.TestOrderByTwoColumnsDesc;

begin
  TestSelect('SELECT B,C FROM A ORDER BY 1 DESC,2');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Two order by fields',2,Select.Orderby.Count);
  AssertOrderBy(Select.OrderBy[0],'',1,obDescending);
  AssertOrderBy(Select.OrderBy[1],'',2,obAscending);
end;

procedure TTestSelectParser.TestOrderByCollate;

Var
  O : TSQLOrderByElement;

begin
  TestSelect('SELECT B,C FROM A ORDER BY D COLLATE E');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('One order by fields',1,Select.Orderby.Count);
  O:=AssertOrderBy(Select.OrderBy[0],'D',0,obAscending);
  AssertIdentifierName('Correct collation','E',O.Collation);
end;

procedure TTestSelectParser.TestOrderByCollateDesc;

Var
  O : TSQLOrderByElement;

begin
  TestSelect('SELECT B,C FROM A ORDER BY D COLLATE E');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('One order by fields',1,Select.Orderby.Count);
  O:=AssertOrderBy(Select.OrderBy[0],'D',0,obAscending);
  AssertIdentifierName('Correct collation','E',O.Collation);
end;

procedure TTestSelectParser.TestOrderByCollateDescTwoFields;

Var
  O : TSQLOrderByElement;

begin
  TestSelect('SELECT B,C FROM A ORDER BY D COLLATE E DESC,F COLLATE E');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One table',1,Select.Tables.Count);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertTable(Select.Tables[0],'A');
  AssertEquals('Two order by fields',2,Select.Orderby.Count);
  O:=AssertOrderBy(Select.OrderBy[0],'D',0,obDescending);
  AssertIdentifierName('Correct collation','E',O.Collation);
  O:=AssertOrderBy(Select.OrderBy[1],'F',0,obAscending);
  AssertIdentifierName('Correct collation','E',O.Collation);
end;

procedure TTestSelectParser.TestGroupByOne;
begin
  TestSelect('SELECT B,COUNT(C) AS THECOUNT FROM A GROUP BY B');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One group by field',1,Select.GroupBy.Count);
  AssertIdentifierName('Correct group by field','B',Select.GroupBy[0]);
  AssertField(Select.Fields[0],'B');
  AssertAggregate(Select.Fields[1],afCount,'C',aoNone,'THECOUNT');
end;

procedure TTestSelectParser.TestGroupByTwo;
begin
  TestSelect('SELECT B,C,SUM(D) AS THESUM FROM A GROUP BY B,C');
  AssertEquals('Three fields',3,Select.Fields.Count);
  AssertEquals('One group two fields',2,Select.GroupBy.Count);
  AssertIdentifierName('Correct first group by field','B',Select.GroupBy[0]);
  AssertIdentifierName('Correct second group by field','C',Select.GroupBy[1]);
  AssertField(Select.Fields[0],'B');
  AssertField(Select.Fields[1],'C');
  AssertAggregate(Select.Fields[2],afSum,'D',aoNone,'THESUM');
end;

procedure TTestSelectParser.TestHavingOne;

Var
  H : TSQLBinaryExpression;
  L : TSQLLiteralExpression;
  S : TSQLIntegerLiteral;

begin
  TestSelect('SELECT B,COUNT(C) AS THECOUNT FROM A GROUP BY B HAVING COUNT(C)>1');
  AssertEquals('Two fields',2,Select.Fields.Count);
  AssertEquals('One group by field',1,Select.GroupBy.Count);
  AssertIdentifierName('Correct group by field','B',Select.GroupBy[0]);
  AssertField(Select.Fields[0],'B');
  AssertAggregate(Select.Fields[1],afCount,'C',aoNone,'THECOUNT');
  AssertNotNull('Have having',Select.Having);
  H:=TSQLBinaryExpression(CheckClass(Select.Having,TSQLBinaryExpression));
  AssertEquals('Larger than',boGT,H.Operation);
  L:=TSQLLiteralExpression(CheckClass(H.Right,TSQLLiteralExpression));
  S:=TSQLIntegerLiteral(CheckClass(L.Literal,TSQLIntegerLiteral));
  AssertEquals('One',1,S.Value);
  AssertAggregateExpression(H.Left,afCount,'C',aoNone);
end;

procedure TTestSelectParser.TestUnionSimple;

Var
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT B FROM A UNION SELECT C FROM D');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  S:=TSQLSelectStatement(CheckClass(Select.Union,TSQLSelectStatement));
  AssertEquals('One field',1,S.Fields.Count);
  AssertField(S.Fields[0],'C');
  AssertEquals('One table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D');
  AssertEquals('No UNION ALL : ',False,Select.UnionAll)
end;

procedure TTestSelectParser.TestUnionSimpleAll;

Var
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT B FROM A UNION ALL SELECT C FROM D');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  S:=TSQLSelectStatement(CheckClass(Select.Union,TSQLSelectStatement));
  AssertEquals('One field',1,S.Fields.Count);
  AssertField(S.Fields[0],'C');
  AssertEquals('One table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D');
  AssertEquals('UNION ALL : ',True,Select.UnionAll)
end;

procedure TTestSelectParser.TestUnionSimpleOrderBy;
Var
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT B FROM A UNION SELECT C FROM D ORDER BY 1');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertOrderBy(Select.OrderBy[0],'',1,obAscending);
  S:=TSQLSelectStatement(CheckClass(Select.Union,TSQLSelectStatement));
  AssertEquals('One field',1,S.Fields.Count);
  AssertField(S.Fields[0],'C');
  AssertEquals('One table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D');
end;

procedure TTestSelectParser.TestUnionDouble;
Var
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT B FROM A UNION SELECT C FROM D UNION SELECT E FROM F ORDER BY 1');
  AssertEquals('One field',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'B');
  AssertEquals('One table',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'A');
  AssertOrderBy(Select.OrderBy[0],'',1,obAscending);
  S:=TSQLSelectStatement(CheckClass(Select.Union,TSQLSelectStatement));
  AssertEquals('One field',1,S.Fields.Count);
  AssertField(S.Fields[0],'C');
  AssertEquals('One table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D');
  S:=TSQLSelectStatement(CheckClass(S.Union,TSQLSelectStatement));
  AssertEquals('One field',1,S.Fields.Count);
  AssertField(S.Fields[0],'E');
  AssertEquals('One table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'F');
end;

procedure TTestSelectParser.TestUnionError1;

begin
  TestSelectError('SELECT B FROM A ORDER BY B UNION SELECT C FROM D');
end;

procedure TTestSelectParser.TestUnionError2;
begin
  TestSelectError('SELECT B FROM A UNION SELECT C,E FROM D');
end;

procedure TTestSelectParser.TestPlanOrderNatural;

Var
  E : TSQLSelectPlanExpr;
  N : TSQLSelectNaturalPLan;

begin
  TestSelect('SELECT A FROM B PLAN SORT (B NATURAL)');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',1,E.Items.Count);
  AssertEquals('Correct plan type',pjtSort,E.JoinType);
  N:=TSQLSelectNaturalPLan(CheckClass(E.Items[0],TSQLSelectNaturalPLan));
  AssertIdentifierName('Correct table','B',N.TableName);
end;

procedure TTestSelectParser.TestPlanOrderOrder;

Var
  E : TSQLSelectPlanExpr;
  O : TSQLSelectOrderedPLan;

begin
  TestSelect('SELECT A FROM B PLAN SORT (B ORDER C)');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',1,E.Items.Count);
  AssertEquals('Correct plan type',pjtSort,E.JoinType);
  O:=TSQLSelectOrderedPLan(CheckClass(E.Items[0],TSQLSelectOrderedPLan));
  AssertIdentifierName('Correct table','B',O.TableName);
  AssertIdentifierName('Correct table','C',O.OrderIndex);
end;

procedure TTestSelectParser.TestPlanOrderIndex1;

Var
  E : TSQLSelectPlanExpr;
  O : TSQLSelectIndexedPLan;

begin
  TestSelect('SELECT A FROM B PLAN SORT (B INDEX (C))');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',1,E.Items.Count);
  AssertEquals('Correct plan type',pjtSort,E.JoinType);
  O:=TSQLSelectIndexedPLan(CheckClass(E.Items[0],TSQLSelectIndexedPlan));
  AssertIdentifierName('Correct table','B',O.TableName);
  AssertEquals('Correct index count',1,O.Indexes.Count);
  AssertIdentifierName('Correct table','C',O.Indexes[0]);
end;

procedure TTestSelectParser.TestPlanOrderIndex2;
Var
  E : TSQLSelectPlanExpr;
  O : TSQLSelectIndexedPLan;

begin
  TestSelect('SELECT A FROM B PLAN SORT (B INDEX (C,D))');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',1,E.Items.Count);
  AssertEquals('Correct plan type',pjtSort,E.JoinType);
  O:=TSQLSelectIndexedPLan(CheckClass(E.Items[0],TSQLSelectIndexedPlan));
  AssertIdentifierName('Correct table','B',O.TableName);
  AssertEquals('Correct index count',2,O.Indexes.Count);
  AssertIdentifierName('Correct table','C',O.Indexes[0]);
  AssertIdentifierName('Correct table','D',O.Indexes[1]);
end;

procedure TTestSelectParser.TestPlanJoinNatural;
Var
  E : TSQLSelectPlanExpr;
  N : TSQLSelectNaturalPLan;
  O : TSQLSelectOrderedPLan;

begin
  TestSelect('SELECT A FROM B PLAN JOIN (B NATURAL, C ORDER D)');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',2,E.Items.Count);
  AssertEquals('Correct plan type',pjtJoin,E.JoinType);
  N:=TSQLSelectNaturalPLan(CheckClass(E.Items[0],TSQLSelectNaturalPlan));
  AssertIdentifierName('Correct table','B',N.TableName);
  O:=TSQLSelectOrderedPLan(CheckClass(E.Items[1],TSQLSelectOrderedPlan));
  AssertIdentifierName('Correct table','C',O.TableName);
  AssertIdentifierName('Correct index','D',O.OrderIndex);
end;

procedure TTestSelectParser.TestPlanDefaultNatural;
Var
  E : TSQLSelectPlanExpr;
  N : TSQLSelectNaturalPLan;
  O : TSQLSelectOrderedPLan;

begin
  TestSelect('SELECT A FROM B PLAN  (B NATURAL, C ORDER D)');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',2,E.Items.Count);
  AssertEquals('Correct plan type',pjtJoin,E.JoinType);
  N:=TSQLSelectNaturalPLan(CheckClass(E.Items[0],TSQLSelectNaturalPlan));
  AssertIdentifierName('Correct table','B',N.TableName);
  O:=TSQLSelectOrderedPLan(CheckClass(E.Items[1],TSQLSelectOrderedPlan));
  AssertIdentifierName('Correct table','C',O.TableName);
  AssertIdentifierName('Correct index','D',O.OrderIndex);
end;

procedure TTestSelectParser.TestPlanMergeNatural;
Var
  E : TSQLSelectPlanExpr;
  N : TSQLSelectNaturalPLan;
  O : TSQLSelectOrderedPLan;

begin
  TestSelect('SELECT A FROM B PLAN MERGE (B NATURAL, C ORDER D)');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('One plan item',2,E.Items.Count);
  AssertEquals('Correct plan type',pjtMerge,E.JoinType);
  N:=TSQLSelectNaturalPLan(CheckClass(E.Items[0],TSQLSelectNaturalPlan));
  AssertIdentifierName('Correct table','B',N.TableName);
  O:=TSQLSelectOrderedPLan(CheckClass(E.Items[1],TSQLSelectOrderedPlan));
  AssertIdentifierName('Correct table','C',O.TableName);
  AssertIdentifierName('Correct index','D',O.OrderIndex);
end;

procedure TTestSelectParser.TestPlanMergeNested;
Var
  E,EN : TSQLSelectPlanExpr;
  N : TSQLSelectNaturalPLan;
  I : TSQLSelectIndexedPLan;

begin
  TestSelect('SELECT A FROM B PLAN MERGE (SORT (B NATURAL), SORT (JOIN (D NATURAL, E INDEX (F))))');
  E:=TSQLSelectPlanExpr(CheckClass(Select.Plan,TSQLSelectPlanExpr));
  AssertEquals('Two plan items',2,E.Items.Count);
  AssertEquals('Correct overall plan type',pjtMerge,E.JoinType);
  // SORT (B NATURAL)
  EN:=TSQLSelectPlanExpr(CheckClass(E.Items[0],TSQLSelectPlanExpr));
  AssertEquals('Correct plan type Item 1',pjtSort,EN.JoinType);
  AssertEquals('On plan item in item 1',1,EN.Items.Count);
  N:=TSQLSelectNaturalPLan(CheckClass(EN.Items[0],TSQLSelectNaturalPlan));
  AssertIdentifierName('Correct table','B',N.TableName);
  // SORT (JOIN (D...
  EN:=TSQLSelectPlanExpr(CheckClass(E.Items[1],TSQLSelectPlanExpr));
  AssertEquals('Correct plan type item 2',pjtSort,EN.JoinType);
  AssertEquals('One plan item in item 2',1,EN.Items.Count);
  // JOIN (D NATURAL, E
  E:=TSQLSelectPlanExpr(CheckClass(EN.Items[0],TSQLSelectPlanExpr));
  AssertEquals('Correct plan type',pjtJoin,E.JoinType);
  AssertEquals('Two plan items in item 2',2,E.Items.Count);
  N:=TSQLSelectNaturalPLan(CheckClass(E.Items[0],TSQLSelectNaturalPlan));
  AssertIdentifierName('Correct table','D',N.TableName);
  // E INDEX (F)
  I:=TSQLSelectIndexedPLan(CheckClass(E.Items[1],TSQLSelectIndexedPlan));
  AssertIdentifierName('Correct table','E',I.TableName);
  AssertEquals('Correct index count for table E',1,I.Indexes.Count);
  AssertIdentifierName('Correct index for table E','F',I.Indexes[0]);
end;

procedure TTestSelectParser.TestSubSelect;

Var
  F : TSQLSelectField;
  E : TSQLSelectExpression;
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT A,(SELECT C FROM D WHERE E=A) AS THECOUNT FROM B');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('2 fields in select',2,Select.Fields.Count);
  AssertField(Select.Fields[0],'A','');
  F:=TSQLSelectField(CheckClass(Select.fields[1],TSQLSelectField));
  AssertIdentifierName('Correct alias name for subselect','THECOUNT',F.AliasName);
  E:=TSQLSelectExpression(CheckClass(F.Expression,TSQLSelectExpression));
  S:=TSQLSelectStatement(CheckClass(E.Select,TSQLSelectStatement));
  AssertEquals('1 field in subselect',1,S.Fields.Count);
  AssertField(S.Fields[0],'C','');
  AssertEquals('1 table in subselect',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D','');
end;

procedure TTestSelectParser.TestWhereExists;

Var
  E : TSQLExistsExpression;
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT A FROM B WHERE EXISTS (SELECT C FROM D WHERE E=A)');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'A','');
  E:=TSQLExistsExpression(CheckClass(Select.Where,TSQLExistsExpression));
  S:=TSQLSelectStatement(CheckClass(E.Select,TSQLSelectStatement));
  AssertEquals('1 field in subselect',1,S.Fields.Count);
  AssertField(S.Fields[0],'C','');
  AssertEquals('1 table in subselect',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D','');
end;

procedure TTestSelectParser.TestWhereSingular;

Var
  E : TSQLSingularExpression;
  S : TSQLSelectStatement;

begin
  TestSelect('SELECT A FROM B WHERE SINGULAR (SELECT C FROM D WHERE E=A)');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'A','');
  E:=TSQLSingularExpression(CheckClass(Select.Where,TSQLSingularExpression));
  S:=TSQLSelectStatement(CheckClass(E.Select,TSQLSelectStatement));
  AssertEquals('1 field in subselect',1,S.Fields.Count);
  AssertField(S.Fields[0],'C','');
  AssertEquals('1 table in subselect',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D','');
end;

procedure TTestSelectParser.TestWhereAll;

Var
  E : TSQLAllExpression;
  S : TSQLSelectStatement;
  B : TSQLBinaryExpression;

begin
  TestSelect('SELECT A FROM B WHERE A > ALL (SELECT C FROM D WHERE E=F)');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'A','');
  B:=TSQLBinaryExpression(CheckClass(Select.Where,TSQLBinaryExpression));
  AssertEquals('Correct operation',boGT,B.Operation);
  E:=TSQLAllExpression(CheckClass(B.right,TSQLAllExpression));
  S:=TSQLSelectStatement(CheckClass(E.Select,TSQLSelectStatement));
  AssertEquals('1 field in subselect',1,S.Fields.Count);
  AssertField(S.Fields[0],'C','');
  AssertEquals('1 table in subselect',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D','');
end;

procedure TTestSelectParser.TestWhereAny;

Var
  E : TSQLANyExpression;
  S : TSQLSelectStatement;
  B : TSQLBinaryExpression;

begin
  TestSelect('SELECT A FROM B WHERE A > ANY (SELECT C FROM D WHERE E=F)');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'A','');
  B:=TSQLBinaryExpression(CheckClass(Select.Where,TSQLBinaryExpression));
  AssertEquals('Correct operation',boGT,B.Operation);
  E:=TSQLAnyExpression(CheckClass(B.right,TSQLANyExpression));
  S:=TSQLSelectStatement(CheckClass(E.Select,TSQLSelectStatement));
  AssertEquals('1 field in subselect',1,S.Fields.Count);
  AssertField(S.Fields[0],'C','');
  AssertEquals('1 table in subselect',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D','');
end;

procedure TTestSelectParser.TestWhereSome;

Var
  E : TSQLSomeExpression;
  S : TSQLSelectStatement;
  B : TSQLBinaryExpression;

begin
  TestSelect('SELECT A FROM B WHERE A > SOME (SELECT C FROM D WHERE E=F)');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertField(Select.Fields[0],'A','');
  B:=TSQLBinaryExpression(CheckClass(Select.Where,TSQLBinaryExpression));
  AssertEquals('Correct operation',boGT,B.Operation);
  E:=TSQLSomeExpression(CheckClass(B.right,TSQLSomeExpression));
  S:=TSQLSelectStatement(CheckClass(E.Select,TSQLSelectStatement));
  AssertEquals('1 field in subselect',1,S.Fields.Count);
  AssertField(S.Fields[0],'C','');
  AssertEquals('1 table in subselect',1,S.Tables.Count);
  AssertTable(S.Tables[0],'D','');
end;


procedure TTestSelectParser.TestParam;

Var
  F : TSQLSelectField;
  P : TSQLParameterExpression;

begin
  TestSelect('SELECT :A FROM B');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertNotNull('Have field',Select.Fields[0]);
  F:=TSQLSelectField(CheckClass(Select.Fields[0],TSQLSelectField));
  AssertNotNull('Have field expresssion,',F.Expression);
  P:=TSQLParameterExpression(CheckClass(F.Expression,TSQLParameterExpression));
  AssertIdentifierName('Correct parameter name','A',P.Identifier);
end;

procedure TTestSelectParser.TestParamExpr;

Var
  F : TSQLSelectField;
  P : TSQLParameterExpression;
  B : TSQLBinaryExpression;

begin
  TestSelect('SELECT :A + 1 FROM B');
  AssertEquals('1 table in select',1,Select.Tables.Count);
  AssertTable(Select.Tables[0],'B','');
  AssertEquals('1 fields in select',1,Select.Fields.Count);
  AssertNotNull('Have field',Select.Fields[0]);
  F:=TSQLSelectField(CheckClass(Select.Fields[0],TSQLSelectField));
  AssertNotNull('Have field expresssion,',F.Expression);
  B:=TSQLBinaryExpression(CheckClass(F.Expression,TSQLBinaryExpression));
  P:=TSQLParameterExpression(CheckClass(B.Left,TSQLParameterExpression));
  AssertIdentifierName('Correct parameter name','A',P.Identifier);
end;

{ TTestRollBackParser }

function TTestRollBackParser.TestRollback(const ASource: String
  ): TSQLRollbackStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLRollbackStatement(CheckClass(FToFree,TSQLRollbackStatement));
  FRollback:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestRollBackParser.TestRollbackError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestRollBackParser.TestRollback;
begin
  TestRollBack('ROLLBACK');
  AssertNull('No transaction name',Rollback.TransactionName);
  AssertEquals('No work',False,Rollback.Work);
  AssertEquals('No release',False,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackWork;
begin
  TestRollBack('ROLLBACK WORK');
  AssertNull('No transaction name',Rollback.TransactionName);
  AssertEquals('work',True,Rollback.Work);
  AssertEquals('No release',False,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackRelease;
begin
  TestRollBack('ROLLBACK RELEASE');
  AssertNull('No transaction name',Rollback.TransactionName);
  AssertEquals('no work',False,Rollback.Work);
  AssertEquals('release',True,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackWorkRelease;
begin
  TestRollBack('ROLLBACK WORK RELEASE');
  AssertNull('No transaction name',Rollback.TransactionName);
  AssertEquals('work',True,Rollback.Work);
  AssertEquals('release',True,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackTransaction;
begin
  TestRollBack('ROLLBACK TRANSACTION T');
  AssertIdentifierName('Transaction name','T',Rollback.TransactionName);
  AssertEquals('No work',False,Rollback.Work);
  AssertEquals('No release',False,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackTransactionWork;
begin
  TestRollBack('ROLLBACK TRANSACTION T WORK');
  AssertIdentifierName('Transaction name','T',Rollback.TransactionName);
  AssertEquals('work',True,Rollback.Work);
  AssertEquals('No release',False,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackTransactionRelease;
begin
  TestRollBack('ROLLBACK TRANSACTION T RELEASE');
  AssertIdentifierName('Transaction name','T',Rollback.TransactionName);
  AssertEquals('no work',False,Rollback.Work);
  AssertEquals('release',True,Rollback.Release);
end;

procedure TTestRollBackParser.TestRollbackTransactionWorkRelease;
begin
  TestRollBack('ROLLBACK TRANSACTION T WORK RELEASE');
  AssertIdentifierName('Transaction name','T',Rollback.TransactionName);
  AssertEquals('work',True,Rollback.Work);
  AssertEquals('release',True,Rollback.Release);
end;
{ TTestCommitParser }

function TTestCommitParser.TestCommit(const ASource: String
  ): TSQLCommitStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLCommitStatement(CheckClass(FToFree,TSQLCommitStatement));
  FCommit:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestCommitParser.TestCommitError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestCommitParser.TestCommit;
begin
  TestCommit('Commit');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('No work',False,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitWork;
begin
  TestCommit('Commit WORK');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitRelease;
begin
  TestCommit('Commit RELEASE');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('no work',False,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitWorkRelease;
begin
  TestCommit('Commit WORK RELEASE');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransaction;
begin
  TestCommit('Commit TRANSACTION T');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('No work',False,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionWork;
begin
  TestCommit('Commit WORK TRANSACTION T ');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionRelease;
begin
  TestCommit('Commit TRANSACTION T RELEASE');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('no work',False,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionWorkRelease;
begin
  TestCommit('Commit WORK TRANSACTION T  RELEASE');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('No Retain',False,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitRetain;
begin
  TestCommit('Commit RETAIN');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('No work',False,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitRetainSnapShot;
begin
  TestCommit('Commit RETAIN SNAPSHOT');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('No work',False,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitWorkRetain;
begin
  TestCommit('Commit WORK  RETAIN');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitReleaseRetain;
begin
  TestCommit('Commit RELEASE  RETAIN');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('no work',False,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitWorkReleaseRetain;
begin
  TestCommit('Commit WORK RELEASE  RETAIN');
  AssertNull('No transaction name',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionRetain;
begin
  TestCommit('Commit TRANSACTION T  RETAIN');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('No work',False,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionWorkRetain;
begin
  TestCommit('Commit WORK TRANSACTION T  RETAIN');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('No release',False,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionReleaseRetain;
begin
  TestCommit('Commit TRANSACTION T RELEASE  RETAIN');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('no work',False,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

procedure TTestCommitParser.TestCommitTransactionWorkReleaseRetain;
begin
  TestCommit('Commit WORK TRANSACTION T  RELEASE RETAIN');
  AssertIdentifierName('Transaction name','T',Commit.TransactionName);
  AssertEquals('work',True,Commit.Work);
  AssertEquals('release',True,Commit.Release);
  AssertEquals('Retain',True,Commit.Retain);
end;

{ TTestExecuteProcedureParser }

function TTestExecuteProcedureParser.TestExecute(const ASource: String
  ): TSQLExecuteProcedureStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLExecuteProcedureStatement(CheckClass(FToFree,TSQLExecuteProcedureStatement));
  FExecute:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestExecuteProcedureParser.TestExecuteError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestExecuteProcedureParser.TestExecuteSimple;
begin
  TestExecute('EXECUTE PROCEDURE A');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('No arguments',0,Execute.Params.Count);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteSimpleTransaction;
begin
  TestExecute('EXECUTE PROCEDURE TRANSACTION B A');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertIdentifierName('Correct transaction name','B',Execute.TransactionName);
  AssertEquals('No arguments',0,Execute.Params.Count);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteSimpleReturningValues;
begin
  TestExecute('EXECUTE PROCEDURE A RETURNING_VALUES :B');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('No arguments',0,Execute.Params.Count);
  AssertEquals('1 return value',1,Execute.Returning.Count);
  AssertIdentifierName('return value','B',Execute.Returning[0]);
end;

procedure TTestExecuteProcedureParser.TestExecuteSimpleReturning2Values;
begin
  TestExecute('EXECUTE PROCEDURE A RETURNING_VALUES :B,:C');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('No arguments',0,Execute.Params.Count);
  AssertEquals('2 return values',2,Execute.Returning.Count);
  AssertIdentifierName('return value','B',Execute.Returning[0]);
  AssertIdentifierName('return value','C',Execute.Returning[1]);
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArg;

Var
  I : TSQLIdentifierExpression;

begin
  TestExecute('EXECUTE PROCEDURE A (B)');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('One argument',1,Execute.Params.Count);
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[0],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','B',I.Identifier);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgNB;

Var
  I : TSQLIdentifierExpression;

begin
  TestExecute('EXECUTE PROCEDURE A B');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('One argument',1,Execute.Params.Count);
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[0],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','B',I.Identifier);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;


procedure TTestExecuteProcedureParser.TestExecuteTwoArgs;
Var
  I : TSQLIdentifierExpression;

begin
  TestExecute('EXECUTE PROCEDURE A (B,C)');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('Two arguments',2,Execute.Params.Count);
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[0],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','B',I.Identifier);
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[1],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','C',I.Identifier);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteTwoArgsNB;
Var
  I : TSQLIdentifierExpression;

begin
  TestExecute('EXECUTE PROCEDURE A B, C');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('Two arguments',2,Execute.Params.Count);
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[0],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','B',I.Identifier);
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[1],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','C',I.Identifier);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgSelect;

Var
  S : TSQLSelectExpression;

begin
  TestExecute('EXECUTE PROCEDURE A ((SELECT B FROM C))');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('One argument',1,Execute.Params.Count);
  S:=TSQLSelectExpression(CheckClass(Execute.Params[0],TSQLSelectExpression));
  AssertField(S.Select.Fields[0],'B','');
  AssertTable(S.Select.Tables[0],'C','');
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgSelectNB;

Var
  S : TSQLSelectExpression;

begin
  TestExecute('EXECUTE PROCEDURE A (SELECT B FROM C)');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('One argument',1,Execute.Params.Count);
  S:=TSQLSelectExpression(CheckClass(Execute.Params[0],TSQLSelectExpression));
  AssertField(S.Select.Fields[0],'B','');
  AssertTable(S.Select.Tables[0],'C','');
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteTwoArgsSelect;

Var
  S : TSQLSelectExpression;
  I : TSQLIdentifierExpression;

begin
  TestExecute('EXECUTE PROCEDURE A ((SELECT B FROM C),D)');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('Two arguments',2,Execute.Params.Count);
  S:=TSQLSelectExpression(CheckClass(Execute.Params[0],TSQLSelectExpression));
  AssertField(S.Select.Fields[0],'B','');
  AssertTable(S.Select.Tables[0],'C','');
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[1],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','D',I.Identifier);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteTwoArgsSelectNB;

Var
  S : TSQLSelectExpression;
  I : TSQLIdentifierExpression;

begin
  TestExecute('EXECUTE PROCEDURE A (SELECT B FROM C),D');
  AssertIdentifierName('Correct procedure name','A',Execute.ProcedureName);
  AssertNull('No transaction name',Execute.TransactionName);
  AssertEquals('Two arguments',2,Execute.Params.Count);
  S:=TSQLSelectExpression(CheckClass(Execute.Params[0],TSQLSelectExpression));
  AssertField(S.Select.Fields[0],'B','');
  AssertTable(S.Select.Tables[0],'C','');
  I:=TSQLIdentifierExpression(CheckClass(Execute.Params[1],TSQLIdentifierExpression));
  AssertIdentifierName('Correct argument','D',I.Identifier);
  AssertEquals('No return values',0,Execute.Returning.Count);
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgSelectErr;
begin
  TestExecuteError('EXECUTE PROCEDURE A ((SELECT B FROM C), 2')
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgSelectErr2;
begin
  TestExecuteError('EXECUTE PROCEDURE A (SELECT B FROM C), 2)')
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgSelectErr3;
begin
  TestExecuteError('EXECUTE PROCEDURE A B)')
end;

procedure TTestExecuteProcedureParser.TestExecuteOneArgSelectErr4;
begin
  TestExecuteError('EXECUTE PROCEDURE A B,C)')
end;

{ EXECUTE PROCEDURE DELETE_EMPLOYEE2 1, 2;
  EXECUTE PROCEDURE DELETE_EMPLOYEE2 (1, 2);
  EXECUTE PROCEDURE DELETE_EMPLOYEE2 ((SELECT A FROM A), 2);
  EXECUTE PROCEDURE DELETE_EMPLOYEE2 (SELECT A FROM A), 2;
}

{ TTestConnectParser }

function TTestConnectParser.TestConnect(const ASource: String
  ): TSQLConnectStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLConnectStatement(CheckClass(FToFree,TSQLConnectStatement));
  FConnect:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestConnectParser.TestConnectError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestConnectParser.TestConnectSimple;
begin
  TestConnect('CONNECT ''/my/database/file''');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','',Connect.UserName);
  AssertEquals('Password','',Connect.Password);
  AssertEquals('Role','',Connect.Role);
  AssertEquals('Cache',0,Connect.Cache);
end;

procedure TTestConnectParser.TestConnectUser;
begin
  TestConnect('CONNECT ''/my/database/file'' USER ''me''');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','me',Connect.UserName);
  AssertEquals('Password','',Connect.Password);
  AssertEquals('Role','',Connect.Role);
  AssertEquals('Cache',0,Connect.Cache);
end;

procedure TTestConnectParser.TestConnectPassword;
begin
  TestConnect('CONNECT ''/my/database/file'' PASSWORD ''secret''');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','',Connect.UserName);
  AssertEquals('Password','secret',Connect.Password);
  AssertEquals('Role','',Connect.Role);
  AssertEquals('Cache',0,Connect.Cache);
end;

procedure TTestConnectParser.TestConnectUserPassword;
begin
  TestConnect('CONNECT ''/my/database/file'' USER ''me'' PASSWORD ''secret''');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','me',Connect.UserName);
  AssertEquals('Password','secret',Connect.Password);
  AssertEquals('Role','',Connect.Role);
  AssertEquals('Cache',0,Connect.Cache);
end;
procedure TTestConnectParser.TestConnectUserPasswordRole;
begin
  TestConnect('CONNECT ''/my/database/file'' USER ''me'' PASSWORD ''secret'' ROLE ''admin''');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','me',Connect.UserName);
  AssertEquals('Password','secret',Connect.Password);
  AssertEquals('Role','admin',Connect.Role);
  AssertEquals('Cache',0,Connect.Cache);
end;

procedure TTestConnectParser.TestConnectUserPasswordRoleCache;
begin
  TestConnect('CONNECT ''/my/database/file'' USER ''me'' PASSWORD ''secret'' ROLE ''admin'' CACHE 2048');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','me',Connect.UserName);
  AssertEquals('Password','secret',Connect.Password);
  AssertEquals('Role','admin',Connect.Role);
  AssertEquals('Cache',2048,Connect.Cache);
end;

procedure TTestConnectParser.TestConnectSimpleCache;
begin
  TestConnect('CONNECT ''/my/database/file'' CACHE 2048');
  AssertEquals('Database name','/my/database/file',Connect.DatabaseName);
  AssertEquals('User name','',Connect.UserName);
  AssertEquals('Password','',Connect.Password);
  AssertEquals('Role','',Connect.Role);
  AssertEquals('Cache',2048,Connect.Cache);
end;

{ TTestCreateDatabaseParser }

function TTestCreateDatabaseParser.TestCreate(const ASource: String
  ): TSQLCreateDatabaseStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLCreateDatabaseStatement(CheckClass(FToFree,TSQLCreateDatabaseStatement));
  FCreateDB:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestCreateDatabaseParser.TestCreateError(const ASource: String);
begin
  FerrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestCreateDatabaseParser.TestSimple;
begin
  TestCreate('CREATE DATABASE ''/my/database/file''');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',0,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestSimpleSchema;
begin
  TestCreate('CREATE SCHEMA ''/my/database/file''');
  AssertEquals('schema',True,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',0,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestSimpleUSer;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' USER ''me''');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','me',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',0,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestSimpleUSerPassword;

begin
  TestCreate('CREATE DATABASE ''/my/database/file'' USER ''me'' PASSWORD ''SECRET''');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','me',CreateDB.UserName);
  AssertEquals('Password','SECRET',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',0,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestSimplePassword;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PASSWORD ''SECRET''');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','SECRET',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',0,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestPageSize;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE = 2048');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestPageSize2;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestPageSizeLength;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH = 2000');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestPageSizeLength2;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestPageSizeLength3;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 PAGES');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestPageSizeLength4;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 PAGE');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestCharset;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' DEFAULT CHARACTER SET UTF8');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertIDentifierName('Character set','UTF8',CreateDB.CharSet);
  AssertEquals('Page size',0,CreateDB.PageSize);
  AssertEquals('Length',0,CreateDB.Length);
  AssertEquals('Secondary files',0,CreateDB.SecondaryFiles.Count);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile1;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2''');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',0,0);
end;
procedure TTestCreateDatabaseParser.TestSecondaryFile2;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' LENGTH 1000');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',1000,0);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile3;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' LENGTH = 1000');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',1000,0);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile4;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' LENGTH = 1000 PAGE');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',1000,0);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile5;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' LENGTH = 1000 PAGES');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',1000,0);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile6;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' STARTING 3000 ');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',0,3000);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile7;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' STARTING AT 3000 ');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',0,3000);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile9;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' LENGTH 201 STARTING AT PAGE 3000 ');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',201,3000);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFile10;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2''  STARTING AT PAGE 3000 LENGTH 201');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',201,3000);
end;
procedure TTestCreateDatabaseParser.TestSecondaryFile8;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' STARTING AT PAGE 3000 ');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',1,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',0,3000);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFileS;
begin
  TestCreate('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' FILE ''/my/database/file3'' ');
  AssertEquals('Not schema',False,CreateDB.UseSchema);
  AssertEquals('Database file','/my/database/file',CreateDB.FileName);
  AssertEquals('Username','',CreateDB.UserName);
  AssertEquals('Password','',CreateDB.Password);
  AssertNull('Character set',CreateDB.CharSet);
  AssertEquals('Page size',2048,CreateDB.PageSize);
  AssertEquals('Length',2000,CreateDB.Length);
  AssertEquals('Secondary files',2,CreateDB.SecondaryFiles.Count);
  AssertSecondaryFile(CreateDB.SecondaryFiles[0],'/my/database/file2',0,0);
  AssertSecondaryFile(CreateDB.SecondaryFiles[1],'/my/database/file3',0,0);
end;

procedure TTestCreateDatabaseParser.TestSecondaryFileError1;
begin
  TestCreateError('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' LENGTH 3 LENGTH 2');
end;

procedure TTestCreateDatabaseParser.TestSecondaryFileError2;
begin
  TestCreateError('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' STARTING 3 STARTING 2');
end;

procedure TTestCreateDatabaseParser.TestSecondaryFileError3;
begin
  TestCreateError('CREATE DATABASE ''/my/database/file'' PAGE_SIZE 2048 LENGTH 2000 FILE ''/my/database/file2'' STARTING 3 LENGTH 2 STARTING 2');
end;

{ TTestAlterDatabaseParser }

function TTestAlterDatabaseParser.TestAlter(const ASource: String
  ): TSQLAlterDatabaseStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLAlterDatabaseStatement(CheckClass(FToFree,TSQLAlterDatabaseStatement));
  FAlterDB:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestAlterDatabaseParser.TestAlterError(const ASource: String);
begin
  FerrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestAlterDatabaseParser.TestSimple;
begin
  TestAlter('ALTER DATABASE ADD FILE ''/my/file''');
  AssertEquals('Operation count',1,AlterDB.Operations.Count);
  AssertSecondaryFile(AlterDB.Operations[0],'/my/file',0,0);
end;

procedure TTestAlterDatabaseParser.TestStarting;
begin
  TestAlter('ALTER DATABASE ADD FILE ''/my/file'' STARTING AT 100');
  AssertEquals('Operation count',1,AlterDB.Operations.Count);
  AssertSecondaryFile(AlterDB.Operations[0],'/my/file',0,100);
end;

procedure TTestAlterDatabaseParser.TestStartingLength;
begin
  TestAlter('ALTER DATABASE ADD FILE ''/my/file'' STARTING AT 100 LENGTH 200');
  AssertEquals('Operation count',1,AlterDB.Operations.Count);
  AssertSecondaryFile(AlterDB.Operations[0],'/my/file',200,100);
end;

procedure TTestAlterDatabaseParser.TestFiles;
begin
  TestAlter('ALTER DATABASE ADD FILE ''/my/file2'' ADD FILE ''/my/file3'' ');
  AssertEquals('Operation count',2,AlterDB.Operations.Count);
  AssertSecondaryFile(AlterDB.Operations[0],'/my/file2',0,0);
  AssertSecondaryFile(AlterDB.Operations[1],'/my/file3',0,0);
end;

procedure TTestAlterDatabaseParser.TestFiles2;
begin
  TestAlter('ALTER DATABASE ADD FILE ''/my/file2'' FILE ''/my/file3'' ');
  AssertEquals('Operation count',2,AlterDB.Operations.Count);
  AssertSecondaryFile(AlterDB.Operations[0],'/my/file2',0,0);
  AssertSecondaryFile(AlterDB.Operations[1],'/my/file3',0,0);
end;

procedure TTestAlterDatabaseParser.TestFilesError;
begin
  TestAlterError('ALTER DATABASE FILE ''/my/file2'' FILE ''/my/file3'' ');
end;

procedure TTestAlterDatabaseParser.TestError;
begin
  TestAlterError('ALTER DATABASE ');
end;

procedure TTestAlterDatabaseParser.TestLength;
begin
  TestAlter('ALTER DATABASE ADD FILE ''/my/file'' LENGTH 200');
  AssertEquals('Operation count',1,AlterDB.Operations.Count);
  AssertSecondaryFile(AlterDB.Operations[0],'/my/file',200,0);
end;

{ TTestCreateViewParser }

function TTestCreateViewParser.TestCreate(const ASource: String
  ): TSQLCreateViewStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLCreateViewStatement(CheckClass(FToFree,TSQLCreateViewStatement));
  FView:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestCreateViewParser.TestCreateError(const ASource: String);
begin
  FerrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestCreateViewParser.TestSimple;

Var
  S : TSQLSelectStatement;

begin
  TestCreate('CREATE VIEW A AS SELECT B FROM C');
  AssertIdentifierName('View name','A',View.ObjectName);
  AssertNotNull('field list created',View.Fields);
  AssertEquals('No fields in list',0,View.Fields.Count);
  S:=TSQLSelectStatement(CheckClass(View.select,TSQLSelectStatement));
  AssertEquals('1 Field',1,S.Fields.Count);
  AssertField(S.Fields[0],'B','');
  AssertEquals('1 table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'C','');
  AssertEquals('No with check option',False,View.WithCheckOption);
end;

procedure TTestCreateViewParser.TestFieldList;

Var
  S : TSQLSelectStatement;

begin
  TestCreate('CREATE VIEW A (D) AS SELECT B FROM C');
  AssertIdentifierName('View name','A',View.ObjectName);
  AssertNotNull('field list created',View.Fields);
  AssertEquals('1 field in list',1,View.Fields.Count);
  AssertIdentifierName('Field name','D',View.Fields[0]);
  S:=TSQLSelectStatement(CheckClass(View.select,TSQLSelectStatement));
  AssertEquals('1 Field',1,S.Fields.Count);
  AssertField(S.Fields[0],'B','');
  AssertEquals('1 table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'C','');
  AssertEquals('No with check option',False,View.WithCheckOption);
end;

procedure TTestCreateViewParser.TestFieldList2;

Var
  S : TSQLSelectStatement;

begin
  TestCreate('CREATE VIEW A (B,C) AS SELECT D,E FROM F');
  AssertIdentifierName('View name','A',View.ObjectName);
  AssertNotNull('field list created',View.Fields);
  AssertEquals('2 fields in list',2,View.Fields.Count);
  AssertIdentifierName('Field name','B',View.Fields[0]);
  AssertIdentifierName('Field name','C',View.Fields[1]);
  S:=TSQLSelectStatement(CheckClass(View.select,TSQLSelectStatement));
  AssertEquals('2 Fields in select',2,S.Fields.Count);
  AssertField(S.Fields[0],'D','');
  AssertField(S.Fields[1],'E','');
  AssertEquals('1 table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'F','');
  AssertEquals('No with check option',False,View.WithCheckOption);
end;

procedure TTestCreateViewParser.TestSimpleWithCheckoption;

Var
  S : TSQLSelectStatement;

begin
  TestCreate('CREATE VIEW A AS SELECT B FROM C WITH CHECK OPTION');
  AssertIdentifierName('View name','A',View.ObjectName);
  AssertNotNull('field list created',View.Fields);
  AssertEquals('No fields in list',0,View.Fields.Count);
  S:=TSQLSelectStatement(CheckClass(View.select,TSQLSelectStatement));
  AssertEquals('1 Field',1,S.Fields.Count);
  AssertField(S.Fields[0],'B','');
  AssertEquals('1 table',1,S.Tables.Count);
  AssertTable(S.Tables[0],'C','');
  AssertEquals('With check option',True,View.WithCheckOption);
end;

{ TTestCreateShadowParser }

function TTestCreateShadowParser.TestCreate(const ASource: String
  ): TSQLCreateShadowStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLCreateShadowStatement(CheckClass(FToFree,TSQLCreateShadowStatement));
  FShadow:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestCreateShadowParser.TestCreateError(const ASource: String);
begin
  FerrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestCreateShadowParser.TestSimple;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file''');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',0,Shadow.Length);
  AssertEquals('No secondary files',0,Shadow.SecondaryFiles.Count);
end;


procedure TTestCreateShadowParser.TestLength;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('No secondary files',0,Shadow.SecondaryFiles.Count);
end;

procedure TTestCreateShadowParser.TestLength2;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH = 2');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('No secondary files',0,Shadow.SecondaryFiles.Count);
end;

procedure TTestCreateShadowParser.TestLength3;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH = 2 PAGE');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('No secondary files',0,Shadow.SecondaryFiles.Count);
end;

procedure TTestCreateShadowParser.TestLength4;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH = 2 PAGES');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('No secondary files',0,Shadow.SecondaryFiles.Count);
end;


procedure TTestCreateShadowParser.TestSecondaryFile1;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2''');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',0,0);
end;

procedure TTestCreateShadowParser.TestSecondaryFile2;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' LENGTH 1000');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',1000,0);

end;

procedure TTestCreateShadowParser.TestSecondaryFile3;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' LENGTH = 1000');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',1000,0);
end;

procedure TTestCreateShadowParser.TestSecondaryFile4;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' LENGTH = 1000 PAGE');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',1000,0);
end;

procedure TTestCreateShadowParser.TestSecondaryFile5;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' LENGTH = 1000 PAGES');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',1000,0);
end;

procedure TTestCreateShadowParser.TestSecondaryFile6;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' STARTING 3000');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',0,3000);
end;

procedure TTestCreateShadowParser.TestSecondaryFile7;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' STARTING AT 3000');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',0,3000);
end;

procedure TTestCreateShadowParser.TestSecondaryFile8;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' STARTING AT PAGE 3000');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('1 secondary file',1,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',0,3000);
end;


procedure TTestCreateShadowParser.TestSecondaryFileS;
begin
  TestCreate('CREATE SHADOW 1 ''/my/file'' LENGTH 2 FILE ''/my/file2'' FILE ''/my/file3''');
  AssertEquals('Not manual',False,Shadow.Manual);
  AssertEquals('Not conditional',False,Shadow.COnditional);
  AssertEquals('Filename','/my/file',Shadow.FileName);
  AssertEquals('No length',2,Shadow.Length);
  AssertEquals('2 secondary file',2,Shadow.SecondaryFiles.Count);
  AssertSecondaryFile(Shadow.SecondaryFiles[0],'/my/file2',0,0);
  AssertSecondaryFile(Shadow.SecondaryFiles[1],'/my/file3',0,0);
end;


{ TTestProcedureStatement }

function TTestProcedureStatement.TestStatement(const ASource: String
  ): TSQLStatement;
begin
  CreateParser(ASource);
  Parser.GetNextToken;
  FToFree:=Parser.ParseProcedureStatements;
  If not (FToFree is TSQLStatement) then
    Fail('Not a TSQLStatement');
  Result:=TSQLStatement(FToFree);
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestProcedureStatement.TestParseStatementError;

begin
  CreateParser(FErrSource);
  FToFree:=Parser.ParseProcedureStatements;
end;

procedure TTestProcedureStatement.TestStatementError(const ASource: String);
begin
  FerrSource:=ASource;
  AssertException(ESQLParser,@TestParseStatementError);
end;

procedure TTestProcedureStatement.TestException;

Var
  E : TSQLExceptionStatement;
begin
  E:=TSQLExceptionStatement(CheckClass(TestStatement('EXCEPTION MYE'),TSQLExceptionStatement));
  AssertIdentifierName('Exception name','MYE',E.ExceptionName);
end;

procedure TTestProcedureStatement.TestExceptionError;
begin
  TestStatementError('EXCEPTION ''MYE''');
end;

procedure TTestProcedureStatement.TestExit;

begin
  CheckClass(TestStatement('EXIT'),TSQLExitStatement);
end;

procedure TTestProcedureStatement.TestSuspend;

begin
  CheckClass(TestStatement('Suspend'),TSQLSuspendStatement);
end;

procedure TTestProcedureStatement.TestEmptyBlock;

Var
  B : TSQLStatementBlock;

begin
  B:=TSQLStatementBlock(CheckClass(TestStatement('BEGIN END'),TSQLStatementBlock));
  AssertEquals('No statements',0,B.Statements.Count)
end;

procedure TTestProcedureStatement.TestExitBlock;

Var
  B : TSQLStatementBlock;

begin
  B:=TSQLStatementBlock(CheckClass(TestStatement('BEGIN EXIT; END'),TSQLStatementBlock));
  AssertEquals('1 statement',1,B.Statements.Count);
  CheckClass(B.Statements[0],TSQLExitStatement);
end;

procedure TTestProcedureStatement.TestExitBlockError;

begin
  TestStatementError('BEGIN EXIT END')
end;

procedure TTestProcedureStatement.TestPostEvent;
Var
  P : TSQLPostEventStatement;

begin
  P:=TSQLPostEventStatement(CheckClass(TestStatement('POST_EVENT ''MYEVENT'''),TSQLPostEventStatement));
  AssertEquals('Correct event name','MYEVENT' , P.EventName);
  AssertNull('No event column',P.ColName);
end;

procedure TTestProcedureStatement.TestPostEventColName;

Var
  P : TSQLPostEventStatement;

begin
  P:=TSQLPostEventStatement(CheckClass(TestStatement('POST_EVENT MyColName'),TSQLPostEventStatement));
  AssertEquals('Correct event name','' , P.EventName);
  AssertIdentifierName('event column','MyColName',P.ColName);
end;

procedure TTestProcedureStatement.TestPostError;

begin
  TestStatementError('POST_EVENT 1');
end;

procedure TTestProcedureStatement.TestAssignSimple;
Var
  A : TSQLAssignStatement;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;
begin
  A:=TSQLAssignStatement(CheckClass(TestStatement('A=1'),TSQLAssignStatement));
  AssertIdentifierName('Variable name','A',A.Variable);
  E:=TSQLLiteralExpression(CheckClass(A.Expression,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,I.Value);
end;

procedure TTestProcedureStatement.TestAssignSimpleNew;
Var
  A : TSQLAssignStatement;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;
begin
  A:=TSQLAssignStatement(CheckClass(TestStatement('NEW.A=1'),TSQLAssignStatement));
  AssertIdentifierName('Variable name','NEW.A',A.Variable);
  E:=TSQLLiteralExpression(CheckClass(A.Expression,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,I.Value);
end;

procedure TTestProcedureStatement.TestAssignSelect;
Var
  A : TSQLAssignStatement;
  S : TSQLSelectExpression;

begin
  A:=TSQLAssignStatement(CheckClass(TestStatement('A=(SELECT B FROM C)'),TSQLAssignStatement));
  AssertIdentifierName('Variable name','A',A.Variable);
  S:=TSQLSelectExpression(CheckClass(A.Expression,TSQLSelectExpression));
  AssertEquals('Field count',1,S.Select.Fields.Count);
  AssertEquals('Table count',1,S.Select.Tables.Count);
  AssertField(S.Select.Fields[0],'B','');
  AssertTable(S.Select.Tables[0],'C','');
end;

procedure TTestProcedureStatement.TestBlockAssignSimple;
Var
  A : TSQLAssignStatement;
  E : TSQLLiteralExpression;
  I : TSQLIntegerLiteral;
  B : TSQLStatementBlock;

begin
  B:=TSQLStatementBlock(CheckClass(TestStatement('BEGIN A=1; EXIT; END'),TSQLStatementBlock));
  AssertEquals('2 statements',2,B.Statements.Count);
  CheckClass(B.Statements[1],TSQLExitStatement);
  A:=TSQLAssignStatement(CheckClass(B.Statements[0],TSQLAssignStatement));
  AssertIdentifierName('Variable name','A',A.Variable);
  E:=TSQLLiteralExpression(CheckClass(A.Expression,TSQLLiteralExpression));
  I:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,I.Value);
end;

procedure TTestProcedureStatement.TestIf;
Var
  I : TSQLIfStatement;
  C : TSQLBinaryExpression;
  E : TSQLLiteralExpression;
  A : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;

begin
  I:=TSQLIfStatement(CheckClass(TestStatement('IF (A=1) THEN EXIT'),TSQLIfStatement));
  C:=TSQLBinaryExpression(CheckClass(I.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boEq,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  CheckClass(I.TrueBranch,TSQLExitStatement);
end;

procedure TTestProcedureStatement.TestIfBlock;

Var
  I : TSQLIfStatement;
  C : TSQLBinaryExpression;
  E : TSQLLiteralExpression;
  A : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;
  B : TSQLStatementBlock;

begin
  I:=TSQLIfStatement(CheckClass(TestStatement('IF (A=1) THEN BEGIN EXIT; END'),TSQLIfStatement));
  C:=TSQLBinaryExpression(CheckClass(I.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boEq,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  B:=TSQLStatementBlock(CheckClass(I.TrueBranch,TSQLStatementBlock));
  AssertEquals('1 statement',1,B.Statements.Count);
  CheckClass(B.Statements[0],TSQLExitStatement);
end;

procedure TTestProcedureStatement.TestIfElse;
Var
  I : TSQLIfStatement;
  C : TSQLBinaryExpression;
  E : TSQLLiteralExpression;
  A : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;
begin
  I:=TSQLIfStatement(CheckClass(TestStatement('IF (A=1) THEN EXIT; ELSE SUSPEND'),TSQLIfStatement));
  C:=TSQLBinaryExpression(CheckClass(I.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boEq,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  CheckClass(I.TrueBranch,TSQLExitStatement);
  CheckClass(I.FalseBranch,TSQLSuspendStatement);
end;

procedure TTestProcedureStatement.TestIfBlockElse;

Var
  I : TSQLIfStatement;
  C : TSQLBinaryExpression;
  E : TSQLLiteralExpression;
  A : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;
  B : TSQLStatementBlock;

begin
  I:=TSQLIfStatement(CheckClass(TestStatement('IF (A=1) THEN BEGIN EXIT; END ELSE SUSPEND'),TSQLIfStatement));
  C:=TSQLBinaryExpression(CheckClass(I.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boEq,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  B:=TSQLStatementBlock(CheckClass(I.TrueBranch,TSQLStatementBlock));
  AssertEquals('1 statement',1,B.Statements.Count);
  CheckClass(B.Statements[0],TSQLExitStatement);
  CheckClass(I.FalseBranch,TSQLSuspendStatement);
end;

procedure TTestProcedureStatement.TestIfElseError;
begin
  TestStatementError('IF (A=B) THEN EXIT ELSE SUSPEND');
  TestStatementError('IF (A=B) THEN BEGIN EXIT; END; ELSE SUSPEND');
end;

procedure TTestProcedureStatement.TestIfBlockElseBlock;

Var
  I : TSQLIfStatement;
  C : TSQLBinaryExpression;
  E : TSQLLiteralExpression;
  A : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;
  B : TSQLStatementBlock;

begin
  I:=TSQLIfStatement(CheckClass(TestStatement('IF (A=1) THEN BEGIN EXIT; END ELSE BEGIN SUSPEND; END'),TSQLIfStatement));
  C:=TSQLBinaryExpression(CheckClass(I.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boEq,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  B:=TSQLStatementBlock(CheckClass(I.TrueBranch,TSQLStatementBlock));
  AssertEquals('1 statement',1,B.Statements.Count);
  CheckClass(B.Statements[0],TSQLExitStatement);
  B:=TSQLStatementBlock(CheckClass(I.FalseBranch,TSQLStatementBlock));
  AssertEquals('1 statement',1,B.Statements.Count);
  CheckClass(B.Statements[0],TSQLSuspendStatement);
end;

procedure TTestProcedureStatement.TestIfErrorBracketLeft;
begin
  TestStatementError('IF A=1) THEN EXIT');
end;

procedure TTestProcedureStatement.TestIfErrorBracketRight;
begin
  TestStatementError('IF (A=1 THEN EXIT');
end;

procedure TTestProcedureStatement.TestIfErrorNoThen;
begin
  TestStatementError('IF (A=1) EXIT');
end;

procedure TTestProcedureStatement.TestIfErrorSemicolonElse;
begin
  TestStatementError('IF (A=1) THEN EXIT; ELSE SUSPEND');
end;

procedure TTestProcedureStatement.TestWhile;

Var
  W : TSQLWhileStatement;
  C : TSQLBinaryExpression;
  E : TSQLLiteralExpression;
  A : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;
  SA : TSQLAssignStatement;

begin
  W:=TSQLWhileStatement(CheckClass(TestStatement('WHILE (A>1) DO A=A-1'),TSQLWhileStatement));
  C:=TSQLBinaryExpression(CheckClass(W.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boGT,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  SA:=TSQLAssignStatement(CheckClass(W.Statement,TSQLAssignStatement));
  AssertIdentifierName('Variable name','A',SA.Variable);
  // Check assignment expression
  C:=TSQLBinaryExpression(CheckClass(SA.Expression,TSQLBinaryExpression));
  AssertEquals('Equals',boAdd,C.Operation);
  // Left operand
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  // Right operand
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',-1,LI.Value);
end;

procedure TTestProcedureStatement.TestWhileBlock;

Var
  W  : TSQLWhileStatement;
  C  : TSQLBinaryExpression;
  E  : TSQLLiteralExpression;
  A  : TSQLIdentifierExpression;
  LI : TSQLIntegerLiteral;
  SA : TSQLAssignStatement;
  B  : TSQLStatementBlock;

begin
  W:=TSQLWhileStatement(CheckClass(TestStatement('WHILE (A>1) DO BEGIN A=A-1; END'),TSQLWhileStatement));
  C:=TSQLBinaryExpression(CheckClass(W.Condition,TSQLBinaryExpression));
  AssertEquals('Equals',boGT,C.Operation);
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',1,LI.Value);
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  B:=TSQLStatementBlock(CheckClass(W.Statement,TSQLStatementBlock));
  AssertEquals('One statement',1,B.Statements.Count);
  SA:=TSQLAssignStatement(CheckClass(B.Statements[0],TSQLAssignStatement));
  AssertIdentifierName('Variable name','A',SA.Variable);
  // Check assignment expression
  C:=TSQLBinaryExpression(CheckClass(SA.Expression,TSQLBinaryExpression));
  AssertEquals('Equals',boAdd,C.Operation);
  // Left operand
  A:=TSQLIdentifierExpression(CheckClass(C.Left,TSQLIdentifierExpression));
  AssertIdentifierName('Variable name','A',A.Identifier);
  // Right operand
  E:=TSQLLiteralExpression(CheckClass(C.Right,TSQLLiteralExpression));
  LI:=TSQLIntegerLiteral(CheckClass(E.Literal,TSQLIntegerLiteral));
  AssertEquals('Correct value',-1,LI.Value);
end;

procedure TTestProcedureStatement.TestWhileErrorBracketLeft;
begin
  TestStatementError('WHILE A>1) DO A=A-1');
end;

procedure TTestProcedureStatement.TestWhileErrorBracketRight;
begin
  TestStatementError('WHILE (A>1 DO A=A-1');
end;

procedure TTestProcedureStatement.TestWhileErrorNoDo;
begin
  TestStatementError('WHILE (A>1) A=A-1');
end;

procedure TTestProcedureStatement.TestWhenAny;
Var
  W : TSQLWhenStatement;
begin
  W:=TSQLWhenStatement(CheckClass(TestStatement('WHEN ANY DO EXIT'),TSQLWhenStatement));
  AssertEquals('No error codes',0,W.Errors.Count);
  AssertEquals('Any error',True,W.AnyError);
  CheckClass(W.Statement,TSQLExitStatement);
end;

procedure TTestProcedureStatement.TestWhenSQLCode;
Var
  W : TSQLWhenStatement;
  E : TSQLWhenSQLError;

begin
  W:=TSQLWhenStatement(CheckClass(TestStatement('WHEN SQLCODE 1 DO EXIT'),TSQLWhenStatement));
  AssertEquals('Not Any error',False,W.AnyError);
  AssertEquals('1 error code',1,W.Errors.Count);
  CheckClass(W.Statement,TSQLExitStatement);
  E:=TSQLWhenSQLError(CheckClass(W.Errors[0],TSQLWhenSQLError));
  AssertEquals('Correct SQL Code',1,E.ErrorCode);
end;

procedure TTestProcedureStatement.TestWhenGDSCode;
Var
  W : TSQLWhenStatement;
  E : TSQLWhenGDSError;

begin
  W:=TSQLWhenStatement(CheckClass(TestStatement('WHEN GDSCODE 1 DO EXIT'),TSQLWhenStatement));
  AssertEquals('Not Any error',False,W.AnyError);
  AssertEquals('1 error code',1,W.Errors.Count);
  CheckClass(W.Statement,TSQLExitStatement);
  E:=TSQLWhenGDSError(CheckClass(W.Errors[0],TSQLWhenGDSError));
  AssertEquals('Correct SQL Code',1,E.GDSErrorNumber);
end;

procedure TTestProcedureStatement.TestWhenException;
Var
  W : TSQLWhenStatement;
  E : TSQLWhenException;

begin
  W:=TSQLWhenStatement(CheckClass(TestStatement('WHEN EXCEPTION MYE DO EXIT'),TSQLWhenStatement));
  AssertEquals('Not Any error',False,W.AnyError);
  AssertEquals('1 error code',1,W.Errors.Count);
  CheckClass(W.Statement,TSQLExitStatement);
  E:=TSQLWhenException(CheckClass(W.Errors[0],TSQLWhenException));
  AssertIdentifierName('Correct SQL Code','MYE',E.ExceptionName);
end;

procedure TTestProcedureStatement.TestWhenExceptionGDS;
Var
  W : TSQLWhenStatement;
  E : TSQLWhenException;
  G : TSQLWhenGDSError;

begin
  W:=TSQLWhenStatement(CheckClass(TestStatement('WHEN EXCEPTION MYE, GDSCODE 1 DO EXIT'),TSQLWhenStatement));
  AssertEquals('Not Any error',False,W.AnyError);
  AssertEquals('2 error code',2,W.Errors.Count);
  CheckClass(W.Statement,TSQLExitStatement);
  E:=TSQLWhenException(CheckClass(W.Errors[0],TSQLWhenException));
  AssertIdentifierName('Correct SQL Code','MYE',E.ExceptionName);
  G:=TSQLWhenGDSError(CheckClass(W.Errors[1],TSQLWhenGDSError));
  AssertEquals('Correct SQL Code',1,G.GDSErrorNumber);
end;

procedure TTestProcedureStatement.TestWhenAnyBlock;

Var
  W : TSQLWhenStatement;
  B : TSQLStatementBlock;

begin
  W:=TSQLWhenStatement(CheckClass(TestStatement('WHEN ANY DO BEGIN EXIT; END'),TSQLWhenStatement));
  AssertEquals('No error codes',0,W.Errors.Count);
  AssertEquals('Any error',True,W.AnyError);
  B:=TSQLStatementBlock(CheckClass(W.Statement,TSQLStatementBlock));
  AssertEquals('One statement',1,B.Statements.Count);
  CheckClass(B.Statements[0],TSQLExitStatement);
end;

procedure TTestProcedureStatement.TestWhenErrorAny;
begin
  TestStatementError('WHEN ANY, EXCEPTION MY DO EXIT');
end;

procedure TTestProcedureStatement.TestWhenErrorNoDo;

begin
  TestStatementError('WHEN ANY EXIT');
end;

procedure TTestProcedureStatement.TestWhenErrorExceptionInt;
begin
  TestStatementError('WHEN EXCEPTION 1 DO EXIT');
end;

procedure TTestProcedureStatement.TestWhenErrorExceptionString;
begin
  TestStatementError('WHEN EXCEPTION ''1'' DO EXIT');
end;

procedure TTestProcedureStatement.TestWhenErrorSqlCode;
begin
  TestStatementError('WHEN SQLCODE A DO EXIT');
end;

procedure TTestProcedureStatement.TestWhenErrorGDSCode;
begin
  TestStatementError('WHEN GDSCODE A DO EXIT');
end;

procedure TTestProcedureStatement.TestExecuteStatement;

Var
  E : TSQLExecuteProcedureStatement;

begin
  E:=TSQLExecuteProcedureStatement(CheckClass(TestStatement('EXECUTE PROCEDURE A'),TSQLExecuteProcedureStatement));
  AssertIDentifierName('Correct procedure','A',E.ProcedureName);
end;

procedure TTestProcedureStatement.TestExecuteStatementReturningValues;

Var
  E : TSQLExecuteProcedureStatement;

begin
  E:=TSQLExecuteProcedureStatement(CheckClass(TestStatement('EXECUTE PROCEDURE A RETURNING_VALUES B'),TSQLExecuteProcedureStatement));
  AssertIDentifierName('Correct procedure','A',E.ProcedureName);
  AssertEquals('Returning 1 value',1,E.Returning.Count);
  AssertIDentifierName('Correct return value','B',E.Returning[0]);
end;
procedure TTestProcedureStatement.TestExecuteStatementReturningValuesColon;

Var
  E : TSQLExecuteProcedureStatement;

begin
  E:=TSQLExecuteProcedureStatement(CheckClass(TestStatement('EXECUTE PROCEDURE A RETURNING_VALUES :B'),TSQLExecuteProcedureStatement));
  AssertIDentifierName('Correct procedure','A',E.ProcedureName);
  AssertEquals('Returning 1 value',1,E.Returning.Count);
  AssertIDentifierName('Correct return value','B',E.Returning[0]);
end;

procedure TTestProcedureStatement.TestExecuteStatementReturningValuesBrackets;

Var
  E : TSQLExecuteProcedureStatement;

begin
  E:=TSQLExecuteProcedureStatement(CheckClass(TestStatement('EXECUTE PROCEDURE A RETURNING_VALUES (:B)'),TSQLExecuteProcedureStatement));
  AssertIDentifierName('Correct procedure','A',E.ProcedureName);
  AssertEquals('Returning 1 value',1,E.Returning.Count);
  AssertIDentifierName('Correct return value','B',E.Returning[0]);
end;

procedure TTestProcedureStatement.TestForSimple;
Var
  F : TSQLForStatement;
  P : TSQLPostEventStatement;

begin
  F:=TSQLForStatement(CheckClass(TestStatement('FOR SELECT A FROM B INTO :C DO POST_EVENT C'),TSQLForStatement));
  AssertEquals('Field count',1,F.Select.Fields.Count);
  AssertEquals('Table count',1,F.Select.Tables.Count);
  AssertField(F.Select.Fields[0],'A','');
  AssertTable(F.Select.Tables[0],'B','');
  AssertEquals('Into Fieldlist count',1,F.FieldList.Count);
  AssertIdentifierName('Correct field name','C',F.FieldList[0]);
  P:=TSQLPostEventStatement(CheckClass(F.Statement,TSQLPostEventStatement));
  AssertIdentifierName('Event name','C',P.ColName);
end;

procedure TTestProcedureStatement.TestForSimpleNoColon;
Var
  F : TSQLForStatement;
  P : TSQLPostEventStatement;

begin
  F:=TSQLForStatement(CheckClass(TestStatement('FOR SELECT A FROM B INTO C DO POST_EVENT C'),TSQLForStatement));
  AssertEquals('Field count',1,F.Select.Fields.Count);
  AssertEquals('Table count',1,F.Select.Tables.Count);
  AssertField(F.Select.Fields[0],'A','');
  AssertTable(F.Select.Tables[0],'B','');
  AssertEquals('Into Fieldlist count',1,F.FieldList.Count);
  AssertIdentifierName('Correct field name','C',F.FieldList[0]);
  P:=TSQLPostEventStatement(CheckClass(F.Statement,TSQLPostEventStatement));
  AssertIdentifierName('Event name','C',P.ColName);
end;

procedure TTestProcedureStatement.TestForSimple2fields;
Var
  F : TSQLForStatement;
  P : TSQLPostEventStatement;

begin
  F:=TSQLForStatement(CheckClass(TestStatement('FOR SELECT A,B FROM C INTO :D,:E DO POST_EVENT D'),TSQLForStatement));
  AssertEquals('Field count',2,F.Select.Fields.Count);
  AssertEquals('Table count',1,F.Select.Tables.Count);
  AssertField(F.Select.Fields[0],'A','');
  AssertField(F.Select.Fields[1],'B','');
  AssertTable(F.Select.Tables[0],'C','');
  AssertEquals('Into Fieldlist count',2,F.FieldList.Count);
  AssertIdentifierName('Correct field name','D',F.FieldList[0]);
  AssertIdentifierName('Correct field name','E',F.FieldList[1]);
  P:=TSQLPostEventStatement(CheckClass(F.Statement,TSQLPostEventStatement));
  AssertIdentifierName('Event name','D',P.ColName);
end;

procedure TTestProcedureStatement.TestForBlock;
Var
  F : TSQLForStatement;
  P : TSQLPostEventStatement;
  B : TSQLStatementBlock;

begin
  F:=TSQLForStatement(CheckClass(TestStatement('FOR SELECT A FROM B INTO :C DO BEGIN POST_EVENT C; END'),TSQLForStatement));
  AssertEquals('Field count',1,F.Select.Fields.Count);
  AssertEquals('Table count',1,F.Select.Tables.Count);
  AssertField(F.Select.Fields[0],'A','');
  AssertTable(F.Select.Tables[0],'B','');
  AssertEquals('Into Fieldlist count',1,F.FieldList.Count);
  AssertIdentifierName('Correct field name','C',F.FieldList[0]);
  B:=TSQLStatementBlock(CheckClass(F.Statement,TSQLStatementBlock));
  AssertEquals('One statement',1,B.Statements.Count);
  P:=TSQLPostEventStatement(CheckClass(B.Statements[0],TSQLPostEventStatement));
  AssertIdentifierName('Event name','C',P.ColName);
end;

{ TTestCreateProcedureParser }

function TTestCreateProcedureParser.TestCreate(const ASource: String
  ): TSQLCreateProcedureStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLCreateProcedureStatement(CheckClass(FToFree,TSQLCreateProcedureStatement));
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestCreateProcedureParser.TestCreateError(const ASource: String
  );
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestCreateProcedureParser.TestEmptyProcedure;

begin
  TestCreate('CREATE PROCEDURE A AS BEGIN END');
  AssertIdentifierName('Correct procedure name','A',Statement.ObjectName);
  AssertEquals('No arguments',0,Statement.InputVariables.Count);
  AssertEquals('No return values',0,Statement.OutputVariables.Count);
  AssertEquals('No local variables',0,Statement.LocalVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestExitProcedure;

begin
  TestCreate('CREATE PROCEDURE A AS BEGIN EXIT; END');
  AssertIdentifierName('Correct procedure name','A',Statement.ObjectName);
  AssertEquals('No arguments',0,Statement.InputVariables.Count);
  AssertEquals('No return values',0,Statement.OutputVariables.Count);
  AssertEquals('No local variables',0,Statement.LocalVariables.Count);
  AssertEquals('One statement',1,Statement.Statements.Count);
  CheckClass(Statement.Statements[0],TSQLExitStatement);
end;

procedure TTestCreateProcedureParser.TestProcedureOneArgument;

Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A (P INT) AS BEGIN END');
  AssertIdentifierName('Correct procedure name','A',Statement.ObjectName);
  AssertEquals('1 arguments',1,Statement.InputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.InputVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  AssertEquals('No return values',0,Statement.OutputVariables.Count);
  AssertEquals('No local variables',0,Statement.LocalVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestProcedureTwoArguments;
Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A (P INT,Q CHAR(4)) AS BEGIN END');
  AssertIdentifierName('Correct procedure name','A',Statement.ObjectName);
  AssertEquals('Two arguments',2,Statement.InputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.InputVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  AssertEquals('No return values',0,Statement.OutputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.InputVariables[1],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','Q',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtChar,P.ParamType.DataType);
  AssertEquals('Correct length',4,P.ParamType.Len);
  //
  AssertEquals('No local variables',0,Statement.LocalVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestProcedureOneReturnValue;
Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A RETURNS (P INT) AS BEGIN END');
  AssertIdentifierName('Correct procedure name','A',Statement.ObjectName);
  AssertEquals('1 return value',1,Statement.OutputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.OutputVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  AssertEquals('No input values',0,Statement.InputVariables.Count);
  AssertEquals('No local variables',0,Statement.LocalVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestProcedureTwoReturnValues;
Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A RETURNS (P INT, Q CHAR(5)) AS BEGIN END');
  AssertIdentifierName('Correct procedure name','A',Statement.ObjectName);
  AssertEquals('2 return values',2,Statement.OutputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.OutputVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  P:=TSQLProcedureParamDef(CheckClass(Statement.OutputVariables[1],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','Q',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtChar,P.ParamType.DataType);
  AssertEquals('Correct length',5,P.ParamType.Len);
  AssertEquals('No input values',0,Statement.InputVariables.Count);
  AssertEquals('No local variables',0,Statement.LocalVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestProcedureOneLocalVariable;
Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A AS DECLARE VARIABLE P INT; BEGIN END');
  AssertIdentifierName('Correcte procedure naam','A',Statement.ObjectName);
  AssertEquals('0 return values',0,Statement.OutputVariables.Count);
  AssertEquals('1 local variable',1,Statement.LocalVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  AssertEquals('No input values',0,Statement.InputVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestProcedureTwoLocalVariable;
Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A AS DECLARE VARIABLE P INT; DECLARE VARIABLE Q CHAR(5); BEGIN END');
  AssertIdentifierName('Correcte procedure naam','A',Statement.ObjectName);
  AssertEquals('0 return values',0,Statement.OutputVariables.Count);
  AssertEquals('2 local variable',2,Statement.LocalVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[1],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','Q',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtChar,P.ParamType.DataType);
  AssertEquals('Correct length',5,P.ParamType.Len);
  AssertEquals('No input values',0,Statement.InputVariables.Count);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

procedure TTestCreateProcedureParser.TestProcedureInputOutputLocal;

Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE PROCEDURE A (P INT) RETURNS (Q CHAR(5)) AS DECLARE VARIABLE R VARCHAR(5); BEGIN END');
  AssertIdentifierName('Correcte procedure naam','A',Statement.ObjectName);
  // Input
  AssertEquals('1 input value',1,Statement.InputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.InputVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  // Output
  AssertEquals('1 return values',1,Statement.OutputVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.OutputVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','Q',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtChar,P.ParamType.DataType);
  AssertEquals('Correct length',5,P.ParamType.Len);
  // Local
  AssertEquals('1 local variable',1,Statement.LocalVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','R',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtvarChar,P.ParamType.DataType);
  AssertEquals('Correct length',5,P.ParamType.Len);
  AssertEquals('No statements',0,Statement.Statements.Count);
end;

{ TTestCreateTriggerParser }

function TTestCreateTriggerParser.TestCreate(const ASource: String
  ): TSQLCreateTriggerStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLCreateTriggerStatement(CheckClass(FToFree,TSQLCreateTriggerStatement));
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

function TTestCreateTriggerParser.TestAlter(const ASource: String
  ): TSQLAlterTriggerStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLAlterTriggerStatement(CheckClass(FToFree,TSQLAlterTriggerStatement));
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestCreateTriggerParser.TestCreateError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestCreateTriggerParser.TestEmptyTrigger;
begin
  TestCreate('CREATE TRIGGER A FOR B BEFORE UPDATE AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('No position',0,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Update operation',[toUpdate],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestExitTrigger;
begin
  TestCreate('CREATE TRIGGER A FOR B BEFORE UPDATE AS BEGIN EXIT; END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('1 Statements',1,Statement.Statements.Count);
  AssertEquals('No position',0,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Update operation',[toUpdate],Statement.Operations);
  CheckClass(Statement.Statements[0],TSQLExitStatement);
end;

procedure TTestCreateTriggerParser.TestEmptyTriggerAfterUpdate;
begin
  TestCreate('CREATE TRIGGER A FOR B AFTER UPDATE AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('No position',0,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmAfter,Statement.Moment);
  AssertEquals('Update operation',[toUpdate],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestEmptyTriggerBeforeDelete;
begin
  TestCreate('CREATE TRIGGER A FOR B BEFORE DELETE AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('No position',0,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Delete operation',[toDelete],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestEmptyTriggerBeforeInsert;
begin
  TestCreate('CREATE TRIGGER A FOR B BEFORE INSERT AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('No position',0,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Insert operation',[toInsert],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestEmptyTriggerBeforeInsertPosition1;
begin
  TestCreate('CREATE TRIGGER A FOR B BEFORE INSERT POSITION 1 AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('position 1',1,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Insert operation',[toInsert],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestEmptyTriggerBeforeInsertPosition1inActive;
begin
  TestCreate('CREATE TRIGGER A FOR B INACTIVE BEFORE INSERT POSITION 1 AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('position 1',1,Statement.Position);
  AssertEquals('inactive',tsInactive,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Insert operation',[toInsert],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestEmptyTriggerBeforeInsertPosition1Active;
begin
  TestCreate('CREATE TRIGGER A FOR B ACTIVE BEFORE INSERT POSITION 1 AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('position 1',1,Statement.Position);
  AssertEquals('Active',tsActive,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Insert operation',[toInsert],Statement.Operations);
end;

procedure TTestCreateTriggerParser.TestTriggerOneLocalVariable;

Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE TRIGGER A FOR B ACTIVE BEFORE INSERT POSITION 1 AS DECLARE VARIABLE P INT; BEGIN END');
  AssertIdentifierName('Correcte procedure naam','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('position 1',1,Statement.Position);
  AssertEquals('Active',tsActive,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Insert operation',[toInsert],Statement.Operations);
  AssertEquals('1 local variable',1,Statement.LocalVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
end;

procedure TTestCreateTriggerParser.TestTriggerTwoLocalVariables;
Var
  P : TSQLProcedureParamDef;

begin
  TestCreate('CREATE TRIGGER A FOR B ACTIVE BEFORE INSERT POSITION 1 AS DECLARE VARIABLE P INT; DECLARE VARIABLE Q INT; BEGIN END');
  AssertIdentifierName('Correcte procedure naam','A',Statement.ObjectName);
  AssertIdentifierName('Correct table','B',Statement.TableName);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('position 1',1,Statement.Position);
  AssertEquals('Active',tsActive,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Insert operation',[toInsert],Statement.Operations);
  AssertEquals('2 local variables',2,Statement.LocalVariables.Count);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[0],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','P',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
  P:=TSQLProcedureParamDef(CheckClass(Statement.LocalVariables[1],TSQLProcedureParamDef));
  AssertIdentifierName('Correct parameter name','Q',P.ParamName);
  AssertNotNull('Have type definition',P.ParamType);
  AssertEquals('Correct type',sdtInteger,P.ParamType.DataType);
end;

procedure TTestCreateTriggerParser.TestAlterTrigger;
begin
  TestAlter('ALTER TRIGGER A BEFORE UPDATE AS BEGIN END');
  AssertIdentifierName('Correct trigger name','A',Statement.ObjectName);
  AssertNull('Correct table',Statement.TableName);
  AssertEquals('No variables',0,Statement.LocalVariables.Count);
  AssertEquals('No Statements',0,Statement.Statements.Count);
  AssertEquals('No position',0,Statement.Position);
  AssertEquals('No active/inactive',tsNone,Statement.State);
  AssertEquals('Before moment',tmBefore,Statement.Moment);
  AssertEquals('Update operation',[toUpdate],Statement.Operations);

end;

{ TTestDeclareExternalFunctionParser }

function TTestDeclareExternalFunctionParser.TestCreate(const ASource: String
  ): TSQLDeclareExternalFunctionStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  Result:=TSQLDeclareExternalFunctionStatement(CheckClass(FToFree,TSQLDeclareExternalFunctionStatement));
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestDeclareExternalFunctionParser.TestCreateError(
  const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestDeclareExternalFunctionParser.TestEmptyfunction;
begin
  TestCreate('DECLARE EXTERNAL FUNCTION A RETURNS INT ENTRY_POINT ''A'' MODULE_NAME ''B''');
  AssertIdentifierName('Correct function','A',Statement.ObjectName);
  AssertEquals('Correct entry point','A',Statement.EntryPoint);
  AssertEquals('Correct module name','B',Statement.ModuleName);
  AssertEquals('No arguments',0,Statement.Arguments.Count);
  AssertNotNull('Have return type',Statement.ReturnType);
  AssertEquals('No FreeIt',False,Statement.FreeIt);
  AssertEquals('Correct return type',sdtInteger,Statement.ReturnType.DataType);
end;

procedure TTestDeclareExternalFunctionParser.TestEmptyfunctionByValue;
begin
  TestCreate('DECLARE EXTERNAL FUNCTION A RETURNS INT BY VALUE ENTRY_POINT ''A'' MODULE_NAME ''B''');
  AssertIdentifierName('Correct function','A',Statement.ObjectName);
  AssertEquals('Correct entry point','A',Statement.EntryPoint);
  AssertEquals('Correct module name','B',Statement.ModuleName);
  AssertEquals('No arguments',0,Statement.Arguments.Count);
  AssertNotNull('Have return type',Statement.ReturnType);
  AssertEquals('No FreeIt',False,Statement.FreeIt);
  AssertEquals('Correct return type',sdtInteger,Statement.ReturnType.DataType);
  AssertEquals('By Value',True,Statement.ReturnType.ByValue);
end;

procedure TTestDeclareExternalFunctionParser.TestCStringfunction;

begin
  TestCreate('DECLARE EXTERNAL FUNCTION A RETURNS CSTRING (50) ENTRY_POINT ''A'' MODULE_NAME ''B''');
  AssertIdentifierName('Correct function','A',Statement.ObjectName);
  AssertEquals('Correct entry point','A',Statement.EntryPoint);
  AssertEquals('Correct module name','B',Statement.ModuleName);
  AssertEquals('No arguments',0,Statement.Arguments.Count);
  AssertNotNull('Have return type',Statement.ReturnType);
  AssertEquals('No FreeIt',False,Statement.FreeIt);
  AssertEquals('Correct return type',sdtCstring,Statement.ReturnType.DataType);
  AssertEquals('Correct return length',50,Statement.ReturnType.Len);
end;

procedure TTestDeclareExternalFunctionParser.TestCStringFreeItfunction;
begin
  TestCreate('DECLARE EXTERNAL FUNCTION A RETURNS CSTRING (50) FREE_IT ENTRY_POINT ''A'' MODULE_NAME ''B''');
  AssertIdentifierName('Correct function','A',Statement.ObjectName);
  AssertEquals('Correct entry point','A',Statement.EntryPoint);
  AssertEquals('Correct module name','B',Statement.ModuleName);
  AssertEquals('No arguments',0,Statement.Arguments.Count);
  AssertNotNull('Have return type',Statement.ReturnType);
  AssertEquals('FreeIt',True,Statement.FreeIt);
  AssertEquals('Correct return type',sdtCstring,Statement.ReturnType.DataType);
  AssertEquals('Correct return length',50,Statement.ReturnType.Len);
end;

procedure TTestDeclareExternalFunctionParser.TestOneArgumentFunction;

Var
  T : TSQLTypeDefinition;

begin
  TestCreate('DECLARE EXTERNAL FUNCTION A INT RETURNS INT ENTRY_POINT ''A'' MODULE_NAME ''B''');
  AssertIdentifierName('Correct function','A',Statement.ObjectName);
  AssertEquals('Correct entry point','A',Statement.EntryPoint);
  AssertEquals('Correct module name','B',Statement.ModuleName);
  AssertEquals('1 argument',1,Statement.Arguments.Count);
  T:=TSQLTypeDefinition(CheckClass(Statement.Arguments[0],TSQLTypeDefinition));
  AssertEquals('Correct return type',sdtInteger,T.DataType);
  AssertNotNull('Have return type',Statement.ReturnType);
  AssertEquals('No FreeIt',False,Statement.FreeIt);
  AssertEquals('Correct return type',sdtInteger,Statement.ReturnType.DataType);
end;

procedure TTestDeclareExternalFunctionParser.TestTwoArgumentsFunction;

Var
  T : TSQLTypeDefinition;

begin
  TestCreate('DECLARE EXTERNAL FUNCTION A INT, CSTRING(10) RETURNS INT ENTRY_POINT ''A'' MODULE_NAME ''B''');
  AssertIdentifierName('Correct function','A',Statement.ObjectName);
  AssertEquals('Correct entry point','A',Statement.EntryPoint);
  AssertEquals('Correct module name','B',Statement.ModuleName);
  AssertEquals('2 arguments',2,Statement.Arguments.Count);
  T:=TSQLTypeDefinition(CheckClass(Statement.Arguments[0],TSQLTypeDefinition));
  AssertEquals('Correct argument type',sdtInteger,T.DataType);
  T:=TSQLTypeDefinition(CheckClass(Statement.Arguments[1],TSQLTypeDefinition));
  AssertEquals('Correct return type',sdtCstring,T.DataType);
  AssertEquals('Correct argument length',10,T.Len);
  AssertNotNull('Have return type',Statement.ReturnType);
  AssertEquals('No FreeIt',False,Statement.FreeIt);
  AssertEquals('Correct return type',sdtInteger,Statement.ReturnType.DataType);
end;

{ TTestGrantParser }

function TTestGrantParser.TestGrant(const ASource: String): TSQLGrantStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  If not (FToFree is  TSQLGrantStatement) then
    Fail(Format('Wrong parse result class. Expected TSQLGrantStatement, got %s',[FTofree.ClassName]));
  Result:=TSQLGrantStatement(Ftofree);
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestGrantParser.TestGrantError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestGrantParser.TestSimple;

Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT SELECT ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.Test2Operations;

Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT SELECT,INSERT ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('Two permissions',2,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  CheckClass(T.Privileges[1],TSQLINSERTPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestDeletePrivilege;

Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT DELETE ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLDeletePrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestUpdatePrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT UPDATE ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLUPDATEPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestInsertPrivilege;

Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT INSERT ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLInsertPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestReferencePrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT REFERENCES ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLReferencePrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestAllPrivileges;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT ALL ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLAllPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestAllPrivileges2;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
  TestGrant('GRANT ALL PRIVILEGES ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLAllPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestUpdateColPrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;
  U : TSQLUPDATEPrivilege;

begin
  TestGrant('GRANT UPDATE (C) ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLUPDATEPrivilege(CheckClass(T.Privileges[0],TSQLUPDATEPrivilege));
  AssertEquals('1 column',1,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestUpdate2ColsPrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;
  U : TSQLUPDATEPrivilege;

begin
  TestGrant('GRANT UPDATE (C,D) ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLUPDATEPrivilege(CheckClass(T.Privileges[0],TSQLUPDATEPrivilege));
  AssertEquals('2 column',2,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertIdentifierName('Column D','D',U.Columns[1]);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestReferenceColPrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;
  U : TSQLReferencePrivilege;

begin
  TestGrant('GRANT REFERENCES (C) ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLReferencePrivilege(CheckClass(T.Privileges[0],TSQLReferencePrivilege));
  AssertEquals('1 column',1,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestReference2ColsPrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;
  U : TSQLReferencePrivilege;

begin
  TestGrant('GRANT REFERENCES (C,D) ON A TO B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLReferencePrivilege(CheckClass(T.Privileges[0],TSQLReferencePrivilege));
  AssertEquals('2 column',2,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertIdentifierName('Column D','D',U.Columns[1]);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestUserPrivilege;

Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
    TestGrant('GRANT SELECT ON A TO USER B');
    T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
    AssertIdentifierName('Table name','A',T.TableName);
    AssertEquals('One grantee', 1,T.Grantees.Count);
    G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
    AssertEquals('Grantee B','B',G.Name);
    AssertEquals('One permission',1,T.Privileges.Count);
    CheckClass(T.Privileges[0],TSQLSelectPrivilege);
    AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestUserPrivilegeWithGrant;
Var
  t : TSQLTableGrantStatement;
  G : TSQLUSerGrantee;

begin
    TestGrant('GRANT SELECT ON A TO USER B WITH GRANT OPTION');
    T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
    AssertIdentifierName('Table name','A',T.TableName);
    AssertEquals('One grantee', 1,T.Grantees.Count);
    G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
    AssertEquals('Grantee B','B',G.Name);
    AssertEquals('One permission',1,T.Privileges.Count);
    CheckClass(T.Privileges[0],TSQLSelectPrivilege);
    AssertEquals('With grant option',True,T.GrantOption);
end;

procedure TTestGrantParser.TestGroupPrivilege;

Var
  t : TSQLTableGrantStatement;
  G : TSQLGroupGrantee;

begin
    TestGrant('GRANT SELECT ON A TO GROUP B');
    T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
    AssertIdentifierName('Table name','A',T.TableName);
    AssertEquals('One grantee', 1,T.Grantees.Count);
    G:=TSQLGroupGrantee(CheckClass(T.Grantees[0],TSQLGroupGrantee));
    AssertEquals('Grantee B','B',G.Name);
    AssertEquals('One permission',1,T.Privileges.Count);
    CheckClass(T.Privileges[0],TSQLSelectPrivilege);
    AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestProcedurePrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLProcedureGrantee;

begin
  TestGrant('GRANT SELECT ON A TO PROCEDURE B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLProcedureGrantee(CheckClass(T.Grantees[0],TSQLProcedureGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestViewPrivilege;
Var
  t : TSQLTableGrantStatement;
  G : TSQLViewGrantee;

begin
  TestGrant('GRANT SELECT ON A TO VIEW B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLViewGrantee(CheckClass(T.Grantees[0],TSQLViewGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestTriggerPrivilege;

Var
  t : TSQLTableGrantStatement;
  G : TSQLTriggerGrantee;

begin
  TestGrant('GRANT SELECT ON A TO TRIGGER B');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  G:=TSQLTriggerGrantee(CheckClass(T.Grantees[0],TSQLTriggerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestPublicPrivilege;
Var
  t : TSQLTableGrantStatement;
begin
  TestGrant('GRANT SELECT ON A TO PUBLIC');
  T:=TSQLTableGrantStatement(CheckClass(Statement,TSQLTableGrantStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One grantee', 1,T.Grantees.Count);
  (CheckClass(T.Grantees[0],TSQLPublicGrantee));
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No grant option',False,T.GrantOption);
end;

procedure TTestGrantParser.TestExecuteToUser;
Var
  P : TSQLProcedureGrantStatement;
  U : TSQLUserGrantee;

begin
  TestGrant('GRANT EXECUTE ON PROCEDURE A TO B');
  P:=TSQLProcedureGrantStatement(CheckClass(Statement,TSQLProcedureGrantStatement));
  AssertIdentifierName('Procedure name','A',P.ProcedureName);
  AssertEquals('One grantee', 1,P.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(P.Grantees[0],TSQLUserGrantee));
  AssertEquals('User name','B',U.Name);
  AssertEquals('No grant option',False,P.GrantOption);
end;

procedure TTestGrantParser.TestExecuteToProcedure;
Var
  P : TSQLProcedureGrantStatement;
  U : TSQLProcedureGrantee;

begin
  TestGrant('GRANT EXECUTE ON PROCEDURE A TO PROCEDURE B');
  P:=TSQLProcedureGrantStatement(CheckClass(Statement,TSQLProcedureGrantStatement));
  AssertIdentifierName('Procedure name','A',P.ProcedureName);
  AssertEquals('One grantee', 1,P.Grantees.Count);
  U:=TSQLProcedureGrantee(CheckClass(P.Grantees[0],TSQLProcedureGrantee));
  AssertEquals('Procedure grantee name','B',U.Name);
  AssertEquals('No grant option',False,P.GrantOption);
end;

procedure TTestGrantParser.TestRoleToUser;
Var
  R : TSQLRoleGrantStatement;
  U : TSQLUserGrantee;

begin
  TestGrant('GRANT A TO B');
  R:=TSQLRoleGrantStatement(CheckClass(Statement,TSQLRoleGrantStatement));
  AssertEquals('One role', 1,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertEquals('One grantee', 1,R.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(R.Grantees[0],TSQLUserGrantee));
  AssertEquals('Procedure grantee name','B',U.Name);
  AssertEquals('No admin option',False,R.AdminOption);
end;

procedure TTestGrantParser.TestRoleToUserWithAdmin;
Var
  R : TSQLRoleGrantStatement;
  U : TSQLUserGrantee;

begin
  TestGrant('GRANT A TO B WITH ADMIN OPTION');
  R:=TSQLRoleGrantStatement(CheckClass(Statement,TSQLRoleGrantStatement));
  AssertEquals('One role', 1,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertEquals('One grantee', 1,R.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(R.Grantees[0],TSQLUserGrantee));
  AssertEquals('Procedure grantee name','B',U.Name);
  AssertEquals('Admin option',True,R.AdminOption);
end;

procedure TTestGrantParser.TestRoleToPublic;
Var
  R : TSQLRoleGrantStatement;

begin
  TestGrant('GRANT A TO PUBLIC');
  R:=TSQLRoleGrantStatement(CheckClass(Statement,TSQLRoleGrantStatement));
  AssertEquals('One role', 1,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertEquals('One grantee', 1,R.Grantees.Count);
  CheckClass(R.Grantees[0],TSQLPublicGrantee);
  AssertEquals('No admin option',False,R.AdminOption);
end;

procedure TTestGrantParser.Test2RolesToUser;

Var
  R : TSQLRoleGrantStatement;
  U : TSQLUserGrantee;

begin
  TestGrant('GRANT A,C TO B');
  R:=TSQLRoleGrantStatement(CheckClass(Statement,TSQLRoleGrantStatement));
  AssertEquals('2 roles', 2,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertIdentifierName('Role name','C',R.Roles[1]);
  AssertEquals('One grantee', 1,R.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(R.Grantees[0],TSQLUserGrantee));
  AssertEquals('Procedure grantee name','B',U.Name);
  AssertEquals('No admin option',False,R.AdminOption);
end;

{ TTestRevokeParser }

function TTestRevokeParser.TestRevoke(const ASource: String): TSQLRevokeStatement;
begin
  CreateParser(ASource);
  FToFree:=Parser.Parse;
  If not (FToFree is  TSQLRevokeStatement) then
    Fail(Format('Wrong parse result class. Expected TSQLRevokeStatement, got %s',[FTofree.ClassName]));
  Result:=TSQLRevokeStatement(Ftofree);
  FSTatement:=Result;
  AssertEquals('End of stream reached',tsqlEOF,Parser.CurrentToken);
end;

procedure TTestRevokeParser.TestRevokeError(const ASource: String);
begin
  FErrSource:=ASource;
  AssertException(ESQLParser,@TestParseError);
end;

procedure TTestRevokeParser.TestSimple;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke SELECT ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.Test2Operations;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke SELECT,INSERT ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('Two permissions',2,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  CheckClass(T.Privileges[1],TSQLINSERTPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestDeletePrivilege;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke DELETE ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLDeletePrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestUpdatePrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke UPDATE ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLUPDATEPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestInsertPrivilege;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke INSERT ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLInsertPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestReferencePrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke REFERENCES ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLReferencePrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestAllPrivileges;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke ALL ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLAllPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestAllPrivileges2;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
  TestRevoke('Revoke ALL PRIVILEGES ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLAllPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestUpdateColPrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;
  U : TSQLUPDATEPrivilege;

begin
  TestRevoke('Revoke UPDATE (C) ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLUPDATEPrivilege(CheckClass(T.Privileges[0],TSQLUPDATEPrivilege));
  AssertEquals('1 column',1,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestUpdate2ColsPrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;
  U : TSQLUPDATEPrivilege;

begin
  TestRevoke('Revoke UPDATE (C,D) ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLUPDATEPrivilege(CheckClass(T.Privileges[0],TSQLUPDATEPrivilege));
  AssertEquals('2 column',2,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertIdentifierName('Column D','D',U.Columns[1]);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestReferenceColPrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;
  U : TSQLReferencePrivilege;

begin
  TestRevoke('Revoke REFERENCES (C) ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLReferencePrivilege(CheckClass(T.Privileges[0],TSQLReferencePrivilege));
  AssertEquals('1 column',1,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestReference2ColsPrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;
  U : TSQLReferencePrivilege;

begin
  TestRevoke('Revoke REFERENCES (C,D) ON A FROM B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  U:=TSQLReferencePrivilege(CheckClass(T.Privileges[0],TSQLReferencePrivilege));
  AssertEquals('2 column',2,U.Columns.Count);
  AssertIdentifierName('Column C','C',U.Columns[0]);
  AssertIdentifierName('Column D','D',U.Columns[1]);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestUserPrivilege;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
    TestRevoke('Revoke SELECT ON A FROM USER B');
    T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
    AssertIdentifierName('Table name','A',T.TableName);
    AssertEquals('One Grantee', 1,T.Grantees.Count);
    G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
    AssertEquals('Grantee B','B',G.Name);
    AssertEquals('One permission',1,T.Privileges.Count);
    CheckClass(T.Privileges[0],TSQLSelectPrivilege);
    AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestUserPrivilegeWithRevoke;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLUSerGrantee;

begin
    TestRevoke('Revoke GRANT OPTION FOR SELECT ON A FROM USER B');
    T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
    AssertIdentifierName('Table name','A',T.TableName);
    AssertEquals('One Grantee', 1,T.Grantees.Count);
    G:=TSQLUSerGrantee(CheckClass(T.Grantees[0],TSQLUSerGrantee));
    AssertEquals('Grantee B','B',G.Name);
    AssertEquals('One permission',1,T.Privileges.Count);
    CheckClass(T.Privileges[0],TSQLSelectPrivilege);
    AssertEquals('With Revoke option',True,T.GrantOption);
end;

procedure TTestRevokeParser.TestGroupPrivilege;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLGroupGrantee;

begin
    TestRevoke('Revoke SELECT ON A FROM GROUP B');
    T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
    AssertIdentifierName('Table name','A',T.TableName);
    AssertEquals('One Grantee', 1,T.Grantees.Count);
    G:=TSQLGroupGrantee(CheckClass(T.Grantees[0],TSQLGroupGrantee));
    AssertEquals('Grantee B','B',G.Name);
    AssertEquals('One permission',1,T.Privileges.Count);
    CheckClass(T.Privileges[0],TSQLSelectPrivilege);
    AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestProcedurePrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLProcedureGrantee;

begin
  TestRevoke('Revoke SELECT ON A FROM PROCEDURE B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLProcedureGrantee(CheckClass(T.Grantees[0],TSQLProcedureGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestViewPrivilege;
Var
  t : TSQLTableRevokeStatement;
  G : TSQLViewGrantee;

begin
  TestRevoke('Revoke SELECT ON A FROM VIEW B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLViewGrantee(CheckClass(T.Grantees[0],TSQLViewGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestTriggerPrivilege;

Var
  t : TSQLTableRevokeStatement;
  G : TSQLTriggerGrantee;

begin
  TestRevoke('Revoke SELECT ON A FROM TRIGGER B');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  G:=TSQLTriggerGrantee(CheckClass(T.Grantees[0],TSQLTriggerGrantee));
  AssertEquals('Grantee B','B',G.Name);
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestPublicPrivilege;
Var
  t : TSQLTableRevokeStatement;
begin
  TestRevoke('Revoke SELECT ON A FROM PUBLIC');
  T:=TSQLTableRevokeStatement(CheckClass(Statement,TSQLTableRevokeStatement));
  AssertIdentifierName('Table name','A',T.TableName);
  AssertEquals('One Grantee', 1,T.Grantees.Count);
  (CheckClass(T.Grantees[0],TSQLPublicGrantee));
  AssertEquals('One permission',1,T.Privileges.Count);
  CheckClass(T.Privileges[0],TSQLSelectPrivilege);
  AssertEquals('No Revoke option',False,T.GrantOption);
end;

procedure TTestRevokeParser.TestExecuteToUser;
Var
  P : TSQLProcedureRevokeStatement;
  U : TSQLUserGrantee;

begin
  TestRevoke('Revoke EXECUTE ON PROCEDURE A FROM B');
  P:=TSQLProcedureRevokeStatement(CheckClass(Statement,TSQLProcedureRevokeStatement));
  AssertIdentifierName('Procedure name','A',P.ProcedureName);
  AssertEquals('One Grantee', 1,P.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(P.Grantees[0],TSQLUserGrantee));
  AssertEquals('User name','B',U.Name);
  AssertEquals('No Revoke option',False,P.GrantOption);
end;

procedure TTestRevokeParser.TestExecuteToProcedure;
Var
  P : TSQLProcedureRevokeStatement;
  U : TSQLProcedureGrantee;

begin
  TestRevoke('Revoke EXECUTE ON PROCEDURE A FROM PROCEDURE B');
  P:=TSQLProcedureRevokeStatement(CheckClass(Statement,TSQLProcedureRevokeStatement));
  AssertIdentifierName('Procedure name','A',P.ProcedureName);
  AssertEquals('One Grantee', 1,P.Grantees.Count);
  U:=TSQLProcedureGrantee(CheckClass(P.Grantees[0],TSQLProcedureGrantee));
  AssertEquals('Procedure Grantee name','B',U.Name);
  AssertEquals('No Revoke option',False,P.GrantOption);
end;

procedure TTestRevokeParser.TestRoleToUser;
Var
  R : TSQLRoleRevokeStatement;
  U : TSQLUserGrantee;

begin
  TestRevoke('Revoke A FROM B');
  R:=TSQLRoleRevokeStatement(CheckClass(Statement,TSQLRoleRevokeStatement));
  AssertEquals('One role', 1,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertEquals('One Grantee', 1,R.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(R.Grantees[0],TSQLUserGrantee));
  AssertEquals('Procedure Grantee name','B',U.Name);
  AssertEquals('No admin option',False,R.AdminOption);
end;

procedure TTestRevokeParser.TestRoleToPublic;
Var
  R : TSQLRoleRevokeStatement;

begin
  TestRevoke('Revoke A FROM PUBLIC');
  R:=TSQLRoleRevokeStatement(CheckClass(Statement,TSQLRoleRevokeStatement));
  AssertEquals('One role', 1,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertEquals('One Grantee', 1,R.Grantees.Count);
  CheckClass(R.Grantees[0],TSQLPublicGrantee);
  AssertEquals('No admin option',False,R.AdminOption);
end;

procedure TTestRevokeParser.Test2RolesToUser;

Var
  R : TSQLRoleRevokeStatement;
  U : TSQLUserGrantee;

begin
  TestRevoke('Revoke A,C FROM B');
  R:=TSQLRoleRevokeStatement(CheckClass(Statement,TSQLRoleRevokeStatement));
  AssertEquals('2 roles', 2,R.Roles.Count);
  AssertIdentifierName('Role name','A',R.Roles[0]);
  AssertIdentifierName('Role name','C',R.Roles[1]);
  AssertEquals('One Grantee', 1,R.Grantees.Count);
  U:=TSQLUserGrantee(CheckClass(R.Grantees[0],TSQLUserGrantee));
  AssertEquals('Procedure Grantee name','B',U.Name);
  AssertEquals('No admin option',False,R.AdminOption);
end;


initialization
  RegisterTests([TTestDropParser,
                 TTestGeneratorParser,
                 TTestRoleParser,
                 TTestTypeParser,
                 TTestCheckParser,
                 TTestDomainParser,
                 TTestExceptionParser,
                 TTestIndexParser,
                 TTestTableParser,
                 TTestDeleteParser,
                 TTestUpdateParser,
                 TTestInsertParser,
                 TTestSelectParser,
                 TTestRollbackParser,
                 TTestCommitParser,
                 TTestExecuteProcedureParser,
                 TTestConnectParser,
                 TTestCreateDatabaseParser,
                 TTestAlterDatabaseParser,
                 TTestCreateViewParser,
                 TTestCreateShadowParser,
                 TTestProcedureStatement,
                 TTestCreateProcedureParser,
                 TTestCreateTriggerParser,
                 TTestDeclareExternalFunctionParser,
                 TTestGrantParser,
                 TTestRevokeParser,
                 TTestGlobalParser,
                 TTestSetTermParser]);
end.

