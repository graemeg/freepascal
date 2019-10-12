{
    This file is part of the Free Component Library
    Copyright (c) 2010-2014 by the Free Pascal development team

    SQL Syntax Tree SQL generation tests

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit tcgensql;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry,fpsqltree;

type
  TSQLDropStatementClass = Class of TSQLDropStatement;
  TSQLGranteeClass = Class of TSQLGrantee;

  { TTestGenerateSQL }

  TTestGenerateSQL= class(TTestCase)
  Private
    FToFree:TSQLElement;
  protected
    Procedure SetUp; override;
    Procedure TearDown; override;
    Procedure DoTestDropStatement(AClass: TSQLDropStatementClass; const AObjectName: String);
    Procedure DoTestAlterCreateProcedure(S: TSQLAlterCreateProcedureStatement; PHEAD: String);
    Procedure DoTestAlterCreateTrigger(S: TSQLAlterCreateTriggerStatement; PHEAD: String);
    Function CreateIdentifier(Const AName : TSQLStringType) : TSQLIdentifierName;
    Function CreateGrantee(Const AName : TSQLStringType; AClass : TSQLGranteeClass = Nil) : TSQLGrantee;
    Function CreateLiteral(Const AValue : Integer) : TSQLIntegerLiteral;
    Function CreateLiteral(Const AValue : TSQLStringType) : TSQLStringLiteral;
    Function CreateLiteral(Const AValue : Double) : TSQLFloatLiteral;
    Function CreateTypeDefinition(Const ADataType : TSQLDataType; ALen : Integer = 0): TSQLTypeDefinition;
    Function CreateLiteralExpression(Const ALiteral : TSQLLiteral) : TSQLLiteralExpression;
    Function CreateIdentifierExpression(Const AName : TSQLStringtype) : TSQLIdentifierExpression;
    Function CreateUnaryExpression(Const AOperand : TSQLExpression) : TSQLUnaryExpression;
    Function CreateBinaryExpression(Const ALeft,ARight : TSQLExpression) : TSQLBinaryExpression;
    Function CreateUpdatePair(Const AField : TSQLStringType; AValue : TSQLExpression) : TSQLUpdatePair;
    Function CreateTernaryExpression(Const ALeft,AMiddle,ARight : TSQLExpression) : TSQLTernaryExpression;
    Function CreateForeignKey(Const ATable : TSQLStringType; AFieldName1,AFieldName2 : TSQLStringType):TSQLForeignKeyDefinition;
    Function CreateSelectField(Const AValue : TSQLExpression; Const AAlias : TSQLStringType) : TSQLSelectField;
    Function CreateSimpleTableReference(Const ATableName : TSQLStringType; AAlias : TSQLStringType) : TSQLSimpleTableReference;
    Function CreateJoinTablereference(Const ALeft,ARight : TSQLTableReference) : TSQLJoinTableReference;
    Function CreateSelect(AField : TSQLExpression; ATable : TSQLStringType) : TSQLSelectStatement;
    Procedure AssertSQL(Const AElement : TSQLElement; Const ASQL : TSQLStringType; AOptions : TSQLFormatOptions = []);
  published
    procedure TestIdentifier;
    Procedure TestIntegerLiteral;
    procedure TestStringLiteral;
    Procedure TestFloatLiteral;
    Procedure TestNullLiteral;
    Procedure TestUserLiteral;
    Procedure TestValueLiteral;
    Procedure TestLiteralExpression;
    Procedure TestSelectField;
    Procedure TestSimpleTablereference;
    Procedure TestSimpleSelect;
    Procedure TestAnyExpression;
    procedure TestAllExpression;
    procedure TestExistsExpression;
    procedure TestSomeExpression;
    procedure TestSingularExpression;
    Procedure TestUnaryExpression;
    procedure TestBinaryExpression;
    procedure TestListExpression;
    procedure TestTernaryExpression;
    Procedure TestGenIDExpression;
    Procedure TestFunctionCall;
    procedure TestAggregateFunction;
    procedure TestForeignKey;
    procedure TestUniqueFieldConstraint;
    procedure TestPrimaryKeyFieldConstraint;
    procedure TestForeignKeyFieldConstraint;
    procedure TestCheckFieldConstraint;
    procedure TestTableUniqueConstraintDef;
    procedure TestTablePrimaryKeyConstraintDef;
    procedure TestTableForeignKeyConstraintDef;
    procedure TestTableCheckConstraintDef;
    Procedure TestTypeDefinition;
    Procedure TestCastExpression;
    procedure TestJoinTableReference;
    procedure TestPlanNatural;
    procedure TestPlanIndex;
    procedure TestPlanOrder;
    procedure TestPlanExpression;
    procedure TestOrderBy;
    Procedure TestSelect;
    procedure TestInsert;
    procedure TestUpdatePair;
    procedure TestUpdate;
    procedure TestDelete;
    procedure TestRollback;
    procedure TestCommit;
    procedure TestExecuteProcedure;
    procedure TestCreateGenerator;
    procedure TestCreateRole;
    procedure TestCreateDomain;
    procedure TestAlterDomainDropDefault;
    procedure TestAlterDomainDropCheck;
    Procedure TestAlterDomainSetDefault;
    Procedure TestAlterDomainRename;
    procedure TestAlterDomainNewType;
    procedure TestAlterDomainAddCheck;
    procedure TestCreateException;
    procedure TestAlterException;
    procedure TestCreateIndex;
    procedure TestAlterIndex;
    procedure TestDeclareExternalFunction;
    procedure TestTableFieldDefinition;
    procedure TestCreateTable;
    procedure TestDropFieldOperation;
    procedure TestDropConstraintOperation;
    procedure TestAlterFieldNameOperation;
    procedure TestAlterFieldTypeOperation;
    procedure TestAlterFieldPositionOperation;
    procedure TestAddFieldOperation;
    procedure TestAddConstraintOperation;
    procedure TestAlterTable;
    procedure TestCreateView;
    procedure TestDatabaseFileInfo;
    procedure TestCreateDatabase;
    procedure TestAlterDatabase;
    procedure TestCreateShadow;
    procedure TestSuspend;
    procedure TestExit;
    procedure TestBlock;
    procedure TestAssignment;
    procedure TestIf;
    procedure TestFor;
    procedure TestWhile;
    Procedure TestWhenSQLError;
    Procedure TestWhenGDSError;
    Procedure TestWhenException;
    Procedure TestWhen;
    Procedure TestException;
    Procedure TestPostEvent;
    Procedure TestTriggerProcedure;
    procedure TestCreateProcedure;
    procedure TestAlterProcedure;
    procedure TestCreateTrigger;
    procedure TestAlterTrigger;
    Procedure TestDropStatement;
    procedure TestConnect;
    procedure TestExtract;
    procedure TestParamExpression;
    Procedure TestGrantTable;
    Procedure TestGrantProcedure;
    Procedure TestGrantRole;
    Procedure TestRevokeTable;
    Procedure TestRevokeProcedure;
    Procedure TestRevokeRole;
  end;

implementation

procedure TTestGenerateSQL.SetUp; 
begin
  FToFree:=Nil;
end; 

procedure TTestGenerateSQL.TearDown; 
begin
  FreeAndNil(FToFree);
end;

function TTestGenerateSQL.CreateIdentifier(const AName: TSQLStringType
  ): TSQLIdentifierName;
begin
  Result:=TSQLIdentifierName.Create(Nil);
  Result.Name:=AName;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateGrantee(const AName: TSQLStringType;
  AClass: TSQLGranteeClass = Nil): TSQLGrantee;
begin
  If AClass=Nil then
    AClass:=TSQLGrantee;
  Result:=AClass.Create(Nil);
  Result.Name:=AName;
  FToFree:=Result;
end;

procedure TTestGenerateSQL.AssertSQL(const AElement: TSQLElement;
  const ASQL: TSQLStringType; AOptions: TSQLFormatOptions = []);

Var
  S: TSQLStringType;
begin
  S:=AElement.GetAsSQL(AOptions);
  AssertEquals('Correct SQL',ASQL,S);
end;


function TTestGenerateSQL.CreateLiteral(const AValue: Integer
  ): TSQLIntegerLiteral;
begin
  Result:=TSQLintegerLiteral.Create(Nil);
  Result.Value:=AValue;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateLiteral(const AValue: TSQLStringType
  ): TSQLStringLiteral;
begin
  Result:=TSQLStringLiteral.Create(Nil);
  Result.Value:=AValue;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateLiteral(const AValue: Double
  ): TSQLFloatLiteral;
begin
  Result:=TSQLFloatLiteral.Create(Nil);
  Result.Value:=AValue;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateTypeDefinition(const ADataType: TSQLDataType;
  ALen: Integer): TSQLTypeDefinition;
begin
  Result:=TSQLTypeDefinition.Create(Nil);
  Result.DataType:=ADataType;
  Result.Len:=ALen;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateLiteralExpression(const ALiteral: TSQLLiteral
  ): TSQLLiteralExpression;
begin
  Result:=TSQLLiteralExpression.Create(Nil);
  FToFree:=Result;
  Result.Literal:=ALiteral;
end;

function TTestGenerateSQL.CreateIdentifierExpression(const AName: TSQLStringtype
  ): TSQLIdentifierExpression;
begin
  Result:=TSQLidentifierExpression.Create(Nil);
  Result.Identifier:=CreateIdentifier(AName);
  FTofree:=Result;
end;

function TTestGenerateSQL.CreateUnaryExpression(const AOperand: TSQLExpression
  ): TSQLUnaryExpression;
begin
  Result:=TSQLUnaryExpression.Create(Nil);
  Result.Operand:=AOperand;
  FTofree:=Result;
end;

function TTestGenerateSQL.CreateBinaryExpression(const ALeft,
  ARight: TSQLExpression): TSQLBinaryExpression;
begin
  Result:=TSQLBinaryExpression.Create(Nil);
  Result.Left:=ALeft;
  Result.Right:=ARight;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateUpdatePair(const AField: TSQLStringType;
  AValue: TSQLExpression): TSQLUpdatePair;
begin
  Result:=TSQLUpdatePair.Create(Nil);
  Result.FieldName:=CreateIdentifier(AFIeld);
  Result.Value:=AValue;
  FTofree:=Result;
end;

function TTestGenerateSQL.CreateTernaryExpression(const ALeft, AMiddle,
  ARight: TSQLExpression): TSQLTernaryExpression;
begin
  Result:=TSQLTernaryExpression.Create(Nil);
  Result.Left:=ALeft;
  Result.Middle:=AMiddle;
  Result.Right:=ARight;
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateForeignKey(const ATable: TSQLStringType;
  AFieldName1, AFieldName2: TSQLStringType): TSQLForeignKeyDefinition;
begin
  Result:=TSQLForeignKeyDefinition.Create(Nil);
  Result.TableName:=CreateIdentifier(ATable);
  Result.FieldList.Add(CreateIdentifier(AFieldName1));
  If (AFieldName2<>'') then
    Result.FieldList.Add(CreateIdentifier(AFieldName2));
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateSelectField(const AValue: TSQLExpression;
  const AAlias: TSQLStringType): TSQLSelectField;
begin
  Result:=TSQLSelectField.Create(Nil);
  Result.Expression:=AValue;
  If (AAlias<>'') then
    Result.AliasName:=CreateIdentifier(AAlias);
  FToFree:=Result;
end;

function TTestGenerateSQL.CreateSimpleTableReference(
  const ATableName: TSQLStringType; AAlias: TSQLStringType
  ): TSQLSimpleTableReference;
begin
  Result:=TSQLSimpleTableReference.Create(Nil);
  Result.ObjectName:=CreateIdentifier(ATableName);
  If AALias<>'' then
    Result.AliasName:=CreateIdentifier(AAlias);
  FTofree:=Result;
end;

function TTestGenerateSQL.CreateJoinTablereference(const ALeft,
  ARight: TSQLTableReference): TSQLJoinTableReference;
begin
  Result:=TSQLJoinTableReference.Create(Nil);
  Result.Left:=ALeft;
  Result.Right:=ARight;
  FTofree:=Result;
end;

function TTestGenerateSQL.CreateSelect(AField: TSQLExpression;
  ATable: TSQLStringType): TSQLSelectStatement;
begin
  Result:=TSQLSelectStatement.Create(Nil);
  Result.Fields.Add(AField);
  Result.Tables.Add(CreateSimpleTableReference(ATable,''));
  FToFree:=Result;
end;

procedure TTestGenerateSQL.TestIdentifier;

Var
  E : TSQLElement;
begin
  E:=CreateIdentifier('A');
  AssertSQL(E,'A',[]);
  AssertSQL(E,'"A"',[sfoDoubleQuoteIdentifier]);
  AssertSQL(E,'`A`',[sfoBackQuoteIdentifier]);
  AssertSQL(E,'''A''',[sfoSingleQuoteIdentifier]);
end;

procedure TTestGenerateSQL.TestIntegerLiteral;
begin
  AssertSQL(CreateLiteral(1),'1',[]);
end;

procedure TTestGenerateSQL.TestStringLiteral;

Var
  E : TSQLElement;

begin
  E:=CreateLiteral('A');
  AssertSQL(E,'"A"',[sfoDoubleQuotes]);
  AssertSQL(E,'''A''',[]);
end;

procedure TTestGenerateSQL.TestFloatLiteral;
begin
  // Needs improvement.
  AssertSQL(CreateLiteral(1.2),FloatToStr(1.2));
end;

procedure TTestGenerateSQL.TestNullLiteral;
begin
  FToFree:=TSQLNullLiteral.Create(Nil);
  AssertSQL(FToFree,'NULL');
  AssertSQL(FToFree,'null',[sfoLowerCaseKeyword]);
end;

procedure TTestGenerateSQL.TestUserLiteral;
begin
  FToFree:=TSQLUserLiteral.Create(Nil);
  AssertSQL(FToFree,'USER');
  AssertSQL(FToFree,'user',[sfoLowerCaseKeyword]);
end;

procedure TTestGenerateSQL.TestValueLiteral;
begin
  FToFree:=TSQLValueLiteral.Create(Nil);
  AssertSQL(FToFree,'VALUE');
  AssertSQL(FToFree,'value',[sfoLowerCaseKeyword]);
end;

procedure TTestGenerateSQL.TestLiteralExpression;

Var
  E : TSQLStringLiteral;

begin
  E:=CreateLiteral('A');
  AssertSQL(CreateLiteralExpression(E),E.GetAsSQL([]),[]);
end;

procedure TTestGenerateSQL.TestSelectField;

Var
  F : TSQLSelectField;

begin
  F:=CreateSelectField(CreateIdentifierExpression('A'),'');
  AssertSQL(F,'A');
  F.AliasName:=CreateIdentifier('B');
  FTofree:=F;
  AssertSQL(F,'A AS B');
end;

procedure TTestGenerateSQL.TestSimpleTablereference;

Var
  T : TSQLSimpleTablereference;

begin
  T:=CreateSimpleTablereference('A','');
  AssertSQL(T,'A');
  T.AliasName:=CreateIdentifier('B');
  AssertSQL(T,'A B');
  T.Params:=TSQLElementList.Create(True);
  T.Params.Add(CreateIdentifierExpression('C'));
  FToFree:=T;
  AssertSQL(T,'A(C) B');
  T.Params.Add(CreateIdentifierExpression('D'));
  FToFree:=T;
  AssertSQL(T,'A(C , D) B');
end;

procedure TTestGenerateSQL.TestSimpleSelect;

Var
  S : TSQLSelectStatement;

begin
  S:=CreateSelect(CreateIdentifierExpression('A'),'B');
  AssertSQL(S,'SELECT A FROM B');
end;

procedure TTestGenerateSQL.TestAnyExpression;
Var
  A : TSQLAnyExpression;
begin
  A:=TSQLAnyExpression.Create(Nil);
  A.Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FTofree:=A;
  AssertSQL(A,'ANY (SELECT A FROM B)');
end;

procedure TTestGenerateSQL.TestSomeExpression;
Var
  A : TSQLSomeExpression;
begin
  A:=TSQLSomeExpression.Create(Nil);
  A.Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FTofree:=A;
  AssertSQL(A,'SOME (SELECT A FROM B)');
end;

procedure TTestGenerateSQL.TestExistsExpression;
Var
  A : TSQLExistsExpression;
begin
  A:=TSQLExistsExpression.Create(Nil);
  A.Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FTofree:=A;
  AssertSQL(A,'EXISTS (SELECT A FROM B)');
end;

procedure TTestGenerateSQL.TestAllExpression;
Var
  A : TSQLAllExpression;
begin
  A:=TSQLAllExpression.Create(Nil);
  A.Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FTofree:=A;
  AssertSQL(A,'ALL (SELECT A FROM B)');
end;

procedure TTestGenerateSQL.TestSingularExpression;
Var
  A : TSQLSingularExpression;
begin
  A:=TSQLSingularExpression.Create(Nil);
  A.Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FTofree:=A;
  AssertSQL(A,'SINGULAR (SELECT A FROM B)');
end;

procedure TTestGenerateSQL.TestUnaryExpression;

Var
  U : TSQLUnaryExpression;

begin
  U:=CreateUnaryExpression(CreateLiteralExpression(CreateLiteral(1)));
  U.Operation:=uoMinus;
  AssertSQL(U,'-1');
  U.Operation:=uoNot;
  AssertSQL(U,'NOT (1)');
end;

procedure TTestGenerateSQL.TestBinaryExpression;

//                         boIs, boIsNot, boLike, boContaining, boStarting);

Var
  B : TSQLBinaryExpression;

begin
  B:=CreateBinaryExpression(CreateLiteralExpression(CreateLiteral(1)),CreateLiteralExpression(CreateLiteral(2)));
  B.Operation:=boAnd;
  AssertSQL(B,'1 AND 2');
  AssertSQL(B,'1'+sLineBreak+'AND 2',[sfoOneLogicalPerLine]);
  B.Operation:=boOR;
  AssertSQL(B,'1 OR 2');
  AssertSQL(B,'1'+sLineBreak+'OR 2',[sfoOneLogicalPerLine]);
  B.Operation:=boEQ;
  AssertSQL(B,'1 = 2');
  B.Operation:=boLT;
  AssertSQL(B,'1 < 2');
  B.Operation:=boGT;
  AssertSQL(B,'1 > 2');
  B.Operation:=boLE;
  AssertSQL(B,'1 <= 2');
  B.Operation:=boGE;
  AssertSQL(B,'1 >= 2');
  B.Operation:=boNE;
  AssertSQL(B,'1 <> 2');
  B.Operation:=boAdd;
  AssertSQL(B,'1 + 2');
  B.Operation:=boSubtract;
  AssertSQL(B,'1 - 2');
  B.Operation:=boMultiply;
  AssertSQL(B,'1 * 2');
  B.Operation:=boDivide;
  AssertSQL(B,'1 / 2');
  B.Right.Free;
  B.Right:=TSQLSelectExpression.Create(Nil);
  TSQLSelectExpression(B.Right).Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FTofree:=B;
  B.Operation:=boIn;
  AssertSQL(B,'1 IN (SELECT A FROM B)');
  B.Operation:=boIS;
  B.Right.Free;
  B.Right:=CreateLiteralExpression(TSQLNullLiteral.Create(Nil));
  FTofree:=B;
  AssertSQL(B,'1 IS NULL');
  B.Operation:=boISNot;
  AssertSQL(B,'1 IS NOT NULL');
  B.Right.Free;
  B.Right:=CreateLiteralExpression(CreateLiteral('%A%'));
  FTofree:=B;
  B.Left.Free;
  B.Left:=CreateLiteralExpression(CreateLiteral('ADAM'));
  FTofree:=B;
  B.Operation:=boLike;
  AssertSQL(B,'''ADAM'' LIKE ''%A%''');
  B.Operation:=boContaining;
  AssertSQL(B,'''ADAM'' CONTAINING ''%A%''');
  B.Operation:=boStarting;
  AssertSQL(B,'''ADAM'' STARTING WITH ''%A%''');
  B.Left.Free;
  B.Left:=CreateBinaryExpression(CreateLiteralExpression(CreateLiteral(1)),CreateLiteralExpression(CreateLiteral(2)));
  TSQLBinaryExpression(B.Left).Operation:=boAdd;
  FTofree:=B;
  B.Right.Free;
  B.Right:=CreateBinaryExpression(CreateLiteralExpression(CreateLiteral(3)),CreateLiteralExpression(CreateLiteral(4)));
  FTofree:=B;
  TSQLBinaryExpression(B.Right).Operation:=boAdd;
  B.Operation:=boLT;
  AssertSQL(B,'(1 + 2) < (3 + 4)');
end;

procedure TTestGenerateSQL.TestListExpression;

Var
  L:TSQLListExpression;

begin
  L:=TSQLListExpression.Create(Nil);
  FToFree:=L;
  AssertSQL(L,'()');
  L.List.Add(CreateLiteralExpression(CreateLiteral(1)));
  FToFree:=L;
  AssertSQL(L,'(1)');
  L.List.Add(CreateLiteralExpression(CreateLiteral(2)));
  FToFree:=L;
  AssertSQL(L,'(1 , 2)');
  AssertSQL(L,'(1, 2)',[sfoListNoSpaceBeforeComma]);
  AssertSQL(L,'(1 ,2)',[sfoListNoSpaceAfterComma]);
  AssertSQL(L,'(1,2)',[sfoListNoSpaceBeforeComma,sfoListNoSpaceAfterComma]);
end;

procedure TTestGenerateSQL.TestTernaryExpression;

Var
  T : TSQLTernaryExpression;

begin
  T:=CreateTernaryExpression(CreateLiteralExpression(CreateLiteral(2)),
                             CreateLiteralExpression(CreateLiteral(1)),
                             CreateLiteralExpression(CreateLiteral(3)));
  T.Operation:=toBetween;
  AssertSQL(T,'2 BETWEEN 1 AND 3');
  FreeAndNil(FToFree);
  T:=CreateTernaryExpression(CreateLiteralExpression(CreateLiteral('A')),
                             CreateLiteralExpression(CreateLiteral('B')),
                             CreateLiteralExpression(CreateLiteral('C')));
  T.Operation:=toLikeEscape;
  AssertSQL(T,'''A'' LIKE ''B'' ESCAPE ''C''');
end;

procedure TTestGenerateSQL.TestGenIDExpression;

Var
  G : TSQLGenIDExpression;

begin
  G:=TSQLGenIDExpression.Create(Nil);
  G.Generator:=self.CreateIdentifier('A');
  G.Value:=CreateLiteralExpression(CreateLiteral(1));
  FTofree:=G;
  AssertSQL(G,'GEN_ID(A,1)');
end;

procedure TTestGenerateSQL.TestFunctionCall;

Var
  F : TSQLFunctionCallExpression;
begin
  F:=TSQLFunctionCallExpression.Create(Nil);
  F.Identifier:='FUNC';
  FTofree:=F;
  AssertSQL(F,'FUNC()');
  F.Arguments:=TSQLElementList.Create(True);
  F.Arguments.Add(CreateLiteralExpression(CreateLiteral(1)));
  FTofree:=F;
  AssertSQL(F,'FUNC(1)');
  F.Arguments.Add(CreateLiteralExpression(CreateLiteral(2)));
  FTofree:=F;
  AssertSQL(F,'FUNC(1 , 2)');
  AssertSQL(F,'func(1,2)',[sfoLowerCaseKeyWord,sfoListNoSpaceBeforeComma,sfoListNoSpaceAfterComma]);
end;

procedure TTestGenerateSQL.TestAggregateFunction;

Var
  A : TSQLAggregateFunctionExpression;


begin
  A:=TSQLAggregateFunctionExpression.Create(Nil);
  FToFree:=A;
  A.Aggregate:=afCount;
  A.Option:=aoAsterisk;
  AssertSQL(A,'COUNT(*)');
  AssertSQL(A,'count(*)',[sfoLowercaseKeyword]);
  A.Option:=aoNone;
  A.Expression:=CreateIdentifierExpression('A');
  FTofree:=A;
  AssertSQL(A,'COUNT(A)');
  A.Option:=aoDistinct;
  AssertSQL(A,'COUNT(DISTINCT A)');
  A.Option:=aoAll;
  AssertSQL(A,'COUNT(ALL A)');
  A.Aggregate:=afMax;
  A.Option:=aoNone;
  AssertSQL(A,'MAX(A)');
  A.Aggregate:=afMin;
  AssertSQL(A,'MIN(A)');
  A.Aggregate:=afAVG;
  AssertSQL(A,'AVG(A)');
  A.Aggregate:=afSUM;
  AssertSQL(A,'SUM(A)');
end;

procedure TTestGenerateSQL.TestForeignKey;
Var
  K : TSQLForeignKeyDefinition;
begin
  K:=CreateForeignKey('A','B','');
  FTofree:=K;
  AssertSQL(K,'A (B)') ;
  K.FieldList.Add(CreateIdentifier('C'));
  FTofree:=K;
  AssertSQL(K,'A (B , C)');
  AssertSQL(K,'A (B,C)',[sfoListNoSpaceBeforeComma,sfoListNoSpaceAfterComma]);
  K.OnDelete:=fkaNoAction;
  AssertSQL(K,'A (B , C) ON DELETE NO ACTION');
  K.OnDelete:=fkaCascade;
  AssertSQL(K,'A (B , C) ON DELETE CASCADE');
  K.OnDelete:=fkaSetDefault;
  AssertSQL(K,'A (B , C) ON DELETE SET DEFAULT');
  K.OnDelete:=fkaSetNull;
  AssertSQL(K,'A (B , C) ON DELETE SET NULL');
  K.OnUpdate:=fkaNoaction;
  AssertSQL(K,'A (B , C) ON UPDATE NO ACTION ON DELETE SET NULL');
  K.OnUpdate:=fkaCascade;
  AssertSQL(K,'A (B , C) ON UPDATE CASCADE ON DELETE SET NULL');
  K.OnUpdate:=fkaSetDefault;
  AssertSQL(K,'A (B , C) ON UPDATE SET DEFAULT ON DELETE SET NULL');
  K.OnUpdate:=fkaSetNull;
  AssertSQL(K,'A (B , C) ON UPDATE SET NULL ON DELETE SET NULL');
  K.OnDelete:=fkaNone;
  AssertSQL(K,'A (B , C) ON UPDATE SET NULL');
end;

procedure TTestGenerateSQL.TestUniqueFieldConstraint;

Var
  F : TSQLUniqueFieldConstraint;

begin
  F:=TSQLUniqueFieldConstraint.Create(Nil);
  FTofree:=F;
  AssertSQL(F,'UNIQUE');
  F.ConstraintName:=CreateIdentifier('A');
  FTofree:=F;
  AssertSQL(F,'CONSTRAINT A UNIQUE');
  AssertSQL(F,'constraint A unique',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestPrimaryKeyFieldConstraint;

Var
  F : TSQLPrimaryKeyFieldConstraint;

begin
  F:=TSQLPrimaryKeyFieldConstraint.Create(Nil);
  FTofree:=F;
  AssertSQL(F,'PRIMARY KEY');
  F.ConstraintName:=CreateIdentifier('A');
  FTofree:=F;
  AssertSQL(F,'CONSTRAINT A PRIMARY KEY');
  AssertSQL(F,'constraint A primary key',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestForeignKeyFieldConstraint;

Var
  F : TSQLForeignKeyFieldConstraint;

begin
  F:=TSQLForeignKeyFieldConstraint.Create(Nil);
  F.Definition:=CreateForeignKey('B','C','');
  FTofree:=F;
  AssertSQL(F,'REFERENCES B (C)');
  F.ConstraintName:=CreateIdentifier('A');
  FTofree:=F;
  AssertSQL(F,'CONSTRAINT A REFERENCES B (C)');
  AssertSQL(F,'constraint A references B (C)',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestCheckFieldConstraint;

Var
  F : TSQLCheckFieldConstraint;

begin
  F:=TSQLCheckFieldConstraint.Create(Nil);
  F.Expression:=CreateBinaryExpression(CreateLiteralExpression(TSQLValueLiteral.Create(Nil)),CreateLiteralExpression(CreateLiteral(1)));
  TSQLBinaryExpression(F.Expression).Operation:=boGT;
  FTofree:=F;
  AssertSQL(F,'CHECK (VALUE > 1)');
  F.ConstraintName:=CreateIdentifier('A');
  FTofree:=F;
  AssertSQL(F,'CONSTRAINT A CHECK (VALUE > 1)');
  AssertSQL(F,'constraint A check (value > 1)',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestTableUniqueConstraintDef;

Var
  U : TSQLTableUniqueConstraintDef;

begin
  U:=TSQLTableUniqueConstraintDef.Create(Nil);
  U.FieldList.Add(CreateIdentifier('A'));
  FToFree:=U;
  AssertSQL(U,'UNIQUE (A)');
  U.ConstraintName:=CreateIdentifier('C');
  FToFree:=U;
  AssertSQL(U,'CONSTRAINT C UNIQUE (A)');
  U.FieldList.Add(CreateIdentifier('B'));
  FToFree:=U;
  AssertSQL(U,'CONSTRAINT C UNIQUE (A , B)');
  AssertSQL(U,'constraint C unique (A , B)',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestTableprimaryKeyConstraintDef;

Var
  U : TSQLTablePrimaryKeyConstraintDef;

begin
  U:=TSQLTablePrimaryKeyConstraintDef.Create(Nil);
  U.FieldList.Add(CreateIdentifier('A'));
  FToFree:=U;
  AssertSQL(U,'PRIMARY KEY (A)');
  U.ConstraintName:=CreateIdentifier('C');
  FToFree:=U;
  AssertSQL(U,'CONSTRAINT C PRIMARY KEY (A)');
  U.FieldList.Add(CreateIdentifier('B'));
  FToFree:=U;
  AssertSQL(U,'CONSTRAINT C PRIMARY KEY (A , B)');
  AssertSQL(U,'constraint C primary key (A , B)',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestTableForeignKeyConstraintDef;

Var
  U : TSQLTableForeignKeyConstraintDef;

begin
  U:=TSQLTableForeignKeyConstraintDef.Create(Nil);
  U.FieldList.Add(CreateIdentifier('A'));
  U.Definition:=CreateForeignKey('D','E','');
  FToFree:=U;
  AssertSQL(U,'FOREIGN KEY (A) REFERENCES D (E)');
  U.ConstraintName:=CreateIdentifier('C');
  FToFree:=U;
  AssertSQL(U,'CONSTRAINT C FOREIGN KEY (A) REFERENCES D (E)');
  U.FieldList.Add(CreateIdentifier('B'));
  U.Definition.FieldList.Add(CreateIdentifier('F'));
  FToFree:=U;
  AssertSQL(U,'CONSTRAINT C FOREIGN KEY (A , B) REFERENCES D (E , F)');
  AssertSQL(U,'constraint C foreign key (A,B) references D (E,F)',[sfoLowercaseKeyWord,sfoListNoSpaceBeforeComma,sfoListNoSpaceAfterComma]);
end;

procedure TTestGenerateSQL.TestTableCheckConstraintDef;

Var
  F : TSQLTableCheckConstraintDef;

begin
  F:=TSQLTableCheckConstraintDef.Create(Nil);
  F.Check:=CreateBinaryExpression(CreateIdentifierExpression('A'),CreateLiteralExpression(CreateLiteral(1)));
  TSQLBinaryExpression(F.Check).Operation:=boGT;
  FTofree:=F;
  AssertSQL(F,'CHECK (A > 1)');
  F.ConstraintName:=CreateIdentifier('C');
  FTofree:=F;
  AssertSQL(F,'CONSTRAINT C CHECK (A > 1)');
  AssertSQL(F,'constraint C check (A > 1)',[sfoLowercaseKeyWord]);
end;

procedure TTestGenerateSQL.TestTypeDefinition;

Var
  T : TSQLTypeDefinition;

begin
  T:=CreateTypeDefinition(sdtInteger,0);
  AssertSQl(T,'INT');
  T.DataType:=sdtSmallInt;
  AssertSQl(T,'SMALLINT');
  T.DataType:=sdtDoublePrecision;
  AssertSQl(T,'DOUBLE PRECISION');
  T.DataType:=sdtFloat;
  AssertSQl(T,'FLOAT');
  T.DataType:=sdtDate;
  AssertSQl(T,'DATE');
  T.DataType:=sdtDateTime;
  AssertSQl(T,'TIMESTAMP');
  T.DataType:=sdtTime;
  AssertSQl(T,'TIME');
  T.DataType:=sdtChar;
  T.Len:=5;
  AssertSQl(T,'CHAR(5)');
  T.DataType:=sdtVarChar;
  AssertSQl(T,'VARCHAR(5)');
  T.DataType:=sdtNChar;
  AssertSQl(T,'NATIONAL CHARACTER(5)');
  T.DataType:=sdtNVarChar;
  AssertSQl(T,'NATIONAL CHARACTER VARYING(5)');
  T.DataType:=sdtDecimal;
  T.Scale:=2;
  AssertSQl(T,'DECIMAL(5,2)');
  T.Scale:=0;
  AssertSQl(T,'DECIMAL(5)');
  T.DataType:=sdtNumeric;
  T.Scale:=2;
  AssertSQl(T,'NUMERIC(5,2)');
  T.Scale:=0;
  AssertSQl(T,'NUMERIC(5)');
  T.DataType:=sdtBlob;
  AssertSQL(T,'BLOB SUB_TYPE 0 SEGMENT_SIZE 5');
  T.Len:=0;
  AssertSQL(T,'BLOB SUB_TYPE 0');
  T.DataType:=sdtInteger;
  T.NotNull:=True;
  AssertSQL(T,'INT NOT NULL');
  T.DefaultValue:=CreateLiteral(1);
  AssertSQL(T,'INT DEFAULT 1 NOT NULL');
  T.Check:=CreateBinaryExpression(CreateLiteralExpression(TSQLValueLiteral.Create(Nil)),CreateLiteralExpression(CreateLiteral(1)));
  FToFree:=T;
  TSQLBinaryExpression(T.Check).Operation:=boGE;
  AssertSQL(T,'INT DEFAULT 1 NOT NULL CHECK (VALUE >= 1)');
  T.DefaultValue.Free;
  T.DefaultValue:=Nil;
  AssertSQL(T,'INT NOT NULL CHECK (VALUE >= 1)');
  T.Constraint:=TSQLCheckFieldConstraint.Create(Nil);
  TSQLCheckFieldConstraint(T.Constraint).Expression:=T.check;
  T.Check:=Nil;
  AssertSQL(T,'INT NOT NULL CHECK (VALUE >= 1)');
  TSQLCheckFieldConstraint(T.Constraint).ConstraintName:=CreateIdentifier('C');
  FToFree:=T;
  AssertSQL(T,'INT NOT NULL CONSTRAINT C CHECK (VALUE >= 1)');
  T.Constraint.Free;
  T.Constraint:=Nil;
  T.NotNull:=False;
  T.DataType:=sdtChar;
  T.Len:=50;
  T.CharSet:='UTF-8';
  AssertSQL(T,'CHAR(50) CHARACTER SET UTF-8');
  T.Collation:=TSQLCollation.Create(Nil);
  T.Collation.Name:='MyCol';
  AssertSQL(T,'CHAR(50) CHARACTER SET UTF-8 COLLATION MYCOL');
end;

procedure TTestGenerateSQL.TestCastExpression;

Var
  C : TSQLCastExpression;

begin
  C:=TSQLCastExpression.Create(Nil);
  FToFree:=C;
  C.Value:=CreateLiteralExpression(CreateLiteral(1));
  C.NewType:=CreateTypeDefinition(sdtNumeric,5);
  C.NewType.Scale:=2;
  FToFree:=C;
  AssertSQL(C,'CAST(1 AS NUMERIC(5,2))');

end;

procedure TTestGenerateSQL.TestJoinTableReference;

var
  J,J2 : TSQLJoinTableReference;

begin
  J:=CreateJoinTableReference(CreateSimpleTableReference('A',''),CreateSimpleTableReference('B',''));
  J.JoinClause:=CreateBinaryExpression(CreateIdentifierExpression('C'),CreateIdentifierExpression('D'));
  TSQLBinaryExpression(J.JoinClause).Operation:=boEQ;
  J.JoinType:=jtInner;
  FToFree:=J;
  AssertSQL(J,'A INNER JOIN B ON (C = D)');
  J.JoinType:=jtLeft;
  AssertSQL(J,'A LEFT JOIN B ON (C = D)');
  J.JoinType:=jtRight;
  AssertSQL(J,'A RIGHT JOIN B ON (C = D)');
  J.JoinType:=jtFullOuter;
  AssertSQL(J,'A FULL OUTER JOIN B ON (C = D)');
  AssertSQL(J,'A'+sLinebreak+'FULL OUTER JOIN B ON (C = D)',[sfoOneTablePerLine]);
  AssertSQL(J,'A'+sLinebreak+'  FULL OUTER JOIN B ON (C = D)',[sfoOneTablePerLine,sfoIndentTables]);
  J2:=CreateJoinTableReference(CreateSimpleTableReference('E',''),CreateSimpleTableReference('F',''));
  J2.JoinClause:=CreateBinaryExpression(CreateIdentifierExpression('G'),CreateIdentifierExpression('H'));
  TSQLBinaryExpression(J2.JoinClause).Operation:=boEQ;
  J.Right.Free;
  J.Right:=J2;
  FTofree:=J;
  AssertSQL(J,'A FULL OUTER JOIN (E JOIN F ON (G = H)) ON (C = D)');
  AssertSQL(J,'A FULL OUTER JOIN E JOIN F ON (G = H) ON (C = D)',[sfoNoBracketRightJoin]);
  J.Right:=J.Left;
  J.Left:=J2;
  AssertSQL(J,'(E JOIN F ON (G = H)) FULL OUTER JOIN A ON (C = D)',[sfoBracketLeftJoin]);
end;

procedure TTestGenerateSQL.TestPlanNatural;

Var
  N : TSQLSelectNaturalPlan;

begin
  N:=TSQLSelectNaturalPlan.Create(Nil);
  N.TableName:=CreateIdentifier('A');
  FToFree:=N;
  AssertSQL(N,'A NATURAL');
  AssertSQL(N,'A natural',[sfoLowercaseKeyword]);
end;

procedure TTestGenerateSQL.TestPlanIndex;

Var
  I : TSQLSelectIndexedPlan;

begin
  I:=TSQLSelectIndexedPlan.Create(Nil);
  I.TableName:=CreateIdentifier('A');
  I.Indexes.Add(CreateIdentifier('B'));
  FToFree:=I;
  AssertSQL(I,'A INDEX (B)');
  AssertSQL(I,'A index (B)',[sfoLowercaseKeyword]);
  I.Indexes.Add(CreateIdentifier('C'));
  FToFree:=I;
  AssertSQL(I,'A INDEX (B,C)',[sfoListNoSpacebeforeComma,sfoListNoSpaceAfterComma]);
end;

procedure TTestGenerateSQL.TestPlanOrder;
Var
  I : TSQLSelectOrderedPlan;

begin
  I:=TSQLSelectOrderedPlan.Create(Nil);
  I.TableName:=CreateIdentifier('A');
  I.OrderIndex:=CreateIdentifier('B');
  FToFree:=I;
  AssertSQL(I,'A ORDER B');
  AssertSQL(I,'A order B',[sfoLowercaseKeyword]);
end;

procedure TTestGenerateSQL.TestPlanExpression;

Var
  P : TSQLSelectPlanExpr;
  N : TSQLSelectNaturalPlan;
  I : TSQLSelectIndexedPlan;

begin
  P:=TSQLSelectPlanExpr.Create(Nil);
  P.JoinType:=pjtJoin;
  N:=TSQLSelectNaturalPlan.Create(Nil);
  N.TableName:=CreateIdentifier('STATE');
  P.Items.Add(N);
  I:=TSQLSelectIndexedPlan.Create(Nil);
  I.TableName:=CreateIdentifier('CITIES');
  I.Indexes.Add(CreateIdentifier('DUPE_CITY'));
  P.Items.Add(I);
  I:=TSQLSelectIndexedPlan.Create(Nil);
  I.TableName:=CreateIdentifier('MAYORS');
  I.Indexes.Add(CreateIdentifier('MAYORS_1'));
  P.Items.Add(I);
  FToFree:=P;
  AssertSQL(P,'JOIN (STATE NATURAL , CITIES INDEX (DUPE_CITY) , MAYORS INDEX (MAYORS_1))');
end;

procedure TTestGenerateSQL.TestOrderBy;

Var
  O : TSQLOrderByElement;

begin
  O:=TSQLOrderByElement.Create(Nil);
  FToFree:=O;
  O.Field:=CreateLiteral(1);
  AssertSQL(O,'1');
  AssertSQL(O,'1 ASC',[sfoForceAscending]);
  AssertSQL(O,'1 asc',[sfoForceAscending,sfoLowerCaseKeyword]);
  O.OrderBy:=obDescending;
  O.Field.free;
  O.Field:=CreateIdentifier('A');
  AssertSQL(O,'A desc',[sfoForceAscending,sfoLowerCaseKeyword]);
  AssertSQL(O,'A DESC',[sfoForceAscending]);
  AssertSQL(O,'A DESC');
  O.Collation:=CreateIdentifier('UTF8');
  AssertSQL(O,'A DESC UTF8');
end;

procedure TTestGenerateSQL.TestSelect;

Var
  S : TSQLSelectStatement;

begin
  S:=CreateSelect(CreateIdentifierExpression('A'),'B');
  AssertSQL(S,'SELECT A FROM B');
  AssertSQL(S,'SELECT'+sLineBreak+'A'+sLineBreak+'FROM B',[sfoOneFieldPerLine]);
  AssertSQL(S,'SELECT'+sLineBreak+'  A'+sLineBreak+'FROM B',[sfoOneFieldPerLine,sfoIndentFields]);
  AssertSQL(S,'SELECT A FROM'+SlineBreak+'B',[sfoOneTablePerLine]);
  AssertSQL(S,'SELECT A FROM'+SlineBreak+'  B',[sfoOneTablePerLine,sfoIndentTables]);
  AssertSQL(S,'SELECT'+sLineBreak+'A'+sLineBreak+'FROM'+sLineBreak+'B',[sfoOneFieldPerLine,sfoOneTablePerLine]);
  S.Where:=CreateBinaryExpression(CreateIdentifierExpression('C'),CreateIdentifierExpression('D'));
  TSQLBinaryExpression(S.Where).Operation:=boEQ;
  AssertSQL(S,'SELECT A FROM B WHERE C = D');
  AssertSQL(S,'SELECT A FROM B WHERE'+sLineBreak+'C = D',[sfoWhereOnSeparateLine]);
  AssertSQL(S,'SELECT A FROM B WHERE'+sLineBreak+'  C = D',[sfoWhereOnSeparateLine,sfoIndentwhere]);
  AssertSQL(S,'SELECT A FROM'+sLineBreak+'B'+sLineBreak+'WHERE'+sLineBreak+'  C = D',[sfoOneTablePerLine,sfoWhereOnSeparateLine,sfoIndentwhere]);
  AssertSQL(S,'SELECT A FROM'+sLineBreak+'B'+sLineBreak+'WHERE'+sLineBreak+'  C = D',[sfoOneTablePerLine,sfoWhereOnSeparateLine,sfoIndentwhere]);
  S.GroupBy.Add(CreateIdentifier('A'));
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A');
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY'+sLineBreak+'A',[sfoOneGroupbyFieldPerLine]);
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY'+sLineBreak+'  A',[sfoOneGroupbyFieldPerLine,sfoIndentGroupByFields]);
  AssertSQL(S,'SELECT A FROM B WHERE'+sLineBreak+'C = D'+sLineBreak+'GROUP BY'+sLineBreak+'  A',[sfoWhereOnSeparateLine,sfoOneGroupbyFieldPerLine,sfoIndentGroupByFields]);
  S.OrderBy.Add(CreateIdentifier('E'));
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A ORDER BY E');
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A ORDER BY'+sLineBreak+'E',[sfoOneOrderbyFieldPerLine]);
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY'+sLineBreak+'A'+sLineBreak+'ORDER BY'+sLineBreak+'  E',[sfoOneGroupbyFieldPerLine,sfoOneOrderbyFieldPerLine,sfoIndentOrderByFields]);
  S.Having:=CreateBinaryExpression(CreateIdentifierExpression('F'),CreateIdentifierExpression('G'));
  TSQLBinaryExpression(S.Having).Operation:=boGT;
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING F > G ORDER BY E');
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING'+sLineBreak+'F > G'+sLineBreak+'ORDER BY E',[sfoHavingOnSeparateLine]);
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING'+sLineBreak+'  F > G'+sLineBreak+'ORDER BY E',[sfoHavingOnSeparateLine,sfoIndentHaving]);
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY'+sLineBreak+'A'+sLineBreak+'HAVING'+sLineBreak+'F > G'+sLineBreak+'ORDER BY E',[sfoOneGroupbyFieldPerLine,sfoHavingOnSeparateLine]);
  S.Union:=CreateSelect(CreateIdentifierExpression('H'),'I');
  S.OrderBy.Delete(0);
  S.OrderBy.Add(CreateLiteral(1));
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING F > G UNION SELECT H FROM I ORDER BY 1');
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING F > G'+sLineBreak+'UNION'+sLineBreak+'SELECT H FROM I'+sLineBreak+'ORDER BY 1',[sfoUnionOnSeparateLine]);
  S.Union.Free;
  S.Union:=Nil;
  FToFree:=S;
  S.Plan:=TSQLSelectNaturalPlan.Create(Nil);
  TSQLSelectNaturalPlan(S.PLan).TableName:=CreateIdentifier('B');
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING F > G PLAN B NATURAL ORDER BY 1');
  AssertSQL(S,'SELECT A FROM B WHERE C = D GROUP BY A HAVING F > G'+sLineBreak+'PLAN'+sLineBreak+'B NATURAL'+sLineBreak+'ORDER BY 1',[sfoPlanOnSeparateLine]);
end;

procedure TTestGenerateSQL.TestInsert;

Var
  I : TSQLInsertStatement;

begin
  I:=TSQLInsertStatement.Create(Nil);
  I.TableName:=CreateIdentifier('A');
  FToFree:=I;
  I.Values:=TSQLelementList.Create(True);
  I.Values.Add(CreateLiteral('B'));
  AssertSQL(I,'INSERT INTO A VALUES (''B'')');
  I.Fields:=TSQLelementList.Create(True);
  I.Fields.Add(CreateIdentifier('C'));
  AssertSQL(I,'INSERT INTO A (C) VALUES (''B'')');
  AssertSQL(I,'INSERT INTO A'+sLineBreak+'(C)'+sLineBreak+'VALUES'+sLineBreak+'(''B'')',[sfoOneFieldPerLine]);
  I.Fields.Add(CreateIdentifier('D'));
  I.Values.Add(CreateLiteral('E'));
  AssertSQL(I,'INSERT INTO A (C, D) VALUES (''B'', ''E'')');
  AssertSQL(I,'INSERT INTO A'+sLineBreak+'(C,'+sLineBreak+'D)'+sLineBreak+'VALUES'+sLineBreak+'(''B'','+sLineBreak+'''E'')',[sfoOneFieldPerLine]);
  AssertSQL(I,'INSERT INTO A'+sLineBreak+'  (C,'+sLineBreak+'  D)'+sLineBreak+'VALUES'+sLineBreak+'  (''B'','+sLineBreak+'  ''E'')',[sfoOneFieldPerLine,sfoIndentFields]);
  I.Select:=CreateSelect(CreateIdentifierExpression('E'),'G');
  I.Select.Fields.Add(CreateIdentifierExpression('F'));
  AssertSQL(I,'INSERT INTO A (C, D) SELECT E, F FROM G');
end;

procedure TTestGenerateSQL.TestUpdatePair;

Var
  P : TSQLUpdatePair;

begin
  P:=CreateUpdatePair('A',CreateLiteralExpression(CreateLiteral(1)));
  AssertSQl(P,'A = 1');
end;

procedure TTestGenerateSQL.TestUpdate;

Var
  U : TSQLUpdateStatement;
  P : TSQLUpdatePair;

begin
  P:=CreateUpdatePair('A',CreateLiteralExpression(CreateLiteral(1)));
  U:=TSQLUpdateStatement.Create(Nil);
  U.TableName:=CreateIdentifier('B');
  U.Values.Add(P);
  FTofree:=U;
  AssertSQL(U,'UPDATE B SET A = 1');
  AssertSQL(U,'UPDATE B SET'+sLineBreak+'A = 1',[sfoOneFieldPerLine]);
  AssertSQL(U,'UPDATE B SET'+sLineBreak+'  A = 1',[sfoOneFieldPerLine,sfoIndentFields]);
  P:=CreateUpdatePair('C',CreateLiteralExpression(CreateLiteral(2)));
  U.Values.Add(P);
  FTofree:=U;
  AssertSQL(U,'UPDATE B SET A = 1, C = 2');
  AssertSQL(U,'UPDATE B SET'+sLineBreak+'A = 1,'+sLineBreak+'C = 2',[sfoOneFieldPerLine]);
  AssertSQL(U,'UPDATE B SET'+sLineBreak+'  A = 1,'+sLineBreak+'  C = 2',[sfoOneFieldPerLine,sfoIndentFields]);
  U.WhereClause:=CreateBinaryExpression(CreateIdentifierExpression('A'),CreateLiteralExpression(CreateLiteral(1)));
  TSQLBinaryExpression(U.WhereClause).Operation:=boGT;
  AssertSQL(U,'UPDATE B SET A = 1, C = 2 WHERE A > 1');
  AssertSQL(U,'UPDATE B SET'+sLineBreak+'  A = 1,'+sLineBreak+'  C = 2'+sLineBreak+'WHERE A > 1',[sfoOneFieldPerLine,sfoIndentFields]);
  AssertSQL(U,'UPDATE B SET A = 1, C = 2'+sLineBreak+'WHERE'+sLineBreak+'A > 1',[sfoWhereOnSeparateLine]);
  AssertSQL(U,'UPDATE B SET A = 1, C = 2'+sLineBreak+'WHERE'+sLineBreak+'  A > 1',[sfoWhereOnSeparateLine,sfoIndentWhere]);
end;

procedure TTestGenerateSQL.TestDelete;

Var
  D : TSQLDeleteStatement;

begin
  D:=TSQLDeleteStatement.Create(Nil);
  D.TableName:=CreateIdentifier('A');
  FToFree:=D;
  AssertSQL(D,'DELETE FROM A');
  D.WhereClause:=CreateBinaryExpression(CreateIdentifierExpression('B'),CreateLiteralExpression(CreateLiteral(1)));
  TSQLBinaryExpression(D.WhereClause).Operation:=boGT;
  FToFree:=D;
  AssertSQL(D,'DELETE FROM A WHERE B > 1');
  AssertSQL(D,'DELETE FROM A'+sLineBreak+'WHERE'+sLineBreak+'B > 1',[sfoWhereOnSeparateLine]);
  AssertSQL(D,'DELETE FROM A'+sLineBreak+'WHERE'+sLineBreak+'  B > 1',[sfoWhereOnSeparateLine,sfoIndentWhere]);
end;

procedure TTestGenerateSQL.TestRollback;

Var
  R : TSQLRollBackStatement;

begin
  R:=TSQLRollBackStatement.Create(Nil);
  FTofree:=R;
  AssertSQL(R,'ROLLBACK');
  R.TransactionName:=CreateIdentifier('A');
  FTofree:=R;
  AssertSQL(R,'ROLLBACK TRANSACTION A');
  R.Work:=True;
  AssertSQL(R,'ROLLBACK TRANSACTION A WORK');
  R.Release:=True;
  AssertSQL(R,'ROLLBACK TRANSACTION A WORK RELEASE');
end;

procedure TTestGenerateSQL.TestCommit;
Var
  C : TSQLCommitStatement;

begin
  C:=TSQLCommitStatement.Create(Nil);
  FTofree:=C;
  AssertSQL(C,'COMMIT');
  C.TransactionName:=CreateIdentifier('A');
  FTofree:=C;
  AssertSQL(C,'COMMIT TRANSACTION A');
  C.Work:=True;
  AssertSQL(C,'COMMIT TRANSACTION A WORK');
  C.Release:=True;
  AssertSQL(C,'COMMIT TRANSACTION A WORK RELEASE');
  C.Release:=False;
  C.Retain:=True;
  AssertSQL(C,'COMMIT TRANSACTION A WORK RETAIN');
end;

procedure TTestGenerateSQL.TestExecuteProcedure;

Var
  E : TSQLExecuteProcedureStatement;

begin
  E:=TSQLExecuteProcedureStatement.Create(Nil);
  E.ProcedureName:=CreateIdentifier('A');
  FTofree:=E;
  AssertSQL(E,'EXECUTE PROCEDURE A');
  E.Params.Add(CreateLiteralExpression(CreateLiteral(1)));
  FTofree:=E;
  AssertSQL(E,'EXECUTE PROCEDURE A(1)');
  E.Params.Add(CreateLiteralExpression(CreateLiteral(2)));
  FTofree:=E;
  AssertSQL(E,'EXECUTE PROCEDURE A(1 , 2)');
  E.Returning.Add(CreateIdentifier('B'));
  FTofree:=E;
  AssertSQL(E,'EXECUTE PROCEDURE A(1 , 2) RETURNING_VALUES :B');
end;

procedure TTestGenerateSQL.TestCreateGenerator;
Var
  G : TSQLCreateGeneratorStatement;
begin
  G:=TSQLCreateGeneratorStatement.Create(Nil);
  G.ObjectName:=CreateIdentifier('A');
  FToFree:=G;
  AssertSQL(G,'CREATE GENERATOR A');
end;

procedure TTestGenerateSQL.TestCreateRole;
Var
  R : TSQLCreateRoleStatement;
begin
  R:=TSQLCreateRoleStatement.Create(Nil);
  R.ObjectName:=CreateIdentifier('A');
  FToFree:=R;
  AssertSQL(R,'CREATE ROLE A');
end;

procedure TTestGenerateSQL.TestCreateDomain;

Var
  D : TSQLCreateDomainStatement;

begin
  D:=TSQLCreateDomainStatement.Create(Nil);
  D.ObjectName:=CreateIdentifier('A');
  D.TypeDefinition:=CreateTypeDefinition(sdtInteger,0);
  D.TypeDefinition.NotNull:=True;
  FToFree:=D;
  AssertSQL(D,'CREATE DOMAIN A INT NOT NULL');
end;

procedure TTestGenerateSQL.TestAlterDomainDropDefault;

Var
  A : TSQLAlterDomainDropDefaultStatement;

begin
  A:=TSQLAlterDomainDropDefaultStatement.Create(Nil);
  A.Objectname:=CreateIdentifier('A');
  FToFree:=A;
  AssertSQL(A,'ALTER DOMAIN A DROP DEFAULT');
end;

procedure TTestGenerateSQL.TestAlterDomainDropCheck;

Var
  A : TSQLAlterDomainDropCheckStatement;

begin
  A:=TSQLAlterDomainDropCheckStatement.Create(Nil);
  A.Objectname:=CreateIdentifier('A');
  FToFree:=A;
  AssertSQL(A,'ALTER DOMAIN A DROP CHECK');
end;

procedure TTestGenerateSQL.TestAlterDomainSetDefault;

Var
  A : TSQLAlterDomainSetDefaultStatement;

begin
  A:=TSQLAlterDomainSetDefaultStatement.Create(Nil);
  A.Objectname:=CreateIdentifier('A');
  A.DefaultValue:=TSQLNullLiteral.Create(Nil);
  FToFree:=A;
  AssertSQL(A,'ALTER DOMAIN A SET DEFAULT NULL');
end;

procedure TTestGenerateSQL.TestAlterDomainRename;

Var
  A : TSQLAlterDomainRenameStatement;

begin
  A:=TSQLAlterDomainRenameStatement.Create(Nil);
  A.Objectname:=CreateIdentifier('A');
  A.NewName:=CreateIdentifier('B');
  FToFree:=A;
  AssertSQL(A,'ALTER DOMAIN A B');
end;

procedure TTestGenerateSQL.TestAlterDomainNewType;

Var
  A : TSQLAlterDomainTypeStatement;

begin
  A:=TSQLAlterDomainTypeStatement.Create(Nil);
  A.Objectname:=CreateIdentifier('A');
  A.NewType:=CreateTypeDefinition(sdtVarChar,5);
  FToFree:=A;
  AssertSQL(A,'ALTER DOMAIN A TYPE VARCHAR(5)');
end;

procedure TTestGenerateSQL.TestAlterDomainAddCheck;

Var
  A : TSQLAlterDomainAddCheckStatement;

begin
  A:=TSQLAlterDomainAddCheckStatement.Create(Nil);
  A.Objectname:=CreateIdentifier('A');
  A.Check:=CreateBinaryExpression(CreateLiteralExpression(TSQLValueLiteral.Create(Nil)),CreateLiteralExpression(CreateLiteral(1)));
  TSQLBinaryExpression(A.Check).Operation:=boGT;
  FToFree:=A;
  AssertSQL(A,'ALTER DOMAIN A ADD CHECK VALUE > 1');
end;

procedure TTestGenerateSQL.TestCreateException;

Var
  C : TSQLCreateExceptionStatement;

begin
  C:=TSQLCreateExceptionStatement.Create(Nil);
  C.ObjectName:=CreateIdentifier('A');
  C.ExceptionMessage:=CreateLiteral('B');
  FToFree:=C;
  AssertSQL(C,'CREATE EXCEPTION A ''B''');
end;

procedure TTestGenerateSQL.TestAlterException;
Var
  C : TSQLAlterExceptionStatement;

begin
  C:=TSQLAlterExceptionStatement.Create(Nil);
  C.ObjectName:=CreateIdentifier('A');
  C.ExceptionMessage:=CreateLiteral('B');
  FToFree:=C;
  AssertSQL(C,'ALTER EXCEPTION A ''B''');
end;

procedure TTestGenerateSQL.TestCreateIndex;

Var
  I : TSQLCreateIndexStatement;

begin
  I:=TSQLCreateIndexStatement.Create(Nil);
  I.ObjectName:=CreateIdentifier('A');
  I.TableName:=CreateIdentifier('B');
  I.FieldNames.Add(CreateIdentifier('C'));
  FTofree:=I;
  AssertSQL(I,'CREATE INDEX A ON B (C)');
  I.FieldNames.Add(CreateIdentifier('D'));
  FTofree:=I;
  AssertSQL(I,'CREATE INDEX A ON B (C , D)');
  I.Options:=[ioUnique];
  AssertSQL(I,'CREATE UNIQUE INDEX A ON B (C , D)');
  I.Options:=[ioAscending];
  AssertSQL(I,'CREATE ASCENDING INDEX A ON B (C , D)');
  I.Options:=[ioUnique,ioDescending];
  AssertSQL(I,'CREATE UNIQUE DESCENDING INDEX A ON B (C , D)');
  I.Options:=[ioUnique,ioAscending,ioDescending];
  AssertSQL(I,'CREATE UNIQUE ASCENDING INDEX A ON B (C , D)');
  I.Options:=[ioDescending];
  AssertSQL(I,'CREATE DESCENDING INDEX A ON B (C , D)');
end;

procedure TTestGenerateSQL.TestAlterIndex;

Var
  I : TSQLAlterIndexStatement;

begin
  I:=TSQLAlterIndexStatement.Create(Nil);
  I.ObjectName:=CreateIdentifier('A');
  FTofree:=I;
  AssertSQL(I,'ALTER INDEX A ACTIVE');
  I.Inactive:=True;
  AssertSQL(I,'ALTER INDEX A INACTIVE');
end;

procedure TTestGenerateSQL.TestDeclareExternalFunction;

Var
  F : TSQLDeclareExternalFunctionStatement;

begin
  F:=TSQLDeclareExternalFunctionStatement.Create(Nil);
  F.ObjectName:=CreateIdentifier('A');
  F.ReturnType:=CreateTypeDefinition(sdtChar,5);
  F.EntryPoint:='B';
  F.ModuleName:='C';
  FToFree:=F;
  AssertSQL(F,'DECLARE EXTERNAL FUNCTION A RETURNS CHAR(5) ENTRY_POINT ''B'' MODULE_NAME ''C''');
  F.Arguments.Add(CreateTypeDefinition(sdtChar,10));
  AssertSQL(F,'DECLARE EXTERNAL FUNCTION A CHAR(10) RETURNS CHAR(5) ENTRY_POINT ''B'' MODULE_NAME ''C''');
  F.Arguments.Add(CreateTypeDefinition(sdtInteger,0));
  AssertSQL(F,'DECLARE EXTERNAL FUNCTION A CHAR(10) , INT RETURNS CHAR(5) ENTRY_POINT ''B'' MODULE_NAME ''C''');
  AssertSQL(F,'DECLARE EXTERNAL FUNCTION A'+sLineBreak+
              '  CHAR(10) , INT'+sLineBreak+
              '  RETURNS CHAR(5)'+sLineBreak+
              '  ENTRY_POINT ''B'''+sLineBreak+
              '  MODULE_NAME ''C''',[sfoMultilineDeclareFunction]);
end;

procedure TTestGenerateSQL.TestTableFieldDefinition;

Var
  F : TSQLTableFieldDef;

begin
  F:=TSQLTableFieldDef.Create(Nil);
  F.FieldName:=CreateIdentifier('A');
  F.FieldType:=CreateTypeDefinition(sdtInteger,0);
  FTofree:=F;
  AssertSQL(F,'A INT');
  F.FieldType.Free;
  F.FieldType:=Nil;
  F.ComputedBy:=CreateBinaryExpression(CreateIdentifierExpression('B'),CreateLiteralExpression(CreateLiteral(1)));
  TSQLBinaryExpression(F.ComputedBy).Operation:=boAdd;
  FTofree:=F;
  AssertSQL(F,'A COMPUTED BY B + 1');
end;

procedure TTestGenerateSQL.TestCreateTable;

Var
  T : TSQLCreateTableStatement;
  F : TSQLTableFieldDef;
  U : TSQLTableUniqueConstraintDef;

begin
  F:=TSQLTableFieldDef.Create(Nil);
  F.FieldName:=CreateIdentifier('B');
  F.FieldType:=CreateTypeDefinition(sdtInteger,0);
  T:=TSQLCreateTableStatement.Create(Nil);
  T.ObjectName:=CreateIdentifier('A');
  FTofree:=T;
  T.FieldDefs.Add(F);
  AssertSQL(T,'CREATE TABLE A (B INT)');
  T.ExternalFileName:=CreateLiteral('/my/file');
  AssertSQL(T,'CREATE TABLE A EXTERNAL FILE ''/my/file'' (B INT)');
  T.ExternalFileName.Free;
  T.ExternalFileName:=Nil;
  F:=TSQLTableFieldDef.Create(Nil);
  F.FieldName:=CreateIdentifier('C');
  F.FieldType:=CreateTypeDefinition(sdtChar,5);
  T.FieldDefs.Add(F);
  FTofree:=T;
  AssertSQL(T,'CREATE TABLE A (B INT, C CHAR(5))');
  AssertSQL(T,'CREATE TABLE A ('+sLineBreak+'B INT,'+sLineBreak+'C CHAR(5))',[sfoOneFieldPerLine]);
  AssertSQL(T,'CREATE TABLE A ('+sLineBreak+'  B INT,'+sLineBreak+'  C CHAR(5))',[sfoOneFieldPerLine,sfoIndentFields]);
  U:=TSQLTableUniqueConstraintDef.Create(Nil);
  U.FieldList.Add(CreateIdentifier('B'));
  T.Constraints.Add(U);
  FTofree:=T;
  AssertSQL(T,'CREATE TABLE A (B INT, C CHAR(5), UNIQUE (B))');
  AssertSQL(T,'CREATE TABLE A ('+sLineBreak+'B INT,'+sLineBreak+'C CHAR(5),'+sLineBreak+'UNIQUE (B))',[sfoOneFieldPerLine]);
  AssertSQL(T,'CREATE TABLE A ('+sLineBreak+'  B INT,'+sLineBreak+'  C CHAR(5),'+sLineBreak+'  UNIQUE (B))',[sfoOneFieldPerLine,sfoIndentFields]);
  U.ConstraintName:=CreateIdentifier('U_A');
  FTofree:=T;
  AssertSQL(T,'CREATE TABLE A (B INT, C CHAR(5), CONSTRAINT U_A UNIQUE (B))');
end;

procedure TTestGenerateSQL.TestDropFieldOperation;

Var
  O : TSQLDropTableFieldOperation;

begin
  O:=TSQLDropTableFieldOperation.Create(Nil);
  O.ObjectName:=CreateIdentifier('A');
  FTofree:=O;
  AssertSQL(O,'DROP A');
end;

procedure TTestGenerateSQL.TestDropConstraintOperation;

Var
  O : TSQLDropTableConstraintOperation;

begin
  O:=TSQLDropTableConstraintOperation.Create(Nil);
  O.ObjectName:=CreateIdentifier('A');
  FTofree:=O;
  AssertSQL(O,'DROP CONSTRAINT A');
end;

procedure TTestGenerateSQL.TestAlterFieldNameOperation;

Var
  O : TSQLAlterTableFieldNameOperation;

begin
  O:=TSQLAlterTableFieldNameOperation.Create(Nil);
  O.ObjectName:=CreateIdentifier('A');
  O.NewName:=CreateIdentifier('B');
  FTofree:=O;
  ASSERTSQl(O,'ALTER COLUMN A TO B');
end;

procedure TTestGenerateSQL.TestAlterFieldTypeOperation;

Var
  O : TSQLAlterTableFieldTypeOperation;

begin
  O:=TSQLAlterTableFieldTypeOperation.Create(Nil);
  O.ObjectName:=CreateIdentifier('A');
  O.NewType:=CreateTypeDefinition(sdtInteger,0);
  FTofree:=O;
  ASSERTSQl(O,'ALTER COLUMN A TYPE INT');
end;

procedure TTestGenerateSQL.TestAlterFieldPositionOperation;

Var
  O : TSQLAlterTableFieldPositionOperation;

begin
  O:=TSQLAlterTableFieldPositionOperation.Create(Nil);
  O.ObjectName:=CreateIdentifier('A');
  O.NewPosition:=2;
  FTofree:=O;
  ASSERTSQl(O,'ALTER COLUMN A POSITION 2');
end;

procedure TTestGenerateSQL.TestAddFieldOperation;

Var
  O : TSQLAlterTableAddFieldOperation;
  F : TSQLTableFieldDef;

begin
  F:=TSQLTableFieldDef.Create(Nil);
  FTofree:=F;
  F.FieldName:=CreateIdentifier('C');
  F.FieldType:=CreateTypeDefinition(sdtChar,5);
  O:=TSQLAlterTableAddFieldOperation.Create(Nil);
  FTofree:=O;
  O.Element:=F;
  AssertSQL(O,'ADD C CHAR(5)');
end;

procedure TTestGenerateSQL.TestAddConstraintOperation;

Var
  O : TSQLAlterTableAddConstraintOperation;
  C : TSQLTableUniqueConstraintDef;

begin
  C:=TSQLTableUniqueConstraintDef.Create(Nil);
  FTofree:=C;
  C.FieldList.Add(CreateIdentifier('C'));
  O:=TSQLAlterTableAddConstraintOperation.Create(Nil);
  FTofree:=O;
  O.Element:=C;
  AssertSQL(O,'ADD CONSTRAINT UNIQUE (C)');
  C.ConstraintName:=CreateIdentifier('D');
  AssertSQL(O,'ADD CONSTRAINT D UNIQUE (C)');
end;

procedure TTestGenerateSQL.TestAlterTable;

Var
  A : TSQLAlterTableStatement;
  DF : TSQLDropTableFieldOperation;
  AC : TSQLAlterTableAddConstraintOperation;
  C : TSQLTableUniqueConstraintDef;

begin
  A:=TSQLAlterTableStatement.Create(Nil);
  A.ObjectName:=CreateIdentifier('A');
  DF:=TSQLDropTableFieldOperation.Create(Nil);
  DF.ObjectName:=CreateIdentifier('B');
  A.Operations.Add(DF);
  FTofree:=A;
  AssertSQL(A,'ALTER TABLE A DROP B');
  AssertSQL(A,'ALTER TABLE A'+sLineBreak+'DROP B',[sfoOneFieldPerLine]);
  AssertSQL(A,'ALTER TABLE A'+sLineBreak+'  DROP B',[sfoOneFieldPerLine,sfoIndentFields]);
  C:=TSQLTableUniqueConstraintDef.Create(Nil);
  FTofree:=C;
  C.FieldList.Add(CreateIdentifier('C'));
  AC:=TSQLAlterTableAddConstraintOperation.Create(Nil);
  AC.Element:=C;
  A.Operations.Add(AC);
  FTofree:=A;
  AssertSQL(A,'ALTER TABLE A DROP B, ADD CONSTRAINT UNIQUE (C)');
end;

procedure TTestGenerateSQL.TestCreateView;

Var
  V : TSQLCreateViewStatement;

begin
  V:=TSQLCreateViewStatement.Create(Nil);
  V.ObjectName:=CreateIdentifier('A');
  V.Select:=CreateSelect(CreateIdentifierExpression('B'),'C');
  FToFree:=V;
  AssertSQL(V,'CREATE VIEW A AS SELECT B FROM C');
  V.Fields.Add(CreateIdentifier('D'));
  FToFree:=V;
  AssertSQL(V,'CREATE VIEW A (D) AS SELECT B FROM C');
end;

procedure TTestGenerateSQL.TestDatabaseFileInfo;

Var
  F : TSQLDatabaseFileInfo;

begin
  F:=TSQLDatabaseFileInfo.Create(Nil);
  F.FileName:='/my/file';
  FTofree:=F;
  AssertSQL(F,'FILE ''/my/file''');
  F.Length:=2;
  AssertSQL(F,'FILE ''/my/file'' LENGTH 2 PAGES');
  F.Length:=0;
  F.StartPage:=3;
  AssertSQL(F,'FILE ''/my/file'' STARTING AT 3');
end;

procedure TTestGenerateSQL.TestCreateDatabase;

Var
  D : TSQLCreateDatabaseStatement;
  F : TSQLDatabaseFileInfo;

begin
  D:=TSQLCreateDatabaseStatement.Create(Nil);
  D.FileName:='/my/file';
  FTofree:=D;
  AssertSQL(D,'CREATE DATABASE ''/my/file''');
  D.UseSchema:=true;
  AssertSQL(D,'CREATE SCHEMA ''/my/file''');
  D.UseSchema:=False;
  D.Password:='Secret';
  AssertSQL(D,'CREATE DATABASE ''/my/file''');
  D.UserName:='me';
  AssertSQL(D,'CREATE DATABASE ''/my/file'' USER ''me'' PASSWORD ''Secret''');
  AssertSQL(D,'CREATE DATABASE ''/my/file'''+SlineBreak+'  USER ''me'' PASSWORD ''Secret''',[sfoMultilineCreateDatabase]);
  D.PageSize:=2048;
  AssertSQL(D,'CREATE DATABASE ''/my/file'' USER ''me'' PASSWORD ''Secret'' PAGE_SIZE 2048');
  AssertSQL(D,'CREATE DATABASE ''/my/file'''+SlineBreak+'  USER ''me'' PASSWORD ''Secret'''+SlineBreak+'  PAGE_SIZE 2048',[sfoMultilineCreateDatabase]);
  D.Length:=1000;
  AssertSQL(D,'CREATE DATABASE ''/my/file'' USER ''me'' PASSWORD ''Secret'' PAGE_SIZE 2048 LENGTH 1000');
  AssertSQL(D,'CREATE DATABASE ''/my/file'''+SlineBreak+'  USER ''me'' PASSWORD ''Secret'''+SlineBreak+'  PAGE_SIZE 2048'+SlineBreak+'  LENGTH 1000',[sfoMultilineCreateDatabase]);
  D.CharSet:=CreateIdentifier('UTF8');
  FToFree:=D;
  AssertSQL(D,'CREATE DATABASE ''/my/file'' USER ''me'' PASSWORD ''Secret'' PAGE_SIZE 2048 LENGTH 1000 DEFAULT CHARACTER SET UTF8');
  AssertSQL(D,'CREATE DATABASE ''/my/file'''+SlineBreak+'  USER ''me'' PASSWORD ''Secret'''+SlineBreak+'  PAGE_SIZE 2048'+SlineBreak+'  LENGTH 1000'+SlineBreak+'  DEFAULT CHARACTER SET UTF8',[sfoMultilineCreateDatabase]);
  F:=TSQLDatabaseFileInfo.Create(Nil);
  FToFree:=D;
  F.FileName:='/my/file2';
  F.StartPage:=3;
  D.SecondaryFiles.Add(F);
  AssertSQL(D,'CREATE DATABASE ''/my/file'' USER ''me'' PASSWORD ''Secret'' PAGE_SIZE 2048 LENGTH 1000 DEFAULT CHARACTER SET UTF8 FILE ''/my/file2'' STARTING AT 3');
  AssertSQL(D,'CREATE DATABASE ''/my/file'''+SlineBreak+'  USER ''me'' PASSWORD ''Secret'''+SlineBreak+'  PAGE_SIZE 2048'+SlineBreak+'  LENGTH 1000'+SlineBreak+'  DEFAULT CHARACTER SET UTF8'+SlineBreak+'  FILE ''/my/file2'' STARTING AT 3',[sfoMultilineCreateDatabase]);
  F:=TSQLDatabaseFileInfo.Create(Nil);
  FToFree:=D;
  F.FileName:='/my/file3';
  F.StartPage:=4;
  D.SecondaryFiles.Add(F);
  AssertSQL(D,'CREATE DATABASE ''/my/file'' USER ''me'' PASSWORD ''Secret'' PAGE_SIZE 2048 LENGTH 1000 DEFAULT CHARACTER SET UTF8 FILE ''/my/file2'' STARTING AT 3 FILE ''/my/file3'' STARTING AT 4');
  AssertSQL(D,'CREATE DATABASE ''/my/file'''+SlineBreak+'  USER ''me'' PASSWORD ''Secret'''+SlineBreak+'  PAGE_SIZE 2048'+SlineBreak+'  LENGTH 1000'+SlineBreak+'  DEFAULT CHARACTER SET UTF8'+SlineBreak+'  FILE ''/my/file2'' STARTING AT 3'+SlineBreak+'  FILE ''/my/file3'' STARTING AT 4',[sfoMultilineCreateDatabase]);
end;

procedure TTestGenerateSQL.TestAlterDatabase;

Var
  D : TSQLAlterDatabaseStatement;
  F : TSQLDatabaseFileInfo;
begin
  D:=TSQLAlterDatabaseStatement.Create(Nil);
  F:=TSQLDatabaseFileInfo.Create(Nil);
  FToFree:=D;
  F.FileName:='/my/file2';
  F.StartPage:=3;
  D.Operations.Add(F);
  FToFree:=D;
  AssertSQL(D,'ALTER DATABASE ADD FILE ''/my/file2'' STARTING AT 3');
  AssertSQL(D,'ALTER DATABASE ADD'+SlineBreak+'  FILE ''/my/file2'' STARTING AT 3',[sfoMultilineCreateDatabase]);
  D.UseSchema:=True;
  AssertSQL(D,'ALTER SCHEMA ADD FILE ''/my/file2'' STARTING AT 3');
  D.UseSchema:=False;
  F:=TSQLDatabaseFileInfo.Create(Nil);
  F.FileName:='/my/file3';
  F.StartPage:=4;
  D.Operations.Add(F);
  FToFree:=D;
  AssertSQL(D,'ALTER DATABASE ADD FILE ''/my/file2'' STARTING AT 3 FILE ''/my/file3'' STARTING AT 4');
  AssertSQL(D,'ALTER DATABASE ADD'+SlineBreak+'  FILE ''/my/file2'' STARTING AT 3'+SlineBreak+'  FILE ''/my/file3'' STARTING AT 4',[sfoMultilineCreateDatabase]);
end;

procedure TTestGenerateSQL.TestCreateShadow;

Var
  S : TSQLCreateShadowStatement;
  F : TSQLDatabaseFileInfo;

begin
  S:=TSQLCreateShadowStatement.Create(Nil);
  S.Number:=1;
  S.FileName:='/my/file';
  FTofree:=S;
  AssertSQL(S,'CREATE SHADOW 1 ''/my/file''');
  S.Manual:=True;
  AssertSQL(S,'CREATE SHADOW 1 MANUAL ''/my/file''');
  S.Conditional:=True;
  AssertSQL(S,'CREATE SHADOW 1 MANUAL CONDITIONAL ''/my/file''');
  S.Length:=4;
  AssertSQL(S,'CREATE SHADOW 1 MANUAL CONDITIONAL ''/my/file'' LENGTH 4 PAGES');
  F:=TSQLDatabaseFileInfo.Create(Nil);
  FTofree:=S;
  F.FileName:='/my/file2';
  F.StartPage:=4;
  S.SecondaryFiles.Add(F);
  AssertSQL(S,'CREATE SHADOW 1 MANUAL CONDITIONAL ''/my/file'' LENGTH 4 PAGES FILE ''/my/file2'' STARTING AT 4');
  AssertSQL(S,'CREATE SHADOW 1 MANUAL CONDITIONAL ''/my/file'' LENGTH 4 PAGES'+sLineBreak+'  FILE ''/my/file2'' STARTING AT 4',[sfoMultilineCreateShadow]);
end;

procedure TTestGenerateSQL.TestSuspend;
Var
  S : TSQLSuspendStatement;

begin
  S:=TSQLSuspendStatement.Create(Nil);
  FToFree:=S;
  AssertSQL(S,'SUSPEND');
end;

procedure TTestGenerateSQL.TestExit;
Var
  S : TSQLExitStatement;

begin
  S:=TSQLExitStatement.Create(Nil);
  FToFree:=S;
  AssertSQL(S,'EXIT');
end;

procedure TTestGenerateSQL.TestBlock;

Var
  B,B2 : TSQLStatementBlock;
  L : TSQLSelectStatement;

begin
  B:=TSQLStatementBlock.Create(Nil);
  FtoFree:=B;
  B.Statements.Add(TSQLExitStatement.Create(Nil));
  FtoFree:=B;
  AssertSQL(B,'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END');
  B.Statements.Add(TSQLSuspendStatement.Create(Nil));
  FtoFree:=B;
  AssertSQL(B,'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'  SUSPEND;'+sLineBreak+'END');
  B.Statements.Clear;
  L:=CreateSelect(CreateIdentifierExpression('A'),'B');
  FtoFree:=B;
  B.Statements.Add(L);
  AssertSQL(B,'BEGIN'+sLineBreak+'  SELECT'+sLineBreak+'  A'+sLineBreak+'  FROM'+sLineBreak+'  B;'+sLineBreak+'END',[sfoOneFieldPerLine,sfoOneTablePerLine]);
  AssertSQL(B,'BEGIN'+sLineBreak+'  SELECT'+sLineBreak+'    A'+sLineBreak+'  FROM'+sLineBreak+'    B;'+sLineBreak+'END',[sfoOneFieldPerLine,sfoIndentFields,sfoOneTablePerLine,sfoIndentTables]);
  B.Statements.Clear;
  B2:=TSQLStatementBlock.Create(Nil);
  FtoFree:=B;
  B2.Statements.Add(TSQLExitStatement.Create(Nil));
  FtoFree:=B;
  B.Statements.Add(B2);
  AssertSQL(B,'BEGIN'+sLineBreak+'  BEGIN'+sLineBreak+'    EXIT;'+sLineBreak+'  END'+sLineBreak+'END');
end;

procedure TTestGenerateSQL.TestAssignment;

var
  A : TSQLAssignStatement;

begin
  A:=TSQLAssignStatement.Create(Nil);
  A.Variable:=CreateIdentifier('A');
  A.Expression:=CreateIdentifierExpression('B');
  FToFree:=A;
  AssertSQL(A,'A = B');
end;

procedure TTestGenerateSQL.TestIf;

Var
  I : TSQLIfStatement;
  C : TSQLBinaryExpression;

begin
  I:=TSQLIfStatement.Create(Nil);
  C:=CreateBinaryExpression(CreateIdentifierExpression('A'),CreateIdentifierExpression('B'));
  C.Operation:=boEQ;
  I.Condition:=C;
  I.TrueBranch:=TSQLSuspendStatement.Create(Nil);
  FToFree:=I;
  AssertSQL(I,'IF (A = B) THEN'+sLineBreak+'  SUSPEND');
  I.FalseBranch:=TSQLExitStatement.Create(Nil);
  AssertSQL(I,'IF (A = B) THEN'+sLineBreak+'  SUSPEND'+sLineBreak+'ELSE'+sLineBreak+'  EXIT');
end;

procedure TTestGenerateSQL.TestFor;
Var
  F : TSQLForStatement;

begin
  F:=TSQLForStatement.Create(Nil);
  F.Select:=CreateSelect(CreateIdentifierExpression('A'),'B');
  F.FieldList.Add(CreateIdentifier('ID'));
  F.Statement:=TSQLSuspendStatement.Create(Nil);
  AssertSQL(F,'FOR SELECT A FROM B'+sLineBreak+'INTO'+sLineBreak+':ID'+sLineBreak+'DO'+sLineBreak+'  SUSPEND',[sfoIndentFields]);
  F.Select.Fields.Add(CreateIdentifier('C'));
  F.FieldList.Add(CreateIdentifier('ID2'));
  AssertSQL(F,'FOR SELECT A, C FROM B'+sLineBreak+'INTO'+sLineBreak+':ID, :ID2'+sLineBreak+'DO'+sLineBreak+'  SUSPEND');
  AssertSQL(F,'FOR SELECT'+sLineBreak+'A,'+sLineBreak+'C'+sLineBreak+'FROM B'+sLineBreak+'INTO'+sLineBreak+':ID,'+sLineBreak+':ID2'+sLineBreak+'DO'+sLineBreak+'  SUSPEND',[sfoOneFieldPerLine]);
end;

procedure TTestGenerateSQL.TestWhile;

Var
  W : TSQLWhileStatement;
  C : TSQLBinaryExpression;
  A : TSQLAssignStatement;

begin
  W:=TSQLWhileStatement.Create(Nil);
  C:=CreateBinaryExpression(CreateIdentifierExpression('A'),CreateIdentifierExpression('B'));
  C.Operation:=boLT;
  W.Condition:=C;
  FToFree:=W;
  A:=TSQLAssignStatement.Create(Nil);
  A.Variable:=CreateIdentifier('A');
  C:=CreateBinaryExpression(CreateIdentifierExpression('A'),CreateLiteralExpression(CreateLiteral(1)));
  C.Operation:=boAdd;
  A.Expression:=C;
  W.Statement:=A;
  FToFree:=W;
  AssertSQL(W,'WHILE (A < B) DO'+slineBreak+'  A = A + 1');
end;

procedure TTestGenerateSQL.TestWhenSQLError;

Var
  S : TSQLWhenSQLError;

begin
  S:=TSQLWhenSQLError.Create(Nil);
  S.ErrorCode:=2;
  FToFree:=S;
  AssertSQL(S,'SQLCODE 2');
end;

procedure TTestGenerateSQL.TestWhenGDSError;

Var
  G : TSQLWhenGDSError;

begin
  G:=TSQLWhenGDSError.Create(Nil);
  G.GDSErrorNumber:=2;
  FToFree:=G;
  AssertSQL(G,'GDSCODE 2');
end;

procedure TTestGenerateSQL.TestWhenException;

Var
  E : TSQLWhenException;

begin
  E:=TSQLWhenException.Create(Nil);
  E.ExceptionName:=CreateIdentifier('E');
  FToFree:=E;
  AssertSQL(E,'EXCEPTION E');
end;

procedure TTestGenerateSQL.TestWhen;

Var
  W : TSQLWhenStatement;
  E : TSQLWhenException;
  G : TSQLWhenGDSError;
  S : TSQLWhenSQLError;

begin
  W:=TSQLWhenStatement.Create(Nil);
  FToFree:=W;
  W.Statement:=TSQLExitStatement.Create(Nil);
  FToFree:=W;
  W.AnyError:=True;
  AssertSQL(W,'WHEN ANY DO'+sLineBreak+'  EXIT');
  W.AnyError:=False;
  E:=TSQLWhenException.Create(Nil);
  E.ExceptionName:=CreateIdentifier('E');
  W.Errors.Add(E);
  AssertSQL(W,'WHEN EXCEPTION E DO'+sLineBreak+'  EXIT');
  W.Errors.Extract(E);
  S:=TSQLWhenSQLError.Create(Nil);
  S.ErrorCode:=2;
  W.Errors.Add(S);
  AssertSQL(W,'WHEN SQLCODE 2 DO'+sLineBreak+'  EXIT');
  W.Errors.Extract(S);
  G:=TSQLWhenGDSError.Create(Nil);
  G.GDSErrorNumber:=3;
  W.Errors.Add(G);
  AssertSQL(W,'WHEN GDSCODE 3 DO'+sLineBreak+'  EXIT');
  W.Errors.Add(S);
  AssertSQL(W,'WHEN GDSCODE 3 , SQLCODE 2 DO'+sLineBreak+'  EXIT');
  W.Errors.Add(E);
  AssertSQL(W,'WHEN GDSCODE 3 , SQLCODE 2 , EXCEPTION E DO'+sLineBreak+'  EXIT');
end;

procedure TTestGenerateSQL.TestException;

Var
  E : TSQLExceptionStatement;

begin
  E:=TSQLExceptionStatement.Create(Nil);
  E.ExceptionName:=CreateIdentifier('E');
  FToFree:=E;
  AssertSQL(E,'EXCEPTION E');
end;

procedure TTestGenerateSQL.TestPostEvent;

Var
  P : TSQLPostEventStatement;

begin
  P:=TSQLPostEventStatement.Create(nil);
  P.ColName:=CreateIdentifier('E');
  FTofree:=P;
  AssertSQL(P,'POST_EVENT E');
  P.ColName.Free;
  P.ColName:=Nil;
  P.EventName:='E';
  AssertSQL(P,'POST_EVENT ''E''');
end;

procedure TTestGenerateSQL.TestTriggerProcedure;

Var
  S : TSQLCreateOrAlterProcedureTriggerStatement;
  P : TSQLProcedureParamDef;

begin
  S:=TSQLCreateOrAlterProcedureTriggerStatement.Create(Nil);
  FToFree:=S;
  AssertSQL(S,'BEGIN'+sLineBreak+'END');
  S.Statements.Add(TSQLExitStatement.Create(Nil));
  AssertSQL(S,'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('A');
  P.ParamType:=CreateTypeDefinition(sdtInteger,0);
  FToFree:=S;
  S.LocalVariables.Add(P);
  AssertSQL(S,'DECLARE VARIABLE A INT;'+sLineBreak+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,'DECLARE VARIABLE A INT;'+sLineBreak+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('B');
  P.ParamType:=CreateTypeDefinition(sdtChar,5);
  FToFree:=S;
  S.LocalVariables.Add(P);
  AssertSQL(S,'DECLARE VARIABLE A INT;'+sLineBreak+'DECLARE VARIABLE B CHAR(5);'+sLineBreak+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,'DECLARE VARIABLE A INT;'+sLineBreak+'DECLARE VARIABLE B CHAR(5);'+sLineBreak+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
end;


procedure TTestGenerateSQL.DoTestAlterCreateProcedure(S : TSQLAlterCreateProcedureStatement; PHEAD : String);

Var
  P : TSQLProcedureParamDef;
  H : TSQLStringType;

begin
  PHead:=PHead+' P';
  FToFree:=S;
  S.ObjectName:=CreateIdentifier('P');
  H:=PHEAD+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.Statements.Add(TSQLExitStatement.Create(Nil));
  AssertSQL(S,H+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('I');
  P.ParamType:=CreateTypeDefinition(sdtInteger,0);
  FToFree:=S;
  S.InputVariables.Add(P);
  H:=PHEAD+' (I INT)'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('J');
  P.ParamType:=CreateTypeDefinition(sdtChar,5);
  FToFree:=S;
  S.InputVariables.Add(P);
  H:=PHEAD+' (I INT , J CHAR(5))'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('R');
  P.ParamType:=CreateTypeDefinition(sdtInteger,0);
  FToFree:=S;
  S.OutputVariables.Add(P);
  H:=PHEAD+' (I INT , J CHAR(5))'+sLineBreak+'RETURNS (R INT)'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('S');
  P.ParamType:=CreateTypeDefinition(sdtChar,5);
  FToFree:=S;
  S.OutputVariables.Add(P);
  H:=PHEAD+' (I INT , J CHAR(5))'+sLineBreak+'RETURNS (R INT , S CHAR(5))'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('A');
  P.ParamType:=CreateTypeDefinition(sdtInteger,0);
  FToFree:=S;
  S.LocalVariables.Add(P);
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('B');
  P.ParamType:=CreateTypeDefinition(sdtChar,5);
  FToFree:=S;
  S.LocalVariables.Add(P);
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'DECLARE VARIABLE B CHAR(5);'+sLineBreak+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'DECLARE VARIABLE B CHAR(5);'+sLineBreak+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
end;
procedure TTestGenerateSQL.TestCreateProcedure;
begin
  DoTestAlterCreateProcedure(TSQLCreateProcedureStatement.Create(Nil),'CREATE PROCEDURE');
end;

procedure TTestGenerateSQL.TestAlterProcedure;

begin
  DoTestAlterCreateProcedure(TSQLAlterProcedureStatement.Create(Nil),'ALTER PROCEDURE');
end;


procedure TTestGenerateSQL.DoTestAlterCreateTrigger(
  S: TSQLAlterCreateTriggerStatement; PHEAD: String);

Var
  P : TSQLProcedureParamDef;
  H : TSQLStringType;

begin
  FToFree:=S;
  S.ObjectName:=CreateIdentifier('TR');
  S.TableName:=CreateIdentifier('TA');
  S.Position:=1;
  S.Moment:=tmBefore;
  S.Operations:=[toDelete];
  H:=PHEAD+sLineBreak+'BEFORE DELETE'+sLineBreak+'POSITION 1'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.Moment:=tmAfter;
  S.Operations:=[toDelete];
  H:=PHEAD+sLineBreak+'AFTER DELETE'+sLineBreak+'POSITION 1'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.Operations:=[toUPDATE];
  H:=PHEAD+sLineBreak+'AFTER UPDATE'+sLineBreak+'POSITION 1'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.Operations:=[toUPDATE,toDelete];
  H:=PHEAD+sLineBreak+'AFTER DELETE OR UPDATE'+sLineBreak+'POSITION 1'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.Operations:=[toInsert];
  S.State:=tsActive;
  H:=PHEAD+sLineBreak+'ACTIVE'+sLineBreak+'AFTER INSERT'+sLineBreak+'POSITION 1'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.State:=tsInActive;
  H:=PHEAD+sLineBreak+'INACTIVE'+sLineBreak+'AFTER INSERT'+sLineBreak+'POSITION 1'+sLineBreak+'AS'+sLineBreak;
  AssertSQL(S,H+'BEGIN'+sLineBreak+'END');
  S.Statements.Add(TSQLExitStatement.Create(Nil));
  AssertSQL(S,H+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('A');
  P.ParamType:=CreateTypeDefinition(sdtInteger,0);
  FToFree:=S;
  S.LocalVariables.Add(P);
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
  P:=TSQLProcedureParamDef.Create(Nil);
  P.ParamName:=CreateIdentifier('B');
  P.ParamType:=CreateTypeDefinition(sdtChar,5);
  FToFree:=S;
  S.LocalVariables.Add(P);
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'DECLARE VARIABLE B CHAR(5);'+sLineBreak+'BEGIN'+sLineBreak+'EXIT;'+sLineBreak+'END');
  AssertSQL(S,H+'DECLARE VARIABLE A INT;'+sLineBreak+'DECLARE VARIABLE B CHAR(5);'+sLineBreak+'BEGIN'+sLineBreak+'  EXIT;'+sLineBreak+'END',[sfoIndentProcedureBlock]);
end;

procedure TTestGenerateSQL.TestCreateTrigger;
begin
  DoTestAlterCreateTrigger(TSQLCreateTriggerStatement.Create(Nil),'CREATE TRIGGER TR FOR TA');
end;

procedure TTestGenerateSQL.TestAlterTrigger;
begin
  DoTestAlterCreateTrigger(TSQLAlterTriggerStatement.Create(Nil),'ALTER TRIGGER TR');
end;

procedure TTestGenerateSQL.DoTestDropStatement(AClass : TSQLDropStatementClass; Const AObjectName : String);

Var
  D : TSQLDropStatement;

begin
  D:=AClass.Create(Nil);
  D.ObjectName:=CreateIdentifier('D');
  FToFree:=Nil;
  Try
    AssertSQL(D,'DROP '+AObjectName+' D');
    AssertSQL(D,'drop '+lowercase(AObjectName)+' D',[sfoLowerCaseKeyWord]);
  Finally
    D.Free;
  end;
end;

procedure TTestGenerateSQL.TestDropStatement;
begin
  DoTestDropStatement(TSQLDropTableStatement,'TABLE');
  DoTestDropStatement(TSQLDropIndexStatement,'INDEX');
  DoTestDropStatement(TSQLDropViewStatement,'VIEW');
  DoTestDropStatement(TSQLDropProcedureStatement,'PROCEDURE');
  DoTestDropStatement(TSQLDropDomainStatement,'DOMAIN');
  DoTestDropStatement(TSQLDropGeneratorStatement,'GENERATOR');
  DoTestDropStatement(TSQLDropTriggerStatement,'TRIGGER');
  DoTestDropStatement(TSQLDropExceptionStatement,'EXCEPTION');
  DoTestDropStatement(TSQLDropDatabaseStatement,'DATABASE');
  DoTestDropStatement(TSQLDropRoleStatement,'ROLE');
  DoTestDropStatement(TSQLDropExternalFunctionStatement,'EXTERNAL FUNCTION');
  DoTestDropStatement(TSQLDropShadowStatement,'SHADOW');
end;

procedure TTestGenerateSQL.TestConnect;

Const
  Base = 'CONNECT ''db''';

Var
  C : TSQLConnectStatement;

begin
  C:=TSQLConnectStatement.Create(Nil);
  FToFree:=C;
  C.DatabaseName:='db';
  AssertSQL(C,Base);
  C.UserName:='u';
  AssertSQL(C,Base+' USER ''u''');
  C.Password:='p';
  AssertSQL(C,Base+' USER ''u'' PASSWORD ''p''');
  C.UserName:='';
  AssertSQL(C,Base+' PASSWORD ''p''');
  C.Password:='';
  C.Cache:=2048;
  AssertSQL(C,Base+' CACHE 2048');
  C.Cache:=0;
  C.Role:='r';
  AssertSQL(C,Base+' ROLE ''r''');
end;

procedure TTestGenerateSQL.TestExtract;

Var
  C : TSQLExtractExpression;

begin
  AssertEquals(ExtractElementNames[eeYear],'YEAR');
  AssertEquals(ExtractElementNames[eeMonth],'MONTH');
  AssertEquals(ExtractElementNames[eeDay],'DAY');
  AssertEquals(ExtractElementNames[eeHour],'HOUR');
  AssertEquals(ExtractElementNames[eeMinute],'MINUTE');
  AssertEquals(ExtractElementNames[eeSecond],'SECOND');
  AssertEquals(ExtractElementNames[eeWeekDay],'WEEKDAY');
  AssertEquals(ExtractElementNames[eeYearDay],'YEARDAY');
  C:=TSQLExtractExpression.Create(Nil);
  C.Value:=CreateIdentifierExpression('A');
  C.Element:=eeYear;
  AssertSQL(C,'EXTRACT(YEAR FROM A)');
  C.Element:=eeMonth;
  AssertSQL(C,'EXTRACT(MONTH FROM A)');
  C.Element:=eeDay;
  AssertSQL(C,'EXTRACT(DAY FROM A)');
  C.Element:=eeHour;
  AssertSQL(C,'EXTRACT(HOUR FROM A)');
  C.Element:=eeMinute;
  AssertSQL(C,'EXTRACT(MINUTE FROM A)');
  C.Element:=eeSecond;
  AssertSQL(C,'EXTRACT(SECOND FROM A)');
  C.Element:=eeWeekday;
  AssertSQL(C,'EXTRACT(WEEKDAY FROM A)');
  C.Element:=eeYearday;
  AssertSQL(C,'EXTRACT(YEARDAY FROM A)');
end;

procedure TTestGenerateSQL.TestParamExpression;

Var
  P : TSQLParameterExpression;

begin
  P:=TSQLParameterExpression.Create(Nil);
  P.Identifier:=CreateIdentifier('P');
  FTofree:=P;
  AssertSQL(P,':P');
end;

procedure TTestGenerateSQL.TestGrantTable;

Var
  G : TSQLTableGrantStatement;
  {%H-}U : TSQLUserGrantee;
  PU : TSQLColumnPrivilege;
begin
  G:=TSQLTableGrantStatement.Create(Nil);
  G.TableName:=CreateIdentifier('A');
  FtoFree:=G;
  G.Privileges.Add(TSQLSelectPrivilege.Create(Nil));
  G.Grantees.add(CreateGrantee('B'));
  FtoFree:=G;
  AssertSQl(G,'GRANT SELECT ON A TO B');
  G.Grantees.add(CreateGrantee('C'));
  FtoFree:=G;
  AssertSQl(G,'GRANT SELECT ON A TO B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLUPdatePrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'GRANT UPDATE ON A TO B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLDeletePrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'GRANT DELETE ON A TO B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLINSERTPrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'GRANT INSERT ON A TO B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLReferencePrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'GRANT REFERENCES ON A TO B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLAllPrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'GRANT ALL PRIVILEGES ON A TO B , C');
  G.GrantOption:=True;
  AssertSQl(G,'GRANT ALL PRIVILEGES ON A TO B , C WITH GRANT OPTION');
  G.GrantOption:=False;
  G.Privileges.Clear;
  G.Privileges.Add(TSQLSelectPrivilege.Create(Nil));
  G.Grantees.Clear;
  G.Grantees.Add(TSQLPublicGrantee.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'GRANT SELECT ON A TO PUBLIC');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLProcedureGrantee));
  AssertSQl(G,'GRANT SELECT ON A TO PROCEDURE B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLViewGrantee));
  AssertSQl(G,'GRANT SELECT ON A TO VIEW B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLTriggerGrantee));
  AssertSQl(G,'GRANT SELECT ON A TO TRIGGER B');
  FtoFree:=G;
  G.Privileges.Clear;
  Pu:=TSQLUPdatePrivilege.Create(Nil);
  PU.Columns:=TSQLElementList.Create(True);
  PU.Columns.Add(CreateIdentifier('C'));
  G.Privileges.Add(PU);
  FtoFree:=G;
  AssertSQl(G,'GRANT UPDATE (C) ON A TO TRIGGER B');
  PU.Columns.Add(CreateIdentifier('D'));
  FtoFree:=G;
  AssertSQl(G,'GRANT UPDATE (C , D) ON A TO TRIGGER B');
  G.Privileges.Clear;
  Pu:=TSQLReferencePrivilege.Create(Nil);
  PU.Columns:=TSQLElementList.Create(True);
  PU.Columns.Add(CreateIdentifier('C'));
  G.Privileges.Add(PU);
  FtoFree:=G;
  AssertSQl(G,'GRANT REFERENCES (C) ON A TO TRIGGER B');
  PU.Columns.Add(CreateIdentifier('D'));
  FtoFree:=G;
  AssertSQl(G,'GRANT REFERENCES (C , D) ON A TO TRIGGER B');
end;

procedure TTestGenerateSQL.TestGrantProcedure;

Var
  G : TSQLProcedureGrantStatement;
begin
  G:=TSQLProcedureGrantStatement.Create(Nil);
  G.ProcedureName:=CreateIdentifier('A');
  FtoFree:=G;
  G.Grantees.add(CreateGrantee('B'));
  FtoFree:=G;
  AssertSQL(G,'GRANT EXECUTE ON PROCEDURE A TO B');
  G.Grantees.add(CreateGrantee('C'));
  FtoFree:=G;
  AssertSQL(G,'GRANT EXECUTE ON PROCEDURE A TO B , C');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLTriggerGrantee));
  FtoFree:=G;
  AssertSQL(G,'GRANT EXECUTE ON PROCEDURE A TO TRIGGER B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLProcedureGrantee));
  FtoFree:=G;
  AssertSQL(G,'GRANT EXECUTE ON PROCEDURE A TO PROCEDURE B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLViewGrantee));
  FtoFree:=G;
  AssertSQL(G,'GRANT EXECUTE ON PROCEDURE A TO VIEW B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLPublicGrantee));
  FtoFree:=G;
  AssertSQL(G,'GRANT EXECUTE ON PROCEDURE A TO PUBLIC');
end;

procedure TTestGenerateSQL.TestGrantRole;
Var
  G : TSQLRoleGrantStatement;
begin
  G:=TSQLRoleGrantStatement.Create(Nil);
  G.Roles.Add(CreateIdentifier('A'));
  FtoFree:=G;
  G.Grantees.add(CreateGrantee('B'));
  FtoFree:=G;
  AssertSQL(G,'GRANT A TO B');
  G.Roles.Add(CreateIdentifier('C'));
  FtoFree:=G;
  AssertSQL(G,'GRANT A , C TO B');
  G.Grantees.add(CreateGrantee('D'));
  FtoFree:=G;
  AssertSQL(G,'GRANT A , C TO B , D');
  G.AdminOption:=True;
  AssertSQL(G,'GRANT A , C TO B , D WITH ADMIN OPTION');
end;

procedure TTestGenerateSQL.TestRevokeTable;
Var
  G : TSQLTableRevokeStatement;
  PU : TSQLColumnPrivilege;
begin
  G:=TSQLTableRevokeStatement.Create(Nil);
  G.TableName:=CreateIdentifier('A');
  FtoFree:=G;
  G.Privileges.Add(TSQLSelectPrivilege.Create(Nil));
  G.Grantees.add(CreateGrantee('B'));
  FtoFree:=G;
  AssertSQl(G,'REVOKE SELECT ON A FROM B');
  G.Grantees.add(CreateGrantee('C'));
  FtoFree:=G;
  AssertSQl(G,'REVOKE SELECT ON A FROM B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLUPdatePrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'REVOKE UPDATE ON A FROM B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLDeletePrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'REVOKE DELETE ON A FROM B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLINSERTPrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'REVOKE INSERT ON A FROM B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLReferencePrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'REVOKE REFERENCES ON A FROM B , C');
  G.Privileges.Clear;
  G.Privileges.Add(TSQLAllPrivilege.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'REVOKE ALL PRIVILEGES ON A FROM B , C');
  G.GrantOption:=True;
  AssertSQl(G,'REVOKE GRANT OPTION FOR ALL PRIVILEGES ON A FROM B , C');
  G.GrantOption:=False;
  G.Privileges.Clear;
  G.Privileges.Add(TSQLSelectPrivilege.Create(Nil));
  G.Grantees.Clear;
  G.Grantees.Add(TSQLPublicGrantee.Create(Nil));
  FtoFree:=G;
  AssertSQl(G,'REVOKE SELECT ON A FROM PUBLIC');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLProcedureGrantee));
  AssertSQl(G,'REVOKE SELECT ON A FROM PROCEDURE B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLViewGrantee));
  AssertSQl(G,'REVOKE SELECT ON A FROM VIEW B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLTriggerGrantee));
  AssertSQl(G,'REVOKE SELECT ON A FROM TRIGGER B');
  FtoFree:=G;
  G.Privileges.Clear;
  Pu:=TSQLUPdatePrivilege.Create(Nil);
  PU.Columns:=TSQLElementList.Create(True);
  PU.Columns.Add(CreateIdentifier('C'));
  G.Privileges.Add(PU);
  FtoFree:=G;
  AssertSQl(G,'REVOKE UPDATE (C) ON A FROM TRIGGER B');
  PU.Columns.Add(CreateIdentifier('D'));
  FtoFree:=G;
  AssertSQl(G,'REVOKE UPDATE (C , D) ON A FROM TRIGGER B');
  G.Privileges.Clear;
  Pu:=TSQLReferencePrivilege.Create(Nil);
  PU.Columns:=TSQLElementList.Create(True);
  PU.Columns.Add(CreateIdentifier('C'));
  G.Privileges.Add(PU);
  FtoFree:=G;
  AssertSQl(G,'REVOKE REFERENCES (C) ON A FROM TRIGGER B');
  PU.Columns.Add(CreateIdentifier('D'));
  FtoFree:=G;
  AssertSQl(G,'REVOKE REFERENCES (C , D) ON A FROM TRIGGER B');
end;

procedure TTestGenerateSQL.TestRevokeProcedure;
Var
  G : TSQLProcedureRevokeStatement;
begin
  G:=TSQLProcedureRevokeStatement.Create(Nil);
  G.ProcedureName:=CreateIdentifier('A');
  FtoFree:=G;
  G.Grantees.add(CreateGrantee('B'));
  FtoFree:=G;
  AssertSQL(G,'REVOKE EXECUTE ON PROCEDURE A FROM B');
  G.Grantees.add(CreateGrantee('C'));
  FtoFree:=G;
  AssertSQL(G,'REVOKE EXECUTE ON PROCEDURE A FROM B , C');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLTriggerGrantee));
  FtoFree:=G;
  AssertSQL(G,'REVOKE EXECUTE ON PROCEDURE A FROM TRIGGER B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLProcedureGrantee));
  FtoFree:=G;
  AssertSQL(G,'REVOKE EXECUTE ON PROCEDURE A FROM PROCEDURE B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLViewGrantee));
  FtoFree:=G;
  AssertSQL(G,'REVOKE EXECUTE ON PROCEDURE A FROM VIEW B');
  G.Grantees.Clear;
  G.Grantees.Add(CreateGrantee('B',TSQLPublicGrantee));
  FtoFree:=G;
  AssertSQL(G,'REVOKE EXECUTE ON PROCEDURE A FROM PUBLIC');
end;

procedure TTestGenerateSQL.TestRevokeRole;
Var
  G : TSQLRoleRevokeStatement;

begin
  G:=TSQLRoleRevokeStatement.Create(Nil);
  G.Roles.Add(CreateIdentifier('A'));
  FtoFree:=G;
  G.Grantees.add(CreateGrantee('B'));
  FtoFree:=G;
  AssertSQL(G,'REVOKE A FROM B');
  G.Roles.Add(CreateIdentifier('C'));
  FtoFree:=G;
  AssertSQL(G,'REVOKE A , C FROM B');
  G.Grantees.add(CreateGrantee('D'));
  FtoFree:=G;
  AssertSQL(G,'REVOKE A , C FROM B , D');
  G.AdminOption:=True;
  AssertSQL(G,'REVOKE A , C FROM B , D');
end;


initialization
  RegisterTest(TTestGenerateSQL);
end.

