{
    This file is part of the Free Component Library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 2010-2014 by the Free Pascal development team
=======
    Copyright (c) 2010 by the Free Pascal development team
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by the Free Pascal development team
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by the Free Pascal development team
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by the Free Pascal development team
>>>>>>> origin/cpstrnew

    SQL source syntax parser

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpsqlparser;
{ $define debugparser}
{ $define debugexpr}
{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpsqlscanner, fpsqltree;

Type
  TParseTypeFlag = (ptfAllowDomainName,ptfAlterDomain,ptfAllowConstraint,
                    ptProcedureParam,ptfTableFieldDef,ptfCast,ptfExternalFunction,
                    ptfExternalFunctionResult);
  TParseTypeFlags = Set of TParseTypeFlag;

  TExpressionOption = (eoCheckConstraint,eoTableConstraint,eoComputedBy,eoOnlyNull,
                       eoFieldValue,eoSelectvalue,eoParamValue,eoWhereClause,eoJoin,
                       eoHaving,eoListValue, eoIF);
  TExpressionOptions = set of TExpressionOption;
  TSelectFlag = (sfSingleTon,sfUnion,sfInto);
  TSelectFlags = Set of TSelectFlag;

  { TSQLParser }

  TSQLParser = Class(TObject)
  Private
    FInput : TStream;
    FScanner : TSQLScanner;
    FCurrent : TSQLToken;
    FCurrentString : String;
    FPrevious : TSQLToken;
    FFreeScanner : Boolean;
    FPeekToken: TSQLToken;
    FPeekTokenString: String;
    Procedure CheckEOF;
  protected
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure UnexpectedToken; overload;
    procedure UnexpectedToken(AExpected : TSQLTokens); overload;
    // All elements must be created with this factory function
    function CreateElement(AElementClass : TSQLElementClass; APArent : TSQLElement)  : TSQLElement; virtual;
    function CreateLiteral(AParent: TSQLElement): TSQLLiteral;
    function CreateIdentifier(AParent : TSQLElement; Const AName : TSQLStringType) : TSQLIdentifierName;
    // Verify that current token is the expected token; raise error if not
    procedure Expect(aToken: TSQLToken);
    // Verify that current token is one of the expected tokens; raise error if not
    procedure Expect(aTokens: TSQLTokens);
    // Expects aToken as current token and eats it by calling GetNextToken
    procedure Consume(aToken: TSQLToken);
    // Expects aTokens tokens and eats the token by calling GetNextToken
    procedure Consume(aTokens: TSQLTokens);
    procedure Error(Msg : String);
    procedure Error(Fmt : String; Args : Array of const);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Procedure UnexpectedToken; overload;
    Procedure UnexpectedToken(AExpected : TSQLTokens); overload;
    // All elements must be created with this factory function
    Function CreateElement(AElementClass : TSQLElementClass; APArent : TSQLElement)  : TSQLElement; virtual;
    function CreateLiteral(AParent: TSQLElement): TSQLLiteral;
    Function CreateIdentifier(AParent : TSQLElement; Const AName : TSQLStringType) : TSQLIdentifierName;
    procedure Expect(aToken: TSQLToken);
    procedure Expect(aTokens: TSQLTokens);
    procedure Consume(aToken: TSQLToken);
    Procedure Error(Msg : String);
    Procedure Error(Fmt : String; Args : Array of const);
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
    // Expression support
    function ParseExprLevel1(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseExprLevel2(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseExprLevel3(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseExprLevel4(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseExprLevel5(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseExprLevel6(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseExprPrimitive(AParent: TSQLElement; EO : TExpressionOptions): TSQLExpression;
    function ParseInoperand(AParent: TSQLElement): TSQLExpression;
    // Lists, primitives
    function ParseIdentifierList(AParent: TSQLElement; AList: TSQLelementList): integer;
    function ParseValueList(AParent: TSQLElement; EO : TExpressionOptions): TSQLElementList;
    function ParseSQLValue(AParent: TSQLElement): TSQLExpression;
    function ParseCheckConstraint(AParent: TSQLElement; TableConstraint : Boolean = False): TSQLExpression;
    // Create/Alter statements
    function ParseAddTableElement(AParent: TSQLElement): TSQLAlterTableAddElementOperation;
    function ParseAlterTableElement(AParent: TSQLElement): TSQLAlterTableOperation;
    function ParseDropTableElement(AParent: TSQLElement): TSQLDropTableElementOperation;
    function ParseFieldConstraint(AParent: TSQLElement): TSQLFieldConstraint;
    function ParseForeignKeyDefinition(AParent: TSQLElement): TSQLForeignKeyDefinition;
    Procedure ParseCharTypeDefinition(Out DT: TSQLDataType; Out Len: Integer; Out ACharset : TSQLStringType);
    procedure ParseBlobDefinition(var ASegmentSize, ABlobType: Integer; Var ACharset : TSQLStringType);
    function ParseTypeDefinition(AParent: TSQLElement; Flags: TParseTypeFlags): TSQLTypeDefinition;
    function ParseTableFieldDef(AParent: TSQLElement): TSQLTableFieldDef;
    function ParseTableConstraint(AParent: TSQLElement): TSQLTableConstraintDef;
    function ParseCreateDomainStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateExceptionStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateGeneratorStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateRoleStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateIndexStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateProcedureStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateTableStatement(AParent: TSQLElement): TSQLCreateOrAlterStatement;
    function ParseAlterTableStatement(AParent: TSQLElement): TSQLAlterTableStatement;
    function ParseCreateViewStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseCreateTriggerStatement(AParent: TSQLElement; IsAlter: Boolean): TSQLCreateOrAlterStatement;
    function ParseSetGeneratorStatement(AParent: TSQLElement) : TSQLSetGeneratorStatement;
    function ParseCreateDatabaseStatement(AParent: TSQLElement; IsAlter: Boolean ): TSQLCreateDatabaseStatement;
    function ParseCreateShadowStatement(AParent: TSQLElement; IsAlter: Boolean ): TSQLCreateShadowStatement;
    function ParseAlterDatabaseStatement(AParent: TSQLElement; IsAlter: Boolean ): TSQLAlterDatabaseStatement;
    function ParseSecondaryFile(AParent: TSQLElement): TSQLDatabaseFileInfo;
    function ParseDeclareFunctionStatement(AParent: TSQLElement): TSQLDeclareExternalFunctionStatement;
    function ParseDeclareStatement(AParent: TSQLElement): TSQLStatement;
    // GRANT parsing
    procedure ParseGranteeList(AParent: TSQLElement; List: TSQLElementList; AllowObject, AllowGroup,AllowPublic : Boolean; IsRevoke: Boolean = False);
    function ParseGrantExecuteStatement(AParent: TSQLElement): TSQLProcedureGrantStatement;
    function ParseGrantRoleStatement(AParent: TSQLElement): TSQLRoleGrantStatement;
    function ParseGrantTableStatement(AParent: TSQLElement): TSQLTableGrantStatement;
    // REVOKE parsing
    function ParseRevokeExecuteStatement(AParent: TSQLElement): TSQLProcedureRevokeStatement;
    function ParseRevokeRoleStatement(AParent: TSQLElement): TSQLRoleRevokeStatement;
    function ParseRevokeTableStatement(AParent: TSQLElement): TSQLTableRevokeStatement;
    // SELECT parsing
    function ParseExprAggregate(AParent: TSQLElement; EO: TExpressionOptions): TSQLAggregateFunctionExpression;
    procedure ParseFromClause(AParent: TSQLSelectStatement; AList: TSQLElementList);
    procedure ParseGroupBy(AParent: TSQLSelectStatement; AList: TSQLElementList);
    procedure ParseOrderBy(AParent: TSQLSelectStatement; AList: TSQLElementList);
    procedure ParseSelectFieldList(AParent: TSQLSelectStatement; AList: TSQLElementList; Singleton : Boolean);
    function ParseForUpdate(AParent: TSQLSelectStatement): TSQLElementList;
    function ParseSelectPlan(AParent: TSQLElement): TSQLSelectPlan;
    function ParseTableRef(AParent: TSQLSelectStatement): TSQLTableReference;
    procedure ParseIntoList(AParent: TSQLElement; List: TSQLElementList);
    // EXECUTE parsing
    function ParseExecuteProcedureStatement(AParent: TSQLElement): TSQLExecuteProcedureStatement;
    // Stored procedure parsing
    function ParseAssignStatement(AParent: TSQLElement): TSQLAssignStatement;
    function ParseExceptionStatement(AParent: TSQLElement): TSQLExceptionStatement;
    function ParseForStatement(AParent: TSQLElement): TSQLForStatement;
    function ParseIfStatement(AParent: TSQLElement): TSQLIFStatement;
    function ParsePostEventStatement(AParent: TSQLElement): TSQLPostEventStatement;
    procedure ParseProcedureParamList(AParent: TSQLElement; AList: TSQLElementList);
    procedure ParseCreateProcedureVariableList(AParent: TSQLElement; AList: TSQLElementList);
    function ParseProcedureStatement(AParent: TSQLElement): TSQLStatement;
    procedure ParseStatementBlock(AParent: TSQLElement; Statements: TSQLElementList);
    function ParseWhenStatement(AParent: TSQLElement): TSQLWhenStatement;
    function ParseWhileStatement(AParent: TSQLElement): TSQLWhileStatement;
  Public
    Constructor Create(AInput: TStream);
    Constructor Create(AScanner : TSQLScanner);
    Destructor Destroy; override;
    Function ParseSelectStatement(AParent : TSQLElement; Flags : TSelectFlags = []) : TSQLSelectStatement;
    Function ParseUpdateStatement(AParent : TSQLElement) : TSQLUpdateStatement;
    Function ParseInsertStatement(AParent : TSQLElement) : TSQLInsertStatement;
    Function ParseDeleteStatement(AParent : TSQLElement) : TSQLDeleteStatement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    // Parses both create and alter statements
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Function ParseCreateStatement(AParent : TSQLElement; IsAlter : Boolean = False) : TSQLCreateOrAlterStatement;
    Function ParseDropStatement(AParent : TSQLElement) : TSQLDropStatement;
    Function ParseRollbackStatement(AParent : TSQLElement) : TSQLRollbackStatement;
    Function ParseCommitStatement(AParent : TSQLElement) : TSQLCommitStatement;
    Function ParseSetStatement(AParent : TSQLElement) : TSQLStatement;
    Function ParseConnectStatement(AParent : TSQLElement) : TSQLConnectStatement;
    Function ParseGrantStatement(AParent: TSQLElement): TSQLGrantStatement;
    Function ParseRevokeStatement(AParent: TSQLElement): TSQLGrantStatement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    // Parse single element
    Function Parse : TSQLElement;
    // Parse script containing 1 or more elements
=======
    Function Parse : TSQLElement;
>>>>>>> graemeg/cpstrnew
=======
    Function Parse : TSQLElement;
>>>>>>> graemeg/cpstrnew
=======
    Function Parse : TSQLElement;
>>>>>>> graemeg/cpstrnew
=======
    Function Parse : TSQLElement;
>>>>>>> origin/cpstrnew
    Function ParseScript(AllowPartial : Boolean = False) : TSQLElementList;
    // Auxiliary stuff
    Function CurrentToken : TSQLToken;
    Function CurrentTokenString : String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    // Gets next token; also updates current token
    Function GetNextToken : TSQLToken;
    // Looks at next token without changing current token
=======
    Function GetNextToken : TSQLToken;
>>>>>>> graemeg/cpstrnew
=======
    Function GetNextToken : TSQLToken;
>>>>>>> graemeg/cpstrnew
=======
    Function GetNextToken : TSQLToken;
>>>>>>> graemeg/cpstrnew
=======
    Function GetNextToken : TSQLToken;
>>>>>>> origin/cpstrnew
    Function PeekNextToken : TSQLToken;
    Function PreviousToken : TSQLToken;
    Function IsEndOfLine : Boolean;
    function CurSource: String;
    Function CurLine : Integer;
    Function CurPos : Integer;
  end;

  { ESQLParser }

  ESQLParser = Class(Exception)
  private
    FCol: Integer;
    FFileName: String;
    FLine: Integer;
  Public
    Property Line : Integer Read FLine Write FLine;
    Property Col : Integer Read FCol Write FCol;
    Property FileName : String Read FFileName Write FFileName;
  end;

Function StringToSQLExtractElement(Const S : TSQLStringType; Out Res : TSQLExtractElement) : Boolean;

implementation

uses typinfo;

Resourcestring
  SerrUnmatchedBrace  = 'Expected ).';
  SErrCommaOrBraceExpected = 'Expected , or ).';
  SErrUnexpectedToken = 'Unexpected token: %s';
  SErrUnexpectedTokenOf = 'Unexpected token: %s, expected one of %s';
  SErrTokenMismatch   = 'Unexpected token: ''%s'', expected: ''%s''';
  SErrExpectedDBObject = 'Expected database object type. Got: ''%s''';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrDomainNotAllowed = 'Domain name not allowed in type definition.';
  SErrExpectedChar = 'Expected CHAR or CHARACTER, got "%s"';
  SErrVaryingNotAllowed = 'VARYING not allowed at this point.';
=======
  SErrDomainNotAllowed = 'Domain name not allowed in typ definition.';
  SErrExpectedChar = 'Expected CHAR or CHARACTER, got "%s"';
  SERRVaryingNotAllowed = 'VARYING not allowed at this point.';
>>>>>>> graemeg/cpstrnew
=======
  SErrDomainNotAllowed = 'Domain name not allowed in typ definition.';
  SErrExpectedChar = 'Expected CHAR or CHARACTER, got "%s"';
  SERRVaryingNotAllowed = 'VARYING not allowed at this point.';
>>>>>>> graemeg/cpstrnew
=======
  SErrDomainNotAllowed = 'Domain name not allowed in typ definition.';
  SErrExpectedChar = 'Expected CHAR or CHARACTER, got "%s"';
  SERRVaryingNotAllowed = 'VARYING not allowed at this point.';
>>>>>>> graemeg/cpstrnew
=======
  SErrDomainNotAllowed = 'Domain name not allowed in typ definition.';
  SErrExpectedChar = 'Expected CHAR or CHARACTER, got "%s"';
  SERRVaryingNotAllowed = 'VARYING not allowed at this point.';
>>>>>>> origin/cpstrnew
  SErrUnknownBooleanOp = 'Unknown boolean operation';
  SErrUnknownComparison = 'unknown Comparison operation';
  SErrIntegerExpected = 'Integer expression expected';
  SErrInvalidUseOfCollate = 'Invalid use of COLLATE';
  SErrCannotAlterGenerator = 'Alter generator statement unknown';
  SErrInvalidLiteral = 'Invalid literal: "%s"';
  SErrNoAggregateAllowed = 'Aggregate function not allowed.';
  SErrAsteriskOnlyInCount = '* allowed only in COUNT aggregate';
  SErrUpperOneArgument = 'Only one argument for UPPER allowed';
  SErrHavingWithoutGroupBy = 'HAVING without GROUP BY clause not allowed';
  SErrNoAsteriskInSingleTon = '* not allowed in singleton select';
  SErrUnionFieldCountMatch =  'Field count mismatch in select union : %d <> %d';
  SErrInvalidExtract = 'Invalid element for extract: %s';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrOuterWithout = 'OUTER without preceding LEFT, RIGHT or FULL';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

Function StringToSQLExtractElement(Const S : TSQLStringType; Out Res : TSQLExtractElement) : Boolean;

Var
  I : TSQLExtractElement;
  SU : TSQLStringTYpe;

begin
  Result:=False;
  SU:=Uppercase(S);
  For I:=Low(TSQLExtractElement) to High(TSQLExtractElement) do
    If ExtractElementNames[i]=SU then
      begin
      Res:=I;
      Exit(True);
      end;
end;

{ TSQLParser }

procedure TSQLParser.Expect(aToken: TSQLToken);
begin
  {$ifdef debugparser}  Writeln('Expecting : ',GetEnumName(TypeInfo(TSQLToken),Ord(AToken)), ' As string: ',TokenInfos[AToken]);{$endif debugparser}
  If (CurrentToken<>aToken) then
    Error(SerrTokenMismatch,[CurrenttokenString,TokenInfos[aToken]]);
end;

procedure TSQLParser.Expect(aTokens: TSQLTokens);

begin
  if not (CurrentToken in aTokens) then
    UnexpectedToken(aTokens);
end;


procedure TSQLParser.Consume(aToken: TSQLToken);
begin
  Expect(aToken);
  GetNextToken;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.Consume(aTokens: TSQLTokens);
begin
  Expect(aTokens);
  GetNextToken;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TSQLParser.CurSource: String;
begin
  Result:=FScanner.CurFilename;
end;

function TSQLParser.CurLine: Integer;
begin
  Result:=FScanner.CurRow;
end;

function TSQLParser.CurPos: Integer;
begin
  Result:=FScanner.CurColumn;
end;

procedure TSQLParser.Error(Msg: String);

Var
  ErrAt : String;
  E : ESQLParser;

begin
  If Assigned(FScanner) then
    If FScanner.CurFilename<>'' then
      ErrAt:=Format('Error: file "%s" line %d, pos %d: ',[FScanner.CurFileName,FScanner.CurRow,FScanner.CurColumn])
    else
      ErrAt:=Format('Error: line %d, pos %d: ',[FScanner.Currow,FScanner.CurColumn]);
   E:=ESQLParser.Create(ErrAt+Msg);
   If Assigned(FScanner) then
     begin
     E.Line:=FScanner.CurRow;
     E.Col:=FScanner.CurColumn;
     E.FileName:=FScanner.CurFilename;
     end;
   Raise E;
end;

procedure TSQLParser.Error(Fmt: String; Args: array of const);
begin
  Error(Format(Fmt,Args));
end;

function TSQLParser.CreateElement(AElementClass: TSQLElementClass;
  APArent: TSQLElement): TSQLElement;
begin
  Result:=AElementClass.Create(AParent);
  Result.Source:=CurSource;
  Result.SourceLine:=CurLine;
  Result.SourcePos:=CurPos;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseTableRef(AParent: TSQLSelectStatement
  ): TSQLTableReference;
=======
Function TSQLParser.ParseTableRef(AParent : TSQLSelectStatement) : TSQLTableReference;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTableRef(AParent : TSQLSelectStatement) : TSQLTableReference;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTableRef(AParent : TSQLSelectStatement) : TSQLTableReference;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTableRef(AParent : TSQLSelectStatement) : TSQLTableReference;
>>>>>>> origin/cpstrnew
Var
  T : TSQLSimpleTablereference;
  J  : TSQLJoinTableReference;

begin
   If (CurrentToken=tsqlBraceOpen) then
     begin
     GetNextToken;
     Result:=ParseTableRef(AParent);
     Consume(tsqlBraceClose)
     end
   else
     begin
     Expect(tsqlIdentifier);
     T:=TSQLSimpleTableReference(CreateElement(TSQLSimpleTableReference,AParent));
     Result:=T;
     T.ObjectName:=CreateIdentifier(T,CurrentTokenString);
     GetNextToken;
     If CurrentToken=tsqlBraceOpen then
       begin
       T.Params:=ParseValueList(AParent,[eoParamValue]);
       GetNextToken;
       end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     // Table aliases with and without AS keyword
     if (CurrentToken in [tsqlIdentifier,tsqlAs]) then
       begin
       if CurrentToken=tsqlAs then
         begin
         GetNextToken;
         Expect(tsqlIdentifier);
         end;
=======
     if (CurrentToken=tsqlIdentifier) then
       begin
>>>>>>> graemeg/cpstrnew
=======
     if (CurrentToken=tsqlIdentifier) then
       begin
>>>>>>> graemeg/cpstrnew
=======
     if (CurrentToken=tsqlIdentifier) then
       begin
>>>>>>> graemeg/cpstrnew
=======
     if (CurrentToken=tsqlIdentifier) then
       begin
>>>>>>> origin/cpstrnew
       T.AliasName:=CreateIdentifier(T,CurrentTokenString);
       GetNextToken;
       end;
     end;
   Repeat
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     If CurrentToken in [tsqlInner,tsqlFull,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight] then
=======
     If CurrentToken in [tsqlInner,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight] then
>>>>>>> graemeg/cpstrnew
=======
     If CurrentToken in [tsqlInner,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight] then
>>>>>>> graemeg/cpstrnew
=======
     If CurrentToken in [tsqlInner,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight] then
>>>>>>> graemeg/cpstrnew
=======
     If CurrentToken in [tsqlInner,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight] then
>>>>>>> origin/cpstrnew
       begin
       J:=TSQLJoinTableReference(CreateElement(TSQLJoinTableReference,AParent));
       J.Left:=Result;
       Result:=J;
       Case CurrentToken of
          tsqlInner : J.JoinType:=jtInner;
          tsqlJoin  : J.JoinType:=jtNone;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          tsqlFull  : J.JoinType:=jtFullOuter;
=======
          tsqlOuter : J.JoinType:=jtOuter;
>>>>>>> graemeg/cpstrnew
=======
          tsqlOuter : J.JoinType:=jtOuter;
>>>>>>> graemeg/cpstrnew
=======
          tsqlOuter : J.JoinType:=jtOuter;
>>>>>>> graemeg/cpstrnew
=======
          tsqlOuter : J.JoinType:=jtOuter;
>>>>>>> origin/cpstrnew
          tsqlLeft  : J.JoinType:=jtLeft;
          tsqlRight : J.JoinType:=jtRight;
       end;
       if CurrentToken<>tsqlJoin then
         GetNextToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       // Ignore OUTER in FULL OUTER, LEFT OUTER, RIGHT OUTER...:
       if CurrentToken=tsqlOuter then
         begin
         if PreviousToken in [tsqlFull, tsqlLeft, tSQLRight] then
           Consume(tsqlOuter)
         else
           Error(SErrOuterWithout);
         end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       Consume(tsqlJoin);
       J.Right:=ParseTableRef(AParent);
       Consume(tsqlOn);
       J.JoinClause:=ParseExprLevel1(J,[eoJOIN]);
       end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  until Not (CurrentToken in [tsqlInner,tsqlFull,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight]);
end;

procedure TSQLParser.ParseFromClause(AParent: TSQLSelectStatement;
  AList: TSQLElementList);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  until Not (CurrentToken in [tsqlInner,tsqlJoin,tsqlOuter,tsqlLeft,tsqlRight]);
end;

Procedure TSQLParser.ParseFromClause(AParent : TSQLSelectStatement; AList : TSQLElementList);
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

Var
  T : TSQLTableReference;
  Done : Boolean;

begin
  // On entry, we are on the FROM keyword.
  Consume(tsqlFrom);
  Repeat
    T:=ParseTableRef(AParent);
    AList.Add(T);
    Done:=(CurrentToken<>tsqlComma);
    If not Done then
      GetNextToken;
  until Done;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseSelectFieldList(AParent: TSQLSelectStatement;
  AList: TSQLElementList; Singleton: Boolean);
=======
Procedure TSQLParser.ParseSelectFieldList(AParent : TSQLSelectStatement; AList : TSQLElementList; Singleton : Boolean);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseSelectFieldList(AParent : TSQLSelectStatement; AList : TSQLElementList; Singleton : Boolean);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseSelectFieldList(AParent : TSQLSelectStatement; AList : TSQLElementList; Singleton : Boolean);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseSelectFieldList(AParent : TSQLSelectStatement; AList : TSQLElementList; Singleton : Boolean);
>>>>>>> origin/cpstrnew
Var
  F : TSQLSelectField;
  B : Boolean;

begin
  // On entry, we're on the token preceding the field list.
  B:=True;
  Repeat
    GetNextToken;
    If B then
      begin
      if (CurrentToken=tsqlDistinct) then
        begin
        AParent.Distinct:=True;
        GetNextToken;
        end
      else if (CurrentToken=tsqlAll) then
        begin
        AParent.All:=True;
        GetNextToken;
        end;
      B:=False;
      end;
    If (CurrentToken=tsqlMul) then
      begin
      If Singleton then
        Error(SErrNoAsteriskInSingleTon);
      AList.Add(CreateElement(TSQLSelectAsterisk,AParent));
      GetNextToken;
      end
    else
      begin
      F:=TSQLSelectField(CreateElement(TSQLSelectField,AParent));
      AList.Add(F);
      F.Expression:=ParseExprLevel1(AParent,[eoSelectvalue]);
      If CurrentToken in [tsqlAs,Tsqlidentifier] then
        begin
        If currentToken=tsqlAs then
          GetNextToken;
        Expect(tsqlIdentifier);
        F.AliasName:=CreateIdentifier(F,CurrentTokenString);
        GetNextToken;
        end;
      end;
    Expect([tsqlComma,tsqlFrom]);
  until (CurrentToken=tsqlFROM);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseGroupBy(AParent: TSQLSelectStatement;
  AList: TSQLElementList);
=======
Procedure TSQLParser.ParseGroupBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseGroupBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseGroupBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseGroupBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> origin/cpstrnew

Var
  N : TSQLStringType;

begin
  // On entry we're on the GROUP token.
  Consume(tsqlGroup);
  Expect(tsqlBy);
  Repeat
    GetNextToken;
    Expect(tsqlIdentifier);
    N:=CurrentTokenString;
    GetNextToken;
    If (CurrentToken=tsqlDot) then
      begin
      GetNextToken;
      Expect(tsqlIdentifier);
      N:=N+'.'+CurrentTokenString;
      GetNextToken;
      end;
    AList.Add(CreateIdentifier(AParent,N));

  until (CurrentToken<>tsqlComma);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseForUpdate(AParent: TSQLSelectStatement
  ): TSQLElementList;
=======
Function TSQLParser.ParseForUpdate(AParent : TSQLSelectStatement) : TSQLElementList;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForUpdate(AParent : TSQLSelectStatement) : TSQLElementList;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForUpdate(AParent : TSQLSelectStatement) : TSQLElementList;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForUpdate(AParent : TSQLSelectStatement) : TSQLElementList;
>>>>>>> origin/cpstrnew

begin
  // On entry we're on the FOR token.
  Consume(tsqlFor);
  Expect(tsqlUpdate);
  Result:=TSQLElementList.Create(True);
  try
    Repeat
      GetNextToken;
      Expect(tsqlIdentifier);
      Result.Add(CreateIdentifier(AParent,CurrentTokenString));
    until (CurrentToken<>tsqlComma);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseOrderBy(AParent: TSQLSelectStatement;
  AList: TSQLElementList);
=======
Procedure TSQLParser.ParseOrderBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseOrderBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseOrderBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseOrderBy(AParent : TSQLSelectStatement; AList : TSQLElementList);
>>>>>>> origin/cpstrnew

Var
  O : TSQLOrderByElement;
  F : TSQLElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  BuildToken : string;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
  // On entry we're on the ORDER token.
  Consume(tsqlOrder);
  Expect(tsqlBy);
  Repeat
    GetNextToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    // Deal with table.column notation:
    Case CurrentToken of
      tsqlIdentifier :
        begin
        BuildToken:=CurrentTokenString;
        If (PeekNextToken=tsqlDot) then
          begin
          GetNextToken; //past tsqlDot
          GetNextToken;
          Expect(tsqlIdentifier);
          BuildToken:=BuildToken+'.'+CurrentTokenString;
          end;
        F:=CreateIdentifier(AParent,BuildToken);
        end;
      tsqlIntegerNumber : //e.g. ORDER BY 1
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Case CurrentToken of
      tsqlIdentifier :
        F:=CreateIdentifier(AParent,CurrentTokenString);
      tsqlIntegerNumber :
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
        begin
        F:=TSQLIntegerLiteral(CreateElement(TSQLIntegerLiteral,AParent));
        TSQLIntegerLiteral(F).Value:=StrToInt(CurrentTokenString);
        end
    else
      UnexpectedToken([tsqlIdentifier,tsqlIntegerNumber]);
    end;
    try
      O:=TSQLOrderByElement(CreateElement(TSQLOrderByElement,APArent));
      AList.Add(O);
      O.Field:=F;
      F:=Nil;
    except
      FreeAndNil(F);
      Raise;
    end;
    GetNextToken;
    If (CurrentToken=tsqlCollate) then
       begin
       GetNextToken;
       Expect(tsqlidentifier);
       O.Collation:=CreateIdentifier(O,CurrentTokenString);
       GetNextToken;
       end;
    If (CurrentToken in [tsqlDesc,tsqlAsc,tsqlDescending,tsqlAscending]) then
      begin
      If (CurrentToken in [tsqlDesc,tsqlDescending]) then
        O.OrderBy:=obDescending
      else
        O.OrderBy:=obAscending;
      GetNextToken;
      end;
  until (CurrentToken<>tsqlComma);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseSelectPlan(AParent: TSQLElement): TSQLSelectPlan;
=======
Function TSQLParser.ParseSelectPlan(AParent : TSQLElement) : TSQLSelectPlan;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseSelectPlan(AParent : TSQLElement) : TSQLSelectPlan;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseSelectPlan(AParent : TSQLElement) : TSQLSelectPlan;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseSelectPlan(AParent : TSQLElement) : TSQLSelectPlan;
>>>>>>> origin/cpstrnew

Var
  E : TSQLSelectPlanExpr;
  I : TSQLSelectPlanItem;
  L : TSQLElementList;
  N : TSQLStringType;

begin
  Result:=Nil;
  try
    Case CurrentToken of
    tsqlIdentifier :
      begin
      If Not (AParent is TSQLSelectPlanExpr) then
        UnexpectedToken([tsqlJoin,tsqlmerge,tsqlSort]);
      N:=CurrentTokenString;
      Case GetNextToken of
      tsqlNatural:
        begin
        I:=TSQLSelectNaturalPlan(CreateElement(TSQLSelectNaturalPlan,AParent));
        Result:=I;
        end;
      tsqlIndex :
        begin
        I:=TSQLSelectIndexedPlan(CreateElement(TSQLSelectIndexedPlan,AParent));
        Result:=I;
        L:=TSQLSelectIndexedPlan(I).Indexes;
        GetNextToken;
        expect(tsqlBraceOpen);
        Repeat
          GetNextToken;
          Expect(tsqlidentifier);
          L.Add(CreateIdentifier(Result,CurrentTokenString));
          GetNextToken;
          Expect([tsqlComma,tsqlBraceClose]);
        until (CurrentToken=tsqlBraceClose);
        end;
       tsqlOrder:
         begin
         GetNextToken;
         expect(tsqlIdentifier);
         I:=TSQLSelectOrderedPlan(CreateElement(TSQLSelectOrderedPlan,AParent));
         Result:=I;
         TSQLSelectOrderedPlan(I).OrderIndex:=CreateIdentifier(I,CurrentTokenstring);
         end;
      else
        Unexpectedtoken([tsqlNatural,tsqlIndex,tsqlOrder]);
      end;
      I.TableName:=CreateIdentifier(I,N);
      end;
    tsqlJoin,
    tsqlmerge,
    tsqlSort,
    tsqlBraceOpen:
      begin
      E:=TSQLSelectPlanExpr(CreateElement(TSQLSelectPlanExpr,AParent));
      Result:=E;
      Case CurrentToken of
        tsqlJoin,
        tsqlBraceOpen : E.Jointype:=pjtJoin;
        tsqlSort  : E.JoinType:=pjtSort;
        tsqlMerge : E.JoinType:=pjtMerge;
      end;
      If (CurrentToken<>tsqlBraceOpen) then
        GetNextToken;
      expect(tsqlBraceOpen);
      repeat
        GetNextToken;
        E.Items.Add(ParseSelectPlan(E));
        Expect([tsqlComma,tsqlBraceClose]);
      until (CurrentToken=tsqlBraceClose);
      end;
    else
      UnexpectedToken([tsqlIdentifier,tsqlJoin,tsqlmerge,tsqlSort]);
    end;
    GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseSelectStatement(AParent: TSQLElement; Flags : TSelectFlags = []): TSQLSelectStatement;

begin
  // On entry, we're on the SELECT keyword
  Expect(tsqlSelect);
  Result:=TSQLSelectStatement(CreateElement(TSQLSelectStatement,AParent));
  try
    If (PeekNextToken=tsqlTransaction) then
      begin
      Consume(tsqlSelect);
      GetNextToken;
      Expect(TSQLIdentifier);
      Result.TransactionName:=CreateIdentifier(Result,CurrentTokenString);
      end;
    ParseSelectFieldList(Result,Result.Fields,sfSingleton in Flags);
    // On return, we are on the FROM keyword.
    ParseFromClause(Result,Result.Tables);
    If CurrentToken=tsqlWhere then
      begin
      GetNextToken;
      Result.Where:=ParseExprLevel1(Result,[eoWhereClause]);
      end;
    if CurrentToken=tsqlGroup then
      ParseGroupBy(Result,Result.GroupBy);
    if CurrentToken=tsqlHaving then
      begin
      If (Result.GroupBy.Count=0) then
        Error(SErrHavingWithoutGroupBy);
      GetNextToken;
      Result.Having:=ParseExprLevel1(Result,[eoHaving]);
      end;
    if (CurrentToken=tsqlUnion) then
      begin
      GetNextToken;
      If (CurrentToken=tsqlAll) then
        begin
        Result.UnionAll:=True;
        GetNextToken;
        end;
      Result.Union:=ParseSelectStatement(Result,Flags + [sfunion]);
      If (Result.Fields.count<>Result.Union.Fields.Count) then
        Error(SErrUnionFieldCountMatch,[Result.Fields.Count,Result.Union.Fields.Count])
      end;
    if (CurrentToken=tsqlPlan) then
      begin
      GetNextToken;
      Result.Plan:=ParseSelectPlan(Result);
      end;
    if not (sfUnion in Flags) then
      begin
      if (CurrentToken=tsqlOrder) then
        ParseOrderBy(Result,Result.OrderBy);
      if (CurrentToken=tsqlFOR) then
        Result.ForUpdate:=ParseForUpdate(Result);
      end;
    if (sfInto in Flags) then
       begin
       if (CurrentToken=tsqlInto) then
         begin
         Result.Into:=TSQLElementList.Create(true);
         ParseIntoList(Result,Result.Into);
         end;
       end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseUpdateStatement(AParent: TSQLElement
  ): TSQLUpdateStatement;

Var
  P : TSQLUpdatePair;
  N : String;
begin
  // On entry, we're on the UPDATE keyword
  Consume(tsqlUpdate);
  Expect(tsqlidentifier);
  Result:=TSQLUpdateStatement(CreateElement(TSQLUpdateStatement,AParent));
  try
    Result.TableName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    Expect(tsqlSet);
    Repeat
      GetNextToken;
      Expect(tsqlIdentifier);
      P:=TSQLUpdatePair(CreateElement(TSQLUpdatePair,Result));
      Result.Values.Add(P);
      N:=CurrentTokenString;
      GetNextToken;
      If (CurrentToken=tsqlDot) then
        begin
        GetNextToken;
        Expect(TSQLIdentifier);
        N:=N+'.'+CurrentTokenString;
        GetNextToken;
        end;
      Consume(tsqlEq);
      P.FieldName:=CreateIdentifier(P,N);
      P.Value:=ParseExprLevel1(P,[eoFieldValue]);
    until (CurrentToken<>tsqlComma);
    If (CurrentToken=tsqlWhere) then
      begin
      GetNextToken;
      Result.WhereClause:=ParseExprLevel1(P,[eoWhereClause]);
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseInsertStatement(AParent: TSQLElement): TSQLInsertStatement;

begin
  // On entry, we're on the INSERT statement
  Consume(tsqlInsert);
  Consume(tsqlInto);
  Expect(tsqlidentifier);
  Result:=TSQLInsertStatement(CreateElement(TSQLinsertStatement,AParent));
  try
    Result.TableName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    If CurrentToken=tsqlBraceOpen then
      begin
      Result.Fields:=TSQLElementList.Create(True);
      Repeat
        GetNextToken;
        Expect(tsqlIdentifier);
        Result.Fields.Add(CreateIdentifier(Result,CurrentTokenString));
        GetNextToken;
        Expect([tsqlBraceClose,tsqlComma]);
      Until (CurrentToken=tsqlBraceClose);
      GetNextToken;
      end;
    Case CurrentToken of
      tsqlSelect :
        Result.Select:=ParseSelectStatement(Result);
      tsqlValues :
        begin
        GetNextToken;
        Result.Values:=ParsevalueList(Result,[eoFieldValue]);
        GetNextToken; // consume )
        end;
    else
      UnexpectedToken([tsqlselect,tsqlValues]);
    end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseDeleteStatement(AParent: TSQLElement
  ): TSQLDeleteStatement;
begin
  // On entry, we're on the DELETE token.
  consume(tsqlDelete);
  consume(tsqlFrom);
  Expect(tsqlidentifier);
  Result:=TSQLDeleteStatement(CreateElement(TSQLDeleteStatement,AParent));
  try
    Result.TableName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    if CurrentToken=tsqlIdentifier then
      begin
      Result.AliasName:=CreateIdentifier(Result,CurrentTokenString);
      GetNextToken;
      end;
    if CurrentToken=tsqlwhere then
      begin
      Consume(tsqlWhere);
      Result.WhereClause:=ParseExprLevel1(Result,[eoWhereClause]);
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseTableFieldDef(AParent: TSQLElement): TSQLTableFieldDef;
=======
Function TSQLParser.ParseTableFieldDef(AParent : TSQLElement) : TSQLTableFieldDef;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTableFieldDef(AParent : TSQLElement) : TSQLTableFieldDef;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTableFieldDef(AParent : TSQLElement) : TSQLTableFieldDef;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTableFieldDef(AParent : TSQLElement) : TSQLTableFieldDef;
>>>>>>> origin/cpstrnew
begin
  // on entry, we're on the field name
  Result:=TSQLTableFieldDef(CreateElement(TSQLTableFieldDef,AParent));
  try
    Result.FieldName:=CreateIdentifier(Result,CurrentTokenString);
    if PeekNextToken = tsqlComputed then
      begin
      GetNextToken;
      Consume(tsqlComputed);
      If CurrentToken=tsqlBy then
        GetNextToken;
      Consume(tsqlBraceopen);
      Result.ComputedBy:=ParseExprLevel1(Result,[eoComputedBy]);
      Consume(tsqlBraceClose);
      end
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    else //not computed, regular field
=======
    else
>>>>>>> graemeg/cpstrnew
=======
    else
>>>>>>> graemeg/cpstrnew
=======
    else
>>>>>>> graemeg/cpstrnew
=======
    else
>>>>>>> origin/cpstrnew
      Result.FieldType:=ParseTypeDefinition(Result,[ptfAllowDomainName,ptfAllowConstraint,ptfTableFieldDef]);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseTableConstraint(AParent: TSQLElement
  ): TSQLTableConstraintDef;

  Procedure ParseFieldList(R : TSQLTableFieldsConstraintDef);

  begin
    GetNextToken;
    Consume(tsqlBraceOpen);
    ParseIdentifierList(AParent,R.FieldList);
//    Consume(tsqlBraceClose);
  end;

Var
  N : TSQLStringType;
  K : TSQLTableForeignKeyConstraintDef;

begin
  If CurrentToken=tsqlConstraint then
    begin
    GetNextToken;
    Expect(tsqlIdentifier);
    N:=CurrentTokenString;
    GetNextToken
    end;
  Result:=Nil;
  try
    Case CurrentToken of
    tsqlUnique :
      begin
      Result:=TSQLTableUniqueConstraintDef(CreateElement(TSQLTableUniqueConstraintDef,AParent));
      ParseFieldList(TSQLTableFieldsConstraintDef(Result));
      end;
    tsqlPrimary :
      begin
      GetNextToken;
      Expect(tsqlKey);
      Result:=TSQLTablePrimaryKeyConstraintDef(CreateElement(TSQLTablePrimaryKeyConstraintDef,AParent));
      ParseFieldList(TSQLTableFieldsConstraintDef(Result));
      end;
    tsqlForeign :
      begin
      GetNextToken;
      Expect(tsqlKey);
      K:=TSQLTableForeignKeyConstraintDef(CreateElement(TSQLTableForeignKeyConstraintDef,AParent));
      Result:=K;
      ParseFieldList(TSQLTableFieldsConstraintDef(Result));
      Expect(tsqlReferences);
      K.Definition:=ParseForeignKeyDefinition(K);
      end;
    tsqlCheck:
      begin
      Result:=TSQLTableCheckConstraintDef(CreateElement(TSQLTableCheckConstraintDef,AParent));
      TSQLTableCheckConstraintDef(Result).Check:=ParseCheckConstraint(Result,True);
      end
    else
      UnexpectedToken([tsqlUnique,tsqlPrimary,tsqlForeign,tsqlCheck]);
    end;
    If (N<>'') then
      Result.ConstraintName:=CreateIdentifier(Result,N);
  //  GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseCreateTableStatement(AParent: TSQLElement): TSQLCreateOrAlterStatement;

Var
  C : TSQLCreateTableStatement;
  HC : Boolean;

begin
  // On enter, we're on the TABLE token.
  Consume(tsqlTable);
  C:=TSQLCreateTableStatement(CreateElement(TSQLCreateTableStatement,AParent));
  try
    Expect(tsqlIdentifier);
    C.ObjectName:=CreateIdentifier(C,CurrentTokenstring);
    GetNextToken;
    If (CurrentToken=tsqlExternal) then
      begin
      GetNextToken;
      If (CurrentToken=tsqlFile) then
        GetNextToken;
      Expect(tsqlString);
      C.ExternalFileName:=CreateLiteral(C) as TSQLStringLiteral;
      GetNextToken;
      end;
    Expect(tsqlBraceOpen);
    HC:=False;
    Repeat
      GetNextToken;
      Case CurrentToken of
        tsqlIdentifier :
          begin
          if HC then
            UnexpectedToken;
          C.FieldDefs.Add(ParseTableFieldDef(C));
          end;
        tsqlCheck,
        tsqlConstraint,
        tsqlForeign,
        tsqlPrimary,
        tsqlUnique:
          begin
          C.Constraints.Add(ParseTableConstraint(C));
          HC:=true;
          end
      else
         UnexpectedToken([tsqlIdentifier,tsqlCheck, tsqlConstraint,tsqlForeign,tsqlPrimary,tsqlUnique]);
      end;
      expect([tsqlBraceClose,tsqlComma]);
    until (CurrentToken=tsqlBraceClose);
    GetNextToken;
    Result:=C;
  except
    FreeandNil(C);
    Raise;
  end;
end;

function TSQLParser.ParseDropTableElement(AParent : TSQLElement) : TSQLDropTableElementOperation;

Var
  C : Boolean;
begin
  // On entry, we are on DROP token
  C:=(GetNextToken=tsqlConstraint);
  If C then
    GetNextToken;
  Expect(tsqlidentifier);
  If C then
    Result:=TSQLDropTableConstraintOperation(CreateElement(TSQLDropTableConstraintOperation,AParent))
  else
    Result:=TSQLDropTableFieldOperation(CreateElement(TSQLDropTableFieldOperation,AParent));
  Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
  GetNextToken;
end;

function TSQLParser.ParseAddTableElement(AParent : TSQLElement) : TSQLAlterTableAddElementOperation;

begin
  Result:=Nil;
  try
    Case GetNextToken of
      tsqlIdentifier :
        begin
        Result:=TSQLAlterTableAddElementOperation(CreateElement(TSQLAlterTableAddFieldOPeration,AParent));
        Result.Element:=ParseTableFieldDef(Result);
        end;
      tsqlCheck,
      tsqlConstraint,
      tsqlForeign,
      tsqlPrimary,
      tsqlUnique:
        begin
        Result:=TSQLAlterTableAddElementOperation(CreateElement(TSQLAlterTableAddConstraintOperation,AParent));
        Result.Element:=ParseTableConstraint(Result);
        end
    else
      UnexpectedToken([tsqlIdentifier,tsqlCheck, tsqlConstraint,tsqlForeign,tsqlPrimary,tsqlUnique]);
    end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseAlterTableElement(AParent : TSQLElement) : TSQLAlterTableOperation;

Var
  N : TSQLStringType;

begin
  Result:=Nil;
  If GetnextToken=tsqlColumn then
    GetNextToken;
  expect(tsqlidentifier);
  N:=CurrentTokenString;
  try
    Case GetNextToken of
      tsqlTo :
        begin
        GetNextToken;
        Result:=TSQLAlterTableOperation(CreateElement(TSQLAlterTableFieldNameOperation,AParent));
        TSQLAlterTableFieldNameOperation(Result).NewName:=CreateIdentifier(Result,CurrentTokenString);
        GetNextToken;
        end;
      tsqltype:
        begin
        Result:=TSQLAlterTableOperation(CreateElement(TSQLAlterTableFieldTypeOperation,AParent));
        TSQLAlterTableFieldTypeOperation(Result).NewType:= ParseTypeDefinition(Result,[ptfAllowDomainName,ptfAllowConstraint,ptfTableFieldDef]);
        end;
      tsqlPosition:
        begin
        GetNextToken;
        Expect(tsqlIntegerNumber);
        Result:=TSQLAlterTableOperation(CreateElement(TSQLAlterTableFieldPositionOperation,AParent));
        TSQLAlterTableFieldPositionOperation(Result).NewPosition:=StrToInt(CurrentTokenString);
        GetNextToken;
        end
    else
      UnexpectedToken([tsqlTo,tsqltype,tsqlPosition]);
    end;
    Result.ObjectName:=CreateIdentifier(Result,N);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseAlterTableStatement(AParent: TSQLElement): TSQLAlterTableStatement;

begin
  // On enter, we're on the TABLE token.
  Consume(tsqlTable);
  Result:=TSQLAlterTableStatement(CreateElement(TSQLAlterTableStatement,AParent));
  try
    Expect(tsqlIdentifier);
    Result.ObjectName:=CreateIdentifier(Result,CurrentTokenstring);
    Repeat
      GetNextToken;
      Case CurrentToken of
        tsqlAdd:
          begin
          Result.Operations.Add(ParseAddTableElement(Result));
          end;
        tsqlAlter:
          begin
          Result.Operations.Add(ParseAlterTableElement(Result));
          end;
        tsqlDrop :
          begin
          Result.Operations.add(ParseDropTableElement(Result));
          end;
      else
         UnexpectedToken([tsqlAdd,tsqlAlter,tsqlDrop]);
      end;
    until (CurrentToken<>tsqlComma);
  except
    FreeandNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseCreateIndexStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;

Var
  O : TIndexOptions;
  C : TSQLCreateIndexStatement;
  A : TSQLAlterIndexStatement;
begin
  O:=[];
  // On enter, we're on the UNIQUE, ASCENDING, DESCENDING or INDEX token
  If IsAlter then
    begin
    expect(tsqlIndex);
    Consume(tsqlIndex);
    A:=TSQLAlterIndexStatement(CreateElement(TSQLAlterIndexStatement,APArent));
    try
      Expect(tsqlIdentifier);
      A.ObjectName:=CreateIdentifier(A,CurrentTokenString);
      GetNextToken;
      Expect([tsqlActive,tsqlInactive]);
      A.Inactive:=CurrentToken=tsqlInactive;
      GetNextToken; // Token after ) or (in)Active
      Result:=A;
    except
      FReeAndNil(A);
      Raise;
    end;
    end
  else
    begin
    C:=TSQLCreateIndexStatement(CreateElement(TSQLCreateIndexStatement,APArent));
    try
      If (CurrentToken=tsqlUnique) then
        begin
        GetNextToken;
        Include(O,ioUnique);
        end;
      If (CurrentToken=tsqlAscending) then
        begin
        GetNextToken;
        Include(O,ioAscending);
        end
      else If (CurrentToken=tsqlDescending) then
        begin
        GetNextToken;
        Include(O,ioDescending);
        end;
      C.Options:=O;
      Consume(tsqlIndex);
      Expect(tsqlIdentifier);
      C.ObjectName:=CreateIdentifier(C,CurrentTokenString);
      GetNextToken;
      Consume(tsqlOn);
      Expect(tsqlIdentifier);
      C.TableName:=Createidentifier(C,CurrentTokenString); // name of table
      GetNextToken;
      Consume(tsqlBraceOpen);
      ParseIdentifierList(C,C.FieldNames);
      Result:=C;
    except
      FreeAndNil(C);
      Raise;
    end;
    end;
end;

function TSQLParser.ParseCreateViewStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;

Var
  V : TSQLCreateViewStatement;

begin
  // on entry, we're on the VIEW token.
  If IsAlter then
    UnexpectedToken;
  Result:=Nil;
  Consume(tsqlView);
  Expect(tsqlIdentifier);
  V:=TSQLCreateViewStatement(CreateElement(TSQLCreateViewStatement,APArent));
  Result:=V;
  try
    V.ObjectName:=CreateIdentifier(V,CurrentTokenString);
    GetNextToken;
    If (CurrentToken=tsqlBraceOpen) then
      begin
      GetNextToken;
      ParseIdentifierList(Result,V.Fields);
      end;
    Consume(tsqlAs);
    V.Select:=ParseSelectStatement(V,[]);
    If (CurrentToken=tsqlWith) then
      begin
      GetNextToken;
      Consume(tsqlCheck);
      Consume(tsqlOption);
      V.WithCheckOption:=True;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseProcedureParamList(AParent: TSQLElement;
  AList: TSQLElementList);
=======
Procedure TSQLParser.ParseProcedureParamList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseProcedureParamList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseProcedureParamList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseProcedureParamList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> origin/cpstrnew

Var
  P : TSQLProcedureParamDef;

begin
  // On Entry, we're on the ( token
  Repeat
    GetNextToken;
    Expect(tsqlIdentifier);
    P:=TSQLProcedureParamDef(CreateElement(TSQLProcedureParamDef,AParent));
    try
      Alist.Add(P);
    except
      P.free;
      Raise;
    end;
    P.ParamName:=CreateIdentifier(P,CurrentTokenString);
    // Typedefinition will go to next token
    P.ParamType:=ParseTypeDefinition(P,[ptProcedureParam]);
  Until (CurrentToken<>tsqlComma);
  Consume(tsqlBraceClose);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseCreateProcedureVariableList(AParent: TSQLElement;
  AList: TSQLElementList);
=======
Procedure TSQLParser.ParseCreateProcedureVariableList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseCreateProcedureVariableList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseCreateProcedureVariableList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseCreateProcedureVariableList(AParent: TSQLElement; AList : TSQLElementList);
>>>>>>> origin/cpstrnew

Var
  P : TSQLProcedureParamDef;

begin
  // On Entry, we're on the DECLARE token
  Repeat
    Consume(tsqlDeclare);
    Consume(tsqlVariable);
    Expect(tsqlIdentifier);
    P:=TSQLProcedureParamDef(CreateElement(TSQLProcedureParamDef,AParent));
    Try
      AList.Add(P);
    except
      P.Free;
      Raise;
    end;
    P.ParamName:=CreateIdentifier(P,CurrentTokenString);
    // Typedefinition will go to next token
    P.ParamType:=ParseTypeDefinition(P,[ptProcedureParam]);
    Consume(tsqlSemicolon);
  Until (CurrentToken<>tsqlDeclare);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseIfStatement(AParent: TSQLElement): TSQLIFStatement;
=======
Function TSQLParser.ParseIfStatement(AParent : TSQLElement) : TSQLIFStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseIfStatement(AParent : TSQLElement) : TSQLIFStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseIfStatement(AParent : TSQLElement) : TSQLIFStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseIfStatement(AParent : TSQLElement) : TSQLIFStatement;
>>>>>>> origin/cpstrnew

Var
  Pt : TSQLToken;

begin
  // On Entry, we're on the IF token
  Consume(tsqlIf);
  Consume(tsqlBraceOpen);
  Result:=TSQLIFStatement(CreateElement(TSQLIFStatement,AParent));
  try
    Result.Condition:=ParseExprLevel1(AParent,[eoIF]);
    Consume(tsqlBraceClose);
    Consume(tsqlThen);
    Result.TrueBranch:=ParseProcedureStatement(Result);
    If (CurrentToken=tsqlSemicolon) and (PeekNextToken=tsqlElse) then
      begin
      PT:=CurrentToken;
      GetNextToken;
      end
    else if (CurrentToken=tsqlElse) then
      if not (PreviousToken=tsqlEnd) then
        UnexpectedToken;
    If CurrentToken=tsqlElse then
      begin
      GetNextToken;
      Result.FalseBranch:=ParseProcedureStatement(Result);
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;


procedure TSQLParser.ParseIntoList(AParent : TSQLElement; List : TSQLElementList);

begin
  // On Entry, we're on the INTO token
  Repeat
    GetNextToken;
    If (currentToken=tsqlColon) then
      Consume(tsqlColon);
    Expect(tsqlIdentifier);
    List.Add(CreateIdentifier(AParent,CurrentTokenString));
    GetNextToken;
  Until (CurrentToken<>tsqlComma);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseForStatement(AParent: TSQLElement): TSQLForStatement;
=======
Function TSQLParser.ParseForStatement(AParent : TSQLElement) : TSQLForStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForStatement(AParent : TSQLElement) : TSQLForStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForStatement(AParent : TSQLElement) : TSQLForStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForStatement(AParent : TSQLElement) : TSQLForStatement;
>>>>>>> origin/cpstrnew

begin
  // On Entry, we're on the FOR token
  Consume(tsqlFor);
  Expect(tsqlSelect);
  Result:=TSQLForStatement(CreateElement(TSQLForStatement,AParent));
  try
    Result.Select:=ParseSelectStatement(Result,[]);
    Expect(tsqlInto);
    ParseIntoList(Result,Result.FieldList);
    Consume(tsqlDo);
    Result.Statement:=ParseProcedureStatement(Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExceptionStatement(AParent: TSQLElement
  ): TSQLExceptionStatement;
=======
Function TSQLParser.ParseExceptionStatement(AParent : TSQLElement) : TSQLExceptionStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExceptionStatement(AParent : TSQLElement) : TSQLExceptionStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExceptionStatement(AParent : TSQLElement) : TSQLExceptionStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExceptionStatement(AParent : TSQLElement) : TSQLExceptionStatement;
>>>>>>> origin/cpstrnew

begin
  // On Entry, we're on the EXCEPTION token
  Consume(tsqlException);
  Expect(tsqlIdentifier);
  Result:=TSQLExceptionStatement(CreateElement(TSQLExceptionStatement,AParent));
  try
    Result.ExceptionName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseAssignStatement(AParent: TSQLElement
  ): TSQLAssignStatement;
=======
Function TSQLParser.ParseAssignStatement(AParent : TSQLElement) : TSQLAssignStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseAssignStatement(AParent : TSQLElement) : TSQLAssignStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseAssignStatement(AParent : TSQLElement) : TSQLAssignStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseAssignStatement(AParent : TSQLElement) : TSQLAssignStatement;
>>>>>>> origin/cpstrnew

Var
  N  : TSQLStringType;

begin
  // On entry, we're on the identifier token;
  expect(tsqlIdentifier);
  Result:=TSQLAssignStatement(CreateElement(TSQLAssignStatement,AParent));
  try
    N:=CurrentTokenString;
    GetNextToken;
    If (CurrentToken=tsqlDot) and (Uppercase(N)='NEW') then
      begin
      GetNextToken;
      Expect(tsqlIdentifier);
      N:=N+'.'+CurrentTokenString;
      GetNextToken;
      end;
    Result.Variable:=CreateIdentifier(Result,N);
    Consume(tsqlEq);
    Result.Expression:=ParseExprLevel1(Result,[]);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParsePostEventStatement(AParent: TSQLElement
  ): TSQLPostEventStatement;
=======
Function TSQLParser.ParsePostEventStatement(AParent : TSQLElement) : TSQLPostEventStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParsePostEventStatement(AParent : TSQLElement) : TSQLPostEventStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParsePostEventStatement(AParent : TSQLElement) : TSQLPostEventStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParsePostEventStatement(AParent : TSQLElement) : TSQLPostEventStatement;
>>>>>>> origin/cpstrnew

begin
  // On Entry, we're on the POST_EVENT token
  Consume(tsqlPostEvent);
  Result:=TSQLPostEventStatement(CreateElement(TSQLPostEventStatement,AParent));
  try
    Case CurrentToken of
      tsqlIdentifier : Result.ColName:=CreateIdentifier(Result,CurrentTokenString);
      tsqlString : Result.EventName:=CurrentTokenString;
    else
      UnexpectedToken([tsqlIdentifier,tsqlString]);
    end;
    GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseWhileStatement(AParent: TSQLElement
  ): TSQLWhileStatement;
=======
Function TSQLParser.ParseWhileStatement(AParent : TSQLElement) : TSQLWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseWhileStatement(AParent : TSQLElement) : TSQLWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseWhileStatement(AParent : TSQLElement) : TSQLWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseWhileStatement(AParent : TSQLElement) : TSQLWhileStatement;
>>>>>>> origin/cpstrnew

begin
  // On entry, we're on the WHILE Token
  Consume(tsqlWhile);
  Consume(tsqlBraceOpen);
  Result:=TSQLWhileStatement(CreateElement(TSQLWhileStatement,AParent));
  try
    Result.Condition:=ParseExprLevel1(Result,[eoIF]);
    Consume(tsqlBraceClose);
    Consume(tsqlDO);
    Result.Statement:=ParseProcedureStatement(Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseWhenStatement(AParent: TSQLElement): TSQLWhenStatement;
=======
Function TSQLParser.ParseWhenStatement(AParent : TSQLElement) : TSQLWhenStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseWhenStatement(AParent : TSQLElement) : TSQLWhenStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseWhenStatement(AParent : TSQLElement) : TSQLWhenStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseWhenStatement(AParent : TSQLElement) : TSQLWhenStatement;
>>>>>>> origin/cpstrnew

Var
  E : TSQLWhenException;
  S : TSQLWhenSQLError;
  G : TSQLWhenGDSError;

begin
  Consume(tsqlWhen);
  Result:=TSQLWhenStatement(CreateElement(TSQLWhenStatement,AParent));
  try
    if (CurrentToken=tsqlAny) then
      begin
      Result.AnyError:=True;
      GetNextToken
      end
    else
      Repeat
        if (Result.Errors.Count>0) then
          GetNextToken;
        Case CurrentToken of
          tsqlException:
            begin
            GetNextToken;
            Expect(tsqlIdentifier);
            E:=TSQLWhenException(CreateElement(TSQLWhenException,AParent));
            E.ExceptionName:=CreateIdentifier(E,CurrentTokenString);
            Result.Errors.Add(E);
            end;
          tsqlSQLCode:
            begin
            GetNextToken;
            Expect(tsqlIntegerNumber);
            S:=TSQLWhenSQLError(CreateElement(TSQLWhenSQLError,AParent));
            S.ErrorCode:=StrToInt(CurrentTokenString);
            Result.Errors.Add(S);
            end;
          tsqlGDSCODE:
            begin
            GetNextToken;
            Expect(tsqlIntegerNumber);
            G:=TSQLWhenGDSError(CreateElement(TSQLWhenGDSError,AParent));
            G.GDSErrorNumber:=StrToInt(CurrentTokenString);
            Result.Errors.Add(G);
            end;
        else
          UnexpectedToken([tsqlException,tsqlSQLCode,tsqlGDSCODE]);
        end;
        GetNextToken;
      until (CurrentToken<>tsqlComma);
    consume(tsqlDo);
    Result.Statement:=ParseProcedureStatement(Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseProcedureStatement(AParent: TSQLElement
  ): TSQLStatement;
=======
Function TSQLParser.ParseProcedureStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseProcedureStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseProcedureStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseProcedureStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> origin/cpstrnew

begin
  Result:=Nil;
  Case CurrentToken of
    tsqlBegin :
      begin
      Result:=TSQLStatementBlock(CreateElement(TSQLStatementBlock,AParent));
      ParseStatementBlock(Result,TSQLStatementBlock(Result).Statements);
      end;
    tsqlIf         : Result:=ParseIfStatement(AParent);
    tsqlFor        : Result:=ParseForStatement(AParent);
    tsqlException  : Result:=ParseExceptionStatement(AParent);
    tsqlIdentifier : Result:=ParseAssignStatement(AParent);
    tsqlExecute    : Result:=ParseExecuteProcedureStatement(AParent);
    tsqlExit       : begin
                     Result:=TSQLExitStatement(CreateElement(TSQLExitStatement,AParent));
                     GetNextToken;
                     end;
    tsqlSuspend    : begin
                     Result:=TSQLSuspendStatement(CreateElement(TSQLSuspendStatement,AParent));
                     GetNextToken;
                     end;
    tsqlPostEvent  : Result:=ParsePostEventStatement(AParent);
    tsqlWhile      : Result:=ParseWhileStatement(AParent);
    tsqlWhen       : Result:=ParseWhenStatement(AParent);
    tsqlSelect     : Result:=ParseSelectStatement(AParent,[sfInto]);
    tsqlInsert     : Result:=ParseInsertStatement(AParent);
    tsqlDelete     : Result:=ParseDeleteStatement(AParent);
    tsqlUpdate     : Result:=ParseUpdateStatement(AParent);
  else
    UnexpectedToken;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseStatementBlock(AParent: TSQLElement;
  Statements: TSQLElementList);
=======
Procedure TSQLParser.ParseStatementBlock(AParent: TSQLElement; Statements : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseStatementBlock(AParent: TSQLElement; Statements : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseStatementBlock(AParent: TSQLElement; Statements : TSQLElementList);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseStatementBlock(AParent: TSQLElement; Statements : TSQLElementList);
>>>>>>> origin/cpstrnew

Var
  S: TSQLStatement;

begin
  Consume(tsqlBegin);
  While (CurrentToken<>tsqlEnd) do
    begin
    S:=ParseProcedureStatement(AParent);
    Statements.Add(S);
    if not (PreviousToken=tsqlEnd) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Consume([tsqlSemicolon]);
=======
      Consume(tsqlSemicolon);
>>>>>>> graemeg/cpstrnew
=======
      Consume(tsqlSemicolon);
>>>>>>> graemeg/cpstrnew
=======
      Consume(tsqlSemicolon);
>>>>>>> graemeg/cpstrnew
=======
      Consume(tsqlSemicolon);
>>>>>>> origin/cpstrnew
    end;
  Consume(tsqlEnd);
end;

function TSQLParser.ParseCreateProcedureStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;

Var
  P : TSQLAlterCreateProcedureStatement;

begin
  // On entry, we're on the PROCEDURE statement.
  Consume(tsqlProcedure);
  expect(tsqlIdentifier);
  If IsAlter then
    P:=TSQLAlterProcedureStatement(CreateElement(TSQLAlterProcedureStatement,AParent))
  else
    P:=TSQLCreateProcedureStatement(CreateElement(TSQLCreateProcedureStatement,AParent));
  Result:=P;
  try
    Result.ObjectName:=CreateIdentifier(P,CurrentTokenString);
    GetNextToken;
    If (CurrentToken=tsqlBraceOpen) then
      ParseProcedureParamList(Result,P.InputVariables);
    If (CurrentToken=tsqlReturns) then
      begin
      GetNextToken;
      expect(tsqlBraceOpen);
      ParseProcedureParamList(Result,P.OutputVariables);
      end;
    Consume(tsqlAs);
    if (CurrentToken=tsqlDeclare) then
      ParseCreateProcedureVariableList(Result,P.LocalVariables);
    expect(tsqlBegin);
    ParseStatementBlock(Result,P.Statements);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseCreateGeneratorStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;
begin
  GetNextToken;
  Expect(tsqlIdentifier);
  If IsAlter then
    Error(SErrCannotAlterGenerator);
  Result:=TSQLCreateOrAlterStatement(CreateElement(TSQLCreateGeneratorStatement,AParent));
  try
    Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
  except
    FreeAndNil(Result);
    Raise;
  end;
  GetNextToken; // Comma;
end;

function TSQLParser.ParseCreateRoleStatement(AParent: TSQLElement;
  IsAlter: Boolean): TSQLCreateOrAlterStatement;
begin
  If IsAlter then
    UnexpectedToken; // no ALTER ROLE
  GetNextToken;
  Expect(tsqlIdentifier);
  Result:=TSQLCreateOrAlterStatement(CreateElement(TSQLCreateRoleStatement,AParent));
  Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
  GetNextToken; // Comma;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseCharTypeDefinition(out DT: TSQLDataType; out
  Len: Integer; out ACharset: TSQLStringType);
=======
Procedure TSQLParser.ParseCharTypeDefinition(Out DT : TSQLDataType; Out Len : Integer; Out ACharset : TSQLStringType);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseCharTypeDefinition(Out DT : TSQLDataType; Out Len : Integer; Out ACharset : TSQLStringType);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseCharTypeDefinition(Out DT : TSQLDataType; Out Len : Integer; Out ACharset : TSQLStringType);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseCharTypeDefinition(Out DT : TSQLDataType; Out Len : Integer; Out ACharset : TSQLStringType);
>>>>>>> origin/cpstrnew

begin
  Len:=0;
  Case CurrentToken of
    tsqlNCHAR     : dt:=sdtNchar;
    tsqlVarChar   : dt:=sdtVarChar;
    tsqlCharacter,
    tsqlChar      : dt:=sdtChar;
    tsqlCString   : dt:=sdtCstring;
    tsqlNational  :
     begin
     dt:=sdtNChar;
     GetNextToken;
     expect([tsqlCharacter,tsqlChar]);
     end;
  end;
  GetNextToken; // VARYING, Start of size, CHARACTER SET or end
  If (CurrentToken=tsqlVarying) then  // CHAR VARYING or CHARACTER VARYING;
    begin
    If (dt in [sdtNCHAR,sdtChar]) then
      begin
      if dt=sdtNCHAR then
        dt:=sdtNVARCHAR
      else
        dt:=sdtVarChar;
      GetNextToken
      end
    else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Error(SErrVaryingNotAllowed);
=======
      Error(SERRVaryingNotAllowed);
>>>>>>> graemeg/cpstrnew
=======
      Error(SERRVaryingNotAllowed);
>>>>>>> graemeg/cpstrnew
=======
      Error(SERRVaryingNotAllowed);
>>>>>>> graemeg/cpstrnew
=======
      Error(SERRVaryingNotAllowed);
>>>>>>> origin/cpstrnew
    end;
  If (CurrentToken=tsqlBraceOpen) then  // (LEN)
    begin
    GetNextToken;
    Expect(tsqlIntegerNumber);
    len:=StrToInt(CurrentTokenString);
    GetNextToken;
    Expect(tsqlBraceClose);
    GetNextToken;
    end
  else if (dt=sdtCstring) then
    UnexpectedToken;
  if (CurrentToken=tsqlCharacter) then // Character SET NNN
    begin
    if (dt=sdtCstring) then
      UnexpectedToken;
    GetNextToken;
    Consume(tsqlSet);
    Expect(tsqlIdentifier);
    ACharSet:=CurrentTokenString;
    GetNextToken;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLParser.ParseBlobDefinition(var ASegmentSize, ABlobType: Integer;
  var ACharset: TSQLStringType);
=======
Procedure TSQLParser.ParseBlobDefinition(Var ASegmentSize,ABlobType : Integer; Var ACharset : TSQLStringType);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseBlobDefinition(Var ASegmentSize,ABlobType : Integer; Var ACharset : TSQLStringType);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseBlobDefinition(Var ASegmentSize,ABlobType : Integer; Var ACharset : TSQLStringType);
>>>>>>> graemeg/cpstrnew
=======
Procedure TSQLParser.ParseBlobDefinition(Var ASegmentSize,ABlobType : Integer; Var ACharset : TSQLStringType);
>>>>>>> origin/cpstrnew

begin
  // On entry, we are on the blob token.
  GetNextToken;
  If (CurrentToken=tsqlBraceOpen) then // (segment[,subtype])
    begin
    GetNextToken;
    Expect(tsqlIntegerNumber);
    ASegmentSize:=StrtoInt(CurrentTokenString);
    GetNextToken;
    If (CurrentToken=tsqlComma) then
       begin
       GetNextToken;
       Expect(tsqlIntegerNumber);
       ABlobType:=StrtoInt(CurrentTokenString);
       GetNextToken;
       end;
    Consume(tsqlBraceClose);
    If CurrentToken in [tsqlSubtype,tsqlSegment] then
      Error(SErrUnexpectedToken,[CurrentTokenString]);
    end
  else
    begin
    If CurrentToken=tsqlSubtype then   // SUB_TYPE T
      begin
      GetNextToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Expect([tsqlIntegerNumber,tsqlBinary,tsqlText]);
      case CurrentToken of
        tsqlBinary: ABlobType:=0; //FB2.0+ see Language Reference Update
        tsqlText: ABlobType:=1;
        tsqlIntegerNumber: ABlobType:=StrtoInt(CurrentTokenString);
        else Error('ParseBlobDefinition: internal error: unknown token type.');
      end;
=======
      Expect(tsqlIntegerNumber);
      ABlobType:=StrtoInt(CurrentTokenString);
>>>>>>> graemeg/cpstrnew
=======
      Expect(tsqlIntegerNumber);
      ABlobType:=StrtoInt(CurrentTokenString);
>>>>>>> graemeg/cpstrnew
=======
      Expect(tsqlIntegerNumber);
      ABlobType:=StrtoInt(CurrentTokenString);
>>>>>>> graemeg/cpstrnew
=======
      Expect(tsqlIntegerNumber);
      ABlobType:=StrtoInt(CurrentTokenString);
>>>>>>> origin/cpstrnew
      GetNextToken;
      end;
    If (CurrentToken=tsqlSegment) then // SEGMENT SIZE S
      begin
      GetNextToken;
      Consume(tsqlSize);
      Expect(tsqlIntegerNumber);
      ASegmentSize:=StrtoInt(CurrentTokenString);
      GetNextToken;
      end;
    end;
  if (CurrentToken=tsqlCharacter) then  // CHARACTER SET NNN
    begin
    GetNextToken;
    Consume(tsqlSet);
    Expect(tsqlIdentifier);
    ACharSet:=CurrentTokenString;
    GetNextToken;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseForeignKeyDefinition(AParent: TSQLElement
  ): TSQLForeignKeyDefinition;
=======
Function TSQLParser.ParseForeignKeyDefinition(AParent : TSQLElement) : TSQLForeignKeyDefinition;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForeignKeyDefinition(AParent : TSQLElement) : TSQLForeignKeyDefinition;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForeignKeyDefinition(AParent : TSQLElement) : TSQLForeignKeyDefinition;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseForeignKeyDefinition(AParent : TSQLElement) : TSQLForeignKeyDefinition;
>>>>>>> origin/cpstrnew

  // On entry, we're on ON Return true if On delete
  Function ParseForeignKeyAction (Out Res : TForeignKeyAction) : Boolean;

  begin
    GetNextToken;
    Case CurrentToken of
      tsqlDelete,
      tsqlUpdate: Result:=CurrentToken=tsqlDelete;
    else
      UnexpectedToken([tsqlDelete,tsqlupdate]);
    end;
    Case GetNextToken of
      tsqlNo :
        begin
        GetNextToken;
        expect(tsqlAction);
        Res:=fkaNoAction;
        end;
      tsqlCascade :
        Res:=fkaCascade;
      tsqlSet:
        begin
        Case GetNextToken of
          tsqlDefault :
            Res:=fkaSetDefault;
          tsqlNull:
            Res:=fkaSetNull;
        else
          UnexpectedToken([tsqlDefault,tsqlNull]);
        end;
        end
    else
      UnexpectedToken([tsqlNo,tsqlCascade,tsqlSet]);
    end;
    GetNextToken;
  end;

Var
  FKA : TForeignKeyAction;

begin
  Result:=Nil;
  // on entry, we are on the 'REFERENCES' token
  GetNextToken;
  Expect(tsqlidentifier);
  Result:=TSQLForeignKeyDefinition(CreateElement(TSQLForeignKeyDefinition,AParent));
  try
    Result.TableName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    If (CurrentToken=tsqlBraceOpen) then
      begin
      GetNextToken;
      ParseidentifierList(Result,Result.FieldList);
      end;
    if (CurrentToken=tsqlOn) then
      begin
      If ParseForeignKeyAction(FKA) then
        Result.OnDelete:=FKA
      else
        Result.OnUpdate:=FKA;
      end;
    if (CurrentToken=tsqlOn) then
      begin
      If ParseForeignKeyAction(FKA) then
        Result.OnDelete:=FKA
      else
        Result.OnUpdate:=FKA;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseFieldConstraint(AParent: TSQLElement
  ): TSQLFieldConstraint;
=======
Function TSQLParser.ParseFieldConstraint(AParent : TSQLElement) : TSQLFieldConstraint;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseFieldConstraint(AParent : TSQLElement) : TSQLFieldConstraint;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseFieldConstraint(AParent : TSQLElement) : TSQLFieldConstraint;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseFieldConstraint(AParent : TSQLElement) : TSQLFieldConstraint;
>>>>>>> origin/cpstrnew

Var
  N : TSQLStringType;
  K : TSQLForeignKeyFieldConstraint;
  C : TSQLCheckFieldConstraint;
  L : TSQLFieldConstraintList;
  P : Boolean;
begin
  Result:=Nil;
  L:=Nil;
  P:=False;
  try
    Repeat
      If (Result<>Nil) then
        begin
        L:=TSQLFieldConstraintList.Create(AParent);
        L.List.Add(Result);
        Result:=Nil;
        end;
      If CurrentToken=tsqlConstraint then
        begin
        GetNextToken;
        Expect(tsqlIdentifier);
        N:=CurrentTokenString;
        GetNextToken
        end;
        Case CurrentToken of
        tsqlUnique :
          begin
          If P then
            Error('Only one primary/unique field constraint allowed');
          Result:=TSQLFieldConstraint(CreateElement(TSQLUniqueFieldConstraint,AParent));
          GetNextToken;
          P:=True;
          end;
        tsqlPrimary :
          begin
          If P then
            Error('Only one primary/unique field constraint allowed');
          GetNextToken;
          Expect(tsqlKey);
          Result:=TSQLFieldConstraint(CreateElement(TSQLPrimaryKeyFieldConstraint,AParent));
          GetNextToken;
          P:=True;
          end;
        tsqlReferences :
          begin
          K:=TSQLForeignKeyFieldConstraint(CreateElement(TSQLForeignKeyFieldConstraint,AParent));
          Result:=K;
          K.Definition:=ParseForeignKeyDefinition(K);
          end;
        tsqlCheck :
          begin
          C:=TSQLCheckFieldConstraint(CreateElement(TSQLCheckFieldConstraint,AParent));
          Result:=C;
          C.Expression:=ParseCheckConstraint(K,True);
          end
        else
          UnexpectedToken([tsqlUnique,tsqlPrimary,tsqlReferences,tsqlCheck]);
        end;
      If (N<>'') then
        Result.ConstraintName:=CreateIdentifier(Result,N);
    Until Not (CurrentToken in [tsqlUnique,tsqlPrimary,tsqlReferences,tsqlCheck,tsqlConstraint]);
    If Assigned(L) then
      begin
      L.List.Add(Result);
      Result:=L;
      end;
  except
    If (L<>Result) then
      FReeAndNil(L);
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseTypeDefinition(AParent: TSQLElement;
  Flags: TParseTypeFlags): TSQLTypeDefinition;
=======
Function TSQLParser.ParseTypeDefinition(AParent : TSQLElement; Flags : TParseTypeFlags) : TSQLTypeDefinition;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTypeDefinition(AParent : TSQLElement; Flags : TParseTypeFlags) : TSQLTypeDefinition;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTypeDefinition(AParent : TSQLElement; Flags : TParseTypeFlags) : TSQLTypeDefinition;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseTypeDefinition(AParent : TSQLElement; Flags : TParseTypeFlags) : TSQLTypeDefinition;
>>>>>>> origin/cpstrnew

Var
  TN : String;
  AD : Integer;
  DT : TSQLDataType;
  AA : Boolean; // Allow Array
  GN : Boolean; // Do GetNextToken ?
  NN : Boolean; // Not Null ?
  sc,prec : Integer;
  bt : integer;
  D : TSQLTypeDefinition;
  cs : TSQLStringType;
  Coll : TSQLCollation;
  C : TSQLFieldConstraint;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // We are positioned on the token prior to the type definition.
=======
 // We are positioned on the token prior to the type definition.
>>>>>>> graemeg/cpstrnew
=======
 // We are positioned on the token prior to the type definition.
>>>>>>> graemeg/cpstrnew
=======
 // We are positioned on the token prior to the type definition.
>>>>>>> graemeg/cpstrnew
=======
 // We are positioned on the token prior to the type definition.
>>>>>>> origin/cpstrnew
  AA:=True;
  GN:=True;
  prec:=0;
  sc:=0;
  bt:=0;
  NN:=True;
  Coll:=Nil;
  Case GetNextToken of
    tsqlIdentifier :
      If not (ptfAllowDomainName in Flags) then
        Error(SErrDomainNotAllowed)
      else
        begin
        DT:=sdtDomain;
        TN:=CurrentTokenString;
        end;
     tsqlInt,
     tsqlInteger :
       dt:=sdtInteger;
     tsqlSmallInt :
       dt:=sdtSmallInt;
     tsqlDate:
       dt:=sdtDate;
     tsqlTimeStamp:
       dt:=sdtDateTime;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     tsqlDouble:
       begin
       GetNextToken;
       Expect(tsqlPrecision); //DOUBLE PRECISION
       dt:=sdtDoublePrecision;
       end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     tsqlFloat:
       dt:=sdtFloat;
     tsqlTime:
       dt:=sdtTime;
     tsqlDecimal,
     tsqlNumeric:
       begin
       if CurrentToken=tsqlDecimal then
         dt:=sdtDecimal
       else
         dt:=sdtNumeric;
       GetNextToken;
       GN:=False;
       If (CurrentToken=tsqlBraceOpen) then
         begin
         GetNextToken;
         Expect(tsqlIntegerNumber);
         prec:=StrToInt(CurrentTokenString);
         if (GetNextToken=tsqlBraceClose) then
           sc:=0
         else
           begin
           GetNextToken;
           Expect(tsqlIntegerNumber);
           sc:=StrToInt(CurrentTokenString);
           GetNextToken;
           Expect(tsqlBraceClose);
           end;
         GetNextToken; // position on first token after closing brace. GN=False !
         end;
       end;
    tsqlCstring,
    tsqlChar,
    tsqlNChar,
    tsqlVarChar,
    tsqlCharacter,
    tsqlNational :
      begin
      If (CurrentToken=tsqlCstring) and  Not (([ptfExternalFunction,ptfExternalFunctionResult]*Flags) <> []) then
       UnexpectedToken;
      GN:=False;
      ParseCharTypeDefinition(DT,Prec,cs);
      end;
    tsqlBlob :
      begin
      dt:=sdtBlob;
      GN:=False;
      ParseBlobDefinition(prec,bt,cs);
      end;
  else
    UnexpectedToken;
  end;
  If GN then
    GetNextToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // We are now on array definition or rest of type.
=======
  // We are now on array or rest of type.
>>>>>>> graemeg/cpstrnew
=======
  // We are now on array or rest of type.
>>>>>>> graemeg/cpstrnew
=======
  // We are now on array or rest of type.
>>>>>>> graemeg/cpstrnew
=======
  // We are now on array or rest of type.
>>>>>>> origin/cpstrnew
  If (CurrentToken=tsqlSquareBraceOpen) then
    begin
    GetNextToken;
    Expect(tsqlIntegerNumber);
    AD:=Strtoint(CurrentTokenString);
    GetNextToken;
    Expect(tsqlSquareBraceClose);
    GetNextToken;
    end
  else
    AD:=0;
  // Collation is here in domain (needs checking ?)
  If (CurrentToken=tsqlCollate) then
    begin
    If not (dt in [sdtChar,sdtVarchar,sdtNchar,sdtNVarChar,sdtBlob]) then
      Error(SErrInvalidUseOfCollate);
    GetNextToken;
    Expect(tsqlIdentifier);
    Coll:=TSQLCollation(CreateElement(TSQLCollation,AParent));
    Coll.Name:=CurrentTokenString;
    GetNextToken;
    end
  else
    Coll:=Nil;
  C:=Nil;
  D:=TSQLTypeDefinition(CreateElement(TSQLTypeDefinition,AParent));
  try
    D.DataType:=DT;
    D.TypeName:=TN;
    D.Len:=PRec;
    D.Scale:=Sc;
    D.BlobType:=bt;
    D.ArrayDim:=AD;
    D.Charset:=CS;
    D.Collation:=Coll;
    D.Constraint:=C;
    if (not (ptfAlterDomain in Flags)) then // Alternative is to raise an error in each of the following
      begin
      If (CurrentToken=tsqlDefault) then
          begin
          GetNextToken;
          D.DefaultValue:=CreateLiteral(D);
          GetNextToken;
          end;
      if (CurrentToken=tsqlNot) then
        begin
        GetNextToken;
        Expect(tsqlNULL);
        D.NotNull:=True;
        GetNextToken;
        end;
      If (CurrentToken=tsqlCheck) and not (ptfTableFieldDef in Flags) then
        begin
        D.Check:=ParseCheckConstraint(D,False);
        // Parsecheckconstraint is on next token.
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      // Firebird 2.5 generates/accepts NOT NULL after CHECK constraint instead
      // of before it in at least domain definitions:
      if (CurrentToken=tsqlNot) then
        begin
        GetNextToken;
        Expect(tsqlNULL);
        D.NotNull:=True;
        GetNextToken;
        end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      // Constraint is before collation.
      if CurrentToken in [tsqlConstraint,tsqlCheck,tsqlUnique,tsqlprimary,tsqlReferences] then
        begin
        If Not (ptfAllowConstraint in Flags) then
          UnexpectedToken;
        D.Constraint:=ParseFieldConstraint(AParent);
        end;
      // table definition can have PRIMARY KEY CHECK
      If (CurrentToken=tsqlCheck) and (ptfTableFieldDef in Flags) then
        begin
        D.Check:=ParseCheckConstraint(D,False);
        // Parsecheckconstraint is on next token.
        end;
      // Collation is after constraint in table
      If (CurrentToken=tsqlCollate) then
        begin
        If not (dt in [sdtChar,sdtVarchar,sdtNchar,sdtNVarChar,sdtBlob]) then
          Error(SErrInvalidUseOfCollate);
        GetNextToken;
        Expect(tsqlIdentifier);
        Coll:=TSQLCollation(CreateElement(TSQLCollation,AParent));
        Coll.Name:=CurrentTokenString;
        GetNextToken;
        end
      else
        Coll:=Nil;
      If (CurrentToken=tsqlBy) and (ptfExternalFunctionResult in Flags) then
        begin
        GetNextToken;
        Consume(tsqlValue);
        D.ByValue:=True;
        end;
      end;
    Result:=D;
  except
    FreeAndNil(D);
    Raise;
  end;
end;

function TSQLParser.CreateLiteral(AParent : TSQLElement) : TSQLLiteral;

begin
  Result:=Nil;
  Case CurrentToken of
    tsqlIntegerNumber:
      begin
      Result:=TSQLLiteral(CreateElement(TSQLIntegerLiteral,AParent));
      TSQLIntegerLiteral(Result).Value:=StrToInt(CurrentTokenString);
      end;
    tsqlString:
      begin
      Result:=TSQLLiteral(CreateElement(TSQLStringLiteral,AParent));
      TSQLStringLiteral(Result).Value:=CurrentTokenString;
      end;
    tsqlFloatNumber:
      begin
      Result:=TSQLLiteral(CreateElement(TSQLFloatLiteral,AParent));
      TSQLFloatLiteral(Result).Value:=StrToFloat(CurrentTokenString);
      end;
    tsqlNull :
      Result:=TSQLLiteral(CreateElement(TSQLNullLiteral,AParent));
    tsqlValue :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Result:=TSQLLiteral(CreateElement(TSQLValueLiteral,AParent));
=======
        Result:=TSQLLiteral(CreateElement(TSQLValueLiteral,AParent));
>>>>>>> graemeg/cpstrnew
=======
        Result:=TSQLLiteral(CreateElement(TSQLValueLiteral,AParent));
>>>>>>> graemeg/cpstrnew
=======
        Result:=TSQLLiteral(CreateElement(TSQLValueLiteral,AParent));
>>>>>>> graemeg/cpstrnew
=======
        Result:=TSQLLiteral(CreateElement(TSQLValueLiteral,AParent));
>>>>>>> origin/cpstrnew
    tsqlUSER :
      Result:=TSQLLiteral(CreateElement(TSQLUserLiteral,AParent));
  else
    Error(SErrInvalidLiteral,[CurrentTokenString]);
  end;
end;

procedure TSQLParser.CheckEOF;

begin
  If CurrentToken=tsqlEOF then
    Error('Unexpected end of command');
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprLevel1(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLExpression;
=======
Function TSQLParser.ParseExprLevel1(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel1(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel1(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel1(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> origin/cpstrnew

var
  tt: TSQLToken;
  B : TSQLBinaryExpression;
  Right: TSQLExpression;
  L : TSQLLiteralExpression;

begin
  Result:=ParseExprLevel2(AParent,EO);
  Try
    while (CurrentToken in [tsqlAnd,tsqlOr{,tsqlIs}]) do
      begin
      tt:=CurrentToken;
      GetNextToken;
      CheckEOF;
      B:=TSQLBinaryExpression(CreateElement(TSQLBinaryExpression,AParent));
      B.Left:=TSQLExpression(Result);
      Result:=B;
      If tt=tsqlIs then
        begin
        If CurrentToken=tsqlNot then
           begin
//           B.Operation:=boIsNot;
           GetNextToken;
           end
        else
          B.Operation:=boIs;
        Expect(tsqlNull);
        L:=TSQLLiteralExpression(CreateElement(TSQLLiteralExpression,AParent));
        L.Literal:=CreateLiteral(AParent);
        B.Right:=L;
        GetNexttoken;
        end
      else
        begin
        case tt of
          tsqlOr  : B.Operation:=boOr;
          tsqlAnd : B.Operation:=boAnd;
        Else
          Error(SErrUnknownBooleanOp)
        end;
        B.Right:=ParseExprLevel2(AParent,EO);
        end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseInoperand(AParent: TSQLElement): TSQLExpression;
=======
Function TSQLParser.ParseInoperand(AParent : TSQLElement) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseInoperand(AParent : TSQLElement) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseInoperand(AParent : TSQLElement) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseInoperand(AParent : TSQLElement) : TSQLExpression;
>>>>>>> origin/cpstrnew

Var
  S : TSQLSelectExpression;
  L : TSQLListExpression;
  Done : Boolean;

begin
  // On entry, we're on the first token after IN token, which is the ( token.
  Consume(tsqlBraceopen);
  try
    If (CurrentToken=tsqlSelect) then
      begin
      S:=TSQLSelectExpression(CreateElement(TSQLSelectExpression,APArent));
      Result:=S;
      S.Select:=ParseSelectStatement(AParent,[sfSingleton]);
      Consume(tsqlBraceClose);
      end
    else
      begin
      L:=TSQLListExpression(CreateElement(TSQLListExpression,AParent));
      Result:=L;
      Repeat
         L.List.Add(ParseExprLevel1(L,[eoListValue]));
         Expect([tsqlBraceClose,tsqlComma]);
         Done:=(CurrentToken=tsqlBraceClose);
         GetNextToken;
      until Done;

      end;
  except
    FreeAndNil(Result);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprLevel2(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLExpression;
=======
Function TSQLParser.ParseExprLevel2(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel2(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel2(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel2(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> origin/cpstrnew
var
  tt: TSQLToken;
  Right : TSQLExpression;
  B : TSQLBinaryExpression;
  T : TSQLTernaryExpression;
  O : TSQLBinaryOperation;
  U : TSQLUnaryExpression;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Inverted,bw,doin : Boolean;
=======
  I,bw,doin : Boolean;
>>>>>>> graemeg/cpstrnew
=======
  I,bw,doin : Boolean;
>>>>>>> graemeg/cpstrnew
=======
  I,bw,doin : Boolean;
>>>>>>> graemeg/cpstrnew
=======
  I,bw,doin : Boolean;
>>>>>>> origin/cpstrnew

begin
  {$ifdef debugexpr}  Writeln('Level 2 ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
  Result:=ParseExprLevel3(AParent,EO);
  try
    if (CurrentToken in sqlComparisons) then
      begin
      tt:=CurrentToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Inverted:=CurrentToken=tsqlnot;
      CheckEOF;
      GetNextToken;
      CheckEOF;
      if Inverted then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      I:=CurrentToken=tsqlnot;
      CheckEOF;
      GetNextToken;
      CheckEOF;
      if I then
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
        begin
        tt:=CurrentToken;
        if Not (tt in sqlInvertableComparisons) then
          Error(SErrUnexpectedToken,[CurrentTokenString]);
        GetNextToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        end
      else
        begin
        if (CurrentToken=tsqlNot) then
          begin
          GetNextToken;
          if not (tt=tsqlIS) then
            UnexpectedToken;
          Inverted:=true;
          end;
        end;

      // Step past expected STARTING WITH
      If (tt=tsqlStarting) and (CurrentToken=tsqlWith) then
        GetNextToken;

      bw:=False;
      doin:=false;
      B:=nil; //needed for test later
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        If (tt=tsqlStarting) and (CurrentToken=tsqlWith) then
          GetNextToken;
        end
      else
        if (CurrentToken=tsqlNot) then
          begin
          GetNextToken;
          if not (tt=tsqlis) then
            UnexpectedToken;
          I:=true;
          end;


      bw:=False;
      doin:=false;
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
      Case tt of
        tsqlLT : O:=boLT;
        tsqlLE : O:=boLE;
        tsqlGT : O:=boGT;
        tsqlGE : O:=boGE;
        tsqlEq : O:=boEq;
        tsqlNE : O:=boNE;
        tsqlLike : O:=boLike;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        tsqlIn : doIn:=true;
=======
        tsqlIn : doin:=true;
>>>>>>> graemeg/cpstrnew
=======
        tsqlIn : doin:=true;
>>>>>>> graemeg/cpstrnew
=======
        tsqlIn : doin:=true;
>>>>>>> graemeg/cpstrnew
=======
        tsqlIn : doin:=true;
>>>>>>> origin/cpstrnew
        tsqlis : O:=boIs;
        tsqlContaining : O:=boContaining;
        tsqlStarting : O:=boStarting;
        tsqlBetween : bw:=true;
      Else
        Error(SErrUnknownComparison)
      end;
      If doIn then
        begin
        Right:=ParseInOperand(AParent);
        B:=TSQLBinaryExpression(CreateElement(TSQLBinaryExpression,AParent));
        B.Operation:=boIn;
        B.Left:=Result;
        Result:=B;
        B.Right:=Right;
        end
      else
        begin
        Right:=ParseExprLevel3(AParent,EO);
        If (O=boLike) and (CurrentToken=tsqlEscape) then
          begin
          GetNextToken;
          T:=TSQLTernaryExpression(CreateElement(TSQLTernaryExpression,AParent));
          T.Left:=Result;
          Result:=T;
          T.Middle:=Right;
          T.Right:=ParseExprLevel3(AParent,EO);
          T.Operation:=toLikeEscape
          end
        else If bw then
          begin
          Consume(tsqlAnd);
          T:=TSQLTernaryExpression(CreateElement(TSQLTernaryExpression,AParent));
          T.Left:=Result;
          Result:=T;
          T.Middle:=Right;
          T.Right:=ParseExprLevel3(AParent,EO);
          T.Operation:=toBetween;
          end
        else
          begin
          B:=TSQLBinaryExpression(CreateElement(TSQLBinaryExpression,AParent));
          B.Operation:=O;
          B.Left:=Result;
          Result:=B;
          B.Right:=Right;
          end;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      If Inverted then
        if (Assigned(B)) and (B.Operation=boIs) then
=======
      If I then
        if B.Operation=boIs then
>>>>>>> graemeg/cpstrnew
=======
      If I then
        if B.Operation=boIs then
>>>>>>> graemeg/cpstrnew
=======
      If I then
        if B.Operation=boIs then
>>>>>>> graemeg/cpstrnew
=======
      If I then
        if B.Operation=boIs then
>>>>>>> origin/cpstrnew
          B.Operation:=boIsNot
        else
          begin
          U:=TSQLUnaryExpression(CreateElement(TSQLUnaryExpression,AParent));
          U.Operand:=Result;
          U.Operation:=uoNot;
          Result:=U;
          end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprLevel3(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLExpression;
=======
Function TSQLParser.ParseExprLevel3(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel3(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel3(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel3(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> origin/cpstrnew

  Function NegativeNumber : Boolean; inline;

  begin
    Result:=(CurrentToken in [tsqlIntegerNumber,tsqlFloatNumber]) and (StrToInt(CurrentTokenString)<0)
  end;

var
  tt : TSQLToken;
  right : TSQLExpression;
  B : TSQLBinaryExpression;


begin
{$ifdef debugexpr}  Writeln('Level 3 ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
  Result:=ParseExprLevel4(AParent,EO);
  try
    {$ifdef debugexpr}  Writeln('Level 3 continues ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
    // Scanner returns -N as an negative number, not as - (positive number)
    // NegativeNumber is for the case A-1 or so: convert to A + -1
    while (CurrentToken in [tsqlConcatenate,tsqlPlus,tsqlMinus]) or NegativeNumber do
      begin
      tt:=CurrentToken;
      If NegativeNumber then
        tt:=tsqlPlus // Pretend we've eaten +
      else
        begin
        GetNextToken;
        CheckEOF;
        end;
      Right:=ParseExprLevel4(AParent,EO);
      B:=TSQLBinaryExpression(CreateElement(TSQLBinaryExpression,AParent));
      B.Left:=Result;
      Result:=B;
      B.Right:=Right;
      Case tt of
        tsqlPlus  : B.Operation:=boAdd;
        tsqlMinus : B.Operation:=boSubtract;
        tsqlConcatenate : B.Operation:=boConcat;
      end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprLevel4(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLExpression;
=======
Function TSQLParser.ParseExprLevel4(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel4(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel4(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel4(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> origin/cpstrnew

var
  tt : TSQLToken;
  right : TSQLExpression;
  B : TSQLBinaryExpression;

begin
{$ifdef debugexpr}  Writeln('Level 4 ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
  Result:=ParseExprLevel5(AParent,EO);
  try
    while (CurrentToken in [tsqlMul,tsqlDiv]) do
      begin
      tt:=CurrentToken;
      GetNextToken;
      CheckEOF;
      Right:=ParseExprLevel5(AParent,EO);
      B:=TSQLBinaryExpression(CreateElement(TSQLBinaryExpression,AParent));
      B.Left:=Result;
      B.Right:=Right;
      Case tt of
        tsqlMul : B.Operation:=boMultiply;
        tsqlDiv : B.Operation:=boDivide;
      end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprLevel5(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLExpression;
=======
Function TSQLParser.ParseExprLevel5(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel5(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel5(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel5(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> origin/cpstrnew

Var
  tt : tsqltoken;
  U : TSQLUnaryExpression;
begin
{$ifdef debugexpr}  Writeln('Level 5 ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
  tt:=tsqlunknown;
  if (CurrentToken in [tsqlNot,tsqlPlus,tsqlMinus]) then
    begin
    tt:=CurrentToken;
    GetNextToken;
    CheckEOF;
    end;
  Result:=ParseExprLevel6(AParent,EO);
  try
    If tt<>tsqlUnknown then
      begin
      U:=TSQLunaryExpression(CreateElement(TSQLunaryExpression,AParent));
      if tt=tsqlNot then
        U.Operation:=uoNot
      else
        U.Operation:=uoMinus;
      U.Operand:=Result;
      Result:=U;
      end;
  except
    FreeandNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprLevel6(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLExpression;
=======
Function TSQLParser.ParseExprLevel6(AParent : TSQLElement ; EO : TExpressionOptions ) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel6(AParent : TSQLElement ; EO : TExpressionOptions ) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel6(AParent : TSQLElement ; EO : TExpressionOptions ) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprLevel6(AParent : TSQLElement ; EO : TExpressionOptions ) : TSQLExpression;
>>>>>>> origin/cpstrnew



begin
{$ifdef debugexpr}  Writeln('Level 6 ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
  if (CurrentToken=tsqlBraceOpen) then
    begin
    GetNextToken;
    If (CurrentToken<>tsqlselect) then
      Result:=ParseExprLevel1(AParent,EO)
    else
      begin
      Result:=TSQLExpression(CreateElement(TSQLSelectExpression,AParent));
      try
        TSQLSelectExpression(Result).Select:=ParseSelectStatement(Result,[sfSingleTon]);
      except
        FreeAndNil(Result);
        Raise;
      end;
      end;
    try
      if (CurrentToken<>tsqlBraceClose) then
        Error(SerrUnmatchedBrace);
      GetNextToken;
    Except
      Result.Free;
      Raise;
    end;
    end
  else
    Result:=ParseExprPrimitive(AParent,EO);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseIdentifierList(AParent: TSQLElement;
  AList: TSQLelementList): integer;
=======
Function TSQLParser.ParseIdentifierList(AParent : TSQLElement; AList : TSQLelementList) : integer;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseIdentifierList(AParent : TSQLElement; AList : TSQLelementList) : integer;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseIdentifierList(AParent : TSQLElement; AList : TSQLelementList) : integer;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseIdentifierList(AParent : TSQLElement; AList : TSQLelementList) : integer;
>>>>>>> origin/cpstrnew

Var
  Done : Boolean;

begin
  // on entry, we're on first identifier
  Expect(tsqlIdentifier);
  Result:=0;
  Done:=False;
  repeat
    if CurrentToken=tsqlComma then
      GetNextToken;
    Expect(tsqlIdentifier);
    AList.add(CreateIdentifier(AParent,CurrentTokenString));
    Inc(Result);
  until (GetNextToken<>tsqlComma);
  Expect(tsqlBraceClose);
  GetNextToken;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseValueList(AParent: TSQLElement; EO: TExpressionOptions
  ): TSQLElementList;
=======
Function TSQLParser.ParseValueList(AParent : TSQLElement; EO : TExpressionOptions) : TSQLElementList;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseValueList(AParent : TSQLElement; EO : TExpressionOptions) : TSQLElementList;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseValueList(AParent : TSQLElement; EO : TExpressionOptions) : TSQLElementList;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseValueList(AParent : TSQLElement; EO : TExpressionOptions) : TSQLElementList;
>>>>>>> origin/cpstrnew

Var
    E : TSQLExpression;

begin
  Result:=Nil;
  E:=Nil;
  // First token is (
  Expect(tsqlBraceOpen);
  Repeat
    GetNextToken;
    If (CurrentToken<>tsqlBraceClose) then
      E:=ParseExprLevel1(AParent,EO);
    If (E<>Nil) then
      begin
      If Result=Nil then
        Result:=TSQLElementList.Create(True);
      Result.Add(E);
      end;
    Expect([tsqlComma,tsqlBraceClose]);
  Until CurrentToken=tsqlBraceClose;
end;

procedure TSQLParser.UnexpectedToken;
begin
  Error(SErrUnexpectedToken,[CurrentTokenString]);
end;

procedure TSQLParser.UnexpectedToken(AExpected: TSQLTokens);

Var
  S : String;
  I : TSQLToken;

begin
  S:='';
  For I:=Low(TSQLToken) to High(TSQLToken) do
    if I in AExpected then
      begin
      If (S<>'') then
        S:=S+',';
      S:=S+TokenInfos[i];
      end;
  Error(SErrUnexpectedTokenOf,[CurrentTokenString,S]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.CreateIdentifier(AParent: TSQLElement;
  const AName: TSQLStringType): TSQLIdentifierName;
=======
function TSQLParser.CreateIdentifier(AParent : TSQLElement; Const AName: TSQLStringType
  ): TSQLIdentifierName;
>>>>>>> graemeg/cpstrnew
=======
function TSQLParser.CreateIdentifier(AParent : TSQLElement; Const AName: TSQLStringType
  ): TSQLIdentifierName;
>>>>>>> graemeg/cpstrnew
=======
function TSQLParser.CreateIdentifier(AParent : TSQLElement; Const AName: TSQLStringType
  ): TSQLIdentifierName;
>>>>>>> graemeg/cpstrnew
=======
function TSQLParser.CreateIdentifier(AParent : TSQLElement; Const AName: TSQLStringType
  ): TSQLIdentifierName;
>>>>>>> origin/cpstrnew
begin
  Result:=TSQLIdentifierName(CreateElement(TSQLIdentifierName,AParent));
  Result.Name:=AName;
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprAggregate(AParent: TSQLElement;
  EO: TExpressionOptions): TSQLAggregateFunctionExpression;
=======
Function TSQLParser.ParseExprAggregate(AParent : TSQLElement; EO : TExpressionOptions) : TSQLAggregateFunctionExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprAggregate(AParent : TSQLElement; EO : TExpressionOptions) : TSQLAggregateFunctionExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprAggregate(AParent : TSQLElement; EO : TExpressionOptions) : TSQLAggregateFunctionExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprAggregate(AParent : TSQLElement; EO : TExpressionOptions) : TSQLAggregateFunctionExpression;
>>>>>>> origin/cpstrnew
begin
  Result:=TSQLAggregateFunctionExpression(CreateElement(TSQLAggregateFunctionExpression,AParent));
  try
    Case CurrentToken of
      tsqlCount : Result.Aggregate:=afCount;
      tsqlSum : Result.Aggregate:=afSum;
      tsqlAvg : Result.Aggregate:=afAvg;
      tsqlMax : Result.Aggregate:=afMax;
      tsqlMin : Result.Aggregate:=afMin;
    end;
    GetNextToken;
    Consume(tsqlBraceOpen);
    If CurrentToken=tsqlMul then
      begin
      If Result.Aggregate<>afCount then
        Error(SErrAsteriskOnlyInCount);
      Result.OPtion:=aoAsterisk;
      GetNextToken;
      end
    else
      begin
      if (CurrentToken in [tsqlAll,tsqlDistinct]) then
        begin
        If CurrentToken=tsqlAll then
          Result.Option:=aoAll
        else
          Result.Option:=aoDistinct;
        GetNextToken;
        end;
      Result.Expression:=ParseExprLevel1(Result,EO);
      end;
    Consume(tsqlBraceClose);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseExprPrimitive(AParent: TSQLElement;
  EO: TExpressionOptions): TSQLExpression;
=======
Function TSQLParser.ParseExprPrimitive(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprPrimitive(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprPrimitive(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseExprPrimitive(AParent : TSQLElement; EO : TExpressionOptions) : TSQLExpression;
>>>>>>> origin/cpstrnew

Var
  L : TSQLElementList;
  N : String;
  C : TSQLElementClass;
  E : TSQLExtractElement;

begin
  Result:=Nil;
  try
  {$ifdef debugexpr}  Writeln('Primitive ',TokenInfos[CurrentToken],': ',CurrentTokenString);{$endif debugexpr}
    Case CurrentToken of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      tsqlIntegerNumber,
=======
      tsqlintegerNumber,
>>>>>>> graemeg/cpstrnew
=======
      tsqlintegerNumber,
>>>>>>> graemeg/cpstrnew
=======
      tsqlintegerNumber,
>>>>>>> graemeg/cpstrnew
=======
      tsqlintegerNumber,
>>>>>>> origin/cpstrnew
      tsqlString,
      tsqlFloatNumber,
      tsqlNull, // True and False belong here
      tsqlValue,
      tsqlUser:
        begin
        Result:=TSQLLiteralExpression(CreateElement(TSQLLiteralExpression,AParent));
        TSQLLiteralExpression(Result).Literal:=CreateLiteral(AParent);
        GetNextToken;
        end;
      tsqlCast:
        begin
        GetNextToken;
        Consume(tsqlBraceOpen);
        Result:=TSQLCastExpression(CreateElement(TSQLCastExpression,AParent));
        TSQLCastExpression(Result).Value:=ParseExprLevel1(Result,EO);
        Expect(tsqlAs);
        TSQLCastExpression(Result).NewType:=ParseTypeDefinition(Result,[ptfCast]);
        Consume(tsqlBraceClose);
        end;
      tsqlExtract:
        begin
        GetNextToken;
        Consume(tsqlBraceOpen);
        Expect(tsqlIdentifier);
        if not StringToSQLExtractElement(CurrentTokenString,E) then
          Error(SErrInvalidExtract,[CurrentTokenString]);
        Consume(tsqlIdentifier);
        Consume(tsqlFrom);
        Result:=TSQLExtractExpression(CreateElement(TSQLExtractExpression,AParent));
        TSQLExtractExpression(Result).Element:=E;
        TSQLExtractExpression(Result).Value:=ParseExprLevel1(Result,EO);
        Consume(tsqlBraceClose);
        end;
      tsqlExists,
      tsqlAll,
      tsqlAny,
      tsqlSome,
      tsqlSingular:
        begin
        Case CurrentToken of
          tsqlExists   : C:=TSQLexistsExpression;
          tsqlAll      : C:=TSQLAllExpression;
          tsqlAny      : C:=TSQLAnyExpression;
          tsqlSome     : C:=TSQLSomeExpression;
          tsqlSingular : C:=TSQLSingularExpression;
        end;
        GetNextToken;
        Consume(tsqlBraceOpen);
        Result:=TSQLSelectionExpression(CreateElement(C,AParent));
        TSQLSelectionExpression(Result).Select:=ParseSelectStatement(Result,[]);
        Consume(tsqlBraceClose);
        end;
      tsqlCount,
      tsqlSum,
      tsqlAvg,
      tsqlMax,
      tsqlMin :
        begin
        If not ([eoSelectValue,eoHaving]*EO <> []) then
          Error(SErrNoAggregateAllowed);
        Result:=ParseExprAggregate(APArent,EO);
        end;
      tsqlUpper :
        begin
          GetNextToken;
          L:=ParseValueList(AParent,EO);
          If L.Count<>1 then
            begin
            FreeAndNil(L);
            Error(SErrUpperOneArgument);
            end;
          GetNextToken; // Consume );
          Result:=TSQLFunctionCallExpression(CreateElement(TSQLFunctionCallExpression,AParent));
          TSQLFunctionCallExpression(Result).IDentifier:='UPPER';
          TSQLFunctionCallExpression(Result).Arguments:=L;
        end;
      tsqlGenID :
        begin
        GetNextToken;
        Consume(tsqlBraceOpen);
        expect(tsqlIdentifier);
        N:=CurrentTokenString;
        GetNextToken;
        Consume(tsqlComma);
        Result:=TSQLGenIDExpression(CreateElement(TSQLGenIDExpression,AParent));
        TSQLGenIDExpression(Result).Generator:=CreateIdentifier(Result,N);
        TSQLGenIDExpression(Result).Value:=ParseExprLevel1(AParent,EO);
        Consume(tsqlBraceClose);
        end;
      tsqlColon:
        begin
        if (([eoCheckConstraint,eoTableConstraint,eoComputedBy] * EO)<>[]) then
          Error(SErrUnexpectedToken,[CurrentTokenString]);
        GetNextToken;
        expect(tsqlIdentifier);
        N:=CurrentTokenString;
        Result:=TSQLParameterExpression(CreateElement(TSQLParameterExpression,AParent));
        TSQLParameterExpression(Result).Identifier:=CreateIdentifier(Result,N);
        Consume(tsqlIdentifier);
        end;
      tsqlIdentifier:
        begin
        N:=CurrentTokenString;
        If (GetNextToken<>tsqlBraceOpen) then
          begin
          If (eoCheckConstraint in EO) and not (eoTableConstraint in EO) then
            Error(SErrUnexpectedToken,[CurrentTokenString]);
          If (CurrentToken=tsqlDot) then
            begin
            GetNextToken;
            Expect(tsqlIdentifier);
            N:=N+'.'+CurrentTokenString;
            GetNextToken;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          // Plain identifier
          Result:=TSQLIdentifierExpression(CreateElement(TSQLIdentifierExpression,APArent));
          TSQLIdentifierExpression(Result).Identifier:=CreateIdentifier(Result,N);
          // Array access ?
          If (CurrentToken=tsqlSquareBraceOpen) then
            // Either something like array[5] or,
            // in procedures etc array[i:] where i is a variable
            begin
            case GetNextToken of
              tsqlIntegerNumber: TSQLIdentifierExpression(Result).ElementIndex:=StrToInt(CurrentTokenString);
              tsqlColon:
                begin
                GetNextToken;
                Expect(tsqlIdentifier);
                // We can't set element index here, but it IS an array...
                //todo: verify if there are repercussions/what these would be
                TSQLIdentifierExpression(Result).ElementIndex:=maxint;
                end;
            else
               Error(SErrIntegerExpected);
            end;
            GetNextToken;
            Consume(tsqlSquareBraceClose);
            end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          // plain identifier
          Result:=TSQLIdentifierExpression(CreateElement(TSQLIdentifierExpression,APArent));
          TSQLIdentifierExpression(Result).IDentifier:=CreateIdentifier(Result,N);
          // Array access ?
          If (CurrentToken=tsqlSquareBraceOpen) then
             begin
             If (GetNextToken<>tsqlIntegerNumber) then
                Error(SErrIntegerExpected);
             TSQLIdentifierExpression(Result).ElementIndex:=StrToInt(CurrentTokenString);
             GetNextToken;
             Consume(tsqlSquareBraceClose);
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
          end
        else
          begin
          L:=ParseValueList(AParent,EO);
          GetNextToken; // Consume );
          // Function call
          Result:=TSQLFunctionCallExpression(CreateElement(TSQLFunctionCallExpression,AParent));
          TSQLFunctionCallExpression(Result).IDentifier:=N;
          TSQLFunctionCallExpression(Result).Arguments:=L;
          end;
        end;
      else
        UnexpectedToken;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseSQLValue(AParent : TSQLElement) : TSQLExpression;

Var
  E : TSQLExpression;
begin
  E:=ParseExprLevel1(AParent,[]);
  Result:=E;
end;

function TSQLParser.ParseCheckConstraint(AParent : TSQLElement; TableConstraint : Boolean = False) : TSQLExpression;

Var
  EO : TExpressionOptions;

begin
  // We are on the 'CHECK' token.
  GetNextToken;
  Consume(tsqlBraceOpen);
  EO:=[eoCheckConstraint];
  If TableConstraint then
    EO:=EO+[eoTableConstraint];
  Result:=ParseExprLevel1(AParent,EO);
  Consume(tsqlBraceClose);
end;

function TSQLParser.ParseCreateDomainStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;

var
  D : TSQLCreateDomainStatement;
  A : TSQLAlterDomainStatement;
  N : TSQLStringType;
  NN : Boolean;

begin
  Result:=Nil;
  GetNextToken;
  Expect(tsqlIdentifier);
  N:=CurrentTokenString;
  If not IsAlter then
    begin
    D:=TSQLCreateDomainStatement(CreateElement(TSQLCreateDomainStatement,AParent));
    try
      D.ObjectName:=CreateIdentifier(D,N);
      If (PeekNextToken=tsqlAs) then
        GetNextToken;
      D.TypeDefinition:=ParseTypeDefinition(D,[])
    except
      FreeAndNil(D);
      Raise;
    end;
    Result:=D;
    end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    begin //alter statement
=======
    begin
>>>>>>> graemeg/cpstrnew
=======
    begin
>>>>>>> graemeg/cpstrnew
=======
    begin
>>>>>>> graemeg/cpstrnew
=======
    begin
>>>>>>> origin/cpstrnew
    A:=Nil;
    NN:=False;
    try
      Case GetNextToken of
        tsqlSet:
          begin
          GetNextToken;
          Expect(tsqlDefault);
          GetNextToken;
          A:=TSQLAlterDomainSetDefaultStatement(CreateElement(TSQLAlterDomainSetDefaultStatement,APArent));
          TSQLAlterDomainSetDefaultStatement(A).DefaultValue:=CreateLiteral(A);
          end;
        tsqlDrop:
          begin
          Case GetNextToken of
            tsqlDefault : A:=TSQLAlterDomainDropDefaultStatement(CreateElement(TSQLAlterDomainDropDefaultStatement,APArent));
            tsqlConstraint : A:=TSQLAlterDomainDropCheckStatement(CreateElement(TSQLAlterDomainDropCheckStatement,APArent));
          else
            Error(SErrUnexpectedToken,[CurrentTokenString]);
          end;
          end;
        tsqlAdd:
          begin
          if (GetNextToken=tsqlConstraint) then
            GetNextToken;
          Expect(tsqlCheck);
          A:=TSQLAlterDomainAddCheckStatement(CreateElement(TSQLAlterDomainAddCheckStatement,APArent));
          TSQLAlterDomainAddCheckStatement(A).Check:=ParseCheckConstraint(A);
          NN:=True;
          end;
        tsqlType:
          begin
          A:=TSQLAlterDomainTypeStatement(CreateElement(TSQLAlterDomainTypeStatement,AParent));
          TSQLAlterDomainTypeStatement(A).NewType:=ParseTypeDefinition(A,[ptfAlterDomain]);
          NN:=True;
          end;
        tsqlIdentifier:
          begin
          A:=TSQLAlterDomainRenameStatement(CreateElement(TSQLAlterDomainRenameStatement,APArent));
          TSQLAlterDomainRenameStatement(A).NewName:=CreateIdentifier(A,CurrentTokenString);
          end;
      else
        UnexpectedToken([tsqlSet,tsqlIdentifier,tsqlAdd,tsqlType,tsqlDrop]);
      end;
      A.ObjectName:=CreateIdentifier(A,N);
      Result:=A;
      If not NN then
        GetNextToken;
    except
      FreeAndNil(A);
      Raise;
    end;
    end;
end;

function TSQLParser.ParseCreateExceptionStatement(AParent: TSQLElement;
  IsAlter: Boolean): TSQLCreateOrAlterStatement;

var
  E : TSQLCreateExceptionStatement;
  N : TSQLStringType;

begin
  GetNextToken;
  Expect(tsqlIdentifier);
  N:=CurrentTokenString;
  try
    if IsAlter then
      E:=TSQLCreateExceptionStatement(CreateElement(TSQLAlterExceptionStatement,AParent))
    else
      E:=TSQLCreateExceptionStatement(CreateElement(TSQLCreateExceptionStatement,AParent));
    E.ObjectName:=CreateIdentifier(E,N);
    GetNextToken;
    Expect(tsqlString);
    E.ExceptionMessage:=TSQLStringLiteral(CreateElement(TSQLStringLiteral,E));
    E.ExceptionMessage.Value:=CurrentTokenString;
    GetNextToken;
  except
    FreeAndNil(E);
    Raise;
  end;
  Result:=E;
end;

function TSQLParser.ParseCreateTriggerStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;

Var
  T : TSQLAlterCreateTriggerStatement;

begin
  // On entry, we're on the 'TRIGGER' token.
  Consume(tsqlTrigger);
  If IsAlter then
    T:=TSQLAlterTriggerStatement(CreateElement(TSQLAlterTriggerStatement,APArent))
  else
    T:=TSQLCreateTriggerStatement(CreateElement(TSQLCreateTriggerStatement,APArent));
  Result:=T;
  try
    Expect(tsqlidentifier);
    Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
    getnexttoken;
    If Not IsAlter then
      begin
      Consume(tsqlfor);
      Expect(tsqlidentifier);
      T.TableName:=CreateIdentifier(Result,CurrentTokenString);
      GetNextToken;
      end;
    if (CurrentToken in [tsqlActive,tsqlInactive]) then
      begin
      If CurrentToken=tsqlActive then
        T.State:=tsActive
      else
        T.State:=tsInactive;
      GetNextToken;
      end;
    Expect([tsqlBefore,tsqlAfter]);
    if CurrentToken=tsqlBefore then
      T.Moment:=tmBefore
    else
      T.Moment:=tmAfter;
    Repeat
      GetNextToken;
      Expect([tsqlDelete,tsqlInsert,tsqlUpdate]);
      Case CurrentToken of
        tsqlDelete : T.Operations:=T.Operations+[toDelete];
        tsqlUpdate : T.Operations:=T.Operations+[toUpdate];
        tsqlInsert : T.Operations:=T.Operations+[toInsert];
      end;
      GetNextToken;
    Until (CurrentToken<>tsqlOr);
    If CurrentToken=tsqlPosition then
      begin
      GetNextToken;
      Expect(tsqlIntegerNumber);
      T.Position:=StrToInt(CurrentTokenString);
      GetNextToken;
      end;
    Consume(tsqlAs);
    if (CurrentToken=tsqlDeclare) then
      ParseCreateProcedureVariableList(Result,T.LocalVariables);
    expect(tsqlBegin);
    ParseStatementBlock(Result,T.Statements);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseSetGeneratorStatement(AParent: TSQLElement
  ): TSQLSetGeneratorStatement;
begin
  // On entry, we're on the 'GENERATOR' token
  Consume(tsqlGenerator) ;
  try
    Result:=TSQLSetGeneratorStatement(CreateElement(TSQLSetGeneratorStatement,AParent));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Expect(tsqlidentifier);
    Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    Consume(tsqlto);
    Expect(tsqlIntegerNumber);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    expect(tsqlidentifier);
    Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    consume(tsqlto);
    expect(tsqlIntegerNumber);
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
    Result.NewValue:=StrToInt(CurrentTokenString);
    GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
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
function TSQLParser.ParseSecondaryFile(AParent: TSQLElement) : TSQLDatabaseFileInfo;

Var
  I : INteger;
  Last : TSQLToken;

begin
  // On entry, we're on the FILE token
  Consume(tsqlFile);
  Result:=TSQLDatabaseFileInfo(CreateElement(TSQLDatabaseFileInfo,APArent));
  try
    Expect(tsqlString);
    Result.FileName:=CurrentTokenString;
    getNextToken;
    I:=0;
    last:=tsqlEOF;
    While (I<2) and (CurrentToken in [tsqlLength,tsqlStarting]) do
      begin
      Inc(I);
      If (CurrentToken=tsqlLength) then
        begin
        If Last=tsqlLength then
          UnexpectedToken;
        Last:=tsqlLength;
        GetNextToken;
        if (CurrentToken=tsqlEq) then
          GetNextToken;
        Expect(tsqlIntegerNumber);
        Result.Length:=StrToInt(CurrentTokenString);
        GetNextToken;
        If CurrentToken in [tsqlPage,tsqlPages] then
          GetNextToken;
        end
      else if (CurrentToken=tsqlStarting) then
        begin
        If Last=tsqlStarting then
          UnexpectedToken;
        Last:=tsqlStarting;
        GetNextToken;
        if (CurrentToken=tsqlAt) then
          begin
          GetNextToken;
          If CurrentToken=tsqlPage then
            GetNextToken;
          end;
        Expect(tsqlIntegerNumber);
        Result.StartPage:=StrToInt(CurrentTokenString);
        GetNextToken;
        end;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseCreateDatabaseStatement(AParent: TSQLElement; IsAlter: Boolean) : TSQLCreateDatabaseStatement;

begin
  // On entry, we're on the DATABASE or SCHEMA token
  Result:=TSQLCreateDatabaseStatement(CreateElement(TSQLCreateDatabaseStatement,AParent));
  try
    Result.UseSchema:=(CurrentToken=tsqlSchema);
    GetNextToken;
    Expect(tsqlString);
    Result.FileName:=CurrentTokenString;
    GetNextToken;
    If (CurrentToken=tsqlUSER) then
      begin
      GetNextToken;
      Expect(tsqlString);
      Result.UserName:=CurrentTokenString;
      GetNextToken;
      end;
    If (CurrentToken=tsqlPassword) then
      begin
      GetNextToken;
      Expect(tsqlString);
      Result.Password:=CurrentTokenString;
      GetNextToken;
      end;
    If (CurrentToken=tsqlPageSize) then
      begin
      GetNextToken;
      if CurrentToken=tsqlEq then
        GetNextToken;
      Expect(tsqlIntegerNumber);
      Result.Pagesize:=StrtoIntDef(CurrentTokenString,0);
      GetNextToken;
      end;
    If (CurrentToken=tsqlLength) then
      begin
      GetNextToken;
      if (CurrentToken=tsqlEq) then
        GetNextToken;
      Expect(tsqlIntegerNumber);
      Result.Length:=StrtoIntDef(CurrentTokenString,0);
      GetNextToken;
      If CurrentToken in [tsqlPage,tsqlPages] then
        GetNextToken;
      end;
    If (CurrentToken=tsqlDefault) then
      begin
      GetNextToken;
      Consume(tsqlCharacter);
      Consume(tsqlSet);
      Expect(tsqlidentifier);
      Result.CharSet:=CreateIdentifier(Result,CurrentTokenString);
      GetNextToken;
      end;
    While (CurrentToken=tsqlFile) do
      Result.SecondaryFiles.Add(ParseSecondaryFile(Result));
  except
    FreeAndNil(Result);
    Raise
  end;
end;

function TSQLParser.ParseCreateShadowStatement(AParent: TSQLElement;
  IsAlter: Boolean): TSQLCreateShadowStatement;
begin
  // On entry, we're on the SHADOW token.
  if IsAlter then
    UnexpectedToken;
  Consume(tsqlShadow);
  Result:=TSQLCreateShadowStatement(CreateElement(TSQLCreateShadowStatement,AParent));
  try
    Expect(tsqlIntegerNumber);
    Result.Number:=StrToInt(CurrentTokenString);
    GetNextToken;
    If (CurrentToken=tsqlManual) then
      begin
      Result.Manual:=True;
      GetNextToken;
      end
    else If (CurrentToken=tsqlAuto) then
      GetNextToken;
    if (CurrentToken=tsqlConditional) then
      begin
      Result.Conditional:=True;
      GetNextToken;
      end;
    expect(tsqlString);
    Result.FileName:=CurrentTokenString;
    GetNextToken;
    If (CurrentToken=tsqlLength) then
      begin
      GetNextToken;
      if (CurrentToken=tsqlEq) then
        GetNextToken;
      Expect(tsqlIntegerNumber);
      Result.Length:=StrtoIntDef(CurrentTokenString,0);
      GetNextToken;
      If CurrentToken in [tsqlPage,tsqlPages] then
        GetNextToken;
      end;
    While (CurrentToken=tsqlFile) do
      Result.SecondaryFiles.Add(ParseSecondaryFile(Result));
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseAlterDatabaseStatement(AParent: TSQLElement;
  IsAlter: Boolean): TSQLAlterDatabaseStatement;
begin
  // On entry, we're on the DATABASE or SCHEMA token.
  Result:=TSQLAlterDatabaseStatement(CreateElement(TSQLAlterDatabaseStatement,APArent));
  try
    Result.UseSchema:=CurrentToken=tsqlSchema;
    GetNextToken;
    expect(tsqlAdd);
    While (CurrentToken in [tsqlAdd,tsqlFile]) do
      begin
      if CurrentToken=tsqlAdd then
        GetNextToken;
      Expect(tsqlFile);
      Result.Operations.Add(ParseSecondaryFile(Result));
      end;
    if Result.Operations.Count=0 then
      UnexpectedToken([tsqlAdd]);
  except
    FreeAndNil(Result);
    Raise;
  end;

end;

function TSQLParser.ParseCreateStatement(AParent: TSQLElement; IsAlter: Boolean
  ): TSQLCreateOrAlterStatement;
begin
  Case GetNextToken of
    tsqlTable      :  if IsAlter then
                        Result:=ParseAlterTableStatement(AParent)
                      else
                        Result:=ParseCreateTableStatement(AParent);

    tsqlUnique,
    tsqlAscending,
    tsqlDescending,
    tsqlIndex      : Result:=ParseCreateIndexStatement(AParent,IsAlter);
    tsqlView       : Result:=ParseCreateViewStatement(AParent,IsAlter);
    tsqlProcedure  : Result:=ParseCreateProcedureStatement(AParent,IsAlter);
    tsqlDomain     : Result:=ParseCreateDomainStatement(AParent,IsAlter);
    tsqlGenerator  : Result:=ParseCreateGeneratorStatement(AParent,IsAlter);
    tsqlException  : Result:=ParseCreateExceptionStatement(AParent,IsAlter);
    tsqlTrigger    : Result:=ParseCreateTriggerStatement(AParent,IsAlter);
    tsqlRole       : Result:=ParseCreateRoleStatement(AParent,IsAlter);
    tsqlSchema,
    tsqlDatabase   : If IsAlter then
                       Result:=ParseAlterDatabaseStatement(AParent,IsAlter)
                     else
                       Result:=ParseCreateDatabaseStatement(AParent,IsAlter);
    tsqlShadow       : Result:=ParseCreateShadowStatement(AParent,IsAlter);
  else
     Error(SErrExpectedDBObject,[CurrentTokenString]);
  end;
end;

function TSQLParser.ParseDropStatement(AParent: TSQLElement
  ): TSQLDropStatement;

Var
   C : TSQLElementClass;

begin
  // We're positioned on the DROP token.
  C:=Nil;
  Case GetNextToken of
    {
    Filter,
    }
    tsqlExternal : begin
                   GetNextToken;
                   Expect(tsqlFunction);
                   C:=TSQLDropExternalFunctionStatement;
                   end;
    tsqlShadow     : C:=TSQLDropShadowStatement;
    tsqlRole       : C:=TSQLDropRoleStatement;
    tsqlDatabase   : C:=TSQLDropDatabaseStatement;
    tsqlException  : C:=TSQLDropExceptionStatement;
    tsqlTable      : C:=TSQLDropTableStatement;
    tsqlIndex      : C:=TSQLDropIndexStatement;
    tsqlView       : C:=TSQLDropViewStatement;
    tsqlProcedure  : C:=TSQLDropProcedureStatement;
    tsqlDomain     : C:=TSQLDropDomainStatement;
    tsqlGenerator  : C:=TSQLDropGeneratorStatement;
    tsqlTrigger    : C:=TSQLDropTriggerStatement;
  else
     Error(SErrExpectedDBObject,[CurrentTokenString]);
  end;
  GetNextToken;
  If C=TSQLDropShadowStatement then
    Expect(tsqlIntegerNumber)
  else
    Expect(tsqlIdentifier);
  Result:=TSQLDropStatement(CreateElement(C,AParent));
  Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
  GetNextToken; // Comma
end;

function TSQLParser.ParseRollbackStatement(AParent: TSQLElement
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ): TSQLRollbackStatement;
=======
  ): TSQLRollBackStatement;
>>>>>>> graemeg/cpstrnew
=======
  ): TSQLRollBackStatement;
>>>>>>> graemeg/cpstrnew
=======
  ): TSQLRollBackStatement;
>>>>>>> graemeg/cpstrnew
=======
  ): TSQLRollBackStatement;
>>>>>>> origin/cpstrnew


begin
  // On entry, we're on the ROLLBACK statement
  Consume(tsqlRollBack);
  Result:=TSQLRollBackStatement(CreateElement(TSQLRollBackStatement,AParent));
  try
    If (CurrentToken=tsqlTransaction) then
      begin
      GetNextToken;
      expect(tsqlidentifier);
      Result.TransactionName:=CreateIdentifier(Result,CurrentTokenString);
      GetNextToken;
      end;
    Result.Work:=(CurrentToken=tsqlWork);
    if Result.Work then
      GetNextToken;
    Result.Release:=(CurrentToken=tsqlRelease);
    if Result.Release then
      GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseCommitStatement(AParent: TSQLElement
  ): TSQLCommitStatement;
begin
  Consume(tsqlCommit);
  Result:=TSQLCommitStatement(CreateElement(TSQLCommitStatement,AParent));
  try
    Result.Work:=(CurrentToken=tsqlWork);
    if Result.Work then
      GetNextToken;
    If (CurrentToken=tsqlTransaction) then
      begin
      GetNextToken;
      expect(tsqlidentifier);
      Result.TransactionName:=CreateIdentifier(Result,CurrentTokenString);
      GetNextToken;
      end;
    Result.Release:=(CurrentToken=tsqlRelease);
    if Result.Release then
      GetNextToken;
    Result.Retain:=(CurrentToken=tsqlRetain);
    if Result.Retain then
      begin
      GetNextToken;
      If CurrentToken=tsqlSnapshot then
        GetNextToken;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseExecuteProcedureStatement(AParent: TSQLElement): TSQLExecuteProcedureStatement;

Var
  NeedClose,
  Done : Boolean;
  TN : TSQLStringType;

begin
  Result:=Nil;
  // On Entry, we're on the EXECUTE statement
  Consume(tsqlExecute);
  Consume(tsqlProcedure);
  If (CurrentToken=tsqlTransaction) then
    begin
    GetNextToken;
    Expect(TSQLIdentifier);
    TN:=CurrentTokenString;
    GetNextToken;
    end;
  Expect(tsqlIdentifier);
  Result:=TSQLExecuteProcedureStatement(CreateELement(TSQLExecuteProcedureStatement,AParent));
  try
    Result.ProcedureName:=CreateIdentifier(Result,CurrentTokenString);
    if (TN<>'') then
      Result.TransactionName:=CreateIdentifier(Result,TN);
    GetNextToken;
    // ( is optional. It CAN be part of a (SELECT, and then it is NOT part of the brackets around the params.
    NeedClose:=(CurrentToken=tsqlBraceOpen) and (PeekNextToken<>tsqlSelect);
    If NeedClose then
      GetNextToken;
    Done:=False;
    If Not (CurrentToken in [tsqlSemicolon,tsqlEOF,tsqlReturningValues]) then
      Repeat
        Result.Params.Add(ParseExprLevel1(Result,[eoFieldValue]));
        If CurrentToken=tsqlComma then
          GetNextToken
        else if (CurrentToken=tsqlBraceClose) then
          begin
          if Not NeedClose then
            UnexpectedToken;
          Done:=True;
          GetNextToken;
          end
        else
          begin
          If NeedClose then
            UnexpectedToken([tsqlBraceClose]);
          Expect([tsqlEOF,tsqlSemicolon,tsqlReturningValues]);
          Done:=True;
          end;
      until Done;
    if (CurrentToken=tsqlReturningValues) then
      begin
      GetNextToken;
      NeedClose:=(CurrentToken=tsqlBraceOpen);
      If NeedClose then
        Consume(tsqlBraceOpen);
      Repeat
        if CurrentToken=tsqlComma then
          GetNextToken;
        if CurrentToken=tsqlColon then
          GetNextToken;
        Expect(tsqlIdentifier);
        Result.Returning.Add(CreateIdentifier(Result,CurrentTokenString));
        GetNextToken;
      until (CurrentToken<>tsqlComma);
      If NeedClose then
        Consume(tsqlBraceClose);
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseSetStatement(AParent: TSQLElement): TSQLStatement;
begin
  // On Entry, we're on the SET statement
  Consume(tsqlSet);
  Case CurrentToken of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tsqlGenerator : Result:=ParseSetGeneratorStatement(AParent); //SET GENERATOR
=======
    tsqlGenerator : Result:=ParseSetGeneratorStatement(AParent)
>>>>>>> graemeg/cpstrnew
=======
    tsqlGenerator : Result:=ParseSetGeneratorStatement(AParent)
>>>>>>> graemeg/cpstrnew
=======
    tsqlGenerator : Result:=ParseSetGeneratorStatement(AParent)
>>>>>>> graemeg/cpstrnew
=======
    tsqlGenerator : Result:=ParseSetGeneratorStatement(AParent)
>>>>>>> origin/cpstrnew
  else
    // For the time being
    UnexpectedToken;
  end;
end;

function TSQLParser.ParseConnectStatement(AParent: TSQLElement
  ): TSQLConnectStatement;
begin
  // On entry, we're on CONNECT
  consume(tsqlConnect);
  Expect(tsqlString);
  Result:=TSQLConnectStatement(CreateElement(TSQLConnectStatement,AParent));
  try
    Result.DatabaseName:=CurrentTokenString;
    GetNextToken;
    If CurrentToken=tsqlUSER then
      begin
      GetNextToken;
      Expect(tsqlString);
      Result.UserName:=CurrentTokenString;
      GetNextToken;
      end;
    If CurrentToken=tsqlPassword then
      begin
      GetNextToken;
      Expect(tsqlString);
      Result.Password:=CurrentTokenString;
      GetNextToken;
      end;
    If CurrentToken=tsqlRole then
      begin
      GetNextToken;
      Expect(tsqlString);
      Result.Role:=CurrentTokenString;
      GetNextToken;
      end;
    If CurrentToken=tsqlCache then
      begin
      GetNextToken;
      Expect(tsqlIntegerNumber);
      Result.Cache:=StrtoIntDef(CurrentTokenString,0);
      GetNextToken;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

constructor TSQLParser.Create(AInput: TStream);
begin
  FInput:=AInput;
  FCurrent:=TSQLUnknown;
  FScanner:=TSQLScanner.Create(FInput);
  FFreeScanner:=True;
end;

constructor TSQLParser.Create(AScanner: TSQLScanner);
begin
  FCurrent:=TSQLUnknown;
  FScanner:=AScanner;
  FFreeScanner:=False;
end;

destructor TSQLParser.Destroy;
begin
  If FFreeScanner then
    FreeAndNil(FScanner);
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseDeclareFunctionStatement(AParent: TSQLElement
  ): TSQLDeclareExternalFunctionStatement;
=======
Function TSQLParser.ParseDeclareFunctionStatement(AParent : TSQLElement) : TSQLDeclareExternalFunctionStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseDeclareFunctionStatement(AParent : TSQLElement) : TSQLDeclareExternalFunctionStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseDeclareFunctionStatement(AParent : TSQLElement) : TSQLDeclareExternalFunctionStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseDeclareFunctionStatement(AParent : TSQLElement) : TSQLDeclareExternalFunctionStatement;
>>>>>>> origin/cpstrnew

begin
  // On entry, we're on the EXTERNAL token
  Consume(tsqlExternal);
  Consume(tsqlFunction);
  Expect(tsqlidentifier);
  Result:=TSQLDeclareExternalFunctionStatement(CreateElement(TSQLDeclareExternalFunctionStatement,AParent));
  try
    Result.ObjectName:=CreateIdentifier(Result,CurrentTokenString);
    If (PeekNextToken=tsqlReturns) then
      GetNextToken
    else
      Repeat
        Result.Arguments.Add(Self.ParseTypeDefinition(Result,[ptfExternalFunction]));
      Until (CurrentToken<>tsqlComma);
    Expect(tsqlReturns);
    Result.ReturnType:=ParseTypeDefinition(Result,[ptfExternalFunctionResult]);
    Result.FreeIt:=(CurrentToken=tsqlFreeIt);
    If Result.FreeIt then
      GetNextToken;
    Consume(tsqlEntryPoint);
    Expect(tsqlString);
    Result.EntryPoint:=CurrentTokenString;
    GetNextToken;
    Consume(tsqlModuleName);
    Expect(tsqlString);
    Result.ModuleName:=CurrentTokenstring;
    GetNextToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLParser.ParseDeclareStatement(AParent: TSQLElement): TSQLStatement;
=======
Function TSQLParser.ParseDeclareStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseDeclareStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseDeclareStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> graemeg/cpstrnew
=======
Function TSQLParser.ParseDeclareStatement(AParent : TSQLElement) : TSQLStatement;
>>>>>>> origin/cpstrnew


begin
  // On entry, we're on the DECLARE statement
  Consume(tsqlDeclare);
  // For the moment, only 'DECLARE EXTERNAL FUNCTION' is supported
  Case CurrentToken of
    tsqlExternal : Result:=ParseDeclareFunctionStatement(AParent);
  else
    UnexpectedToken([tsqlExternal]);
  end;

end;

procedure TSQLParser.ParseGranteeList(AParent: TSQLElement;
  List: TSQLElementList; AllowObject, AllowGroup, AllowPublic: Boolean; IsRevoke: Boolean = False);

Type
  TSQLGranteeClass = Class of TSQLGrantee;

  Function CreateGrantee(NextIdentifier : Boolean; AClass : TSQLGranteeClass) : TSQLGrantee;
  begin
    if NextIdentifier then
      begin
      GetNextToken;
      Expect(tsqlIdentifier);
      end;
    Result:=TSQLGrantee(CreateElement(AClass,AParent));
    Result.Name:=CurrentTokenString;
    List.Add(Result);
  end;

Var
  E : TSQLTokens;

begin
  if IsRevoke then
    Consume(tsqlFrom)
  else
    Consume(tsqlTo);
  E:=[tsqlIdentifier,tsqlUser];
  If AllowObject then
    E:=E+[tsqlProcedure,tsqlView,tsqlTrigger,tsqlPublic]
  else If AllowPublic then
    E:=E+[tsqlPublic];
  If AllowGroup then
    E:=E+[tsqlGROUP];
  Expect(E);
  Repeat
    If CurrentToken=tsqlComma then
      GetNextToken;
    Case CurrentToken of
      tsqlUser,
      tsqlIdentifier :
          CreateGrantee(CurrentToken=tsqlUser,TSQLUserGrantee);
      TsqlGroup :
        begin
        If Not AllowGroup then
          UnexpectedToken;
        CreateGrantee(true,TSQLGroupGrantee);
        end;
      TsqlPublic :
        begin
        If Not (AllowPublic or AllowObject)  then
          UnexpectedToken;
        CreateGrantee(False,TSQLPublicGrantee);
        end;
      TsqlTrigger:
        begin
        If Not AllowObject then
          UnexpectedToken;
        CreateGrantee(True,TSQLTriggerGrantee);
        end;
      TsqlView:
        begin
        If Not AllowObject then
          UnexpectedToken;
        CreateGrantee(true,TSQLViewGrantee);
        end;
      TsqlProcedure:
        begin
        If Not AllowObject then
          UnexpectedToken;
        CreateGrantee(true,TSQLProcedureGrantee);
        end;
    end;
  Until (GetNextToken<>tsqlComma);

end;

function TSQLParser.ParseGrantTableStatement(AParent: TSQLElement): TSQLTableGrantStatement;

Var
  C : TSQLColumnPrivilege;
  P : TSQLPrivilege;

begin
  Result:=TSQLTableGrantStatement(CreateElement(TSQLTableGrantStatement,APArent));
  try
    // On entry, we're on the first ALL/SELECT/UPDATE/INSERT/DELETE/REFERENCE etc. token.
    if CurrentToken=tsqlAll then
      begin
      Result.Privileges.Add(CreateElement(TSQLAllPrivilege,Result));
      If GetNextToken=tsqlPrivileges then
        GetNextToken;
      end
    else
      Repeat
        P:=Nil;
        C:=Nil;
        if CurrentToken=tsqlComma then
          GetNextToken;
        Case CurrentToken of
          tsqlSelect : P:=TSQLSelectPrivilege(CreateElement(TSQLSelectPrivilege,Result));
          tsqlInsert : P:=TSQLInsertPrivilege(CreateElement(TSQLInsertPrivilege,Result));
          tsqlDelete : P:=TSQLDeletePrivilege(CreateElement(TSQLDeletePrivilege,Result));
          tsqlUpdate,
          tsqlReferences :
            begin
            if CurrentToken=tsqlUpdate then
              C:=TSQLUpdatePrivilege(CreateElement(TSQLUpdatePrivilege,AParent))
            else
              C:=TSQLReferencePrivilege(CreateElement(TSQLReferencePrivilege,AParent));
            P:=C;
            GetNextToken;
            If (CurrentToken=tsqlBraceOpen) then
              begin
              GetNextToken;
              C.Columns:=TSQLElementList.Create(True);
              ParseIdentifierList(C,C.Columns);
              end;
            end;
        else
          UnexpectedToken([tsqlselect,tsqlInsert,tsqlDelete,tsqlUpdate,tsqlReferences]);
        end;
        Result.Privileges.Add(P);
        If C=Nil then
          GetNextToken;
      Until (CurrentToken<>tsqlComma);
    Consume(tsqlOn);
    Expect(tsqlidentifier);
    Result.TableName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    ParseGranteeList(Result,Result.Grantees,True,True,True);
    If (CurrentToken=tsqlWith) then
      begin
      Consume(tsqlWith);
      Consume(tsqlGrant);
      Consume(tsqlOption);
      Result.GrantOption:=True;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseRevokeExecuteStatement(AParent: TSQLElement
  ): TSQLProcedureRevokeStatement;
BEGIN
  // On entry, we're on the EXECUTE token
  Consume(tsqlExecute);
  Consume(tsqlOn);
  Consume(tsqlProcedure);
  Expect(tsqlIdentifier);
  Result:=TSQLProcedureRevokeStatement(CreateElement(TSQLProcedureRevokeStatement,AParent));
  try
    Result.ProcedureName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    ParseGranteeList(Result,Result.Grantees,True,False,True,True);
    If (CurrentToken=tsqlWith) then
      begin
      Consume(tsqlWith);
      Consume(tsqlGrant);
      Consume(tsqlOption);
      Result.GrantOption:=True;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseRevokeRoleStatement(AParent: TSQLElement
  ): TSQLRoleRevokeStatement;
begin
  Result:=Nil;
  // On entry, we're on the identifier token
  expect(tsqlIdentifier);
  Result:=TSQLRoleRevokeStatement(CreateElement(TSQLRoleRevokeStatement,AParent));
  try
    Repeat
      if CurrentToken=tsqlComma then
        GetNextToken;
      expect(tsqlIdentifier);
      Result.Roles.Add(CreateIDentifier(Aparent,CurrentTokenString));
    Until (GetNextToken<>tsqlComma);
    Expect(tsqlFrom);
    ParseGranteeList(Result,Result.Grantees,False,False,True,True);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseRevokeTableStatement(AParent: TSQLElement
  ): TSQLTableRevokeStatement;
Var
  C : TSQLColumnPrivilege;
  P : TSQLPrivilege;

begin
  Result:=TSQLTableRevokeStatement(CreateElement(TSQLTableRevokeStatement,APArent));
  try
    // On entry, we're on the first GRANT,ALL/SELECT/UPDATE/INSERT/DELETE/REFERENCE etc. token.
    If (CurrentToken=tsqlGrant) then
      begin
      Consume(tsqlGrant);
      Consume(tsqlOption);
      Consume(tsqlFor);
      Result.GrantOption:=True;
      end;
    if CurrentToken=tsqlAll then
      begin
      Result.Privileges.Add(CreateElement(TSQLAllPrivilege,Result));
      If GetNextToken=tsqlPrivileges then
        GetNextToken;
      end
    else
      Repeat
        P:=Nil;
        C:=Nil;
        if CurrentToken=tsqlComma then
          GetNextToken;
        Case CurrentToken of
          tsqlSelect : P:=TSQLSelectPrivilege(CreateElement(TSQLSelectPrivilege,Result));
          tsqlInsert : P:=TSQLInsertPrivilege(CreateElement(TSQLInsertPrivilege,Result));
          tsqlDelete : P:=TSQLDeletePrivilege(CreateElement(TSQLDeletePrivilege,Result));
          tsqlUpdate,
          tsqlReferences :
            begin
            if CurrentToken=tsqlUpdate then
              C:=TSQLUpdatePrivilege(CreateElement(TSQLUpdatePrivilege,AParent))
            else
              C:=TSQLReferencePrivilege(CreateElement(TSQLReferencePrivilege,AParent));
            P:=C;
            GetNextToken;
            If (CurrentToken=tsqlBraceOpen) then
              begin
              GetNextToken;
              C.Columns:=TSQLElementList.Create(True);
              ParseIdentifierList(C,C.Columns);
              end;
            end;
        else
          UnexpectedToken([tsqlselect,tsqlInsert,tsqlDelete,tsqlUpdate,tsqlReferences]);
        end;
        Result.Privileges.Add(P);
        If C=Nil then
          GetNextToken;
      Until (CurrentToken<>tsqlComma);
    Consume(tsqlOn);
    Expect(tsqlidentifier);
    Result.TableName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    ParseGranteeList(Result,Result.Grantees,True,True,True,True);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseGrantExecuteStatement(AParent: TSQLElement): TSQLProcedureGrantStatement;

begin
  // On entry, we're on the EXECUTE token
  Consume(tsqlExecute);
  Consume(tsqlOn);
  Consume(tsqlProcedure);
  Expect(tsqlIdentifier);
  Result:=TSQLProcedureGrantStatement(CreateElement(TSQLProcedureGrantStatement,AParent));
  try
    Result.ProcedureName:=CreateIdentifier(Result,CurrentTokenString);
    GetNextToken;
    ParseGranteeList(Result,Result.Grantees,True,False,True);
    If (CurrentToken=tsqlWith) then
      begin
      Consume(tsqlWith);
      Consume(tsqlGrant);
      Consume(tsqlOption);
      Result.GrantOption:=True;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseGrantRoleStatement(AParent: TSQLElement): TSQLRoleGrantStatement;

begin
  Result:=Nil;
  // On entry, we're on the identifier token
  expect(tsqlIdentifier);
  Result:=TSQLRoleGrantStatement(CreateElement(TSQLRoleGrantStatement,AParent));
  try
    Repeat
      if CurrentToken=tsqlComma then
        GetNextToken;
      expect(tsqlIdentifier);
      Result.Roles.Add(CreateIDentifier(Aparent,CurrentTokenString));
    Until (GetNextToken<>tsqlComma);
    Expect(tsqlTo);
    ParseGranteeList(Result,Result.Grantees,False,False,True);
    If (CurrentToken=tsqlWith) then
      begin
      Consume(tsqlWith);
      Consume(tsqlAdmin);
      Consume(tsqlOption);
      Result.AdminOption:=True;
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseGrantStatement(AParent: TSQLElement): TSQLGrantStatement;

begin
  // On entry, we're on the GRANT token
  Result:=Nil;
  try
    Consume(tsqlGrant);
    Case CurrentToken of
      tsqlExecute: Result:=ParseGrantExecutestatement(AParent);
      tsqlAll,
      tsqlUpdate,
      tsqlReferences,
      tsqlInsert,
      tsqldelete,
      tsqlSelect : Result:=ParseGrantTablestatement(AParent);
      tsqlIdentifier : Result:=ParseGrantRolestatement(AParent);
    else
      UnExpectedToken([tsqlIdentifier, tsqlExecute, tsqlall,
                       tsqlUpdate, tsqldelete, tsqlReferences, tsqlInsert, tsqlSelect]);
    end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.ParseRevokeStatement(AParent: TSQLElement
  ): TSQLGrantStatement;
begin
  // On entry, we're on the GRANT token
  Result:=Nil;
  try
    Consume(tsqlRevoke);
    Case CurrentToken of
      tsqlExecute: Result:=ParseRevokeExecutestatement(AParent);
      tsqlGrant,
      tsqlAll,
      tsqlUpdate,
      tsqlReferences,
      tsqlInsert,
      tsqldelete,
      tsqlSelect : Result:=ParseRevokeTablestatement(AParent);
      tsqlIdentifier : Result:=ParseRevokeRolestatement(AParent);
    else
      UnExpectedToken([tsqlIdentifier, tsqlExecute,tsqlgrant,tsqlall,
                       tsqlUpdate, tsqldelete, tsqlReferences, tsqlInsert, tsqlSelect]);
    end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TSQLParser.Parse: TSQLElement;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if CurrentToken=tsqlEOF then begin
    Result:=nil;
    Exit;
  end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  GetNextToken;
  Case CurrentToken of
    tsqlSelect : Result:=ParseSelectStatement(Nil,[]);
    tsqlUpdate : Result:=ParseUpdateStatement(Nil);
    tsqlInsert : Result:=ParseInsertStatement(Nil);
    tsqlDelete : Result:=ParseDeleteStatement(Nil);
    tsqlCreate,
    tsqlAlter : Result:=ParseCreateStatement(Nil,(tsqlAlter=CurrentToken));
    tsqlDrop  : Result:=ParseDropStatement(Nil);
    tsqlSet : Result:=ParseSetStatement(Nil);
    tsqlRollback : Result:=ParseRollBackStatement(Nil);
    tsqlCommit : Result:=ParseCommitStatement(Nil);
    tsqlExecute : Result:=ParseExecuteProcedureStatement(Nil);
    tsqlConnect : Result:=ParseConnectStatement(Nil);
    tsqlDeclare : Result:=ParseDeclareStatement(Nil);
    tsqlGrant : Result:=ParseGrantStatement(Nil);
    tsqlRevoke : Result:=ParseRevokeStatement(Nil);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tsqlEOF : Result:=nil;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  else
    UnexpectedToken;
  end;
  if Not (CurrentToken in [tsqlEOF,tsqlSemicolon]) then
    begin
    FreeAndNil(Result);
    if (CurrentToken=tsqlBraceClose) then
      Error(SerrUnmatchedBrace);
    Error(SErrUnexpectedToken,[CurrentTokenString]);
    end;
end;

function TSQLParser.ParseScript(AllowPartial : Boolean = False): TSQLElementList;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  E : TSQLElement;
=======
Var
   E : TSQLElement;
>>>>>>> graemeg/cpstrnew
=======
Var
   E : TSQLElement;
>>>>>>> graemeg/cpstrnew
=======
Var
   E : TSQLElement;
>>>>>>> graemeg/cpstrnew
=======
Var
   E : TSQLElement;
>>>>>>> origin/cpstrnew

begin
  Result:=TSQLElementList.Create(True);
  try
    E:=Parse;
    While (E<>Nil) do
      begin
      Result.Add(E);
      E:=Parse;
      end;
  except
    If Not AllowPartial then
      begin
      FreeAndNil(Result);
      Raise;
      end;
  end;
end;

function TSQLParser.CurrentToken: TSQLToken;
begin
  Result:=FCurrent;
end;

function TSQLParser.CurrentTokenString: String;
begin
  Result:=FCurrentString;
end;

function TSQLParser.GetNextToken: TSQLToken;
begin
  FPrevious:=FCurrent;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // Set if not already peeked; otherwise fetch and look
  If (FPeekToken<>tsqlUnknown) then
    begin
    FCurrent:=FPeekToken;
    FCurrentString:=FPeekTokenString;
    FPeekToken:=tsqlUnknown;
    FPeekTokenString:='';
    end
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  If (FPeekToken<>tsqlunknown) then
     begin
     FCurrent:=FPeekToken;
     FCurrentString:=FPeekTokenString;
     FPeekToken:=tsqlUnknown;
     FPeekTokenString:='';
     end
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
  else
    begin
    FCurrent:=FScanner.FetchToken;
    FCurrentString:=FScanner.CurTokenString;
    end;
  Result:=FCurrent;
  {$ifdef debugparser}Writeln('GetNextToken : ',GetEnumName(TypeInfo(TSQLToken),Ord(FCurrent)), ' As string: ',FCurrentString);{$endif debugparser}
end;

function TSQLParser.PeekNextToken: TSQLToken;
begin
  If (FPeekToken=tsqlUnknown) then
    begin
    FPeekToken:=FScanner.FetchToken;
    FPeekTokenString:=FScanner.CurTokenString;
    end;
  {$ifdef debugparser}Writeln('PeekNextToken : ',GetEnumName(TypeInfo(TSQLToken),Ord(FPeekToken)), ' As string: ',FPeekTokenString);{$endif debugparser}
  Result:=FPeekToken;
end;

function TSQLParser.PreviousToken: TSQLToken;
begin
  Result:=FPRevious;
end;

function TSQLParser.IsEndOfLine: Boolean;
begin
  Result:=FScanner.IsEndOfLine;
end;

end.

