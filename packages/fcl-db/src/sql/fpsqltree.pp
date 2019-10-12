{
    This file is part of the Free Component Library
    Copyright (c) 2010-2014 by the Free Pascal development team

    SQL Abstract syntax tree

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpsqltree;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, contnrs;

Type
  TSQLStringType = String;
  TSQLFormatOption = (sfoDoubleQuotes,           // Use double quote character for string literals
                      sfoBackslashEscape,        // Backslash escapes in string literals
                      sfoSingleQuoteIdentifier,  // quote Identifiers using '
                      sfoDoubleQuoteIdentifier,  // quote Identifiers using " (e.g. as in Firebird)
                      sfoBackQuoteIdentifier,    // quote Identifiers using `
                      sfoLowercaseKeyword,       // Lowercase SQL keywords
                      sfoOneFieldPerLine,        // One field per line in SELECT, Update, Insert
                      sfoIndentFields,           // Indent fields (indent=2 space characters)
                      sfoOneTablePerLine,        // One table per line in select FROM clause
                      sfoIndentTables,           // Indent tables in FROM clause
                      sfoNoBracketRightJoin,     // In join, Do not put ( ) around right table if it is also a join
                      sfoBracketLeftJoin,        // In join, put ( ) around left table if it is also a join
                      sfoWhereOnSeparateLine,    // Put WHERE clause on a separate line
                      sfoIndentWhere,            // Indent WHERE clause
                      sfoOneGroupByFieldPerLine, // One field per line in GROUP BY
                      sfoIndentGroupByFields,    // Indent GROUP BY fields (indent=2 space characters)
                      sfoHavingOnSeparateLine,   // Put HAVING clause on a separate line
                      sfoIndentHaving,           // Indent HAVING clause
                      sfoUnionOnSeparateLine,    // Put UNION on separate line
                      sfoOneOrderByFieldPerLine, // One field per line in ORDER BY
                      sfoIndentOrderByFields,    // Indent ORDER BY fields (indent=2 space characters)
                      sfoPlanOnSeparateLine,     // Put HAVING clause on a separate line
                      sfoIndentPlan,             // Indent HAVING clause
                      sfoOneLogicalPerLine,      // in AND or OR clauses, put newline before AND or OR
                      sfoListNoSpaceBeforeComma, // In comma-separated lists, do not put space before ,
                      sfoListNoSpaceAfterComma,  // In comma-separated lists, do not put space after ,
                      sfoForceAscending,         // In ORDER BY, explicitly write ASC
                      sfoMultilineDeclareFunction, // Separate parts of 'Declare function' with newlines
                      sfoMultilineCreateDatabase,  // Separate parts of create/alter database with newlines
                      sfoMultilineCreateShadow,    // Separate additional filespecs of create/alter shadow with newlines
                      sfoIndentProcedureBlock      // Indent statements inside procedure/trigger statement block
                      );
  TSQLFormatOptions = Set of TSQLFormatOption;

Const
  sfoUseSeparateLines = [sfoOneFieldPerLine, sfoOneTablePerLine, sfoWhereOnSeparateLine,
                         sfoOneGroupByFieldPerLine, sfoHavingOnSeparateLine, sfoUnionOnSeparateLine,
                         sfoOneOrderByFieldPerLine, sfoPlanOnSeparateLine, sfoOneLogicalPerLine];
Const
  sfoUseIndentedLines = sfoUseSeparateLines // no use without it...
                        + [sfoIndentFields, sfoIndentTables, sfoIndentWhere,
                           sfoIndentGroupByFields, sfoIndentHaving, sfoIndentOrderByFields,
                           sfoIndentPlan];
Type

  { TSQLElement }

  TSQLElement = Class(TObject)
  private
    Fline: Integer;
    FParent: TSQLElement;
    FPos: Integer;
    FSource: String;
  Public
    Constructor Create(AParent : TSQLElement); virtual;
    destructor destroy; override;
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; virtual; abstract;
    Property Parent: TSQLElement Read FParent;
    Property Source : String Read FSource write FSource;
    Property SourceLine : Integer Read Fline Write Fline;
    Property SourcePos : Integer Read FPos Write FPos;
  end;
  TSQLElementClass = Class of TSQLElement;

  { TSQLElementList }

  TSQLElementList = Class(TObjectList)
  private
    function GetE(AIndex : Integer): TSQLElement;
    procedure SetE(AIndex : Integer; const AValue: TSQLElement);
  Public
    Property Elements[AIndex : Integer] : TSQLElement Read GetE Write SetE; default;
  end;

  TSQLLiteral = Class(TSQLElement);

  { TSQLNullLiteral }

  TSQLNullLiteral = Class(TSQLLiteral)
  Public
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLUserLiteral }

  TSQLUserLiteral = Class(TSQLLiteral)
  Public
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLValueLiteral }

  TSQLValueLiteral = Class(TSQLLiteral)
  Public
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLIntegerLiteral }

  TSQLIntegerLiteral = Class(TSQLLiteral)
  private
    FValue: Integer;
  Public
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Value : Integer Read FValue Write FValue;
  end;

  { TSQLFloatLiteral }

  TSQLFloatLiteral = Class(TSQLLiteral)
  private
    FValue: Double;
  Public
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Value : Double Read FValue Write FValue;
  end;

  { TSQLStringLiteral }

  TSQLStringLiteral = Class(TSQLLiteral)
  private
    FValue: TSQLStringType;
  Public
    function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Value : TSQLStringType Read FValue Write FValue;
  end;


  { TSQLIdentifierElement }

  TSQLIdentifierName = Class(TSQLElement)
  Private
    FName: TSQLStringType;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Name : TSQLStringType Read FName Write FName;
  end;

  TSQLCollation = Class(TSQLIdentifierName);

  { TSQLExpression }

  TSQLExpression = Class(TSQLElement)
  Public
    Function UseBrackets : Boolean; virtual;
  end;

  TSQLUnaryOperation = (uoNot,uoMinus);

  { TSQLLiteralExpression }

  TSQLLiteralExpression = Class(TSQLExpression)
  private
    FLiteral: TSQLLiteral;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Literal : TSQLLiteral Read FLiteral write FLiteral;
  end;

  { TSQLIdentifierExpression }

  TSQLIdentifierExpression = Class(TSQLExpression)
  private
    FElementIndex: Integer;
    FIdentifier: TSQLIdentifierName;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Identifier : TSQLIdentifierName Read FIdentifier Write FIdentifier;
    // For array types: index of element in array
    Property ElementIndex : Integer Read FElementIndex Write FElementIndex;
  end;

  { TSQLParameterExpression }

  TSQLParameterExpression = Class(TSQLExpression)
  private
    FIdentifier: TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Identifier : TSQLIdentifierName Read FIdentifier Write FIdentifier;
  end;

  { TSQLListExpression }

  TSQLListExpression = Class(TSQLExpression)
  private
    FList: TSQLElementList;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property List : TSQLElementList Read FList;
  end;

  TSQLSelectStatement = Class;

  { TSQLSelectionExpression }

  TSQLSelectionExpression = Class(TSQLExpression)
  private
    FSelect: TSQLSelectStatement;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Select : TSQLSelectStatement Read FSelect Write FSelect;
  end;

  { TSQLSelectExpression }

  TSQLSelectExpression = Class(TSQLSelectionExpression)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLExistsExpression }

  TSQLExistsExpression = Class(TSQLSelectionExpression)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLSingularExpression }

  TSQLSingularExpression = Class(TSQLSelectionExpression)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAllExpression }

  TSQLAllExpression = Class(TSQLSelectionExpression)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLSomeExpression }

  TSQLSomeExpression = Class(TSQLSelectionExpression)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAnyExpression }

  TSQLAnyExpression = Class(TSQLSelectionExpression)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLUnaryExpression }

  TSQLUnaryExpression = Class(TSQLExpression)
  private
    FOperand: TSQLExpression;
    FOperation: TSQLUnaryOperation;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Operation : TSQLUnaryOperation Read FOperation Write FOperation;
    Property Operand : TSQLExpression Read FOperand Write FOperand;
  end;

  TSQLBinaryOperation = (boAnd, boOr, boEQ, boLT, boGT, boLE, boGE, boNE,
                         boConcat,boAdd, boSubtract, boMultiply, boDivide, boIn,
                         boIs, boIsNot, boLike, boContaining, boStarting);

  { TSQLBinaryExpression }

  TSQLBinaryExpression = Class(TSQLExpression)
  private
    FLeft: TSQLExpression;
    FOperation: TSQLBinaryoperation;
    FRight: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function UseBrackets : Boolean; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Operation : TSQLBinaryoperation Read FOperation Write FOperation;
    Property Left : TSQLExpression Read FLeft Write FLeft;
    Property Right : TSQLExpression Read FRight Write FRight;
  end;

  TSQLTernaryOperation = (toLikeEscape,toBetween);

  { TSQLTernaryExpression }

  TSQLTernaryExpression = Class(TSQLExpression)
  private
    FLeft: TSQLExpression;
    FMiddle: TSQLExpression;
    FOperation: TSQLTernaryoperation;
    FRight: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function UseBrackets : Boolean; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Operation : TSQLTernaryoperation Read FOperation Write FOperation;
    Property Left : TSQLExpression Read FLeft Write FLeft;
    Property Middle : TSQLExpression Read FMiddle Write FMiddle;
    Property Right : TSQLExpression Read FRight Write FRight;
  end;

  { TSQLGenIDExpression }

  TSQLGenIDExpression = Class(TSQLExpression)
  private
    FIdentifier: TSQLIdentifierName;
    FValue: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Generator : TSQLIdentifierName Read FIdentifier Write FIdentifier;
    Property Value : TSQLExpression Read FValue Write FValue;
  end;

  { TSQLFunctionCallExpression }

  TSQLFunctionCallExpression = Class(TSQLExpression)
  private
    FArguments:TSQLElementList;
    FIdentifier: TSQLStringType;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Identifier : TSQLStringType Read FIdentifier Write FIdentifier;
    Property Arguments : TSQLElementList Read FArguments Write Farguments;
  end;

  TSQLAggregateFunction = (afCount,afSum,afAVG,afMax,afMin);
  TSQLAggregateOption = (aoNone,aoAsterisk,aoAll,aoDistinct);

  { TSQLAggregateFunctionExpression }

  TSQLAggregateFunctionExpression = Class(TSQLExpression)
  private
    Fagg: TSQLAggregateFunction;
    FExpression: TSQLExpression;
    FOption: TSQLAggregateOption;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Aggregate : TSQLAggregateFunction Read Fagg Write Fagg;
    Property Expression : TSQLExpression Read FExpression Write FExpression;
    Property Option :TSQLAggregateOption Read FOption Write FOption;
  end;

  { TSQLForeignKeyDefinition }
  // fkaNone when none was specified. fkaNoAction is when NO ACTION is specified.
  TForeignKeyAction = (fkaNone,fkaNoAction,fkaCascade,fkaSetDefault,fkaSetNull);
  TSQLForeignKeyDefinition = Class(TSQLElement)
  private
    FFieldList: TSQLElementList;
    FOnDelete: TForeignKeyAction;
    FOnUpdate: TForeignKeyAction;
    FTableName: TSQLIdentifierName;
  Public
    Constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
    Property FieldList : TSQLElementList Read FFieldList;
    Property OnDelete : TForeignKeyAction Read FOnDelete Write FOnDelete;
    Property OnUpdate : TForeignKeyAction Read FOnUpdate Write FOnUpdate;
  end;


  { TSQLFieldConstraint }

  { TSQLConstraintDef }

  TSQLConstraintDef = Class(TSQLElement)
  private
    FConstraintName: TSQLIdentifierName;
  Public
    Destructor Destroy;override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    property ConstraintName : TSQLIdentifierName Read FConstraintName Write FConstraintName;
  end;
  TSQLFieldConstraint = Class(TSQLConstraintDef);

  { TSQLUniqueFieldConstraint }

  TSQLUniqueFieldConstraint = Class(TSQLFieldConstraint)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLPrimaryKeyFieldConstraint }

  TSQLPrimaryKeyFieldConstraint = Class(TSQLFieldConstraint)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;


  { TSQLForeignKeyFieldConstraint }

  TSQLForeignKeyFieldConstraint = Class(TSQLFieldConstraint)
  private
    FDef: TSQLForeignKeyDefinition;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Definition : TSQLForeignKeyDefinition Read FDef Write FDEf;
  end;

  { TSQLCheckFieldConstraint }

  TSQLCheckFieldConstraint = Class(TSQLFieldConstraint)
  private
    FExpression: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Expression : TSQLExpression Read FExpression Write FExpression;
  end;

  { TSQLFieldConstraintList }

  TSQLFieldConstraintList = Class(TSQLFieldConstraint)
  private
    FList : TSQLElementList;
  public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property List : TSQLElementList Read FList;
  end;
  TSQLTableConstraintDef = Class(TSQLConstraintDef);

  { TSQLTableCheckConstraintDef }

  TSQLTableCheckConstraintDef = Class(TSQLTableConstraintDef)
  private
    FCheck: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Check : TSQLExpression Read FCheck Write FCheck;
  end;

  { TSQLTableFieldsConstraintDef }

  TSQLTableFieldsConstraintDef = Class(TSQLTableConstraintDef)
  private
    FFieldList: TSQLElementList;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function FieldListSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
    Property FieldList : TSQLElementList Read FFieldList;
  end;

  { TSQLTableUniqueConstraintDef }

  TSQLTableUniqueConstraintDef = Class(TSQLTableFieldsConstraintDef)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLTablePrimaryKeyConstraintDef }

  TSQLTablePrimaryKeyConstraintDef = Class(TSQLTableFieldsConstraintDef)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLTableForeignKeyConstraintDef }

  TSQLTableForeignKeyConstraintDef = Class(TSQLTableFieldsConstraintDef)
  private
    FDef: TSQLForeignKeyDefinition;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Definition : TSQLForeignKeyDefinition Read FDef Write FDEf;
  end;

  { TSQLTypeDefinition }
  TSQLDataType = (sdtDomain,sdtSmallInt,sdtInteger,sdtFloat,sdtDoublePrecision,
                  sdtDecimal, sdtNumeric, sdtDate,sdtDateTime,sdtTime,
                  sdtChar,sdtVarChar, sdtNChar, sdtNVarChar, sdtCstring,
                  sdtBlob);

  TArrayDim = Array[1..2] of Integer;
  TArrayDims = Array of TArrayDim;

  TSQLTypeDefinition = Class(TSQLElement)
  private
    FArrayDims: TArrayDims;
    FBlobType: Integer;
    FByValue: Boolean;
    FCharSet: TSQLStringType;
    FCollation: TSQLCollation;
    FCheck: TSQLExpression;
    FConstraint : TSQLFieldConstraint;
    FDataType: TSQLDataType;
    FDefault: TSQLLiteral;
    FNotNull: Boolean;
    Flen: Integer;
    FScale: Byte;
    FtypeName: String;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property DataType : TSQLDataType Read FDataType Write FDataType;
    Property TypeName : String Read FtypeName Write FTypeName;
    Property Len : Integer Read Flen Write Flen; // Length of string or precision for BCD
    Property Scale : Byte Read FScale Write FScale;
    Property ArrayDims : TArrayDims Read FArrayDims Write FArrayDims;
    Property BlobType : Integer Read FBlobType Write FBlobType;
    Property NotNull : Boolean Read FNotNull Write FNotNull;
    Property Collation : TSQLCollation Read FCollation Write FCollation;
    Property Check : TSQLExpression Read FCheck Write FCheck;
    Property DefaultValue : TSQLLiteral Read FDefault Write FDefault;
    Property Charset : TSQLStringType Read FCharSet Write FCharset;
    Property Constraint : TSQLFieldConstraint Read FConstraint Write FConstraint;
    Property ByValue : Boolean Read FByValue Write FByValue;
  end;

  { TSQLCastExpression }

  TSQLCastExpression =Class(TSQLExpression)
  private
    FNewType: TSQLTypeDefinition;
    FValue: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Value : TSQLExpression Read FValue Write FValue;
    Property NewType : TSQLTypeDefinition Read FNewType Write FNewType;
  end;

  { TSQLExtractExpression }
  TSQLExtractElement = (eeYear,eeMonth,eeDay,eeHour,eeMinute,eeSecond,eeWeekDay,eeYearDay);
  TSQLExtractExpression =Class(TSQLExpression)
  private
    FElement : TSQLExtractElement;
    FValue : TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Value : TSQLExpression Read FValue Write FValue;
    Property Element: TSQLExtractElement Read FElement Write FElement;
  end;


  TSQLStatement = Class(TSQLElement);

  TSQLDMLStatement = Class(TSQLStatement);
  TSQLDDLStatement = Class(TSQLStatement);


  { TSelectField }

  TSQLSelectElement = Class(TSQLElement);
  TSQLSelectAsterisk = Class(TSQLSelectElement);

  { TSQLSelectField }

  TSQLSelectField = Class(TSQLSelectElement)
  private
    FAliasName: TSQLIdentifierName;
    FExpression: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Expression : TSQLExpression Read FExpression Write FExpression;
    Property AliasName : TSQLIdentifierName Read FAliasName Write FAliasName;
  end;

  { TSQLTableReference }

  TSQLTableReference = Class(TSQLElement);

  { TSQLSimpleTableReference }

  TSQLSimpleTableReference = Class(TSQLTableReference)
  private
    FAliasName: TSQLIdentifierName;
    FObjectName: TSQLIdentifierName;
    FParams: TSQLElementList;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ObjectName : TSQLIdentifierName Read FObjectName Write FObjectName;
    Property Params : TSQLElementList Read FParams Write FParams;
    Property AliasName : TSQLIdentifierName Read FAliasName Write FAliasName;
  end;

  { TSQLJoinTableReference }
  TSQLJoinType = (jtNone,jtInner,jtLeft,jtRight,jtFullOuter);
  TSQLJoinTableReference = Class(TSQLTableReference)
  private
    FJoinClause: TSQLExpression;
    FJoinType: TSQLJoinType;
    FLeft: TSQLTablereference;
    FRight: TSQLTableReference;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Left : TSQLTablereference Read FLeft Write FLeft;
    Property Right : TSQLTableReference Read FRight Write FRight;
    Property JoinType : TSQLJoinType Read FJoinType Write FJoinType;
    Property JoinClause : TSQLExpression Read FJoinClause Write FJoinClause;
  end;

  TSQLSelectPlan = Class(TSQLElement);

  { TSQLSelectPlanItem }

  TSQLSelectPlanItem = Class(TSQLSelectPlan)
  private
    FTableName: TSQLIdentifierName;
  Public
    Destructor Destroy; Override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
  end;

  { TSQLSelectNaturalPlan }

  TSQLSelectNaturalPlan = Class(TSQLSelectPlanItem)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLSelectIndexedPlan }

  TSQLSelectIndexedPlan = Class(TSQLSelectPlanItem)
  private
    FIndexes: TSQLElementList;
  Public
    Constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; Override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Indexes : TSQLElementList Read FIndexes;
  end;

  { TSQLSelectOrderedPlan }

  TSQLSelectOrderedPlan = Class(TSQLSelectPlanItem)
  private
    FIndex: TSQLIdentifierName;
  Public
    Destructor Destroy; Override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property OrderIndex : TSQLIdentifierName Read FIndex Write FIndex;
  end;

  TPlanJoinType = (pjtJoin,pjtSort,pjtMerge);

  { TSQLSelectPlanExpr }

  TSQLSelectPlanExpr = Class(TSQLSelectPlan)
  private
    FItems: TSQLElementList;
    FJoinType: TPLanJoinType;
  Public
    Constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Items : TSQLElementList Read FItems;
    Property JoinType : TPLanJoinType Read FJoinType Write FJOinType;
  end;

  TSQLOrderDirection = (obAscending,obDescending);

  { TSQLOrderByElement }

  TSQLOrderByElement = Class(TSQLElement)
  private
    FCollation: TSQLIdentifierName;
    FField: TSQLElement;
    FOrderBy: TSQLOrderDirection;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Field : TSQLElement Read FField Write FField;
    Property Collation : TSQLIdentifierName Read FCollation Write FCollation;
    Property OrderBy : TSQLOrderDirection Read FOrderBy write FOrderBy;
  end;

  { TSQLSelectStatement }

  TSQLSelectStatement = Class(TSQLDMLStatement)
  private
    FAll: Boolean;
    FDistinct: Boolean;
    FEndAt: TSQLExpression;
    FFields: TSQLElementList;
    FForUpdate: TSQLElementList;
    FInto: TSQLElementList;
    FOrderBy: TSQLElementList;
    FGroupBy: TSQLElementList;
    FHaving: TSQLExpression;
    FPlan: TSQLSelectPlan;
    FStartAt: TSQLExpression;
    FTables: TSQLElementList;
    FTN: TSQLIdentifierName;
    FUnion: TSQLSelectStatement;
    FUnionAll: Boolean;
    FWhere: TSQLExpression;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TransactionName : TSQLIdentifierName Read FTN Write FTN;
    Property Tables : TSQLElementList Read FTables;
    Property Fields : TSQLElementList Read FFields;
    Property Where : TSQLExpression read FWhere write FWhere;
    Property GroupBy : TSQLElementList Read FGroupBy;
    Property Having : TSQLExpression Read FHaving Write FHaving;
    Property Orderby : TSQLElementList Read FOrderBy;
    Property ForUpdate : TSQLElementList Read FForUpdate Write FForUpdate;
    Property Union : TSQLSelectStatement Read FUnion Write FUnion;
    Property Plan : TSQLSelectPlan Read FPlan Write FPlan;
    Property Distinct : Boolean Read FDistinct Write FDistinct;
    Property All : Boolean Read FAll Write FAll;
    Property UnionAll : Boolean Read FUnionAll Write FUnionAll;
    property StartAt : TSQLExpression Read FStartAt Write FStartAt;
    Property EndAt : TSQLExpression Read FEndAt Write FEndAt;
    Property Into : TSQLElementList Read FInto Write FInto;
  end;

  { TSQLInsertStatement }

  { TSQLTableDMLStatement }

  TSQLTableDMLStatement = Class(TSQLDMLStatement)
  private
    FTableName: TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
  end;

  TSQLInsertStatement = Class(TSQLTableDMLStatement)
  private
    FFields: TSQLElementList;
    FSelect: TSQLSelectStatement;
    FValues: TSQLElementList;
    procedure SetSelect(const AValue: TSQLSelectStatement);
    procedure SetValues(const AValue: TSQLElementList);
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Fields : TSQLElementList Read FFields Write FFields;
    Property Values : TSQLElementList Read FValues Write SetValues;
    Property Select : TSQLSelectStatement Read FSelect Write SetSelect;
  end;

  { TSQLUpdatePair }

  TSQLUpdatePair = Class(TSQLElement)
  private
    FFieldName: TSQLIdentifierName;
    FValue: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property FieldName : TSQLIdentifierName Read FFieldName Write FFieldName;
    Property Value : TSQLExpression Read FValue Write FValue;
  end;

  { TSQLUpdateStatement }

  TSQLUpdateStatement = Class(TSQLTableDMLStatement)
  private
    FValues: TSQLElementList;
    FWhereClause: TSQLExpression;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Values : TSQLElementList Read FValues;
    Property WhereClause : TSQLExpression Read FWhereClause Write FWhereClause;
  end;

  { TSQLDeleteStatement }

  TSQLDeleteStatement = Class(TSQLDMLStatement)
  private
    FAliasName: TSQLIdentifierName;
    FTableName: TSQLIdentifierName;
    FWhereClause: TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
    Property AliasName : TSQLIdentifierName Read FAliasName Write FAliasName;
    Property WhereClause : TSQLExpression Read FWhereClause Write FWhereClause;
  end;


  { TSQLTransactionStatement }

  TSQLTransactionStatement = Class(TSQLStatement)
  private
    FRelease: Boolean;
    FTransactionName: TSQLIdentifierName;
    FWork: Boolean;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TransactionName : TSQLIdentifierName Read FTransactionName Write FTransactionName;
    Property Work : Boolean Read FWork Write FWork;
    Property Release : Boolean Read FRelease Write FRelease;
  end;

  { TSQLRollBackStatement }

  TSQLRollBackStatement = Class(TSQLTransactionStatement)
  public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLCommitStatement }

  TSQLCommitStatement  = Class(TSQLTransactionStatement)
  private
    FRetain: Boolean;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Retain : Boolean Read FRetain Write FRetain;
  end;

  { TSQLExecuteProcedureStatement }

  TSQLExecuteProcedureStatement = Class(TSQLStatement)
  private
    FOutParams: TSQLElementList;
    FParams: TSQLElementList;
    FPN: TSQLIdentifierName;
    FTN: TSQLIdentifierName;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TransactionName : TSQLIdentifierName Read FTN Write FTN;
    Property ProcedureName : TSQLIdentifierName Read FPN Write FPN;
    Property Params : TSQLElementList Read FParams;
    Property Returning : TSQLElementList Read FOutParams;
  end;

  { TSQLCreateOrAlterStatement }

  TSQLCreateOrAlterStatement = Class(TSQLDDLStatement)
  private
    FDBO: TSQLIdentifierName;
    FIsCreateOrAlter: Boolean;
    FIsReCreate: Boolean;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ObjectName : TSQLIdentifierName Read FDBO Write FDBO;
    Property IsCreateOrAlter : Boolean Read FIsCreateOrAlter Write FIsCreateOrAlter;
    Property IsRecreate : Boolean Read FIsReCreate Write FIsReCreate;
  end;

  { Generator }

  TSQLCreateOrAlterGenerator = Class(TSQLCreateOrAlterStatement)
  Private
    FIsIsSequence: Boolean;
  public
    Property IsSequence : Boolean Read FIsIsSequence Write FIsIsSequence;
  end;

  { TSQLCreateGeneratorStatement }

  TSQLCreateGeneratorStatement = Class(TSQLCreateOrAlterGenerator)
  public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;


  { TAlterGeneratorStatement }

  TSQLAlterGeneratorStatement = Class(TSQLCreateOrAlterGenerator)
  private
    FHasRestart: Boolean;
    FRestart: Int64;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Restart : Int64 Read FRestart Write FRestart;
    Property HasRestart : Boolean Read FHasRestart Write FHasRestart;
  end;
  { TSQLSetGeneratorStatement }

  TSQLSetGeneratorStatement = Class(TSQLCreateOrAlterGenerator)
  private
    FNewValue: Integer;
  Public
    Property NewValue : Integer Read FNewValue Write FNewValue;
  end;

  { TSQLSetISQLStatement }
  // SET statements as used by the isql Firebird command line utility
  TSQLSetISQLStatement = Class(TSQLStatement)
  private
    FArgument: string;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    // The test of the SET statement excluding the SET command
    Property Arguments : string Read FArgument Write FArgument;
  end;

  { TSQLCreateRoleStatement }

  TSQLCreateRoleStatement = Class(TSQLCreateOrAlterStatement)
  public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TDomain }

  { TSQLCreateDomainStatement }

  TSQLCreateDomainStatement = Class(TSQLCreateOrAlterStatement)
  private
    FType: TSQLTypeDefinition;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TypeDefinition : TSQLTypeDefinition Read FType Write FType;
  end;

  { TSQLAlterDomainStatement }

  TSQLAlterDomainStatement = Class(TSQLCreateOrAlterStatement)
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAlterDomainDropDefaultStatement }

  TSQLAlterDomainDropDefaultStatement = Class(TSQLAlterDomainStatement)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAlterDomainDropCheckStatement }

  TSQLAlterDomainDropCheckStatement = Class(TSQLAlterDomainStatement)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAlterDomainSetDefaultStatement }

  TSQLAlterDomainSetDefaultStatement = Class(TSQLAlterDomainStatement)
  private
    FDefault: TSQLLiteral;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property DefaultValue : TSQLLiteral Read FDefault Write FDefault;
  end;

  { TSQLAlterDomainRenameStatement }

  TSQLAlterDomainRenameStatement = Class(TSQLAlterDomainStatement)
  private
    FNewName : TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property NewName : TSQLIdentifierName Read FNewName Write FNewName;
  end;

  { TSQLAlterDomainTypeStatement }

  TSQLAlterDomainTypeStatement = Class(TSQLAlterDomainStatement)
  private
    FNewtype: TSQLTypeDefinition;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property NewType : TSQLTypeDefinition Read FNewtype Write FNewType;
  end;

  { TSQLAlterDomainAddCheckStatement }

  TSQLAlterDomainAddCheckStatement = Class(TSQLAlterDomainStatement)
  private
    FCheck : TSQLExpression;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Check : TSQLExpression Read FCheck Write FCheck;
  end;

  { TSQLCreateExceptionStatement }

  TSQLCreateExceptionStatement = Class(TSQLCreateOrAlterStatement)
  private
    FMessage: TSQLStringLiteral;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ExceptionMessage : TSQLStringLiteral Read Fmessage Write FMessage;
  end;
  TSQLAlterExceptionStatement = Class(TSQLCreateExceptionStatement);

  TIndexOption = (ioAscending,ioDescending,ioUnique);
  TIndexOptions = Set of TIndexOption;

  { TSQLCreateIndexStatement }
  TSQLCreateOrAlterIndexStatement = Class(TSQLCreateOrAlterStatement);
  TSQLCreateIndexStatement = Class(TSQLCreateOrAlterIndexStatement)
  private
    FOptions: TIndexOptions;
    FFieldNames: TSQLElementList;
    FTableName: TSQLIdentifierName;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Options : TIndexOptions Read FOptions Write FOptions;
    Property FieldNames : TSQLElementList Read FFieldNames;
  end;

  { TSQLAlterIndexStatement }

  TSQLAlterIndexStatement = Class(TSQLCreateOrAlterIndexStatement)
  private
    FInactive: Boolean;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Inactive : Boolean Read FInactive Write FInactive;
  end;

  { TDeclareExternalFunctionStatement }

  { TSQLDeclareExternalFunctionStatement }

  TSQLDeclareExternalFunctionStatement = Class(TSQLCreateOrAlterStatement)
  private
    FArguments: TSQLElementList;
    FEntryPoint: TSQLStringType;
    FFreeIt: Boolean;
    FModuleName: TSQLStringType;
    FReturnType: TSQLTypeDefinition;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ModuleName : TSQLStringType Read FModuleName Write FModuleName;
    Property EntryPoint : TSQLStringType Read FEntryPoint Write FEntryPoint;
    Property ReturnType : TSQLTypeDefinition Read FReturnType Write FReturnType;
    Property Arguments : TSQLElementList Read FArguments;
    Property FreeIt : Boolean Read FFreeIt Write FFreeit;
  end;

  { TSQLTableFieldDef }

  TSQLTableFieldDef = Class(TSQLElement)
  private
    FComputedBy: TSQLExpression;
    FFieldName: TSQLIdentifierName;
    FFieldType: TSQLTypeDefinition;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property FieldName : TSQLIdentifierName Read FFieldName Write FFieldName;
    Property FieldType : TSQLTypeDefinition Read FFieldType Write FFieldType;
    Property ComputedBy : TSQLExpression Read FComputedBy Write FComputedBy;
  end;

  { TSQLCreateOrAlterTableStatement }

  TSQLCreateOrAlterTableStatement = Class(TSQLCreateOrAlterStatement);

  { TSQLCreateTableStatement }

  TSQLCreateTableStatement = Class(TSQLCreateOrAlterTableStatement)
  private
    FConstraints: TSQLElementList;
    FExternalFile: TSQLStringLiteral;
    FFieldDefs: TSQLElementList;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ExternalFileName : TSQLStringLiteral Read FExternalFile Write FExternalFile;
    Property FieldDefs : TSQLElementList Read FFieldDefs;
    Property Constraints : TSQLElementList Read FConstraints;
  end;

  { TSQLAlterTableOperation }
  TSQLAlterTableOperation = Class(TSQLElement)
  private
    FName: TSQLIdentifierName;
  public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ObjectName : TSQLIdentifierName Read FName Write FName;
  end;

  { TSQLDropTableElementOperation }

  TSQLDropTableElementOperation = Class(TSQLAlterTableOperation);

  { TSQLDropTableFieldOperation }

  TSQLDropTableFieldOperation = Class(TSQLDropTableElementOperation)
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLDropTableConstraintOperation }

  TSQLDropTableConstraintOperation = Class(TSQLDropTableElementOperation)
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAlterTableFieldNameOperation }

  TSQLAlterTableFieldNameOperation = Class(TSQLAlterTableOperation)
  private
    FNewName: TSQLIdentifierName;
  public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property NewName : TSQLIdentifierName Read FNewName Write FNewName;
  end;

  { TSQLAlterTableFieldTypeOperation }

  TSQLAlterTableFieldTypeOperation = Class(TSQLAlterTableOperation)
  private
    FNewtype: TSQLTypeDefinition;
  public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property NewType : TSQLTypeDefinition Read FNewType Write FNewType;
  end;

  { TSQLAlterTableFieldPositionOperation }

  TSQLAlterTableFieldPositionOperation = Class(TSQLAlterTableOperation)
  private
    FNewPosition: Integer;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property NewPosition : Integer Read FNewPosition Write FNewPosition;
  end;

  { TSQLAddTableElementOperation }

  TSQLAlterTableAddElementOperation = Class(TSQLElement)
  private
    FElement: TSQLElement;
  public
    Destructor Destroy; override;
    Property Element : TSQLElement Read FElement Write FElement;
  end;

  { TSQLAlterTableAddFieldOperation }

  TSQLAlterTableAddFieldOperation = Class(TSQLAlterTableAddElementOperation)
  private
    function GetF: TSQLTableFieldDef;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property FieldDef : TSQLTableFieldDef Read GetF;
  end;

  { TSQLAlterTableAddConstraintOperation }

  TSQLAlterTableAddConstraintOperation = Class(TSQLAlterTableAddElementOperation)
  private
    function GetC: TSQLTableConstraintDef;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ConstraintDef : TSQLTableConstraintDef Read GetC;
  end;

  { TSQLAlterTableStatement }

  TSQLAlterTableStatement = Class(TSQLCreateOrAlterTableStatement)
  private
    FOperations: TSQLElementList;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Operations : TSQLElementList Read FOperations;
  end;

  { TSQLAlterCreateViewStatement }

  TSQLAlterCreateViewStatement = Class(TSQLCreateOrAlterStatement)
  private
    FFields: TSQLElementList;
    FSelect: TSQLSelectStatement;
    FWCo: Boolean;
  Public
    Constructor Create(APArent : TSQLElement); override;
    destructor destroy; override;
    Property Fields : TSQLElementList Read FFields;
    Property Select : TSQLSelectStatement Read FSelect Write FSelect;
    Property WithCheckOption : Boolean Read FWCo Write FWCO;
  end;

  { TSQLCreateViewStatement }

  TSQLCreateViewStatement = Class(TSQLAlterCreateViewStatement)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAlterCreateDatabaseStatement }

  TSQLAlterCreateDatabaseStatement = Class(TSQLCreateOrAlterStatement)
  private
    FUseSchema: Boolean;
  Public
    Property UseSchema : Boolean Read FUseSchema write FUseSchema;
  end;

  { TSQLDatabaseFileInfo }

  TSQLDatabaseFileInfo =Class(TSQLElement)
  private
    FFileName: TSQLStringType;
    FLength: Integer;
    FStartPage: Integer;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property FileName : TSQLStringType Read FFileName Write FFileName;
    Property Length : Integer Read FLength Write Flength;
    Property StartPage : Integer Read FStartPage Write FStartPage;
  end;

  { TSQLCreateDatabaseStatement }

  TSQLCreateDatabaseStatement = Class(TSQLAlterCreateDatabaseStatement)
  private
    FCharSet: TSQLIdentifierName;
    FFileName: TSQLStringType;
    Flength: INteger;
    FPageSize: Integer;
    FPassword: TSQLStringType;
    FSecondaryFiles: TSQLElementList;
    FUserName: TSQLStringType;
  Public
    constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property FileName : TSQLStringType Read FFileName Write FFileName;
    Property UserName : TSQLStringType Read FUserName Write FUserName;
    property Password : TSQLStringType Read FPassword write FPassword;
    Property PageSize : Integer Read FPageSize Write FPageSize;
    Property Length  : INteger Read Flength Write Flength;
    Property CharSet : TSQLIdentifierName Read FCharSet Write FCharset;
    Property SecondaryFiles : TSQLElementList Read FSecondaryFiles;
  end;

  { TSQLAlterDatabaseStatement }

  TSQLAlterDatabaseStatement = Class(TSQLAlterCreateDatabaseStatement)
  private
    FOperations: TSQLElementList;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Operations : TSQLElementList Read FOperations Write FOperations;
  end;

  { TSQLCreateShadowStatement }

  TSQLCreateShadowStatement = Class(TSQLCreateOrAlterStatement)
  private
    FConditional: Boolean;
    FFileName: String;
    FLength: Integer;
    FManual: Boolean;
    FNumber: Integer;
    FSecondaryFiles: TSQLElementList;
  Public
    constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Number : Integer Read FNumber Write FNumber;
    Property Manual : Boolean Read FManual Write FManual;
    Property Conditional : Boolean Read FConditional Write FConditional;
    Property FileName : TSQLStringtype Read FFileName write FFileName;
    Property Length : Integer Read FLength Write FLength;
    Property SecondaryFiles : TSQLElementList Read FSecondaryFiles;
  end;

  { TSQLProcedureParamDef }

  TSQLProcedureParamDef = Class(TSQLElement)
  private
    FParamName: TSQLIdentifierName;
    FParamType: TSQLTypeDefinition;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ParamName : TSQLIdentifierName Read FParamName Write FParamName;
    Property ParamType : TSQLTypeDefinition Read FParamType Write FParamType;
  end;

  { TSQLStatementBlock }

  TSQLStatementBlock = Class(TSQLStatement)
  private
    FStatements: TSQLElementList;
  Public
    Constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Statements : TSQLElementList Read FStatements Write FStatements;
  end;

  { TSQLAssignStatement }

  TSQLAssignStatement = Class(TSQLStatement)
  private
    FExpression: TSQLExpression;
    FVar: TSQLIdentifierName;
  public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Variable : TSQLIdentifierName Read FVar Write FVar;
    Property Expression : TSQLExpression Read FExpression Write FExpression;
  end;

  { TSQLIFStatement }

  TSQLIFStatement = Class(TSQLStatement)
  private
    FCondition: TSQLExpression;
    FFalseBranch: TSQLStatement;
    FTrueBranch: TSQLStatement;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Condition : TSQLExpression Read FCondition Write FCondition;
    Property TrueBranch : TSQLStatement Read FTrueBranch Write FTrueBranch;
    Property FalseBranch : TSQLStatement Read FFalseBranch Write FFalseBranch;
  end;

  { TSQLForStatement }

  TSQLForStatement = Class(TSQLStatement)
  private
    FFieldList: TSQLElementList;
    FSelect: TSQLSelectStatement;
    FStatement: TSQLStatement;
  Public
    Constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Select : TSQLSelectStatement Read FSelect Write FSelect;
    Property FieldList : TSQLElementList Read FFieldList Write FFieldList;
    Property Statement : TSQLStatement Read FStatement Write FStatement;
  end;

  { TSQLWhileStatement }

  TSQLWhileStatement = Class(TSQLStatement)
  private
    FCondition: TSQLExpression;
    FStatement: TSQLStatement;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Condition : TSQLExpression Read FCondition Write FCondition;
    Property Statement : TSQLStatement Read FStatement Write FStatement;
  end;

  { TSQLWhenError }

  TSQLWhenError = Class(TSQLElement);

  { TSQLWhenSQLErrorCode }

  TSQLWhenSQLError = Class(TSQLWhenError)
  private
    FErrorCode: Integer;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ErrorCode : Integer Read FErrorCode Write FErrorCode;
  end;

  { TSQLWhenException }

  TSQLWhenException = Class(TSQLWhenError)
  private
    FEN: TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ExceptionName : TSQLIdentifierName Read FEN Write FEN;
  end;

  { TSQLWhenGDSErrorCode }

  TSQLWhenGDSError = Class(TSQLWhenError)
  private
    FErrorNumber: Integer;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property GDSErrorNumber : Integer Read FErrorNumber Write FErrorNumber;
  end;

  { TSQLWhenStatement }

  TSQLWhenStatement = Class(TSQLStatement)
  private
    FAnyError: Boolean;
    FErrors: TSQLElementList;
    FStatement: TSQLStatement;
  Public
    Constructor Create(APArent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property AnyError : Boolean Read FAnyError Write FAnyError;
    Property Errors : TSQLElementList Read FErrors Write FErrors;
    Property Statement : TSQLStatement Read FStatement Write FStatement;
  end;

  { TSQLExceptionStatement }

  TSQLExceptionStatement = Class(TSQLStatement)
  private
    FEN: TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ExceptionName : TSQLIdentifierName Read FEN Write FEN;
  end;

  { TSQLExitStatement }

  TSQLExitStatement = Class(TSQLStatement)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLSuspendStatement }

  TSQLSuspendStatement = Class(TSQLStatement)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLPostEventStatement }

  TSQLPostEventStatement = Class(TSQLStatement)
  private
    FCN: TSQLIdentifierName;
    FEN: TSQLStringType;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ColName : TSQLIdentifierName Read FCN Write FCN;
    Property EventName : TSQLStringType Read FEN Write FEN;
  end;

  { TSQLCreateOrAlterProcedureTriggerStatement }

  TSQLCreateOrAlterProcedureTriggerStatement = Class(TSQLCreateOrAlterStatement)
  private
    FLocalVariables: TSQLElementList;
    FStatements: TSQLElementList;
  Public
    constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property LocalVariables : TSQLElementList Read FLocalVariables Write FLocalVariables;
    Property Statements : TSQLElementList Read FStatements Write FStatements;
  end;


  { TSQLAlterCreateProcedureStatement }

  TSQLAlterCreateProcedureStatement = Class(TSQLCreateOrAlterProcedureTriggerStatement)
  private
    FInputVariables: TSQLElementList;
    FOutputVariables: TSQLElementList;
  Public
    constructor Create(AParent: TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property InputVariables : TSQLElementList Read FInputVariables Write FInputVariables;
    Property OutputVariables : TSQLElementList Read FOutputVariables Write FOutputVariables;
  end;

  { TSQLCreateProcedureStatement }

  TSQLCreateProcedureStatement = Class(TSQLAlterCreateProcedureStatement);
  TSQLAlterProcedureStatement = Class(TSQLAlterCreateProcedureStatement);

  TTriggerState = (tsNone,tsActive,tsInactive);
  TTriggerMoment = (tmBefore,tmAfter);
  TTriggerOperation = (toDelete,toInsert,toUpdate);
  TTriggerOperations = set of TTriggerOperation;

  { TSQLAlterCreateTriggerStatement }

  TSQLAlterCreateTriggerStatement = Class(TSQLCreateOrAlterProcedureTriggerStatement)
  private
    FMoment: TTriggerMoment;
    FOperations: TTriggerOperations;
    FPosition: Integer;
    FState: TTriggerState;
    FTableName: TSQLIdentifierName;
  Public
    Destructor destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
    Property State : TTriggerState Read FState Write FState;
    Property Moment : TTriggerMoment Read FMoment Write FMoment;
    Property Operations : TTriggerOperations Read FOperations Write FOperations;
    Property Position : Integer Read FPosition Write FPosition;
  end;
  TSQLCreateTriggerStatement = Class(TSQLAlterCreateTriggerStatement);
  TSQLAlterTriggerStatement = Class(TSQLAlterCreateTriggerStatement);


  { TSQLDropStatement }

  TSQLDropStatement = Class(TSQLDDLStatement)
  private
    FIdentifier: TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Function SQLObjectType(Options : TSQLFormatOptions) : String; virtual; abstract;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ObjectName : TSQLIdentifierName Read FIdentifier Write FIdentifier;
  end;

  { TSQLDropTableStatement }

  TSQLDropTableStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropIndexStatement }

  TSQLDropIndexStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropViewStatement }

  TSQLDropViewStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropProcedureStatement }

  TSQLDropProcedureStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions) : String; override;
  end;

  { TSQLDropDomainStatement }

  TSQLDropDomainStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropGeneratorStatement }

  TSQLDropGeneratorStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions) : String; override;
  end;

  { TSQLDropTriggerStatement }

  TSQLDropTriggerStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropExceptionStatement }

  TSQLDropExceptionStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropDatabaseStatement }

  TSQLDropDatabaseStatement  = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropRoleStatement }

  TSQLDropRoleStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropExternalFunctionStatement }

  TSQLDropExternalFunctionStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;

  { TSQLDropShadowStatement }

  TSQLDropShadowStatement = Class(TSQLDropStatement)
    Function SQLObjectType (Options : TSQLFormatOptions): String; override;
  end;
 {
 TSQLDROPFilterStatement = Class(TSQLDropStatement);
}

  { TSQLConnectStatement }

  TSQLConnectStatement = Class(TSQLStatement)
  private
    FCache: Integer;
    FDBN: TSQLStringType;
    FPWD: TSQLStringType;
    FRole: TSQLStringType;
    FUN: TSQLStringType;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property DatabaseName : TSQLStringType Read FDBN  Write FDBN;
    Property UserName : TSQLStringType Read FUN Write FUN;
    Property Password : TSQLStringType Read FPWD Write FPWD;
    Property Role : TSQLStringType Read FRole Write FRole;
    Property Cache : Integer Read FCache Write FCache;
  end;

  TSQLPrivilegeKind = (pkSelect,pkInsert,pkDelete,pkUpdate,pkReference);
  { TSQLPrivilege }

  TSQLPrivilege = Class(TSQLElement);

  { TSQLInsertPrivilege }

  TSQLInsertPrivilege = Class(TSQLPrivilege)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLDeletePrivilege }

  TSQLDeletePrivilege = Class(TSQLPrivilege)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLSelectPrivilege }

  TSQLSelectPrivilege = Class(TSQLPrivilege)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLAllPrivilege }

  TSQLAllPrivilege = Class(TSQLPrivilege)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLColumnPrivilege }

  TSQLColumnPrivilege = Class(TSQLPrivilege)
  private
    FColumns: TSQLElementList;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Columns : TSQLElementList Read FColumns Write FColumns;
  end;

  { TSQLUpdatePrivilege }

  TSQLUpdatePrivilege = Class(TSQLColumnPrivilege)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLReferencePrivilege }

  TSQLReferencePrivilege = Class(TSQLColumnPrivilege)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  TSQLGrantee = Class(TSQLIdentifierName);

  TSQLUserGrantee = Class(TSQLGrantee);

  { TSQLGroupGrantee }

  TSQLGroupGrantee = Class(TSQLGrantee)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLProcedureGrantee }

  TSQLProcedureGrantee = Class(TSQLGrantee)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLViewGrantee }

  TSQLViewGrantee = Class(TSQLGrantee)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLTriggerGrantee }

  TSQLTriggerGrantee = Class(TSQLGrantee)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLPublicGrantee }

  TSQLPublicGrantee = Class(TSQLGrantee)
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLGrantStatement }

  TSQLGrantStatement = Class(TSQLStatement)
  private
    FGrantees: TSQLElementList;
  Public
    Function GranteesAsSQL(Options : TSQLFormatOptions; AIndent : Integer; IsRevoke : Boolean = False) : TSQLStringType;
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Property Grantees : TSQLElementList Read FGrantees;
  end;
  TSQLRevokeStatement = TSQLGrantStatement;
  { TSQLTableGrantStatement }

  TSQLTableGrantStatement = Class(TSQLGrantStatement)
  private
    FGrantOption: Boolean;
    FPrivileges: TSQLElementList;
    FTableName: TSQLIdentifierName;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property TableName : TSQLIdentifierName Read FTableName Write FTableName;
    Property Privileges : TSQLElementList Read FPrivileges;
    Property GrantOption : Boolean Read FGrantOption Write FGrantOption;
  end;

  { TSQLTableRevokeStatement }

  TSQLTableRevokeStatement = Class(TSQLTableGrantStatement)
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;
  { TSQLProcedureGrantStatement }

  TSQLProcedureGrantStatement = Class(TSQLGrantStatement)
  private
    FGrantOption: Boolean;
    FProcedureName: TSQLIdentifierName;
  Public
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property ProcedureName : TSQLIdentifierName Read FProcedureName Write FProcedureName;
    Property GrantOption : Boolean Read FGrantOption Write FGrantOption;
  end;

  { TSQLProcedureRevokeStatement }

  TSQLProcedureRevokeStatement = Class(TSQLProcedureGrantStatement)
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLRoleGrantStatement }

  TSQLRoleGrantStatement = Class(TSQLGrantStatement)
  private
    FAdminOption: Boolean;
    FRoles: TSQLElementList;
  Public
    Constructor Create(AParent : TSQLElement); override;
    Destructor Destroy; override;
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property Roles : TSQLElementList Read FRoles;
    Property AdminOption : Boolean Read FAdminOption Write FAdminOption;
  end;

  { TSQLRoleRevokeStatement }

  TSQLRoleRevokeStatement = Class(TSQLRoleGrantStatement)
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
  end;

  { TSQLSetTermStatement }

  TSQLSetTermStatement = Class(TSQLStatement)
  private
    FNewValue: string;
  Public
    Function GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType; override;
    Property NewValue : string Read FNewValue Write FNewValue;
  end;

Const
  CharTypes = [sdtChar,sdtVarChar,sdtNChar,sdtNVarChar,sdtCString];
  ExtractElementNames : Array[TSQLExtractElement] of String
          = ('YEAR','MONTH','DAY','HOUR','MINUTE','SECOND','WEEKDAY','YEARDAY');

// Format a SQL keyword according to OPTIONS
Function SQLKeyWord(Const AWord : TSQLStringType; Options : TSQLFormatOptions) : TSQLStringType;
Function SQLListSeparator(Options: TSQLFormatOptions) : String;
Procedure GetSepPrefixIndent(DoNewLine,DoIndent : Boolean; Out Sep,Prefix : TSQLStringType; Out AIndent : Integer);
Function SQLFormatString(Const AValue : TSQLStringType; Options : TSQLFormatOptions) : TSQLStringType;

implementation

{ TSQLElementList }

Function SQLFormatString(Const AValue : TSQLStringType; Options : TSQLFormatOptions) : TSQLStringType;

begin
  If sfoDoubleQuotes in Options then
    Result:='"'+StringReplace(AValue,'"','""',[rfreplaceAll])+'"'
  else
    Result:=''''+StringReplace(AValue,'''','''''',[rfreplaceAll])+'''';
end;


Function SQLKeyWord(Const AWord : TSQLStringType; Options : TSQLFormatOptions) : TSQLStringType;


begin
  If (sfoLowercaseKeyword in Options) then
    Result:=LowerCase(AWord)
  else
    Result:=AWord;
end;

Function SQLListSeparator(Options: TSQLFormatOptions) : String;

begin
  Result:=' , ';
  If (SfoListNoSpaceBeforeComma in Options) then
    Delete(Result,1,1);
  If (SfoListNoSpaceAfterComma in Options) then
    Delete(Result,Length(Result),1);
end;

Procedure GetSepPrefixIndent(DoNewLine,DoIndent : Boolean; Out Sep,Prefix : TSQLStringType; Out AIndent : Integer);

begin
  Prefix:='';
  AIndent:=0;
  If DoNewLine then
    begin
    Sep:=','+SlineBreak;
    If DoIndent then
      begin
      Prefix:='  ';
      Aindent:=2;
      end
    end
  else
    Sep:=', ';
end;

{ TSQLSetTermStatement }

function TSQLSetTermStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  Result:='SET TERM '+NewValue;
end;

{ TSQLAlterGeneratorStatement }

function TSQLAlterGeneratorStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
end;

{ TSQLSetISQLStatement }

function TSQLSetISQLStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  // Note: we generate this as a comment as this is ISQL-specific and will generate
  // errors when passed as SQL to servers
  Result:='-- SET '+Arguments;
end;

function TSQLElementList.GetE(AIndex : Integer): TSQLElement;
begin
  Result:=TSQLElement(Items[AIndex]);
end;

procedure TSQLElementList.SetE(AIndex : Integer; const AValue: TSQLElement);
begin
  Items[AIndex]:=AValue;
end;

{ TSQLIntegerLiteral }


function TSQLIntegerLiteral.GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:=IntToStr(FValue);
end;

{ TSQLFloatLiteral }

function TSQLFloatLiteral.GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  // Needs improvement.
  Result:=FloatToStr(FValue);
end;

{ TSQLStringElement }

function TSQLStringLiteral.GetAsSQL(Options : TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;

begin
  Result:=SQLFormatString(Value,Options);
end;

{ TSQLElement }

constructor TSQLElement.Create(AParent: TSQLElement);
begin
  FParent:=AParent;
end;

destructor TSQLElement.destroy;
begin
  inherited destroy;
end;

{ TSQLSelectStatement }

constructor TSQLSelectStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FFields:=TSQLElementList.create(True);
  FTables:=TSQLElementList.Create(True);
  FGroupBy:=TSQLElementList.Create(True);
  FOrderBy:=TSQLElementList.Create(True);
end;

destructor TSQLSelectStatement.Destroy;
begin
  FreeAndNil(FFields);
  FreeAndNil(FTables);
  FreeAndNil(FGroupBy);
  FreeAndNil(FOrderBy);
  FreeAndNil(FWhere);
  FreeAndNil(FHaving);
  FreeAndNil(FStartAt);
  FreeAndNil(FEndAt);
  FreeAndNil(FUnion);
  FreeAndNil(FPlan);
  FreeAndNil(FForUpdate);
  FreeAndNil(FTN);
  FreeAndNil(FInto);
  inherited Destroy;
end;


function TSQLSelectStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;

Var
  NewLinePending : Boolean;

  Procedure AddList(Const AKeyWord : String; List : TSQLElementList; UseNewLine,UseIndent : boolean);

  Var
    S,Pref,Sep : TSQLStringType;
    I,Ind : Integer;

  begin
    S:='';
    if Not Assigned(List) or (List.Count=0) then
      exit;
    If (AkeyWord<>'') then
      If NewlinePending then
        Result:=Result+sLinebreak+SQLKeyWord(AKeyWord,Options)
      else
        Result:=Result+' '+SQLKeyWord(AKeyWord,Options);
    GetSepPrefixIndent(UseNewLine,UseIndent,Sep,Pref,Ind);
    For I:=0 to List.Count-1 do
      begin
      If (S<>'') then
          S:=S+Sep;
      S:=S+Pref+List[I].GetAsSQL(Options,AIndent+Ind);
      end;
    NewLinePending:=UseNewLine;
    If UseNewline then
      Result:=Result+sLinebreak+S
    else
      Result:=Result+' '+S;
  end;

  Procedure AddExpression(Const AKeyWord : TSQLStringType;AExpression  : TSQLElement; UseNewLine,UseIndent : boolean);

  Var
    S,Pref,Sep : TSQLStringType;
    Ind : Integer;

  begin
    If Not Assigned(AExpression) then
      Exit;
    If NewlinePending then
      S:=slineBreak
    else
      S:=' ';
    Result:=Result+S;
    If UseNewline then
      S:=slineBreak
    else
      S:=' ';
    Result:=Result+SQLKeyWord(AKeyword,Options)+S;
    GetSepPrefixIndent(UseNewLine,UseIndent,Sep,Pref,Ind);
    Result:=Result+Pref+AExpression.GetAsSQL(Options,0{AIndent+Ind});
    NewLinePending:=UseNewLine;
  end;

Var
  Ind : Boolean;

begin
  Result:=SQLKeyWord('SELECT',Options);
  If Distinct then
    Result:=Result+' '+SQLKeyword('DISTINCT',Options);
  NewLinePending:=(sfoOneFieldPerLine in Options);
  AddList('',Fields,(sfoOneFieldPerLine in Options),(sfoIndentFields in Options));
  AddList('FROM',Tables,(sfoOneTablePerLine in Options),(sfoIndentTables in Options));
  AddExpression('WHERE',Where,(sfoWhereOnSeparateLine in Options),(sfoIndentWhere in Options));
  AddList('GROUP BY',GroupBy,(sfoOneGroupByFieldPerLine in Options),(sfoIndentGroupByFields in Options));
  AddExpression('HAVING',Having,(sfoHavingOnSeparateLine in Options),(sfoIndentHaving in Options));
  If Assigned(Union) then
    NewLinePending:=NewLinePending or (sfoUnionOnSeparateLine in Options);
  AddExpression('UNION',Union,(sfoUnionOnSeparateLine in Options),False);
  If Assigned(Plan) then
    NewLinePending:=NewLinePending or (sfoPlanOnSeparateLine in Options);
  AddExpression('PLAN',Plan,(sfoPlanOnSeparateLine in Options),(sfoIndentPlan in Options));
  AddList('ORDER BY',OrderBy,(sfoOneOrderByFieldPerLine in Options),(sfoIndentOrderByFields in Options));
end;

{ TSQLInsertStatement }

procedure TSQLInsertStatement.SetSelect(const AValue: TSQLSelectStatement);
begin
  FreeAndNil(FValues);
  FSelect:=AValue;
end;

procedure TSQLInsertStatement.SetValues(const AValue: TSQLElementList);
begin
  FreeAndNil(FSelect);
  FValues:=AValue;
end;


destructor TSQLInsertStatement.Destroy;
begin
  FreeAndNil(FFields);
  FreeAndNil(Fvalues);
  FreeAndNil(FSelect);
  inherited Destroy;
end;

function TSQLInsertStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  S,Pref,Sep : TSQLStringType;
  I,Ind : Integer;
  UseNewLine : Boolean;

begin
  S:='';
  Result:=SQLKeyword('INSERT INTO ',Options);
  If Assigned(FTableName) then
    Result:=Result+TableName.GetAsSQL(Options,AIndent);
  UseNewLine:=sfoOneFieldPerLine in Options;
  If UseNewLine then
     Result:=Result+sLineBreak
  else
     Result:=Result+' ';
  If Assigned(FFields) and (Fields.Count>0) then
    begin
    GetSepPrefixIndent(useNewLine,sfoIndentFields in Options,Sep,Pref,Ind);
    For I:=0 to Fields.Count-1 do
      begin
      If (S<>'') then
          S:=S+Sep;
      If I>0 then
        S:=S+Pref;
      S:=S+Fields[I].GetAsSQL(Options,AIndent+Ind);
      end;
    S:=Pref+'('+S+')';
    If UseNewLine then
       Result:=Result+S+sLineBreak
    else
       Result:=Result+S+' ';
    end;
  If Assigned(FSelect) then
    Result:=Result+Select.GetAsSQL(Options,AIndent)
  else if Assigned(FValues) then
    begin
    Result:=Result+SQLKeyword('VALUES',Options);
    GetSepPrefixIndent(useNewLine,sfoIndentFields in Options,Sep,Pref,Ind);
    S:='';
    For I:=0 to Values.Count-1 do
      begin
      If (S<>'') then
        S:=S+Sep;
      If I>0 then
        S:=S+Pref;
      S:=S+Values[I].GetAsSQL(Options,AIndent+Ind);
      end;
    S:=Pref+'('+S+')';
    If UseNewLine then
       Result:=Result+sLineBreak+S
    else
       Result:=Result+' '+S;
    end;
end;

{ TSQLIdentifierName }

function TSQLIdentifierName.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  // Maybe add quoting options later on ?
  Result:=FName;
  If sfoSingleQuoteIdentifier in Options then
    Result:=''''+Result+''''
  else if sfoDoubleQuoteIdentifier in Options then
    Result:='"'+Result+'"'
  else if sfoBackQuoteIdentifier in Options then
    Result:='`'+Result+'`'
  else
    Result:=UpperCase(Result);
end;

{ TSQLDropStatement }

destructor TSQLDropStatement.Destroy;
begin
  FreeAndNil(FIdentifier);
  inherited Destroy;
end;

function TSQLDropStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('DROP '+SQLObjectType(Options)+' ',Options);
  If Assigned(FIdentifier) then
    Result:=Result+ObjectName.GetAsSQl(Options,AIndent);
end;

{ TSQLCreateOrAlterStatement }

destructor TSQLCreateOrAlterStatement.Destroy;
begin
  FreeAndNil(FDBO);
  inherited Destroy;
end;

function TSQLCreateOrAlterStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Assigned(FDBO) then
    Result:=FDBO.GetAsSQL(Options, AIndent);
end;

{ TSQLCreateDomainStatement }

destructor TSQLCreateDomainStatement.Destroy;
begin
  FreeAndNil(FType);
  inherited Destroy;
end;

function TSQLCreateDomainStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('CREATE DOMAIN ',Options)+inherited GetAsSQL(Options, AIndent);
  If Assigned(FType) then
    Result:=Result+' '+FType.GetAsSQL(Options,AIndent);
end;

{ TSQLTypeDefinition }

destructor TSQLTypeDefinition.Destroy;
begin
  FreeAndNil(FCollation);
  FreeAndNil(FConstraint);
  FreeAndNil(FDefault);
  FreeAndNil(FConstraint);
  inherited Destroy;
end;

function TSQLTypeDefinition.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;

Var
  Opcodes : Array[TSQLDataType] of TSQLStringType
          = ('','SMALLINT','INT','FLOAT','DOUBLE PRECISION',
             'DECIMAL','NUMERIC','DATE','TIMESTAMP','TIME',
             'CHAR','VARCHAR','NATIONAL CHARACTER','NATIONAL CHARACTER VARYING','CSTRING',
             'BLOB');
Var
  D : TArrayDim;
  I : integer;

begin
  If DataType=sdtDomain then
    Result:=SQLKeyWord(UpperCase(TypeName),Options)
  else
    Result:=SQLKeyWord(OpCodes[DataType],Options);
  If (Len>0) then
    If (DataType in CharTypes) then
      Result:=Result+Format('(%d)',[Len])
    else if (DataType in [sdtNumeric,sdtDecimal]) then
      begin
      If (Scale=0) then
        Result:=Result+Format('(%d)',[Len])
      else
        Result:=Result+Format('(%d,%d)',[Len,Scale])
      end;
  If DataType=sdtBlob then
    begin
    Result:=Result+SQLKeyWord(' SUB_TYPE ',Options)+IntToStr(BlobType);
    If Len>0 then
      Result:=Result+SQLKeyWord(' SEGMENT_SIZE ',Options)+IntToStr(Len);
    end;
  If (CharSet<>'') then
    Result:=Result+SQLKeyWord(' CHARACTER SET ',Options)+CharSet;
  If (Length(ArrayDims)>0) then
     begin
     Result:=Result+'[';
     For I:=0 to Length(ArrayDims)-1  do
       begin
       If I>0 then
           Result:=Result+',';
       D:=ArrayDims[I];
       if D[1]<>1 then
         Result:=Result+Format('%d:%d',[D[1],D[2]])
       else
         Result:=Result+Format('%d',[D[1],D[2]]);
       end;
     Result:=Result+']';
     end;
  If Assigned(FDefault) then
    Result:=Result+SQLKeyWord(' DEFAULT ',Options)+DefaultValue.GetAsSQL(Options,AIndent);
  If NotNull then
    Result:=Result+SQLKeyWord(' NOT NULL',Options);
  If Assigned(Constraint) then
    Result:=Result+' '+Constraint.GetAsSQl(Options,AIndent)
  else if Assigned(Check) then
    Result:=Result+SQLKeyWord(' CHECK ',Options)+'('+Check.GetAsSQl(Options,AIndent)+')';
  If Assigned(Collation) then
    Result:=Result+SQLKeyWord(' COLLATION ',Options)+Collation.GetAsSQl(Options,AIndent);
end;


{ TSQLLiteralExpression }

destructor TSQLLiteralExpression.Destroy;
begin
  FreeAndNil(FLiteral);
  inherited Destroy;
end;

function TSQLLiteralExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0
  ): TSQLStringType;
begin
  If Assigned(Literal) then
    Result:=Literal.GetAsSQL(Options)
  else
    Result:='';
end;

{ TSQLUnaryExpression }

destructor TSQLUnaryExpression.Destroy;
begin
  FreeAndNil(Foperand);
  inherited Destroy;
end;

function TSQLUnaryExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Assigned(FOperand) then
    Result:=Operand.GetAsSQL(Options, AIndent);
  Case Operation of
    uoNot   : Result:=SQLKeyWord('NOT',Options)+' ('+Result+')';
    uoMinus : Result:='-'+Result;
  end;
end;

{ TSQLBinaryExpression }

destructor TSQLBinaryExpression.Destroy;
begin
  FreeAndNil(FLeft);
  FreeAndNil(FRight);
  inherited Destroy;
end;

function TSQLBinaryExpression.UseBrackets: Boolean;
begin
  Result:=True;
end;

function TSQLBinaryExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Const
  OpCodes : Array[TSQLBinaryOperation] of string =
          ('AND', 'OR', '=', '<', '>', '<=', '>=', '<>',
           '||','+', '-', '*', '/', 'IN',
           'IS', 'IS NOT', 'LIKE', 'CONTAINING','STARTING WITH');

Var
  L,R,S : TSQLStringType;

begin
  If Assigned(Fleft) then
    begin
    L:=Left.GetAsSQL(Options, AIndent);
    If Left.UseBrackets then
      L:='('+L+')';
    end;
  If Assigned(FRight) then
    begin
    R:=Right.GetAsSQL(Options, AIndent);
    If Right.UseBrackets then
      R:='('+R+')';
    end;
  Result:=L;
  S:=SQLKeyWord(Opcodes[Operation],Options);
  If (Operation in [boOR,boAnd]) and (sfoOneLogicalPerLine in Options) then
    Result:=Result+sLineBreak
  else
    Result:=Result+' ';
  Result:=Result+S+' '+R;
end;

{ TSQLFunctionCallExpression }

destructor TSQLFunctionCallExpression.Destroy;
begin
  FreeAndNil(Farguments);
  inherited Destroy;
end;

function TSQLFunctionCallExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I : Integer;
  Sep : String;

begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  If Assigned(FArguments) and (FArguments.Count>0) then
    For I:=0 to FArguments.Count-1 do
      begin
      If (Result<>'') then
        Result:=Result+Sep;
      Result:=Result+Farguments[i].GetAsSQL(Options,AIndent);
      end;
  Result:=SQLKeyWord(Identifier,Options)+'('+Result+')';
end;

{ TSQLTernaryExpression }

destructor TSQLTernaryExpression.Destroy;
begin
  FreeAndNil(FLeft);
  FreeAndNil(FMiddle);
  FreeAndNil(FRight);
  inherited Destroy;
end;

function TSQLTernaryExpression.UseBrackets: Boolean;
begin
  Result:=True;
end;

function TSQLTernaryExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  M,L,R : TSQLStringType;

begin
  If Assigned(FMiddle) then
    M:=FMiddle.GetAsSQL(Options,AIndent);
  If Assigned(FLeft) then
    L:=FLeft.GetAsSQL(Options,AIndent);
  If Assigned(FRight) then
    R:=FRight.GetAsSQL(Options,AIndent);
  If Operation=toLikeEscape then
    Result:=L+' '+
            SQLKeyWord('LIKE',Options)+' '+
            M+' '+
            SQLKeyWord('ESCAPE',Options)+' '+
            R
  else if Operation=toBetween then
    Result:=L+' '+
            SQLKeyWord('BETWEEN',Options)+' '+
            M+' '+
            SQLKeyWord('AND',Options)+' '+
            R;
end;

{ TSQLAlterDomainSetDefaultStatement }

destructor TSQLAlterDomainSetDefaultStatement.Destroy;
begin
  FreeAndnil(FDefault);
  inherited Destroy;
end;

function TSQLAlterDomainSetDefaultStatement.GetAsSQL(
  Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=Result+SQLKeyWord(' SET DEFAULT ',Options);
  If Assigned(FDefault) then
    Result:=Result+DefaultValue.GetAsSQL(Options,AIndent);
end;

{ TSQLAlterDomainTypeStatement }

destructor TSQLAlterDomainTypeStatement.Destroy;
begin
  FreeAndNil(FNewType);
  inherited Destroy;
end;

function TSQLAlterDomainTypeStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent)+SQLKeyWord(' TYPE ',Options);
  If Assigned(FNewType) then
    Result:=Result+NewType.GetAsSQL(Options,AIndent);
end;

{ TSQLAlterDomainAddCheckStatement }

destructor TSQLAlterDomainAddCheckStatement.Destroy;
begin
  FreeAndNil(FCheck);
  inherited Destroy;
end;

function TSQLAlterDomainAddCheckStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If Assigned(FCheck) then
    Result:=Result+SQLKeyWord(' ADD CHECK ',Options)+Check.GetASSQL(Options,AIndent);
end;


{ TSQLCreateIndexStatement }

constructor TSQLCreateIndexStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FFieldNames:=TSQLElementList.Create(True);
end;

destructor TSQLCreateIndexStatement.Destroy;
begin
  FreeAndNil(FFieldNames);
  FreeAndNil(FTableName);
  inherited Destroy;
end;

function TSQLCreateIndexStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I : Integer;
  S,Sep : TSQLStringType;

begin
  Result:=SQLKeyWord('CREATE ',Options);
  If (ioUnique in Self.Options) then
    Result:=Result+SQLKeyWord('UNIQUE ',Options);
  If ioAscending in Self.Options then
    Result:=Result+SQLKeyWord('ASCENDING ',Options)
  else If ioDescending in Self.Options then
    Result:=Result+SQLKeyWord('DESCENDING ',Options);
  Result:=Result+SQLKeyWord('INDEX ',Options)+inherited GetAsSQL(Options, AIndent);
  Result:=Result+SQLKeyWord(' ON ',Options);
  If Assigned(FTableName) then
    Result:=Result+FTableName.GetAsSQL(Options,AIndent);
  If (FieldNames.Count>0) then
     begin
     Sep:=SQLListSeparator(Options);
     S:='';
     For I:=0 to FieldNames.Count-1 do
       begin
       If (S<>'') then
         S:=S+Sep;
       S:=S+FieldNames[i].GetAsSQL(Options,AIndent);
       end;
     S:='('+S+')';
     end;
  Result:=Result+' '+S;
end;

{ TSQLCreateTableStatement }

constructor TSQLCreateTableStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FFieldDefs:=TSQLElementList.Create(True);
  FConstraints:=TSQLElementList.Create(true);
end;

destructor TSQLCreateTableStatement.Destroy;

Var
  N : String;

begin
  N:=Self.ObjectName.Name;
  Writeln(N);
  FreeAndNil(FexternalFile);
  FreeAndNil(FFieldDefs);
  FreeAndNil(FConstraints);
  inherited Destroy;
end;

function TSQLCreateTableStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep,Pref : TSQLStringType;
  I,Ind : Integer;
  S : String;

begin
  Result:='';
  GetSepPrefixIndent(sfoOneFieldPerLine in Options,sfoIndentfields in Options,Sep,Pref,Ind);
  For I:=0 to FieldDefs.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Pref+FieldDefs[i].GetAsSQL(Options,Ind+AIndent);
    end;
  For I:=0 to Constraints.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Pref+Constraints[i].GetAsSQL(Options,Ind+AIndent);
    end;
  If (sfoOneFieldPerLine in Options) then
    Result:=' ('+sLineBreak+Result+')'
  else
    Result:=' ('+Result+')';
  S:='CREATE';
  if IsRecreate then
    S:='RE'+S;
  S:=SQLKeyWord(S+' TABLE ',Options)+inherited GetAsSQL(Options, AIndent);
  If Assigned(FExternalFile) then
    S:=S+SQLKeyWord(' EXTERNAL FILE ',Options)+ExternalFileName.GetAsSQL(Options,AIndent);
  Result:=S+Result;
end;

{ TSQLTableFieldDef }

destructor TSQLTableFieldDef.Destroy;
begin
  FreeAndNil(FFieldName);
  FreeAndNil(FFieldType);
  FreeAndNil(FComputedBy);
  inherited Destroy;
end;

function TSQLTableFieldDef.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  If Assigned(FFieldName) then
    Result:=FieldName.GetAsSQL(Options, AIndent);
  if Assigned(FComputedBy) then
    Result:=Result+SQLKeyword(' COMPUTED BY ',OPtions)+ComputedBy.GetAsSQL(Options,AIndent)
  else if Assigned(FFieldType) then
    Result:=Result+' '+FieldType.GetAsSQL(Options,AIndent);
end;

{ TSQLConstraintDef }

destructor TSQLConstraintDef.Destroy;
begin
  FreeAndNil(FConstraintName);
  Inherited;
end;

function TSQLConstraintDef.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:='';
  If Assigned(FConstraintName) then
    Result:=SQLKeyWord('CONSTRAINT ',Options)+FConstraintname.GetAsSQl(Options,AIndent);
end;

{ TSQLForeignKeyFieldConstraint }

destructor TSQLForeignKeyFieldConstraint.Destroy;
begin
  FreeAndNil(FDef);
  inherited Destroy;
end;

function TSQLForeignKeyFieldConstraint.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  If Assigned(FDef) then
    Result:=Result+SQLKeyWord('REFERENCES ',Options)+Definition.GetAsSQL(Options,AIndent);
end;

{ TSQLForeignKeyDefinition }

constructor TSQLForeignKeyDefinition.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FFieldList:=TSQLElementList.Create(True);
end;

destructor TSQLForeignKeyDefinition.Destroy;
begin
  FreeAndNil(FTableName);
  FreeAndNil(FFieldList);
  inherited Destroy;
end;

function TSQLForeignKeyDefinition.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Codes : Array[TForeignKeyAction] of string
        = ('','NO ACTION','CASCADE','SET DEFAULT','SET NULL');
Var
  Sep : String;
  I : Integer;

begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to FieldList.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+FieldList[i].GetAsSQl(Options,AIndent);
    end;
  Result:='('+Result+')';
  If Assigned(FTableName) then
    Result:=TableName.GetAsSQl(Options,AIndent)+' '+Result;
  If OnUpdate<>fkaNone then
    Result:=Result+SQLKeyWord(' ON UPDATE '+Codes[OnUpdate],Options);
  If OnDelete<>fkaNone then
    Result:=Result+SQLKeyWord(' ON DELETE '+Codes[OnDelete],Options);
end;

{ TSQLTableFieldsConstraintDef }

constructor TSQLTableFieldsConstraintDef.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FFieldList:=TSQLElementList.Create(True);
end;

destructor TSQLTableFieldsConstraintDef.Destroy;
begin
  FreeAndNil(FFieldList);
  inherited Destroy;
end;

function TSQLTableFieldsConstraintDef.FieldListSQL(Options: TSQLFormatOptions;
  AIndent: Integer = 0): TSQLStringType;

Var
  I : Integer;
  Sep : String;

begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to FieldList.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Fieldlist[I].GetAsSQL(Options,AIndent);
    end;
  Result:='('+Result+')';
end;

{ TSQLTableForeignKeyConstraintDef }

destructor TSQLTableForeignKeyConstraintDef.Destroy;
begin
  FreeAndNil(FDef);
  inherited Destroy;
end;

function TSQLTableForeignKeyConstraintDef.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  Result:=Result+SQLKeyWord('FOREIGN KEY',Options)+' '+FieldListSQl(Options,AIndent);
  If Assigned(FDef) then
    Result:=Result+' '+SQLKeyWord('REFERENCES',Options)+' '+Definition.GetAsSQL(Options, AIndent);
end;

{ TSQLTableCheckConstraintDef }

destructor TSQLTableCheckConstraintDef.Destroy;
begin
  FreeAndNil(FCheck);
  inherited Destroy;
end;

function TSQLTableCheckConstraintDef.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  If Assigned(FCheck) then
    Result:=Result+SQLKeyWord('CHECK',Options)+' ('+FCheck.GetAsSQL(Options,AIndent)+')';
end;

{ TSQLDropTableElementOperation }

destructor TSQLAlterTableOperation.Destroy;
begin
  FreeAndNil(FName);
  inherited Destroy;
end;

function TSQLAlterTableOperation.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Assigned(FName) then
    Result:=FName.GetAsSQL(Options, AIndent);
end;

{ TSQLAlterTableStatement }

constructor TSQLAlterTableStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FOperations:=TSQLElementList.Create(True);
end;

destructor TSQLAlterTableStatement.Destroy;
begin
  FreeAndNil(FOperations);
  inherited Destroy;
end;

function TSQLAlterTableStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Ind,I : Integer;
  Sep,Pref : TSQLStringType;


begin
  Result:='';
  GetSepPrefixIndent(sfoOneFieldPerLine in Options,sfoIndentFields in Options,Sep,Pref,Ind);
  For I:=0 to Operations.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Pref+Operations[i].GetAsSQL(OPtions,Ind+AIndent);
    end;
  If sfoOneFieldPerLine in Options then
    Pref:=slineBreak
  else
    Pref:=' ';
  Result:=SQLKeyWord('ALTER TABLE ',Options)+inherited GetAsSQL(Options, AIndent)+Pref+Result;
end;

{ TSQLAddTableElementOperation }

destructor TSQLAlterTableAddElementOperation.Destroy;
begin
  FreeAndNil(FElement);
  inherited Destroy;
end;

{ TSQLAlterTableFieldNameOperation }

destructor TSQLAlterTableFieldNameOperation.Destroy;
begin
  FreeAndNil(FNewname);
  inherited Destroy;
end;

function TSQLAlterTableFieldNameOperation.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyword('ALTER COLUMN ',Options)+inherited GetAsSQL(Options, AIndent);
  If Assigned(FNewName) then
    Result:=Result+SQLKeyWord(' TO ',Options)+NewName.GetAsSQl(Options,AIndent);
end;

{ TSQLAlterTableFieldTypeOperation }

destructor TSQLAlterTableFieldTypeOperation.Destroy;
begin
  FreeAndNil(FNewType);
  inherited Destroy;
end;

function TSQLAlterTableFieldTypeOperation.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyword('ALTER COLUMN ',Options)+inherited GetAsSQL(Options, AIndent);
  If Assigned(FNewType) then
    Result:=Result+SQLKeyWord(' TYPE ',Options)+NewType.GetAsSQL(Options,AIndent);
end;

{ TSQLAlterTableAddFieldOperation }

function TSQLAlterTableAddFieldOperation.GetF: TSQLTableFieldDef;
begin
  Result:=Element as TSQLTableFieldDef;
end;

function TSQLAlterTableAddFieldOperation.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyword('ADD ',Options);
  If (FieldDef<>Nil) then
    Result:=Result+FieldDef.GetAsSQL(Options, AIndent);
end;

{ TSQLAlterTableAddConstraintOperation }

function TSQLAlterTableAddConstraintOperation.GetC: TSQLTableConstraintDef;
begin
  Result:=Element as TSQLTableConstraintDef;
end;

function TSQLAlterTableAddConstraintOperation.GetAsSQL(
  Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  If(ConstraintDef<>Nil) And (ConstraintDef.ConstraintName<>Nil) then
    Result:=SQLKeyWord('ADD ',Options)
  else
    Result:=SQLKeyWord('ADD CONSTRAINT ',Options);
  If (ConstraintDef<>Nil) then
    Result:=Result+ConstraintDef.GetAsSQL(Options, AIndent);
end;

{ TSQLCheckFieldConstraint }

destructor TSQLCheckFieldConstraint.Destroy;
begin
  FreeAndNil(FExpression);
  inherited Destroy;
end;

function TSQLCheckFieldConstraint.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  If Assigned(FExpression) then
    Result:=Result+SQLKeyWord('CHECK',Options)+' ('+Expression.GetAsSQL(Options,AIndent)+')';
end;

{ TSQLDeleteStatement }

destructor TSQLDeleteStatement.Destroy;
begin
  FreeAndNil(FTableName);
  FreeAndNil(FAliasName);
  FreeAndNil(FWhereClause);
  inherited Destroy;
end;

function TSQLDeleteStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep,Pref : TSQLStringType;

begin
  Result:=SQLKeyWord('DELETE FROM ',Options);
  If Assigned(FTableName) then
   Result:=Result+TableName.GetAsSQL(Options,AIndent);
  If Assigned(FAliasName) then
   Result:=Result+' '+AliasName.GetAsSQL(Options,AIndent);
  If Assigned(FWhereClause) then
    begin
    If (sfoWhereOnSeparateLine in Options)  then
      begin
      Sep:=sLineBreak;
      If (sfoIndentWhere in options) then
        Pref:=sLineBreak+'  '
      else
        Pref:=sLineBreak;
      end
    else
      begin
      Sep:=' ';
      Pref:=' ';
      end;
    Result:=Result+Sep+SQLKeyWord('WHERE',Options);
    Result:=Result+Pref+WhereClause.GetAsSQL(Options,AIndent+Length(Pref));
    end;
end;

{ TSQLTableDMLStatement }

destructor TSQLTableDMLStatement.Destroy;
begin
  FreeAndNil(FTableName);
  inherited Destroy;
end;

{ TSQLUpdatePair }

destructor TSQLUpdatePair.Destroy;
begin
  FreeAndNil(FFieldName);
  FreeAndNil(FValue);
  inherited Destroy;
end;

function TSQLUpdatePair.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;

Var
  F,V : TSQLStringType;

begin
  If Assigned(FFieldName) then
    F:=FFieldName.GetAsSQl(Options,AIndent);
  If Assigned(FValue) then
    V:=FValue.GetAsSQl(Options,AIndent);
  Result:=F+' = '+V;
end;

{ TSQLUpdateStatement }

constructor TSQLUpdateStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FValues:=TSQLElementList.Create(True);
end;

destructor TSQLUpdateStatement.Destroy;
begin
  FreeAndNil(FValues);
  FreeAndNil(FWhereClause);
  inherited Destroy;
end;

function TSQLUpdateStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
Var
   S,Sep,Pref : TSQLStringType;
   I,Ind : Integer;
   useNewLine : Boolean;

begin
  SQLListseparator(Options);
  useNewLine:=sfoOneFieldPerline in options;
  GetSepPrefixIndent(useNewLine,sfoIndentFields in Options,Sep,Pref,Ind);
  S:='';
  For I:=0 to Values.Count-1 do
    begin
    If (S<>'') then
      S:=S+Sep;
    S:=S+Pref+Values[i].GetAsSQL(Options,AIndent+Ind);
    end;
  Delete(Sep,1,1); // remove comma
  Result:=SQLKeyWord('UPDATE ',Options);
  If Assigned(FTableName) then
    Result:=Result+TableName.GetAsSQL(Options,AIndent)+' ';
  Result:=Result+SQLKeyWord('SET',Options)+Sep+S;
  If Assigned(FWhereClause) then
    begin
    If (sfoWhereOnSeparateLine in Options) or useNewLine then
      begin
      Sep:=sLineBreak;
      if not (sfoWhereOnSeparateLine in Options) then
        Pref:=' '
      else
        If (sfoIndentWhere in options) then
          Pref:=sLineBreak+'  '
        else
          Pref:=sLineBreak;
      end
    else
      begin
      Sep:=' ';
      Pref:=' ';
      end;
    Result:=Result+Sep+SQLKeyWord('WHERE',Options);
    Result:=Result+Pref+WhereClause.GetAsSQL(Options,AIndent+Length(Pref));
    end;
end;

{ TSQLSelectField }

destructor TSQLSelectField.Destroy;
begin
  FreeAndNil(FExpression);
  FreeAndNil(FAliasName);
  inherited Destroy;
end;

function TSQLSelectField.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  If Assigned(FExpression) then
    Result:=FExpression.GetAsSQL(Options);
  If Assigned(FAliasName) then
    Result:=Result+' AS '+FAliasName.GetAsSQL(Options);
end;

{ TSQLSimpleTableReference }

destructor TSQLSimpleTableReference.Destroy;
begin
  FreeAndNil(FObjectName);
  FreeAndNil(FParams);
  FreeAndNil(FAliasName);
  inherited Destroy;
end;

function TSQLSimpleTableReference.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;

Var
  I : integer;

begin
  Result:='';
  If Assigned(FParams) and (FParams.Count>0) then
    begin
    For I:=0 to FParams.Count-1 do
      begin
      If (Result<>'') then
        Result:=Result+' , ';
      Result:=Result+FParams[I].GetAsSQL(Options);
      end;
    Result:='('+Result+')';
    end;
  If Assigned(FObjectname) then
    Result:= FObjectName.GetAsSQL(Options)+Result;
  if Assigned(FAliasName) then
    Result:=Result+' '+FAliasName.GetAsSQL(Options);
end;

{ TSQLJoinTableReference }

destructor TSQLJoinTableReference.Destroy;
begin
  FreeAndNil(FLeft);
  FreeAndNil(FRight);
  FreeAndNil(FJoinClause);
  inherited Destroy;
end;

function TSQLJoinTableReference.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Const
  Opcodes : Array[TSQLJoinTYpe] of String
          = ('','INNER ','LEFT ','RIGHT ','FULL OUTER ');

Var
  L,R,O,Sep,prefix : TSQLStringType;
  Ind : Integer;

begin
  GetSepPrefixIndent(sfoOneTablePerLine in Options,sfoIndentTables in Options,Sep,Prefix,Ind);
  Delete(Sep,1,1); // remove comma
  If Assigned(Left) then
    begin
    L:=Left.getAsSQL(Options,AIndent);
    If (sfoBracketLeftJoin in options) and (Left is TSQLJoinTableReference)  then
      L:='('+L+')';
    end;
  If Assigned(Right) then
    begin
    R:=Right.getAsSQL(Options,AIndent+Ind);
    If (Not (sfoNoBracketRightJoin in options)) and (Right is TSQLJoinTableReference)  then
      R:='('+R+')';
    end;
  If Assigned(JoinClause) then
    O:=JoinClause.GetAsSQL(Options,AIndent);
  Result:=L+Sep;
  Result:=Result+Prefix+SQLKEYWORD(Opcodes[JoinType]+'JOIN ',Options)+R;
  If (O<>'') then
    Result:=Result+SQLKeyWord(' ON ',Options)+'('+O+')';
end;

{ TSQLAggregateFunctionExpression }

destructor TSQLAggregateFunctionExpression.Destroy;
begin
  FreeAndNil(FExpression);
  inherited Destroy;
end;

function TSQLAggregateFunctionExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Const
  OpCodes : Array[TSQLAggregateFunction] of string = ('COUNT','SUM','AVG','MAX','MIN');

Var
  E : TSQLStringType;

begin
  Result:=SQLKeyWord(Opcodes[Aggregate],Options);
  Case Option of
    aoAsterisk : E:='*';
    aoAll      : E:=SQLKeyword('ALL',Options);
    aoDistinct : E:=SQLKeyWord('DISTINCT',Options);
  else
    E:='';
  end;
  If Assigned(FExpression) and (Option<>aoAsterisk) then
    begin
    If E<>'' then
      E:=E+' ';
    E:=E+Expression.GetAsSQl(Options,AIndent);
    end;
  Result:=Result+'('+E+')';
end;

{ TSQLGenIDExpression }

destructor TSQLGenIDExpression.Destroy;
begin
  FreeAndNil(FValue);
  FreeAndNil(FIdentifier);
  inherited Destroy;
end;

function TSQLGenIDExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  G,E : TSQLStringType;
begin
  If Assigned(FIdentifier) then
    G:=FIdentifier.GetAsSQl(Options,AIndent);
  If Assigned(FValue) then
    E:=FValue.GetAsSQl(Options,AIndent);

  Result:=SQLKeyword('GEN_ID',options)+'('+G+','+E+')';
end;

{ TSQLCastExpression }

destructor TSQLCastExpression.Destroy;
begin
  FreeAndNil(FValue);
  FreeAndNil(FNewType);
  inherited Destroy;
end;

function TSQLCastExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('CAST(',Options);
  If Assigned(FValue) then
    Result:=Result+FValue.GetAsSQL(Options, AIndent);
  Result:=Result+SQLKeyWord(' AS ',Options);
  If Assigned(FNewType) then
    Result:=Result+FNewType.GetAsSQL(Options, AIndent);
  Result:=Result+')';
end;

{ TSQLOrderByElement }

destructor TSQLOrderByElement.Destroy;
begin
  FreeAndNil(FField);
  FreeAndNil(FCollation);
  inherited Destroy;
end;

function TSQLOrderByElement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Const
  OpCodes : Array[TSQLOrderDirection] of TSQLStringType
          = ('ASC','DESC');

begin
  If Assigned(FField) then
    Result:=FField.GetAsSQL(Options, AIndent);
  If (OrderBy=obDescending) or (sfoForceAscending in Options) then
    Result:=Result+' '+SQLKeyWord(Opcodes[OrderBy],Options);
  If (Collation<>Nil) then
    Result:=Result+' '+Collation.GetAsSQL(Options,AIndent);
end;

{ TSQLSelectIndexedPlan }

constructor TSQLSelectIndexedPlan.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FIndexes:=TSQLElementList.Create(True);
end;

destructor TSQLSelectIndexedPlan.Destroy;
begin
  FreeAndNil(FIndexes);
  inherited Destroy;
end;

function TSQLSelectIndexedPlan.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I : Integer;
  Sep : TSQLStringType;
begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to FIndexes.Count-1 do
    begin
    If Result<>'' then
      Result:=Result+Sep;
    Result:=Result+Findexes[I].GetAsSQL(Options,AIndent);
    end;
  Result:=' ('+Result+')';
  Result:=inherited GetAsSQL(Options, AIndent)+ SQLKeyword(' INDEX',Options)+Result;
end;

{ TSQLSelectPlanItem }

destructor TSQLSelectPlanItem.Destroy;
begin
  FreeandNil(FTableName);
  inherited Destroy;
end;

function TSQLSelectPlanItem.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Assigned(FTableName) then
    Result:=FTableName.GetAsSQL(Options, AIndent);
end;

{ TSQLSelectOrderedPlan }

destructor TSQLSelectOrderedPlan.Destroy;
begin
  FReeAndNil(FIndex);
  inherited Destroy;
end;

function TSQLSelectOrderedPlan.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If Assigned(FIndex) then
    Result:=Result+SQLKeyWord(' ORDER ',Options)+FIndex.GetAsSQL(Options,AIndent);
end;

{ TSQLSelectPlanExpr }

constructor TSQLSelectPlanExpr.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  Fitems:=TSQLElementList.Create(True);
end;

destructor TSQLSelectPlanExpr.Destroy;
begin
  FreeAndNil(FItems);
  inherited Destroy;
end;

function TSQLSelectPlanExpr.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Const
  Opcodes : Array[TPLanJoinType] of TSQLStringType
          = ('JOIN ','SORT ','MERGE ');
Var
  I : Integer;
  Sep : String;
begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to Fitems.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Fitems[I].GetAsSQl(Options,AIndent);
    end;
  Result:='('+Result+')';
  Result:=SQLKeyWord(Opcodes[JoinType],Options)+Result;
end;

{ TSQLSelectionExpression }

destructor TSQLSelectionExpression.Destroy;
begin
  FreeAndNil(FSelect);
  inherited Destroy;
end;

function TSQLSelectionExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  If Assigned(FSelect) then
    Result:=Select.GetAsSQL(Options);
end;

{ TSQLListExpression }

constructor TSQLListExpression.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FList:=TSQLElementList.Create(true);
end;

destructor TSQLListExpression.Destroy;
begin
  FreeAndNil(Flist);
  inherited Destroy;
end;


function TSQLListExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I : integer;
  Sep : String;

begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to List.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+List[i].GetAsSQL(Options, AIndent);
    end;
  Result:='('+Result+')';
end;

{ TSQLTransactionStatement }

destructor TSQLTransactionStatement.Destroy;
begin
  FreeAndNil(FTransactionName);
  inherited Destroy;
end;

function TSQLTransactionStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Assigned(FTransactionName) then
    Result:=SQLKeyWord(' TRANSACTION ',Options)+TransactionName.GetAsSQL(Options, AIndent);
  If Work then
    Result:=Result+SQLKeyWord(' WORK',Options);
  If Release then
    Result:=Result+SQLKeyWord(' RELEASE',Options);
end;

{ TSQLExecuteProcedureStatement }

constructor TSQLExecuteProcedureStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FParams:=TSQLElementList.Create(true);
  FOutParams:=TSQLElementList.Create(true);
end;

destructor TSQLExecuteProcedureStatement.Destroy;
begin
  FreeAndNil(FParams);
  FreeAndNil(FOutParams);
  FreeAndNil(FPN);
  FreeAndNil(FTN);
  inherited Destroy;
end;

function TSQLExecuteProcedureStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  S,Sep : TSQLStringType;
  I: Integer;

begin
  S:='';
  Result:=SQLKeyWord('EXECUTE PROCEDURE',Options);
  If Assigned(FTN) then
    Result:=Result+' '+TransactionName.GetAsSQl(Options,AIndent);
  If Assigned(FPN) then
    Result:=Result+' '+ProcedureName.GetAsSQl(Options,AIndent);
  Sep:=SQLListSeparator(Options);
  If (Params.Count>0) then
    begin
    For I:=0 to Params.Count-1 do
       begin
       If (S<>'') then
         S:=S+Sep;
       S:=S+Params[i].GetAsSQL(Options,AIndent);
       end;
    S:='('+S+')';
    end;
  Result:=Result+S;
  If (Returning.Count>0) then
    begin
    S:='';
    For I:=0 to Returning.Count-1 do
       begin
       If (S<>'') then
         S:=S+Sep;
       S:=S+':'+Returning[i].GetAsSQL(Options,AIndent);
       end;
    S:=SQLKeyWord(' RETURNING_VALUES ',Options)+S;
    Result:=Result+S;
    end;
end;

{ TSQLAlterDatabaseStatement }

constructor TSQLAlterDatabaseStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  Foperations:=TSQLElementList.Create(True);
end;

destructor TSQLAlterDatabaseStatement.Destroy;
begin
  FreeAndNil(FOperations);
  inherited Destroy;
end;

function TSQLAlterDatabaseStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I : Integer;
  Sep : TSQLStringType;
begin
  If sfoMultilineCreateDatabase in Options then
    Sep:=sLineBreak+'  '
  else
    Sep:=' ';
  Result:=SQLKeyWord('ALTER ',Options);
  If UseSchema then
    Result:=Result+SQLKeyWord('SCHEMA',Options)
  else
    Result:=Result+SQLKeyWord('DATABASE',Options);
  Result:=Result+SqlKeyWord(' ADD',Options);
  For I:=0 To Operations.Count-1 do
    Result:=Result+Sep+Operations[i].GetAsSQL(Options,Aindent);
end;

{ TSQLCreateDatabaseStatement }

constructor TSQLCreateDatabaseStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FSecondaryFiles:=TSQLElementList.Create(True);
end;

destructor TSQLCreateDatabaseStatement.Destroy;
begin
  FreeAndNil(FSecondaryFiles);
  FreeAndNil(FCharSet);
  inherited Destroy;
end;

function TSQLCreateDatabaseStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep : String;
  I : Integer;
begin
  If sfoMultilineCreateDatabase in Options then
    Sep:=sLineBreak+'  '
  else
    Sep:=' ';
  Result:=SQLKeyWord('CREATE ',Options);
  If UseSchema then
    Result:=Result+SQLKeyWord('SCHEMA ',Options)
  else
    Result:=Result+SQLKeyWord('DATABASE ',Options);
  Result:=Result+SQLFormatString(FileName,Options);
  If (UserName<>'') then
    begin
    Result:=Result+Sep+SQLKeyWord('USER ',Options)+SQLFormatString(UserName,Options);
    If (Password<>'') then
      Result:=Result+SQLKeyWord(' PASSWORD ',Options)+SQLFormatString(Password,Options);
    end;
  If (PageSize<>0) then
    Result:=Result+Sep+SQLKeyWord('PAGE_SIZE ',Options)+IntToStr(PageSize);
  If (Length<>0) then
    Result:=Result+Sep+SQLKeyWord('LENGTH ',Options)+IntToStr(Length);
  If Assigned(FCharset) then
    Result:=Result+Sep+SQLKeyWord('DEFAULT CHARACTER SET ',Options)+Charset.GetAsSQl(Options,AIndent);
  For I:=0 to SecondaryFiles.Count-1 do
    Result:=Result+Sep+SecondaryFiles[i].GetAsSQL(Options,AIndent);
end;

{ TSQLAlterCreateViewStatement }

constructor TSQLAlterCreateViewStatement.Create(APArent: TSQLElement);

begin
  inherited Create(APArent);
  FFields:=TSQLElementList.Create(True);
end;

destructor TSQLAlterCreateViewStatement.destroy;
begin
  FreeAndNil(FFields);
  FreeAndNil(FSelect);
  inherited destroy;
end;

{ TSQLCreateShadowStatement }

constructor TSQLCreateShadowStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FSecondaryFiles:=TSQLElementList.Create(True);
end;

destructor TSQLCreateShadowStatement.Destroy;
begin
  FreeAndNil(FSecondaryFiles);
  inherited Destroy;
end;

function TSQLCreateShadowStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep: TSQLStringType;
  I : Integer;

begin
  If sfoMultilineCreateShadow in Options then
    Sep:=sLineBreak+'  '
  else
    Sep:=' ';
  Result:=SQLKeyWord('CREATE SHADOW ',Options)+intToStr(Number);
  If Manual then
    Result:=Result+SQLKeyWord(' MANUAL',Options);
  If Conditional then
    Result:=Result+SQLKeyWord(' CONDITIONAL',Options);
  Result:=Result+' '+SQLFormatString(FileName,Options);
  If (Length<>0) then
    Result:=Result+SQLKeyWord(' LENGTH ',Options)+IntToStr(Length)+SQLKeyWord(' PAGES',Options);
  For I:=0 to SecondaryFiles.Count-1 do
    Result:=Result+Sep+SecondaryFiles[i].GetAsSQL(Options,AIndent);
end;

{ TSQLCreateProcedureStatement }

constructor TSQLAlterCreateProcedureStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FInputVariables:=TSQLElementList.Create(True);
  FOutputVariables:=TSQLElementList.Create(True);
end;

destructor TSQLAlterCreateProcedureStatement.Destroy;
begin
  FreeAndNil(FInputVariables);
  FreeAndNil(FOutputVariables);
  inherited Destroy;
end;

function TSQLAlterCreateProcedureStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  S,Sep : TSQLStringType;
  I : Integer;

begin
  S:='';
  Result:='';
  If Self is TSQLAlterProcedureStatement then
    Result:=SQLKeyword('ALTER ',Options)
  else if IsRecreate then
    Result:=SQLKeyword('RECREATE ',Options)
  else
    Result:=SQLKeyword('CREATE ',Options);
  Result:=Result+SQLKeyWord('PROCEDURE ',Options);
  If (ObjectName<>Nil) then
    Result:=Result+ObjectName.GetAsSQL(Options, AIndent);
  Sep:=SQLListSeparator(Options);
  For I:=0 to InputVariables.Count-1 do
    begin
    If (S<>'') then
      S:=S+Sep;
    S:=S+InputVariables[i].GetAsSQL(Options,AIndent);
    end;
  If (S<>'') then
    Result:=Result+' ('+S+')';
  S:='';
  For I:=0 to OutputVariables.Count-1 do
    begin
    If (S<>'') then
      S:=S+Sep;
    S:=S+OutputVariables[i].GetAsSQL(Options,AIndent);
    end;
  If (S<>'') then
    Result:=Result+sLineBreak+'RETURNS ('+S+')';
  Result:=Result+sLineBreak+SQLKeyword('AS',Options)+sLineBreak;
  Result:=Result+Inherited GetAsSQL(Options,AIndent);
end;

{ TSQLProcedureParamDef }

destructor TSQLProcedureParamDef.Destroy;
begin
  FreeAndNil(FParamName);
  FreeAndNil(FParamType);
  inherited Destroy;
end;

function TSQLProcedureParamDef.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Assigned(FParamName) then
    Result:=ParamName.getAsSQL(OPtions,AIndent);
  If Assigned(FParamType) then
    Result:=Result+' '+ParamType.getAsSQL(OPtions,AIndent);
end;

{ TSQLStatementBlock }

constructor TSQLStatementBlock.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FStatements:=TSQLElementList.Create(True);
end;

destructor TSQLStatementBlock.Destroy;
begin
  FreeAndNil(FStatements);
  inherited Destroy;
end;


Function SQLIndentStatement(S : String; Options: TSQLFormatOptions) : String;

Var
  L : TStringList;
  I : Integer;
  Sep : String;

begin
  L:=TStringList.Create;
  Sep:='  ';
  try
    L.Text:=S;
    If (L.Count>0) then
     Result:=Sep+L[0];
    Sep:=sLineBreak+Sep;
    For i:=1 to L.Count-1 do
      Result:=Result+Sep+L[i];
  finally
    L.Free;
  end;
end;

function TSQLStatementBlock.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I: Integer;
  S : String;
begin
  S:='';
  Result:=SQLKeyword('BEGIN',Options)+slineBreak;
  For I:=0 to Statements.Count-1 do
    begin
    If (S<>'') then
      S:=S+sLineBreak;
    S:=S+Statements[i].GetAsSQL(Options,AIndent);
    If Not (Statements[i] is TSQLStatementBlock) then
     S:=S+';';
    end;
  Result:=Result+SQLIndentStatement(S,Options);
  Result:=Result+SlineBreak+SQLKeyWord('END',Options);
end;

{ TSQLIFStatement }

destructor TSQLIFStatement.Destroy;
begin
  FreeAndNil(FCondition);
  FreeAndNil(FFalseBranch);
  FreeAndNil(FTrueBranch);
  inherited Destroy;
end;

function TSQLIFStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('IF (',Options);
  If Assigned(FCondition) then
    Result:=Result+Condition.GetAsSQL(Options,AIndent);
  Result:=Result+SQLKeyWord(') THEN',Options)+sLineBreak;
  If Assigned(FTrueBranch) then
    Result:=Result+SQLindentStatement(TrueBranch.GetAsSQL(Options,AIndent),Options);
  If Assigned(FFalseBranch) then
    begin
    Result:=Result+sLineBreak+SQLKeyWord('ELSE',Options)+sLineBreak;
    Result:=Result+SQLindentStatement(FalseBranch.GetAsSQL(Options,AIndent),Options)
    end;
end;

{ TSQLForStatement }

constructor TSQLForStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FFieldList:=TSQLElementList.Create(True);
end;

destructor TSQLForStatement.Destroy;
begin
  FreeAndNil(FFieldList);
  FreeAndNil(FSelect);
  FreeAndNil(FStatement);
  inherited Destroy;
end;

function TSQLForStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;

Var
  S,Sep,Prefix : TSQLStringType;
  I,Ind : Integer;
  DoNewLine : Boolean;

begin

  S:='';
  Result:=SQLKeyWord('FOR ',Options);
  If Assigned(FSelect) then
    Result:=Result+Select.GetAsSQL(Options,AIndent)+sLineBreak;
  Result:=Result+SQLKeyWord('INTO',Options);
  DoNewLine:=sfoOneFieldPerLine in Options;
  GetSepPrefixIndent(DoNewLine,sfoIndentFields in Options,Sep,Prefix,Ind);
  For I:=0 to FieldList.Count-1 do
    begin
    If (S<>'') then
      S:=S+Sep;
    S:=S+Prefix+':'+FieldList[i].GetAsSQl(Options,AIndent);
    end;
  Result:=Result+sLineBreak+S+sLineBreak+SQLKeyWord('DO',Options)+sLineBreak;
  If Assigned(FStatement) then
    Result:=Result+SQLIndentStatement(Statement.GetAsSQL(Options,AIndent),Options)
end;

{ TSQLExceptionStatement }

destructor TSQLExceptionStatement.Destroy;
begin
  FreeAndNil(FEN);
  inherited Destroy;
end;

function TSQLExceptionStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('EXCEPTION ',Options);
  If Assigned(FEN) then
    Result:=Result+ExceptionName.GetAsSQL(Options, AIndent);
end;

{ TSQLPostEventStatement }

destructor TSQLPostEventStatement.Destroy;
begin
  FreeAndNil(FCN);
  inherited Destroy;
end;

function TSQLPostEventStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('POST_EVENT ',Options);
  If EventName<>'' then
    Result:=Result+SQLFormatString(EventName,Options)
  else
    Result:=Result+ColName.GetAsSQl(Options,AIndent);
end;

{ TSQLAssignStatement }

destructor TSQLAssignStatement.Destroy;
begin
  FreeAndNil(FVar);
  FreeAndNil(FExpression);
  inherited Destroy;
end;

function TSQLAssignStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  if assigned(FVar) then
    Result:=Variable.GetAsSQl(Options,AIndent);
  If Assigned(FExpression) then
    Result:=Result+' = '+Expression.GetAsSQL(Options, AIndent);
end;

{ TSQLWhileStatement }

destructor TSQLWhileStatement.Destroy;
begin
  FreeAndNil(FCondition);
  FreeAndNil(FStatement);
  inherited Destroy;
end;

function TSQLWhileStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('WHILE (',Options);
  If Assigned(FCondition) then
    Result:=Result+Condition.GetAsSQL(Options, AIndent);
  Result:=Result+SQLKeyWord(') DO',Options)+sLineBreak;
  If Assigned(FStatement) then
    Result:=Result+SQLIndentStatement(Statement.GetAsSQL(Options,AIndent),Options);
end;

{ TSQLWhenException }

destructor TSQLWhenException.Destroy;
begin
  FReeAndNil(FEN);
  inherited Destroy;
end;

function TSQLWhenException.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('EXCEPTION ',Options);
  If Assigned(FEN) then
    Result:=Result+ExceptionName.GetAsSQL(Options, AIndent);;
end;

{ TSQLWhenStatement }

constructor TSQLWhenStatement.Create(APArent: TSQLElement);
begin
  inherited Create(APArent);
  FErrors:=TSQLElementList.Create(True);
end;

destructor TSQLWhenStatement.Destroy;
begin
  FreeAndNil(FErrors);
  FreeAndNil(FStatement);
  inherited Destroy;
end;

function TSQLWhenStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;

Var
  I : Integer;
  S,Sep : TSQLStringType;

begin
  Result:=SQLKeyWord('WHEN ',Options);
  If AnyError then
    Result:=Result+SQLKeyWord('ANY',Options)
  else
    begin
    S:='';
    For I:=0 to Errors.Count-1 do
      begin
      Sep:=SQLListSeparator(Options);
      If (S<>'') then
        S:=S+Sep;
      S:=S+Errors[i].GetAsSQL(Options,AIndent);
      end;
    Result:=Result+S;
    end;
  Result:=Result+SQLKeyWord(' DO',Options)+sLineBreak;
  If Assigned(FStatement) then
    Result:=Result+SQLIndentStatement(Statement.GetAsSQL(Options,AIndent),Options);
end;

{ TSQLCreateOrAlterProcedureTriggerStatement }

constructor TSQLCreateOrAlterProcedureTriggerStatement.Create(
  AParent: TSQLElement);
begin
  inherited Create(AParent);
  FLocalVariables:=TSQLElementList.Create(True);
  FStatements:=TSQLElementList.Create(True);
end;

destructor TSQLCreateOrAlterProcedureTriggerStatement.Destroy;
begin
  FreeAndNil(FLocalVariables);
  FreeAndNil(FStatements);
  inherited Destroy;
end;

function TSQLCreateOrAlterProcedureTriggerStatement.GetAsSQL(
  Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;

Var
  I : Integer;
  S : TSQLStringType;

begin
  Result:='';
  For I:=0 to LocalVariables.Count-1 do
    begin
    Result:=Result+SQLKeyWord('DECLARE VARIABLE ',Options);
    Result:=Result+LocalVariables[i].GetAsSQL(Options,AIndent)+';'+sLineBreak;
    end;
  Result:=Result+SQLKeyWord('BEGIN',Options)+sLineBreak;
  For I:=0 to Statements.Count-1 do
    begin
    S:=Statements[i].GetAsSQL(Options,AIndent);
    If sfoIndentProcedureBlock in Options then
      S:=SQLIndentStatement(S,Options);
    Result:=Result+S+';'+sLineBreak;
    end;
  Result:=Result+SQLKeyWord('END',Options);
end;

{ TSQLAlterCreateTriggerStatement }

destructor TSQLAlterCreateTriggerStatement.destroy;
begin
  FreeAndNil(FTableName);
  inherited destroy;
end;

function TSQLAlterCreateTriggerStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Const
  SStates    : Array[TTriggerState]     of String = ('','ACTIVE','INACTIVE');
  SMoment    : Array[TTriggerMoment]    of String = ('BEFORE','AFTER');
  SOperation : Array[TTriggerOperation] of String = ('DELETE','INSERT','UPDATE');

Var
  A : Boolean;
  S: TSQLStringType;
  O : TTriggerOperation;

begin
  A:=Self is TSQLAlterTriggerStatement;
  If A then
    Result:=SQLKeyword('ALTER ',Options)
  else
    Result:=SQLKeyword('CREATE ',Options);
  Result:=Result+SQLKeyWord('TRIGGER ',Options);
  If (ObjectName<>Nil) then
    Result:=Result+ObjectName.GetAsSQL(Options, AIndent);
  If Not A and Assigned(FTableName) then
    Result:=Result+SQLKeyWord(' FOR ',Options)+TableName.GetAsSQL(Options, AIndent);
  Result:=Result+sLineBreak;
  If (State<>tsNone) then
    Result:=Result+SStates[State]+sLineBreak;
  Result:=Result+SQLKeyWord(SMoment[Moment]+' ',Options);
  S:='';
  for O:=Low(TTriggerOperation) to High(TTriggerOperation) do
    if O in Operations then
      begin
      If S<>'' then
        S:=S+SQLKeyWord(' OR ',Options);
      S:=S+SQLKeyWord(SOperation[O],Options);
      end;
  Result:=Result+S+sLineBreak;
  If (Position<>0) then
    Result:=Result+SQLKeyWord('POSITION ',OPtions)+IntToStr(Position)+sLineBreak;
  Result:=Result+SQLKeyword('AS',Options)+sLineBreak;
  Result:=Result+Inherited GetAsSQL(Options,AIndent);
end;

{ TDeclareExternalFunctionStatement }

constructor TSQLDeclareExternalFunctionStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  Farguments:=TSQLElementList.Create(True);
end;

destructor TSQLDeclareExternalFunctionStatement.Destroy;
begin
  FreeAndNil(FArguments);
  FreeAndNil(FReturnType);
  inherited Destroy;
end;

function TSQLDeclareExternalFunctionStatement.GetAsSQL(
  Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;

Var
  Sp,S,Sep : TSQLStringType;
  I : Integer;

begin
  if (sfoMultilineDeclareFunction in Options) then
    Sp:=sLineBreak+'  '
  else
    sp:=' ';
  Result:=SQLKeyWord('DECLARE EXTERNAL FUNCTION ',Options)+inherited GetAsSQL(Options, AIndent);
  If Arguments.Count>0 then
    begin
    Sep:=SQLlistSeparator(Options);
    S:='';
    For I:=0 to Arguments.Count-1 do
      begin
      If (S<>'') then
        S:=S+Sep;
      S:=S+Arguments[i].GetAsSQL(Options,AIndent);
      end;
    Result:=Result+Sp+S;
    end;
  If Assigned(FReturnType) then
    Result:=Result+sp+SQLKeyWord('RETURNS ',Options)+ReturnType.GetAsSQL(Options,AIndent);
  Result:=Result+sp+SQLKeyWord('ENTRY_POINT ',Options)+SQLFormatString(EntryPoint,Options);
  Result:=Result+sp+SQLKeyWord('MODULE_NAME ',Options)+SQLFormatString(ModuleName,Options);
end;

{ TSQLIdentifierExpression }

constructor TSQLIdentifierExpression.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FElementIndex:=-1;
end;

destructor TSQLIdentifierExpression.Destroy;
begin
  FreeAndNil(FIdentifier);
  inherited Destroy;
end;

function TSQLIdentifierExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  If Assigned(FIdentifier) then
    Result:= Identifier.GetAsSQL(Options);
  If (ElementIndex<>-1) then
    Result:=Result+Format('[%d]',[Elementindex]);
end;

{ TSQLSelectExpression }

function TSQLSelectExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:='('+inherited GetAsSQL(Options)+')';
end;

{ TSQLExistsExpression }

function TSQLExistsExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:=SQLKeyword('EXISTS',Options)+' ('+inherited GetAsSQL(Options)+')';
end;

{ TSQLSingularExpression }

function TSQLSingularExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:=SQLKeyWord('SINGULAR',Options)+' ('+inherited GetAsSQL(Options)+')';
end;

{ TSQLAllExpression }

function TSQLAllExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:=SQLKeyWord('ALL',Options)+' ('+inherited GetAsSQL(Options)+')';
end;

{ TSQLSomeExpression }

function TSQLSomeExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:=SQLKeyWord('SOME',Options)+' ('+inherited GetAsSQL(Options)+')';
end;

{ TSQLAnyExpression }

function TSQLAnyExpression.GetAsSQL(Options: TSQLFormatOptions; AIndent : Integer = 0): TSQLStringType;
begin
  Result:=SQLKeyWord('ANY',Options)+' ('+inherited GetAsSQL(Options)+')';
end;

{ TSQLExpression }

function TSQLExpression.UseBrackets: Boolean;
begin
  Result:=False;
end;

{ TSQLNullLiteral }

function TSQLNullLiteral.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('NULL',Options);
end;

{ TSQLUserLiteral }

function TSQLUserLiteral.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('USER',Options);
end;

{ TSQLValueLiteral }

function TSQLValueLiteral.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('VALUE',Options);
end;

{ TSQLUniqueFieldConstraint }

function TSQLUniqueFieldConstraint.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  Result:=Result+SQLKeyWord('UNIQUE',Options);
end;

{ TSQLPrimaryKeyFieldConstraint }

function TSQLPrimaryKeyFieldConstraint.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  Result:=Result+SQLKeyWord('PRIMARY KEY',Options);
end;

{ TSQLTableUniqueConstraintDef }

function TSQLTableUniqueConstraintDef.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  Result:=Result+SQLKeyWord('UNIQUE',Options)+' '+FieldListSQl(Options,AIndent);
end;

{ TSQLTablePrimaryKeyConstraintDef }

function TSQLTablePrimaryKeyConstraintDef.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If (Result<>'') then
    Result:=Result+' ';
  Result:=Result+SQLKeyWord('PRIMARY KEY',Options)+' '+FieldListSQl(Options,AIndent);
end;

{ TSQLSelectNaturalPlan }

function TSQLSelectNaturalPlan.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=Result+SQLKeyWord(' NATURAL',Options);
end;

{ TSQLRollBackStatement }

function TSQLRollBackStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyword('ROLLBACK',Options)+Inherited GetAsSQL(Options,AIndent);
end;

{ TSQLCommitStatement }

function TSQLCommitStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyword('COMMIT',Options)+Inherited GetAsSQL(Options,AIndent);
  If Retain then
    Result:=Result+SQLKeyWord(' RETAIN',Options);
end;

{ TSQLCreateGeneratorStatement }

function TSQLCreateGeneratorStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

begin
  if IsSequence then
    Result:=SQLKeyWord('CREATE SEQUENCE ',Options)
  else
    Result:=SQLKeyWord('CREATE GENERATOR ',Options);
  Result:=Result+Inherited GetAsSQL(Options, AIndent);
end;

{ TSQLCreateRoleStatement }

function TSQLCreateRoleStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('CREATE ROLE ',Options)+inherited GetAsSQL(Options, AIndent);
end;

{ TSQLAlterDomainDropDefaultStatement }

function TSQLAlterDomainDropDefaultStatement.GetAsSQL(
  Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent)+
          SQLKeyword(' DROP DEFAULT',Options);
end;

{ TSQLAlterDomainDropCheckStatement }

function TSQLAlterDomainDropCheckStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent)+
          SQLKeyword(' DROP CHECK',Options);
end;

{ TSQLAlterDomainStatement }

function TSQLAlterDomainStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('ALTER DOMAIN ',Options)+inherited GetAsSQL(Options, AIndent);
end;

{ TSQLAlterDomainRenameStatement }

destructor TSQLAlterDomainRenameStatement.Destroy;
begin
  FreeAndNil(FNewName);
  inherited Destroy;
end;

function TSQLAlterDomainRenameStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  If Assigned(FNewName) then
    Result:=Result+' '+NewName.GetAsSQL(Options,AIndent);
end;

{ TSQLCreateExceptionStatement }

destructor TSQLCreateExceptionStatement.Destroy;
begin
  FreeAndNil(FMessage);
  inherited Destroy;
end;

function TSQLCreateExceptionStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  If Self is TSQLAlterExceptionStatement then
    Result:=SQLKeyWord('ALTER ',Options)
  else
    Result:=SQLKeyWord('CREATE ',Options);
  Result:=Result+SQLKeyWord('EXCEPTION ',Options)+inherited GetAsSQL(Options, AIndent);
  If Assigned(FMessage) then
    Result:=Result+' '+FMessage.GetAsSQL(Options,AIndent);
end;

{ TSQLAlterIndexStatement }

function TSQLAlterIndexStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('ALTER INDEX ',Options)+inherited GetAsSQL(Options, AIndent);
  If Inactive then
    Result:=Result+SQLKeyWord(' INACTIVE',Options)
  else
    Result:=Result+SQLKeyWord(' ACTIVE',Options)
end;

{ TSQLDropTableFieldOperation }

function TSQLDropTableFieldOperation.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('DROP ',OPtions)+inherited GetAsSQL(Options, AIndent);
end;

{ TSQLDropTableConstraintOperation }

function TSQLDropTableConstraintOperation.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('DROP CONSTRAINT ',OPtions)+inherited GetAsSQL(Options, AIndent);
end;

{ TSQLAlterTableFieldPositionOperation }

function TSQLAlterTableFieldPositionOperation.GetAsSQL(
  Options: TSQLFormatOptions; AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyword('ALTER COLUMN ',Options)+inherited GetAsSQL(Options, AIndent);
  Result:=Result+SQLKeyWord(' POSITION ',Options)+IntToStr(NewPosition);
end;

{ TSQLCreateViewStatement }

function TSQLCreateViewStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  S,Sep : TSQLStringType;
  I : Integer;

begin
  if IsRecreate then
    Result:=SQLKeyWord('RECREATE VIEW ',Options)
  else
    Result:=SQLKeyWord('CREATE VIEW ',Options);
  Result:=Result+inherited GetAsSQL(Options, AIndent);
  If (Fields.Count>0) then
    begin
    S:='';
    Sep:=SQLListSeparator(Options);
    For I:=0 to Fields.Count-1 do
      begin
      If (S<>'') then
        S:=S+Sep;
      S:=S+Fields[i].GetAsSQl(Options,AIndent);
      end;
    S:=' ('+S+') AS ';
    end
  else
    S:=' AS ';
  Result:=Result+S;
  If Assigned(FSelect) then
    Result:=Result+Select.GetAsSQL(Options,AIndent)
end;

{ TSQLDatabaseFileInfo }

function TSQLDatabaseFileInfo.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQlKeyWord('FILE ',Options)+SQLFormatString(FileName,Options);
  If Length>0 then
    Result:=Result+SQlKeyWord(' LENGTH ',Options)+IntToStr(Length)+SQlKeyWord(' PAGES',Options)
  else if (StartPage>0) then
    Result:=Result+SQlKeyWord(' STARTING AT ',Options)+IntToStr(StartPage);
end;

{ TSQLExitStatement }

function TSQLExitStatement.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('EXIT',Options);
end;

{ TSQLSuspendStatement }

function TSQLSuspendStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('SUSPEND',Options);
end;

{ TSQLWhenSQLError }

function TSQLWhenSQLError.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('SQLCODE ',Options)+IntTostr(ErrorCode);
end;

{ TSQLWhenGDSError }

function TSQLWhenGDSError.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('GDSCODE ',Options)+IntTostr(GDSErrorNumber);
end;

{ TSQLDropTableStatement }

function TSQLDropTableStatement.SQLObjectType(Options : TSQLFormatOptions): String;
begin
  Result:='TABLE';
end;

{ TSQLDropIndexStatement }

function TSQLDropIndexStatement.SQLObjectType(Options : TSQLFormatOptions): String;
begin
  Result:='INDEX';
end;

{ TSQLDropViewStatement }

function TSQLDropViewStatement.SQLObjectType(Options : TSQLFormatOptions): String;
begin
  Result:='VIEW';
end;

{ TSQLDropProcedureStatement }

function TSQLDropProcedureStatement.SQLObjectType(Options : TSQLFormatOptions): String;
begin
  Result:='PROCEDURE';
end;

{ TSQLDropDomainStatement }

function TSQLDropDomainStatement.SQLObjectType(Options : TSQLFormatOptions): String;
begin
  Result:='DOMAIN';
end;

{ TSQLDropGeneratorStatement }

function TSQLDropGeneratorStatement.SQLObjectType(Options : TSQLFormatOptions): String;
begin
  Result:='GENERATOR';
end;

{ TSQLDropTriggerStatement }

function TSQLDropTriggerStatement.SQLObjectType(Options: TSQLFormatOptions
  ): String;
begin
  Result:='TRIGGER';
end;

{ TSQLDropExceptionStatement }

function TSQLDropExceptionStatement.SQLObjectType(Options: TSQLFormatOptions
  ): String;
begin
  Result:='EXCEPTION';
end;

{ TSQLDropDatabaseStatement }

function TSQLDropDatabaseStatement.SQLObjectType(Options: TSQLFormatOptions
  ): String;
begin
  Result:='DATABASE';
end;

{ TSQLDropRoleStatement }

function TSQLDropRoleStatement.SQLObjectType(Options: TSQLFormatOptions
  ): String;
begin
  Result:='ROLE';
end;

{ TSQLDropExternalFunctionStatement }

function TSQLDropExternalFunctionStatement.SQLObjectType(
  Options: TSQLFormatOptions): String;
begin
  Result:='EXTERNAL FUNCTION';
end;

{ TSQLDropShadowStatement }

function TSQLDropShadowStatement.SQLObjectType(Options: TSQLFormatOptions
  ): String;
begin
  Result:='SHADOW';
end;

{ TSQLConnectStatement }

function TSQLConnectStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('CONNECT ',Options)+SQLFormatString(DatabaseName,Options);
  If (UserName<>'') then
    Result:=Result+SQLKeyWord(' USER ',Options)+SQLFormatString(UserName,Options);
  If (Password<>'') then
    Result:=Result+SQLKeyWord(' PASSWORD ',Options)+SQLFormatString(Password,Options);
  If (Cache<>0) then
    Result:=Result+SQLKeyWord(' CACHE ',Options)+IntToStr(Cache);
  If (Role<>'') then
    Result:=Result+SQLKeyWord(' ROLE ',Options)+SQLFormatString(Role,Options);
end;

{ TSQLExtractExpression }

destructor TSQLExtractExpression.Destroy;
begin
  FreeAndNil(FValue);
  inherited Destroy;
end;

function TSQLExtractExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;


begin
  Result:=SQLKeyWord('EXTRACT('+ExtractElementNames[Element]+' FROM ',Options);

  If Assigned(FValue) then
    Result:=Result+Value.GetAsSQL(Options, AIndent);

  Result:=Result+')';
end;

{ TSQLParameterExpression }

destructor TSQLParameterExpression.Destroy;
begin
  FreeAndNil(FIdentifier);
  inherited Destroy;
end;

function TSQLParameterExpression.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=':';
  If Assigned(FIdentifier) then
   Result:=Result+IDentifier.GetAsSQL(Options, AIndent);
end;

{ TSQLFieldConstraintList }

constructor TSQLFieldConstraintList.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FList:=TSQLElementList.Create;
end;

destructor TSQLFieldConstraintList.Destroy;
begin
  FreeAndNil(FList);
  inherited Destroy;
end;

function TSQLFieldConstraintList.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  I : Integer;

begin
  Result:='';
  For I:=0 to List.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+' ';
    Result:=Result+List[i].GetAsSQL(Options, AIndent);
    end;
end;


{ TSQLColumnPrivilege }

destructor TSQLColumnPrivilege.Destroy;
begin
  FreeAndNil(FColumns);
  inherited Destroy;
end;

function TSQLColumnPrivilege.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep : TSQLStringType;
  I : Integer;
begin
  Result:='';
  If Assigned(FColumns) then
    begin
    Sep:=SQLListSeparator(Options);
    For I:=0 to Columns.Count-1 do
      begin
      If (Result<>'') then
        Result:=Result+Sep;
      Result:=Result+Columns[i].GetAsSql(Options);
      end;
    end;
  If (Result<>'') then
    Result:=' ('+Result+')';
end;

{ TSQLGrantStatement }

function TSQLGrantStatement.GranteesAsSQL(Options: TSQLFormatOptions; AIndent : Integer; IsRevoke : Boolean = False): TSQLStringType;

Var
  Sep : TSQLStringType;
  I : Integer;

begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to Grantees.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Grantees[i].GetAsSQl(Options,AIndent);
    end;
  If IsRevoke then
    Result:=SQLKeyWord(' FROM ',Options)+Result
  else
    Result:=SQLKeyWord(' TO ',Options)+Result;
end;

constructor TSQLGrantStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FGrantees:=TSQLElementList.Create(True);
end;

destructor TSQLGrantStatement.Destroy;
begin
  FreeAndNil(FGrantees);
  inherited Destroy;
end;

{ TSQLTableGrantStatement }

constructor TSQLTableGrantStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FPrivileges:=TSQLElementList.Create(True);
end;

destructor TSQLTableGrantStatement.Destroy;
begin
  FreeAndNil(FPrivileges);
  FreeAndNil(FTableName);
  inherited Destroy;
end;

function TSQLTableGrantStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep : TSQLStringType;
  I : Integer;

begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to Privileges.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Privileges[i].GetAsSQL(Options,AIndent);
    end;
  Result:=SQLKeyWord('GRANT ',Options)+Result+SQLKeyWord(' ON ',Options);
  If Assigned(FTableName) then
    Result:=Result+FTableName.GetAsSQl(Options,AIndent);
  Result:=Result+Self.GranteesAsSQL(Options,AIndent);
  If GrantOption then
    Result:=Result+SQLKeyWord(' WITH GRANT OPTION',Options);
end;

{ TSQLProcedureGrantStatement }

destructor TSQLProcedureGrantStatement.Destroy;
begin
  FreeAndNil(FProcedureName);
  inherited Destroy;
end;

function TSQLProcedureGrantStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('GRANT EXECUTE ON PROCEDURE ',OPtions);
  If Assigned(FProcedureName) then
    Result:=Result+FProcedureName.GetAsSQl(Options,AIndent);
  Result:=Result+GranteesAsSQL(Options,AIndent);
end;

{ TSQLRoleGrantStatement }

constructor TSQLRoleGrantStatement.Create(AParent: TSQLElement);
begin
  inherited Create(AParent);
  FRoles:=TSQLElementList.Create(True);
end;

destructor TSQLRoleGrantStatement.Destroy;
begin
  FreeAndNil(FRoles);
  inherited Destroy;
end;

function TSQLRoleGrantStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep : TSQLStringType;
  I : Integer;
begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to Roles.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
     Result:=Result+Roles[i].GetAsSQl(Options,AIndent);
    end;
  Result:=SQLKeyWord('GRANT ',Options)+Result;
  Result:=Result+GranteesAsSQL(Options,AIndent);
  If AdminOption then
    Result:=Result+SQLKeyWord(' WITH ADMIN OPTION',OPtions);
end;

{ TSQLInsertPrivilege }

function TSQLInsertPrivilege.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('INSERT',Options);
end;

{ TSQLDeletePrivilege }

function TSQLDeletePrivilege.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('DELETE',Options);
end;

{ TSQLSelectPrivilege }

function TSQLSelectPrivilege.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('SELECT',Options);
end;

{ TSQLAllPrivilege }

function TSQLAllPrivilege.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('ALL PRIVILEGES',Options);
end;

{ TSQLUpdatePrivilege }

function TSQLUpdatePrivilege.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=SQLKeyWord('UPDATE',Options)+Result;
end;

{ TSQLReferencePrivilege }

function TSQLReferencePrivilege.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=SQLKeyWord('REFERENCES',Options)+Result;
end;

{ TSQLGroupGrantee }

function TSQLGroupGrantee.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=SQLkeyWord('GROUP ',Options)+Result;
end;

{ TSQLProcedureGrantee }

function TSQLProcedureGrantee.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=SQLkeyWord('PROCEDURE ',Options)+Result;
end;

{ TSQLViewGrantee }

function TSQLViewGrantee.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=SQLkeyWord('VIEW ',Options)+Result;
end;

{ TSQLTriggerGrantee }

function TSQLTriggerGrantee.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=inherited GetAsSQL(Options, AIndent);
  Result:=SQLkeyWord('TRIGGER ',Options)+Result;
end;

{ TSQLPublicGrantee }

function TSQLPublicGrantee.GetAsSQL(Options: TSQLFormatOptions; AIndent: Integer
  ): TSQLStringType;
begin
  Result:=SQLKeyWord('PUBLIC',Options);
end;

{ TSQLTableRevokeStatement }

function TSQLTableRevokeStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  S,Sep : TSQLStringType;
  I : Integer;

begin
  Sep:=SQLListSeparator(Options);
  S:='';
  For I:=0 to Privileges.Count-1 do
    begin
    If (S<>'') then
      S:=S+Sep;
    S:=S+Privileges[i].GetAsSQL(Options,AIndent);
    end;
  Result:=SQLKeyWord('REVOKE ',Options);
  If GrantOption then
    Result:=Result+SQLKeyWord('GRANT OPTION FOR ',Options);
  Result:=Result+S+SQLKeyWord(' ON ',Options);
  If Assigned(FTableName) then
    Result:=Result+FTableName.GetAsSQl(Options,AIndent);
  Result:=Result+Self.GranteesAsSQL(Options,AIndent,True);
end;

{ TSQLProcedureRevokeStatement }

function TSQLProcedureRevokeStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;
begin
  Result:=SQLKeyWord('REVOKE EXECUTE ON PROCEDURE ',OPtions);
  If Assigned(FProcedureName) then
    Result:=Result+FProcedureName.GetAsSQl(Options,AIndent);
  Result:=Result+GranteesAsSQL(Options,AIndent,True);
end;

{ TSQLRoleRevokeStatement }

function TSQLRoleRevokeStatement.GetAsSQL(Options: TSQLFormatOptions;
  AIndent: Integer): TSQLStringType;

Var
  Sep : TSQLStringType;
  I : Integer;
begin
  Result:='';
  Sep:=SQLListSeparator(Options);
  For I:=0 to Roles.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+Sep;
    Result:=Result+Roles[i].GetAsSQl(Options,AIndent);
    end;
  Result:=SQLKeyWord('REVOKE ',Options)+Result;
  Result:=Result+GranteesAsSQL(Options,AIndent,True);
end;

end.

