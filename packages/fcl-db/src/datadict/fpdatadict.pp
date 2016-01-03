{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Michael Van Canneyt, member of the
    Free Pascal development team

    Data Dictionary Implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpdatadict;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,inicol, inifiles, contnrs, db;

Type
  // Supported objects in this data dictionary
<<<<<<< HEAD
<<<<<<< HEAD
  TObjectType = (otUnknown,otDictionary,
                 otTables,otTable,
                 otFields,otField,
                 otConnection,otTableData,
                 otIndexDefs,otIndexDef,
                 otSequenceDefs,otSequenceDef,
                 otForeignKeyDefs,otForeignKeyDef,
                 otDomainDefs,otDomainDef);

=======
  TObjectType = (otUnknown,otDictionary,otTables,otTable,otFields,otField,
                 otConnection,otTableData,otIndexDefs,otIndexDef);
>>>>>>> graemeg/fixes_2_2
=======
  TObjectType = (otUnknown,otDictionary,otTables,otTable,otFields,otField,
                 otConnection,otTableData,otIndexDefs,otIndexDef);
>>>>>>> origin/fixes_2_2
  TDDProgressEvent = Procedure(Sender : TObject; Const Msg : String) of Object;

  TFPDDFieldList = Class;
  TFPDDIndexList = Class;
  TDDTableDef = Class;
  TDDTableDefs = Class;
  TDDFieldDefs = Class;
  TDDDomainDef = Class;
  TFPDataDictionary = Class;

  { TDDFieldDef }

  TDDFieldDef = Class(TIniCollectionItem)
  private
    FAlignMent: TAlignMent;
    FConstraint: string;
    FConstraintErrorMessage: string;
    FCustomConstraint: string;
    FDefault: String;
    FDefaultExpression: string;
    FDisplayLabel: string;
    FDisplayWidth: Longint;
    FDomain: TDDDomainDef;
    FDomainName: string;
    FFieldName: string;
    FFieldType: TFieldType;
    FHint: String;
    FPrecision: Integer;
    FProviderFlags: TProviderFlags;
    FReadOnly: Boolean;
    FRequired: Boolean;
    FSize: Integer;
    FVisible: Boolean;
    function GetDomainName: string;
    Function IsSizeStored : Boolean;
    Function IsPrecisionStored : Boolean;
    procedure SetDomain(const AValue: TDDDomainDef);
    procedure SetDomainName(const AValue: string);
  protected
    function GetSectionName: String; override;
    procedure SetSectionName(const Value: String); override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetDisplayName: string; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    Constructor Create(ACollection : TCollection); override;
    Function FieldDefs : TDDFieldDefs;
    Function DataDictionary : TFPDataDictionary;
    // Will return True if the field or the domain it is based on is required
    Function FieldIsRequired : Boolean;
    Procedure ResolveDomain(ErrorOnFail : Boolean);
    Procedure ImportFromField(F: TField; Existing : Boolean = True);
    Procedure ApplyToField(F : TField);
    Procedure Assign(Source : TPersistent);  override;
    Procedure SaveToIni(Ini: TCustomInifile; ASection : String); override;
    Procedure LoadFromIni(Ini: TCustomInifile; ASection : String); override;
    Property Domain : TDDDomainDef Read FDomain Write SetDomain;
  Published
    property FieldType : TFieldType Read FFieldType Write FFieldType;
    property AlignMent : TAlignMent Read FAlignMent write FAlignment default taLeftJustify;
    property CustomConstraint: string read FCustomConstraint write FCustomConstraint;
    property ConstraintErrorMessage: string read FConstraintErrorMessage write FConstraintErrorMessage;
    Property DBDefault : String Read FDefault Write FDEfault;
    property DefaultExpression: string read FDefaultExpression write FDefaultExpression;
    property DisplayLabel : string read FDisplayLabel write FDisplayLabel;
    property DisplayWidth: Longint read FDisplayWidth write FDisplayWidth;
    property FieldName: string read FFieldName write FFieldName;
    property DomainName: string read GetDomainName write SetDomainName;
    property Constraint: string read FConstraint write FConstraint;
    property ReadOnly: Boolean read FReadOnly write FReadOnly;
    property Required: Boolean read FRequired write FRequired;
    property Visible: Boolean read FVisible write FVisible default True;
    Property Size : Integer Read FSize Write FSize Stored IsSizeStored;
    Property Precision : Integer Read FPrecision Write FPrecision Stored IsPrecisionStored;
    Property Hint : String Read FHint Write FHint;
    Property ProviderFlags : TProviderFlags Read FProviderFlags Write FProviderFlags;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  TDDFieldDefClass = Class of TDDFieldDef;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { TDDTableCollection }
  TDDTableCollection = Class(TIniCollection)
  private
    FTableDef : TDDTableDef;
    FTableName: String;
    function GetTableName: String;
  Protected
    Procedure SetTableDef(ATableDef : TDDTableDef);
    procedure SetTableName(const AValue: String); virtual;
  Public
    Function DataDictionary : TFPDataDictionary;
    Property TableDef : TDDTableDef Read FTableDef;
    Property TableName : String Read GetTableName Write SetTableName;
  end;

  { TDDFieldDefs }

  TDDFieldDefs = Class(TDDTableCollection)
  private
    function GetField(Index : Integer): TDDFieldDef;
    procedure SetField(Index : Integer; const AValue: TDDFieldDef);
  Protected
    procedure SetTableName(const AValue: String); override;
  Public
    Constructor Create(ATableDef : TDDTableDef);
    Constructor Create(ATableName : string);
<<<<<<< HEAD
<<<<<<< HEAD
    Class Function FieldDefClass : TDDFieldDefClass; virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property TableDef : TDDTableDef Read FTableDef;
    Property TableName : String Read GetTableName Write SetTableName;
    Function AddField(AFieldName: String = '') : TDDFieldDef;
    Function IndexOfField(AFieldName : String) : Integer;
    Function FindField(AFieldName : String) : TDDFieldDef;
    Function FieldByName(AFieldName : String) : TDDFieldDef;
    Procedure FillFieldList(Const AFieldNames: String; List : TFPDDFieldList);
    Property Fields[Index : Integer] : TDDFieldDef Read GetField Write SetField; default;
  end;
  
  { TDDIndexDef }
  TDDIndexDef = Class(TIniCollectionItem)
  private
    FCaseinsFields: string;
    FDescFields: string;
    FExpression: string;
    FFields: string;
    FIndexName: String;
    FOptions: TIndexOptions;
    FSource: string;
  protected
    function GetSectionName: String; override;
    procedure SetSectionName(const Value: String); override;
    procedure Assign(ASource : TPersistent); override;
  Public
    Procedure SaveToIni(Ini: TCustomInifile; ASection : String); override;
    Procedure LoadFromIni(Ini: TCustomInifile; ASection : String); override;
  Published
    Property IndexName : String Read FIndexName Write FIndexName;
    property Expression: string read FExpression write FExpression;
    property Fields: string read FFields write FFields;
    property CaseInsFields: string read FCaseinsFields write FCaseInsFields;
    property DescFields: string read FDescFields write FDescFields;
    property Options: TIndexOptions read FOptions write FOptions;
    property Source: string read FSource write FSource;
  end;
  
  { TDDIndexDefs }
  TDDIndexDefs = Class(TDDTableCollection)
  private
    function GetIndex(Index : Integer): TDDIndexDef;
    procedure SetIndex(Index : Integer; const AValue: TDDIndexDef);
  Protected
    procedure SetTableName(const AValue: String); override;
  Public
    Constructor Create(ATableDef : TDDTableDef);
    Constructor Create(ATableName : String);
    Function AddDDIndexDef(AName : String) : TDDIndexDef;
    function AddIndex (AName: String) : TDDIndexDef;
    function IndexByName(AIndexName: String): TDDIndexDef;
    function FindIndex(AIndexName: String): TDDIndexDef;
    function IndexOfIndex(AIndexName: String): Integer;
    Property Indexes[Index : Integer] : TDDIndexDef Read GetIndex Write SetIndex; default;
  end;
  
  { TDDForeignKeyDef }
  
  TDDForeignKeyDef = Class(TIniCollectionItem)
  private
    FKeyFields: String;
    FKeyName: String;
    FReferencedFields: String;
    FTableName: String;
    procedure SetKeyName(const AValue: String);
  protected
    function GetSectionName: String; override;
    procedure SetSectionName(const Value: String); override;
    procedure Assign(ASource : TPersistent); override;
  Public
    Procedure SaveToIni(Ini: TCustomInifile; ASection : String); override;
    Procedure LoadFromIni(Ini: TCustomInifile; ASection : String); override;
  Published
    Property KeyName : String Read FKeyName Write SetKeyName;
    Property ReferencesTable : String Read FTableName Write FTableName;
    Property KeyFields : String Read FKeyFields Write FKeyFields;
    Property ReferencedFields : String Read FReferencedFields Write FReferencedFields;
  end;
  
  { TDDForeignKeyDefs }

  TDDForeignKeyDefs = Class(TIniCollection)
  private
    FTableName: String;
    function GetKey(AIndex : Integer): TDDForeignKeyDef;
    procedure SetKey(AIndex : Integer; const AValue: TDDForeignKeyDef);
    procedure SetTableName(const AValue: String);
  Public
    Constructor Create(ATableName : String);
    Function AddForeignKeyDef(AName : String) : TDDForeignKeyDef;
    Property TableName : String Read FTableName Write SetTableName;
    Property Indexes[AIndex : Integer] : TDDForeignKeyDef Read GetKey Write SetKey; default;
  end;

  { TDDTableDef }

  TDDTableDef = Class(TIniCollectionItem)
  private
    FFieldDefs: TDDFieldDefs;
    FIndexDefs: TDDIndexDefs;
    FKeyDefs: TDDForeignKeyDefs;
    FPrimaryKeyName: String;
    FTableName: String;
    function GetOnProgress: TDDProgressEvent;
    function GetPrimaryKeyName: String;
    function GetPrimaryIndexDef : TDDIndexDef;
    procedure SetTableName(const AValue: String);
  protected
    function GetSectionName: String; override;
    procedure SetSectionName(const Value: String); override;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    Function DataDictionary : TFPDataDictionary;
    Function TableDefs : TDDTableDefs;
    Function ImportFromDataset(Dataset : TDataSet; DoClear : Boolean = False; UpdateExisting : Boolean = True) : Integer;
    Procedure ApplyToDataset(Dataset : TDataset);
    Function AddField(AFieldName : String = '') : TDDFieldDef;
    Procedure SaveToIni(Ini: TCustomInifile; ASection : String); override;
    Procedure LoadFromIni(Ini: TCustomInifile; ASection : String); override;
    procedure PrimaryIndexToFields;
    procedure FieldsToPrimaryIndex;
    Property Fields : TDDFieldDefs Read FFieldDefs;
    Property Indexes : TDDIndexDefs Read FIndexDefs;
    Property ForeignKeys : TDDForeignKeyDefs Read FKeyDefs;
    Property OnProgress : TDDProgressEvent Read GetOnProgress;
    Property PrimaryIndexDef : TDDIndexDef read GetPrimaryIndexDef;
  Published
    Property TableName : String Read FTableName Write SetTableName;
    Property PrimaryKeyConstraintName : String Read GetPrimaryKeyName Write FPrimaryKeyName;
  end;
  
  { TDDTableDefs }

  TDDTableDefs = Class(TIniCollection)
  private
    FDataDictionary: TFPDataDictionary;
    FOnProgress: TDDProgressEvent;
    function GetTable(Index : Integer): TDDTableDef;
    procedure SetTable(Index : Integer; const AValue: TDDTableDef);
  Public
    Property DataDictionary: TFPDataDictionary Read FDataDictionary;
    Function AddTable(ATableName : String = '') : TDDTableDef;
    Function IndexOfTable(ATableName : String) : Integer;
    Function FindTable(ATableName : String) : TDDTableDef;
    Function TableByName(ATableName : String) : TDDTableDef;
    Property Tables[Index : Integer] : TDDTableDef Read GetTable Write SetTable; default;
    Property OnProgress : TDDProgressEvent Read FOnProgress Write FOnProgress;
  end;

  { TDDSequenceDef }

  TDDSequenceDef = Class(TIniCollectionItem)
  private
    FIncrement: Integer;
    FSequenceName: String;
    FStartValue: Integer;
    procedure SetSequenceName(const AValue: String);
  protected
    function GetSectionName: String; override;
    procedure SetSectionName(const Value: String); override;
    procedure Assign(ASource : TPersistent); override;
  Public
    Procedure SaveToIni(Ini: TCustomInifile; ASection : String); override;
    Procedure LoadFromIni(Ini: TCustomInifile; ASection : String); override;
  Published
    Property SequenceName : String Read FSequenceName Write SetSequenceName;
    Property StartValue : Integer Read FStartValue Write FStartValue;
    Property Increment : Integer Read FIncrement Write FIncrement;
  end;

  { TDDSequenceDefs }

  TDDSequenceDefs = Class(TIniCollection)
  private
    FDataDictionary: TFPDataDictionary;
    FOnProgress: TDDProgressEvent;
    function GetSequence(Index : Integer): TDDSequenceDef;
    procedure SetSequence(Index : Integer; const AValue: TDDSequenceDef);
  Public
    Constructor Create;
    Function AddSequence(ASequenceName : String = '') : TDDSequenceDef;
    Function IndexOfSequence(ASequenceName : String) : Integer;
    Function FindSequence(ASequenceName : String) : TDDSequenceDef;
    Function SequenceByName(ASequenceName : String) : TDDSequenceDef;
    Property DataDictionary : TFPDataDictionary Read FDataDictionary;
    Property Sequences[Index : Integer] : TDDSequenceDef Read GetSequence Write SetSequence; default;
    Property OnProgress : TDDProgressEvent Read FOnProgress Write FOnProgress;
  end;

  { TDDDomainDef }

  TDDDomainDef = Class(TIniCollectionItem)
    procedure SetDomainName(const AValue: String);
  private
    FCheckConstraint: String;
    FDomainName: String;
    FFieldType: TFieldType;
    FPrecision: Integer;
    FRequired: Boolean;
    FSize: Integer;
  protected
    function GetSectionName: String; override;
    procedure SetSectionName(const Value: String); override;
    procedure Assign(ASource : TPersistent); override;
  Public
    Procedure SaveToIni(Ini: TCustomInifile; ASection : String); override;
    Procedure LoadFromIni(Ini: TCustomInifile; ASection : String); override;
  Published
    Property DomainName : String Read FDomainName Write SetDomainName;
    Property FieldType : TFieldType Read FFieldType Write FFieldType;
    property Size : Integer Read FSize Write FSize;
    property Precision : Integer Read FPrecision Write FPrecision;
    Property Required : Boolean Read FRequired Write FRequired;
    Property CheckConstraint : String Read FCheckConstraint Write FCheckConstraint;
  end;

  { TDDDomainDefs }

  TDDDomainDefs = Class(TIniCollection)
  private
    FDataDictionary: TFPDataDictionary;
    FOnProgress: TDDProgressEvent;
    function GetDomain(Index : Integer): TDDDomainDef;
    procedure SetDomain(Index : Integer; const AValue: TDDDomainDef);
  Public
    Constructor Create;
    Property DataDictionary : TFPDataDictionary Read FDataDictionary;
    Function AddDomain(ADomainName : String = '') : TDDDomainDef;
    Function IndexOfDomain(ADomainName : String) : Integer;
    Function FindDomain(ADomainName : String) : TDDDomainDef;
    Function DomainByName(ADomainName : String) : TDDDomainDef;
    Property Domains[Index : Integer] : TDDDomainDef Read GetDomain Write SetDomain; default;
    Property OnProgress : TDDProgressEvent Read FOnProgress Write FOnProgress;
  end;

  { TFPDataDictionary }
  TOnApplyDataDictEvent = Procedure (Sender : TObject; Source : TDDFieldDef; Dest : TField; Var Allow : Boolean) of object;

  TFPDataDictionary = Class(TPersistent)
  private
    FDDName: String;
    FDomains: TDDDomainDefs;
    FFileName: String;
    FOnApplyDataDictEvent: TOnApplyDataDictEvent;
    FOnProgress: TDDProgressEvent;
    FSequences: TDDSequenceDefs;
    FTables: TDDTableDefs;
    // Last table that returned a match for findfieldDef
    FLastMatchTableDef : TDDTableDef;
    procedure SetOnProgress(const AValue: TDDProgressEvent);
  Public
    Constructor Create;
    Destructor Destroy; override;
    Procedure SaveToFile(AFileName : String; KeepBackup: Boolean = True);
    Procedure SaveToIni(Ini : TCustomIniFile; ASection : String); virtual;
    Procedure LoadFromFile(AFileName : String);
    Procedure LoadFromIni(Ini : TCustomIniFile; ASection : String); virtual;
    Procedure ApplyToDataset(ADataset : TDataset);
    Procedure ApplyToDataset(ADataset : TDataset; OnApply : TOnApplyDataDictEvent);
    Function FindFieldDef(FieldName : String; out TableName : String) : TDDFieldDef;
    Function FindFieldDef(FieldName : String) : TDDFieldDef;
    function CanonicalizeFieldName(const InFN: String; Out TN, FN: String): Boolean;
    function CanonicalizeFieldName(const InFN: String; Out TableDef : TDDTableDef; Out FN: String): Boolean;
    Property Tables : TDDTableDefs Read FTables;
    Property Sequences : TDDSequenceDefs Read FSequences;
    Property Domains : TDDDomainDefs Read FDomains;
    Property FileName : String Read FFileName;
    Property Name : String Read FDDName Write FDDName;
    Property OnProgress : TDDProgressEvent Read FOnProgress Write SetOnProgress;
  Published
    // Using name confuses the object inspector grid.
    Property DataDictionaryName : String Read FDDName Write FDDName;
    Property OnApplyDataDictEvent : TOnApplyDataDictEvent Read FOnApplyDataDictEvent Write FOnApplyDataDictEvent;
  end;


  { TFPDDFieldList }

  TFPDDFieldList = Class(TObjectList)
  private
    function GetFieldDef(Index : Integer): TDDFieldDef;
    procedure SetFieldDef(Index : Integer; const AValue: TDDFieldDef);
  Public
    Constructor CreateFromTableDef(TD : TDDTableDef);
    Constructor CreateFromFieldDefs(FD : TDDFieldDefs);
    Property FieldDefs[Index : Integer] : TDDFieldDef Read GetFieldDef Write SetFieldDef; default;
  end;
  
  { TFPDDIndexList }

  TFPDDIndexList = Class(TObjectList)
  private
    function GetIndexDef(AIndex : Integer): TDDIndexDef;
    procedure SetIndexDef(AIndex : Integer; const AValue: TDDIndexDef);
  Public
    Constructor CreateFromIndexDefs(FD : TDDIndexDefs);
    Property IndexDefs[AIndex : Integer] : TDDIndexDef Read GetIndexDef Write SetIndexDef; default;
  end;

  
  { TFPDDSequenceList }

  TFPDDSequenceList = Class(TObjectList)
  private
    function GetSequenceDef(AIndex : Integer): TDDSequenceDef;
    procedure SetSequenceDef(AIndex : Integer; const AValue: TDDSequenceDef);
  Public
    Constructor CreateFromSequenceDefs(SD : TDDSequenceDefs);
    Property SequenceDefs[AIndex : Integer] : TDDSequenceDef Read GetSequenceDef Write SetSequenceDef; default;
  end;

  { TFPDDDomainList }

  TFPDDDomainList = Class(TObjectList)
  private
    function GetDomainDef(AIndex : Integer): TDDDomainDef;
    procedure SetDomainDef(AIndex : Integer; const AValue: TDDDomainDef);
  Public
    Constructor CreateFromDomainDefs(DD : TDDDomainDefs);
    Property DomainDefs[AIndex : Integer] : TDDDomainDef Read GetDomainDef Write SetDomainDef; default;
  end;

  { TFPDDSQLEngine }
<<<<<<< HEAD
<<<<<<< HEAD
  TSQLEngineOption = (eoLineFeedAfterField,eoUseOldInWhereParams,eoAndTermsInBrackets,
                      eoQuoteFieldNames,eoLineFeedAfterAndTerm,eoAddTerminator,
                      eoSkipForeignkeys);
=======
  TSQLEngineOption = (eoLineFeedAfterField,eoUseOldInWhereParams,eoAndTermsInBrackets,eoQuoteFieldNames,eoLineFeedAfterAndTerm,eoAddTerminator);
>>>>>>> graemeg/fixes_2_2
=======
  TSQLEngineOption = (eoLineFeedAfterField,eoUseOldInWhereParams,eoAndTermsInBrackets,eoQuoteFieldNames,eoLineFeedAfterAndTerm,eoAddTerminator);
>>>>>>> origin/fixes_2_2
  TSQLEngineOptions = Set of TSQLEngineOption;
  

  TFPDDSQLEngine = Class(TPersistent)
  private
    FFieldQuoteChar: Char;
    FIndent: Integer;
    FMaxLineLength: Integer;
    FLastLength: integer;
    FOptions: TSQLEngineOptions;
    FTableDef: TDDTableDef;
    FNoIndent : Boolean;
    FTerminatorChar : Char;
  Protected
    procedure CheckTableDef;
    Procedure NoIndent;
    Procedure ResetLine;
    Procedure AddToStringLN(Var Res : String; S : String);
    Procedure AddToString(Var Res : String; S : String);
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure FixUpStatement(var Res : String; ForceTerminator : Boolean = False);
    Procedure FixUpStatement(SQL : TStrings; ForceTerminator : Boolean = False);
=======
    Procedure FixUpStatement(var Res : String);
>>>>>>> graemeg/fixes_2_2
=======
    Procedure FixUpStatement(var Res : String);
>>>>>>> origin/fixes_2_2
    Procedure AddWhereClause(Var Res : String; FieldList: TFPDDFieldList; UseOldParam:Boolean);
    Function CreateAndTerm(FD : TDDFieldDef; UseOldParam : Boolean): string;
    // Primitives. Override for engine-specifics
    Procedure AddFieldString(Var Res: String; S : String);
    Function FieldNameString(FD : TDDFieldDef) : string; virtual;
    Function TableNameString(TD : TDDTableDef) : string; virtual;
    Function FieldParamString(FD : TDDFieldDef; UseOldParam : Boolean) : string; virtual;
    Function FieldTypeString(ft : TFieldType; ASize,APrecision : Integer) : String; virtual;
    Function FieldTypeString(FD : TDDFieldDef) : String;
    Function FieldDefaultString(FD : TDDFieldDef) : String; virtual;
    Function FieldCheckString(FD : TDDFieldDef) : String; virtual;
    Function FieldDeclarationString(FD : TDDFieldDef) : String; virtual;
    Property FieldQuoteChar : Char Read FFieldQuoteChar Write FFieldQuoteChar;
    Property TerminatorChar : Char Read FTerminatorChar Write FTerminatorChar;
  Public
    Constructor Create; virtual;
    function  CreateWhereSQL(Var Res : String; FieldList: TFPDDFieldList; UseOldParam:Boolean): String;
<<<<<<< HEAD
<<<<<<< HEAD
    // Methods that fill a stringlist
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Procedure CreateSelectSQLStrings(FieldList,KeyFields : TFPDDFieldList; SQL : TStrings);
    Procedure CreateInsertSQLStrings(FieldList : TFPDDFieldList; SQL : TStrings);
    Procedure CreateUpdateSQLStrings(FieldList,KeyFields : TFPDDFieldList; SQL : TStrings);
    Procedure CreateDeleteSQLStrings(KeyFields : TFPDDFieldList; SQL : TStrings);
    Procedure CreateCreateSQLStrings(Fields,KeyFields : TFPDDFieldList; SQL : TStrings);
    Procedure CreateCreateSQLStrings(KeyFields : TFPDDFieldList; SQL : TStrings);
    Procedure CreateIndexesSQLStrings(Indexes : TFPDDIndexList; SQL : TStrings);
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure CreateForeignKeysSQLStrings(ForeignKeys: TDDForeignKeyDefs; SQL : TStrings);
    Procedure CreateSequencesSQLStrings(Sequences : TFPDDSequenceList; SQL : TStrings);
    Procedure CreateDomainsSQLStrings(Domains : TFPDDDomainList; SQL : TStrings);
    // Insert/Update/Delete statements.
=======
    Procedure CreateSequencesSQLStrings(Sequences : TFPDDSequenceList; SQL : TStrings);
    Procedure CreateDomainsSQLStrings(Domains : TFPDDDomainList; SQL : TStrings);
>>>>>>> graemeg/fixes_2_2
=======
    Procedure CreateSequencesSQLStrings(Sequences : TFPDDSequenceList; SQL : TStrings);
    Procedure CreateDomainsSQLStrings(Domains : TFPDDDomainList; SQL : TStrings);
>>>>>>> origin/fixes_2_2
    Function  CreateSelectSQL(FieldList,KeyFields : TFPDDFieldList) : String; virtual;
    Function  CreateInsertSQL(FieldList : TFPDDFieldList) : String; virtual;
    Function  CreateUpdateSQL(FieldList,KeyFields : TFPDDFieldList) : String; virtual;
    Function  CreateDeleteSQL(KeyFields : TFPDDFieldList) : String; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    // CREATE TABLE statement
    Function  CreateCreateSQL(Fields,KeyFields : TFPDDFieldList) : String; virtual;
    Function  CreateCreateSQL(KeyFields : TFPDDFieldList) : String; virtual;
    // CREATE INDEX
    Function  CreateIndexSQL(Index : TDDIndexDef) : String; virtual;
    Function  CreateIndexesSQL(Indexes : TFPDDIndexList) : String;
    Function  CreateIndexesSQL(Indexes : TDDIndexDefs) : String;
    // CONSTRAINT: Foreign keys
    Function  CreateForeignKeySQL(ForeignKey: TDDForeignKeyDef) : String;virtual;
    Function  CreateForeignKeysSQL(ForeignKeys: TDDForeignKeyDefs) : String;
    // CREATE SEQUENCE
    Function  CreateSequenceSQL(Sequence : TDDSequenceDef) : String; virtual;
    Function  CreateSequencesSQL(Sequences : TFPDDSequenceList) : String;
    Function  CreateSequencesSQL(Sequences : TDDSequenceDefs) : String;
    // CREATE DOMAIN
=======
=======
>>>>>>> origin/fixes_2_2
    Function  CreateCreateSQL(Fields,KeyFields : TFPDDFieldList) : String; virtual;
    Function  CreateCreateSQL(KeyFields : TFPDDFieldList) : String; virtual;
    // Indexes
    Function  CreateIndexSQL(Index : TDDIndexDef) : String; virtual;
    Function  CreateIndexesSQL(Indexes : TFPDDIndexList) : String;
    Function  CreateIndexesSQL(Indexes : TDDIndexDefs) : String;
    // Sequences
    Function  CreateSequenceSQL(Sequence : TDDSequenceDef) : String; virtual;
    Function  CreateSequencesSQL(Sequences : TFPDDSequenceList) : String;
    Function  CreateSequencesSQL(Sequences : TDDSequenceDefs) : String;
    // Domains
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function  CreateDomainSQL(Domain : TDDDomainDef) : String; virtual;
    Function  CreateDomainsSQL(Domains : TFPDDDomainList) : String;
    Function  CreateDomainsSQL(Domains : TDDDomainDefs) : String;
    // Convenience calls
    Function  CreateTableSQL : String;
    Procedure CreateTableSQLStrings(SQL : TStrings);
    Property TableDef : TDDTableDef Read FTableDef Write FTableDef;
  Published
    Property MaxLineLength : Integer Read FMaxLineLength Write FMaxLineLength default 72;
    Property Indent : Integer Read FIndent Write FIndent default 2;
    Property Options : TSQLEngineOptions Read FOptions Write FOptions;
  end;
  
  { TFPDDEngine }
  TFPDDEngineCapability =(ecImport,ecCreateTable,ecViewTable, ecTableIndexes,
                          ecRunQuery, ecRowsAffected, ecSequences, ecDomains);
  TFPDDEngineCapabilities = set of TFPDDEngineCapability;
  {
    to avoid dependencies on GUI elements in the data dictionary engines,
    connection string dialogs must be registered separately.

    TGetConnectionEvent is the callback prototype for such a dialog
  }
  TGetConnectionEvent = Procedure(Sender: TObject; Var Connection : String) of object;

  TFPDDEngine = Class(TComponent)
  private
    FOnProgress: TDDProgressEvent;
  Protected
    FConnected: Boolean;
    FConnectString: String;
    Procedure DoProgress(Const Msg : String);
    // Utility routine which can be used by descendents.
    procedure IndexDefsToDDIndexDefs(IDS : TIndexDefs; DDIDS : TDDindexDefs);
  Public
    Destructor Destroy; override;
    Function GetConnectString : String; virtual;
    // Mandatory for all data dictionary engines.
    Class function Description : string; virtual; abstract;
    Class function DBType : String; virtual; abstract;
    Class function EngineCapabilities : TFPDDEngineCapabilities; virtual;
    Function Connect(const ConnectString : String) : Boolean; virtual; abstract;
    Procedure Disconnect ; virtual; abstract;
    procedure ImportDatadict (Adatadict: TFPDataDictionary; UpdateExisting : Boolean);
    Function GetTableList(List : TStrings) : Integer; virtual; abstract;
    Function ImportTables(Tables : TDDTableDefs; List : TStrings; UpdateExisting : Boolean) : Integer;
    Function ImportFields(Table : TDDTableDef) : Integer; virtual; abstract;
    Function ImportIndexes(Table : TDDTableDef) : Integer; virtual; abstract;
    function GetDomainList(List: TSTrings) : integer; virtual;
    Function ImportDomains(Domains : TDDDomainDefs; List : TStrings; UpdateExisting : boolean) : Integer; virtual;
    function GetSequenceList (List:TStrings): integer; virtual;
    Function ImportSequences(Sequences : TDDSequenceDefs; List : TStrings; UpdateExisting : boolean) : Integer; virtual;

    // Override depending on capabilities
    Procedure CreateTable(Table : TDDTableDef); virtual;
    // Should not open the dataset.
    Function ViewTable(Const TableName: String; DatasetOwner : TComponent) : TDataset; virtual;
    // Run a non-select query. If possible, returns the number of modified records.
    Function RunQuery(SQL : String) : Integer; Virtual;
    // Create a select query TDataset. Do not open the resulting dataset.
    Function CreateQuery(SQL : String; DatasetOwner : TComponent) : TDataset; Virtual;
    // Assign a select query and open the resulting dataset.
    Procedure SetQueryStatement(SQL : String; AQuery : TDataset); Virtual;
    // Get table index defs. Return number of defs (if ecTableIndexes in capabilities)
    Function GetTableIndexDefs(ATableName : String; Defs : TDDIndexDefs) : integer ;virtual;
    // Override if a better implementation exists.
    Function CreateSQLEngine : TFPDDSQLEngine; virtual;
    Property OnProgress : TDDProgressEvent Read FOnProgress Write FOnProgress;
    Property ConnectString : String Read FConnectString;
    Property Connected : Boolean Read FConnected Write FConnected;
  end;
  TFPDDEngineClass = Class of TFPDDEngine;


  EDataDict = Class(Exception);

Procedure RegisterDictionaryEngine(AEngine :TFPDDEngineClass);
Function IsDictionaryEngineRegistered(AEngine :TFPDDEngineClass) : boolean;
Procedure RegisterConnectionStringCallback(Const AName: String; CallBack : TGetConnectionEvent);
Procedure UnRegisterDictionaryEngine(AEngine :TFPDDEngineClass);
Function  GetDictionaryEngineList(List : TStrings) : Integer;
Function  GetDictionaryEngineInfo(Const AName : String; out ADescription,ADBType: String; out ACapabilities : TFPDDEngineCapabilities) : boolean;
Function  CreateDictionaryEngine(AName : String; AOWner : TComponent) : TFPDDEngine;
Function IndexOptionsToString (Options : TIndexOptions) : String;

Var
  DefaultDDExt : String = '.fpd';
  
  // Default values for SQL Engine properties.
  
  DefaultSQLEngineOptions : TSQLEngineOptions
                          = [eoLineFeedAfterField,eoUseOldInWhereParams,
                             eoAndTermsInBrackets,eoLineFeedAfterAndTerm];
                             
  DefaultSQLEngineIndent     : Integer = 2;
  DefaultSQLEngineLineLength : Integer = 72;
  DefaultSQLTerminatorChar   : Char = ';';
  DefaultSQLFieldQuoteChar   : Char = '"';
  
implementation

uses typinfo;

{ ---------------------------------------------------------------------
  Constants, not to be localized
  ---------------------------------------------------------------------}

Const
  // Datadict saving
  SDataDict                 = 'FPDataDict';
  KeyDataDictName           = 'DataDictName';

  // Tables Saving
  SDataDictTables           = SDataDict+'_Tables';
  KeyTableName              = 'TableName';
  KeyPrimaryKeyConstraint   = 'PrimaryKeyConstraint';
  
  // Fields Saving
  SFieldSuffix              = '_Fields';
  KeyAlignMent              = 'AlignMent';
  KeyCustomConstraint       = 'CustomConstraint';
  KeyConstraintErrorMessage = 'ConstraintErrorMessage';
  KeyDBDefault              = 'DBDefault';
  KeyDefaultExpression      = 'DefaultExpression';
  KeyDisplayLabel           = 'DisplayLabel';
  KeyDisplayWidth           = 'DisplayWidth';
  KeyFieldName              = 'FieldName';
  KeyDomainName             = 'DomainName';
  KeyConstraint             = 'Constraint';
  KeyReadOnly               = 'ReadOnly';
  KeyRequired               = 'Required';
  KeyVisible                = 'Visible';
  KeySize                   = 'Size';
  KeyPrecision              = 'Precision';
  KeyFieldType              = 'FieldType';
  KeyHint                   = 'Hint';
  KeyProviderFlags          = 'Providerflags';
  
  // Index saving
  SIndexSuffix              = '_Indices';
  KeyExpression             = 'Expression';
  KeyFields                 = 'Fields';
  KeyCaseInsFields          = 'CaseInsFields';
  KeyDescFields             = 'DescFields';
  KeySource                 = 'Source';
  KeyOptions                = 'Options';
  
  // Foreign key Saving
  SKeySuffix                = '_FOREIGNKEYS';
  KeyKeyFields              = 'KeyFields';
  KeyKeyName                = 'KeyName';
  KeyReferencesTable        = 'ReferencesTable';
  KeyReferencedFields       = 'ReferencedFields';

  // Sequence saving
  SDatadictSequences        = SDataDict+'_Sequences';
  KeyStartValue             = 'StartValue';
  KeyIncrement              = 'Increment';

  // Domain saving
  SDataDictDomains          = SDataDict+'_Domains';
  KeyCheckConstraint        = 'Constraint';

  // SQL Keywords
  SSelect      = 'SELECT';
  SFrom        = 'FROM';
  SWhere       = 'WHERE';
  SInsertInto  = 'INSERT INTO';
  SUpdate      = 'UPDATE';
  SSet         = 'SET';
  SDeleteFrom  = 'DELETE FROM';
  SAnd         = 'AND';
  SOLD         = 'OLD';
  SValues      = 'VALUES';
  SCreateTable = 'CREATE TABLE';
  SNotNull     = 'NOT NULL';
  SDefault     = 'DEFAULT';
  SCheck       = 'CHECK';  // Check constraint
  SPrimaryKey  = 'PRIMARY KEY';
  SConstraint  = 'CONSTRAINT';

  SQLFieldTypes : Array[TFieldType] of string = (
    '', 'VARCHAR', 'SMALLINT', 'INT', 'SMALLINT',
    'BOOL', 'FLOAT', 'DECIMAL','DECIMAL','DATE', 'TIME', 'TIMESTAMP',
    '', '', 'INT', 'BLOB', 'BLOB', 'BLOB', 'BLOB',
    '', '', '', '', 'CHAR',
    'CHAR', 'BIGINT', '', '', '',
    '', '', '', '', '',
    '', '', 'TIMESTAMP', 'DECIMAL','CHAR','BLOB');
    
{ ---------------------------------------------------------------------
  Constants which can be localized
  ---------------------------------------------------------------------}

Resourcestring
  SErrFieldNotFound           = '"%s": Field "%s" not found.';
  SErrIndexNotFound           = '"%s": Index "%s" not found.';
  SErrTableNotFound           = 'Table "%s" not found.';
  SErrDuplicateTableName      = 'Duplicate table name: "%s"';
  SErrDuplicateFieldName      = '"%s": Duplicate field name: "%s"';
  SNewTable                   = 'NewTable';
  SNewField                   = 'NewField';
  SErrNoFileName              = 'No filename given for save';
  SErrNotRegistering          = 'Not registering data dictionary engine "%s": %s';
  SErrNoEngineCapabilities    = 'It reports no capabilities.';
  SErrNoEngineDBType          = 'It reports no database type';
  SErrNoEngineDescription     = 'It reports no description';
  SErrUnknownEngine           = 'Unknown datadictionary: "%s"';
  SErrMissingTableDef         = 'Cannot perform this operation without tabledef.';
  SErrFieldTypeNotSupported   = 'Field type "%s" is not supported in this SQL dialect';
  SErrNoConnectionDialog      = 'No connection dialog registered for data dictionary engine "%s".';
  SDDImportingTable           = 'Importing table definition for table "%s"';
  SErrCreateTableNotSupported = 'Creating tables is not supported by the "%s" engine.';
  SErrViewTableNotSupported   = 'Viewing tables is not supported by the "%s" engine.';
  SErrRunQueryNotSupported    = 'Running queries is not supported by the "%s" engine.';
  SErrOpenQueryNotSupported   = 'Running and opening SELECT queries is not supported by the "%s" engine.';
  SErrSetQueryStatementNotSupported = 'Setting the SQL statement is not supported by the "%s" engine.';
  SErrGetTableIndexDefsNotSupported = 'Getting index definitions of a table is not supported by the "%s" engine.';
  SSavingFieldsFrom           = 'Saving fields from %s';
  SLoadingFieldsFrom          = 'Loading fields from %s';
  SIndexOptionPrimary         = 'Primary key';
  SIndexOptionUnique          = 'Unique';
  SIndexOptionDescending      = 'Descending';
  SIndexOptionCaseInsensitive = 'Case insensitive';
  SIndexOptionExpression      = 'Expression';
  SIndexOptionNonMaintained   = 'Not maintained';
  SWarnFieldNotFound          = 'Could not find field "%s".';
  SLogFieldFoundIn            = 'Field "%s" found in table "%s".';
  SErrSequenceNotFound        = 'Sequence "%s" not found.';
  SErrDuplicateSequence       = 'Duplicate sequence name: "%s"';
  SErrDuplicateDomain         = 'Duplicate domain name: "%s"';
  SErrDomainNotFound          = 'Domain "%s" not found.';
  SErrNoDataDict              = '%s : No data dictionary available';
  SErrResolveDomain           = 'Cannot resolve domain';

Const
  IndexOptionNames : Array [TIndexOption] of String
                   = (SIndexOptionPrimary, SIndexOptionUnique,
                      SIndexOptionDescending, SIndexOptionCaseInsensitive,
                      SIndexOptionExpression, SIndexOptionNonMaintained);
                      
{ ---------------------------------------------------------------------
  Dictionary Engine registration
  ---------------------------------------------------------------------}

Var
  DDEngines : TStringList = nil;
  
Type

  { TEngineRegistration }

  TEngineRegistration = Class(TObject)
  Private
    FEngine : TFPDDEngineClass;
    FCallBack : TGetConnectionEvent;
  Public
    Constructor Create(AEngine : TFPDDEngineClass);
  end;

{ TEngineRegistration }

constructor TEngineRegistration.Create(AEngine: TFPDDEngineClass);
begin
  FEngine:=AEngine;
end;

procedure RegisterDictionaryEngine(AEngine: TFPDDEngineClass);
begin
  If (AEngine.EngineCapabilities=[]) then
     Raise EDataDict.CreateFmt(SErrNotRegistering,[AEngine.ClassName,SErrNoEngineCapabilities]);
  If (AEngine.DBType='') then
     Raise EDataDict.CreateFmt(SErrNotRegistering,[AEngine.ClassName,SErrNoEngineDBType]);
  If (AEngine.Description='') then
     Raise EDataDict.CreateFmt(SErrNotRegistering,[AEngine.ClassName,SErrNoEngineDescription]);
  If not assigned(DDEngines) then
    begin
    DDEngines:=TStringList.Create;
    DDEngines.Sorted:=true;
    DDEngines.Duplicates:=dupError;
    end;
  DDEngines.AddObject(Aengine.ClassName,TEngineRegistration.Create(AEngine));
end;


procedure UnRegisterDictionaryEngine(AEngine: TFPDDEngineClass);

Var
  I : Integer;
  
begin
  If Assigned(DDEngines) then
    begin
    I:=DDEngines.IndexOf(Aengine.ClassName);
    If (i<>-1) then
      begin
      DDEngines.Objects[i].Free;
      DDEngines.Delete(i);
      end;
    if (DDEngines.Count=0) then
      FreeAndNil(DDEngines);
    end;
end;

function GetDictionaryEngineList(List: TStrings): Integer;
begin
  If Not Assigned(DDEngines) then
    Result:=0
  else
    begin
    If Assigned(List) then
      List.Text:=DDEngines.Text;
    Result:=DDEngines.Count;
    end;
end;

Function IndexOfDDEngine(Const AName: String) : Integer;

begin
  If Assigned(DDEngines) then
    Result:=DDEngines.IndexOf(AName)
  else
    Result:=-1;
end;

Function FindEngineRegistration(Const AName : String) : TEngineRegistration;

Var
   I : integer;

begin
  I:=IndexOfDDEngine(AName);
  if (I<>-1) then
    Result:=TEngineRegistration(DDEngines.Objects[i])
  else
    Result:=Nil;
end;

Function GetEngineRegistration(Const AName : String) : TEngineRegistration;

begin
  Result:=FindEngineRegistration(AName);
  If (Result=Nil) then
    Raise EDataDict.CreateFmt(SErrUnknownEngine,[AName]);
end;

Function FindDictionaryClass(Const AName : String) : TFPDDEngineClass;

Var
   R : TEngineRegistration;

begin
  R:=FindEngineRegistration(AName);
  If (R=Nil) then
    Result:=Nil
  else
    Result:=R.FEngine;
end;

Function GetDictionaryClass(Const AName : String) : TFPDDEngineClass;

begin
  Result:=GetEngineRegistration(AName).FEngine;
end;

function IsDictionaryEngineRegistered(AEngine: TFPDDEngineClass): boolean;

Var
  I : Integer;

begin
  Result:=Assigned(DDEngines);
  If Result then
    begin
    Result:=False;
    I:=0;
    While (Not Result) and (I<DDEngines.Count) do
      begin
      Result:=(TEngineRegistration(DDEngines.Objects[i]).FEngine=AEngine);
      inc(I);
      end;
    end;
end;

procedure RegisterConnectionStringCallback(Const AName : String;
  CallBack: TGetConnectionEvent);
begin
  GetEngineRegistration(AName).FCallBack:=CallBack;
end;

function GetEngineConnectionStringCallBack(Const AName : String) : TGetConnectionEvent;

begin
  Result:=GetEngineRegistration(AName).FCallBack;
end;

Function  GetDictionaryEngineInfo(Const AName : String; out ADescription,ADBType: String;out ACapabilities : TFPDDEngineCapabilities) : boolean;

Var
  DDEC : TFPDDEngineClass;
  
begin
  DDEC:=FindDictionaryClass(AName);
  Result:=DDEC<>Nil;
  If Result then
    begin
    ADescription:=DDEC.Description;
    ADBType:=DDEC.DBType;
    ACapabilities:=DDEC.EngineCapabilities;
    end;
end;

function CreateDictionaryEngine(AName: String; AOWner : TComponent): TFPDDEngine;

begin
  Result:=GetDictionaryClass(AName).Create(AOwner);
end;

function IndexOptionsToString(Options: TIndexOptions): String;

Var
  IO : TIndexOption;

begin
  Result:='';
  For IO:=Low(TIndexOption) to High(TIndexOption) do
    If IO in Options then
      begin
      If (Result<>'') then
        Result:=Result+',';
      Result:=Result+IndexOptionNames[IO];
      end;
end;

{ ---------------------------------------------------------------------
  TDDFieldDef
  ---------------------------------------------------------------------}
  
function TDDFieldDef.IsSizeStored: Boolean;
begin
  Result:=FieldType in [ftUnknown, ftString, ftBCD,
    ftBytes, ftVarBytes, ftBlob, ftMemo, ftGraphic, ftFmtMemo,
    ftParadoxOle, ftDBaseOle, ftTypedBinary, ftFixedChar,
    ftWideString,ftArray, ftOraBlob, ftOraClob, ftFMTBcd];
end;

function TDDFieldDef.GetDomainName: string;
begin
  If Assigned(FDomain) then
    Result:=FDomain.DomainName
  else // Not resolved yet
    Result:=FDomainName;
end;

function TDDFieldDef.IsPrecisionStored: Boolean;
begin
  Result:=FieldType in [ftFloat,ftBCD,ftFMTBCD];
end;

procedure TDDFieldDef.SetDomain(const AValue: TDDDomainDef);
begin
  if FDomain=AValue then exit;
  FDomain:=AValue;
  If Assigned(FDomain) then
    FDomainName:=FDomain.DomainName;
end;

procedure TDDFieldDef.SetDomainName(const AValue: string);
begin
  FDomainName:=AValue;
  If (AValue<>'') then
    ResolveDomain(False);
end;

function TDDFieldDef.GetSectionName: String;
begin
  Result:=FFieldName;
end;

procedure TDDFieldDef.SetSectionName(const Value: String);
begin
  FFieldName:=Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDDFieldDef.GetDisplayName: string;
begin
  If (FieldName<>'') then
    Result:=FieldName
  else
    Result:=inherited GetDisplayName;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
constructor TDDFieldDef.Create(ACollection: TCollection);
begin
  Inherited;
  FVisible:=True;
  FAlignMent:=taLeftJustify;
end;

function TDDFieldDef.FieldDefs: TDDFieldDefs;
begin
  Result:=(Collection as TDDFieldDefs)
end;

function TDDFieldDef.DataDictionary: TFPDataDictionary;
begin
  If Assigned(FieldDefs) then
    Result:=FieldDefs.DataDictionary
  else
    Result:=Nil;
end;

function TDDFieldDef.FieldIsRequired: Boolean;
begin
  Result:=Required;
  If (Not Result) and (DomainName<>'') then
    begin
    ResolveDomain(True);
    Result:=Domain.Required;
    end;
end;

procedure TDDFieldDef.ResolveDomain(ErrorOnFail : Boolean);

Var
  DD : TFPDataDictionary;

begin
  If (FDomainName<>'') then
    Exit;
  DD:=DataDictionary;
  If Not Assigned(DD) then
    begin
    If ErrorOnFail then
      Raise EDataDict.CreateFmt(SErrNoDataDict,[SErrResolveDomain]);
    end
  else if (Not Assigned(FDomain)) or (CompareText(FDomain.DomainName,FDomainName)<>0) then
    begin
    If ErrorOnFail then
      FDomain:=DD.Domains.DomainByName(FDomainName)
    else
      FDomain:=DD.Domains.FindDomain(FDomainName);
    end;
end;

procedure TDDFieldDef.ImportFromField(F: TField; Existing : Boolean = True);
begin
  FieldName:=F.FieldName;
  FieldType:=F.DataType;
  If IsSizeStored then
    Size:=F.Size;
  If IsPrecisionStored then
    begin
    If F is TBCDFIeld then
      Precision:=TBCDField(F).Precision
    else if F is TFloatField then
      Precision:=TFloatField(F).Precision;
    end;
  if not Existing then
  begin
    AlignMent:=F.AlignMent;
    DisplayWidth:=F.DisplayWidth;
    CustomConstraint:=F.CustomConstraint;
    ConstraintErrorMessage:=F.ConstraintErrorMessage;
    DefaultExpression:=F.DefaultExpression;
    DisplayLabel:=F.DisplayLabel;
    ReadOnly:=F.ReadOnly;
    Required:=F.Required;
    Visible:=F.Visible;
    ProviderFlags:=F.ProviderFlags;
  end;
end;

procedure TDDFieldDef.ApplyToField(F: TField);
begin
{ // Normally, these should never be assigned...
  F.FieldName              := FieldName;
  F.DataType               := FieldType;
  If IsSizeStored then
    F.Size:=Size;
}
  F.AlignMent              := AlignMent;
  F.DisplayWidth           := DisplayWidth;
  F.CustomConstraint       := CustomConstraint;
  F.ConstraintErrorMessage := ConstraintErrorMessage;
  F.DefaultExpression      := DefaultExpression;
  F.DisplayLabel           := DisplayLabel;
  F.ReadOnly               := ReadOnly;
  F.Required               := Required;
  F.Visible                := Visible;
  F.ProviderFlags          := ProviderFlags;
end;

procedure TDDFieldDef.Assign(Source: TPersistent);

Var
  DF : TDDFieldDef;
  
begin
  if Source is TField then
    ImportFromField(TField(Source))
  else If Source is TDDFieldDef then
    begin
    DF:=TDDFieldDef(Source);
    FieldType:=DF.FieldType;
    If IsSizeStored then
      Size:=DF.Size;
    AlignMent:=DF.AlignMent;
    DisplayWidth:=DF.DisplayWidth;
    CustomConstraint:=DF.CustomConstraint;
    ConstraintErrorMessage:=DF.ConstraintErrorMessage;
    DefaultExpression:=DF.DefaultExpression;
    DBDefault:=DF.DBDefault;
    DisplayLabel:=DisplayLabel;
    FieldName:=DF.FieldName;
    DomainName:=DF.DomainName;
    Constraint:=DF.Constraint;
    Hint:=DF.Hint;
    ReadOnly:=DF.ReadOnly;
    Required:=DF.Required;
    Visible:=DF.Visible;
    ProviderFlags:=DF.ProviderFlags;
    end
  else
    Inherited;
end;

procedure TDDFieldDef.SaveToIni(Ini: TCustomInifile; ASection: String);

Var
  T : PTypeInfo;
  O : Integer;

begin
  With Ini do
    begin
    WriteInteger(ASection,KeyFieldType,Ord(Fieldtype));
    If IsSizeStored then
      WriteInteger(ASection,KeySize,Size);
    If IsPrecisionStored then
      WriteInteger(ASection,KeyPrecision,Precision);
    WriteInteger(ASection,KeyAlignMent,Ord(AlignMent));
    WriteInteger(ASection,KeyDisplayWidth,DisplayWidth);
    WriteString(ASection,KeyCustomConstraint,CustomConstraint);
    WriteString(ASection,KeyConstraintErrorMessage,ConstraintErrorMessage);
    WriteString(ASection,KeyDefaultExpression,DefaultExpression);
    WriteString(ASection,KeyDBDefault,DBDefault);
    WriteString(ASection,KeyDisplayLabel,DisplayLabel);
    WriteString(ASection,KeyFieldName,FieldName);
    WriteString(ASection,KeyDomainName,DomainName);
    WriteString(ASection,KeyConstraint,Constraint);
    WriteString(ASection,KeyHint,Hint);
    O:=Integer(ProviderFlags);
    T:=TypeInfo(TProviderFlags);
    WriteString(ASection,KeyProviderFlags,SetToString(T,O,False));
    WriteBool(ASection,KeyReadOnly,ReadOnly);
    WriteBool(ASection,KeyRequired,Required);
    WriteBool(ASection,KeyVisible,Visible);
    end;
end;

procedure TDDFieldDef.LoadFromIni(Ini: TCustomInifile; ASection: String);

Var
  T : PTypeInfo;
  O : Integer;
  PF : TProviderFlags;
  S : String;

begin
  With Ini do
    begin
    FieldType:=TFieldType(ReadInteger(ASection,KeyFieldType,Ord(Fieldtype)));
    If IsSizeStored then
      Size:=ReadInteger(ASection,KeySize,0);
    If IsPrecisionStored then
      Precision:=ReadInteger(ASection,KeyPrecision,0);
    Alignment:=TAlignment(ReadInteger(ASection,KeyAlignMent,Ord(AlignMent)));
    DisplayWidth:=ReadInteger(ASection,KeyDisplayWidth,DisplayWidth);
    CustomConstraint:=ReadString(ASection,KeyCustomConstraint,CustomConstraint);
    ConstraintErrorMessage:=ReadString(ASection,KeyConstraintErrorMessage,ConstraintErrorMessage);
    DefaultExpression:=ReadString(ASection,KeyDefaultExpression,DefaultExpression);
    DBDefault:=ReadString(ASection,KeyDBDefault,DBDefault);
    DisplayLabel:=ReadString(ASection,KeyDisplayLabel,DisplayLabel);
    FieldName:=ReadString(ASection,KeyFieldName,FieldName);
    DomainName:=ReadString(ASection,KeyDomainName,DomainName);
    Constraint:=ReadString(ASection,KeyConstraint,Constraint);
    Hint:=ReadString(ASection,KeyHint,Hint);
    S:=ReadString(ASection,KeyProviderFlags,'');
    T:=TypeInfo(TProviderFlags);
    O:=StringToSet(T,S);
    Integer(PF):=O;
    ProviderFlags:=PF;
    ReadOnly:=ReadBool(ASection,KeyReadOnly,ReadOnly);
    Required:=ReadBool(ASection,KeyRequired,Required);
    Visible:=ReadBool(ASection,KeyVisible,Visible);
    end;
end;

{ ---------------------------------------------------------------------
  TDDFieldDefs
  ---------------------------------------------------------------------}

procedure TDDFieldDefs.SetTableName(const AValue: String);
begin
  Inherited;
  FSectionPrefix:=AValue;
  GlobalSection:=AValue+SFieldSuffix;
end;

constructor TDDFieldDefs.Create(ATableDef: TDDTableDef);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Inherited Create(FieldDefClass);
=======
  Inherited Create(TDDFieldDef);
>>>>>>> graemeg/fixes_2_2
=======
  Inherited Create(TDDFieldDef);
>>>>>>> origin/fixes_2_2
  FPrefix:='Field';
  SetTableDef(ATableDef);
end;

constructor TDDFieldDefs.Create(ATableName: String);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Inherited Create(FieldDefClass);
=======
  Inherited Create(TDDFieldDef);
>>>>>>> graemeg/fixes_2_2
=======
  Inherited Create(TDDFieldDef);
>>>>>>> origin/fixes_2_2
  FPrefix:='Field';
  TableName:=ATableName;
end;

<<<<<<< HEAD
<<<<<<< HEAD
class function TDDFieldDefs.FieldDefClass: TDDFieldDefClass;
begin
  Result:=TDDFieldDef
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TDDFieldDefs.GetField(Index : Integer): TDDFieldDef;
begin
  Result:=TDDFieldDef(Items[Index]);
end;


<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TDDFieldDefs.SetField(Index : Integer; const AValue: TDDFieldDef);
begin
  Items[Index]:=AValue;
end;


function TDDFieldDefs.AddField(AFieldName: String): TDDFieldDef;

Var
  I : Integer;

begin
  If (AFieldName<>'') and (IndexOfField(AFieldName)<>-1) then
    Raise EDataDict.CreateFmt(SErrDuplicateFieldName,[TableName,AFieldName]);
  If (AFieldName='') then
    begin
    I:=0;
    Repeat
      Inc(I);
      AFieldName:=SNewField+IntToStr(i);
    Until (IndexOfField(AFieldName)=-1);
    end;
  Result:=Add as TDDFieldDef;
  Result.FieldName:=AFieldName;
end;

function TDDFieldDefs.IndexOfField(AFieldName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetField(Result).FieldName,AFieldName)<>0) do
    Dec(Result)
end;

function TDDFieldDefs.FindField(AFieldName: String): TDDFieldDef;

Var
  I : integer;
  
begin
  I:=IndexOfField(AFieldName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetField(I);
end;

function TDDFieldDefs.FieldByName(AFieldName: String): TDDFieldDef;
begin
  Result:=FindField(AFieldName);
  If Result=Nil then
    Raise EDatadict.CreateFmt(SErrFieldNotFound,[TableName,AFieldName]);
end;

procedure TDDFieldDefs.FillFieldList(const AFieldNames: String;
  List: TFPDDFieldList);

Var
  I : Integer;
  S,T : String;
  F : TDDFieldDef;
  
begin
  T:=Trim(AFieldNames);
  Repeat
    I:=Pos(';',T);
    If I=0 Then
      I:=Length(T)+1;
    S:=Trim(Copy(T,1,I-1));
    System.Delete(T,1,I);
    List.Add(FieldByName(S));
  Until (T='');
end;

{ ---------------------------------------------------------------------
  TDDTableDef
  ---------------------------------------------------------------------}
  
  
procedure TDDTableDef.SetTableName(const AValue: String);
begin
  FTableName:=AValue;
  FFieldDefs.TableName:=AValue;
  FIndexDefs.TableName:=AValue;
<<<<<<< HEAD
<<<<<<< HEAD
  FKeyDefs.TableName:=AValue;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TDDTableDef.GetPrimaryKeyName: String;
var i : TDDIndexDef;
begin
  if FPrimaryKeyName <> '' then
    Result := FPrimaryKeyName
  else
    begin
    I := GetPrimaryIndexDef;
    if assigned (I) then
      Result := I.IndexName
    else
      Result:=Tablename+'_PK';
    end;
end;

function TDDTableDef.GetPrimaryIndexDef: TDDIndexDef;
var r : integer;
begin
  r := Indexes.count;
  repeat
    dec (r);
  until (r < 0) or (ixPrimary in Indexes[r].Options);
  if r < 0 then
    result := nil
  else
    result := Indexes[r];
end;

function TDDTableDef.GetOnProgress: TDDProgressEvent;
begin
  Result:=Nil;
  If (Collection Is TDDTableDefs) then
    Result:=(Collection As TDDTableDefs).OnProgress;
end;

function TDDTableDef.GetSectionName: String;
begin
  Result:=FTableName;
end;

procedure TDDTableDef.SetSectionName(const Value: String);
begin
  TableName:=Value;
end;

constructor TDDTableDef.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FFieldDefs:=TDDFieldDefs.Create(Self);
  FIndexDefs:=TDDIndexDefs.Create(Self);
  FKeyDefs:=TDDForeignkeyDefs.Create('NewTable');
end;

destructor TDDTableDef.Destroy;

begin
  FreeAndNil(FKeyDefs);
  FreeAndNil(FFieldDefs);
  FreeAndNil(FIndexDefs);
  inherited Destroy;
end;

function TDDTableDef.DataDictionary: TFPDataDictionary;
begin
  If Assigned(TableDefs) then
    Result:=TableDefs.DataDictionary
  else
    Result:=Nil;
end;

function TDDTableDef.TableDefs: TDDTableDefs;
begin
  Result:=TDDTableDefs(Collection);
end;

Function TDDTableDef.ImportFromDataset(Dataset: TDataSet; DoClear : Boolean = False; UpdateExisting : Boolean = True) : Integer;

Var
  I : Integer;
  FD : TDDFieldDef;
  F : TField;
  FieldExists : Boolean;
  
begin
  if DoClear then
    FFieldDefs.Clear;
  Result:=0;
  For I:=0 to Dataset.Fields.Count-1 do
  begin
    F:=Dataset.Fields[i];
    FD:=FFieldDefs.FindField(F.FieldName);
    If (FD=Nil) then
    begin
      FD:=FFieldDefs.AddField(F.FieldName);
      FieldExists := False;
    end
    else
    begin
      if not UpdateExisting then FD:=Nil;
      FieldExists := True;
    end;
    if (FD<>Nil) then
    begin
      Inc(Result);
      FD.ImportFromField(F,FieldExists);
    end;
  end;
end;

procedure TDDTableDef.ApplyToDataset(Dataset: TDataset);

var
  I  : integer;
  FD : TDDFieldDef;
  F  : TField;
  
begin
  For I:=0 to Dataset.FieldCount-1 do
    begin
    F:=Dataset.Fields[i];
    FD:=FFieldDefs.FieldByName(F.FieldName);
    If (FD<>Nil) then
      FD.ApplyToField(F);
    end;
end;

function TDDTableDef.AddField(AFieldName: String): TDDFieldDef;
begin
  Result:=Fields.AddField(AFieldName);
end;

procedure TDDTableDef.SaveToIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini do
    begin
    WriteString(ASection,KeyTableName,TableName);
    WriteString(ASection,KeyPrimaryKeyConstraint,FPRimaryKeyName);
    end;
  If Assigned(OnProgress) then
    OnProgress(Self,Format(SSavingFieldsFrom,[TableName]));
  FFieldDefs.SaveToIni(Ini,ASection+SFieldSuffix);
  FIndexDefs.SaveToIni(Ini,ASection+SIndexSuffix);
<<<<<<< HEAD
<<<<<<< HEAD
  FKeyDefs.SaveToIni(Ini,ASection+SKeySuffix);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TDDTableDef.LoadFromIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini do
    begin
    TableName:=ReadString(ASection,KeyTableName,TableName);
    FPrimaryKeyName:=ReadString(ASection,KeyPrimaryKeyConstraint,'');
    end;
  If Assigned(OnProgress) then
    OnProgress(Self,Format(SLoadingFieldsFrom,[TableName]));
  FFieldDefs.LoadFromIni(Ini,ASection+SFieldSuffix);
  FIndexDefs.LoadFromIni(Ini,ASection+SIndexSuffix);
<<<<<<< HEAD
<<<<<<< HEAD
  FKeyDefs.LoadFromIni(Ini,ASection+SKeySuffix);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TDDTableDef.PrimaryIndexToFields;
var I : TDDIndexDef;
    r : integer;
    l : TFPDDFieldList;
begin
  I := GetPrimaryIndexDef;
  if assigned (I) then
    begin
    for r := 0 to Fields.count-1 do
      Fields[r].ProviderFlags := Fields[r].ProviderFlags - [pfInKey];
    l := TFPDDFieldList.create;
    try
      Fields.FillFieldList (I.Fields, l);
      for r := 0 to l.count-1 do
        l[r].ProviderFlags := l[r].ProviderFlags + [pfInKey];
    finally
      l.Free;
    end;
    end;
end;

procedure TDDTableDef.FieldsToPrimaryIndex;
var I : TDDIndexDef;
    r : integer;
    s : string;
begin
  I := GetPrimaryIndexDef;
  s := '';
  for r := 0 to fields.count-1 do
    if pfInKey in fields[r].ProviderFlags then
      s := s + ';' + fields[r].FieldName;
  if s = '' then
    begin
    if assigned (I) then
      I.Free;
    end
  else
    begin
    s := copy(s, 2, maxint);
    if assigned (I) then
      I.Fields := s
    else
      begin
      I := Indexes.AddIndex(GetPrimaryKeyName);
      I.Fields := s;
      I.Options := I.Options + [ixPrimary];
      end;
    end;
end;

{ ---------------------------------------------------------------------
  TDDTableDefs
  ---------------------------------------------------------------------}

function TDDTableDefs.GetTable(Index : Integer): TDDTableDef;
begin
  Result:=TDDTableDef(Items[Index]);
end;

procedure TDDTableDefs.SetTable(Index : Integer; const AValue: TDDTableDef);
begin
  Items[Index]:=AValue;
end;


function TDDTableDefs.AddTable(ATableName: String): TDDTableDef;

Var
  I : Integer;
  
begin
  If (ATableName<>'') and (IndexOfTable(ATableName)<>-1) then
    Raise EDataDict.CreateFmt(SErrDuplicateTableName,[ATableName]);
  If (ATableName='') then
    begin
    I:=0;
    Repeat
      Inc(I);
      ATAbleName:=SNewTable+IntToStr(i);
    Until (IndexOfTable(ATableName)=-1);
    end;
  Result:=Add as TDDTableDef;
  Result.TableName:=ATableName;
end;

function TDDTableDefs.IndexOfTable(ATableName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetTable(Result).TableName,ATableName)<>0) do
    Dec(Result)
end;

function TDDTableDefs.FindTable(ATableName: String): TDDTableDef;

Var
  I : integer;

begin
  I:=IndexOfTable(ATableName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetTable(I);
end;

function TDDTableDefs.TableByName(ATableName: String): TDDTableDef;
begin
  Result:=FindTable(ATableName);
  If Result=Nil then
    Raise EDatadict.CreateFmt(SErrTableNotFound,[ATableName]);
end;

{ ---------------------------------------------------------------------
  TDatadictionary
  ---------------------------------------------------------------------}

procedure TFPDataDictionary.SetOnProgress(const AValue: TDDProgressEvent);
begin
  FOnProgress:=AValue;
  FTables.OnProgress:=FOnProgress;
end;

constructor TFPDataDictionary.Create;
begin
  FTables:=TDDTableDefs.Create(TDDTableDef);
  FTables.FDataDictionary:=Self;
  FSequences:=TDDSequenceDefs.Create;
  FSequences.FDataDictionary:=Self;
  FDomains:=TDDDomainDefs.Create;
  FDomains.FDataDictionary:=Self;
end;

destructor TFPDataDictionary.Destroy;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FreeAndNil(FDomains);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FreeAndNil(FSequences);
  FreeAndNil(FTables);
  inherited Destroy;
end;

procedure TFPDataDictionary.SaveToFile(AFileName: String; KeepBackup: Boolean = True);

Var
  Ini : TMemIniFile;

begin
  If (AFileName='') then
    AFileName:=FFileName;
  if (AFileName='') and (Name<>'') then
    AFileName:=Name+DefaultDDExt;
  if (AFileName='') then
    Raise EDataDict.Create(SErrNoFileName);
  If FileExists(AFileName) then
    If KeepBackup then
      RenameFile(AFileName,AFileName+'.bak')
    else
      DeleteFile(AFileName);
  Ini:=TMemIniFile.Create(AFileName);
  try
    SaveToIni(Ini,SDataDict);
    Ini.UpdateFile;
    FFileName:=AFileName;
  finally
    FreeAndNil(Ini);
  end;
end;

procedure TFPDataDictionary.SaveToIni(Ini: TCustomIniFile; ASection: String);
begin
  Ini.WriteString(ASection,KeyDataDictName,Name);
  FDomains.SaveToIni(Ini,SDatadictDomains);
  FSequences.SaveToIni(Ini,SDatadictSequences);
  FTables.SaveToIni(Ini,SDatadictTables);
end;

procedure TFPDataDictionary.LoadFromFile(AFileName: String);

Var
  Ini : TMemInifile;

begin
  if (AFileName='') then
    Raise EDataDict.Create(SErrNoFileName);
  Ini:=TMemIniFile.Create(AFileName);
  try
    LoadFromIni(Ini,SDataDict);
    FFileName:=AFileName;
    If (Name='') then
      Name:=ChangeFileExt(ExtractFileName(AFileName),'');
  finally
    FreeAndNil(Ini);
  end;

end;

procedure TFPDataDictionary.LoadFromIni(Ini: TCustomIniFile; ASection: String);
begin
  FDDName:=Ini.ReadString(ASection,KeyDataDictName,'');
  FDomains.Clear;
  FDomains.LoadFromIni(Ini,SDataDictDomains);
  FSequences.Clear;
  FSequences.LoadFromIni(Ini,SDataDictSequences);
  FTables.Clear;
  FTables.LoadFromIni(Ini,SDataDictTables);
end;

procedure TFPDataDictionary.ApplyToDataset(ADataset: TDataset);
begin
  ApplytoDataset(ADataset,FOnApplyDatadictEvent);
end;

procedure TFPDataDictionary.ApplyToDataset(ADataset: TDataset;
  OnApply: TOnApplyDataDictEvent);

Var
  I : Integer;
  F : TField;
  FD : TDDFieldDef;
  FN,TN : String;
  Allow : Boolean;
  
begin
  For I:=0 to ADataset.Fields.Count-1 do
    begin
    F:=ADataset.Fields[i];
    FN:=F.Origin;
    If (FN='') then
      FN:=F.FieldName;
    FD:=FindFieldDef(FN,TN);
    Allow:=(FD<>Nil);
    If Assigned(OnApply) then
      OnApply(Self,FD,F,Allow);
    If (FD<>Nil) and Allow then
      FD.ApplyToField(F);
    end;
end;

function TFPDataDictionary.CanonicalizeFieldName(const InFN: String; Out TableDef : TDDTableDef; Out FN: String): Boolean;

Var
  TN : String;
  P : integer;
begin
  Result:=False;
  FN:=InFN;
  TableDef:=Nil;
  // Improve to check for quotes
  P:=Pos('.',FN);
  If (P<>0) then
    begin
    TN:=Copy(FN,1,P-1);
    Delete(FN,1,P);
    TableDef:=Tables.FindTable(TN);
    end;
  Result:=TableDef<>Nil;
end;

Function TFPDataDictionary.CanonicalizeFieldName(Const InFN : String; Out TN,FN : String) : Boolean;

Var
  TD : TDDTableDef;

begin
  Result:=CanonicalizeFieldName(InFN,TD,FN);
  If Result then
    TN:=TD.TableName
  else
    TN:='';
end;

// To be good, we should make a hashlist with all tables.fields and search that...
// For now, we cache the last matching table. This should work well for most common cases.
function TFPDataDictionary.FindFieldDef(FieldName: String; out TableName: String
  ): TDDFieldDef;

Var
  TD : TDDTableDef;
  FN,TN : String;
  I : Integer;
  
begin
  Result:=Nil;
  If CanonicalizeFieldName(FieldName,TD,FN) then
    begin
    Result:=TD.Fields.FieldByName(FN);
    If (Result<>Nil) then
      FLastMatchTableDef:=TD;
    end
  else
    begin
    If (FLastMatchTableDef<>Nil) then
      TD:=FLastMatchTableDef;
    If (TD<>Nil) then
      Result:=TD.Fields.FindField(FN);
    If Result=Nil then
      begin
      // Hard scan of all tables...
      I:=0;
      While (Result=Nil) and (I<Tables.Count) do
        begin
        TD:=Tables[i];
        Result:=TD.Fields.FindField(FN);
        If (Result<>Nil) then
          FLastMatchTableDef:=TD;
        Inc(I);
        end;
      end;
    end;
  If (Result<>Nil) then
    TableName:=FLastMatchTableDef.TableName;
end;

function TFPDataDictionary.FindFieldDef(FieldName: String): TDDFieldDef;

Var
  Dummy : String;

begin
  Result:=FindFieldDef(FieldName,Dummy);
end;

{ ---------------------------------------------------------------------
  TFPDDEngine
  ---------------------------------------------------------------------}

procedure TFPDDEngine.DoProgress(const Msg: String);
begin
  If Assigned(FOnProgress) then
    FOnProgress(Self,Msg);
end;

procedure TFPDDEngine.IndexDefsToDDIndexDefs(IDS: TIndexDefs; DDIDS: TDDindexDefs
  );
  
Var
  D : TIndexDef;
  DD : TDDindexDef;
  I : Integer;
  
begin
  DDIDS.Clear;
  For I:=0 to IDS.Count-1 do
    begin
    D:=IDS[I];
    DD:=DDIDS.AddDDIndexDef(D.Name);
    DD.Assign(D);
    end;
end;

destructor TFPDDEngine.Destroy;
begin
  Disconnect;
  inherited Destroy;
end;

function TFPDDEngine.GetConnectString: String;

Var
  CB : TGetConnectionEvent;
  
begin
  CB:=GetEngineConnectionStringCallBack(Self.ClassName);
  if (CB=Nil) then
    Raise EDataDict.CreateFmt(SerrNoConnectionDialog,[Self.ClassName]);
  Result:='';
  CB(Self,Result);
end;

function TFPDDEngine.ImportTables(Tables: TDDTableDefs; List: TStrings; UpdateExisting : Boolean): Integer;

Var
  I,J : Integer;
  TD : TDDTableDef;

begin
  Result:=0;
  For I:=0 to List.Count-1 do
    begin
    TD:=Nil;
    j:=Tables.IndexOfTable(List[i]);
    If (J=-1) then
      TD:=Tables.AddTable(List[i])
    else if UpdateExisting then
      TD:=Tables[J];
    If (TD<>nil) then
      begin
      DoProgress(Format(SDDImportingTable,[TD.TableName]));
      ImportFields(TD);
      if ecTableIndexes in EngineCapabilities then
        ImportIndexes(TD);
      Inc(Result);
      end
    end;
end;

function TFPDDEngine.GetDomainList(List: TSTrings): integer;
begin
  List.Clear;
  result := 0;
end;

function TFPDDEngine.CreateSQLEngine: TFPDDSQLEngine;
begin
  Result:=TFPDDSQLEngine.Create;
end;

class function TFPDDEngine.EngineCapabilities: TFPDDEngineCapabilities;
begin
  Result:=[];
end;

procedure TFPDDEngine.ImportDatadict(Adatadict: TFPDatadictionary;
  UpdateExisting: Boolean);
var L : TStringList;
    r : integer;
begin
  l := TStringlist.Create;
  try
    if ecDomains in EngineCapabilities then
      begin
      GetDomainList (L);
      if UpdateExisting then // Delete domains that don't exist anymore
        begin
        for r := ADatadict.Domains.count-1 downto 0 do
          if L.indexOf(ADatadict.Domains[r].DomainName) < 0 then
            ADatadict.Domains[r].Free;
        end;
      ImportDomains (ADatadict.Domains, L, UpdateExisting);
      end;

    L.Clear;
    GetTableList (L);
    if UpdateExisting then // delete tables that don't exist anymore
      begin
      for r := ADatadict.Tables.count-1 downto 0 do
        if L.indexOf(ADatadict.Tables[r].TableName) < 0 then
          ADatadict.Tables[r].Free;
      end;
    ImportTables (ADatadict.Tables, L, UpdateExisting);

    if ecSequences in EngineCapabilities then
      begin
      L.Clear;
      GetSequenceList (L);
      if UpdateExisting then // Delete sequences that don't exist anymore
        begin
        for r := ADatadict.Sequences.count-1 downto 0 do
          if L.indexOf(ADatadict.Sequences[r].SequenceName) < 0 then
            ADatadict.Sequences[r].Free;
        end;
      ImportSequences (ADatadict.Sequences, L, UpdateExisting);
      end;
  finally
    L.Free;
  end;
end;

function TFPDDEngine.ImportDomains(Domains: TDDDomainDefs; List : TStrings; UpdateExisting : boolean) : Integer;
begin
  result := 0;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  writeln ('importing no domains');
>>>>>>> graemeg/fixes_2_2
=======
  writeln ('importing no domains');
>>>>>>> origin/fixes_2_2
end;

function TFPDDEngine.GetSequenceList(List: TStrings): integer;
begin
  List.Clear;
  result := 0;
end;

function TFPDDEngine.ImportSequences(Sequences: TDDSequenceDefs; List : TStrings; UpdateExisting : boolean) : Integer;
begin
  result := 0;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  writeln ('importing no sequences');
>>>>>>> graemeg/fixes_2_2
=======
  writeln ('importing no sequences');
>>>>>>> origin/fixes_2_2
end;

procedure TFPDDEngine.CreateTable(Table: TDDTableDef);
begin
  Raise EDataDict.CreateFmt(SErrCreateTableNotSupported,[DBType]);
end;

function TFPDDEngine.ViewTable(Const TableName: String; DatasetOwner: TComponent
  ): TDataset;
begin
  Raise EDataDict.CreateFmt(SErrViewTableNotSupported,[DBType]);
end;

function TFPDDEngine.RunQuery(SQL: String): Integer;

begin
  Raise EDataDict.CreateFmt(SErrRunQueryNotSupported,[DBType]);
end;

function TFPDDEngine.CreateQuery(SQL: String; DatasetOwner : TComponent): TDataset;
begin
  Raise EDataDict.CreateFmt(SErrOpenQueryNotSupported,[DBType]);
end;

procedure TFPDDEngine.SetQueryStatement(SQL: String; AQuery: TDataset);
begin
  Raise EDataDict.CreateFmt(SErrSetQueryStatementNotSupported,[DBType]);
end;

function TFPDDEngine.GetTableIndexDefs(ATableName: String; Defs: TDDIndexDefs
  ): integer;
begin
  Raise EDataDict.CreateFmt(SErrGetTableIndexDefsNotSupported,[DBType]);
end;

{ ---------------------------------------------------------------------
  TFPDDSQLEngine
  ---------------------------------------------------------------------}

{ Utility functions }

constructor TFPDDSQLEngine.Create;
begin
  FTerminatorChar:=DefaultSQLTerminatorChar;
  FFieldQuoteChar:=DefaultSQLFieldQuoteChar;
  FOptions:=DefaultSQLEngineOptions;
  FMaxLineLength:=DefaultSQLEngineLineLength;
  FIndent:=DefaultSQLEngineIndent;
end;

procedure TFPDDSQLEngine.CheckTableDef;
begin
  If (FTableDef=Nil) then
    Raise EDataDict.Create(SErrMissingTableDef);
end;

procedure TFPDDSQLEngine.NoIndent;
begin
  FNoIndent:=True;
end;

procedure TFPDDSQLEngine.ResetLine;
begin
  FLastLength:=0;
  NoIndent;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFPDDSQLEngine.FixUpStatement(var Res: String; ForceTerminator : Boolean = False);

Var
  L : Integer;

begin
  Res:=Trim(Res);
  if (eoAddTerminator in Options) or ForceTerminator then
    begin
    L:=Length(Res);
    If (L=0) or (Res[L]<>FTerminatorChar) then
      Res:=Res+FTerminatorChar;
    end;
end;

procedure TFPDDSQLEngine.FixUpStatement(SQL: TStrings; ForceTerminator: Boolean = False);

Var
  S : String;

begin
  If (SQL.Count>0) then
    begin
    S:=SQL[SQL.Count-1];
    FixupStatement(S,ForceTerminator);
    SQL[SQL.Count-1]:=S;
    end;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFPDDSQLEngine.FixUpStatement(var Res: String);
begin
  Res:=Trim(Res);
  if (eoAddTerminator in Options) then
    Res:=Res+FTerminatorChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Procedure TFPDDSQLEngine.AddToStringLN(Var Res : String;S : String);

begin
  AddToString(Res,S);
  Res:=Res+LineEnding;
  FLastLength:=0;
end;

procedure TFPDDSQLEngine.AddToString(Var Res: String; S: String);
begin
  If (FMaxLineLength>0) and (FLastLength+Length(S)+1>FMaxLineLength) then
    begin
    FLastLength:=0;
    Res:=Res+LineEnding;
    end
  else If (FLastLength<>0) and (S<>'') then
    S:=' '+S;
  If (FLastlength=0) then
    begin
    If not FNoIndent then
      begin
      Res:=Res+StringOfChar(' ',Indent);
      FLastlength:=FlastLength+Indent;
      end;
    end;
  FLastLength:=FLastLength+Length(S);
  FNoIndent:=False;
  Res:=Res+S;
end;

procedure TFPDDSQLEngine.AddFieldString(var Res: String; S: String);
begin
  If eoLineFeedAfterField in FOptions then
    AddToStringLn(Res,S)
  else
    AddToString(Res,S)
end;

function TFPDDSQLEngine.CreateAndTerm(FD: TDDFieldDef; UseOldParam: Boolean
  ): string;
begin
  Result:=FieldNameString(FD)+' = '+FieldParamString(FD,UseOldParam);
  if (eoAndTermsInBrackets in FOptions) then
    Result:='('+Result+')';
end;

function TFPDDSQLEngine.CreateWhereSQL(var Res : String;FieldList: TFPDDFieldList; UseOldParam:Boolean): String;

Var
  i : Integer;
  FD : TDDFieldDef;
  S : String;

begin
  Result:='';
  If Assigned(FieldList) and (FieldList.Count>0) then
    begin
    For i:=0 to FieldList.Count-1 do
      begin
      FD:=FieldList[i];
      S:=CreateAndTerm(FD,UseOldParam);
      If (I>0) then
        S:=SAnd+' '+S;
      If eoLineFeedAfterAndTerm in Options then
        AddToStringLN(Res,S)
      else
        AddToString(Res,S);
      end;
    end;
end;

procedure TFPDDSQLEngine.AddWhereClause(var Res: String;
  FieldList: TFPDDFieldList; UseOldParam: Boolean);
begin
  If Assigned(FieldList) and (FieldList.Count>0) then
    begin
    NoIndent;
    AddToStringLn(Res,SWhere);
    CreateWhereSQL(Res,FieldList,UseOldParam);
    end;
end;

{ Functions with engine-specific strings in it. Can be overridden }

function TFPDDSQLEngine.FieldNameString(FD: TDDFieldDef): string;
begin
  Result:=FD.FieldName;
  if (eoQuoteFieldNames in FOptions) then
    Result:=FFieldQuoteChar+Result+FFieldQuoteChar;
end;

function TFPDDSQLEngine.TableNameString(TD: TDDTableDef): string;
begin
  Result:=TD.TableName;
end;

function TFPDDSQLEngine.FieldParamString(FD: TDDFieldDef; UseOldParam: Boolean
  ): string;
begin
  Result:=FD.FieldName;
  If UseOldParam then
    Result:=SOLD+Result;
  Result:=':'+Result;
end;

function TFPDDSQLEngine.FieldTypeString(FD : TDDFieldDef) : String;

begin
  if FD.DomainName <> '' then
    Result := FD.DomainName
  else
    Result:=FieldTypeString(FD.FieldType,FD.Size,FD.Precision);
end;


Function TFPDDSQLEngine.FieldTypeString(FT : TFieldType; ASize,APrecision : Integer) : String;
{
ftUnknown, ftString, ftSmallint, ftInteger, ftWord,
    ftBoolean, ftFloat, ftCurrency, ftBCD, ftDate,  ftTime, ftDateTime,
    ftBytes, ftVarBytes, ftAutoInc, ftBlob, ftMemo, ftGraphic, ftFmtMemo,
    ftParadoxOle, ftDBaseOle, ftTypedBinary, ftCursor, ftFixedChar,
    ftWideString, ftLargeint, ftADT, ftArray, ftReference,
    ftDataSet, ftOraBlob, ftOraClob, ftVariant, ftInterface,
    ftIDispatch, ftGuid, ftTimeStamp, ftFMTBcd}
begin
  Result:=SQLFieldTypes[FT];
  If (Result='') then
    Raise EDataDict.CreateFmt(SErrFieldTypeNotSupported,[GetEnumName(TypeInfo(TFieldType),Ord(FT))]);
  case FT of
    ftString,
    ftFixedChar,
    ftWideString :
      Result:=Result+Format('(%d)',[ASize]);
    ftBCD,
    ftFMTBCD :
      Result:=Result+Format('(%d,%d)',[APrecision,ASize]);
  end;
end;

function TFPDDSQLEngine.FieldDefaultString(FD : TDDFieldDef) : String;

begin
  Result:=SDefault+' '+FD.DBDefault;
end;

function TFPDDSQLEngine.FieldCheckString(FD : TDDFieldDef) : String;

begin
  Result:=Trim(FD.Constraint);
  If (Result<>'') then
    begin
    If (Result[1]<>'(') or (Result[Length(Result)]<>')') then
      Result:='('+Result+')';
    Result:=SCheck+' '+Result;
    end;
end;

function TFPDDSQLEngine.FieldDeclarationString(FD : TDDFieldDef) : String;

var
  S : String;

begin
  Result:=FieldNameString(FD)+' '+FieldTypeString(FD);
  If (FD.DBDefault<>'') then
    Result:=Result+' '+FieldDefaultString(FD);
  If FD.Required then
    Result:=Result+' '+SNotNull;
  S:=FieldCheckString(FD);
  If (S<>'') then
    Result:=Result+' '+S;
end;

{ SQL Creation functions. Can be overridden if needed. }
  
function TFPDDSQLEngine.CreateSelectSQL(FieldList, KeyFields: TFPDDFieldList
  ): String;
  
Var
  i : Integer;
  FD : TDDFieldDef;
  S : String;
  
begin
  CheckTableDef;
  Result:='';
  ResetLine;
  AddToStringLn(Result,SSelect);
  For i:=0 to FieldList.Count-1 do
    begin
    FD:=FieldList[i];
    S:=FieldNameString(FD);
    If (I<FieldList.Count-1) then
      S:=S+',';
    AddFieldString(Result,S);
    end;
  If Not (eoLineFeedAfterField in FOptions) then
    AddToStringLn(Result,'');
  NoIndent;
  AddToStringLn(Result,SFrom);
  AddToStringLn(Result,TableNameString(TableDef));
  AddWhereClause(Result,KeyFields,False);
  FixUpStatement(Result);
end;


function TFPDDSQLEngine.CreateInsertSQL(FieldList: TFPDDFieldList): String;

Var
  i : Integer;
  FD : TDDFieldDef;
  S : String;

begin
  CheckTableDef;
  Result:='';
  ResetLine;
  AddToString(Result,SInsertInto);
  AddToStringLn(Result,TableNameString(TableDef));
  For i:=0 to FieldList.Count-1 do
    begin
    FD:=FieldList[i];
    S:=FieldNameString(FD);
    If (I=0) then
      S:='('+S;
    If (I<FieldList.Count-1) then
      S:=S+','
    else
      S:=S+')';
    AddFieldString(Result,S);
    end;
  If Not (eoLineFeedAfterField in FOptions) then
    AddToStringLn(Result,'');
  NoIndent;
  AddToStringLn(Result,SValues);
  For i:=0 to FieldList.Count-1 do
    begin
    FD:=FieldList[i];
    S:=FieldParamString(FD,False);
    If (I=0) then
      S:='('+S;
    If (I<FieldList.Count-1) then
      S:=S+','
    else
      S:=S+')';
    AddFieldString(Result,S);
    end;
  FixUpStatement(Result);
end;

function TFPDDSQLEngine.CreateUpdateSQL(FieldList, KeyFields: TFPDDFieldList
  ): String;

Var
  i : Integer;
  FD : TDDFieldDef;
  S : String;

begin
  CheckTableDef;
  ResetLine;
  Result:='';
  AddToString(Result,SUPDATE);
  AddToStringLN(Result,TableNameString(TableDef));
  NoIndent;
  AddToStringLN(Result,SSET);
  If Assigned(FieldList) and (FieldList.Count>0) then
    begin
    For i:=0 to FieldList.Count-1 do
      begin
      FD:=FieldList[i];
      S:=FieldNameString(FD)+' = '+FieldParamString(FD,False);
      If (I<FieldList.Count-1) then
        S:=S+',';
      AddFieldString(Result,S);
      end;
    end;
  AddWhereClause(Result,KeyFields,eoUseOldInWhereParams in Options);
  FixUpStatement(Result);
end;

function TFPDDSQLEngine.CreateDeleteSQL(KeyFields: TFPDDFieldList): String;
begin
  CheckTableDef;
  ResetLine;
  Result:='';
  AddToStringLN(Result,SDeleteFrom);
  AddToStringLN(Result,TableNameString(TableDef));
  AddWhereClause(Result,KeyFields,eoUseOldInWhereParams in Options);
  FixUpStatement(Result);
end;


function TFPDDSQLEngine.CreateCreateSQL(Fields, KeyFields: TFPDDFieldList
  ): String;
  
Var
  S : String;
  I : integer;
  
begin
  CheckTableDef;
  Result:='';
  ResetLine;
  AddToStringLn(Result,SCreateTable+' '+TableNameString(TableDef)+' (');
  For I:=0 to Fields.Count-1 do
    begin
    S:=FieldDeclarationString(Fields[i]);
    If (I<Fields.Count-1) or (Assigned(KeyFields) and (KeyFields.Count<>0)) then
      S:=S+',';
    AddToStringLn(Result,S);
    end;
  If (Assigned(KeyFields) and (KeyFields.Count<>0)) then
    begin
    S:=SCONSTRAINT+' '+TableDef.PrimaryKeyConstraintName+' '+SPrimaryKey+' (';
    For I:=0 to KeyFields.Count-1 do
      begin
      S:=S+FieldNameString(KeyFields[i]);
      If I<KeyFields.Count-1 then
        S:=S+','
      else
        S:=S+')'
      end;
    AddToStringLn(Result,S);
    end;
  NoIndent;
  AddToStringLn(Result,')');
  FixUpStatement(Result);
end;

function TFPDDSQLEngine.CreateCreateSQL(KeyFields: TFPDDFieldList): String;

Var
  Fl : TFPDDFieldList;

begin
  CheckTableDef;
  FL:=TFPDDfieldList.CreateFromTableDef(TableDef);
  try
    FL.OwnsObjects:=False;
    Result:=CreateCreateSQL(FL,KeyFields);
  finally
    FL.Free;
  end;
end;

function TFPDDSQLEngine.CreateIndexSQL(Index: TDDIndexDef): String;

Var
  L : TFPDDFieldList;
  I : Integer;
  
begin
  Result:='CREATE ';
  If ixUnique in Index.Options then
    Result:=Result+'UNIQUE ';
  If ixDescending in Index.Options then
    Result:=Result+'DESCENDING ';
  Result:=Result+'INDEX '+Index.IndexName;
  Result:=Result+' ON '+TableDef.TableName+' (';
  L:=TFPDDFieldList.Create;
  try
    L.OwnsObjects:=False;
    TableDef.Fields.FillFieldList(Index.Fields,L);
    For I:=0 to L.Count-1 do
      begin
      If (I>0) then
        Result:=Result+',';
      Result:=Result+L[I].FieldName;
      end;
  finally
    L.Free;
  end;
  Result:=Result+')';
end;

function TFPDDSQLEngine.CreateIndexesSQL(Indexes: TFPDDIndexList): String;

Var
  SQL : TStringList;

begin
  SQL:=TStringList.Create;
  try
    CreateIndexesSQLStrings(Indexes,SQL);
    Result:=SQL.Text;
  finally
    SQL.free;
  end;
end;

function TFPDDSQLEngine.CreateIndexesSQL(Indexes: TDDIndexDefs): String;

Var
  IL : TFPDDIndexList;

begin
  IL:=TFPDDIndexList.CreateFromIndexDefs(Indexes);
  try
    IL.OwnsObjects:=False;
    Result:=CreateIndexesSQL(IL);
  finally
    IL.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TFPDDSQLEngine.CreateForeignKeySQL(ForeignKey: TDDForeignKeyDef
  ): String;

begin
  Result:=Format('ALTER TABLE %s ADD CONSTRAINT %s',[TableDef.TableName,ForeignKey.KeyName]);
  Result:=Result+Format(' FOREIGN KEY (%s)',[ForeignKey.KeyFields]);
  Result:=Result+Format(' REFERENCES %s(%s)',[ForeignKey.ReferencesTable,ForeignKey.ReferencedFields])
end;

function TFPDDSQLEngine.CreateForeignKeysSQL(ForeignKeys: TDDForeignKeyDefs
  ): String;

Var
  SQL : TStrings;

begin
  SQL:=TStringList.Create;
  try
    CreateForeignKeysSQLStrings(ForeignKeys,SQL);
    Result:=SQL.Text;
  finally
    SQL.Free;
  end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TFPDDSQLEngine.CreateSequenceSQL(Sequence: TDDSequenceDef): String;
begin
  Result:='CREATE SEQUENCE '+Sequence.SequenceName;
  If (Sequence.StartValue>0) then
    Result:=Result+'START WITH '+IntToStr(Sequence.StartValue);
  If (Sequence.Increment<>0) then
    Result:=Result+'INCREMENT BY '+IntToStr(Sequence.Increment);
end;

function TFPDDSQLEngine.CreateSequencesSQL(Sequences: TFPDDSequenceList): String;

Var
  SQL : TStrings;

begin
  SQL:=TStringList.Create;
  Try
    CreateSequencesSQLStrings(Sequences,SQL);
    Result:=SQL.Text;
  Finally
    SQL.Free;
  end;
end;

function TFPDDSQLEngine.CreateSequencesSQL(Sequences: TDDSequenceDefs): String;

Var
  L : TFPDDSequenceList;

begin
  L:=TFPDDSequenceList.CreateFromSequenceDefs(Sequences);
  try
    L.OwnsObjects:=False;
    Result:=CreateSequencesSQl(L);
  finally
    L.Free;
  end;
end;

function TFPDDSQLEngine.CreateDomainSQL(Domain: TDDDomainDef): String;
begin
  Result:='CREATE DOMAIN '+Domain.DomainName+' ';
  Result:=Result+FieldTypeString(Domain.FieldType,Domain.Size,Domain.Precision);
  If Domain.Required then
    Result:=Result+' NOT NULL';
  If (Domain.CheckConstraint<>'') then
    Result:=Result+' CHECK ('+Domain.CheckConstraint+')';
end;

function TFPDDSQLEngine.CreateDomainsSQL(Domains: TFPDDDomainList): String;

Var
  SQL : TStrings;

begin
  SQL:=TStringList.Create;
  Try
    CreateDomainsSQLStrings(Domains,SQL);
    Result:=SQL.Text;
  Finally
    SQL.Free;
  end;
end;

function TFPDDSQLEngine.CreateDomainsSQL(Domains: TDDDomainDefs): String;
Var
  L : TFPDDDomainList;

begin
  L:=TFPDDDomainList.CreateFromDomainDefs(Domains);
  try
    L.OwnsObjects:=False;
    Result:=CreateDomainsSQl(L);
  finally
    L.Free;
  end;
end;

function TFPDDSQLEngine.CreateTableSQL: String;

Var
  SQL : TStrings;

begin
  SQL:=TStringList.Create;
  try
    CreateTableSQLStrings(SQL);
    Result:=SQL.Text;
  finally
    SQL.Free;
  end;
end;

procedure TFPDDSQLEngine.CreateTableSQLStrings(SQL: TStrings);

Var
  L : TStrings;
  I : Integer;
  KF : TFPDDFieldlist;
  ID : TDDIndexDef;
  FD : TDDFieldDef;
<<<<<<< HEAD
<<<<<<< HEAD
  S : String;

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
begin
  CheckTableDef;
  L:=TStringList.Create;
  try
    KF:=TFPDDFieldlist.Create(False);
    try
      KF.OwnsObjects:=False;
      if assigned (TableDef.PrimaryIndexDef) then
        TableDef.fields.FillFieldList(TableDef.PrimaryIndexDef.Fields, KF)
      else
        For I:=0 to TableDef.Fields.Count-1 do
          begin
          FD:=TableDef.Fields[I];
          If pfInKey in FD.ProviderFlags then
            KF.Add(FD);
          end;
      CreateCreateSQLStrings(KF,SQL);
<<<<<<< HEAD
<<<<<<< HEAD
      FixupStatement(SQL,True);
      L.Text:=CreateIndexesSQL(TableDef.Indexes);
      If (L.Count>0) then
        begin
        SQL.AddStrings(L);
        FixupStatement(SQL,True);
        end;
      L.Clear;
      If Not (eoSkipForeignKeys in Options) then
        L.Text:=CreateForeignKeysSQL(TableDef.ForeignKeys);
=======
      L.Text:=CreateIndexesSQL(TableDef.Indexes);
>>>>>>> graemeg/fixes_2_2
=======
      L.Text:=CreateIndexesSQL(TableDef.Indexes);
>>>>>>> origin/fixes_2_2
      SQL.AddStrings(L);
    finally
      KF.Free;
    end;
  finally
    L.Free;
  end;
end;

{ TStrings versions of SQL creation statements. }

procedure TFPDDSQLEngine.CreateSelectSQLStrings(FieldList,KeyFields: TFPDDFieldList; SQL: TStrings);

begin
  SQL.Text:=CreateSelectSQL(FieldList,KeyFields);
end;

procedure TFPDDSQLEngine.CreateInsertSQLStrings(FieldList: TFPDDFieldList; SQL: TStrings);
begin
  SQL.Text:=CreateInsertSQL(FieldList);
end;

procedure TFPDDSQLEngine.CreateUpdateSQLStrings(FieldList, KeyFields: TFPDDFieldList;
  SQL: TStrings);
begin
  SQL.Text:=CreateUpdateSQL(FieldList,KeyFields);
end;

procedure TFPDDSQLEngine.CreateDeleteSQLStrings(KeyFields: TFPDDFieldList;
  SQL: TStrings);
begin
  SQL.Text:=CreateDeleteSQL(KeyFields);
end;

procedure TFPDDSQLEngine.CreateCreateSQLStrings(Fields,
  KeyFields: TFPDDFieldList; SQL: TStrings);
begin
  SQL.Text:=CreateCreateSQL(Fields,KeyFields);
end;

procedure TFPDDSQLEngine.CreateCreateSQLStrings(KeyFields: TFPDDFieldList;
  SQL: TStrings);
begin
  SQL.Text:=CreateCreateSQL(KeyFields);
end;

procedure TFPDDSQLEngine.CreateIndexesSQLStrings(Indexes: TFPDDIndexList; SQL: TStrings);

Var
  I : integer;

begin
  For I:=0 to Indexes.Count-1 do
    if not (ixPrimary in Indexes[i].Options) then
      SQL.Add(CreateIndexSQL(Indexes[i])+TerminatorChar);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFPDDSQLEngine.CreateForeignKeysSQLStrings(
  ForeignKeys: TDDForeignKeyDefs; SQL: TStrings);

Var
  I : integer;

begin
  For I:=0 to ForeignKeys.Count-1 do
    SQL.Add(CreateForeignKeySQL(ForeignKeys[i])+TerminatorChar);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TFPDDSQLEngine.CreateSequencesSQLStrings(Sequences: TFPDDSequenceList;
  SQL: TStrings);

Var
  I : integer;

begin
  For I:=0 to Sequences.Count-1 do
    SQL.Add(CreateSequenceSQL(Sequences[i])+TerminatorChar);
end;

procedure TFPDDSQLEngine.CreateDomainsSQLStrings(Domains: TFPDDDomainList;
  SQL: TStrings);

Var
  I : integer;

begin
  For I:=0 to Domains.Count-1 do
    SQL.Add(CreateDomainSQL(Domains[i])+TerminatorChar);
end;

{ ---------------------------------------------------------------------
  TDDFieldList
  ---------------------------------------------------------------------}

function TFPDDFieldList.GetFieldDef(Index : Integer): TDDFieldDef;
begin
  Result:=TDDFieldDef(Items[Index]);
end;

procedure TFPDDFieldList.SetFieldDef(Index : Integer; const AValue: TDDFieldDef);
begin
  Items[Index]:=AValue;
end;

constructor TFPDDFieldList.CreateFromTableDef(TD: TDDTableDef);

begin
  CreateFromFieldDefs(TD.Fields);
end;

constructor TFPDDFieldList.CreateFromFieldDefs(FD: TDDFieldDefs);

Var
  I : Integer;

begin
  Inherited Create;
  Capacity:=FD.Count;
  For I:=0 to FD.Count-1 do
    Add(FD[i]);
end;

function TFPDDIndexList.GetIndexDef(AIndex: Integer): TDDIndexDef;
begin
  Result:=TDDIndexDef(Items[AIndex]);
end;

procedure TFPDDIndexList.SetIndexDef(AIndex: Integer; const AValue: TDDIndexDef
  );
begin
  Items[AIndex]:=AValue
end;

constructor TFPDDIndexList.CreateFromIndexDefs(FD: TDDIndexDefs);

var
  I : Integer;

begin
  Inherited Create;
  For I:=0 to FD.Count-1 do
    Add(FD[I]);
end;

{ TDDIndexDef }

function TDDIndexDef.GetSectionName: String;
begin
  Result:=IndexName;
end;

procedure TDDIndexDef.SetSectionName(const Value: String);
begin
  IndexName:=Value;
end;

procedure TDDIndexDef.Assign(ASource: TPersistent);

Var
  DD : TDDIndexDef;
  D : TIndexDef;

begin
  If ASource is TDDIndexDef then
    begin
    DD:=ASource as TDDIndexDef;
    IndexName:=DD.IndexName;
    Expression:=DD.Expression;
    Fields:=DD.Expression;
    CaseInsFields:=DD.CaseInsFields;
    DescFields:=DD.DescFields;
    Options:=DD.Options;
    Source:=DD.Source;
    end
  else if ASource is TIndexDef then
    begin
    D:=ASource as TIndexDef;
    IndexName:=D.Name;
    Expression:=D.Expression;
    Fields:=D.Fields;
    CaseInsFields:=D.CaseInsFields;
    DescFields:=D.DescFields;
    Options:=D.Options;
    Source:=D.Source;
    end
  else
    inherited Assign(ASource);
end;

procedure TDDIndexDef.SaveToIni(Ini: TCustomInifile; ASection: String);

Var
  O : Integer;
  T : PTypeInfo;
  
begin
  With Ini do
    begin
    WriteString(ASection,KeyExpression,Expression);
    WriteString(ASection,KeyFields,Fields);
    WriteString(ASection,KeyCaseInsFields,CaseInsFields);
    WriteString(ASection,KeyDescFields,DescFields);
    WriteString(ASection,KeySource,Source);
    O:=Integer(Options);
    T:=TypeInfo(TIndexOptions);
    WriteString(ASection,KeyOptions,SetToString(T,O,False));
    end;
end;

procedure TDDIndexDef.LoadFromIni(Ini: TCustomInifile; ASection: String);

Var
  O : Integer;
  OP : TIndexOptions;
  T : PTypeInfo;
  S : String;

begin
  With Ini do
    begin
    Expression:=ReadString(ASection,KeyExpression,'');
    Fields:=ReadString(ASection,KeyFields,'');
    CaseInsFields:=ReadString(ASection,KeyCaseInsFields,'');
    DescFields:=ReadString(ASection,KeyDescFields,'');
    Source:=ReadString(ASection,KeySource,'');
    S:=ReadString(ASection,KeyOptions,'');
    T:=TypeInfo(TIndexOptions);
    O:=StringToSet(T,S);
    OP:=TIndexOptions(O);
    Options:=OP;
    end;
end;

{ TDDIndexDefs }

function TDDIndexDefs.GetIndex(Index : Integer): TDDIndexDef;
begin
  Result:=Items[Index] as TDDIndexDef;
end;

procedure TDDIndexDefs.SetIndex(Index : Integer; const AValue: TDDIndexDef);
begin
  Items[Index]:=AValue;
end;

procedure TDDIndexDefs.SetTableName(const AValue: String);
begin
  Inherited;
  FSectionPrefix:=AValue;
  GlobalSection:=AValue+SIndexSuffix;
end;

constructor TDDIndexDefs.Create(ATableDef: TDDTableDef);
begin
  FTableDef:=ATableDef;
  If Assigned(FTableDef) then
    Create(FTableDef.TableName)
  else
    Create('')
end;

constructor TDDIndexDefs.Create(ATableName: String);
begin
  FPrefix:='Index';
  TableName:=ATableName;
  Inherited Create(TDDIndexDef);
end;

function TDDIndexDefs.AddDDIndexDef(AName: String): TDDIndexDef;
begin
  result := AddIndex (AName);
end;

function TDDIndexDefs.AddIndex(AName: String): TDDIndexDef;
begin
  Result:=Add as TDDIndexDef;
  Result.IndexName:=AName;
end;

{ TDDForeignKeyDef }

procedure TDDForeignKeyDef.SetKeyName(const AValue: String);
begin
  if FKeyName=AValue then exit;

  FKeyName:=AValue;
end;

function TDDForeignKeyDef.GetSectionName: String;
begin
  Result:=FKeyName;
end;

procedure TDDForeignKeyDef.SetSectionName(const Value: String);
begin
  FkeyName:=Value;
end;

procedure TDDForeignKeyDef.Assign(ASource: TPersistent);

Var
  K : TDDForeignKeyDef;

begin
  if ASource is TDDForeignKeyDef then
    begin
    K:=ASource as TDDForeignKeyDef;
    FKeyFields:=K.KeyFields;
    FKeyName:=K.KeyName;
    FReferencedFields:=K.ReferencedFields;
    FTableName:=K.FTableName;
    end
  else
    inherited Assign(ASource);
end;

procedure TDDForeignKeyDef.SaveToIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini Do
    begin
    WriteString(ASection,KeyKeyFields,KeyFields);
    WriteString(ASection,KeyKeyName,KeyName);
    WriteString(ASection,KeyReferencesTable,ReferencesTable);
    WriteString(ASection,KeyReferencedFields,ReferencedFields);
    end;
end;

procedure TDDForeignKeyDef.LoadFromIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini Do
    begin
    KeyFields:=ReadString(ASection,KeyKeyFields,'');
    KeyName:=ReadString(ASection,KeyKeyName,'');
    ReferencesTable:=ReadString(ASection,KeyReferencesTable,'');
    ReferencedFields:=ReadString(ASection,KeyReferencedFields,'');
    end;
end;

{ TDDForeignKeyDefs }

function TDDForeignKeyDefs.GetKey(AIndex : Integer): TDDForeignKeyDef;
begin
  Result:=TDDForeignKeyDef(Items[AIndex]);
end;

procedure TDDForeignKeyDefs.SetKey(AIndex : Integer; const AValue: TDDForeignKeyDef
  );
begin
  Items[AIndex]:=AValue
end;

procedure TDDForeignKeyDefs.SetTableName(const AValue: String);
begin
  if FTableName=AValue then exit;
  FSectionPrefix:=AValue;
  GlobalSection:=AValue+SKeySuffix;
end;

constructor TDDForeignKeyDefs.Create(ATableName: String);
begin
  Inherited Create(TDDForeignKeyDef);
  FPrefix:='Key';
  SetTableName(ATAbleName);
end;

function TDDForeignKeyDefs.AddForeignKeyDef(AName: String): TDDForeignKeyDef;
begin
  Result:=Add as TDDForeignKeyDef;
  Result.KeyName:=AName;
end;

function TDDIndexDefs.IndexOfIndex(AIndexName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetIndex(Result).IndexName,AIndexName)<>0) do
    Dec(Result)
end;

function TDDIndexDefs.FindIndex(AIndexName: String): TDDIndexDef;
Var
  I : integer;
begin
  I:=IndexOfIndex(AIndexName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetIndex(I);
end;

function TDDIndexDefs.IndexByName(AIndexName: String): TDDIndexDef;
begin
  Result:=FindIndex(AIndexName);
  If Result=Nil then
    Raise EDatadict.CreateFmt(SErrIndexNotFound,[TableName,AIndexName]);
end;

{ TDDDomainDefs }

function TDDDomainDefs.GetDomain(Index: Integer): TDDDomainDef;
begin
  Result:=TDDDomainDef(Items[Index]);
end;

procedure TDDDomainDefs.SetDomain(Index: Integer;
  const AValue: TDDDomainDef);
begin
  Items[Index]:=AValue;
end;

constructor TDDDomainDefs.Create;
begin
  FPrefix:='Domain';
  FSectionPrefix:='Domain';
  GlobalSection:='Domains';
  inherited Create(TDDDomainDef);
end;


function TDDDomainDefs.AddDomain(ADomainName: String): TDDDomainDef;
begin
  Result:=Add as TDDDomainDef;
  Result.DomainName:=ADomainName;
end;

function TDDDomainDefs.IndexOfDomain(ADomainName: String): Integer;

begin
  Result := Count;
  repeat
    Dec(Result);
  until (Result < 0) or (CompareText(GetDomain(Result).DomainName,ADomainName) = 0);
end;

function TDDDomainDefs.FindDomain(ADomainName: String): TDDDomainDef;

Var
  I : Integer;

begin
  I:=IndexOfDomain(ADomainName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetDomain(I);
end;

function TDDDomainDefs.DomainByName(ADomainName: String): TDDDomainDef;
begin
  Result:=FindDomain(ADomainName);
  If (Result=Nil) then
    Raise EDatadict.CreateFmt(SErrDomainNotFound,[ADomainName]);
end;

{ TDDDomainDef }

procedure TDDDomainDef.SetDomainName(const AValue: String);
begin
  if FDomainName=AValue then exit;
  If Assigned(Collection) and
     ((Collection as TDDDomainDefs).FindDomain(AValue)<>Nil) then
     EDataDict.CreateFmt(SErrDuplicateDomain,[AValue]);
  FDomainName:=AValue;
end;

function TDDDomainDef.GetSectionName: String;
begin
  Result:=FDomainName;
end;

procedure TDDDomainDef.SetSectionName(const Value: String);
begin
  FDomainName:=Value;
end;

procedure TDDDomainDef.Assign(ASource: TPersistent);

Var
  D : TDDDomainDef;

begin
  if (ASource is TDDDomainDef) then
    begin
    D:=(ASource as TDDDomainDef);
    FDomainName:=D.DomainName;
    FFieldType:=D.FieldType;
    FCheckconstraint:=D.Checkconstraint;
    FSize:=D.Size;
    FPrecision:=D.Precision;
    end
  else
    inherited Assign(ASource);
end;

procedure TDDDomainDef.SaveToIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini do
    begin
    WriteInteger(ASection,KeyFieldType,Ord(Fieldtype));
    WriteBool(ASection,KeyRequired,Required);
    WriteString(ASection,KeyCheckConstraint,CheckConstraint);
    WriteInteger(ASection,KeySize,Size);
    WriteInteger(ASection,KeyPrecision,Precision);
    end;
end;

procedure TDDDomainDef.LoadFromIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini do
    begin
    FieldType:=TFieldType(ReadInteger(ASection,KeyFieldType,Ord(Fieldtype)));
    Required:=ReadBool(ASection,KeyRequired,Required);
    CheckConstraint:=ReadString(ASection,KeyCheckConstraint,CheckConstraint);
    Size:=ReadInteger(ASection,KeySize,Size);
    Precision:=ReadInteger(ASection,KeyPrecision,Precision);
    end;
end;

{ TFPDDDomainList }

function TFPDDDomainList.GetDomainDef(AIndex: Integer): TDDDomainDef;
begin
  Result:=TDDDomainDef(Items[AIndex]);
end;

procedure TFPDDDomainList.SetDomainDef(AIndex: Integer;
  const AValue: TDDDomainDef);
begin
  Items[AIndex]:=AValue;
end;

constructor TFPDDDomainList.CreateFromDomainDefs(DD: TDDDomainDefs);

Var
  I : Integer;

begin
  Inherited Create;
  For I:=0 to DD.Count-1 do
    Add(DD[I]);
end;

{ TDDSequenceDef }

procedure TDDSequenceDef.SetSequenceName(const AValue: String);
begin
  if FSequenceName=AValue then exit;
  If Assigned(Collection) and
     ((Collection as TDDSequenceDefs).FindSequence(AValue)<>Nil) then
     EDataDict.CreateFmt(SErrDuplicateSequence,[AValue]);
  FSequenceName:=AValue;
end;

function TDDSequenceDef.GetSectionName: String;
begin
  Result:=SequenceName;
end;

procedure TDDSequenceDef.SetSectionName(const Value: String);
begin
  SequenceName:=Value;
end;

procedure TDDSequenceDef.Assign(ASource: TPersistent);

Var
  S : TDDSequenceDef;

begin
  If ASource is TDDSequenceDef then
    begin
    S:=ASource as TDDSequenceDef;
    FSequenceName:=S.SequenceName;
    FStartvalue:=S.Startvalue;
    FIncrement:=S.Increment;
    end
  else
    inherited Assign(ASource);
end;

procedure TDDSequenceDef.SaveToIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini do
    begin
    WriteInteger(ASection,KeyStartValue,StartValue);
    WriteInteger(ASection,KeyIncrement,StartValue);
    end;
end;

procedure TDDSequenceDef.LoadFromIni(Ini: TCustomInifile; ASection: String);
begin
  With Ini do
    begin
    StartValue:=ReadInteger(ASection,KeyStartValue,0);
    Increment:=ReadInteger(ASection,KeyIncrement,0);
    end;
end;

{ TDDSequenceDefs }

function TDDSequenceDefs.GetSequence(Index: Integer): TDDSequenceDef;
begin
  Result:=TDDSequenceDef(Items[Index]);
end;

procedure TDDSequenceDefs.SetSequence(Index: Integer; const AValue: TDDSequenceDef);
begin
  Items[Index]:=AValue;
end;

constructor TDDSequenceDefs.Create;
begin
  FPrefix:='Sequence';
  FSectionPrefix:='Sequence';
  GlobalSection:='Sequences';
  Inherited Create(TDDSequenceDef);
end;

function TDDSequenceDefs.AddSequence(ASequenceName: String): TDDSequenceDef;
begin
  Result:=Add as TDDSequenceDef;
  Result.SequenceName:=ASequenceName;
end;

function TDDSequenceDefs.IndexOfSequence(ASequenceName: String): Integer;
begin
  result := count;
  repeat
    Dec(Result);
  until (Result<0) or (CompareText(GetSequence(Result).SequenceName,ASequenceName)=0);
end;

function TDDSequenceDefs.FindSequence(ASequenceName: String): TDDSequenceDef;

Var
  I : Integer;

begin
  I:=IndexOfSequence(ASequenceName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetSequence(I);
end;

function TDDSequenceDefs.SequenceByName(ASequenceName: String): TDDSequenceDef;
begin
  Result:=FindSequence(ASequenceName);
  If (Result=Nil) then
    Raise EDatadict.CreateFmt(SErrSequenceNotFound,[ASequenceName]);
end;


{ TFPDDSequenceList }

function TFPDDSequenceList.GetSequenceDef(AIndex: Integer): TDDSequenceDef;
begin
  Result:=TDDSequenceDef(Items[AIndex]);
end;

procedure TFPDDSequenceList.SetSequenceDef(AIndex: Integer;
  const AValue: TDDSequenceDef);
begin
  Items[AIndex]:=AValue;
end;

constructor TFPDDSequenceList.CreateFromSequenceDefs(SD: TDDSequenceDefs);

Var
  I : Integer;

begin
  Inherited Create;
  For I:=0 to SD.Count-1 do
    Add(SD[I]);
end;


{ TDDTableCollection }

function TDDTableCollection.GetTableName: String;
begin
  If Assigned(FTableDef) then
    Result:=FTableDef.TableName
  else
    Result:=FTableName;
end;

procedure TDDTableCollection.SetTableDef(ATableDef: TDDTableDef);
begin
  FTableDef:=ATableDef;
  If Assigned(FTableDef) then
    TableName:=FTableDef.TableName;
end;

procedure TDDTableCollection.SetTableName(const AValue: String);
begin
  FTableName:=AValue;
end;


function TDDTableCollection.DataDictionary: TFPDataDictionary;
begin
  If Assigned(FTableDef) then
    Result:=FTableDef.DataDictionary
  else
    Result:=Nil;
end;

initialization

finalization
  if assigned(DDEngines) then FreeAndNil(DDEngines);
end.

