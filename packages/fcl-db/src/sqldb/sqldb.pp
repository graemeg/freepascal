{
    Copyright (c) 2004-2014 by Joost van der Sluis, FPC contributors

    SQL database & dataset

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit SQLDB;

{$mode objfpc}
{$H+}
{$M+}   // ### remove this!!!

interface

uses SysUtils, Classes, DB, bufdataset, sqlscript, sqltypes;

type
  TSchemaType = sqltypes.TSchemaType;
  TStatementType = sqltypes.TStatementType; 
  TDBEventType = sqltypes.TDBEventType; 
  TDBEventTypes = sqltypes.TDBEventTypes;
  TQuoteChars = sqltypes.TQuoteChars;

const
  StatementTokens : Array[TStatementType] of string = ('(unknown)', 'select',
                  'insert', 'update', 'delete',
                  'create', 'get', 'put', 'execute',
                  'start','commit','rollback', '?'
                 );
  TSchemaObjectNames: array[TSchemaType] of String = ('???', 'table_name',
      '???', 'procedure_name', 'column_name', 'param_name',
      'index_name', 'package_name', 'schema_name','sequence');
  SingleQuotes : TQuoteChars = ('''','''');
  DoubleQuotes : TQuoteChars = ('"','"');
  LogAllEvents      = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack];
  LogAllEventsExtra = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack, detParamValue,detActualSQL];

  // Backwards compatibility alias constants.
  
  stNoSchema         = sqltypes.stNoSchema;
  stTables           = sqltypes.stTables;
  stSysTables        = sqltypes.stSysTables;
  stProcedures       = sqltypes.stProcedures;
  stColumns          = sqltypes.stColumns;
  stProcedureParams  = sqltypes.stProcedureParams;
  stIndexes          = sqltypes.stIndexes;
  stPackages         = sqltypes.stPackages;
  stSchemata         = sqltypes.stSchemata;
  stSequences        = sqltypes.stSequences;

  stUnknown       = sqltypes.stUnknown; 
  stSelect        = sqltypes.stSelect; 
  stInsert        = sqltypes.stInsert; 
  stUpdate        = sqltypes.stUpdate; 
  stDelete        = sqltypes.stDelete;
  stDDL           = sqltypes.stDDL; 
  stGetSegment    = sqltypes.stGetSegment; 
  stPutSegment    = sqltypes.stPutSegment; 
  stExecProcedure = sqltypes.stExecProcedure;
  stStartTrans    = sqltypes.stStartTrans; 
  stCommit        = sqltypes.stCommit; 
  stRollback      = sqltypes.stRollback;  
  stSelectForUpd  = sqltypes.stSelectForUpd;

  detCustom      = sqltypes.detCustom; 
  detPrepare     = sqltypes.detPrepare; 
  detExecute     = sqltypes.detExecute; 
  detFetch       = sqltypes.detFetch; 
  detCommit      = sqltypes.detCommit; 
  detRollBack    = sqltypes.detRollBack; 
  detParamValue  = sqltypes.detParamValue; 
  detActualSQL   = sqltypes.detActualSQL;
  DefaultMacroChar     = '%';
Type
  TRowsCount = LargeInt;

  TSQLStatementInfo = Record
    StatementType : TStatementType;
    TableName : String;
    Updateable : Boolean;
    WhereStartPos ,
    WhereStopPos : integer;
  end;

  TSQLConnection = class;
  TSQLTransaction = class;
  TCustomSQLQuery = class;
  TCustomSQLStatement = Class;
  TSQLQuery = class;
  TSQLScript = class;

  TSQLHandle = Class(TObject)
  end;

  { TSQLCursor }

  TSQLCursor = Class(TSQLHandle)
  public
    FDirect        : Boolean;
    FPrepared      : Boolean;
    FSelectable    : Boolean;
    FInitFieldDef  : Boolean;
    FStatementType : TStatementType;
    FSchemaType    : TSchemaType;
  end;

  { ESQLDatabaseError}

  ESQLDatabaseError = class(EDatabaseError)
    public
      ErrorCode: integer;
      SQLState : string;
      constructor CreateFmt(const Fmt: string; const Args: array of const;
                            Comp : TComponent; AErrorCode: integer; ASQLState: string); overload;
  end;

  { TSQLDBFieldDef }

  TSQLDBFieldDef = Class(TFieldDef)
  private
    FData: Pointer;
  Public
    Property SQLDBData : Pointer Read FData Write FData;
  end;

  { TSQLDBFieldDefs }

  TSQLDBFieldDefs = Class(TFieldDefs)
  Protected
    Class Function FieldDefClass : TFieldDefClass; override;
  end;

  { TSQLDBParam }

  TSQLDBParam = Class(TParam)
  private
    FFieldDef: TFieldDef;
    FData : Pointer;
  Public
    Property FieldDef : TFieldDef Read FFieldDef Write FFieldDef;
    Property SQLDBData : Pointer Read FData Write FData;
  end;

  { TSQLDBParams }

  TSQLDBParams = Class(TParams)
  Protected
    Class Function ParamClass : TParamClass; override;
  end;


type

  { TServerIndexDefs }

  TServerIndexDefs = class(TIndexDefs)
  Private
  public
    constructor Create(ADataSet: TDataSet); override;
    procedure Update; override;
  end;

type

  { TSQLConnection }
  
  TDBLogNotifyEvent = Procedure (Sender : TSQLConnection; EventType : TDBEventType; Const Msg : String) of object;

  TConnOption = (sqSupportParams, sqSupportEmptyDatabaseName, sqEscapeSlash, sqEscapeRepeat, sqImplicitTransaction, sqLastInsertID, sqSupportReturning,sqSequences);
  TConnOptions= set of TConnOption;

  TSQLConnectionOption = (scoExplicitConnect, scoApplyUpdatesChecksRowsAffected);
  TSQLConnectionOptions = Set of TSQLConnectionOption;

  TConnInfoType=(citAll=-1, citServerType, citServerVersion, citServerVersionString, citClientName, citClientVersion);

  TSQLConnection = class (TDatabase)
  private
    FFieldNameQuoteChars : TQuoteChars;
    FOptions             : TSQLConnectionOptions;
    FPassword            : string;
    FTransaction         : TSQLTransaction;
    FUserName            : string;
    FHostName            : string;
    FCharSet             : string;
    FCodePage            : TSystemCodePage;
    FRole                : String;
    FStatements          : TThreadList;
    FLogEvents: TDBEventTypes;
    FOnLog: TDBLogNotifyEvent;
    function GetPort: cardinal;
    procedure SetOptions(AValue: TSQLConnectionOptions);
    procedure SetPort(const AValue: cardinal);
    function AttemptCommit(trans : TSQLHandle) : boolean; 
    function AttemptRollBack(trans : TSQLHandle) : boolean; 
  protected
    FConnOptions         : TConnOptions;
    FSQLFormatSettings   : TFormatSettings;

    // Updating of DB records is moved out of TSQLQuery.
    // It is done here, so descendents can override it and implement DB-specific.
    // One day, this may be factored out to a TSQLResolver class.
    // The following allow construction of update queries. They can be adapted as needed by descendents to fit the DB engine.
    procedure AddFieldToUpdateWherePart(var sql_where: string; UpdateMode : TUpdateMode; F: TField); virtual;
    function ConstructInsertSQL(Query: TCustomSQLQuery; Var ReturningClause : Boolean): string; virtual;
    function ConstructUpdateSQL(Query: TCustomSQLQuery; Var ReturningClause : Boolean): string; virtual;
    function ConstructDeleteSQL(Query: TCustomSQLQuery): string; virtual;
    function ConstructRefreshSQL(Query: TCustomSQLQuery; UpdateKind : TUpdateKind): string; virtual;
    // factory function used to create custom statements
    function CreateCustomQuery(aOwner: TComponent): TCustomSQLQuery; virtual;
    function InitialiseUpdateStatement(Query: TCustomSQLQuery; var qry: TCustomSQLQuery): TCustomSQLQuery;
    procedure ApplyFieldUpdate(C : TSQLCursor; P: TSQLDBParam; F: TField; UseOldValue: Boolean); virtual;
    // This is the call that updates a record, it used to be in TSQLQuery.
    procedure ApplyRecUpdate(Query : TCustomSQLQuery; UpdateKind : TUpdateKind); virtual;
    function RefreshLastInsertID(Query : TCustomSQLQuery; Field : TField): Boolean; virtual;
    procedure GetDBInfo(const ASchemaType : TSchemaType; const ASchemaObjectName, AReturnField : string; AList: TStrings);
    function GetConnectionCharSet: string; virtual;
    procedure SetTransaction(Value : TSQLTransaction); virtual;
    procedure DoConnect; override;
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    function GetAsString(Param: TParam): RawByteString;
    function GetAsSQLText(Field : TField) : string; overload; virtual;
    function GetAsSQLText(Param : TParam) : string; overload; virtual;
    function GetHandle : pointer; virtual;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure LogParams(Const AParams : TParams); virtual;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
    Procedure RegisterStatement(S : TCustomSQLStatement);
    Procedure UnRegisterStatement(S : TCustomSQLStatement);

    Function AllocateCursorHandle : TSQLCursor; virtual; abstract;
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); virtual; abstract;
    function StrToStatementType(s : string) : TStatementType; virtual;
    procedure PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams); virtual; abstract;
    procedure UnPrepareStatement(cursor : TSQLCursor); virtual; abstract;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); virtual; abstract;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; virtual;
    function Fetch(cursor : TSQLCursor) : boolean; virtual; abstract;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TFieldDefs); virtual; abstract;
    function AddFieldDef(AFieldDefs: TFieldDefs; AFieldNo: Longint; const AName: string; ADataType: TFieldType; ASize, APrecision: Integer; AByteSize, ARequired, AReadOnly: Boolean): TFieldDef;
    function LoadField(cursor : TSQLCursor; FieldDef : TFieldDef; buffer : pointer; out CreateBlob : boolean) : boolean; virtual; abstract;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); virtual; abstract;
    procedure FreeFldBuffers(cursor : TSQLCursor); virtual;

    Function AllocateTransactionHandle : TSQLHandle; virtual; abstract;
    function GetTransactionHandle(trans : TSQLHandle): pointer; virtual; abstract;
    function Commit(trans : TSQLHandle) : boolean; virtual; abstract;
    function RollBack(trans : TSQLHandle) : boolean; virtual; abstract;
    function StartImplicitTransaction(trans : TSQLHandle; aParams : string) : boolean; virtual;
    function StartDBTransaction(trans : TSQLHandle; aParams : string) : boolean; virtual; abstract;
    procedure CommitRetaining(trans : TSQLHandle); virtual; abstract;
    procedure RollBackRetaining(trans : TSQLHandle); virtual; abstract;

    procedure UpdateIndexDefs(IndexDefs : TIndexDefs; TableName : string); virtual;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; virtual;
    function GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string; virtual;

    Procedure MaybeConnect;

    Property Statements : TThreadList Read FStatements;
    property Port: cardinal read GetPort write SetPort;
    property CodePage: TSystemCodePage read FCodePage;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure StartTransaction; override;
    procedure EndTransaction; override;
    procedure ExecuteDirect(SQL : String); overload; virtual;
    procedure ExecuteDirect(SQL : String; ATransaction : TSQLTransaction); overload; virtual;
    // Unified version
    function GetObjectNames(ASchemaType: TSchemaType; AList : TSqlObjectIdentifierList): Integer; virtual;
    // Older versions.
    procedure GetTableNames(List : TStrings; SystemTables : Boolean = false); virtual;
    procedure GetProcedureNames(List : TStrings); virtual;
    procedure GetFieldNames(const TableName : string; List : TStrings); virtual;
    procedure GetSchemaNames(List: TStrings); virtual;
    procedure GetSequenceNames(List: TStrings); virtual;
    function GetConnectionInfo(InfoType:TConnInfoType): string; virtual;
    function GetStatementInfo(const ASQL: string): TSQLStatementInfo; virtual;
    procedure CreateDB; virtual;
    procedure DropDB; virtual;
    function GetNextValue(const SequenceName: string; IncrementBy: integer=1): Int64; virtual;
    property ConnOptions: TConnOptions read FConnOptions;
    property Handle: Pointer read GetHandle;
    property FieldNameQuoteChars: TQuoteChars read FFieldNameQuoteChars write FFieldNameQuoteChars;
  published
    property Password : string read FPassword write FPassword;
    property Transaction : TSQLTransaction read FTransaction write SetTransaction;
    property UserName : string read FUserName write FUserName;
    property CharSet : string read FCharSet write FCharSet;
    property HostName : string Read FHostName Write FHostName;
    Property OnLog : TDBLogNotifyEvent Read FOnLog Write FOnLog;
    Property LogEvents : TDBEventTypes Read FLogEvents Write FLogEvents Default LogAllEvents;
    Property Options : TSQLConnectionOptions Read FOptions Write SetOptions default [];
    Property Role :  String read FRole write FRole;
    property Connected;
    property DatabaseName;
    property KeepConnection;
    property LoginPrompt;
    property Params;
    property OnLogin;
  end;

  { TSQLTransaction }

  TCommitRollbackAction = (caNone, caCommit, caCommitRetaining, caRollback,
    caRollbackRetaining);

  TSQLTransactionOption = (stoUseImplicit, stoExplicitStart);
  TSQLTransactionOptions = Set of TSQLTransactionOption;

  TSQLTransaction = class (TDBTransaction)
  private
    FOptions             : TSQLTransactionOptions;
    FTrans               : TSQLHandle;
    FAction              : TCommitRollbackAction;
    FParams              : TStringList;
    function GetSQLConnection: TSQLConnection;
    procedure SetOptions(AValue: TSQLTransactionOptions);
    procedure SetParams(const AValue: TStringList);
    procedure SetSQLConnection(AValue: TSQLConnection);
  protected
    Procedure MaybeStartTransaction;
    Function AllowClose(DS: TDBDataset): Boolean; override;
    function GetHandle : Pointer; virtual;
    Procedure SetDatabase (Value : TDatabase); override;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
  public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure Commit; override;
    procedure CommitRetaining; override;
    procedure Rollback; override;
    procedure RollbackRetaining; override;
    procedure StartTransaction; override;
    procedure EndTransaction; override;
    property Handle: Pointer read GetHandle;
    Property SQLConnection : TSQLConnection Read GetSQLConnection Write SetSQLConnection;
  published
    property Action : TCommitRollbackAction read FAction write FAction Default caRollBack;
    property Database;
    property Params : TStringList read FParams write SetParams;
    Property Options : TSQLTransactionOptions Read FOptions Write SetOptions default [];
  end;



  { TCustomSQLStatement }

  TCustomSQLStatement = Class(TComponent)
  Private
    FCursor : TSQLCursor;
    FDatabase: TSQLConnection;
    FParamCheck: Boolean;
    FParams: TParams;
    FMacroCheck: Boolean;
    FMacroChar: Char;
    FMacros: TParams;
    FSQL: TStrings;
    FOrigSQL : String;
    FServerSQL : String;
    FTransaction: TSQLTransaction;
    FParseSQL: Boolean;
    FDoUnPrepare : Boolean;
    FDataLink : TDataLink;
    FRowsAffected : TRowsCount;
    function ExpandMacros( OrigSQL: String): String;
    procedure SetDatabase(AValue: TSQLConnection);
    procedure SetMacroChar(AValue: Char);
    procedure SetMacroCheck(AValue: Boolean);
    procedure SetParams(AValue: TParams);
    procedure SetMacros(AValue: TParams);
    procedure SetSQL(AValue: TStrings);
    procedure SetTransaction(AValue: TSQLTransaction);
    procedure RecreateMacros;
    Function GetPrepared : Boolean;
    Procedure CheckUnprepare;
    Procedure CheckPrepare;
  Protected
    Function CreateDataLink : TDataLink; virtual;
    procedure OnChangeSQL(Sender : TObject); virtual;
    function GetDataSource: TDataSource; Virtual;
    procedure SetDataSource(AValue: TDataSource); virtual;
    Procedure CopyParamsFromMaster(CopyBound : Boolean); virtual;
    procedure AllocateCursor;
    procedure DeAllocateCursor;
    Function GetSchemaType : TSchemaType; virtual;
    Function GetSchemaObjectName : String; virtual;
    Function GetSchemaPattern: String; virtual;
    Function IsSelectable : Boolean ; virtual;
    procedure GetStatementInfo(var ASQL: String; out Info: TSQLStatementInfo); virtual;
    Procedure DoExecute; virtual;
    procedure DoPrepare; virtual;
    procedure DoUnPrepare; virtual;
    Function CreateParams : TSQLDBParams; virtual;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    Property Cursor : TSQLCursor read FCursor;
    Property Database : TSQLConnection Read FDatabase Write SetDatabase;
    Property Transaction : TSQLTransaction Read FTransaction Write SetTransaction;
    Property SQL : TStrings Read FSQL Write SetSQL;
    Property Params : TParams Read FParams Write SetParams;
    Property Macros : TParams Read FMacros Write SetMacros;
    property MacroChar: Char read FMacroChar write SetMacroChar default DefaultMacroChar;
    Property DataSource : TDataSource Read GetDataSource Write SetDataSource;
    Property ParseSQL : Boolean Read FParseSQL Write FParseSQL;
    Property ParamCheck : Boolean Read FParamCheck Write FParamCheck default true;
    Property MacroCheck : Boolean Read FMacroCheck Write SetMacroCheck default false;
  Public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    Procedure Prepare;
    Procedure Execute;
    Procedure Unprepare;
    function ParamByName(Const AParamName : String) : TParam;
    function RowsAffected: TRowsCount; virtual;
    Property Prepared : boolean read GetPrepared;
  end;

  TSQLStatement = Class(TCustomSQLStatement)
  Published
    Property Database;
    Property DataSource;
    Property ParamCheck;
    Property Params;
    Property MacroCheck;
    Property Macros;
    Property ParseSQL;
    Property SQL;
    Property Transaction;
  end;


  { TSQLSequence }

  TSQLSequenceApplyEvent = (saeOnNewRecord, saeOnPost);

  TSQLSequence = class(TPersistent)
  private
    FQuery: TCustomSQLQuery;
    FFieldName: String;
    FSequenceName: String;
    FIncrementBy: Integer;
    FApplyEvent: TSQLSequenceApplyEvent;
  public
    constructor Create(AQuery: TCustomSQLQuery);
    procedure Assign(Source: TPersistent); override;
    procedure Apply;
    function GetNextValue: Int64;
  published
    property FieldName: String read FFieldName write FFieldName;
    property SequenceName: String read FSequenceName write FSequenceName;
    property IncrementBy: Integer read FIncrementBy write FIncrementBy default 1;
    property ApplyEvent: TSQLSequenceApplyEvent read FApplyEvent write FApplyEvent default saeOnNewRecord;
  end;


  { TCustomSQLQuery }

  TSQLQueryOption = (sqoKeepOpenOnCommit, sqoAutoApplyUpdates, sqoAutoCommit, sqoCancelUpdatesOnRefresh, sqoRefreshUsingSelect);
  TSQLQueryOptions = Set of TSQLQueryOption;

  TCustomSQLQuery = class (TCustomBufDataset)
  private
    FOptions             : TSQLQueryOptions;
    FSchemaType          : TSchemaType;
    FUpdateable          : boolean;
    FTableName           : string;
    FStatement           : TCustomSQLStatement;
    FInsertSQL,
    FUpdateSQL,
    FDeleteSQL,
    FRefreshSQL          : TStringList;
    FIsEOF               : boolean;
    FLoadingFieldDefs    : boolean;
    FUpdateMode          : TUpdateMode;
    FDoUnprepare : Boolean;
    FusePrimaryKeyAsKey  : Boolean;
    FWhereStartPos       : integer;
    FWhereStopPos        : integer;
    FServerFilterText    : string;
    FServerFiltered      : Boolean;
    FServerIndexDefs     : TServerIndexDefs;

    // Used by SetSchemaType
    FSchemaObjectName    : string;
    FSchemaPattern       : string;

    FInsertQry,
    FUpdateQry,
    FDeleteQry           : TCustomSQLQuery;
    FSequence            : TSQLSequence;
    procedure CheckPrepare;
    procedure CheckUnPrepare;
    procedure FreeFldBuffers;
    function GetMacroChar: Char;
    function GetParamCheck: Boolean;
    function GetParams: TParams;
    function GetMacroCheck: Boolean;
    function GetMacros: TParams;
    function GetParseSQL: Boolean;
    function GetServerIndexDefs: TServerIndexDefs;
    function GetSQL: TStringList;
    function GetSQLConnection: TSQLConnection;
    function GetSQLTransaction: TSQLTransaction;
    function GetStatementType : TStatementType;
    Function NeedLastInsertID: TField;
    procedure SetMacroChar(AValue: Char);
    procedure SetOptions(AValue: TSQLQueryOptions);
    procedure SetParamCheck(AValue: Boolean);
    procedure SetMacroCheck(AValue: Boolean);
    procedure SetSQLConnection(AValue: TSQLConnection);
    procedure SetSQLTransaction(AValue: TSQLTransaction);
    procedure SetInsertSQL(const AValue: TStringList);
    procedure SetUpdateSQL(const AValue: TStringList);
    procedure SetDeleteSQL(const AValue: TStringList);
    procedure SetRefreshSQL(const AValue: TStringList);
    procedure SetParams(AValue: TParams);
    procedure SetMacros(AValue: TParams);
    procedure SetParseSQL(AValue : Boolean);
    procedure SetSQL(const AValue: TStringList);
    procedure SetUsePrimaryKeyAsKey(AValue : Boolean);
    procedure SetUpdateMode(AValue : TUpdateMode);
    procedure OnChangeModifySQL(Sender : TObject);
    procedure Execute;
    procedure ApplyFilter;
    Function AddFilter(SQLstr : string) : string;
  protected
    procedure OpenCursor(InfoQuery: Boolean); override;
    function CreateSQLStatement(aOwner: TComponent): TCustomSQLStatement; virtual;
    Function CreateParams: TSQLDBParams; virtual;
    Function RefreshLastInsertID(Field: TField): Boolean; virtual;
    Function NeedRefreshRecord (UpdateKind: TUpdateKind): Boolean; virtual;
    Function RefreshRecord (UpdateKind: TUpdateKind) : Boolean; virtual;
    Procedure ApplyReturningResult(Q : TCustomSQLQuery; UpdateKind : TUpdateKind);
    Function Cursor : TSQLCursor;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
    // abstract & virtual methods of TBufDataset
    function Fetch : boolean; override;
    function LoadField(FieldDef : TFieldDef; buffer : pointer; out CreateBlob : boolean) : boolean; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField); override;
    procedure ApplyRecUpdate(UpdateKind : TUpdateKind); override;
    procedure SetPacketRecords(aValue : integer); override;
    // abstract & virtual methods of TDataset
    procedure UpdateServerIndexDefs; virtual;
    procedure SetDatabase(Value : TDatabase); override;
    Procedure SetTransaction(Value : TDBTransaction); override;
    procedure InternalAddRecord(Buffer: Pointer; AAppend: Boolean); override;
    procedure InternalClose; override;
    procedure InternalInitFieldDefs; override;
    procedure InternalOpen; override;
    Procedure InternalRefresh; override;
    function  GetCanModify: Boolean; override;
    Function IsPrepared : Boolean; virtual;
    procedure SetServerFiltered(Value: Boolean); virtual;
    procedure SetServerFilterText(const Value: string); virtual;
    Function GetDataSource : TDataSource; override;
    Procedure SetDataSource(AValue : TDataSource);
    procedure BeforeRefreshOpenCursor; override;
    procedure SetReadOnly(AValue : Boolean); override;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    procedure DoOnNewRecord; override;
    procedure DoBeforePost; override;
    class function FieldDefsClass : TFieldDefsClass; override;
    // IProviderSupport methods
    function PSGetUpdateException(E: Exception; Prev: EUpdateError): EUpdateError; override;
    function PSGetTableName: string; override;
    Property TableName : String Read FTableName Write FTableName; // alternative: procedure DoGetTableName
  public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure Prepare; virtual;
    procedure UnPrepare; virtual;
    procedure ExecSQL; virtual;
    procedure SetSchemaInfo( ASchemaType : TSchemaType; ASchemaObjectName, ASchemaPattern : string); virtual;
    function RowsAffected: TRowsCount; virtual;
    function ParamByName(Const AParamName : String) : TParam;
    function MacroByName(Const AParamName : String) : TParam;
    Property Prepared : boolean read IsPrepared;
    Property SQLConnection : TSQLConnection Read GetSQLConnection Write SetSQLConnection;
    Property SQLTransaction: TSQLTransaction Read GetSQLTransaction Write SetSQLTransaction;
    // overriden TBufDataSet methods
    Procedure ApplyUpdates(MaxErrors: Integer); override; overload;
    // overriden TDataSet methods
    Procedure Post; override;
    Procedure Delete; override;
  protected
    // redeclared TDataSet properties
    property Active;
    property Filter;
    property Filtered;
    property BeforeOpen;
    property AfterOpen;
    property BeforeClose;
    property AfterClose;
    property BeforeInsert;
    property AfterInsert;
    property BeforeEdit;
    property AfterEdit;
    property BeforePost;
    property AfterPost;
    property BeforeCancel;
    property AfterCancel;
    property BeforeDelete;
    property AfterDelete;
    property BeforeRefresh;
    property AfterRefresh;
    property BeforeScroll;
    property AfterScroll;
    property OnCalcFields;
    property OnDeleteError;
    property OnEditError;
    property OnFilterRecord;
    property OnNewRecord;
    property OnPostError;
    property AutoCalcFields;
    // protected
    property Database;
    property Transaction;
    property SchemaType : TSchemaType read FSchemaType default stNoSchema;
    property SQL : TStringlist read GetSQL write SetSQL;
    property InsertSQL : TStringList read FInsertSQL write SetInsertSQL;
    property UpdateSQL : TStringList read FUpdateSQL write SetUpdateSQL;
    property DeleteSQL : TStringList read FDeleteSQL write SetDeleteSQL;
    property RefreshSQL : TStringList read FRefreshSQL write SetRefreshSQL;
    Property Options : TSQLQueryOptions Read FOptions Write SetOptions default [];
    property Params : TParams read GetParams Write SetParams;
    Property ParamCheck : Boolean Read GetParamCheck Write SetParamCheck default true;
    property Macros : TParams read GetMacros Write SetMacros;
    Property MacroCheck : Boolean Read GetMacroCheck Write SetMacroCheck default false;
    Property MacroChar : Char Read GetMacroChar Write SetMacroChar default DefaultMacroChar;
    property ParseSQL : Boolean read GetParseSQL write SetParseSQL default true;
    property UpdateMode : TUpdateMode read FUpdateMode write SetUpdateMode default upWhereKeyOnly;
    property UsePrimaryKeyAsKey : boolean read FUsePrimaryKeyAsKey write SetUsePrimaryKeyAsKey default true;
    property StatementType : TStatementType read GetStatementType;
    Property DataSource : TDataSource Read GetDataSource Write SetDataSource;
    property Sequence: TSQLSequence read FSequence write FSequence;
    property ServerFilter: string read FServerFilterText write SetServerFilterText;
    property ServerFiltered: Boolean read FServerFiltered write SetServerFiltered default False;
    property ServerIndexDefs : TServerIndexDefs read GetServerIndexDefs;
  end;

{ TSQLQuery }
  TSQLQuery = Class(TCustomSQLQuery)
  public
    property SchemaType;
    Property StatementType;
  Published
    property MaxIndexesCount;
   // TDataset stuff
    property FieldDefs;
    Property Active;
    Property AutoCalcFields;
    Property Filter;
    Property Filtered;
    Property AfterCancel;
    Property AfterClose;
    Property AfterDelete;
    Property AfterEdit;
    Property AfterInsert;
    Property AfterOpen;
    Property AfterPost;
    Property AfterRefresh;
    Property AfterScroll;
    Property BeforeCancel;
    Property BeforeClose;
    Property BeforeDelete;
    Property BeforeEdit;
    Property BeforeInsert;
    Property BeforeOpen;
    Property BeforePost;
    Property BeforeRefresh;
    Property BeforeScroll;
    Property OnCalcFields;
    Property OnDeleteError;
    Property OnEditError;
    Property OnFilterRecord;
    Property OnNewRecord;
    Property OnPostError;

    //    property SchemaInfo default stNoSchema;
    property Database;
    property Transaction;
    property ReadOnly;
    property SQL;
    property InsertSQL;
    property UpdateSQL;
    property DeleteSQL;
    property RefreshSQL;
    property IndexDefs;
    Property Options;
    property Params;
    Property ParamCheck;
    property Macros;
    Property MacroCheck;
    Property MacroChar;
    property ParseSQL;
    property UpdateMode;
    property UsePrimaryKeyAsKey;
    Property DataSource;
    property Sequence;
    property ServerFilter;
    property ServerFiltered;
    property ServerIndexDefs;
  end;

{ TSQLScript }

  TSQLScript = class (TCustomSQLscript)
  private
    FOnDirective: TSQLScriptDirectiveEvent;
    FQuery   : TCustomSQLQuery;
    FDatabase : TDatabase;
    FTransaction : TDBTransaction;
  protected
    procedure ExecuteStatement (SQLStatement: TStrings; var StopExecution: Boolean); override;
    procedure ExecuteDirective (Directive, Argument: String; var StopExecution: Boolean); override;
    procedure ExecuteCommit(CommitRetaining: boolean=true); override;
    Procedure SetDatabase (Value : TDatabase); virtual;
    Procedure SetTransaction(Value : TDBTransaction); virtual;
    Procedure CheckDatabase;
    function CreateQuery: TCustomSQLQuery; virtual;
  public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure Execute; override;
    procedure ExecuteScript;
    Property Aborted;
    Property Line;
  published
    Property DataBase : TDatabase Read FDatabase Write SetDatabase;
    Property Transaction : TDBTransaction Read FTransaction Write SetTransaction;
    property OnDirective: TSQLScriptDirectiveEvent read FOnDirective write FOnDirective;
    Property AutoCommit;
    Property UseDollarString;
    Property DollarStrings;
    property Directives;
    property Defines;
    property Script;
    property Terminator;
    property CommentsinSQL;
    property UseSetTerm;
    property UseCommit;
    property UseDefines;
    property OnException;
  end;

  { TSQLConnector }

  TSQLConnector = Class(TSQLConnection)
  private
    FProxy : TSQLConnection;
    FConnectorType: String;
    procedure SetConnectorType(const AValue: String);
  protected
    procedure SetTransaction(Value : TSQLTransaction);override;
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    Procedure CheckProxy;
    Procedure CreateProxy; virtual;
    Procedure FreeProxy; virtual;
    function StrToStatementType(s : string) : TStatementType; override;
    function GetAsSQLText(Field : TField) : string; overload; override;
    function GetAsSQLText(Param : TParam) : string; overload; override;
    function GetHandle : pointer; override;

    Function AllocateCursorHandle : TSQLCursor; override;
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); override;
    Function AllocateTransactionHandle : TSQLHandle; override;

    procedure PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams); override;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function Fetch(cursor : TSQLCursor) : boolean; override;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TfieldDefs); override;
    procedure UnPrepareStatement(cursor : TSQLCursor); override;
    function LoadField(cursor : TSQLCursor; FieldDef : TFieldDef; buffer : pointer; out CreateBlob : boolean) : boolean; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    procedure FreeFldBuffers(cursor : TSQLCursor); override;
    function GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string; override;
    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function StartDBTransaction(trans : TSQLHandle; aParams : string) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    procedure RollBackRetaining(trans : TSQLHandle); override;
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs; TableName : string); override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    Property Proxy : TSQLConnection Read FProxy;
  Published
    Property ConnectorType : String Read FConnectorType Write SetConnectorType;
  end;

  TSQLConnectionClass = Class of TSQLConnection;

  { TConnectionDef }
  TLibraryLoadFunction = Function (Const S : AnsiString) : Integer;
  TLibraryUnLoadFunction = Procedure;
  TConnectionDef = Class(TPersistent)
    Class Function TypeName : String; virtual;
    Class Function ConnectionClass : TSQLConnectionClass; virtual;
    Class Function Description : String; virtual;
    Class Function DefaultLibraryName : String; virtual;
    Class Function LoadFunction : TLibraryLoadFunction; virtual;
    Class Function UnLoadFunction : TLibraryUnLoadFunction; virtual;
    Class Function LoadedLibraryName : string; virtual;
    Procedure ApplyParams(Params : TStrings; AConnection : TSQLConnection); virtual;
  end;
  TConnectionDefClass = class of TConnectionDef;

Var
  GlobalDBLogHook : TDBLogNotifyEvent;

Procedure RegisterConnection(Def : TConnectionDefClass);
Procedure UnRegisterConnection(Def : TConnectionDefClass);
Procedure UnRegisterConnection(ConnectionName : String);
Function GetConnectionDef(ConnectorName : String) : TConnectionDef;
Procedure GetConnectionList(List : TSTrings);

const DefaultSQLFormatSettings : TFormatSettings = (
  CurrencyFormat: 1;
  NegCurrFormat: 5;
  ThousandSeparator: #0;
  DecimalSeparator: '.';
  CurrencyDecimals: 2;
  DateSeparator: '-';
  TimeSeparator: ':';
  ListSeparator: ' ';
  CurrencyString: '$';
  ShortDateFormat: 'yyyy-mm-dd';
  LongDateFormat: '';
  TimeAMString: '';
  TimePMString: '';
  ShortTimeFormat: 'hh:nn:ss';
  LongTimeFormat: 'hh:nn:ss.zzz';
  ShortMonthNames: ('','','','','','','','','','','','');
  LongMonthNames: ('','','','','','','','','','','','');
  ShortDayNames: ('','','','','','','');
  LongDayNames: ('','','','','','','');
  TwoDigitYearCenturyWindow: 50;
);

implementation

uses dbconst, strutils;

Const
  // Flags to check which fields must be refreshed.
  RefreshFlags : Array [ukModify..ukInsert] of TProviderFlag = (pfRefreshOnUpdate,pfRefreshOnInsert);


function TimeIntervalToString(Time: TDateTime): string;
var
  millisecond: word;
  second     : word;
  minute     : word;
  hour       : word;
begin
  DecodeTime(Time,hour,minute,second,millisecond);
  hour := hour + trunc(Time)*24;
  Result := Format('%.2d:%.2d:%.2d.%.3d',[hour,minute,second,millisecond]);
end;


{ TSQLDBFieldDefs }

class function TSQLDBFieldDefs.FieldDefClass: TFieldDefClass;
begin
  Result:=TSQLDBFieldDef;
end;


{ TSQLDBParams }

class function TSQLDBParams.ParamClass: TParamClass;
begin
  Result:=TSQLDBParam;
end;


{ ESQLDatabaseError }

constructor ESQLDatabaseError.CreateFmt(const Fmt: string; const Args: array of const;
  Comp: TComponent; AErrorCode: integer; ASQLState: string);
const CompNameFmt='%s : %s';
var Msg: string;
begin
  if not assigned(Comp) then
    Msg := Fmt
  else if Comp.Name = '' then
    Msg := Format(CompNameFmt, [Comp.ClassName,Fmt])
  else
    Msg := Format(CompNameFmt, [Comp.Name,Fmt]);

  if Length(Args) = 0 then
    inherited Create(Msg)
  else
    inherited CreateFmt(Msg, Args);

  ErrorCode := AErrorCode;
  SQLState  := ASQLState;
end;


{ TCustomSQLStatement }

procedure TCustomSQLStatement.OnChangeSQL(Sender: TObject);

var
  ConnOptions : TConnOptions;
  NewParams: TSQLDBParams;

begin
  UnPrepare;
  RecreateMacros;
  if not ParamCheck then
    exit;
  if assigned(DataBase) then
    ConnOptions:=DataBase.ConnOptions
  else
    ConnOptions := [sqEscapeRepeat,sqEscapeSlash];
  NewParams := CreateParams;
  try
    NewParams.ParseSQL(FSQL.Text, True, sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions, psInterbase);
    NewParams.AssignValues(FParams);
    FParams.Assign(NewParams);
  finally
    NewParams.Free;
  end;
end;

procedure TCustomSQLStatement.SetDatabase(AValue: TSQLConnection);
begin
  if FDatabase=AValue then Exit;
  UnPrepare;
  If Assigned(FDatabase) then
    begin
    FDatabase.UnregisterStatement(Self);
    FDatabase.RemoveFreeNotification(Self);
    end;
  FDatabase:=AValue;
  If Assigned(FDatabase) then
    begin
    FDatabase.FreeNotification(Self);
    FDatabase.RegisterStatement(Self);
    if Assigned(Database.Transaction) and (not Assigned(Transaction) or (Transaction.DataBase <> Database)) then
      Transaction := Database.Transaction;
    OnChangeSQL(Self);
    end;
end;

procedure TCustomSQLStatement.SetMacroChar(AValue: Char);
begin
  if FMacroChar=AValue then Exit;
  FMacroChar:=AValue;
  RecreateMacros;
end;

procedure TCustomSQLStatement.SetMacroCheck(AValue: Boolean);
begin
  if FMacroCheck=AValue then Exit;
  FMacroCheck:=AValue;
  RecreateMacros;
end;

procedure TCustomSQLStatement.SetTransaction(AValue: TSQLTransaction);
begin
  if FTransaction=AValue then Exit;
  UnPrepare;
  if Assigned(FTransaction) then
    FTransaction.RemoveFreeNotification(Self);
  FTransaction:=AValue;
  if Assigned(FTransaction) then
    begin
    FTransaction.FreeNotification(Self);
    if Assigned(Transaction.DataBase) and (Database <> Transaction.DataBase) then
      Database := Transaction.DataBase as TSQLConnection;
    end;
end;

procedure TCustomSQLStatement.RecreateMacros;
var
  NewParams: TSQLDBParams;
  ConnOptions: TConnOptions;
  PO : TSQLParseOptions;
  PB : TParamBinding;
  RS : String;

begin
  if MacroCheck then begin
    if assigned(DataBase) then
      ConnOptions:=DataBase.ConnOptions
    else
      ConnOptions := [sqEscapeRepeat,sqEscapeSlash];
    NewParams := CreateParams;
    try
      PO:=[spoCreate,spoUseMacro];
      if sqEscapeSlash in ConnOptions then
        Include(PO,spoEscapeSlash);
      if sqEscapeRepeat in ConnOptions then
        Include(PO,spoEscapeRepeat);
      NewParams.ParseSQL(FSQL.Text, PO, psInterbase, PB, MacroChar,RS);
      NewParams.AssignValues(FMacros);
      FMacros.Assign(NewParams);
    finally
      NewParams.Free;
    end;
  end;
end;

procedure TCustomSQLStatement.SetDataSource(AValue: TDataSource);

begin
  if GetDataSource=AValue then Exit;
  if (FDataLink=Nil) then
    FDataLink:=CreateDataLink;
  FDataLink.DataSource:=AValue;
end;

procedure TCustomSQLStatement.CopyParamsFromMaster(CopyBound: Boolean);
begin
  if Assigned(DataSource) and Assigned(DataSource.Dataset) then
    FParams.CopyParamValuesFromDataset(DataSource.Dataset,CopyBound);
end;

procedure TCustomSQLStatement.SetParams(AValue: TParams);
begin
  if FParams=AValue then Exit;
  FParams.Assign(AValue);
end;

procedure TCustomSQLStatement.SetMacros(AValue: TParams);
begin
  if FMacros=AValue then Exit;
  FMacros.Assign(AValue);
end;

procedure TCustomSQLStatement.SetSQL(AValue: TStrings);
begin
  if FSQL=AValue then Exit;
  FSQL.Assign(AValue);
  RecreateMacros;
end;

procedure TCustomSQLStatement.DoExecute;
begin
  FRowsAffected:=-1;
  If (FParams.Count>0) and Assigned(DataSource) then
    CopyParamsFromMaster(False);
  If LogEvent(detExecute) then
    Log(detExecute,FServerSQL);
  Database.Execute(FCursor,Transaction, FParams);
end;

function TCustomSQLStatement.GetPrepared: Boolean;

begin
  Result := Assigned(FCursor) and (FCursor.FPrepared or FCursor.FDirect);
end;

procedure TCustomSQLStatement.CheckUnprepare;
begin
  if FDoUnPrepare then
    begin
    UnPrepare;
    FDoUnPrepare:=False;
    end;
end;

procedure TCustomSQLStatement.CheckPrepare;
begin
  if Not Prepared then
    begin
    FDoUnprepare:=True;
    Prepare;
    end;
end;

function TCustomSQLStatement.CreateDataLink: TDataLink;
begin
  Result:=TDataLink.Create;
end;

function TCustomSQLStatement.CreateParams: TSQLDBParams;
begin
  Result:=TSQLDBParams.Create(Nil);
end;

function TCustomSQLStatement.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and Database.LogEvent(EventType);
end;

procedure TCustomSQLStatement.Log(EventType: TDBEventType; const Msg: String);
Var
  M : String;

begin
  If LogEvent(EventType) then
    begin
    If (Name<>'') then
      M:=Name
    else
      M:=ClassName;
    Database.Log(EventType,M+' : '+Msg);
    end;
end;

procedure TCustomSQLStatement.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (operation=opRemove) then
    If (AComponent=FTransaction) then
      FTransaction:=Nil
    else if (AComponent=FDatabase) then
      begin
      UnPrepare;
      FDatabase:=Nil;
      end;
end;

constructor TCustomSQLStatement.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FSQL:=TStringList.Create;
  TStringList(FSQL).OnChange:=@OnChangeSQL;
  FParams:=CreateParams;
  FParamCheck:=True;
  FMacros:=CreateParams;
  FMacroChar:=DefaultMacroChar;
  FMacroCheck:=False;
  FParseSQL:=True;
  FRowsAffected:=-1;
end;

destructor TCustomSQLStatement.Destroy;
begin
  UnPrepare;
  Transaction:=Nil;
  Database:=Nil;
  DataSource:=Nil;
  FreeAndNil(FDataLink);
  FreeAndNil(FParams);
  FreeAndNil(FMacros);
  FreeAndNil(FSQL);
  inherited Destroy;
end;

function TCustomSQLStatement.GetSchemaType: TSchemaType;

begin
  Result:=stNoSchema
end;

function TCustomSQLStatement.GetSchemaObjectName: String;
begin
  Result:='';
end;

function TCustomSQLStatement.GetSchemaPattern: String;
begin
  Result:='';
end;

function TCustomSQLStatement.IsSelectable: Boolean;
begin
  Result:=False;
end;

procedure TCustomSQLStatement.GetStatementInfo(var ASQL: String; out Info: TSQLStatementInfo);

begin
  Info:=Database.GetStatementInfo(ASQL);
end;

procedure TCustomSQLStatement.AllocateCursor;

begin
  if not assigned(FCursor) then
    // Do this as late as possible.
    FCursor:=Database.AllocateCursorHandle;
end;

procedure TCustomSQLStatement.DeAllocateCursor;
begin
  if Assigned(FCursor) and Assigned(Database) then
    DataBase.DeAllocateCursorHandle(FCursor);
end;

function TCustomSQLStatement.ExpandMacros( OrigSQL : String ) : String;

Const
  Terminators = SQLDelimiterCharacters+
                [ #0,'=','+','-','*','\','/','[',']','|' ];

var
  I: Integer;
  Ch : Char;
  TermArr : Set of Char;
  TempStr, TempMacroName : String;
  MacroFlag : Boolean;

  Procedure SubstituteMacro;

  var
    Param: TParam;
  begin
    Param := Macros.FindParam( TempMacroName );
    if Assigned( Param ) then
      Result := Result + Param.AsString
    else
      Result := Result + MacroChar + TempMacroName;
    TempMacroName:='';
  end;

begin
  Result := OrigSQL;
  if not MacroCheck then
    Exit;
  TermArr := Terminators +[MacroChar];
  Result := '';
  MacroFlag := False;
  for Ch in OrigSQL do
    begin
    if not MacroFlag and (Ch=MacroChar) then
      begin
      MacroFlag := True;
      TempMacroName := '';
      end
    else if MacroFlag then
      begin
      if not (Ch In TermArr) then
        TempMacroName := TempMacroName + Ch
      else
        begin
        SubstituteMacro;
        if Ch <> MacroChar then
          MacroFlag := False;
        TempMacroName := '';
        end
      end;
    if not MacroFlag then
      Result := Result + Ch;
    end;
  if (TempMacroName<>'') then
    SubstituteMacro;
end;

procedure TCustomSQLStatement.DoPrepare;

var
  StmInfo: TSQLStatementInfo;
begin
  if GetSchemaType=stNoSchema then
    FOrigSQL := TrimRight(FSQL.Text)
  else
    FOrigSQL := Database.GetSchemaInfoSQL(GetSchemaType, GetSchemaObjectName, GetSchemaPattern);
  if (FOrigSQL='') then
    DatabaseError(SErrNoStatement);
  FServerSQL:=ExpandMacros( FOrigSQL );
  GetStatementInfo(FServerSQL,StmInfo);
  AllocateCursor;
  FCursor.FSelectable:=True; // let PrepareStatement and/or Execute alter it
  FCursor.FStatementType:=StmInfo.StatementType;
  FCursor.FSchemaType:=GetSchemaType;
  If LogEvent(detPrepare) then
    Log(detPrepare,FServerSQL);
  Database.PrepareStatement(FCursor,Transaction,FServerSQL,FParams);
  // Update
  FCursor.FInitFieldDef:=FCursor.FSelectable;
end;

procedure TCustomSQLStatement.Prepare;

begin
  if Prepared then
    exit;
  if not assigned(Database) then
    DatabaseError(SErrDatabasenAssigned);
  if not assigned(Transaction) then
    DatabaseError(SErrTransactionnSet);
  Database.MaybeConnect;
  if not Transaction.Active then
    Transaction.MaybeStartTransaction;
  try
    DoPrepare;
  except
    DeAllocateCursor;
    Raise;
  end;
end;

procedure TCustomSQLStatement.Execute;
begin
  CheckPrepare;
  try
    DoExecute;
  finally
    CheckUnPrepare;
  end;
end;

procedure TCustomSQLStatement.DoUnPrepare;

begin
  If Assigned(FCursor) then
    If Assigned(Database) then
      begin
      DataBase.UnPrepareStatement(FCursor);
      DeAllocateCursor;
      end
    else // this should never happen. It means a cursor handle leaks in the DB itself.
      FreeAndNil(FCursor);
end;

function TCustomSQLStatement.GetDataSource: TDataSource;
begin
  if Assigned(FDataLink) then
    Result:=FDataLink.DataSource
  else
    Result:=Nil;
end;

procedure TCustomSQLStatement.Unprepare;
begin
  // Some SQLConnections does not support statement [un]preparation, but they have allocated local cursor(s)
  //  so let them do cleanup f.e. cancel pending queries and/or free resultset
  //  and also do UnRegisterStatement!
  if assigned(FCursor) then
    DoUnprepare;
end;

function TCustomSQLStatement.ParamByName(const AParamName: String): TParam;
begin
  Result:=FParams.ParamByName(AParamName);
end;

function TCustomSQLStatement.RowsAffected: TRowsCount;
begin
  if FRowsAffected=-1 then
    begin
    if Assigned(Database) then
      FRowsAffected:=Database.RowsAffected(FCursor);
    end;
  Result:=FRowsAffected;
end;


{ TSQLConnection }

constructor TSQLConnection.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FSQLFormatSettings:=DefaultSQLFormatSettings;
  FFieldNameQuoteChars:=DoubleQuotes;
  FLogEvents:=LogAllEvents; //match Property LogEvents...Default LogAllEvents
  FStatements:=TThreadList.Create;
  FStatements.Duplicates:=dupIgnore;
end;

destructor TSQLConnection.Destroy;
begin
  Connected:=False; // needed because we want to de-allocate statements
  FreeAndNil(FStatements);
  inherited Destroy;
end;

function TSQLConnection.StrToStatementType(s : string) : TStatementType;
var T : TStatementType;
begin
  S:=Lowercase(s);
  for T:=stSelect to stRollback do
    if (S=StatementTokens[T]) then
      Exit(T);
  Result:=stUnknown;
end;

procedure TSQLConnection.SetTransaction(Value : TSQLTransaction);
begin
  if FTransaction<>value then
    begin
    if Assigned(FTransaction) and FTransaction.Active then
      DatabaseError(SErrAssTransaction);
    if Assigned(Value) then
      Value.Database := Self;
    FTransaction := Value;
    If Assigned(FTransaction) and (FTransaction.Database=Nil) then
      FTransaction.Database:=Self;
    end;
end;

procedure TSQLConnection.UpdateIndexDefs(IndexDefs : TIndexDefs; TableName : string);
begin
  // Empty abstract
end;

procedure TSQLConnection.DoConnect;
var ConnectionCharSet: string;
begin
  inherited;

  // map connection CharSet to corresponding local CodePage
  // do not set FCodePage to CP_ACP if FCodePage = DefaultSystemCodePage
  // aliases listed here are commonly used, but not recognized by CodePageNameToCodePage()
  ConnectionCharSet := LowerCase(GetConnectionCharSet);
  case ConnectionCharSet of
    'utf8','utf-8','utf8mb4':
      FCodePage := CP_UTF8;
    'win1250','cp1250':
      FCodePage := 1250;
    'win1251','cp1251':
      FCodePage := 1251;
    'win1252','cp1252','latin1','iso8859_1':
      FCodePage := 1252;
    else
      begin
      FCodePage := CodePageNameToCodePage(ConnectionCharSet);
      if FCodePage = CP_NONE then
        FCodePage := CP_ACP;
      end;
  end;
end;

procedure TSQLConnection.DoInternalConnect;
begin
  if (DatabaseName = '') and not(sqSupportEmptyDatabaseName in FConnOptions) then
    DatabaseError(SErrNoDatabaseName,Self);
end;

procedure TSQLConnection.DoInternalDisconnect;

Var
  I : integer;
  L : TList;

begin
  L:=FStatements.LockList;
  try
    For I:=0 to L.Count-1 do
      TCustomSQLStatement(L[i]).Unprepare;
    L.Clear;
  finally
    FStatements.UnlockList;
  end;
end;

procedure TSQLConnection.StartTransaction;
begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet)
  else
    Transaction.StartTransaction;
end;

procedure TSQLConnection.EndTransaction;
begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet)
  else
    Transaction.EndTransaction;
end;

procedure TSQLConnection.ExecuteDirect(SQL: String);

begin
  ExecuteDirect(SQL,FTransaction);
end;

procedure TSQLConnection.ExecuteDirect(SQL: String;
  ATransaction: TSQLTransaction);

var Cursor : TSQLCursor;

begin
  if not assigned(ATransaction) then
    DatabaseError(SErrTransactionnSet);

  if not Connected then Open;
  if not ATransaction.Active then
    ATransaction.MaybeStartTransaction;

  SQL := TrimRight(SQL);
  if SQL = '' then
    DatabaseError(SErrNoStatement);

  try
    Cursor := AllocateCursorHandle;
    Cursor.FStatementType := stUnknown;
    If LogEvent(detPrepare) then
      Log(detPrepare,SQL);
    PrepareStatement(Cursor,ATransaction,SQL,Nil);
    try
      If LogEvent(detExecute) then
        Log(detExecute,SQL);
      Execute(Cursor,ATransaction, Nil);
    finally
      UnPrepareStatement(Cursor);
    end;
  finally;
    DeAllocateCursorHandle(Cursor);
  end;
end;

function TSQLConnection.GetPort: cardinal;
begin
  result := StrToIntDef(Params.Values['Port'],0);
end;

procedure TSQLConnection.SetOptions(AValue: TSQLConnectionOptions);
begin
  if FOptions=AValue then Exit;
  FOptions:=AValue;
end;


procedure TSQLConnection.SetPort(const AValue: cardinal);
begin
  if AValue<>0 then
    Params.Values['Port']:=IntToStr(AValue)
  else with params do if IndexOfName('Port') > -1 then
    Delete(IndexOfName('Port'));
end;

function TSQLConnection.AttemptCommit(trans: TSQLHandle): boolean;
begin
  try
    Result:=Commit(trans);
  except
    if ForcedClose then
      Result:=True
    else
      Raise;
  end;
end;

function TSQLConnection.AttemptRollBack(trans: TSQLHandle): boolean;
begin
  try
    Result:=Rollback(trans);
  except
    if ForcedClose then
      Result:=True
    else
      Raise;
  end;
end;

procedure TSQLConnection.GetDBInfo(const ASchemaType : TSchemaType; const ASchemaObjectName, AReturnField : string; AList: TStrings);

var qry : TCustomSQLQuery;

begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet);

  qry := TCustomSQLQuery.Create(nil);
  try
    qry.transaction := Transaction;
    qry.database := Self;
    with qry do
      begin
      ParseSQL := False;
      SetSchemaInfo(ASchemaType,ASchemaObjectName,'');
      open;
      AList.Clear;
      while not eof do
        begin
        AList.Append(trim(fieldbyname(AReturnField).asstring));
        Next;
        end;
      end;
  finally
    qry.free;
  end;  
end;

function TSQLConnection.GetConnectionCharSet: string;
begin
  // default implementation returns user supplied FCharSet
  // (can be overriden by descendants, which are able retrieve current connection charset using client API)
  Result := LowerCase(FCharSet);
end;

function TSQLConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  Result := -1;
end;

function TSQLConnection.AddFieldDef(AFieldDefs: TFieldDefs; AFieldNo: Longint;
  const AName: string; ADataType: TFieldType; ASize, APrecision: Integer;
  AByteSize, ARequired, AReadOnly: Boolean): TFieldDef;
var
  ACodePage: TSystemCodePage;
begin
  // helper function used by descendants
  if ADataType in [ftString, ftFixedChar, ftMemo] then
  begin
    ACodePage := FCodePage;
    // if ASize of character data is passed as "byte length",
    //  translate it to "character length" as expected by TFieldDef
    if AByteSize and (ACodePage = CP_UTF8) then
      ASize := ASize div 4;
  end
  else
    ACodePage := 0;
  Result := AFieldDefs.Add(AName, ADataType, ASize, APrecision, ARequired, AReadOnly, AFieldNo, ACodePage);
end;

procedure TSQLConnection.GetTableNames(List: TStrings; SystemTables: Boolean);
begin
  if not SystemTables then
    GetDBInfo(stTables,'','table_name',List)
  else
    GetDBInfo(stSysTables,'','table_name',List);
end;

procedure TSQLConnection.GetProcedureNames(List: TStrings);
begin
  GetDBInfo(stProcedures,'','procedure_name',List);
end;

procedure TSQLConnection.GetFieldNames(const TableName: string; List: TStrings);
begin
  GetDBInfo(stColumns,TableName,'column_name',List);
end;

procedure TSQLConnection.GetSchemaNames(List: TStrings);
begin
  GetDBInfo(stSchemata,'','SCHEMA_NAME',List);
end;

procedure TSQLConnection.GetSequenceNames(List: TStrings);
begin
  GetDBInfo(stSequences,'','SEQUENCE_NAME',List);
end;

{
  See if we can integrate/merge this with GetDBInfo. They are virtually identical
}

Function TSQLConnection.GetObjectNames(ASchemaType: TSchemaType; AList : TSqlObjectIdentifierList) : Integer; 
var
  qry : TCustomSQLQuery;
  vSchemaName, vObjectName: String;
  f: TField;
begin
  Result:=0;
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet);

  qry := TCustomSQLQuery.Create(nil);
  try
    qry.transaction := Transaction;
    qry.database := Self;
    with qry do
      begin
      ParseSQL := False;
      SetSchemaInfo(ASchemaType,TSchemaObjectNames[ASchemaType],'');
      open;
      f:=FindField(TSchemaObjectNames[stSchemata]);
      while not eof do
        begin
        vSchemaName:='';
        if Assigned(f) then
           vSchemaName:=f.AsString;
        vObjectName:=FieldByName(FSchemaObjectName).AsString;
        AList.AddIdentifier(vObjectName, vSchemaName);
        Next;
        Inc(Result);
        end;
      end;
  finally
    qry.free;
  end;
end;

function TSQLConnection.GetConnectionInfo(InfoType: TConnInfoType): string;
var i: TConnInfoType;
begin
  Result:='';
  if InfoType = citAll then
    for i:=citServerType to citClientVersion do
      begin
      if Result<>'' then Result:=Result+',';
      Result:=Result+'"'+GetConnectionInfo(i)+'"';
      end;
end;

function TSQLConnection.GetStatementInfo(const ASQL: string): TSQLStatementInfo;

type
  TParsePart = (ppStart,ppWith,ppSelect,ppTableName,ppFrom,ppWhere,ppGroup,ppOrder,ppBogus);
  TPhraseSeparator = (sepNone, sepWhiteSpace, sepComma, sepComment, sepParentheses, sepDoubleQuote, sepEnd);
  TKeyword = (kwWITH, kwSELECT, kwINSERT, kwUPDATE, kwDELETE, kwFROM, kwJOIN, kwWHERE, kwGROUP, kwORDER, kwUNION, kwROWS, kwLIMIT, kwUnknown);

const
  KeywordNames: array[TKeyword] of string =
    ('WITH', 'SELECT', 'INSERT', 'UPDATE', 'DELETE', 'FROM', 'JOIN', 'WHERE', 'GROUP', 'ORDER', 'UNION', 'ROWS', 'LIMIT', '');

var
  PSQL, CurrentP, SavedP,
  PhraseP, PStatementPart : pchar;
  S                       : string;
  ParsePart               : TParsePart;
  BracketCount            : Integer;
  Separator               : TPhraseSeparator;
  Keyword, K              : TKeyword;

begin
  PSQL:=PChar(ASQL);
  ParsePart := ppStart;

  CurrentP := PSQL-1;
  PhraseP := PSQL;

  Result.TableName := '';
  Result.Updateable := False;
  Result.WhereStartPos := 0;
  Result.WhereStopPos := 0;

  repeat
    inc(CurrentP);
    SavedP := CurrentP;

    case CurrentP^ of
      ' ', #9..#13:
        Separator := sepWhiteSpace;
      ',':
        Separator := sepComma;
      #0, ';':
        Separator := sepEnd;
      '(':
        begin
        Separator := sepParentheses;
        // skip everything between brackets, since it could be a sub-select, and
        // further nothing between brackets could be interesting for the parser.
        BracketCount := 1;
        repeat
          inc(CurrentP);
          if CurrentP^ = '(' then inc(BracketCount)
          else if CurrentP^ = ')' then dec(BracketCount);
        until (CurrentP^ = #0) or (BracketCount = 0);
        if CurrentP^ <> #0 then inc(CurrentP);
        end;
      '"','`':
        if SkipComments(CurrentP, sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions) then
          Separator := sepDoubleQuote;
      else
        if SkipComments(CurrentP, sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions) then
          Separator := sepComment
        else
          Separator := sepNone;
    end;

    if Separator <> sepNone then
      begin
      if (CurrentP > SavedP) and (SavedP > PhraseP) then
        CurrentP := SavedP;  // there is something before comment or left parenthesis or double quote

      if (Separator in [sepWhitespace,sepComment]) and (SavedP = PhraseP) then
        PhraseP := CurrentP;  // skip comments (but not parentheses) and white spaces

      if (CurrentP-PhraseP > 0) or (Separator = sepEnd) then
        begin
        SetString(s, PhraseP, CurrentP-PhraseP);

        Keyword := kwUnknown;
        for K in TKeyword do
          if SameText(s, KeywordNames[K]) then
          begin
            Keyword := K;
            break;
          end;

        case ParsePart of
          ppStart  : begin
                     Result.StatementType := StrToStatementType(s);
                     case Keyword of
                       kwWITH  : ParsePart := ppWith;
                       kwSELECT: ParsePart := ppSelect;
                       else      break;
                     end;
                     end;
          ppWith   : begin
                     // WITH [RECURSIVE] CTE_name [ ( column_names ) ] AS ( CTE_query_definition ) [, ...]
                     //  { SELECT | INSERT | UPDATE | DELETE } ...
                     case Keyword of
                       kwSELECT: Result.StatementType := stSelect;
                       kwINSERT: Result.StatementType := stInsert;
                       kwUPDATE: Result.StatementType := stUpdate;
                       kwDELETE: Result.StatementType := stDelete;
                     end;
                     if Result.StatementType <> stUnknown then break;
                     end;
          ppSelect : begin
                     if Keyword = kwFROM then
                       ParsePart := ppTableName;
                     end;
          ppTableName:
                     begin
                     // Meta-data requests are never updateable
                     //  and select statements from more than one table
                     //  and/or derived tables are also not updateable
                     if Separator in [sepWhitespace, sepComment, sepDoubleQuote, sepEnd] then
                       begin
                       Result.TableName := Result.TableName + s;
                       Result.Updateable := True;
                       end;
                     // compound delimited classifier like: "schema name"."table name"
                     if not (CurrentP^ in ['.','"']) then
                       ParsePart := ppFrom;
                     end;
          ppFrom   : begin
                     if (Keyword in [kwWHERE, kwGROUP, kwORDER, kwLIMIT, kwROWS]) or
                        (Separator = sepEnd) then
                       begin
                       case Keyword of
                         kwWHERE: ParsePart := ppWhere;
                         kwGROUP: ParsePart := ppGroup;
                         kwORDER: ParsePart := ppOrder;
                         else     ParsePart := ppBogus;
                       end;

                       Result.WhereStartPos := PhraseP-PSQL+1;
                       PStatementPart := CurrentP;
                       end
                     else
                     // joined table or user_defined_function (...)
                     if (Keyword = kwJOIN) or (Separator in [sepComma, sepParentheses]) then
                       begin
                       Result.TableName := '';
                       Result.Updateable := False;
                       end;
                     end;
          ppWhere  : begin
                     if (Keyword in [kwGROUP, kwORDER, kwLIMIT, kwROWS]) or
                        (Separator = sepEnd) then
                       begin
                       ParsePart := ppBogus;
                       Result.WhereStartPos := PStatementPart-PSQL;
                       if (Separator = sepEnd) then
                         Result.WhereStopPos := CurrentP-PSQL+1
                       else
                         Result.WhereStopPos := PhraseP-PSQL+1;
                       end
                     else if (Keyword = kwUNION) then
                       begin
                       ParsePart := ppBogus;
                       Result.Updateable := False;
                       end;
                     end;
        end; {case}
        end;
      if Separator in [sepComment, sepParentheses, sepDoubleQuote] then
        dec(CurrentP);
      PhraseP := CurrentP+1;
      end
  until CurrentP^=#0;
end;

function TSQLConnection.GetAsString(Param: TParam): RawByteString;
begin
  // converts parameter value to connection charset
  if FCodePage = CP_UTF8 then
    Result := Param.AsUTF8String
  else if (FCodePage = DefaultSystemCodePage) or
          (FCodePage = CP_ACP) or (FCodePage = CP_NONE) then
    Result := Param.AsAnsiString
  else
  begin
    Result := Param.AsAnsiString;
    SetCodePage(Result, FCodePage, True);
  end;
end;

function TSQLConnection.GetAsSQLText(Field : TField) : string;
begin
  if (not assigned(Field)) or Field.IsNull then Result := 'Null'
  else case Field.DataType of
    ftString   : Result := QuotedStr(Field.AsString);
    ftDate     : Result := '''' + FormatDateTime('yyyy-mm-dd',Field.AsDateTime,FSQLFormatSettings) + '''';
    ftDateTime : Result := '''' + FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz',Field.AsDateTime,FSQLFormatSettings) + '''';
    ftTime     : Result := '''' + TimeIntervalToString(Field.AsDateTime) + '''';
  else
    Result := Field.AsString;
  end; {case}
end;

function TSQLConnection.GetAsSQLText(Param: TParam) : string;
begin
  if (not assigned(Param)) or Param.IsNull then Result := 'Null'
  else case Param.DataType of
    ftGuid,
    ftMemo,
    ftFixedChar,
    ftString   : Result := QuotedStr(GetAsString(Param));
    ftDate     : Result := '''' + FormatDateTime('yyyy-mm-dd', Param.AsDateTime, FSQLFormatSettings) + '''';
    ftTime     : Result := '''' + TimeIntervalToString(Param.AsDateTime) + '''';
    ftDateTime : Result := '''' + FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz', Param.AsDateTime, FSQLFormatSettings) + '''';
    ftCurrency,
    ftBcd      : Result := CurrToStr(Param.AsCurrency, FSQLFormatSettings);
    ftFloat    : Result := FloatToStr(Param.AsFloat, FSQLFormatSettings);
    ftFMTBcd   : Result := StringReplace(Param.AsString, DefaultFormatSettings.DecimalSeparator, FSQLFormatSettings.DecimalSeparator, []);
  else
    Result := Param.AsString;
  end; {case}
end;


function TSQLConnection.GetHandle: pointer;
begin
  Result := nil;
end;

function TSQLConnection.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=(Assigned(FOnLog) or Assigned(GlobalDBLogHook)) and (EventType in LogEvents);
end;

procedure TSQLConnection.LogParams(const AParams: TParams);

Var
  S : String;
  P : TParam;

begin
  if not LogEvent(detParamValue) or not Assigned(AParams) then
    Exit;
  For P in AParams do
    begin
    if P.IsNull then
      S:='<NULL>'
    else if (P.DataType in ftBlobTypes) and  not (P.DataType in [ftMemo, ftFmtMemo,ftWideMemo]) then
      S:='<BLOB>'
    else
      S:=P.AsString;
    Log(detParamValue,Format(SLogParamValue,[P.Name,S]));
    end;
end;

procedure TSQLConnection.Log(EventType: TDBEventType; const Msg: String);

Var
  M : String;

begin
  If LogEvent(EventType) then
    begin
    If Assigned(FonLog) then
      FOnLog(Self,EventType,Msg);
    If Assigned(GlobalDBLogHook) then
      begin
      If (Name<>'') then
        M:=Name+' : '+Msg
      else
        M:=ClassName+' : '+Msg;
      GlobalDBLogHook(Self,EventType,M);
      end;
    end;
end;

procedure TSQLConnection.RegisterStatement(S: TCustomSQLStatement);

begin
  FStatements.Add(S);
end;

procedure TSQLConnection.UnRegisterStatement(S: TCustomSQLStatement);
begin
  if Assigned(FStatements) then // Can be nil, when we are destroying and datasets are uncoupled.
    FStatements.Remove(S);
end;

function TSQLConnection.CreateCustomQuery(aOwner : TComponent) : TCustomSQLQuery;

begin
  Result:=TCustomSQLQuery.Create(AOwner);
end;

function TSQLConnection.InitialiseUpdateStatement(Query : TCustomSQLQuery; var qry : TCustomSQLQuery): TCustomSQLQuery;

begin
  if not assigned(qry) then
  begin
    qry := TCustomSQLQuery(TComponentClass(Query.ClassType).Create(Nil));
    qry.ParseSQL := False;
    qry.DataBase := Self;
    qry.Transaction := Query.SQLTransaction;
    qry.Unidirectional:=True;
    qry.UsePrimaryKeyAsKey:=False;
    qry.PacketRecords:=1;
  end;
  Result:=qry;
end;


procedure TSQLConnection.AddFieldToUpdateWherePart(var sql_where : string;UpdateMode : TUpdateMode; F : TField);

begin
  if (pfInKey in F.ProviderFlags)
     or ((UpdateMode = upWhereAll) and (pfInWhere in F.ProviderFlags))
     or ((UpdateMode = UpWhereChanged) and (pfInWhere in F.ProviderFlags) and (F.Value <> F.OldValue)) then
     begin
     if (sql_where<>'') then
       sql_where:=sql_where + ' and ';
     sql_where:= sql_where + '(' + FieldNameQuoteChars[0] + F.FieldName + FieldNameQuoteChars[1];
     if F.OldValue = NULL then
        sql_where :=  sql_where + ' is null '
     else
        sql_where :=  sql_where +'= :"' + 'OLD_' + F.FieldName + '"';
     sql_where:=sql_where+') ';
     end;
end;


function TSQLConnection.ConstructInsertSQL(Query: TCustomSQLQuery;
  var ReturningClause: Boolean): string;

var x          : integer;
    sql_fields : string;
    sql_values : string;
    returning_fields : String;
    F : TField;


begin
  sql_fields := '';
  sql_values := '';
  returning_fields := '';
  for x := 0 to Query.Fields.Count -1 do
    begin
    F:=Query.Fields[x];
    if (not F.IsNull) and (pfInUpdate in F.ProviderFlags) and (not F.ReadOnly) then
      begin
      sql_fields := sql_fields + FieldNameQuoteChars[0] + F.FieldName + FieldNameQuoteChars[1] + ',';
      sql_values := sql_values + ':"' + F.FieldName + '",';
      end;
    if ReturningClause and (pfRefreshOnInsert in F.ProviderFlags) then
      returning_fields := returning_fields + FieldNameQuoteChars[0] + F.FieldName + FieldNameQuoteChars[1] + ',';
    end;
  if length(sql_fields) = 0 then
    DatabaseErrorFmt(sNoUpdateFields,['insert'],self);
  setlength(sql_fields,length(sql_fields)-1);
  setlength(sql_values,length(sql_values)-1);
  result := 'insert into ' + Query.FTableName + ' (' + sql_fields + ') values (' + sql_values + ')';
  if ReturningClause then
    begin
    ReturningClause:=length(returning_fields) <> 0 ;
    if ReturningClause then
      begin
      setlength(returning_fields,length(returning_fields)-1);
      Result := Result + ' returning ' + returning_fields;
      end;
    end;
end;


function TSQLConnection.ConstructUpdateSQL(Query: TCustomSQLQuery;
  var ReturningClause: Boolean): string;

var x : integer;
    F : TField;
    sql_set    : string;
    sql_where  : string;
    returning_fields : String;

begin
  sql_set := '';
  sql_where := '';
  returning_fields := '';
  for x := 0 to Query.Fields.Count -1 do
    begin
    F:=Query.Fields[x];
    AddFieldToUpdateWherePart(sql_where,Query.UpdateMode,F);
    if (pfInUpdate in F.ProviderFlags) and (not F.ReadOnly) then
      sql_set := sql_set +FieldNameQuoteChars[0] + F.FieldName + FieldNameQuoteChars[1] +'=:"' + F.FieldName + '",';
    if ReturningClause and (pfRefreshOnUpdate in F.ProviderFlags) then
      returning_fields := returning_fields + FieldNameQuoteChars[0] + F.FieldName + FieldNameQuoteChars[1] + ',';
    end;
  if length(sql_set) = 0 then DatabaseErrorFmt(sNoUpdateFields,['update'],self);
  setlength(sql_set,length(sql_set)-1);
  if length(sql_where) = 0 then DatabaseErrorFmt(sNoWhereFields,['update'],self);
  result := 'update ' + Query.FTableName + ' set ' + sql_set + ' where ' + sql_where;
  if ReturningClause then
    begin
    ReturningClause:=length(returning_fields) <> 0 ;
    if ReturningClause then
      begin
      setlength(returning_fields,length(returning_fields)-1);
      Result := Result + ' returning ' + returning_fields;
      end;
    end;
end;


function TSQLConnection.ConstructDeleteSQL(Query : TCustomSQLQuery) :  string;

var
  x          : integer;
  sql_where  : string;

begin
  sql_where := '';
  for x := 0 to Query.Fields.Count -1 do
    AddFieldToUpdateWherePart(sql_where,Query.UpdateMode, Query.Fields[x]);
  if length(sql_where) = 0 then
    DatabaseErrorFmt(sNoWhereFields,['delete'],self);
  result := 'delete from ' + Query.FTableName + ' where ' + sql_where;
end;

function TSQLConnection.ConstructRefreshSQL(Query: TCustomSQLQuery; UpdateKind: TUpdateKind): string;

Var
  F : TField;
  PF : TProviderFlag;
  Where : String;

begin
  Result:=Trim(Query.RefreshSQL.Text);
  if (Result='') then
    begin
    Where:='';
    PF:=RefreshFlags[UpdateKind];
    For F in Query.Fields do
      begin
      if PF in F.ProviderFlags then
        begin
        if (Result<>'') then
          Result:=Result+', ';
        if (F.Origin<>'') and (F.Origin<>F.FieldName) then
          Result:=Result+F.Origin+' AS '+F.FieldName
        else
          Result:=Result+FieldNameQuoteChars[0]+F.FieldName+FieldNameQuoteChars[1]
        end;
      if pfInkey in F.ProviderFlags then
        begin
        if (Where<>'') then
          Where:=Where+' AND ';
        Where:=Where+'('+FieldNameQuoteChars[0]+F.FieldName+FieldNameQuoteChars[1]+' = :'+F.FieldName+')';
        end;
      end;
    if (Where='') then
      DatabaseError(SErrNoKeyFieldForRefreshClause,Query);
    Result:='SELECT '+Result+' FROM '+Query.FTableName+' WHERE '+Where;
    end;
end;

procedure TSQLConnection.ApplyFieldUpdate(C : TSQLCursor; P : TSQLDBParam; F : TField; UseOldValue : Boolean);

begin
  if UseOldValue then
    P.AssignFieldValue(F,F.OldValue)
  else
    P.AssignFieldValue(F,F.Value);
  P.FFieldDef:=F.FieldDef;
end;

procedure TSQLConnection.ApplyRecUpdate(Query: TCustomSQLQuery; UpdateKind: TUpdateKind);

var
  qry : TCustomSQLQuery;
  s   : string;
  x   : integer;
  Fld : TField;
  Par, P : TParam;
  UseOldValue, HasReturningClause : Boolean;

begin
  qry:=Nil;
  HasReturningClause:=(sqSupportReturning in ConnOptions) and not (sqoRefreshUsingSelect in Query.Options) and (Trim(Query.RefreshSQL.Text)='');
  case UpdateKind of
    ukInsert : begin
               s := Trim(Query.FInsertSQL.Text);
               if s = '' then
                 s := ConstructInsertSQL(Query, HasReturningClause)
               else
                 HasReturningClause := False;
               qry := InitialiseUpdateStatement(Query, Query.FInsertQry);
               end;
    ukModify : begin
               s := Trim(Query.FUpdateSQL.Text);
               if s = '' then begin
                 //if not assigned(Query.FUpdateQry) or (Query.UpdateMode<>upWhereKeyOnly) then // first time or dynamic where part
                   s := ConstructUpdateSQL(Query, HasReturningClause);
               end
               else
                 HasReturningClause := False;
               qry := InitialiseUpdateStatement(Query, Query.FUpdateQry);
               end;
    ukDelete : begin
               s := Trim(Query.FDeleteSQL.Text);
               if (s='') and (not assigned(Query.FDeleteQry) or (Query.UpdateMode<>upWhereKeyOnly)) then
                 s := ConstructDeleteSQL(Query);
               HasReturningClause := False;
               qry := InitialiseUpdateStatement(Query, Query.FDeleteQry);
               end;
  end;
  if (s<>'') and (qry.SQL.Text<>s) then
    qry.SQL.Text:=s; //assign only when changed, to avoid UnPrepare/Prepare
  Assert(qry.SQL.Text<>'');
  for x:=0 to Qry.Params.Count-1 do
    begin
    P:=Qry.Params[x];
    S:=P.Name;
    UseOldValue:=SameText(Copy(S,1,4),'OLD_');
    if UseOldValue then
      begin
      Delete(S,1,4);
      Fld:=Query.FieldByName(S);
      end
    else
      Fld:=Query.FindField(S);
    if Assigned(Fld) then
      ApplyFieldUpdate(Query.Cursor, P as TSQLDBParam, Fld, UseOldValue)
    else
      begin
      // if does not exists field with given name, try look for param
      Par:=Query.Params.FindParam(S);
      if Assigned(Par) then
        P.Assign(Par)
      else
        DatabaseErrorFmt(SFieldNotFound,[S],Query); // same error as raised by FieldByName()
      end;
    end;
  if HasReturningClause then
    begin
    Qry.Close;
    Qry.Open
    end
  else
    Qry.ExecSQL;
  if (scoApplyUpdatesChecksRowsAffected in Options) and (Qry.RowsAffected<>1) then
    begin
    Qry.Close;
    DatabaseErrorFmt(SErrFailedToUpdateRecord, [Qry.RowsAffected], Query);
    end;
  if HasReturningClause then
    Query.ApplyReturningResult(Qry,UpdateKind);
end;

function TSQLConnection.RefreshLastInsertID(Query: TCustomSQLQuery; Field: TField): Boolean;
begin
  Result:=False;
end;

procedure TSQLConnection.FreeFldBuffers(cursor: TSQLCursor);
begin
  // empty
end;

function TSQLConnection.StartImplicitTransaction(trans: TSQLHandle; aParams: string): boolean;
begin
  Result:=False;
end;

function TSQLConnection.GetSchemaInfoSQL( SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string;

begin
  case SchemaType of
    stTables    : Result := 'SELECT * FROM INFORMATION_SCHEMA.TABLES WHERE TABLE_TYPE=''BASE TABLE''';
    stColumns   : Result := 'SELECT * FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_NAME='+QuotedStr(SchemaObjectName);
    stProcedures: Result := 'SELECT *, ROUTINE_NAME AS PROCEDURE_NAME FROM INFORMATION_SCHEMA.ROUTINES';
    stSchemata  : Result := 'SELECT * FROM INFORMATION_SCHEMA.SCHEMATA';
    stSequences : Result := 'SELECT * FROM INFORMATION_SCHEMA.SEQUENCES';
    else DatabaseError(SMetadataUnavailable);
  end;
end;

function TSQLConnection.GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
begin
  Result := 'SELECT NEXT VALUE FOR ' + SequenceName;
end;

function TSQLConnection.GetNextValue(const SequenceName: string; IncrementBy: integer): Int64;
var
  Q: TCustomSQLQuery;
begin
  Result := 0;
  Q := TCustomSQLQuery.Create(nil);
  try
    Q.DataBase := Self;
    Q.Transaction := Transaction;
    Q.SQL.Text := GetNextValueSQL(SequenceName, IncrementBy);
    Q.Open;
    if not Q.Eof then
      Result := Q.Fields[0].AsLargeInt;
    Q.Close;
  finally
    FreeAndNil(Q);
  end;
end;

procedure TSQLConnection.MaybeConnect;
begin
  If Not Connected then
    begin
    If (scoExplicitConnect in Options) then
      DatabaseErrorFmt(SErrImplicitConnect,[Name]);
    Connected:=True;
    end;
end;

procedure TSQLConnection.CreateDB;

begin
  DatabaseError(SNotSupported);
end;

procedure TSQLConnection.DropDB;

begin
  DatabaseError(SNotSupported);
end;


{ TSQLTransaction }

constructor TSQLTransaction.Create(AOwner : TComponent);
begin
  inherited Create(AOwner);
  FParams := TStringList.Create;
  Action := caRollBack;
end;

destructor TSQLTransaction.Destroy;
begin
  EndTransaction;
  FreeAndNil(FTrans);
  FreeAndNil(FParams);
  inherited Destroy;
end;

procedure TSQLTransaction.EndTransaction;

begin
  Case Action of
    caCommit, caCommitRetaining :
      Commit;
    caNone,
    caRollback, caRollbackRetaining :
      if not (stoUseImplicit in Options) then
        RollBack
      else
        CloseTrans;
  end;
end;

procedure TSQLTransaction.SetParams(const AValue: TStringList);
begin
  FParams.Assign(AValue);
end;

function TSQLTransaction.GetSQLConnection: TSQLConnection;
begin
  Result:=Database as TSQLConnection;
end;

procedure TSQLTransaction.SetOptions(AValue: TSQLTransactionOptions);
begin
  if FOptions=AValue then Exit;
  if (stoUseImplicit in Avalue) and Assigned(SQLConnection) And Not (sqImplicitTransaction in SQLConnection.ConnOptions) then
    DatabaseErrorFmt(SErrNoImplicitTransaction, [SQLConnection.ClassName]);
  FOptions:=AValue;
end;

procedure TSQLTransaction.SetSQLConnection(AValue: TSQLConnection);
begin
  Database:=AValue;
end;

Procedure TSQLTransaction.MaybeStartTransaction;
begin
  if not Active then
    begin
    if (stoExplicitStart in Options) then
      DatabaseErrorFmt(SErrImplictTransactionStart, [Database.Name,Name]);
    StartTransaction;
    end;
end;

function TSQLTransaction.GetHandle: Pointer;
begin
  Result := SQLConnection.GetTransactionHandle(FTrans);
end;

Function TSQLTransaction.AllowClose(DS: TDBDataset): Boolean;
begin
  if (DS is TSQLQuery) then
    Result:=not (sqoKeepOpenOnCommit in TSQLQuery(DS).Options)
  else
    Result:=Inherited AllowClose(DS);
end;

procedure TSQLTransaction.Commit;
begin
  if Active  then
    begin
    CloseDataSets;
    If LogEvent(detCommit) then
      Log(detCommit,SCommitting);
    // The inherited closetrans must always be called.
    // So the last (FTrans=Nil) is for the case of forced close. (Bug IDs 35246 and 33737)
    // Order is important:
    // some connections do not have FTrans, but they must still go through AttemptCommit.
    if (stoUseImplicit in Options) or SQLConnection.AttemptCommit(FTrans) or (FTrans=Nil) then
      begin
      CloseTrans;
      FreeAndNil(FTrans);
      end;
    end;
end;

procedure TSQLTransaction.CommitRetaining;
begin
  if Active then
    begin
    If LogEvent(detCommit) then
      Log(detCommit,SCommitRetaining);
    SQLConnection.CommitRetaining(FTrans);
    end;
end;

procedure TSQLTransaction.Rollback;
begin
  if Active then
    begin
    if (stoUseImplicit in Options) then
      DatabaseError(SErrImplicitNoRollBack);
    CloseDataSets;
    If LogEvent(detRollback) then
      Log(detRollback,SRollingBack);
    // The inherited closetrans must always be called.
    // So the last (FTrans=Nil) is for the case of forced close. (Bug IDs 35246 and 33737)
    // Order is important:
    // some connections do not have FTrans, but they must still go through AttemptCommit.
    // FTrans=Nil for the case of forced close.
    if SQLConnection.AttemptRollBack(FTrans) or (FTrans=Nil) then
      begin
      CloseTrans;
      FreeAndNil(FTrans);
      end;
    end;
end;

procedure TSQLTransaction.RollbackRetaining;
begin
  if Active then
    begin
    if (stoUseImplicit in Options) then
      DatabaseError(SErrImplicitNoRollBack);
    If LogEvent(detRollback) then
      Log(detRollback,SRollBackRetaining);
    SQLConnection.RollBackRetaining(FTrans);
    end;
end;

procedure TSQLTransaction.StartTransaction;

var db : TSQLConnection;

begin
  if Active then
    DatabaseError(SErrTransAlreadyActive);

  db := SQLConnection;

  if Db = nil then
    DatabaseError(SErrDatabasenAssigned);

  Db.MaybeConnect;

  if not assigned(FTrans) then FTrans := Db.AllocateTransactionHandle;

  if (stoUseImplicit in Options) then
    begin
    if Db.StartImplicitTransaction(FTrans,FParams.CommaText) then
      OpenTrans
    end
  else
    begin
    if Db.StartDBTransaction(FTrans,FParams.CommaText) then
      OpenTrans
    end;
end;

Procedure TSQLTransaction.SetDatabase(Value: TDatabase);

begin
  If Value<>Database then
    begin
    if Assigned(Value) and not (Value is TSQLConnection) then
      DatabaseErrorFmt(SErrNotASQLConnection, [Value.Name], Self);
    CheckInactive;
    if (stoUseImplicit in Options) and Assigned(Value) and Not (sqImplicitTransaction in TSQLConnection(Value).ConnOptions) then
      DatabaseErrorFmt(SErrNoImplicitTransaction, [Value.ClassName]);
    If Assigned(Database) then
      if SQLConnection.Transaction = Self then SQLConnection.Transaction := nil;
    inherited;
    If Assigned(Database) and not (csLoading in ComponentState) then
      If SQLConnection.Transaction = Nil then SQLConnection.Transaction := Self;
    end;
end;

Function TSQLTransaction.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and SQLConnection.LogEvent(EventType);
end;

Procedure TSQLTransaction.Log(EventType: TDBEventType; Const Msg: String);

Var
  M : String;

begin
  If LogEvent(EventType) then
    begin
    If (Name<>'') then
      M:=Name+' : '+Msg
    else
      M:=Msg;
    SQLConnection.Log(EventType,M);
    end;
end;


{ TSQLSequence }

constructor TSQLSequence.Create(AQuery: TCustomSQLQuery);
begin
  inherited Create;
  FQuery := AQuery;
  FApplyEvent := saeOnNewRecord;
  FIncrementBy := 1;
end;

procedure TSQLSequence.Assign(Source: TPersistent);
var SourceSequence: TSQLSequence;
begin
  if Source is TSQLSequence then
  begin
    SourceSequence := TSQLSequence(Source);
    FFieldName    := SourceSequence.FieldName;
    FSequenceName := SourceSequence.SequenceName;
    FIncrementBy  := SourceSequence.IncrementBy;
    FApplyEvent   := SourceSequence.ApplyEvent;
  end
  else
    inherited;
end;

procedure TSQLSequence.Apply;
var Field: TField;
begin
  if Assigned(FQuery) and (FSequenceName<>'') and (FFieldName<>'') then
  begin
    Field := FQuery.FindField(FFieldName);
    if Assigned(Field) and Field.IsNull then
      Field.AsLargeInt := GetNextValue;
  end;
end;

function TSQLSequence.GetNextValue: Int64;
begin
  if (FQuery=Nil) or (FQuery.SQLConnection=Nil) then
    DatabaseError(SErrDatabasenAssigned);
  Result := FQuery.SQLConnection.GetNextValue(FSequenceName, FIncrementBy);
end;


Type

  { TQuerySQLStatement }

  TQuerySQLStatement = Class(TCustomSQLStatement)
  protected
    FQuery : TCustomSQLQuery;
    function CreateParams: TSQLDBParams; override;
    Function CreateDataLink : TDataLink; override;
    Function GetSchemaType : TSchemaType; override;
    Function GetSchemaObjectName : String; override;
    Function GetSchemaPattern: String; override;
    procedure GetStatementInfo(var ASQL: String; out Info: TSQLStatementInfo); override;
    procedure OnChangeSQL(Sender : TObject); override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TQuerySQLStatement }

constructor TQuerySQLStatement.Create(AOwner: TComponent);
begin
  FQuery:=TCustomSQLQuery(AOwner);
  inherited Create(AOwner);
end;

function TQuerySQLStatement.CreateDataLink: TDataLink;
begin
  Result:=TMasterParamsDataLink.Create(FQuery);
end;

function TQuerySQLStatement.CreateParams: TSQLDBParams;
begin
  Result:=FQuery.CreateParams;
end;

function TQuerySQLStatement.GetSchemaType: TSchemaType;
begin
  if Assigned(FQuery) then
    Result:=FQuery.FSchemaType
  else
    Result:=stNoSchema;
end;

function TQuerySQLStatement.GetSchemaObjectName: String;
begin
  if Assigned(FQuery) then
    Result:=FQuery.FSchemaObjectname
  else
    Result:=inherited GetSchemaObjectName;
end;

function TQuerySQLStatement.GetSchemaPattern: String;
begin
  if Assigned(FQuery) then
    Result:=FQuery.FSchemaPattern
  else
    Result:=inherited GetSchemaPattern;
end;

procedure TQuerySQLStatement.GetStatementInfo(var ASQL: String; out Info: TSQLStatementInfo);
begin
  inherited GetStatementInfo(ASQL, Info);
  If Assigned(FQuery) then
    // Note: practical side effect of switch off ParseSQL is that UpdateServerIndexDefs is bypassed
    //       which is used as performance tuning option
    if (FQuery.FSchemaType = stNoSchema) and FParseSQL then
      begin
      FQuery.FUpdateable:=Info.Updateable;
      FQuery.FTableName:=Info.TableName;
      FQuery.FWhereStartPos:=Info.WhereStartPos;
      FQuery.FWhereStopPos:=Info.WhereStopPos;
      if FQuery.ServerFiltered then
        ASQL:=FQuery.AddFilter(ASQL);
      end
    else
      begin
      FQuery.FUpdateable:=false;
      FQuery.FTableName:='';
      FQuery.FWhereStartPos:=0;
      FQuery.FWhereStopPos:=0;
      end;
end;

procedure TQuerySQLStatement.OnChangeSQL(Sender: TObject);
begin
  UnPrepare;
  inherited OnChangeSQL(Sender);
  If ParamCheck and Assigned(FDataLink) then
    (FDataLink as TMasterParamsDataLink).RefreshParamNames;
  FQuery.ServerIndexDefs.Updated:=false;
end;

{ TCustomSQLQuery }

function TCustomSQLQuery.CreateSQLStatement(aOwner: TComponent
  ): TCustomSQLStatement;

begin
  Result:=TQuerySQLStatement.Create(Self);
end;

constructor TCustomSQLQuery.Create(AOwner : TComponent);

begin
  inherited Create(AOwner);
  FStatement:=CreateSQLStatement(Self);

  FInsertSQL := TStringList.Create;
  FInsertSQL.OnChange := @OnChangeModifySQL;
  FUpdateSQL := TStringList.Create;
  FUpdateSQL.OnChange := @OnChangeModifySQL;
  FDeleteSQL := TStringList.Create;
  FDeleteSQL.OnChange := @OnChangeModifySQL;
  FRefreshSQL := TStringList.Create;
  FRefreshSQL.OnChange := @OnChangeModifySQL;

  FSequence := TSQLSequence.Create(Self);
  FServerIndexDefs := TServerIndexDefs.Create(Self);

  FServerFiltered := False;
  FServerFilterText := '';

  FSchemaType:=stNoSchema;
  FSchemaObjectName:='';
  FSchemaPattern:='';

// Delphi has upWhereAll as default, but since strings and oldvalue's don't work yet
// (variants) set it to upWhereKeyOnly
  FUpdateMode := upWhereKeyOnly;
  FUsePrimaryKeyAsKey := True;
end;

destructor TCustomSQLQuery.Destroy;
begin
  if Active then Close;
  UnPrepare;
  FreeAndNil(FStatement);
  FreeAndNil(FInsertSQL);
  FreeAndNil(FUpdateSQL);
  FreeAndNil(FDeleteSQL);
  FreeAndNil(FRefreshSQL);
  FreeAndNil(FSequence);
  FreeAndNil(FServerIndexDefs);
  inherited Destroy;
end;

function TCustomSQLQuery.ParamByName(const AParamName: String): TParam;

begin
  Result:=Params.ParamByName(AParamName);
end;

function TCustomSQLQuery.MacroByName(const AParamName: String): TParam;
begin
  Result:=Macros.ParamByName(AParamName);
end;

procedure TCustomSQLQuery.OnChangeModifySQL(Sender : TObject);

begin
  CheckInactive;
end;

procedure TCustomSQLQuery.SetDatabase(Value : TDatabase);

var DB : TSQLConnection;

begin
  if Database = Value then Exit;
  if Assigned(Value) and not (Value is TSQLConnection) then
    DatabaseErrorFmt(SErrNotASQLConnection, [Value.Name], Self);
  UnPrepare;
  DB := TSQLConnection(Value);
  If Assigned(FStatement) then
    FStatement.Database := DB;
  inherited;
  if Assigned(DB) and Assigned(DB.Transaction) and (not Assigned(Transaction) or (Transaction.DataBase<>Database)) then
    Transaction := DB.Transaction;
end;

procedure TCustomSQLQuery.SetTransaction(Value: TDBTransaction);

begin
  if Transaction = Value then Exit;
  UnPrepare;
  inherited;
  If Assigned(FStatement) then
    FStatement.Transaction := TSQLTransaction(Value);
  If Assigned(Transaction) and Assigned(Transaction.DataBase) and (Database<>Transaction.DataBase) then
    Database := Transaction.Database;
end;

function TCustomSQLQuery.IsPrepared: Boolean;

begin
  if Assigned(Fstatement) then
    Result := FStatement.Prepared
  else
    Result := False;
end;

function TCustomSQLQuery.AddFilter(SQLstr: string): string;

begin
  if (FWhereStartPos > 0) and (FWhereStopPos > 0) then
    begin
    system.insert('(',SQLstr,FWhereStartPos+1);
    system.insert(')',SQLstr,FWhereStopPos+1);
    end;

  if FWhereStartPos = 0 then
    SQLstr := SQLstr + ' where (' + ServerFilter + ')'
  else if FWhereStopPos > 0 then
    system.insert(' and ('+ServerFilter+') ',SQLstr,FWhereStopPos+2)
  else
    system.insert(' where ('+ServerFilter+') ',SQLstr,FWhereStartPos);
  Result := SQLstr;
end;

procedure TCustomSQLQuery.OpenCursor(InfoQuery: Boolean);
begin
  if InfoQuery then
    CheckPrepare;
  try
    inherited OpenCursor(InfoQuery);
  finally
    if InfoQuery then
      CheckUnPrepare;
  end;
end;

function TCustomSQLQuery.NeedRefreshRecord(UpdateKind: TUpdateKind): Boolean;


Var
  PF : TProviderFlag;
  I : Integer;
  DoReturning : Boolean;

begin
  Result:=(Trim(FRefreshSQL.Text)<>'');
  DoReturning:=(sqSupportReturning in SQLConnection.ConnOptions) and not (sqoRefreshUsingSelect in Options);
  if Not (Result or DoReturning) then
    begin
    PF:=RefreshFlags[UpdateKind];
    I:=0;
    While (Not Result) and (I<Fields.Count) do
      begin
      Result:=PF in Fields[i].ProviderFlags;
      Inc(I);
      end;
    end;
end;

function TCustomSQLQuery.RefreshRecord(UpdateKind: TUpdateKind): Boolean;

Var
  Q : TCustomSQLQuery;
  P : TParam;
  F,FD : TField;
  N : String;

begin
  Result:=False;
  Q:=TCustomSQLQuery.Create(Nil);
  try
    Q.Database:=Self.Database;
    Q.Transaction:=Self.Transaction;
    Q.SQL.Text:=SQLConnection.ConstructRefreshSQL(Self,UpdateKind);
    For P in Q.Params do
      begin
      N:=P.Name;
      If CompareText(Copy(N,1,4),'OLD_')=0 then
        system.Delete(N,1,4);
      F:=Fields.FindField(N);
      if Assigned(F) then
        P.AssignField(F);
      end;
    Q.Open;
    try
      if (Q.EOF and Q.BOF) then
        DatabaseError(SErrRefreshEmptyResult,Self)
      else
        begin
        if Q.RecordCount<>1 then
          DatabaseErrorFmt(SErrRefreshNotSingleton,[Q.RecordCount],Self);
        For F in Q.Fields do
          begin
          FD:=Fields.FindField(F.FieldName);
          if Assigned(FD) then
            begin
            FD.Assign(F);
            Result:=True; // We could check if the new value differs from the old, but we won't.
            end;
          end;
        end
    finally
      Q.Close;
    end;
  finally
    Q.Free;
  end;
end;

procedure TCustomSQLQuery.ApplyReturningResult(Q: TCustomSQLQuery; UpdateKind : TUpdateKind);

Var
  S : TDataSetState;
  refreshFlag  : TProviderFlag;
  F : TField;

begin
  RefreshFlag:=RefreshFlags[UpdateKind];
  S:=SetTempState(dsRefreshFields);
  try
    For F in Fields do
      if RefreshFlag in F.ProviderFlags then
        F.Assign(Q.FieldByName(F.FieldName));
  finally
    RestoreState(S);
  end;
end;

procedure TCustomSQLQuery.ApplyFilter;

begin
  if Prepared then
    FStatement.Unprepare;
  InternalRefresh;
  First;
end;

procedure TCustomSQLQuery.SetServerFiltered(Value: Boolean);

begin
  if Value and not ParseSQL then
    DatabaseErrorFmt(SNoParseSQL,['Filtering ']);
  if (ServerFiltered <> Value) then
    begin
    FServerFiltered := Value;
    if Active then ApplyFilter;
    end;
end;

procedure TCustomSQLQuery.SetServerFilterText(const Value: string);
begin
  if Value <> ServerFilter then
    begin
    FServerFilterText := Value;
    if Active then ApplyFilter;
    end;
end;


procedure TCustomSQLQuery.Prepare;

begin
  FStatement.Prepare;
end;

procedure TCustomSQLQuery.UnPrepare;

begin
  if Not Refreshing then
    CheckInactive;
  If Assigned(FStatement) then
    FStatement.Unprepare;
end;

procedure TCustomSQLQuery.FreeFldBuffers;
begin
  if assigned(Cursor) then
     SQLConnection.FreeFldBuffers(Cursor);
end;

function TCustomSQLQuery.GetMacroChar: Char;
begin
  Result := FStatement.MacroChar;
end;

function TCustomSQLQuery.GetParamCheck: Boolean;
begin
  Result:=FStatement.ParamCheck;
end;

function TCustomSQLQuery.GetParams: TParams;
begin
  Result:=FStatement.Params;
end;

function TCustomSQLQuery.GetMacroCheck: Boolean;
begin
  Result:=FStatement.MacroCheck;
end;

function TCustomSQLQuery.GetMacros: TParams;
begin
  Result:=FStatement.Macros;
end;

function TCustomSQLQuery.GetParseSQL: Boolean;
begin
  Result:=FStatement.ParseSQL;
end;

function TCustomSQLQuery.GetServerIndexDefs: TServerIndexDefs;
begin
  Result := FServerIndexDefs;
end;

function TCustomSQLQuery.GetSQL: TStringList;
begin
  Result:=TStringList(Fstatement.SQL);
end;

function TCustomSQLQuery.GetSQLConnection: TSQLConnection;
begin
  Result:=Database as TSQLConnection;
end;

function TCustomSQLQuery.GetSQLTransaction: TSQLTransaction;
begin
  Result:=Transaction as TSQLTransaction;
end;

function TCustomSQLQuery.Cursor: TSQLCursor;
begin
  Result:=FStatement.Cursor;
end;

function TCustomSQLQuery.Fetch : boolean;
begin
  if Not Assigned(Cursor) then
    Exit;
  if not Cursor.FSelectable then
    Exit;
  If LogEvent(detFetch) then
    Log(detFetch,FStatement.FServerSQL);
  if not FIsEof then FIsEOF := not SQLConnection.Fetch(Cursor);
  Result := not FIsEOF;
end;

procedure TCustomSQLQuery.Execute;
begin
  FStatement.DoExecute;
end;

function TCustomSQLQuery.RowsAffected: TRowsCount;
begin
  Result:=FStatement.RowsAffected;
end;

function TCustomSQLQuery.LoadField(FieldDef : TFieldDef; buffer : pointer; out CreateBlob : boolean) : boolean;
begin
  Result := SQLConnection.LoadField(Cursor, FieldDef, buffer, CreateBlob);
  // disable deferred blob loading for "disconnected" datasets
  if Result and (FieldDef.DataType in ftBlobTypes) and (sqoKeepOpenOnCommit in Options) then
    CreateBlob:=True
end;

procedure TCustomSQLQuery.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField);
begin
  SQLConnection.LoadBlobIntoBuffer(FieldDef, ABlobBuf, Cursor,SQLTransaction);
end;

procedure TCustomSQLQuery.InternalAddRecord(Buffer: Pointer; AAppend: Boolean);
begin
  // not implemented - sql dataset
end;

procedure TCustomSQLQuery.InternalClose;
begin

  if assigned(Cursor) then
    begin
    if Cursor.FSelectable then
      FreeFldBuffers;
    CheckUnPrepare;
    // Some SQLConnections does not support statement [un]preparation,
    //  so let them do cleanup f.e. cancel pending queries and/or free resultset
    // if not Prepared then
    //  FStatement.DoUnprepare;
    end;

  if DefaultFields then
    DestroyFields;

  FIsEOF := False;
  if assigned(FUpdateQry) then FreeAndNil(FUpdateQry);
  if assigned(FInsertQry) then FreeAndNil(FInsertQry);
  if assigned(FDeleteQry) then FreeAndNil(FDeleteQry);
//  FRecordSize := 0;

  inherited InternalClose;
end;

procedure TCustomSQLQuery.InternalInitFieldDefs;

begin
  if FLoadingFieldDefs then
    Exit;
  FLoadingFieldDefs := True;
  try
    FieldDefs.Clear;
    SQLConnection.AddFieldDefs(Cursor,FieldDefs);
  finally
    FLoadingFieldDefs := False;
    if assigned(Cursor) then Cursor.FInitFieldDef := False;
  end;
end;

procedure TCustomSQLQuery.InternalOpen;

var counter, fieldc : integer;
    F               : TField;
    IndexFields     : TStrings;

begin
  if IsReadFromPacket then
    begin
    // When we read from file there is no need for Cursor, also note that Database may not be assigned
    //FStatement.AllocateCursor;
    //Cursor.FSelectable:=True;
    //Cursor.FStatementType:=stSelect;
    FUpdateable:=True;
    end
  else
    begin
    CheckPrepare;
    if not Cursor.FSelectable then
      DatabaseError(SErrNoSelectStatement,Self);

    // Call UpdateServerIndexDefs before Execute, to avoid problems with connections
    // which do not allow processing multiple recordsets at a time. (Microsoft
    // calls this MARS, see bug 13241)
    if DefaultFields and FUpdateable and FusePrimaryKeyAsKey and (not IsUniDirectional) then
      UpdateServerIndexDefs;

    FStatement.Execute;
    if not Cursor.FSelectable then
      DatabaseError(SErrNoSelectStatement,Self);

    // InternalInitFieldDef is only called after a prepare. i.e. not twice if
    // a dataset is opened - closed - opened.
    if Cursor.FInitFieldDef then
      InternalInitFieldDefs;
    if DefaultFields then
      begin
      CreateFields;

      if FUpdateable and FusePrimaryKeyAsKey and (not IsUniDirectional) then
        for counter := 0 to ServerIndexDefs.Count-1 do
          if ixPrimary in ServerIndexDefs[counter].Options then
            begin
            IndexFields := TStringList.Create;
            ExtractStrings([';'],[' '],pchar(ServerIndexDefs[counter].Fields),IndexFields);
            for fieldc := 0 to IndexFields.Count-1 do
              begin
              F := FindField(IndexFields[fieldc]);
              if F <> nil then
                F.ProviderFlags := F.ProviderFlags + [pfInKey];
              end;
            IndexFields.Free;
            end;
      end;
    end;
  BindFields(True);

  if not ReadOnly and not FUpdateable and (FSchemaType=stNoSchema) then
    begin
    if (trim(FDeleteSQL.Text) <> '') or (trim(FUpdateSQL.Text) <> '') or
       (trim(FInsertSQL.Text) <> '') then FUpdateable := True;
    end;

  inherited InternalOpen;
end;

procedure TCustomSQLQuery.InternalRefresh;
begin
  if (ChangeCount>0) and (sqoCancelUpdatesOnRefresh in Options) then
    CancelUpdates;
  inherited InternalRefresh;
end;

// public part

procedure TCustomSQLQuery.CheckPrepare;

begin
  if Not IsPrepared then
    begin
    Prepare;
    FDoUnPrepare:=True;
    end;
end;

procedure TCustomSQLQuery.CheckUnPrepare;

begin
  if FDoUnPrepare then
    begin
    FDoUnPrepare:=False;
    UnPrepare;
    end;
end;


procedure TCustomSQLQuery.ExecSQL;

begin
  CheckPrepare;
  try
    Execute;
    // Always retrieve rows affected
    FStatement.RowsAffected;
    If sqoAutoCommit in Options then
      SQLTransaction.Commit;
  finally
    CheckUnPrepare;
    // if not Prepared and (assigned(Database)) and (assigned(Cursor)) then SQLConnection.UnPrepareStatement(Cursor);
  end;
end;

procedure TCustomSQLQuery.ApplyUpdates(MaxErrors: Integer);
begin
  inherited ApplyUpdates(MaxErrors);
  If sqoAutoCommit in Options then
    begin
    // Retrieve rows affected for last update.
    FStatement.RowsAffected;
    SQLTransaction.Commit;
    end;
end;

procedure TCustomSQLQuery.Post;
begin
  inherited Post;
  If (sqoAutoApplyUpdates in Options) then
    ApplyUpdates;
end;

procedure TCustomSQLQuery.Delete;
begin
  inherited Delete;
  If (sqoAutoApplyUpdates in Options) then
    ApplyUpdates;
end;

procedure TCustomSQLQuery.SetReadOnly(AValue : Boolean);

begin
  CheckInactive;
  inherited SetReadOnly(AValue);
end;

procedure TCustomSQLQuery.SetParseSQL(AValue : Boolean);

begin
  CheckInactive;
  FStatement.ParseSQL:=AValue;
  if not AValue then
    FServerFiltered := False;
end;

procedure TCustomSQLQuery.SetSQL(const AValue: TStringList);
begin
  FStatement.SQL.Assign(AValue);
end;

procedure TCustomSQLQuery.SetUsePrimaryKeyAsKey(AValue : Boolean);

begin
  if not Active then FusePrimaryKeyAsKey := AValue
  else
    begin
    // Just temporary, this should be possible in the future
    DatabaseError(SActiveDataset);
    end;
end;

procedure TCustomSQLQuery.UpdateServerIndexDefs;

begin
  FServerIndexDefs.Clear;
  if assigned(DataBase) and (FTableName<>'') then
    SQLConnection.UpdateIndexDefs(ServerIndexDefs,FTableName);
end;

function TCustomSQLQuery.NeedLastInsertID: TField;

Var
  I : Integer;

begin
  Result:=Nil;
  if sqLastInsertID in SQLConnection.ConnOptions then
    begin
    I:=0;
    While (Result=Nil) and (I<Fields.Count) do
      begin
      Result:=Fields[i];
      if (Result.DataType<>ftAutoInc) or not Result.IsNull then
        Result:=Nil;
      Inc(I);
      end;
    end
end;

procedure TCustomSQLQuery.SetMacroChar(AValue: Char);
begin
  FStatement.MacroChar:=AValue;
end;

function TCustomSQLQuery.RefreshLastInsertID(Field: TField): Boolean;

begin
  Result:=SQLConnection.RefreshLastInsertID(Self, Field);
end;

procedure TCustomSQLQuery.ApplyRecUpdate(UpdateKind: TUpdateKind);

Var
  DoRefresh : Boolean;
  LastIDField : TField;
  S : TDataSetState;

begin
  // Moved to connection: the SQLConnection always has more information about types etc.
  // than SQLQuery itself.
  SQLConnection.ApplyRecUpdate(Self,UpdateKind);

  if UpdateKind=ukInsert then
    LastIDField:=NeedLastInsertID
  else
    LastIDField:=nil;
  DoRefresh:=(UpdateKind in [ukModify,ukInsert]) and NeedRefreshRecord(UpdateKind);
  if assigned(LastIDField) or DoRefresh then
    begin
    // updates fields directly in record buffer of TBufDataSet
    //   TDataSet buffers are resynchronized at end of ApplyUpdates process
    S:=SetTempState(dsRefreshFields);
    try
      if assigned(LastIDField) then
        RefreshLastInsertID(LastIDField);
      if DoRefresh then
        RefreshRecord(UpdateKind);
    finally
      RestoreState(S);
    end;
    end;
end;

procedure TCustomSQLQuery.SetPacketRecords(aValue: integer);
begin
  if (AValue=PacketRecords) then exit;
  if (AValue<>-1) and (sqoKeepOpenOnCommit in Options) then
    DatabaseError(SErrDisconnectedPacketRecords);
  Inherited SetPacketRecords(aValue);
end;


function TCustomSQLQuery.GetCanModify: Boolean;

begin
  // the test for assigned(Cursor) is needed for the case that the dataset isn't opened
  if assigned(Cursor) and (Cursor.FStatementType = stSelect) then
    Result:= FUpdateable and (not ReadOnly) and (not IsUniDirectional)
  else
    Result := False;
end;

procedure TCustomSQLQuery.SetUpdateMode(AValue : TUpdateMode);

begin
  FUpdateMode := AValue;
end;

procedure TCustomSQLQuery.SetSchemaInfo( ASchemaType : TSchemaType; ASchemaObjectName, ASchemaPattern : string);

begin
  FSchemaType:=ASchemaType;
  FSchemaObjectName:=ASchemaObjectName;
  FSchemaPattern:=ASchemaPattern;
end;

procedure TCustomSQLQuery.BeforeRefreshOpenCursor;
begin
  // This is only necessary because TIBConnection can not re-open a
  // prepared cursor. In fact this is wrong, but has never led to
  // problems because in SetActive(false) queries are always
  // unprepared. (which is also wrong, but has to be fixed later)
  if IsPrepared then with SQLConnection do
    UnPrepareStatement(Cursor);
end;

function TCustomSQLQuery.CreateParams: TSQLDBParams;
begin
  Result:=TSQLDBParams.Create(Nil);
end;

function TCustomSQLQuery.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and SQLConnection.LogEvent(EventType);
end;

procedure TCustomSQLQuery.Log(EventType: TDBEventType; const Msg: String);

Var
  M : String;

begin
  If LogEvent(EventType) then
    begin
    M:=Msg;
    If (Name<>'') then
      M:=Name+' : '+M;
    SQLConnection.Log(EventType,M);
    end;
end;

class function TCustomSQLQuery.FieldDefsClass: TFieldDefsClass;
begin
  Result:=TSQLDBFieldDefs;
end;

function TCustomSQLQuery.GetStatementType : TStatementType;

begin
  if Assigned(Cursor) then
    Result:=Cursor.FStatementType
  else
    Result:=stUnknown;
end;

procedure TCustomSQLQuery.SetParamCheck(AValue: Boolean);
begin
  FStatement.ParamCheck:=AValue;
end;

procedure TCustomSQLQuery.SetMacroCheck(AValue: Boolean);
begin
  FStatement.MacroCheck:=AValue;
end;

procedure TCustomSQLQuery.SetOptions(AValue: TSQLQueryOptions);
begin
  if FOptions=AValue then Exit;
  CheckInactive;
  FOptions:=AValue;
  if sqoKeepOpenOnCommit in FOptions then
    PacketRecords:=-1;
end;

procedure TCustomSQLQuery.SetSQLConnection(AValue: TSQLConnection);
begin
  Database:=AValue;
end;

procedure TCustomSQLQuery.SetSQLTransaction(AValue: TSQLTransaction);
begin
  Transaction:=AValue;
end;

procedure TCustomSQLQuery.SetInsertSQL(const AValue: TStringList);
begin
  FInsertSQL.Assign(AValue);
end;

procedure TCustomSQLQuery.SetUpdateSQL(const AValue: TStringList);
begin
  FUpdateSQL.Assign(AValue);
end;

procedure TCustomSQLQuery.SetDeleteSQL(const AValue: TStringList);
begin
  FDeleteSQL.Assign(AValue);
end;

procedure TCustomSQLQuery.SetRefreshSQL(const AValue: TStringList);
begin
  FRefreshSQL.Assign(AValue);
end;


procedure TCustomSQLQuery.SetParams(AValue: TParams);
begin
  FStatement.Params.Assign(AValue);
end;

procedure TCustomSQLQuery.SetMacros(AValue: TParams);
begin
  FStatement.Macros.Assign(AValue);
end;

procedure TCustomSQLQuery.SetDataSource(AValue: TDataSource);

Var
  DS : TDataSource;

begin
  DS:=DataSource;
  If (AValue<>DS) then
    begin
    If Assigned(AValue) and (AValue.Dataset=Self) then
      DatabaseError(SErrCircularDataSourceReferenceNotAllowed,Self);
    If Assigned(DS) then
      DS.RemoveFreeNotification(Self);
    FStatement.DataSource:=AValue;
    end;
end;

function TCustomSQLQuery.GetDataSource: TDataSource;

begin
  If Assigned(FStatement) then
    Result:=FStatement.DataSource
  else
    Result:=Nil;
end;

procedure TCustomSQLQuery.Notification(AComponent: TComponent; Operation: TOperation);

begin
  Inherited;
  If (Operation=opRemove) and (AComponent=DataSource) then
    DataSource:=Nil;
end;

procedure TCustomSQLQuery.DoOnNewRecord;
begin
  inherited;
  if FSequence.ApplyEvent = saeOnNewRecord then
    FSequence.Apply;
end;

procedure TCustomSQLQuery.DoBeforePost;
begin
  if (State = dsInsert) and (FSequence.ApplyEvent = saeOnPost) then
    FSequence.Apply;
  inherited;
end;

function TCustomSQLQuery.PSGetUpdateException(E: Exception; Prev: EUpdateError): EUpdateError;
var
  PrevErrorCode, ErrorCode: Integer;
begin
  if Assigned(Prev) then
    PrevErrorCode := Prev.ErrorCode
  else
    PrevErrorCode := 0;

  if E is ESQLDatabaseError then
    ErrorCode := ESQLDatabaseError(E).ErrorCode
  else
    ErrorCode := 0;

  Result := EUpdateError.Create(SOnUpdateError, E.Message, ErrorCode, PrevErrorCode, E);
end;

function TCustomSQLQuery.PSGetTableName: string;
begin
  Result := FTableName;
end;

{ TSQLScript }

procedure TSQLScript.ExecuteStatement(SQLStatement: TStrings;
  var StopExecution: Boolean);
begin
  fquery.SQL.assign(SQLStatement);
  fquery.ExecSQL;
end;

procedure TSQLScript.ExecuteDirective(Directive, Argument: String;
  var StopExecution: Boolean);
begin
  if assigned (FOnDirective) then
    FOnDirective (Self, Directive, Argument, StopExecution);
end;

procedure TSQLScript.ExecuteCommit(CommitRetaining: boolean=true);
begin
  if FTransaction is TSQLTransaction then
    if CommitRetaining then
      TSQLTransaction(FTransaction).CommitRetaining
    else
      begin
      TSQLTransaction(FTransaction).Commit;
      TSQLTransaction(FTransaction).StartTransaction;
      end
  else
    begin
    FTransaction.Active := false;
    FTransaction.Active := true;
    end;
end;

procedure TSQLScript.SetDatabase(Value: TDatabase);
begin
  FDatabase := Value;
end;

procedure TSQLScript.SetTransaction(Value: TDBTransaction);
begin
  FTransaction := Value;
end;

procedure TSQLScript.CheckDatabase;
begin
  If (FDatabase=Nil) then
    DatabaseError(SErrNoDatabaseAvailable,Self)
end;

function TSQLScript.CreateQuery: TCustomSQLQuery;
begin
  Result := TCustomSQLQuery.Create(nil);
  Result.ParamCheck := false; // Do not parse for parameters; breaks use of e.g. select bla into :bla in Firebird procedures
end;

constructor TSQLScript.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FQuery := CreateQuery;
end;

destructor TSQLScript.Destroy;
begin
  FQuery.Free;
  inherited Destroy;
end;

procedure TSQLScript.Execute;
begin
  FQuery.DataBase := FDatabase;
  FQuery.Transaction := FTransaction;
  inherited Execute;
end;

procedure TSQLScript.ExecuteScript;
begin
  Execute;
end;


{ Connection definitions }

Var
  ConnDefs : TStringList;

Procedure CheckDefs;

begin
  If (ConnDefs=Nil) then
    begin
    ConnDefs:=TStringList.Create;
    ConnDefs.Sorted:=True;
    ConnDefs.Duplicates:=dupError;
    end;
end;

Procedure DoneDefs;

Var
  I : Integer;


begin
  If Assigned(ConnDefs) then
    begin
    For I:=ConnDefs.Count-1 downto 0 do
      begin
      ConnDefs.Objects[i].Free;
      ConnDefs.Delete(I);
      end;
    FreeAndNil(ConnDefs);
    end;
end;


Function GetConnectionDef(ConnectorName : String) : TConnectionDef;

Var
  I : Integer;

begin
  CheckDefs;
  I:=ConnDefs.IndexOf(ConnectorName);
  If (I<>-1) then
    Result:=TConnectionDef(ConnDefs.Objects[i])
  else
    Result:=Nil;
end;

procedure RegisterConnection(Def: TConnectionDefClass);

Var
  I : Integer;

begin
  CheckDefs;
  I:=ConnDefs.IndexOf(Def.TypeName);
  If (I=-1) then
    ConnDefs.AddObject(Def.TypeName,Def.Create)
  else
    begin
    ConnDefs.Objects[I].Free;
    ConnDefs.Objects[I]:=Def.Create;
    end;
end;

procedure UnRegisterConnection(Def: TConnectionDefClass);
begin
  UnRegisterConnection(Def.TypeName);
end;

procedure UnRegisterConnection(ConnectionName: String);

Var
  I : Integer;

begin
  if (ConnDefs<>Nil) then
    begin
    I:=ConnDefs.IndexOf(ConnectionName);
    If (I<>-1) then
      begin
      ConnDefs.Objects[I].Free;
      ConnDefs.Delete(I);
      end;
    end;
end;

procedure GetConnectionList(List: TSTrings);
begin
  CheckDefs;
  List.Text:=ConnDefs.Text;
end;

{ TSQLConnector }

procedure TSQLConnector.SetConnectorType(const AValue: String);
begin
  if FConnectorType<>AValue then
    begin
    CheckDisconnected;
    If Assigned(FProxy) then
      FreeProxy;
    FConnectorType:=AValue;
    CreateProxy;
    end;
end;

procedure TSQLConnector.SetTransaction(Value: TSQLTransaction);
begin
  inherited SetTransaction(Value);
  If Assigned(FProxy) and (FProxy.Transaction<>Value) then
    FProxy.FTransaction:=Value;
end;

procedure TSQLConnector.DoInternalConnect;

Var
  D : TConnectionDef;

begin
  inherited DoInternalConnect;
  CheckProxy;
  FProxy.CharSet:=Self.CharSet;
  FProxy.DatabaseName:=Self.DatabaseName;
  FProxy.HostName:=Self.HostName;
  FProxy.LogEvents:=Self.LogEvents;
  FProxy.Password:=Self.Password;
  FProxy.Role:=Self.Role;
  FProxy.UserName:=Self.UserName;
  FProxy.FTransaction:=Self.Transaction;
  FProxy.LogEvents:=Self.LogEvents;
  FProxy.OnLog:=Self.OnLog;
  FProxy.Options:=Self.Options;
  D:=GetConnectionDef(ConnectorType);
  D.ApplyParams(Params,FProxy);
  FProxy.Connected:=True;
end;

procedure TSQLConnector.DoInternalDisconnect;
begin
  FProxy.Connected:=False;
  inherited DoInternalDisconnect;
end;

procedure TSQLConnector.CheckProxy;
begin
  If (FProxy=Nil) then
    CreateProxy;
end;

procedure TSQLConnector.CreateProxy;

Var
  D : TConnectionDef;

begin
  D:=GetConnectionDef(ConnectorType);
  If (D=Nil) then
    DatabaseErrorFmt(SErrUnknownConnectorType,[ConnectorType],Self);
  FProxy:=D.ConnectionClass.Create(Self);
  FFieldNameQuoteChars := FProxy.FieldNameQuoteChars;
  FConnOptions := FProxy.ConnOptions;
end;

procedure TSQLConnector.FreeProxy;
begin
  FProxy.Connected:=False;
  FreeAndNil(FProxy);
end;

function TSQLConnector.StrToStatementType(s: string): TStatementType;
begin
  CheckProxy;
  Result:=FProxy.StrToStatementType(s);
end;

function TSQLConnector.GetAsSQLText(Field: TField): string;
begin
  CheckProxy;
  Result:=FProxy.GetAsSQLText(Field);
end;

function TSQLConnector.GetAsSQLText(Param: TParam): string;
begin
  CheckProxy;
  Result:=FProxy.GetAsSQLText(Param);
end;

function TSQLConnector.GetHandle: pointer;
begin
  CheckProxy;
  Result:=FProxy.GetHandle;
end;

function TSQLConnector.AllocateCursorHandle: TSQLCursor;
begin
  CheckProxy;
  Result:=FProxy.AllocateCursorHandle;
end;

procedure TSQLConnector.DeAllocateCursorHandle(var cursor: TSQLCursor);
begin
  CheckProxy;
  FProxy.DeAllocateCursorHandle(cursor);
end;

function TSQLConnector.AllocateTransactionHandle: TSQLHandle;
begin
  CheckProxy;
  Result:=FProxy.AllocateTransactionHandle;
end;

procedure TSQLConnector.PrepareStatement(cursor: TSQLCursor;
  ATransaction: TSQLTransaction; buf: string; AParams: TParams);
begin
  CheckProxy;
  FProxy.PrepareStatement(cursor, ATransaction, buf, AParams);
end;

procedure TSQLConnector.Execute(cursor: TSQLCursor;
  atransaction: tSQLtransaction; AParams: TParams);
begin
  CheckProxy;
  FProxy.Execute(cursor, atransaction, AParams);
end;

function TSQLConnector.Fetch(cursor: TSQLCursor): boolean;
begin
  CheckProxy;
  Result:=FProxy.Fetch(cursor);
end;

procedure TSQLConnector.AddFieldDefs(cursor: TSQLCursor; FieldDefs: TfieldDefs
  );
begin
  CheckProxy;
  FProxy.AddFieldDefs(cursor, FieldDefs);
end;

procedure TSQLConnector.UnPrepareStatement(cursor: TSQLCursor);
begin
  CheckProxy;
  FProxy.UnPrepareStatement(cursor);
end;

procedure TSQLConnector.FreeFldBuffers(cursor: TSQLCursor);
begin
  CheckProxy;
  FProxy.FreeFldBuffers(cursor);
end;

function TSQLConnector.GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
begin
  Result:=Proxy.GetNextValueSQL(SequenceName, IncrementBy);
end;

function TSQLConnector.LoadField(cursor: TSQLCursor; FieldDef: TFieldDef;
  buffer: pointer; out CreateBlob: boolean): boolean;
begin
  CheckProxy;
  Result:=FProxy.LoadField(cursor, FieldDef, buffer, CreateBlob);
end;

procedure TSQLConnector.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction);
begin
  CheckProxy;
  FProxy.LoadBlobIntoBuffer(FieldDef, ABlobBuf, cursor, ATransaction);
end;

function TSQLConnector.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  CheckProxy;
  Result := FProxy.RowsAffected(cursor);
end;

function TSQLConnector.GetTransactionHandle(trans: TSQLHandle): pointer;
begin
  CheckProxy;
  Result:=FProxy.GetTransactionHandle(trans);
end;

function TSQLConnector.Commit(trans: TSQLHandle): boolean;
begin
  CheckProxy;
  Result:=FProxy.Commit(trans);
end;

function TSQLConnector.RollBack(trans: TSQLHandle): boolean;
begin
  CheckProxy;
  Result:=FProxy.RollBack(trans);
end;

function TSQLConnector.StartDBTransaction(trans: TSQLHandle; aParams: string): boolean;
begin
  CheckProxy;
  Result:=FProxy.StartDBTransaction(trans, aParams);
end;

procedure TSQLConnector.CommitRetaining(trans: TSQLHandle);
begin
  CheckProxy;
  FProxy.CommitRetaining(trans);
end;

procedure TSQLConnector.RollBackRetaining(trans: TSQLHandle);
begin
  CheckProxy;
  FProxy.RollBackRetaining(trans);
end;

procedure TSQLConnector.UpdateIndexDefs(IndexDefs: TIndexDefs;
  TableName: string);
begin
  CheckProxy;
  FProxy.UpdateIndexDefs(IndexDefs, TableName);
end;

function TSQLConnector.GetSchemaInfoSQL(SchemaType: TSchemaType;
  SchemaObjectName, SchemaPattern: string): string;
begin
  CheckProxy;
  Result:=FProxy.GetSchemaInfoSQL(SchemaType, SchemaObjectName, SchemaPattern);
end;


{ TConnectionDef }

class function TConnectionDef.TypeName: String;
begin
  Result:='';
end;

class function TConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  Result:=Nil;
end;

class function TConnectionDef.Description: String;
begin
  Result:='';
end;

class function TConnectionDef.DefaultLibraryName: String;
begin
  Result:='';
end;

class function TConnectionDef.LoadFunction: TLibraryLoadFunction;
begin
  Result:=Nil;
end;

class function TConnectionDef.UnLoadFunction: TLibraryUnLoadFunction;
begin
  Result:=Nil;
end;

class function TConnectionDef.LoadedLibraryName: string;
begin
  Result:='';
end;

procedure TConnectionDef.ApplyParams(Params: TStrings;
  AConnection: TSQLConnection);
begin
  AConnection.Params.Assign(Params);
end;

{ TServerIndexDefs }

constructor TServerIndexDefs.create(ADataset: TDataset);
begin
  if not (ADataset is TCustomSQLQuery) then
    DatabaseErrorFmt(SErrNotASQLQuery,[ADataset.Name]);
  inherited create(ADataset);
end;

procedure TServerIndexDefs.Update;
begin
  if (not updated) and assigned(Dataset) then
    begin
    TCustomSQLQuery(Dataset).UpdateServerIndexDefs;
    updated := True;
    end;
end;




Initialization

Finalization
  DoneDefs;
end.
