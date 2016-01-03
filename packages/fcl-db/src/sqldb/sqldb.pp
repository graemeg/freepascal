{
    Copyright (c) 2004-2014 by Joost van der Sluis, FPC contributors

    SQL database & dataset

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit sqldb;

{$mode objfpc}
{$H+}
{$M+}   // ### remove this!!!

interface

uses SysUtils, Classes, DB, bufdataset, sqlscript;

<<<<<<< HEAD
type
  TSchemaType = (stNoSchema, stTables, stSysTables, stProcedures, stColumns, stProcedureParams, stIndexes, stPackages, stSchemata, stSequences);
=======
type TSchemaType = (stNoSchema, stTables, stSysTables, stProcedures, stColumns, stProcedureParams, stIndexes, stPackages);
     TConnOption = (sqSupportParams,sqEscapeSlash,sqEscapeRepeat,sqQuoteFieldnames);
     TConnOptions= set of TConnOption;
>>>>>>> graemeg/cpstrnew

const
  TSchemaObjectNames: array[TSchemaType] of String = ('???', 'table_name',
      '???', 'procedure_name', 'column_name', 'param_name',
      'index_name', 'package_name', 'schema_name','sequence');

     TRowsCount = LargeInt;

     TRowsCount = LargeInt;

type

  TStatementType = (stUnknown, stSelect, stInsert, stUpdate, stDelete,
    stDDL, stGetSegment, stPutSegment, stExecProcedure,
    stStartTrans, stCommit, stRollback, stSelectForUpd);


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
<<<<<<< HEAD
<<<<<<< HEAD
  TCustomSQLStatement = Class;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  TSQLQuery = class;
  TSQLScript = class;


  TDBEventType = (detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack, detParamValue, detActualSQL);
  TDBEventTypes = set of TDBEventType;
  TDBLogNotifyEvent = Procedure (Sender : TSQLConnection; EventType : TDBEventType; Const Msg : String) of object;

  TDBEventType = (detCustom, detPrepare, detExecute, detFetch, detCommit,detRollBack);
  TDBEventTypes = set of TDBEventType;
  TDBLogNotifyEvent = Procedure (Sender : TSQLConnection; EventType : TDBEventType; Const Msg : String) of object;

  TDBEventType = (detCustom, detPrepare, detExecute, detFetch, detCommit,detRollBack);
  TDBEventTypes = set of TDBEventType;
  TDBLogNotifyEvent = Procedure (Sender : TSQLConnection; EventType : TDBEventType; Const Msg : String) of object;

  TDBEventType = (detCustom, detPrepare, detExecute, detFetch, detCommit,detRollBack);
  TDBEventTypes = set of TDBEventType;
  TDBLogNotifyEvent = Procedure (Sender : TSQLConnection; EventType : TDBEventType; Const Msg : String) of object;

  TDBEventType = (detCustom, detPrepare, detExecute, detFetch, detCommit,detRollBack);
  TDBEventTypes = set of TDBEventType;
  TDBLogNotifyEvent = Procedure (Sender : TSQLConnection; EventType : TDBEventType; Const Msg : String) of object;

  TSQLHandle = Class(TObject)
  end;

  { TSQLCursor }

  TSQLCursor = Class(TSQLHandle)
  public
    FPrepared      : Boolean;
    FSelectable    : Boolean;
    FInitFieldDef  : Boolean;
    FStatementType : TStatementType;
    FSchemaType    : TSchemaType;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

  TQuoteChars = array[0..1] of char;
=======
type TQuoteChars = array[0..1] of char;
>>>>>>> graemeg/cpstrnew
=======
type TQuoteChars = array[0..1] of char;
>>>>>>> graemeg/cpstrnew
=======
type TQuoteChars = array[0..1] of char;
>>>>>>> graemeg/cpstrnew
=======
type TQuoteChars = array[0..1] of char;
>>>>>>> origin/cpstrnew

const
  SingleQuotes : TQuoteChars = ('''','''');
  DoubleQuotes : TQuoteChars = ('"','"');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  LogAllEvents      = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack];
  LogAllEventsExtra = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack, detParamValue,detActualSQL];
  StatementTokens : Array[TStatementType] of string = ('(unknown)', 'select',
=======
  LogAllEvents = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack];
  StatementTokens : Array[TStatementType] of string = ('(none)', 'select',
>>>>>>> graemeg/cpstrnew
=======
  LogAllEvents = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack];
  StatementTokens : Array[TStatementType] of string = ('(none)', 'select',
>>>>>>> graemeg/cpstrnew
=======
  LogAllEvents = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack];
  StatementTokens : Array[TStatementType] of string = ('(none)', 'select',
>>>>>>> graemeg/cpstrnew
=======
  LogAllEvents = [detCustom, detPrepare, detExecute, detFetch, detCommit, detRollBack];
  StatementTokens : Array[TStatementType] of string = ('(none)', 'select',
>>>>>>> origin/cpstrnew
                  'insert', 'update', 'delete',
                  'create', 'get', 'put', 'execute',
                  'start','commit','rollback', '?'
                 );

type
<<<<<<< HEAD
<<<<<<< HEAD
=======

  { TServerIndexDefs }
=======

  { TServerIndexDefs }

  TServerIndexDefs = class(TIndexDefs)
  Private
  public
    constructor Create(ADataSet: TDataSet); override;
    procedure Update; override;
  end;

>>>>>>> origin/fixes_2_2

  TServerIndexDefs = class(TIndexDefs)
  Private
  public
    constructor Create(ADataSet: TDataSet); override;
    procedure Update; override;
  end;

>>>>>>> graemeg/fixes_2_2

  { TServerIndexDefs }

  TServerIndexDefs = class(TIndexDefs)
  Private
  public
    constructor Create(ADataSet: TDataSet); override;
    procedure Update; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  TSqlObjectIdentifierList = class;

  { TSqlObjectIdenfier }

  TSqlObjectIdenfier = class(TCollectionItem)
  private
    FObjectName: String;
    FSchemaName: String;
  public
    constructor Create(ACollection: TSqlObjectIdentifierList; Const AObjectName: String; Const ASchemaName: String = '');
    property SchemaName: String read FSchemaName write FSchemaName;
    property ObjectName: String read FObjectName write FObjectName;
  end;

  { TSqlObjectIdentifierList }

  TSqlObjectIdentifierList = class(TCollection)
  private
    function GetIdentifier(Index: integer): TSqlObjectIdenfier;
    procedure SetIdentifier(Index: integer; AValue: TSqlObjectIdenfier);
  public
    function AddIdentifier: TSqlObjectIdenfier; overload;
    function AddIdentifier(Const AObjectName: String; Const ASchemaName: String = ''): TSqlObjectIdenfier; overload;
    property Identifiers[Index: integer]: TSqlObjectIdenfier read GetIdentifier write SetIdentifier; default;
  end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type

  { TSQLConnection }

  TConnOption = (sqSupportParams, sqSupportEmptyDatabaseName, sqEscapeSlash, sqEscapeRepeat, sqImplicitTransaction, sqLastInsertID, sqSupportReturning);
  TConnOptions= set of TConnOption;

  TSQLConnectionOption = (scoExplicitConnect, scoApplyUpdatesChecksRowsAffected);
  TSQLConnectionOptions = Set of TSQLConnectionOption;

  TConnInfoType=(citAll=-1, citServerType, citServerVersion, citServerVersionString, citClientName, citClientVersion);

  TSQLConnection = class (TDatabase)
  private
    FFieldNameQuoteChars : TQuoteChars;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FOptions             : TSQLConnectionOptions;
=======
    FLogEvents: TDBEventTypes;
    FOnLog: TDBLogNotifyEvent;
>>>>>>> graemeg/cpstrnew
=======
    FLogEvents: TDBEventTypes;
    FOnLog: TDBLogNotifyEvent;
>>>>>>> graemeg/cpstrnew
=======
    FLogEvents: TDBEventTypes;
    FOnLog: TDBLogNotifyEvent;
>>>>>>> graemeg/cpstrnew
=======
    FLogEvents: TDBEventTypes;
    FOnLog: TDBLogNotifyEvent;
>>>>>>> origin/cpstrnew
    FPassword            : string;
    FTransaction         : TSQLTransaction;
    FUserName            : string;
    FHostName            : string;
    FCharSet             : string;
    FRole                : String;
<<<<<<< HEAD
<<<<<<< HEAD
    FStatements          : TFPList;
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
    function InitialiseUpdateStatement(Query: TCustomSQLQuery; var qry: TCustomSQLQuery): TCustomSQLQuery;
    procedure ApplyFieldUpdate(C : TSQLCursor; P: TSQLDBParam; F: TField; UseOldValue: Boolean); virtual;
    // This is the call that updates a record, it used to be in TSQLQuery.
    procedure ApplyRecUpdate(Query : TCustomSQLQuery; UpdateKind : TUpdateKind); virtual;
    function RefreshLastInsertID(Query : TCustomSQLQuery; Field : TField): Boolean; virtual;
    procedure GetDBInfo(const ASchemaType : TSchemaType; const ASchemaObjectName, AReturnField : string; AList: TStrings);
    procedure SetTransaction(Value : TSQLTransaction); virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    
    function GetPort: cardinal;
    procedure Setport(const AValue: cardinal);
  protected
    FConnOptions         : TConnOptions;
    procedure GetDBInfo(const SchemaType : TSchemaType; const SchemaObjectName, ReturnField : string; List: TStrings);
    procedure SetTransaction(Value : TSQLTransaction);virtual;
    function StrToStatementType(s : string) : TStatementType; virtual;
>>>>>>> graemeg/fixes_2_2
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    function GetAsSQLText(Field : TField) : string; overload; virtual;
    function GetAsSQLText(Param : TParam) : string; overload; virtual;
<<<<<<< HEAD
    function GetHandle : pointer; virtual;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure LogParams(Const AParams : TParams); virtual;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
    Procedure RegisterStatement(S : TCustomSQLStatement);
    Procedure UnRegisterStatement(S : TCustomSQLStatement);
=======
    function GetHandle : pointer; virtual; virtual;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
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
    Function AllocateCursorHandle : TSQLCursor; virtual; abstract;
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); virtual; abstract;
    function StrToStatementType(s : string) : TStatementType; virtual;
    procedure PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams); virtual; abstract;
    procedure UnPrepareStatement(cursor : TSQLCursor); virtual; abstract;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); virtual; abstract;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; virtual;
    function Fetch(cursor : TSQLCursor) : boolean; virtual; abstract;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TfieldDefs); virtual; abstract;
    function LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; virtual; abstract;
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
<<<<<<< HEAD
<<<<<<< HEAD

    procedure UpdateIndexDefs(IndexDefs : TIndexDefs; TableName : string); virtual;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; virtual;
    function GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string; virtual;

    function GetObjectNames(ASchemaType: TSchemaType; AList : TSqlObjectIdentifierList): Integer; virtual;
    Procedure MaybeConnect;

    Property Statements : TFPList Read FStatements;
    property Port: cardinal read GetPort write SetPort;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); virtual;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; virtual;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); virtual; abstract;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; virtual;
    property port: cardinal read GetPort write Setport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  public
    property Handle: Pointer read GetHandle;
    property FieldNameQuoteChars: TQuoteChars read FFieldNameQuoteChars write FFieldNameQuoteChars;
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure StartTransaction; override;
    procedure EndTransaction; override;
    procedure ExecuteDirect(SQL : String); overload; virtual;
    procedure ExecuteDirect(SQL : String; ATransaction : TSQLTransaction); overload; virtual;
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
  published
    property Password : string read FPassword write FPassword;
    property Transaction : TSQLTransaction read FTransaction write SetTransaction;
    property UserName : string read FUserName write FUserName;
    property CharSet : string read FCharSet write FCharSet;
    property HostName : string Read FHostName Write FHostName;
    Property OnLog : TDBLogNotifyEvent Read FOnLog Write FOnLog;
    Property LogEvents : TDBEventTypes Read FLogEvents Write FLogEvents Default LogAllEvents;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property Options : TSQLConnectionOptions Read FOptions Write SetOptions;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    property Connected;
>>>>>>> graemeg/cpstrnew
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
    Property Options : TSQLTransactionOptions Read FOptions Write SetOptions;
  end;

<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD

  { TCustomSQLStatement }

  TCustomSQLStatement = Class(TComponent)
  Private
    FCursor : TSQLCursor;
    FDatabase: TSQLConnection;
    FParamCheck: Boolean;
    FParams: TParams;
    FSQL: TStrings;
    FOrigSQL : String;
    FServerSQL : String;
    FTransaction: TSQLTransaction;
    FParseSQL: Boolean;
    FDataLink : TDataLink;
    FRowsAffected : TRowsCount;
    procedure SetDatabase(AValue: TSQLConnection);
    procedure SetParams(AValue: TParams);
    procedure SetSQL(AValue: TStrings);
    procedure SetTransaction(AValue: TSQLTransaction);
    Function GetPrepared : Boolean;
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
    Property DataSource : TDataSource Read GetDataSource Write SetDataSource;
    Property ParseSQL : Boolean Read FParseSQL Write FParseSQL;
    Property ParamCheck : Boolean Read FParamCheck Write FParamCheck default true;
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


<<<<<<< HEAD
<<<<<<< HEAD
  { TCustomSQLQuery }

  TSQLQueryOption = (sqoKeepOpenOnCommit, sqoAutoApplyUpdates, sqoAutoCommit, sqoCancelUpdatesOnRefresh, sqoRefreshUsingSelect);
  TSQLQueryOptions = Set of TSQLQueryOption;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TCustomSQLQuery = class (TCustomBufDataset)
=======
{ TCustomSQLQuery }

  TCustomSQLQuery = class (Tbufdataset)
>>>>>>> graemeg/fixes_2_2
=======
{ TCustomSQLQuery }

  TCustomSQLQuery = class (Tbufdataset)
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
    FReadOnly            : boolean;
>>>>>>> graemeg/fixes_2_2
    FUpdateMode          : TUpdateMode;
    FusePrimaryKeyAsKey  : Boolean;
<<<<<<< HEAD
=======
    FSQLBuf              : String;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FWhereStartPos       : integer;
    FWhereStopPos        : integer;
    FServerFilterText    : string;
    FServerFiltered      : Boolean;
    
    FServerIndexDefs     : TServerIndexDefs;
<<<<<<< HEAD

<<<<<<< HEAD
    FServerIndexDefs     : TServerIndexDefs;
=======
>>>>>>> origin/fixes_2_2

    // Used by SetSchemaType
    FSchemaObjectName    : string;
    FSchemaPattern       : string;

    FInsertQry,
    FUpdateQry,
    FDeleteQry           : TCustomSQLQuery;
    FSequence            : TSQLSequence;
    procedure FreeFldBuffers;
    function GetParamCheck: Boolean;
    function GetParams: TParams;
    function GetParseSQL: Boolean;
    function GetServerIndexDefs: TServerIndexDefs;
    function GetSQL: TStringList;
    function GetSQLConnection: TSQLConnection;
    function GetSQLTransaction: TSQLTransaction;
    function GetStatementType : TStatementType;
    Function NeedLastInsertID: TField;
    procedure SetOptions(AValue: TSQLQueryOptions);
    procedure SetParamCheck(AValue: Boolean);
    procedure SetSQLConnection(AValue: TSQLConnection);
    procedure SetSQLTransaction(AValue: TSQLTransaction);
    procedure SetInsertSQL(const AValue: TStringList);
    procedure SetUpdateSQL(const AValue: TStringList);
    procedure SetDeleteSQL(const AValue: TStringList);
    procedure SetRefreshSQL(const AValue: TStringList);
    procedure SetParams(AValue: TParams);
    procedure SetParseSQL(AValue : Boolean);
    procedure SetSQL(const AValue: TStringList);
=======
    FUpdateQry,
    FDeleteQry,
    FInsertQry           : TCustomSQLQuery;
    FOpenDidPrepare : Boolean;
    procedure FreeFldBuffers;
    function GetServerIndexDefs: TServerIndexDefs;
    function GetStatementType : TStatementType;
    procedure SetDeleteSQL(const AValue: TStringlist);
    procedure SetInsertSQL(const AValue: TStringlist);
    procedure SetReadOnly(AValue : Boolean);
    procedure SetParseSQL(AValue : Boolean);
    procedure SetUpdateSQL(const AValue: TStringlist);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SetUsePrimaryKeyAsKey(AValue : Boolean);
    procedure SetUpdateMode(AValue : TUpdateMode);
    procedure OnChangeModifySQL(Sender : TObject);
    procedure Execute;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    Function SQLParser(const ASQL : string) : TStatementType;
>>>>>>> graemeg/cpstrnew
=======
    Function SQLParser(const ASQL : string) : TStatementType;
>>>>>>> graemeg/cpstrnew
=======
    Function SQLParser(const ASQL : string) : TStatementType;
>>>>>>> graemeg/cpstrnew
=======
    Function SQLParser(const ASQL : string) : TStatementType;
>>>>>>> origin/cpstrnew
    procedure ApplyFilter;
    Function AddFilter(SQLstr : string) : string;
  protected
    Function RefreshLastInsertID(Field: TField): Boolean; virtual;
    Function NeedRefreshRecord (UpdateKind: TUpdateKind): Boolean; virtual;
    Function RefreshRecord (UpdateKind: TUpdateKind) : Boolean; virtual;
    Procedure ApplyReturningResult(Q : TCustomSQLQuery; UpdateKind : TUpdateKind);
    Function Cursor : TSQLCursor;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
    // abstract & virtual methods of TBufDataset
    function Fetch : boolean; override;
    function LoadField(FieldDef : TFieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; override;
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
    Procedure SetActive (Value : Boolean); override;
    procedure SetServerFiltered(Value: Boolean); virtual;
    procedure SetServerFilterText(const Value: string); virtual;
<<<<<<< HEAD
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
=======
    Function GetDataSource : TDatasource; override;
    Procedure SetDataSource(AValue : TDatasource);
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField); override;
    procedure BeforeRefreshOpenCursor; override;
    Function LogEvent(EventType : TDBEventType) : Boolean;
    Procedure Log(EventType : TDBEventType; Const Msg : String); virtual;
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
  public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure Prepare; virtual;
    procedure UnPrepare; virtual;
    procedure ExecSQL; virtual;
<<<<<<< HEAD
    procedure SetSchemaInfo( ASchemaType : TSchemaType; ASchemaObjectName, ASchemaPattern : string); virtual;
    function RowsAffected: TRowsCount; virtual;
    function ParamByName(Const AParamName : String) : TParam;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property Prepared : boolean read IsPrepared;
    Property SQLConnection : TSQLConnection Read GetSQLConnection Write SetSQLConnection;
    Property SQLTransaction: TSQLTransaction Read GetSQLTransaction Write SetSQLTransaction;
    // overriden TBufDataSet methods
    Procedure ApplyUpdates(MaxErrors: Integer); override; overload;
    // overriden TDataSet methods
    Procedure Post; override;
    Procedure Delete; override;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  protected
    // redeclared TDataSet properties
    property Active;
    property Filter;
    property Filtered;
=======
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure SetSchemaInfo( SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string); virtual;
    property Prepared : boolean read IsPrepared;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    function RowsAffected: TRowsCount; virtual;
  protected
      
    // redeclared data set properties
    property Active;
    property Filter;
    property Filtered;
//    property FilterOptions;
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    property Transaction;
    property SchemaType : TSchemaType read FSchemaType default stNoSchema;
    property SQL : TStringlist read GetSQL write SetSQL;
    property InsertSQL : TStringList read FInsertSQL write SetInsertSQL;
    property UpdateSQL : TStringList read FUpdateSQL write SetUpdateSQL;
    property DeleteSQL : TStringList read FDeleteSQL write SetDeleteSQL;
    property RefreshSQL : TStringList read FRefreshSQL write SetRefreshSQL;
    Property Options : TSQLQueryOptions Read FOptions Write SetOptions;
    property Params : TParams read GetParams Write SetParams;
    Property ParamCheck : Boolean Read GetParamCheck Write SetParamCheck default true;
    property ParseSQL : Boolean read GetParseSQL write SetParseSQL default true;
    property UpdateMode : TUpdateMode read FUpdateMode write SetUpdateMode default upWhereKeyOnly;
    property UsePrimaryKeyAsKey : boolean read FUsePrimaryKeyAsKey write SetUsePrimaryKeyAsKey default true;
    property StatementType : TStatementType read GetStatementType;
    Property DataSource : TDataSource Read GetDataSource Write SetDataSource;
    property Sequence: TSQLSequence read FSequence write FSequence;
=======
=======
>>>>>>> origin/fixes_2_2
  // protected
//    property SchemaInfo : TSchemaInfo read FSchemaInfo default stNoSchema;
    property Transaction;
    property ReadOnly : Boolean read FReadOnly write SetReadOnly;
    property SQL : TStringlist read FSQL write FSQL;
    property UpdateSQL : TStringlist read FUpdateSQL write SetUpdateSQL;
    property InsertSQL : TStringlist read FInsertSQL write SetInsertSQL;
    property DeleteSQL : TStringlist read FDeleteSQL write SetDeleteSQL;
    property Params : TParams read FParams write FParams;
    property UpdateMode : TUpdateMode read FUpdateMode write SetUpdateMode;
    property UsePrimaryKeyAsKey : boolean read FUsePrimaryKeyAsKey write SetUsePrimaryKeyAsKey default true;
    property StatementType : TStatementType read GetStatementType;
    property ParseSQL : Boolean read FParseSQL write SetParseSQL default true;
    Property DataSource : TDatasource Read GetDataSource Write SetDatasource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property ServerFilter: string read FServerFilterText write SetServerFilterText;
    property ServerFiltered: Boolean read FServerFiltered write SetServerFiltered default False;
    property ServerIndexDefs : TServerIndexDefs read GetServerIndexDefs;
  end;

{ TSQLQuery }
  TSQLQuery = Class(TCustomSQLQuery)
<<<<<<< HEAD
<<<<<<< HEAD
  public
    property SchemaType;
    Property StatementType;
  Published
    property MaxIndexesCount;
   // TDataset stuff
    property FieldDefs;
=======
  Published
   // TDataset stuff
>>>>>>> graemeg/fixes_2_2
=======
  Published
   // TDataset stuff
>>>>>>> origin/fixes_2_2
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
    Property AfterScroll;
    Property BeforeCancel;
    Property BeforeClose;
    Property BeforeDelete;
    Property BeforeEdit;
    Property BeforeInsert;
    Property BeforeOpen;
    Property BeforePost;
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
<<<<<<< HEAD
<<<<<<< HEAD
    property InsertSQL;
    property UpdateSQL;
    property DeleteSQL;
    property RefreshSQL;
    property IndexDefs;
    Property Options;
    property Params;
    Property ParamCheck;
    property ParseSQL;
    property UpdateMode;
    property UsePrimaryKeyAsKey;
    Property DataSource;
    property Sequence;
=======
=======
>>>>>>> origin/fixes_2_2
    property UpdateSQL;
    property InsertSQL;
    property DeleteSQL;
    property IndexDefs;
    property Params;
    property UpdateMode;
    property UsePrimaryKeyAsKey;
    property ParseSQL;
    Property DataSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    procedure ExecuteCommit(CommitRetaining: boolean=true); override;
=======
    procedure ExecuteCommit; override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure ExecuteCommit; override;
>>>>>>> origin/fixes_2_2
    Procedure SetDatabase (Value : TDatabase); virtual;
    Procedure SetTransaction(Value : TDBTransaction); virtual;
    Procedure CheckDatabase;
  public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    procedure Execute; override;
    procedure ExecuteScript;
  published
    Property DataBase : TDatabase Read FDatabase Write SetDatabase;
    Property Transaction : TDBTransaction Read FTransaction Write SetTransaction;
    property OnDirective: TSQLScriptDirectiveEvent read FOnDirective write FOnDirective;
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
    function LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    procedure FreeFldBuffers(cursor : TSQLCursor); override;

=======
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
>>>>>>> graemeg/fixes_2_2
=======
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
>>>>>>> origin/fixes_2_2
    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function StartDBTransaction(trans : TSQLHandle; aParams : string) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    procedure RollBackRetaining(trans : TSQLHandle); override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs; TableName : string); override;
=======
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override;
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD

{ TSqlObjectIdentifierList }
=======
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

procedure TSQLConnection.UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function TSqlObjectIdentifierList.GetIdentifier(Index: integer): TSqlObjectIdenfier;
begin
  Result := Items[Index] as TSqlObjectIdenfier;
end;

procedure TSqlObjectIdentifierList.SetIdentifier(Index: integer; AValue: TSqlObjectIdenfier);
begin
  Items[Index] := AValue;
end;

function TSqlObjectIdentifierList.AddIdentifier: TSqlObjectIdenfier;
begin
  Result:=Add as TSqlObjectIdenfier;
end;

function TSqlObjectIdentifierList.AddIdentifier(Const AObjectName: String;
  Const ASchemaName: String = ''): TSqlObjectIdenfier;
begin
  Result:=AddIdentifier();
  Result.SchemaName:=ASchemaName;
  Result.ObjectName:=AObjectName;
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLStatement.SetDatabase(AValue: TSQLConnection);
=======
=======
>>>>>>> origin/fixes_2_2
function TSQLConnection.GetPort: cardinal;
begin
  result := StrToIntDef(Params.Values['Port'],0);
end;

procedure TSQLConnection.Setport(const AValue: cardinal);
begin
  if AValue<>0 then
    params.Values['Port']:=IntToStr(AValue)
  else with params do if IndexOfName('Port') > -1 then
    Delete(IndexOfName('Port'));
end;

procedure TSQLConnection.GetDBInfo(const SchemaType : TSchemaType; const SchemaObjectName, ReturnField : string; List: TStrings);

var qry : TCustomSQLQuery;

>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
  qry := TCustomSQLQuery.Create(nil);
  qry.transaction := Transaction;
  qry.database := Self;
  with qry do
    begin
    ParseSQL := False;
    SetSchemaInfo(SchemaType,SchemaObjectName,'');
    open;
    List.Clear;
    while not eof do
      begin
      List.Append(trim(fieldbyname(ReturnField).asstring));
      Next;
      end;
>>>>>>> graemeg/fixes_2_2
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLStatement.SetDataSource(AValue: TDataSource);
=======
function TSQLConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
=======
function TSQLConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  Result := -1;
end;

procedure TSQLConnection.GetTableNames(List: TStrings; SystemTables: Boolean);
>>>>>>> origin/fixes_2_2
begin
  Result := -1;
end;
>>>>>>> graemeg/fixes_2_2

begin
<<<<<<< HEAD
  if GetDataSource=AValue then Exit;
  if (FDataLink=Nil) then
    FDataLink:=CreateDataLink;
  FDataLink.DataSource:=AValue;
=======
  inherited Create(AOwner);
  FSQLServerFormatSettings.DecimalSeparator:='.';
  FFieldNameQuoteChars:=DoubleQuotes;
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
end;

Procedure TCustomSQLStatement.CopyParamsFromMaster(CopyBound: Boolean);
begin
  if Assigned(DataSource) and Assigned(DataSource.Dataset) then
    FParams.CopyParamValuesFromDataset(DataSource.Dataset,CopyBound);
end;

procedure TCustomSQLStatement.SetParams(AValue: TParams);
begin
  if FParams=AValue then Exit;
  FParams.Assign(AValue);
end;

procedure TCustomSQLStatement.SetSQL(AValue: TStrings);
begin
  if FSQL=AValue then Exit;
  FSQL.Assign(AValue);
end;

Procedure TCustomSQLStatement.DoExecute;
begin
  FRowsAffected:=-1;
  If (FParams.Count>0) and Assigned(DataSource) then
    CopyParamsFromMaster(False);
  If LogEvent(detExecute) then
    Log(detExecute,FServerSQL);
  Database.Execute(FCursor,Transaction, FParams);
end;

Function TCustomSQLStatement.GetPrepared: Boolean;
begin
  Result := Assigned(FCursor) and FCursor.FPrepared;
end;

Function TCustomSQLStatement.CreateDataLink: TDataLink;
begin
  Result:=TDataLink.Create;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomSQLStatement.CreateParams: TSQLDBParams;
=======
function TSQLConnection.LogEvent(EventType: TDBEventType): Boolean;
=======
function TSQLConnection.LogEvent(EventType: TDBEventType): Boolean;
=======
function TSQLConnection.LogEvent(EventType: TDBEventType): Boolean;
=======
function TSQLConnection.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=(Assigned(FOnLog) or Assigned(GlobalDBLogHook)) and (EventType in LogEvents);
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

procedure TSQLConnection.FreeFldBuffers(cursor: TSQLCursor);
>>>>>>> origin/cpstrnew
begin
  Result:=(Assigned(FOnLog) or Assigned(GlobalDBLogHook)) and (EventType in LogEvents);
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

procedure TSQLConnection.FreeFldBuffers(cursor: TSQLCursor);
>>>>>>> graemeg/cpstrnew
begin
  Result:=(Assigned(FOnLog) or Assigned(GlobalDBLogHook)) and (EventType in LogEvents);
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

procedure TSQLConnection.FreeFldBuffers(cursor: TSQLCursor);
>>>>>>> graemeg/cpstrnew
begin
  Result:=(Assigned(FOnLog) or Assigned(GlobalDBLogHook)) and (EventType in LogEvents);
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

procedure TSQLConnection.FreeFldBuffers(cursor: TSQLCursor);
>>>>>>> graemeg/cpstrnew
begin
  Result:=TSQLDBParams.Create(Nil);
end;

Function TCustomSQLStatement.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and Database.LogEvent(EventType);
end;

Procedure TCustomSQLStatement.Log(EventType: TDBEventType; Const Msg: String);
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
  FreeAndNil(FSQL);
  inherited Destroy;
end;

Function TCustomSQLStatement.GetSchemaType: TSchemaType;

begin
  Result:=stNoSchema
end;

Function TCustomSQLStatement.GetSchemaObjectName: String;
begin
  Result:='';
end;

Function TCustomSQLStatement.GetSchemaPattern: String;
begin
  Result:='';
end;

Function TCustomSQLStatement.IsSelectable: Boolean;
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
  FServerSQL:=FOrigSQL;
  GetStatementInfo(FServerSQL,StmInfo);
  AllocateCursor;
  FCursor.FSelectable:=True; // let PrepareStatement and/or Execute alter it
  FCursor.FStatementType:=StmInfo.StatementType;
  FCursor.FSchemaType:=GetSchemaType;
  If LogEvent(detPrepare) then
    Log(detPrepare,FServerSQL);
  Database.PrepareStatement(FCursor,Transaction,FServerSQL,FParams);
end;

Procedure TCustomSQLStatement.Prepare;

begin
  if Prepared then exit;
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

Procedure TCustomSQLStatement.Execute;
begin
  Prepare;
  DoExecute;
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

Procedure TCustomSQLStatement.Unprepare;
begin
  // Some SQLConnections does not support statement [un]preparation, but they have allocated local cursor(s)
  //  so let them do cleanup f.e. cancel pending queries and/or free resultset
  //  and also do UnRegisterStatement!
  if assigned(FCursor) then
    DoUnprepare;
end;

function TCustomSQLStatement.ParamByName(Const AParamName: String): TParam;
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
  FStatements:=TFPList.Create;
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

procedure TSQLConnection.DoInternalConnect;
begin
  if (DatabaseName = '') and not(sqSupportEmptyDatabaseName in FConnOptions) then
    DatabaseError(SErrNoDatabaseName,self);
end;

procedure TSQLConnection.DoInternalDisconnect;

Var
  I : integer;

begin
  For I:=0 to FStatements.Count-1 do
    TCustomSQLStatement(FStatements[i]).Unprepare;
  FStatements.Clear;
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
    If LogEvent(detExecute) then
      Log(detExecute,SQL);
    Execute(Cursor,ATransaction, Nil);
    UnPrepareStatement(Cursor);
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

function TSQLConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  Result := -1;
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

type TParsePart = (ppStart,ppWith,ppSelect,ppTableName,ppFrom,ppWhere,ppGroup,ppOrder,ppBogus);
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
  PSQL:=Pchar(ASQL);
  ParsePart := ppStart;

  CurrentP := PSQL-1;
  PhraseP := PSQL;

  Result.TableName := '';
  Result.Updateable := False;
  Result.WhereStartPos := 0;
  Result.WhereStopPos := 0;

  repeat
    begin
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

    if (CurrentP > SavedP) and (SavedP > PhraseP) then
      CurrentP := SavedP;  // there is something before comment or left parenthesis

    if Separator <> sepNone then
      begin
      if ((Separator in [sepWhitespace,sepComment]) and (PhraseP = SavedP)) then
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
                       Result.TableName := s;
                       Result.Updateable := True;
                       end;
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
    end;
  until CurrentP^=#0;
end;

function TSQLConnection.GetAsSQLText(Field : TField) : string;
begin
  if (not assigned(Field)) or Field.IsNull then Result := 'Null'
  else case field.DataType of
    ftString   : Result := QuotedStr(Field.AsString);
    ftDate     : Result := '''' + FormatDateTime('yyyy-mm-dd',Field.AsDateTime,FSqlFormatSettings) + '''';
    ftDateTime : Result := '''' + FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz',Field.AsDateTime,FSqlFormatSettings) + '''';
    ftTime     : Result := QuotedStr(TimeIntervalToString(Field.AsDateTime));
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
    ftString   : Result := QuotedStr(Param.AsString);
    ftDate     : Result := '''' + FormatDateTime('yyyy-mm-dd',Param.AsDateTime,FSQLFormatSettings) + '''';
    ftTime     : Result := QuotedStr(TimeIntervalToString(Param.AsDateTime));
    ftDateTime : Result := '''' + FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz', Param.AsDateTime, FSQLFormatSettings) + '''';
    ftCurrency,
    ftBcd      : Result := CurrToStr(Param.AsCurrency, FSQLFormatSettings);
    ftFloat    : Result := FloatToStr(Param.AsFloat, FSQLFormatSettings);
    ftFMTBcd   : Result := stringreplace(Param.AsString, DefaultFormatSettings.DecimalSeparator, FSQLFormatSettings.DecimalSeparator, []);
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
  if not LogEvent(detParamValue) then
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
  if FStatements.IndexOf(S)=-1 then
    FStatements.Add(S);
end;

procedure TSQLConnection.UnRegisterStatement(S: TCustomSQLStatement);
begin
  if Assigned(FStatements) then // Can be nil, when we are destroying and datasets are uncoupled.
    FStatements.Remove(S);
end;


function TSQLConnection.InitialiseUpdateStatement(Query : TCustomSQLQuery; var qry : TCustomSQLQuery): TCustomSQLQuery;

begin
  if not assigned(qry) then
  begin
    qry := TCustomSQLQuery.Create(nil);
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
  Result:=Query.RefreshSQL.Text;
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

procedure TSQLConnection.ApplyFieldUpdate(C : TSQLCursor; P : TSQLDBParam;F : TField; UseOldValue : Boolean);

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
  P : TParam;
  B,ReturningClause : Boolean;

begin
  qry:=Nil;
  ReturningClause:=(sqSupportReturning in ConnOptions) and not (sqoRefreshUsingSelect in Query.Options) and (Query.RefreshSQL.Count=0);
  case UpdateKind of
    ukInsert : begin
               s := Trim(Query.FInsertSQL.Text);
               if s = '' then
                 s := ConstructInsertSQL(Query, ReturningClause)
               else
                 ReturningClause := False;
               qry := InitialiseUpdateStatement(Query, Query.FInsertQry);
               end;
    ukModify : begin
               s := Trim(Query.FUpdateSQL.Text);
               if s = '' then begin
                 //if not assigned(Query.FUpdateQry) or (Query.UpdateMode<>upWhereKeyOnly) then // first time or dynamic where part
                   s := ConstructUpdateSQL(Query, ReturningClause);
               end
               else
                 ReturningClause := False;
               qry := InitialiseUpdateStatement(Query, Query.FUpdateQry);
               end;
    ukDelete : begin
               s := Trim(Query.FDeleteSQL.Text);
               if (s='') and (not assigned(Query.FDeleteQry) or (Query.UpdateMode<>upWhereKeyOnly)) then
                 s := ConstructDeleteSQL(Query);
               ReturningClause := False;
               qry := InitialiseUpdateStatement(Query, Query.FDeleteQry);
               end;
  end;
  if (s<>'') and (qry.SQL.Text<>s) then
    qry.SQL.Text:=s; //assign only when changed, to avoid UnPrepare/Prepare
  Assert(qry.SQL.Text<>'');
  for x:=0 to Qry.Params.Count-1 do
    begin
    P:=Qry.Params[x];
    S:=p.name;
    B:=SameText(leftstr(S,4),'OLD_');
    if B then
      Delete(S,1,4);
    Fld:=Query.FieldByName(S);
    ApplyFieldUpdate(Query.Cursor,P as TSQLDBParam,Fld,B);
    end;
  if ReturningClause then
    begin
    Qry.Close;
    Qry.Open
    end
  else
    Qry.Execute;
  if (scoApplyUpdatesChecksRowsAffected in Options) and (Qry.RowsAffected<>1) then
    begin
    Qry.Close;
    DatabaseErrorFmt(SErrFailedToUpdateRecord, [Qry.RowsAffected], Query);
    end;
  if ReturningClause then
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
<<<<<<< HEAD
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
<<<<<<< HEAD
  if (DS is TSQLQuery) then
    Result:=not (sqoKeepOpenOnCommit in TSQLQuery(DS).Options)
  else
    Result:=Inherited AllowClose(DS);
=======
  Result := TSQLConnection(Database).GetTransactionHandle(FTrans);
>>>>>>> graemeg/fixes_2_2
=======
  Result := TSQLConnection(Database).GetTransactionHandle(FTrans);
>>>>>>> origin/fixes_2_2
end;

procedure TSQLTransaction.Commit;
begin
<<<<<<< HEAD
  if Active  then
    begin
<<<<<<< HEAD
    CloseDataSets;
    If LogEvent(detCommit) then
      Log(detCommit,SCommitting);
    if (stoUseImplicit in Options) or SQLConnection.AttemptCommit(FTrans) then
=======
    closedatasets;
    if TSQLConnection(Database).commit(FTrans) then
>>>>>>> origin/fixes_2_2
      begin
      CloseTrans;
      FreeAndNil(FTrans);
      end;
    end;
end;

procedure TSQLTransaction.CommitRetaining;
begin
<<<<<<< HEAD
  if Active then
    begin
    If LogEvent(detCommit) then
      Log(detCommit,SCommitRetaining);
    SQLConnection.CommitRetaining(FTrans);
    end;
=======
  if active then
    TSQLConnection(Database).commitRetaining(FTrans);
>>>>>>> origin/fixes_2_2
end;

procedure TSQLTransaction.Rollback;
begin
  if Active then
    begin
<<<<<<< HEAD
    if (stoUseImplicit in Options) then
      DatabaseError(SErrImplicitNoRollBack);
    CloseDataSets;
    If LogEvent(detRollback) then
      Log(detRollback,SRollingBack);
    if SQLConnection.AttemptRollBack(FTrans) then
=======
    closedatasets;
    if TSQLConnection(Database).RollBack(FTrans) then
>>>>>>> origin/fixes_2_2
      begin
      CloseTrans;
      FreeAndNil(FTrans);
      end;
    end;
end;

procedure TSQLTransaction.RollbackRetaining;
begin
<<<<<<< HEAD
  if Active then
    begin
    if (stoUseImplicit in Options) then
      DatabaseError(SErrImplicitNoRollBack);
    If LogEvent(detRollback) then
      Log(detRollback,SRollBackRetaining);
    SQLConnection.RollBackRetaining(FTrans);
    end;
=======
  if active then
    TSQLConnection(Database).RollBackRetaining(FTrans);
>>>>>>> origin/fixes_2_2
end;

procedure TSQLTransaction.StartTransaction;

var db : TSQLConnection;

begin
  if Active then
    DatabaseError(SErrTransAlreadyActive);

<<<<<<< HEAD
  db := SQLConnection;
=======
  db := TSQLConnection(Database);
>>>>>>> origin/fixes_2_2

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
    Function CreateDataLink : TDataLink; override;
    Function GetSchemaType : TSchemaType; override;
    Function GetSchemaObjectName : String; override;
    Function GetSchemaPattern: String; override;
    procedure GetStatementInfo(var ASQL: String; out Info: TSQLStatementInfo); override;
    procedure OnChangeSQL(Sender : TObject); override;
  end;

{ TQuerySQLStatement }

function TQuerySQLStatement.CreateDataLink: TDataLink;
begin
  Result:=TMasterParamsDataLink.Create(FQuery);
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
=======
  if active then
    begin
    closedatasets;
<<<<<<< HEAD
    If LogEvent(detCommit) then
      Log(detCommit,SCommitting);
=======
>>>>>>> graemeg/fixes_2_2
    if TSQLConnection(Database).commit(FTrans) then
      begin
      closeTrans;
      FreeAndNil(FTrans);
      end;
    end;
>>>>>>> graemeg/cpstrnew
end;

function TQuerySQLStatement.GetSchemaPattern: String;
begin
<<<<<<< HEAD
  if Assigned(FQuery) then
    Result:=FQuery.FSchemaPattern
  else
    Result:=inherited GetSchemaPattern;
=======
  if active then
<<<<<<< HEAD
    begin
    If LogEvent(detCommit) then
      Log(detCommit,SCommitRetaining);
    TSQLConnection(Database).commitRetaining(FTrans);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

procedure TQuerySQLStatement.GetStatementInfo(var ASQL: String; out Info: TSQLStatementInfo);
begin
<<<<<<< HEAD
  inherited GetStatementInfo(ASQL, Info);
  If Assigned(FQuery) then
    // Note: practical side effect of switch off ParseSQL is that UpdateServerIndexDefs is bypassed
    //       which is used as performance tuning option
    if (FQuery.FSchemaType = stNoSchema) and FParseSQL then
=======
=======
=======
    TSQLConnection(Database).commitRetaining(FTrans);
>>>>>>> graemeg/fixes_2_2
end;

procedure TSQLTransaction.Rollback;
begin
>>>>>>> origin/cpstrnew
  if active then
    begin
    closedatasets;
<<<<<<< HEAD
    If LogEvent(detRollback) then
      Log(detRollback,SRollingBack);
    if TSQLConnection(Database).RollBack(FTrans) then
>>>>>>> graemeg/cpstrnew
=======
    if TSQLConnection(Database).RollBack(FTrans) then
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
  UnPrepare;
  inherited OnChangeSQL(Sender);
  If ParamCheck and Assigned(FDataLink) then
    (FDataLink as TMasterParamsDataLink).RefreshParamNames;
  FQuery.ServerIndexDefs.Updated:=false;
=======
  if active then
<<<<<<< HEAD
    begin
    If LogEvent(detRollback) then
      Log(detRollback,SRollBackRetaining);
    TSQLConnection(Database).RollBackRetaining(FTrans);
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
=======
    TSQLConnection(Database).RollBackRetaining(FTrans);
>>>>>>> graemeg/fixes_2_2
end;

{ TCustomSQLQuery }

constructor TCustomSQLQuery.Create(AOwner : TComponent);

Var
  F : TQuerySQLStatement;

begin
  inherited Create(AOwner);
  F:=TQuerySQLStatement.Create(Self);
  F.FQuery:=Self;
  FStatement:=F;

<<<<<<< HEAD
  FInsertSQL := TStringList.Create;
  FInsertSQL.OnChange := @OnChangeModifySQL;
  FUpdateSQL := TStringList.Create;
  FUpdateSQL.OnChange := @OnChangeModifySQL;
  FDeleteSQL := TStringList.Create;
  FDeleteSQL.OnChange := @OnChangeModifySQL;
  FRefreshSQL := TStringList.Create;
  FRefreshSQL.OnChange := @OnChangeModifySQL;
=======
  db := TSQLConnection(Database);
>>>>>>> graemeg/fixes_2_2

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

<<<<<<< HEAD
function TCustomSQLQuery.ParamByName(const AParamName: String): TParam;
=======
Procedure TSQLTransaction.SetDatabase(Value : TDatabase);

begin
  If Value<>Database then
    begin
    if assigned(value) and not (Value is TSQLConnection) then
      DatabaseErrorFmt(SErrNotASQLConnection,[value.Name],self);
    CheckInactive;
    If Assigned(Database) then
      with TSQLConnection(DataBase) do
        if Transaction = self then Transaction := nil;
    inherited SetDatabase(Value);
<<<<<<< HEAD
    If Assigned(Database) and not (csLoading in ComponentState) then
=======
    If Assigned(Database) then
>>>>>>> graemeg/fixes_2_2
      If (TSQLConnection(DataBase).Transaction=Nil) then
        TSQLConnection(DataBase).Transaction:=Self;
    end;
end;

<<<<<<< HEAD
function TSQLTransaction.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and TSQLConnection(Database).LogEvent(EventType);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
{ TCustomSQLQuery }
procedure TCustomSQLQuery.OnChangeSQL(Sender : TObject);
>>>>>>> graemeg/fixes_2_2

procedure TSQLTransaction.Log(EventType: TDBEventType; const Msg: String);

Var
  M : String;

begin
  If LogEVent(EventType) then
    begin
<<<<<<< HEAD
    If (Name<>'') then
      M:=Name+' : '+Msg
    else
      M:=Msg;
    TSQLConnection(Database).Log(EventType,M);
=======
    if assigned(value) and not (Value is TSQLConnection) then
      DatabaseErrorFmt(SErrNotASQLConnection,[value.Name],self);
    CheckInactive;
    If Assigned(Database) then
      with TSQLConnection(DataBase) do
        if Transaction = self then Transaction := nil;
    inherited SetDatabase(Value);
    If Assigned(Database) then
      If (TSQLConnection(DataBase).Transaction=Nil) then
        TSQLConnection(DataBase).Transaction:=Self;
>>>>>>> origin/fixes_2_2
    end;
end;

{ TCustomSQLQuery }
procedure TCustomSQLQuery.OnChangeSQL(Sender : TObject);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/fixes_2_2

procedure TSQLTransaction.Log(EventType: TDBEventType; const Msg: String);

Var
  M : String;

begin
  If LogEVent(EventType) then
    begin
<<<<<<< HEAD
    If (Name<>'') then
      M:=Name+' : '+Msg
=======
    if assigned(DataBase) then
      ConnOptions := TSQLConnection(DataBase).ConnOptions
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    else
      M:=Msg;
    TSQLConnection(Database).Log(EventType,M);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ TCustomSQLQuery }
procedure TCustomSQLQuery.OnChangeSQL(Sender : TObject);

var ConnOptions : TConnOptions;
>>>>>>> graemeg/cpstrnew

begin
  Result:=Params.ParamByName(AParamName);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomSQLQuery.ParamByName(Const AParamName : String) : TParam;

begin
  Result:=Params.ParamByName(AParamName);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TCustomSQLQuery.OnChangeModifySQL(Sender : TObject);

begin
  CheckInactive;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.SetDatabase(Value : TDatabase);

var DB : TSQLConnection;
=======
Procedure TCustomSQLQuery.SetTransaction(Value : TDBTransaction);
>>>>>>> graemeg/fixes_2_2

begin
  if Database = Value then Exit;
  if Assigned(Value) and not (Value is TSQLConnection) then
    DatabaseErrorFmt(SErrNotASQLConnection, [Value.Name], Self);
  UnPrepare;
  DB := TSQLConnection(Value);
  If Assigned(FStatement) then
    FStatement.Database := DB;
  inherited;
<<<<<<< HEAD
  if Assigned(DB) and Assigned(DB.Transaction) and (not Assigned(Transaction) or (Transaction.DataBase<>Database)) then
    Transaction := DB.Transaction;
end;

procedure TCustomSQLQuery.SetTransaction(Value: TDBTransaction);
=======
Procedure TCustomSQLQuery.SetTransaction(Value : TDBTransaction);
>>>>>>> origin/fixes_2_2

begin
  if Transaction = Value then Exit;
  UnPrepare;
  inherited;
<<<<<<< HEAD
  If Assigned(FStatement) then
    FStatement.Transaction := TSQLTransaction(Value);
  If Assigned(Transaction) and Assigned(Transaction.DataBase) and (Database<>Transaction.DataBase) then
    Database := Transaction.Database;
end;

function TCustomSQLQuery.IsPrepared: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
  If (Transaction<>Nil) and (Database=Nil) then
    Database:=TSQLTransaction(Transaction).Database;
end;

procedure TCustomSQLQuery.SetDatabase(Value : TDatabase);

var db : tsqlconnection;

begin
  if (Database <> Value) then
    begin
    if assigned(value) and not (Value is TSQLConnection) then
      DatabaseErrorFmt(SErrNotASQLConnection,[value.Name],self);
    UnPrepare;
    if assigned(FCursor) then TSQLConnection(DataBase).DeAllocateCursorHandle(FCursor);
    db := TSQLConnection(Value);
    inherited setdatabase(value);
    if assigned(value) and (Transaction = nil) and (Assigned(db.Transaction)) then
      transaction := Db.Transaction;
    OnChangeSQL(Self);
    end;
end;

Function TCustomSQLQuery.IsPrepared : Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  if Assigned(Fstatement) then
    Result := FStatement.Prepared
  else
    Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSQLQuery.AddFilter(SQLstr: string): string;
=======
Function TCustomSQLQuery.AddFilter(SQLstr : string) : string;
>>>>>>> graemeg/fixes_2_2
=======
Function TCustomSQLQuery.AddFilter(SQLstr : string) : string;
>>>>>>> origin/fixes_2_2

begin
  if (FWhereStartPos > 0) and (FWhereStopPos > 0) then
    begin
    system.insert('(',SQLstr,FWhereStartPos+1);
    system.insert(')',SQLstr,FWhereStopPos+1);
    end;

  if FWhereStartPos = 0 then
    SQLstr := SQLstr + ' where (' + ServerFilter + ')'
  else if FWhereStopPos > 0 then
<<<<<<< HEAD
<<<<<<< HEAD
    system.insert(' and ('+ServerFilter+') ',SQLstr,FWhereStopPos+2)
=======
    system.insert(' and ('+ServerFilter+') ',SQLstr,FWhereStopPos+1)
>>>>>>> graemeg/fixes_2_2
=======
    system.insert(' and ('+ServerFilter+') ',SQLstr,FWhereStopPos+1)
>>>>>>> origin/fixes_2_2
  else
    system.insert(' where ('+ServerFilter+') ',SQLstr,FWhereStartPos);
  Result := SQLstr;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSQLQuery.NeedRefreshRecord(UpdateKind: TUpdateKind): Boolean;
=======
procedure TCustomSQLQuery.ApplyFilter;
>>>>>>> graemeg/fixes_2_2


Var
  PF : TProviderFlag;
  I : Integer;
  DoReturning : Boolean;

begin
<<<<<<< HEAD
  Result:=(FRefreshSQL.Count<>0);
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
=======
procedure TCustomSQLQuery.ApplyFilter;
>>>>>>> origin/fixes_2_2

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
=======
  FreeFldBuffers;
  TSQLConnection(Database).UnPrepareStatement(FCursor);
  FIsEOF := False;
  inherited internalclose;
>>>>>>> graemeg/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.ApplyFilter;
=======
  TSQLConnection(Database).PrepareStatement(Fcursor,(transaction as tsqltransaction),S,FParams);
>>>>>>> graemeg/fixes_2_2
=======
  TSQLConnection(Database).PrepareStatement(Fcursor,(transaction as tsqltransaction),S,FParams);
>>>>>>> origin/fixes_2_2

begin
  FreeFldBuffers;
  FStatement.Unprepare;
  FIsEOF := False;
  inherited InternalClose;
  FStatement.DoPrepare;
  FStatement.DoExecute;
  inherited InternalOpen;
  First;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.SetActive(Value: Boolean);
=======
Procedure TCustomSQLQuery.SetActive (Value : Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TCustomSQLQuery.SetActive (Value : Boolean);
>>>>>>> origin/fixes_2_2

begin
  inherited SetActive(Value);
// The query is UnPrepared, so that if a transaction closes all datasets
// they also get unprepared
  if not Value and IsPrepared then UnPrepare;
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

<<<<<<< HEAD
<<<<<<< HEAD

procedure TCustomSQLQuery.Prepare;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomSQLQuery.Prepare;
var
  db    : tsqlconnection;
  sqltr : tsqltransaction;
>>>>>>> graemeg/fixes_2_2

begin
<<<<<<< HEAD
  FStatement.Prepare;
  if Assigned(FStatement.FCursor) then
    with FStatement.FCursor do
      FInitFieldDef := FSelectable;
=======
  if not IsPrepared then
    begin
    db := TSQLConnection(Database);
    sqltr := (transaction as tsqltransaction);
    if not assigned(Db) then
      DatabaseError(SErrDatabasenAssigned);
    if not assigned(sqltr) then
      DatabaseError(SErrTransactionnSet);

    if not Db.Connected then db.Open;
    if not sqltr.Active then sqltr.StartTransaction;

    if FSchemaType=stNoSchema then
      FSQLBuf := TrimRight(FSQL.Text)
    else
      FSQLBuf := db.GetSchemaInfoSQL(FSchemaType, FSchemaObjectName, FSchemaPattern);

    if FSQLBuf = '' then
      DatabaseError(SErrNoStatement);

    StmType:=SQLParser(FSQLBuf);

    // There may no error occur between the allocation of the cursor and
    // the preparation of the cursor. Because internalclose (which is called in
    // case of an exception) assumes that allocated cursors are also prepared,
    // and thus calls unprepare.
    // A call to unprepare while the cursor is not prepared at all can lead to
    // unpredictable results.
    if not assigned(fcursor) then
      FCursor := Db.AllocateCursorHandle;
    FCursor.FStatementType:=StmType;
    FCursor.FSchemaType := FSchemaType;
    if ServerFiltered then
      begin
      If LogEvent(detprepare) then
        Log(detPrepare,AddFilter(FSQLBuf));
      Db.PrepareStatement(Fcursor,sqltr,AddFilter(FSQLBuf),FParams)
      end
    else
<<<<<<< HEAD
      begin
      If LogEvent(detprepare) then
        Log(detPrepare,FSQLBuf);
      Db.PrepareStatement(Fcursor,sqltr,FSQLBuf,FParams);
      end;
    if (FCursor.FStatementType in [stSelect,stExecProcedure]) then
=======
      Db.PrepareStatement(Fcursor,sqltr,FSQLBuf,FParams);

    if (FCursor.FStatementType = stSelect) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      FCursor.FInitFieldDef := True;
    end;
>>>>>>> graemeg/cpstrnew
end;

procedure TCustomSQLQuery.UnPrepare;

begin
  CheckInactive;
<<<<<<< HEAD
<<<<<<< HEAD
  If Assigned(FStatement) then
    FStatement.Unprepare;
end;

procedure TCustomSQLQuery.FreeFldBuffers;
begin
  if assigned(Cursor) then
     SQLConnection.FreeFldBuffers(Cursor);
end;

function TCustomSQLQuery.GetParamCheck: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
  if IsPrepared then with TSQLConnection(DataBase) do
    UnPrepareStatement(FCursor);
end;

procedure TCustomSQLQuery.FreeFldBuffers;
begin
  if assigned(FCursor) then TSQLConnection(Database).FreeFldBuffers(FCursor);
end;

function TCustomSQLQuery.GetServerIndexDefs: TServerIndexDefs;
<<<<<<< HEAD
begin
  Result := FServerIndexDefs;
end;

function TCustomSQLQuery.Fetch : boolean;
>>>>>>> graemeg/fixes_2_2
begin
  Result:=FStatement.ParamCheck;
end;

function TCustomSQLQuery.GetParams: TParams;
begin
  Result:=FStatement.Params;
end;

function TCustomSQLQuery.GetParseSQL: Boolean;
begin
<<<<<<< HEAD
  Result:=FStatement.ParseSQL;
=======
  If (FParams.Count>0) and Assigned(FMasterLink) then
    FMasterLink.CopyParamsFromMaster(False);
  If LogEvent(detExecute) then
    Log(detExecute,FSQLBuf);
  TSQLConnection(Database).execute(Fcursor,Transaction as tsqltransaction, FParams);
>>>>>>> graemeg/cpstrnew
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
=======
begin
  Result := FServerIndexDefs;
end;

function TCustomSQLQuery.Fetch : boolean;
>>>>>>> origin/fixes_2_2
begin
  Result:=Transaction as TSQLTransaction;
end;

function TCustomSQLQuery.Cursor: TSQLCursor;
begin
  Result:=FStatement.Cursor;
end;

<<<<<<< HEAD
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  if not FIsEof then FIsEOF := not TSQLConnection(Database).Fetch(Fcursor);
  Result := not FIsEOF;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TCustomSQLQuery.Execute;
begin
<<<<<<< HEAD
  FStatement.Execute;
=======
  If (FParams.Count>0) and Assigned(FMasterLink) then
    FMasterLink.CopyParamsFromMaster(False);
<<<<<<< HEAD
<<<<<<< HEAD
  If LogEvent(detExecute) then
    Log(detExecute,FSQLBuf);
  TSQLConnection(Database).execute(Fcursor,Transaction as tsqltransaction, FParams);
>>>>>>> graemeg/cpstrnew
end;
=======
=======
>>>>>>> graemeg/cpstrnew
function TCustomSQLQuery.SQLParser(const ASQL : string) : TStatementType;

type TParsePart = (ppStart,ppSelect,ppWhere,ppFrom,ppOrder,ppComment,ppGroup,ppBogus);

Var
  PSQL,CurrentP,
  PhraseP, PStatementPart : pchar;
  S                       : string;
  ParsePart               : TParsePart;
  StrLength               : Integer;
  EndOfComment            : Boolean;
  BracketCount            : Integer;
  ConnOptions             : TConnOptions;
  FFromPart               : String;
>>>>>>> graemeg/cpstrnew

function TCustomSQLQuery.RowsAffected: TRowsCount;
begin
  Result:=FStatement.RowsAffected;
end;

function TCustomSQLQuery.LoadField(FieldDef : TFieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;
begin
  result := SQLConnection.LoadField(Cursor,FieldDef,buffer, Createblob)
end;

procedure TCustomSQLQuery.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField);
begin
  SQLConnection.LoadBlobIntoBuffer(FieldDef, ABlobBuf, Cursor,SQLTransaction);
=======
  TSQLConnection(Database).execute(Fcursor,Transaction as tsqltransaction, FParams);
end;

function TCustomSQLQuery.LoadField(FieldDef : TFieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;
=======
  TSQLConnection(Database).execute(Fcursor,Transaction as tsqltransaction, FParams);
end;

function TCustomSQLQuery.LoadField(FieldDef : TFieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;
>>>>>>> origin/fixes_2_2

begin
  result := TSQLConnection(Database).LoadField(FCursor,FieldDef,buffer, Createblob)
end;

function TCustomSQLQuery.RowsAffected: TRowsCount;
begin
  Result := -1;
  if not Assigned(Database) then Exit;
  //assert(Database is TSQLConnection);
  Result := TSQLConnection(Database).RowsAffected(FCursor);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TCustomSQLQuery.InternalAddRecord(Buffer: Pointer; AAppend: Boolean);
begin
  // not implemented - sql dataset
end;

procedure TCustomSQLQuery.InternalClose;
begin
<<<<<<< HEAD
  if assigned(Cursor) then
    begin
<<<<<<< HEAD
    if Cursor.FSelectable then
      FreeFldBuffers;
    // Some SQLConnections does not support statement [un]preparation,
    //  so let them do cleanup f.e. cancel pending queries and/or free resultset
    if not Prepared then FStatement.DoUnprepare;
    end;

=======
  if StatementType = stSelect then FreeFldBuffers;
// Database and FCursor could be nil, for example if the database is not assigned, and .open is called
  if (FOpenDidPrepare) and (assigned(database)) and (assigned(FCursor)) then
    begin
    FOpenDidPrepare:=False;
    TSQLConnection(database).UnPrepareStatement(FCursor);
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
    Prepare;
    SQLConnection.AddFieldDefs(Cursor,FieldDefs);
=======

    TSQLConnection(Database).AddFieldDefs(fcursor,FieldDefs);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  finally
    FLoadingFieldDefs := False;
    if assigned(Cursor) then Cursor.FInitFieldDef := False;
  end;
<<<<<<< HEAD
=======
=======
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSQLQuery.SQLParser(const ASQL : string) : TStatementType;
=======
procedure TCustomSQLQuery.SQLParser(var ASQL : string);
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomSQLQuery.SQLParser(var ASQL : string);
>>>>>>> origin/fixes_2_2

type TParsePart = (ppStart,ppSelect,ppWhere,ppFrom,ppOrder,ppComment,ppGroup,ppBogus);

Var
  PSQL,CurrentP,
  PhraseP, PStatementPart : pchar;
  S                       : string;
  ParsePart               : TParsePart;
  StrLength               : Integer;
  EndOfComment            : Boolean;
  BracketCount            : Integer;
  ConnOptions             : TConnOptions;
  FFromPart               : String;

begin
  PSQL:=Pchar(ASQL);
  ParsePart := ppStart;

  CurrentP := PSQL-1;
  PhraseP := PSQL;

  FWhereStartPos := 0;
  FWhereStopPos := 0;
<<<<<<< HEAD

=======
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ConnOptions := TSQLConnection(DataBase).ConnOptions;
  FUpdateable := False;

  repeat
    begin
>>>>>>> graemeg/cpstrnew
    inc(CurrentP);

    EndOfComment := SkipComments(CurrentP,sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions);
    if EndOfcomment then dec(currentp);
    if EndOfComment and (ParsePart = ppStart) then PhraseP := CurrentP;

    // skip everything between bracket, since it could be a sub-select, and
    // further nothing between brackets could be interesting for the parser.
    if currentp^='(' then
      begin
      inc(currentp);
      BracketCount := 0;
      while (currentp^ <> #0) and ((currentp^ <> ')') or (BracketCount > 0 )) do
        begin
        if currentp^ = '(' then inc(bracketcount)
        else if currentp^ = ')' then dec(bracketcount);
        inc(currentp);
        end;
      EndOfComment := True;
      end;

    if EndOfComment or (CurrentP^ in [' ',#13,#10,#9,#0,';']) then
      begin
      if (CurrentP-PhraseP > 0) or (CurrentP^ in [';',#0]) then
        begin
        strLength := CurrentP-PhraseP;
        Setlength(S,strLength);
        if strLength > 0 then Move(PhraseP^,S[1],(strLength));
        s := uppercase(s);

        case ParsePart of
          ppStart  : begin
<<<<<<< HEAD
<<<<<<< HEAD
                     Result := TSQLConnection(Database).StrToStatementType(s);
                     if Result = stSelect then ParsePart := ppSelect
=======
=======
>>>>>>> origin/fixes_2_2
                     FCursor.FStatementType := TSQLConnection(Database).StrToStatementType(s);
                     if FCursor.FStatementType = stSelect then ParsePart := ppSelect
>>>>>>> graemeg/fixes_2_2
                       else break;
                     if not FParseSQL then break;
                     PStatementPart := CurrentP;
                     end;
          ppSelect : begin
                     if s = 'FROM' then
                       begin
                       ParsePart := ppFrom;
                       PhraseP := CurrentP;
                       PStatementPart := CurrentP;
                       end;
                     end;
          ppFrom   : begin
                     if (s = 'WHERE') or (s = 'ORDER') or (s = 'GROUP') or (s = 'LIMIT') or (CurrentP^=#0) or (CurrentP^=';') then
                       begin
                       if (s = 'WHERE') then
                         begin
                         ParsePart := ppWhere;
                         StrLength := PhraseP-PStatementPart;
                         end
                       else if (s = 'GROUP') then
                         begin
                         ParsePart := ppGroup;
                         StrLength := PhraseP-PStatementPart;
                         end
                       else if (s = 'ORDER') then
                         begin
                         ParsePart := ppOrder;
                         StrLength := PhraseP-PStatementPart
                         end
                       else if (s = 'LIMIT') then
                         begin
                         ParsePart := ppBogus;
                         StrLength := PhraseP-PStatementPart
                         end
                       else
                         begin
                         ParsePart := ppBogus;
                         StrLength := CurrentP-PStatementPart;
                         end;
<<<<<<< HEAD
<<<<<<< HEAD
                       if Result = stSelect then
=======
                       if FCursor.FStatementType = stSelect then
>>>>>>> graemeg/fixes_2_2
=======
                       if FCursor.FStatementType = stSelect then
>>>>>>> origin/fixes_2_2
                         begin
                         Setlength(FFromPart,StrLength);
                         Move(PStatementPart^,FFromPart[1],(StrLength));
                         FFrompart := trim(FFrompart);

<<<<<<< HEAD
<<<<<<< HEAD
                         // Meta-data requests and are never updateable select-statements
                         // from more then one table are not updateable
                         if (FSchemaType=stNoSchema) and
                            (ExtractStrings([',',' '],[],pchar(FFromPart),nil) = 1) then
=======
                         // select-statements from more then one table are not updateable
                         if ExtractStrings([',',' '],[],pchar(FFromPart),nil) = 1 then
>>>>>>> graemeg/fixes_2_2
=======
                         // select-statements from more then one table are not updateable
                         if ExtractStrings([',',' '],[],pchar(FFromPart),nil) = 1 then
>>>>>>> origin/fixes_2_2
                           begin
                           FUpdateable := True;
                           FTableName := FFromPart;
                           end;
                         end;

                       FWhereStartPos := PStatementPart-PSQL+StrLength+1;
                       PStatementPart := CurrentP;
                       end;
                     end;
          ppWhere  : begin
                     if (s = 'ORDER') or (s = 'GROUP') or (s = 'LIMIT') or (CurrentP^=#0) or (CurrentP^=';') then
                       begin
                       ParsePart := ppBogus;
                       FWhereStartPos := PStatementPart-PSQL;
                       if (s = 'ORDER') or (s = 'GROUP') or (s = 'LIMIT') then
                         FWhereStopPos := PhraseP-PSQL+1
                       else
                         FWhereStopPos := CurrentP-PSQL+1;
                       end
                     else if (s = 'UNION') then
                       begin
                       ParsePart := ppBogus;
                       FUpdateable := False;
                       end;
                     end;
        end; {case}
        end;
      PhraseP := CurrentP+1;
      end
    end;
  until CurrentP^=#0;
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
end;

procedure TCustomSQLQuery.InternalOpen;
<<<<<<< HEAD
<<<<<<< HEAD

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
    Prepare;
    if not Cursor.FSelectable then
      DatabaseError(SErrNoSelectStatement,Self);

    // Call UpdateServerIndexDefs before Execute, to avoid problems with connections
    // which do not allow processing multiple recordsets at a time. (Microsoft
    // calls this MARS, see bug 13241)
    if DefaultFields and FUpdateable and FusePrimaryKeyAsKey and (not IsUniDirectional) then
      UpdateServerIndexDefs;

    Execute;
    if not Cursor.FSelectable then
      DatabaseError(SErrNoSelectStatement,Self);

    // InternalInitFieldDef is only called after a prepare. i.e. not twice if
    // a dataset is opened - closed - opened.
    if Cursor.FInitFieldDef then InternalInitFieldDefs;
    if DefaultFields then
      begin
<<<<<<< HEAD
      CreateFields;

      if FUpdateable and FusePrimaryKeyAsKey and (not IsUniDirectional) then
        for counter := 0 to ServerIndexDefs.Count-1 do
          if ixPrimary in ServerIndexDefs[counter].Options then
=======
      // Call UpdateServerIndexDefs before Execute, to avoid problems with connections
      // which do not allow processing multiple recordsets at a time. (Microsoft
      // calls this MARS, see bug 13241)
      if DefaultFields and FUpdateable and FusePrimaryKeyAsKey and (not IsUniDirectional) then
        UpdateServerIndexDefs;
      Execute;
      // InternalInitFieldDef is only called after a prepare. i.e. not twice if
      // a dataset is opened - closed - opened.
      if FCursor.FInitFieldDef then InternalInitFieldDefs;
      if DefaultFields then
        begin
        CreateFields;

        if FUpdateable and (not IsUniDirectional) then
          begin
          if FusePrimaryKeyAsKey then
>>>>>>> graemeg/cpstrnew
            begin
            IndexFields := TStringList.Create;
            ExtractStrings([';'],[' '],pchar(ServerIndexDefs[counter].Fields),IndexFields);
            for fieldc := 0 to IndexFields.Count-1 do
              begin
              F := FindField(IndexFields[fieldc]);
              if F <> nil then
                F.ProviderFlags := F.ProviderFlags + [pfInKey];
=======
=======
>>>>>>> origin/fixes_2_2

var tel, fieldc : integer;
    f           : TField;
    s           : string;
    IndexFields : TStrings;
    ReadFromFile: Boolean;
begin
  try
    ReadFromFile:=IsReadFromPacket;
    FOpenDidPrepare:=Not Prepared;
    If FOpenDidPrepare then
      Prepare;
    if FCursor.FStatementType in [stSelect] then
      begin
      if not ReadFromFile then
        begin
        Execute;
        // InternalInitFieldDef is only called after a prepare. i.e. not twice if
        // a dataset is opened - closed - opened.
        if FCursor.FInitFieldDef then InternalInitFieldDefs;
        if DefaultFields then
          begin
          CreateFields;

          if FUpdateable then
            begin
            if FusePrimaryKeyAsKey then
              begin
              UpdateServerIndexDefs;
              for tel := 0 to ServerIndexDefs.count-1 do
                begin
                if ixPrimary in ServerIndexDefs[tel].options then
                  begin
                    IndexFields := TStringList.Create;
                    ExtractStrings([';'],[' '],pchar(ServerIndexDefs[tel].fields),IndexFields);
                    for fieldc := 0 to IndexFields.Count-1 do
                      begin
                      F := Findfield(IndexFields[fieldc]);
                      if F <> nil then
                        F.ProviderFlags := F.ProviderFlags + [pfInKey];
                      end;
                    IndexFields.Free;
                  end;
                end;
>>>>>>> graemeg/fixes_2_2
              end;
            IndexFields.Free;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
      end;
    end;
  BindFields(True);

  if not ReadOnly and not FUpdateable and (FSchemaType=stNoSchema) then
    begin
    if (trim(FDeleteSQL.Text) <> '') or (trim(FUpdateSQL.Text) <> '') or
       (trim(FInsertSQL.Text) <> '') then FUpdateable := True;
    end;

=======
=======
>>>>>>> origin/fixes_2_2
          end
        else
          BindFields(True);
        end
      else
        BindFields(True);
      if not ReadOnly and not FUpdateable then
        begin
        if (trim(FDeleteSQL.Text) <> '') or (trim(FUpdateSQL.Text) <> '') or
           (trim(FInsertSQL.Text) <> '') then FUpdateable := True;
        end
      end
    else
      DatabaseError(SErrNoSelectStatement,Self);
  except
    on E:Exception do
      raise;
  end;
>>>>>>> graemeg/fixes_2_2
  inherited InternalOpen;
end;

procedure TCustomSQLQuery.InternalRefresh;
begin
  if (ChangeCount>0) and (sqoCancelUpdatesOnRefresh in Options) then
    CancelUpdates;
  inherited InternalRefresh;
end;

// public part

procedure TCustomSQLQuery.ExecSQL;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

Var
  ExecDidPrepare : Boolean;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  try
    ExecDidPrepare:=Not IsPrepared;
    If ExecDidPrepare then
      Prepare;
    Execute;
    If sqoAutoCommit in Options then
      begin
      // Retrieve rows affected
      FStatement.RowsAffected;
      SQLTransaction.Commit;
      end;
  finally
<<<<<<< HEAD
    // Cursor has to be assigned, or else the prepare went wrong before PrepareStatment was
    //   called, so UnPrepareStatement shoudn't be called either
    // Don't deallocate cursor; f.e. RowsAffected is requested later
    if not Prepared and (assigned(Database)) and (assigned(Cursor)) then SQLConnection.UnPrepareStatement(Cursor);
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
=======
    // FCursor has to be assigned, or else the prepare went wrong before PrepareStatment was
    // called, so UnPrepareStatement shoudn't be called either
    if (ExecDidPrepare) and (assigned(database)) and (assigned(FCursor)) then
      TSQLConnection(database).UnPrepareStatement(Fcursor);
  end;
end;

constructor TCustomSQLQuery.Create(AOwner : TComponent);
begin
  inherited Create(AOwner);
  FParams := TParams.create(self);
  FSQL := TStringList.Create;
  FSQL.OnChange := @OnChangeSQL;

  FUpdateSQL := TStringList.Create;
  FUpdateSQL.OnChange := @OnChangeModifySQL;
  FInsertSQL := TStringList.Create;
  FInsertSQL.OnChange := @OnChangeModifySQL;
  FDeleteSQL := TStringList.Create;
  FDeleteSQL.OnChange := @OnChangeModifySQL;

  FServerIndexDefs := TServerIndexDefs.Create(Self);

  FReadOnly := false;
  FParseSQL := True;
  
  FServerFiltered := False;
  FServerFilterText := '';
  
// Delphi has upWhereAll as default, but since strings and oldvalue's don't work yet
// (variants) set it to upWhereKeyOnly
  FUpdateMode := upWhereKeyOnly;
  FUsePrimaryKeyAsKey := True;
end;

destructor TCustomSQLQuery.Destroy;
begin
  if Active then Close;
  UnPrepare;
  if assigned(FCursor) then TSQLConnection(Database).DeAllocateCursorHandle(FCursor);
  FreeAndNil(FMasterLink);
  FreeAndNil(FParams);
  FreeAndNil(FSQL);
  FreeAndNil(FInsertSQL);
  FreeAndNil(FDeleteSQL);
  FreeAndNil(FUpdateSQL);
  FServerIndexDefs.Free;
  inherited Destroy;
>>>>>>> graemeg/fixes_2_2
end;

procedure TCustomSQLQuery.SetReadOnly(AValue : Boolean);

begin
  CheckInactive;
<<<<<<< HEAD
<<<<<<< HEAD
  inherited SetReadOnly(AValue);
=======
  FReadOnly:=AValue;
>>>>>>> graemeg/fixes_2_2
=======
  FReadOnly:=AValue;
>>>>>>> origin/fixes_2_2
end;

procedure TCustomSQLQuery.SetParseSQL(AValue : Boolean);

begin
  CheckInactive;
  FStatement.ParseSQL:=AValue;
  if not AValue then
<<<<<<< HEAD
=======
    begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FServerFiltered := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.SetSQL(const AValue: TStringList);
begin
  FStatement.SQL.Assign(AValue);
=======
procedure TCustomSQLQuery.SetUpdateSQL(const AValue: TStringlist);
begin
  FUpdateSQL.Assign(AValue);
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomSQLQuery.SetUpdateSQL(const AValue: TStringlist);
begin
  FUpdateSQL.Assign(AValue);
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.UpdateServerIndexDefs;
=======
Procedure TCustomSQLQuery.UpdateServerIndexDefs;
>>>>>>> graemeg/fixes_2_2
=======
Procedure TCustomSQLQuery.UpdateServerIndexDefs;
>>>>>>> origin/fixes_2_2

begin
  FServerIndexDefs.Clear;
  if assigned(DataBase) and (FTableName<>'') then
<<<<<<< HEAD
<<<<<<< HEAD
    SQLConnection.UpdateIndexDefs(ServerIndexDefs,FTableName);
end;

function TCustomSQLQuery.NeedLastInsertID: TField;
=======
=======
>>>>>>> origin/fixes_2_2
    TSQLConnection(DataBase).UpdateIndexDefs(ServerIndexDefs,FTableName);
end;

Procedure TCustomSQLQuery.ApplyRecUpdate(UpdateKind : TUpdateKind);

var FieldNamesQuoteChar : char;

  procedure InitialiseModifyQuery(var qry : TCustomSQLQuery; aSQL: String);

  begin
    qry := TCustomSQLQuery.Create(nil);
    with qry do
      begin
      ParseSQL := False;
      DataBase := Self.DataBase;
      Transaction := Self.Transaction;
      SQL.text := aSQL;
      end;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Var
  I : Integer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var FieldNamesQuoteChars : TQuoteChars;

  procedure InitialiseModifyQuery(var qry : TCustomSQLQuery; aSQL: String);
>>>>>>> graemeg/cpstrnew

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
=======
>>>>>>> origin/fixes_2_2

function TCustomSQLQuery.RefreshLastInsertID(Field: TField): Boolean;

<<<<<<< HEAD
begin
  Result:=SQLConnection.RefreshLastInsertID(Self, Field);
end;
=======
  begin
    if (pfInKey in Fields[x].ProviderFlags) or
       ((FUpdateMode = upWhereAll) and (pfInWhere in Fields[x].ProviderFlags)) or
       ((FUpdateMode = UpWhereChanged) and (pfInWhere in Fields[x].ProviderFlags) and (fields[x].value <> fields[x].oldvalue)) then
<<<<<<< HEAD
<<<<<<< HEAD
      sql_where := sql_where + '(' + FieldNamesQuoteChars[0] + fields[x].FieldName + FieldNamesQuoteChars[1] + '= :"' + 'OLD_' + fields[x].FieldName + '") and ';
=======
      sql_where := sql_where + '(' + FieldNamesQuoteChar + fields[x].FieldName + FieldNamesQuoteChar + '= :OLD_' + fields[x].FieldName + ') and ';
>>>>>>> graemeg/fixes_2_2
=======
      sql_where := sql_where + '(' + FieldNamesQuoteChar + fields[x].FieldName + FieldNamesQuoteChar + '= :OLD_' + fields[x].FieldName + ') and ';
>>>>>>> origin/fixes_2_2
  end;
>>>>>>> graemeg/cpstrnew

procedure TCustomSQLQuery.ApplyRecUpdate(UpdateKind: TUpdateKind);

Var
  DoRefresh : Boolean;
  LastIDField : TField;
  S : TDataSetState;

begin
  // Moved to connection: the SQLConnection always has more information about types etc.
  // than SQLQuery itself.
  SQLConnection.ApplyRecUpdate(Self,UpdateKind);

<<<<<<< HEAD
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
=======
      if (pfInUpdate in Fields[x].ProviderFlags) then
<<<<<<< HEAD
<<<<<<< HEAD
        sql_set := sql_set +FieldNamesQuoteChars[0] + fields[x].FieldName + FieldNamesQuoteChars[1] +'=:"' + fields[x].FieldName + '",';
=======
        sql_set := sql_set +FieldNamesQuoteChar + fields[x].FieldName + FieldNamesQuoteChar +'=:' + fields[x].FieldName + ',';
>>>>>>> graemeg/fixes_2_2
=======
        sql_set := sql_set +FieldNamesQuoteChar + fields[x].FieldName + FieldNamesQuoteChar +'=:' + fields[x].FieldName + ',';
>>>>>>> origin/fixes_2_2
      end;
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.SetPacketRecords(aValue: integer);
begin
  if (AValue=PacketRecords) then exit;
  if (AValue<>-1) and (sqoKeepOpenOnCommit in Options) then
    DatabaseError(SErrDisconnectedPacketRecords);
  Inherited SetPacketRecords(aValue);
end;
=======
=======
>>>>>>> origin/fixes_2_2
    if length(sql_set) = 0 then DatabaseErrorFmt(sNoUpdateFields,['update'],self);
    setlength(sql_set,length(sql_set)-1);
    if length(sql_where) = 0 then DatabaseErrorFmt(sNoWhereFields,['update'],self);
    setlength(sql_where,length(sql_where)-5);
    result := 'update ' + FTableName + ' set ' + sql_set + ' where ' + sql_where;
>>>>>>> graemeg/fixes_2_2


function TCustomSQLQuery.GetCanModify: Boolean;

begin
  // the test for assigned(Cursor) is needed for the case that the dataset isn't opened
  if assigned(Cursor) and (Cursor.FStatementType = stSelect) then
    Result:= FUpdateable and (not ReadOnly) and (not IsUniDirectional)
  else
    Result := False;
end;

<<<<<<< HEAD
procedure TCustomSQLQuery.SetUpdateMode(AValue : TUpdateMode);
=======
  begin
    sql_fields := '';
    sql_values := '';
    for x := 0 to Fields.Count -1 do
      begin
      if (not fields[x].IsNull) and (pfInUpdate in Fields[x].ProviderFlags) then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        sql_fields := sql_fields + FieldNamesQuoteChars[0] + fields[x].FieldName + FieldNamesQuoteChars[1] + ',';
        sql_values := sql_values + ':"' + fields[x].FieldName + '",';
=======
=======
>>>>>>> origin/fixes_2_2
        sql_fields := sql_fields + FieldNamesQuoteChar + fields[x].FieldName + FieldNamesQuoteChar + ',';
        sql_values := sql_values + ':' + fields[x].FieldName + ',';
>>>>>>> graemeg/fixes_2_2
        end;
      end;
    if length(sql_fields) = 0 then DatabaseErrorFmt(sNoUpdateFields,['insert'],self);
    setlength(sql_fields,length(sql_fields)-1);
    setlength(sql_values,length(sql_values)-1);
>>>>>>> graemeg/cpstrnew

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

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSQLQuery.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and SQLConnection.LogEvent(EventType);
end;
=======
=======
>>>>>>> origin/fixes_2_2
    if length(sql_where) = 0 then DatabaseErrorFmt(sNoWhereFields,['delete'],self);
    setlength(sql_where,length(sql_where)-5);
>>>>>>> graemeg/fixes_2_2

procedure TCustomSQLQuery.Log(EventType: TDBEventType; const Msg: String);

Var
  M : String;

<<<<<<< HEAD
<<<<<<< HEAD
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  If LogEvent(EventType) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FieldNamesQuoteChars := TSQLConnection(DataBase).FieldNameQuoteChars;
=======
=======
>>>>>>> origin/fixes_2_2
var qry : TCustomSQLQuery;
    x   : integer;
    Fld : TField;
    
begin
  if sqQuoteFieldnames in TSQLConnection(DataBase).ConnOptions then
    FieldNamesQuoteChar := '"'
  else
    FieldNamesQuoteChar := ' ';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  case UpdateKind of
    ukModify : begin
               if not assigned(FUpdateQry) then
                 begin
                 if (trim(FUpdateSQL.Text)<> '') then
                   InitialiseModifyQuery(FUpdateQry,FUpdateSQL.Text)
                 else
                   InitialiseModifyQuery(FUpdateQry,ModifyRecQuery);
<<<<<<< HEAD
<<<<<<< HEAD
                 end;
               qry := FUpdateQry;
               end;
    ukInsert : begin
               if not assigned(FInsertQry) then
                 begin
                 if (trim(FInsertSQL.Text)<> '') then
                   InitialiseModifyQuery(FInsertQry,FInsertSQL.Text)
                 else
                   InitialiseModifyQuery(FInsertQry,InsertRecQuery);
                 end;
               qry := FInsertQry;
               end;
    ukDelete : begin
               if not assigned(FDeleteQry) then
                 begin
                 if (trim(FDeleteSQL.Text)<> '') then
                   InitialiseModifyQuery(FDeleteQry,FDeleteSQL.Text)
                 else
                   InitialiseModifyQuery(FDeleteQry,DeleteRecQuery);
                 end;
=======
                 end;
=======
                 end;
>>>>>>> origin/fixes_2_2
               qry := FUpdateQry;
               end;
    ukInsert : begin
               if not assigned(FInsertQry) then
                 begin
                 if (trim(FInsertSQL.Text)<> '') then
                   InitialiseModifyQuery(FInsertQry,FInsertSQL.Text)
                 else
                   InitialiseModifyQuery(FInsertQry,InsertRecQuery);
                 end;
               qry := FInsertQry;
               end;
    ukDelete : begin
<<<<<<< HEAD
               if not assigned(FDeleteQry) and (trim(FDeleteSQL.Text)<> '') then
                 InitialiseModifyQuery(FDeleteQry,FDeleteSQL.Text)
               else
                 InitialiseModifyQuery(FDeleteQry,DeleteRecQuery);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
               if not assigned(FDeleteQry) then
                 begin
                 if (trim(FDeleteSQL.Text)<> '') then
                   InitialiseModifyQuery(FDeleteQry,FDeleteSQL.Text)
                 else
                   InitialiseModifyQuery(FDeleteQry,DeleteRecQuery);
                 end;
>>>>>>> origin/fixes_2.4
               qry := FDeleteQry;
               end;
  end;
  assert(qry.sql.Text<>'');
  with qry do
>>>>>>> graemeg/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSQLQuery.GetStatementType : TStatementType;
=======
Function TCustomSQLQuery.GetCanModify: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
Function TCustomSQLQuery.GetCanModify: Boolean;
>>>>>>> origin/fixes_2_2

begin
<<<<<<< HEAD
  if Assigned(Cursor) then
    Result:=Cursor.FStatementType
=======
  // the test for assigned(FCursor) is needed for the case that the dataset isn't opened
  if assigned(FCursor) and (FCursor.FStatementType = stSelect) then
    Result:= FUpdateable and (not FReadOnly) and (not IsUniDirectional)
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
    Result:=stUnknown;
end;

<<<<<<< HEAD
procedure TCustomSQLQuery.SetParamCheck(AValue: Boolean);
begin
  FStatement.ParamCheck:=AValue;
end;

procedure TCustomSQLQuery.SetOptions(AValue: TSQLQueryOptions);
begin
  if FOptions=AValue then Exit;
  CheckInactive;
  FOptions:=AValue;
  if sqoKeepOpenOnCommit in FOptions then
    PacketRecords:=-1;
end;

<<<<<<< HEAD
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
=======
procedure TCustomSQLQuery.SetUpdateMode(AValue : TUpdateMode);
>>>>>>> graemeg/fixes_2_2

<<<<<<< HEAD
procedure TCustomSQLQuery.SetUpdateSQL(const AValue: TStringList);
begin
  FUpdateSQL.Assign(AValue);
end;
=======
procedure TCustomSQLQuery.BeforeRefreshOpenCursor;
begin
  // This is only necessary because TIBConnection can not re-open a
  // prepared cursor. In fact this is wrong, but has never led to
  // problems because in SetActive(false) queries are always
  // unprepared. (which is also wrong, but has to be fixed later)
  if IsPrepared then with TSQLConnection(DataBase) do
    UnPrepareStatement(FCursor);
end;

function TCustomSQLQuery.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and TSQLConnection(Database).LogEvent(EventType);
end;

procedure TCustomSQLQuery.Log(EventType: TDBEventType; const Msg: String);

Var
  M : String;
=======
procedure TCustomSQLQuery.SetUpdateMode(AValue : TUpdateMode);
>>>>>>> origin/fixes_2_2

<<<<<<< HEAD
begin
  If LogEvent(EventType) then
    begin
    M:=Msg;
    If (Name<>'') then
      M:=Name+' : '+M;
    TSQLConnection(Database).Log(EventType,M);
    end;
end;
=======
procedure TCustomSQLQuery.SetSchemaInfo( SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string);
>>>>>>> graemeg/fixes_2_2

procedure TCustomSQLQuery.BeforeRefreshOpenCursor;
begin
<<<<<<< HEAD
  // This is only necessary because TIBConnection can not re-open a
  // prepared cursor. In fact this is wrong, but has never led to
  // problems because in SetActive(false) queries are always
  // unprepared. (which is also wrong, but has to be fixed later)
  if IsPrepared then with TSQLConnection(DataBase) do
    UnPrepareStatement(FCursor);
end;

function TCustomSQLQuery.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and TSQLConnection(Database).LogEvent(EventType);
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
    TSQLConnection(Database).Log(EventType,M);
    end;
end;

procedure TCustomSQLQuery.BeforeRefreshOpenCursor;
begin
  // This is only necessary because TIBConnection can not re-open a
  // prepared cursor. In fact this is wrong, but has never led to
  // problems because in SetActive(false) queries are always
  // unprepared. (which is also wrong, but has to be fixed later)
  if IsPrepared then with TSQLConnection(DataBase) do
    UnPrepareStatement(FCursor);
end;

function TCustomSQLQuery.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and TSQLConnection(Database).LogEvent(EventType);
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
    TSQLConnection(Database).Log(EventType,M);
    end;
end;

procedure TCustomSQLQuery.BeforeRefreshOpenCursor;
begin
  // This is only necessary because TIBConnection can not re-open a
  // prepared cursor. In fact this is wrong, but has never led to
  // problems because in SetActive(false) queries are always
  // unprepared. (which is also wrong, but has to be fixed later)
  if IsPrepared then with TSQLConnection(DataBase) do
    UnPrepareStatement(FCursor);
end;

function TCustomSQLQuery.LogEvent(EventType: TDBEventType): Boolean;
begin
  Result:=Assigned(Database) and TSQLConnection(Database).LogEvent(EventType);
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
    TSQLConnection(Database).Log(EventType,M);
    end;
end;

<<<<<<< HEAD
function TCustomSQLQuery.GetStatementType : TStatementType;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomSQLQuery.SetSchemaInfo( SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string);
>>>>>>> origin/fixes_2_2

procedure TCustomSQLQuery.SetDeleteSQL(const AValue: TStringList);
begin
  FDeleteSQL.Assign(AValue);
end;

procedure TCustomSQLQuery.SetRefreshSQL(const AValue: TStringList);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FRefreshSQL.Assign(AValue);
=======
  FDeleteSQL.Assign(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FDeleteSQL.Assign(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FDeleteSQL.Assign(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FDeleteSQL.Assign(AValue);
>>>>>>> origin/cpstrnew
end;

=======
  ReadOnly := True;
  SQL.Clear;
  SQL.Add(TSQLConnection(DataBase).GetSchemaInfoSQL(SchemaType, SchemaObjectName, SchemaPattern));
end;

procedure TCustomSQLQuery.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField);
begin
  TSQLConnection(DataBase).LoadBlobIntoBuffer(FieldDef, ABlobBuf, FCursor,(Transaction as tsqltransaction));
<<<<<<< HEAD
end;

function TCustomSQLQuery.GetStatementType : TStatementType;
>>>>>>> graemeg/fixes_2_2

procedure TCustomSQLQuery.SetParams(AValue: TParams);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FStatement.Params.Assign(AValue);
=======
  FInsertSQL.Assign(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FInsertSQL.Assign(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FInsertSQL.Assign(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FInsertSQL.Assign(AValue);
>>>>>>> origin/cpstrnew
end;

<<<<<<< HEAD
procedure TCustomSQLQuery.SetDataSource(AValue: TDataSource);
=======
procedure TCustomSQLQuery.SetDeleteSQL(const AValue: TStringlist);
begin
  FDeleteQry.Assign(AValue);
end;
=======
end;

function TCustomSQLQuery.GetStatementType : TStatementType;
>>>>>>> origin/fixes_2_2

procedure TCustomSQLQuery.SetInsertSQL(const AValue: TStringlist);
begin
  FInsertQry.Assign(AValue);
end;

<<<<<<< HEAD
Procedure TCustomSQLQuery.SetDataSource(AVAlue : TDatasource);
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomSQLQuery.SetDeleteSQL(const AValue: TStringlist);
begin
  FDeleteQry.Assign(AValue);
end;

procedure TCustomSQLQuery.SetInsertSQL(const AValue: TStringlist);
begin
  FInsertQry.Assign(AValue);
end;

Procedure TCustomSQLQuery.SetDataSource(AVAlue : TDatasource);
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
    FStatement.DataSource:=AValue;
    end;
end;

function TCustomSQLQuery.GetDataSource: TDataSource;
=======
    If Assigned(AValue) then
      begin
      AValue.FreeNotification(Self);  
      If (FMasterLink=Nil) then
        FMasterLink:=TMasterParamsDataLink.Create(Self);
      FMasterLink.Datasource:=AValue;
      end
    else
      FreeAndNil(FMasterLink);  
    end;
end;

Function TCustomSQLQuery.GetDataSource : TDatasource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  If Assigned(FStatement) then
    Result:=FStatement.DataSource
  else
    Result:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSQLQuery.Notification(AComponent: TComponent; Operation: TOperation);
=======
procedure TCustomSQLQuery.Notification(AComponent: TComponent; Operation: TOperation); 
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomSQLQuery.Notification(AComponent: TComponent; Operation: TOperation); 
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2_2
begin
  if assigned (FOnDirective) then
    FOnDirective (Self, Directive, Argument, StopExecution);
end;

<<<<<<< HEAD
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
=======
begin
  if assigned (FOnDirective) then
    FOnDirective (Self, Directive, Argument, StopExecution);
end;

=======
>>>>>>> origin/fixes_2_2
procedure TSQLScript.ExecuteCommit;
begin
  if FTransaction is TSQLTransaction then
    TSQLTransaction(FTransaction).CommitRetaining
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

constructor TSQLScript.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
<<<<<<< HEAD
<<<<<<< HEAD
  FQuery := TCustomSQLQuery.Create(nil);
  FQuery.ParamCheck := false; // Do not parse for parameters; breaks use of e.g. select bla into :bla in Firebird procedures
=======
  FQuery := TCustomSQLQuery.Create(nil); 
>>>>>>> graemeg/fixes_2_2
=======
  FQuery := TCustomSQLQuery.Create(nil); 
>>>>>>> origin/fixes_2_2
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

function TSQLConnector.LoadField(cursor: TSQLCursor; FieldDef: TfieldDef;
  buffer: pointer; out CreateBlob: boolean): boolean;
begin
  CheckProxy;
  Result:=FProxy.LoadField(cursor, FieldDef, buffer, CreateBlob);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLConnector.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction);
begin
  CheckProxy;
  FProxy.LoadBlobIntoBuffer(FieldDef, ABlobBuf, cursor, ATransaction);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
{ TSqlObjectIdenfier }

constructor TSqlObjectIdenfier.Create(ACollection: TSqlObjectIdentifierList;
  const AObjectName: String; Const ASchemaName: String = '');
begin
  inherited Create(ACollection);
  FSchemaName:=ASchemaName;
  FObjectName:=AObjectName;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Initialization

Finalization
  DoneDefs;
end.
