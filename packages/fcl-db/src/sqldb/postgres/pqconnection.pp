unit PQConnection;

{$mode objfpc}{$H+}

{$Define LinkDynamically}

interface

uses
  Classes, SysUtils, sqldb, db, dbconst,bufdataset,
{$IfDef LinkDynamically}
  postgres3dyn;
{$Else}
  postgres3;
{$EndIf}

type
  TPQCursor = Class;

  { TPQTrans }

  TPQTrans = Class(TSQLHandle)
  protected
    PGConn : PPGConn;
    FList  : TThreadList;
    Procedure RegisterCursor(Cursor : TPQCursor);
    Procedure UnRegisterCursor(Cursor : TPQCursor);
  Public
    Constructor Create;
    Destructor Destroy; override;
  end;

  // TField and TFieldDef only support a limited amount of fields.
  // TFieldBinding and TExtendedFieldType can be used to map PQ types
  // on standard fields and retain mapping info.
  TExtendedFieldType = (eftNone,eftEnum,eftCitext);

  TFieldBinding = record
    FieldDef : TSQLDBFieldDef; // FieldDef this is associated with
    Index : Integer; // Tuple index
    TypeOID : oid; // Filled with type OID if it is not standard.
    TypeName : String; // Filled with type name by GetExtendedFieldInfo
    ExtendedFieldType: TExtendedFieldType; //
  end;
  PFieldBinding = ^TFieldBinding;
  TFieldBindings = Array of TFieldBinding;

  { TPQCursor }

  TPQCursor = Class(TSQLCursor)
  protected
    Statement    : string;
    StmtName     : string;
    tr           : TPQTrans;
    res          : PPGresult;
    CurTuple     : integer;
    FieldBinding : TFieldBindings;
    Function GetFieldBinding(F : TFieldDef): PFieldBinding;
   Public
    Destructor Destroy; override;
  end;

  { EPQDatabaseError }

  EPQDatabaseError = class(EDatabaseError)
    public
      SEVERITY:string;
      SQLSTATE: string;
      MESSAGE_PRIMARY:string;
      MESSAGE_DETAIL:string;
      MESSAGE_HINT:string;
      STATEMENT_POSITION:string;
  end;

  { TPQTranConnection }

  TPQTranConnection = class
  protected
    FPGConn        : PPGConn;
    FTranActive    : boolean
  end;

  { TPQConnection }

  TPQConnection = class (TSQLConnection)
  private
    FConnectionPool      : TThreadList;
    FCursorCount         : dword;
    FConnectString       : string;
    FIntegerDateTimes    : boolean;
    FVerboseErrors       : Boolean;
  protected
    // Protected so they can be used by descendents.
    procedure CheckConnectionStatus(var conn: PPGconn);
    procedure CheckResultError(var res: PPGresult; conn:PPGconn; ErrMsg: string);
    function TranslateFldType(res : PPGresult; Tuple : integer; out Size : integer; Out ATypeOID : oid) : TFieldType;
    procedure ExecuteDirectPG(const Query : String);
    Procedure GetExtendedFieldInfo(cursor: TPQCursor; Bindings : TFieldBindings);

    procedure ApplyFieldUpdate(C : TSQLCursor; P: TSQLDBParam; F: TField; UseOldValue: Boolean); override;
    Function ErrorOnUnknownType : Boolean;
    // Add connection to pool.
    procedure AddConnection(T: TPQTranConnection);
    // Release connection in pool.
    procedure ReleaseConnection(Conn: PPGConn; DoClear : Boolean);

    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    function GetHandle : pointer; override;

    Function AllocateCursorHandle : TSQLCursor; override;
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); override;
    Function AllocateTransactionHandle : TSQLHandle; override;

    procedure PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams); override;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); override;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TfieldDefs); override;
    function Fetch(cursor : TSQLCursor) : boolean; override;
    procedure UnPrepareStatement(cursor : TSQLCursor); override;
    function LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; override;
    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    function StartImplicitTransaction(trans : TSQLHandle; AParams : string) : boolean; override;
    function StartDBTransaction(trans : TSQLHandle; AParams : string) : boolean; override;
    procedure RollBackRetaining(trans : TSQLHandle); override;
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor;ATransaction : TSQLTransaction); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    function GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string; override;
  public
    constructor Create(AOwner : TComponent); override;
    destructor Destroy; override;
    function GetConnectionInfo(InfoType:TConnInfoType): string; override;
    procedure CreateDB; override;
    procedure DropDB; override;
  published
    property DatabaseName;
    property KeepConnection;
    property LoginPrompt;
    property Params;
    property OnLogin;
    Property VerboseErrors : Boolean Read FVerboseErrors Write FVerboseErrors default true;
  end;

  { TPQConnectionDef }

  TPQConnectionDef = Class(TConnectionDef)
    Class Function TypeName : String; override;
    Class Function ConnectionClass : TSQLConnectionClass; override;
    Class Function Description : String; override;
    Class Function DefaultLibraryName : String; override;
    Class Function LoadFunction : TLibraryLoadFunction; override;
    Class Function UnLoadFunction : TLibraryUnLoadFunction; override;
    Class Function LoadedLibraryName: string; override;
  end;

implementation

uses math, strutils, FmtBCD;

ResourceString
  SErrRollbackFailed = 'Rollback transaction failed';
  SErrCommitFailed = 'Commit transaction failed';
  SErrConnectionFailed = 'Connection to database failed';
  SErrTransactionFailed = 'Start of transacion failed';
  SErrExecuteFailed = 'Execution of query failed';
  SErrPrepareFailed = 'Preparation of query failed.';
  SErrUnPrepareFailed = 'Unpreparation of query failed.';

const Oid_Bool     = 16;
      Oid_Bytea    = 17;
      Oid_char     = 18;
      Oid_Text     = 25;
      Oid_Oid      = 26;
      Oid_Name     = 19;
      Oid_Int8     = 20;
      Oid_int2     = 21;
      Oid_Int4     = 23;
      Oid_JSON     = 114;
      Oid_Float4   = 700;
      Oid_Money    = 790;
      Oid_Float8   = 701;
      Oid_Unknown  = 705;
      Oid_MacAddr  = 829;
      Oid_Inet     = 869;
      Oid_bpchar   = 1042;
      Oid_varchar  = 1043;
      oid_date      = 1082;
      oid_time      = 1083;
      Oid_timeTZ    = 1266;
      Oid_timestamp = 1114;
      Oid_timestampTZ = 1184;
      Oid_interval  = 1186;
      oid_numeric   = 1700;
      Oid_uuid      = 2950;


{ TPQTrans }

constructor TPQTrans.Create;
begin
  FList:=TThreadList.Create;
  FList.Duplicates:=dupIgnore;
end;

destructor TPQTrans.Destroy;

Var
  L : TList;
  I : integer;

begin
  L:=FList.LockList;
  try
    For I:=0 to L.Count-1 do
      TPQCursor(L[i]).tr:=Nil;
  finally
    FList.UnlockList;
  end;
  FreeAndNil(FList);
  inherited Destroy;
end;

procedure TPQTrans.RegisterCursor(Cursor: TPQCursor);
begin
  FList.Add(Cursor);
  Cursor.tr:=Self;
end;

procedure TPQTrans.UnRegisterCursor(Cursor: TPQCursor);
begin
  Cursor.tr:=Nil;
  FList.Remove(Cursor);
end;


{ TPQCursor }

destructor TPQCursor.Destroy;
begin
  if Assigned(tr) then
    tr.UnRegisterCursor(Self);
  inherited Destroy;
end;

function TPQCursor.GetFieldBinding(F: TFieldDef): PFieldBinding;

Var
  I : Integer;

begin
  Result:=Nil;
  if (F=Nil) then exit;
  // This is an optimization: it is so for 99% of cases (FieldNo-1=array index)
  if F is TSQLDBFieldDef then
    Result:=PFieldBinding(TSQLDBFieldDef(F).SQLDBData)
  else If (FieldBinding[F.FieldNo-1].FieldDef=F) then
    Result:=@FieldBinding[F.FieldNo-1]
  else
    begin
    I:=Length(FieldBinding)-1;
    While (I>=0) and (FieldBinding[i].FieldDef<>F) do
      Dec(I);
    if I>=0 then
      Result:=@FieldBinding[i];
    end;
end;


{ TPQConnection }

constructor TPQConnection.Create(AOwner : TComponent);

begin
  inherited;
  FConnOptions := FConnOptions + [sqSupportParams, sqSupportEmptyDatabaseName, sqEscapeRepeat, sqEscapeSlash, sqImplicitTransaction,sqSupportReturning,sqSequences];
  FieldNameQuoteChars:=DoubleQuotes;
  VerboseErrors:=True;
  FConnectionPool:=TThreadlist.Create;
end;

destructor TPQConnection.Destroy;
begin
  // We must disconnect here. If it is done in inherited, then connection pool is gone.
  Connected:=False;
  FreeAndNil(FConnectionPool);
  inherited destroy;
end;

procedure TPQConnection.CreateDB;

begin
  ExecuteDirectPG('CREATE DATABASE ' +DatabaseName);
end;

procedure TPQConnection.DropDB;

begin
  ExecuteDirectPG('DROP DATABASE ' +DatabaseName);
end;

procedure TPQConnection.ExecuteDirectPG(const Query: String);

var ASQLDatabaseHandle    : PPGConn;
    res                   : PPGresult;

begin
  CheckDisConnected;
{$IfDef LinkDynamically}
  InitialisePostgres3;
{$EndIf}

  FConnectString := '';
  if (UserName <> '') then FConnectString := FConnectString + ' user=''' + UserName + '''';
  if (Password <> '') then FConnectString := FConnectString + ' password=''' + Password + '''';
  if (HostName <> '') then FConnectString := FConnectString + ' host=''' + HostName + '''';
  FConnectString := FConnectString + ' dbname=''template1''';
  if (Params.Text <> '') then FConnectString := FConnectString + ' '+Params.Text;

  ASQLDatabaseHandle := PQconnectdb(pchar(FConnectString));

  CheckConnectionStatus(ASQLDatabaseHandle);

  res := PQexec(ASQLDatabaseHandle,pchar(query));

  CheckResultError(res,ASQLDatabaseHandle,SDBCreateDropFailed);

  PQclear(res);
  PQFinish(ASQLDatabaseHandle);
{$IfDef LinkDynamically}
  ReleasePostgres3;
{$EndIf}
end;

procedure TPQConnection.GetExtendedFieldInfo(cursor: TPQCursor;
  Bindings: TFieldBindings);

Var
  tt,tc,Tn,S : String;
  I,J : Integer;
  Res : PPGResult;
  toid : oid;

begin
  For I:=0 to Length(Bindings)-1 do
    if (Bindings[i].TypeOID>0) then
      begin
      if (S<>'') then
        S:=S+', ';
      S:=S+IntToStr(Bindings[i].TypeOID);
      end;
  if (S='') then
    exit;
  S:='select oid,typname,typtype,typcategory from pg_type where oid in ('+S+') order by oid';
  Res:=PQExec(Cursor.tr.PGConn,PChar(S));
  if (PQresultStatus(res)<>PGRES_TUPLES_OK) then
    CheckResultError(Res,Cursor.tr.PGConn,'Error getting type info');
  try
    For I:=0 to PQntuples(Res)-1 do
      begin
      toid:=Strtoint(pqgetvalue(Res,i,0));
      tn:=pqgetvalue(Res,i,1);
      tt:=pqgetvalue(Res,i,2);
      tc:=pqgetvalue(Res,i,3);
      J:=length(Bindings)-1;
      while (J>= 0) do
        begin
        if (Bindings[j].TypeOID=toid) then
          Case tt of
           'e':
            Bindings[j].ExtendedFieldType:=eftEnum;
           'citext':
            Bindings[j].ExtendedFieldType:=eftCitext;
          end;
        Dec(J);
        end;
      end;
  finally
    PQClear(Res);
  end;
end;

procedure TPQConnection.ApplyFieldUpdate(C : TSQLCursor; P: TSQLDBParam; F: TField;
  UseOldValue: Boolean);
begin
  inherited ApplyFieldUpdate(C,P, F, UseOldValue);
  if (C is TPQCursor) then
    P.SQLDBData:=TPQCursor(C).GetFieldBinding(F.FieldDef);
end;

function TPQConnection.ErrorOnUnknownType: Boolean;
begin
  Result:=False;
end;

procedure TPQConnection.AddConnection(T: TPQTranConnection);

begin
  FConnectionPool.Add(T);
end;

procedure TPQConnection.ReleaseConnection(Conn: PPGConn; DoClear: Boolean);

Var
  I : Integer;
  L : TList;
  T : TPQTranConnection;

begin
  L:=FConnectionPool.LockList;
  // make connection available in pool
  try
    for i:=0 to L.Count-1 do
      begin
      T:=TPQTranConnection(L[i]);
      if (T.FPGConn=Conn) then
        begin
        T.FTranActive:=false;
        if DoClear then
          T.FPGConn:=Nil;
        break;
        end;
      end
  finally
    FConnectionPool.UnlockList;
  end;
end;


function TPQConnection.GetTransactionHandle(trans : TSQLHandle): pointer;
begin
  Result := trans;
end;

function TPQConnection.RollBack(trans : TSQLHandle) : boolean;
var
  res : PPGresult;
  tr  : TPQTrans;
  i   : Integer;
  L   : TList;

begin
  result := false;
  tr := trans as TPQTrans;
  // unprepare statements associated with given transaction
  L:=tr.FList.LockList;
  try
    For I:=0 to L.Count-1 do
      begin
      UnprepareStatement(TPQCursor(L[i]));
      TPQCursor(L[i]).tr:=Nil;
      end;
    L.Clear;
  finally
    tr.FList.UnlockList;
  end;

  res := PQexec(tr.PGConn, 'ROLLBACK');
  CheckResultError(res,tr.PGConn,SErrRollbackFailed);
  PQclear(res);
  ReleaseConnection(tr.PGCOnn,false);
  result := true;
end;

function TPQConnection.Commit(trans : TSQLHandle) : boolean;
var
  res : PPGresult;
  tr  : TPQTrans;
begin
  result := false;
  tr := trans as TPQTrans;
  res := PQexec(tr.PGConn, 'COMMIT');
  CheckResultError(res,tr.PGConn,SErrCommitFailed);
  PQclear(res);
  //make connection available in pool
  ReleaseConnection(tr.PGConn,false);
  result := true;
end;

procedure TPQConnection.RollBackRetaining(trans : TSQLHandle);
var
  res : PPGresult;
  tr  : TPQTrans;
begin
  tr := trans as TPQTrans;
  res := PQexec(tr.PGConn, 'ROLLBACK');
  CheckResultError(res,tr.PGConn,SErrRollbackFailed);

  PQclear(res);
  res := PQexec(tr.PGConn, 'BEGIN');
  CheckResultError(res,tr.PGConn,sErrTransactionFailed);

  PQclear(res);
end;

procedure TPQConnection.CommitRetaining(trans : TSQLHandle);
var
  res : PPGresult;
  tr  : TPQTrans;
begin
  tr := trans as TPQTrans;
  res := PQexec(tr.PGConn, 'COMMIT');
  CheckResultError(res,tr.PGConn,SErrCommitFailed);

  PQclear(res);
  res := PQexec(tr.PGConn, 'BEGIN');
  CheckResultError(res,tr.PGConn,sErrTransactionFailed);

  PQclear(res);
end;

function TPQConnection.StartImplicitTransaction(trans : TSQLHandle; AParams : string) : boolean;
var
  i : Integer;
  T : TPQTranConnection;
  L : TList;
begin
  //find an unused connection in the pool
  i:=0;
  T:=Nil;
  L:=FConnectionPool.LockList;
  try
    while (i<L.Count) do
      begin
      T:=TPQTranConnection(L[i]);
      if (T.FPGConn=nil) or not T.FTranActive then
        break
      else
        T:=Nil;
      i:=i+1;
      end;
    // set to active now, so when we exit critical section,
    // it will be marked active and will not be found.
    if Assigned(T) then
      T.FTranActive:=true;
  finally
    FConnectionPool.UnLockList;
  end;

  if (T=Nil) then
    begin
    T:=TPQTranConnection.Create;
    T.FTranActive:=True;
    AddConnection(T);
    end;

  if (T.FPGConn=nil) then
    begin
    T.FPGConn := PQconnectdb(pchar(FConnectString));
    CheckConnectionStatus(T.FPGConn);
    if CharSet <> '' then
      PQsetClientEncoding(T.FPGConn, pchar(CharSet));
    end;

  TPQTrans(trans).PGConn := T.FPGConn;
  Result := true;
end;

function TPQConnection.StartDBTransaction(trans: TSQLHandle; AParams: string
  ): boolean;

Var
  res : PPGresult;
  tr  : TPQTrans;

begin
  Result:=StartImplicitTransaction(trans, AParams);
  if Result then
    begin
    tr := trans as TPQTrans;
    res := PQexec(tr.PGConn, 'BEGIN');
    CheckResultError(res,tr.PGConn,sErrTransactionFailed);
    PQclear(res);
    end;
end;


procedure TPQConnection.DoInternalConnect;
var
  ASQLDatabaseHandle   : PPGConn;
  T : TPQTranConnection;

begin
{$IfDef LinkDynamically}
  InitialisePostgres3;
{$EndIf}

  inherited DoInternalConnect;

  FConnectString := '';
  if (UserName <> '') then FConnectString := FConnectString + ' user=''' + UserName + '''';
  if (Password <> '') then FConnectString := FConnectString + ' password=''' + Password + '''';
  if (HostName <> '') then FConnectString := FConnectString + ' host=''' + HostName + '''';
  if (DatabaseName <> '') then FConnectString := FConnectString + ' dbname=''' + DatabaseName + '''';
  if (Params.Text <> '') then FConnectString := FConnectString + ' '+Params.Text;

  ASQLDatabaseHandle := PQconnectdb(pchar(FConnectString));
  try
    CheckConnectionStatus(ASQLDatabaseHandle);
  except
    DoInternalDisconnect;
    raise;
  end;

  // This only works for pg>=8.0, so timestamps won't work with earlier versions of pg which are compiled with integer_datetimes on
  if PQparameterStatus<>nil then
    FIntegerDateTimes := PQparameterStatus(ASQLDatabaseHandle,'integer_datetimes') = 'on';
  T:=TPQTranConnection.Create;
  T.FPGConn:=ASQLDatabaseHandle;
  T.FTranActive:=false;
  AddConnection(T);
end;

procedure TPQConnection.DoInternalDisconnect;
var
  i:integer;
  L : TList;
  T : TPQTranConnection;

begin
  Inherited;
  L:=FConnectionPool.LockList;
  try
    for i:=0 to L.Count-1 do
      begin
      T:=TPQTranConnection(L[i]);
      if assigned(T.FPGConn) then
        PQfinish(T.FPGConn);
      T.Free;
      end;
    L.Clear;
  finally
    FConnectionPool.UnLockList;
  end;
{$IfDef LinkDynamically}
  ReleasePostgres3;
{$EndIf}
end;

procedure TPQConnection.CheckConnectionStatus(var conn: PPGconn);
var sErr: string;
begin
  if (PQstatus(conn) = CONNECTION_BAD) then
    begin
    sErr := PQerrorMessage(conn);
    //make connection available in pool
    ReleaseConnection(Conn,True);
    PQfinish(conn);
    DatabaseError(sErrConnectionFailed + ' (PostgreSQL: ' + sErr + ')', Self);
    end;
end;

procedure TPQConnection.CheckResultError(var res: PPGresult; conn: PPGconn;
  ErrMsg: string);

  Procedure MaybeAdd(Var S : String; Prefix,Msg : String);

  begin
    if (Msg='') then
      exit;
    S:=S+LineEnding+Prefix+': '+Msg;
  end;

var
  E: EPQDatabaseError;
  sErr: string;
  CompName: string;
  SEVERITY: string;
  SQLSTATE: string;
  MESSAGE_PRIMARY: string;
  MESSAGE_DETAIL: string;
  MESSAGE_HINT: string;
  STATEMENT_POSITION: string;
  P : Pchar;
  haveError : Boolean;

begin
  HaveError:=False;
  if (Res=Nil) then
    begin
    HaveError:=True;
    P:=PQerrorMessage(conn);
    If Assigned(p) then
      ErrMsg:=StrPas(P);
    end
  else if (PQresultStatus(res) <> PGRES_COMMAND_OK) then
    begin
    HaveError:=True;
    SEVERITY:=PQresultErrorField(res,ord('S'));
    SQLSTATE:=PQresultErrorField(res,ord('C'));
    MESSAGE_PRIMARY:=PQresultErrorField(res,ord('M'));
    MESSAGE_DETAIL:=PQresultErrorField(res,ord('D'));
    MESSAGE_HINT:=PQresultErrorField(res,ord('H'));
    STATEMENT_POSITION:=PQresultErrorField(res,ord('P'));
    sErr:=PQresultErrorMessage(res);
    if VerboseErrors then
      begin
      MaybeAdd(sErr,'Severity',SEVERITY);
      MaybeAdd(sErr,'SQL State',SQLSTATE);
      MaybeAdd(sErr,'Primary Error',MESSAGE_PRIMARY);
      MaybeAdd(sErr,'Error Detail',MESSAGE_DETAIL);
      MaybeAdd(sErr,'Hint',MESSAGE_HINT);
      MaybeAdd(sErr,'Character',STATEMENT_POSITION);
      end;
    end;
  if HaveError then
    begin
    if (Self.Name='') then CompName := Self.ClassName else CompName := Self.Name;
    E:=EPQDatabaseError.CreateFmt('%s : %s  (PostgreSQL: %s)', [CompName, ErrMsg, sErr]);
    E.SEVERITY:=SEVERITY;
    E.SQLSTATE:=SQLSTATE;
    E.MESSAGE_PRIMARY:=MESSAGE_PRIMARY;
    E.MESSAGE_DETAIL:=MESSAGE_DETAIL;
    E.MESSAGE_HINT:=MESSAGE_HINT;
    E.STATEMENT_POSITION:=STATEMENT_POSITION;
    PQclear(res);
    res:=nil;
    if assigned(conn) then
      begin
      PQFinish(conn);
      ReleaseConnection(Conn,True);
      end;
    raise E;
    end;
end;

function TPQConnection.TranslateFldType(res: PPGresult; Tuple: integer; out
  Size: integer; out ATypeOID: oid): TFieldType;

const
  VARHDRSZ=sizeof(longint);
var
  li : longint;
  aoid : oid;

begin
  Size := 0;
  ATypeOID:=0;
  AOID:=PQftype(res,Tuple);
  case AOID of
    Oid_varchar,Oid_bpchar,
    Oid_name               : begin
                             Result := ftString;
                             size := PQfsize(Res, Tuple);
                             if (size = -1) then
                               begin
                               li := PQfmod(res,Tuple);
                               if li = -1 then
                                 size := dsMaxStringSize
                               else
                                 size := (li-VARHDRSZ) and $FFFF;
                               end;
                             if size > MaxSmallint then size := MaxSmallint;
                             end;
//    Oid_text               : Result := ftString;
    Oid_text,Oid_JSON      : Result := ftMemo;
    Oid_Bytea              : Result := ftBlob;
    Oid_oid                : Result := ftInteger;
    Oid_int8               : Result := ftLargeInt;
    Oid_int4               : Result := ftInteger;
    Oid_int2               : Result := ftSmallInt;
    Oid_Float4             : Result := ftFloat;
    Oid_Float8             : Result := ftFloat;
    Oid_TimeStamp,
    Oid_TimeStampTZ        : Result := ftDateTime;
    Oid_Date               : Result := ftDate;
    Oid_Interval,
    Oid_Time,
    Oid_TimeTZ             : Result := ftTime;
    Oid_Bool               : Result := ftBoolean;
    Oid_Numeric            : begin
                             Result := ftBCD;
                             li := PQfmod(res,Tuple);
                             if li = -1 then
                               size := 4 // No information about the size available, use the maximum value
                             else
                             // The precision is the high 16 bits, the scale the
                             // low 16 bits with an offset of sizeof(int32).
                               begin
                               size := (li-VARHDRSZ) and $FFFF;
                               if (size > MaxBCDScale) or ((li shr 16)-size > MaxBCDPrecision-MaxBCDScale) then
                                 Result := ftFmtBCD;
                               end;
                             end;
    Oid_Money              : Result := ftCurrency;
    Oid_char               : begin
                             Result := ftFixedChar;
                             Size := 1;
                             end;
    Oid_uuid               : begin
                             Result := ftGuid;
                             Size := 38;
                             end;
    Oid_MacAddr            : begin
                             Result := ftFixedChar;
                             Size := 17;
                             end;
    Oid_Inet               : begin
                             Result := ftString;
                             Size := 39;
                             end;
    Oid_Unknown            : Result := ftUnknown;
  else
    Result:=ftUnknown;
    ATypeOID:=AOID;
  end;
end;

function TPQConnection.AllocateCursorHandle: TSQLCursor;

begin
  result := TPQCursor.create;
end;

procedure TPQConnection.DeAllocateCursorHandle(var cursor: TSQLCursor);
begin
  FreeAndNil(cursor);
end;

function TPQConnection.AllocateTransactionHandle: TSQLHandle;

begin
  result := TPQTrans.create;
end;

procedure TPQConnection.PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams);
                          
const TypeStrings : array[TFieldType] of string =
    (
      'Unknown',   // ftUnknown
      'text',      // ftString
      'smallint',  // ftSmallint
      'int',       // ftInteger
      'int',       // ftWord
      'bool',      // ftBoolean
      'float',     // ftFloat
      'money',     // ftCurrency
      'numeric',   // ftBCD
      'date',      // ftDate
      'time',      // ftTime
      'timestamp', // ftDateTime
      'Unknown',   // ftBytes
      'Unknown',   // ftVarBytes
      'Unknown',   // ftAutoInc
      'bytea',     // ftBlob 
      'text',      // ftMemo
      'bytea',     // ftGraphic
      'text',      // ftFmtMemo
      'Unknown',   // ftParadoxOle
      'Unknown',   // ftDBaseOle
      'Unknown',   // ftTypedBinary
      'Unknown',   // ftCursor
      'char',      // ftFixedChar
      'text',      // ftWideString
      'bigint',    // ftLargeint
      'Unknown',   // ftADT
      'Unknown',   // ftArray
      'Unknown',   // ftReference
      'Unknown',   // ftDataSet
      'Unknown',   // ftOraBlob
      'Unknown',   // ftOraClob
      'Unknown',   // ftVariant
      'Unknown',   // ftInterface
      'Unknown',   // ftIDispatch
      'uuid',      // ftGuid
      'Unknown',   // ftTimeStamp
      'numeric',   // ftFMTBcd
      'Unknown',   // ftFixedWideChar
      'Unknown'    // ftWideMemo
    );


var
  s,ts : string;
  i : integer;
  P : TParam;
  PQ : TSQLDBParam;

begin
  with (cursor as TPQCursor) do
    begin
    FPrepared := False;
    FDirect := False;
    // Prior to v8 there is no support for cursors and parameters.
    // So that's not supported.
    if FStatementType in [stInsert,stUpdate,stDelete, stSelect] then
      begin
      StmtName := 'prepst'+inttostr(FCursorCount);
      InterlockedIncrement(FCursorCount);
      TPQTrans(aTransaction.Handle).RegisterCursor(Cursor as TPQCursor);

      // Only available for pq 8.0, so don't use it...
      // Res := pqprepare(tr,'prepst'+name+nr,pchar(buf),params.Count,pchar(''));
      s := 'prepare '+StmtName+' ';
      if Assigned(AParams) and (AParams.Count > 0) then
        begin
        s := s + '(';
        for i := 0 to AParams.Count-1 do
          begin
          P:=AParams[i];
          If (P is TSQLDBParam) then
            PQ:=TSQLDBParam(P)
          else
            PQ:=Nil;
          TS:=TypeStrings[P.DataType];
          if (TS<>'Unknown') then
            begin
            If Assigned(PQ)
               and Assigned(PQ.SQLDBData)
               and (PFieldBinding(PQ.SQLDBData)^.ExtendedFieldType=eftEnum) then
                ts:='unknown';
            s := s + ts + ','
            end
          else
            begin
            if AParams[i].DataType = ftUnknown then
              begin
              if AParams[i].IsNull then
                s:=s+' unknown ,'
              else
                DatabaseErrorFmt(SUnknownParamFieldType,[AParams[i].Name],self)
              end
            else
              DatabaseErrorFmt(SUnsupportedParameter,[Fieldtypenames[AParams[i].DataType]],self);
            end;
          end;
        s[length(s)] := ')';
        buf := AParams.ParseSQL(buf,false,sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions,psPostgreSQL);
        end;
      s := s + ' as ' + buf;
      if LogEvent(detActualSQL) then
        Log(detActualSQL,S);
      res := PQexec(tr.PGConn,pchar(s));
      CheckResultError(res,nil,SErrPrepareFailed);
      // if statement is INSERT, UPDATE, DELETE with RETURNING clause, then
      // override the statement type derrived by parsing the query.
      if (FStatementType in [stInsert,stUpdate,stDelete]) and (pos('RETURNING', upcase(s)) > 0) then
        begin
        PQclear(res);
        res := PQdescribePrepared(tr.PGConn,pchar(StmtName));
        if (PQresultStatus(res) = PGRES_COMMAND_OK) and (PQnfields(res) > 0) then
          FStatementType := stSelect;
        end;
      FPrepared := True;
      end
    else
      begin
      if Assigned(AParams) then
        Statement := AParams.ParseSQL(buf,false,sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions,psPostgreSQL)
      else
        Statement:=Buf;
      FDirect:=True;
      end;
    end;
end;

procedure TPQConnection.UnPrepareStatement(cursor : TSQLCursor);
begin
  with (cursor as TPQCursor) do
    begin
    PQclear(res);
    res:=nil;
    if FPrepared then
      begin
      if assigned(tr) and (PQtransactionStatus(tr.PGConn) <> PQTRANS_INERROR) then
        begin
        res := PQexec(tr.PGConn,pchar('deallocate '+StmtName));
        CheckResultError(res,nil,SErrUnPrepareFailed);
        PQclear(res);
        res:=nil;
        end;
      FPrepared := False;
      end;
    end;
end;

procedure TPQConnection.Execute(cursor: TSQLCursor;atransaction:tSQLtransaction;AParams : TParams);

var ar  : array of PAnsiChar;
    handled : boolean;
    l,i : integer;
    s   : RawByteString;
    bd : TBlobData;
    lengths,formats : array of integer;
    ParamNames,
    ParamValues : array of string;
    cash: int64;

    function FormatTimeInterval(Time: TDateTime): string; // supports Time >= '24:00:00'
    var hour, minute, second, millisecond: word;
    begin
      DecodeTime(Time, hour, minute, second, millisecond);
      Result := Format('%.2d:%.2d:%.2d.%.3d',[Trunc(Time)*24+hour,minute,second,millisecond]);
    end;

begin
  with cursor as TPQCursor do
    begin
    CurTuple:=-1;
    PQclear(res);
    if FStatementType in [stInsert,stUpdate,stDelete,stSelect] then
      begin
      if LogEvent(detParamValue) then
        LogParams(AParams);
      if Assigned(AParams) and (AParams.Count > 0) then
        begin
        l:=AParams.Count;
        setlength(ar,l);
        setlength(lengths,l);
        setlength(formats,l);
        for i := 0 to AParams.Count -1 do if not AParams[i].IsNull then
          begin
          handled:=False;
          case AParams[i].DataType of
            ftDateTime:
              s := FormatDateTime('yyyy"-"mm"-"dd hh":"nn":"ss.zzz', AParams[i].AsDateTime);
            ftDate:
              s := FormatDateTime('yyyy"-"mm"-"dd', AParams[i].AsDateTime);
            ftTime:
              s := FormatTimeInterval(AParams[i].AsDateTime);
            ftFloat:
              Str(AParams[i].AsFloat, s);
            ftBCD:
              Str(AParams[i].AsCurrency, s);
            ftCurrency:
              begin
                cash:=NtoBE(round(AParams[i].AsCurrency*100));
                setlength(s, sizeof(cash));
                Move(cash, s[1], sizeof(cash));
              end;
            ftFmtBCD:
              s := BCDToStr(AParams[i].AsFMTBCD, FSQLFormatSettings);
            ftBlob, ftGraphic:
              begin
              Handled:=true;
              bd:= AParams[i].AsBlob;
              l:=length(BD);
              if l>0 then
                begin
                GetMem(ar[i],l+1);
                ar[i][l]:=#0;
                Move(BD[0],ar[i]^, L);
                lengths[i]:=l;
                end;
              end
            else
              s := GetAsString(AParams[i]);
          end; {case}
          if not handled then
            begin
            l:=length(s);
            GetMem(ar[i],l+1);
            StrMove(PAnsiChar(ar[i]), PAnsiChar(s), L+1);
            lengths[i]:=L;
            end;
          if (AParams[i].DataType in [ftBlob,ftMemo,ftGraphic,ftCurrency]) then
            Formats[i]:=1
          else
            Formats[i]:=0;  
          end
        else
          FreeAndNil(ar[i]);
        res := PQexecPrepared(tr.PGConn,pchar(StmtName),AParams.Count,@Ar[0],@Lengths[0],@Formats[0],1);
        for i := 0 to AParams.Count -1 do
          FreeMem(ar[i]);
        end
      else
        res := PQexecPrepared(tr.PGConn,pchar(StmtName),0,nil,nil,nil,1);
      end
    else
      begin
      // RegisterCursor sets tr
      TPQTrans(aTransaction.Handle).RegisterCursor(Cursor as TPQCursor);

      if Assigned(AParams) and (AParams.Count > 0) then
        begin
        setlength(ParamNames,AParams.Count);
        setlength(ParamValues,AParams.Count);
        for i := 0 to AParams.Count -1 do
          begin
          ParamNames[AParams.Count-i-1] := '$'+inttostr(AParams[i].index+1);
          ParamValues[AParams.Count-i-1] := GetAsSQLText(AParams[i]);
          end;
        s := stringsreplace(Statement,ParamNames,ParamValues,[rfReplaceAll]);
        end
      else
        s := Statement;
      res := PQexec(tr.PGConn,pchar(s));
      if (PQresultStatus(res) in [PGRES_COMMAND_OK]) then
        begin
        PQclear(res);
        res:=nil;
        end;
      end;

    if assigned(res) and not (PQresultStatus(res) in [PGRES_COMMAND_OK,PGRES_TUPLES_OK]) then
      begin
      // Don't perform the rollback, only make it possible to do a rollback.
      // The other databases also don't do this.
      //atransaction.Rollback;
      CheckResultError(res,nil,SErrExecuteFailed);
      end;

    FSelectable := assigned(res) and (PQresultStatus(res)=PGRES_TUPLES_OK);
    end;
end;


procedure TPQConnection.AddFieldDefs(cursor: TSQLCursor; FieldDefs : TfieldDefs);
var
  i         : integer;
  asize     : integer;
  aoid      : oid;
  fieldtype : tfieldtype;
  nFields   : integer;
  b : Boolean;
  Q : TPQCursor;
  FD : TSQLDBFieldDef;
  FB : PFieldBinding;

begin
  B:=False;
  Q:=cursor as TPQCursor;
  with Q do
    begin
    nFields := PQnfields(Res);
    setlength(FieldBinding,nFields);
    for i := 0 to nFields-1 do
      begin
      fieldtype := TranslateFldType(Res, i, asize, aoid );
      FD := AddFieldDef(FieldDefs, i+1, PQfname(Res, i), fieldtype, asize, -1, False, False, False) as TSQLDBFieldDef;
      With FD do
        begin
        SQLDBData:=@FieldBinding[i];
        FieldBinding[i].Index:=i;
        FieldBinding[i].FieldDef:=FD;
        FieldBinding[i].TypeOID:=aOID;
        B:=B or (aOID>0);
        end;
      end;
    CurTuple := -1;
    end;
  if B then
    begin
    // get all information in 1 go.
    GetExtendedFieldInfo(Q,Q.FieldBinding);
    For I:=0 to Length(Q.FieldBinding)-1 do
      begin
      FB:=@Q.FieldBinding[i];
      if (FB^.TypeOID>0) then
        begin
        FD:=FB^.FieldDef;
        Case FB^.ExtendedFieldType of
          eftEnum :
            begin
            FD.DataType:=ftString;
            FD.Size:=64;
            //FD.Attributes:=FD.Attributes+[faReadonly];
            end;
          eftCitext:
            begin
            FD.DataType:=ftMemo;
            end
        else
          if ErrorOnUnknownType then
            DatabaseError('Unhandled field type :'+FB^.TypeName,Self);
        end;
        end;
      end;
    end;
end;

function TPQConnection.GetHandle: pointer;
var
  i:integer;
  L : TList;
  T : TPQTranConnection;

begin
  result:=nil;
  if not Connected then
    exit;
  //Get any handle that is (still) connected
  L:=FConnectionPool.LockList;
  try
    I:=L.Count-1;
    While (I>=0) and (Result=Nil) do
      begin
      T:=TPQTranConnection(L[i]);
      if assigned(T.FPGConn) and (PQstatus(T.FPGConn)<>CONNECTION_BAD) then
        Result:=T.FPGConn;
      Dec(I);
      end;
  finally
    FConnectionPool.UnLockList;
  end;
  if Result<>Nil then
     exit;
  //Nothing connected!! Reconnect
  // T is element 0 after loop
  if assigned(T.FPGConn) then
    PQreset(T.FPGConn)
  else
    T.FPGConn := PQconnectdb(pchar(FConnectString));
  CheckConnectionStatus(T.FPGConn);
  if CharSet <> '' then
    PQsetClientEncoding(T.FPGConn, pchar(CharSet));
  result:=T.FPGConn;
end;


function TPQConnection.Fetch(cursor : TSQLCursor) : boolean;

begin
  with cursor as TPQCursor do
    begin
    inc(CurTuple);
    Result := (PQntuples(res)>CurTuple);
    end;
end;

function TPQConnection.LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;

const NBASE=10000;
      DAYS_PER_MONTH=30;

type TNumericRecord = record
       Digits : SmallInt;
       Weight : SmallInt;
       Sign   : SmallInt;
       Scale  : Smallint;
     end;
     TIntervalRec = packed record
       time  : int64;
       day   : longint;
       month : longint;
     end;
     TMacAddrRec = packed record
       a, b, c, d, e, f: byte;
     end;
     TInetRec = packed record
       family : byte;
       bits   : byte;
       is_cidr: byte;
       nb     : byte;
       ipaddr : array[1..16] of byte;
     end;

var
  x,i           : integer;
  s             : string;
  li            : Longint;
  CurrBuff      : pchar;
  dbl           : pdouble;
  cur           : currency;
  NumericRecord : ^TNumericRecord;
  guid          : TGUID;
  bcd           : TBCD;
  macaddr       : ^TMacAddrRec;
  inet          : ^TInetRec;

begin
  Createblob := False;
  with cursor as TPQCursor do
    begin
    x := GetFieldBinding(FieldDef)^.Index;

    // Joost, 5 jan 2006: I disabled the following, since it's useful for
    // debugging, but it also slows things down. In principle things can only go
    // wrong when FieldDefs is changed while the dataset is opened. A user just
    // shoudn't do that. ;) (The same is done in IBConnection)
    //if PQfname(Res, x) <> FieldDef.Name then
    //  DatabaseErrorFmt(SFieldNotFound,[FieldDef.Name],self);

    if pqgetisnull(res,CurTuple,x)=1 then
      result := false
    else
      begin
      CurrBuff := pqgetvalue(res,CurTuple,x);

      result := true;

      case FieldDef.DataType of
        ftInteger, ftSmallint, ftLargeInt :
          case PQfsize(res, x) of  // postgres returns big-endian numbers
            sizeof(int64) : pint64(buffer)^ := BEtoN(pint64(CurrBuff)^); // INT8
            sizeof(integer) : pinteger(buffer)^ := BEtoN(pinteger(CurrBuff)^); // INT4
            sizeof(smallint) : psmallint(buffer)^ := BEtoN(psmallint(CurrBuff)^); // INT2
          end; {case}
        ftFloat :
          case PQfsize(res, x) of  // postgres returns big-endian numbers
            sizeof(int64) :  // FLOAT8
              pint64(buffer)^ := BEtoN(pint64(CurrBuff)^);
            sizeof(integer) :  // FLOAT4
              begin
              li := BEtoN(pinteger(CurrBuff)^);
              pdouble(buffer)^ := psingle(@li)^
              end;
          end; {case}
        ftString, ftFixedChar :
          begin
          case PQftype(res, x) of
            Oid_MacAddr:
            begin
              macaddr := Pointer(CurrBuff);
              li := FormatBuf(Buffer^, FieldDef.Size, '%.2x:%.2x:%.2x:%.2x:%.2x:%.2x', 29,
                    [macaddr^.a,macaddr^.b,macaddr^.c,macaddr^.d,macaddr^.e,macaddr^.f]);
            end;
            Oid_Inet:
            begin
              inet := Pointer(CurrBuff);
              if inet^.nb = 4 then
                li := FormatBuf(Buffer^, FieldDef.Size, '%d.%d.%d.%d', 11,
                      [inet^.ipaddr[1],inet^.ipaddr[2],inet^.ipaddr[3],inet^.ipaddr[4]])
              else if inet^.nb = 16 then
                li := FormatBuf(Buffer^, FieldDef.Size, '%x%.2x:%x%.2x:%x%.2x:%x%.2x:%x%.2x:%x%.2x:%x%.2x:%x%.2x', 55,
                      [inet^.ipaddr[1],inet^.ipaddr[2],inet^.ipaddr[3],inet^.ipaddr[4],inet^.ipaddr[5],inet^.ipaddr[6],inet^.ipaddr[7],inet^.ipaddr[8],inet^.ipaddr[9],inet^.ipaddr[10],inet^.ipaddr[11],inet^.ipaddr[12],inet^.ipaddr[13],inet^.ipaddr[14],inet^.ipaddr[15],inet^.ipaddr[16]])
              else
                li := 0;
            end
            else
            begin
              li := pqgetlength(res,curtuple,x);
              if li > FieldDef.Size then li := FieldDef.Size;
              Move(CurrBuff^, Buffer^, li);
            end;
          end;
          pchar(Buffer + li)^ := #0;
          end;
        ftBlob, ftMemo :
          CreateBlob := True;
        ftDate :
          begin
          dbl := pointer(buffer);
          dbl^ := BEtoN(plongint(CurrBuff)^) + 36526;
          end;
        ftDateTime, ftTime :
          begin
          dbl := pointer(buffer);
          if FIntegerDateTimes then
            dbl^ := BEtoN(pint64(CurrBuff)^) / 1000000
          else
            pint64(dbl)^ := BEtoN(pint64(CurrBuff)^);
          case PQftype(res, x) of
            Oid_Timestamp, Oid_TimestampTZ:
              dbl^ := dbl^ + 3.1558464E+009; // postgres counts seconds elapsed since 1-1-2000
            Oid_Interval:
              dbl^ := dbl^ + BEtoN(plongint(CurrBuff+ 8)^) * SecsPerDay
                           + BEtoN(plongint(CurrBuff+12)^) * SecsPerDay * DAYS_PER_MONTH;
          end;
          dbl^ := dbl^ / SecsPerDay;
          // Now convert the mathematically-correct datetime to the
          // illogical windows/delphi/fpc TDateTime:
          if (dbl^ <= 0) and (frac(dbl^) < 0) then
            dbl^ := trunc(dbl^)-2-frac(dbl^);
          end;
        ftBCD, ftFmtBCD:
          begin
          NumericRecord := pointer(CurrBuff);
          NumericRecord^.Digits := BEtoN(NumericRecord^.Digits);
          NumericRecord^.Weight := BEtoN(NumericRecord^.Weight);
          NumericRecord^.Sign := BEtoN(NumericRecord^.Sign);
          NumericRecord^.Scale := BEtoN(NumericRecord^.Scale);
          inc(pointer(currbuff),sizeof(TNumericRecord));
          if (NumericRecord^.Digits = 0) and (NumericRecord^.Scale = 0) then // = NaN, which is not supported by Currency-type, so we return NULL
            result := false
          else if FieldDef.DataType = ftBCD then
            begin
            cur := 0;
            for i := 0 to NumericRecord^.Digits-1 do
              begin
              cur := cur + beton(pword(CurrBuff)^) * intpower(NBASE, NumericRecord^.weight-i);
              inc(pointer(CurrBuff),2);
              end;
            if NumericRecord^.Sign <> 0 then cur := -cur;
            Move(Cur, Buffer^, sizeof(currency));
            end
          else //ftFmtBCD
            begin
            bcd := 0;
            for i := 0 to NumericRecord^.Digits-1 do
              begin
              BCDAdd(bcd, beton(pword(CurrBuff)^) * intpower(NBASE, NumericRecord^.weight-i), bcd);
              inc(pointer(CurrBuff),2);
              end;
            if NumericRecord^.Sign <> 0 then BCDNegate(bcd);
            Move(bcd, Buffer^, sizeof(bcd));
            end;
          end;
        ftCurrency  :
          begin
          dbl := pointer(buffer);
          dbl^ := BEtoN(PInt64(CurrBuff)^) / 100;
          end;
        ftBoolean:
          pchar(buffer)[0] := CurrBuff[0];
        ftGuid:
          begin
          Move(CurrBuff^, guid, sizeof(guid));
          guid.D1:=BEtoN(guid.D1);
          guid.D2:=BEtoN(guid.D2);
          guid.D3:=BEtoN(guid.D3);
          s:=GUIDToString(guid);
          StrPLCopy(PChar(Buffer), s, FieldDef.Size);
          end
        else
          result := false;
      end;
      end;
    end;
end;

procedure TPQConnection.UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string);

var qry : TSQLQuery;
    relname : string;

begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet);

  if (length(TableName)>2) and (TableName[1]='"') and (TableName[length(TableName)]='"') then
    relname := QuotedStr(AnsiDequotedStr(TableName, '"'))
  else
    relname := 'lower(' + QuotedStr(TableName) + ')';  // unquoted names are stored lower case in PostgreSQL which is incompatible with the SQL standard

  qry := tsqlquery.Create(nil);
  qry.transaction := Transaction;
  qry.database := Self;
  with qry do
    begin
    ReadOnly := True;
    sql.clear;

    sql.add('select '+
              'ic.relname as indexname,  '+
              'tc.relname as tablename, '+
              'ia.attname, '+
              'i.indisprimary, '+
              'i.indisunique '+
            'from '+
              'pg_attribute ta, '+
              'pg_attribute ia, '+
              'pg_class tc, '+
              'pg_class ic, '+
              'pg_index i '+
            'where '+
              '(i.indrelid = tc.oid) and '+
              '(ta.attrelid = tc.oid) and '+
              '(ia.attrelid = i.indexrelid) and '+
              '(ic.oid = i.indexrelid) and '+
              '(ta.attnum = i.indkey[ia.attnum-1]) and '+
              '(tc.relname = ' + relname + ') '+
            'order by '+
              'ic.relname;');
    open;
    end;
  while not qry.eof do with IndexDefs.AddIndexDef do
    begin
    Name := trim(qry.fields[0].asstring);
    Fields := trim(qry.Fields[2].asstring);
    If qry.fields[3].asboolean then options := options + [ixPrimary];
    If qry.fields[4].asboolean then options := options + [ixUnique];
    qry.next;
    while (name = qry.fields[0].asstring) and (not qry.eof) do
      begin
      Fields := Fields + ';' + trim(qry.Fields[2].asstring);
      qry.next;
      end;
    end;
  qry.close;
  qry.free;
end;

function TPQConnection.GetSchemaInfoSQL(SchemaType: TSchemaType;
  SchemaObjectName, SchemaPattern: string): string;

var s : string;

begin
  // select * from information_schema.tables with 
  // where table_schema [not] in ('pg_catalog','information_schema') may be better.
  // But the following should work:
  case SchemaType of
    stTables     : s := 'select '+
                          'relfilenode        as recno, '+
                          'current_database() as catalog_name, '+
                          'nspname            as schema_name, '+
                          'relname            as table_name, '+
                          '0                  as table_type '+
                        'from pg_class c '+
                          'left join pg_namespace n on c.relnamespace=n.oid '+
                        'where (relkind=''r'') and not (nspname in (''pg_catalog'',''information_schema''))' +
                        'order by relname';

    stSysTables  : s := 'select '+
                          'relfilenode        as recno, '+
                          'current_database() as catalog_name, '+
                          'nspname            as schema_name, '+
                          'relname            as table_name, '+
                          '0                  as table_type '+
                        'from pg_class c '+
                          'left join pg_namespace n on c.relnamespace=n.oid '+
                        'where (relkind=''r'') and nspname in ((''pg_catalog'',''information_schema'')) ' + // only system tables
                        'order by relname';

    stColumns    : s := 'select '+
                          'a.attnum           as recno, '+
                          'current_database() as catalog_name, '+
                          'nspname            as schema_name, '+
                          'c.relname          as table_name, '+
                          'a.attname          as column_name, '+
                          'a.attnum           as column_position, '+
                          '0                  as column_type, '+
                          'a.atttypid         as column_datatype, '+
                          't.typname          as column_typename, '+
                          '0                  as column_subtype, '+
                          '0                  as column_precision, '+
                          '0                  as column_scale, '+
                          'a.atttypmod        as column_length, '+
                          'not a.attnotnull   as column_nullable '+
                        'from pg_class c '+
                          'join pg_attribute a on c.oid=a.attrelid '+
                          'join pg_type t on t.oid=a.atttypid '+
                          'left join pg_namespace n on c.relnamespace=n.oid '+
                          // This can lead to problems when case-sensitive tablenames are used.
                        'where (a.attnum>0) and (not a.attisdropped) and (upper(c.relname)=''' + Uppercase(SchemaObjectName) + ''') '+
                        'order by a.attname';
  else
    s := inherited;
  end; {case}
  result := s;
end;

function TPQConnection.GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
begin
  Result := Format('SELECT nextval(''%s'')', [SequenceName]);
end;

procedure TPQConnection.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction);
var
  x             : integer;
  li            : Longint;
begin
  with cursor as TPQCursor do
    begin
    x := FieldBinding[FieldDef.FieldNo-1].Index;
    li := pqgetlength(res,curtuple,x);
    ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer,li);
    Move(pqgetvalue(res,CurTuple,x)^, ABlobBuf^.BlobBuffer^.Buffer^, li);
    ABlobBuf^.BlobBuffer^.Size := li;
    end;
end;

function TPQConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  if assigned(cursor) and assigned((cursor as TPQCursor).res) then
    Result := StrToIntDef(PQcmdTuples((cursor as TPQCursor).res),-1)
  else
    Result := -1;
end;

function TPQConnection.GetConnectionInfo(InfoType: TConnInfoType): string;
begin
  Result:='';
  try
    {$IFDEF LinkDynamically}
    InitialisePostgres3;
    {$ENDIF}
    case InfoType of
      citServerType:
        Result:=TPQConnectionDef.TypeName;
      citServerVersion,
      citServerVersionString:
        if Connected then
          Result:=format('%6.6d', [PQserverVersion(GetHandle)]);
      citClientName:
        Result:=TPQConnectionDef.LoadedLibraryName;
    else
      Result:=inherited GetConnectionInfo(InfoType);
    end;
  finally
    {$IFDEF LinkDynamically}
    ReleasePostgres3;
    {$ENDIF}
  end;
end;


{ TPQConnectionDef }

class function TPQConnectionDef.TypeName: String;
begin
  Result:='PostgreSQL';
end;

class function TPQConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  Result:=TPQConnection;
end;

class function TPQConnectionDef.Description: String;
begin
  Result:='Connect to a PostgreSQL database directly via the client library';
end;

class function TPQConnectionDef.DefaultLibraryName: String;
begin
  {$IfDef LinkDynamically}
  Result:=pqlib;
  {$else}
  Result:='';
  {$endif}
end;

class function TPQConnectionDef.LoadFunction: TLibraryLoadFunction;
begin
  {$IfDef LinkDynamically}
  Result:=@InitialisePostgres3;
  {$else}
  Result:=Nil;
  {$endif}
end;

class function TPQConnectionDef.UnLoadFunction: TLibraryUnLoadFunction;
begin
  {$IfDef LinkDynamically}
  Result:=@ReleasePostgres3;
  {$else}
  Result:=Nil;
  {$endif}
end;

class function TPQConnectionDef.LoadedLibraryName: string;
begin
  {$IfDef LinkDynamically}
  Result:=Postgres3LoadedLibrary;
  {$else}
  Result:='';
  {$endif}
end;

initialization
  RegisterConnection(TPQConnectionDef);
finalization
  UnRegisterConnection(TPQConnectionDef);
end.
