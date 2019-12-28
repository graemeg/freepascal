{$IFDEF MYSQL57_UP}
  {$DEFINE MYSQL56_UP}
{$ENDIF}
{$IFDEF MYSQL56_UP}
  {$DEFINE MYSQL55_UP}
{$ENDIF}
{$IFDEF MYSQL55_UP}
  {$DEFINE MYSQL51_UP}
{$ENDIF}
{$IFDEF MYSQL51_UP}
  {$DEFINE MYSQL50_UP}
{$ENDIF}

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,bufdataset,sqldb,db,ctypes,fmtbcd,
{$IFDEF mysql57}
  mysql57dyn;
{$ELSE}
{$IFDEF mysql56}
  mysql56dyn;
{$ELSE}
{$IFDEF mysql55}
  mysql55dyn;
{$ELSE}
{$IFDEF mysql51}
   mysql51dyn;
{$ELSE}  
  {$IfDef mysql50}
    mysql50dyn;
  {$ELSE}
    {$IfDef mysql41}
      mysql41dyn;
    {$ELSE}
      mysql40dyn;
    {$EndIf}
  {$EndIf}
{$endif}
{$endif}
{$ENDIF}
{$ENDIF}

Const
  MySQLVersion =
{$IFDEF mysql57}
    '5.7';
{$ELSE}
{$IFDEF mysql56}
    '5.6';
{$ELSE}
{$IFDEF mysql55}
    '5.5';
{$ELSE}
{$IFDEF mysql51}
    '5.1';
{$else}    
  {$IfDef mysql50}
    '5.0';
  {$ELSE}
    {$IfDef mysql41}
      '4.1';
    {$ELSE}
      '4.0';
    {$EndIf}
  {$EndIf}
{$endif}
{$endif}
{$ENDIF}
{$ENDIF}

  MariaDBVersion =
{$IFDEF mysql57}
    '10.';
{$ELSE}
{$IFDEF mysql56}   // MariaDB 10.0 is compatible with MySQL 5.6
    '10.';
{$ELSE} // MariaDB 5.1..5.5 presumably report the same version number as MySQL
    MySQLVersion;
{$ENDIF}
{$ENDIF}

Type
  TTransactionName = Class(TSQLHandle)
  protected
  end;

  { TCursorName }

  TCursorName = Class(TSQLCursor)
  protected
    FRes: PMYSQL_RES;                   { Record pointer }
    // Statement with param placeholders $1 $2 etc.
    FPreparedStatement : String;
    // Statement with param placeholders replaced with actual values.
    FActualStatement : String;
    Row : MYSQL_ROW;
    Lengths : pculong;                  { Lengths of the columns of the current row }
    RowsAffected : QWord;
    LastInsertID : QWord;
    ParamBinding : TParamBinding;
    ParamReplaceString : String;
    MapDSRowToMSQLRow  : array of integer;
  end;

  { TConnectionName }

  TConnectionName = class (TSQLConnection)
  private
    FSkipLibraryVersionCheck : Boolean;
    FHostInfo: String;
    FServerInfo: String;
    FMySQL : PMySQL;
{$IFDEF MYSQL50_UP}
    FConnectionCharsetInfo: MY_CHARSET_INFO;
{$ENDIF}
    function GetClientInfo: string;
    function GetServerStatus: String;
    procedure ConnectMySQL(var HMySQL: PMySQL);
    procedure ExecuteDirectMySQL(const query : string);
    function InternalStrToBCD(C: pchar; Len: integer): tBCD;
    function InternalStrToCurrency(C: pchar; Len: integer): Currency;
    function InternalStrToDate(C: pchar; Len: integer): TDateTime;
    function InternalStrToDateTime(C: pchar; Len: integer): TDateTime;
    function InternalStrToFloat(C: pchar; Len: integer): Extended;
    function InternalStrToInt(C: pchar; Len: integer): integer;
    function InternalStrToInt64(C: pchar; Len: integer): Int64;
    function InternalStrToTime(C: pchar; Len: integer): TDateTime;
    function StrToMSecs(C: pchar; Len: integer): Word;
{$IFDEF MYSQL40}
    function InternalStrToTimeStamp(C: pchar; Len: integer): TDateTime;
{$ENDIF}
  protected
    Procedure ConnectToServer; virtual;
    Procedure SelectDatabase; virtual;
    function MySQLDataType(AField: PMYSQL_FIELD; var NewType: TFieldType; var NewSize: Integer): Boolean;
    function MySQLWriteData(AField: PMYSQL_FIELD; FieldDef: TFieldDef; Source, Dest: PChar; Len: integer; out CreateBlob : boolean): Boolean;
    function EscapeString(const Str : string) : string;

    // SQLConnection methods
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    function GetHandle : pointer; override;
    function GetConnectionCharSet: string; override;

    function GetAsSQLText(Param : TParam) : string; overload; override;

    Function AllocateCursorHandle : TSQLCursor; override;
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); override;
    Function AllocateTransactionHandle : TSQLHandle; override;

    function StrToStatementType(s : string) : TStatementType; override;
    procedure PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams); override;
    procedure UnPrepareStatement(cursor:TSQLCursor); override;
    procedure FreeFldBuffers(cursor : TSQLCursor); override;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction;AParams : TParams); override;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TfieldDefs); override;
    function Fetch(cursor : TSQLCursor) : boolean; override;
    function LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor;ATransaction : TSQLTransaction); override;
    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function StartdbTransaction(trans : TSQLHandle; AParams : string) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    procedure RollBackRetaining(trans : TSQLHandle); override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function RefreshLastInsertID(Query : TCustomSQLQuery; Field : TField): Boolean; override;
  Public
    constructor Create(AOwner : TComponent); override;
{$IFNDEF MYSQL50_UP}
    procedure GetFieldNames(const TableName : string; List :  TStrings); override;
    procedure GetTableNames(List : TStrings; SystemTables : Boolean = false); override;
{$ENDIF}
    function GetConnectionInfo(InfoType:TConnInfoType): string; override;
    Function GetInsertID: int64;
    procedure CreateDB; override;
    procedure DropDB; override;
    Property ServerInfo : String Read FServerInfo;
    Property HostInfo : String Read FHostInfo;
    property ClientInfo: string read GetClientInfo;
    property ServerStatus : String read GetServerStatus;
  published
    Property SkipLibraryVersionCheck : Boolean Read FSkipLibraryVersionCheck Write FSkipLibraryVersionCheck;
    property DatabaseName;
    property HostName;
    property KeepConnection;
    property LoginPrompt;
    property Params;
    property Port stored false;
    property OnLogin;
  end;

  { TMySQLConnectionDef }

  TMySQLConnectionDef = Class(TConnectionDef)
    Class Function TypeName : String; override;
    Class Function ConnectionClass : TSQLConnectionClass; override;
    Class Function Description : String; override;
    Class Function DefaultLibraryName : String; override;
    Class Function LoadFunction : TLibraryLoadFunction; override;
    Class Function UnLoadFunction : TLibraryUnLoadFunction; override;
    Class Function LoadedLibraryName : string; override;
  end;


  {$IFDEF mysql57}
    TMySQL57Connection = Class(TConnectionName);
    TMySQL57ConnectionDef = Class(TMySQLConnectionDef);
    TMySQL57Transaction = Class(TTransactionName);
    TMySQL57Cursor = Class(TCursorName);
  {$ELSE}
  {$IFDEF mysql56}
    TMySQL56Connection = Class(TConnectionName);
    TMySQL56ConnectionDef = Class(TMySQLConnectionDef);
    TMySQL56Transaction = Class(TTransactionName);
    TMySQL56Cursor = Class(TCursorName);
  {$ELSE}
  {$ifdef mysql55}
    TMySQL55Connection = Class(TConnectionName);
    TMySQL55ConnectionDef = Class(TMySQLConnectionDef);
    TMySQL55Transaction = Class(TTransactionName);
    TMySQL55Cursor = Class(TCursorName);
  {$else}
    {$IfDef mysql51}
      TMySQL51Connection = Class(TConnectionName);
      TMySQL51ConnectionDef = Class(TMySQLConnectionDef);
      TMySQL51Transaction = Class(TTransactionName);
      TMySQL51Cursor = Class(TCursorName);
    {$ELSE}
      {$IfDef mysql50}
        TMySQL50Connection = Class(TConnectionName);
        TMySQL50ConnectionDef = Class(TMySQLConnectionDef);
        TMySQL50Transaction = Class(TTransactionName);
        TMySQL50Cursor = Class(TCursorName);
      {$ELSE}
        {$IfDef mysql41}
          TMySQL41Connection = Class(TConnectionName);
          TMySQL41ConnectionDef = Class(TMySQLConnectionDef);
          TMySQL41Transaction = Class(TTransactionName);
          TMySQL41Cursor = Class(TCursorName);
        {$ELSE}
          TMySQL40Connection = Class(TConnectionName);
          TMySQL40ConnectionDef = Class(TMySQLConnectionDef);
          TMySQL40Transaction = Class(TTransactionName);
          TMySQL40Cursor = Class(TCursorName);
        {$EndIf}
      {$endif}
    {$EndIf}
  {$ENDIF}
  {$ENDIF}
  {$ENDIF}

implementation

uses
  dbconst,
  StrUtils,
  DateUtils;

const
  Mysql_Option_Names : array[mysql_option] of string = ('MYSQL_OPT_CONNECT_TIMEOUT','MYSQL_OPT_COMPRESS',
     'MYSQL_OPT_NAMED_PIPE','MYSQL_INIT_COMMAND',
     'MYSQL_READ_DEFAULT_FILE','MYSQL_READ_DEFAULT_GROUP',
     'MYSQL_SET_CHARSET_DIR','MYSQL_SET_CHARSET_NAME',
     'MYSQL_OPT_LOCAL_INFILE','MYSQL_OPT_PROTOCOL',
     'MYSQL_SHARED_MEMORY_BASE_NAME','MYSQL_OPT_READ_TIMEOUT',
     'MYSQL_OPT_WRITE_TIMEOUT','MYSQL_OPT_USE_RESULT',
     'MYSQL_OPT_USE_REMOTE_CONNECTION','MYSQL_OPT_USE_EMBEDDED_CONNECTION',
     'MYSQL_OPT_GUESS_CONNECTION','MYSQL_SET_CLIENT_IP',
     'MYSQL_SECURE_AUTH'
{$IFDEF MYSQL50_UP}
     ,'MYSQL_REPORT_DATA_TRUNCATION', 'MYSQL_OPT_RECONNECT'
{$IFDEF mysql51_UP}
     ,'MYSQL_OPT_SSL_VERIFY_SERVER_CERT'
{$IFDEF mysql55_UP}
     ,'MYSQL_PLUGIN_DIR', 'MYSQL_DEFAULT_AUTH'
{$IFDEF MYSQL56_UP}
     ,'MYSQL_OPT_BIND'
     ,'MYSQL_OPT_SSL_KEY', 'MYSQL_OPT_SSL_CERT', 'MYSQL_OPT_SSL_CA', 'MYSQL_OPT_SSL_CAPATH', 'MYSQL_OPT_SSL_CIPHER', 'MYSQL_OPT_SSL_CRL', 'MYSQL_OPT_SSL_CRLPATH'
     ,'MYSQL_OPT_CONNECT_ATTR_RESET', 'MYSQL_OPT_CONNECT_ATTR_ADD', 'MYSQL_OPT_CONNECT_ATTR_DELETE'
     ,'MYSQL_SERVER_PUBLIC_KEY'
     ,'MYSQL_ENABLE_CLEARTEXT_PLUGIN'
     ,'MYSQL_OPT_CAN_HANDLE_EXPIRED_PASSWORDS'
{$IFDEF MYSQL57_UP}
     ,'MYSQL_OPT_SSL_ENFORCE'
{$ENDIF}
{$ENDIF}
{$ENDIF}
{$ENDIF}
{$ENDIF}
     );

Resourcestring
  SErrServerConnectFailed = 'Server connect failed.';
  SErrSetCharsetFailed = 'Failed to set connection character set: %s';
  SErrDatabaseSelectFailed = 'Failed to select database: %s';
  SErrDatabaseCreate = 'Failed to create database: %s';
  SErrDatabaseDrop = 'Failed to drop database: %s';
  SErrNoData = 'No data for record';
  SErrExecuting = 'Error executing query: %s';
  SErrFetchingdata = 'Error fetching row data: %s';
  SErrGettingResult = 'Error getting result set: %s';
  SErrNoQueryResult = 'No result from query.';
  SErrVersionMismatch = '%s can not work with the installed MySQL client version: Expected (%s), got (%s).';
  SErrSettingParameter = 'Error setting parameter "%s"';

Procedure MySQLError(R : PMySQL; Msg: String; Comp : TComponent);

Var
  MySQLError, MySQLState : String;
  MySQLErrno: integer;

begin
  If (R<>Nil) then
    begin
    MySQLError:=StrPas(mysql_error(R));
    MySQLErrno:=mysql_errno(R);
    MySQLState:=StrPas(mysql_sqlstate(R));
    end
  else
    begin
    MySQLError:='';
    MySQLErrno:=0;
    MySQLState:='';
    end;

  raise ESQLDatabaseError.CreateFmt(Msg, [MySQLError], Comp, MySQLErrno, MySQLState);
end;

function MysqlOption(const OptionName: string; out AMysql_Option: mysql_option) : boolean;
var AMysql_Option_i: mysql_option;
begin
  result := false;
  for AMysql_Option_i:=low(AMysql_Option) to high(AMysql_Option) do
    if sametext(Mysql_Option_Names[AMysql_Option_i],OptionName) then
      begin
      result := true;
      AMysql_Option:=AMysql_Option_i;
      break;
      end;
end;

{ TConnectionName }

function TConnectionName.StrToStatementType(s : string) : TStatementType;

begin
  s:=Lowercase(s);
  if (s='analyze') or (s='check') or (s='checksum') or (s='optimize') or (s='repair') or (s='show') then
    exit(stSelect)
  else if s='call' then
    exit(stExecProcedure)
  else
    Result := inherited StrToStatementType(s);
end;


function TConnectionName.GetClientInfo: string;

begin
  // To make it possible to call this if there's no connection yet
  InitialiseMysql;
  Try  
    Result:=strpas(mysql_get_client_info());
  Finally  
    ReleaseMysql;
  end;  
end;

function TConnectionName.GetServerStatus: String;
begin
  CheckConnected;
  Result := mysql_stat(FMYSQL);
end;

Function TConnectionName.GetInsertID: int64;
begin
  CheckConnected;
  Result:=mysql_insert_id(GetHandle);
end;

procedure TConnectionName.ConnectMySQL(var HMySQL: PMySQL);

Var
  APort : Cardinal;
  i,e: integer;
  AMysql_Option: mysql_option;
  OptStr: string;
  OptInt: cuint;
  Opt: pointer;

begin
  HMySQL := mysql_init(HMySQL);
  APort:=Abs(StrToIntDef(Params.Values['Port'],0));

  for i := 0 to Params.Count-1 do
    begin
    if MysqlOption(Params.Names[i],AMysql_Option) then
      begin
      OptStr:=Params.ValueFromIndex[i];
      val(OptStr,OptInt,e);
      if e=0 then
        Opt := @OptInt
      else
        Opt := pchar(OptStr);
      if mysql_options(HMySQL,AMysql_Option,Opt) <> 0 then
          MySQLError(HMySQL,Format(SErrSettingParameter,[Params.Names[i]]),Self);
      end;
    end;

  if mysql_real_connect(HMySQL,PChar(HostName),PChar(UserName),PChar(Password),Nil,APort,Nil,CLIENT_MULTI_RESULTS) = nil then //CLIENT_MULTI_RESULTS is required by CALL SQL statement(executes stored procedure), that produces result sets
    MySQLError(HMySQL,SErrServerConnectFailed,Self);

  if (trim(CharSet) <> '') then
    // major_version*10000 + minor_version *100 + sub_version
    if (50007 <= mysql_get_server_version(HMySQL)) then
      begin
      // Only available for MySQL 5.0.7 and later...
      if mysql_set_character_set(HMySQL, PChar(CharSet)) <> 0 then
        MySQLError(HMySQL,SErrSetCharsetFailed,Self);
      end
    else
      if mysql_query(HMySQL,PChar('SET NAMES ''' + EscapeString(CharSet) +'''')) <> 0 then
        MySQLError(HMySQL,SErrExecuting,Self);
end;

function TConnectionName.GetAsSQLText(Param: TParam) : string;

begin
  if (not assigned(Param)) or Param.IsNull then
    Result := 'Null'
  else if Param.DataType in [ftString,ftFixedChar,ftMemo] then
    Result := '''' + EscapeString(GetAsString(Param)) + ''''
  else if Param.DataType in [ftBlob,ftBytes,ftVarBytes] then
    Result := '''' + EscapeString(Param.AsString) + ''''
  else
    Result := inherited GetAsSQLText(Param);
end;


Procedure TConnectionName.ConnectToServer;
begin
  ConnectMySQL(FMySQL);
  FServerInfo := strpas(mysql_get_server_info(FMYSQL));
  FHostInfo := strpas(mysql_get_host_info(FMYSQL));
{$IFDEF MYSQL50_UP}
  mysql_get_character_set_info(FMYSQL, @FConnectionCharsetInfo);
{$ENDIF}
end;


Procedure TConnectionName.SelectDatabase;
begin
  if mysql_select_db(FMySQL,pchar(DatabaseName))<>0 then
    MySQLError(FMySQL,SErrDatabaseSelectFailed,Self);
end;


procedure TConnectionName.CreateDB;

begin
  ExecuteDirectMySQL('CREATE DATABASE ' +DatabaseName);
end;

procedure TConnectionName.DropDB;

begin
  ExecuteDirectMySQL('DROP DATABASE ' +DatabaseName);
end;

procedure TConnectionName.ExecuteDirectMySQL(const query : string);

var AMySQL      : PMySQL;

begin
  CheckDisConnected;

  InitialiseMysql;

  try
    AMySQL := nil;
    ConnectMySQL(AMySQL);
    try
      if mysql_query(AMySQL,pchar(query))<>0 then
        MySQLError(AMySQL,SErrExecuting,Self);
    finally
      mysql_close(AMySQL);
    end;
  finally
    ReleaseMysql;
  end;
end;

function TConnectionName.EscapeString(const Str: string): string;

var Len : integer;

begin
  SetLength(result,length(str)*2+1);
  Len := mysql_real_escape_string(FMySQL,pchar(Result),pchar(Str),length(Str));
  SetLength(result,Len);
end;

procedure TConnectionName.DoInternalConnect;
var
  FullVersion: string;
begin
  InitialiseMysql;
  if not SkipLibraryVersionCheck then
    begin
    FullVersion:=strpas(mysql_get_client_info());
    // Version string should start with version number:
    // Note: in case of MariaDB version mismatch: tough luck, we report MySQL
    // version only.
    if (pos(MySQLVersion, FullVersion) <> 1) and
       (pos(MariaDBVersion, FullVersion) <> 1) then
      Raise EInOutError.CreateFmt(SErrVersionMisMatch,[ClassName,MySQLVersion,FullVersion]);
    end;  
  inherited DoInternalConnect;
  ConnectToServer;
  SelectDatabase;
end;

procedure TConnectionName.DoInternalDisconnect;
begin
  inherited DoInternalDisconnect;
  mysql_close(FMySQL);
  FMySQL:=Nil;
  ReleaseMysql;
end;

function TConnectionName.GetHandle: pointer;
begin
  Result:=FMySQL;
end;

function TConnectionName.GetConnectionCharSet: string;
begin
  Result:=StrPas(mysql_character_set_name(FMySQL));
end;

Function TConnectionName.AllocateCursorHandle: TSQLCursor;
begin
  {$IFDEF mysql57}
    Result:=TMySQL57Cursor.Create;
  {$ELSE}
  {$IFDEF mysql56}
    Result:=TMySQL56Cursor.Create;
  {$ELSE}
  {$IfDef mysql55}
    Result:=TMySQL55Cursor.Create;
  {$ELSE}
    {$IfDef mysql51}
      Result:=TMySQL51Cursor.Create;
    {$ELSE}
      {$IfDef mysql50}
        Result:=TMySQL50Cursor.Create;
      {$ELSE}
        {$IfDef mysql41}
          Result:=TMySQL41Cursor.Create;
        {$ELSE}
          Result:=TMySQL40Cursor.Create;
        {$EndIf}
      {$EndIf}
    {$EndIf}
  {$EndIf}
  {$ENDIF}
  {$ENDIF}
end;

Procedure TConnectionName.DeAllocateCursorHandle(var cursor : TSQLCursor);

begin
  FreeAndNil(cursor);
end;

Function TConnectionName.AllocateTransactionHandle: TSQLHandle;
begin
//  Result:=TTransactionName.Create;
  Result := nil;
end;

procedure TConnectionName.PrepareStatement(cursor: TSQLCursor;
  ATransaction: TSQLTransaction; buf: string;AParams : TParams);
begin
//  if assigned(AParams) and (AParams.count > 0) then
//    DatabaseError('Parameters (not) yet supported for the MySQL SqlDB connection.',self);
  With Cursor as TCursorName do
    begin
    FPreparedStatement:=Buf;
    if assigned(AParams) and (AParams.count > 0) then
      FPreparedStatement := AParams.ParseSQL(FPreparedStatement,false,sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions,psSimulated,paramBinding,ParamReplaceString);
    FPrepared:=True;
    end
end;

procedure TConnectionName.UnPrepareStatement(cursor: TSQLCursor);
Var
  C : TCursorName;

begin
  C:=Cursor as TCursorName;
  if assigned(C.FRes) then  //ExecSQL with dataset returned
    begin
    mysql_free_result(C.FRes);
    C.FRes:=nil;
    end;
end;

procedure TConnectionName.FreeFldBuffers(cursor: TSQLCursor);

Var
  C : TCursorName;

begin
  C:=Cursor as TCursorName;
  if assigned(C.FRes) then
    begin
    mysql_free_result(C.FRes);
    C.FRes:=Nil;
    end;
  C.FInitFieldDef:=True;
  SetLength(c.MapDSRowToMSQLRow,0);
  inherited;
end;

procedure TConnectionName.Execute(cursor: TSQLCursor;
  atransaction: tSQLtransaction;AParams : TParams);

Var
  C : TCursorName;
  i : integer;
  ParamNames,ParamValues : array of string;
  Res: PMYSQL_RES;
  Status : Integer;

begin
  C:=Cursor as TCursorName;
  If (C.FRes=Nil) then
    begin
    if Assigned(AParams) and (AParams.count > 0) then
      begin
      setlength(ParamNames,AParams.Count);
      setlength(ParamValues,AParams.Count);
      for i := 0 to AParams.count -1 do
        begin
        ParamNames[AParams.count-i-1] := C.ParamReplaceString+inttostr(AParams[i].Index+1);
        ParamValues[AParams.count-i-1] := GetAsSQLText(AParams[i]);
        end;
      C.FActualStatement := stringsreplace(C.FPreparedStatement,ParamNames,ParamValues,[rfReplaceAll]);
      end
    else
      C.FActualStatement:=C.FPreparedStatement;

    if LogEvent(detParamValue) then
      LogParams(AParams);
    if LogEvent(detExecute) then
      Log(detExecute, C.FPreparedStatement);
    if LogEvent(detActualSQL) then
      Log(detActualSQL,C.FActualStatement);

    if mysql_query(FMySQL,Pchar(C.FActualStatement))<>0 then
      begin
      if not ForcedClose then
        MySQLError(FMYSQL,SErrExecuting,Self)
      else //don't return a resulset. We are shutting down, not opening.
        begin
        C.RowsAffected:=0;
        C.FSelectable:= False;
        C.FRes:=nil;
        end;
      end
    else
      begin
      C.RowsAffected := mysql_affected_rows(FMYSQL);
      C.LastInsertID := mysql_insert_id(FMYSQL);
      C.FSelectable  := False;
      repeat
        Res:=mysql_store_result(FMySQL); //returns a null pointer also if the statement didn't return a result set
        if mysql_errno(FMySQL)<>0 then
          begin
          if not ForcedClose then
            MySQLError(FMySQL, SErrGettingResult, Self)
          else
            begin
            C.RowsAffected:=0;
            C.FSelectable:= False;
            C.FRes:=nil;
            break;
            end;
          end;
        if Res<>nil then
          begin
          mysql_free_result(C.FRes);
          C.FRes:=Res;
          C.FSelectable:=True;
          end;
        Status:=mysql_next_result(FMySQL);
        if (Status>0) then
          begin
          if not ForcedClose then
            MySQLError(FMySQL, SErrGettingResult, Self)
          else
            begin
            C.RowsAffected:=0;
            C.FSelectable:= False;
            C.FRes:=nil;
            break;
            end;
          end;
      until (Status<>0);
      end;
    end;
end;

function TConnectionName.MySQLDataType(AField: PMYSQL_FIELD; var NewType: TFieldType; var NewSize: Integer): Boolean;
var
  ASize: culong;
  ADecimals: cuint;
begin
  Result := True;
  ASize := AField^.length;
  NewSize := 0;
  case AField^.ftype of
    FIELD_TYPE_LONGLONG:
      begin
      NewType := ftLargeint;
      end;
    FIELD_TYPE_TINY, FIELD_TYPE_SHORT, FIELD_TYPE_YEAR:
      begin
      if AField^.flags and UNSIGNED_FLAG <> 0 then
        NewType := ftWord
      else
        NewType := ftSmallint;
      end;
    FIELD_TYPE_LONG, FIELD_TYPE_INT24:
      begin
      if AField^.flags and AUTO_INCREMENT_FLAG <> 0 then
        NewType := ftAutoInc
      else
        NewType := ftInteger;
      end;
{$ifdef mysql50_up}
    FIELD_TYPE_NEWDECIMAL,
{$endif}
    FIELD_TYPE_DECIMAL: 
      begin
        ADecimals:=AField^.decimals;
        if (ADecimals < 5) and (ASize-2-ADecimals < 15) then //ASize is display size i.e. with sign and decimal point
          NewType := ftBCD
        else if (ADecimals = 0) and (ASize < 20) then
          NewType := ftLargeInt
        else
          NewType := ftFmtBCD;
        NewSize := ADecimals;
      end;
    FIELD_TYPE_FLOAT, FIELD_TYPE_DOUBLE:
      begin
      NewType := ftFloat;
      end;
    FIELD_TYPE_TIMESTAMP, FIELD_TYPE_DATETIME:
      begin
      NewType := ftDateTime;
      end;
    FIELD_TYPE_DATE:
      begin
      NewType := ftDate;
      end;
    FIELD_TYPE_TIME:
      begin
      NewType := ftTime;
      end;
    FIELD_TYPE_VAR_STRING, FIELD_TYPE_STRING, FIELD_TYPE_ENUM, FIELD_TYPE_SET:
      begin
      // Since mysql server version 5.0.3 string-fields with a length of more
      // then 256 characters are suported
      if AField^.ftype = FIELD_TYPE_STRING then
        NewType := ftFixedChar
      else
        NewType := ftString;
{$IFDEF MYSQL50_UP}
      if AField^.charsetnr = 63 then begin //BINARY vs. CHAR, VARBINARY vs. VARCHAR
        if NewType = ftFixedChar then
          NewType := ftBytes
        else
          NewType := ftVarBytes;
        NewSize := ASize;
      end
      else
        NewSize := ASize div FConnectionCharsetInfo.mbmaxlen;
{$ELSE}
      NewSize := ASize;
{$ENDIF}
      end;
    FIELD_TYPE_TINY_BLOB..FIELD_TYPE_BLOB:
      begin
{$IFDEF MYSQL50_UP}
      if AField^.charsetnr = 63 then //character set is binary
        NewType := ftBlob
      else
        NewType := ftMemo;
{$ELSE}
      NewType := ftBlob;
{$ENDIF}
      end;
{$IFDEF MYSQL50_UP}
    FIELD_TYPE_BIT:
      NewType := ftLargeInt;
{$ENDIF}
  else
    Result := False;
  end;
end;

procedure TConnectionName.AddFieldDefs(cursor: TSQLCursor;
  FieldDefs: TfieldDefs);

var
  C : TCursorName;
  I, FC: Integer;
  field: PMYSQL_FIELD;
  DFT: TFieldType;
  DFS: Integer;

begin
//  Writeln('MySQL: Adding fielddefs');
  C:=(Cursor as TCursorName);
  If (C.FRes=Nil) then
    begin
//    Writeln('res is nil');
    MySQLError(FMySQL,SErrNoQueryResult,Self);
    end;
//  Writeln('MySQL: have result');
  FC:=mysql_num_fields(C.FRes);
  SetLength(c.MapDSRowToMSQLRow,FC);

  For I := 0 to FC-1 do
    begin
    field := mysql_fetch_field_direct(C.FRES, I);
//    Writeln('MySQL: creating fielddef ',I+1);

    if MySQLDataType(field, DFT, DFS) then
      begin
      AddFieldDef(FieldDefs, I+1, field^.name, DFT, DFS, -1,
                    False,
                    (field^.flags and (AUTO_INCREMENT_FLAG or NOT_NULL_FLAG {$IFDEF MYSQL50_UP}or NO_DEFAULT_VALUE_FLAG{$ENDIF})) = (NOT_NULL_FLAG {$IFDEF MYSQL50_UP}or NO_DEFAULT_VALUE_FLAG{$ENDIF}),
                    False);
      c.MapDSRowToMSQLRow[I] := I;
      end
    end;
//  Writeln('MySQL: Finished adding fielddefs');
end;

function TConnectionName.Fetch(cursor: TSQLCursor): boolean;

Var
  C : TCursorName;

begin
  C:=Cursor as TCursorName;
  C.Row:=MySQL_Fetch_row(C.FRes);
  Result:=(C.Row<>Nil);
  if Result then
    C.Lengths := mysql_fetch_lengths(C.FRes)
  else
    C.Lengths := nil;
end;

function TConnectionName.LoadField(cursor : TSQLCursor;
  FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;

var
  field: PMYSQL_FIELD;
  C : TCursorName;
  i : integer;

begin
//  Writeln('LoadFieldsFromBuffer');
  C:=Cursor as TCursorName;
  if (C.Row=nil) or (C.Lengths=nil) then
     begin
  //   Writeln('LoadFieldsFromBuffer: row=nil');
     MySQLError(FMySQL,SErrFetchingData,Self);
     end;

  i := c.MapDSRowToMSQLRow[FieldDef.FieldNo-1];
  field := mysql_fetch_field_direct(C.FRES, i);

  Result := MySQLWriteData(field, FieldDef, C.Row[i], Buffer, C.Lengths[i], CreateBlob);
end;

procedure TConnectionName.LoadBlobIntoBuffer(FieldDef: TFieldDef;
  ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction);
var
  C : TCursorName;
  i : integer;
  len : longint;
begin
  C:=Cursor as TCursorName;
  if (C.Row=nil) or (C.Lengths=nil) then
    MySQLError(FMySQL,SErrFetchingData,Self);

  i := c.MapDSRowToMSQLRow[FieldDef.FieldNo-1];
  len := C.Lengths[i];

  ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer, len);
  Move(C.Row[i]^, ABlobBuf^.BlobBuffer^.Buffer^, len);
  ABlobBuf^.BlobBuffer^.Size := len;
end;

function TConnectionName.InternalStrToInt(C: pchar; Len: integer): integer;

Var
  S : String;

begin
  Result := 0;
  if (Len=0) or (C=Nil) then
    exit;
  SetString(S,C,Len);
  Result:=StrToInt(S);
end;

function TConnectionName.InternalStrToInt64(C: pchar; Len: integer): Int64;

Var
  S : String;

begin
  Result := 0;
  if (Len=0) or (C=Nil) then
    exit;
  SetString(S,C,Len);
  Result:=StrToInt64(S);
end;

function TConnectionName.InternalStrToFloat(C: pchar; Len: integer): Extended;

var
  Tmp: string;

begin
  SetString(Tmp, C, Len);
  if Tmp='' then
    Exit(0);
  Result := StrToFloat(Tmp, FSQLFormatSettings);
end;

function TConnectionName.InternalStrToCurrency(C: pchar; Len: integer): Currency;

var
  Tmp: string;

begin
  SetString(Tmp, C, Len);
  if Tmp='' then
    Exit(0);
  Result := StrToCurr(Tmp, FSQLFormatSettings);
end;

function TConnectionName.InternalStrToBCD(C: pchar; Len: integer): tBCD;

var
  Tmp: string;

begin
  SetString(Tmp, C, Len);
  if Tmp='' then
    Exit(0);
  Result := StrToBCD(Tmp, FSQLFormatSettings);
end;

function TConnectionName.InternalStrToDate(C: pchar; Len: integer): TDateTime;

var
  EY, EM, ED: Word;

begin
  if Len=0 then
    Exit(0);
  if Len<10 then
    raise EConvertError.Create('Invalid date string');
  EY := InternalStrToInt(C,4);
  EM := InternalStrToInt(C+5,2);
  ED := InternalStrToInt(C+8,2);
  if (EY = 0) or (EM = 0) or (ED = 0) then
    Result:=0
  else
    Result:=EncodeDate(EY, EM, ED);
end;

function TConnectionName.StrToMSecs(C: pchar; Len: integer): Word;
{$IFDEF MYSQL56_UP}
var I: Integer;
    d, MSecs: double;
{$ENDIF}
begin
{$IFDEF MYSQL56_UP}
  // datetime(n), where n is fractional seconds precision (between 0 and 6)
  MSecs := 0;
  d := 100;
  for I := 1 to Len do
    begin
    case C^ of
      '0'..'9': MSecs := MSecs + (ord(C^)-ord('0'))*d;
      #0: break;
    end;
    d := d / 10;
    Inc(C);
    end;
  Result := Round(MSecs);
{$ELSE}
  Result := 0;
{$ENDIF}
end;

function TConnectionName.InternalStrToDateTime(C: pchar; Len: integer): TDateTime;

var
  EY, EM, ED: Word;
  EH, EN, ES, EMS: Word;

begin
  if Len=0 then
    Exit(0);
  if Len<19 then
    raise EConvertError.Create('Invalid datetime string');
  EY := InternalStrToInt(C,4);
  EM := InternalStrToInt(C+5,2);
  ED := InternalStrToInt(C+8,2);
  EH := InternalStrToInt(C+11, 2);
  EN := InternalStrToInt(C+14, 2);
  ES := InternalStrToInt(C+17, 2);
  if Len>20 then
    EMS := StrToMSecs(C+20, Len-20)
  else
    EMS := 0;
  if (EY = 0) or (EM = 0) or (ED = 0) then
    Result := 0
  else
    Result := EncodeDate(EY, EM, ED);
  Result := ComposeDateTime(Result, EncodeTime(EH, EN, ES, EMS));
end;

function TConnectionName.InternalStrToTime(C: pchar; Len: integer): TDateTime;

var
  EH, EM, ES, EMS: Word;
  M: PChar;
  I: Integer;

begin
  if Len=0 then
    Exit(0);
  if Len<8 then
    raise EConvertError.Create('Invalid time string');
  //hours can be 2 or 3 digits
  M:=C;
  for I := 1 to Len do
  begin
    if M^=':' then
      break;
    Inc(M);
  end;
  if M^<>':' then
    raise EConvertError.Create('Invalid time string');
  EH := InternalStrToInt(C, NativeInt(M-C));
  EM := InternalStrToInt(M+1, 2);
  ES := InternalStrToInt(M+4, 2);
  if Len>NativeInt(M-C)+7 then
    EMS := StrToMSecs(M+7, Len-(NativeInt(M-C)+7))
  else
    EMS := 0;
  Result := EncodeTimeInterval(EH, EM, ES, EMS);
end;

{$IFDEF mysql40}
function TConnectionName.InternalStrToTimeStamp(C: pchar; Len: integer): TDateTime;

var
  EY, EM, ED: Word;
  EH, EN, ES: Word;

begin
  if Len=0 then
    Exit(0);
  if Len<14 then
    raise EConvertError.Create('Invalid timestamp string');
  EY := InternalStrToInt(C, 4);
  EM := InternalStrToInt(C+4, 2));
  ED := InternalStrToInt(C+6, 2));
  EH := InternalStrToInt(C+8, 2));
  EN := InternalStrToInt(C+10, 2));
  ES := InternalStrToInt(C+12, 2));
  if (EY = 0) or (EM = 0) or (ED = 0) then
    Result := 0
  else
    Result := EncodeDate(EY, EM, ED);
  Result := ComposeDateTime(Result, EncodeTime(EH, EN, ES, 0));
end;
{$ENDIF}

function TConnectionName.MySQLWriteData(AField: PMYSQL_FIELD; FieldDef: TFieldDef; Source, Dest: PChar; Len: integer; out CreateBlob : boolean): Boolean;

var
  VI: Integer;
  VL: LargeInt;
  VS: Smallint;
  VW: Word;
  VF: Double;
  VC: Currency;
  VD: TDateTime;
  VB: TBCD;

begin
  Result := False;
  CreateBlob := False;
  if Source = Nil then // If the pointer is NULL, the field is NULL
    exit;

  case FieldDef.DataType of
    ftSmallint:
      begin
      VS := InternalStrToInt(Source, Len);
      Move(VS, Dest^, SizeOf(Smallint));
      end;
    ftWord:
      begin
      VW := InternalStrToInt(Source, Len);
      Move(VW, Dest^, SizeOf(Word));
      end;
    ftInteger, ftAutoInc:
      begin
      VI := InternalStrToInt(Source, Len);
      Move(VI, Dest^, SizeOf(Integer));
      end;
    ftLargeInt:
      begin
      {$IFDEF MYSQL50_UP}
      if AField^.ftype = FIELD_TYPE_BIT then
        begin
        VL := 0;
        for VI := 0 to Len-1 do
          VL := VL * 256 + PByte(Source+VI)^;
        end
      else
      {$ENDIF}
      VL := InternalStrToInt64(Source, Len);
      Move(VL, Dest^, SizeOf(LargeInt));
      end;
    ftFloat:
      begin
      VF := InternalStrToFloat(Source, Len);
      Move(VF, Dest^, SizeOf(Double));
      end;
    ftBCD:
      begin
      VC := InternalStrToCurrency(Source, Len);
      Move(VC, Dest^, SizeOf(Currency));
      end;
    ftFmtBCD:
      begin
      VB := InternalStrToBCD(Source, Len);
      Move(VB, Dest^, SizeOf(TBCD));
      end;
    ftDate:
      begin
      VD := InternalStrToDate(Source, Len);
      Move(VD, Dest^, SizeOf(TDateTime));
      end;
    ftTime:
      begin
      VD := InternalStrToTime(Source, Len);
      Move(VD, Dest^, SizeOf(TDateTime));
      end;
    ftDateTime:
      begin
      {$IFDEF mysql40}
      if AField^.ftype = FIELD_TYPE_TIMESTAMP then
        VD := InternalStrToTimeStamp(Source, Len)
      else
      {$ENDIF}
        VD := InternalStrToDateTime(Source, Len);
      Move(VD, Dest^, SizeOf(TDateTime));
      end;
    ftString, ftFixedChar:
      // String-fields which can contain more then dsMaxStringSize characters
      // are mapped to ftBlob fields, while their mysql-datatype is FIELD_TYPE_BLOB
      begin
      if Len > FieldDef.Size*FieldDef.CharSize then Len := FieldDef.Size*FieldDef.CharSize;
      Move(Source^, Dest^, Len);
      (Dest+Len)^ := #0;
      end;
    ftVarBytes:
      begin
      if Len > FieldDef.Size then Len := FieldDef.Size;
      PWord(Dest)^ := Len;
      Move(Source^, (Dest+sizeof(Word))^, Len);
      end;
    ftBytes:
      begin
      if Len > FieldDef.Size then Len := FieldDef.Size;
      Move(Source^, Dest^, Len);
      end;
    ftBlob, ftMemo:
      CreateBlob := True;
  end;
  Result := True;
end;

procedure TConnectionName.UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string);

var qry : TSQLQuery;

begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet);

  qry := TSQLQuery.Create(nil);
  qry.Transaction := Transaction;
  qry.Database := Self;
  try
    with qry do
      begin
      ParseSQL := False;
      SQL.Clear;
      SQL.Add('show index from ' +  TableName);
      Open;
      end;
    while not qry.Eof do with IndexDefs.AddIndexDef do
      begin
      Name := trim(qry.FieldByName('Key_name').AsString);
      Fields := trim(qry.FieldByName('Column_name').AsString);
      If Name = 'PRIMARY' then Options := Options + [ixPrimary];
      If qry.FieldByName('Non_unique').AsInteger = 0 then Options := Options + [ixUnique];
      qry.Next;
      while (Name = trim(qry.FieldByName('Key_name').AsString)) and (not qry.Eof) do
        begin
        Fields := Fields + ';' + trim(qry.FieldByName('Column_name').AsString);
        qry.Next;
        end;
      end;
    qry.Close;
  finally
    qry.Free;
  end;
end;

function TConnectionName.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  if assigned(cursor) then
    // Compile this without range-checking. RowsAffected can be -1, although
    // it's an unsigned integer. (small joke from the mysql-guys)
    // Without range-checking this goes ok. If Range is turned on, this results
    // in range-check errors.
    Result := (cursor as TCursorName).RowsAffected
  else
    Result := -1;
end;

function TConnectionName.RefreshLastInsertID(Query: TCustomSQLQuery; Field: TField): Boolean;
begin
  Field.AsLargeInt:=GetInsertID;
  Result := True;
end;

constructor TConnectionName.Create(AOwner: TComponent);
const SingleBackQoutes: TQuoteChars = ('`','`');
begin
  inherited Create(AOwner);
  FConnOptions := [sqEscapeRepeat, sqEscapeSlash, sqImplicitTransaction, sqLastInsertID];
  FieldNameQuoteChars:=SingleBackQoutes;
  FMySQL := Nil;
end;

{$IFNDEF MYSQL50_UP}
procedure TConnectionName.GetFieldNames(const TableName: string; List: TStrings);
begin
  GetDBInfo(stColumns,TableName,'field',List);
end;

procedure TConnectionName.GetTableNames(List: TStrings; SystemTables: Boolean);
begin
  GetDBInfo(stTables,'','tables_in_'+DatabaseName,List)
end;
{$ENDIF}

function TConnectionName.GetConnectionInfo(InfoType: TConnInfoType): string;
begin
  Result:='';
  try
    InitialiseMysql;
    case InfoType of
      citServerType:
        Result:='MySQL';
      citServerVersion:
        if Connected then
          Result:=format('%6.6d', [mysql_get_server_version(FMySQL)]);
      citServerVersionString:
        if Connected then
          Result:=mysql_get_server_info(FMySQL);
      citClientVersion:
        Result:=format('%6.6d', [mysql_get_client_version()]);
      citClientName:
        Result:=TMySQLConnectionDef.LoadedLibraryName;
      else
        Result:=inherited GetConnectionInfo(InfoType);
    end;
  finally
    ReleaseMysql;
  end;
end;

function TConnectionName.GetTransactionHandle(trans: TSQLHandle): pointer;
begin
  Result:=Nil;
end;

function TConnectionName.Commit(trans: TSQLHandle): boolean;
begin
  //mysql_commit(FMySQL);
  Result := (mysql_query(FMySQL, 'COMMIT') = 0) or ForcedClose;
  if not Result then
    MySQLError(FMySQL, SErrExecuting, Self);
end;

function TConnectionName.RollBack(trans: TSQLHandle): boolean;
begin
  //mysql_rollback(FMySQL);
  Result := (mysql_query(FMySQL, 'ROLLBACK') = 0) or ForcedClose;
  if not Result then
    MySQLError(FMySQL, SErrExecuting, Self);
end;

function TConnectionName.StartdbTransaction(trans: TSQLHandle; AParams : string): boolean;
begin
  Result := mysql_query(FMySQL, 'START TRANSACTION') = 0;
  if not Result then
    MySQLError(FMySQL, SErrExecuting, Self);
end;

procedure TConnectionName.CommitRetaining(trans: TSQLHandle);
begin
  {$IFDEF MYSQL50_UP}
  if mysql_query(FMySQL, 'COMMIT AND CHAIN') <> 0 then
    MySQLError(FMySQL, SErrExecuting, Self);
  {$ELSE}
  if mysql_query(FMySQL, 'COMMIT') <> 0 then
    MySQLError(FMySQL, SErrExecuting, Self);
  if mysql_query(FMySQL, 'START TRANSACTION') <> 0 then
    MySQLError(FMySQL, SErrExecuting, Self);
  {$ENDIF}
end;

procedure TConnectionName.RollBackRetaining(trans: TSQLHandle);
begin
  {$IFDEF MYSQL50_UP}
  if mysql_query(FMySQL, 'ROLLBACK AND CHAIN') <> 0 then
    MySQLError(FMySQL, SErrExecuting, Self);
  {$ELSE}
  if mysql_query(FMySQL, 'ROLLBACK') <> 0 then
    MySQLError(FMySQL, SErrExecuting, Self);
  if mysql_query(FMySQL, 'START TRANSACTION') <> 0 then
    MySQLError(FMySQL, SErrExecuting, Self);
  {$ENDIF}
end;

function TConnectionName.GetSchemaInfoSQL(SchemaType: TSchemaType;
  SchemaObjectName, SchemaPattern: string): string;

begin
  case SchemaType of
    {$IFDEF MYSQL50_UP}
    stTables     : result := 'SELECT * FROM INFORMATION_SCHEMA.TABLES WHERE TABLE_SCHEMA=SCHEMA() AND TABLE_TYPE IN (''BASE TABLE'',''VIEW'')';
    stColumns    : result := 'SELECT * FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_SCHEMA=SCHEMA() AND TABLE_NAME='+QuotedStr(SchemaObjectName);
    {$ELSE}
    stTables     : result := 'show tables';
    stColumns    : result := 'show columns from ' + EscapeString(SchemaObjectName);
    {$ENDIF}
  else
                   result := inherited;
  end; {case}
end;


{ TMySQLConnectionDef }

class function TMySQLConnectionDef.TypeName: String;
begin
  Result:='MySQL '+MySQLVersion;
end;

class function TMySQLConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  {$IFDEF mysql57}
    Result:=TMySQL57Connection;
  {$ELSE}
  {$IFDEF mysql56}
    Result:=TMySQL56Connection;
  {$ELSE}
  {$IfDef mysql55}
     Result:=TMySQL55Connection;
  {$ELSE}
    {$IfDef mysql51}
      Result:=TMySQL51Connection;
    {$ELSE}
      {$IfDef mysql50}
        Result:=TMySQL50Connection;
      {$ELSE}
        {$IfDef mysql41}
          Result:=TMySQL41Connection;
        {$ELSE}
          Result:=TMySQL40Connection;
        {$EndIf}
      {$EndIf}
    {$endif}
  {$endif}
  {$ENDIF}
  {$ENDIF}
end;

class function TMySQLConnectionDef.Description: String;
begin
  Result:='Connect to a MySQL '+MySQLVersion+' database directly via the client library';
end;

class function TMySQLConnectionDef.DefaultLibraryName: String;
begin
  Result:=mysqlvlib;
end;

class function TMySQLConnectionDef.LoadFunction: TLibraryLoadFunction;
begin
  Result:=@InitialiseMySQL;
end;

class function TMySQLConnectionDef.UnLoadFunction: TLibraryUnLoadFunction;
begin
  Result:=@ReleaseMySQL;
end;

class function TMySQLConnectionDef.LoadedLibraryName: string;
begin
  Result:=MysqlLoadedLibrary;
end;

{$IFDEF mysql57}
  initialization
    RegisterConnection(TMySQL57ConnectionDef);
  finalization
    UnRegisterConnection(TMySQL57ConnectionDef);
{$ELSE}
{$IFDEF mysql56}
  initialization
    RegisterConnection(TMySQL56ConnectionDef);
  finalization
    UnRegisterConnection(TMySQL56ConnectionDef);
{$ELSE}
{$IfDef mysql55}
  initialization
    RegisterConnection(TMySQL55ConnectionDef);
  finalization
    UnRegisterConnection(TMySQL55ConnectionDef);
{$else}
  {$IfDef mysql51}
    initialization
      RegisterConnection(TMySQL51ConnectionDef);
    finalization
      UnRegisterConnection(TMySQL51ConnectionDef);
  {$ELSE}
    {$IfDef mysql50}
      initialization
        RegisterConnection(TMySQL50ConnectionDef);
      finalization
        UnRegisterConnection(TMySQL50ConnectionDef);
    {$ELSE}
      {$IfDef mysql41}
        initialization
          RegisterConnection(TMySQL41ConnectionDef);
        finalization
          UnRegisterConnection(TMySQL41ConnectionDef);
      {$ELSE}
        initialization
          RegisterConnection(TMySQL40ConnectionDef);
        finalization
          UnRegisterConnection(TMySQL40ConnectionDef);
      {$EndIf}
    {$EndIf}
  {$ENDIF}
{$endif}
{$ENDIF}
{$ENDIF}

end.
