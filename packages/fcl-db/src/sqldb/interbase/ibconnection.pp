unit IBConnection;

{$mode objfpc}{$H+}

{$Define LinkDynamically}

interface

uses
  Classes, SysUtils, sqldb, db, dbconst, bufdataset,
{$IfDef LinkDynamically}
  ibase60dyn;
{$Else}
  ibase60;
{$EndIf}

const
  DEFDIALECT = 3;
<<<<<<< HEAD
<<<<<<< HEAD
  MAXBLOBSEGMENTSIZE = 65535; //Maximum number of bytes that fit in a blob segment.
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type
  TDatabaseInfo = record
    Dialect             : integer; //Dialect set in database
    ODSMajorVersion     : integer; //On-Disk Structure version of file
    ServerVersion       : string;  //Representation of major.minor (.build)
    ServerVersionString : string;  //Complete version string, including name, platform
  end;

  EIBDatabaseError = class(ESQLDatabaseError)
    public
      property GDSErrorCode: integer read ErrorCode; deprecated 'Please use ErrorCode instead of GDSErrorCode'; // Nov 2014
  end;

  { TIBCursor }

  TIBCursor = Class(TSQLCursor)
    protected
    Status               : array [0..19] of ISC_STATUS;
    Statement            : pointer;
    SQLDA                : PXSQLDA;
    in_SQLDA             : PXSQLDA;
    ParamBinding         : array of integer;
    FieldBinding         : array of integer;
  end;

  TIBTrans = Class(TSQLHandle)
    protected
    TransactionHandle   : pointer;
    TPB                 : string;                // Transaction parameter buffer
    Status              : array [0..19] of ISC_STATUS;
  end;

  { TIBConnection }

  TIBConnection = class (TSQLConnection)
  private
<<<<<<< HEAD
    FSQLDatabaseHandle     : pointer;
    FStatus                : array [0..19] of ISC_STATUS;
    FDatabaseInfo          : TDatabaseInfo;
    FDialect               : integer;
    FBlobSegmentSize       : word; //required for backward compatibilty; not used

    procedure ConnectFB;

    procedure AllocSQLDA(var aSQLDA : PXSQLDA;Count : integer);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    // Metadata:
    procedure GetDatabaseInfo; //Queries for various information from server once connected
    procedure ResetDatabaseInfo; //Useful when disconnecting
    function GetDialect: integer;
    function GetODSMajorVersion: integer;
    function ParseServerVersion(const CompleteVersion: string): string; //Extract version info from complete version identification string

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure TranslateFldType(SQLType, SQLSubType, SQLLen, SQLScale : integer;
=======
    FSQLDatabaseHandle   : pointer;
    FStatus              : array [0..19] of ISC_STATUS;
    FDialect             : integer;
    FDBDialect           : integer;
    FBLobSegmentSize     : word;

    procedure ConnectFB;
    function GetDialect: integer;
    procedure AllocSQLDA(var aSQLDA : PXSQLDA;Count : integer);
    procedure TranslateFldType(SQLType, SQLLen, SQLScale : integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      var TrType : TFieldType; var TrLen : word);
>>>>>>> graemeg/cpstrnew
    // conversion methods
    procedure TranslateFldType(SQLType, SQLSubType, SQLLen, SQLScale : integer;
      var TrType : TFieldType; var TrLen : word);
    procedure GetDateTime(CurrBuff, Buffer : pointer; AType : integer);
    procedure SetDateTime(CurrBuff: pointer; PTime : TDateTime; AType : integer);
    procedure GetFloat(CurrBuff, Buffer : pointer; Size : Byte);
    procedure SetFloat(CurrBuff: pointer; Dbl: Double; Size: integer);

    procedure CheckError(ProcName : string; Status : PISC_STATUS);
    procedure SetParameters(cursor : TSQLCursor; aTransation : TSQLTransaction; AParams : TParams);
    procedure FreeSQLDABuffer(var aSQLDA : PXSQLDA);
    function  IsDialectStored: boolean;
  protected
    procedure DoConnect; override;
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    function GetHandle : pointer; override;

    Function AllocateCursorHandle : TSQLCursor; override;
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); override;
    Function AllocateTransactionHandle : TSQLHandle; override;

    procedure PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams); override;
    procedure UnPrepareStatement(cursor : TSQLCursor); override;
    procedure FreeFldBuffers(cursor : TSQLCursor); override;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); override;
    procedure AddFieldDefs(cursor: TSQLCursor;FieldDefs : TfieldDefs); override;
    function Fetch(cursor : TSQLCursor) : boolean; override;
    function LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; override;
    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function StartDBTransaction(trans : TSQLHandle; AParams : string) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    procedure RollBackRetaining(trans : TSQLHandle); override;
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    function GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string; override;
=======
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
>>>>>>> graemeg/fixes_2_2
=======
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
>>>>>>> origin/fixes_2_2
  public
    constructor Create(AOwner : TComponent); override;
    function GetConnectionInfo(InfoType:TConnInfoType): string; override;
    procedure CreateDB; override;
    procedure DropDB; override;
<<<<<<< HEAD
    //Segment size is not used in the code; property kept for backward compatibility
    property BlobSegmentSize : word read FBlobSegmentSize write FBlobSegmentSize; deprecated;
    property ODSMajorVersion : integer read GetODSMajorVersion; //ODS major version number; influences database compatibility/feature level.
=======
    property BlobSegmentSize : word read FBlobSegmentSize write FBlobSegmentSize;
    function GetDBDialect: integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  published
    property DatabaseName;
    property Dialect : integer read GetDialect write FDialect stored IsDialectStored default DEFDIALECT;
    property KeepConnection;
    property LoginPrompt;
    property Params;
    property OnLogin;
  end;
  
  { TIBConnectionDef }
  
  TIBConnectionDef = Class(TConnectionDef)
    Class Function TypeName : String; override;
    Class Function ConnectionClass : TSQLConnectionClass; override;
    Class Function Description : String; override;
    Class Function DefaultLibraryName : String; override;
    Class Function LoadFunction : TLibraryLoadFunction; override;
    Class Function UnLoadFunction : TLibraryUnLoadFunction; override;
    Class Function LoadedLibraryName: string; override;
  end;
                  
implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  StrUtils, FmtBCD;

const
  SQL_BOOLEAN_INTERBASE = 590;
  SQL_BOOLEAN_FIREBIRD = 32764;
  INVALID_DATA = -1;
=======
  strutils;
>>>>>>> graemeg/cpstrnew
=======
  strutils;
>>>>>>> graemeg/cpstrnew
=======
  strutils;
>>>>>>> graemeg/cpstrnew
=======
  strutils;
>>>>>>> origin/cpstrnew


procedure TIBConnection.CheckError(ProcName : string; Status : PISC_STATUS);
var
<<<<<<< HEAD
<<<<<<< HEAD
  ErrorCode : longint;
  Msg, SQLState : string;
  Buf : array [0..1023] of char;

begin
  if ((Status[0] = 1) and (Status[1] <> 0)) then
  begin
    ErrorCode := Status[1];
{$IFDEF LinkDynamically}
    if assigned(fb_sqlstate) then // >= Firebird 2.5
    begin
      fb_sqlstate(Buf, Status);
      SQLState := StrPas(Buf);
    end;
{$ENDIF}
    Msg := '';
    while isc_interprete(Buf, @Status) > 0 do
      Msg := Msg + LineEnding + ' -' + StrPas(Buf);
    raise EIBDatabaseError.CreateFmt('%s : %s', [ProcName,Msg], Self, ErrorCode, SQLState);
=======
=======
>>>>>>> origin/fixes_2_2
  buf : array [0..1023] of char;
  Msg : string;
  E   : EIBDatabaseError;
  Err : longint;
  
begin
  if ((Status[0] = 1) and (Status[1] <> 0)) then
  begin
    Err := Status[1];
    msg := '';
    while isc_interprete(Buf, @Status) > 0 do
      Msg := Msg + LineEnding +' -' + StrPas(Buf);
    E := EIBDatabaseError.CreateFmt('%s : %s : %s',[self.Name,ProcName,Msg]);
    E.GDSErrorCode := Err;
    Raise E;
>>>>>>> graemeg/fixes_2_2
  end;
end;


constructor TIBConnection.Create(AOwner : TComponent);

begin
  inherited;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FConnOptions := FConnOptions + [sqSupportParams, sqEscapeRepeat, sqSupportReturning];
  FBlobSegmentSize := 65535; //Shows we're using the maximum segment size
  FDialect := INVALID_DATA;
  ResetDatabaseInfo;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FConnOptions := FConnOptions + [sqSupportParams] + [sqEscapeRepeat];
  FieldNameQuoteChars:=DoubleQuotes;
  FBLobSegmentSize := 80;
  FDialect := -1;
  FDBDialect := -1;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  FConnOptions := FConnOptions + [sqSupportParams] + [sqEscapeRepeat] + [sqQuoteFieldnames];
  FBLobSegmentSize := 80;
  FDialect := -1;
  FDBDialect := -1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


function TIBConnection.GetTransactionHandle(trans : TSQLHandle): pointer;
begin
  Result := (trans as TIBtrans).TransactionHandle;
end;

function TIBConnection.Commit(trans : TSQLHandle) : boolean;
begin
  result := false;
  with (trans as TIBTrans) do
    if isc_commit_transaction(@Status[0], @TransactionHandle) <> 0 then
      CheckError('Commit', Status)
    else result := true;
end;

function TIBConnection.RollBack(trans : TSQLHandle) : boolean;
begin
  result := false;
  if isc_rollback_transaction(@TIBTrans(trans).Status[0], @TIBTrans(trans).TransactionHandle) <> 0 then
    CheckError('Rollback', TIBTrans(trans).Status)
  else result := true;
end;

function TIBConnection.StartDBTransaction(trans: TSQLHandle; AParams: string
  ): boolean;
var
  DBHandle : pointer;
  tr       : TIBTrans;
  i        : integer;
  s        : string;
begin
  result := false;

  DBHandle := GetHandle;
  tr := trans as TIBtrans;
  with tr do
    begin
    TPB := chr(isc_tpb_version3);

    i := 1;
    s := ExtractSubStr(AParams,i,stdWordDelims);
    while s <> '' do
      begin
      if s='isc_tpb_write' then TPB := TPB + chr(isc_tpb_write)
      else if s='isc_tpb_read' then TPB := TPB + chr(isc_tpb_read)
      else if s='isc_tpb_consistency' then TPB := TPB + chr(isc_tpb_consistency)
      else if s='isc_tpb_concurrency' then TPB := TPB + chr(isc_tpb_concurrency)
      else if s='isc_tpb_read_committed' then TPB := TPB + chr(isc_tpb_read_committed)
      else if s='isc_tpb_rec_version' then TPB := TPB + chr(isc_tpb_rec_version)
      else if s='isc_tpb_no_rec_version' then TPB := TPB + chr(isc_tpb_no_rec_version)
      else if s='isc_tpb_wait' then TPB := TPB + chr(isc_tpb_wait)
      else if s='isc_tpb_nowait' then TPB := TPB + chr(isc_tpb_nowait)
      else if s='isc_tpb_shared' then TPB := TPB + chr(isc_tpb_shared)
      else if s='isc_tpb_protected' then TPB := TPB + chr(isc_tpb_protected)
      else if s='isc_tpb_exclusive' then TPB := TPB + chr(isc_tpb_exclusive)
      else if s='isc_tpb_lock_read' then TPB := TPB + chr(isc_tpb_lock_read)
      else if s='isc_tpb_lock_write' then TPB := TPB + chr(isc_tpb_lock_write)
      else if s='isc_tpb_verb_time' then TPB := TPB + chr(isc_tpb_verb_time)
      else if s='isc_tpb_commit_time' then TPB := TPB + chr(isc_tpb_commit_time)
      else if s='isc_tpb_ignore_limbo' then TPB := TPB + chr(isc_tpb_ignore_limbo)
      else if s='isc_tpb_autocommit' then TPB := TPB + chr(isc_tpb_autocommit)
      else if s='isc_tpb_restart_requests' then TPB := TPB + chr(isc_tpb_restart_requests)
      else if s='isc_tpb_no_auto_undo' then TPB := TPB + chr(isc_tpb_no_auto_undo);
      s := ExtractSubStr(AParams,i,stdWordDelims);

      end;

    TransactionHandle := nil;

    if isc_start_transaction(@Status[0], @TransactionHandle, 1,
       [@DBHandle, Length(TPB), @TPB[1]]) <> 0 then
      CheckError('StartTransaction',Status)
    else Result := True;
    end;
end;


procedure TIBConnection.CommitRetaining(trans : TSQLHandle);
begin
  with trans as TIBtrans do
    if isc_commit_retaining(@Status[0], @TransactionHandle) <> 0 then
      CheckError('CommitRetaining', Status);
end;

procedure TIBConnection.RollBackRetaining(trans : TSQLHandle);
begin
  with trans as TIBtrans do
    if isc_rollback_retaining(@Status[0], @TransactionHandle) <> 0 then
      CheckError('RollBackRetaining', Status);
end;


procedure TIBConnection.DropDB;

begin
  CheckDisConnected;

{$IfDef LinkDynamically}
  InitialiseIBase60;
{$EndIf}

  ConnectFB;

  if isc_drop_database(@FStatus[0], @FSQLDatabaseHandle) <> 0 then
    CheckError('DropDB', FStatus);

{$IfDef LinkDynamically}
  ReleaseIBase60;
{$EndIf}
end;


procedure TIBConnection.CreateDB;

var ASQLDatabaseHandle,
    ASQLTransactionHandle : pointer;
    CreateSQL : String;
    pagesize : String;
begin
  CheckDisConnected;
{$IfDef LinkDynamically}
  InitialiseIBase60;
{$EndIf}
  ASQLDatabaseHandle := nil;
  ASQLTransactionHandle := nil;

  CreateSQL := 'CREATE DATABASE ';
  if HostName <> '' then CreateSQL := CreateSQL + ''''+ HostName+':'+DatabaseName + ''''
    else CreateSQL := CreateSQL + '''' + DatabaseName + '''';
  if UserName <> '' then
    CreateSQL := CreateSQL + ' USER ''' + Username + '''';
  if Password <> '' then
    CreateSQL := CreateSQL + ' PASSWORD ''' + Password + '''';
  pagesize := params.Values['PAGE_SIZE'];
  if pagesize <> '' then
    CreateSQL := CreateSQL + ' PAGE_SIZE '+pagesize;
<<<<<<< HEAD
<<<<<<< HEAD
  if CharSet <> '' then
    CreateSQL := CreateSQL + ' DEFAULT CHARACTER SET ' + CharSet;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  if isc_dsql_execute_immediate(@FStatus[0],@ASQLDatabaseHandle,@ASQLTransactionHandle,length(CreateSQL),@CreateSQL[1],Dialect,nil) <> 0 then
    CheckError('CreateDB', FStatus);

  if isc_detach_database(@FStatus[0], @ASQLDatabaseHandle) <> 0 then
    CheckError('CreateDB', FStatus);
{$IfDef LinkDynamically}
  ReleaseIBase60;
{$EndIf}
end;

procedure TIBConnection.DoInternalConnect;
begin
{$IfDef LinkDynamically}
  InitialiseIBase60;
{$EndIf}
  inherited dointernalconnect;

  ConnectFB;
end;

procedure TIBConnection.DoInternalDisconnect;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Inherited;
  FDialect := INVALID_DATA;
=======
  FDialect := -1;
  FDBDialect := -1;
>>>>>>> graemeg/fixes_2_2
=======
  FDialect := -1;
  FDBDialect := -1;
>>>>>>> origin/fixes_2_2
  if not Connected then
  begin
    ResetDatabaseInfo;
    FSQLDatabaseHandle := nil;
    Exit;
  end;

  if isc_detach_database(@FStatus[0], @FSQLDatabaseHandle) <> 0 then
    CheckError('Close', FStatus);
{$IfDef LinkDynamically}
  ReleaseIBase60;
{$ELSE}
  // Shutdown embedded subsystem with timeout 300ms (Firebird 2.5+)
  // Required before unloading library; has no effect on non-embedded client
  if (pointer(fb_shutdown)<>nil) and (fb_shutdown(300,1)<>0) then
  begin
    //todo: log error; still try to unload library below as the timeout may have been insufficient
  end;
{$EndIf}
end;

function TIBConnection.GetConnectionInfo(InfoType: TConnInfoType): string;
begin
  result:='';
  {$IFDEF LinkDynamically}
  InitialiseIBase60;
  {$ENDIF}
  try
    case InfoType of
      citServerType:
        // Firebird returns own name in ServerVersion; Interbase 7.5 doesn't.
        if Pos('Firebird', FDatabaseInfo.ServerVersionString)=0 then
          result := 'Interbase'
        else
          result := 'Firebird';
      citServerVersion:
        // Firebird returns major.minor, Interbase major.minor.build
        result := FDatabaseInfo.ServerVersion;
      citServerVersionString:
        result := FDatabaseInfo.ServerVersionString;
      citClientName:
        result:=TIBConnectionDef.LoadedLibraryName;
    else
      //including citClientVersion, for which no single IB+FB and Win+*nux solution exists
      result:=inherited GetConnectionInfo(InfoType);
    end;
  finally
    {$IFDEF LinkDynamically}
    ReleaseIBase60;
    {$ENDIF}
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TIBConnection.GetDatabaseInfo;
// Asks server for multiple values
const
  ResBufHigh = 512; //hopefully enough to include version string as well.
=======
function TIBConnection.GetDBDialect: integer;
>>>>>>> graemeg/fixes_2_2
=======
function TIBConnection.GetDBDialect: integer;
>>>>>>> origin/fixes_2_2
var
  x : integer;
  Len : integer;
  ReqBuf : array [0..3] of byte;
  ResBuf : array [0..ResBufHigh] of byte; // should be big enough for version string etc
begin
<<<<<<< HEAD
<<<<<<< HEAD
  ResetDatabaseInfo;
  if Connected then
  begin
    ReqBuf[0] := isc_info_ods_version;
    ReqBuf[1] := isc_info_version;
    ReqBuf[2] := isc_info_db_sql_dialect;
    ReqBuf[3] := isc_info_end;
    if isc_database_info(@FStatus[0], @FSQLDatabaseHandle, Length(ReqBuf),
      pchar(@ReqBuf[0]), SizeOf(ResBuf), pchar(@ResBuf[0])) <> 0 then
        CheckError('CacheServerInfo', FStatus);
    x := 0;
    while x < ResBufHigh+1 do
=======
=======
>>>>>>> origin/fixes_2_2
  result := -1;
  if Connected then
    begin
    Buffer[0] := isc_info_db_sql_dialect;
    Buffer[1] := isc_info_end;
    if isc_database_info(@FStatus[0], @FSQLDatabaseHandle, Length(Buffer),
      pchar(@Buffer[0]), SizeOf(ResBuf), pchar(@ResBuf[0])) <> 0 then
        CheckError('SetDBDialect', FStatus);
    x := 0;
    while x < 40 do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      case ResBuf[x] of
        isc_info_db_sql_dialect :
          begin
          Inc(x);
          Len := isc_vax_integer(pchar(@ResBuf[x]), 2);
          Inc(x, 2);
<<<<<<< HEAD
<<<<<<< HEAD
          FDatabaseInfo.Dialect := isc_vax_integer(pchar(@ResBuf[x]), Len);
          Inc(x, Len);
          end;
        isc_info_ods_version :
          begin
          Inc(x);
          Len := isc_vax_integer(pchar(@ResBuf[x]), 2);
          Inc(x, 2);
          FDatabaseInfo.ODSMajorVersion := isc_vax_integer(pchar(@ResBuf[x]), Len);
          Inc(x, Len);
          end;
        isc_info_version :
          begin
          Inc(x);
          Len := isc_vax_integer(pchar(@ResBuf[x]), 2);
          Inc(x, 2);
          SetString(FDatabaseInfo.ServerVersionString, PAnsiChar(@ResBuf[x + 2]), Len-2);
          FDatabaseInfo.ServerVersion := ParseServerVersion(FDatabaseInfo.ServerVersionString);
          Inc(x, Len);
          end;
        isc_info_end, isc_info_error : Break;
        isc_info_truncated : Break; //result buffer too small; fix your code!
      else
        inc(x);
      end;
  end;
end;

procedure TIBConnection.ResetDatabaseInfo;
begin
  FDatabaseInfo.Dialect:=0;
  FDatabaseInfo.ODSMajorVersion:=0;
  FDatabaseInfo.ServerVersion:='';
  FDatabaseInfo.ServerVersionString:=''; // don't confuse applications with 'Firebird' or 'Interbase'
end;


function TIBConnection.GetODSMajorVersion: integer;
begin
  result:=FDatabaseInfo.ODSMajorVersion;
end;

function TIBConnection.ParseServerVersion(const CompleteVersion: string): string;
// String representation of integer version number derived from
// major.minor.build => should give e.g. 020501
const
  Delimiter = '.';
  DigitsPerNumber = 2;
  MaxNumbers = 3;
var
  BeginPos,EndPos,StartLook,i: integer;
  NumericPart: string;
begin
  result := '';
  // Ignore 6.x version number in front of "Firebird"
  StartLook := Pos('Firebird', CompleteVersion);
  if StartLook = 0 then
    StartLook := 1;
  BeginPos := 0;
  // Catch all numerics + decimal point:
  for i := StartLook to Length(CompleteVersion) do
  begin
    if (BeginPos > 0) and
      ((CompleteVersion[i] < '0') or (CompleteVersion[i] > '9')) and (CompleteVersion[i] <> '.') then
    begin
      EndPos := i - 1;
      break;
=======
          Result := isc_vax_integer(pchar(@ResBuf[x]), Len);
          Inc(x, Len);
          end;
=======
          Result := isc_vax_integer(pchar(@ResBuf[x]), Len);
          Inc(x, Len);
          end;
>>>>>>> origin/fixes_2_2
        isc_info_end : Break;
      else
        inc(x);
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
    end;
    if (BeginPos = 0) and
      (CompleteVersion[i] >= '0') and (CompleteVersion[i] <= '9') then
    begin
      BeginPos := i;
    end;
  end;
  if BeginPos > 0 then
  begin
    NumericPart := copy(CompleteVersion, BeginPos, 1+EndPos-BeginPos);
    BeginPos := 1;
    for i := 1 to MaxNumbers do
    begin
      EndPos := PosEx(Delimiter,NumericPart,BeginPos);
      if EndPos > 0 then
      begin
        result := result + rightstr(StringOfChar('0',DigitsPerNumber)+copy(NumericPart,BeginPos,EndPos-BeginPos),DigitsPerNumber);
        BeginPos := EndPos+1;
      end
      else
      begin
        result := result + rightstr(StringOfChar('0',DigitsPerNumber)+copy(NumericPart,BeginPos,Length(NumericPart)),DigitsPerNumber);
        break;
      end;
=======
>>>>>>> origin/fixes_2_2
    end;
    result := leftstr(result + StringOfChar('0',DigitsPerNumber * MaxNumbers), DigitsPerNumber * MaxNumbers);
  end;
end;


procedure TIBConnection.ConnectFB;
var
  ADatabaseName: String;
  DPB: string;
begin
  DPB := chr(isc_dpb_version1);
  if (UserName <> '') then
  begin
    DPB := DPB + chr(isc_dpb_user_name) + chr(Length(UserName)) + UserName;
    if (Password <> '') then
      DPB := DPB + chr(isc_dpb_password) + chr(Length(Password)) + Password;
  end;
  if (Role <> '') then
     DPB := DPB + chr(isc_dpb_sql_role_name) + chr(Length(Role)) + Role;
  if Length(CharSet) > 0 then
    DPB := DPB + Chr(isc_dpb_lc_ctype) + Chr(Length(CharSet)) + CharSet;

  FSQLDatabaseHandle := nil;
  if HostName <> '' then ADatabaseName := HostName+':'+DatabaseName
    else ADatabaseName := DatabaseName;
  if isc_attach_database(@FStatus[0], Length(ADatabaseName), @ADatabaseName[1],
    @FSQLDatabaseHandle,
         Length(DPB), @DPB[1]) <> 0 then
    CheckError('DoInternalConnect', FStatus);
end;

function TIBConnection.GetDialect: integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FDialect = INVALID_DATA then
  begin
    if FDatabaseInfo.Dialect=0 then
      Result := DEFDIALECT
    else
      Result := FDatabaseInfo.Dialect;
=======
=======
>>>>>>> origin/fixes_2_2
  if FDialect = -1 then
  begin
    if FDBDialect = -1 then
      Result := DEFDIALECT
    else
      Result := FDBDialect;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end else
    Result := FDialect;
end;

procedure TIBConnection.AllocSQLDA(var aSQLDA : PXSQLDA;Count : integer);

begin
  FreeSQLDABuffer(aSQLDA);

  if count > -1 then
    begin
    reAllocMem(aSQLDA, XSQLDA_Length(Count));
    { Zero out the memory block to avoid problems with exceptions within the
      constructor of this class. }
    FillChar(aSQLDA^, XSQLDA_Length(Count), 0);

    aSQLDA^.Version := sqlda_version1;
    aSQLDA^.SQLN := Count;
    end
  else
    reAllocMem(aSQLDA,0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TIBConnection.TranslateFldType(SQLType, SQLSubType, SQLLen, SQLScale : integer;
           var TrType : TFieldType; var TrLen : word);
begin
  TrLen := 0;
  if SQLScale < 0 then
    begin
    TrLen := abs(SQLScale);
    if (TrLen <= MaxBCDScale) then //Note: NUMERIC(18,3) or (17,2) must be mapped to ftFmtBCD, but we do not know Precision
=======
procedure TIBConnection.TranslateFldType(SQLType, SQLLen, SQLScale : integer;
           var TrType : TFieldType; var TrLen : word);
begin
=======
procedure TIBConnection.TranslateFldType(SQLType, SQLLen, SQLScale : integer;
           var TrType : TFieldType; var TrLen : word);
begin
>>>>>>> origin/fixes_2_2
  trlen := 0;
  if SQLScale < 0 then
    begin
    if (SQLScale >= -4) and (SQLScale <= -1) then //in [-4..-1] then
      begin
      TrLen := abs(SQLScale);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      TrType := ftBCD
    else
      TrType := ftFMTBcd;
    end
  else case (SQLType and not 1) of
    SQL_VARYING,SQL_TEXT :
      begin
        TrType := ftString;
<<<<<<< HEAD
<<<<<<< HEAD
        TrLen := SQLLen;
      end;
    SQL_TEXT :
      begin
        TrType := ftFixedChar;
        TrLen := SQLLen;
=======
=======
>>>>>>> origin/fixes_2_2
        if SQLLen > dsMaxStringSize then
          TrLen := dsMaxStringSize
        else
          TrLen := SQLLen;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
    SQL_TYPE_DATE :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        TrType := ftDate;
=======
      TrType := ftDate;
>>>>>>> graemeg/cpstrnew
=======
      TrType := ftDate;
>>>>>>> graemeg/cpstrnew
=======
      TrType := ftDate;
>>>>>>> graemeg/cpstrnew
=======
      TrType := ftDate;
>>>>>>> origin/cpstrnew
    SQL_TYPE_TIME :
        TrType := ftTime;
    SQL_TIMESTAMP :
        TrType := ftDateTime;
    SQL_ARRAY :
      begin
        TrType := ftArray;
        TrLen := SQLLen;
      end;
    SQL_BLOB :
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if SQLSubType = isc_blob_text then
          TrType := ftMemo
        else
          TrType := ftBlob;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if SQLSubType = 1 then
           TrType := ftMemo
        else
           TrType := ftBlob;
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
        TrType := ftBlob;
>>>>>>> graemeg/fixes_2_2
=======
        TrType := ftBlob;
>>>>>>> origin/fixes_2_2
        TrLen := SQLLen;
      end;
    SQL_SHORT :
        TrType := ftSmallint;
    SQL_LONG :
        TrType := ftInteger;
    SQL_INT64 :
        TrType := ftLargeInt;
    SQL_DOUBLE :
        TrType := ftFloat;
    SQL_FLOAT :
        TrType := ftFloat;
<<<<<<< HEAD
<<<<<<< HEAD
    SQL_BOOLEAN_INTERBASE, SQL_BOOLEAN_FIREBIRD :
        TrType := ftBoolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    else
        TrType := ftUnknown;
  end;
end;

function TIBConnection.AllocateCursorHandle: TSQLCursor;

var curs : TIBCursor;

begin
  curs := TIBCursor.create;
  curs.sqlda := nil;
  curs.statement := nil;
  curs.FPrepared := False;
  AllocSQLDA(curs.SQLDA,0);
  result := curs;
end;

procedure TIBConnection.DeAllocateCursorHandle(var cursor : TSQLCursor);

begin
  if assigned(cursor) then with cursor as TIBCursor do
    begin
    AllocSQLDA(SQLDA,-1);
    AllocSQLDA(in_SQLDA,-1);
    end;
  FreeAndNil(cursor);
end;

function TIBConnection.AllocateTransactionHandle: TSQLHandle;

begin
  result := TIBTrans.create;
end;

procedure TIBConnection.PrepareStatement(cursor: TSQLCursor;ATransaction : TSQLTransaction;buf : string; AParams : TParams);

var dh    : pointer;
    tr    : pointer;
    x     : Smallint;
    info_request   : string;
    resbuf         : array[0..7] of byte;
    blockSize      : integer;
    IBStatementType: integer;

begin
  with cursor as TIBcursor do
    begin
    dh := GetHandle;
    if isc_dsql_allocate_statement(@Status[0], @dh, @Statement) <> 0 then
      CheckError('PrepareStatement', Status);
    tr := aTransaction.Handle;
    
    if assigned(AParams) and (AParams.count > 0) then
      begin
      buf := AParams.ParseSQL(buf,false,sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions,psInterbase,paramBinding);
      if LogEvent(detActualSQL) then
        Log(detActualSQL,Buf);
      end;
    if isc_dsql_prepare(@Status[0], @tr, @Statement, 0, @Buf[1], Dialect, nil) <> 0 then
      CheckError('PrepareStatement', Status);
    if assigned(AParams) and (AParams.count > 0) then
      begin
      AllocSQLDA(in_SQLDA,Length(ParamBinding));
      if isc_dsql_describe_bind(@Status[0], @Statement, 1, in_SQLDA) <> 0 then
        CheckError('PrepareStatement', Status);
      if in_SQLDA^.SQLD > in_SQLDA^.SQLN then
        DatabaseError(SParameterCountIncorrect,self);
      {$push}
      {$R-}
      for x := 0 to in_SQLDA^.SQLD - 1 do with in_SQLDA^.SQLVar[x] do
        begin
        if ((SQLType and not 1) = SQL_VARYING) then
          SQLData := AllocMem(in_SQLDA^.SQLVar[x].SQLLen+2)
        else
          SQLData := AllocMem(in_SQLDA^.SQLVar[x].SQLLen);
        // Always force the creation of slqind for parameters. It could be
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        // that a database trigger takes care of inserting null values, so
        // it should always be possible to pass null parameters. If that fails,
        // the database server will generate the appropriate error.
=======
        // that a database-trigger takes care of inserting null-values, so
        // it should always be possible to pass null-parameters. If that fails,
        // the database-server will generate the appropiate error.
>>>>>>> graemeg/cpstrnew
=======
        // that a database-trigger takes care of inserting null-values, so
        // it should always be possible to pass null-parameters. If that fails,
        // the database-server will generate the appropiate error.
>>>>>>> graemeg/cpstrnew
=======
        // that a database-trigger takes care of inserting null-values, so
        // it should always be possible to pass null-parameters. If that fails,
        // the database-server will generate the appropiate error.
>>>>>>> graemeg/cpstrnew
=======
        // that a database-trigger takes care of inserting null-values, so
        // it should always be possible to pass null-parameters. If that fails,
        // the database-server will generate the appropiate error.
>>>>>>> origin/cpstrnew
        sqltype := sqltype or 1;
        new(sqlind);
        end;
      {$pop}
      end
    else
      AllocSQLDA(in_SQLDA,0);

    // Get the statement type from firebird/interbase
    info_request := chr(isc_info_sql_stmt_type);
    if isc_dsql_sql_info(@Status[0],@Statement,Length(info_request), @info_request[1],sizeof(resbuf),@resbuf) <> 0 then
      CheckError('PrepareStatement', Status);
    assert(resbuf[0]=isc_info_sql_stmt_type);
    BlockSize:=isc_vax_integer(@resbuf[1],2);
    IBStatementType:=isc_vax_integer(@resbuf[3],blockSize);
    assert(resbuf[3+blockSize]=isc_info_end);
    // If the StatementType is isc_info_sql_stmt_exec_procedure then
    // override the statement type derived by parsing the query.
    // This to recognize statements like 'insert into .. returning' correctly
    case IBStatementType of
      isc_info_sql_stmt_select: FStatementType := stSelect;
      isc_info_sql_stmt_insert: FStatementType := stInsert;
      isc_info_sql_stmt_update: FStatementType := stUpdate;
      isc_info_sql_stmt_delete: FStatementType := stDelete;
      isc_info_sql_stmt_exec_procedure: FStatementType := stExecProcedure;
    end;
    FSelectable := FStatementType in [stSelect,stExecProcedure];

    if FSelectable then
      begin
      if isc_dsql_describe(@Status[0], @Statement, 1, SQLDA) <> 0 then
        CheckError('PrepareSelect', Status);
      if SQLDA^.SQLD > SQLDA^.SQLN then
        begin
        AllocSQLDA(SQLDA,SQLDA^.SQLD);
        if isc_dsql_describe(@Status[0], @Statement, 1, SQLDA) <> 0 then
          CheckError('PrepareSelect', Status);
        end;
      FSelectable := SQLDA^.SQLD > 0;
      {$push}
      {$R-}
      for x := 0 to SQLDA^.SQLD - 1 do with SQLDA^.SQLVar[x] do
        begin
        if ((SQLType and not 1) = SQL_VARYING) then
          SQLData := AllocMem(SQLDA^.SQLVar[x].SQLLen+2)
        else
          SQLData := AllocMem(SQLDA^.SQLVar[x].SQLLen);
        if (SQLType and 1) = 1 then New(SQLInd);
        end;
      {$pop}
      end;
    FPrepared := True;
    end;
end;

procedure TIBConnection.UnPrepareStatement(cursor : TSQLCursor);

begin
  with cursor as TIBcursor do
    if assigned(Statement) Then
      begin
        if isc_dsql_free_statement(@Status[0], @Statement, DSQL_Drop) <> 0 then
          CheckError('FreeStatement', Status);
        Statement := nil;
        FPrepared := False;
      end;
end;

procedure TIBConnection.FreeSQLDABuffer(var aSQLDA : PXSQLDA);

var x : Smallint;

begin
{$push}
{$R-}
  if assigned(aSQLDA) then
    for x := 0 to aSQLDA^.SQLN - 1 do
      begin
      reAllocMem(aSQLDA^.SQLVar[x].SQLData,0);
      if assigned(aSQLDA^.SQLVar[x].sqlind) then
        begin
        Dispose(aSQLDA^.SQLVar[x].sqlind);
        aSQLDA^.SQLVar[x].sqlind := nil;
        end
      end;
{$pop}
end;

function TIBConnection.IsDialectStored: boolean;
begin
  result := (FDialect<>INVALID_DATA);
end;

procedure TIBConnection.DoConnect;
const NoQuotes: TQuoteChars = (' ',' ');
begin
  inherited DoConnect;
  GetDatabaseInfo; //Get db dialect, db metadata
  if Dialect < 3 then
    FieldNameQuoteChars := NoQuotes
  else
    FieldNameQuoteChars := DoubleQuotes;
end;

function TIBConnection.IsDialectStored: boolean;
begin
  result := (FDialect<>-1);
end;

procedure TIBConnection.DoConnect;
begin
  inherited DoConnect;
  FDbDialect := GetDBDialect;
end;

function TIBConnection.IsDialectStored: boolean;
begin
  result := (FDialect<>-1);
end;

procedure TIBConnection.DoConnect;
begin
  inherited DoConnect;
  FDbDialect := GetDBDialect;
end;

procedure TIBConnection.FreeFldBuffers(cursor : TSQLCursor);

begin
  with cursor as TIBCursor do
    begin
    FreeSQLDABuffer(SQLDA);
    FreeSQLDABuffer(in_SQLDA);
    SetLength(FieldBinding,0);
    end;
end;

procedure TIBConnection.Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams);
var tr : pointer;
    out_SQLDA : PXSQLDA;
begin
  tr := aTransaction.Handle;
  if Assigned(APArams) and (AParams.count > 0) then SetParameters(cursor, atransaction, AParams);
  if LogEvent(detParamValue) then
    LogParams(AParams);
  with cursor as TIBCursor do
  begin
    if FStatementType = stExecProcedure then
      out_SQLDA := SQLDA
    else
      out_SQLDA := nil;
    if isc_dsql_execute2(@Status[0], @tr, @Statement, 1, in_SQLDA, out_SQLDA) <> 0 then
      CheckError('Execute', Status);
  end;
end;


procedure TIBConnection.AddFieldDefs(cursor: TSQLCursor;FieldDefs : TfieldDefs);
var
  x         : integer;
  TransLen  : word;
  TransType : TFieldType;
  FD        : TFieldDef;

begin
  {$push}
  {$R-}
  with cursor as TIBCursor do
    begin
    setlength(FieldBinding,SQLDA^.SQLD);
    for x := 0 to SQLDA^.SQLD - 1 do
      begin
<<<<<<< HEAD
      TranslateFldType(SQLDA^.SQLVar[x].SQLType, SQLDA^.SQLVar[x].sqlsubtype, SQLDA^.SQLVar[x].SQLLen, SQLDA^.SQLVar[x].SQLScale,
        TransType, TransLen);

      FD := FieldDefs.Add(FieldDefs.MakeNameUnique(SQLDA^.SQLVar[x].AliasName), TransType,
         TransLen, (SQLDA^.SQLVar[x].sqltype and 1)=0, (x + 1));
      if TransType in [ftBCD, ftFmtBCD] then
        case (SQLDA^.SQLVar[x].sqltype and not 1) of
          SQL_SHORT : FD.precision := 4;
          SQL_LONG  : FD.precision := 9;
          SQL_DOUBLE,
=======
      TranslateFldType(SQLDA^.SQLVar[x].SQLType, SQLDA^.SQLVar[x].SQLLen, SQLDA^.SQLVar[x].SQLScale,
        TransType, TransLen);

      FD := TFieldDef.Create(FieldDefs, FieldDefs.MakeNameUnique(SQLDA^.SQLVar[x].AliasName), TransType,
         TransLen, (SQLDA^.SQLVar[x].sqltype and 1)=0, (x + 1));
      if TransType = ftBCD then
        case (SQLDA^.SQLVar[x].sqltype and not 1) of
          SQL_SHORT : FD.precision := 4;
          SQL_LONG  : FD.precision := 9;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          SQL_INT64 : FD.precision := 18;
          else FD.precision := SQLDA^.SQLVar[x].SQLLen;
        end;
//      FD.DisplayName := SQLDA^.SQLVar[x].AliasName;
      FieldBinding[FD.FieldNo-1] := x;
      end;
    end;
  {$pop}
end;

function TIBConnection.GetHandle: pointer;
begin
  Result := FSQLDatabaseHandle;
end;

function TIBConnection.Fetch(cursor : TSQLCursor) : boolean;
var
  retcode : integer;
begin
  with cursor as TIBCursor do
  begin
    if FStatementType = stExecProcedure then
      //do not fetch from a non-select statement, i.e. statement which has no cursor
      //on Firebird 2.5+ it leads to error 'Invalid cursor reference'
      if SQLDA^.SQLD = 0 then
        retcode := 100 //no more rows to retrieve
      else
      begin
        retcode := 0;
        SQLDA^.SQLD := 0; //hack: mark after first fetch
      end
    else
      retcode := isc_dsql_fetch(@Status[0], @Statement, 1, SQLDA);
    if (retcode <> 0) and (retcode <> 100) then
      CheckError('Fetch', Status);
  end;
  Result := (retcode = 0);
end;

function IntPower10(e: integer): double;
const PreComputedPower10: array[0..9] of integer = (1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000);
var n: integer;
begin
  n := abs(e); //exponent can't be greater than 18
  if n <= 9 then
    Result := PreComputedPower10[n]
  else
    Result := PreComputedPower10[9] * PreComputedPower10[n-9];
  if e < 0 then
    Result := 1 / Result;
end;

procedure TIBConnection.SetParameters(cursor : TSQLCursor; aTransation : TSQLTransaction; AParams : TParams);

var ParNr,SQLVarNr : integer;
    s               : string;
    i               : integer;
    si              : smallint;
    li              : LargeInt;
    currbuff        : pchar;
    w               : word;

    TransactionHandle : pointer;
    blobId            : ISC_QUAD;
    blobHandle        : Isc_blob_Handle;
    BlobSize,
    BlobBytesWritten  : longint;
    
  procedure SetBlobParam;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  
  begin
{$R-}
    with cursor as TIBCursor do
      begin
      TransactionHandle := transaction.Handle;
      blobhandle := nil;
      if isc_create_blob(@FStatus[0], @FSQLDatabaseHandle, @TransactionHandle, @blobHandle, @blobId) <> 0 then
       CheckError('TIBConnection.CreateBlobStream', FStatus);

      s := AParams[ParNr].AsString;
      BlobSize := length(s);

      BlobBytesWritten := 0;
      i := 0;

      while BlobBytesWritten < (BlobSize-BlobSegmentSize) do
        begin
        isc_put_segment(@FStatus[0], @blobHandle, BlobSegmentSize, @s[(i*BlobSegmentSize)+1]);
        inc(BlobBytesWritten,BlobSegmentSize);
        inc(i);
        end;
      if BlobBytesWritten <> BlobSize then
        isc_put_segment(@FStatus[0], @blobHandle, BlobSize-BlobBytesWritten, @s[(i*BlobSegmentSize)+1]);

      if isc_close_blob(@FStatus[0], @blobHandle) <> 0 then
        CheckError('TIBConnection.CreateBlobStream isc_close_blob', FStatus);
      Move(blobId, in_sqlda^.SQLvar[SQLVarNr].SQLData^, in_SQLDA^.SQLVar[SQLVarNr].SQLLen);
      end;
{$R+}
  end;

var
  // This should be a pointer, because the ORIGINAL variables must
  // be modified.
  VSQLVar: ^XSQLVAR;
>>>>>>> origin/fixes_2_2

  begin
    {$push}
    {$R-}
    with cursor as TIBCursor do
      begin
      TransactionHandle := aTransation.Handle;
      blobhandle := FB_API_NULLHANDLE;
      if isc_create_blob(@FStatus[0], @FSQLDatabaseHandle, @TransactionHandle, @blobHandle, @blobId) <> 0 then
       CheckError('TIBConnection.CreateBlobStream', FStatus);
=======
  
  begin
{$R-}
    with cursor as TIBCursor do
      begin
      TransactionHandle := transaction.Handle;
      blobhandle := nil;
      if isc_create_blob(@FStatus[0], @FSQLDatabaseHandle, @TransactionHandle, @blobHandle, @blobId) <> 0 then
       CheckError('TIBConnection.CreateBlobStream', FStatus);

      s := AParams[ParNr].AsString;
      BlobSize := length(s);

      BlobBytesWritten := 0;
      i := 0;

      while BlobBytesWritten < (BlobSize-BlobSegmentSize) do
        begin
        isc_put_segment(@FStatus[0], @blobHandle, BlobSegmentSize, @s[(i*BlobSegmentSize)+1]);
        inc(BlobBytesWritten,BlobSegmentSize);
        inc(i);
        end;
      if BlobBytesWritten <> BlobSize then
        isc_put_segment(@FStatus[0], @blobHandle, BlobSize-BlobBytesWritten, @s[(i*BlobSegmentSize)+1]);

      if isc_close_blob(@FStatus[0], @blobHandle) <> 0 then
        CheckError('TIBConnection.CreateBlobStream isc_close_blob', FStatus);
      Move(blobId, in_sqlda^.SQLvar[SQLVarNr].SQLData^, in_SQLDA^.SQLVar[SQLVarNr].SQLLen);
      end;
{$R+}
  end;

var
  // This should be a pointer, because the ORIGINAL variables must
  // be modified.
  VSQLVar: ^XSQLVAR;
>>>>>>> graemeg/fixes_2_2

      s := AParams[ParNr].AsString;
      BlobSize := length(s);

      BlobBytesWritten := 0;
      i := 0;

      // Write in segments of MAXBLOBSEGMENTSIZE, as that is the fastest.
      // We ignore BlobSegmentSize property.
      while BlobBytesWritten < (BlobSize-MAXBLOBSEGMENTSIZE) do
        begin
        isc_put_segment(@FStatus[0], @blobHandle, MAXBLOBSEGMENTSIZE, @s[(i*MAXBLOBSEGMENTSIZE)+1]);
        inc(BlobBytesWritten,MAXBLOBSEGMENTSIZE);
        inc(i);
        end;
      if BlobBytesWritten <> BlobSize then
        isc_put_segment(@FStatus[0], @blobHandle, BlobSize-BlobBytesWritten, @s[(i*MAXBLOBSEGMENTSIZE)+1]);

      if isc_close_blob(@FStatus[0], @blobHandle) <> 0 then
        CheckError('TIBConnection.CreateBlobStream isc_close_blob', FStatus);
      Move(blobId, in_sqlda^.SQLvar[SQLVarNr].SQLData^, in_SQLDA^.SQLVar[SQLVarNr].SQLLen);
      end;
      {$pop}
  end;

Const
  DateF = 'yyyy-mm-dd';
  TimeF = 'hh:nn:ss';
  DateTimeF = DateF+' '+TimeF;

var
  // This should be a pointer, because the ORIGINAL variables must
  // be modified.
  VSQLVar: ^XSQLVAR;
  P: TParam;
  ft : TFieldType;
begin
  {$push}
  {$R-}
  with cursor as TIBCursor do for SQLVarNr := 0 to High(ParamBinding){AParams.count-1} do
    begin
    ParNr := ParamBinding[SQLVarNr];
    VSQLVar := @in_sqlda^.SQLvar[SQLVarNr];
    if AParams[ParNr].IsNull then
<<<<<<< HEAD
      VSQLVar^.SQLInd^ := -1
    else
      begin
      VSQLVar^.SQLInd^ := 0;

      case (VSQLVar^.sqltype and not 1) of
        SQL_SHORT, SQL_BOOLEAN_INTERBASE :
          begin
            if VSQLVar^.sqlscale = 0 then
              si := AParams[ParNr].AsSmallint
            else
              si := Round(AParams[ParNr].AsCurrency * IntPower10(-VSQLVar^.sqlscale));
            i := si;
            Move(i, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_LONG :
=======
      begin
      If Assigned(VSQLVar^.SQLInd) then
        VSQLVar^.SQLInd^ := -1;
      end
    else
      begin
      if assigned(VSQLVar^.SQLInd) then VSQLVar^.SQLInd^ := 0;

      case (VSQLVar^.sqltype and not 1) of
        SQL_LONG :
          begin
            if VSQLVar^.sqlscale = 0 then
              i := AParams[ParNr].AsInteger
            else
              i := Round(AParams[ParNr].AsCurrency * IntPower(10, -VSQLVar^.sqlscale));
            Move(i, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_SHORT :
<<<<<<< HEAD
=======
          begin
            if VSQLVar^.sqlscale = 0 then
              si := AParams[ParNr].AsSmallint
            else
              si := Round(AParams[ParNr].AsCurrency * IntPower(10, -VSQLVar^.sqlscale));
            i := si;
            Move(i, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_BLOB :
          SetBlobParam;
        SQL_VARYING, SQL_TEXT :
>>>>>>> origin/fixes_2_2
          begin
            if VSQLVar^.sqlscale = 0 then
              si := AParams[ParNr].AsSmallint
            else
              si := Round(AParams[ParNr].AsCurrency * IntPower(10, -VSQLVar^.sqlscale));
            i := si;
            Move(i, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_BLOB :
          SetBlobParam;
        SQL_VARYING, SQL_TEXT :
>>>>>>> graemeg/fixes_2_2
          begin
            if VSQLVar^.sqlscale = 0 then
              i := AParams[ParNr].AsInteger
            else
              i := Round(AParams[ParNr].AsFloat * IntPower10(-VSQLVar^.sqlscale)); //*any number of digits
            Move(i, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_INT64:
          begin
            if VSQLVar^.sqlscale = 0 then
              li := AParams[ParNr].AsLargeInt
            else if AParams[ParNr].DataType = ftFMTBcd then
              li := AParams[ParNr].AsFMTBCD * IntPower10(-VSQLVar^.sqlscale)
            else
              li := Round(AParams[ParNr].AsCurrency * IntPower10(-VSQLVar^.sqlscale));
            Move(li, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_DOUBLE, SQL_FLOAT:
          SetFloat(VSQLVar^.SQLData, AParams[ParNr].AsFloat, VSQLVar^.SQLLen);
        SQL_BLOB :
          SetBlobParam;
        SQL_VARYING, SQL_TEXT :
          begin
          P:=AParams[ParNr];
          ft:=P.DataType;
          if Not (ft in [ftDate,ftTime,ftDateTime,ftTimeStamp]) then
            S:=P.AsString
          else
            begin
            Case ft of
              ftDate : S:=DateF;
              ftTime : S:=TimeF;
              ftDateTime,
              ftTimeStamp : S:=DateTimeF;
            end;
            S:=FormatDateTime(S,P.AsDateTime);
            end;
          w := length(s); // a word is enough, since the max-length of a string in interbase is 32k
          if ((VSQLVar^.SQLType and not 1) = SQL_VARYING) then
            begin
            VSQLVar^.SQLLen := w;
            ReAllocMem(VSQLVar^.SQLData, VSQLVar^.SQLLen+2);
            CurrBuff := VSQLVar^.SQLData;
            move(w,CurrBuff^,sizeof(w));
            inc(CurrBuff,2);
            end
          else
            begin
            // The buffer-length is always VSQLVar^.sqllen, nothing more, nothing less
            // so fill the complete buffer with valid data. Adding #0 will lead
            // to problems, because the #0 will be seen as a part of the (binary) string
            CurrBuff := VSQLVar^.SQLData;
            w := VSQLVar^.sqllen;
            s := PadRight(s,w);
            end;
          Move(s[1], CurrBuff^, w);
<<<<<<< HEAD
          end;
        SQL_TYPE_DATE, SQL_TYPE_TIME, SQL_TIMESTAMP :
          SetDateTime(VSQLVar^.SQLData, AParams[ParNr].AsDateTime, VSQLVar^.SQLType);
<<<<<<< HEAD
        SQL_BOOLEAN_FIREBIRD:
          PByte(VSQLVar^.SQLData)^ := Byte(AParams[ParNr].AsBoolean);
=======
          end;
        SQL_TYPE_DATE, SQL_TYPE_TIME, SQL_TIMESTAMP :
          SetDateTime(VSQLVar^.SQLData, AParams[ParNr].AsDateTime, VSQLVar^.SQLType);
=======
>>>>>>> origin/fixes_2_2
        SQL_INT64:
          begin
            if VSQLVar^.sqlscale = 0 then
              li := AParams[ParNr].AsLargeInt
            else
              li := Round(AParams[ParNr].AsCurrency * IntPower(10, -VSQLVar^.sqlscale));
            Move(li, VSQLVar^.SQLData^, VSQLVar^.SQLLen);
          end;
        SQL_DOUBLE, SQL_FLOAT:
          SetFloat(VSQLVar^.SQLData, AParams[ParNr].AsFloat, VSQLVar^.SQLLen);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      else
        DatabaseErrorFmt(SUnsupportedParameter,[Fieldtypenames[AParams[ParNr].DataType]],self);
      end {case}
      end;
    end;
{$pop}
end;

function TIBConnection.LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;

var
  VSQLVar    : PXSQLVAR;
  VarcharLen : word;
  CurrBuff     : pchar;
  c            : currency;
<<<<<<< HEAD
<<<<<<< HEAD
  AFmtBcd      : tBCD;

  function BcdDivPower10(Dividend: largeint; e: integer): TBCD;
  var d: double;
  begin
    d := Dividend / IntPower10(e);
    Result := StrToBCD( FloatToStr(d) );
  end;
=======
  smalli       : smallint;
  longi        : longint;
  largei       : largeint;
>>>>>>> graemeg/fixes_2_2
=======
  smalli       : smallint;
  longi        : longint;
  largei       : largeint;
>>>>>>> origin/fixes_2_2

begin
  CreateBlob := False;
  with cursor as TIBCursor do
    begin
    {$push}
    {$R-}
    VSQLVar := @SQLDA^.SQLVar[ FieldBinding[FieldDef.FieldNo-1] ];

    // Joost, 5 jan 2006: I disabled the following, since it's useful for
    // debugging, but it also slows things down. In principle things can only go
    // wrong when FieldDefs is changed while the dataset is opened. A user just
    // shoudn't do that. ;) (The same is done in PQConnection)

    // if VSQLVar^.AliasName <> FieldDef.Name then
    // DatabaseErrorFmt(SFieldNotFound,[FieldDef.Name],self);
    if assigned(VSQLVar^.SQLInd) and (VSQLVar^.SQLInd^ = -1) then
      result := false
    else
      begin

      with VSQLVar^ do
        if ((SQLType and not 1) = SQL_VARYING) then
          begin
          Move(SQLData^, VarcharLen, 2);
          if VarcharLen > dsMaxStringSize then
            VarcharLen:=dsMaxStringSize;
          CurrBuff := SQLData + 2;
          end
        else
          begin
          CurrBuff := SQLData;
          VarCharLen := FieldDef.Size;
          end;

      Result := true;
      case FieldDef.DataType of
        ftBCD :
          begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            case VSQLVar^.SQLLen of
              2 : c := PSmallint(CurrBuff)^ / IntPower10(-VSQLVar^.SQLScale);
              4 : c := PLongint(CurrBuff)^  / IntPower10(-VSQLVar^.SQLScale);
              8 : if Dialect < 3 then
                    c := PDouble(CurrBuff)^
                  else
                    c := PLargeint(CurrBuff)^ / IntPower10(-VSQLVar^.SQLScale);
=======
            case SQLDA^.SQLVar[x].SQLLen of
              2 : begin
                  Move(CurrBuff^, smalli, 2);
                  c := smalli*intpower(10,SQLDA^.SQLVar[x].SQLScale);
=======
=======
>>>>>>> origin/fixes_2_2
            case SQLDA^.SQLVar[x].SQLLen of
              2 : begin
                  Move(CurrBuff^, smalli, 2);
                  c := longi*intpower(10,SQLDA^.SQLVar[x].SQLScale);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  end;
              4 : begin
                  Move(CurrBuff^, longi, 4);
                  c := longi*intpower(10,SQLDA^.SQLVar[x].SQLScale);
                  end;
              8 : begin
                  Move(CurrBuff^, largei, 8);
                  c := largei*intpower(10,SQLDA^.SQLVar[x].SQLScale);
                  end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              else
                Result := False; // Just to be sure, in principle this will never happen
            end; {case}
            Move(c, buffer^ , sizeof(c));
          end;
        ftFMTBcd :
          begin
            case VSQLVar^.SQLLen of
              2 : AFmtBcd := BcdDivPower10(PSmallint(CurrBuff)^, -VSQLVar^.SQLScale);
              4 : AFmtBcd := BcdDivPower10(PLongint(CurrBuff)^,  -VSQLVar^.SQLScale);
              8 : if Dialect < 3 then
                    AFmtBcd := PDouble(CurrBuff)^
                  else
                    AFmtBcd := BcdDivPower10(PLargeint(CurrBuff)^, -VSQLVar^.SQLScale);
              else
                Result := False; // Just to be sure, in principle this will never happen
            end; {case}
            Move(AFmtBcd, buffer^ , sizeof(AFmtBcd));
          end;
        ftInteger :
          begin
            FillByte(buffer^,sizeof(Longint),0);
            Move(CurrBuff^, Buffer^, VSQLVar^.SQLLen);
          end;
        ftLargeint :
          begin
            FillByte(buffer^,sizeof(LargeInt),0);
            Move(CurrBuff^, Buffer^, VSQLVar^.SQLLen);
          end;
        ftSmallint :
          begin
            FillByte(buffer^,sizeof(Smallint),0);
            Move(CurrBuff^, Buffer^, VSQLVar^.SQLLen);
          end;
        ftSmallint :
          begin
            FillByte(buffer^,sizeof(Smallint),0);
            Move(CurrBuff^, Buffer^, SQLDA^.SQLVar[x].SQLLen);
          end;
        ftSmallint :
          begin
            FillByte(buffer^,sizeof(Smallint),0);
            Move(CurrBuff^, Buffer^, SQLDA^.SQLVar[x].SQLLen);
          end;
        ftDate, ftTime, ftDateTime:
          GetDateTime(CurrBuff, Buffer, VSQLVar^.SQLType);
        ftString, ftFixedChar  :
          begin
            Move(CurrBuff^, Buffer^, VarCharLen);
            PChar(Buffer + VarCharLen)^ := #0;
          end;
        ftFloat   :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          GetFloat(CurrBuff, Buffer, VSQLVar^.SQLLen);
=======
          GetFloat(CurrBuff, Buffer, SQLDA^.SQLVar[x].SQLLen);
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
        ftBlob,
        ftMemo :
          begin  // load the BlobIb in field's buffer
=======
=======
>>>>>>> origin/fixes_2_2
          GetFloat(CurrBuff, Buffer, SQLDA^.SQLVar[x].SQLLen);
        ftBlob : begin  // load the BlobIb in field's buffer
>>>>>>> graemeg/fixes_2_2
            FillByte(buffer^,sizeof(TBufBlobField),0);
<<<<<<< HEAD
            Move(CurrBuff^, Buffer^, VSQLVar^.SQLLen);
          end;
        ftBoolean :
          begin
            case VSQLVar^.SQLLen of
              1: PWordBool(Buffer)^ := PByte(CurrBuff)^ <> 0; // Firebird
              2: PWordBool(Buffer)^ := PSmallint(CurrBuff)^ <> 0; // Interbase
            end;
          end
        else
          begin
            result := false;
            databaseerrorfmt(SUnsupportedFieldType, [Fieldtypenames[FieldDef.DataType], Self]);
          end
      end;  { case }
      end; { if/else }
      {$pop}
    end; { with cursor }
end;

{$DEFINE SUPPORT_MSECS}
{$IFDEF SUPPORT_MSECS}
  const
    IBDateOffset = 15018; //an offset from 17 Nov 1858.
    IBTimeFractionsPerDay  = SecsPerDay * ISC_TIME_SECONDS_PRECISION; //Number of Firebird time fractions per day
{$ELSE}
  {$PACKRECORDS C}
  type
    TTm = record
      tm_sec  : longint;
      tm_min  : longint;
      tm_hour : longint;
      tm_mday : longint;
      tm_mon  : longint;
      tm_year : longint;
      tm_wday : longint;
      tm_yday : longint;
      tm_isdst: longint;
      __tm_gmtoff : PtrInt;    // Seconds east of UTC
      __tm_zone   : PAnsiChar; // Timezone abbreviation
    end;
  {$PACKRECORDS DEFAULT}
{$ENDIF}
=======
            Move(CurrBuff^, Buffer^, SQLDA^.SQLVar[x].SQLLen);
          end;

        else
          begin
            result := false;
            databaseerrorfmt(SUnsupportedFieldType, [Fieldtypenames[FieldDef.DataType], Self]);
          end
      end;  { case }
      end; { if/else }
{$R+}
    end; { with cursor }
end;
>>>>>>> graemeg/cpstrnew

procedure TIBConnection.GetDateTime(CurrBuff, Buffer : pointer; AType : integer);
var
  {$IFNDEF SUPPORT_MSECS}
  CTime : TTm;          // C struct time
  STime : TSystemTime;  // System time
  {$ENDIF}
  PTime : TDateTime;    // Pascal time
begin
  case (AType and not 1) of
    SQL_TYPE_DATE :
      {$IFNDEF SUPPORT_MSECS}
      isc_decode_sql_date(PISC_DATE(CurrBuff), @CTime);
      {$ELSE}
      PTime := PISC_DATE(CurrBuff)^ - IBDateOffset;
      {$ENDIF}
    SQL_TYPE_TIME :
      {$IFNDEF SUPPORT_MSECS}
      isc_decode_sql_time(PISC_TIME(CurrBuff), @CTime);
      {$ELSE}
      PTime :=  PISC_TIME(CurrBuff)^ / IBTimeFractionsPerDay;
      {$ENDIF}
    SQL_TIMESTAMP :
      begin
      {$IFNDEF SUPPORT_MSECS}
      isc_decode_timestamp(PISC_TIMESTAMP(CurrBuff), @CTime);
      {$ELSE}
      PTime := ComposeDateTime(
                  PISC_TIMESTAMP(CurrBuff)^.timestamp_date - IBDateOffset,
                  PISC_TIMESTAMP(CurrBuff)^.timestamp_time / IBTimeFractionsPerDay
               );
      {$ENDIF}
      end
  else
    Raise EIBDatabaseError.CreateFmt('Invalid parameter type for date Decode : %d',[(AType and not 1)]);
  end;

  {$IFNDEF SUPPORT_MSECS}
  STime.Year        := CTime.tm_year + 1900;
  STime.Month       := CTime.tm_mon + 1;
  STime.Day         := CTime.tm_mday;
  STime.Hour        := CTime.tm_hour;
  STime.Minute      := CTime.tm_min;
  STime.Second      := CTime.tm_sec;
  STime.Millisecond := 0;

  PTime := SystemTimeToDateTime(STime);
  {$ENDIF}
  Move(PTime, Buffer^, SizeOf(PTime));
end;

procedure TIBConnection.SetDateTime(CurrBuff: pointer; PTime : TDateTime; AType : integer);
{$IFNDEF SUPPORT_MSECS}
var
  CTime : TTm;          // C struct time
  STime : TSystemTime;  // System time
{$ENDIF}
begin
  {$IFNDEF SUPPORT_MSECS}
  DateTimeToSystemTime(PTime,STime);
  
  CTime.tm_year := STime.Year - 1900;
  CTime.tm_mon  := STime.Month -1;
  CTime.tm_mday := STime.Day;
  CTime.tm_hour := STime.Hour;
  CTime.tm_min  := STime.Minute;
  CTime.tm_sec  := STime.Second;
  {$ENDIF}
  case (AType and not 1) of
    SQL_TYPE_DATE :
      {$IFNDEF SUPPORT_MSECS}
      isc_encode_sql_date(@CTime, PISC_DATE(CurrBuff));
      {$ELSE}
      PISC_DATE(CurrBuff)^ := Trunc(PTime) + IBDateOffset;
      {$ENDIF}
    SQL_TYPE_TIME :
      {$IFNDEF SUPPORT_MSECS}
      isc_encode_sql_time(@CTime, PISC_TIME(CurrBuff));
      {$ELSE}
      PISC_TIME(CurrBuff)^ := Round(abs(Frac(PTime)) * IBTimeFractionsPerDay);
      {$ENDIF}
    SQL_TIMESTAMP :
      begin
      {$IFNDEF SUPPORT_MSECS}
      isc_encode_timestamp(@CTime, PISC_TIMESTAMP(CurrBuff));
      {$ELSE}
      PISC_TIMESTAMP(CurrBuff)^.timestamp_date := Trunc(PTime) + IBDateOffset;
      PISC_TIMESTAMP(CurrBuff)^.timestamp_time := Round(abs(Frac(PTime)) * IBTimeFractionsPerDay);
      {$ENDIF}
      end
  else
    Raise EIBDatabaseError.CreateFmt('Invalid parameter type for date encode : %d',[(AType and not 1)]);
  end;
end;

function TIBConnection.GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string;

var s : string;

begin
  case SchemaType of
    stTables     : s := 'select '+
                          'rdb$relation_id          as recno, '+
                          '''' + DatabaseName + ''' as catalog_name, '+
                          '''''                     as schema_name, '+
                          'rdb$relation_name        as table_name, '+
                          '0                        as table_type '+
                        'from '+
                          'rdb$relations '+
                        'where '+
                          '(rdb$system_flag = 0 or rdb$system_flag is null) ' + // and rdb$view_blr is null
                        'order by rdb$relation_name';

    stSysTables  : s := 'select '+
                          'rdb$relation_id          as recno, '+
                          '''' + DatabaseName + ''' as catalog_name, '+
                          '''''                     as schema_name, '+
                          'rdb$relation_name        as table_name, '+
                          '0                        as table_type '+
                        'from '+
                          'rdb$relations '+
                        'where '+
                          '(rdb$system_flag > 0) ' + // and rdb$view_blr is null
                        'order by rdb$relation_name';

    stProcedures : s := 'select '+
                           'rdb$procedure_id        as recno, '+
                          '''' + DatabaseName + ''' as catalog_name, '+
                          '''''                     as schema_name, '+
                          'rdb$procedure_name       as procedure_name, '+
                          '0                        as procedure_type, '+
                          'rdb$procedure_inputs     as in_params, '+
                          'rdb$procedure_outputs    as out_params '+
                        'from '+
                          'rdb$procedures '+
                        'WHERE '+
                          '(rdb$system_flag = 0 or rdb$system_flag is null)';

    stColumns    : s := 'SELECT '+
                          'rdb$field_id             as recno, '+
                          '''' + DatabaseName + ''' as catalog_name, '+
                          '''''                     as schema_name, '+
                          'rdb$relation_name        as table_name, '+
                          'r.rdb$field_name         as column_name, '+
                          'rdb$field_position+1     as column_position, '+
                          '0                        as column_type, '+
                          'rdb$field_type           as column_datatype, '+
                          'rdb$type_name            as column_typename, '+
                          'rdb$field_sub_type       as column_subtype, '+
                          'rdb$field_precision      as column_precision, '+
                          '-rdb$field_scale         as column_scale, '+
                          'rdb$field_length         as column_length, '+
                          'case r.rdb$null_flag when 1 then 0 else 1 end as column_nullable '+
                        'FROM '+
                          'rdb$relation_fields r '+
                            'JOIN rdb$fields f ON r.rdb$field_source=f.rdb$field_name '+
                            'JOIN rdb$types t ON f.rdb$field_type=t.rdb$type AND t.rdb$field_name=''RDB$FIELD_TYPE'' '+
                        'WHERE '+
                          '(r.rdb$system_flag = 0 or r.rdb$system_flag is null) and (rdb$relation_name = ''' + Uppercase(SchemaObjectName) + ''') ' +
                        'ORDER BY '+
                          'r.rdb$field_name';
    stSequences  : s := 'SELECT ' +
                          'rdb$generator_id         as recno,' +
                          '''' + DatabaseName + ''' as sequence_catalog,' +
                          '''''                     as sequence_schema,' +
                          'rdb$generator_name       as sequence_name ' +
                        'FROM ' +
                          'rdb$generators ' +
                        'WHERE ' +
                          'rdb$system_flag = 0 or rdb$system_flag is null ' +
                        'ORDER BY ' +
                          'rdb$generator_name';
  else
    DatabaseError(SMetadataUnavailable)
  end; {case}
  result := s;
end;

function TIBConnection.GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
begin
  Result := Format('SELECT gen_id(%s, %d) FROM RDB$DATABASE', [SequenceName, IncrementBy]);
end;

procedure TIBConnection.UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string);

var qry : TSQLQuery;

begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet);

  if (length(TableName)>2) and (TableName[1]='"') and (TableName[length(TableName)]='"') then
    TableName := AnsiDequotedStr(TableName, '"')
  else
    TableName := UpperCase(TableName);

  qry := tsqlquery.Create(nil);
  qry.transaction := Transaction;
  qry.database := Self;
  with qry do
    begin
    ReadOnly := True;
    sql.clear;
    sql.add('select '+
              'ind.rdb$index_name, '+
              'ind.rdb$relation_name, '+
              'ind.rdb$unique_flag, '+
              'ind_seg.rdb$field_name, '+
              'rel_con.rdb$constraint_type, '+
              'ind.rdb$index_type '+
            'from '+
              'rdb$index_segments ind_seg, '+
              'rdb$indices ind '+
             'left outer join '+
              'rdb$relation_constraints rel_con '+
             'on '+
              'rel_con.rdb$index_name = ind.rdb$index_name '+
            'where '+
              '(ind_seg.rdb$index_name = ind.rdb$index_name) and '+
              '(ind.rdb$relation_name=' + QuotedStr(TableName) + ') '+
            'order by '+
              'ind.rdb$index_name;');
    open;
    end;
  while not qry.eof do with IndexDefs.AddIndexDef do
    begin
    Name := trim(qry.fields[0].asstring);
    Fields := trim(qry.Fields[3].asstring);
    If qry.fields[4].asstring = 'PRIMARY KEY' then options := options + [ixPrimary];
    If qry.fields[2].asinteger = 1 then options := options + [ixUnique];
    If qry.fields[5].asInteger = 1 then options:=options+[ixDescending];
    qry.next;
    while (name = trim(qry.fields[0].asstring)) and (not qry.eof) do
      begin
      Fields := Fields + ';' + trim(qry.Fields[3].asstring);
      qry.next;
      end;
    end;
  qry.close;
  qry.free;
end;

procedure TIBConnection.SetFloat(CurrBuff: pointer; Dbl: Double; Size: integer);

var
  Ext : extended;
  Sin : single;
begin
  case Size of
    4 :
      begin
        Sin := Dbl;
        Move(Sin, CurrBuff^, 4);
      end;
    8 :
      begin
        Move(Dbl, CurrBuff^, 8);
      end;
    10:
      begin
        Ext := Dbl;
        Move(Ext, CurrBuff^, 10);
      end;
  else
    Raise EIBDatabaseError.CreateFmt('Invalid float size for float encode : %d',[Size]);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TIBConnection.GetFloat(CurrBuff, Buffer: pointer; Size: Byte);
=======
procedure TIBConnection.GetFloat(CurrBuff, Buffer : pointer; Size : byte);
>>>>>>> graemeg/fixes_2_2
=======
procedure TIBConnection.GetFloat(CurrBuff, Buffer : pointer; Size : byte);
>>>>>>> origin/fixes_2_2
var
  Ext : extended;
  Dbl : double;
  Sin : single;
begin
  case Size of
    4 :
      begin
        Move(CurrBuff^, Sin, 4);
        Dbl := Sin;
      end;
    8 :
      begin
        Move(CurrBuff^, Dbl, 8);
      end;
    10:
      begin
        Move(CurrBuff^, Ext, 10);
        Dbl := double(Ext);
      end;
  else
    Raise EIBDatabaseError.CreateFmt('Invalid float size for float Decode : %d',[Size]);
  end;
  Move(Dbl, Buffer^, 8);
end;

procedure TIBConnection.LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction);
const
  isc_segstr_eof = 335544367; // It's not defined in ibase60 but in ibase40. Would it be better to define in ibase60?

var
  blobHandle : Isc_blob_Handle;
  blobSegment : pointer;
  blobSegLen : word;
  TransactionHandle : pointer;
  blobId : PISC_QUAD;
  ptr : Pointer;
begin
  blobId := PISC_QUAD(@(ABlobBuf^.ConnBlobBuffer));

  TransactionHandle := Atransaction.Handle;
  blobHandle := FB_API_NULLHANDLE;

  if isc_open_blob(@FStatus[0], @FSQLDatabaseHandle, @TransactionHandle, @blobHandle, blobId) <> 0 then
    CheckError('TIBConnection.CreateBlobStream', FStatus);

  //For performance, read as much as we can, regardless of any segment size set in database.
  blobSegment := AllocMem(MAXBLOBSEGMENTSIZE);

  with ABlobBuf^.BlobBuffer^ do
    begin
    Size := 0;
    while (isc_get_segment(@FStatus[0], @blobHandle, @blobSegLen, MAXBLOBSEGMENTSIZE, blobSegment) = 0) do
      begin
      ReAllocMem(Buffer,Size+blobSegLen);
      ptr := Buffer+Size;
      move(blobSegment^,ptr^,blobSegLen);
      inc(Size,blobSegLen);
      end;
    freemem(blobSegment);

    if FStatus[1] = isc_segstr_eof then
      begin
        if isc_close_blob(@FStatus[0], @blobHandle) <> 0 then
          CheckError('TIBConnection.CreateBlobStream isc_close_blob', FStatus);
      end
    else
      CheckError('TIBConnection.CreateBlobStream isc_get_segment', FStatus);
  end;
end;

function TIBConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;

var info_request       : string;
    resbuf             : array[0..63] of byte;
    i                  : integer;
    BlockSize,
    subBlockSize       : integer;
    SelectedRows,
    InsertedRows       : integer;
    
begin
  SelectedRows:=-1;
  InsertedRows:=-1;

  if assigned(cursor) then with cursor as TIBCursor do
   if assigned(statement) then
    begin
    info_request := chr(isc_info_sql_records);
    if isc_dsql_sql_info(@Status[0],@Statement,Length(info_request), @info_request[1],sizeof(resbuf),@resbuf) <> 0 then
      CheckError('RowsAffected', Status);

    i := 0;
    while not (byte(resbuf[i]) in [isc_info_end,isc_info_truncated]) do
      begin
      BlockSize:=isc_vax_integer(@resbuf[i+1],2);
      if resbuf[i]=isc_info_sql_records then
        begin
        inc(i,3);
        BlockSize:=BlockSize+i;
        while (resbuf[i] <> isc_info_end) and (i < BlockSize) do
          begin
          subBlockSize:=isc_vax_integer(@resbuf[i+1],2);
          if resbuf[i] = isc_info_req_select_count then
            SelectedRows := isc_vax_integer(@resbuf[i+3],subBlockSize)
          else if resbuf[i] = isc_info_req_insert_count then
            InsertedRows := isc_vax_integer(@resbuf[i+3],subBlockSize);
          inc(i,subBlockSize+3);
          end;
        end
      else
        inc(i,BlockSize+3);
      end;
    end;
  if SelectedRows>0 then result:=SelectedRows
  else Result:=InsertedRows;
end;

function TIBConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;

var info_request       : string;
    resbuf             : array[0..63] of byte;
    i                  : integer;
    BlockSize,
    subBlockSize       : integer;
    SelectedRows,
    InsertedRows       : integer;
    
begin
  SelectedRows:=-1;
  InsertedRows:=-1;

  if assigned(cursor) then with cursor as TIBCursor do
   if assigned(statement) then
    begin
    info_request := chr(isc_info_sql_records);
    if isc_dsql_sql_info(@Status[0],@Statement,Length(info_request), @info_request[1],sizeof(resbuf),@resbuf) <> 0 then
      CheckError('RowsAffected', Status);

    i := 0;
    while not (byte(resbuf[i]) in [isc_info_end,isc_info_truncated]) do
      begin
      BlockSize:=isc_vax_integer(@resbuf[i+1],2);
      if resbuf[i]=isc_info_sql_records then
        begin
        inc(i,3);
        BlockSize:=BlockSize+i;
        while (resbuf[i] <> isc_info_end) and (i < BlockSize) do
          begin
          subBlockSize:=isc_vax_integer(@resbuf[i+1],2);
          if resbuf[i] = isc_info_req_select_count then
            SelectedRows := isc_vax_integer(@resbuf[i+3],subBlockSize)
          else if resbuf[i] = isc_info_req_insert_count then
            InsertedRows := isc_vax_integer(@resbuf[i+3],subBlockSize);
          inc(i,subBlockSize+3);
          end;
        end
      else
        inc(i,BlockSize+3);
      end;
    end;
  if SelectedRows>0 then result:=SelectedRows
  else Result:=InsertedRows;
end;

function TIBConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;

var info_request       : string;
    resbuf             : array[0..63] of byte;
    i                  : integer;
    BlockSize,
    subBlockSize       : integer;
    SelectedRows,
    InsertedRows       : integer;
    
begin
  SelectedRows:=-1;
  InsertedRows:=-1;

  if assigned(cursor) then with cursor as TIBCursor do
   if assigned(statement) then
    begin
    info_request := chr(isc_info_sql_records);
    if isc_dsql_sql_info(@Status[0],@Statement,Length(info_request), @info_request[1],sizeof(resbuf),@resbuf) <> 0 then
      CheckError('RowsAffected', Status);

    i := 0;
    while not (byte(resbuf[i]) in [isc_info_end,isc_info_truncated]) do
      begin
      BlockSize:=isc_vax_integer(@resbuf[i+1],2);
      if resbuf[i]=isc_info_sql_records then
        begin
        inc(i,3);
        BlockSize:=BlockSize+i;
        while (resbuf[i] <> isc_info_end) and (i < BlockSize) do
          begin
          subBlockSize:=isc_vax_integer(@resbuf[i+1],2);
          if resbuf[i] = isc_info_req_select_count then
            SelectedRows := isc_vax_integer(@resbuf[i+3],subBlockSize)
          else if resbuf[i] = isc_info_req_insert_count then
            InsertedRows := isc_vax_integer(@resbuf[i+3],subBlockSize);
          inc(i,subBlockSize+3);
          end;
        end
      else
        inc(i,BlockSize+3);
      end;
    end;
  if SelectedRows>0 then result:=SelectedRows
  else Result:=InsertedRows;
end;

{ TIBConnectionDef }

class function TIBConnectionDef.TypeName: String;
begin
  Result:='Firebird';
end;
  
class function TIBConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  Result:=TIBConnection;
end;
    
class function TIBConnectionDef.Description: String;
begin
  Result:='Connect to Firebird/Interbase directly via the client library';
end;

class function TIBConnectionDef.DefaultLibraryName: String;
begin
{$IFDEF LinkDynamically}
  If UseEmbeddedFirebird then
    Result:=fbembedlib
  else
    Result:=fbclib;
{$ELSE}
  Result:='';
{$ENDIF}
end;

class function TIBConnectionDef.LoadFunction: TLibraryLoadFunction;
begin
{$IFDEF LinkDynamically}
  Result:=@InitialiseIBase60;
{$ELSE}
  Result:=nil;
{$ENDIF}
end;

class function TIBConnectionDef.UnLoadFunction: TLibraryUnLoadFunction;
begin
{$IFDEF LinkDynamically}
  Result:=@ReleaseIBase60
{$ELSE}
  Result:=nil;
{$ENDIF}
end;

class function TIBConnectionDef.LoadedLibraryName: string;
begin
{$IFDEF LinkDynamically}
  Result:=IBaseLoadedLibrary;
{$ELSE}
  Result:='';
{$ENDIF}
end;

initialization
  RegisterConnection(TIBConnectionDef);

finalization
  UnRegisterConnection(TIBConnectionDef);
end.
