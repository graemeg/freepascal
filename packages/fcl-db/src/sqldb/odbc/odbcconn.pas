(******************************************************************************
 *                                                                            *
 *  (c) 2005 Hexis BV                                                         *
 *                                                                            *
 *  File:        odbcconn.pas                                                 *
 *  Author:      Bram Kuijvenhoven (bkuijvenhoven@eljakim.nl)                 *
 *  Description: ODBC SQLDB unit                                              *
 *  License:     (modified) LGPL                                              *
 *                                                                            *
 ******************************************************************************)

unit odbcconn;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, db, sqldb, BufDataset, odbcsqldyn;

type

  // forward declarations
  TODBCConnection = class;

  { TODBCCursor }

  TODBCCursor = class(TSQLCursor)
  protected
    FSTMTHandle:SQLHSTMT; // ODBC Statement Handle
    FQuery:string;        // last prepared query, with :ParamName converted to ?
    FParamIndex:TParamBinding; // maps the i-th parameter in the query to the TParams passed to PrepareStatement
    FParamBuf:array of pointer; // buffers that can be used to bind the i-th parameter in the query
  public
    constructor Create(Connection:TODBCConnection);
    destructor Destroy; override;
  end;

  { TODBCHandle } // this name is a bit confusing, but follows the standards for naming classes in sqldb

  TODBCHandle = class(TSQLHandle)
  protected
  end;

  { TODBCEnvironment }

  TODBCEnvironment = class
  protected
    FENVHandle:SQLHENV; // ODBC Environment Handle
  public
    constructor Create;
    destructor Destroy; override;
  end;

  { TODBCConnection }

  TODBCConnection = class(TSQLConnection)
  private
    type
      TDBMSInfo = record
        GetLastInsertIDSQL: string; // SQL statement for get last insert value for autoincrement column
      end;

    var
      FDriver: string;
      FEnvironment:TODBCEnvironment;
      FDBCHandle:SQLHDBC; // ODBC Connection Handle
      FFileDSN: string;
      FDBMSInfo: TDBMSInfo;

    procedure SetParameters(ODBCCursor:TODBCCursor; AParams:TParams);
    procedure FreeParamBuffers(ODBCCursor:TODBCCursor);
  protected
    // Overrides from TSQLConnection
    function GetConnectionCharSet: string; override;
    function GetHandle:pointer; override;
    // - Connect/disconnect
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    // - Handle (de)allocation
    function AllocateCursorHandle:TSQLCursor; override;
    procedure DeAllocateCursorHandle(var cursor:TSQLCursor); override;
    function AllocateTransactionHandle:TSQLHandle; override;
    // - Statement handling
    function StrToStatementType(s : string) : TStatementType; override;
    procedure PrepareStatement(cursor:TSQLCursor; ATransaction:TSQLTransaction; buf:string; AParams:TParams); override;
    procedure UnPrepareStatement(cursor:TSQLCursor); override;
    // - Transaction handling
    function GetTransactionHandle(trans:TSQLHandle):pointer; override;
    function StartDBTransaction(trans:TSQLHandle; AParams:string):boolean; override;
    function Commit(trans:TSQLHandle):boolean; override;
    function Rollback(trans:TSQLHandle):boolean; override;
    procedure CommitRetaining(trans:TSQLHandle); override;
    procedure RollbackRetaining(trans:TSQLHandle); override;
    // - Statement execution
    procedure Execute(cursor:TSQLCursor; ATransaction:TSQLTransaction; AParams:TParams); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function RefreshLastInsertID(Query : TCustomSQLQuery; Field : TField): boolean; override;
    // - Result retrieving
    procedure AddFieldDefs(cursor:TSQLCursor; FieldDefs:TFieldDefs); override;
    function Fetch(cursor:TSQLCursor):boolean; override;
    function LoadField(cursor:TSQLCursor; FieldDef:TFieldDef; buffer:pointer; out CreateBlob : boolean):boolean; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    procedure FreeFldBuffers(cursor:TSQLCursor); override;
    // - UpdateIndexDefs
    procedure UpdateIndexDefs(IndexDefs:TIndexDefs; TableName:string); override;
    // - Schema info
    function GetSchemaInfoSQL(SchemaType:TSchemaType; SchemaObjectName, SchemaObjectPattern:string):string; override;

    // Internal utility functions
    function CreateConnectionString:string;
  public
    constructor Create(AOwner : TComponent); override;
    function GetConnectionInfo(InfoType:TConnInfoType): string; override;
    property Environment:TODBCEnvironment read FEnvironment;
  published
    property Driver:string read FDriver write FDriver;    // will be passed as DRIVER connection parameter
    property FileDSN:string read FFileDSN write FFileDSN; // will be passed as FILEDSN parameter
    // Redeclare properties from TSQLConnection
    property Password;     // will be passed as PWD connection parameter
    property Transaction;
    property UserName;     // will be passed as UID connection parameter
    property CharSet;
    property HostName;     // ignored
    // Redeclare properties from TDatabase
    property Connected;
    property Role;
    property DatabaseName; // will be passed as DSN connection parameter
    property KeepConnection;
    property LoginPrompt;  // if true, ODBC drivers might prompt for more details that are not in the connection string
    property Params;       // will be added to connection string
    property OnLogin;
  end;

  EODBCException = class(ESQLDatabaseError);

  { TODBCConnectionDef }

  TODBCConnectionDef = Class(TConnectionDef)
    Class Function TypeName : String; override;
    Class Function ConnectionClass : TSQLConnectionClass; override;
    Class Function Description : String; override;
  end;

implementation

uses
  ctypes;

const
  DefaultEnvironment:TODBCEnvironment = nil;
  ODBCLoadCount:integer = 0; // ODBC is loaded when > 0; modified by TODBCEnvironment.Create/Destroy

{ Generic ODBC helper functions }

function ODBCSuccess(const Res:SQLRETURN):boolean;
begin
  Result:=(Res=SQL_SUCCESS) or (Res=SQL_SUCCESS_WITH_INFO);
end;

function ODBCResultToStr(Res:SQLRETURN):string;
begin
  case Res of
    SQL_SUCCESS:          Result:='SQL_SUCCESS';
    SQL_SUCCESS_WITH_INFO:Result:='SQL_SUCCESS_WITH_INFO';
    SQL_ERROR:            Result:='SQL_ERROR';
    SQL_INVALID_HANDLE:   Result:='SQL_INVALID_HANDLE';
    SQL_NO_DATA:          Result:='SQL_NO_DATA';
    SQL_NEED_DATA:        Result:='SQL_NEED_DATA';
    SQL_STILL_EXECUTING:  Result:='SQL_STILL_EXECUTING';
  else
    Result:='';
  end;
end;

procedure ODBCCheckResult(LastReturnCode:SQLRETURN; HandleType:SQLSMALLINT; AHandle: SQLHANDLE; ErrorMsg: string; const FmtArgs:array of const);

  // check return value from SQLGetDiagField/Rec function itself
  procedure CheckSQLGetDiagResult(const Res:SQLRETURN);
  begin
    case Res of
      SQL_INVALID_HANDLE:
        raise EODBCException.Create('Invalid handle passed to SQLGetDiagRec/Field');
      SQL_ERROR:
        raise EODBCException.Create('An invalid parameter was passed to SQLGetDiagRec/Field');
      SQL_NO_DATA:
        raise EODBCException.Create('A too large RecNumber was passed to SQLGetDiagRec/Field');
    end;
  end;

var
  NativeError, NativeError1: SQLINTEGER;
  TextLength:SQLSMALLINT;
  Res:SQLRETURN;
  SqlState, SQLState1, MessageText, TotalMessage: string;
  RecNumber:SQLSMALLINT;
begin
  // check result
  if ODBCSuccess(LastReturnCode) then
    Exit; // no error; all is ok

  //WriteLn('LastResultCode: ',ODBCResultToStr(LastReturnCode));
  try
    NativeError1:=0;
    SQLState1:='';
    // build TotalMessage for exception to throw
    TotalMessage:=Format(ErrorMsg,FmtArgs)+Format(' ODBC error details: LastReturnCode: %s;',[ODBCResultToStr(LastReturnCode)]);
    // retrieve status records
    SetLength(MessageText,1);
    RecNumber:=1;
    repeat
      SetLength(SqlState,5); // reset 5-character buffer
      // dummy call to get correct TextLength
      //WriteLn('Getting error record ',RecNumber);
      Res:=SQLGetDiagRec(HandleType,AHandle,RecNumber,@(SqlState[1]),NativeError,@(MessageText[1]),0,TextLength);
      if Res=SQL_NO_DATA then
        Break; // no more status records
      CheckSQLGetDiagResult(Res);
      if TextLength>0 then // if TextLength=0 we don't need another call; also our string buffer would not point to a #0, but be a nil pointer
      begin
        // allocate large enough buffer
        SetLength(MessageText,TextLength); // note: ansistrings of Length>0 are always terminated by a #0 character, so this is safe
        // actual call
        Res:=SQLGetDiagRec(HandleType,AHandle,RecNumber,@(SqlState[1]),NativeError,@(MessageText[1]),Length(MessageText)+1,TextLength);
        CheckSQLGetDiagResult(Res);
      end;
      // add to TotalMessage
      TotalMessage:=TotalMessage+Format(' Record %d: SqlState: %s; NativeError: %d; Message: %s;',[RecNumber,SqlState,NativeError,MessageText]);
      // save most significant error
      if RecNumber = 1 then
      begin
        NativeError1 := NativeError;
        SQLState1 := SqlState;
      end;
      // incement counter
      Inc(RecNumber);
    until false;
  except
    on E:EODBCException do begin
      TotalMessage:=TotalMessage+Format('Could not get error message: %s',[E.Message]);
    end
  end;
  // raise error
  raise EODBCException.CreateFmt(TotalMessage, [], nil, NativeError1, SQLState1);
end;

procedure ODBCCheckResult(LastReturnCode:SQLRETURN; HandleType:SQLSMALLINT; AHandle: SQLHANDLE; ErrorMsg: string);
begin
  ODBCCheckResult(LastReturnCode, HandleType, AHandle, ErrorMsg, []);
end;

{ TODBCConnection }

// Creates a connection string using the current value of the fields
function TODBCConnection.CreateConnectionString: string;

  // encloses a param value with braces if necessary, i.e. when any of the characters []{}(),;?*=!@ is in the value
  function EscapeParamValue(const s:string):string;
  var
    NeedEscape:boolean;
    i:integer;
  begin
    NeedEscape:=false;
    for i:=1 to Length(s) do
      if s[i] in ['[',']','{','}','(',')',',','*','=','!','@'] then
      begin
        NeedEscape:=true;
        Break;
      end;
    if NeedEscape then
      Result:='{'+s+'}'
    else
      Result:=s;
  end;

var
  i: Integer;
  Param: string;
  EqualSignPos:integer;
begin
  Result:='';
  if DatabaseName<>'' then Result:=Result + 'DSN='+EscapeParamValue(DatabaseName)+';';
  if Driver      <>'' then Result:=Result + 'DRIVER='+EscapeParamValue(Driver)+';';
  if UserName    <>'' then Result:=Result + 'UID='+EscapeParamValue(UserName)+';PWD='+EscapeParamValue(Password)+';';
  if FileDSN     <>'' then Result:=Result + 'FILEDSN='+EscapeParamValue(FileDSN)+'';
  for i:=0 to Params.Count-1 do
  begin
    Param:=Params[i];
    EqualSignPos:=Pos('=',Param);
    if EqualSignPos=0 then
      raise EODBCException.CreateFmt('Invalid parameter in Params[%d]; can''t find a ''='' in ''%s''',[i, Param])
    else if EqualSignPos=1 then
      raise EODBCException.CreateFmt('Invalid parameter in Params[%d]; no identifier before the ''='' in ''%s''',[i, Param])
    else
      Result:=Result + EscapeParamValue(Copy(Param,1,EqualSignPos-1))+'='+EscapeParamValue(Copy(Param,EqualSignPos+1,MaxInt))+';';
  end;
end;

constructor TODBCConnection.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FConnOptions := FConnOptions + [sqSupportEmptyDatabaseName, sqEscapeRepeat, sqEscapeSlash];
end;

function TODBCConnection.StrToStatementType(s : string) : TStatementType;
begin
  case Lowercase(s) of
    'transform': // MS Access
      Result := stSelect;
    'exec', 'call':
      Result := stExecProcedure;
    else
      Result := inherited StrToStatementType(s);
  end;
end;

(*
function BCDToNumericStruct(const bcd: TBCD): SQL_NUMERIC_STRUCT;
var i, j, p: integer;
    nibble: Byte;
    a, m, carry: DWord;
    BigInt: array[0..3] of QWord;
    qw: QWord;
    pdw: PDWord;
begin
  Result.precision := BCDPrecision(bcd);
  if Result.precision = 0 then Result.precision := 1; // if bcd is NULL
  Result.scale := BCDScale(bcd);
  if IsBCDNegative(bcd) then
    Result.sign := 0
  else
    Result.sign := 1;
  // BigInt := 0
  FillByte(BigInt, sizeof(BigInt), 0);
  // process BCD nibbles (high nibble is at 0)
  p := Result.precision;
  i := 0;
  while p > 0 do
  begin
    nibble := bcd.Fraction[i];
    if p = 1 then
      begin
      a := nibble shr 4;
      m := 10;
      end
    else
      begin
      a := (nibble shr 4)*10 + nibble and $0F;
      m := 100;
      end;

    // BigInt := BigInt * m + a
    // big multiplication
    j := 0; carry := 0;
    repeat
      qw := BigInt[j] * m + carry;
      BigInt[j] := qw and $FFFFFFFF;
      carry := qw shr 32;
      inc(j);
    until j>high(BigInt);

    // big addition
    j := 0; carry := 0;
    repeat
      qw := BigInt[j] + a + carry;
      BigInt[j] := qw and $FFFFFFFF;
      carry := qw shr 32;
      inc(j);
    until (carry = 0) or (j>high(BigInt));

    dec(p,2);
    inc(i);
  end;

  // SQL_NUMERIC_STRUCT.val size must be 16 bytes (128bit integer)
  pdw := @Result.val;
  for j:=0 to high(BigInt) do
    begin
    pdw^ := NtoLE(BigInt[j]);
    inc(pdw);
    end;
end;
*)

procedure TODBCConnection.SetParameters(ODBCCursor: TODBCCursor; AParams: TParams);
var
  ParamIndex: integer;
  PVal, Buf, PStrLenOrInd: pointer;
  I, Size: integer;
  IntVal: clong;
  LargeVal: clonglong;
  StrVal: ansistring;
  WideStrVal: widestring;
  FloatVal: cdouble;
  DateVal: SQL_DATE_STRUCT;
  TimeVal: SQL_TIME_STRUCT;
  TimeStampVal: SQL_TIMESTAMP_STRUCT;
  BoolVal: byte;
  NumericVal: SQL_NUMERIC_STRUCT;
  ColumnSize: SQLULEN;
  BufferLength, StrLenOrInd: SQLLEN;
  CType, SqlType, DecimalDigits:SQLSMALLINT;
  APD: SQLHDESC;
  BytesVal: TBytes;
begin
  // Note: it is assumed that AParams is the same as the one passed to PrepareStatement, in the sense that
  //       the parameters have the same order and names

  if Length(ODBCCursor.FParamIndex)>0 then
    if not Assigned(AParams) then
      raise EODBCException.CreateFmt('The query has parameter markers in it, but no actual parameters were passed',[]);

  SetLength(ODBCCursor.FParamBuf, Length(ODBCCursor.FParamIndex));
  for i:=0 to High(ODBCCursor.FParamIndex) do
    ODBCCursor.FParamBuf[i]:=nil;
  for i:=0 to High(ODBCCursor.FParamIndex) do
  begin
    ParamIndex:=ODBCCursor.FParamIndex[i];
    if (ParamIndex<0) or (ParamIndex>=AParams.Count) then
      raise EODBCException.CreateFmt('Parameter %d in query does not have a matching parameter set',[i]);

    DecimalDigits:=0;
    BufferLength:=0;
    StrLenOrInd:=0;

    case AParams[ParamIndex].DataType of
      ftInteger, ftSmallInt, ftWord, ftAutoInc:
        begin
          IntVal:=AParams[ParamIndex].AsInteger;
          PVal:=@IntVal;
          Size:=SizeOf(IntVal);
          CType:=SQL_C_LONG;
          SqlType:=SQL_INTEGER;
          ColumnSize:=10;
        end;
      ftLargeInt:
        begin
          LargeVal:=AParams[ParamIndex].AsLargeInt;
          PVal:=@LargeVal;
          Size:=SizeOf(LargeVal);
          CType:=SQL_C_SBIGINT;
          SqlType:=SQL_BIGINT;
          ColumnSize:=19;
        end;
      ftBlob, ftBytes, ftVarBytes:
        begin
          BytesVal:=AParams[ParamIndex].AsBytes;
          StrLenOrInd:=Length(BytesVal);
          if Length(BytesVal)=0 then //HY104
             begin
             BytesVal:=[0];
             StrLenOrInd:=SQL_NTS;
             end;
          PVal:=@BytesVal[0];
          Size:=Length(BytesVal);
          ColumnSize:=Size;
          BufferLength:=Size;
          CType:=SQL_C_BINARY;
          case AParams[ParamIndex].DataType of
            ftBytes, ftVarBytes: SqlType:=SQL_VARBINARY;
            else // ftBlob
              SqlType:=SQL_LONGVARBINARY;
          end;
        end;
      ftString, ftFixedChar, ftMemo, ftGuid, // string parameters must be passed as widestring to support FPC 3.0.x character conversion
      ftWideString, ftFixedWideChar, ftWideMemo:
        begin
          WideStrVal:=AParams[ParamIndex].AsWideString;
          StrLenOrInd:=Length(WideStrVal)*sizeof(widechar);
          if WideStrVal='' then //HY104
             begin
             WideStrVal:=#0;
             StrLenOrInd:=SQL_NTS;
             end;
          PVal:=@WideStrVal[1];
          Size:=Length(WideStrVal)*sizeof(widechar);
          ColumnSize:=Size; //The defined or maximum column size in characters of the column or parameter
          BufferLength:=Size;
          CType:=SQL_C_WCHAR;
          case AParams[ParamIndex].DataType of
            ftMemo, ftWideMemo: SqlType:=SQL_WLONGVARCHAR;
            else        SqlType:=SQL_WVARCHAR;
          end;
        end;
      ftFloat:
        begin
          FloatVal:=AParams[ParamIndex].AsFloat;
          PVal:=@FloatVal;
          Size:=SizeOf(FloatVal);
          CType:=SQL_C_DOUBLE;
          SqlType:=SQL_DOUBLE;
          ColumnSize:=15;
        end;
      ftCurrency, ftBCD:
        begin
          NumericVal:=CurrToNumericStruct(AParams[ParamIndex].AsCurrency);
          PVal:=@NumericVal;
          Size:=SizeOf(NumericVal);
          CType:=SQL_C_NUMERIC;
          SqlType:=SQL_NUMERIC;
          ColumnSize:=NumericVal.precision;
          DecimalDigits:=NumericVal.scale;
        end;
      ftFmtBCD:
        begin
          // bind FmtBCD parameter as string to support higher precision than 10^38 (supported by SQL_NUMERIC_STRUCT)
          StrVal:=GetAsSQLText(AParams[ParamIndex]);
          StrLenOrInd:=Length(StrVal);
          PVal:=@StrVal[1];
          Size:=Length(StrVal);
          CType:=SQL_C_CHAR;
          SqlType:=SQL_CHAR;
          ColumnSize:=Size;
          BufferLength:=Size;
        end;
      ftDate:
        begin
          DateVal:=DateTimeToDateStruct(AParams[ParamIndex].AsDate);
          PVal:=@DateVal;
          Size:=SizeOf(DateVal);
          CType:=SQL_C_TYPE_DATE;
          SqlType:=SQL_TYPE_DATE;
          ColumnSize:=10;
        end;
      ftTime:
        begin
          TimeVal:=DateTimeToTimeStruct(AParams[ParamIndex].AsTime);
          PVal:=@TimeVal;
          Size:=SizeOf(TimeVal);
          CType:=SQL_C_TYPE_TIME;
          SqlType:=SQL_TYPE_TIME;
          ColumnSize:=12;
        end;
      ftDateTime:
        begin
          DateTime2TimeStampStruct(TimeStampVal, AParams[ParamIndex].AsDateTime);
          PVal:=@TimeStampVal;
          Size:=SizeOf(TimeStampVal);
          CType:=SQL_C_TYPE_TIMESTAMP;
          SqlType:=SQL_TYPE_TIMESTAMP;
          ColumnSize:=23;
          DecimalDigits:=3; // fractional seconds
        end;
      ftBoolean:
        begin
          BoolVal:=ord(AParams[ParamIndex].AsBoolean);
          PVal:=@BoolVal;
          Size:=SizeOf(BoolVal);
          CType:=SQL_C_BIT;
          SqlType:=SQL_BIT;
          ColumnSize:=Size;
        end;
      ftUnknown:
        begin
          if AParams[ParamIndex].IsNull then // Null variant is stored as ftUnknown - send it over as TINYINT (it doesn't really matter what type)
          begin
            PVal:=nil;
            Size:=0;
            CType:=SQL_C_TINYINT;
            SqlType:=SQL_TINYINT;
            ColumnSize:=Size;
          end else
            raise EDataBaseError.CreateFmt('Not-null unknown Parameter (index %d) is not supported.',[ParamIndex]);
        end;
      else
        raise EDataBaseError.CreateFmt('Parameter %d is of type %s, which not supported yet',[ParamIndex, Fieldtypenames[AParams[ParamIndex].DataType]]);
    end;

    if AParams[ParamIndex].IsNull then
       StrLenOrInd:=SQL_NULL_DATA;

    Buf:=GetMem(Size+SizeOf(StrLenOrInd));
    if Size>0 then
      Move(PVal^, Buf^, Size);
    if StrLenOrInd<>0 then
       begin
       PStrLenOrInd:=Buf + Size;
       Move(StrLenOrInd, PStrLenOrInd^, SizeOf(StrLenOrInd));
       end
    else
       PStrLenOrInd:=nil;
    ODBCCursor.FParamBuf[i]:=Buf;

    ODBCCheckResult(
         SQLBindParameter(ODBCCursor.FSTMTHandle, // StatementHandle
                          i+1,                    // ParameterNumber
                          SQL_PARAM_INPUT,        // InputOutputType
                          CType,                  // ValueType
                          SqlType,                // ParameterType
                          ColumnSize,             // ColumnSize
                          DecimalDigits,          // DecimalDigits
                          Buf,                    // ParameterValuePtr
                          BufferLength,           // BufferLength
                          PStrLenOrInd),          // StrLen_or_IndPtr
         SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not bind parameter %d.', [i]
       );

    // required by MSSQL:
    if CType = SQL_C_NUMERIC then
    begin
      ODBCCheckResult(
        SQLGetStmtAttr(ODBCCursor.FSTMTHandle, SQL_ATTR_APP_PARAM_DESC, @APD, 0, nil),
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get parameter descriptor.'
      );
      SQLSetDescRec(APD, i+1, SQL_C_NUMERIC, 0, ColumnSize+2, ColumnSize, DecimalDigits, Buf, PStrLenOrInd, PStrLenOrInd);
    end;
  end;
end;

procedure TODBCConnection.FreeParamBuffers(ODBCCursor: TODBCCursor);
var
  i:integer;
begin
  for i:=0 to High(ODBCCursor.FParamBuf) do
    if assigned(ODBCCursor.FParamBuf[i]) then
      FreeMem(ODBCCursor.FParamBuf[i]);
  SetLength(ODBCCursor.FParamBuf,0);
end;

function TODBCConnection.GetConnectionCharSet: string;
begin
  Result := inherited GetConnectionCharSet;
  if Result='' then
    Result := TEncoding.ANSI.EncodingName; // by default, ODBC talks in ANSI, which can be different from CP_ACP (DefaultSystemCodePage)
end;

function TODBCConnection.GetHandle: pointer;
begin
  // I'm not sure whether this is correct; perhaps we should return nil
  // note that FDBHandle is a LongInt, because ODBC handles are integers, not pointers
  // I wonder how this will work on 64 bit platforms then (FK)
  Result:=pointer(PtrInt(FDBCHandle));
end;

procedure TODBCConnection.DoInternalConnect;
const
  BufferLength = 1024; // should be at least 1024 according to the ODBC specification
var
  ConnectionString:string;
  OutConnectionString:string;
  ActualLength:SQLSMALLINT;
  DBMS_NAME: array[0..20] of AnsiChar;
begin
  // Do not call the inherited method as it checks for a non-empty DatabaseName, and we don't even use DatabaseName!
  // inherited DoInternalConnect;

  // make sure we have an environment
  if not Assigned(FEnvironment) then
  begin
    if not Assigned(DefaultEnvironment) then
      DefaultEnvironment:=TODBCEnvironment.Create;
    FEnvironment:=DefaultEnvironment;
  end;

  // allocate connection handle
  ODBCCheckResult(
    SQLAllocHandle(SQL_HANDLE_DBC,Environment.FENVHandle,FDBCHandle),
    SQL_HANDLE_ENV,Environment.FENVHandle,'Could not allocate ODBC Connection handle.'
  );

  try
    // connect
    ConnectionString:=CreateConnectionString;
    SetLength(OutConnectionString,BufferLength-1); // allocate completed connection string buffer (using the ansistring #0 trick)
    ODBCCheckResult(
      SQLDriverConnect(FDBCHandle,               // the ODBC connection handle
                       nil,                      // no parent window (would be required for prompts)
                       PChar(ConnectionString),  // the connection string
                       Length(ConnectionString), // connection string length
                       @(OutConnectionString[1]),// buffer for storing the completed connection string
                       BufferLength,             // length of the buffer
                       ActualLength,             // the actual length of the completed connection string
                       SQL_DRIVER_NOPROMPT),     // don't prompt for password etc.
      SQL_HANDLE_DBC,FDBCHandle,'Could not connect with connection string "%s".',[ConnectionString]
    );
  except
    on E:Exception do begin
      // free connection handle
      ODBCCheckResult(
        SQLFreeHandle(SQL_HANDLE_DBC,FDBCHandle),
        SQL_HANDLE_DBC,FDBCHandle,'Could not free ODBC Connection handle.'
      );
      raise; // re-raise exception
    end;
  end;

  // set DBMS specific options
  if SQLGetInfo(FDBCHandle, SQL_DBMS_NAME, @DBMS_NAME, sizeof(DBMS_NAME), @ActualLength) = SQL_SUCCESS then
    case AnsiString(DBMS_NAME) of
      'Microsoft SQL Server':
        begin
        FDBMSInfo.GetLastInsertIDSQL := 'SELECT @@IDENTITY';
        Include(FConnOptions, sqLastInsertID);
        end;
      'MySQL':
        begin
        FDBMSInfo.GetLastInsertIDSQL := 'SELECT last_insert_id()';
        Include(FConnOptions, sqLastInsertID);
        end;
      else
        begin
        FDBMSInfo.GetLastInsertIDSQL := '';
        Exclude(FConnOptions, sqLastInsertID);
        end;
    end;

  // set connection attributes (none yet)
end;

procedure TODBCConnection.DoInternalDisconnect;
var
  Res:SQLRETURN;
begin
  inherited DoInternalDisconnect;

  // disconnect
  ODBCCheckResult(
    SQLDisconnect(FDBCHandle),
    SQL_HANDLE_DBC,FDBCHandle,'Could not disconnect.'
  );

  // deallocate connection handle
  Res:=SQLFreeHandle(SQL_HANDLE_DBC, FDBCHandle);
  if Res=SQL_ERROR then
    ODBCCheckResult(Res,SQL_HANDLE_DBC,FDBCHandle,'Could not free ODBC Connection handle.');
end;

function TODBCConnection.AllocateCursorHandle: TSQLCursor;
begin
  Result:=TODBCCursor.Create(self);
end;

procedure TODBCConnection.DeAllocateCursorHandle(var cursor: TSQLCursor);
begin
  FreeAndNil(cursor); // the destructor of TODBCCursor frees the ODBC Statement handle
end;

function TODBCConnection.AllocateTransactionHandle: TSQLHandle;
begin
  Result:=nil; // not yet supported; will move connection handles to transaction handles later
end;

procedure TODBCConnection.PrepareStatement(cursor: TSQLCursor; ATransaction: TSQLTransaction; buf: string; AParams: TParams);
var
  ODBCCursor:TODBCCursor;
  wbuf: widestring;
begin
  ODBCCursor:=cursor as TODBCCursor;

  // allocate statement handle
  ODBCCheckResult(
    SQLAllocHandle(SQL_HANDLE_STMT, FDBCHandle, ODBCCursor.FSTMTHandle),
    SQL_HANDLE_DBC, FDBCHandle, 'Could not allocate ODBC Statement handle.'
  );
  ODBCCursor.FPrepared:=True;

  // Parameter handling
  // Note: We can only pass ? parameters to ODBC, so we should convert named parameters like :MyID
  //       ODBCCursor.FParamIndex will map th i-th ? token in the (modified) query to an index for AParams

  // Parse the SQL and build FParamIndex
  if assigned(AParams) and (AParams.count > 0) then
    begin
    buf := AParams.ParseSQL(buf,false,sqEscapeSlash in ConnOptions, sqEscapeRepeat in ConnOptions,psInterbase,ODBCCursor.FParamIndex);
    if LogEvent(detActualSQL) then
      Log(detActualSQL,Buf);
    end;

  // prepare statement
  ODBCCursor.FQuery:=Buf;
  if not (ODBCCursor.FSchemaType in [stTables, stSysTables, stColumns, stProcedures, stIndexes]) then
    begin
      wbuf := buf;
      ODBCCheckResult(
        SQLPrepareW(ODBCCursor.FSTMTHandle, PWideChar(wbuf), Length(wbuf)),
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not prepare statement.'
      );
    end;
  if ODBCCursor.FSchemaType <> stNoSchema then
    ODBCCursor.FStatementType:=stSelect;
end;

procedure TODBCConnection.UnPrepareStatement(cursor: TSQLCursor);
var Res:SQLRETURN;
begin
  with TODBCCursor(cursor) do
  begin
    if FSTMTHandle<>SQL_NULL_HSTMT then
    begin
      // deallocate statement handle
      Res:=SQLFreeHandle(SQL_HANDLE_STMT, FSTMTHandle);
      if Res=SQL_ERROR then
        ODBCCheckResult(Res,SQL_HANDLE_STMT, FSTMTHandle, 'Could not free ODBC Statement handle.');
      FSTMTHandle:=SQL_NULL_HSTMT;
      FPrepared := False;
    end;
  end;
end;

function TODBCConnection.GetTransactionHandle(trans: TSQLHandle): pointer;
begin
  Result := nil;
end;

function TODBCConnection.StartDBTransaction(trans: TSQLHandle; AParams:string): boolean;
var AutoCommit: SQLINTEGER;
begin
  // set some connection attributes
  if StrToBoolDef(Params.Values['AUTOCOMMIT'], False) then
    AutoCommit := SQL_AUTOCOMMIT_ON
  else
    AutoCommit := SQL_AUTOCOMMIT_OFF;

  ODBCCheckResult(
    SQLSetConnectAttr(FDBCHandle, SQL_ATTR_AUTOCOMMIT, SQLPOINTER(AutoCommit), SQL_IS_UINTEGER),
    SQL_HANDLE_DBC,FDBCHandle,'Could not start transaction!'
  );

  Result := AutoCommit=SQL_AUTOCOMMIT_OFF;
end;

function TODBCConnection.Commit(trans: TSQLHandle): boolean;
begin
  ODBCCheckResult(
    SQLEndTran(SQL_HANDLE_DBC, FDBCHandle, SQL_COMMIT),
    SQL_HANDLE_DBC, FDBCHandle, 'Could not commit!'
  );
  Result := True;
end;

function TODBCConnection.Rollback(trans: TSQLHandle): boolean;
begin
  ODBCCheckResult(
    SQLEndTran(SQL_HANDLE_DBC, FDBCHandle, SQL_ROLLBACK),
    SQL_HANDLE_DBC, FDBCHandle, 'Could not rollback!'
  );
  Result := True;
end;

procedure TODBCConnection.CommitRetaining(trans: TSQLHandle);
begin
  Commit(trans);
end;

procedure TODBCConnection.RollbackRetaining(trans: TSQLHandle);
begin
  Rollback(trans);
end;

procedure TODBCConnection.Execute(cursor: TSQLCursor; ATransaction: TSQLTransaction; AParams: TParams);
const
  TABLE_TYPE_USER='TABLE,VIEW,GLOBAL TEMPORARY,LOCAL TEMPORARY'; //no spaces before/after comma
  TABLE_TYPE_SYSTEM='SYSTEM TABLE';
var
  ODBCCursor:TODBCCursor;
  Res:SQLRETURN;
  ColumnCount:SQLSMALLINT;
begin
  ODBCCursor:=cursor as TODBCCursor;

  try
    // set parameters
    if Assigned(APArams) and (AParams.count > 0) then SetParameters(ODBCCursor, AParams);
    // execute the statement
    case ODBCCursor.FSchemaType of
      stTables    : Res:=SQLTables (ODBCCursor.FSTMTHandle, nil, 0, nil, 0, nil, 0, TABLE_TYPE_USER, length(TABLE_TYPE_USER) );
      stSysTables : Res:=SQLTables (ODBCCursor.FSTMTHandle, nil, 0, nil, 0, nil, 0, TABLE_TYPE_SYSTEM, length(TABLE_TYPE_SYSTEM) );
      stColumns   : Res:=SQLColumns(ODBCCursor.FSTMTHandle, nil, 0, nil, 0, @ODBCCursor.FQuery[1], length(ODBCCursor.FQuery), nil, 0 );
      stProcedures: Res:=SQLProcedures(ODBCCursor.FSTMTHandle, nil, 0, nil, 0, nil, 0 );
      stIndexes   : Res:=SQLStatistics(ODBCCursor.FSTMTHandle, nil, 0, nil, 0, @ODBCCursor.FQuery[1], length(ODBCCursor.FQuery), SQL_INDEX_ALL, SQL_QUICK);
      else          Res:=SQLExecute(ODBCCursor.FSTMTHandle); //SQL_NO_DATA returns searched update or delete statement that does not affect any rows
    end; {case}

    if (Res<>SQL_NO_DATA) then ODBCCheckResult( Res, SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not execute statement.' );

    if ODBCSuccess(SQLNumResultCols(ODBCCursor.FSTMTHandle, ColumnCount)) then
      ODBCCursor.FSelectable:=ColumnCount>0
    else
      ODBCCursor.FSelectable:=False;

  finally
    // free parameter buffers
    FreeParamBuffers(ODBCCursor);
  end;
end;

function TODBCConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
var
  RowCount: SQLLEN;
begin
  if assigned(cursor) then
    if ODBCSuccess( SQLRowCount((cursor as TODBCCursor).FSTMTHandle, RowCount) ) then
       Result:=RowCount
    else
       Result:=-1
  else
    Result:=-1;
end;

function TODBCConnection.RefreshLastInsertID(Query : TCustomSQLQuery; Field : TField): boolean;
var
  STMTHandle: SQLHSTMT;
  StrLenOrInd: SQLLEN;
  LastInsertID: LargeInt;
begin
  Result := false;
  if SQLAllocHandle(SQL_HANDLE_STMT, FDBCHandle, STMTHandle) = SQL_SUCCESS then
    begin
    if SQLExecDirect(STMTHandle, PChar(FDBMSInfo.GetLastInsertIDSQL), Length(FDBMSInfo.GetLastInsertIDSQL)) = SQL_SUCCESS then
      if SQLFetch(STMTHandle) = SQL_SUCCESS then
        if SQLGetData(STMTHandle, 1, SQL_C_SBIGINT, @LastInsertID, SizeOf(LargeInt), @StrLenOrInd) = SQL_SUCCESS then
          Field.AsLargeInt := LastInsertID;
    SQLFreeHandle(SQL_HANDLE_STMT, STMTHandle);
    end;
end;

procedure TODBCConnection.AddFieldDefs(cursor: TSQLCursor; FieldDefs: TFieldDefs);
const
  ColNameDefaultLength  = 40; // should be > 0, because an ansistring of length 0 is a nil pointer instead of a pointer to a #0
  TypeNameDefaultLength = 80; // idem
  BLOB_BUF_SIZE = 0;
var
  ODBCCursor:TODBCCursor;
  ColumnCount:SQLSMALLINT;
  i:integer;
  ColNameLength,TypeNameLength,DataType,DecimalDigits,Nullable:SQLSMALLINT;
  ColumnSize:SQLULEN;
  ColName,TypeName:RawByteString;
  FieldType:TFieldType;
  FieldSize:word;
  AutoIncAttr, FixedPrecScale, Unsigned, Updatable: SQLLEN;
begin
  ODBCCursor:=cursor as TODBCCursor;

  // get number of columns in result set
  ODBCCheckResult(
    SQLNumResultCols(ODBCCursor.FSTMTHandle, ColumnCount),
    SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not determine number of columns in result set.'
  );

  AutoIncAttr:=SQL_FALSE;
  for i:=1 to ColumnCount do
  begin
    SetLength(ColName,ColNameDefaultLength); // also garantuees uniqueness

    // call with default column name buffer
    ODBCCheckResult(
      SQLDescribeCol(ODBCCursor.FSTMTHandle, // statement handle
                     i,                      // column number, is 1-based (Note: column 0 is the bookmark column in ODBC)
                     @(ColName[1]),          // default buffer
                     ColNameDefaultLength+1, // and its length; we include the #0 terminating any ansistring of Length > 0 in the buffer
                     ColNameLength,          // actual column name length
                     DataType,               // the SQL datatype for the column
                     ColumnSize,             // column size (in characters)
                     DecimalDigits,          // number of decimal digits
                     Nullable),              // SQL_NO_NULLS, SQL_NULLABLE or SQL_NULLABLE_UNKNOWN
      SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get column properties for column %d.',[i]
    );

    // truncate buffer or make buffer long enough for entire column name (note: the call is the same for both cases!)
    SetLength(ColName,ColNameLength);
    // check whether entire column name was returned
    if ColNameLength>ColNameDefaultLength then
    begin
      // request column name with buffer that is long enough
      ODBCCheckResult(
        SQLColAttribute(ODBCCursor.FSTMTHandle, // statement handle
                        i,                      // column number
                        SQL_DESC_NAME,          // the column name or alias
                        @(ColName[1]),          // buffer
                        ColNameLength+1,        // buffer size
                        @ColNameLength,         // actual length
                        nil),                   // no numerical output
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get column name for column %d.',[i]
      );
    end;
    // ColName is received in ANSI - convert to DefaultSystemCodePage
    SetCodePage(ColName, CodePage, False);
    SetCodePage(ColName, DefaultSystemCodePage, True);

    // convert type
    // NOTE: I made some guesses here after I found only limited information about TFieldType; please report any problems
    case DataType of
      SQL_CHAR:          begin FieldType:=ftFixedChar;  FieldSize:=ColumnSize; end;
      SQL_VARCHAR:       begin FieldType:=ftString;     FieldSize:=ColumnSize; end;
      SQL_LONGVARCHAR:   begin FieldType:=ftMemo;       FieldSize:=BLOB_BUF_SIZE; end; // is a blob
      SQL_WCHAR:         begin FieldType:=ftFixedWideChar; FieldSize:=ColumnSize; end;
      SQL_WVARCHAR:      begin FieldType:=ftWideString; FieldSize:=ColumnSize; end;
      SQL_SS_XML,
      SQL_WLONGVARCHAR:  begin FieldType:=ftWideMemo;   FieldSize:=BLOB_BUF_SIZE; end; // is a blob
      SQL_DECIMAL:       begin FieldType:=ftFloat;      FieldSize:=0; end;
      SQL_NUMERIC:       begin FieldType:=ftFloat;      FieldSize:=0; end;
      SQL_SMALLINT:      begin FieldType:=ftSmallint;   FieldSize:=0; end;
      SQL_INTEGER:       begin FieldType:=ftInteger;    FieldSize:=0; end;
      SQL_REAL:          begin FieldType:=ftFloat;      FieldSize:=0; end;
      SQL_FLOAT:         begin FieldType:=ftFloat;      FieldSize:=0; end;
      SQL_DOUBLE:        begin FieldType:=ftFloat;      FieldSize:=0; end;
      SQL_BIT:           begin FieldType:=ftBoolean;    FieldSize:=0; end;
      SQL_TINYINT:       begin FieldType:=ftSmallint;   FieldSize:=0; end;
      SQL_BIGINT:        begin FieldType:=ftLargeint;   FieldSize:=0; end;
      SQL_BINARY:        begin FieldType:=ftBytes;      FieldSize:=ColumnSize; end;
      SQL_VARBINARY:
      begin
        FieldSize:=ColumnSize;
        if FieldSize=BLOB_BUF_SIZE then // SQL_VARBINARY declared as VARBINARY(MAX) must be ftBlob - variable data size
          FieldType:=ftBlob
        else
          FieldType:=ftVarBytes;
      end;
      SQL_LONGVARBINARY: begin FieldType:=ftBlob;       FieldSize:=BLOB_BUF_SIZE; end; // is a blob
      SQL_TYPE_DATE:     begin FieldType:=ftDate;       FieldSize:=0; end;
      SQL_SS_TIME2,
      SQL_TYPE_TIME:     begin FieldType:=ftTime;       FieldSize:=0; end;
      SQL_TYPE_TIMESTAMP:begin FieldType:=ftDateTime;   FieldSize:=0; end;
{      SQL_TYPE_UTCDATETIME:FieldType:=ftUnknown;}
{      SQL_TYPE_UTCTIME:    FieldType:=ftUnknown;}
{      SQL_INTERVAL_MONTH:           FieldType:=ftUnknown;}
{      SQL_INTERVAL_YEAR:            FieldType:=ftUnknown;}
{      SQL_INTERVAL_YEAR_TO_MONTH:   FieldType:=ftUnknown;}
{      SQL_INTERVAL_DAY:             FieldType:=ftUnknown;}
{      SQL_INTERVAL_HOUR:            FieldType:=ftUnknown;}
{      SQL_INTERVAL_MINUTE:          FieldType:=ftUnknown;}
{      SQL_INTERVAL_SECOND:          FieldType:=ftUnknown;}
{      SQL_INTERVAL_DAY_TO_HOUR:     FieldType:=ftUnknown;}
{      SQL_INTERVAL_DAY_TO_MINUTE:   FieldType:=ftUnknown;}
{      SQL_INTERVAL_DAY_TO_SECOND:   FieldType:=ftUnknown;}
{      SQL_INTERVAL_HOUR_TO_MINUTE:  FieldType:=ftUnknown;}
{      SQL_INTERVAL_HOUR_TO_SECOND:  FieldType:=ftUnknown;}
{      SQL_INTERVAL_MINUTE_TO_SECOND:FieldType:=ftUnknown;}
      SQL_GUID:          begin FieldType:=ftGuid;       FieldSize:=38; end; //SQL_GUID defines 36, but TGuidField requires 38
      SQL_SS_VARIANT:    begin FieldType:=ftVariant;    FieldSize:=0; end;
    else
      begin FieldType:=ftUnknown; FieldSize:=ColumnSize; end
    end;

    if (FieldType in [ftString,ftFixedChar]) and // field types mapped to TStringField
       (FieldSize > MaxSmallint) then
    begin
      FieldSize := MaxSmallint;
    end
    else
    // any exact numeric type with scale 0 can have identity attr.
    // only one column per table can have identity attr.
    if (FieldType in [ftInteger,ftLargeInt]) and (AutoIncAttr=SQL_FALSE) then
    begin
      ODBCCheckResult(
        SQLColAttribute(ODBCCursor.FSTMTHandle,     // statement handle
                        i,                          // column number
                        SQL_DESC_AUTO_UNIQUE_VALUE, // FieldIdentifier
                        nil,                        // buffer
                        0,                          // buffer size
                        nil,                        // actual length
                        @AutoIncAttr),              // NumericAttribute
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get autoincrement attribute for column %d.',[i]
      );
      if (AutoIncAttr=SQL_TRUE) and (FieldType=ftInteger) then
        FieldType:=ftAutoInc;
    end
    else
    if FieldType in [ftFloat] then
    begin
      FixedPrecScale:=0;
      ODBCCheckResult(
        SQLColAttribute(ODBCCursor.FSTMTHandle,
                        i,
                        SQL_DESC_FIXED_PREC_SCALE,
                        nil,
                        0,
                        nil,
                        @FixedPrecScale),
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get money attribute for column %d.',[i]
      );
      if FixedPrecScale=SQL_TRUE then
        FieldType:=ftCurrency;
    end;

    if FieldType in [ftSmallint] then
    begin
      Unsigned:=0;
      ODBCCheckResult(
        SQLColAttribute(ODBCCursor.FSTMTHandle,
                        i,
                        SQL_DESC_UNSIGNED,
                        nil,
                        0,
                        nil,
                        @Unsigned),
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get unsigned attribute for column %d.',[i]
      );
      if Unsigned=SQL_TRUE then
        case FieldType of
          ftSmallint: FieldType:=ftWord;
        end;
    end;

    Updatable:=0;
    ODBCCheckResult(
      SQLColAttribute(ODBCCursor.FSTMTHandle,
                      i,
                      SQL_DESC_UPDATABLE,
                      nil,
                      0,
                      nil,
                      @Updatable),
      SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get updatable attribute for column %d.',[i]
    );

    if FieldType=ftUnknown then // if unknown field type encountered, try finding more specific information about the ODBC SQL DataType
    begin
      SetLength(TypeName,TypeNameDefaultLength); // also garantuees uniqueness

      ODBCCheckResult(
        SQLColAttribute(ODBCCursor.FSTMTHandle,  // statement handle
                        i,                       // column number
                        SQL_DESC_TYPE_NAME,      // FieldIdentifier indicating the datasource dependent data type name (useful for diagnostics)
                        @(TypeName[1]),          // default buffer
                        TypeNameDefaultLength+1, // and its length; we include the #0 terminating any ansistring of Length > 0 in the buffer
                        @TypeNameLength,         // actual type name length
                        nil                      // no need for a pointer to return a numeric attribute at
        ),
        SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get datasource dependent type name for column %s.',[ColName]
      );
      // truncate buffer or make buffer long enough for entire column name (note: the call is the same for both cases!)
      SetLength(TypeName,TypeNameLength);
      // check whether entire column name was returned
      if TypeNameLength>TypeNameDefaultLength then
      begin
        // request column name with buffer that is long enough
        ODBCCheckResult(
          SQLColAttribute(ODBCCursor.FSTMTHandle, // statement handle
                        i,                        // column number
                        SQL_DESC_TYPE_NAME,       // FieldIdentifier indicating the datasource dependent data type name (useful for diagnostics)
                        @(TypeName[1]),           // buffer
                        TypeNameLength+1,         // buffer size
                        @TypeNameLength,          // actual length
                        nil),                     // no need for a pointer to return a numeric attribute at
          SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get datasource dependent type name for column %s.',[ColName]
        );
      end;
      // TypeName is received in ANSI - convert to DefaultSystemCodePage
      SetCodePage(TypeName, CodePage, False);
      SetCodePage(TypeName, DefaultSystemCodePage, True);

      DatabaseErrorFmt('Column %s has an unknown or unsupported column type. Datasource dependent type name: %s. ODBC SQL data type code: %d.', [ColName, TypeName, DataType]);
    end;

    // add FieldDef
    AddFieldDef(FieldDefs, i, ColName, FieldType, FieldSize, -1, False, (Nullable=SQL_NO_NULLS) and (AutoIncAttr=SQL_FALSE), Updatable=SQL_ATTR_READONLY);
  end;
end;

function TODBCConnection.Fetch(cursor: TSQLCursor): boolean;
var
  ODBCCursor:TODBCCursor;
  Res:SQLRETURN;
begin
  ODBCCursor:=cursor as TODBCCursor;

  // fetch new row
  Res:=SQLFetch(ODBCCursor.FSTMTHandle);
  if Res<>SQL_NO_DATA then
    ODBCCheckResult(Res,SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not fetch new row from result set.');

  // result is true if a new row was available
  Result:=Res<>SQL_NO_DATA;
end;

const
  DEFAULT_BLOB_BUFFER_SIZE = 1024;

function TODBCConnection.LoadField(cursor: TSQLCursor; FieldDef: TFieldDef; buffer: pointer; out CreateBlob : boolean): boolean;
const
  SQL_CA_SS_VARIANT_TYPE = 1215;
var
  ODBCCursor:TODBCCursor;
  StrLenOrInd,VariantCType:SQLLEN;
  ODBCDateStruct:SQL_DATE_STRUCT;
  ODBCTimeStruct:SQL_TIME_STRUCT;
  ODBCTimeStampStruct:SQL_TIMESTAMP_STRUCT;
  DateTime:TDateTime;
  Res:SQLRETURN;
begin
  CreateBlob := False;
  ODBCCursor:=cursor as TODBCCursor;

  // load the field using SQLGetData
  // Note: optionally we can implement the use of SQLBindCol later for even more speed
  // TODO: finish this
  case FieldDef.DataType of
    ftWideString,ftFixedWideChar: // mapped to TWideStringField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_WCHAR, buffer, FieldDef.Size*FieldDef.CharSize+sizeof(WideChar), @StrLenOrInd); //buffer must contain space for the null-termination character
    ftGuid, ftFixedChar,ftString: // are mapped to a TStringField (including TGuidField)
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_CHAR, buffer, FieldDef.Size*FieldDef.CharSize+1, @StrLenOrInd);
    ftSmallint:           // mapped to TSmallintField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_SSHORT, buffer, SizeOf(Smallint), @StrLenOrInd);
    ftInteger,ftAutoInc:  // mapped to TLongintField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_SLONG, buffer, SizeOf(Longint), @StrLenOrInd);
    ftWord:               // mapped to TWordField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_USHORT, buffer, SizeOf(Word), @StrLenOrInd);
    ftLargeint:           // mapped to TLargeintField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_SBIGINT, buffer, SizeOf(Largeint), @StrLenOrInd);
    ftFloat,ftCurrency:   // mapped to TFloatField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_DOUBLE, buffer, SizeOf(Double), @StrLenOrInd);
    ftTime:               // mapped to TTimeField
    begin
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_TYPE_TIME, @ODBCTimeStruct, SizeOf(SQL_TIME_STRUCT), @StrLenOrInd);
      if StrLenOrInd<>SQL_NULL_DATA then
      begin
        DateTime:=TimeStructToDateTime(@ODBCTimeStruct);
        Move(DateTime, buffer^, SizeOf(TDateTime));
      end;
    end;
    ftDate:               // mapped to TDateField
    begin
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_TYPE_DATE, @ODBCDateStruct, SizeOf(SQL_DATE_STRUCT), @StrLenOrInd);
      if StrLenOrInd<>SQL_NULL_DATA then
      begin
        DateTime:=DateStructToDateTime(@ODBCDateStruct);
        Move(DateTime, buffer^, SizeOf(TDateTime));
      end;
    end;
    ftDateTime:           // mapped to TDateTimeField
    begin
      // Seems like not all ODBC-drivers (mysql on Linux) set the fractional part. Initialize
      // it's value to avoid 'random' data.
      ODBCTimeStampStruct.Fraction:=0;
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_TYPE_TIMESTAMP, @ODBCTimeStampStruct, SizeOf(SQL_TIMESTAMP_STRUCT), @StrLenOrInd);
      if StrLenOrInd<>SQL_NULL_DATA then
      begin
        DateTime:=TimeStampStructToDateTime(@ODBCTimeStampStruct);
        Move(DateTime, buffer^, SizeOf(TDateTime));
      end;
    end;
    ftBoolean:            // mapped to TBooleanField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BIT, buffer, SizeOf(Wordbool), @StrLenOrInd);
    ftBytes:              // mapped to TBytesField
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, buffer, FieldDef.Size, @StrLenOrInd);
    ftVarBytes:           // mapped to TVarBytesField
    begin
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, buffer+SizeOf(Word), FieldDef.Size, @StrLenOrInd);
      if StrLenOrInd < 0 then
        PWord(buffer)^ := 0
      else
        PWord(buffer)^ := StrLenOrInd;
    end;
    ftVariant:
    begin
      // Try to read sql_variant header
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, buffer, 0, @StrLenOrInd);
      if ODBCSuccess(Res) then
      begin
        Res := SQLColAttribute(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_CA_SS_VARIANT_TYPE, nil, 0, nil, @VariantCType);
        // map only types, which holds values directly in TVarData record
        case VariantCType of
          SQL_C_SSHORT, SQL_C_USHORT, SQL_C_SLONG, SQL_C_ULONG, SQL_C_SBIGINT:
            begin
            VariantCType := SQL_C_SBIGINT;
            PVarData(buffer)^.vtype := varInt64;
            buffer := @PVarData(buffer)^.vint64;
            end;
          SQL_C_FLOAT, SQL_C_DOUBLE:
            begin
            VariantCType := SQL_C_DOUBLE;
            PVarData(buffer)^.vtype := varDouble;
            buffer := @PVarData(buffer)^.vdouble;
            end
          else
            StrLenOrInd := SQL_NULL_DATA;
        end;
        if StrLenOrInd<>SQL_NULL_DATA then
          Res := SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, VariantCType, buffer, 8, @StrLenOrInd);
      end;
    end;
    ftWideMemo,
    ftBlob, ftMemo:       // BLOBs
    begin
      //Writeln('BLOB');
      // Try to discover BLOB data length
      Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, buffer, 0, @StrLenOrInd);
      // Read the data if not NULL
      if ODBCSuccess(Res) and (StrLenOrInd<>SQL_NULL_DATA) then
      begin
        CreateBlob:=true; // defer actual loading of blob data to LoadBlobIntoBuffer method
        //WriteLn('Deferring loading of blob of length ',StrLenOrInd);
      end;
    end;
    // TODO: Loading of other field types
  else
    raise EODBCException.CreateFmt('Tried to load field of unsupported field type %s',[Fieldtypenames[FieldDef.DataType]]);
  end;
  ODBCCheckResult(Res, SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get field data for field "%s" (index %d).',[FieldDef.Name, FieldDef.Index+1]);
  Result:=StrLenOrInd<>SQL_NULL_DATA; // Result indicates whether the value is non-null

  //writeln(Format('Field.Size: %d; StrLenOrInd: %d',[FieldDef.Size, StrLenOrInd]));
end;

procedure TODBCConnection.LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction);
var
  ODBCCursor: TODBCCursor;
  Res: SQLRETURN;
  StrLenOrInd:SQLLEN;
  BlobBuffer:pointer;
  BlobBufferSize,BytesRead:SQLINTEGER;
  BlobMemoryStream:TMemoryStream;
begin
  ODBCCursor:=cursor as TODBCCursor;
  // Try to discover BLOB data length
  //   NB MS ODBC requires that TargetValuePtr is not nil, so we supply it with a valid pointer, even though BufferLength is 0
  StrLenOrInd:=0;
  Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, @BlobBuffer, 0, @StrLenOrInd);
  if Res<>SQL_NO_DATA then
    ODBCCheckResult(Res, SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not get field data for field "%s" (index %d).',[FieldDef.Name, FieldDef.Index+1]);
  // Read the data if not NULL
  if StrLenOrInd<>SQL_NULL_DATA then
  begin
    // Determine size of buffer to use
    if StrLenOrInd<>SQL_NO_TOTAL then begin
      // Size is known on beforehand
      // set size & alloc buffer
      //WriteLn('Loading blob of length ',StrLenOrInd);
      BlobBufferSize:=StrLenOrInd;
      ABlobBuf^.BlobBuffer^.Size:=BlobBufferSize;
      ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer, BlobBufferSize);
      // get blob data
      if BlobBufferSize>0 then begin
        Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, ABlobBuf^.BlobBuffer^.Buffer, BlobBufferSize, @StrLenOrInd);
        ODBCCheckResult(Res, SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not load blob data for field "%s" (index %d).',[FieldDef.Name, FieldDef.Index+1]);
      end;
    end else begin
      // Size is not known on beforehand; read data in chuncks; write to a TMemoryStream (which implements O(n) writing)
      BlobBufferSize:=DEFAULT_BLOB_BUFFER_SIZE;
      // init BlobBuffer and BlobMemoryStream to nil pointers
      BlobBuffer:=nil; // the buffer that will hold the chuncks of data; not to be confused with ABlobBuf^.BlobBuffer
      BlobMemoryStream:=nil;
      try
        // Allocate the buffer and memorystream
        BlobBuffer:=GetMem(BlobBufferSize);
        BlobMemoryStream:=TMemoryStream.Create;
        // Retrieve data in parts
        repeat
          Res:=SQLGetData(ODBCCursor.FSTMTHandle, FieldDef.Index+1, SQL_C_BINARY, BlobBuffer, BlobBufferSize, @StrLenOrInd);
          ODBCCheckResult(Res, SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not load (partial) blob data for field "%s" (index %d).',[FieldDef.Name, FieldDef.Index+1]);
          // Append data in buffer to memorystream
          if (StrLenOrInd=SQL_NO_TOTAL) or (StrLenOrInd>BlobBufferSize) then
            BytesRead:=BlobBufferSize
          else
            BytesRead:=StrLenOrInd;
          BlobMemoryStream.Write(BlobBuffer^, BytesRead);
        until Res=SQL_SUCCESS;
        // Copy memory stream data to ABlobBuf^.BlobBuffer
        BlobBufferSize:=BlobMemoryStream.Size; // actual blob size
        //   alloc ABlobBuf^.BlobBuffer
        ABlobBuf^.BlobBuffer^.Size:=BlobBufferSize;
        ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer, BlobBufferSize);
        //   read memory stream data into ABlobBuf^.BlobBuffer
        BlobMemoryStream.Position:=0;
        BlobMemoryStream.Read(ABlobBuf^.BlobBuffer^.Buffer^, BlobBufferSize);
      finally
        // free buffer and memory stream
        BlobMemoryStream.Free;
        if BlobBuffer<>nil then
          Freemem(BlobBuffer,BlobBufferSize);
      end;
    end;
  end;
end;

procedure TODBCConnection.FreeFldBuffers(cursor: TSQLCursor);
var
  ODBCCursor:TODBCCursor;
begin
  ODBCCursor:=cursor as TODBCCursor;

  if ODBCCursor.FSTMTHandle <> SQL_NULL_HSTMT then
    ODBCCheckResult(
      SQLFreeStmt(ODBCCursor.FSTMTHandle, SQL_CLOSE),
      SQL_HANDLE_STMT, ODBCCursor.FSTMTHandle, 'Could not close ODBC statement cursor.'
    );
end;

procedure TODBCConnection.UpdateIndexDefs(IndexDefs: TIndexDefs; TableName: string);
var
  Len: integer;
  StmtHandle:SQLHSTMT;
  Res:SQLRETURN;
  IndexDef: TIndexDef;
  KeyName: String;
  // variables for binding
  NonUnique :SQLSMALLINT; NonUniqueIndOrLen :SQLLEN;
  IndexName :string;      IndexNameIndOrLen :SQLLEN;
  _Type     :SQLSMALLINT; _TypeIndOrLen     :SQLLEN;
  OrdinalPos:SQLSMALLINT; OrdinalPosIndOrLen:SQLLEN;
  ColName   :string;      ColNameIndOrLen   :SQLLEN;
  AscOrDesc :char;        AscOrDescIndOrLen :SQLLEN;
  PKName    :string;      PKNameIndOrLen    :SQLLEN;
const
  DEFAULT_NAME_LEN = 255;
begin
  Len := length(TableName);
  if Len > 2 then
    if (TableName[1] in ['"','`']) and (TableName[Len]  in ['"','`']) then
      TableName := AnsiDequotedStr(TableName, TableName[1])
    else if (TableName[1] in ['[']) and (TableName[Len] in [']']) then
      TableName := copy(TableName, 2, Len-2);

  // allocate statement handle
  StmtHandle := SQL_NULL_HANDLE;
  ODBCCheckResult(
    SQLAllocHandle(SQL_HANDLE_STMT, FDBCHandle, StmtHandle),
    SQL_HANDLE_DBC, FDBCHandle, 'Could not allocate ODBC Statement handle.'
  );

  try
    // Disabled: only works if we can specify a SchemaName and, if supported by the data source, a CatalogName
    //           otherwise SQLPrimaryKeys returns error HY0009 (Invalid use of null pointer)
    // set the SQL_ATTR_METADATA_ID so parameters to Catalog functions are considered as identifiers (e.g. case-insensitive)
    //ODBCCheckResult(
    //  SQLSetStmtAttr(StmtHandle, SQL_ATTR_METADATA_ID, SQLPOINTER(SQL_TRUE), SQL_IS_UINTEGER),
    //  SQL_HANDLE_STMT, StmtHandle, 'Could not set SQL_ATTR_METADATA_ID statement attribute to SQL_TRUE.'
    //);

    // alloc result column buffers
    SetLength(ColName,  DEFAULT_NAME_LEN);
    SetLength(PKName,   DEFAULT_NAME_LEN);
    SetLength(IndexName,DEFAULT_NAME_LEN);

    // Fetch primary key info using SQLPrimaryKeys
    ODBCCheckResult(
      SQLPrimaryKeys(
        StmtHandle,
        nil, 0, // any catalog
        nil, 0, // any schema
        PChar(TableName), Length(TableName)
      ),
      SQL_HANDLE_STMT, StmtHandle, 'Could not retrieve primary key metadata for table %s using SQLPrimaryKeys.', [TableName]
    );

    // init key name & fields; we will set the IndexDefs.Option ixPrimary below when there is a match by IndexName=KeyName
    KeyName:='';
    try
      // bind result columns; the column numbers are documented in the reference for SQLStatistics
      ODBCCheckResult(SQLBindCol(StmtHandle,  4, SQL_C_CHAR  , @ColName[1], Length(ColName)+1, @ColNameIndOrLen), SQL_HANDLE_STMT, StmtHandle, 'Could not bind primary key metadata column COLUMN_NAME.');
      ODBCCheckResult(SQLBindCol(StmtHandle,  5, SQL_C_SSHORT, @OrdinalPos, 0, @OrdinalPosIndOrLen), SQL_HANDLE_STMT, StmtHandle, 'Could not bind primary key metadata column KEY_SEQ.');
      ODBCCheckResult(SQLBindCol(StmtHandle,  6, SQL_C_CHAR  , @PKName [1], Length(PKName )+1, @PKNameIndOrLen ), SQL_HANDLE_STMT, StmtHandle, 'Could not bind primary key metadata column PK_NAME.');

      // fetch result
      repeat
        // go to next row; loads data in bound columns
        Res:=SQLFetch(StmtHandle);
        // if no more row, break
        if Res=SQL_NO_DATA then
          Break;
        // handle data
        if ODBCSuccess(Res) then begin
          if OrdinalPos=1 then begin
            // create new IndexDef if OrdinalPos=1
            IndexDef:=IndexDefs.AddIndexDef;
            IndexDef.Name:=PChar(@PKName[1]);
            IndexDef.Fields:=PChar(@ColName[1]);
            IndexDef.Options:=IndexDef.Options+[ixUnique]+[ixPrimary]; // Primary key is always unique
            KeyName:=IndexDef.Name;
          end else begin
            assert(Assigned(IndexDef));
            IndexDef.Fields:=IndexDef.Fields+';'+PChar(@ColName[1]); // NB ; is the separator to be used for IndexDef.Fields
          end;
        end else begin
          ODBCCheckResult(Res, SQL_HANDLE_STMT, StmtHandle, 'Could not fetch primary key metadata row.');
        end;
      until false;
    finally
      // unbind columns & close cursor
      ODBCCheckResult(SQLFreeStmt(StmtHandle, SQL_UNBIND), SQL_HANDLE_STMT, StmtHandle, 'Could not unbind columns.');
      ODBCCheckResult(SQLFreeStmt(StmtHandle, SQL_CLOSE),  SQL_HANDLE_STMT, StmtHandle, 'Could not close cursor.');
    end;

    //WriteLn('KeyName: ',KeyName,'; KeyFields: ',KeyFields);

    // use SQLStatistics to get index information
    ODBCCheckResult(
      SQLStatistics(
        StmtHandle,
        nil, 0, // catalog unknown; request for all catalogs
        nil, 0, // schema unknown; request for all schemas
        PChar(TableName), Length(TableName), // request information for TableName
        SQL_INDEX_ALL,
        SQL_QUICK
      ),
      SQL_HANDLE_STMT, StmtHandle, 'Could not retrieve index metadata for table %s using SQLStatistics.', [TableName]
    );

    try
      // bind result columns; the column numbers are documented in the reference for SQLStatistics
      ODBCCheckResult(SQLBindCol(StmtHandle,  4, SQL_C_SSHORT, @NonUnique , 0, @NonUniqueIndOrLen ), SQL_HANDLE_STMT, StmtHandle, 'Could not bind index metadata column NON_UNIQUE.');
      ODBCCheckResult(SQLBindCol(StmtHandle,  6, SQL_C_CHAR  , @IndexName[1], Length(IndexName)+1, @IndexNameIndOrLen), SQL_HANDLE_STMT, StmtHandle, 'Could not bind index metadata column INDEX_NAME.');
      ODBCCheckResult(SQLBindCol(StmtHandle,  7, SQL_C_SSHORT, @_Type     , 0, @_TypeIndOrLen     ), SQL_HANDLE_STMT, StmtHandle, 'Could not bind index metadata column TYPE.');
      ODBCCheckResult(SQLBindCol(StmtHandle,  8, SQL_C_SSHORT, @OrdinalPos, 0, @OrdinalPosIndOrLen), SQL_HANDLE_STMT, StmtHandle, 'Could not bind index metadata column ORDINAL_POSITION.');
      ODBCCheckResult(SQLBindCol(StmtHandle,  9, SQL_C_CHAR  , @ColName  [1], Length(ColName  )+1, @ColNameIndOrLen  ), SQL_HANDLE_STMT, StmtHandle, 'Could not bind index metadata column COLUMN_NAME.');
      ODBCCheckResult(SQLBindCol(StmtHandle, 10, SQL_C_CHAR  , @AscOrDesc , 1, @AscOrDescIndOrLen ), SQL_HANDLE_STMT, StmtHandle, 'Could not bind index metadata column ASC_OR_DESC.');

      // clear index defs
      IndexDef:=nil;

      // fetch result
      repeat
        // go to next row; loads data in bound columns
        Res:=SQLFetch(StmtHandle);
        // if no more row, break
        if Res=SQL_NO_DATA then
          Break;
        // handle data
        if ODBCSuccess(Res) then begin
          // note: SQLStatistics not only returns index info, but also statistics; we skip the latter
          if _Type<>SQL_TABLE_STAT then begin
            if PChar(@IndexName[1])=KeyName then begin
              // The indexdef is already made as the primary key
              // Only if the index is descending is not known yet.
              if (AscOrDescIndOrLen<>SQL_NULL_DATA) and (AscOrDesc='D') then begin
                IndexDef:=IndexDefs.Find(KeyName);
                IndexDef.Options:=IndexDef.Options+[ixDescending];
              end;
            end else if (OrdinalPos=1) or not Assigned(IndexDef) then begin
              // create new IndexDef if OrdinalPos=1 or not Assigned(IndexDef) (the latter should not occur though)
              IndexDef:=IndexDefs.AddIndexDef;
              IndexDef.Name:=PChar(@IndexName[1]); // treat ansistring as zero terminated string
              IndexDef.Fields:=PChar(@ColName[1]);
              if NonUnique=SQL_FALSE then
                IndexDef.Options:=IndexDef.Options+[ixUnique];
              if (AscOrDescIndOrLen<>SQL_NULL_DATA) and (AscOrDesc='D') then
                IndexDef.Options:=IndexDef.Options+[ixDescending];
              // TODO: figure out how we can tell whether COLUMN_NAME is an expression or not
              //       if it is an expression, we should include ixExpression in Options and set Expression to ColName
            end else // NB we re-use the last IndexDef
              IndexDef.Fields:=IndexDef.Fields+';'+PChar(@ColName[1]); // NB ; is the separator to be used for IndexDef.Fields
          end;
        end else begin
          ODBCCheckResult(Res, SQL_HANDLE_STMT, StmtHandle, 'Could not fetch index metadata row.');
        end;
      until false;
    finally
      // unbind columns & close cursor
      ODBCCheckResult(SQLFreeStmt(StmtHandle, SQL_UNBIND), SQL_HANDLE_STMT, StmtHandle, 'Could not unbind columns.');
      ODBCCheckResult(SQLFreeStmt(StmtHandle, SQL_CLOSE),  SQL_HANDLE_STMT, StmtHandle, 'Could not close cursor.');
    end;

  finally
    if StmtHandle<>SQL_NULL_HANDLE then begin
      // Free the statement handle
      Res:=SQLFreeHandle(SQL_HANDLE_STMT, StmtHandle);
      if Res=SQL_ERROR then
        ODBCCheckResult(Res, SQL_HANDLE_STMT, STMTHandle, 'Could not free ODBC Statement handle.');
    end;
  end;
end;

function TODBCConnection.GetSchemaInfoSQL(SchemaType: TSchemaType; SchemaObjectName, SchemaObjectPattern: string): string;
begin
  if SchemaType in [stTables, stSysTables, stColumns, stProcedures, stIndexes] then
  begin
    if SchemaObjectName<>'' then
      Result := SchemaObjectName
    else
      Result := ' ';
  end
  else
    Result := inherited;
end;

function TODBCConnection.GetConnectionInfo(InfoType: TConnInfoType): string;
var i,l: SQLSMALLINT;
    b: array[0..41] of AnsiChar;
begin
  case InfoType of
    citServerType:
      i:=SQL_DBMS_NAME;
    citServerVersion,
    citServerVersionString:
      i:=SQL_DBMS_VER;
    citClientName:
      i:=SQL_DRIVER_NAME;
    citClientVersion:
      i:=SQL_DRIVER_VER;
  else
    Result:=inherited GetConnectionInfo(InfoType);
    Exit;
  end;

  if Connected and (SQLGetInfo(FDBCHandle, i, @b, sizeof(b), @l) = SQL_SUCCESS) then
    SetString(Result, @b, l)
  else
    Result:='';
end;


{ TODBCEnvironment }

constructor TODBCEnvironment.Create;
begin
  // make sure odbc is loaded
  if ODBCLoadCount=0 then InitialiseOdbc;
  Inc(ODBCLoadCount);

  // allocate environment handle
  if SQLAllocHandle(SQL_HANDLE_ENV, SQL_NULL_HANDLE, FENVHandle)=SQL_Error then
    raise EODBCException.Create('Could not allocate ODBC Environment handle'); // we can't retrieve any more information, because we don't have a handle for the SQLGetDiag* functions

  // set odbc version
  ODBCCheckResult(
    SQLSetEnvAttr(FENVHandle, SQL_ATTR_ODBC_VERSION, SQLPOINTER(SQL_OV_ODBC3), 0),
    SQL_HANDLE_ENV, FENVHandle,'Could not set ODBC version to 3.'
  );
end;

destructor TODBCEnvironment.Destroy;
var
  Res:SQLRETURN;
begin
  // free environment handle
  if assigned(FENVHandle) then
    begin
    Res:=SQLFreeHandle(SQL_HANDLE_ENV, FENVHandle);
    if Res=SQL_ERROR then
      ODBCCheckResult(Res,SQL_HANDLE_ENV, FENVHandle, 'Could not free ODBC Environment handle.');
    end;

  // free odbc if not used by any TODBCEnvironment object anymore
  if ODBCLoadCount>0 then
    begin
    Dec(ODBCLoadCount);
    if ODBCLoadCount=0 then ReleaseOdbc;
    end;
end;

{ TODBCCursor }

constructor TODBCCursor.Create(Connection:TODBCConnection);
begin
end;

destructor TODBCCursor.Destroy;
begin
  inherited Destroy;
end;

class function TODBCConnectionDef.TypeName: String;
begin
  Result:='ODBC';
end;

class function TODBCConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  Result:=TODBCConnection;
end;

class function TODBCConnectionDef.Description: String;
begin
  Result:='Connect to any database via an ODBC driver';
end;

initialization
  RegisterConnection(TODBCConnectionDef);

finalization
  UnRegisterConnection(TODBCConnectionDef);
  if Assigned(DefaultEnvironment) then
    DefaultEnvironment.Free;
end.

