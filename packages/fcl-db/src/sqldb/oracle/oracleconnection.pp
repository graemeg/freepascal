unit oracleconnection;

{
    Copyright (c) 2006-2019 by Joost van der Sluis, FPC contributors

    Oracle RDBMS connector using the OCI protocol

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

 **********************************************************************}

{$mode objfpc}{$H+}

{$Define LinkDynamically}

interface

uses
  Classes, SysUtils, db, dbconst, sqldb, bufdataset,
{$IfDef LinkDynamically}
  ocidyn,
{$ELSE}
  oci,
{$ENDIF}
  oratypes;

const
  DefaultTimeOut = 60;

type
  EOraDatabaseError = class(ESQLDatabaseError)
    public
      property ORAErrorCode: integer read ErrorCode; deprecated 'Please use ErrorCode instead of ORAErrorCode'; // June 2014
  end;

  TOracleTrans = Class(TSQLHandle)
  protected
    FOciSvcCtx  : POCISvcCtx;
    FOciTrans   : POCITrans;
    FOciFlags   : ub4;
  public
    destructor Destroy(); override;
  end;
  
  TOraFieldBuf = record
    DescType : ub4;      // descriptor type
    Buffer   : pointer;
    Ind      : sb2;      // indicator
    Len      : ub4;
    Size     : ub4;
  end;

  TOracleCursor = Class(TSQLCursor)
    protected
      FOciStmt     : POCIStmt;
      FieldBuffers : array of TOraFieldBuf;
      ParamBuffers : array of TOraFieldBuf;
    end;

  { TOracleConnection }

  TOracleConnection = class (TSQLConnection)
  private
    FOciEnvironment : POciEnv;
    FOciError       : POCIError;
    FOciServer      : POCIServer;
    FOciUserSession : POCISession;
    FUserMem        : pointer;
    procedure HandleError;
    procedure GetParameters(cursor : TSQLCursor; ATransaction : TSQLTransaction; AParams : TParams);
    procedure SetParameters(cursor : TSQLCursor; ATransaction : TSQLTransaction; AParams : TParams);
  protected
    // - Connect/disconnect
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    // - Handle (de)allocation
    function AllocateCursorHandle:TSQLCursor; override;
    procedure DeAllocateCursorHandle(var cursor:TSQLCursor); override;
    function AllocateTransactionHandle:TSQLHandle; override;
    // - Statement handling
    procedure PrepareStatement(cursor:TSQLCursor; ATransaction:TSQLTransaction; buf:string; AParams:TParams); override;
    procedure UnPrepareStatement(cursor:TSQLCursor); override;
    // - Transaction handling
    procedure InternalStartDBTransaction(trans:TOracleTrans);
    function GetTransactionHandle(trans:TSQLHandle):pointer; override;
    function StartDBTransaction(trans:TSQLHandle; AParams:string):boolean; override;
    function Commit(trans:TSQLHandle):boolean; override;
    function Rollback(trans:TSQLHandle):boolean; override;
    procedure CommitRetaining(trans:TSQLHandle); override;
    procedure RollbackRetaining(trans:TSQLHandle); override;
    // - Statement execution
    procedure Execute(cursor:TSQLCursor; ATransaction:TSQLTransaction; AParams:TParams); override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    // - Result retrieval
    procedure AddFieldDefs(cursor:TSQLCursor; FieldDefs:TFieldDefs); override;
    function Fetch(cursor:TSQLCursor):boolean; override;
    function LoadField(cursor:TSQLCursor; FieldDef:TFieldDef; buffer:pointer; out CreateBlob : boolean):boolean; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction); override;
    procedure FreeFldBuffers(cursor:TSQLCursor); override;
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;

  public
    constructor Create(AOwner : TComponent); override;
  end;

  { TOracleConnectionDef }

  TOracleConnectionDef = Class(TConnectionDef)
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
  math, StrUtils, FmtBCD;

const
  ObjectQuote='"'; //beginning and ending quote for objects such as table names. Note: can be different from quotes around field names

ResourceString
  SErrEnvCreateFailed = 'The creation of an Oracle environment failed.';
  SErrHandleAllocFailed = 'The allocation of the error handle failed.';
  SErrOracle = 'Oracle returned error %s:';

type
  TODateTime = record
    year  : sb2;
    month : ub1;
    day   : ub1;
    hour  : ub1;
    min   : ub1;
    sec   : ub1;
    fsec  : ub4;
  end;

// Callback functions

function cbf_no_data(ictxp:Pdvoid; bindp:POCIBind; iter:ub4; index:ub4; bufpp:PPdvoid;
             alenp:Pub4; piecep:Pub1; indp:PPdvoid):sb4;cdecl;

begin
  bufpp^ := nil;
  alenp^ := 0;
  indp^ := nil;
  piecep^ := OCI_ONE_PIECE;
  result:=OCI_CONTINUE;
end;


function cbf_get_data(octxp:Pdvoid; bindp:POCIBind; iter:ub4; index:ub4; bufpp:PPdvoid;
             alenp:PPub4; piecep:Pub1; indp:PPdvoid; rcodep:PPub2):sb4;cdecl;

begin
// Only 1 row can be stored. No support for multiple rows: only the last row is kept.
  bufpp^:=TOraFieldBuf(octxp^).Buffer;
  indp^ := @TOraFieldBuf(octxp^).Ind;
  TOraFieldBuf(octxp^).Len:=TOraFieldBuf(octxp^).Size;   //reset size to full buffer
  alenp^ := @TOraFieldBuf(octxp^).Len;
  rcodep^:=nil;
  piecep^ := OCI_ONE_PIECE;
  result:=OCI_CONTINUE;
end;

// Conversions

Procedure FmtBCD2Nvu(bcd:tBCD;b:pByte);
var
  i,j,cnt   : integer;
  nibbles   : array [0..maxfmtbcdfractionsize-1] of byte;
  exp       : shortint;
  bb        : byte;
begin
  fillchar(b[0],22,#0);
  if BCDPrecision(bcd)=0 then // zero, special case
    begin
    b[0]:=1;
    b[1]:=$80;
    end
  else
    begin
    if (BCDPrecision(bcd)-BCDScale(bcd)) mod 2 <>0 then // odd number before decimal point
      begin
      nibbles[0]:=0;
      j:=1;
      end
    else
      j:=0;
    for i:=0 to bcd.Precision -1 do
      if i mod 2 =0 then
        nibbles[i+j]:=bcd.Fraction[i div 2] shr 4
      else
        nibbles[i+j]:=bcd.Fraction[i div 2] and $0f;
    nibbles[bcd.Precision+j]:=0; // make sure last nibble is also 0 in case we have odd scale
    exp:=(BCDPrecision(bcd)-BCDScale(bcd)+1) div 2;
    cnt:=exp+(BCDScale(bcd)+1) div 2;
    // to avoid "ora 01438: value larger than specified precision allowed for this column"
    // remove trailing zeros (scale < 0)...
    while (nibbles[cnt*2-2]*10+nibbles[cnt*2-1])=0 do
      cnt:=cnt-1;
    // ... and remove leading zeros (scale > precision)
    j:=0;
    while (nibbles[j*2]*10+nibbles[j*2+1])=0 do
      begin
      j:=j+1;
      exp:=exp-1;
      end;
    if IsBCDNegative(bcd) then
      begin
      b[0]:=cnt-j+1;
      b[1]:=not(exp+64) and $7f ;
      for i:=j to cnt-1 do
        begin
        bb:=nibbles[i*2]*10+nibbles[i*2+1];
        b[2+i-j]:=101-bb;
        end;
      if 2+cnt-j<22 then  // add a 102 at the end of the number if place left.
        begin
        b[0]:=b[0]+1;
        b[2+cnt-j]:=102;
        end;
      end
    else
      begin
      b[0]:=cnt-j+1;
      b[1]:=(exp+64) or $80 ;
      for i:=j to cnt-1 do
        begin
        bb:=nibbles[i*2]*10+nibbles[i*2+1];
        b[2+i-j]:=1+bb;
        end;
      end;
    end;
end;

function Nvu2FmtBCD(b:pbyte):tBCD;
var
  i,j       : integer;
  bb,size   : byte;
  exp       : shortint;
  nibbles   : array [0..maxfmtbcdfractionsize-1] of byte;
  scale     : integer;
begin
  size := b[0];
  if (size=1) and (b[1]=$80) then // special representation for 0
    result:=IntegerToBCD(0)
  else
    begin
    result.SignSpecialPlaces:=0; //sign positive, non blank, scale 0
    result.Precision:=1;         //BCDNegate works only if Precision <>0
    if (b[1] and $80)=$80 then // then the number is positive
      begin
      exp := (b[1] and $7f)-65;
      for i := 0 to size-2 do
        begin
        bb := b[i+2]-1;
        nibbles[i*2]:=bb div 10;
        nibbles[i*2+1]:=(bb mod 10);
        end;
      end
    else
      begin
      BCDNegate(result);
      exp := (not(b[1]) and $7f)-65;
      if b[size]=102 then  // last byte doesn't count if = 102
        size:=size-1;
      for i := 0 to size-2 do
        begin
        bb := 101-b[i+2];
        nibbles[i*2]:=bb div 10;
        nibbles[i*2+1]:=(bb mod 10);
        end;
      end;
    nibbles[(size-1)*2]:=0;
    result.Precision:=(size-1)*2;
    scale:=result.Precision-(exp*2+2);
    if scale>=0 then
      begin
      if (scale>result.Precision) then  // need to add leading 0s
        begin
        for i:=0 to (scale-result.Precision+1) div 2 do
          result.Fraction[i]:=0;
        i:=scale-result.Precision;
        result.Precision:=scale;
        end
      else
        i:=0;
      j:=i;
      if (i=0) and (nibbles[0]=0) then // get rid of leading zero received from oci
        begin
        result.Precision:=result.Precision-1;
        j:=-1;
        end;
      while i<=result.Precision do // copy nibbles
        begin
        if i mod 2 =0 then
          result.Fraction[i div 2]:=nibbles[i-j] shl 4
        else
          result.Fraction[i div 2]:=result.Fraction[i div 2] or nibbles[i-j];
        i:=i+1;
        end;
      result.SignSpecialPlaces:=result.SignSpecialPlaces or scale;
      end
    else
      begin // add trailing zeroes, increase precision to take them into account
      i:=0;
      while i<=result.Precision do // copy nibbles
        begin
        if i mod 2 =0 then
          result.Fraction[i div 2]:=nibbles[i] shl 4
        else
          result.Fraction[i div 2]:=result.Fraction[i div 2] or nibbles[i];
        i:=i+1;
        end;
      result.Precision:=result.Precision-scale;
      for i := size -1 to High(result.Fraction) do
        result.Fraction[i] := 0;
      end;
    end;
end;



// TOracleConnection

procedure TOracleConnection.HandleError;

var
    errcode : sb4;
    buf     : array[0..1023] of char;

begin
  OCIErrorGet(FOciError,1,nil,errcode,@buf[0],1024,OCI_HTYPE_ERROR);

  raise EOraDatabaseError.CreateFmt(pchar(buf), [], Self, errcode, '')
end;

procedure TOracleConnection.GetParameters(cursor: TSQLCursor; ATransaction : TSQLTransaction; AParams: TParams);
var
    i    : integer;
    odt  : TODateTime;
    s    : string;

begin
  with cursor as TOracleCursor do for i := 0 to High(ParamBuffers) do
    with AParams[i] do
      if ParamType=ptOutput then
      begin
      if ParamBuffers[i].ind = -1 then
        Value:=null;

      case DataType of
        ftInteger         : AsInteger := PInteger(ParamBuffers[i].buffer)^;
        ftLargeint        : AsLargeInt := PInt64(ParamBuffers[i].buffer)^;
        ftFloat           : AsFloat := PDouble(ParamBuffers[i].buffer)^;
        ftString          : begin
                            SetLength(s,ParamBuffers[i].Len);
                            move(ParamBuffers[i].buffer^,s[1],length(s)+1);
                            AsString:=s;
                            end;
        ftDate, ftDateTime: begin
                            OCIDateTimeGetDate(FOciUserSession, FOciError, ParamBuffers[i].buffer, @odt.year, @odt.month, @odt.day);
                            OCIDateTimeGetTime(FOciUserSession, FOciError, ParamBuffers[i].buffer, @odt.hour, @odt.min, @odt.sec, @odt.fsec);
                            AsDateTime := ComposeDateTime(EncodeDate(odt.year,odt.month,odt.day), EncodeTime(odt.hour,odt.min,odt.sec,odt.fsec div 1000000));
                            end;
        ftFMTBcd          : begin
                            AsFMTBCD:=Nvu2FmtBCD(ParamBuffers[i].buffer);
                            end;
        end;

      end;

end;

procedure TOracleConnection.DoInternalConnect;
var
  ConnectString : string;
  TempServiceContext : POCISvcCtx;
  IsConnected : boolean;
  CharSetId: ub2;
begin
{$IfDef LinkDynamically}
  InitialiseOCI;
{$EndIf}

  inherited DoInternalConnect;
  //ToDo: get rid of FUserMem, as it isn't used
  FUserMem := nil;
  IsConnected := false;

  try
    case GetConnectionCharSet of
      'utf8': CharSetId := 873;
      else    CharSetId := 0; // if it is 0, the NLS_LANG and NLS_NCHAR environment variables are used
    end;
    // Create environment handle
    if OCIEnvNlsCreate(FOciEnvironment,OCI_DEFAULT,nil,nil,nil,nil,0,FUserMem,CharSetId,CharSetId) <> OCI_SUCCESS then
      DatabaseError(SErrEnvCreateFailed,self);
    // Create error handle
    if OciHandleAlloc(FOciEnvironment,FOciError,OCI_HTYPE_ERROR,0,FUserMem) <> OCI_SUCCESS then
      DatabaseError(SErrHandleAllocFailed,self);
    // Create server handle
    if OciHandleAlloc(FOciEnvironment,FOciServer,OCI_HTYPE_SERVER,0,FUserMem) <> OCI_SUCCESS then
      DatabaseError(SErrHandleAllocFailed,self);

    // Initialize server handle
    if HostName='' then
      ConnectString := DatabaseName
    else
      ConnectString := '//'+HostName+'/'+DatabaseName;
    if OCIServerAttach(FOciServer,FOciError,@(ConnectString[1]),Length(ConnectString),OCI_DEFAULT) <> OCI_SUCCESS then
      HandleError();

    try
      // Create temporary service-context handle for user authentication
      if OciHandleAlloc(FOciEnvironment,TempServiceContext,OCI_HTYPE_SVCCTX,0,FUserMem) <> OCI_SUCCESS then
        DatabaseError(SErrHandleAllocFailed,self);

      try
        // Create user-session handle
        if OciHandleAlloc(FOciEnvironment,FOciUserSession,OCI_HTYPE_SESSION,0,FUserMem) <> OCI_SUCCESS then
          DatabaseError(SErrHandleAllocFailed,self);
        try
          // Set the server-handle in the service-context handle
          if OCIAttrSet(TempServiceContext,OCI_HTYPE_SVCCTX,FOciServer,0,OCI_ATTR_SERVER,FOciError) <> OCI_SUCCESS then
            HandleError();
          // Set username and password in the user-session handle
          if OCIAttrSet(FOciUserSession,OCI_HTYPE_SESSION,@(Self.UserName[1]),Length(Self.UserName),OCI_ATTR_USERNAME,FOciError) <> OCI_SUCCESS then
            HandleError();
          if OCIAttrSet(FOciUserSession,OCI_HTYPE_SESSION,@(Self.Password[1]),Length(Self.Password),OCI_ATTR_PASSWORD,FOciError) <> OCI_SUCCESS then
            HandleError();
          // Authenticate
          if OCISessionBegin(TempServiceContext,FOciError,FOcIUserSession,OCI_CRED_RDBMS,OCI_DEFAULT) <> OCI_SUCCESS then
            HandleError();
          IsConnected := true;
        finally
          if not IsConnected then
          begin
            OCIHandleFree(FOciUserSession,OCI_HTYPE_SESSION);
            FOciUserSession := nil;
          end;
        end;
      finally
        // Free temporary service-context handle
        OCIHandleFree(TempServiceContext,OCI_HTYPE_SVCCTX);
      end;
    finally
      if not IsConnected then
        OCIServerDetach(FOciServer,FOciError,OCI_DEFAULT);
    end;
  finally
    if not IsConnected then
    begin
      if assigned(FOciServer) then
        OCIHandleFree(FOciServer,OCI_HTYPE_SERVER);
      if assigned(FOciError) then
        OCIHandleFree(FOciError,OCI_HTYPE_ERROR);
      if assigned(FOciEnvironment) then
        OCIHandleFree(FOciEnvironment,OCI_HTYPE_ENV);
      FOciEnvironment := nil;
      FOciError := nil;
      FOciServer := nil;
    end;
  end;
end;

procedure TOracleConnection.DoInternalDisconnect;
var
  TempServiceContext : POCISvcCtx;
begin
  inherited DoInternalDisconnect;

  if assigned(FOciEnvironment) then
  begin
    if assigned(FOciError) then
    begin
      if assigned(FOciServer) then
      begin
        if assigned(FOciUserSession) then
        begin
          try
            // Create temporary service-context handle for user-disconnect
            if OciHandleAlloc(FOciEnvironment,TempServiceContext,OCI_HTYPE_SVCCTX,0,FUserMem) <> OCI_SUCCESS then
              DatabaseError(SErrHandleAllocFailed,self);

            // Set the server handle in the service-context handle
            if OCIAttrSet(TempServiceContext,OCI_HTYPE_SVCCTX,FOciServer,0,OCI_ATTR_SERVER,FOciError) <> OCI_SUCCESS then
              HandleError();
            // Set the user session handle in the service-context handle
            if OCIAttrSet(TempServiceContext,OCI_HTYPE_SVCCTX,FOciUserSession,0,OCI_ATTR_SESSION,FOciError) <> OCI_SUCCESS then
              HandleError();
            // Disconnect uses-session handle
            if OCISessionEnd(TempServiceContext,FOciError,FOcIUserSession,OCI_DEFAULT) <> OCI_SUCCESS then
              HandleError();
          finally
            // Free user-session handle
            OCIHandleFree(FOciUserSession,OCI_HTYPE_SESSION);
            // Free temporary service-context handle
            OCIHandleFree(TempServiceContext,OCI_HTYPE_SVCCTX);
            FOciUserSession := nil;
          end;
        end;

        try
          // Disconnect server handle
          if OCIServerDetach(FOciServer,FOciError,OCI_DEFAULT) <> OCI_SUCCESS then
            HandleError();
        finally
          // Free connection handles
          OCIHandleFree(FOciServer,OCI_HTYPE_SERVER);
          FOciServer := nil;
        end;
      end;
      OCIHandleFree(FOciError,OCI_HTYPE_ERROR);
      FOciError := nil;
    end;
    OCIHandleFree(FOciEnvironment,OCI_HTYPE_ENV);
    FOciEnvironment := nil;
  end;
{$IfDef LinkDynamically}
  ReleaseOCI;
{$EndIf}
end;

function TOracleConnection.AllocateCursorHandle: TSQLCursor;

var
  Cursor : TOracleCursor;
begin
  Cursor:=TOracleCursor.Create;
  Result := cursor;
end;

procedure TOracleConnection.DeAllocateCursorHandle(var cursor: TSQLCursor);

  procedure FreeOraFieldBuffers(b: array of TOraFieldBuf);
  var i : integer;
  begin
    if Length(b) > 0 then
      for i := low(b) to high(b) do
        if b[i].DescType <> 0 then
          OciDescriptorFree(b[i].buffer, b[i].DescType)
        else
          freemem(b[i].buffer);
  end;

begin
  with cursor as TOracleCursor do
    begin
    FreeOraFieldBuffers(FieldBuffers);
    FreeOraFieldBuffers(ParamBuffers);
    end;
  FreeAndNil(cursor);
end;

function TOracleConnection.AllocateTransactionHandle: TSQLHandle;
var
  locRes : TOracleTrans;
begin
  locRes := TOracleTrans.Create();
  try
    // Allocate service-context handle
    if OciHandleAlloc(FOciEnvironment,locRes.FOciSvcCtx,OCI_HTYPE_SVCCTX,0,FUserMem) <> OCI_SUCCESS then
      DatabaseError(SErrHandleAllocFailed,self);
    // Set the server-handle in the service-context handle
    if OCIAttrSet(locRes.FOciSvcCtx,OCI_HTYPE_SVCCTX,FOciServer,0,OCI_ATTR_SERVER,FOciError) <> OCI_SUCCESS then
      HandleError();
    // Set the user-session handle in the service-context handle
    if OCIAttrSet(locRes.FOciSvcCtx,OCI_HTYPE_SVCCTX,FOciUserSession,0,OCI_ATTR_SESSION,FOciError) <> OCI_SUCCESS then
      HandleError();

    // Allocate transaction handle
    if OciHandleAlloc(FOciEnvironment,locRes.FOciTrans,OCI_HTYPE_TRANS,0,FUserMem) <> OCI_SUCCESS then
      DatabaseError(SErrHandleAllocFailed,self);
    // Set the transaction handle in the service-context handle
    if OCIAttrSet(locRes.FOciSvcCtx,OCI_HTYPE_SVCCTX,locRes.FOciTrans,0,OCI_ATTR_TRANS,FOciError) <> OCI_SUCCESS then
      HandleError();
  except
    locRes.Free();
    raise;
  end;
  Result := locRes;
end;

procedure TOracleConnection.PrepareStatement(cursor: TSQLCursor;
  ATransaction: TSQLTransaction; buf: string; AParams: TParams);
  
var i        : integer;
    FOcibind : POCIDefine;
    
    OFieldType   : ub2;
    OFieldSize   : sb4;
    ODescType    : ub4;
    OBuffer      : pointer;

    stmttype     : ub2;

begin
  with cursor as TOracleCursor do
    begin
    if LogEvent(detActualSQL) then
      Log(detActualSQL,Buf);
    if OCIStmtPrepare2(TOracleTrans(ATransaction.Handle).FOciSvcCtx,FOciStmt,FOciError,@buf[1],length(buf),nil,0,OCI_NTV_SYNTAX,OCI_DEFAULT) = OCI_ERROR then
      HandleError;
    // Get statement type
    if OCIAttrGet(FOciStmt,OCI_HTYPE_STMT,@stmttype,nil,OCI_ATTR_STMT_TYPE,FOciError) = OCI_ERROR then
      HandleError;
    case stmttype of
      OCI_STMT_SELECT: FStatementType := stSelect;
      OCI_STMT_UPDATE: FStatementType := stUpdate;
      OCI_STMT_DELETE: FStatementType := stDelete;
      OCI_STMT_INSERT: FStatementType := stInsert;
      OCI_STMT_CREATE,
      OCI_STMT_DROP,
      OCI_STMT_DECLARE,
      OCI_STMT_ALTER:  FStatementType := stDDL;
    else
      FStatementType := stUnknown;
    end;
    if FStatementType in [stUpdate,stDelete,stInsert,stDDL] then
      FSelectable:=false;

    if assigned(AParams) then
      begin
      setlength(ParamBuffers,AParams.Count);
      for i := 0 to AParams.Count-1 do
        begin
        ODescType := 0;
        case AParams[i].DataType of
          ftSmallInt, ftInteger :
            begin OFieldType := SQLT_INT; OFieldSize := sizeof(integer); end;
          ftLargeInt :
            begin OFieldType := SQLT_INT; OFieldSize := sizeof(int64); end;
          ftFloat :
            begin OFieldType := SQLT_FLT; OFieldSize := sizeof(double); end;
          ftDate, ftDateTime :
            begin OFieldType := SQLT_TIMESTAMP; OFieldSize := sizeof(pointer); ODescType := OCI_DTYPE_TIMESTAMP; end;
          ftFixedChar, ftString :
            begin OFieldType := SQLT_STR; OFieldSize := 4000; end;
          ftFMTBcd, ftBCD :
            begin OFieldType := SQLT_VNU; OFieldSize := 22; end;
          ftBlob :
            //begin OFieldType := SQLT_LVB; OFieldSize := 65535; end;
            begin OFieldType := SQLT_BLOB; OFieldSize := sizeof(pointer); ODescType := OCI_DTYPE_LOB; end;
          ftMemo :
            begin OFieldType := SQLT_LVC; OFieldSize := 65535; end;
        else
          DatabaseErrorFmt(SUnsupportedParameter,[Fieldtypenames[AParams[i].DataType]],self);
        end;

        ParamBuffers[i].DescType := ODescType;
        ParamBuffers[i].Len      := OFieldSize;
        ParamBuffers[i].Size     := OFieldSize;
        if ODescType <> 0 then
          begin
          OBuffer := @ParamBuffers[i].buffer;
          OCIDescriptorAlloc(FOciEnvironment, OBuffer, ODescType, 0, nil);
          end
        else
          begin
          OBuffer := getmem(OFieldSize);
          ParamBuffers[i].buffer := OBuffer;
          end;

        FOciBind := nil;

        if AParams[i].ParamType=ptInput then
          begin
          if OCIBindByName(FOciStmt,FOcibind,FOciError,pchar(AParams[i].Name),length(AParams[i].Name),OBuffer,OFieldSize,OFieldType,@ParamBuffers[i].ind,nil,nil,0,nil,OCI_DEFAULT )= OCI_ERROR then
            HandleError;
          end
        else if AParams[i].ParamType=ptOutput then
          begin
          if OCIBindByName(FOciStmt,FOcibind,FOciError,pchar(AParams[i].Name),length(AParams[i].Name),nil,OFieldSize,OFieldType,nil,nil,nil,0,nil,OCI_DATA_AT_EXEC )= OCI_ERROR then
            HandleError;
          if OCIBindDynamic(FOcibind, FOciError, nil, @cbf_no_data, @parambuffers[i], @cbf_get_data) <> OCI_SUCCESS then
            HandleError;
          end;
        end;
      end;
    FPrepared := True;
    end;
end;

procedure TOracleConnection.SetParameters(cursor : TSQLCursor; ATransaction : TSQLTransaction; AParams : TParams);

var i         : integer;
    year, month, day, hour, min, sec, msec : word;
    s         : string;
    LobBuffer : TBytes;
    LobLength : ub4;

begin
  with cursor as TOracleCursor do for i := 0 to High(ParamBuffers) do with AParams[i] do
    if ParamType=ptInput then
      begin
      if IsNull then ParamBuffers[i].ind := -1 else ParamBuffers[i].ind := 0;

      case DataType of
        ftSmallInt,
        ftInteger         : PInteger(ParamBuffers[i].buffer)^ := AsInteger;
        ftLargeInt        : PInt64(ParamBuffers[i].buffer)^ := AsLargeInt;
        ftFloat           : PDouble(ParamBuffers[i].buffer)^ := AsFloat;
        ftString,
        ftFixedChar       : begin
                            s := asString+#0;
                            move(s[1],parambuffers[i].buffer^,length(s)+1);
                            end;
        ftDate, ftDateTime: begin
                            DecodeDate(asDateTime,year,month,day);
                            DecodeTime(asDateTime,hour,min,sec,msec);
                            if OCIDateTimeConstruct(FOciUserSession, FOciError, ParamBuffers[i].buffer, year, month, day, hour, min, sec, msec*1000000, nil, 0) = OCI_ERROR then
                              HandleError;
{                           pb := ParamBuffers[i].buffer;
                            pb[0] := (year div 100)+100;
                            pb[1] := (year mod 100)+100;
                            pb[2] := month;
                            pb[3] := day;
                            pb[4] := hour+1;
                            pb[5] := minute+1;
                            pb[6] := second+1;
}
                            end;
        ftFmtBCD, ftBCD   : begin
                            FmtBCD2Nvu(asFmtBCD,parambuffers[i].buffer);
                            end;
        ftBlob            : begin
                            LobBuffer := AsBlob; // todo: use AsBytes
                            LobLength := length(LobBuffer);
                            // create empty temporary LOB with zero length
                            if OciLobCreateTemporary(TOracleTrans(ATransaction.Handle).FOciSvcCtx, FOciError, ParamBuffers[i].Buffer, OCI_DEFAULT, OCI_DEFAULT, OCI_TEMP_BLOB, False, OCI_DURATION_SESSION) = OCI_ERROR then
                              HandleError;
                            if (LobLength > 0) and (OciLobWrite(TOracleTrans(ATransaction.Handle).FOciSvcCtx, FOciError, ParamBuffers[i].Buffer, @LobLength, 1, @LobBuffer[0], LobLength, OCI_ONE_PIECE, nil, nil, 0, SQLCS_IMPLICIT) = OCI_ERROR) then
                              HandleError;
                            end;
        ftMemo            : begin
                            LobBuffer := AsBytes;
                            LobLength := length(LobBuffer);
                            if LobLength > 65531 then LobLength := 65531;
                            PInteger(ParamBuffers[i].Buffer)^ := LobLength;
                            Move(LobBuffer[0], (ParamBuffers[i].Buffer+sizeof(integer))^, LobLength);
                            end;
        else
          DatabaseErrorFmt(SUnsupportedParameter,[DataType],self);
      end;

      end;

end;

procedure TOracleConnection.UnPrepareStatement(cursor: TSQLCursor);
begin
  if OCIStmtRelease(TOracleCursor(cursor).FOciStmt,FOciError,nil,0,OCI_DEFAULT)<> OCI_SUCCESS then
    HandleError();
  cursor.FPrepared:=False;
end;

procedure TOracleConnection.InternalStartDBTransaction(trans : TOracleTrans);
begin
  if OCITransStart(trans.FOciSvcCtx,FOciError,DefaultTimeOut,trans.FOciFlags) <> OCI_SUCCESS then
    HandleError();
end;

function TOracleConnection.GetTransactionHandle(trans: TSQLHandle): pointer;
begin
  Result := trans;
end;

function TOracleConnection.StartDBTransaction(trans: TSQLHandle; AParams: string): boolean;
var
  flags : ub4;
  i : Integer;
  s : string;
  locTrans : TOracleTrans;
begin
  flags := OCI_TRANS_READWRITE;
  if AParams <> '' then begin
    i := 1;
    s := ExtractSubStr(AParams,i,StdWordDelims);
    while ( s <> '' ) do begin
      if ( s = 'readonly' ) then
        flags := OCI_TRANS_READONLY
      else if ( s = 'serializable' ) then
        flags := OCI_TRANS_SERIALIZABLE
      else if ( s = 'readwrite' ) then
        flags := OCI_TRANS_READWRITE;
      s := ExtractSubStr(AParams,i,StdWordDelims);
    end;
  end;
  locTrans := TOracleTrans(trans);
  locTrans.FOciFlags := flags or OCI_TRANS_NEW;
  InternalStartDBTransaction(locTrans);
  Result := True;
end;

function TOracleConnection.Commit(trans: TSQLHandle): boolean;
begin
  if OCITransCommit(TOracleTrans(trans).FOciSvcCtx,FOciError,OCI_DEFAULT) <> OCI_SUCCESS then
    HandleError();
  Result := True;
end;

function TOracleConnection.Rollback(trans: TSQLHandle): boolean;
begin
  if OCITransRollback(TOracleTrans(trans).FOciSvcCtx,FOciError,OCI_DEFAULT) <> OCI_SUCCESS then
    HandleError();
  Result := True;
end;

procedure TOracleConnection.CommitRetaining(trans: TSQLHandle);
begin
  Commit(trans);
  InternalStartDBTransaction(TOracleTrans(trans));
end;

procedure TOracleConnection.RollbackRetaining(trans: TSQLHandle);
begin
  Rollback(trans);
  InternalStartDBTransaction(TOracleTrans(trans));
end;

procedure TOracleConnection.Execute(cursor: TSQLCursor; ATransaction: TSQLTransaction; AParams: TParams);
  procedure FreeParameters;
  var i: integer;
  begin
    with cursor as TOracleCursor do
      for i:=0 to high(ParamBuffers) do
        if ParamBuffers[i].DescType = OCI_DTYPE_LOB then
          if OciLobFreeTemporary(TOracleTrans(ATransaction.Handle).FOciSvcCtx, FOciError, ParamBuffers[i].Buffer) = OCI_ERROR then
            HandleError;
  end;
begin
  if Assigned(AParams) and (AParams.Count > 0) then SetParameters(cursor, ATransaction, AParams);
  if LogEvent(detParamValue) then
    LogParams(AParams);
  if cursor.FStatementType = stSelect then
    begin
    if OCIStmtExecute(TOracleTrans(ATransaction.Handle).FOciSvcCtx,(cursor as TOracleCursor).FOciStmt,FOciError,0,0,nil,nil,OCI_DEFAULT) = OCI_ERROR then
      HandleError;
    end
  else
    begin
    if OCIStmtExecute(TOracleTrans(ATransaction.Handle).FOciSvcCtx,(cursor as TOracleCursor).FOciStmt,FOciError,1,0,nil,nil,OCI_DEFAULT) = OCI_ERROR then
      HandleError;
    if Assigned(AParams) and (AParams.Count > 0) then GetParameters(cursor, ATransaction, AParams);
    end;
  FreeParameters;
end;

function TOracleConnection.RowsAffected(cursor: TSQLCursor): TRowsCount;
var rowcount: ub4;
begin
  if Assigned(cursor) and (OCIAttrGet((cursor as TOracleCursor).FOciStmt, OCI_HTYPE_STMT, @rowcount, nil, OCI_ATTR_ROW_COUNT, FOciError) = OCI_SUCCESS) then
    Result:=rowcount
  else
    Result:=inherited RowsAffected(cursor);
end;

procedure TOracleConnection.AddFieldDefs(cursor: TSQLCursor; FieldDefs: TFieldDefs);

var Param      : POCIParam;
    counter    : ub4;

    FieldType  : TFieldType;
    FieldName  : string;
    FieldSize  : cardinal;

    OFieldType   : ub2;
    OFieldName   : Pchar;
    OFieldSize   : ub4;
    OFNameLength : ub4;
    NumCols      : ub4;
    FOciDefine   : POCIDefine;
    OPrecision   : sb2;
    OScale       : sb1;
    ODescType    : ub4;
    OBuffer      : pointer;

begin
  Param := nil;
  with cursor as TOracleCursor do
    begin
    if OCIAttrGet(FOciStmt,OCI_HTYPE_STMT,@numcols,nil,OCI_ATTR_PARAM_COUNT,FOciError) = OCI_ERROR then
      HandleError;

    // Note: needs to be cleared then allocated in one go.
    Setlength(FieldBuffers,numcols);

    for counter := 1 to numcols do
      begin
      // Clear OFieldSize. Oracle 9i, 10g doc says *ub4 but some clients use *ub2 leaving
      // high 16 bit untouched resulting in huge values and ORA-01062
      // WARNING: this does not work on big endian systems !!!!
      // To be tested if BE systems have this *ub2<->*ub4 problem
      OFieldSize:=0;
      ODescType :=0;

      if OCIParamGet(FOciStmt,OCI_HTYPE_STMT,FOciError,Param,counter) = OCI_ERROR then
        HandleError;

      if OCIAttrGet(Param,OCI_DTYPE_PARAM,@OFieldType,nil,OCI_ATTR_DATA_TYPE,FOciError) = OCI_ERROR then
        HandleError;

      if OCIAttrGet(Param,OCI_DTYPE_PARAM,@OFieldSize,nil,OCI_ATTR_DATA_SIZE,FOciError) = OCI_ERROR then
        HandleError;

      FieldSize := 0;
      
      case OFieldType of
        OCI_TYPECODE_NUMBER   : begin
                                if OCIAttrGet(Param,OCI_DTYPE_PARAM,@Oprecision,nil,OCI_ATTR_PRECISION,FOciError) = OCI_ERROR then
                                  HandleError;
                                if OCIAttrGet(Param,OCI_DTYPE_PARAM,@Oscale,nil,OCI_ATTR_SCALE,FOciError) = OCI_ERROR then
                                  HandleError;
                                if (Oscale = 0) and (Oprecision < 10) then
                                  begin
                                  if Oprecision=0 then //Number(0,0) = number(32,4)
                                    begin
                                    FieldType := ftFMTBCD;
                                    FieldSize := 4;
                                    OFieldType := SQLT_VNU;
                                    OFieldSize:= 22;
                                    end
                                  else if Oprecision < 5 then
                                    begin
                                    FieldType := ftSmallint;
                                    OFieldType := SQLT_INT;
                                    OFieldSize := sizeof(smallint);
                                    end
                                  else // OPrecision=5..9, OScale=0
                                    begin
                                    FieldType := ftInteger;
                                    OFieldType := SQLT_INT;
                                    OFieldSize:= sizeof(integer);
                                    end;
                                  end
                                else if (Oscale = -127) {and (OPrecision=0)} then
                                  begin
                                  FieldType := ftFloat;
                                  OFieldType := SQLT_FLT;
                                  OFieldSize:=sizeof(double);
                                  end
                                else if (Oscale >=0) and (Oscale <=4) and (OPrecision<=12) then
                                  begin
                                  FieldType := ftBCD;
                                  FieldSize := oscale;
                                  OFieldType := SQLT_VNU;
                                  OFieldSize:= 22;
                                  end
                                else if (OPrecision-Oscale<64) and (Oscale < 64) then // limited to 63 digits before or after decimal point
                                  begin
                                  FieldType := ftFMTBCD;
                                  FieldSize := oscale;
                                  OFieldType := SQLT_VNU;
                                  OFieldSize:= 22;
                                  end
                                else // approximation with double, best we can do
                                  begin
                                  FieldType := ftFloat;
                                  OFieldType := SQLT_FLT;
                                  OFieldSize:=sizeof(double);
                                  end;
                                end;
        SQLT_LNG              : begin
                                FieldType := ftString;
                                FieldSize := MaxSmallint; // OFieldSize is zero for LONG data type
                                OFieldSize:= MaxSmallint+1;
                                OFieldType:=SQLT_STR;
                                end;
        OCI_TYPECODE_CHAR,
        OCI_TYPECODE_VARCHAR,
        OCI_TYPECODE_VARCHAR2 : begin
                                FieldType := ftString;
                                FieldSize := OFieldSize;
                                inc(OFieldSize);
                                OFieldType:=SQLT_STR;
                                end;
        OCI_TYPECODE_DATE     : FieldType := ftDate;
        OCI_TYPECODE_TIMESTAMP,
        OCI_TYPECODE_TIMESTAMP_LTZ,
        OCI_TYPECODE_TIMESTAMP_TZ :
                                begin
                                FieldType := ftDateTime;
                                OFieldType := SQLT_TIMESTAMP;
                                ODescType := OCI_DTYPE_TIMESTAMP;
                                end;
        OCI_TYPECODE_BFLOAT,
        OCI_TYPECODE_BDOUBLE  : begin
                                FieldType := ftFloat;
                                OFieldType := SQLT_BDOUBLE;
                                OFieldSize := sizeof(double);
                                end;
        SQLT_BLOB             : begin
                                FieldType := ftBlob;
                                ODescType := OCI_DTYPE_LOB;
                                end;
        SQLT_CLOB             : begin
                                FieldType := ftMemo;
                                ODescType := OCI_DTYPE_LOB;
                                end
      else
        FieldType := ftUnknown;
      end;

      FieldBuffers[counter-1].DescType := ODescType;
      if ODescType <> 0 then
        begin
        OBuffer := @FieldBuffers[counter-1].buffer;
        OCIDescriptorAlloc(FOciEnvironment, OBuffer, ODescType, 0, nil);
        OFieldSize := sizeof(pointer);
        end
      else
        begin
        OBuffer := getmem(OFieldSize);
        FieldBuffers[counter-1].buffer := OBuffer;
        end;

      if FieldType <> ftUnknown then
        begin
        FOciDefine := nil;
        if OciDefineByPos(FOciStmt,FOciDefine,FOciError,counter,OBuffer,OFieldSize,OFieldType,@FieldBuffers[counter-1].ind,nil,nil,OCI_DEFAULT) = OCI_ERROR then
          HandleError;
        end;

      if OCIAttrGet(Param,OCI_DTYPE_PARAM,@OFieldName,@OFNameLength,OCI_ATTR_NAME,FOciError) <> OCI_SUCCESS then
        HandleError;

      setlength(Fieldname,OFNameLength);
      move(OFieldName^,Fieldname[1],OFNameLength);

      FieldDefs.Add(FieldDefs.MakeNameUnique(FieldName), FieldType, FieldSize, False, counter);
      end;
  end;
end;

function TOracleConnection.Fetch(cursor: TSQLCursor): boolean;
begin
  case OCIStmtFetch2((cursor as TOracleCursor).FOciStmt,FOciError,1,OCI_FETCH_NEXT,1,OCI_DEFAULT) of
    OCI_ERROR   : begin
                  Result := False;
                  HandleError;
                  end;
    OCI_NO_DATA : Result := False;
    OCI_SUCCESS : Result := True;
    OCI_SUCCESS_WITH_INFO : Begin
                            Result := True;
                            HandleError;
                            end;
  end; {case}
end;

function TOracleConnection.LoadField(cursor: TSQLCursor; FieldDef: TFieldDef; buffer: pointer; out CreateBlob : boolean): boolean;

var
  b       : pbyte;
  size,i  : byte;
  exp     : shortint;
  cur     : Currency;
  odt     : TODateTime;

begin
  CreateBlob := False;
  with cursor as TOracleCursor do if fieldbuffers[FieldDef.FieldNo-1].ind = -1 then
    Result := False
  else
    begin
    Result := True;
    case FieldDef.DataType of
      ftString :
        move(fieldbuffers[FieldDef.FieldNo-1].buffer^,buffer^,FieldDef.Size);
      ftBCD :
        begin
        b := fieldbuffers[FieldDef.FieldNo-1].buffer;
        size := b[0];
        cur := 0;
        if (b[1] and $80)=$80 then // the number is positive
          begin
          exp := (b[1] and $7f)-65;
          for i := 2 to size do
            cur := cur + (b[i]-1) * intpower(100,-(i-2)+exp);
          end
        else
          begin
          exp := (not(b[1]) and $7f)-65;
          for i := 2 to size-1 do
            cur := cur + (101-b[i]) * intpower(100,-(i-2)+exp);
          cur := -cur;
          end;
        move(cur,buffer^,SizeOf(Currency));
        end;
      ftFmtBCD :
        pBCD(buffer)^:= Nvu2FmtBCD(fieldbuffers[FieldDef.FieldNo-1].buffer);
      ftFloat :
        move(fieldbuffers[FieldDef.FieldNo-1].buffer^,buffer^,sizeof(double));
      ftSmallInt :
        move(fieldbuffers[FieldDef.FieldNo-1].buffer^,buffer^,sizeof(smallint));
      ftInteger :
        move(fieldbuffers[FieldDef.FieldNo-1].buffer^,buffer^,sizeof(integer));
      ftDate :
        begin
        b := fieldbuffers[FieldDef.FieldNo-1].buffer;
        PDateTime(buffer)^ := ComposeDateTime(EncodeDate((b[0]-100)*100+(b[1]-100),b[2],b[3]), EncodeTime(b[4]-1, b[5]-1, b[6]-1, 0));
        end;
      ftDateTime :
        begin
        OCIDateTimeGetDate(FOciUserSession, FOciError, FieldBuffers[FieldDef.FieldNo-1].buffer, @odt.year, @odt.month, @odt.day);
        OCIDateTimeGetTime(FOciUserSession, FOciError, FieldBuffers[FieldDef.FieldNo-1].buffer, @odt.hour, @odt.min, @odt.sec, @odt.fsec);
        PDateTime(buffer)^ := ComposeDateTime(EncodeDate(odt.year,odt.month,odt.day), EncodeTime(odt.hour,odt.min,odt.sec,odt.fsec div 1000000));
        end;
      ftBlob,
      ftMemo :
        CreateBlob := True;
    else
      Result := False;
    end;
    end;
end;

procedure TOracleConnection.LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction: TSQLTransaction);
var LobLocator: pointer;
    LobCharSetForm: ub1;
    LobLength, LobSize: ub4;
begin
  LobLocator := (cursor as TOracleCursor).FieldBuffers[FieldDef.FieldNo-1].Buffer;
  //if OCILobLocatorIsInit(TOracleTrans(ATransaction.Handle).FOciSvcCtx, FOciError, LobLocator, @is_init) = OCI_ERROR then
  //  HandleError;
  // For character LOBs, it is the number of characters, for binary LOBs and BFILEs it is the number of bytes
  if OciLobGetLength(TOracleTrans(ATransaction.Handle).FOciSvcCtx, FOciError, LobLocator, @LobLength) = OCI_ERROR then
    HandleError;
  if OCILobCharSetForm(FOciEnvironment, FOciError, LobLocator, @LobCharSetForm) = OCI_ERROR then
    HandleError;
  // Adjust initial buffer size (in bytes), while LobLength can be in characters
  case LobCharSetForm of
    0: ;            // BLOB
    SQLCS_IMPLICIT, // CLOB
    SQLCS_NCHAR:    // NCLOB
      LobLength := LobLength*4;
  end;
  ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer, LobLength);
  LobSize := 0;
  // For CLOBs and NCLOBs the total amount of data which should be readed is on input in characters, but on output is in bytes if client character set is varying-width
  // The application must call OCILobRead() (in streamed mode) over and over again to read more pieces of the LOB until the OCI_NEED_DATA error code is not returned.
  // If the LOB is a BLOB, the csid and csfrm parameters are ignored.
  if (LobLength > 0) and (OciLobRead(TOracleTrans(ATransaction.Handle).FOciSvcCtx, FOciError, LobLocator, @LobSize, 1, ABlobBuf^.BlobBuffer^.Buffer, LobLength, nil, nil, 0, LobCharSetForm) = OCI_ERROR) then
    HandleError;
  // Shrink initial buffer if needed (we assume that LobSize is in bytes, what is true for CLOB,NCLOB if client character set is varying-width, but if client character set is fixed-width then it is in characters)
  if LobSize <> LobLength then
    ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer, LobSize);
  ABlobBuf^.BlobBuffer^.Size := LobSize;
end;

procedure TOracleConnection.FreeFldBuffers(cursor: TSQLCursor);
begin
//  inherited FreeFldBuffers(cursor);
end;

procedure TOracleConnection.UpdateIndexDefs(IndexDefs: TIndexDefs;
  TableName: string);
var qry : TSQLQuery;

begin
  if not assigned(Transaction) then
    DatabaseError(SErrConnTransactionnSet);

  // Get table name into canonical format
  if (length(TableName)>2) and (TableName[1]=ObjectQuote) and (TableName[length(TableName)]=ObjectQuote) then
    TableName := AnsiDequotedStr(TableName, ObjectQuote)
  else
    TableName := UpperCase(TableName); //ANSI SQL: the name of an identifier (such as table names) are implicitly converted to uppercase, unless double quotes are used when referring to the identifier.

  qry := tsqlquery.Create(nil);
  qry.transaction := Transaction;
  qry.database := Self;
  with qry do
    begin
    ReadOnly := True;
    sql.clear;
    sql.add('SELECT '+
              'i.INDEX_NAME,  '+
              'c.COLUMN_NAME, '+
              'p.CONSTRAINT_TYPE '+
            'FROM ALL_INDEXES i, ALL_IND_COLUMNS c,ALL_CONSTRAINTS p  '+
            'WHERE '+
              'i.OWNER=c.INDEX_OWNER AND '+
              'i.INDEX_NAME=c.INDEX_NAME AND '+
              'p.INDEX_NAME(+)=i.INDEX_NAME AND '+
              'c.TABLE_NAME = ''' + TableName + ''' '+
            'ORDER by i.INDEX_NAME,c.COLUMN_POSITION');
    open;
    end;
  while not qry.eof do with IndexDefs.AddIndexDef do
    begin
    Name := trim(qry.fields[0].asstring);
    Fields := trim(qry.Fields[1].asstring);
    If UpperCase(qry.fields[2].asstring)='P' then options := options + [ixPrimary];
    If UpperCase(qry.fields[2].asstring)='U' then options := options + [ixUnique];
    qry.next;
    while (name = qry.fields[0].asstring) and (not qry.eof) do
      begin
      Fields := Fields + ';' + trim(qry.Fields[1].asstring);
      qry.next;
      end;
    end;
  qry.close;
  qry.free;
end;

function TOracleConnection.GetSchemaInfoSQL(SchemaType: TSchemaType;
  SchemaObjectName, SchemaPattern: string): string;
var
  s : string;
begin
  case SchemaType of
    stTables     : s := 'SELECT '+
                          '''' + DatabaseName + ''' as catalog_name, '+
                          'sys_context( ''userenv'', ''current_schema'' ) as schema_name, '+
                          'TABLE_NAME,'+
                          'TABLE_TYPE '+
                        'FROM USER_CATALOG ' +
                        'WHERE '+
                          'TABLE_TYPE<>''SEQUENCE'' '+
                        'ORDER BY TABLE_NAME';

    stSysTables  : s := 'SELECT '+
                          '''' + DatabaseName + ''' as catalog_name, '+
                          'OWNER as schema_name, '+
                          'TABLE_NAME,'+
                          'TABLE_TYPE '+
                        'FROM ALL_CATALOG ' +
                        'WHERE '+
                          'TABLE_TYPE<>''SEQUENCE'' '+
                        'ORDER BY TABLE_NAME';
    stColumns    : s := 'SELECT '+
                          'OWNER as schema_name, '+
                          'COLUMN_NAME, '+
                          'DATA_TYPE as column_datatype, '+
                          'CHARACTER_SET_NAME, '+
                          'NULLABLE as column_nullable, '+
                          'DATA_LENGTH as column_length, '+
                          'DATA_PRECISION as column_precision, '+
                          'DATA_SCALE as column_scale, '+
                          'DATA_DEFAULT as column_default '+
                        'FROM ALL_TAB_COLUMNS '+
                        'WHERE Upper(TABLE_NAME) = '''+UpperCase(SchemaObjectName)+''' '+
                        'ORDER BY COLUMN_NAME';
    // Columns of tables, views and clusters accessible to user; hidden columns are filtered out.												
    stProcedures : s := 'SELECT '+
                          'case when PROCEDURE_NAME is null then OBJECT_NAME ELSE OBJECT_NAME || ''.'' || PROCEDURE_NAME end AS procedure_name '+
                        'FROM USER_PROCEDURES ';
  else
    DatabaseError(SMetadataUnavailable)
  end; {case}
  result := s;
end;

constructor TOracleConnection.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FConnOptions := FConnOptions + [sqEscapeRepeat,sqSequences];
  FOciEnvironment := nil;
  FOciError := nil;
  FOciServer := nil;
  FOciUserSession := nil;
  FUserMem := nil;
end;

{ TOracleConnectionDef }

class function TOracleConnectionDef.TypeName: String;
begin
  Result:='Oracle';
end;

class function TOracleConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  Result:=TOracleConnection;
end;

class function TOracleConnectionDef.Description: String;
begin
  Result:='Connect to an Oracle database directly via the client library';
end;

class function TOracleConnectionDef.DefaultLibraryName: String;
begin
  {$IfDef LinkDynamically}
  Result:=ocilib;
  {$else}
  Result:='';
  {$endif}
end;

class function TOracleConnectionDef.LoadFunction: TLibraryLoadFunction;
begin
  {$IfDef LinkDynamically}
  Result:=@InitialiseOCI;
  {$else}
  Result:=Nil;
  {$endif}
end;

class function TOracleConnectionDef.UnLoadFunction: TLibraryUnLoadFunction;
begin
  {$IfDef LinkDynamically}
  Result:=@ReleaseOCI;
  {$else}
  Result:=Nil;
  {$endif}
end;

class function TOracleConnectionDef.LoadedLibraryName: string;
begin
  {$IfDef LinkDynamically}
  Result:=OCILoadedLibrary;
  {$else}
  Result:='';
  {$endif}
end;

{ TOracleTrans }

destructor TOracleTrans.Destroy();
begin
  OCIHandleFree(FOciTrans,OCI_HTYPE_TRANS);
  OCIHandleFree(FOciSvcCtx,OCI_HTYPE_SVCCTX);
  inherited Destroy();
end;

initialization
  RegisterConnection(TOracleConnectionDef);
finalization
  UnRegisterConnection(TOracleConnectionDef);
end.

