{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2009 by the Free Pascal development team

    TFCgiApplication class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{ $define CGIDEBUG}
{$mode objfpc}
{$H+}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Disable rangechecks. 
  Buffers of unknown size are received and handled with a dummy array type }

{$RANGECHECKS OFF}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
unit custfcgi;

Interface

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(win32) or defined(win64)}
{$define windowspipe}
{$ifend}

uses
  Classes,SysUtils, httpdefs, 
{$ifdef unix}
  BaseUnix,
{$else}
  winsock2, windows,
{$endif}
  Sockets, custweb, cgiprotocol, httpprotocol, custcgi, fastcgi;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
uses
  Classes,SysUtils, httpdefs, 
{$ifdef unix}
  BaseUnix, TermIO,
{$else}
  winsock2,
{$endif}
  Sockets, custweb, custcgi, fastcgi;
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

Type
  { TFCGIRequest }
  TCustomFCgiApplication = Class;
  TFCGIRequest = Class;
  TFCGIResponse = Class;

  TProtocolOption = (poNoPadding,poStripContentLength, poFailonUnknownRecord,
                     poReuseAddress, poUseSelect );
  TProtocolOptions = Set of TProtocolOption;

  TUnknownRecordEvent = Procedure (ARequest : TFCGIRequest; AFCGIRecord: PFCGI_Header) Of Object;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TFastCGIReadEvent = Function (AHandle : THandle; Var ABuf; ACount : Integer) : Integer of Object;
  TFastCGIWriteEvent = Function (AHandle : THandle; Const ABuf; ACount : Integer; Out ExtendedErrorCode : Integer) : Integer of Object;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  TFCGIRequest = Class(TCGIRequest)
  Private
    FHandle: THandle;
    FKeepConnectionAfterRequest: boolean;
    FPO: TProtoColOptions;
    FRequestID : Word;
    FCGIParams : TSTrings;
    FUR: TUnknownRecordEvent;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FLog : TLogEvent;
    FSTDin : String;
  Protected
    function DoGetCGIVar(AVarName: String): String; override;
    procedure GetNameValuePairsFromContentRecord(const ARecord : PFCGI_ContentRecord; NameValueList : TStrings); virtual;
    Procedure Log(EventType : TEventType; Const Msg : String);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure GetNameValuePairsFromContentRecord(const ARecord : PFCGI_ContentRecord; NameValueList : TStrings);
  Protected
    Function GetFieldValue(Index : Integer) : String; override;
    procedure ReadContent; override;
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
  Public
    destructor Destroy; override;
    function ProcessFCGIRecord(AFCGIRecord : PFCGI_Header) : boolean; virtual;
    property RequestID : word read FRequestID write FRequestID;
    property Handle : THandle read FHandle write FHandle;
    property KeepConnectionAfterRequest : boolean read FKeepConnectionAfterRequest;
    Property ProtocolOptions : TProtoColOptions read FPO Write FPO;
    Property OnUnknownRecord : TUnknownRecordEvent Read FUR Write FUR;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TFCGIRequestClass = Class of TFCGIRequest;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  { TFCGIResponse }

  TFCGIResponse = Class(TCGIResponse)
  private
    FPO: TProtoColOptions;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FOnWrite : TFastCGIWriteEvent;
  Protected
    procedure Write_FCGIRecord(ARecord : PFCGI_Header); virtual;
=======
    procedure Write_FCGIRecord(ARecord : PFCGI_Header);
  Protected
>>>>>>> graemeg/cpstrnew
=======
    procedure Write_FCGIRecord(ARecord : PFCGI_Header);
  Protected
>>>>>>> graemeg/cpstrnew
=======
    procedure Write_FCGIRecord(ARecord : PFCGI_Header);
  Protected
>>>>>>> graemeg/cpstrnew
=======
    procedure Write_FCGIRecord(ARecord : PFCGI_Header);
  Protected
>>>>>>> origin/cpstrnew
    Procedure DoSendHeaders(Headers : TStrings); override;
    Procedure DoSendContent; override;
    Property ProtocolOptions : TProtoColOptions Read FPO Write FPO;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TFCGIResponseClass = Class of TFCGIResponse;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  TReqResp = record
             Request : TFCgiRequest;
             Response : TFCgiResponse;
             end;

  { TFCgiHandler }

  TFCgiHandler = class(TWebHandler)
  Private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FLingerTimeOut: integer;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FOnUnknownRecord: TUnknownRecordEvent;
    FPO: TProtoColOptions;
    FRequestsArray : Array of TReqResp;
    FRequestsAvail : integer;
    FHandle : THandle;
    Socket: longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FIAddress      : TInetSockAddr;
    FAddressLength : tsocklen;
    FAddress: string;
    FTimeOut,
    FPort: integer;
{$ifdef windowspipe}
    FIsWinPipe: Boolean;
{$endif}
{$IFDEF WINDOWS}
    FShutdownThread : TThread;
    Procedure CheckShutDownEvent;
    Procedure HandleShutDownEvent(Sender : TObject);
{$ENDIF}
    function AcceptConnection: Integer;
    procedure CloseConnection;
    function Read_FCGIRecord : PFCGI_Header;
    function DataAvailable : Boolean;
  protected
    function CreateRequest : TFCGIRequest; virtual;
    function CreateResponse(ARequest: TFCGIRequest) : TFCGIResponse; virtual;
    Function DoFastCGIRead(AHandle : THandle; Var ABuf; ACount : Integer) : Integer; virtual;
    Function DoFastCGIWrite(AHandle : THandle; Const ABuf; ACount : Integer; Out ExtendedErrorCode : Integer) : Integer; virtual;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    FAddress: string;
    FTimeOut,
    FPort: integer;
    function Read_FCGIRecord : PFCGI_Header;
    function DataAvailable : Boolean;
  protected
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    FAddress: string;
    FTimeOut,
    FPort: integer;
    function Read_FCGIRecord : PFCGI_Header;
    function DataAvailable : Boolean;
  protected
>>>>>>> origin/cpstrnew
    function  ProcessRecord(AFCGI_Record: PFCGI_Header; out ARequest: TRequest;  out AResponse: TResponse): boolean; virtual;
    procedure SetupSocket(var IAddress: TInetSockAddr;  var AddressLength: tsocklen); virtual;
    function  WaitForRequest(out ARequest : TRequest; out AResponse : TResponse) : boolean; override;
    procedure EndRequest(ARequest : TRequest;AResponse : TResponse); override;
  Public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    property Port: integer read FPort write FPort;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property LingerTimeOut : integer read FLingerTimeOut write FLingerTimeOut;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    property Address: string read FAddress write FAddress;
    Property ProtocolOptions : TProtoColOptions Read FPO Write FPO;
    Property OnUnknownRecord : TUnknownRecordEvent Read FOnUnknownRecord Write FOnUnknownRecord;
    Property TimeOut : Integer Read FTimeOut Write FTimeOut;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TFCgiHandlerClass = Class of TFCgiHandler;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  { TCustomFCgiApplication }

  TCustomFCgiApplication = Class(TCustomWebApplication)
  private
    function GetAddress: string;
    function GetFPO: TProtoColOptions;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function GetLingerTimeOut: integer;
    function GetOnUnknownRecord: TUnknownRecordEvent;
    function GetPort: integer;
    procedure SetAddress(const AValue: string);
    procedure SetLingerTimeOut(const AValue: integer);
=======
    function GetOnUnknownRecord: TUnknownRecordEvent;
    function GetPort: integer;
    procedure SetAddress(const AValue: string);
>>>>>>> graemeg/cpstrnew
=======
    function GetOnUnknownRecord: TUnknownRecordEvent;
    function GetPort: integer;
    procedure SetAddress(const AValue: string);
>>>>>>> graemeg/cpstrnew
=======
    function GetOnUnknownRecord: TUnknownRecordEvent;
    function GetPort: integer;
    procedure SetAddress(const AValue: string);
>>>>>>> graemeg/cpstrnew
=======
    function GetOnUnknownRecord: TUnknownRecordEvent;
    function GetPort: integer;
    procedure SetAddress(const AValue: string);
>>>>>>> origin/cpstrnew
    procedure SetOnUnknownRecord(const AValue: TUnknownRecordEvent);
    procedure SetPort(const AValue: integer);
    procedure SetPO(const AValue: TProtoColOptions);
  protected
    function InitializeWebHandler: TWebHandler; override;
  Public
    property Port: integer read GetPort write SetPort;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property LingerTimeOut : integer read GetLingerTimeOut write SetLingerTimeOut;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    property Address: string read GetAddress write SetAddress;
    Property ProtocolOptions : TProtoColOptions Read GetFPO Write SetPO;
    Property OnUnknownRecord : TUnknownRecordEvent Read GetOnUnknownRecord Write SetOnUnknownRecord;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Var
  FCGIRequestClass : TFCGIRequestClass = TFCGIRequest;
  FCGIResponseClass : TFCGIResponseClass = TFCGIResponse;
  FCGIWebHandlerClass : TFCgiHandlerClass = TFCgiHandler;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
ResourceString
  SNoInputHandle    = 'Failed to open input-handle passed from server. Socket Error: %d';
  SNoSocket         = 'Failed to open socket. Socket Error: %d';
  SBindFailed       = 'Failed to bind to port %d. Socket Error: %d';
  SListenFailed     = 'Failed to listen to port %d. Socket Error: %d';
  SErrReadingSocket = 'Failed to read data from socket. Error: %d';
  SErrReadingHeader = 'Failed to read FastCGI header. Read only %d bytes';
  SErrWritingSocket = 'Failed to write data to socket. Error: %d';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrNoRequest     = 'Internal error: No request available when writing data';
  
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
Implementation

{$ifdef CGIDEBUG}
uses
  dbugintf;
{$endif}
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
{$undef nosignal}

{$if defined(FreeBSD) or defined(Linux)}
  {$define nosignal}
{$ifend}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF WINDOWS}
Type

  { TShutdownThread }
  TShutdownEvent = Procedure (Sender : TObject) Of Object;
  TShutdownThread = Class(TThread)
  Private
    FEvent : THandle;
    FOnShutDown : TShutdownEvent;
  Public
    Constructor CreateWithEvent(AEvent : THandle; AOnShutDown : TShutdownEvent);
    Procedure Execute; override;
  end;
{$ENDIF}

Const 
   NoSignalAttr =  {$ifdef nosignal} MSG_NOSIGNAL{$else}0{$endif};

{$IFDEF WINDOWS}
{ TShutdownThread }

constructor TShutdownThread.CreateWithEvent(AEvent: THandle; AOnShutDown : TShutdownEvent);
begin
  Inherited Create(False);
  FEvent:=AEvent;
  FOnShutDown:=AOnShutDown;
  OnTerminate:=AOnShutDown;
end;

procedure TShutdownThread.Execute;
begin
  WaitForSingleObject(FEvent,INFINITE);
  If Assigned(FOnShutDown) then
    FOnShutDown(Self);
  // This is very ugly, but there is no other way to stop the named pipe
  // from accepting new connections.
  // Using Halt(0) is not enough.
  ExitProcess(0);
end;
{$ENDIF WINDOWS}

{ TFCGIHTTPRequest }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Const 
   NoSignalAttr =  {$ifdef nosignal} MSG_NOSIGNAL{$else}0{$endif};

{ TFCGIHTTPRequest }

procedure TFCGIRequest.ReadContent;
begin
  // Nothing has to be done. This should never be called
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

destructor TFCGIRequest.Destroy;
begin
  FCGIParams.Free;
  inherited Destroy;
end;

function TFCGIRequest.ProcessFCGIRecord(AFCGIRecord: PFCGI_Header): boolean;
var cl,rcl : Integer;
begin
  Result := False;
  case AFCGIRecord^.reqtype of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FCGI_BEGIN_REQUEST :
         begin
         FKeepConnectionAfterRequest := (PFCGI_BeginRequestRecord(AFCGIRecord)^.body.flags and FCGI_KEEP_CONN) = FCGI_KEEP_CONN;
//         With PFCGI_BeginRequestRecord(AFCGIRecord)^.body do
//           log(etDebug,Format('Begin request body role & flags: %d %d',[Beton(Role),Flags]));
         end;
=======
    FCGI_BEGIN_REQUEST : FKeepConnectionAfterRequest := (PFCGI_BeginRequestRecord(AFCGIRecord)^.body.flags and FCGI_KEEP_CONN) = FCGI_KEEP_CONN;
>>>>>>> graemeg/cpstrnew
=======
    FCGI_BEGIN_REQUEST : FKeepConnectionAfterRequest := (PFCGI_BeginRequestRecord(AFCGIRecord)^.body.flags and FCGI_KEEP_CONN) = FCGI_KEEP_CONN;
>>>>>>> graemeg/cpstrnew
=======
    FCGI_BEGIN_REQUEST : FKeepConnectionAfterRequest := (PFCGI_BeginRequestRecord(AFCGIRecord)^.body.flags and FCGI_KEEP_CONN) = FCGI_KEEP_CONN;
>>>>>>> graemeg/cpstrnew
=======
    FCGI_BEGIN_REQUEST : FKeepConnectionAfterRequest := (PFCGI_BeginRequestRecord(AFCGIRecord)^.body.flags and FCGI_KEEP_CONN) = FCGI_KEEP_CONN;
>>>>>>> origin/cpstrnew
    FCGI_PARAMS :       begin
                        if AFCGIRecord^.contentLength=0 then
                          Result := False
                        else
                          begin
                          if not assigned(FCGIParams) then
                            FCGIParams := TStringList.Create;
                          GetNameValuePairsFromContentRecord(PFCGI_ContentRecord(AFCGIRecord),FCGIParams);
                          end;
                        end;
    FCGI_STDIN :        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        Result:=AFCGIRecord^.contentLength=0;
                        if not Result then
                          begin
                          cl := length(FSTDin);
                          rcl := BetoN(PFCGI_ContentRecord(AFCGIRecord)^.header.contentLength);
                          SetLength(FSTDin, rcl+cl);
                          move(PFCGI_ContentRecord(AFCGIRecord)^.ContentData[0],FSTDin[cl+1],rcl);
                          InitContent(FSTDin);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                        if AFCGIRecord^.contentLength=0 then
                          begin
                          Result := True;
                          InitRequestVars;
                          ParseCookies;
                          end
                        else
                          begin
                          cl := length(FContent);
                          rcl := BetoN(PFCGI_ContentRecord(AFCGIRecord)^.header.contentLength);
                          SetLength(FContent, rcl+cl);
                          move(PFCGI_ContentRecord(AFCGIRecord)^.ContentData[0],FContent[cl+1],rcl);
                          FContentRead:=True;
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
                        end;
  else
    if Assigned(FUR) then
      FUR(Self,AFCGIRecord)
    else
      if poFailonUnknownRecord in FPO then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        TFCgiHandler.DoError('Unknown FASTCGI record type: %s',[AFCGIRecord^.reqtype]);
  end;
  if Result then
    InitRequestVars;
end;

function TFCGIRequest.DoGetCGIVar(AVarName: String): String;
begin
  Result:=FCGIParams.Values[AVarName];
=======
        Raise EFPWebError.CreateFmt('Unknown FASTCGI record type: %s',[AFCGIRecord^.reqtype]);
  end;
>>>>>>> graemeg/cpstrnew
=======
        Raise EFPWebError.CreateFmt('Unknown FASTCGI record type: %s',[AFCGIRecord^.reqtype]);
  end;
>>>>>>> graemeg/cpstrnew
=======
        Raise EFPWebError.CreateFmt('Unknown FASTCGI record type: %s',[AFCGIRecord^.reqtype]);
  end;
>>>>>>> graemeg/cpstrnew
=======
        Raise EFPWebError.CreateFmt('Unknown FASTCGI record type: %s',[AFCGIRecord^.reqtype]);
  end;
>>>>>>> origin/cpstrnew
end;

procedure TFCGIRequest.GetNameValuePairsFromContentRecord(const ARecord: PFCGI_ContentRecord; NameValueList: TStrings);

var
  i : integer;

  function GetVarLength : Integer;
  begin
    if (ARecord^.ContentData[i] and 128) = 0 then
      Result:=ARecord^.ContentData[i]
    else
      begin
//      Result:=BEtoN(PLongint(@(ARecord^.ContentData[i]))^);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Result:=Int64(((ARecord^.ContentData[i] and $7f) shl 24)) + (ARecord^.ContentData[i+1] shl 16)
=======
      Result:=((ARecord^.ContentData[i] and $7f) shl 24) + (ARecord^.ContentData[i+1] shl 16)
>>>>>>> graemeg/cpstrnew
=======
      Result:=((ARecord^.ContentData[i] and $7f) shl 24) + (ARecord^.ContentData[i+1] shl 16)
>>>>>>> graemeg/cpstrnew
=======
      Result:=((ARecord^.ContentData[i] and $7f) shl 24) + (ARecord^.ContentData[i+1] shl 16)
>>>>>>> graemeg/cpstrnew
=======
      Result:=((ARecord^.ContentData[i] and $7f) shl 24) + (ARecord^.ContentData[i+1] shl 16)
>>>>>>> origin/cpstrnew
                   + (ARecord^.ContentData[i+2] shl 8) + (ARecord^.ContentData[i+3]);
      inc(i,3);
      end;
    inc(i);
  end;

  function GetString(ALength : integer) : string;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if (ALength<0) then
      ALength:=0;
    SetLength(Result,ALength);
    if (ALength>0) then
      move(ARecord^.ContentData[i],Result[1],ALength);
=======
    SetLength(Result,ALength);
    move(ARecord^.ContentData[i],Result[1],ALength);
>>>>>>> graemeg/cpstrnew
=======
    SetLength(Result,ALength);
    move(ARecord^.ContentData[i],Result[1],ALength);
>>>>>>> graemeg/cpstrnew
=======
    SetLength(Result,ALength);
    move(ARecord^.ContentData[i],Result[1],ALength);
>>>>>>> graemeg/cpstrnew
=======
    SetLength(Result,ALength);
    move(ARecord^.ContentData[i],Result[1],ALength);
>>>>>>> origin/cpstrnew
    inc(i,ALength);
  end;

var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  VarNo,NameLength, ValueLength : Integer;
  RecordLength : Integer;
  Name,Value : String;
  h : THeader;
  v : THTTPVariableType;
=======
  NameLength, ValueLength : Integer;
  RecordLength : Integer;
  Name,Value : String;
>>>>>>> graemeg/cpstrnew
=======
  NameLength, ValueLength : Integer;
  RecordLength : Integer;
  Name,Value : String;
>>>>>>> graemeg/cpstrnew
=======
  NameLength, ValueLength : Integer;
  RecordLength : Integer;
  Name,Value : String;
>>>>>>> graemeg/cpstrnew
=======
  NameLength, ValueLength : Integer;
  RecordLength : Integer;
  Name,Value : String;
>>>>>>> origin/cpstrnew

begin
  i := 0;
  RecordLength:=BetoN(ARecord^.Header.contentLength);
  while i < RecordLength do
    begin
    NameLength:=GetVarLength;
    ValueLength:=GetVarLength;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Name:=GetString(NameLength);
    Value:=GetString(ValueLength);
    VarNo:=IndexOfCGIVar(Name);
    if Not DoMapCgiToHTTP(Name,H,V) then
      NameValueList.Add(Name+'='+Value)
    else if (H<>hhUnknown) then
      SetHeader(H,Value)
    else if (v<>hvUnknown) then
      begin
      if (V=hvPathInfo) and (Copy(Value,1,2)='//') then //mod_proxy_fcgi gives double slashes at the beginning for some reason
          Delete(Value,1,3);
      if (V<>hvQuery) then
        Value:=HTTPDecode(Value);
      SetHTTPVariable(v,Value);
      end
    else
      NameValueList.Add(Name+'='+Value)
    end;
end;

procedure TFCGIRequest.Log(EventType: TEventType; const Msg: String);
begin
  If Assigned(FLog) then
    FLog(EventType,Msg);
end;

{ TCGIResponse }

procedure TFCGIResponse.Write_FCGIRecord(ARecord : PFCGI_Header);

var ErrorCode,
    BytesToWrite ,
    BytesWritten  : Integer;
    P : PByte;
    r : TFCGIRequest;
    
begin
  if Not (Request is TFCGIRequest) then
    TFCgiHandler.DoError(SErrNorequest);
  R:=TFCGIRequest(Request);
  BytesToWrite := BEtoN(ARecord^.contentLength) + ARecord^.paddingLength+sizeof(FCGI_Header);
  P:=PByte(Arecord);
  Repeat
    BytesWritten:=FOnWrite(R.Handle, P^, BytesToWrite,ErrorCode);
    If (BytesWritten<0) then
      begin
      // TODO : Better checking on ErrorCode
      R.FKeepConnectionAfterRequest:=False;

{$ifdef windowspipe}
      case ErrorCode of
        ERROR_BROKEN_PIPE, ERROR_NO_DATA : Exit; //No error here. Server cancel pipe
      end;
{$endif}

      TFCgiHandler.DoError(SErrWritingSocket,[ErrorCode]);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    Name:=GetString(NameLength);
    Value:=GetString(ValueLength);
    NameValueList.Add(Name+'='+Value);
    end;
end;


Function TFCGIRequest.GetFieldValue(Index : Integer) : String;

Type THttpToCGI = array[1..CGIVarCount] of byte;

const HttpToCGI : THttpToCGI =
   (
     18,  //  1 'HTTP_ACCEPT'           - fieldAccept
     19,  //  2 'HTTP_ACCEPT_CHARSET'   - fieldAcceptCharset
     20,  //  3 'HTTP_ACCEPT_ENCODING'  - fieldAcceptEncoding
      0,  //  4
      0,  //  5
      0,  //  6
      0,  //  7
      0,  //  8
      2,  //  9 'CONTENT_LENGTH'
      3,  // 10 'CONTENT_TYPE'          - fieldAcceptEncoding
     24,  // 11 'HTTP_COOKIE'           - fieldCookie
      0,  // 12
      0,  // 13
      0,  // 14
     21,  // 15 'HTTP_IF_MODIFIED_SINCE'- fieldIfModifiedSince
      0,  // 16
      0,  // 17
      0,  // 18
     22,  // 19 'HTTP_REFERER'          - fieldReferer
      0,  // 20
      0,  // 21
      0,  // 22
     23,  // 23 'HTTP_USER_AGENT'       - fieldUserAgent
      1,  // 24 'AUTH_TYPE'             - fieldWWWAuthenticate
      5,  // 25 'PATH_INFO'
      6,  // 26 'PATH_TRANSLATED'
      8,  // 27 'REMOTE_ADDR'
      9,  // 28 'REMOTE_HOST'
     13,  // 29 'SCRIPT_NAME'
     15,  // 30 'SERVER_PORT'
     12,  // 31 'REQUEST_METHOD'
      0,  // 32
      7,  // 33 'QUERY_STRING'
     27,  // 34 'HTTP_HOST'
      0,  // 35 'CONTENT'
     36   // 36 'XHTTPREQUESTEDWITH'
    );

var ACgiVarNr : Integer;

begin
  Result := '';
  if assigned(FCGIParams) and (index < high(HttpToCGI)) and (index > 0) and (index<>35) then
    begin
    ACgiVarNr:=HttpToCGI[Index];
    if ACgiVarNr>0 then
      Result:=FCGIParams.Values[CgiVarNames[ACgiVarNr]]
    else
      Result := '';
    end
  else
    Result:=inherited GetFieldValue(Index);
end;

{ TCGIResponse }
procedure TFCGIResponse.Write_FCGIRecord(ARecord : PFCGI_Header);

var BytesToWrite : Integer;
    BytesWritten  : Integer;
    P : PByte;
begin
  BytesToWrite := BEtoN(ARecord^.contentLength) + ARecord^.paddingLength+sizeof(FCGI_Header);
  P:=PByte(Arecord);
  Repeat
    BytesWritten := sockets.fpsend(TFCGIRequest(Request).Handle, P, BytesToWrite, NoSignalAttr);
    If (BytesWritten<0) then
      begin
      // TODO : Better checking for closed connection, EINTR
      Raise HTTPError.CreateFmt(SErrWritingSocket,[BytesWritten]);
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
    Inc(P,BytesWritten);
    Dec(BytesToWrite,BytesWritten);
  until (BytesToWrite=0) or (BytesWritten=0);
end;

procedure TFCGIResponse.DoSendHeaders(Headers : TStrings);
var
  cl : word;
  pl : byte;
  str : String;
  ARespRecord : PFCGI_ContentRecord;
  I : Integer;

begin
  For I:=Headers.Count-1 downto 0 do
    If (Headers[i]='') then
      Headers.Delete(I);
  // IndexOfName Does not work ?
  If (poStripContentLength in ProtocolOptions) then
    For I:=Headers.Count-1 downto 0 do
      If (Pos('Content-Length',Headers[i])<>0)  then
        Headers.Delete(i);
  str := Headers.Text+sLineBreak;
  cl := length(str);
  if ((cl mod 8)=0) or (poNoPadding in ProtocolOptions) then
    pl:=0
  else
    pl := 8-(cl mod 8);
  ARespRecord:=nil;
  Getmem(ARespRecord,8+cl+pl);
  try
    FillChar(ARespRecord^,8+cl+pl,0);
    ARespRecord^.header.version:=FCGI_VERSION_1;
    ARespRecord^.header.reqtype:=FCGI_STDOUT;
    ARespRecord^.header.paddingLength:=pl;
    ARespRecord^.header.contentLength:=NtoBE(cl);
    ARespRecord^.header.requestId:=NToBE(TFCGIRequest(Request).RequestID);
    move(str[1],ARespRecord^.ContentData,cl);
    Write_FCGIRecord(PFCGI_Header(ARespRecord));
  finally
    Freemem(ARespRecord);
  end;
end;

procedure TFCGIResponse.DoSendContent;

Const
  MaxBuf = $EFFF;

var
  bs,l : Integer;
  cl : word;
  pl : byte;
  str : String;
  ARespRecord : PFCGI_ContentRecord;
  EndRequest : FCGI_EndRequestRecord;

begin
  If Assigned(ContentStream) then
    begin
    setlength(str,ContentStream.Size);
    ContentStream.Position:=0;
    ContentStream.Read(str[1],ContentStream.Size);
    end
  else
    str := Contents.Text;
  L:=Length(Str);
  BS:=0;
  Repeat
    If (L-BS)>MaxBuf then
      cl := MaxBuf
    else
      cl:=L-BS ;
    if ((cl mod 8)=0) or (poNoPadding in ProtocolOptions) then
      pl:=0
    else
      pl := 8-(cl mod 8);
    ARespRecord:=Nil;
    Getmem(ARespRecord,8+cl+pl);
    try
      ARespRecord^.header.version:=FCGI_VERSION_1;
      ARespRecord^.header.reqtype:=FCGI_STDOUT;
      ARespRecord^.header.paddingLength:=pl;
      ARespRecord^.header.contentLength:=NtoBE(cl);
      ARespRecord^.header.requestId:=NToBE(TFCGIRequest(Request).RequestID);
      move(Str[BS+1],ARespRecord^.ContentData,cl);
      Write_FCGIRecord(PFCGI_Header(ARespRecord));
    finally
      Freemem(ARespRecord);
    end;
    Inc(BS,cl);
  Until (BS=L);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  EndRequest := Default(FCGI_EndRequestRecord);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FillChar(EndRequest,SizeOf(FCGI_EndRequestRecord),0);
  EndRequest.header.version:=FCGI_VERSION_1;
  EndRequest.header.reqtype:=FCGI_END_REQUEST;
  EndRequest.header.contentLength:=NtoBE(8);
  EndRequest.header.paddingLength:=0;
  EndRequest.header.requestId:=NToBE(TFCGIRequest(Request).RequestID);
  EndRequest.body.protocolStatus:=FCGI_REQUEST_COMPLETE;
  Write_FCGIRecord(PFCGI_Header(@EndRequest));
end;

{ TFCgiHandler }

constructor TFCgiHandler.Create(AOwner: TComponent);
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
begin
  Inherited Create(AOwner);
  FRequestsAvail:=5;
  SetLength(FRequestsArray,FRequestsAvail);
  FHandle := THandle(-1);
  FTimeOut:=50;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF WINDOWS}
  CheckShutdownEvent;
{$ENDIF}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

destructor TFCgiHandler.Destroy;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF WINDOWS}
  IF (FShutDownThread<>Nil) then
    begin
    TShutDownThread(FShutDownThread).FOnShutDown:=Nil;
    TShutDownThread(FShutDownThread).OnTerminate:=Nil;
    end;
{$ENDIF}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SetLength(FRequestsArray,0);
  if (Socket<>0) then
    begin
    CloseSocket(Socket);
    Socket:=0;
    end;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF WINDOWS}
Procedure TFCgiHandler.CheckShutdownEvent;

Var
  H : THandle;

begin
  // This is normally only used in mod_fastcgi.
  // mod_fcgid just kills off the process...
  H:=THandle(StrToIntDef(sysutils.GetEnvironmentVariable('_FCGI_SHUTDOWN_EVENT_'),0));
  If (H<>0) then
    FShutDownThread:=TShutdownThread.CreateWithEvent(H,@HandleShutDownEvent);
end;

procedure TFCgiHandler.HandleShutDownEvent(Sender : TOBject);
begin
  TShutDownThread(Sender).FOnShutDown:=Nil;
  TShutDownThread(Sender).OnTerminate:=Nil;
  FShutDownThread:=Nil;
  Terminate;
end;

{$ENDIF}

procedure TFCgiHandler.CloseConnection;
Var
  i : Integer;
begin
{$ifdef windowspipe}
  if FIsWinPipe then
    begin
    if not FlushFileBuffers(FHandle) then
      begin
      I:=GetLastError;
      Log(etError,Format('Failed to flush file buffers: %d ',[i]));
      end;
    if not DisconnectNamedPipe(FHandle) then
      begin
      I:=GetLastError;
      Log(etError,Format('Failed to disconnect named pipe: %d ',[i]));
      end
    end
  else
{$endif}
    begin
    i:=fpshutdown(FHandle,SHUT_RDWR);
//      Log(etError,Format('Shutting down socket: %d ',[i]));
    i:=CloseSocket(FHandle);
//      Log(etError,Format('Closing socket %d',[i]));
    end;
  FHandle := THandle(-1);
end;

procedure TFCgiHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);


=======
procedure TFCgiHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> graemeg/cpstrnew
=======
procedure TFCgiHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> graemeg/cpstrnew
=======
procedure TFCgiHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> graemeg/cpstrnew
=======
procedure TFCgiHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> origin/cpstrnew
begin
  with FRequestsArray[TFCGIRequest(ARequest).RequestID] do
    begin
    Assert(ARequest=Request);
    Assert(AResponse=Response);
    if (not TFCGIRequest(ARequest).KeepConnectionAfterRequest) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      CloseConnection;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
      fpshutdown(FHandle,SHUT_RDWR);
      CloseSocket(FHandle);
      FHandle := THandle(-1);
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
    Request := Nil;
    Response := Nil;
    end;
  Inherited;
end;

function TFCgiHandler.Read_FCGIRecord : PFCGI_Header;
{ $DEFINE DUMPRECORD}
{$IFDEF DUMPRECORD}
  Procedure DumpFCGIRecord (Var Header :FCGI_Header; ContentLength : word; PaddingLength : byte; ResRecord : Pointer);

  Var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    S, s1, s2 : string;
    I : Integer;

  begin
      Writeln(Format('Dumping record, Sizeof(Header)=%d, ContentLength=%d, PaddingLength=%d',[SizeOf(Header),ContentLength,PaddingLength]));
      S:=''; s1 := '';
      For I:=0 to Sizeof(Header)+ContentLength+PaddingLength-1 do
      begin
        s2 := Format('%:2X ',[PByte(ResRecord)[i]]);
        if s2[1] = ' ' then s2[1] := '0';
        s1 := s1 + s2;
        If PByte(ResRecord)[i]>32 then
          S:=S+char(PByte(ResRecord)[i])
        else
          S:=S+' ';
        if (I>0) and (((I+1) mod 16) = 0) then
        begin
           Writeln(s1 + '  ' + S);
           S:=''; s1 := '';
        end;
      end;
      if length(s1)<48 then
        repeat s1 := s1 + ' ' until length(s1)>=48;
      Writeln(s1 + '  '+S)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    s : string;
    I : Integer;

  begin
      Writeln('Dumping record ', Sizeof(Header),',',Contentlength,',',PaddingLength);
      For I:=0 to Sizeof(Header)+ContentLength+PaddingLength-1 do
        begin
        Write(Format('%:3d ',[PByte(ResRecord)[i]]));
        If PByte(ResRecord)[i]>30 then
          S:=S+char(PByte(ResRecord)[i]);
        if (I mod 16) = 0 then
           begin
           writeln('  ',S);
           S:='';
           end;
        end;
      Writeln('  ',S)
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
{$ENDIF DUMPRECORD}

  function ReadBytes(ReadBuf: Pointer; ByteAmount : Word) : Integer;

  Var
    P : PByte;
    Count : Integer;

  begin
    Result := 0;
    P:=ReadBuf;
    if (ByteAmount=0) then exit;
    Repeat
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Count:=DoFastCGIRead(FHandle,P^,ByteAmount);
=======
      Count:=sockets.fpRecv(FHandle, P, ByteAmount, NoSignalAttr);
>>>>>>> graemeg/cpstrnew
=======
      Count:=sockets.fpRecv(FHandle, P, ByteAmount, NoSignalAttr);
>>>>>>> graemeg/cpstrnew
=======
      Count:=sockets.fpRecv(FHandle, P, ByteAmount, NoSignalAttr);
>>>>>>> graemeg/cpstrnew
=======
      Count:=sockets.fpRecv(FHandle, P, ByteAmount, NoSignalAttr);
>>>>>>> origin/cpstrnew
      If (Count>0) then
        begin
        Dec(ByteAmount,Count);
        P:=P+Count;
        Inc(Result,Count);
        end
      else if (Count<0) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        DoError(SErrReadingSocket,[Count]);
=======
        Raise HTTPError.CreateFmt(SErrReadingSocket,[Count]);
>>>>>>> graemeg/cpstrnew
=======
        Raise HTTPError.CreateFmt(SErrReadingSocket,[Count]);
>>>>>>> graemeg/cpstrnew
=======
        Raise HTTPError.CreateFmt(SErrReadingSocket,[Count]);
>>>>>>> graemeg/cpstrnew
=======
        Raise HTTPError.CreateFmt(SErrReadingSocket,[Count]);
>>>>>>> origin/cpstrnew
    until (ByteAmount=0) or (Count=0);
  end;

var Header : FCGI_Header;
    BytesRead : integer;
    ContentLength : word;
    PaddingLength : byte;
    ResRecord : pointer;
    ReadBuf : pointer;


begin
  Result := Nil;
  ResRecord:=Nil;
  ReadBuf:=@Header;
  BytesRead:=ReadBytes(ReadBuf,Sizeof(Header));
  If (BytesRead=0) then
    Exit // Connection closed gracefully.
    // TODO : if connection closed gracefully, the request should no longer be handled.
    // Need to discard request/response
  else If (BytesRead<>Sizeof(Header)) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    DoError(SErrReadingHeader,[BytesRead]);
=======
    Raise HTTPError.CreateFmt(SErrReadingHeader,[BytesRead]);
>>>>>>> graemeg/cpstrnew
=======
    Raise HTTPError.CreateFmt(SErrReadingHeader,[BytesRead]);
>>>>>>> graemeg/cpstrnew
=======
    Raise HTTPError.CreateFmt(SErrReadingHeader,[BytesRead]);
>>>>>>> graemeg/cpstrnew
=======
    Raise HTTPError.CreateFmt(SErrReadingHeader,[BytesRead]);
>>>>>>> origin/cpstrnew
  ContentLength:=BetoN(Header.contentLength);
  PaddingLength:=Header.paddingLength;
  Getmem(ResRecord,BytesRead+ContentLength+PaddingLength);
  try
    PFCGI_Header(ResRecord)^:=Header;
    ReadBuf:=ResRecord+BytesRead;
    BytesRead:=ReadBytes(ReadBuf,ContentLength);
    If (BytesRead=0) and (ContentLength>0) then
      begin
      FreeMem(resRecord);
      Exit // Connection closed gracefully.
      // TODO : properly handle connection close
      end;
    ReadBuf:=ReadBuf+BytesRead;
    BytesRead:=ReadBytes(ReadBuf,PaddingLength);
    If (BytesRead=0) and (PaddingLength>0) then
      begin
      FreeMem(resRecord);
      Exit // Connection closed gracefully.
      // TODO : properly handle connection close
      end;
    Result := ResRecord;
  except
    FreeMem(resRecord);
    Raise;
  end;
end;

procedure TFCgiHandler.SetupSocket(var IAddress : TInetSockAddr; Var AddressLength : tsocklen);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Var
  L : Linger;
  ll,lr : integer;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  AddressLength:=Sizeof(IAddress);
  Socket := fpsocket(AF_INET,SOCK_STREAM,0);
  if Socket=-1 then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    DoError(SNoSocket,[socketerror]);
=======
    raise EFPWebError.CreateFmt(SNoSocket,[socketerror]);
>>>>>>> graemeg/cpstrnew
=======
    raise EFPWebError.CreateFmt(SNoSocket,[socketerror]);
>>>>>>> graemeg/cpstrnew
=======
    raise EFPWebError.CreateFmt(SNoSocket,[socketerror]);
>>>>>>> graemeg/cpstrnew
=======
    raise EFPWebError.CreateFmt(SNoSocket,[socketerror]);
>>>>>>> origin/cpstrnew
  IAddress.sin_family:=AF_INET;
  IAddress.sin_port:=htons(Port);
  if FAddress<>'' then
    Iaddress.sin_addr := StrToHostAddr(FAddress)
  else
    IAddress.sin_addr.s_addr:=0;
    {$IFDEF Unix}
    // remedy socket port locking on Posix platforms
    If (poReuseAddress in ProtocolOptions) then
      fpSetSockOpt(Socket, SOL_SOCKET, SO_REUSEADDR, @IAddress, SizeOf(IAddress));
    {$ENDIF}
  if fpbind(Socket,@IAddress,AddressLength)=-1 then
    begin
    CloseSocket(socket);
    Socket:=0;
    Terminate;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    DoError(SBindFailed,[port,socketerror]);
    end;
  if (FLingerTimeout>0) then
    begin
    ll:=SizeOf(l);
    if fpgetsockopt(Socket,SOL_SOCKET,SO_LINGER,@l,@ll)=0 then
      begin
//      Log(etDebug,Format('Socket linger : %d, %d',[L.l_linger,L.l_onoff]));
      if (L.l_onoff=0) then
        begin
        l.l_onoff:=1;
        l.l_linger:=1;
        lr:=fpsetsockopt(Socket,SOL_SOCKET,SO_LINGER,@l,ll);
        if (lr<>0) then
          Log(etError,Format('Set socket linger failed : %d',[lr]));
        end;
      end;
    end;
  if fplisten(Socket,10)=-1 then
=======
    raise Exception.CreateFmt(SBindFailed,[port,socketerror]);
    end;
  if fplisten(Socket,1)=-1 then
>>>>>>> graemeg/cpstrnew
=======
    raise Exception.CreateFmt(SBindFailed,[port,socketerror]);
    end;
  if fplisten(Socket,1)=-1 then
>>>>>>> graemeg/cpstrnew
=======
    raise Exception.CreateFmt(SBindFailed,[port,socketerror]);
    end;
  if fplisten(Socket,1)=-1 then
>>>>>>> graemeg/cpstrnew
=======
    raise Exception.CreateFmt(SBindFailed,[port,socketerror]);
    end;
  if fplisten(Socket,1)=-1 then
>>>>>>> origin/cpstrnew
    begin
    CloseSocket(socket);
    Socket:=0;
    Terminate;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    DoError(SListenFailed,[port,socketerror]);
=======
    raise Exception.CreateFmt(SListenFailed,[port,socketerror]);
>>>>>>> graemeg/cpstrnew
=======
    raise Exception.CreateFmt(SListenFailed,[port,socketerror]);
>>>>>>> graemeg/cpstrnew
=======
    raise Exception.CreateFmt(SListenFailed,[port,socketerror]);
>>>>>>> graemeg/cpstrnew
=======
    raise Exception.CreateFmt(SListenFailed,[port,socketerror]);
>>>>>>> origin/cpstrnew
    end;
end;

{$ifdef unix}
function TFCgiHandler.DataAvailable: Boolean;

var
  FDS: TFDSet;
  TimeV: TTimeVal;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FDS := Default(TFDSet);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  fpFD_Zero(FDS);
  fpFD_Set(FHandle, FDS);
  TimeV.tv_usec := (Timeout mod 1000) * 1000;
  TimeV.tv_sec := Timeout div 1000;
  Result := fpSelect(FHandle + 1, @FDS, @FDS, @FDS, @TimeV) > 0;
end;
{$else}
function TFCgiHandler.DataAvailable: Boolean;

var
  FDS: TFDSet;
  TimeV: TTimeVal;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FDS := Default(TFDSet);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FD_Zero(FDS);
  FD_Set(FHandle, FDS);
  TimeV.tv_usec := (Timeout mod 1000) * 1000;
  TimeV.tv_sec := Timeout div 1000;
  Result := Select(FHandle + 1, @FDS, @FDS, @FDS, @TimeV) <> 0;
end;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TFCgiHandler.CreateRequest: TFCGIRequest;

Var
  C : TFCGIRequestClass;

begin
  C:=FCGIRequestClass;
  if (C=Nil) then
    C:=TFCGIRequest;
  Result:=C.Create;
end;

function TFCgiHandler.CreateResponse(ARequest: TFCGIRequest): TFCGIResponse;
Var
  C : TFCGIResponseClass;

begin
  C:=FCGIResponseClass;
  if (C=Nil) then
    C:=TFCGIResponse;
  Result := C.Create(ARequest);
end;

function TFCgiHandler.DoFastCGIRead(AHandle: THandle; var ABuf; ACount: Integer): Integer;
begin
{$ifdef windowspipe}
  if FIsWinPipe then
    Result:=FileRead(AHandle,ABuf,ACount)
  else
{$endif}
    Result:=sockets.fpRecv(AHandle, @Abuf, ACount, NoSignalAttr);
end;

function TFCgiHandler.DoFastCGIWrite(AHandle: THandle; const ABuf;
  ACount: Integer; Out ExtendedErrorCode : Integer): Integer;
begin
  {$ifdef windowspipe}
  if FIsWinPipe then
    begin
    ExtendedErrorCode:=0;
    Result := FileWrite(AHandle, ABuf, ACount);
    if (Result<0) then
      ExtendedErrorCode:=GetLastOSError;
    end
  else
  {$endif windows}
    begin
    Repeat
      ExtendedErrorCode:=0;
      Result:=sockets.fpsend(AHandle, @ABuf, ACount, NoSignalAttr);
      if (Result<0) then
        ExtendedErrorCode:=sockets.socketerror;
    until (Result>=0) {$ifdef unix} or (ExtendedErrorCode<>ESysEINTR);{$endif}
    end;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TFCgiHandler.ProcessRecord(AFCGI_Record  : PFCGI_Header; out ARequest: TRequest; out AResponse: TResponse): boolean;

var
  ARequestID    : word;
  ATempRequest  : TFCGIRequest;
begin
  Result:=False;
  ARequestID:=BEtoN(AFCGI_Record^.requestID);
  if AFCGI_Record^.reqtype = FCGI_BEGIN_REQUEST then
    begin
    if ARequestID>FRequestsAvail then
      begin
      inc(FRequestsAvail,10);
      SetLength(FRequestsArray,FRequestsAvail);
      end;
    assert(not assigned(FRequestsArray[ARequestID].Request));
    assert(not assigned(FRequestsArray[ARequestID].Response));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ATempRequest:=CreateRequest;
    InitRequest(ATempRequest);
=======
    ATempRequest:=TFCGIRequest.Create;
>>>>>>> graemeg/cpstrnew
=======
    ATempRequest:=TFCGIRequest.Create;
>>>>>>> graemeg/cpstrnew
=======
    ATempRequest:=TFCGIRequest.Create;
>>>>>>> graemeg/cpstrnew
=======
    ATempRequest:=TFCGIRequest.Create;
>>>>>>> origin/cpstrnew
    ATempRequest.RequestID:=ARequestID;
    ATempRequest.Handle:=FHandle;
    ATempRequest.ProtocolOptions:=Self.Protocoloptions;
    ATempRequest.OnUnknownRecord:=Self.OnUnknownRecord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ATempRequest.FLog:=@Log;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FRequestsArray[ARequestID].Request := ATempRequest;
    end;
  if (ARequestID>FRequestsAvail) then
    begin
    // TODO : ARequestID can be invalid. What to do ?
    // in each case not try to access the array with requests.
    end
  else if FRequestsArray[ARequestID].Request.ProcessFCGIRecord(AFCGI_Record) then
    begin
    ARequest:=FRequestsArray[ARequestID].Request;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FRequestsArray[ARequestID].Response := CreateResponse(TFCGIRequest(ARequest));
    InitResponse(FRequestsArray[ARequestID].Response);
    FRequestsArray[ARequestID].Response.ProtocolOptions:=Self.ProtocolOptions;
    FRequestsArray[ARequestID].Response.FOnWrite:=@DoFastCGIWrite;
=======
    FRequestsArray[ARequestID].Response := TFCGIResponse.Create(ARequest);
    FRequestsArray[ARequestID].Response.ProtocolOptions:=Self.ProtocolOptions;
>>>>>>> graemeg/cpstrnew
=======
    FRequestsArray[ARequestID].Response := TFCGIResponse.Create(ARequest);
    FRequestsArray[ARequestID].Response.ProtocolOptions:=Self.ProtocolOptions;
>>>>>>> graemeg/cpstrnew
=======
    FRequestsArray[ARequestID].Response := TFCGIResponse.Create(ARequest);
    FRequestsArray[ARequestID].Response.ProtocolOptions:=Self.ProtocolOptions;
>>>>>>> graemeg/cpstrnew
=======
    FRequestsArray[ARequestID].Response := TFCGIResponse.Create(ARequest);
    FRequestsArray[ARequestID].Response.ProtocolOptions:=Self.ProtocolOptions;
>>>>>>> origin/cpstrnew
    AResponse:=FRequestsArray[ARequestID].Response;
    Result := True;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TFCgiHandler.AcceptConnection : Integer;

{$ifdef windows}
Var
  B : BOOL;
  pipeMode : DWORD = PIPE_READMODE_BYTE or PIPE_WAIT;
  i : integer;
{$endif}

begin
{$ifndef windows}
  repeat
    Result:=fpaccept(Socket,nil,nil);
  until ((result<>-1) or (SocketError<>ESysEINTR)) and not Terminated;
{$else}
{$ifndef windowspipe}
  Result:=fpaccept(Socket,Nil,nil);
{$else windowspipe}
  if Not fIsWinPipe then
    Result:=fpaccept(Socket,Nil,Nil);
  If FIsWinPipe or ((Result<0) and ((socketerror=10038) or (socketerror = 10022))) then
    begin
    Result:=-1;
    B:=ConnectNamedPipe(Socket,Nil);
    if B or (GetLastError=ERROR_PIPE_CONNECTED) then
       begin
       Result:=Socket;
       if Not FIsWinPipe then // First time, set handle state
         if not SetNamedPipeHandleState(Result,@PipeMode,Nil,Nil) then
           begin
           I:=GetLastError;
           Log(etError,'Setting named pipe handle state failed : '+intToStr(i));
           end;
       FIsWinPipe:=True;
       end;
    end;
{$endif}
{$endif}
end;

function TFCgiHandler.WaitForRequest(out ARequest: TRequest; out AResponse: TResponse): boolean;

var
  AFCGI_Record  : PFCGI_Header;


begin
  Result := False;
  AResponse:=Nil;
  ARequest:=Nil;
  if Socket=0 then
    if Port<>0 then
      SetupSocket(FIAddress,FAddressLength)
    else
      Socket:=StdInputHandle;
  Repeat
    if FHandle=THandle(-1) then
      FHandle:=AcceptConnection;
    if FHandle=THandle(-1) then
      begin
      if not terminated then
        begin
        Terminate;
        DoError(SNoInputHandle,[socketerror]);
        end
      end;
    repeat
      If (poUseSelect in ProtocolOptions) then
        begin
        While Not DataAvailable do
          If (OnIdle<>Nil) then
            OnIdle(Self);
        end;
      AFCGI_Record:=Read_FCGIRecord;
      // If connection closed gracefully, we have nil.
      if Not Assigned(AFCGI_Record) then
        CloseConnection
      else
        try
        Result:=ProcessRecord(AFCGI_Record,ARequest,AResponse);
        Finally
          FreeMem(AFCGI_Record);
          AFCGI_Record:=Nil;
        end;
    until Result or (FHandle=THandle(-1));
  Until Result;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TFCgiHandler.WaitForRequest(out ARequest: TRequest; out AResponse: TResponse): boolean;

var
  IAddress      : TInetSockAddr;
  AddressLength : tsocklen;
  AFCGI_Record  : PFCGI_Header;

begin
  Result := False;
  if Socket=0 then
    if Port<>0 then
      SetupSocket(IAddress,AddressLength)
    else
      Socket:=StdInputHandle;
  if FHandle=THandle(-1) then
    begin
    FHandle:=fpaccept(Socket,psockaddr(@IAddress),@AddressLength);
    if FHandle=THandle(-1) then
      begin
      Terminate;
      raise Exception.CreateFmt(SNoInputHandle,[socketerror]);
      end;
    end;
  repeat
    If (poUseSelect in ProtocolOptions) then
      begin
      While Not DataAvailable do
        If (OnIdle<>Nil) then
          OnIdle(Self);
      end;
    AFCGI_Record:=Read_FCGIRecord;

    if assigned(AFCGI_Record) then
    try
      Result:=ProcessRecord(AFCGI_Record,ARequest,AResponse);
    Finally
      FreeMem(AFCGI_Record);
      AFCGI_Record:=Nil;
    end;
  until Result;
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

{ TCustomFCgiApplication }

function TCustomFCgiApplication.GetAddress: string;
begin
  result := TFCgiHandler(WebHandler).Address;
end;

function TCustomFCgiApplication.GetFPO: TProtoColOptions;
begin
  result := TFCgiHandler(WebHandler).ProtocolOptions;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomFCgiApplication.GetLingerTimeOut: integer;
begin
  Result:=TFCgiHandler(WebHandler).LingerTimeOut;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomFCgiApplication.GetOnUnknownRecord: TUnknownRecordEvent;
begin
  result := TFCgiHandler(WebHandler).OnUnknownRecord;
end;

function TCustomFCgiApplication.GetPort: integer;
begin
  result := TFCgiHandler(WebHandler).Port;
end;

procedure TCustomFCgiApplication.SetAddress(const AValue: string);
begin
  TFCgiHandler(WebHandler).Address := AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomFCgiApplication.SetLingerTimeOut(const AValue: integer);
begin
  TFCgiHandler(WebHandler).LingerTimeOut:=AValue;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomFCgiApplication.SetOnUnknownRecord(const AValue: TUnknownRecordEvent);
begin
  TFCgiHandler(WebHandler).OnUnknownRecord := AValue;
end;

procedure TCustomFCgiApplication.SetPort(const AValue: integer);
begin
  TFCgiHandler(WebHandler).Port := AValue;
end;

procedure TCustomFCgiApplication.SetPO(const AValue: TProtoColOptions);
begin
  TFCgiHandler(WebHandler).ProtocolOptions := AValue;
end;

function TCustomFCgiApplication.InitializeWebHandler: TWebHandler;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Var
  C : TFCGIHandlerClass;

begin
  C:=FCGIWebHandlerClass;
  If C=Nil then
    C:=TFCgiHandler;
  Result:=C.Create(self);
=======
begin
  Result:=TFCgiHandler.Create(self);
>>>>>>> graemeg/cpstrnew
=======
begin
  Result:=TFCgiHandler.Create(self);
>>>>>>> graemeg/cpstrnew
=======
begin
  Result:=TFCgiHandler.Create(self);
>>>>>>> graemeg/cpstrnew
=======
begin
  Result:=TFCgiHandler.Create(self);
>>>>>>> origin/cpstrnew
end;

end.
