{

    HTTPBase: Common HTTP utility declarations and classes
    Copyright (C) 2000-2003 by Sebastian Guenther (sg@freepascal.org)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{$mode objfpc}
{$H+}

unit HTTPBase;

interface


uses Classes, fpAsync;

const

  fieldAccept = 'Accept';
  fieldAcceptCharset = 'Accept-Charset';
  fieldAcceptEncoding = 'Accept-Encoding';
  fieldAcceptLanguage = 'Accept-Language';
  fieldAuthorization = 'Authorization';
  fieldConnection = 'Connection';
  fieldContentEncoding = 'Content-Encoding';
  fieldContentLanguage = 'Content-Language';
  fieldContentLength = 'Content-Length';
  fieldContentType = 'Content-Type';
  fieldCookie = 'Cookie';
  fieldDate = 'Date';
  fieldExpires = 'Expires';
  fieldFrom = 'From';
  fieldIfModifiedSince = 'If-Modified-Since';
  fieldLastModified = 'Last-Modified';
  fieldLocation = 'Location';
  fieldPragma = 'Pragma';
  fieldReferer = 'Referer';
  fieldRetryAfter = 'Retry-After';
  fieldServer = 'Server';
  fieldSetCookie = 'Set-Cookie';
  fieldUserAgent = 'User-Agent';
  fieldWWWAuthenticate = 'WWW-Authenticate';

type

  PHttpField = ^THttpField;
  THttpField = record
    Name, Value: String;
  end;


  THttpHeader = class
  protected
    FReader: TAsyncStreamLineReader;
    FWriter: TAsyncWriteStream;
    FOnCompleted: TNotifyEvent;
    FOnEOF: TNotifyEvent;
    FFields: TList;

    procedure ParseFirstHeaderLine(const line: String); virtual; abstract;
    procedure LineReceived(const ALine: String);
    procedure ReaderEOF(Sender: TObject);
    function GetFirstHeaderLine: String; virtual; abstract;
    procedure WriterCompleted(ASender: TObject);

    function GetFieldCount: Integer;
    function GetFields(AIndex: Integer): String;
    function GetFieldNames(AIndex: Integer): String;
    procedure SetFieldNames(AIndex: Integer; const AName: String);
    function GetFieldValues(AIndex: Integer): String;
    procedure SetFieldValues(AIndex: Integer; const AValue: String);


    function  GetAccept: String;
    procedure SetAccept(const AValue: String);
    function  GetAcceptCharset: String;
    procedure SetAcceptCharset(const AValue: String);
    function  GetAcceptEncoding: String;
    procedure SetAcceptEncoding(const AValue: String);
    function  GetAcceptLanguage: String;
    procedure SetAcceptLanguage(const AValue: String);
    function  GetAuthorization: String;
    procedure SetAuthorization(const AValue: String);
    function  GetConnection: String;
    procedure SetConnection(const AValue: String);
    function  GetContentEncoding: String;
    procedure SetContentEncoding(const AValue: String);
    function  GetContentLanguage: String;
    procedure SetContentLanguage(const AValue: String);
    function  GetContentLength: Integer;
    procedure SetContentLength(AValue: Integer);
    function  GetContentType: String;
    procedure SetContentType(const AValue: String);
    function  Get_Cookie: String;
    procedure Set_Cookie(const AValue: String);
    function  GetDate: String;
    procedure SetDate(const AValue: String);
    function  GetExpires: String;
    procedure SetExpires(const AValue: String);
    function  GetFrom: String;
    procedure SetFrom(const AValue: String);
    function  GetIfModifiedSince: String;
    procedure SetIfModifiedSince(const AValue: String);
    function  GetLastModified: String;
    procedure SetLastModified(const AValue: String);
    function  GetLocation: String;
    procedure SetLocation(const AValue: String);
    function  GetPragma: String;
    procedure SetPragma(const AValue: String);
    function  GetReferer: String;
    procedure SetReferer(const AValue: String);
    function  GetRetryAfter: String;
    procedure SetRetryAfter(const AValue: String);
    function  GetServer: String;
    procedure SetServer(const AValue: String);
    function  Get_SetCookie: String;
    procedure Set_SetCookie(const AValue: String);
    function  GetUserAgent: String;
    procedure SetUserAgent(const AValue: String);
    function  GetWWWAuthenticate: String;
    procedure SetWWWAuthenticate(const AValue: String);

  public
    DataReceived, CmdReceived: Boolean;         // !!!: Only temporarily here

    HttpVersion: String;

    constructor Create;
    destructor Destroy; override;
    procedure SetFieldByName(const AName, AValue: String);
    function  GetFieldByName(const AName: String): String;

    procedure AsyncSend(AManager: TEventLoop; AStream: THandleStream);
    procedure AsyncReceive(AManager: TEventLoop; AStream: THandleStream);

    property Reader: TAsyncStreamLineReader read FReader;
    property Writer: TAsyncWriteStream read FWriter;
    property FieldCount: Integer read GetFieldCount;
    property Fields[AIndex: Integer]: String read GetFields;
    property FieldNames[AIndex: Integer]: String read GetFieldNames write SetFieldNames;
    property FieldValues[AIndex: Integer]: String read GetFieldValues write SetFieldValues;

    property OnCompleted: TNotifyEvent read FOnCompleted write FOnCompleted;
    property OnEOF: TNotifyEvent read FOnEOF write FOnEOF;

    property Accept: String read GetAccept write SetAccept;
    property AcceptCharset: String read GetAcceptCharset write SetAcceptCharset;
    property AcceptEncoding: String read GetAcceptEncoding write SetAcceptEncoding;
    property AcceptLanguage: String read GetAcceptLanguage write SetAcceptLanguage;
    property Authorization: String read GetAuthorization write SetAuthorization;
    property Connection: String read GetConnection write SetConnection;
    property ContentEncoding: String read GetContentEncoding write SetContentEncoding;
    property ContentLanguage: String read GetContentLanguage write SetContentLanguage;
    property ContentLength: Integer read GetContentLength write SetContentLength;
    property ContentType: String read GetContentType write SetContentType;
    property Cookie: String read Get_Cookie write Set_Cookie;
    property Date: String read GetDate write SetDate;
    property Expires: String read GetExpires write SetExpires;
    property From: String read GetFrom write SetFrom;
    property IfModifiedSince: String read GetIfModifiedSince write SetIfModifiedSince;
    property LastModified: String read GetLastModified write SetLastModified;
    property Location: String read GetLocation write SetLocation;
    property Pragma: String read GetPragma write SetPragma;
    property Referer: String read GetReferer write SetReferer;
    property RetryAfter: String read GetRetryAfter write SetRetryAfter;
    property Server: String read GetServer write SetServer;
    property SetCookie: String read Get_SetCookie write Set_SetCookie;
    property UserAgent: String read GetUserAgent write SetUserAgent;
    property WWWAuthenticate: String read GetWWWAuthenticate write SetWWWAuthenticate;
  end;


  THttpRequestHeader = class(THttpHeader)
  protected
    procedure ParseFirstHeaderLine(const line: String); override;
    function  GetFirstHeaderLine: String; override;
  public
    CommandLine: String;
    Command: String;
    URI: String;                // Uniform Resource Identifier
    QueryString: String;
  end;


  THttpResponseHeader = class(THttpHeader)
  protected
    procedure ParseFirstHeaderLine(const line: String); override;
    function  GetFirstHeaderLine: String; override;
  public
    Code: Integer;
    CodeText: String;
    constructor Create;
  end;


implementation

uses SysUtils;


// THttpHeader

procedure THttpHeader.LineReceived(const ALine: String);
var
  i: Integer;
begin
  if Length(ALine) = 0 then
  begin
    FReader.OnLine := nil;      // Stop receiving
    FReader.StopAndFree;
    if Assigned(FOnCompleted) then
      FOnCompleted(Self);
    FReader := nil;
  end else
    DataReceived := True;
    if not CmdReceived then
    begin
      CmdReceived := True;
      ParseFirstHeaderLine(ALine);
    end else
    begin
      i := Pos(':', ALine);
      SetFieldByName(Trim(Copy(ALine, 1, i - 1)),
        Trim(Copy(ALine, i + 1, Length(ALine))));
    end;
end;

procedure THttpHeader.ReaderEOF(Sender: TObject);
begin
  if Assigned(OnEOF) then
    OnEOF(Self);
end;

procedure THttpHeader.WriterCompleted(ASender: TObject);
begin
  if Assigned(FOnCompleted) then
    FOnCompleted(Self);
  FreeAndNil(FWriter);
end;

function THttpHeader.GetFieldCount: Integer;
begin
  Result := FFields.Count;
end;

function THttpHeader.GetFields(AIndex: Integer): String;
begin
  Result := FieldNames[AIndex] + ': ' + FieldValues[AIndex];
end;

function THttpHeader.GetFieldNames(AIndex: Integer): String;
begin
  Result := PHttpField(FFields.Items[AIndex])^.Name;
end;

procedure THttpHeader.SetFieldNames(AIndex: Integer; const AName: String);
begin
  PHttpField(FFields.Items[AIndex])^.Name := AName;
end;

function THttpHeader.GetFieldValues(AIndex: Integer): String;
begin
  Result := PHttpField(FFields.Items[AIndex])^.Value;
end;

procedure THttpHeader.SetFieldValues(AIndex: Integer; const AValue: String);
begin
  PHttpField(FFields.Items[AIndex])^.Value := AValue;
end;

function  THttpHeader.GetAccept: String; begin Result := GetFieldByName(fieldAccept) end;
procedure THttpHeader.SetAccept(const AValue: String); begin SetFieldByName(fieldAccept, AValue) end;
function  THttpHeader.GetAcceptCharset: String; begin Result := GetFieldByName(fieldAcceptCharset) end;
procedure THttpHeader.SetAcceptCharset(const AValue: String); begin SetFieldByName(fieldAcceptCharset, AValue) end;
function  THttpHeader.GetAcceptEncoding: String; begin Result := GetFieldByName(fieldAcceptEncoding) end;
procedure THttpHeader.SetAcceptEncoding(const AValue: String); begin SetFieldByName(fieldAcceptEncoding, AValue) end;
function  THttpHeader.GetAcceptLanguage: String; begin Result := GetFieldByName(fieldAcceptLanguage) end;
procedure THttpHeader.SetAcceptLanguage(const AValue: String); begin SetFieldByName(fieldAcceptLanguage, AValue) end;
function  THttpHeader.GetAuthorization: String; begin Result := GetFieldByName(fieldAuthorization) end;
procedure THttpHeader.SetAuthorization(const AValue: String); begin SetFieldByName(fieldAuthorization, AValue) end;
function  THttpHeader.GetConnection: String; begin Result := GetFieldByName(fieldConnection) end;
procedure THttpHeader.SetConnection(const AValue: String); begin SetFieldByName(fieldConnection, AValue) end;
function  THttpHeader.GetContentEncoding: String; begin Result := GetFieldByName(fieldContentEncoding) end;
procedure THttpHeader.SetContentEncoding(const AValue: String); begin SetFieldByName(fieldContentEncoding, AValue) end;
function  THttpHeader.GetContentLanguage: String; begin Result := GetFieldByName(fieldContentLanguage) end;
procedure THttpHeader.SetContentLanguage(const AValue: String); begin SetFieldByName(fieldContentLanguage, AValue) end;
function  THttpHeader.GetContentLength: Integer; var s: String; begin s := GetFieldByName(fieldContentLength); if Length(s) = 0 then Result := -1 else Result := StrToInt(s) end;
procedure THttpHeader.SetContentLength(AValue: Integer); begin SetFieldByName(fieldContentLength, IntToStr(AValue)) end;
function  THttpHeader.GetContentType: String; begin Result := GetFieldByName(fieldContentType) end;
procedure THttpHeader.SetContentType(const AValue: String); begin SetFieldByName(fieldContentType, AValue) end;
function  THttpHeader.Get_Cookie: String; begin Result := GetFieldByName(fieldCookie) end;
procedure THttpHeader.Set_Cookie(const AValue: String); begin SetFieldByName(fieldCookie, AValue) end;
function  THttpHeader.GetDate: String; begin Result := GetFieldByName(fieldDate) end;
procedure THttpHeader.SetDate(const AValue: String); begin SetFieldByName(fieldDate, AValue) end;
function  THttpHeader.GetExpires: String; begin Result := GetFieldByName(fieldExpires) end;
procedure THttpHeader.SetExpires(const AValue: String); begin SetFieldByName(fieldExpires, AValue) end;
function  THttpHeader.GetFrom: String; begin Result := GetFieldByName(fieldFrom) end;
procedure THttpHeader.SetFrom(const AValue: String); begin SetFieldByName(fieldFrom, AValue) end;
function  THttpHeader.GetIfModifiedSince: String; begin Result := GetFieldByName(fieldIfModifiedSince) end;
procedure THttpHeader.SetIfModifiedSince(const AValue: String); begin SetFieldByName(fieldIfModifiedSince, AValue) end;
function  THttpHeader.GetLastModified: String; begin Result := GetFieldByName(fieldLastModified) end;
procedure THttpHeader.SetLastModified(const AValue: String); begin SetFieldByName(fieldLastModified, AValue) end;
function  THttpHeader.GetLocation: String; begin Result := GetFieldByName(fieldLocation) end;
procedure THttpHeader.SetLocation(const AValue: String); begin SetFieldByName(fieldLocation, AValue) end;
function  THttpHeader.GetPragma: String; begin Result := GetFieldByName(fieldPragma) end;
procedure THttpHeader.SetPragma(const AValue: String); begin SetFieldByName(fieldPragma, AValue) end;
function  THttpHeader.GetReferer: String; begin Result := GetFieldByName(fieldReferer) end;
procedure THttpHeader.SetReferer(const AValue: String); begin SetFieldByName(fieldReferer, AValue) end;
function  THttpHeader.GetRetryAfter: String; begin Result := GetFieldByName(fieldRetryAfter) end;
procedure THttpHeader.SetRetryAfter(const AValue: String); begin SetFieldByName(fieldRetryAfter, AValue) end;
function  THttpHeader.GetServer: String; begin Result := GetFieldByName(fieldServer) end;
procedure THttpHeader.SetServer(const AValue: String); begin SetFieldByName(fieldServer, AValue) end;
function  THttpHeader.Get_SetCookie: String; begin Result := GetFieldByName(fieldSetCookie) end;
procedure THttpHeader.Set_SetCookie(const AValue: String); begin SetFieldByName(fieldSetCookie, AValue) end;
function  THttpHeader.GetUserAgent: String; begin Result := GetFieldByName(fieldUserAgent) end;
procedure THttpHeader.SetUserAgent(const AValue: String); begin SetFieldByName(fieldUserAgent, AValue) end;
function  THttpHeader.GetWWWAuthenticate: String; begin Result := GetFieldByName(fieldWWWAuthenticate) end;
procedure THttpHeader.SetWWWAuthenticate(const AValue: String); begin SetFieldByName(fieldWWWAuthenticate, AValue) end;

constructor THttpHeader.Create;
begin
  inherited Create;
  FFields := TList.Create;
  HttpVersion := '1.1';
end;

destructor THttpHeader.Destroy;
var
  i: Integer;
  Field: PHttpField;
begin
  if Assigned(FReader) then
    FReader.StopAndFree;
  if Assigned(FWriter) then
    FWriter.StopAndFree;
  for i := 0 to FFields.Count - 1 do
  begin
    Field := PHttpField(FFields.Items[i]);
{    SetLength(Field^.Name, 0);
    SetLength(Field^.Value, 0);}
    Dispose(Field);
  end;
  FFields.Free;
  inherited Destroy;
end;

function THttpHeader.GetFieldByName(const AName: String): String;
var
  i: Integer;
  Name: String;
begin
  Name := UpperCase(AName);
  for i := 0 to FFields.Count - 1 do
    if UpperCase(FieldNames[i]) = Name then
    begin
      Result := FieldValues[i];
      exit;
    end;
  SetLength(Result, 0);
end;

procedure THttpHeader.SetFieldByName(const AName, AValue: String);
var
  i: Integer;
  Name: String;
  Field: PHttpField;
begin
  Name := UpperCase(AName);
  for i := 0 to FFields.Count - 1 do
    if UpperCase(FieldNames[i]) = Name then
    begin
      FieldNames[i] := AName;   // preserve case
      FieldValues[i] := AValue;
      exit;
    end;
  New(Field);
  FillChar(Field^, SizeOf(Field^), 0);
  Field^.Name := AName;
  Field^.Value := AValue;
  FFields.Add(field);
end;

procedure THttpHeader.AsyncSend(AManager: TEventLoop; AStream: THandleStream);
var
  i: Integer;
begin
  if Assigned(FWriter) then
    FWriter.StopAndFree;
  FWriter := TAsyncWriteStream.Create(AManager, AStream);
  FWriter.OnBufferSent := @WriterCompleted;
  FWriter.EndOfLineMarker := #13#10;
  FWriter.WriteLine(GetFirstHeaderLine);
  for i := 0 to FFields.Count - 1 do
    FWriter.WriteLine(Fields[i]);
  FWriter.WriteLine('');
end;

procedure THttpHeader.AsyncReceive(AManager: TEventLoop; AStream: THandleStream);
begin
  CmdReceived := False;
  FReader.Free;
  FReader := TAsyncStreamLineReader.Create(AManager, AStream);
  FReader.OnLine := @LineReceived;
  FReader.OnEOF := @ReaderEOF;
end;


// -------------------------------------------------------------------
//   THttpRequestHeader
// -------------------------------------------------------------------

procedure THttpRequestHeader.ParseFirstHeaderLine(const line: String);
var
  i: Integer;
begin
  CommandLine := line;
  i := Pos(' ', line);
  Command := UpperCase(Copy(line, 1, i - 1));
  URI := Copy(line, i + 1, Length(line));

  // Extract HTTP version
  i := Pos(' ', URI);
  if i > 0 then
  begin
    HttpVersion := Copy(URI, i + 1, Length(URI));
    URI := Copy(URI, 1, i - 1);
    HttpVersion := Copy(HttpVersion, Pos('/', HttpVersion) + 1, Length(HttpVersion));
  end;

  // Extract query string
  i := Pos('?', URI);
  if i > 0 then
  begin
    QueryString := Copy(URI, i + 1, Length(URI));
    URI := Copy(URI, 1, i - 1);
  end;
end;

function THttpRequestHeader.GetFirstHeaderLine: String;
begin
  Result := Command + ' ' + URI;
  if Length(HttpVersion) > 0 then
    Result := Result + ' HTTP/' + HttpVersion;
end;


// -------------------------------------------------------------------
//   THttpResponseHeader
// -------------------------------------------------------------------

procedure THttpResponseHeader.ParseFirstHeaderLine(const line: String);
var
  i: Integer;
  s: String;
begin
  i := Pos('/', line);
  s := Copy(line, i + 1, Length(line));
  i := Pos(' ', s);
  HttpVersion := Copy(s, 1, i - 1);
  s := Copy(s, i + 1, Length(s));
  i := Pos(' ', s);
  if i > 0 then begin
    CodeText := Copy(s, i + 1, Length(s));
    s := Copy(s, 1, i - 1);
  end;
  Code := StrToInt(s);
end;

function THttpResponseHeader.GetFirstHeaderLine: String;
begin
  Result := Format('HTTP/%s %d %s', [HttpVersion, Code, CodeText]);
end;

constructor THttpResponseHeader.Create;
begin
  inherited Create;
  Code := 200;
  CodeText := 'OK';
end;

end.
