{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{

    HTTPDefs: Basic HTTP protocol declarations and classes

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}
{ $DEFINE CGIDEBUG}
unit HTTPDefs;

interface

uses Classes,Sysutils;

const
  fieldAccept          = 'Accept';
  fieldAcceptCharset   = 'Accept-Charset';
  fieldAcceptEncoding  = 'Accept-Encoding';
  fieldAcceptLanguage  = 'Accept-Language';
  fieldAuthorization   = 'Authorization';
  fieldConnection      = 'Connection';
  fieldContentEncoding = 'Content-Encoding';
  fieldContentLanguage = 'Content-Language';
  fieldContentLength   = 'Content-Length';
  fieldContentType     = 'Content-Type';
  fieldCookie          = 'Cookie';
  fieldDate            = 'Date';
  fieldExpires         = 'Expires';
  fieldFrom            = 'From';
  fieldIfModifiedSince = 'If-Modified-Since';
  fieldLastModified    = 'Last-Modified';
  fieldLocation        = 'Location';
  fieldPragma          = 'Pragma';
  fieldReferer         = 'Referer';
  fieldRetryAfter      = 'Retry-After';
  fieldServer          = 'Server';
  fieldSetCookie       = 'Set-Cookie';
  fieldUserAgent       = 'User-Agent';
  fieldWWWAuthenticate = 'WWW-Authenticate';

  NoHTTPFields = 24;

  HTTPDateFmt   = '"%s", dd "%s" yyyy hh:mm:ss'; // For use in FormatDateTime
  SCookieExpire = ' "expires="'+HTTPDateFmt+' "GMT;"';
  SCookieDomain = ' domain=%s;';
  SCookiePath   = ' path=%s;';
  SCookieSecure = ' secure';

  HTTPMonths: array[1..12] of string[3] = (
    'Jan', 'Feb', 'Mar', 'Apr',
    'May', 'Jun', 'Jul', 'Aug',
    'Sep', 'Oct', 'Nov', 'Dec');
  HTTPDays: array[1..7] of string[3] = (
    'Sun', 'Mon', 'Tue', 'Wed',
    'Thu', 'Fri', 'Sat');


Type
  THttpFields = Array[1..NoHTTPFields] of string;

Const
  HTTPFieldNames : THttpFields
             = (fieldAccept, fieldAcceptCharset, fieldAcceptEncoding, 
                fieldAcceptLanguage, fieldAuthorization, fieldConnection,
                fieldContentEncoding, fieldContentLanguage, fieldContentLength,
                fieldContentType, fieldCookie, fieldDate, fieldExpires, 
                fieldFrom, fieldIfModifiedSince, fieldLastModified, fieldLocation,
                fieldPragma, fieldReferer, fieldRetryAfter, fieldServer, 
                fieldSetCookie, fieldUserAgent, fieldWWWAuthenticate);
                

type

  { TCookie }

  TCookie = class(TCollectionItem)
  private
    FName: string;
    FValue: string;
    FPath: string;
    FDomain: string;
    FExpires: TDateTime;
    FSecure: Boolean;
  protected
    Function GetAsString: string;
  public
    constructor Create(ACollection: TCollection); override;
    procedure Assign(Source: TPersistent); override;
    property Name: string read FName write FName;
    property Value: string read FValue write FValue;
    property Domain: string read FDomain write FDomain;
    property Path: string read FPath write FPath;
    property Expires: TDateTime read FExpires write FExpires;
    property Secure: Boolean read FSecure write FSecure;
    Property AsString : String Read GetAsString;
  end;

{ TCookies }

  TCookies = class(TCollection)
  private
  protected
    function GetCookie(Index: Integer): TCookie;
    procedure SetCookie(Index: Integer; Value: TCookie);
  public
    function  Add: TCookie;
    Function CookieByName(AName : String) : TCookie;
    Function FindCookie(AName : String): TCookie;
    Function IndexOfCookie(AName : String) : Integer;
    property Items[Index: Integer]: TCookie read GetCookie write SetCookie; default;
  end;
  { TUploadedFile }

  TUploadedFile = Class(TCollectionItem)
  Private
    FContentType: String;
    FDisposition: String;
    FFieldName: String;
    FFileName: String;
    FLocalFileName: String;
    FSize: Int64;
    FStream : TStream;
  Protected
    function GetStream: TStream; virtual;
  Public
    Destructor Destroy; override;
    Property FieldName : String Read FFieldName Write FFieldName;
    Property FileName : String Read FFileName Write FFileName;
    Property Stream : TStream Read GetStream;
    Property Size : Int64 Read FSize Write FSize;
    Property ContentType : String Read FContentType Write FContentType;
    Property Disposition : String Read FDisposition Write FDisposition;
    Property LocalFileName : String Read FLocalFileName Write FLocalFileName;
  end;
  
  { TUploadedFiles }

  TUploadedFiles = Class(TCollection)
  private
    function GetFile(Index : Integer): TUploadedFile;
    procedure SetFile(Index : Integer; const AValue: TUploadedFile);
  public
    Function IndexOfFile(AName : String) : Integer;
    Function FileByName(AName : String) : TUploadedFile;
    Function FindFile(AName : String) : TUploadedFile;
    Property Files[Index : Integer] : TUploadedFile read GetFile Write SetFile; default;
  end;

  { THTTPHeader }

  THTTPHeader = class(TObject)
  private
    FContentFields: TStrings;
    FCookieFields: TStrings;
    FHTTPVersion : String;
    FFields : THttpFields;
    FQueryFields: TStrings;
    function GetSetField(AIndex: Integer): String;
    function GetSetFieldName(AIndex: Integer): String;
    procedure SetCookieFields(const AValue: TStrings);
    Function GetFieldCount : Integer;
    Function GetFieldName(Index : Integer) : String;
    Function GetContentLength : Integer;
    Procedure SetContentLength(Value : Integer);
    Function GetFieldIndex(AIndex : Integer) : Integer;
    Function GetServerPort : Word;
    Function GetSetFieldValue(Index : Integer) : String; virtual;
  Protected
    Function GetFieldValue(Index : Integer) : String; virtual;
    Procedure SetFieldValue(Index : Integer; Value : String); virtual;
    procedure ParseFirstHeaderLine(const line: String);virtual;
    Procedure ParseCookies; virtual;
  public
    constructor Create; virtual;
    destructor Destroy; override;
    procedure SetFieldByName(const AName, AValue: String);
    function  GetFieldByName(const AName: String): String;
    Function LoadFromStream(Stream : TStream; IncludeCommand : Boolean) : integer;
    Function LoadFromStrings(Strings: TStrings; IncludeCommand : Boolean) : integer; virtual;
    // Common access
    // This is an internal table. We should try to get rid of it,
    // It requires a lot of duplication.
    property FieldCount: Integer read GetFieldCount;
    property Fields[AIndex: Integer]: String read GetSetField;
    property FieldNames[AIndex: Integer]: String read GetSetFieldName;
    property FieldValues[AIndex: Integer]: String read GetSetFieldValue;
    // Various properties.
    Property HttpVersion : String Index 0 Read GetFieldValue Write SetFieldValue;
    Property ProtocolVersion : String Index 0 Read GetFieldValue Write SetFieldValue;
    property Accept: String Index 1 read GetFieldValue write SetFieldValue;
    property AcceptCharset: String Index 2 Read GetFieldValue Write SetFieldValue;
    property AcceptEncoding: String Index 3 Read GetFieldValue Write SetFieldValue;
    property AcceptLanguage: String Index 4 Read GetFieldValue Write SetFieldValue;
    property Authorization: String Index 5 Read GetFieldValue Write SetFieldValue;
    property Connection: String Index 6 Read GetFieldValue Write SetFieldValue;
    property ContentEncoding: String Index 7 Read GetFieldValue Write SetFieldValue;
    property ContentLanguage: String Index 8 Read GetFieldValue Write SetFieldValue;
    property ContentLength: Integer Read GetContentLength Write SetContentLength; // Index 9
    property ContentType: String Index 10 Read GetFieldValue Write SetFieldValue;
    property Cookie: String Index 11 Read GetFieldValue Write SetFieldValue;
    property Date: String Index 12 Read GetFieldValue Write SetFieldValue;
    property Expires: String Index 13 Read GetFieldValue Write SetFieldValue;
    property From: String Index 14 Read GetFieldValue Write SetFieldValue;
    property IfModifiedSince: String Index 15 Read GetFieldValue Write SetFieldValue;
    property LastModified: String Index 16 Read GetFieldValue Write SetFieldValue;
    property Location: String Index 17 Read GetFieldValue Write SetFieldValue;
    property Pragma: String Index 18 Read GetFieldValue Write SetFieldValue;
    property Referer: String Index 19 Read GetFieldValue Write SetFieldValue;
    property RetryAfter: String Index 20 Read GetFieldValue Write SetFieldValue;
    property Server: String Index 21 Read GetFieldValue Write SetFieldValue;
    property SetCookie: String Index 22 Read GetFieldValue Write SetFieldValue;
    property UserAgent: String Index 23 Read GetFieldValue Write SetFieldValue;
    property WWWAuthenticate: String Index 24 Read GetFieldValue Write SetFieldValue;
    // Various aliases, for compatibility
    Property PathInfo : String index 25 read GetFieldValue Write SetFieldValue;
    Property PathTranslated : String Index 26 read GetFieldValue Write SetFieldValue;
    Property RemoteAddress : String Index 27 read GetFieldValue Write SetFieldValue;
    Property RemoteHost : String Index 28 read  GetFieldValue Write SetFieldValue;
    Property ScriptName : String Index 29 read  GetFieldValue Write SetFieldValue;
    Property ServerPort : Word Read GetServerPort; // Index 30
    Property HTTPAccept : String Index 1 read GetFieldValue Write SetFieldValue;
    Property HTTPAcceptCharset : String Index 2 read GetFieldValue Write SetFieldValue;
    Property HTTPAcceptEncoding : String Index 3 read GetFieldValue Write SetFieldValue;
    Property HTTPIfModifiedSince : String Index 15 read GetFieldValue Write SetFieldValue; // Maybe change to TDateTime ??
    Property HTTPReferer : String Index 19 read GetFieldValue Write SetFieldValue;
    Property HTTPUserAgent : String Index 23 read GetFieldValue Write SetFieldValue;
    Property Method : String Index 31 read GetFieldValue Write SetFieldValue;
    Property URL : String Index 32 read GetFieldValue Write SetFieldValue;
    Property Query : String Index 33 read GetFieldValue Write SetFieldValue;
    Property Host : String Index 34 Read GetFieldValue Write SetFieldValue;
    Property Content : String Index 35 Read GetFieldValue Write SetFieldValue;
    // Lists
    Property CookieFields : TStrings Read FCookieFields Write SetCookieFields;
    Property ContentFields: TStrings read FContentFields;
    property QueryFields : TStrings read FQueryFields;
  end;


  { TRequest }

  TRequest = class(THttpHeader)
  private
    FCommand: String;
    FCommandLine: String;
    FQuery: String;
    FURI: String;
    FFiles : TUploadedFiles;
    FReturnedPathInfo : String;
    procedure ParseFirstHeaderLine(const line: String);override;
    function GetFirstHeaderLine: String;
  Protected
    Procedure ProcessMultiPart(Stream : TStream; Const Boundary : String;SL:TStrings); virtual;
    Procedure ProcessQueryString(Const FQueryString : String; SL:TStrings); virtual;
    procedure ProcessURLEncoded(Stream : TStream;SL:TStrings); virtual;
    Function  GetTempUploadFileName : String; virtual;
    Property ReturnedPathInfo : String Read FReturnedPathInfo Write FReturnedPathInfo;
  public
    constructor Create; override;
    destructor destroy; override;
    Function  GetNextPathInfo : String;
    Property  CommandLine : String Read FCommandLine;
    Property  Command : String read FCommand;
    Property  URI : String read FURI;                // Uniform Resource Identifier
    Property  Query : String Read FQuery;
    Property  QueryString : String Read FQuery; // Alias
    Property  HeaderLine : String read GetFirstHeaderLine;
    Property  Files : TUploadedFiles Read FFiles;
  end;


  { TResponse }

  TResponse = class(THttpHeader)
  private
    FContents: TStrings;
    FContentStream : TStream;
    FCode: Integer;
    FCodeText: String;
    FHeadersSent: Boolean;
    FContentSent: Boolean;
    FRequest : TRequest;
    FCookies : TCookies;
    function GetContent: String;
    procedure SetContent(const AValue: String);
    procedure SetContents(AValue: TStrings);
    procedure SetContentStream(const AValue: TStream);
    procedure SetFirstHeaderLine(const line: String);
    function  GetFirstHeaderLine: String;
    procedure ContentsChanged(Sender : TObject);
  Protected
    Procedure DoSendHeaders(Headers : TStrings); virtual; abstract;
    Procedure DoSendContent; virtual; abstract;
    Procedure CollectHeaders(Headers : TStrings); virtual;
  public
    constructor Create(ARequest : TRequest);
    destructor destroy; override;
    Procedure SendContent;
    Procedure SendHeaders;
    Procedure SendResponse; // Delphi compatibility
    Property Request : TRequest Read FRequest;
    Property Code: Integer Read FCode Write FCode;
    Property CodeText: String Read FCodeText Write FCodeText;
    Property FirstHeaderLine : String Read GetFirstHeaderLine Write SetFirstHeaderLine;
    Property ContentStream : TStream Read FContentStream Write SetContentStream;
    Property Content : String Read GetContent Write SetContent;
    property Contents : TStrings read FContents Write SetContents;
    Property HeadersSent : Boolean Read FHeadersSent;
    Property ContentSent : Boolean Read FContentSent;
    property Cookies: TCookies read FCookies;
  end;
  
  { TSessionVariable }


  { TCustomSession }

  TCustomSession = Class(TComponent)
  Private
    FTimeOut: Integer;
  Protected
    Function GetSessionID : String; virtual;
    Function GetSessionVariable(VarName : String) : String; Virtual; abstract;
    procedure SetSessionVariable(VarName : String; const AValue: String);Virtual;abstract;
  Public
    Constructor Create(AOwner : TComponent); override;
    // Init session from request.
    Procedure InitSession(ARequest : TRequest; OnNewSession,OnExpired : TNotifyEvent); virtual;
    // Init response from session (typically, add cookie to response).
    Procedure InitResponse(AResponse : TResponse); virtual;
    // Update response from session (typically, change cookie to response and write session data).
    Procedure UpdateResponse(AResponse : TResponse); virtual; Abstract;
    Procedure RemoveVariable(VariableName : String); virtual; abstract;
    Procedure Terminate; virtual; abstract;
    Property TimeOutMinutes : Integer Read FTimeOut Write FTimeOut;
    Property SessionID : String Read GetSessionID;
    Property Variables[VarName : String] : String Read GetSessionVariable Write SetSessionVariable;
  end;

  TRequestEvent = Procedure (Sender: TObject; ARequest : TRequest) of object;
  TResponseEvent = Procedure (Sender: TObject; AResponse : TResponse) of object;
  
  HTTPError = Class(Exception);

Function HTTPDecode(const AStr: String): String;
Function HTTPEncode(const AStr: String): String;

implementation

{$ifdef CGIDEBUG}
uses dbugintf;
{$endif}

Resourcestring
  SErrContentAlreadySent        = 'HTTP Response content was already sent';
  SErrHeadersAlreadySent        = 'HTTP headers were already sent';
  SErrInternalUploadedFileError = 'Internal uploaded file configuration error';
  SErrNoSuchUploadedFile        = 'No such uploaded file : "%s"';
  SErrUnknownCookie             = 'Unknown cookie: "%s"';
  
const
   hexTable = '0123456789ABCDEF';

{ ---------------------------------------------------------------------
  Auxiliary functions
  ---------------------------------------------------------------------}
  
Function GetFieldNameIndex(AName : String) : Integer;

var
  Name: String;
begin
  Name := UpperCase(AName);
  Result:=NoHTTPFields;
  While (Result>0) and (UpperCase(HTTPFieldNames[Result])<>Name) do
    Dec(Result);
end;

function HTTPDecode(const AStr: String): String;

var
  S,SS, R : PChar;
  H : String[3];
  L,C : Integer;

begin
  L:=Length(Astr);
  SetLength(Result,L);
  If (L=0) then
    exit;
  S:=PChar(AStr);
  SS:=S;
  R:=PChar(Result);
  while (S-SS)<L do
    begin
    case S^ of
      '+': R^ := ' ';
      '%': begin
           Inc(S);
           if ((S-SS)<L) then
             begin
             if (S^='%') then
               R^:='%'
             else
               begin
               H:='$00';
               H[2]:=S^;
               Inc(S);
               If (S-SS)<L then
                 begin
                 H[3]:=S^;
                 Val(H,PByte(R)^,C);
                 If (C<>0) then
                   R^:=' ';
                 end;
               end;
             end;
           end;
      else
        R^ := S^;
      end;
    Inc(R);
    Inc(S);
    end;
  SetLength(Result,R-PChar(Result));
end;

function HTTPEncode(const AStr: String): String;

const
  HTTPAllowed = ['A'..'Z','a'..'z',
                 '*','@','.','_','-',
                 '0'..'9',
                 '$','!','''','(',')'];
                 
var
  SS,S,R: PChar;
  H : String[2];
  L : Integer;
  
begin
  L:=Length(AStr);
  SetLength(Result,L*3); // Worst case scenario
  if (L=0) then
    exit;
  R:=PChar(Result);
  S:=PChar(AStr);
  SS:=S; // Avoid #0 limit !!
  while ((S-SS)<L) do
    begin
    if S^ in HTTPAllowed then
      R^:=S^
    else if (S^=' ') then
      R^:='+'
    else
      begin
      R^:='%';
      H:=HexStr(Ord(S^),2);
      Inc(R);
      R^:=H[1];
      Inc(R);
      R^:=H[2];
      end;
    Inc(R);
    Inc(S);
    end;
  SetLength(Result,R-PChar(Result));
end;


{ ---------------------------------------------------------------------
  THTTPHeader
  ---------------------------------------------------------------------}

function THttpHeader.GetFieldCount: Integer;

Var
  I : Integer;

begin
  Result:=0;
  For I:=1 to NoHTTPFields do
    If (FFields[i]<>'') then
      Inc(Result);
end;

function THTTPHeader.GetContentLength: Integer;
begin
  Result:=StrToIntDef(FFields[9],0);
end;

procedure THTTPHeader.SetContentLength(Value: Integer);
begin
  FFields[9]:=IntToStr(Value);
end;


Function THttpHeader.GetFieldIndex(AIndex : Integer) : Integer;

var
  I : Integer;
   
begin
  I:=1;
  While (I<=NoHTTPFields) and (AIndex>=0) do
    begin
    If (FFields[i]<>'') then
      Dec(AIndex);
    Inc(I);
    end;
  If (AIndex=-1) then
    Result:=I-1
  else
    Result:=-1;  
end;

function THTTPHeader.GetServerPort: Word;
begin
  Result:=StrToIntDef(GetFieldValue(30),0);
end;

function THTTPHeader.GetSetFieldValue(Index: Integer): String;

Var
  I : Integer;

begin
  I:=GetFieldIndex(Index);
  If (I<>-1) then
    Result:=FFields[I];
end;

function THTTPHeader.GetSetField(AIndex: Integer): String;
var
  I : Integer;

begin
  I:=GetFieldIndex(AIndex);
  If (I<>-1) then
    Result := HTTPFieldNames[I] + ': ' + FFields[I];
end;

function THTTPHeader.GetSetFieldName(AIndex: Integer): String;
var
  I : Integer;

begin
  I:=GetFieldIndex(AIndex);
  if (I<>-1) then
    Result:=HTTPFieldNames[I];
end;


function THttpHeader.GetFieldName(Index: Integer): String;

Var
  I : Integer;

begin
  I:=GetFieldIndex(Index);
  If (I<>-1) then
    Result := HTTPFieldNames[i];
end;

Function THttpHeader.GetFieldValue(Index : Integer) : String;

begin
  if (Index>1) and (Index<NoHTTPFields) then
    Result:=FFields[Index]
  else
    case Index of
      0  : Result:=FHTTPVersion;
      25 : Result:=''; // Property PathInfo
      26 : Result:=''; // Property PathTranslated
      27 : Result:=''; // Property RemoteAddress
      28 : Result:=''; // Property RemoteHost
      29 : Result:=''; // Property ScriptName
      30 : Result:=''; // Property ServerPort
    end;
end;

procedure THTTPHeader.SetCookieFields(const AValue: TStrings);
begin
  FCookieFields.Assign(AValue);
end;


Procedure THttpHeader.SetFieldValue(Index : Integer; Value : String);

begin
  if (Index>1) and (Index<NoHTTPFields) then
    begin
    FFields[Index]:=Value;
    If (Index=11) then
      ParseCookies;
    end
  else
    case Index of
      0  : FHTTPVersion:=Value;
      25 : ; // Property PathInfo : String index 25 read GetFieldValue Write SetFieldValue;
      26 : ; // Property PathTranslated : String Index 26 read GetFieldValue Write SetFieldValue;
      27 : ; // Property RemoteAddress : String Index 27 read GetFieldValue Write SetFieldValue;
      28 : ; // Property RemoteHost : String Index 28 read  GetFieldValue Write SetFieldValue;
      29 : ; // Property ScriptName : String Index 29 read  GetFieldValue Write SetFieldValue;
      30 : ; // Property ServerPort : Word Read GetServerPort; // Index 30
    end;
end;

procedure THTTPHeader.ParseFirstHeaderLine(const line: String);
begin
  // Do nothing.
end;

procedure THTTPHeader.ParseCookies;

Var
  P : Integer;
  S,C : String;
  
begin
{$ifdef cgidebug}  SendMethodEnter('Parsecookies');{$endif}
  S:=Cookie;
  While (S<>'') do
    begin
    P:=Pos(';',S);
    If (P=0) then
      P:=length(S)+1;
    C:=Copy(S,1,P-1);
    While (P<Length(S)) and (S[P+1]=' ') do
      Inc(P);
    System.Delete(S,1,P);
    FCookieFields.Add(HTTPDecode(C));
    end;
{$ifdef cgidebug}  SendMethodExit('Parsecookies done');{$endif}
end;

constructor THttpHeader.Create;
begin
  FCookieFields:=TStringList.Create;
  FQueryFields:=TStringList.Create;
  FContentFields:=TStringList.Create;
  FHttpVersion := '1.1';
end;

destructor THttpHeader.Destroy;

begin
  FreeAndNil(FContentFields);
  FreeAndNil(FQueryFields);
  FreeAndNil(FCookieFields);
  inherited Destroy;
end;


function THttpHeader.GetFieldByName(const AName: String): String;
var
  i: Integer;

begin
  I:=GetFieldNameIndex(AName);
  If (I<>0) then
    Result:=self.GetFieldValue(i);
end;

Function THTTPHeader.LoadFromStream(Stream: TStream; IncludeCommand : Boolean) : Integer;

Var
  S : TStrings;

begin
  S:=TStringList.Create;
  Try
    S.LoadFromStream(Stream);
    Result:=LoadFromStrings(S,IncludeCommand);
  Finally
    S.Free;
  end;
end;

Function THTTPHeader.LoadFromStrings(Strings: TStrings; IncludeCommand : Boolean) : integer;

Var
  P  : Integer;
  S,VN : String;

begin
  Result:=0;
  if (Strings.Count>0) then
    begin
    if IncludeCommand then
      begin
      ParseFirstHeaderLine(Strings[0]);
      Inc(Result);
      end;
    While (Result<Strings.Count) and (Strings[Result]<>'') do
      begin
      S:=Strings[Result];
      P:=Pos(':',S);
      if (P<>0) then
        begin
        VN:=Copy(S,1,P-1);
        Delete(S,1,P);
        P:=GetFieldNameIndex(VN);
        If (P<>-1) then
          SetFieldValue(P,S);
        end;
      Inc(Result);
      end;
    end;
end;

procedure THttpHeader.SetFieldByName(const AName, AValue: String);
var
  i: Integer;

begin
  I:=GetFieldNameIndex(AName);
  If (I<>0) then
    SetFieldValue(i,AValue);
end;

{ -------------------------------------------------------------------
  TFormItem, used by TRequest to process Multipart-encoded data.
  -------------------------------------------------------------------}

Type
  TFormItem = Class(TObject)
    Disposition : String;
    Name : String;
    IsFile : Boolean;
    FileName : String;
    ContentType : String;
    DLen : Integer;
    Data : String;
    Procedure Process;
  end;

Procedure TFormItem.Process;

  Function GetLine(Var S : String) : String;

  Var
    P : Integer;

  begin
    P:=Pos(#13#10,S);
    If (P<>0) then
      begin
      Result:=Copy(S,1,P-1);
      Delete(S,1,P+1);
      end;
  end;

  Function GetWord(Var S : String) : String;

  Var
    I,len : Integer;
    Quoted : Boolean;
    C : Char;

  begin
    len:=length(S);
    quoted:=false;
    Result:='';
    for i:=1 to len do
      Begin
      c:=S[i];
      if (c='"') then
        Quoted:=Not Quoted
      else
        begin
        if not (c in [' ','=',';',':']) or Quoted then
          Result:=Result+C;
        if (c in [';',':','=']) and (not quoted) then
          begin
          Delete(S,1,I);
          Exit;
          end;
        end;
      end;
     S:='';
  end;

Var
  Line : String;
  len : integer;
  S : string;

begin
  Line:=GetLine(Data);
  While (Line<>'') do
    begin
    S:=GetWord(Line);
    While (S<>'') do
      begin
      If CompareText(S,'Content-Disposition')=0 then
        Disposition:=GetWord(Line)
      else if CompareText(S,'name')=0 Then
        Name:=GetWord(Line)
      else if CompareText(S,'filename')=0 then
        begin
        FileName:=GetWord(Line);
        isFile:=True;
        end
      else if CompareText(S,'Content-Type')=0 then
        ContentType:=GetWord(Line);
      S:=GetWord(Line);
      end;
    Line:=GetLine(Data);
    end;
  // Now Data contains the rest of the data, plus a CR/LF. Strip the CR/LF
  Len:=Length(Data);
  If (len>2) then
    Data:=Copy(Data,1,Len-2);
end;

{
  This needs MASSIVE improvements for large files.
  Best would be to do this directly from the input stream
  and save the files at once if needed. (e.g. when a
  certain size is reached.)
}

procedure FormSplit(var Cnt : String; boundary: String; List : TList);

// Splits the form into items
var
  Sep : string;
  Clen,slen, p:longint;
  FI : TFormItem;

begin
  Sep:='--'+boundary+#13+#10;
  Slen:=length(Sep);
  CLen:=Pos('--'+Boundary+'--',Cnt);
  // Cut last marker
  Cnt:=Copy(Cnt,1,Clen-1);
  // Cut first marker
  Delete(Cnt,1,Slen);
  Clen:=Length(Cnt);
  While Clen>0 do
    begin
    Fi:=TFormItem.Create;
    List.Add(Fi);
    P:=pos(Sep,Cnt);
    If (P=0) then
      P:=CLen+1;
    FI.Data:=Copy(Cnt,1,P-1);
    delete(Cnt,1,P+SLen-1);
    CLen:=Length(Cnt);
    end;
end;

{ -------------------------------------------------------------------
  TRequest
  -------------------------------------------------------------------}
  
constructor TRequest.create;
begin
  inherited create;
  FFiles:=TUploadedFiles.Create(TUPloadedFile);
end;

destructor TRequest.destroy;
begin
  FreeAndNil(FFiles);
  inherited destroy;
end;

function TRequest.GetNextPathInfo: String;

Var
  P : String;
  i : Integer;
  
begin
  P:=PathInfo;
  if (P <> '') and (P[length(P)] = '/') then
    Delete(P, length(P), 1);//last char is '/'
  If (P<>'') and (P[1]='/') then
    Delete(P,1,1);
  I:=Pos('/',P);
  If (I>0) then
  begin//only if there was a module name, otherwise only the action name is there
    Delete(P,1,Length(FReturnedPathInfo));
    I:=Pos('/',P);
  end;
  If (I=0) then
    I:=Length(P)+1;
  Result:=Copy(P,1,I-1);
  FReturnedPathInfo:=FReturnedPathInfo+'/'+Result;
end;

procedure TRequest.ParseFirstHeaderLine(const line: String);
var
  i: Integer;
begin
  FCommandLine := line;
  i := Pos(' ', line);
  FCommand := UpperCase(Copy(line, 1, i - 1));
  FURI := Copy(line, i + 1, Length(line));

  // Extract HTTP version
  i := Pos(' ', URI);
  if i > 0 then
  begin
    FHttpVersion := Copy(URI, i + 1, Length(URI));
    FURI := Copy(URI, 1, i - 1);
    FHttpVersion := Copy(HttpVersion, Pos('/', HttpVersion) + 1, Length(HttpVersion));
  end;

  // Extract query string
  i := Pos('?', URI);
  if i > 0 then
  begin
    FQuery:= Copy(URI, i + 1, Length(URI));
    FURI := Copy(URI, 1, i - 1);
  end;
end;

function TRequest.GetFirstHeaderLine: String;
begin
  Result := Command + ' ' + URI;
  if Length(HttpVersion) > 0 then
    Result := Result + ' HTTP/' + HttpVersion;
end;

Procedure TRequest.ProcessQueryString(Const FQueryString : String; SL:TStrings);


var
  queryItem : String;
  delimiter : Char;
  aString : String;
  aSepStr : String;
  aPos    : Integer;
  aLenStr : Integer;
  aLenSep : Integer;

  function hexConverter(h1, h2 : Char) : Char;

  var
    B : Byte;

  begin
    B:=(Pos(upcase(h1),hexTable)-1)*16;
    B:=B+Pos(upcase(h2),hexTable)-1;
    Result:=chr(B);
  end;


  procedure InitToken(aStr, aSep : String);

  begin
    aString := aStr;
    aSepStr := aSep;
    aPos    := 1;
    aLenStr := Length(aString);
    aLenSep := Length(aSepStr);
  end;

  function NextToken(var aToken : String; out aSepChar : Char) : Boolean;

  var
    i : Integer;
    j : Integer;
    BoT : Integer;
    EoT : Integer;
    isSep : Boolean;

  begin
    BoT:=aPos;
    EoT:=aPos;
    for i:=aPos to aLenStr do
      begin
      IsSep := false;
      for j := 1 to aLenSep do
        begin
        if aString[i] = aSepStr[j] then
          begin
          IsSep := true;
          Break;
          end;
        end;
      if IsSep then
        begin
        EoT  := i;
        aPos := i + 1;
        aSepChar := aString[i];
        Break;
        end
      else
        begin
        if i = aLenStr then
          begin
          EoT  := i;
          aPos := i;
          Break;
          end;
        end;
      end;
    if aPos < aLenStr then
      begin
      aToken := Copy(aString, BoT, EoT - BoT);
      Result := true;
      end
    else
      begin
      if aPos = aLenStr then
        begin
        aToken := Copy(aString, BoT, EoT - BoT + 1);
        Result := true;
        aPos   := aPos + 1;
        end
      else
        begin
        Result := false;
       end;
    end;
  end;


begin
{$ifdef CGIDEBUG}SendMethodEnter('ProcessQueryString');{$endif CGIDEBUG}
  InitToken(FQueryString, '&');
  while NextToken(QueryItem, delimiter) do
    begin
    if (QueryItem<>'') then
      begin
      QueryItem:=HTTPDecode(QueryItem);
      SL.Add(QueryItem);
      end;
    end;
{$ifdef CGIDEBUG}SendMethodExit('ProcessQueryString');{$endif CGIDEBUG}
end;

function TRequest.GetTempUploadFileName: String;

begin
//Result:=GetTempFileName('/tmp/','CGI') {Hard coded path no good for all OS-es}
{
GetTempDir returns the OS temporary directory if possible, or from the
environment variable TEMP . For CGI programs you need to pass global environment
 variables, it is not automatic. For example in the Apache httpd.conf with a
"PassEnv TEMP" or "SetEnv TEMP /pathtotmpdir" line so the web server passes this
 global environment variable to the CGI programs' local environment variables.
}
  Result := GetTempFileName(GetTempDir, 'CGI');
end;


Procedure TRequest.ProcessMultiPart(Stream : TStream; Const Boundary : String; SL:TStrings);

Var
  L : TList;
  B : String;
  I : Integer;
  S,FF,key, Value : String;
  FI : TFormItem;
  F : TStream;

begin
{$ifdef CGIDEBUG} SendMethodEnter('ProcessMultiPart');{$endif CGIDEBUG}
  i:=Pos('=',Boundary);
  B:=Copy(Boundary,I+1,Length(Boundary)-I);
  I:=Length(B);
  If (I>0) and (B[1]='"') then
    B:=Copy(B,2,I-2);
  L:=TList.Create;
  Try
    SetLength(S,Stream.Size);
    If Length(S)>0 then
      if Stream is TCustomMemoryStream then
        // Faster.
        Move(TCustomMemoryStream(Stream).Memory^,S[1],Length(S))
      else
        begin
        Stream.Read(S[1],Length(S));
        Stream.Position:=0;
        end;
    FormSplit(S,B,L);
    For I:=L.Count-1 downto 0 do
      begin
      FI:=TFormItem(L[i]);
      FI.Process;
      If (FI.Name='') then
        Raise Exception.CreateFmt('Invalid multipart encoding: %s',[FI.Data]);
{$ifdef CGIDEBUG}
      With FI Do
        begin
        SendSeparator;
        SendDebug  ('PMP item Name        : '+Name);
        SendDebug  ('PMP item Disposition : '+Disposition);
        SendDebug  ('PMP item FileName    : '+FileName);
        SendBoolean('PMP item IsFile      : ',IsFile);
        SendDebug  ('PMP item ContentType : '+ContentType);
        SendInteger('PMP item DLen        : ',DLen);
        SendDebug  ('PMP item Data        : '+Data);
        end;
{$endif CGIDEBUG}
      Key:=FI.Name;
      If Not FI.IsFile Then
        Value:=FI.Data
      else
        begin
        Value:=FI.FileName;
        if Length(FI.Data)=0 then
          FF:=''
        else
          begin
          FF:=GetTempUploadFileName;
          F:=TFileStream.Create(FF,fmCreate);
          Try
            F.Write(FI.Data[1],Length(FI.Data));
          finally
            F.Free;
          end;
          end;
        With Files.Add as TUploadedFile do
          begin
          FieldName:=FI.Name;
          FileName:=FI.FileName;
          ContentType:=FI.ContentType;
          Disposition:=FI.Disposition;
          Size:=FI.DLen;
          LocalFileName:=FF;
          end;
        end;
      FI.Free;
      L[i]:=Nil;
      SL.Add(Key+'='+Value)
      end;
  Finally
    For I:=0 to L.Count-1 do
      TObject(L[i]).Free;
    L.Free;
  end;
{$ifdef CGIDEBUG}  SendMethodExit('ProcessMultiPart');{$endif CGIDEBUG}
end;

Procedure TRequest.ProcessURLEncoded(Stream: TStream; SL:TStrings);

var
  S : String;

begin
{$ifdef CGIDEBUG} SendMethodEnter('ProcessURLEncoded');{$endif CGIDEBUG}
  SetLength(S,Stream.Size); // Skip added Null.
  Stream.ReadBuffer(S[1],Stream.Size);
{$ifdef CGIDEBUG}SendDebugFmt('Query string : %s',[s]);{$endif CGIDEBUG}
  ProcessQueryString(S,SL);
{$ifdef CGIDEBUG} SendMethodEnter('ProcessURLEncoded');{$endif CGIDEBUG}
end;

{ ---------------------------------------------------------------------
  TUploadedFiles
  ---------------------------------------------------------------------}

function TUploadedFiles.GetFile(Index : Integer): TUploadedFile;
begin
  Result:=TUPloadedFile(Items[Index]);
end;

procedure TUploadedFiles.SetFile(Index : Integer; const AValue: TUploadedFile);
begin
  Items[Index]:=AValue;
end;

function TUploadedFiles.IndexOfFile(AName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(Files[Result].FieldName,AName)<>0) do
    Dec(Result);
end;

function TUploadedFiles.FileByName(AName: String): TUploadedFile;


begin
  Result:=FindFile(AName);
  If (Result=Nil) then
    Raise HTTPError.CreateFmt(SErrNoSuchUploadedFile,[AName]);
end;

Function TUploadedFiles.FindFile(AName: String): TUploadedFile;

Var
  I : Integer;
  
begin
  I:=IndexOfFile(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=Files[I];
end;

{ ---------------------------------------------------------------------
  TUploadedFile
  ---------------------------------------------------------------------}

function TUploadedFile.GetStream: TStream;
begin
  If (FStream=Nil) then
    begin
    If (FLocalFileName='') then
      Raise HTTPError.Create(SErrInternalUploadedFileError);
    FStream:=TFileStream.Create(FLocalFileName,fmOpenRead);
    end;
  Result:=FStream;
end;

destructor TUploadedFile.Destroy;
begin
  FreeAndNil(FStream);
  Inherited;
end;

{ ---------------------------------------------------------------------
  TResponse
  ---------------------------------------------------------------------}

constructor TResponse.Create(ARequest : TRequest);
begin
  inherited Create;
  FRequest:=ARequest;
  FCode := 200;
  FCodeText := 'OK';
  ContentType:='text/html';
  FContents:=TStringList.Create;
  TStringList(FContents).OnChange:=@ContentsChanged;
  FCookies:=TCookies.Create(TCookie);
end;

destructor TResponse.destroy;
begin
  FreeAndNil(FCookies);
  FreeAndNil(FContents);
  inherited destroy;
end;


procedure TResponse.SendContent;
begin
  if ContentSent then
    Raise HTTPError.Create(SErrContentAlreadySent);
  if Not HeadersSent then
    SendHeaders;
  DoSendContent;
  FContentSent:=True;
end;

procedure TResponse.SendHeaders;

Var
  FHeaders  : TStringList;

begin
  if HeadersSent then
    Raise HTTPError.Create(SErrHeadersAlreadySent);
  FHeaders:=TStringList.Create;
  CollectHeaders(FHeaders);
  With Fheaders do
  If (Count>0) and (Strings[Count-1]<>'') then
    Add('');
  Try
    DoSendHeaders(FHeaders);
    FHeadersSent:=True;
  Finally
    FHeaders.Free;
  end;
end;

procedure TResponse.SendResponse;
begin
  SendContent;
end;

procedure TResponse.SetFirstHeaderLine(const line: String);
var
  i: Integer;
  s: String;
begin
  i := Pos('/', line);
  s := Copy(line, i + 1, Length(line));
  i := Pos(' ', s);
  FHttpVersion := Copy(s, 1, i - 1);
  s := Copy(s, i + 1, Length(s));
  i := Pos(' ', s);
  if i > 0 then begin
    FCodeText := Copy(s, i + 1, Length(s));
    s := Copy(s, 1, i - 1);
  end;
  FCode := StrToInt(s);
end;

procedure TResponse.SetContents(AValue: TStrings);
begin
  FContentStream:=Nil;
  FContents.Assign(AValue);
end;

function TResponse.GetContent: String;
begin
  Result:=Contents.Text;
end;

procedure TResponse.SetContent(const AValue: String);
begin
  FContentStream:=Nil;
  FContents.Text:=AValue;
end;

procedure TResponse.SetContentStream(const AValue: TStream);
begin
  If (FContentStream<>AValue) then
    begin
    FContentStream:=AValue;
    If (FContentStream<>Nil) then
      ContentLength:=FContentStream.Size
    else
      ContentLength:=0;
    end;
end;

function TResponse.GetFirstHeaderLine: String;
begin
  Result := Format('HTTP/%s %d %s', [HttpVersion, Code, CodeText]);
end;

procedure TResponse.ContentsChanged(Sender: TObject);

Var
  I,L,LE : Integer;

begin
  L:=0;
  LE:=Length(LineEnding);
  For I:=0 to FContents.Count-1 do
    L:=L+Length(FContents[i])+LE;
  ContentLength:=L;
end;

procedure TResponse.CollectHeaders(Headers: TStrings);

Var
  I : Integer;

begin
  Headers.add(Format('Status: %d %s',[Code,CodeText]));
{$ifdef cgidebug}
  SendMethodEnter('Collectheaders');
  If Not Assigned(FCookies) then
    SendDebug('No cookies')
  else
    SendInteger('Nr of cookies',FCookies.Count);
{$endif}
  For I:=0 to FCookies.Count-1 do
    Headers.Add('Set-Cookie: '+FCookies[i].AsString);
  For I:=0 to FieldCount-1 do
    Headers.Add(Fields[i]);
  Headers.Add('');
{$ifdef cgidebug} SendMethodExit('Collectheaders');{$endif}
end;


{ TCookie }


function TCookie.GetAsString: string;

Var
  Y,M,D : Word;

begin
{$ifdef cgidebug}SendMethodEnter('TCookie.GetAsString');{$endif}
  try
    Result:=Format('%s=%s;',[HTTPEncode(FName),HTTPEncode(FValue)]);
    if (Length(FDomain)>0) then
      Result:=Result+Format(SCookieDomain,[FDomain]);
    if (Length(FPath)>0) then
      Result:=Result+Format(SCookiePath,[FPath]);
    if (FExpires>-1) then
      begin
      DecodeDate(Expires,Y,M,D);
      Result:=Result+Format(FormatDateTime(SCookieExpire,Expires),
                          [HTTPDays[DayOfWeek(Expires)],HTTPMonths[M]]);
      end;
    if Secure then
      Result:=Result+SCookieSecure;
  except
{$ifdef cgidebug}
    On E : Exception do
      SendDebug('Exception in cookie asstring : '+E.Message)
{$endif}
  end;
{$ifdef cgidebug}SendMethodExit('TCookie.GetAsString');{$endif}
end;

constructor TCookie.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FExpires:=-1;
end;

procedure TCookie.Assign(Source: TPersistent);
begin
  if Source is TCookie then
    with TCookie(Source) do
      begin
      Self.FName:=Name;
      Self.FValue:=Value;
      Self.FDomain:=Domain;
      Self.FPath:=Path;
      Self.FExpires:=Expires;
      Self.FSecure:=Secure;
      end
  else
    inherited Assign(Source);
end;

{ TCookieCollection }

function TCookies.GetCookie(Index: Integer): TCookie;
begin
{$ifdef cgidebug}SendMethodExit('TCookies.GetCookie');{$endif}
  Result:=TCookie(inherited Items[Index]);
{$ifdef cgidebug}SendMethodExit('TCookies.GetCookie');{$endif}
end;

procedure TCookies.SetCookie(Index: Integer; Value: TCookie);
begin
  Items[Index]:=Value
end;

function TCookies.Add: TCookie;
begin
  Result:=TCookie(Inherited Add);
end;

function TCookies.CookieByName(AName: String): TCookie;
begin
  Result:=FindCookie(AName);
  If (Result=Nil) then
    Raise HTTPError.CreateFmt(SErrUnknownCookie,[AName]);
end;

function TCookies.FindCookie(AName: String): TCookie;
Var
  I : Integer;

begin
  I:=IndexOfCookie(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetCookie(I);
end;

function TCookies.IndexOfCookie(AName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetCookie(Result).Name,AName)<>0) do
    Dec(Result);
end;

{ TCustomSession }


function TCustomSession.GetSessionID: String;

Var
  G : TGUID;

begin
  CreateGUID(G);
  Result:=GuiDToString(G);
end;

constructor TCustomSession.Create(AOwner: TComponent);
begin
  FTimeOut:=15;
  inherited Create(AOwner);
end;

procedure TCustomSession.InitResponse(AResponse: TResponse);
begin
  // do nothing
end;

procedure TCustomSession.InitSession(ARequest: TRequest; OnNewSession,OnExpired : TNotifyEvent);
begin
  // Do nothing
end;


end.
