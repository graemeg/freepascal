{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2009 by the Free Pascal development team

    TWebApplication class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{ $define CGIDEBUG}
{$mode objfpc}
{$H+}

unit custweb;

Interface

uses
  CustApp,Classes,SysUtils, httpdefs, fphttp, eventlog;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Type
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Const
  CGIVarCount = 36;

Type
  TCGIVarArray = Array[1..CGIVarCount] of String;

Const
  CgiVarNames : TCGIVarArray =
   ({ 1  } 'AUTH_TYPE',
    { 2  } 'CONTENT_LENGTH',
    { 3  } 'CONTENT_TYPE',
    { 4  } 'GATEWAY_INTERFACE',
    { 5  } 'PATH_INFO',
    { 6  } 'PATH_TRANSLATED',
    { 7  } 'QUERY_STRING',
    { 8  } 'REMOTE_ADDR',
    { 9  } 'REMOTE_HOST',
    { 10 } 'REMOTE_IDENT',
    { 11 } 'REMOTE_USER',
    { 12 } 'REQUEST_METHOD',
    { 13 } 'SCRIPT_NAME',
    { 14 } 'SERVER_NAME',
    { 15 } 'SERVER_PORT',
    { 16 } 'SERVER_PROTOCOL',
    { 17 } 'SERVER_SOFTWARE',
    { 18 } 'HTTP_ACCEPT',
    { 19 } 'HTTP_ACCEPT_CHARSET',
    { 20 } 'HTTP_ACCEPT_ENCODING',
    { 21 } 'HTTP_IF_MODIFIED_SINCE',
    { 22 } 'HTTP_REFERER',
    { 23 } 'HTTP_USER_AGENT',
    { 24 } 'HTTP_COOKIE',
     // Additional Apache vars
    { 25 } 'HTTP_CONNECTION',
    { 26 } 'HTTP_ACCEPT_LANGUAGE',
    { 27 } 'HTTP_HOST',
    { 28 } 'SERVER_SIGNATURE',
    { 29 } 'SERVER_ADDR',
    { 30 } 'DOCUMENT_ROOT',
    { 31 } 'SERVER_ADMIN',
    { 32 } 'SCRIPT_FILENAME',
    { 33 } 'REMOTE_PORT',
    { 34 } 'REQUEST_URI',
    { 35 } 'CONTENT',
    { 36 } 'HTTP_X_REQUESTED_WITH'
    );

Type

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
  { TCustomWebApplication }

  TGetModuleEvent = Procedure (Sender : TObject; ARequest : TRequest;
                               Var ModuleClass : TCustomHTTPModuleClass) of object;
  TOnShowRequestException = procedure(AResponse: TResponse; AnException: Exception; var handled: boolean);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TLogEvent = Procedure (EventType: TEventType; const Msg: String) of object;
  TInitModuleEvent = Procedure (Sender : TObject; Module: TCustomHTTPModule) of object;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  { TWebHandler }

  TWebHandler = class(TComponent)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FDefaultModuleName: String;
    FOnIdle: TNotifyEvent;
    FOnInitModule: TInitModuleEvent;
    FOnUnknownRequestEncoding: TOnUnknownEncodingEvent;
=======
    FOnIdle: TNotifyEvent;
>>>>>>> graemeg/cpstrnew
=======
    FOnIdle: TNotifyEvent;
>>>>>>> graemeg/cpstrnew
=======
    FOnIdle: TNotifyEvent;
>>>>>>> graemeg/cpstrnew
=======
    FOnIdle: TNotifyEvent;
>>>>>>> origin/cpstrnew
    FTerminated: boolean;
    FAdministrator: String;
    FAllowDefaultModule: Boolean;
    FApplicationURL: String;
    FEmail: String;
    FModuleVar: String;
    FOnGetModule: TGetModuleEvent;
    FOnShowRequestException: TOnShowRequestException;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    FRequest : TRequest;
>>>>>>> graemeg/cpstrnew
=======
    FRequest : TRequest;
>>>>>>> graemeg/cpstrnew
=======
    FRequest : TRequest;
>>>>>>> graemeg/cpstrnew
=======
    FRequest : TRequest;
>>>>>>> origin/cpstrnew
    FHandleGetOnPost : Boolean;
    FRedirectOnError : Boolean;
    FRedirectOnErrorURL : String;
    FTitle: string;
    FOnTerminate : TNotifyEvent;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FOnLog : TLogEvent;
    FPreferModuleName : Boolean;
  protected
    Class Procedure DoError(Msg : String; AStatusCode : Integer = 0; AStatusText : String = '');
    Class Procedure DoError(Fmt : String; Const Args : Array of const;AStatusCode : Integer = 0; AStatusText : String = '');
    procedure Terminate; virtual;
=======
  protected
    procedure Terminate;
>>>>>>> graemeg/cpstrnew
=======
  protected
    procedure Terminate;
>>>>>>> graemeg/cpstrnew
=======
  protected
    procedure Terminate;
>>>>>>> graemeg/cpstrnew
=======
  protected
    procedure Terminate;
>>>>>>> origin/cpstrnew
    Function GetModuleName(Arequest : TRequest) : string;
    function WaitForRequest(out ARequest : TRequest; out AResponse : TResponse) : boolean; virtual; abstract;
    procedure EndRequest(ARequest : TRequest;AResponse : TResponse); virtual;
    function FindModule(ModuleClass : TCustomHTTPModuleClass): TCustomHTTPModule;
    Procedure SetBaseURL(AModule : TCustomHTTPModule; Const AModuleName : String; ARequest : TRequest); virtual;
    function GetApplicationURL(ARequest : TRequest): String; virtual;
    procedure ShowRequestException(R: TResponse; E: Exception); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure InitRequest(ARequest : TRequest); virtual;
    Procedure InitResponse(AResponse : TResponse); virtual;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Function GetEmail : String; virtual;
    Function GetAdministrator : String; virtual;
    property Terminated: boolean read FTerminated;
  Public
    constructor Create(AOwner: TComponent); override;
    Procedure Run; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Log(EventType : TEventType; Const Msg : String);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Procedure DoHandleRequest(ARequest : TRequest; AResponse : TResponse);
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); virtual;
    Property HandleGetOnPost : Boolean Read FHandleGetOnPost Write FHandleGetOnPost;
    Property RedirectOnError : boolean Read FRedirectOnError Write FRedirectOnError;
    Property RedirectOnErrorURL : string Read FRedirectOnErrorURL Write FRedirectOnErrorURL;
    Property ApplicationURL : String Read FApplicationURL Write FApplicationURL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property AllowDefaultModule : Boolean Read FAllowDefaultModule Write FAllowDefaultModule;
    Property DefaultModuleName : String Read FDefaultModuleName Write FDefaultModuleName;
=======
    Property Request : TRequest read FRequest;
    Property AllowDefaultModule : Boolean Read FAllowDefaultModule Write FAllowDefaultModule;
>>>>>>> graemeg/cpstrnew
=======
    Property Request : TRequest read FRequest;
    Property AllowDefaultModule : Boolean Read FAllowDefaultModule Write FAllowDefaultModule;
>>>>>>> graemeg/cpstrnew
=======
    Property Request : TRequest read FRequest;
    Property AllowDefaultModule : Boolean Read FAllowDefaultModule Write FAllowDefaultModule;
>>>>>>> graemeg/cpstrnew
=======
    Property Request : TRequest read FRequest;
    Property AllowDefaultModule : Boolean Read FAllowDefaultModule Write FAllowDefaultModule;
>>>>>>> origin/cpstrnew
    Property ModuleVariable : String Read FModuleVar Write FModuleVar;
    Property OnGetModule : TGetModuleEvent Read FOnGetModule Write FOnGetModule;
    Property Email : String Read GetEmail Write FEmail;
    property Title: string read FTitle write FTitle;
    Property Administrator : String Read GetAdministrator Write FAdministrator;
    property OnShowRequestException: TOnShowRequestException read FOnShowRequestException write FOnShowRequestException;
    property OnIdle: TNotifyEvent read FOnIdle write FOnIdle;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property OnLog : TLogEvent Read FOnLog Write FOnLog;
    Property OnUnknownRequestEncoding : TOnUnknownEncodingEvent Read FOnUnknownRequestEncoding Write FOnUnknownRequestEncoding;
    Property OnInitModule: TInitModuleEvent Read FOnInitModule write FOnInitModule;
    Property PreferModuleName : Boolean Read FPreferModuleName Write FPreferModuleName;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

  TCustomWebApplication = Class(TCustomApplication)
  Private
    FEventLog: TEventLog;
    FWebHandler: TWebHandler;
    function GetAdministrator: String;
    function GetAllowDefaultModule: Boolean;
    function GetApplicationURL: String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function GetDefaultModuleName: String;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function GetEmail: String;
    function GetEventLog: TEventLog;
    function GetHandleGetOnPost: Boolean;
    function GetModuleVar: String;
    function GetOnGetModule: TGetModuleEvent;
    function GetOnShowRequestException: TOnShowRequestException;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function GetOnUnknownRequestEncoding: TOnUnknownEncodingEvent;
    function GetRedirectOnError: boolean;
    function GetRedirectOnErrorURL: string;
    function GetPreferModuleName: boolean;
    procedure SetAdministrator(const AValue: String);
    procedure SetAllowDefaultModule(const AValue: Boolean);
    procedure SetApplicationURL(const AValue: String);
    procedure SetDefaultModuleName(AValue: String);
=======
=======
>>>>>>> graemeg/cpstrnew
    function GetRedirectOnError: boolean;
    function GetRedirectOnErrorURL: string;
    procedure SetAdministrator(const AValue: String);
    procedure SetAllowDefaultModule(const AValue: Boolean);
    procedure SetApplicationURL(const AValue: String);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    function GetRedirectOnError: boolean;
    function GetRedirectOnErrorURL: string;
    procedure SetAdministrator(const AValue: String);
    procedure SetAllowDefaultModule(const AValue: Boolean);
    procedure SetApplicationURL(const AValue: String);
>>>>>>> graemeg/cpstrnew
=======
    function GetRedirectOnError: boolean;
    function GetRedirectOnErrorURL: string;
    procedure SetAdministrator(const AValue: String);
    procedure SetAllowDefaultModule(const AValue: Boolean);
    procedure SetApplicationURL(const AValue: String);
>>>>>>> origin/cpstrnew
    procedure SetEmail(const AValue: String);
    procedure SetHandleGetOnPost(const AValue: Boolean);
    procedure SetModuleVar(const AValue: String);
    procedure SetOnGetModule(const AValue: TGetModuleEvent);
    procedure SetOnShowRequestException(const AValue: TOnShowRequestException);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetOnUnknownRequestEncoding(AValue: TOnUnknownEncodingEvent);
    procedure SetRedirectOnError(const AValue: boolean);
    procedure SetRedirectOnErrorURL(const AValue: string);
    procedure SetPreferModuleName(const AValue: boolean);
    procedure DoOnTerminate(Sender : TObject);
  protected
    Procedure DoRun; override;
    Function CreateEventLog : TEventLog; virtual;
    function InitializeWebHandler: TWebHandler; virtual; abstract;
    Procedure DoLog(EventType: TEventType; const Msg: String); override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    procedure SetRedirectOnError(const AValue: boolean);
    procedure SetRedirectOnErrorURL(const AValue: string);
    procedure DoOnTerminate(Sender : TObject);
  protected
    Procedure DoRun; override;
    function InitializeWebHandler: TWebHandler; virtual; abstract;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    procedure SetRedirectOnError(const AValue: boolean);
    procedure SetRedirectOnErrorURL(const AValue: string);
    procedure DoOnTerminate(Sender : TObject);
  protected
    Procedure DoRun; override;
    function InitializeWebHandler: TWebHandler; virtual; abstract;
>>>>>>> origin/cpstrnew
    procedure SetTitle(const AValue: string); override;
    property WebHandler: TWebHandler read FWebHandler write FWebHandler;
  Public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    Procedure CreateForm(AClass : TComponentClass; out Reference);
    Procedure Initialize; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    Procedure Log(EventType: TEventType; const Msg: String); override;
>>>>>>> graemeg/cpstrnew
=======
    Procedure Log(EventType: TEventType; const Msg: String); override;
>>>>>>> graemeg/cpstrnew
=======
    Procedure Log(EventType: TEventType; const Msg: String); override;
>>>>>>> graemeg/cpstrnew
=======
    Procedure Log(EventType: TEventType; const Msg: String); override;
>>>>>>> origin/cpstrnew
    procedure Terminate; override;
    Property HandleGetOnPost : Boolean Read GetHandleGetOnPost Write SetHandleGetOnPost;
    Property RedirectOnError : boolean Read GetRedirectOnError Write SetRedirectOnError;
    Property RedirectOnErrorURL : string Read GetRedirectOnErrorURL Write SetRedirectOnErrorURL;
    Property ApplicationURL : String Read GetApplicationURL Write SetApplicationURL;
    Property AllowDefaultModule : Boolean Read GetAllowDefaultModule Write SetAllowDefaultModule;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property DefaultModuleName : String Read GetDefaultModuleName Write SetDefaultModuleName;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Property ModuleVariable : String Read GetModuleVar Write SetModuleVar;
    Property OnGetModule : TGetModuleEvent Read GetOnGetModule Write SetOnGetModule;
    Property Email : String Read GetEmail Write SetEmail;
    Property Administrator : String Read GetAdministrator Write SetAdministrator;
    property OnShowRequestException: TOnShowRequestException read GetOnShowRequestException write SetOnShowRequestException;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property OnUnknownRequestEncoding : TOnUnknownEncodingEvent Read GetOnUnknownRequestEncoding Write SetOnUnknownRequestEncoding;
    Property EventLog: TEventLog read GetEventLog;
    Property PreferModuleName : Boolean Read GetPreferModuleName Write SetPreferModuleName;
  end;

  EFPWebError = Class(EFPHTTPError);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Property EventLog: TEventLog read GetEventLog;
  end;

  EFPWebError = Class(Exception);
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

procedure ExceptionToHTML(S: TStrings; const E: Exception; const Title, Email, Administrator: string);

Implementation

{$ifdef CGIDEBUG}
uses
  dbugintf;
{$endif}

resourcestring
  SErrNoModuleNameForRequest = 'Could not determine HTTP module name for request';
  SErrNoModuleForRequest = 'Could not determine HTTP module for request "%s"';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrSendingContent = 'An error (%s) happened while sending response content: %s';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SModuleError = 'Module Error';
  SAppEncounteredError = 'The application encountered the following error:';
  SError = 'Error: ';
  SNotify = 'Notify: ';

procedure ExceptionToHTML(S: TStrings; const E: Exception; const Title, Email, Administrator: string);
var
  FrameNumber: Integer;
  Frames: PPointer;
  FrameCount: integer;
  TheEmail: String;
begin
  With S do
    begin
    Add('<html><head><title>'+Title+': '+SModuleError+'</title></head>'+LineEnding);
    Add('<body>');
    Add('<center><hr><h1>'+Title+': ERROR</h1><hr></center><br><br>');
    Add(SAppEncounteredError+'<br>');
    Add('<ul>');
    Add('<li>'+SError+' <b>'+E.Message+'</b>');
    Add('<li> Stack trace:<br>');
    Add(BackTraceStrFunc(ExceptAddr)+'<br>');
    FrameCount:=ExceptFrameCount;
    Frames:=ExceptFrames;
    for FrameNumber := 0 to FrameCount-1 do
      Add(BackTraceStrFunc(Frames[FrameNumber])+'<br>');
    Add('</ul><hr>');
    TheEmail:=Email;
    If (TheEmail<>'') then
      Add('<h5><p><i>'+SNotify+Administrator+': <a href="mailto:'+TheEmail+'">'+TheEmail+'</a></i></p></h5>');
    Add('</body></html>');
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TWebHandler.Run;
=======
procedure TWebHandler.Run;
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.Run;
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.Run;
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.Run;
>>>>>>> origin/cpstrnew
var ARequest : TRequest;
    AResponse : TResponse;
begin
  while not FTerminated do
    begin
    if WaitForRequest(ARequest,AResponse) then
      DoHandleRequest(ARequest,AResponse);
    if assigned(OnIdle) then
      OnIdle(Self);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TWebHandler.Log(EventType: TEventType; Const Msg: String);
begin
  If Assigned(FOnLog) then
    FOnLog(EventType,Msg);
end;

procedure TWebHandler.ShowRequestException(R: TResponse; E: Exception);

  Function GetStatusCode : integer;

  begin
    if (E is EHTTP) then
      Result:=EHTTP(E).StatusCode
    else
      Result:=E.HelpContext;
    if (Result=0) then
      Result:=500;
  end;

Var
  S : TStrings;
  handled: boolean;
  CT : String;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TWebHandler.ShowRequestException(R: TResponse; E: Exception);
Var
 S : TStrings;
 handled: boolean;
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

begin
  if R.ContentSent then exit;
  if assigned(OnShowRequestException) then
    begin
    handled:=false;
    OnShowRequestException(R,E,Handled);
    if handled then exit;
    end;
  If RedirectOnError and not R.HeadersSent then
    begin
    R.SendRedirect(format(RedirectOnErrorURL,[HTTPEncode(E.Message)]));
    R.SendContent;
    Exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  If (not R.HeadersSent) then
    begin
    R.Code:=GetStatusCode;
    if (E is EHTTP) Then
      CT:=EHTTP(E).StatusText
    else
      CT:='';
    if (CT='') then
      CT:='Application error '+E.ClassName;;
    R.CodeText:=CT;
    R.ContentType:='text/html';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  If not R.HeadersSent then
    begin
    R.ContentType:='text/html';
    R.SendHeaders;
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
  If (R.ContentType='text/html') then
    begin
    S:=TStringList.Create;
    Try
      ExceptionToHTML(S, E, Title, Email, Administrator);
      R.Content:=S.Text;
      R.SendContent;
    Finally
      FreeAndNil(S);
    end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TWebHandler.InitRequest(ARequest: TRequest);
begin
  ARequest.OnUnknownEncoding:=Self.OnUnknownRequestEncoding;
end;

Procedure TWebHandler.InitResponse(AResponse: TResponse);
begin
  // Do nothing
end;

Function TWebHandler.GetEmail: String;
=======
function TWebHandler.GetEmail: String;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetEmail: String;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetEmail: String;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetEmail: String;
>>>>>>> origin/cpstrnew
begin
  Result := FEmail;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TWebHandler.GetAdministrator: String;
=======
function TWebHandler.GetAdministrator: String;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetAdministrator: String;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetAdministrator: String;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetAdministrator: String;
>>>>>>> origin/cpstrnew
begin
  Result := FAdministrator;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TWebHandler.HandleRequest(ARequest: TRequest; AResponse: TResponse);
=======
procedure TWebHandler.HandleRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.HandleRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.HandleRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.HandleRequest(ARequest: TRequest; AResponse: TResponse);
>>>>>>> origin/cpstrnew
Var
  MC : TCustomHTTPModuleClass;
  M  : TCustomHTTPModule;
  MN : String;
  MI : TModuleItem;

begin
  try
    MC:=Nil;
    M:=NIL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    MI:=Nil;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    If (OnGetModule<>Nil) then
      OnGetModule(Self,ARequest,MC);
    If (MC=Nil) then
      begin
      MN:=GetModuleName(ARequest);
      MI:=ModuleFactory.FindModule(MN);
      if (MI=Nil) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        DoError(SErrNoModuleForRequest,[MN],400,'Not found');
=======
        Raise EFPWebError.CreateFmt(SErrNoModuleForRequest,[MN]);
>>>>>>> graemeg/cpstrnew
=======
        Raise EFPWebError.CreateFmt(SErrNoModuleForRequest,[MN]);
>>>>>>> graemeg/cpstrnew
=======
        Raise EFPWebError.CreateFmt(SErrNoModuleForRequest,[MN]);
>>>>>>> graemeg/cpstrnew
=======
        Raise EFPWebError.CreateFmt(SErrNoModuleForRequest,[MN]);
>>>>>>> origin/cpstrnew
      MC:=MI.ModuleClass;
      end;
    M:=FindModule(MC); // Check if a module exists already
    If (M=Nil) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if assigned(MI) and Mi.SkipStreaming then
=======
      if Mi.SkipStreaming then
>>>>>>> graemeg/cpstrnew
=======
      if Mi.SkipStreaming then
>>>>>>> graemeg/cpstrnew
=======
      if Mi.SkipStreaming then
>>>>>>> graemeg/cpstrnew
=======
      if Mi.SkipStreaming then
>>>>>>> origin/cpstrnew
        M:=MC.CreateNew(Self)
      else
        M:=MC.Create(Self);
    SetBaseURL(M,MN,ARequest);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if (OnInitModule<>Nil) then
      OnInitModule(Self,M);
    M.DoAfterInitModule(ARequest);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    if M.Kind=wkOneShot then
      begin
      try
        M.HandleRequest(ARequest,AResponse);
      finally
        M.Free;
      end;
      end
    else
      M.HandleRequest(ARequest,AResponse);
  except
    On E : Exception do
      ShowRequestException(AResponse,E);
  end;
end;

function TWebHandler.GetApplicationURL(ARequest: TRequest): String;
begin
  Result:=FApplicationURL;
  If (Result='') then
    Result:=ARequest.ScriptName;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Class Procedure TWebHandler.DoError(Msg : String;AStatusCode : Integer = 0; AStatusText : String = '');

Var
  E : EFPWebError;

begin
  E:=EFPWebError.Create(Msg);
  E.StatusCode:=AStatusCode;
  E.StatusText:=AStatusText;
  Raise E;
end;

Class Procedure TWebHandler.DoError(Fmt: String; Const Args: Array of const;
  AStatusCode: Integer = 0; AStatusText: String = '');
begin
  DoError(Format(Fmt,Args),AStatusCode,AStatusText);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TWebHandler.Terminate;
begin
  FTerminated := true;
  If Assigned(FOnTerminate) then 
    FOnTerminate(Self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TWebHandler.GetModuleName(Arequest: TRequest): string;
=======
function TWebHandler.GetModuleName(Arequest: TRequest): string;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetModuleName(Arequest: TRequest): string;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetModuleName(Arequest: TRequest): string;
>>>>>>> graemeg/cpstrnew
=======
function TWebHandler.GetModuleName(Arequest: TRequest): string;
>>>>>>> origin/cpstrnew

   Function GetDefaultModuleName : String;

   begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if (DefaultModuleName<>'') then
        Result:=DefaultModuleName
      else if (ModuleFactory.Count=1) then
=======
      If (ModuleFactory.Count=1) then
>>>>>>> graemeg/cpstrnew
=======
      If (ModuleFactory.Count=1) then
>>>>>>> graemeg/cpstrnew
=======
      If (ModuleFactory.Count=1) then
>>>>>>> graemeg/cpstrnew
=======
      If (ModuleFactory.Count=1) then
>>>>>>> origin/cpstrnew
        Result:=ModuleFactory[0].ModuleName;
   end;

var
  S : String;
  I : Integer;

begin
  If (FModuleVar<>'') then
    Result:=ARequest.QueryFields.Values[FModuleVar];//Module name from query parameter using the FModuleVar as parameter name (default is 'Module')
  If (Result='') then
    begin
    S:=ARequest.PathInfo;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    If (Length(S)>0) and (S[1]='/') then  
      Delete(S,1,1);                      //Delete the leading '/' if exists
    I:=Length(S);
    If (I>0) and (S[I]='/') then
      Delete(S,I,1);                      //Delete the trailing '/' if exists
    I:=Pos('/',S);
    if (I>0) or PreferModuleName then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    If (Length(S)>0) and (S[1]='/') then
      Delete(S,1,1);
    I:=Pos('/',S);
    if (I>0) then
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
      Result:=ARequest.GetNextPathInfo;
    end;
  If (Result='') then
    begin
    if Not AllowDefaultModule then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      DoError(SErrNoModuleNameForRequest,400,'Not found');
=======
      Raise EFPWebError.Create(SErrNoModuleNameForRequest);
>>>>>>> graemeg/cpstrnew
=======
      Raise EFPWebError.Create(SErrNoModuleNameForRequest);
>>>>>>> graemeg/cpstrnew
=======
      Raise EFPWebError.Create(SErrNoModuleNameForRequest);
>>>>>>> graemeg/cpstrnew
=======
      Raise EFPWebError.Create(SErrNoModuleNameForRequest);
>>>>>>> origin/cpstrnew
    Result:=GetDefaultModuleName
    end;
end;

procedure TWebHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);
begin
  AResponse.Free;
  ARequest.Free;
end;

function TWebHandler.FindModule(ModuleClass: TCustomHTTPModuleClass): TCustomHTTPModule;
Var
  I : Integer;
begin
  I:=ComponentCount-1;
  While (I>=0) and (Not ((Components[i] is ModuleClass) and (TCustomHTTPModule(Components[i]).Kind<>wkOneShot))) do
    Dec(i);
  if (I>=0) then
    Result:=Components[i] as TCustomHTTPModule
  else
    Result:=Nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TWebHandler.SetBaseURL(AModule: TCustomHTTPModule;
  Const AModuleName: String; ARequest: TRequest);
=======
procedure TWebHandler.SetBaseURL(AModule: TCustomHTTPModule;
  Const AModuleName : String; ARequest: TRequest);
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.SetBaseURL(AModule: TCustomHTTPModule;
  Const AModuleName : String; ARequest: TRequest);
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.SetBaseURL(AModule: TCustomHTTPModule;
  Const AModuleName : String; ARequest: TRequest);
>>>>>>> graemeg/cpstrnew
=======
procedure TWebHandler.SetBaseURL(AModule: TCustomHTTPModule;
  Const AModuleName : String; ARequest: TRequest);
>>>>>>> origin/cpstrnew

Var
  S,P : String;

begin
  S:=IncludeHTTPPathDelimiter(GetApplicationURL(ARequest));
  P:=IncludeHTTPPathDelimiter(ARequest.ReturnedPathInfo);
  If (P='') or (P='/') then
    P:=IncludeHTTPPathDelimiter(AModuleName);
  if (Length(P)>0) and (P[1]='/') then
    Delete(P,1,1);
{$ifdef CGIDEBUG}
  senddebug(Format('SetBaseURL : "%s" "%s"',[S,P]));
{$endif CGIDEBUG}
  AModule.BaseURL:=S+P;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TWebHandler.DoHandleRequest(ARequest: TRequest; AResponse: TResponse);
begin
  Try
    HandleRequest(ARequest,AResponse);
    If Not AResponse.ContentSent then
      try
        AResponse.SendContent;
      except
        On E : Exception do
          Log(etError,Format(SErrSendingContent,[E.ClassName,E.Message]));
      end;
  Finally
    EndRequest(ARequest,AResponse);
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TWebHandler.DoHandleRequest(ARequest: TRequest; AResponse: TResponse);
begin
  HandleRequest(ARequest,AResponse);
  If Not AResponse.ContentSent then
    AResponse.SendContent;
  EndRequest(ARequest,AResponse);
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

constructor TWebHandler.Create(AOwner:TComponent);
begin
  inherited Create(AOwner);
  FModuleVar:='Module'; // Do not localize
  FAllowDefaultModule:=True;
  FHandleGetOnPost := True;
  FRedirectOnError := False;
  FRedirectOnErrorURL := '';
end;

{ TCustomWebApplication }

function TCustomWebApplication.GetAdministrator: String;
begin
  result := FWebHandler.Administrator;
end;

function TCustomWebApplication.GetAllowDefaultModule: Boolean;
begin
  result := FWebHandler.AllowDefaultModule;
end;

function TCustomWebApplication.GetApplicationURL: String;
begin
  result := FWebHandler.ApplicationURL;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomWebApplication.GetDefaultModuleName: String;
begin
  Result:=FWebHandler.DefaultModuleName;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomWebApplication.GetEmail: String;
begin
  result := FWebHandler.Email;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomWebApplication.CreateEventLog: TEventLog;
begin
  Result:=TEventLog.Create(Nil);
  With Result do
    begin
    Name:=Self.Name+'Logger';
    Identification:=Title;
    RegisterMessageFile(ParamStr(0));
    LogType:=ltSystem;
    Active:=True;
    end;
end;

function TCustomWebApplication.GetEventLog: TEventLog;

begin
  if not assigned(FEventLog) then
    FEventLog := CreateEventLog;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomWebApplication.GetEventLog: TEventLog;
begin
  if not assigned(FEventLog) then
    FEventLog := TEventLog.Create(self);
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
  Result := FEventLog;
end;

function TCustomWebApplication.GetHandleGetOnPost: Boolean;
begin
  result := FWebHandler.HandleGetOnPost;
end;

function TCustomWebApplication.GetModuleVar: String;
begin
  result := FWebHandler.ModuleVariable;
end;

function TCustomWebApplication.GetOnGetModule: TGetModuleEvent;
begin
  result := FWebHandler.OnGetModule;
end;

function TCustomWebApplication.GetOnShowRequestException: TOnShowRequestException;
begin
  result := FWebHandler.OnShowRequestException;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomWebApplication.GetOnUnknownRequestEncoding: TOnUnknownEncodingEvent;
begin
  Result := FWebHandler.OnUnknownRequestEncoding
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomWebApplication.GetRedirectOnError: boolean;
begin
  result := FWebHandler.RedirectOnError;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomWebApplication.GetPreferModuleName: boolean;
begin
  result := FWebHandler.PreferModuleName;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomWebApplication.GetRedirectOnErrorURL: string;
begin
  result := FWebHandler.RedirectOnErrorURL;
end;

procedure TCustomWebApplication.SetAdministrator(const AValue: String);
begin
  FWebHandler.Administrator := AValue;
end;

procedure TCustomWebApplication.SetAllowDefaultModule(const AValue: Boolean);
begin
  FWebHandler.AllowDefaultModule := AValue;
end;

procedure TCustomWebApplication.SetApplicationURL(const AValue: String);
begin
  FWebHandler.ApplicationURL := AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomWebApplication.SetDefaultModuleName(AValue: String);
begin
  FWebHandler.DefaultModuleName:=AValue;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomWebApplication.SetEmail(const AValue: String);
begin
  FWebHandler.Email := AValue;
end;

procedure TCustomWebApplication.SetHandleGetOnPost(const AValue: Boolean);
begin
  FWebHandler.HandleGetOnPost := AValue;
end;

procedure TCustomWebApplication.SetModuleVar(const AValue: String);
begin
  FWebHandler.ModuleVariable := AValue;
end;

procedure TCustomWebApplication.SetOnGetModule(const AValue: TGetModuleEvent);
begin
  FWebHandler.OnGetModule := AValue;
end;

procedure TCustomWebApplication.SetOnShowRequestException(const AValue: TOnShowRequestException);
begin
  FWebHandler.OnShowRequestException := AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomWebApplication.SetOnUnknownRequestEncoding(
  AValue: TOnUnknownEncodingEvent);
begin
  FWebHandler.OnUnknownRequestEncoding:=AValue;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomWebApplication.SetRedirectOnError(const AValue: boolean);
begin
  FWebHandler.RedirectOnError := AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomWebApplication.SetPreferModuleName(const AValue: boolean);
begin
  FWebHandler.PreferModuleName := AValue;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomWebApplication.SetRedirectOnErrorURL(const AValue: string);
begin
  FWebHandler.RedirectOnErrorURL :=AValue;
end;

procedure TCustomWebApplication.DoRun;
begin
  FWebHandler.Run;
end;

procedure TCustomWebApplication.SetTitle(const AValue: string);
begin
  inherited SetTitle(AValue);
  FWebHandler.Title := Title;
end;

constructor TCustomWebApplication.Create(AOwner: TComponent);
begin
  Inherited Create(AOwner);
  FWebHandler := InitializeWebHandler;
  FWebHandler.FOnTerminate:=@DoOnTerminate;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FWebHandler.FOnLog:=@Log;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TCustomWebApplication.DoOnTerminate(Sender : TObject);
begin
  If Not Terminated then
    Terminate;
end;

destructor TCustomWebApplication.Destroy;
begin
  FWebHandler.Free;
  if assigned(FEventLog) then
    FEventLog.Free;
end;

procedure TCustomWebApplication.CreateForm(AClass: TComponentClass; out Reference);
begin
  TComponent(Reference):=AClass.Create(FWebHandler);
end;

procedure TCustomWebApplication.Initialize;
begin
  StopOnException:=True;
  Inherited;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomWebApplication.DoLog(EventType: TEventType; const Msg: String);
=======
procedure TCustomWebApplication.Log(EventType: TEventType; const Msg: String);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomWebApplication.Log(EventType: TEventType; const Msg: String);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomWebApplication.Log(EventType: TEventType; const Msg: String);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomWebApplication.Log(EventType: TEventType; const Msg: String);
>>>>>>> origin/cpstrnew
begin
  EventLog.log(EventType,Msg);
end;

Procedure TCustomWebApplication.Terminate;

begin
  Inherited;
  If Not Webhandler.FTerminated then
    WebHandler.Terminate;
end;


end.

