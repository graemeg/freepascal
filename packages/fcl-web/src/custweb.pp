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
  CustApp,Classes,SysUtils, httpdefs, fphttp;

Const
  CGIVarCount = 34;

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
    { 34 } 'REQUEST_URI'
    );

Type
  { TCustomWebApplication }
  TGetModuleEvent = Procedure (Sender : TObject; ARequest : TRequest;
                               Var ModuleClass : TCustomHTTPModuleClass) of object;

  TCustomWebApplication = Class(TCustomApplication)
  Private
    FAdministrator: String;
    FAllowDefaultModule: Boolean;
    FEmail: String;
    FModuleVar: String;
    FOnGetModule: TGetModuleEvent;
    FRequest : TRequest;
    FHandleGetOnPost : Boolean;
    FRedirectOnError : Boolean;
    FRedirectOnErrorURL : String;
  protected
    Function GetModuleName(Arequest : TRequest) : string;
    function WaitForRequest(out ARequest : TRequest; out AResponse : TResponse) : boolean; virtual; abstract;
    procedure EndRequest(ARequest : TRequest;AResponse : TResponse); virtual;
    function FindModule(ModuleClass : TCustomHTTPModuleClass): TCustomHTTPModule;
    Procedure DoRun; override;
    procedure ShowRequestException(R: TResponse; E: Exception); virtual;
    Function GetEmail : String; virtual;
    Function GetAdministrator : String; virtual;
  Public
    constructor Create(AOwner: TComponent); override;
    Procedure CreateForm(AClass : TComponentClass; Var Reference : TComponent);
    Procedure Initialize; override;
    Procedure ShowException(E: Exception);override;
    Procedure DoHandleRequest(ARequest : TRequest; AResponse : TResponse);
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); virtual;
    Property HandleGetOnPost : Boolean Read FHandleGetOnPost Write FHandleGetOnPost;
    Property RedirectOnError : boolean Read FRedirectOnError Write FRedirectOnError;
    Property RedirectOnErrorURL : string Read FRedirectOnErrorURL Write FRedirectOnErrorURL;
    Property Request : TRequest read FRequest;
    Property AllowDefaultModule : Boolean Read FAllowDefaultModule Write FAllowDefaultModule;
    Property ModuleVariable : String Read FModuleVar Write FModuleVar;
    Property OnGetModule : TGetModuleEvent Read FOnGetModule Write FOnGetModule;
    Property Email : String Read GetEmail Write FEmail;
    Property Administrator : String Read GetAdministrator Write FAdministrator;
  end;

  EFPWebError = Class(Exception);

Implementation

{$ifdef CGIDEBUG}
uses
  dbugintf;
{$endif}

resourcestring
  SErrNoModuleNameForRequest = 'Could not determine HTTP module name for request';
  SErrNoModuleForRequest = 'Could not determine HTTP module for request "%s"';
  SModuleError = 'Module Error';
  SAppEncounteredError = 'The application encountered the following error:';
  SError = 'Error: ';
  SNotify = 'Notify: ';

procedure TCustomWebApplication.DoRun;
var ARequest : TRequest;
    AResponse : TResponse;
begin
  while not Terminated do
    begin
    if WaitForRequest(ARequest,AResponse) then
      DoHandleRequest(ARequest,AResponse);
    end;
end;

procedure TCustomWebApplication.ShowRequestException(R: TResponse; E: Exception);
Var
 TheEmail : String;
 FrameCount: integer;
 Frames: PPointer;
 FrameNumber:Integer;
 S : TStrings;

begin
  if R.ContentSent then exit;
  If RedirectOnError and not R.HeadersSent then
    begin
    R.SendRedirect(format(RedirectOnErrorURL,[HTTPEncode(E.Message)]));
    R.SendContent;
    Exit;
    end;
  If not R.HeadersSent then
    begin
    R.ContentType:='text/html';
    R.SendHeaders;
    end;
  If (R.ContentType='text/html') then
    begin
    S:=TStringList.Create;
    Try
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
      R.Content:=S.Text;
      R.SendContent;
    Finally
      FreeAndNil(S);
    end;
    end;
end;

function TCustomWebApplication.GetEmail: String;
begin
  Result := FEmail;
end;

function TCustomWebApplication.GetAdministrator: String;
begin
  Result := FAdministrator;
end;

procedure TCustomWebApplication.ShowException(E: Exception);
var Buf:ShortString;
begin
{$ifdef CGIDEBUG}
  SetLength(Buf,ExceptionErrorMessage(ExceptObject,ExceptAddr,@Buf[1],255));
  senddebug('Exception: ' + Buf);
{$endif CGIDEBUG}
  inherited ShowException(E);
end;

procedure TCustomWebApplication.HandleRequest(ARequest: TRequest; AResponse: TResponse);
Var
  MC : TCustomHTTPModuleClass;
  M  : TCustomHTTPModule;
  MN : String;
  MI : TModuleItem;

begin
  try
    MC:=Nil;
    M:=NIL;
    If (OnGetModule<>Nil) then
      OnGetModule(Self,ARequest,MC);
    If (MC=Nil) then
      begin
      MN:=GetModuleName(ARequest);
      If (MN='') and Not AllowDefaultModule then
        Raise EFPWebError.Create(SErrNoModuleNameForRequest);
      MI:=ModuleFactory.FindModule(MN);
      If (MI=Nil) and (ModuleFactory.Count=1) then
        MI:=ModuleFactory[0];
      if (MI=Nil) then
        begin
        Raise EFPWebError.CreateFmt(SErrNoModuleForRequest,[MN]);
        end;
      MC:=MI.ModuleClass;
      end;
    M:=FindModule(MC); // Check if a module exists already
    If (M=Nil) then
      M:=MC.Create(Self);
    M.HandleRequest(ARequest,AResponse);
  except
    On E : Exception do
      begin
      ShowException(E);
      ShowRequestException(AResponse,E);
      end;
  end;
end;

Procedure TCustomWebApplication.Initialize;

begin
  StopOnException:=True;
  Inherited;
end;

function TCustomWebApplication.GetModuleName(Arequest: TRequest): string;
var
  S : String;
begin
  If (FModuleVar<>'') then
    Result:=ARequest.QueryFields.Values[FModuleVar];//Module name from query parameter using the FModuleVar as parameter name (default is 'Module')
  If (Result='') then
    begin
    S:=ARequest.PathInfo;
    Delete(S,1,1);
    if (Pos('/',S) <= 0) and AllowDefaultModule then
      Exit;//There is only 1 '/' in ARequest.PathInfo -> only ActionName is there -> use default module
    Result:=ARequest.GetNextPathInfo;
    end;
end;

procedure TCustomWebApplication.EndRequest(ARequest: TRequest; AResponse: TResponse);
begin
  AResponse.Free;
  ARequest.Free;
end;

function TCustomWebApplication.FindModule(ModuleClass: TCustomHTTPModuleClass): TCustomHTTPModule;
Var
  I : Integer;
begin
  I:=ComponentCount-1;
  While (I>=0) and (Not (Components[i] is ModuleClass)) do
    Dec(i);
  if (I>=0) then
    Result:=Components[i] as TCustomHTTPModule
  else
    Result:=Nil;
end;

procedure TCustomWebApplication.DoHandleRequest(ARequest: TRequest; AResponse: TResponse);
begin
  HandleRequest(ARequest,AResponse);
  If Not AResponse.ContentSent then
    AResponse.SendContent;
  EndRequest(ARequest,AResponse);
end;

constructor TCustomWebApplication.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FModuleVar:='Module'; // Do not localize
  FAllowDefaultModule:=True;
  FHandleGetOnPost := True;
  FRedirectOnError := False;
  FRedirectOnErrorURL := '';
end;

procedure TCustomWebApplication.CreateForm(AClass: TComponentClass; var Reference: TComponent);
begin
  Reference:=AClass.Create(Self);
end;

end.
