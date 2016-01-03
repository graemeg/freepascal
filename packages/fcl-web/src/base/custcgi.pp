{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    TCGIApplication class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{ $define CGIDEBUG}
{$mode objfpc}
{$H+}

unit custcgi;

Interface

uses
  CustWeb,Classes,SysUtils, httpdefs;

Type
  { TCGIRequest }
  TCGIHandler = Class;

  TCGIRequest = Class(TRequest)
  Private
    FCGI : TCGIHandler;
    function GetCGIVar(Index: integer): String;
  Protected
    Function GetFieldValue(Index : Integer) : String; override;
    Procedure InitFromEnvironment;
    procedure ReadContent; override;
  Public
    Constructor CreateCGI(ACGI : TCGIHandler);
    Property GatewayInterface : String Index 1 Read GetCGIVar;
    Property RemoteIdent : String Index 2 read GetCGIVar;
    Property RemoteUser : String Index 3 read GetCGIVar;
    Property RequestMethod : String Index 4 read GetCGIVar;
    Property ServerName : String Index 5 read GetCGIVar;
    Property ServerProtocol : String Index 6 read GetCGIVar;
    Property ServerSoftware : String Index 7 read GetCGIVar;
  end;
  
  { TCGIResponse }

  TCGIResponse = Class(TResponse)
  private
    FCGI : TCGIHandler;
    FOutput : TStream;
  Protected
    Procedure DoSendHeaders(Headers : TStrings); override;
    Procedure DoSendContent; override;
  Public
    Constructor CreateCGI(ACGI : TCGIHandler; AStream : TStream);
  end;

  { TCustomCgiApplication }

  TCgiHandler = Class(TWebHandler)
  Private
    FResponse : TCGIResponse;
    FRequest : TCGIRequest;
    FOutput : TStream;
  protected
    Function GetEmail : String; override;
    Function GetAdministrator : String; override;
    Function CreateResponse(AOutput : TStream) : TCGIResponse; virtual;
    Function CreateRequest : TCGIRequest; virtual;
    function WaitForRequest(out ARequest : TRequest; out AResponse : TResponse) : boolean; override;
    procedure EndRequest(ARequest : TRequest;AResponse : TResponse); override;
  Public
    Procedure GetCGIVarList(List : TStrings);
    Property Request : TCGIRequest read FRequest;
    Property Response: TCGIResponse Read FResponse;
  end;

  { TCustomCgiApplication }

  TCustomCGIApplication = Class(TCustomWebApplication)
  private
    function GetRequest: TCGIRequest;
    function GetRequestVariable(VarName : String): String;
    function GetRequestVariableCount: Integer;
    function GetResponse: TCGIResponse;
  protected
    function InitializeWebHandler: TWebHandler; override;
  public
    Procedure ShowException(E: Exception);override;
    Property Request : TCGIRequest read GetRequest;
    Property Response: TCGIResponse Read GetResponse;
    Procedure AddResponse(Const S : String);
    Procedure AddResponse(Const Fmt : String; Args : Array of const);
    Procedure AddResponseLn(Const S : String);
    Procedure AddResponseLn(Const Fmt : String; Args : Array of const);
    Procedure GetCGIVarList(List : TStrings);
    Function VariableIsUploadedFile(Const VarName : String) : boolean;
    Function UploadedFileName(Const VarName : String) : String;
    Property RequestVariables[VarName : String] : String Read GetRequestVariable;
    Property RequestVariableCount : Integer Read GetRequestVariableCount;
  end;


ResourceString
  SWebMaster = 'webmaster';
  SErrNoContentLength = 'No content length passed from server!';

Implementation

uses
{$ifdef CGIDEBUG}
  dbugintf,
{$endif}
  iostream;

Const
  MapCgiToHTTP : TCGIVarArray =
   ({ 1: 'AUTH_TYPE'               } fieldWWWAuthenticate, // ?
    { 2: 'CONTENT_LENGTH'          } FieldContentLength,
    { 3: 'CONTENT_TYPE'            } FieldContentType,
    { 4: 'GATEWAY_INTERFACE'       } '',
    { 5: 'PATH_INFO'               } '',
    { 6: 'PATH_TRANSLATED'         } '',
    { 7: 'QUERY_STRING'            } '',
    { 8: 'REMOTE_ADDR'             } '',
    { 9: 'REMOTE_HOST'             } '',
    { 10: 'REMOTE_IDENT'           } '',
    { 11: 'REMOTE_USER'            } '',
    { 12: 'REQUEST_METHOD'         } '',
    { 13: 'SCRIPT_NAME'            } '',
    { 14: 'SERVER_NAME'            } '',
    { 15: 'SERVER_PORT'            } '',
    { 16: 'SERVER_PROTOCOL'        } '',
    { 17: 'SERVER_SOFTWARE'        } '',
    { 18: 'HTTP_ACCEPT'            } FieldAccept,
    { 19: 'HTTP_ACCEPT_CHARSET'    } FieldAcceptCharset,
    { 20: 'HTTP_ACCEPT_ENCODING'   } FieldAcceptEncoding,
    { 21: 'HTTP_IF_MODIFIED_SINCE' } FieldIfModifiedSince,
    { 22: 'HTTP_REFERER'           } FieldReferer,
    { 23: 'HTTP_USER_AGENT'        } FieldUserAgent,
    { 24: 'HTTP_COOKIE'            } FieldCookie,
     // Additional Apache vars
    { 25: 'HTTP_CONNECTION'        } FieldConnection,
    { 26: 'HTTP_ACCEPT_LANGUAGE'   } FieldAcceptLanguage,
    { 27: 'HTTP_HOST'              } '',
    { 28: 'SERVER_SIGNATURE'       } '',
    { 29: 'SERVER_ADDR'            } '',
    { 30: 'DOCUMENT_ROOT'          } '',
    { 31: 'SERVER_ADMIN'           } '',
    { 32: 'SCRIPT_FILENAME'        } '',
    { 33: 'REMOTE_PORT'            } '',
    { 34: 'REQUEST_URI'            } '',
    { 35: 'CONTENT'                } '',
    { 36: 'XHTTPREQUESTEDWITH'     } ''
  );

Procedure TCgiHandler.GetCGIVarList(List : TStrings);

Var
  I : Integer;

begin
  List.Clear;
  For I:=1 to cgiVarCount do
    List.Add(CGIVarNames[i]+'='+GetEnvironmentVariable(CGIVarNames[i]));
end;

Function TCgiHandler.GetEmail : String;

Var
  H : String;

begin
  Result:=inherited GetEmail;
  If (Result='') then
    begin
    H:=Request.ServerName;
    If (H<>'') then
      Result:=Administrator+'@'+H
    end;
end;

Function TCgiHandler.GetAdministrator : String;

begin
  Result:=Inherited GetAdministrator;
  If (result='') then
    Result:=SWebMaster;
end;

function TCgiHandler.CreateResponse(AOutput : TStream): TCGIResponse;
begin
  result := TCGIResponse.CreateCGI(Self,AOutput);
end;

function TCgiHandler.CreateRequest: TCGIRequest;
begin
  Result:=TCGIRequest.CreateCGI(Self);
end;

function TCgiHandler.WaitForRequest(out ARequest: TRequest; out AResponse: TResponse): boolean;
begin
  FRequest:=CreateRequest;
  FRequest.InitFromEnvironment;
  FRequest.InitRequestVars;
  FOutput:=TIOStream.Create(iosOutput);
  FResponse:=CreateResponse(FOutput);
  ARequest:=FRequest;
  AResponse:=FResponse;
  Result := True;
end;

procedure TCgiHandler.EndRequest(ARequest: TRequest; AResponse: TResponse);
begin
  inherited;
  FreeAndNil(FOutPut);
  Terminate;
end;

constructor TCgiRequest.CreateCGI(ACGI: TCgiHandler);
begin
  Inherited Create;
  FCGI:=ACGI;
end;

{ TCGIHTTPRequest }

function TCGIRequest.GetCGIVar(Index: integer): String;

Var
  R : String;

begin
  Case Index of
   1 : R:=GetEnvironmentVariable(CGIVarNames[4]); // Property GatewayInterface : String Index 1 Read GetCGIVar;
   2 : R:=GetEnvironmentVariable(CGIVarNames[10]); // Property RemoteIdent : String Index 2 read GetCGIVar;
   3 : R:=GetEnvironmentVariable(CGIVarNames[11]); // Property RemoteUser : String Index 3 read GetCGIVar;
   4 : R:=GetEnvironmentVariable(CGIVarNames[12]); // Property RequestMethod : String Index 4 read GetCGIVar;
   5 : R:=GetEnvironmentVariable(CGIVarNames[14]); // Property ServerName : String Index 5 read GetCGIVar;
   6 : R:=GetEnvironmentVariable(CGIVarNames[16]); // Property ServerProtocol : String Index 6 read GetCGIVar;
   7 : R:=GetEnvironmentVariable(CGIVarNames[17]); // Property ServerSoftware : String Index 7 read GetCGIVar;
  end;
  Result:=HTTPDecode(R);
end;

Procedure TCGIRequest.InitFromEnvironment;


Var
  I : Integer;
  N,V,OV : String;
  
  
begin
  For I:=1 to CGIVarCount do
    begin
    N:=MapCgiToHTTP[i];
    if (N<>'') then
      begin
      OV:=GetFieldByName(N);
      V:=GetEnvironmentVariable(CGIVarNames[I]);
      If (OV='') or (V<>'') then
        SetFieldByName(N,HTTPDecode(V));
      end;
    end;
end;

procedure TCGIRequest.ReadContent;
var
  I : TIOStream;
  Cl : Integer;
  B : Byte;
  retrycount: integer;
  BytesRead, a: longint;
begin
  Cl := ContentLength;
  I:=TIOStream.Create(iosInput);
  Try
    if (CL<>0) then
      begin
      // It can be that the complete content is not yet send by the server so repeat the read
      // until all data is really read
      SetLength(FContent,Cl);
      BytesRead:=0;
      repeat
      a := I.Read(FContent[BytesRead+1],Cl-BytesRead);
      BytesRead:=BytesRead+a;
      if a=0 then // In fact this can not happen, but the content could be delayed...
        begin
        sleep(10);
        a := I.Read(FContent[BytesRead+1],Cl-BytesRead);
        if a=0 then for retrycount := 0 to 149 do // timeout of about 15 seconds
          begin
          sleep(100);
          a := I.Read(FContent[BytesRead+1],Cl-BytesRead);
          if a<>0 then break;
          end;
        BytesRead:=BytesRead+a;
        end;
      until (BytesRead>=Cl) or (a=0);
      // In fact the request is incomplete, but this is not the place thrown an error for that
      if BytesRead<Cl then
        SetLength(FContent,BytesRead);
      end
    else
      begin
      FContent:='';
      While (I.Read(B,1)>0) do
        FContent:=FContent + chr(B);
      end;
  Finally
    I.Free;
  end;
  FContentRead:=True;
end;

Function TCGIRequest.GetFieldValue(Index : Integer) : String;

  Function DecodeVar(I : Integer) : String;
  
  begin
    Result:=HTTPDecode(GetEnvironmentVariable(CGIVarNames[I]));
  end;

begin
  Case Index of
    21,
    34 : Result:=DecodeVar(14); // Property ServerName and Host
    25 : Result:=Decodevar(5); // Property PathInfo
    26 : Result:=DecodeVar(6); // Property PathTranslated
    27 : Result:=DecodeVar(8); // Property RemoteAddress
    28 : Result:=DecodeVar(9); // Property RemoteHost
    29 : Result:=DecodeVar(13); // Property ScriptName
    30 : Result:=DecodeVar(15); // Property ServerPort
    31 : Result:=DecodeVar(12); // Property RequestMethod
    32 : Result:=DecodeVar(34); // Property URI
    33 : Result:=DecodeVar(7); // Property QueryString
    36 : Result:=DecodeVar(36); // Property XRequestedWith
  else
    Result:=Inherited GetFieldValue(Index);
  end;
end;


{ TCGIResponse }

procedure TCGIResponse.DoSendHeaders(Headers : TStrings);
begin
{$ifdef CGIDEBUG}
  SendMethodEnter('TCGIResponse.DoSendHeaders');
  SendDebug('Headers: '+Headers.Text);
{$endif}
  if Assigned(FOutput) then
    Headers.SaveToStream(FOutput);
{$ifdef CGIDEBUG}
  SendMethodExit('TCGIResponse.DoSendHeaders');
{$endif}
end;

procedure TCGIResponse.DoSendContent;
begin
{$ifdef CGIDEBUG}
  SendMethodEnter('TCGIResponse.DoSendContent');
{$endif}
  If Assigned(ContentStream) then
    FOutput.CopyFrom(ContentStream,0)
  else
    Contents.SaveToStream(FOutput);
{$ifdef CGIDEBUG}
  SendMethodExit('TCGIResponse.DoSendContent');
{$endif}
end;

constructor TCGIResponse.CreateCGI(ACGI: TCgiHandler; AStream: TStream);
begin
  inherited Create(ACGI.Request);
  FCGI:=ACGI;
  FOutput:=AStream;
end;

{ TCustomCGIApplication }

function TCustomCGIApplication.GetRequest: TCGIRequest;
begin
  result := TCgiHandler(WebHandler).Request;
end;

function TCustomCGIApplication.GetRequestVariable(VarName : String): String;
begin
  If Assigned(Request) then
    Result:=Request.QueryFields.Values[VarName]
  else
    Result:='';
end;

function TCustomCGIApplication.GetRequestVariableCount: Integer;
begin
  If Assigned(Request) then
     Result:=Request.QueryFields.Count
   else
     Result:=0;
end;

function TCustomCGIApplication.GetResponse: TCGIResponse;
begin

end;

function TCustomCGIApplication.InitializeWebHandler: TWebHandler;
begin
  Result:=TCgiHandler.Create(self);
end;

Procedure TCustomCGIApplication.ShowException(E: Exception);
var
  CgiHandler: TCgiHandler;
begin
  CgiHandler := WebHandler as TCgiHandler;
  if assigned(CgiHandler.FResponse) then
    CgiHandler.ShowRequestException(CgiHandler.FResponse,E)
  else
    inherited ShowException(E);
end;

procedure TCustomCGIApplication.AddResponse(const S: String);
Var
  L : Integer;

begin
  L:=Length(S);
  If L>0 then
    Response.Content:=Response.Content+S;
end;

procedure TCustomCGIApplication.AddResponse(const Fmt: String; Args: array of const);
begin
  AddResponse(Format(Fmt,Args));
end;

procedure TCustomCGIApplication.AddResponseLn(const S: String);
begin
  AddResponse(S+LineEnding);
end;

procedure TCustomCGIApplication.AddResponseLn(const Fmt: String; Args: array of const);
begin
  AddResponseLN(Format(Fmt,Args));
end;

procedure TCustomCGIApplication.GetCGIVarList(List: TStrings);
begin
  TCgiHandler(WebHandler).GetCGIVarList(list);
end;

function TCustomCGIApplication.VariableIsUploadedFile(const VarName: String): boolean;
begin
  Result:=Request.Files.IndexOfFile(VarName)<>-1;
end;

function TCustomCGIApplication.UploadedFileName(const VarName: String): String;
begin
  If VariableIsUploadedFile(VarName) then
    Result:=Request.Files.FileByName(VarName).LocalFileName
  else
    Result:='';
end;

initialization

finalization
{$ifdef CGIDEBUG}
  if (SendError<>'') then
    Writeln('Debug failed: ',SendError);
{$endif}
end.
