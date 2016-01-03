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
unit websession;

{$mode objfpc}{$H+}
{ $define cgidebug}
interface

uses
  Classes, SysUtils, fphttp, inifiles, httpdefs;
  
Type

  { TSessionHTTPModule }

  TSessionHTTPModule = Class(TCustomHTTPModule)
  Private
    FCreateSession : Boolean;
    FOnNewSession: TNotifyEvent;
    FOnSessionExpired: TNotifyEvent;
    FSession: TCustomSession;
    function GetSession: TCustomSession;
    procedure SetSession(const AValue: TCustomSession);
  Protected
    Procedure CheckSession(ARequest : TRequest);
    Procedure InitSession(AResponse : TResponse);
    Procedure UpdateSession(AResponse : TResponse);
  Public
    Procedure Notification(AComponent : TComponent;Operation : TOperation); override;
    Procedure Loaded; Override;
    Property CreateSession : Boolean Read FCreateSession Write FCreateSession;
    Property Session : TCustomSession Read GetSession Write SetSession;
    Property OnNewSession : TNotifyEvent Read FOnNewSession Write FOnNewSession;
    Property OnSessionExpired : TNotifyEvent Read FOnSessionExpired Write FOnSessionExpired;
  end;
  
  { TIniWebSession }

  TIniWebSession = Class(TCustomSession)
  Private
    FSessionStarted : Boolean;
    FCached: Boolean;
    FIniFile : TMemInifile;
    FSessionCookie: String;
    FSessionCookiePath: String;
    FSessionDir: String;
    FTerminated :Boolean;
    SID : String;
  Protected
    Procedure CheckSession;
    Function GetSessionID : String; override;
    Function GetSessionVariable(VarName : String) : String; override;
    procedure SetSessionVariable(VarName : String; const AValue: String); override;
    Property Cached : Boolean Read FCached Write FCached;
    property SessionCookie : String Read FSessionCookie Write FSessionCookie;
    Property SessionDir : String Read FSessionDir Write FSessionDir;
    Property SessionCookiePath : String Read FSessionCookiePath write FSessionCookiePath;
  Public
    Destructor Destroy; override;
    Procedure Terminate; override;
    Procedure UpdateResponse(AResponse : TResponse); override;
    Procedure InitSession(ARequest : TRequest; OnNewSession, OnExpired: TNotifyEvent); override;
    Procedure InitResponse(AResponse : TResponse); override;
    Procedure RemoveVariable(VariableName : String); override;
  end;

  TFPWebSession = Class(TIniWebSession)
  Public
    Property Cached;
    property SessionCookie;
    Property SessionCookiePath;
    Property SessionDir;
  end;

  EWebSessionError = Class(HTTPError);
  TGetSessionEvent = Procedure(Var ASession : TCustomSession) of object;


Var
  GlobalSessionDir : String;
  OnGetDefaultSession : TGetSessionEvent;

Function GetDefaultSession : TCustomSession;

implementation

{$ifdef cgidebug}
uses dbugintf;
{$endif}

Const
  // Sections in ini file
  SSession   = 'Session';
  SData      = 'Data';

  KeyStart   = 'Start';         // Start time of session
  KeyLast    = 'Start';         // Last seen time of session
  KeyTimeOut = 'Timeout';       // Timeout in seconds;

  SFPWebSession = 'FPWebSession'; // Cookie name for session.

resourcestring
  SErrSessionTerminated = 'No web session active: Session was terminated';
  SErrNoSession         = 'No web session active: Session was not started';

Function GetDefaultSession : TCustomSession;

Var
  W : TFPWebSession;

begin
{$ifdef cgidebug}SendMethodEnter('GetDefaultSession');{$endif}
  Result:=Nil;
  If (GlobalSessionDir='') then
    GlobalSessionDir:=IncludeTrailingPathDelimiter(GetTempDir(True));
  If Assigned(OnGetDefaultSession) then
    OnGetDefaultSession(Result);
  if (Result=Nil) then
    begin
    {$ifdef cgidebug}Senddebug('Creating iniwebsession');{$endif}
    W:=TFPWebSession.Create(Nil);
    W.SessionDir:=GlobalSessionDir;
    Result:=W;
    end;
{$ifdef cgidebug}SendMethodExit('GetDefaultSession');{$endif}
end;

{ TIniWebSession }

function TIniWebSession.GetSessionID: String;
begin
  If (SID='') then
    SID:=inherited GetSessionID;
  Result:=SID;
end;

Procedure TIniWebSession.CheckSession;

begin
  If Not Assigned(FInifile) then
    if FTerminated then
      Raise EWebSessionError.Create(SErrSessionTerminated)
    else
      Raise EWebSessionError.Create(SErrNoSession)

end;

function TIniWebSession.GetSessionVariable(VarName: String): String;
begin
  CheckSession;
  Result:=FIniFile.ReadString(SData,VarName,'');
end;

procedure TIniWebSession.SetSessionVariable(VarName: String;
  const AValue: String);
begin
  CheckSession;
  FIniFile.WriteString(SData,VarName,AValue);
  If Not Cached then
    TMemIniFile(FIniFile).UpdateFile;
end;

destructor TIniWebSession.Destroy;
begin
  If Cached and Assigned(FIniFile) then
    TMemIniFile(FIniFile).UpdateFile;
  FreeAndNil(FIniFile);
  inherited Destroy;
end;

procedure TIniWebSession.Terminate;
begin
  FTerminated:=True;
  If Assigned(FIniFile) Then
    begin
    DeleteFile(Finifile.FileName);
    FreeAndNil(FIniFile);
    end;
end;

procedure TIniWebSession.UpdateResponse(AResponse: TResponse);
begin
  // Do nothing. Init has done the job.
  If Cached and Assigned(FIniFile) then
    TMemIniFile(FIniFile).UpdateFile;
end;

procedure TIniWebSession.InitSession(ARequest: TRequest; OnNewSession,OnExpired: TNotifyEvent);

Var
  L,D   : TDateTime;
  T   : Integer;
  S : String;
begin
{$ifdef cgidebug}SendMethodEnter('TIniWebSession.InitSession');{$endif}
  If (SessionCookie='') then
    SessionCookie:=SFPWebSession;
  S:=ARequest.CookieFields.Values[SessionCookie];
  // have session cookie ?
  If (S<>'') then
    begin
{$ifdef cgidebug}SendDebug('Reading ini file:'+S);{$endif}
    FiniFile:=TMemIniFile.Create(IncludeTrailingPathDelimiter(SessionDir)+S);
    L:=Finifile.ReadDateTime(SSession,KeyLast,0);
{$ifdef cgidebug}
    If (L=0) then
    SendDebug('No datetime in inifile');
{$endif}
    T:=FIniFile.ReadInteger(SSession,KeyTimeOut,Self.TimeOutMinutes);
{$ifdef cgidebug}SendDebug('Timeout :'+IntToStr(t));{$endif}
{$ifdef cgidebug}SendDebug('Last    :'+FormatDateTime('yyyy/mm/dd hh:nn:ss.zzz',L));{$endif}
    If ((Now-L)>(T/(24*60))) then
      begin
{$ifdef cgidebug}SendDebug('Timeout :'+FloatToStr(T/(24*60)));{$endif}
{$ifdef cgidebug}SendDebug('Timeout :'+FormatDateTime('hh:nn:ss.zzz',(T/(24*60))));{$endif}
{$ifdef cgidebug}SendDebug('Diff    :'+FormatDateTime('hh:nn:ss.zzz',Now-L));{$endif}
{$ifdef cgidebug}SendDebug('Ini file session expired: '+S);{$endif}
      // Expire session.
      If Assigned(OnExpired) then
        OnExpired(Self);
      DeleteFile(FIniFIle.FileName);
      FreeAndNil(FInifile);
      S:='';
      end
    else
      SID:=S;
    end;
  If (S='') then
    begin
    If Assigned(OnNewSession) then
      OnNewSession(Self);
    GetSessionID;
{$ifdef cgidebug}SendDebug('Creating new Ini file : '+SessionID);{$endif}
    FIniFile:=TMemIniFile.Create(IncludeTrailingPathDelimiter(SessionDir)+SessionID);
    FIniFile.WriteDateTime(SSession,KeyStart,Now);
    FIniFile.WriteInteger(SSession,KeyTimeOut,Self.TimeOutMinutes);
    end;
  FIniFile.WriteDateTime(SSession,KeyLast,Now);
  If not FCached then
    FIniFile.UpdateFile;
  FSessionStarted:=True;
{$ifdef cgidebug}SendMethodExit('TIniWebSession.InitSession');{$endif}
end;

procedure TIniWebSession.InitResponse(AResponse: TResponse);

Var
  C : TCookie;

begin
{$ifdef cgidebug}SendMethodEnter('TIniWebSession.InitResponse');{$endif}
  If FSessionStarted then
    begin
{$ifdef cgidebug}SendDebug('Session started');{$endif}
    C:=AResponse.Cookies.Add;
    C.Name:=SessionCookie;
    C.Value:=SID;
    C.Path:=FSessionCookiePath;
    end
  else If FTerminated then
    begin
{$ifdef cgidebug}SendDebug('Session terminated');{$endif}
    C:=AResponse.Cookies.Add;
    C.Name:=SessionCookie;
    C.Value:='';
    end;
{$ifdef cgidebug}SendMethodExit('TIniWebSession.InitResponse');{$endif}
end;

procedure TIniWebSession.RemoveVariable(VariableName: String);
begin
{$ifdef cgidebug}SendMethodEnter('TIniWebSession.RemoveVariable');{$endif}
  CheckSession;
  FIniFile.DeleteKey(SData,VariableName);
  If Not Cached then
    TMemIniFile(FIniFile).UpdateFile;
{$ifdef cgidebug}SendMethodExit('TIniWebSession.RemoveVariable');{$endif}
end;


function TSessionHTTPModule.GetSession: TCustomSession;
begin
{$ifdef cgidebug}SendMethodEnter('SessionHTTPModule.GetSession');{$endif}
  If (csDesigning in ComponentState) then
    begin
{$ifdef cgidebug}SendDebug('Sending session');{$endif}
    Result:=FSession
    end
  else
    begin
    If (FSession=Nil) then
      begin
{$ifdef cgidebug}SendDebug('Getting default session');{$endif}
      FSession:=GetDefaultSession;
      end;
    Result:=FSession
    end;
{$ifdef cgidebug}SendMethodExit('SessionHTTPModule.GetSession');{$endif}
end;

procedure TSessionHTTPModule.SetSession(const AValue: TCustomSession);

begin
  if FSession<>AValue then
    begin
    If Assigned(FSession) then
      FSession.RemoveFreeNotification(Self);
    FSession:=AValue;
    If Assigned(FSession) then
      FSession.FreeNotification(Self);
    end;
end;

procedure TSessionHTTPModule.CheckSession(ARequest : TRequest);

Var
  S : TCustomSession;

begin
{$ifdef cgidebug}SendMethodEnter('SessionHTTPModule('+Name+').CheckSession');{$endif}
  If CreateSession and Assigned(FSession) then
    begin
    S:=FSession;
    FSession.InitSession(ARequest,FOnNewSession,FOnSessionExpired);
    end;
{$ifdef cgidebug}SendMethodExit('SessionHTTPModule('+Name+').CheckSession');{$endif}
end;

procedure TSessionHTTPModule.InitSession(AResponse: TResponse);
begin
{$ifdef cgidebug}SendMethodEnter('SessionHTTPModule('+Name+').InitSession');{$endif}
  If CreateSession and Assigned(FSession) then
    FSession.InitResponse(AResponse);
{$ifdef cgidebug}SendMethodExit('SessionHTTPModule('+Name+').InitSession');{$endif}
end;

procedure TSessionHTTPModule.UpdateSession(AResponse: TResponse);
begin
  If CreateSession And Assigned(FSession) then
    FSession.UpdateResponse(AResponse);
end;

procedure TSessionHTTPModule.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
{$ifdef cgidebug}SendMethodEnter('SessionHTTPModule('+Name+').Notification');{$endif}
  inherited Notification(AComponent, Operation);
  If (Operation=opRemove) then
    if (AComponent=FSession) Then
      FSession:=Nil;
{$ifdef cgidebug}SendMethodExit('SessionHTTPModule('+Name+').Notification');{$endif}
end;

procedure TSessionHTTPModule.Loaded;

begin
{$ifdef cgidebug}SendMethodEnter('SessionHTTPModule.Loaded');{$endif}
  inherited Loaded;
  If CreateSession And (FSession=Nil) then
    FSession:=GetDefaultSession;
{$ifdef cgidebug}SendMethodExit('SessionHTTPModule.Loaded');{$endif}
end;

end.

