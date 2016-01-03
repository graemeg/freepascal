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
{$mode objfpc}
{$H+}
unit fphttp;

Interface

uses sysutils,classes,httpdefs;

Type

  { THTTPContentProducer }
  TWebActionEvent = Procedure (Sender : TObject;
                               ARequest : TRequest;
                               AResponse : TResponse;
                               Var Handled : Boolean) of object;

  THTTPContentProducer = Class(TComponent)
  private
    FAfterResponse: TResponseEvent;
    FBeforeRequest: TRequestEvent;
    FRequest      : TRequest;
  Protected
    Procedure DoHandleRequest(ARequest : TRequest; AResponse : TResponse; Var Handled : Boolean); virtual;
    Procedure DoGetContent(ARequest : TRequest; Content : TStream; Var Handled : Boolean); virtual;
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse; Var Handled : Boolean);
    Function ProduceContent : String; virtual;
  Protected
    Property BeforeRequest : TRequestEvent Read FBeforeRequest Write FBeforeRequest;
    Property AfterResponse : TResponseEvent Read FAfterResponse Write FAfterResponse;
  Public
    Procedure GetContent(ARequest : TRequest; Content : TStream; Var Handled : Boolean);
    Function  HaveContent : Boolean; virtual;
    function ContentToStream(Stream : TStream) : boolean; virtual;
    Property Request : TRequest Read FRequest;
  end;
  
  { TCustomWebAction }
  TCustomWebAction = Class(TCollectionItem)
  private
    FAfterResponse: TResponseEvent;
    FBeforeRequest: TRequestEvent;
    FContentproducer: THTTPContentProducer;
    FDefault: Boolean;
    FName : String;
  Protected
    procedure SetContentProducer(const AValue: THTTPContentProducer);virtual;
    Function  GetDisplayName : String; override;
    Procedure SetDisplayName(AValue : String);
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse; Var Handled : Boolean);
    Procedure DoHandleRequest(ARequest : TRequest; AResponse : TResponse; Var Handled : Boolean); virtual;
  published
    Property Name : String Read GetDisplayName Write SetDisplayName;
    Property ContentProducer : THTTPContentProducer Read FContentproducer Write SetContentProducer;
    Property Default : Boolean Read FDefault Write FDefault;
    Property BeforeRequest : TRequestEvent Read FBeforeRequest Write FBeforeRequest;
    Property AfterResponse : TResponseEvent Read FAfterResponse Write FAfterResponse;
  end;

  { TCustomWebActions }
  TGetActionEvent = Procedure (Sender : TObject; ARequest : TRequest; Var ActionName : String) of object;

  TCustomWebActions = Class(TCollection)
  private
    FActionVar : String;
    FOnGetAction: TGetActionEvent;
    function GetActions(Index : Integer): TCustomWebAction;
    procedure SetActions(Index : Integer; const AValue: TCustomWebAction);
  Protected
    Function  GetRequestAction(ARequest: TRequest) : TCustomWebAction;
    Function  GetActionName(ARequest : TRequest) : String;
    Property  ActionVar : String Read FactionVar Write FActionVar;
  public
    Procedure Assign(Source : TPersistent); override;
    Function Add : TCustomWebAction;
    Function ActionByName(AName : String) : TCustomWebAction;
    Function FindAction(AName : String): TCustomWebAction;
    Function IndexOfAction(AName : String) : Integer;
    Property OnGetAction : TGetActionEvent Read FOnGetAction Write FOnGetAction;
    Property Actions[Index : Integer] : TCustomWebAction Read GetActions Write SetActions; Default;
  end;
  
  TCustomHTTPModule = Class(TDataModule)
  public
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); virtual; abstract;
  end;
  
  TCustomHTTPModuleClass = Class of TCustomHTTPModule;

  { TModuleItem }

  TModuleItem = Class(TCollectionItem)
  private
    FModuleClass: TCustomHTTPModuleClass;
    FModuleName: String;
  Public
    Property ModuleClass : TCustomHTTPModuleClass Read FModuleClass Write FModuleClass;
    Property ModuleName : String Read FModuleName Write FModuleName;
  end;

  { TModuleFactory }

  TModuleFactory = Class(TCollection)
  private
    function GetModule(Index : Integer): TModuleItem;
    procedure SetModule(Index : Integer; const AValue: TModuleItem);
  Public
    Function FindModule(AModuleName : String) : TModuleItem;
    Function ModuleByName(AModuleName : String) : TModuleItem;
    Function IndexOfModule(AModuleName : String) : Integer;
    Property Modules [Index : Integer]: TModuleItem Read GetModule Write SetModule;default;
  end;

  EFPHTTPError = Class(Exception);

Procedure RegisterHTTPModule(ModuleClass : TCustomHTTPModuleClass);
Procedure RegisterHTTPModule(Const ModuleName : String; ModuleClass : TCustomHTTPModuleClass);

Var
  ModuleFactory : TModuleFactory;
  
Resourcestring
  SErrNosuchModule = 'No such module registered: "%s"';
  SErrNoSuchAction = 'No action found for action: "%s"';
  SErrUnknownAction = 'Unknown action: "%s"';
  SErrNoDefaultAction = 'No action name and no default action';
  SErrRequestNotHandled = 'Web request was not handled by actions.';

Implementation

{$ifdef cgidebug}
uses dbugintf;
{$endif}


{ TModuleFactory }

function TModuleFactory.GetModule(Index : Integer): TModuleItem;
begin
  Result:=TModuleItem(Items[Index]);
end;

procedure TModuleFactory.SetModule(Index : Integer; const AValue: TModuleItem);
begin
  Items[Index]:=AValue;
end;

function TModuleFactory.FindModule(AModuleName: String): TModuleItem;

Var
  I : Integer;

begin
  I:=IndexOfModule(AModuleName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetModule(I);
end;

function TModuleFactory.ModuleByName(AModuleName: String): TModuleItem;
begin
  Result:=FindModule(AModuleName);
  If (Result=Nil) then
    Raise EFPHTTPError.CreateFmt(SErrNosuchModule,[AModuleName]);
end;

function TModuleFactory.IndexOfModule(AModuleName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(Modules[Result].ModuleName,AModuleName)<>0) do
    Dec(Result);
end;


procedure RegisterHTTPModule(ModuleClass: TCustomHTTPModuleClass);
begin
  RegisterHTTPModule(ModuleClass.ClassName,ModuleClass);
end;

procedure RegisterHTTPModule(const ModuleName: String;
  ModuleClass: TCustomHTTPModuleClass);
  
Var
  I : Integer;
  MI : TModuleItem;
  
begin
  I:=ModuleFactory.IndexOfModule(ModuleName);
  If (I=-1) then
    begin
    MI:=ModuleFactory.Add as TModuleItem;
    MI.ModuleName:=ModuleName;
    end
  else
    MI:=ModuleFactory[I];
  MI.ModuleClass:=ModuleClass;
end;

{ THTTPContentProducer }


procedure THTTPContentProducer.HandleRequest(ARequest: TRequest;
  AResponse: TResponse; Var Handled : Boolean);
  
begin
  If Assigned(FBeforeRequest) then
    FBeforeRequest(Self,ARequest);
  DoHandleRequest(Arequest,AResponse,Handled);
  If Assigned(FAfterResponse) then
    FAfterResponse(Self,AResponse);
end;

procedure THTTPContentProducer.GetContent(ARequest: TRequest; Content: TStream; Var Handled : Boolean);
begin
  If Assigned(FBeforeRequest) then
    FBeforeRequest(Self,ARequest);
  DoGetContent(Arequest,Content,Handled);
end;
  
procedure THTTPContentProducer.DoHandleRequest(ARequest: TRequest;
  AResponse: TResponse; Var Handled : Boolean);

Var
  M : TMemoryStream;
  
begin
  M:=TMemoryStream.Create;
  DoGetContent(ARequest,M,Handled);
  AResponse.ContentStream:=M;
end;

procedure THTTPContentProducer.DoGetContent(ARequest: TRequest; Content: TStream; Var Handled : Boolean);
begin
  FRequest := ARequest;
  Handled:=ContentToStream(Content);
end;

function THTTPContentProducer.ProduceContent: String;
begin
  Result:='';
end;

function THTTPContentProducer.HaveContent: Boolean;
begin
  Result:=(ProduceContent<>'');
end;

function THTTPContentProducer.ContentToStream(Stream: TStream) : boolean;

Var
  S : String;

begin
  S:=ProduceContent;
  If length(S)>0 then
    begin
    Stream.WriteBuffer(S[1],Length(S));
    Result := True;
    end
  else
    Result := False;
end;

{ TCustomWebAction }

procedure TCustomWebAction.SetContentProducer(const AValue: THTTPContentProducer
  );
begin
  FContentProducer:=AValue;
end;

function TCustomWebAction.GetDisplayName: String;
begin
  If (FName='') then
    FName:=ClassName+IntToStr(self.Index);
  Result:=FName;
end;

procedure TCustomWebAction.SetDisplayName(AValue: String);
begin
  Inherited;
  FName:=AValue;
end;

procedure TCustomWebAction.HandleRequest(ARequest: TRequest; AResponse: TResponse; Var Handled : Boolean);

begin
  If Assigned(FBeforeRequest) then
    FBeforeRequest(Self,ARequest);
  DoHandleRequest(Arequest,AResponse,Handled);
  If Assigned(FAfterResponse) then
    FAfterResponse(Self,AResponse);
end;

procedure TCustomWebAction.DoHandleRequest(ARequest: TRequest; AResponse: TResponse; Var Handled : Boolean);

begin
  If Assigned(FContentProducer) then
   FContentProducer.HandleRequest(ARequest,AResponse,Handled)
end;


{ TCustomWebActions }

function TCustomWebActions.GetActions(Index : Integer): TCustomWebAction;
begin
  Result:=TCustomWebAction(Items[Index]);
end;

procedure TCustomWebActions.SetActions(Index : Integer; const AValue: TCustomWebAction);
begin
  Items[Index]:=AValue;
end;

Function TCustomWebActions.GetRequestAction(ARequest: TRequest) : TCustomWebAction;

Var
  I : Integer;
  S : String;

begin
  Result:=Nil;
  S:=GetActionName(ARequest);
  If (S<>'') then
    Result:=FindAction(S)
  else
    begin
    I:=0;
    While (Result=Nil) and (I<Count) do
      begin
      If Actions[i].Default then
        Result:=Actions[i];
      Inc(i);
      end;
    If (Result=Nil) then
      Raise EFPHTTPError.Create(SErrNoDefaultAction);
    end;
end;


function TCustomWebActions.GetActionName(ARequest: TRequest): String;

begin
  If Assigned(FOnGetAction) then
    FOnGetAction(Self,ARequest,Result);
  If (Result='') then
    begin
    If (FActionVar<>'') then
      Result:=ARequest.QueryFields.Values[FActionVar];
    If (Result='') then
      Result:=ARequest.GetNextPathInfo;
    end;
end;

procedure TCustomWebActions.Assign(Source: TPersistent);
begin
  If (Source is TCustomWebActions) then
    ActionVar:=(Source as TCustomWebActions).ActionVar
  else
    inherited Assign(Source);
end;

function TCustomWebActions.Add: TCustomWebAction;
begin
  Result:=TCustomWebAction(Inherited Add);
end;

function TCustomWebActions.ActionByName(AName: String): TCustomWebAction;
begin
  Result:=FindAction(AName);
  If (Result=Nil) then
    Raise HTTPError.CreateFmt(SErrUnknownAction,[AName]);
end;

function TCustomWebActions.FindAction(AName: String): TCustomWebAction;

Var
  I : Integer;

begin
  I:=IndexOfAction(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=Actions[I];
end;

function TCustomWebActions.IndexOfAction(AName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(Actions[Result].Name,AName)<>0) do
    Dec(Result);
end;

Initialization
  ModuleFactory:=TModuleFactory.Create(TModuleItem);

Finalization
  FreeAndNil(ModuleFactory);
end.
