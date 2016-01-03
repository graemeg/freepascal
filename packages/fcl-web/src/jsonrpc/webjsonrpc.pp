<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the Free Component Library

    JSON-RPC functionality - http dependant part
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
unit webjsonrpc;

{$mode objfpc}{$H+}
{ $define debugjsonrpc}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, fpjson, fpjsonrpc, httpdefs, fphttp, jsonparser;
=======
  Classes, SysUtils, fpjson, fpjsonrpc, httpdefs, fphttp, jsonparser, websession;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, fpjson, fpjsonrpc, httpdefs, fphttp, jsonparser, websession;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, fpjson, fpjsonrpc, httpdefs, fphttp, jsonparser, websession;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, fpjson, fpjsonrpc, httpdefs, fphttp, jsonparser, websession;
>>>>>>> origin/cpstrnew

Type
{ ---------------------------------------------------------------------
  HTTP handling and content producing methods
  ---------------------------------------------------------------------}

  { TCustomJSONRPCContentProducer }

  TCustomJSONRPCContentProducer = Class(THTTPContentProducer)
  Protected
    Function GetIDProperty : String; virtual;
    Procedure DoGetContent(ARequest : TRequest; Content : TStream; Var Handled : Boolean); override;
    Function GetDispatcher : TCustomJSONRPCDispatcher; virtual; abstract;
  end;

  { TJSONRPCContentProducer }

  TJSONRPCContentProducer = Class(TCustomJSONRPCContentProducer)
  private
    FDispatcher: TCustomJSONRPCDispatcher;
    procedure SetDispatcher(const AValue: TCustomJSONRPCDispatcher);
  Protected
    Function GetDispatcher : TCustomJSONRPCDispatcher; override;
    procedure Notification(AComponent: TComponent; Operation: TOperation);override;
  Published
    Property Dispatcher :  TCustomJSONRPCDispatcher Read FDispatcher Write SetDispatcher;
  end;


  { TJSONRPCSessionContext }

  TJSONRPCSessionContext = Class(TJSONRPCCallContext)
  private
    FSession: TCustomSession;
  Public
    Constructor CreateSession(ASession : TCustomSession);
    Property Session : TCustomSession Read FSession;
  end;

  { TSessionJSONRPCDispatcher }

  TSessionJSONRPCDispatcher = Class(TCustomJSONRPCDispatcher)
  Protected
    Function FindHandler(Const AClassName,AMethodName : TJSONStringType;AContext : TJSONRPCCallContext; Out FreeObject : TComponent) : TCustomJSONRPCHandler; override;
  Published
    Property OnStartBatch;
    Property OnDispatchRequest;
    Property OnFindHandler;
    Property OnEndBatch;
    Property Options;
  end;

  { TJSONRPCDispatchModule }

  TJSONRPCDispatchModule = Class(TSessionHTTPModule)
  protected
    Function CreateContext : TJSONRPCSessionContext;
    Function DispatchRequest(Const ARequest : TRequest; ADispatcher : TCustomJSONRPCDispatcher) : TJSONData;
  end;

  { TCustomJSONRPCModule }

  TCustomJSONRPCModule = Class(TJSONRPCDispatchModule)
  private
    FDispatcher: TCustomJSONRPCDispatcher;
    FOptions: TJSONRPCDispatchOptions;
    FRequest: TRequest;
    FResponse: TResponse;
    procedure SetDispatcher(const AValue: TCustomJSONRPCDispatcher);
  Protected
    Function CreateDispatcher : TCustomJSONRPCDispatcher; virtual;
    procedure Notification(AComponent: TComponent; Operation: TOperation);override;
    Property Dispatcher :  TCustomJSONRPCDispatcher Read FDispatcher Write SetDispatcher;
    Property DispatchOptions : TJSONRPCDispatchOptions Read FOptions Write FOptions default DefaultDispatchOptions;
  Public
    Constructor CreateNew(AOwner : TComponent; CreateMode : Integer); override;
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); override;
    // Access to request
    Property Request: TRequest Read FRequest;
    // Access to response
    Property Response: TResponse Read FResponse;
  end;

  { TJSONRPCDataModule }

  TJSONRPCModule = Class(TCustomJSONRPCModule)
  Published
    Property Dispatcher;
    Property DispatchOptions;
  end;

implementation

{$ifdef debugjsonrpc}
uses dbugintf;
{$endif}

{ TCustomJSONRPCContentProducer }

function TCustomJSONRPCContentProducer.GetIDProperty: String;
begin
  Result:='id';
end;


procedure TCustomJSONRPCContentProducer.DoGetContent(ARequest: TRequest;
  Content: TStream; var Handled: Boolean);

Var
  Disp : TCustomJSONRPCDispatcher;
  P : TJSONParser;
  Req,res : TJSONData;
  R : String;

begin
  Disp:=Self.GetDispatcher;
  P:= TJSONParser.Create(ARequest.Content);
  try
    Res:=Nil;
    Req:=Nil;
    try
      try
        Req:=P.Parse;
        If (Disp<>Nil) then
          Res:=Disp.Execute(Req,Nil)
        else // No dispatcher, create error(s)
          Res:=CreateErrorForRequest(Req,CreateJSON2ErrorResponse(SErrNoDispatcher,EJSONRPCInternalError,Nil,GetIDProperty));
      except
        On E : Exception Do
          begin
          Res:=CreateJSON2ErrorResponse(E.Message,EJSONRPCParseError,Nil,GetIDProperty);
          end;
      end;
      try
        If Assigned(Res) then
          begin
          R:=Res.AsJSON;
          Content.WriteBuffer(R[1],Length(R));
          end;
        Handled:=True;
      finally
        FreeAndNil(Res);
      end;
    finally
      Req.Free;
    end;
  finally
    P.Free;
  end;
end;

{ TJSONRPCContentProducer }

procedure TJSONRPCContentProducer.SetDispatcher(
  const AValue: TCustomJSONRPCDispatcher);
begin
  if FDispatcher=AValue then exit;
  If Assigned(FDispatcher) then
    FDispatcher.RemoveFreeNotification(Self);
  FDispatcher:=AValue;
  If Assigned(FDispatcher) then
    FDispatcher.FreeNotification(Self);
end;

function TJSONRPCContentProducer.GetDispatcher: TCustomJSONRPCDispatcher;
begin
  Result:=FDispatcher;
end;

procedure TJSONRPCContentProducer.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  If (Operation=opRemove) and (AComponent=FDispatcher) then
    FDispatcher:=Nil;
end;

{ TCustomJSONRPCModule }

procedure TCustomJSONRPCModule.SetDispatcher(
  const AValue: TCustomJSONRPCDispatcher);
begin
  if FDispatcher=AValue then exit;
  If Assigned(FDispatcher) then
    FDispatcher.RemoveFreeNotification(Self);
  FDispatcher:=AValue;
  If Assigned(FDispatcher) then
    FDispatcher.FreeNotification(Self);
end;

function TCustomJSONRPCModule.CreateDispatcher: TCustomJSONRPCDispatcher;

Var
  S : TSessionJSONRPCDispatcher;

begin
  S:=TSessionJSONRPCDispatcher.Create(Self);
  S.Options:=DispatchOptions;
  Result:=S;
end;

procedure TCustomJSONRPCModule.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  If (Operation=opRemove) and (AComponent=FDispatcher) then
    FDispatcher:=Nil;
end;

constructor TCustomJSONRPCModule.CreateNew(AOwner: TComponent;
  CreateMode: Integer);
begin
  inherited CreateNew(AOwner, CreateMode);
  FOptions:=DefaultDispatchOptions+[jdoSearchRegistry];
end;



procedure TCustomJSONRPCModule.HandleRequest(ARequest: TRequest;
  AResponse: TResponse);

Var
  Disp : TCustomJSONRPCDispatcher;
  res : TJSONData;

begin
  If (Dispatcher=Nil) then
    Dispatcher:=CreateDispatcher;
  Disp:=Dispatcher;
  Res:=DispatchRequest(ARequest,Disp);
  try
    If Assigned(Res) then
      begin
      AResponse.Content:=Res.AsJSON;
      AResponse.ContentLength:=Length(AResponse.Content);
      end;
  AResponse.SendResponse;
  finally
    Res.Free;
  end;
end;

{ TJSONRPCSessionContext }

constructor TJSONRPCSessionContext.CreateSession(ASession: TCustomSession);
begin
  FSession:=ASession;
end;

{ TJSONRPCDispatchModule }

function TJSONRPCDispatchModule.CreateContext: TJSONRPCSessionContext;
begin
  If CreateSession then
    Result:=TJSONRPCSessionContext.CreateSession(Session)
  else
    Result:=TJSONRPCSessionContext.CreateSession(Nil);
end;

Function TJSONRPCDispatchModule.DispatchRequest(const ARequest: TRequest;
  ADispatcher: TCustomJSONRPCDispatcher): TJSONData;
var
  P : TJSONParser;
  Req : TJSONData;
  C : TJSONRPCSessionContext;


begin
  P:= TJSONParser.Create(ARequest.Content);
  try
    Result:=Nil;
    Req:=Nil;
    try
      try
        Req:=P.Parse;
        C:=CreateContext;
        try
         {$ifdef debugjsonrpc}SendDebugFmt('Dispatching request : "%s"',[Req.AsJSON]);{$endif}
          Result:=ADispatcher.Execute(Req,C);
        finally
          C.Free;
        end;
      except
        On E : Exception Do
          Result:=CreateJSON2ErrorResponse(E.Message,EJSONRPCParseError,Nil,ADispatcher.TransactionProperty);
      end;
    finally
      Req.Free;
    end;
  finally
    P.Free;
  end;
end;

{ TSessionJSONRPCDispatcher }

function TSessionJSONRPCDispatcher.FindHandler(const AClassName,
  AMethodName: TJSONStringType; AContext: TJSONRPCCallContext; out
  FreeObject: TComponent): TCustomJSONRPCHandler;
begin
  Result:=Inherited FindHandler(AClassName,AMethodName,AContext,FreeObject);
  If (AContext is TJSONRPCSessionContext) and (FreeObject is TCustomJSONRPCModule) then
    TCustomJSONRPCModule(FreeObject).Session:=TJSONRPCSessionContext(AContext).Session;
end;

end.

