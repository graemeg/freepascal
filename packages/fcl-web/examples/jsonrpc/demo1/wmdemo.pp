unit wmdemo; 

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, HTTPDefs, websession, fpHTTP, fpWeb; 
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb; 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb; 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb; 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb; 
>>>>>>> origin/cpstrnew

type

  { TFPWebModule1 }

  TFPWebModule1 = class(TFPWebModule)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    procedure DataModuleCreate(Sender: TObject);
>>>>>>> graemeg/cpstrnew
=======
    procedure DataModuleCreate(Sender: TObject);
>>>>>>> graemeg/cpstrnew
=======
    procedure DataModuleCreate(Sender: TObject);
>>>>>>> graemeg/cpstrnew
=======
    procedure DataModuleCreate(Sender: TObject);
>>>>>>> origin/cpstrnew
    procedure TFPWebActions0Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions1Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions2Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions3Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions4Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions5Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions6Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  FPWebModule1: TFPWebModule1; 

implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$R *.lfm}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Uses fpjson,jsonparser,fpjsonrpc,webjsonrpc, fpextdirect;

{ TFPWebModule1 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TFPWebModule1.DataModuleCreate(Sender: TObject);
begin
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
procedure TFPWebModule1.TFPWebActions0Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);
{
  Demo 1. Manually do everything.
  Only a single method call is supported.
}
Var
  P : TJSONParser;
  Req,Res : TJSONData;
  Env,O : TJSONObject;
  M : TJSONStringType;
  E : TJSONRPCEcho;
  I : Integer;
  ID : TJSONData;
  Err : TJSONData;

begin
  Res:=Nil;
  Err:=Nil;
  ID:=Nil;
  try
    P:=TJSONParser.Create(ARequest.Content);
    try
      Req:=P.Parse;
      try
        If Not (Req is TJSONObject) then
          JSONRPCError(SErrRequestMustBeObject);
        O:=(Req as TJSONObject);
        I:=O.IndexOfName('id');
        If (I=-1) then
          JSONRPCError(SErrNoIDProperty);
        ID:=O.Items[i].Clone;
        if O.IndexOfName('method')=-1 then
          JSONRPCError(SErrNoMethodName);
        M:=O.Strings['method'];
        If (m<>'echo') then
          JSONRPCError('Only echo method is supported');
        E:=TJSONRPCEcho.Create(Self);
        try
          I:=O.IndexOfName('params');
          Res:=E.Execute(O.Items[i],Nil);
        finally
          E.Free;
        end;
      finally
        Req.Free;
      end;
    finally
      P.Free;
    end;
  except
    On E : Exception do
      Err:=TJSONObject.Create(['message',E.Message,'name',E.ClassName,'code',-1]);
  end;
  If Assigned(ID) and (ID.JSONType<>jtNull) then
    begin
    Env:=TJSONObject.Create();
    try
      If not Assigned(Res) then
        Res:=TJSONNull.Create;
      Env.Add('result',Res);
      If (Err=Nil) then
        Err:=TJSONNull.Create;
      Env.Add('error',Err);
      Env.Add('id',ID);
      AResponse.Content:=Env.AsJSON;
    finally
      Env.Free;
    end;
    end;
  AResponse.SendContent;
  Handled:=True;
end;

procedure TFPWebModule1.TFPWebActions1Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);

{
  Demo 2. Use a dispatcher to dispatch the requests.
  The handler is located on the owner module
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  (it is created run-time, though).
=======
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  (it is created run-time, though)
>>>>>>> origin/cpstrnew
}

Var
  Echohandler:TJSONRPCEcho;
  Disp : TJSONRPCDispatcher;
  P : TJSONParser;
  Req,res : TJSONData;
  O : TJSONRPCDispatchOptions;

begin
  Echohandler:=TJSONRPCEcho.Create(Self);
  try
    EchoHandler.Name:='echo';
    Disp:=TJSONRPCDispatcher.Create(Self);
    try
      O:=Disp.Options;
      Include(O,jdoRequireClass);
      Disp.Options:=O;
      P:= TJSONParser.Create(ARequest.Content);
      try
        Req:=P.Parse;
        try
          Res:=Nil;
          Res:=Disp.Execute(Req,Nil);
          try
            If Assigned(Res) then
              begin
              AResponse.Content:=Res.AsJSON;
              end;
            AResponse.SendContent;
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
    finally
      Disp.Free;
    end;
  finally
    EchoHandler.Free;
  end;

end;

procedure TFPWebModule1.TFPWebActions2Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Demo 3. Use a dispatcher to dispatch the requests.
  The handler is registered in the JSONFPCHandlerManager
  (it is created run-time, though).
=======
  Demo 3. Use a dispatcher to dispatch the requests,
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  Demo 3. Use a dispatcher to dispatch the requests,
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  Demo 3. Use a dispatcher to dispatch the requests,
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  Demo 3. Use a dispatcher to dispatch the requests,
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> origin/cpstrnew
}

Var
  Disp : TJSONRPCDispatcher;
  P : TJSONParser;
  Req,res : TJSONData;
  O : TJSONRPCDispatchOptions;

begin
  JSONRpcHandlerManager.RegisterHandler('','echo',TJSONRPCEcho);
  try
    Disp:=TJSONRPCDispatcher.Create(Self);
    try
      O:=Disp.Options;
      Include(O,jdoSearchRegistry);
      Disp.Options:=O;
      P:= TJSONParser.Create(ARequest.Content);
      try
        Req:=P.Parse;
        try
          Res:=Nil;
          Res:=Disp.Execute(Req,Nil);
          try
            If Assigned(Res) then
              begin
              AResponse.Content:=Res.AsJSON;
              end;
            AResponse.SendContent;
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
    finally
      Disp.Free;
    end;
  finally
    JSONRpcHandlerManager.UnRegisterHandler('','echo');
  end;
end;

procedure TFPWebModule1.TFPWebActions3Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);

{
  Demo 4. Ext.Direct dispatcher
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  The handler is registered in the JSONFPCHandlerManager
  (it is created run-time, though).
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> origin/cpstrnew
}

Var
  Disp : TExtDirectDispatcher;
  P : TJSONParser;
  Req,res : TJSONData;
  O : TJSONRPCDispatchOptions;

begin
  JSONRpcHandlerManager.RegisterHandler('test','echo',TJSONRPCEcho);
  try
    Disp:=TExtDirectDispatcher.Create(Self);
    try
      O:=Disp.Options;
      Include(O,jdoSearchRegistry);
      Disp.Options:=O;
      P:= TJSONParser.Create(ARequest.Content);
      try
        Req:=P.Parse;
        try
          Res:=Nil;
          Res:=Disp.Execute(Req,Nil);
          try
            If Assigned(Res) then
              begin
              AResponse.Content:=Res.AsJSON;
              end;
            AResponse.ContentLength:=Length(AResponse.Content);
            AResponse.SendContent;
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
    finally
      Disp.Free;
    end;
  finally
    JSONRpcHandlerManager.UnRegisterHandler('','echo');
  end;
end;

procedure TFPWebModule1.TFPWebActions4Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);

{
  Demo 5. Using a TJSONRPCContentProducer.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  The handler is registered in the JSONFPCHandlerManager
  (it is created run-time, though).
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> origin/cpstrnew
}

Var
  Cont : TJSONRPCContentProducer;
  disp : TJSONRPCDispatcher;

begin
  JSONRpcHandlerManager.RegisterHandler('test','echo',TJSONRPCEcho);
  try
    Cont:=TJSONRPCContentProducer.Create(Self);
    try
      disp:=TJSONRPCDispatcher.Create(Cont);
      Disp.Options:=Disp.OPtions+[jdoSearchRegistry];
      Cont.Dispatcher:=Disp;
      AResponse.ContentStream:=TMemoryStream.Create;
      try
        Cont.GetContent(ARequest,AResponse.ContentStream,Handled);
        AResponse.ContentLength:=AResponse.ContentStream.Size;
        If Handled then
          AResponse.SendContent;
      finally
        AResponse.ContentStream.Free;
      end;
    finally
      Cont.Free;
    end;
  finally
    JSONRpcHandlerManager.UnRegisterHandler('','echo');
  end;
end;

procedure TFPWebModule1.TFPWebActions5Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Demo 6. Creating an API response for Ext.Direct
  The handler is registered in the JSONFPCHandlerManager
  (it is created run-time, though).
=======
  Demo 6. creating an API response for Ext.Direct
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  Demo 6. creating an API response for Ext.Direct
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  Demo 6. creating an API response for Ext.Direct
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  Demo 6. creating an API response for Ext.Direct
  The handler is registered in the JSONFPCHandlerManager.
  (it is created run-time, though)
>>>>>>> origin/cpstrnew
}

Var
  D : TExtDirectDispatcher;
  I : Integer;

begin
  JSONRpcHandlerManager.RegisterHandler('test','echo',TJSONRPCEcho);
  try
    D:=TExtDirectDispatcher.Create(Self);
    try
      D.URL:=BaseURL+'ExtDirect';
      D.Options:=D.Options+[jdoSearchRegistry];
      AResponse.Content:=D.APIAsString;
      AResponse.ContentLength:=Length(AResponse.Content);
    finally
      D.Free;
    end;
  finally
    JSONRpcHandlerManager.UnRegisterHandler('','echo');
  end;
end;

procedure TFPWebModule1.TFPWebActions6Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);
{
  Demo 6. Using a TJSONRPCModule instance to handle the request.
  The handler is registered in the JSONFPCHandlerManager.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  (it is created run-time, though).
=======
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  (it is created run-time, though)
>>>>>>> graemeg/cpstrnew
=======
  (it is created run-time, though)
>>>>>>> origin/cpstrnew
}

Var
  M : TJSONRPCModule;

begin
  JSONRpcHandlerManager.RegisterHandler('test','echo',TJSONRPCEcho);
  try
    M:=TJSONRPCModule.CreateNew(Self,0);
    try
      M.HandleRequest(ARequest,AResponse);
      Handled:=True;
    finally
      M.Free;
    end;
  finally
    JSONRpcHandlerManager.UnRegisterHandler('','echo');
  end;
end;

initialization
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  {$I wmdemo.lrs}

>>>>>>> graemeg/cpstrnew
=======
  {$I wmdemo.lrs}

>>>>>>> graemeg/cpstrnew
=======
  {$I wmdemo.lrs}

>>>>>>> graemeg/cpstrnew
=======
  {$I wmdemo.lrs}

>>>>>>> origin/cpstrnew
  RegisterHTTPModule('echo', TFPWebModule1);
end.

