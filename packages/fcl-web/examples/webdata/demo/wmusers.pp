unit wmusers;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, HTTPDefs, websession, fpHTTP, fpWeb,
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb,
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb,
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb,
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb,
>>>>>>> origin/cpstrnew
  db, dbf, fpwebdata, fpextjs,extjsjson,extjsxml;

type

  { TFPWebModule1 }

  TFPWebModule1 = class(TFPWebModule)
    Dbf1: TDbf;
    procedure TFPWebActions0Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions1Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions2Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
    procedure TFPWebActions3Request(Sender: TObject; ARequest: TRequest;
      AResponse: TResponse; var Handled: Boolean);
  private
    { private declarations }
    procedure GetAdaptorAndFormatter(P : TFPWebDataProvider; Var F :TExtJSDataFormatter; ARequest : TRequest; AResponse : TResponse);
  public
    { public declarations }
  end; 

var
  FPWebModule1: TFPWebModule1; 

Var
  ResponseFileName : String; // Set to non empty to write request responses to a file.

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
{$define wmdebug}

{$ifdef wmdebug}
uses dbugintf;
{$endif}

{ TFPWebModule1 }

Procedure SaveResponse(M : TStream);

begin
  if (ResponseFileName<>'') then
    With TFileStream.Create(ResponseFileName,fmCreate) do
      try
        CopyFrom(M,0);
      finally
        Free;
      end;
end;

procedure TFPWebModule1.GetAdaptorAndFormatter(P : TFPWebDataProvider; Var F :TExtJSDataFormatter; ARequest : TRequest; AResponse : TResponse);

begin
  If Request.QueryFields.values['format']='xml' then
    begin
    F:=TExtJSXMLDataFormatter.Create(Self);
    TExtJSXMLDataFormatter(F).TotalProperty:='total';
    AResponse.ContentType:='text/xml';
    P.Adaptor:=TExtJSXMLWebdataInputAdaptor.Create(Nil); 
    end
  else
    begin
    P.Adaptor:=TExtJSJSonWebdataInputAdaptor.Create(Nil); 
    F:=TExtJSJSONDataFormatter.Create(Self);
    end;
  P.Adaptor.Request:=ARequest;
  F.Adaptor:=P.Adaptor;
  F.Provider:=P;
end;

procedure TFPWebModule1.TFPWebActions0Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);

Var
  PN : String;
  P : TFPWebDataProvider;
  F : TExtJSDataFormatter;
  DS : TDatasource;
  M : TMemoryStream;
  L : Text;

begin
  // Providername;
  PN:=ARequest.GetNextPathInfo;
  P:=TFPWebDataProvider.Create(Self);
  try
    GetAdaptorAndFormatter(P,F,ARequest,AResponse);
    {$ifdef wmdebug} SendDebug(className+' '+F.ClassName);{$endif}
    try
      DS:=TDatasource.Create(Self);
      try
        DBF1.TableName:=ExtractFilePath(ParamStr(0))+'users.dbf';
        DS.Dataset:=DBf1;
        DBF1.Open;
        try
          P.Datasource:=DS;
          P.Adaptor.Action:=wdaRead;
          P.ApplyParams;
          M:=TMemoryStream.Create;
          try
            F.GetContent(ARequest,M,Handled);
            M.Position:=0;
            Response.ContentStream:=M;
            Response.SendResponse;
            Response.ContentStream:=Nil;
            SaveResponse(M);
          finally
            M.Free;
          end;
        finally
          DBF1.Close;
        end;
      finally
        DS.Free;
      end;
    finally
      F.Free;
    end;
  finally
    P.Free;
  end;
end;

procedure TFPWebModule1.TFPWebActions1Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);
Var
  PN : String;
  P : TFPWebDataProvider;
  F : TExtJSDataFormatter;
  DS : TDatasource;
  M : TMemoryStream;
  L : Text;

begin
  // Providername;
  PN:=ARequest.GetNextPathInfo;
//  P:=GetWebDataProvider(PN);
  P:=TFPWebDataProvider.Create(Self);
  try
    P.IDFieldName:='ID';
    GetAdaptorAndFormatter(P,F,ARequest,AResponse);
    {$ifdef wmdebug} SendDebug(className+' '+F.ClassName);{$endif}
    try
      DS:=TDatasource.Create(Self);
      try
        DBF1.TableName:=ExtractFilePath(ParamStr(0))+'users.dbf';
        DS.Dataset:=DBf1;
        DBF1.Open;
        try
          P.Datasource:=DS;
          P.Adaptor.Action:=wdaInsert;
          P.ApplyParams;
          M:=TMemoryStream.Create;
          try
            F.GetContent(ARequest,M,Handled);
            M.Position:=0;
            Response.ContentStream:=M;
            Response.SendResponse;
            Response.ContentStream:=Nil;
            SaveResponse(M);
          finally
            M.Free;
          end;
        finally
          DBF1.Close;
        end;
      finally
        DS.Free;
      end;
    finally
      F.Free;
    end;
  finally
    P.Free;
  end;
end;

procedure TFPWebModule1.TFPWebActions2Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);
Var
  PN : String;
  P : TFPWebDataProvider;
  F : TExtJSDataFormatter;
  DS : TDatasource;
  M : TMemoryStream;
  L : Text;

begin
  // Providername;
  {$ifdef wmdebug} SendDebug('Update request received');{$endif}
  PN:=ARequest.GetNextPathInfo;
//  P:=GetWebDataProvider(PN);
  P:=TFPWebDataProvider.Create(Self);
  try
    P.IDFieldName:='ID';
    GetAdaptorAndFormatter(P,F,ARequest,AResponse);
    {$ifdef wmdebug} SendDebug(className+' '+F.ClassName);{$endif}
    try
      DS:=TDatasource.Create(Self);
      try
        DBF1.TableName:=ExtractFilePath(ParamStr(0))+'users.dbf';
        DS.Dataset:=DBf1;
        DBF1.Open;
        try
          P.Datasource:=DS;
          P.Adaptor.Action:=wdaUpdate;
          P.ApplyParams;
          M:=TMemoryStream.Create;
          try
            F.GetContent(ARequest,M,Handled);
            M.Position:=0;
            Response.ContentStream:=M;
            Response.SendResponse;
            Response.ContentStream:=Nil;
            SaveResponse(M);
          finally
            M.Free;
          end;
        finally
          DBF1.Close;
        end;
      finally
        DS.Free;
      end;
    finally
      F.Free;
    end;
  finally
    P.Free;
  end;
end;

procedure TFPWebModule1.TFPWebActions3Request(Sender: TObject;
  ARequest: TRequest; AResponse: TResponse; var Handled: Boolean);

Var
  PN : String;
  P : TFPWebDataProvider;
  F : TExtJSDataFormatter;
  DS : TDatasource;
  M : TMemoryStream;
  L : Text;

begin
  // Providername;
  PN:=ARequest.GetNextPathInfo;
//  P:=GetWebDataProvider(PN);
  P:=TFPWebDataProvider.Create(Self);
  try
    P.IDFieldName:='ID';
    GetAdaptorAndFormatter(P,F,ARequest,AResponse);
    {$ifdef wmdebug} SendDebug('className '+F.ClassName);{$endif}
    try
      DS:=TDatasource.Create(Self);
      try
        DBF1.TableName:=ExtractFilePath(ParamStr(0))+'users.dbf';
        DS.Dataset:=DBf1;
        DBF1.Open;
        try
          P.Datasource:=DS;
          P.Adaptor.Action:=wdaDelete;
          P.ApplyParams;
          M:=TMemoryStream.Create;
          try
            F.GetContent(ARequest,M,Handled);
            M.Position:=0;
            Response.ContentStream:=M;
            Response.SendResponse;
            Response.ContentStream:=Nil;
            SaveResponse(M);
          finally
            M.Free;
          end;
        finally
          DBF1.Close;
        end;
      finally
        DS.Free;
      end;
    finally
      F.Free;
    end;
  finally
    P.Free;
  end;
end;

initialization
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  {$I wmusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
  {$I wmusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
  {$I wmusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
  {$I wmusers.lrs}

>>>>>>> origin/cpstrnew
  RegisterHTTPModule('Provider', TFPWebModule1);
end.

