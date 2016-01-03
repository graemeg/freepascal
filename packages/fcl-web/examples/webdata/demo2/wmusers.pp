unit wmusers; 

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, fpwebdata, extjsjson, extjsxml,
=======
  Classes, SysUtils, FileUtil, LResources, fpwebdata, extjsjson, extjsxml,
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, fpwebdata, extjsjson, extjsxml,
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, fpwebdata, extjsjson, extjsxml,
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, fpwebdata, extjsjson, extjsxml,
>>>>>>> origin/cpstrnew
  HTTPDefs, websession, fpHTTP, fpWeb, dbf, db;

type

  { TFPWebModule1 }

  TFPWebModule1 = class(TFPWebModule)
    Datasource1: TDatasource;
    Dbf1: TDbf;
    ExtJSJSONDataFormatter1: TExtJSJSONDataFormatter;
    ExtJSJSonWebdataInputAdaptor1: TExtJSJSonWebdataInputAdaptor;
    ExtJSXMLDataFormatter1: TExtJSXMLDataFormatter;
    ExtJSXMLWebdataInputAdaptor1: TExtJSXMLWebdataInputAdaptor;
    FPWebDataProvider1: TFPWebDataProvider;
    procedure DataModuleCreate(Sender: TObject);
    procedure DataModuleRequest(Sender: TObject; ARequest: TRequest;
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
uses dbugintf;

{ TFPWebModule1 }

procedure TFPWebModule1.DataModuleRequest(Sender: TObject; ARequest: TRequest;
  AResponse: TResponse; var Handled: Boolean);

Var
  CP : TCustomHTTPDataContentProducer;
  M : TMemoryStream;
  A : TCustomWebDataInputAdaptor;
  PN : String;

begin
  PN:=ARequest.GetNextPathInfo;
  If ARequest.QueryFields.values['format']='xml' then
    begin
    A:=ExtJSXMLWebdataInputAdaptor1;
    CP:=ExtJSXMLDataFormatter1;
    end
  else
    begin
    A:=ExtJSJSONWebdataInputAdaptor1;
    cp:=ExtJSJSONDataFormatter1;
    end;
  FPWebDataProvider1.Adaptor:=A;
  CP.Adaptor:=A;
  CP.Adaptor.Request:=ARequest;
  CP.Provider:=FPWebDataProvider1;
  M:=TmemoryStream.Create;
  AResponse.ContentStream:=M;
  AResponse.ContentType:=CP.DataContentType;
  CP.GetContent(ARequest,M,Handled);
  M.Position:=0;
  AResponse.SendContent;
end;

procedure TFPWebModule1.DataModuleCreate(Sender: TObject);
begin
  DBF1.TableName:=ExtractFilePath(ParamStr(0))+'users.dbf';
  DBF1.Open;
  With DBF1.FieldByname('ID') do
    ProviderFlags:=ProviderFlags+[pfInKey];
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

