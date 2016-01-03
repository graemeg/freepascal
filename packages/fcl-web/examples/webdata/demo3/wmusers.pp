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
  dbf, db, fpwebdata, extjsjson, extjsxml;

type

  { TFPWebProviderDataModule1 }

  TFPWebProviderDataModule1 = class(TFPWebProviderDataModule)
    Datasource1: TDatasource;
    Dbf1: TDbf;
    ExtJSJSONDataFormatter1: TExtJSJSONDataFormatter;
    ExtJSJSonWebdataInputAdaptor1: TExtJSJSonWebdataInputAdaptor;
    ExtJSXMLDataFormatter1: TExtJSXMLDataFormatter;
    ExtJSXMLWebdataInputAdaptor1: TExtJSXMLWebdataInputAdaptor;
    AName: TFPWebDataProvider;
    procedure DataModuleCreate(Sender: TObject);
    procedure DataModuleGetContentProducer(Sender: TObject;
      var AContentProducer: TCustomHTTPDataContentProducer);
    procedure DataModuleGetInputAdaptor(Sender: TObject;
      var AInputAdaptor: TCustomWebdataInputAdaptor);
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  FPWebProviderDataModule1: TFPWebProviderDataModule1; 

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
{ TFPWebProviderDataModule1 }

procedure TFPWebProviderDataModule1.DataModuleGetInputAdaptor(Sender: TObject;
  var AInputAdaptor: TCustomWebdataInputAdaptor);
begin
  If Request.QueryFields.values['format']='xml' then
    AInputAdaptor:=ExtJSXMLWebdataInputAdaptor1;
end;

procedure TFPWebProviderDataModule1.DataModuleGetContentProducer(
  Sender: TObject; var AContentProducer: TCustomHTTPDataContentProducer);
begin
  If Request.QueryFields.values['format']='xml' then
    AContentProducer:=ExtJSXMLDataFormatter1;
end;

procedure TFPWebProviderDataModule1.DataModuleCreate(Sender: TObject);
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
  RegisterHTTPModule('Provider', TFPWebProviderDataModule1);
end.

