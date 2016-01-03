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
  IBConnection, sqldb, fpwebdata, extjsjson, extjsxml, sqldbwebdata;

type

  { TFPWebProviderDataModule1 }

  TFPWebProviderDataModule1 = class(TFPWebProviderDataModule)
    ExtJSJSONDataFormatter1: TExtJSJSONDataFormatter;
    ExtJSJSonWebdataInputAdaptor1: TExtJSJSonWebdataInputAdaptor;
    ExtJSXMLDataFormatter1: TExtJSXMLDataFormatter;
    ExtJSXMLWebdataInputAdaptor1: TExtJSXMLWebdataInputAdaptor;
    IBConnection1: TIBConnection;
    ANAME: TSQLDBWebDataProvider;
    QGetID: TSQLQuery;
    SQLQuery1: TSQLQuery;
    SQLTransaction1: TSQLTransaction;
    procedure ANAMEGetNewID(Sender: TObject; out AID: String);
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
uses inifiles;

{ TFPWebProviderDataModule1 }

procedure TFPWebProviderDataModule1.DataModuleGetContentProducer(
  Sender: TObject; var AContentProducer: TCustomHTTPDataContentProducer);
begin
  If Request.QueryFields.values['format']='xml' then
    AContentProducer:=ExtJSXMLDataFormatter1;
end;

procedure TFPWebProviderDataModule1.DataModuleCreate(Sender: TObject);

Var
  FN : String;
  Ini : TMemIniFile;

begin
  FN:=ChangeFileExt(Paramstr(0),'.ini');
  If FileExists(FN) then
    begin
    Ini:=TMemIniFile.Create(FN);
    try
      With IBConnection1 do
        begin
        DatabaseName:=Ini.ReadString('Database','Path',DatabaseName);
        UserName:=Ini.ReadString('Database','UserName',UserName);
        Password:=Ini.ReadString('Database','Password',Password);
        end;
    finally
      Ini.Free;
    end;
    end;
  IBConnection1.Connected:=True;
end;

procedure TFPWebProviderDataModule1.ANAMEGetNewID(Sender: TObject; out
  AID: String);
begin
  With QGetID Do
    begin
    Close;
    Open;
    try
      if (EOF and BOF) then
        Raise Exception.Create('No ID generated');
      AID:=Fields[0].AsString;
    finally
      Close;
    end;
    end;
end;

procedure TFPWebProviderDataModule1.DataModuleGetInputAdaptor(Sender: TObject;
  var AInputAdaptor: TCustomWebdataInputAdaptor);
begin
  If Request.QueryFields.values['format']='xml' then
    AInputAdaptor:=ExtJSXMLWebdataInputAdaptor1;
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

