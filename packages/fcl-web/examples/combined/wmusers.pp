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
  IBConnection, sqldb, fpwebdata, fpjson, extjsjson, extjsxml, sqldbwebdata;

type

  { TCombinedModule }

  TCombinedModule = class(TFPWebProviderDataModule)
    ProviderFormatter: TExtJSJSONDataFormatter;
    ProviderInputAdaptor: TExtJSJSonWebdataInputAdaptor;
    IBConnection1: TIBConnection;
    Users: TSQLDBWebDataProvider;
    QGetID: TSQLQuery;
    SQLTransaction1: TSQLTransaction;
    procedure DataModuleNewSession(Sender: TObject);
    procedure ProviderFormatterBeforeDataToJSON(Sender: TObject;
      AObject: TJSONObject);
    procedure ProviderFormatterBeforeDelete(Sender: TObject);
    procedure ProviderFormatterBeforeInsert(Sender: TObject);
    procedure ProviderFormatterBeforeUpdate(Sender: TObject);
    procedure UsersGetNewID(Sender: TObject; out AID: String);
    procedure DataModuleCreate(Sender: TObject);
    procedure DataModuleGetContentProducer(Sender: TObject;
      var AContentProducer: TCustomHTTPDataContentProducer);
    procedure DataModuleGetInputAdaptor(Sender: TObject;
      var AInputAdaptor: TCustomWebdataInputAdaptor);
  private
    procedure CheckLoggedIn;
    { private declarations }
  public
    { public declarations }
  end; 

var
  CombinedModule: TCombinedModule;

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

{ TCombinedModule }

procedure TCombinedModule.DataModuleGetContentProducer(
  Sender: TObject; var AContentProducer: TCustomHTTPDataContentProducer);
begin
end;

procedure TCombinedModule.DataModuleCreate(Sender: TObject);

Var
  FN : String;
  Ini : TMemIniFile;

begin
  // Not yet published.
  CreateSession:=True;
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

procedure TCombinedModule.UsersGetNewID(Sender: TObject; out
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

procedure TCombinedModule.CheckLoggedIn;

begin
  If StrToIntDef(Session.Variables['UserID'],-1)=-1 then
    Raise Exception.Create('You must be logged in to see or modify data');
end;
procedure TCombinedModule.ProviderFormatterBeforeDataToJSON(
  Sender: TObject; AObject: TJSONObject);
begin
  CheckLoggedIn;
end;

procedure TCombinedModule.DataModuleNewSession(Sender: TObject);
begin
  // The cookies must all originate from the same path, otherwise the 2 datamodules will use a different session.
  (Sender as TFPWebSession).SessionCookiePath:='/';
end;

procedure TCombinedModule.ProviderFormatterBeforeDelete(
  Sender: TObject);
begin
  CheckLoggedIn;
end;

procedure TCombinedModule.ProviderFormatterBeforeInsert(
  Sender: TObject);
begin
  CheckLoggedIn;
end;

procedure TCombinedModule.ProviderFormatterBeforeUpdate(
  Sender: TObject);
begin
  CheckLoggedIn;
end;

procedure TCombinedModule.DataModuleGetInputAdaptor(Sender: TObject;
  var AInputAdaptor: TCustomWebdataInputAdaptor);
begin
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
  RegisterHTTPModule('Provider', TCombinedModule);
end.

