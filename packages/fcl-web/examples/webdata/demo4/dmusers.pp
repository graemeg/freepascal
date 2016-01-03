unit dmusers;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, db, dbf, fpwebdata;
=======
  Classes, SysUtils, FileUtil, LResources, db, dbf,
  fpwebdata;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, db, dbf,
  fpwebdata;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, db, dbf,
  fpwebdata;
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, db, dbf,
  fpwebdata;
>>>>>>> origin/cpstrnew

type

  { TDataModule1 }

  TDataModule1 = class(TDataModule)
    AName: TFPWebDataProvider;
    Datasource1: TDatasource;
    Dbf1: TDbf;
    procedure DataModuleCreate(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  DataModule1: TDataModule1; 

implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$R *.lfm}

=======
uses dbugintf;
>>>>>>> graemeg/cpstrnew
=======
uses dbugintf;
>>>>>>> graemeg/cpstrnew
=======
uses dbugintf;
>>>>>>> graemeg/cpstrnew
=======
uses dbugintf;
>>>>>>> origin/cpstrnew
{ TDataModule1 }

procedure TDataModule1.DataModuleCreate(Sender: TObject);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  senddebug('Creating datamodule 1 '+InTToStr(Ord(WebDataProviderManager.Registering)));
>>>>>>> graemeg/cpstrnew
=======
  senddebug('Creating datamodule 1 '+InTToStr(Ord(WebDataProviderManager.Registering)));
>>>>>>> graemeg/cpstrnew
=======
  senddebug('Creating datamodule 1 '+InTToStr(Ord(WebDataProviderManager.Registering)));
>>>>>>> graemeg/cpstrnew
=======
  senddebug('Creating datamodule 1 '+InTToStr(Ord(WebDataProviderManager.Registering)));
>>>>>>> origin/cpstrnew
  If not WebDataProviderManager.Registering then
    begin
    DBF1.TableName:=ExtractFilePath(ParamStr(0))+'users.dbf';
    DBF1.Open;
    With DBF1.FieldByname('ID') do
      ProviderFlags:=ProviderFlags+[pfInKey];
    DBF1.First;
    end;
end;

initialization
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  {$I dmusers.lrs}
>>>>>>> graemeg/cpstrnew
=======
  {$I dmusers.lrs}
>>>>>>> graemeg/cpstrnew
=======
  {$I dmusers.lrs}
>>>>>>> graemeg/cpstrnew
=======
  {$I dmusers.lrs}
>>>>>>> origin/cpstrnew
  WebDataProviderManager.RegisterDatamodule(TDataModule1)

end.

