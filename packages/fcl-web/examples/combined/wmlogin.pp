unit wmlogin;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, HTTPDefs, websession, fpHTTP, fpWeb, fpjsonrpc, 
=======
  Classes, SysUtils, FileUtil, HTTPDefs, websession, fpHTTP, fpWeb, fpjsonrpc, 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, HTTPDefs, websession, fpHTTP, fpWeb, fpjsonrpc, 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, HTTPDefs, websession, fpHTTP, fpWeb, fpjsonrpc, 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, HTTPDefs, websession, fpHTTP, fpWeb, fpjsonrpc, 
>>>>>>> origin/cpstrnew
  fpjson, IBConnection, sqldb, webjsonrpc, fpextdirect, sqldbwebdata;

type

  { TSessionManagement }

  TSessionManagement = class(TExtDirectModule)
    IBConnection1: TIBConnection;
    Logout: TJSONRPCHandler;
    Login: TJSONRPCHandler;
    SessionManagement: TJSONRPCHandler;
    QAuthenticate: TSQLQuery;
    SQLTransaction1: TSQLTransaction;
    procedure DataModuleCreate(Sender: TObject);
    procedure LoginExecute(Sender: TObject; const Params: TJSONData;
      out Res: TJSONData);
    procedure LogoutExecute(Sender: TObject; const Params: TJSONData;
      out Res: TJSONData);
  private
    function AuthenticateUser(AUsername, APassword: String): Integer;
    procedure DoOnNewSession(Sender: TObject);
    { private declarations }
  public
    { public declarations }
  end; 

var
  SessionManagement: TSessionManagement;

implementation

uses inifiles;

{$R *.lfm}

{ TSessionManagement }

function TSessionManagement.AuthenticateUser(AUsername,APassword : String) : Integer;

begin
  With QAuthenticate do
    begin
    ParamByName('Login').AsString:=AUserName;
    ParamByName('Password').AsString:=APassword;
    Open;
    try
      if (EOF and BOF) then
        Result:=-1
      else
        begin
        Result:=FieldByName('U_ID').AsInteger;
        Session.Variables['UserName']:=FieldByName('U_NAME').AsString;
        end;
      Session.Variables['UserID']:=IntToStr(Result);
    finally
      Close;
    end;
    end;
end;

procedure TSessionManagement.LoginExecute(Sender: TObject;
  const Params: TJSONData; out Res: TJSONData);

Var
  A : TJSONArray ;
  AUserName,APassword : String;
begin
  A:=Params as TJSONArray;
  AUserName:=A.Strings[0];
  APassword:=A.Strings[1];
  Res:=TJSONIntegerNumber.Create(AuthenticateUser(AUsername,APassword));
end;

procedure TSessionManagement.LogoutExecute(Sender: TObject;
  const Params: TJSONData; out Res: TJSONData);
begin
  // To be sure
  Session.Variables['UserID']:='-1';
  Session.Terminate;
  // A result must always be sent back.
  Res:=TJSONString.Create('Bye');
end;

procedure TSessionManagement.DoOnNewSession(Sender : TObject);

begin
  // The cookies must all originate from the same path, otherwise the 2 datamodules will use a different session.
  (Sender as TFPWebSession).SessionCookiePath:='/';
end;

procedure TSessionManagement.DataModuleCreate(Sender: TObject);
Var
  FN : String;
  Ini : TMemIniFile;

begin
  // The following 2 statements are needed because the 2 properties are (currently) not published.
  OnNewSession:=@DoOnNewSession;
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

initialization
  RegisterHTTPModule('Login', TSessionManagement);
end.

