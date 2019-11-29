{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2019 by the Free Pascal development team

    SQLDB REST rest bridge demo applocation:  readonly access

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program demorestbridge;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Classes, SysUtils, CustApp, sqldbrestbridge, fphttpapp, IBConnection, odbcconn, mysql55conn, mysql56conn, pqconnection,
  mssqlconn, oracleconnection, sqldbrestxml, sqldbrestio, sqldbrestschema, sqldbrestdata, sqldbrestjson, sqldbrestcsv, sqldbrestcds,
  sqldbrestado,  sqldbrestconst, sqldbrestauth, sqldbrestini, sqldb, sqldbrestauthini;

type
  { TRestServerDemoApplication }

  TRestServerDemoApplication = class(THTTPApplication)
  private
    procedure DisplayInfo;
    function SetUpConnection: TSQLDBRestConnection;
  Protected
    FDisp : TSQLDBRestDispatcher;
  protected
    procedure DoRun; override;
  public
    constructor Create(TheOwner: TComponent); override;
    destructor Destroy; override;
    procedure WriteHelp; virtual;
  end;

{ TRestServerDemoApplication }

procedure TRestServerDemoApplication.DoRun;
var
  ErrorMsg: String;
  C : TSQLDBRestConnection;
  I : Integer;

begin
  // quick check parameters
  ErrorMsg:=CheckOptions('hc:p:q', ['help','config:','port:','quiet']);
  if ErrorMsg<>'' then 
    begin
    ShowException(Exception.Create(ErrorMsg));
    Terminate;
    Exit;
    end;
  // parse parameters
  if HasOption('h', 'help') then 
    begin
    WriteHelp;
    Terminate;
    Exit;
    end;
  Port:=StrToIntDef(GetOptionValue('p','port'),3000);
  // Set up dispatcher

  // Create connection
  C:=SetUpConnection;
  // Allow filtering
  FDisp.ExposeConnection(C,Nil,[foFilter]);
  // Mark resources as read-only
  With FDisp.Schemas[0].Schema.Resources do
    For I:=0 to Count-1 do
      Resources[i].AllowedOperations:=[roGet,roHead,roOptions];
  FDisp.Active:=true;
  if not HasOption('q','quiet') then
    DisplayInfo;
  Inherited DoRun;
end;

constructor TRestServerDemoApplication.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  // Create dispatcher
  FDisp:=TSQLDBRestDispatcher.Create(Self);
  StopOnException:=True;
end;

destructor TRestServerDemoApplication.Destroy;
begin
  FreeAndNil(FDisp);
  inherited Destroy;
end;

Function TRestServerDemoApplication.SetUpConnection : TSQLDBRestConnection;

Var
  C : TSQLDBRestConnection;
  FN : String;
begin
  // Create with empty connection config.
  C:=FDisp.Connections.AddConnection('','','','','');
  C.name:='connection';
  // Read connection settings if available
  FN:=GetOptionValue('c', 'config');
  if FN='' then
    FN:='connection.ini';
  if FileExists(FN) then
    C.LoadFromIniFile(FN,'database',[])
  else
    begin
    // Or set in code.
    C.ConnectionType:=TPQConnectionDef.TypeName;
    C.DatabaseName:='fpctest';
    C.HostName:='localhost';
    C.UserName:='user';
    C.Password:='secret';
    end;
  Result:=C;
end;

procedure TRestServerDemoApplication.DisplayInfo;



Var
  I : integer;
  L : TStrings;
  C : TSQLDBRestConnection;

begin
  Writeln('Listening on port   : ',Port);
  Writeln('local URL           : http://localhost:',Port,'/'+FDisp.BasePath);
  C:=FDisp.Connections[0];
  Writeln('Database connection : Type=',C.ConnectionType,', Host: ',C.HostName,', Database: ',C.DatabaseName,', User: ',C.UserName);
  Writeln('Available resources :');
  With FDisp.Schemas[0].Schema.Resources do
    For I:=0 to Count-1 do
      Writeln('  ',Resources[i].ResourceName);
  L:=TStringList.Create;
  try
    Writeln('Available output formats:');
    TStreamerFactory.Instance.GetStreamerList(L,rstOutput);
    For I:=0 to L.Count-1 do
      Writeln('  ',L[i]);
    Writeln('Available Input formats:');
    L.Clear;
    TStreamerFactory.Instance.GetStreamerList(L,rstOutput);
    For I:=0 to L.Count-1 do
      Writeln('  ',L[i]);
  Finally
    L.Free;
  end;
end;

procedure TRestServerDemoApplication.WriteHelp;

begin
  writeln('Usage: ', ExeName, ' [options]');
  Writeln('Where options is one or more of:');
  Writeln('-h --help             this message');
  Writeln('-c --config=File      Read connection data from .ini file');
  Writeln('-p --port=N           Set listen port number');
  Writeln('-q --quiet            Do not display info');
end;

var
  Application: TRestServerDemoApplication;

begin
  Application:=TRestServerDemoApplication.Create(nil);
  Application.Title:='SQLDB REST bridge Application - Readonly';
  Application.Run;
  Application.Free;
end.

