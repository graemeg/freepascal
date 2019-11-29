program clientapp;

{$IFDEF WINDOWS}
{$APPTYPE CONSOLE}
{$ENDIF}
{$mode objfpc}{$H+}

uses
  custapp, Classes, SysUtils, enet, uenetclass;

type

  { TClientApplication }

  TClientApplication = class(TCustomApplication)
  Private
    myClient: TENetClass;
    procedure DoConnect;
    procedure OnConnect(const Event:ENetEvent);
    procedure OnReceive(const Event:ENetEvent; var BroadcastMsg : Boolean; var BroadcastChannel : Byte);
    procedure ProcessLoop;
    procedure SendMessage(AMessage: String);
    procedure WriteHelp;
  Protected
    Procedure DoRun; override;
  public
    Constructor Create(AOWner : TComponent); override;
    Destructor Destroy; override;
  end;

Var
  Application : TClientApplication;

constructor TClientApplication.Create(AOWner: TComponent);

begin
  INherited;
  StopOnException:=True;
  myClient := TENetClass.Create(30000,False);
  myClient.MessageTimeout:=100;
  myClient.OnReceive:=@OnReceive;
  myClient.OnConnect:=@OnConnect;
  myClient.OnDisconnect:=@OnConnect;
end;

destructor TClientApplication.Destroy;
begin
  myClient.Free;
  Inherited;
end;

procedure TClientApplication.SendMessage(AMessage : String);
begin
  myClient.BroadcastMsg(1,PChar(AMessage),Length(AMessage)+1,[ENetPacketReliable]);
end;

procedure TClientApplication.ProcessLoop;
begin
  myClient.ProcessMsg;
end;

procedure TClientApplication.DoConnect;

Var
  H : String;
  P : Integer;
begin
  H:=GetOptionValue('H','host');
  if (H='') then
    H:='localhost';
  P:=StrToIntDef(GetOptionValue('p','port'),0);
  if P=0 then
    P:=30000;
  myClient.Connect(H,P);
end;

procedure TClientApplication.OnConnect(const Event: ENetEvent);
begin
  if Event.kind=ENET_EVENT_TYPE_CONNECT then
    Writeln('Connected')
  else
    Writeln('Disconnected');
end;

procedure TClientApplication.OnReceive(const Event: ENetEvent; var BroadcastMsg: Boolean;
  var BroadcastChannel: Byte);
var
  msg : string;
begin
  msg := StrPas(PChar(Event.packet^.data));
  Writeln('Received : "',Msg,'"');
end;

procedure TClientApplication.WriteHelp;

begin
  Writeln('Usage ',ExtractFileName(Self.ExeName),' [options]');
  Writeln('Where options is one or more of');
  Writeln('-h --help           This message');
  Writeln('-H --host=hostname  Hostname to connect to (default : localhost)');
  Writeln('-p --port=portno    Port to connect to (default 30000)');
  Writeln('-p --port=portno    Port to connect to (default 30000)');
  Writeln('-m --message=msg    Message to send to server (none means no message is sent)');
  Writeln('-c --messagecount=N Number of times the message should be sent');
  Writeln('-P --pingcout=N     Number of times the server should be pinged. (default 0)');
  Writeln('-c --messagecount=N Number of times the message should be sent (default 1)');
end;

procedure TClientApplication.DoRun;
Var
  I,PingCount,MessageCount : Integer;
  Msg : String;
begin
  if HasOption('h','help') then
    begin
    WriteHelp;
    Terminate;
    exit;
    end;
  DoConnect;
  PingCount:=StrToIntDef(GetOptionValue('P','pingcount'),0);
  if PingCount>0 then
    begin
    Writeln('Pinging server');
    For I:=1 to PingCount do
      begin
      myClient.Ping;
      ProcessLoop;
      end;
    end;
  Msg:=GetOptionValue('m','message');
  if (Msg<>'') then
    begin
    MessageCount:=StrToIntDef(GetOptionValue('c','messagecount'),1);
    Writeln('Sending message to server (count:',MessageCount,')');
    For I:=1 to MessageCount do
      begin
      SendMessage(Msg+' (iteration : '+IntToStr(i)+')');
      ProcessLoop;
      end;
    end;
  Terminate;
end;

begin
  Application:=TClientApplication.Create(Nil);
  try
    Application.Initialize;
    Application.Run;
  finally
    Application.Free;
  end;
end.

