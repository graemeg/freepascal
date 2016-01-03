{
    This file is part of the Free Component library.
    Copyright (c) 2005 by Michael Van Canneyt, member of
    the Free Pascal development team

    Unit implementing one-way IPC between 2 processes

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit simpleipc;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

Const
  MsgVersion = 1;
  
  //Message types
  mtUnknown = 0;
  mtString = 1;
  
Type

  TMessageType = LongInt;
  TMsgHeader = Packed record
    Version : Byte;
    MsgType : TMessageType;
    MsgLen  : Integer;
  end;

  TSimpleIPCServer = class;
  TSimpleIPCClient = class;

  { TIPCServerComm }
  
  TIPCServerComm = Class(TObject)
  Private
    FOwner  : TSimpleIPCServer;
  Protected  
    Function  GetInstanceID : String; virtual; abstract;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure DoError(Msg : String; Args : Array of const);
    Procedure SetMsgType(AMsgType: TMessageType); 
    Function MsgData : TStream;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    Constructor Create(AOwner : TSimpleIPCServer); virtual;
    Property Owner : TSimpleIPCServer read FOwner;
    Procedure StartServer; virtual; Abstract;
    Procedure StopServer;virtual; Abstract;
    Function  PeekMessage(TimeOut : Integer) : Boolean;virtual; Abstract;
    Procedure ReadMessage ;virtual; Abstract;
    Property InstanceID : String read GetInstanceID;
  end;
  TIPCServerCommClass = Class of TIPCServerComm;

  { TSimpleIPC }
  TSimpleIPC = Class(TComponent)
  Private
    procedure SetActive(const AValue: Boolean);
    procedure SetServerID(const AValue: String);
  Protected
    FBusy: Boolean;
    FActive : Boolean;
    FServerID : String;
    Procedure DoError(Msg : String; Args : Array of const);
    Procedure CheckInactive;
    Procedure CheckActive;
    Procedure Activate; virtual; abstract;
    Procedure Deactivate; virtual; abstract;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Loaded; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property Busy : Boolean Read FBusy;
  Published
    Property Active : Boolean Read FActive Write SetActive;
    Property ServerID : String Read FServerID Write SetServerID;
  end;

  { TSimpleIPCServer }

  TSimpleIPCServer = Class(TSimpleIPC)
  private
    FGlobal: Boolean;
    FOnMessage: TNotifyEvent;
    FMsgType: TMessageType;
    FMsgData : TStream;
    function GetInstanceID: String;
    function GetStringMessage: String;
    procedure SetGlobal(const AValue: Boolean);
  Protected
    FIPCComm: TIPCServerComm;
    Function CommClass : TIPCServerCommClass; virtual;
    Procedure Activate; override;
    Procedure Deactivate; override;
    Procedure ReadMessage;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure StartServer;
    Procedure StopServer;
    Function PeekMessage(TimeOut : Integer; DoReadMessage : Boolean): Boolean;
    Property  StringMessage : String Read GetStringMessage;
    Procedure GetMessageData(Stream : TStream);
    Property  MsgType: TMessageType Read FMsgType;
    Property  MsgData : TStream Read FMsgData;
    Property  InstanceID : String Read GetInstanceID;
  Published
    Property Global : Boolean Read FGlobal Write SetGlobal;
    Property OnMessage : TNotifyEvent Read FOnMessage Write FOnMessage;
  end;


  { TIPCClientComm}
  TIPCClientComm = Class(TObject)
  private
    FOwner: TSimpleIPCClient;
<<<<<<< HEAD
<<<<<<< HEAD
  protected
   Procedure DoError(Msg : String; Args : Array of const);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    Constructor Create(AOwner : TSimpleIPCClient); virtual;
    Property  Owner : TSimpleIPCClient read FOwner;
    Procedure Connect; virtual; abstract;
    Procedure Disconnect; virtual; abstract;
    Function  ServerRunning : Boolean; virtual; abstract;
    Procedure SendMessage(MsgType : TMessageType; Stream : TStream);virtual;Abstract;
  end;
  TIPCClientCommClass = Class of TIPCClientComm;
  
  { TSimpleIPCClient }
  TSimpleIPCClient = Class(TSimpleIPC)
  Private
    FServerInstance: String;
    procedure SetServerInstance(const AValue: String);
  Protected
    FIPCComm : TIPCClientComm;
    Procedure Activate; override;
    Procedure Deactivate; override;
    Function CommClass : TIPCClientCommClass; virtual;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure Connect;
    Procedure Disconnect;
    Function  ServerRunning : Boolean;
    Procedure SendMessage(MsgType : TMessageType; Stream: TStream);
    Procedure SendStringMessage(const Msg : String);
    Procedure SendStringMessage(MsgType : TMessageType; const Msg : String);
    Procedure SendStringMessageFmt(const Msg : String; Args : Array of const);
    Procedure SendStringMessageFmt(MsgType : TMessageType; const Msg : String; Args : Array of const);
    Property  ServerInstance : String Read FServerInstance Write SetServerInstance;
  end;


  EIPCError = Class(Exception);

Var
  DefaultIPCServerClass : TIPCServerCommClass = Nil;
  DefaultIPCClientClass : TIPCClientCommClass = Nil;

resourcestring
  SErrServerNotActive = 'Server with ID %s is not active.';
  SErrActive = 'This operation is illegal when the server is active.';
  SErrInActive = 'This operation is illegal when the server is inactive.';


implementation

{ ---------------------------------------------------------------------
  Include platform specific implementation. 
  Should implement the CommClass method of both server and client component, 
  as well as the communication class itself.
  
  This comes first, to allow the uses clause to be set.
<<<<<<< HEAD
<<<<<<< HEAD
  If the include file defines OSNEEDIPCINITDONE then the unit will
  call IPCInit and IPCDone in the initialization/finalization code.
  
  --------------------------------------------------------------------- }
{$UNDEF OSNEEDIPCINITDONE}
=======
  --------------------------------------------------------------------- }
>>>>>>> graemeg/fixes_2_2
=======
  --------------------------------------------------------------------- }
>>>>>>> origin/fixes_2_2

{$i simpleipc.inc}

{ ---------------------------------------------------------------------
    TIPCServerComm
  ---------------------------------------------------------------------}

constructor TIPCServerComm.Create(AOwner: TSimpleIPCServer);
begin
  FOwner:=AOWner;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TIPCServerComm.DoError(Msg : String; Args : Array of const);

begin
  FOwner.DoError(Msg,Args);
end;  

Function TIPCServerComm.MsgData : TStream;

begin
  Result:=FOwner.FMsgData;
end;

Procedure TIPCServerComm.SetMsgType(AMsgType: TMessageType); 

begin
  Fowner.FMsgType:=AMsgType;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
    TIPCClientComm
  ---------------------------------------------------------------------}
  
constructor TIPCClientComm.Create(AOwner: TSimpleIPCClient);
begin
  FOwner:=AOwner;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TIPCClientComm.DoError(Msg : String; Args : Array of const);

begin
  FOwner.DoError(Msg,Args);
end;  

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
    TSimpleIPC
  ---------------------------------------------------------------------}

procedure TSimpleIPC.DoError(Msg: String; Args: array of const);
begin
  Raise EIPCError.Create(Name+': '+Format(Msg,Args));
end;

procedure TSimpleIPC.CheckInactive;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if not (csLoading in ComponentState) then
    If Active then
      DoError(SErrActive,[]);
=======
  If Active then
    DoError(SErrActive,[]);
>>>>>>> graemeg/fixes_2_2
=======
  If Active then
    DoError(SErrActive,[]);
>>>>>>> origin/fixes_2_2
end;

procedure TSimpleIPC.CheckActive;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if not (csLoading in ComponentState) then
    If Not Active then
      DoError(SErrInActive,[]);
=======
  If Not Active then
    DoError(SErrInActive,[]);
>>>>>>> graemeg/fixes_2_2
=======
  If Not Active then
    DoError(SErrInActive,[]);
>>>>>>> origin/fixes_2_2
end;

procedure TSimpleIPC.SetActive(const AValue: Boolean);
begin
  if (FActive<>AValue) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    if (csLoading in ComponentState) then
      FActive:=AValue
    else  
      If AValue then
        Activate
      else
        Deactivate;
=======
=======
>>>>>>> origin/fixes_2_2
    If AValue then
      Activate
    else
      Deactivate;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;

procedure TSimpleIPC.SetServerID(const AValue: String);
begin
  if (FServerID<>AValue) then
    begin
    CheckInactive;
    FServerID:=AValue
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TSimpleIPC.Loaded; 

Var
  B : Boolean;

begin
  B:=FActive;
  if B then
    begin
    Factive:=False;
    Activate;
    end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
    TSimpleIPCServer
  ---------------------------------------------------------------------}

constructor TSimpleIPCServer.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FGlobal:=False;
  FActive:=False;
  FBusy:=False;
  FMsgData:=TStringStream.Create('');
end;

destructor TSimpleIPCServer.Destroy;
begin
  Active:=False;
  FreeAndNil(FMsgData);
  inherited Destroy;
end;

procedure TSimpleIPCServer.SetGlobal(const AValue: Boolean);
begin
  if (FGlobal<>AValue) then
    begin
    CheckInactive;
    FGlobal:=AValue;
    end;
end;

function TSimpleIPCServer.GetInstanceID: String;
begin
  Result:=FIPCComm.InstanceID;
end;


function TSimpleIPCServer.GetStringMessage: String;
begin
  Result:=TStringStream(FMsgData).DataString;
end;


procedure TSimpleIPCServer.StartServer;
begin
  if Not Assigned(FIPCComm) then
    begin
    If (FServerID='') then
      FServerID:=ApplicationName;
    FIPCComm:=CommClass.Create(Self);
    FIPCComm.StartServer;
    end;
  FActive:=True;
end;

procedure TSimpleIPCServer.StopServer;
begin
  If Assigned(FIPCComm) then
    begin
    FIPCComm.StopServer;
    FreeAndNil(FIPCComm);
    end;
  FActive:=False;
end;

function TSimpleIPCServer.PeekMessage(TimeOut: Integer; DoReadMessage: Boolean
  ): Boolean;
begin
  CheckActive;
  FBusy:=True;
  Try
    Result:=FIPCComm.PeekMessage(Timeout);
  Finally
    FBusy:=False;
  end;
  If Result then
    If DoReadMessage then
      Readmessage;
end;

procedure TSimpleIPCServer.ReadMessage;
begin
  CheckActive;
  FBusy:=True;
  Try
    FIPCComm.ReadMessage;
    If Assigned(FOnMessage) then
      FOnMessage(Self);
  Finally
    FBusy:=False;
  end;
end;

procedure TSimpleIPCServer.GetMessageData(Stream: TStream);
begin
  Stream.CopyFrom(FMsgData,0);
end;

procedure TSimpleIPCServer.Activate;
begin
  StartServer;
end;

procedure TSimpleIPCServer.Deactivate;
begin
  StopServer;
end;

<<<<<<< HEAD
<<<<<<< HEAD



=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
    TSimpleIPCClient
  ---------------------------------------------------------------------}

procedure TSimpleIPCClient.SetServerInstance(const AValue: String);
begin
  CheckInactive;
  FServerInstance:=AVAlue;
end;

procedure TSimpleIPCClient.Activate;
begin
  Connect;
end;

procedure TSimpleIPCClient.Deactivate;
begin
  DisConnect;
end;
constructor TSimpleIPCClient.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
end;

destructor TSimpleIPCClient.destroy;
begin
  Active:=False;
  Inherited;
end;

procedure TSimpleIPCClient.Connect;
begin
  If Not assigned(FIPCComm) then
    begin
    FIPCComm:=CommClass.Create(Self);
    Try
      FIPCComm.Connect;
    Except
      FreeAndNil(FIPCComm);
      Raise;
    end;  
    FActive:=True;
    end;
end;

procedure TSimpleIPCClient.Disconnect;
begin
  If Assigned(FIPCComm) then
    Try
      FIPCComm.DisConnect;
    Finally
      FActive:=False;
      FreeAndNil(FIPCComm);
    end;  
end;

function TSimpleIPCClient.ServerRunning: Boolean;

begin
  If Assigned(FIPCComm) then
    Result:=FIPCComm.ServerRunning
  else
    With CommClass.Create(Self) do
      Try
        Result:=ServerRunning;
      finally
        Free;
      end;
end;

procedure TSimpleIPCClient.SendMessage(MsgType : TMessageType; Stream: TStream);

begin
  CheckActive;
  FBusy:=True;
  Try
    FIPCComm.SendMessage(MsgType,Stream);
  Finally
    FBusy:=False;
  end;
end;

procedure TSimpleIPCClient.SendStringMessage(const Msg: String);
begin
  SendStringMessage(mtString,Msg);
end;

procedure TSimpleIPCClient.SendStringMessage(MsgType: TMessageType; const Msg: String
  );
Var
  S : TStringStream;
begin
  S:=TStringStream.Create(Msg);
  try
    SendMessage(MsgType,S);
  finally
    S.free;
  end;
end;

procedure TSimpleIPCClient.SendStringMessageFmt(const Msg: String;
  Args: array of const);
begin
  SendStringMessageFmt(mtString,Msg,Args);
end;

procedure TSimpleIPCClient.SendStringMessageFmt(MsgType: TMessageType;
  const Msg: String; Args: array of const);
begin
  SendStringMessage(MsgType, Format(Msg,Args));
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF OSNEEDIPCINITDONE}
initialization
  IPCInit;
finalization
  IPCDone;
{$ENDIF}  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

