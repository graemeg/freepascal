{
    This file is part of the Free Component library.
    Copyright (c) 2005 by Michael Van Canneyt, member of
    the Free Pascal development team

    Unix implementation of one-way IPC between 2 processes

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

uses baseunix;

ResourceString
  SErrFailedToCreatePipe = 'Failed to create named pipe: %s';
  SErrFailedToRemovePipe = 'Failed to remove named pipe: %s';

{ ---------------------------------------------------------------------
    TPipeClientComm
  ---------------------------------------------------------------------}

Type
  TPipeClientComm = Class(TIPCClientComm)
  Private
    FFileName: String;
    FStream: TFileStream;
  Public
    Constructor Create(AOWner : TSimpleIPCClient); override;
    Procedure Connect; override;
    Procedure Disconnect; override;
    Procedure SendMessage(MsgType : TMessageType; AStream : TStream); override;
    Function  ServerRunning : Boolean; override;
    Property FileName : String Read FFileName;
    Property Stream : TFileStream Read FStream;
  end;


constructor TPipeClientComm.Create(AOWner: TSimpleIPCClient);

Var
  D : String;

begin
  inherited Create(AOWner);
  FFileName:=Owner.ServerID;
  If (Owner.ServerInstance<>'') then
    FFileName:=FFileName+'-'+Owner.ServerInstance;
  D:='/tmp/'; // Change to something better later
  FFileName:=D+FFileName;
end;


procedure TPipeClientComm.Connect;
begin
  If Not ServerRunning then
    Owner.DoError(SErrServerNotActive,[Owner.ServerID]);
  FStream:=TFileStream.Create(FFileName,fmOpenReadWrite);
end;

procedure TPipeClientComm.Disconnect;
begin
  FreeAndNil(FStream);
end;

procedure TPipeClientComm.SendMessage(MsgType : TMessagetype; AStream: TStream);

Var
  Hdr : TMsgHeader;
  P,L,Count : Integer;

begin
  Hdr.Version:=MsgVersion;
  Hdr.msgType:=MsgType;
  Hdr.MsgLen:=AStream.Size;
  FStream.WriteBuffer(hdr,SizeOf(hdr));
  FStream.CopyFrom(AStream,0);
end;

function TPipeClientComm.ServerRunning: Boolean;
begin
  Result:=FileExists(FFileName);
end;


{ ---------------------------------------------------------------------
    TPipeServerComm
  ---------------------------------------------------------------------}

Type
  TPipeServerComm = Class(TIPCServerComm)
  Private
    FFileName: String;
    FStream: TFileStream;
  Public
    Constructor Create(AOWner : TSimpleIPCServer); override;
    Procedure StartServer; override;
    Procedure StopServer; override;
    Function  PeekMessage(TimeOut : Integer) : Boolean; override;
    Procedure ReadMessage ; override;
    Function GetInstanceID : String;override;
    Property FileName : String Read FFileName;
    Property Stream : TFileStream Read FStream;
  end;

constructor TPipeServerComm.Create(AOWner: TSimpleIPCServer);

Var
  D : String;

begin
  inherited Create(AOWner);
  FFileName:=Owner.ServerID;
  If Not Owner.Global then
    FFileName:=FFileName+'-'+IntToStr(fpGetPID);
  D:='/tmp/'; // Change to something better later
  FFileName:=D+FFileName;
end;


procedure TPipeServerComm.StartServer;
begin
  If not FileExists(FFileName) then
    If (fpmkFifo(FFileName,438)<>0) then
      Owner.DoError(SErrFailedToCreatePipe,[FFileName]);
  FStream:=TFileStream.Create(FFileName,fmOpenReadWrite);
end;

procedure TPipeServerComm.StopServer;
begin
  FreeAndNil(FStream);
  if Not DeleteFile(FFileName) then
    Owner.DoError(SErrFailedtoRemovePipe,[FFileName]);
end;

function TPipeServerComm.PeekMessage(TimeOut: Integer): Boolean;

Var
  FDS : TFDSet;

begin
  fpfd_zero(FDS);
  fpfd_set(FStream.Handle,FDS);
  Result:=fpSelect(FStream.Handle+1,@FDS,Nil,Nil,TimeOut)>0;
end;

procedure TPipeServerComm.ReadMessage;

Var
  L,P,Count : Integer;
  Hdr : TMsgHeader;

begin
  FStream.ReadBuffer(Hdr,SizeOf(Hdr));
  Owner.FMsgType:=Hdr.MsgType;
  Count:=Hdr.MsgLen;
  if count > 0 then
    begin
    Owner.FMsgData.Seek(0,soFrombeginning);
    Owner.FMsgData.CopyFrom(FStream,Count);
    end
  else
    Owner.FMsgData.Size := 0;
end;

function TPipeServerComm.GetInstanceID: String;
begin
  Result:=IntToStr(fpGetPID);
end;

{ ---------------------------------------------------------------------
    Set TSimpleIPCClient / TSimpleIPCServer defaults.
  ---------------------------------------------------------------------}

Function TSimpleIPCServer.CommClass : TIPCServerCommClass;

begin
  if (DefaultIPCServerClass<>Nil) then
    Result:=DefaultIPCServerClass
  else
    Result:=TPipeServerComm;
end;

function TSimpleIPCClient.CommClass: TIPCClientCommClass;
begin
  if (DefaultIPCClientClass<>Nil) then
    Result:=DefaultIPCClientClass
  else
    Result:=TPipeClientComm;
end;
