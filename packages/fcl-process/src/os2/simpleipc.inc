{
    This file is part of the Free Component library.
    Copyright (c) 2007 by Tomas Hajny, member of
    the Free Pascal development team

    OS/2 implementation of one-way IPC between 2 processes

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

uses DosCalls, OS2Def;

ResourceString
  SErrFailedToCreatePipe = 'Failed to create named pipe: %s';
  SErrFailedToDisconnectPipe = 'Failed to disconnect named pipe: %s';

const
(* Constant used as key identifying a pipe connected to event semaphore. *)
(* 'FP' *)
  PipeKey = $4650;
  PipeBufSize = 256;

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


constructor TPipeClientComm.Create (AOWner: TSimpleIPCClient);
begin
  inherited Create (AOWner);
  FFileName:= '\PIPE\' + Owner.ServerID;
  If (Owner.ServerInstance <> '') then
    FFileName := FFileName + '.' + Owner.ServerInstance;
end;


procedure TPipeClientComm.Connect;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  try
    FStream := TFileStream.Create (FFileName, fmOpenWrite);
  finally
    Owner.DoError (SErrServerNotActive, [Owner.ServerID]);
  end;
=======
  If Not ServerRunning then
    Owner.DoError (SErrServerNotActive, [Owner.ServerID]);
  FStream := TFileStream.Create (FFileName, fmOpenWrite);
>>>>>>> graemeg/fixes_2_2
=======
  If Not ServerRunning then
    Owner.DoError (SErrServerNotActive, [Owner.ServerID]);
  FStream := TFileStream.Create (FFileName, fmOpenWrite);
>>>>>>> origin/fixes_2_2
end;


procedure TPipeClientComm.Disconnect;
begin
  FreeAndNil (FStream);
end;


procedure TPipeClientComm.SendMessage (MsgType: TMessageType; AStream: TStream);
var
  Hdr: TMsgHeader;
begin
  Hdr.Version := MsgVersion;
  Hdr.MsgType := MsgType;
  Hdr.MsgLen := AStream.Size;
  FStream.WriteBuffer (Hdr, SizeOf (Hdr));
  FStream.CopyFrom (AStream, 0);
end;


function TPipeClientComm.ServerRunning: boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
{$WARNING Fake TPipeClientComm.ServerRunning - no safe solution known}
  Result := true;
=======
  Result := FileExists (FFileName);
>>>>>>> graemeg/fixes_2_2
=======
  Result := FileExists (FFileName);
>>>>>>> origin/fixes_2_2
end;


{ ---------------------------------------------------------------------
    TPipeServerComm
  ---------------------------------------------------------------------}

type
  TPipeServerComm = class (TIPCServerComm)
  private
    FFileName: string;
    FStream: THandleStream;
    EventSem: THandle;
    SemName: string;
  public
    constructor Create (AOWner: TSimpleIPCServer); override;
    procedure StartServer; override;
    procedure StopServer; override;
    function  PeekMessage (TimeOut: integer): boolean; override;
    procedure ReadMessage; override;
    function GetInstanceID: string; override;
    property FileName: string read FFileName;
    property Stream: THandleStream read FStream;
  end;


constructor TPipeServerComm.Create (AOWner: TSimpleIPCServer);
begin
  inherited Create (AOWner);
  FFileName := '\PIPE\' + Owner.ServerID;
  SemName := '\SEM32\PIPE\' + Owner.ServerID;
  If not Owner.Global then
    FFileName := FFileName + '.' + IntToStr (GetProcessID);
end;


procedure TPipeServerComm.StartServer;
var
  H: THandle;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if not (Assigned (FStream)) then
=======
  if not FileExists (FFileName) then
>>>>>>> graemeg/fixes_2_2
=======
  if not FileExists (FFileName) then
>>>>>>> origin/fixes_2_2
    if (DosCreateNPipe (PChar (FFileName), H, np_Access_Inbound,
        np_ReadMode_Message or np_WriteMode_Message or 1, PipeBufSize,
                                               PipeBufSize, 0) <> 0) or
           (DosCreateEventSem (PChar (SemName), EventSem, 0, 0) <> 0) or
                          (DosSetNPipeSem (H, EventSem, PipeKey) <> 0) or
                                            (DosConnectNPipe (H) <> 0) then
                           Owner.DoError (SErrFailedToCreatePipe, [FFileName]);
  FStream := THandleStream.Create (H);
end;


procedure TPipeServerComm.StopServer;
begin
  if (DosDisconnectNPipe (FStream.Handle) <> 0) or
                 (DosCloseEventSem (EventSem) <> 0) then
                       Owner.DoError (SErrFailedToDisconnectPipe, [FFileName]);
  FreeAndNil (FStream);
end;


function TPipeServerComm.PeekMessage (TimeOut: integer): boolean;
var
  PipeSemState: TPipeSemState;
begin
  Result := (DosQueryNPipeSemState (EventSem, PipeSemState,
             SizeOf (PipeSemState)) = 0) and (PipeSemState.Status = 1) and
                (PipeSemState.Avail <> 0) and (PipeSemState.Key = PipeKey);
  if not (Result) then
    Result := (DosWaitEventSem (EventSem, TimeOut) = 0) and
               (DosQueryNPipeSemState (EventSem, PipeSemState,
                SizeOf (PipeSemState)) = 0) and (PipeSemState.Status = 1) and
                    (PipeSemState.Avail <> 0) and (PipeSemState.Key = PipeKey);
end;


procedure TPipeServerComm.ReadMessage;
var
  Hdr: TMsgHeader;
begin
  FStream.ReadBuffer (Hdr, SizeOf (Hdr));
  Owner.FMsgType := Hdr.MsgType;
  if Hdr.MsgLen > 0 then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      Owner.FMsgData.Size:=0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Owner.FMsgData.Seek (0, soFromBeginning);
      Owner.FMsgData.CopyFrom (FStream, Hdr.MsgLen);
    end
  else
    Owner.FMsgData.Size := 0;
end;

function TPipeServerComm.GetInstanceID: string;
begin
  Result := IntToStr (GetProcessID);
end;

{ ---------------------------------------------------------------------
    Set TSimpleIPCClient / TSimpleIPCServer defaults.
  ---------------------------------------------------------------------}

function TSimpleIPCServer.CommClass: TIPCServerCommClass;
begin
  if (DefaultIPCServerClass <> nil) then
    Result := DefaultIPCServerClass
  else
    Result := TPipeServerComm;
end;

function TSimpleIPCClient.CommClass: TIPCClientCommClass;
begin
  if (DefaultIPCClientClass <> nil) then
    Result := DefaultIPCClientClass
  else
    Result := TPipeClientComm;
end;
