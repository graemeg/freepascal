{
    This file is part of the Free Component library.
    Copyright (c) 2005 by Michael Van Canneyt, member of
    the Free Pascal development team

    Debugserver client interface, based on SimpleIPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit dbugintf;

interface

Type
  TDebugLevel = (dlInformation,dlWarning,dlError);

procedure SendBoolean(const Identifier: string; const Value: Boolean);
procedure SendDateTime(const Identifier: string; const Value: TDateTime);
procedure SendInteger(const Identifier: string; const Value: Integer; HexNotation: Boolean = False);
procedure SendPointer(const Identifier: string; const Value: Pointer);
procedure SendDebugEx(const Msg: string; MType: TDebugLevel);
procedure SendDebug(const Msg: string);
procedure SendMethodEnter(const MethodName: string);
procedure SendMethodExit(const MethodName: string);
procedure SendSeparator;
procedure SendDebugFmt(const Msg: string; const Args: array of const);
procedure SendDebugFmtEx(const Msg: string; const Args: array of const; MType: TDebugLevel);

procedure SetDebuggingEnabled(const AValue : boolean);
function GetDebuggingEnabled : Boolean;

{ low-level routines }

Function  StartDebugServer : integer;
Function InitDebugClient : Boolean;

Const
  SendError       : String = '';

ResourceString
  SProcessID = 'Process %s';
  SEntering = '> Entering ';
  SExiting  = '< Exiting ';
  SSeparator = '>-=-=-=-=-=-=-=-=-=-=-=-=-=-=-<';
<<<<<<< HEAD
<<<<<<< HEAD
  SServerStartFailed = 'Failed to start debugserver. (%s)';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

Uses 
  SysUtils, classes,dbugmsg, process, simpleipc;

Const
  DmtInformation = lctInformation;
  DmtWarning     = lctWarning;
  DmtError       = lctError;
  ErrorLevel     : Array[TDebugLevel] of integer
                 = (dmtInformation,dmtWarning,dmtError);
  IndentChars    = 2;
  
var
  DebugClient : TSimpleIPCClient = nil;
  MsgBuffer : TMemoryStream = Nil;
  ServerID : Integer;
  DebugDisabled : Boolean = False;
  Indent : Integer = 0;
  
Procedure WriteMessage(Const Msg : TDebugMessage);

begin
  MsgBuffer.Seek(0,soFrombeginning);
  WriteDebugMessageToStream(MsgBuffer,Msg);
  DebugClient.SendMessage(mtUnknown,MsgBuffer);
end;


procedure SendDebugMessage(Var Msg : TDebugMessage);

begin
  if DebugDisabled then exit;
  try
    If (DebugClient=Nil) then
      if InitDebugClient = false then exit;
    if (Indent>0) then
      Msg.Msg:=StringOfChar(' ',Indent)+Msg.Msg;
    WriteMessage(Msg);
  except
    On E : Exception do
      SendError:=E.Message;
  end;
end;

procedure SendBoolean(const Identifier: string; const Value: Boolean);

Const
  Booleans : Array[Boolean] of string = ('False','True');

begin
  SendDebugFmt('%s = %s',[Identifier,Booleans[value]]);
end;

procedure SendDateTime(const Identifier: string; const Value: TDateTime);

begin
  SendDebugFmt('%s = %s',[Identifier,DateTimeToStr(Value)]);
end;

procedure SendInteger(const Identifier: string; const Value: Integer; HexNotation: Boolean = False);

Const
  Msgs : Array[Boolean] of string = ('%s = %d','%s = %x');

begin
  SendDebugFmt(Msgs[HexNotation],[Identifier,Value]);
end;

procedure SendPointer(const Identifier: string; const Value: Pointer);

begin
  SendDebugFmt('%s = %p',[Identifier,Value]);
end;

procedure SendDebugEx(const Msg: string; MType: TDebugLevel);

Var
  Mesg : TDebugMessage;

begin
  Mesg.MsgTimeStamp:=Now;
  Mesg.MsgType:=ErrorLevel[MTYpe];
  Mesg.Msg:=Msg;
  SendDebugMessage(Mesg);
end;

procedure SendDebug(const Msg: string);

Var
  Mesg : TDebugMessage;
begin
  Mesg.MsgTimeStamp:=Now;
  Mesg.MsgType:=dmtInformation;
  Mesg.Msg:=Msg;
  SendDebugMessage(Mesg);
end;

procedure SendMethodEnter(const MethodName: string);

begin
  SendDebug(SEntering+MethodName);
  inc(Indent,IndentChars);
end;

procedure SendMethodExit(const MethodName: string);

begin
  Dec(Indent,IndentChars);
  If (Indent<0) then
    Indent:=0;
  SendDebug(SExiting+MethodName);
end;

procedure SendSeparator;

begin
  SendDebug(SSeparator);
end;

procedure SendDebugFmt(const Msg: string; const Args: array of const);

Var
  Mesg : TDebugMessage;

begin
  Mesg.MsgTimeStamp:=Now;
  Mesg.MsgType:=dmtInformation;
  Mesg.Msg:=Format(Msg,Args);
  SendDebugMessage(Mesg);
end;

procedure SendDebugFmtEx(const Msg: string; const Args: array of const; MType: TDebugLevel);

Var
  Mesg : TDebugMessage;

begin
  Mesg.MsgTimeStamp:=Now;
  Mesg.MsgType:=ErrorLevel[mType];
  Mesg.Msg:=Format(Msg,Args);
  SendDebugMessage(Mesg);
end;

procedure SetDebuggingEnabled(const AValue: boolean);
begin
  DebugDisabled := not AValue;
end;

function GetDebuggingEnabled: Boolean;
begin
  Result := not DebugDisabled;
end;

function StartDebugServer : Integer;

begin
  With TProcess.Create(Nil) do
    begin
    Try
<<<<<<< HEAD
<<<<<<< HEAD
      CommandLine:='dbugsrv';
      Execute;
      Result:=ProcessID;
    Except On E: Exception do
      begin
      SendError := Format(SServerStartFailed,[E.Message]);
      Result := 0;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
      CommandLine:='debugserver';
      Execute;
      Result:=ProcessID;
    Except
      Result := 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
    Free;
    end;
end;

procedure FreeDebugClient;

Var
  msg : TDebugMessage;

begin
  try
    If (DebugClient<>Nil) and
       (DebugClient.ServerRunning) then
      begin
      Msg.MsgType:=lctStop;
      Msg.MsgTimeStamp:=Now;
      Msg.Msg:=Format(SProcessID,[ApplicationName]);
      WriteMessage(Msg);
      end;
    if assigned(MsgBuffer) then FreeAndNil(MsgBuffer);
    if assigned(DebugClient) then FreeAndNil(DebugClient);
  except
  end;
end;

Function InitDebugClient : Boolean;

Var
  msg : TDebugMessage;
  I : Integer;
  
begin
  Result := False;
  DebugClient:=TSimpleIPCClient.Create(Nil);
  DebugClient.ServerID:=DebugServerID;
  If not DebugClient.ServerRunning then
    begin
    ServerID:=StartDebugServer;
    if ServerID = 0 then
      begin
      DebugDisabled := True;
<<<<<<< HEAD
<<<<<<< HEAD
      FreeAndNil(DebugClient);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Exit;
      end
    else
      DebugDisabled := False;
    I:=0;
    While (I<10) and not DebugClient.ServerRunning do
      begin
      Inc(I);
      Sleep(100);
      end;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  try
    DebugClient.Connect;
  except
    FreeAndNil(DebugClient);
    DebugDisabled:=True;
    Raise;
  end;
=======
  DebugClient.Connect;
>>>>>>> graemeg/fixes_2_2
=======
  DebugClient.Connect;
>>>>>>> origin/fixes_2_2
  MsgBuffer:=TMemoryStream.Create;
  Msg.MsgType:=lctIdentify;
  Msg.MsgTimeStamp:=Now;
  Msg.Msg:=Format(SProcessID,[ApplicationName]);
  WriteMessage(Msg);
  Result := True;
end;

Finalization
  FreeDebugClient;
end.
