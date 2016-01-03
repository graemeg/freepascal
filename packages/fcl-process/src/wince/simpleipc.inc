{
    This file is part of the Free Component library.
    Copyright (c) 2005 by Michael Van Canneyt, member of
    the Free Pascal development team

    Windows implementation of one-way IPC between 2 processes

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

uses Windows,messages;

Const
  MsgWndClassName : pwidechar = 'FPCMsgWindowCls';

Resourcestring
  SErrFailedToRegisterWindowClass = 'Failed to register message window class';
  SErrFailedToCreateWindow = 'Failed to create message window %s';

var
  MsgWindowClass: TWndClass = (
    style: 0;
    lpfnWndProc: Nil;
    cbClsExtra: 0;
    cbWndExtra: 0;
    hInstance: 0;
    hIcon: 0;
    hCursor: 0;
    hbrBackground: 0;
    lpszMenuName: nil;
    lpszClassName: Nil);

{ ---------------------------------------------------------------------
    TWinMsgServerComm
  ---------------------------------------------------------------------}

Type
  TWinMsgServerComm = Class(TIPCServerComm)
  Private
    FHWND : HWND;
    FWindowName : Widestring;
    FDataPushed : Boolean;
    Function AllocateHWnd(const cwsWindowName : widestring) : HWND;
  Public
    Constructor Create(AOwner : TSimpleIPCServer); override;
    procedure ReadMsgData(var Msg: TMsg);
    Procedure StartServer; override;
    Procedure StopServer; override;
    Function  PeekMessage(TimeOut : Integer) : Boolean; override;
    Procedure ReadMessage ; override;
    Function GetInstanceID : String;override;
    Property WindowName : WideString Read FWindowName;
  end;


function MsgWndProc(HWindow: HWnd; Message, WParam, LParam: Longint): Longint;stdcall;

Var
  I   : TWinMsgServerComm;
  Msg : TMsg;

begin
  Result:=0;
  If (Message=WM_COPYDATA) then
    begin
    I:=TWinMsgServerComm(GetWindowLong(HWindow,GWL_USERDATA));
    If (I<>NIl) then
      begin
      Msg.Message:=Message;
      Msg.WParam:=WParam;
      Msg.LParam:=LParam;
      I.ReadMsgData(Msg);
      I.FDataPushed:=True;
      If Assigned(I.Owner.OnMessage) then
        I.Owner.ReadMessage;
      Result:=1;
      end
    end
  else
    Result:=DefWindowProc(HWindow,Message,WParam,LParam);
end;


function TWinMsgServerComm.AllocateHWnd(const cwsWindowName: Widestring): HWND;

var
  cls: LPWNDCLASS;
  isreg : Boolean;

begin
  Pointer(MsgWindowClass.lpfnWndProc):=@MsgWndProc;
  MsgWindowClass.hInstance := HInstance;
  MsgWindowClass.lpszClassName:=MsgWndClassName;
  isreg:=GetClassInfo(HInstance,MsgWndClassName,cls);
  if not isreg then
    if (Windows.RegisterClass(MsgWindowClass)=0) then
      Owner.DoError(SErrFailedToRegisterWindowClass,[]);
  Result:=CreateWindowEx(WS_EX_TOOLWINDOW, MsgWndClassName,
    PWidechar(cwsWindowName), WS_POPUP {!0}, 0, 0, 0, 0, 0, 0, HInstance, nil);
  if (Result=0) then
    Owner.DoError(SErrFailedToCreateWindow,[cwsWindowName]);
  SetWindowLong(Result,GWL_USERDATA,Longint(Self));
end;

constructor TWinMsgServerComm.Create(AOWner: TSimpleIPCServer);
begin
  inherited Create(AOWner);
  FWindowName:=Owner.ServerID;
  If not Owner.Global then
    FWindowName:=FWindowName+'_'+InstanceID;
end;

procedure TWinMsgServerComm.StartServer;

begin
  FHWND:=AllocateHWND(FWindowName);
end;

procedure TWinMsgServerComm.StopServer;
begin
  DestroyWindow(FHWND);
  FHWND:=0;
end;

function TWinMsgServerComm.PeekMessage(TimeOut: Integer): Boolean;

Var
  Msg : Tmsg;
  B : Boolean;
  R : DWORD;

begin
  Result:=FDataPushed;
  If Result then
    Exit;
  B:=Windows.PeekMessage(Msg, FHWND, 0, 0, PM_NOREMOVE);
  If not B then
    // No message yet. Wait for a message to arrive available within specified time.
    begin
    if (TimeOut=0) then
      TimeOut:=Integer(INFINITE);
    R:=MsgWaitForMultipleObjects(1,FHWND,False,TimeOut,QS_SENDMESSAGE);
    B:=(R<>WAIT_TIMEOUT);
    end;
  If B then
    Repeat
    B:=Windows.PeekMessage(Msg, FHWND, 0, 0, PM_NOREMOVE);
    if B then
      begin
      Result:=(Msg.Message=WM_COPYDATA);
      // Remove non WM_COPY messages from Queue
      if not Result then
        GetMessage(@Msg,FHWND,0,0);
      end;
    Until Result or (not B);
end;

procedure TWinMsgServerComm.ReadMsgData(var Msg: TMsg);

Var
  CDS : PCopyDataStruct;

begin
  CDS:=PCopyDataStruct(Msg.Lparam);
  Owner.FMsgData.Seek(0,soFrombeginning);
  Owner.FMsgData.WriteBuffer(CDS^.lpData^,CDS^.cbData);
end;

procedure TWinMsgServerComm.ReadMessage;

Var
  Msg : TMsg;

begin
  If FDataPushed then
    FDataPushed:=False
  else
    If Windows.PeekMessage(Msg, FHWND, 0, 0, PM_REMOVE) then
      if (Msg.Message=WM_COPYDATA) then
        ReadMsgData(Msg);
end;

function TWinMsgServerComm.GetInstanceID: String;
begin
  Result:=IntToStr(HInstance);
end;

{ ---------------------------------------------------------------------
    TWinMsgClientComm
  ---------------------------------------------------------------------}
  
Type
  TWinMsgClientComm = Class(TIPCClientComm)
  Private
    FWindowName: String;
    FHWND : HWnd;
  Public
    Constructor Create(AOWner : TSimpleIPCClient); override;
    Procedure Connect; override;
    Procedure Disconnect; override;
    Procedure SendMessage(MsgType : TMessageType; Stream : TStream); override;
    Function  ServerRunning : Boolean; override;
    Property WindowName : String Read FWindowName;
  end;


constructor TWinMsgClientComm.Create(AOWner: TSimpleIPCClient);
begin
  inherited Create(AOWner);
  FWindowName:=Owner.ServerID;
  If (Owner.ServerInstance<>'') then
    FWindowName:=FWindowName+'_'+Owner.ServerInstance;
end;

procedure TWinMsgClientComm.Connect;
begin
  FHWND:=FindWindow(MsgWndClassName,Pwidechar(FWindowName));
  If (FHWND=0) then
    Owner.DoError(SErrServerNotActive,[Owner.ServerID]);
end;

procedure TWinMsgClientComm.Disconnect;
begin
  FHWND:=0;
end;

procedure TWinMsgClientComm.SendMessage(MsgType: TMessageType; Stream: TStream
  );
Var
  CDS : TCopyDataStruct;
  Data,FMemstr : TMemorySTream;

begin
  If Stream is TMemoryStream then
    begin
    Data:=TMemoryStream(Stream);
    FMemStr:=Nil
    end
  else
    begin
    FMemStr:=TMemoryStream.Create;
    Data:=FMemstr;
    end;
  Try
    If Assigned(FMemStr) then
      begin
      FMemStr.CopyFrom(Stream,0);
      FMemStr.Seek(0,soFromBeginning);
      end;
    CDS.lpData:=Data.Memory;
    CDS.cbData:=Data.Size;
    Windows.SendMessage(FHWnd,WM_COPYDATA,0,Integer(@CDS));
  Finally
    FreeAndNil(FMemStr);
  end;
end;

function TWinMsgClientComm.ServerRunning: Boolean;
begin
  Result:=FindWindow(MsgWndClassName,PWidechar(FWindowName))<>0;
end;

{ ---------------------------------------------------------------------
    Set TSimpleIPCClient / TSimpleIPCServer defaults.
  ---------------------------------------------------------------------}


Function TSimpleIPCServer.CommClass : TIPCServerCommClass;

begin
  if (DefaultIPCServerClass<>Nil) then
    Result:=DefaultIPCServerClass
  else  
    Result:=TWinMsgServerComm;
end;

Function TSimpleIPCClient.CommClass : TIPCClientCommClass;

begin
  if (DefaultIPCClientClass<>Nil) then
    Result:=DefaultIPCClientClass
  else  
    Result:=TWinMsgClientComm;
end;

