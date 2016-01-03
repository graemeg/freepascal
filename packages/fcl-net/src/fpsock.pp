{

    Socket communication components
    Copyright (c) 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{$mode objfpc}
{$H+}

unit fpSock;

interface

uses Errors, SysUtils, Sockets, Classes, fpAsync, Resolve;

type

  ESocketError = class(Exception)
  end;

  TSocketComponent = class(TComponent)
  private
    FEventLoop: TEventLoop;
  public
    property EventLoop: TEventLoop read FEventLoop write FEventLoop;
  end;

  TSocketStream = class(THandleStream)
  private
    FOnDisconnect: TNotifyEvent;
    function GetLocalAddress: TSockAddr;
    function GetPeerAddress: TSockAddr;
  protected
    procedure Disconnected; virtual;
  public
    destructor Destroy; override;
    function Read(var Buffer; Count: LongInt): LongInt; override;
    function Write(const Buffer; Count: LongInt): LongInt; override;

    property LocalAddress: TSockAddr read GetLocalAddress;
    property PeerAddress: TSockAddr read GetPeerAddress;
    property OnDisconnect: TNotifyEvent read FOnDisconnect write FOnDisconnect;
  end;


  // Connection-based sockets

  TConnectionBasedSocket = class(TSocketComponent)
  protected
    FStream: TSocketStream;
    FActive: Boolean;
    procedure SetActive(Value: Boolean); virtual; abstract;
    property Active: Boolean read FActive write SetActive;
    property Stream: TSocketStream read FStream;
  public
    destructor Destroy; override;
  end;


  TConnectionState = (
    connDisconnected,
    connResolving,
    connConnecting,
    connConnected);

  TClientConnectionSocket = class;

  TConnectionStateChangeEvent = procedure(Sender: TClientConnectionSocket;
    OldState, NewState: TConnectionState) of object;

  TClientConnectionSocket = class(TConnectionBasedSocket)
  private
    FOnStateChange: TConnectionStateChangeEvent;
    FRetries: Integer;
    FRetryDelay: Integer;       // Delay between retries in ms
    RetryCounter: Integer;
    RetryTimerNotifyHandle: Pointer;
    CanWriteNotifyHandle: Pointer;
    procedure RetryTimerNotify(Sender: TObject);
    procedure SocketCanWrite(Sender: TObject);
  protected
    FConnectionState: TConnectionState;

    procedure CreateSocket; virtual; abstract;
    procedure DoResolve; virtual;
    procedure DoConnect; virtual;
    function GetPeerName: String; virtual; abstract;

    procedure SetActive(Value: Boolean); override;
    procedure SetConnectionState(NewState: TConnectionState);

    property ConnectionState: TConnectionState read FConnectionState;
    property Retries: Integer read FRetries write FRetries default 0;
    property RetryDelay: Integer read FRetryDelay write FRetryDelay default 500;
    property OnConnectionStateChange: TConnectionStateChangeEvent
      read FOnStateChange write FOnStateChange;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
  end;


  TQueryConnectEvent = procedure(Sender: TConnectionBasedSocket; Socket: Integer;
    var DoConnect: Boolean) of object;
  TConnectEvent = procedure(Sender: TConnectionBasedSocket;
    Stream: TSocketStream) of object;

  TSocketConnectionServer = class(TConnectionBasedSocket)
  private
    FOnQueryConnect: TQueryConnectEvent;
    FOnConnect: TConnectEvent;
  protected
    DataAvailableNotifyHandle: Pointer;
    procedure ListenerDataAvailable(Sender: TObject);
    function DoQueryConnect(ASocket: Integer): Boolean;
    procedure DoConnect(AStream: TSocketStream); virtual;
    property OnQueryConnect: TQueryConnectEvent read FOnQueryConnect
      write FOnQueryConnect;
    property OnConnect: TConnectEvent read FOnConnect write FOnConnect;
  end;


  // TCP/IP components

  TCustomTCPClient = class(TClientConnectionSocket)
  private
    FHost: String;
    FPort: Word;
    HostAddr: THostAddr;
    procedure SetHost(const Value: String);
    procedure SetPort(Value: Word);
  protected
    procedure CreateSocket; override;
    procedure DoResolve; override;
    procedure DoConnect; override;
    function GetPeerName: String; override;

    property Host: String read FHost write SetHost;
    property Port: Word read FPort write SetPort;
  public
    destructor Destroy; override;
  end;

  TTCPClient = class(TCustomTCPClient)
  public
    property ConnectionState;
    property Stream;
  published
    property Active;
    property Host;
    property Port;
    property Retries;
    property RetryDelay;
    property OnConnectionStateChange;
  end;

  TCustomTCPServer = class;

  TCustomTCPServer = class(TSocketConnectionServer)
  private
    FPort: Word;
    procedure SetActive(Value: Boolean); override;
  protected
    //!!!: Interface/bindings list?
    property Port: Word read FPort write FPort;
  public
    destructor Destroy; override;
  end;

  TTCPServer = class(TCustomTCPServer)
  public
    property Stream;
  published
    property Active;
    property Port;
    property OnQueryConnect;
    property OnConnect;
  end;


implementation

uses
  baseunix,Unix;

resourcestring
  SSocketNoEventLoopAssigned = 'No event loop assigned';
  SSocketCreationError = 'Could not create socket: %s';
  SHostNotFound = 'Host "%s" not found';
  SSocketConnectFailed = 'Could not connect to %s: %s';
  SSocketBindingError = 'Could not bind socket to port %d: %s';
  SSocketAcceptError = 'Connection accept failed: %s';
  SSocketIsActive = 'Cannot change parameters while active';

Const
  Sys_EAGAIN = ESYSEAGAIN;
  Sys_EINPROGRESS = ESYSEINPROGRESS;


// TSocketStream

destructor TSocketStream.Destroy;
begin
  CloseSocket(Handle);
  inherited Destroy;
end;

function TSocketStream.Read(var Buffer; Count: LongInt): LongInt;
begin
  Result := fprecv(Handle, @Buffer, Count, MSG_NOSIGNAL);
  if Result = -1 then
  begin
    Result := 0;
    if SocketError <> Sys_EAGAIN then
      Disconnected;
  end;
end;

function TSocketStream.Write(const Buffer; Count: LongInt): LongInt;
begin
  Result := FPsend(Handle, @Buffer, Count, MSG_NOSIGNAL);
  if Result = -1 then
  begin
    Result := 0;
    if SocketError <> Sys_EAGAIN then
      Disconnected;
  end;
end;

procedure TSocketStream.Disconnected;
begin
  if Assigned(OnDisconnect) then
    OnDisconnect(Self);
end;

function TSocketStream.GetLocalAddress: TSockAddr;
var
  len: LongInt;
begin
  len := SizeOf(TSockAddr);
  if fpGetSockName(Handle, @Result, @len) <> 0 then
    FillChar(Result, SizeOf(Result), 0);
end;

function TSocketStream.GetPeerAddress: TSockAddr;
var
  len: LongInt;
begin
  len := SizeOf(TSockAddr);
  if FpGetPeerName(Handle, @Result, @len) <> 0 then
    FillChar(Result, SizeOf(Result), 0);
end;


// TConnectionBasedSocket

destructor TConnectionBasedSocket.Destroy;
begin
  FreeAndNil(FStream);
  inherited Destroy;
end;


// TClientConnectionSocket

constructor TClientConnectionSocket.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FRetryDelay := 500;
end;

destructor TClientConnectionSocket.Destroy;
begin
  if Assigned(RetryTimerNotifyHandle) then
    EventLoop.RemoveTimerNotify(RetryTimerNotifyHandle);
  inherited Destroy;
end;

procedure TClientConnectionSocket.DoResolve;
begin
  // By default, no resolving is done, so continue directly with connecting
  DoConnect;
end;

procedure TClientConnectionSocket.DoConnect;
begin
  SetConnectionState(connConnecting);

  try
    if not Assigned(EventLoop) then
      raise ESocketError.Create(SSocketNoEventLoopAssigned);
    CanWriteNotifyHandle := EventLoop.SetCanWriteNotify(Stream.Handle,
      @SocketCanWrite, nil);
  except
    SetConnectionState(connDisconnected);
    raise;
  end;
end;

procedure TClientConnectionSocket.SetActive(Value: Boolean);
begin
  if Value <> Active then
  begin
    if Value then
    begin
      // Activate the connection
      FActive := True;
      RetryCounter := 0;

      CreateSocket;
      DoResolve;
    end else
    begin
      // Close the connection
      FActive := False;
      try
        FreeAndNil(FStream);
        if Assigned(CanWriteNotifyHandle) then
        begin
          EventLoop.ClearCanWriteNotify(CanWriteNotifyHandle);
          CanWriteNotifyHandle := nil;
        end;
        if Assigned(RetryTimerNotifyHandle) then
        begin
          EventLoop.RemoveTimerNotify(RetryTimerNotifyHandle);
          RetryTimerNotifyHandle := nil;
        end;
      finally
        SetConnectionState(connDisconnected);
      end;
    end;
  end;
end;

procedure TClientConnectionSocket.SetConnectionState(NewState:
  TConnectionState);
var
  OldState: TConnectionState;
begin
  if NewState <> ConnectionState then
  begin
    OldState := ConnectionState;
    FConnectionState := NewState;
    if Assigned(OnConnectionStateChange) then
      OnConnectionStateChange(Self, OldState, NewState);
  end;
end;


procedure TClientConnectionSocket.RetryTimerNotify(Sender: TObject);
begin
  RetryTimerNotifyHandle := nil;
  Active := True;
end;

procedure TClientConnectionSocket.SocketCanWrite(Sender: TObject);
var
  Error: Integer;
  ErrorLen, GetResult: LongInt;
begin
  if ConnectionState = connConnecting then
  begin
    EventLoop.ClearCanWriteNotify(CanWriteNotifyHandle);
    CanWriteNotifyHandle := nil;

    ErrorLen := SizeOf(Error);
    GetResult := Sockets.fpGetSockOpt(Stream.Handle, SOL_SOCKET, SO_ERROR,
      @Error, @ErrorLen);
    if GetResult <> 0 then
      raise ESocketError.CreateFmt(SSocketConnectFailed,
        [GetPeerName, StrError(GetResult)]);
    if Error <> 0 then
      if (RetryCounter >= Retries) and (Retries >= 0) then
        raise ESocketError.CreateFmt(SSocketConnectFailed,
          [GetPeerName, StrError(Error)])
      else begin
        Active := False;
        RetryTimerNotifyHandle := EventLoop.AddTimerNotify(RetryDelay, False,
          @RetryTimerNotify, Self);
        Inc(RetryCounter);
      end
    else
    begin
      RetryCounter := 0;
      SetConnectionState(connConnected);
    end;
  end;
end;


// TSocketConnectionServer

procedure TSocketConnectionServer.ListenerDataAvailable(Sender: TObject);
var
  ClientSocket: Integer;
  Addr: TInetSockAddr;
  AddrSize: Integer;
begin
  AddrSize := SizeOf(Addr);
  ClientSocket := FpAccept(Stream.Handle, @Addr, @AddrSize);
  if ClientSocket = -1 then
    raise ESocketError.CreateFmt(SSocketAcceptError, [StrError(SocketError)]);

  if DoQueryConnect(ClientSocket) then
    DoConnect(TSocketStream.Create(ClientSocket));
end;

function TSocketConnectionServer.DoQueryConnect(ASocket: Integer): Boolean;
begin
  Result := True;
  if Assigned(OnQueryConnect) then
    OnQueryConnect(Self, ASocket, Result);
end;

procedure TSocketConnectionServer.DoConnect(AStream: TSocketStream);
begin
  if Assigned(OnConnect) then
    OnConnect(Self, AStream);
end;


// TCustomTCPClient

type
  TClientSocketStream = class(TSocketStream)
  protected
    Client: TCustomTCPClient;
    procedure Disconnected; override;
  end;

procedure TClientSocketStream.Disconnected;
begin
  inherited Disconnected;
  Client.Active := False;
end;


destructor TCustomTCPClient.Destroy;
begin
  if Assigned(CanWriteNotifyHandle) then
  begin
    EventLoop.ClearCanWriteNotify(CanWriteNotifyHandle);
    // Set to nil to be sure that descendant classes don't do something stupid
    CanWriteNotifyHandle := nil;
  end;
  inherited Destroy;
end;

procedure TCustomTCPClient.SetHost(const Value: String);
begin
  if Value <> Host then
  begin
    if Active then
      raise ESocketError.Create(SSocketIsActive);
    FHost := Value;
  end;
end;

procedure TCustomTCPClient.SetPort(Value: Word);
begin
  if Value <> Port then
  begin
    if Active then
      raise ESocketError.Create(SSocketIsActive);
    FPort := Value;
  end;
end;

procedure TCustomTCPClient.DoResolve;
var
  HostResolver: THostResolver;
begin
  HostAddr := StrToNetAddr(Host);
  if HostAddr.s_bytes[4] = 0 then
  begin
    HostResolver := THostResolver.Create(nil);
    try
      SetConnectionState(connResolving);
      if not HostResolver.NameLookup(FHost) then
        raise ESocketError.CreateFmt(SHostNotFound, [Host]);
      HostAddr := HostResolver.HostAddress;
    finally
      HostResolver.Free;
    end;
  end;
  DoConnect;
end;

procedure TCustomTCPClient.CreateSocket;
var
  Socket: Integer;
begin

  Socket := Sockets.FPSocket(AF_INET, SOCK_STREAM, 0);
  if Socket = -1 then
    raise ESocketError.CreateFmt(SSocketCreationError,
      [StrError(SocketError)]);
  FStream := TClientSocketStream.Create(Socket);
  TClientSocketStream(FStream).Client := Self;
end;

procedure TCustomTCPClient.DoConnect;
var
  SockAddr: TInetSockAddr;
begin
  inherited DoConnect;
<<<<<<< HEAD
  SockAddr.sin_Family := AF_INET;
  SockAddr.sin_Port := ShortHostToNet(Port);
  SockAddr.sin_Addr.s_addr := Cardinal(HostAddr);
=======
  SockAddr.Family := AF_INET;
  SockAddr.Port := ShortHostToNet(Port);
  SockAddr.Addr := Cardinal(HostAddr);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if Sockets.FpConnect(Stream.Handle, @SockAddr, SizeOf(SockAddr))<>0 Then
    if (SocketError <> sys_EINPROGRESS) and (SocketError <> 0) then
      raise ESocketError.CreateFmt(SSocketConnectFailed,
        [GetPeerName, StrError(SocketError)]);
end;

function TCustomTCPClient.GetPeerName: String;
begin
  Result := Format('%s:%d', [Host, Port]);
end;


// TCustomTCPServer

destructor TCustomTCPServer.Destroy;
begin
  if Assigned(DataAvailableNotifyHandle) then
  begin
    EventLoop.ClearDataAvailableNotify(DataAvailableNotifyHandle);
    // Set to nil to be sure that descendant classes don't do something stupid
    DataAvailableNotifyHandle := nil;
  end;
  inherited Destroy;
end;

procedure TCustomTCPServer.SetActive(Value: Boolean);
var
  Socket, TrueValue: Integer;
  Addr: TInetSockAddr;
begin
  if Active <> Value then
  begin
    FActive := False;
    if Value then
    begin
      Socket := Sockets.fpSocket(AF_INET, SOCK_STREAM, 0);
      if Socket = -1 then
        raise ESocketError.CreateFmt(SSocketCreationError,
          [StrError(SocketError)]);
      TrueValue := 1;
      Sockets.fpSetSockOpt(Socket, SOL_SOCKET, SO_REUSEADDR,
        @TrueValue, SizeOf(TrueValue));
      FStream := TSocketStream.Create(Socket);
<<<<<<< HEAD
      Addr.sin_Family := AF_INET;
      Addr.sin_Port := ShortHostToNet(Port);
      Addr.sin_Addr.s_addr := 0;
=======
      Addr.Family := AF_INET;
      Addr.Port := ShortHostToNet(Port);
      Addr.Addr := 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      if  fpBind(Socket, @Addr, SizeOf(Addr))<>0 then
        raise ESocketError.CreateFmt(SSocketBindingError,
          [Port, StrError(SocketError)]);
      fpListen(Socket, 5);
      if not Assigned(EventLoop) then
        raise ESocketError.Create(SSocketNoEventLoopAssigned);
      DataAvailableNotifyHandle := EventLoop.SetDataAvailableNotify(Socket,
        @ListenerDataAvailable, nil);
      FActive := True;
    end else
    begin
      FreeAndNil(FStream);
      EventLoop.ClearDataAvailableNotify(DataAvailableNotifyHandle);
      DataAvailableNotifyHandle := nil;
    end;
  end;
end;

end.
