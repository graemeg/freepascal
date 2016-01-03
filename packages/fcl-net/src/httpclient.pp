{

    HTTPClient: HTTP client component
    Copyright (C) 2000-2003 by Sebastian Guenther (sg@freepascal.org)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}


unit HTTPClient;

interface

{$mode objfpc}
{$H+}

uses Classes, HTTPBase, fpSock, fpAsync;

type

  TCustomHTTPClient = class(TCustomTCPClient)
  protected
    SendBuffer: TAsyncWriteStream;
    FOnPrepareSending: TNotifyEvent;
    FOnHeaderSent: TNotifyEvent;
    FOnStreamSent: TNotifyEvent;
    FOnPrepareReceiving: TNotifyEvent;
    FOnHeaderReceived: TNotifyEvent;
    FOnStreamReceived: TNotifyEvent;
    FOnDestroy: TNotifyEvent;
    RecvSize: Integer;  // How many bytes are still to be read. -1 if unknown.
    DataAvailableNotifyHandle: Pointer;
    ReceivedHTTPVersion: String;

    procedure HeaderToSendCompleted(Sender: TObject);
    procedure StreamToSendCompleted(Sender: TObject);
    procedure ReceivedHeaderCompleted(Sender: TObject);
    procedure ReceivedHeaderEOF(Sender: TObject);
    procedure DataAvailable(Sender: TObject);
    procedure ReceivedStreamCompleted(Sender: TObject);

    property OnPrepareSending: TNotifyEvent read FOnPrepareSending write FOnPrepareSending;
    property OnHeaderSent: TNotifyEvent read FOnHeaderSent write FOnHeaderSent;
    property OnStreamSent: TNotifyEvent read FOnStreamSent write FOnStreamSent;
    property OnPrepareReceiving: TNotifyEvent read FOnPrepareReceiving write FOnPrepareReceiving;
    property OnHeaderReceived: TNotifyEvent read FOnHeaderReceived write FOnHeaderReceived;
    property OnStreamReceived: TNotifyEvent read FOnStreamReceived write FOnStreamReceived;
    property OnDestroy: TNotifyEvent read FOnDestroy write FOnDestroy;

  public
    HeaderToSend: THttpHeader;
    StreamToSend: TStream;
    ReceivedHeader: THttpHeader;
    ReceivedStream: TStream;
    DoDestroy: Boolean;

    destructor Destroy; override;
    procedure Receive;
    procedure Send;
  end;

  THttpClient = class(TCustomHttpClient)
  public
    property OnPrepareSending;
    property OnHeaderSent;
    property OnStreamSent;
    property OnPrepareReceiving;
    property OnHeaderReceived;
    property OnStreamReceived;
    property OnDestroy;
  end;

  {TCustomHTTPClient = class
  protected
    FEventLoop: TEventLoop;
    FSocket: TInetSocket;
    SendBuffer: TAsyncWriteStream;
    FOnPrepareSending: TNotifyEvent;
    FOnHeaderSent: TNotifyEvent;
    FOnStreamSent: TNotifyEvent;
    FOnPrepareReceiving: TNotifyEvent;
    FOnHeaderReceived: TNotifyEvent;
    FOnStreamReceived: TNotifyEvent;
    FOnDestroy: TNotifyEvent;
    RecvSize: Integer;  // How many bytes are still to be read. -1 if unknown.
    DataAvailableNotifyHandle: Pointer;
    ReceivedHTTPVersion: String;

    procedure HeaderToSendCompleted(Sender: TObject);
    procedure StreamToSendCompleted(Sender: TObject);
    procedure ReceivedHeaderCompleted(Sender: TObject);
    procedure ReceivedHeaderEOF(Sender: TObject);
    procedure DataAvailable(Sender: TObject);
    procedure ReceivedStreamCompleted(Sender: TObject);

    property OnPrepareSending: TNotifyEvent read FOnPrepareSending write FOnPrepareSending;
    property OnHeaderSent: TNotifyEvent read FOnHeaderSent write FOnHeaderSent;
    property OnStreamSent: TNotifyEvent read FOnStreamSent write FOnStreamSent;
    property OnPrepareReceiving: TNotifyEvent read FOnPrepareReceiving write FOnPrepareReceiving;
    property OnHeaderReceived: TNotifyEvent read FOnHeaderReceived write FOnHeaderReceived;
    property OnStreamReceived: TNotifyEvent read FOnStreamReceived write FOnStreamReceived;
    property OnDestroy: TNotifyEvent read FOnDestroy write FOnDestroy;

  public
    HeaderToSend: THttpHeader;
    StreamToSend: TStream;
    ReceivedHeader: THttpHeader;
    ReceivedStream: TStream;
    DoDestroy: Boolean;

    constructor Create(AEventLoop: TEventLoop; ASocket: TInetSocket);
    destructor Destroy; override;
    procedure Receive;
    procedure Send;
  end;}


implementation

uses SysUtils;

procedure TCustomHttpClient.HeaderToSendCompleted(Sender: TObject);
begin
  // WriteLn('TCustomHttpClient.HeaderToSendCompleted');
  if Assigned(FOnHeaderSent) then
    FOnHeaderSent(Self);
  if Assigned(StreamToSend) then
  begin
    SendBuffer := TAsyncWriteStream.Create(EventLoop, Stream);
    SendBuffer.CopyFrom(StreamToSend, StreamToSend.Size);
    SendBuffer.OnBufferSent := @StreamToSendCompleted;
  end else
  begin
    StreamToSendCompleted(nil);
    if DoDestroy then
      Self.Free;
  end;
end;

procedure TCustomHttpClient.StreamToSendCompleted(Sender: TObject);
begin
  // WriteLn('TCustomHttpClient.StreamToSendCompleted');
  if Assigned(FOnStreamSent) then
    FOnStreamSent(Self);
  FreeAndNil(SendBuffer);
  if DoDestroy then
    Self.Free
  else
    Receive;
end;

procedure TCustomHttpClient.ReceivedHeaderCompleted(Sender: TObject);
var
  BytesInBuffer: Integer;
  NeedMoreData: Boolean;
begin
  // WriteLn('TCustomHttpClient.ReceivedHeaderCompleted');
  ReceivedHeader.DataReceived := False;
  ReceivedHTTPVersion := ReceivedHeader.HttpVersion;
  BytesInBuffer := ReceivedHeader.Reader.BytesInBuffer;
  //WriteLn('BytesInBuffer: ', BytesInBuffer, ', Content length: ', ReceivedHeader.ContentLength);
  if Assigned(FOnHeaderReceived) then
    FOnHeaderReceived(Self);

  RecvSize := ReceivedHeader.ContentLength;
  if Assigned(ReceivedStream) then
  begin
    if BytesInBuffer = 0 then
      NeedMoreData := True
    else
    begin
      ReceivedStream.Write(ReceivedHeader.Reader.Buffer^, BytesInBuffer);
      if RecvSize > 0 then
        Dec(RecvSize, BytesInBuffer);
      if BytesInBuffer = ReceivedHeader.ContentLength then
        NeedMoreData := False
      else
        NeedMoreData := (not ReceivedHeader.InheritsFrom(THttpRequestHeader)) or
          (THttpRequestHeader(ReceivedHeader).Command <> 'GET');
    end;
  end else
    NeedMoreData := False;

  if NeedMoreData then
    DataAvailableNotifyHandle :=
      EventLoop.SetDataAvailableNotify(Stream.Handle, @DataAvailable, Stream)
  else
    ReceivedStreamCompleted(nil);

  if DoDestroy then
    Self.Free;
end;

procedure TCustomHttpClient.ReceivedHeaderEOF(Sender: TObject);
begin
  Self.Free;
end;

procedure TCustomHttpClient.DataAvailable(Sender: TObject);
var
  FirstRun: Boolean;
  ReadNow, BytesRead: Integer;
  buf: array[0..1023] of Byte;
begin
  FirstRun := True;
  while True do
  begin
    if RecvSize >= 0 then
    begin
      ReadNow := RecvSize;
      if ReadNow > 1024 then
        ReadNow := 1024;
    end else
      ReadNow := 1024;
    BytesRead := Stream.Read(buf, ReadNow);
    // WriteLn('TCustomHttpClient.DataAvailable: Read ', BytesRead, ' bytes; RecvSize=', RecvSize);
    if BytesRead <= 0 then
    begin
      if FirstRun then
        ReceivedStreamCompleted(nil);
      break;
    end;
    FirstRun := False;
    ReceivedStream.Write(buf, BytesRead);
    if RecvSize > 0 then
      Dec(RecvSize, BytesRead);
    if RecvSize = 0 then
    begin
      ReceivedStreamCompleted(nil);
      break;
    end;
  end;
  if DoDestroy then
    Self.Free;
end;

procedure TCustomHttpClient.ReceivedStreamCompleted(Sender: TObject);
begin
  // WriteLn('TCustomHttpClient.ReceivedStreamCompleted');
  if Assigned(DataAvailableNotifyHandle) then
  begin
    EventLoop.ClearDataAvailableNotify(DataAvailableNotifyHandle);
    DataAvailableNotifyHandle := nil;
  end;
  if Assigned(FOnStreamReceived) then
    FOnStreamReceived(Self);
  if DoDestroy then
    Self.Free
  else
    Send;
end;

{constructor TCustomHttpClient.Create(AManager: TEventLoop; ASocket: TInetSocket);
begin
  inherited Create;
  EventLoop := AManager;
  Stream := ASocket;
end;}

destructor TCustomHttpClient.Destroy;
begin
  if Assigned(DataAvailableNotifyHandle) then
    EventLoop.ClearDataAvailableNotify(DataAvailableNotifyHandle);
  if Assigned(OnDestroy) then
    OnDestroy(Self);
  FreeAndNil(SendBuffer);
  inherited Destroy;
end;

procedure TCustomHttpClient.Receive;
begin
  // Start receiver
  ReceivedHttpVersion := '';
  if Assigned(OnPrepareReceiving) then
    OnPrepareReceiving(Self);
  if Assigned(ReceivedHeader) then
  begin
    ReceivedHeader.OnCompleted := @ReceivedHeaderCompleted;
    ReceivedHeader.OnEOF := @ReceivedHeaderEOF;
    ReceivedHeader.AsyncReceive(EventLoop, Stream);
  end;
end;

procedure TCustomHttpClient.Send;
begin
  // Start sender
  if Assigned(OnPrepareSending) then
    OnPrepareSending(Self);
  if Assigned(HeaderToSend) then
  begin
    if ReceivedHttpVersion <> '' then
    begin
      HeaderToSend.HttpVersion := ReceivedHttpVersion;
      ReceivedHttpVersion := '';
    end;
    HeaderToSend.OnCompleted := @HeaderToSendCompleted;
    HeaderToSend.AsyncSend(EventLoop, Stream);
  end;
end;


end.
