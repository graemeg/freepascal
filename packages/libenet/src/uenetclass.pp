{$MODE OBJFPC}
{$LONGSTRINGS ON}

unit UENetClass;

{
  ENet UDP Class for Free Pascal

  Copyright (c) 2013 Do-wan Kim
  2016-08-24: revised by Dmitry D. Chernov

  Permission is hereby granted, free of charge, to any person obtaining a
  copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.

  TODO:
  - Add SendMsgEventPeer
}

interface

uses
  SysUtils, Classes, ENet;

type
  TENetEventType = ( ENetEventNone, ENetEventConnect, ENetEventDisconnect,
    ENetEventReceive );

  TENetPacketFlag = ( ENetPacketReliable, ENetPacketUnsequenced,
    ENetPacketNoAllocate, ENetPacketUnReliableFragment );
  TENetPacketFlags = set of TENetPacketFlag;

  TENetEventProc = procedure( const Event: ENetEvent ) of object;
  TENetEventRecv = procedure( const Event: ENetEvent; var BroadcastMsg: Boolean;
    var BroadcastChannel: Byte ) of object;

  { TENetClass }

  TENetClass = class
    private
      FInit : Boolean;

      FHostname : string;
      FAddress : ENetAddress;
      FIsServer : Boolean;

      FMaxPeer : Cardinal;
      FMaxChannels : Byte;
      FBandwidthIncoming : Cardinal;
      FBandwidthOutgoing : Cardinal;

      FHost : pENetHost;
      FEvent : ENetEvent;
      FPeer : pENetPeer;

      FClientData : Cardinal;
      FConnectTimeout : Cardinal;
      FMessageTimeout : Cardinal;

      FEventNone : TENetEventProc;
      FEventConnect : TENetEventProc;
      FEventDisconnect : TENetEventProc;
      FEventReceive : TENetEventRecv;

    public
      constructor Create( Port: Word; bServer: Boolean );
      destructor Destroy(); override;

      function InitHost(): Boolean;
      procedure DeinitHost();
      function Connect( const Host: string; Port: Word ): Boolean;
      function Disconnect( bNow: Boolean ): Boolean;
      function SendMsg( Channel: Byte; Data: Pointer; Length: Integer;
        flag: TENetPacketFlags; WaitResponse: Boolean = False ): Boolean;
      function SendMsgEventPeer( Data: Pointer; Length: Integer;
        flag: TENetPacketFlags; WaitResponse: Boolean = False ): Boolean;
      procedure FlushMsg();
      function BroadcastMsg( Channel: Byte; Data: Pointer; Length: Integer;
        flag: TENetPacketFlags; WaitResponse: Boolean = False ): Boolean;
      function ProcessMsg(): Boolean; virtual;
      procedure Ping();


      property Port: Word read FAddress.port write FAddress.port;
      property MaxClient: Cardinal read FMaxPeer write FMaxPeer;
      property MaxChannels: Byte read FMaxChannels write FMaxChannels;
      property BandwidthIncoming: Cardinal read FBandwidthIncoming write FBandwidthIncoming;
      property BandwidthOutgoing: Cardinal read FBandwidthOutgoing write FBandwidthOutgoing;
      property ClientData: Cardinal read FClientData;
      property ConnectTimeout: Cardinal read FConnectTimeout write FConnectTimeout;
      property MessageTimeout: Cardinal read FMessageTimeout write FMessageTimeout;
      property OnNone: TENetEventProc read FEventNone write FEventNone;
      property OnConnect: TENetEventProc read FEventConnect write FEventConnect;
      property OnDisconnect: TENetEventProc read FEventDisconnect write FEventDisconnect;
      property OnReceive: TENetEventRecv read FEventReceive write FEventReceive;
  end;

implementation

function GetPacketFlag( flag: TENetPacketFlags ): Integer;
begin
  Result := 0;
  if ENetPacketReliable in flag then
    Inc( Result, Ord(ENET_PACKET_FLAG_RELIABLE) );
  if ENetPacketUnsequenced in flag then
    Inc( Result, Ord(ENET_PACKET_FLAG_UNSEQUENCED) );
  if ENetPacketNoAllocate in flag then
    Inc( Result, Ord(ENET_PACKET_FLAG_NO_ALLOCATE) );
  if ENetPacketUnReliableFragment in flag then
    Inc( Result, Ord(ENET_PACKET_FLAG_UNRELIABLE_FRAGMENT) );
end;

{ TENetClass }

constructor TENetClass.Create( Port: word; bServer: Boolean );
begin
  FAddress.port := Port;
  FMaxPeer := 100;
  FMaxChannels := 255;
  FBandwidthIncoming := 0;
  FBandwidthOutgoing := 0;
  FIsServer := False;
  FConnectTimeout := 5000;
  FMessageTimeout := 100;
  FHost := nil;
  FPeer := nil;
  FEventNone := nil;
  FEventConnect := nil;
  FEventDisconnect := nil;
  FEventReceive := nil;

  FIsServer := bServer;
  FInit := enet_initialize() = 0;
end;

destructor TENetClass.Destroy();
begin
  Disconnect(True);
  if FInit then
    enet_deinitialize();
  inherited Destroy();
end;

function TENetClass.InitHost(): Boolean;
begin
  DeinitHost;
  if FInit then begin
    if FIsServer then begin
      // for server
      FAddress.host := ENET_HOST_ANY;
      FHost := enet_host_create( @FAddress, FMaxPeer, FMaxChannels,
        FBandwidthIncoming, FBandwidthOutgoing );
    end else begin
      // for client
      FMaxPeer := 1;
      FHost := enet_host_create( nil, FMaxPeer, FMaxChannels,
        FBandwidthIncoming, FBandwidthOutgoing );
    end;
  end;
  Result := FHost <> nil;
end;

procedure TENetClass.DeinitHost();
begin
  if FHost <> nil then
    enet_host_destroy(FHost);
  FHost := nil;
end;

function TENetClass.Connect( const Host: string; Port: Word ): Boolean;
begin
  Result := False;
  if not FIsServer then begin
    Disconnect(True);
    InitHost();
    enet_address_set_host( @FAddress, PAnsiChar(Host) );
    FAddress.port := Port;

    FClientData := Random(MaxInt);
    FPeer := enet_host_connect( FHost, @FAddress, FMaxChannels, FClientData );

    Result := FPeer <> nil;
    if Result then begin
      if (enet_host_service( FHost, @FEvent, FConnectTimeout ) > 0) and
        (FEvent.kind = ENET_EVENT_TYPE_CONNECT) then
      begin
        Result := True;
        if Assigned(FEventConnect) then
          FEventConnect(FEvent);
      end else begin
        enet_peer_reset(FPeer);
        FPeer := nil;
        DeinitHost();
      end;
    end;
  end;
end;

function TENetClass.Disconnect( bNow: Boolean ): Boolean;
begin
  Result := False;
  if (not FIsServer) and (FHost <> nil) then begin
    if FPeer <> nil then begin
      if bNow then
        enet_peer_disconnect_now( FPeer, FClientData )
      else
        enet_peer_disconnect( FPeer, FClientData );
      FPeer := nil;
    end;
  Result := True;
  end;
  DeinitHost();
end;

function TENetClass.SendMsg( Channel: Byte; Data: Pointer; Length: Integer;
  flag: TENetPacketFlags; WaitResponse: Boolean ): Boolean;
var
  FPacket : pENetPacket;
  PacketFlag : Cardinal;
begin
  Result := False;
  if FPeer <> nil then begin
    PacketFlag := GetPacketFlag(flag);
    FPacket := enet_packet_create( Data, Length, PacketFlag );
    if enet_peer_send( FPeer, Channel, FPacket ) = 0 then
      if WaitResponse then
        Result := ProcessMsg();
  end;
end;

function TENetClass.SendMsgEventPeer( Data: Pointer; Length: Integer;
  flag: TENetPacketFlags; WaitResponse: Boolean ): Boolean;
var
  FPacket : pENetPacket;
  PacketFlag : Cardinal;
begin
  Result := False;
  if FEvent.Peer <> nil then begin
    PacketFlag := GetPacketFlag(flag);
    FPacket := enet_packet_create( Data, Length, PacketFlag );
    if enet_peer_send( FEvent.Peer, FEvent.channelID, FPacket ) = 0 then
      if WaitResponse then
        Result := ProcessMsg();
  end;
end;

procedure TENetClass.FlushMsg();
begin
  if FHost <> nil then
    enet_host_flush(FHost);
end;

function TENetClass.BroadcastMsg( Channel: Byte; Data: Pointer; Length: Integer;
  flag: TENetPacketFlags; WaitResponse: Boolean ): Boolean;
var
  FPacket : pENetPacket;
  PacketFlag : Cardinal;
begin
  Result := False;
  if FPeer <> nil then begin
     PacketFlag := GetPacketFlag(flag);
     FPacket := enet_packet_create( Data, Length, PacketFlag );
     enet_host_broadcast( FHost, Channel, FPacket );
     if WaitResponse then
       Result := ProcessMsg();
  end;
end;

function TENetClass.ProcessMsg(): Boolean;
var
  broadcast : Boolean;
  bdChannel : Byte;
  packet : pENetPacket;
  pflag : Integer;
  svcres : integer;
begin
  Result := False;
  if FHost <> nil then begin
    SvcRes := enet_host_service( FHost, @FEvent, FMessageTimeout );
    if SvcRes > 0 then begin
      case FEvent.kind of
        ENET_EVENT_TYPE_NONE:
          if Assigned(FEventNone) then
            FEventNone(FEvent);
        ENET_EVENT_TYPE_CONNECT:
          if Assigned(FEventConnect) then
            FEventConnect(FEvent);
        ENET_EVENT_TYPE_DISCONNECT:
          if Assigned(FEventDisconnect) then
            FEventDisconnect(FEvent);
        ENET_EVENT_TYPE_RECEIVE: begin
          try
            if FIsServer then begin
               broadcast := True;
               pflag := FEvent.packet^.flags;
               bdChannel := FEvent.channelID;
            end;
            if Assigned(FEventReceive) then
               FEventReceive( FEvent, broadcast, bdChannel );
            if FIsServer and broadcast then begin
               packet := enet_packet_create( FEvent.packet^.data,
                 FEvent.packet^.dataLength, pflag );
               enet_host_broadcast( FHost, bdChannel, packet );
            end;
          finally
            enet_packet_destroy( FEvent.packet );
          end;
        end;
      end;
      Result := True;
    end;
  end;
end;

procedure TENetClass.Ping();
begin
  if (not FIsServer) and (FPeer <> nil) then
    enet_peer_ping(FPeer);
end;

end.

