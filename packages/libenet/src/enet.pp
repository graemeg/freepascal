{$MODE OBJFPC}
{$PACKRECORDS C}

{$LONGSTRINGS ON}
{$MACRO ON}

unit ENet;

{
  ENet - Reliable UDP networking library
  Copyright (c) 2002-2019 Lee Salzman

  DLL header for Free Pascal
  Version 4 for 1.3.14: 2019-07-01
  Copyright (c) 2015-2019 Dmitry D. Chernov aka Black Doomer

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
}

interface

uses
  ctypes,
{$IFDEF WINDOWS}
  WinSock2;
{$ELSE}
  BaseUnix, Sockets;
{$ENDIF}

////////////////////////////////////////////////////////////////////////////////
// types.h
////////////////////////////////////////////////////////////////////////////////

type
  enet_uint8 = cuchar;
  penet_uint8 = ^enet_uint8;

  enet_uint16 = cushort;
  penet_uint16 = ^enet_uint16;

  enet_uint32 = cuint;
  penet_uint32 = ^enet_uint32;

////////////////////////////////////////////////////////////////////////////////
// callbacks.h
////////////////////////////////////////////////////////////////////////////////

type
  pENetCallbacks = ^TENetCallbacks;
  TENetCallbacks = record
    malloc    : function( size: csize_t ): Pointer; cdecl;
    free      : procedure( memory: Pointer ); cdecl;
    no_memory : procedure(); cdecl;
  end;

////////////////////////////////////////////////////////////////////////////////
// protocol.h
////////////////////////////////////////////////////////////////////////////////

const
  { unnamed enums }
  ENET_PROTOCOL_MINIMUM_MTU             = 576;
  ENET_PROTOCOL_MAXIMUM_MTU             = 4096;
  ENET_PROTOCOL_MAXIMUM_PACKET_COMMANDS = 32;
  ENET_PROTOCOL_MINIMUM_WINDOW_SIZE     = 4096;
  ENET_PROTOCOL_MAXIMUM_WINDOW_SIZE     = 65536;
  ENET_PROTOCOL_MINIMUM_CHANNEL_COUNT   = 1;
  ENET_PROTOCOL_MAXIMUM_CHANNEL_COUNT   = 255;
  ENET_PROTOCOL_MAXIMUM_PEER_ID         = $FFF;
  ENET_PROTOCOL_MAXIMUM_FRAGMENT_COUNT  = 1024 * 1024;

  { enum ENetProtocolFlag }
  ENET_PROTOCOL_COMMAND_FLAG_ACKNOWLEDGE = 1 shl 7;
  ENET_PROTOCOL_COMMAND_FLAG_UNSEQUENCED = 1 shl 6;
  ENET_PROTOCOL_HEADER_FLAG_COMPRESSED   = 1 shl 14;
  ENET_PROTOCOL_HEADER_FLAG_SENT_TIME    = 1 shl 15;
  ENET_PROTOCOL_HEADER_FLAG_MASK         = ENET_PROTOCOL_HEADER_FLAG_COMPRESSED or
                                           ENET_PROTOCOL_HEADER_FLAG_SENT_TIME;
  ENET_PROTOCOL_HEADER_SESSION_MASK      = 3 shl 12;
  ENET_PROTOCOL_HEADER_SESSION_SHIFT     = 12;

type
  { enums }
  ENetProtocolCommand = ( ENET_PROTOCOL_COMMAND_NONE,
                          ENET_PROTOCOL_COMMAND_ACKNOWLEDGE,
                          ENET_PROTOCOL_COMMAND_CONNECT,
                          ENET_PROTOCOL_COMMAND_VERIFY_CONNECT,
                          ENET_PROTOCOL_COMMAND_DISCONNECT,
                          ENET_PROTOCOL_COMMAND_PING,
                          ENET_PROTOCOL_COMMAND_SEND_RELIABLE,
                          ENET_PROTOCOL_COMMAND_SEND_UNRELIABLE,
                          ENET_PROTOCOL_COMMAND_SEND_FRAGMENT,
                          ENET_PROTOCOL_COMMAND_SEND_UNSEQUENCED,
                          ENET_PROTOCOL_COMMAND_BANDWIDTH_LIMIT,
                          ENET_PROTOCOL_COMMAND_THROTTLE_CONFIGURE,
                          ENET_PROTOCOL_COMMAND_SEND_UNRELIABLE_FRAGMENT,
                          ENET_PROTOCOL_COMMAND_COUNT,
                          ENET_PROTOCOL_COMMAND_MASK = $0F );

  { structs / unions }
  pENetProtocolHeader = ^ENetProtocolHeader;
  ENetProtocolHeader = packed record
    peerID   : enet_uint16;
    sentTime : enet_uint16;
  end;

  pENetProtocolCommandHeader = ^ENetProtocolCommandHeader;
  ENetProtocolCommandHeader = packed record
    command                : enet_uint8;
    channelID              : enet_uint8;
    reliableSequenceNumber : enet_uint16;
  end;

  pENetProtocolAcknowledge = ^ENetProtocolAcknowledge;
  ENetProtocolAcknowledge = packed record
    header                         : ENetProtocolCommandHeader;
    receivedReliableSequenceNumber : enet_uint16;
    receivedSentTime               : enet_uint16;
  end;

  pENetProtocolConnect = ^ENetProtocolConnect;
  ENetProtocolConnect = packed record
    header                     : ENetProtocolCommandHeader;
    outgoingPeerID             : enet_uint16;
    incomingSessionID          : enet_uint8;
    outgoingSessionID          : enet_uint8;
    mtu                        : enet_uint32;
    windowSize                 : enet_uint32;
    channelCount               : enet_uint32;
    incomingBandwidth          : enet_uint32;
    outgoingBandwidth          : enet_uint32;
    packetThrottleInterval     : enet_uint32;
    packetThrottleAcceleration : enet_uint32;
    packetThrottleDeceleration : enet_uint32;
    connectID                  : enet_uint32;
    data                       : enet_uint32;
  end;

  pENetProtocolVerifyConnect = ^ENetProtocolVerifyConnect;
  ENetProtocolVerifyConnect = packed record
    header                     : ENetProtocolCommandHeader;
    outgoingPeerID             : enet_uint16;
    incomingSessionID          : enet_uint8;
    outgoingSessionID          : enet_uint8;
    mtu                        : enet_uint32;
    windowSize                 : enet_uint32;
    channelCount               : enet_uint32;
    incomingBandwidth          : enet_uint32;
    outgoingBandwidth          : enet_uint32;
    packetThrottleInterval     : enet_uint32;
    packetThrottleAcceleration : enet_uint32;
    packetThrottleDeceleration : enet_uint32;
    connectID                  : enet_uint32;
  end;

  pENetProtocolBandwidthLimit = ^ENetProtocolBandwidthLimit;
  ENetProtocolBandwidthLimit = packed record
    header            : ENetProtocolCommandHeader;
    incomingBandwidth : enet_uint32;
    outgoingBandwidth : enet_uint32;
  end;

  pENetProtocolThrottleConfigure = ^ENetProtocolThrottleConfigure;
  ENetProtocolThrottleConfigure = packed record
    header                     : ENetProtocolCommandHeader;
    packetThrottleInterval     : enet_uint32;
    packetThrottleAcceleration : enet_uint32;
    packetThrottleDeceleration : enet_uint32;
  end;

  pENetProtocolDisconnect = ^ENetProtocolDisconnect;
  ENetProtocolDisconnect = packed record
    header : ENetProtocolCommandHeader;
    data   : enet_uint32;
  end;

  pENetProtocolPing = ^ENetProtocolPing;
  ENetProtocolPing = packed record
    header : ENetProtocolCommandHeader;
  end;

  pENetProtocolSendReliable = ^ENetProtocolSendReliable;
  ENetProtocolSendReliable = packed record
    header     : ENetProtocolCommandHeader;
    dataLength : enet_uint16;
  end;

  pENetProtocolSendUnreliable = ^ENetProtocolSendUnreliable;
  ENetProtocolSendUnreliable = packed record
    header                   : ENetProtocolCommandHeader;
    unreliableSequenceNumber : enet_uint16;
    dataLength               : enet_uint16;
  end;

  pENetProtocolSendUnsequenced = ^ENetProtocolSendUnsequenced;
  ENetProtocolSendUnsequenced = packed record
    header           : ENetProtocolCommandHeader;
    unsequencedGroup : enet_uint16;
    dataLength       : enet_uint16;
  end;

  pENetProtocolSendFragment = ^ENetProtocolSendFragment;
  ENetProtocolSendFragment = packed record
    header              : ENetProtocolCommandHeader;
    startSequenceNumber : enet_uint16;
    dataLength          : enet_uint16;
    fragmentCount       : enet_uint32;
    fragmentNumber      : enet_uint32;
    totalLength         : enet_uint32;
    fragmentOffset      : enet_uint32;
  end;

  pENetProtocol = ^TENetProtocol;
  TENetProtocol = packed record  // union
    case Byte of
    $00: (header            : ENetProtocolCommandHeader);
    $01: (acknowledge       : ENetProtocolAcknowledge);
    $02: (connect           : ENetProtocolConnect);
    $03: (verifyConnect     : ENetProtocolVerifyConnect);
    $04: (disconnect        : ENetProtocolDisconnect);
    $05: (ping              : ENetProtocolPing);
    $06: (sendReliable      : ENetProtocolSendReliable);
    $07: (sendUnreliable    : ENetProtocolSendUnreliable);
    $08: (sendUnsequenced   : ENetProtocolSendUnsequenced);
    $09: (sendFragment      : ENetProtocolSendFragment);
    $0A: (bandwidthLimit    : ENetProtocolBandwidthLimit);
    $0B: (throttleConfigure : ENetProtocolThrottleConfigure);
  end;

////////////////////////////////////////////////////////////////////////////////
// win32.h / unix.h
////////////////////////////////////////////////////////////////////////////////

const
{$IFDEF WINDOWS}
  ENET_SOCKET_NULL = INVALID_SOCKET;
{$ELSE}
  ENET_SOCKET_NULL = -1;
{$ENDIF}

type
{$IFDEF WINDOWS}
  ENetSocket = TSocket;
{$ELSE}
  ENetSocket = cint;
{$ENDIF}

  ENetSocketSet = TFDSet;
  pENetSocketSet = ^ENetSocketSet;

  pENetBuffer = ^ENetBuffer;
  ENetBuffer = record
  {$IFDEF WINDOWS}
    dataLength : csize_t;
    data       : Pointer;
  {$ELSE}
    data       : Pointer;
    dataLength : csize_t;
  {$ENDIF}
  end;

{ inline macros }

function ENET_HOST_TO_NET_16( value: cuint16 ): cuint16; inline;
function ENET_HOST_TO_NET_32( value: cuint32 ): cuint32; inline;

function ENET_NET_TO_HOST_16( value: cuint16 ): cuint16; inline;
function ENET_NET_TO_HOST_32( value: cuint32 ): cuint32; inline;

procedure ENET_SOCKETSET_EMPTY( var sockset: ENetSocketSet ); inline;
procedure ENET_SOCKETSET_ADD( var sockset: ENetSocketSet; socket: ENetSocket ); inline;
procedure ENET_SOCKETSET_REMOVE( var sockset: ENetSocketSet; socket: ENetSocket ); inline;
function ENET_SOCKETSET_CHECK( var sockset: ENetSocketSet; socket: ENetSocket ): cbool; inline;

////////////////////////////////////////////////////////////////////////////////
// list.h
////////////////////////////////////////////////////////////////////////////////

type
  pENetListNode = ^ENetListNode;
  ENetListNode = record
    next     : pENetListNode;
    previous : pENetListNode;
  end;

  pENetList = ^TENetList;
  TENetList = record
    sentinel : ENetListNode;
  end;

  ENetListIterator = pENetListNode;

{ inline macros }
function enet_list_begin( list: pENetList ): ENetListIterator; inline;
function enet_list_end( list: pENetList ): ENetListIterator; inline;

function enet_list_empty( list: pENetList ): Boolean; inline;

function enet_list_next( iterator: ENetListIterator ): ENetListIterator; inline;
function enet_list_previous( iterator: ENetListIterator ): ENetListIterator; inline;

function enet_list_front( list: pENetList ): Pointer; inline;
function enet_list_back( list: pENetList ): Pointer; inline;

////////////////////////////////////////////////////////////////////////////////
// time.h
////////////////////////////////////////////////////////////////////////////////

const
  ENET_TIME_OVERFLOW = 86400000;

{ inline macros }
function ENET_TIME_LESS( const a, b: cint ): cbool; inline;
function ENET_TIME_GREATER( const a, b: cint ): cbool; inline;

function ENET_TIME_LESS_EQUAL( const a, b: cint ): cbool; inline;
function ENET_TIME_GREATER_EQUAL( const a, b: cint ): cbool; inline;

function ENET_TIME_DIFFERENCE( const a, b: cint ): cint; inline;

////////////////////////////////////////////////////////////////////////////////
// enet.h
////////////////////////////////////////////////////////////////////////////////

{$DEFINE libraryENet := cdecl; external 'enet'}

const
  { defines }
  ENET_VERSION_MAJOR = 1;
  ENET_VERSION_MINOR = 3;
  ENET_VERSION_PATCH = 14;

  ENET_HOST_ANY        = 0;
  ENET_HOST_BROADCAST_ = $FFFFFFFF;  // "_" due to the name conflict
  ENET_PORT_ANY        = 0;

  ENET_BUFFER_MAXIMUM = 1 + 2 * ENET_PROTOCOL_MAXIMUM_PACKET_COMMANDS;

  { unnamed enums }
  ENET_HOST_RECEIVE_BUFFER_SIZE          = 256 * 1024;
  ENET_HOST_SEND_BUFFER_SIZE             = 256 * 1024;
  ENET_HOST_BANDWIDTH_THROTTLE_INTERVAL  = 1000;
  ENET_HOST_DEFAULT_MTU                  = 1400;
  ENET_HOST_DEFAULT_MAXIMUM_PACKET_SIZE  = 32 * 1024 * 1024;
  ENET_HOST_DEFAULT_MAXIMUM_WAITING_DATA = 32 * 1024 * 1024;

  ENET_PEER_DEFAULT_ROUND_TRIP_TIME      = 500;
  ENET_PEER_DEFAULT_PACKET_THROTTLE      = 32;
  ENET_PEER_PACKET_THROTTLE_SCALE        = 32;
  ENET_PEER_PACKET_THROTTLE_COUNTER      = 7;
  ENET_PEER_PACKET_THROTTLE_ACCELERATION = 2;
  ENET_PEER_PACKET_THROTTLE_DECELERATION = 2;
  ENET_PEER_PACKET_THROTTLE_INTERVAL     = 5000;
  ENET_PEER_PACKET_LOSS_SCALE            = 1 shl 16;
  ENET_PEER_PACKET_LOSS_INTERVAL         = 10000;
  ENET_PEER_WINDOW_SIZE_SCALE            = 64 * 1024;
  ENET_PEER_TIMEOUT_LIMIT                = 32;
  ENET_PEER_TIMEOUT_MINIMUM              = 5000;
  ENET_PEER_TIMEOUT_MAXIMUM              = 30000;
  ENET_PEER_PING_INTERVAL_               = 500;  // "_" due to the name conflict
  ENET_PEER_UNSEQUENCED_WINDOWS          = 64;
  ENET_PEER_UNSEQUENCED_WINDOW_SIZE      = 1024;
  ENET_PEER_FREE_UNSEQUENCED_WINDOWS     = 32;
  ENET_PEER_RELIABLE_WINDOWS             = 16;
  ENET_PEER_RELIABLE_WINDOW_SIZE         = $1000;
  ENET_PEER_FREE_RELIABLE_WINDOWS        = 8;

  { enum ENetSocketWait }
  ENET_SOCKET_WAIT_NONE       = 0;
  ENET_SOCKET_WAIT_SEND       = 1 shl 0;
  ENET_SOCKET_WAIT_RECEIVE    = 1 shl 1;
  ENET_SOCKET_WAIT_INTERRUPT  = 1 shl 2;

  { enum ENetPacketFlag }
  ENET_PACKET_FLAG_RELIABLE            = 1 shl 0;
  ENET_PACKET_FLAG_UNSEQUENCED         = 1 shl 1;
  ENET_PACKET_FLAG_NO_ALLOCATE         = 1 shl 2;
  ENET_PACKET_FLAG_UNRELIABLE_FRAGMENT = 1 shl 3;
  ENET_PACKET_FLAG_SENT                = 1 shl 8;

type
  { enums }
  ENetSocketType = ( ENET_SOCKET_TYPE_STREAM   = 1,
                     ENET_SOCKET_TYPE_DATAGRAM = 2 );
  
  ENetSocketOption = ( ENET_SOCKOPT_NONBLOCK  = 1,
                       ENET_SOCKOPT_BROADCAST = 2,
                       ENET_SOCKOPT_RCVBUF    = 3,
                       ENET_SOCKOPT_SNDBUF    = 4,
                       ENET_SOCKOPT_REUSEADDR = 5,
                       ENET_SOCKOPT_RCVTIMEO  = 6,
                       ENET_SOCKOPT_SNDTIMEO  = 7,
                       ENET_SOCKOPT_ERROR     = 8,
                       ENET_SOCKOPT_NODELAY   = 9 );

  ENetSocketShutdown = ( ENET_SOCKET_SHUTDOWN_READ,
                         ENET_SOCKET_SHUTDOWN_WRITE,
                         ENET_SOCKET_SHUTDOWN_READ_WRITE );

  ENetPeerState = ( ENET_PEER_STATE_DISCONNECTED,
                    ENET_PEER_STATE_CONNECTING,
                    ENET_PEER_STATE_ACKNOWLEDGING_CONNECT,
                    ENET_PEER_STATE_CONNECTION_PENDING,
                    ENET_PEER_STATE_CONNECTION_SUCCEEDED,
                    ENET_PEER_STATE_CONNECTED,
                    ENET_PEER_STATE_DISCONNECT_LATER,
                    ENET_PEER_STATE_DISCONNECTING,
                    ENET_PEER_STATE_ACKNOWLEDGING_DISCONNECT,
                    ENET_PEER_STATE_ZOMBIE );

  ENetEventType = ( ENET_EVENT_TYPE_NONE,
                    ENET_EVENT_TYPE_CONNECT,
                    ENET_EVENT_TYPE_DISCONNECT,
                    ENET_EVENT_TYPE_RECEIVE );

  { typedefs }
  ENetVersion = enet_uint32;

  { pointers to structs }
  pENetAddress         = ^ENetAddress;
  pENetPacket          = ^ENetPacket;
  pENetChannel         = ^ENetChannel;
  pENetPeer            = ^ENetPeer;
  pENetCompressor      = ^ENetCompressor;
  pENetHost            = ^ENetHost;
  pENetEvent           = ^ENetEvent;

  { callbacks }
  ENetPacketFreeCallback = procedure( packet: pENetPacket ); cdecl;
  ENetChecksumCallback = function( const buffers: pENetBuffer;
    bufferCount: csize_t ): enet_uint32; cdecl;
  ENetInterceptCallback = function( host: pENetHost;
    event: pENetEvent ): cint; cdecl;

  { structs }
  ENetAddress = record
    host : enet_uint32;
    port : enet_uint16;
  end;
  ENetPacket = record
    referenceCount : csize_t;
    flags          : enet_uint32;
    data           : penet_uint8;
    dataLength     : csize_t;
    freeCallback   : ENetPacketFreeCallback;
    userData       : Pointer;
  end;
  ENetChannel = record
    outgoingReliableSequenceNumber   : enet_uint16;
    outgoingUnreliableSequenceNumber : enet_uint16;
    usedReliableWindows              : enet_uint16;
    reliableWindows                  : array[ 0..ENET_PEER_RELIABLE_WINDOWS-1 ] of enet_uint16;
    incomingReliableSequenceNumber   : enet_uint16;
    incomingUnreliableSequenceNumber : enet_uint16;
    incomingReliableCommands         : TENetList;
    incomingUnreliableCommands       : TENetList;
  end;
  ENetPeer = record
    dispatchList                   : ENetListNode;
    host                           : pENetHost;
    outgoingPeerID                 : enet_uint16;
    incomingPeerID                 : enet_uint16;
    connectID                      : enet_uint32;
    outgoingSessionID              : enet_uint8;
    incomingSessionID              : enet_uint8;
    address                        : ENetAddress;
    data                           : Pointer;
    state                          : ENetPeerState;
    channels                       : pENetChannel;
    channelCount                   : csize_t;
    incomingBandwidth              : enet_uint32;
    outgoingBandwidth              : enet_uint32;
    incomingBandwidthThrottleEpoch : enet_uint32;
    outgoingBandwidthThrottleEpoch : enet_uint32;
    incomingDataTotal              : enet_uint32;
    outgoingDataTotal              : enet_uint32;
    lastSendTime                   : enet_uint32;
    lastReceiveTime                : enet_uint32;
    nextTimeout                    : enet_uint32;
    earliestTimeout                : enet_uint32;
    packetLossEpoch                : enet_uint32;
    packetsSent                    : enet_uint32;
    packetsLost                    : enet_uint32;
    packetLoss                     : enet_uint32;
    packetLossVariance             : enet_uint32;
    packetThrottle                 : enet_uint32;
    packetThrottleLimit            : enet_uint32;
    packetThrottleCounter          : enet_uint32;
    packetThrottleEpoch            : enet_uint32;
    packetThrottleAcceleration     : enet_uint32;
    packetThrottleDeceleration     : enet_uint32;
    packetThrottleInterval         : enet_uint32;
    pingInterval                   : enet_uint32;
    timeoutLimit                   : enet_uint32;
    timeoutMinimum                 : enet_uint32;
    timeoutMaximum                 : enet_uint32;
    lastRoundTripTime              : enet_uint32;
    lowestRoundTripTime            : enet_uint32;
    lastRoundTripTimeVariance      : enet_uint32;
    highestRoundTripTimeVariance   : enet_uint32;
    roundTripTime                  : enet_uint32;
    roundTripTimeVariance          : enet_uint32;
    mtu                            : enet_uint32;
    windowSize                     : enet_uint32;
    reliableDataInTransit          : enet_uint32;
    outgoingReliableSequenceNumber : enet_uint16;
    acknowledgements               : TENetList;
    sentReliableCommands           : TENetList;
    sentUnreliableCommands         : TENetList;
    outgoingReliableCommands       : TENetList;
    outgoingUnreliableCommands     : TENetList;
    dispatchedCommands             : TENetList;
    needsDispatch                  : cint;
    incomingUnsequencedGroup       : enet_uint16;
    outgoingUnsequencedGroup       : enet_uint16;
    unsequencedWindow              : array[ 0..(ENET_PEER_UNSEQUENCED_WINDOW_SIZE div 32)-1 ] of enet_uint32;
    eventData                      : enet_uint32;
    totalWaitingData               : csize_t;
  end;
  ENetCompressor = record
    context    : Pointer;
    compress   : function( context: Pointer; const inBuffers: pENetBuffer; inBufferCount, inLimit: csize_t; outData: penet_uint8; outLimit: csize_t ): csize_t; cdecl;
    decompress : function( context: Pointer; const inData: penet_uint8; inLimit: csize_t; outData: penet_uint8; outLimit: csize_t ): csize_t; cdecl;
    destroy    : procedure( context: Pointer ); cdecl;
  end;
  ENetHost = record
    socket                     : ENetSocket;
    address                    : ENetAddress;
    incomingBandwidth          : enet_uint32;
    outgoingBandwidth          : enet_uint32;
    bandwidthThrottleEpoch     : enet_uint32;
    mtu                        : enet_uint32;
    randomSeed                 : enet_uint32;
    recalculateBandwidthLimits : cint;
    peers                      : pENetPeer;
    peerCount                  : csize_t;
    channelLimit               : csize_t;
    serviceTime                : enet_uint32;
    dispatchQueue              : TENetList;
    continueSending            : cint;
    packetSize                 : csize_t;
    headerFlags                : enet_uint16;
    commands                   : array[ 0..ENET_PROTOCOL_MAXIMUM_PACKET_COMMANDS-1 ] of TENetProtocol;
    commandCount               : csize_t;
    buffers                    : array[ 0..ENET_BUFFER_MAXIMUM-1 ] of ENetBuffer;
    bufferCount                : csize_t;
    checksum                   : ENetChecksumCallback;
    compressor                 : ENetCompressor;
    packetData                 : array[ 0..1, 0..ENET_PROTOCOL_MAXIMUM_MTU-1 ] of enet_uint8;
    receivedAddress            : ENetAddress;
    receivedData               : penet_uint8;
    receivedDataLength         : csize_t;
    totalSentData              : enet_uint32;
    totalSentPackets           : enet_uint32;
    totalReceivedData          : enet_uint32;
    totalReceivedPackets       : enet_uint32;
    intercept                  : ENetInterceptCallback;
    connectedPeers             : csize_t;
    bandwidthLimitedPeers      : csize_t;
    duplicatePeers             : csize_t;
    maximumPacketSize          : csize_t;
    maximumWaitingData         : csize_t;
  end;
  ENetEvent = record
    kind      : ENetEventType;  // originally "type", which conflicts
    peer      : pENetPeer;
    channelID : enet_uint8;
    data      : enet_uint32;
    packet    : pENetPacket;
  end;

{ inline macros }
function ENET_VERSION_CREATE( const major, minor, patch: cint ): ENetVersion; inline;
function ENET_VERSION_GET_MAJOR( const version: ENetVersion ): cint; inline;
function ENET_VERSION_GET_MINOR( const version: ENetVersion ): cint; inline;
function ENET_VERSION_GET_PATCH( const version: ENetVersion ): cint; inline;
function ENET_VERSION(): ENetVersion; inline;

{ library functions }
function enet_initialize(): cint; libraryENet;
function enet_initialize_with_callbacks( version: ENetVersion; const inits: pENetCallbacks ): cint; libraryENet;
procedure enet_deinitialize(); libraryENet;
function enet_linked_version(): ENetVersion; libraryENet;

function enet_time_get(): enet_uint32; libraryENet;
procedure enet_time_set( newTimeBase: enet_uint32 ); libraryENet;

function enet_socket_create( kind: ENetSocketType ): ENetSocket; libraryENet;
function enet_socket_bind( socket: ENetSocket; const address: pENetAddress ): cint; libraryENet;
function enet_socket_get_address( socket: ENetSocket; address: pENetAddress ): cint; libraryENet;
function enet_socket_listen( socket: ENetSocket; backlog: cint ): cint; libraryENet;
function enet_socket_accept( socket: ENetSocket; address: pENetAddress ): ENetSocket; libraryENet;
function enet_socket_connect( socket: ENetSocket; const address: pENetAddress ): cint; libraryENet;
function enet_socket_send( socket: ENetSocket; const address: pENetAddress; const buffers: pENetBuffer; bufferCount: csize_t ): cint; libraryENet;
function enet_socket_receive( socket: ENetSocket; address: pENetAddress; buffers: pENetBuffer; bufferCount: csize_t ): cint; libraryENet;
function enet_socket_wait( socket: ENetSocket; condition: penet_uint32; timeout: enet_uint32 ): cint; libraryENet;
function enet_socket_set_option( socket: ENetSocket; option: ENetSocketOption; value: cint ): cint; libraryENet;
function enet_socket_get_option( socket: ENetSocket; option: ENetSocketOption; value: pcint ): cint; libraryENet;
function enet_socket_shutdown( socket: ENetSocket; how: ENetSocketShutdown ): cint; libraryENet;
procedure enet_socket_destroy( socket: ENetSocket ); libraryENet;
function enet_socketset_select( maxSocket: ENetSocket; readSet: pENetSocketSet; writeSet: pENetSocketSet; timeout: enet_uint32 ): cint; libraryENet;

function enet_address_set_host_ip( address: pENetAddress; const hostName: PChar ): cint; libraryENet;
function enet_address_set_host( address: pENetAddress; const hostName: PChar ): cint; libraryENet;
function enet_address_get_host_ip( const address: pENetAddress; hostName: PChar; nameLength: csize_t ): cint; libraryENet;
function enet_address_get_host( const address: pENetAddress; hostName: PChar; nameLength: csize_t ): cint; libraryENet;

function enet_packet_create( const data: Pointer; dataLength: csize_t; flags: enet_uint32 ): pENetPacket; libraryENet;
procedure enet_packet_destroy( packet: pENetPacket ); libraryENet;
function enet_packet_resize( packet: pENetPacket; dataLength: csize_t ): cint; libraryENet;
function enet_crc32( const buffers: pENetBuffer; bufferCount: csize_t ): enet_uint32; libraryENet;

function enet_host_create( const address: pENetAddress; peerCount, channelLimit: csize_t; incomingBandwidth, outgoingBandwidth: enet_uint32 ): pENetHost; libraryENet;
procedure enet_host_destroy( host: pENetHost ); libraryENet;
function enet_host_connect( host: pENetHost; const address: pENetAddress; channelCount: csize_t; data: enet_uint32 ): pENetPeer; libraryENet;
function enet_host_check_events( host: pENetHost; event: pENetEvent ): cint; libraryENet;
function enet_host_service( host: pENetHost; event: pENetEvent; timeout: enet_uint32 ): cint; libraryENet;
procedure enet_host_flush( host: pENetHost ); libraryENet;
procedure enet_host_broadcast( host: pENetHost; channelID: enet_uint8; packet: pENetPacket ); libraryENet;
procedure enet_host_compress( host: pENetHost; const compressor: pENetCompressor ); libraryENet;
function enet_host_compress_with_range_coder( host: pENetHost ): cint; libraryENet;
procedure enet_host_channel_limit( host: pENetHost; channelLimit: csize_t ); libraryENet;
procedure enet_host_bandwidth_limit( host: pENetHost; incomingBandwidth, outgoingBandwidth: enet_uint32 ); libraryENet;

function enet_peer_send( peer: pENetPeer; channelID: enet_uint8; packet: pENetPacket ): cint; libraryENet;
function enet_peer_receive( peer: pENetPeer; channelID: penet_uint8 ): pENetPacket; libraryENet;
procedure enet_peer_ping( peer: pENetPeer ); libraryENet;
procedure enet_peer_ping_interval( peer: pENetPeer; pingInterval: enet_uint32 ); libraryENet;
procedure enet_peer_timeout( peer: pENetPeer; timeoutLimit, timeoutMinimum, timeoutMaximum: enet_uint32 ); libraryENet;
procedure enet_peer_reset( peer: pENetPeer ); libraryENet;
procedure enet_peer_disconnect( peer: pENetPeer; data: enet_uint32 ); libraryENet;
procedure enet_peer_disconnect_now( peer: pENetPeer; data: enet_uint32 ); libraryENet;
procedure enet_peer_disconnect_later( peer: pENetPeer; data: enet_uint32 ); libraryENet;
procedure enet_peer_throttle_configure( peer: pENetPeer; interval, acceleration, deceleration: enet_uint32 ); libraryENet;

function enet_range_coder_create(): Pointer; libraryENet;
procedure enet_range_coder_destroy( context: Pointer ); libraryENet;
function enet_range_coder_compress( context: Pointer; const inBuffers: pENetBuffer; inBufferCount, inLiit: csize_t; outData: penet_uint8; outLimit: csize_t ): csize_t; libraryENet;
function enet_range_coder_decompress( context: Pointer; const inData: penet_uint8; inLimit: csize_t; outData: penet_uint8; outLimit: csize_t ): csize_t; libraryENet;

implementation

////////////////////////////////////////////////////////////////////////////////
// win32.h / unix.h
////////////////////////////////////////////////////////////////////////////////

function ENET_HOST_TO_NET_16( value: cuint16 ): cuint16;
begin
  Result := htons(value);
end;

function ENET_HOST_TO_NET_32( value: cuint32 ): cuint32;
begin
  Result := htonl(value);
end;

function ENET_NET_TO_HOST_16( value: cuint16 ): cuint16;
begin
  Result := ntohs(value);
end;

function ENET_NET_TO_HOST_32( value: cuint32 ): cuint32;
begin
  Result := ntohl(value);
end;

procedure ENET_SOCKETSET_EMPTY( var sockset: ENetSocketSet );
begin
{$IFDEF WINDOWS}
  FD_ZERO( sockset );
{$ELSE}
  fpFD_ZERO( sockset );
{$ENDIF}
end;

procedure ENET_SOCKETSET_ADD( var sockset: ENetSocketSet; socket: ENetSocket );
begin
{$IFDEF WINDOWS}
  FD_SET( socket, sockset );
{$ELSE}
  fpFD_SET( socket, sockset );
{$ENDIF}
end;

procedure ENET_SOCKETSET_REMOVE( var sockset: ENetSocketSet; socket: ENetSocket );
begin
{$IFDEF WINDOWS}
  FD_CLR( socket, sockset );
{$ELSE}
  fpFD_CLR( socket, sockset );
{$ENDIF}
end;

function ENET_SOCKETSET_CHECK( var sockset: ENetSocketSet; socket: ENetSocket ): cbool;
begin
{$IFDEF WINDOWS}
  Result := FD_ISSET( socket, sockset );
{$ELSE}
  Result := fpFD_ISSET( socket, sockset ) = 1;
{$ENDIF}
end;

////////////////////////////////////////////////////////////////////////////////
// list.h
////////////////////////////////////////////////////////////////////////////////

function enet_list_begin( list: pENetList ): ENetListIterator;
begin
  Result := list^.sentinel.next;
end;

function enet_list_end( list: pENetList ): ENetListIterator;
begin
  Result := @( list^.sentinel );
end;

function enet_list_empty( list: pENetList ): Boolean;
begin
  Result := enet_list_begin(list) = enet_list_end(list);
end;

function enet_list_next( iterator: ENetListIterator ): ENetListIterator;
begin
  Result := iterator^.next;
end;

function enet_list_previous( iterator: ENetListIterator ): ENetListIterator;
begin
  Result := iterator^.previous;
end;

function enet_list_front( list: pENetList ): Pointer;
begin
  Result := Pointer( list^.sentinel.next );
end;

function enet_list_back( list: pENetList ): Pointer;
begin
  Result := Pointer( list^.sentinel.previous );
end;

////////////////////////////////////////////////////////////////////////////////
// time.h
////////////////////////////////////////////////////////////////////////////////

function ENET_TIME_LESS( const a, b: cint ): cbool;
begin
  Result := (a - b) >= ENET_TIME_OVERFLOW;
end;

function ENET_TIME_GREATER( const a, b: cint ): cbool;
begin
  Result := (b - a) >= ENET_TIME_OVERFLOW;
end;

function ENET_TIME_LESS_EQUAL( const a, b: cint ): cbool;
begin
  Result := not ENET_TIME_GREATER(a, b);
end;

function ENET_TIME_GREATER_EQUAL( const a, b: cint ): cbool;
begin
  Result := not ENET_TIME_LESS(a, b);
end;

function ENET_TIME_DIFFERENCE( const a, b: cint ): cint;
begin
  if (a - b) >= ENET_TIME_OVERFLOW then
    Result := b - a
  else
    Result := a - b;
end;

////////////////////////////////////////////////////////////////////////////////
// enet.h
////////////////////////////////////////////////////////////////////////////////

function ENET_VERSION_CREATE( const major, minor, patch: cint ): ENetVersion;
begin
  Result := (major shl 16) or (minor shl 8) or patch;
end;

function ENET_VERSION_GET_MAJOR( const version: ENetVersion ): cint;
begin
  Result := (version shr 16) and $FF;
end;

function ENET_VERSION_GET_MINOR( const version: ENetVersion ): cint;
begin
  Result := (version shr 8) and $FF;
end;

function ENET_VERSION_GET_PATCH( const version: ENetVersion ): cint;
begin
  Result := version and $FF;
end;

function ENET_VERSION(): ENetVersion;
begin
  Result := ENET_VERSION_CREATE( ENET_VERSION_MAJOR, ENET_VERSION_MINOR, ENET_VERSION_PATCH );
end;

end.

