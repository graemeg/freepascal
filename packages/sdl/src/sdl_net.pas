unit sdl_net;
{******************************************************************************}
{
  $Id: sdl_net.pas,v 1.7 2005/01/01 02:14:21 savage Exp $

}
{                                                                              }
{       Borland Delphi SDL_Net - A x-platform network library for use with SDL.}
{       Conversion of the Simple DirectMedia Layer Network Headers             }
{                                                                              }
{ Portions created by Sam Lantinga <slouken@devolution.com> are                }
{ Copyright (C) 1997, 1998, 1999, 2000, 2001  Sam Lantinga                     }
{ 5635-34 Springhouse Dr.                                                      }
{ Pleasanton, CA 94588 (USA)                                                   }
{                                                                              }
{ All Rights Reserved.                                                         }
{                                                                              }
{ The original files are : SDL_net.h                                           }
{                                                                              }
{ The initial developer of this Pascal code was :                              }
{ Dominqiue Louis <Dominique@SavageSoftware.com.au>                            }
{                                                                              }
{ Portions created by Dominqiue Louis are                                      }
{ Copyright (C) 2000 - 2001 Dominqiue Louis.                                   }
{                                                                              }
{                                                                              }
{ Contributor(s)                                                               }
{ --------------                                                               }
{ Matthias Thoma <ma.thoma@gmx.de>                                             }
{                                                                              }
{ Obtained through:                                                            }
{ Joint Endeavour of Delphi Innovators ( Project JEDI )                        }
{                                                                              }
{ You may retrieve the latest version of this file at the Project              }
{ JEDI home page, located at http://delphi-jedi.org                            }
{                                                                              }
{ The contents of this file are used with permission, subject to               }
{ the Mozilla Public License Version 1.1 (the "License"); you may              }
{ not use this file except in compliance with the License. You may             }
{ obtain a copy of the License at                                              }
{ http://www.mozilla.org/MPL/MPL-1.1.html                                      }
{                                                                              }
{ Software distributed under the License is distributed on an                  }
{ "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or               }
{ implied. See the License for the specific language governing                 }
{ rights and limitations under the License.                                    }
{                                                                              }
{ Description                                                                  }
{ -----------                                                                  }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{ Requires                                                                     }
{ --------                                                                     }
{   SDL.pas somehere in your search path                                       }
{                                                                              }
{ Programming Notes                                                            }
{ -----------------                                                            }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{ Revision History                                                             }
{ ----------------                                                             }
{   April   09 2001 - DL : Initial Translation                                 }
{                                                                              }
{   April   03 2003 - DL : Added jedi-sdl.inc include file to support more     }
{                          Pascal compilers. Initial support is now included   }
{                          for GnuPascal, VirtualPascal, TMT and obviously     }
{                          continue support for Delphi Kylix and FreePascal.   }
{                                                                              }
{   April   24 2003 - DL : under instruction from Alexey Barkovoy, I have added}
{                          better TMT Pascal support and under instruction     }
{                          from Prof. Abimbola Olowofoyeku (The African Chief),}
{                          I have added better Gnu Pascal support              }
{                                                                              }
{   April   30 2003 - DL : under instruction from David Mears AKA              }
{                          Jason Siletto, I have added FPC Linux support.      }
{                          This was compiled with fpc 1.1, so remember to set  }
{                          include file path. ie. -Fi/usr/share/fpcsrc/rtl/*   }
{                                                                              }
{
  $Log: sdl_net.pas,v $
  Revision 1.7  2005/01/01 02:14:21  savage
  Updated to v1.2.5

  Revision 1.6  2004/08/14 22:54:30  savage
  Updated so that Library name defines are correctly defined for MacOS X.

  Revision 1.5  2004/05/10 14:10:04  savage
  Initial MacOS X support. Fixed defines for MACOS ( Classic ) and DARWIN ( MacOS X ).

  Revision 1.4  2004/04/13 09:32:08  savage
  Changed Shared object names back to just the .so extension to avoid conflicts on various Linux/Unix distros. Therefore developers will need to create Symbolic links to the actual Share Objects if necessary.

  Revision 1.3  2004/04/01 20:53:23  savage
  Changed Linux Shared Object names so they reflect the Symbolic Links that are created when installing the RPMs from the SDL site.

  Revision 1.2  2004/03/30 20:23:28  savage
  Tidied up use of UNIX compiler directive.

  Revision 1.1  2004/02/16 22:16:40  savage
  v1.0 changes


}
{******************************************************************************}


{$I jedi-sdl.inc}

interface

uses
{$IFDEF __GPC__}
  gpc,
{$ENDIF}

{$IFDEF windows}
{$IFNDEF __GPC__}
  Windows,
{$ENDIF}
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD

{$IFDEF MORPHOS}
  exec,
{$ENDIF}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  sdl;

const
{$IFDEF windows}
  SDLNetLibName = 'SDL_net.dll';
{$ENDIF}

{$IFDEF UNIX}
{$IFDEF DARWIN}
  SDLNetLibName = 'libSDL_net.dylib';
{$ELSE}
  SDLNetLibName = 'libSDL_net.so';
{$ENDIF}
{$ENDIF}

{$IFDEF MACOS}
  SDLNetLibName = 'SDL_net';
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MACOS}
  SDLNetLibName = 'powersdl_net.library';
{$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {* Printable format: "%d.%d.%d", MAJOR, MINOR, PATCHLEVEL *}
  SDL_NET_MAJOR_VERSION = 1;
{$EXTERNALSYM SDL_NET_MAJOR_VERSION}
  SDL_NET_MINOR_VERSION = 2;
{$EXTERNALSYM SDL_NET_MINOR_VERSION}
  SDL_NET_PATCHLEVEL = 5;
{$EXTERNALSYM SDL_NET_PATCHLEVEL}

  // SDL_Net.h constants
  {* Resolve a host name and port to an IP address in network form.
   If the function succeeds, it will return 0.
   If the host couldn't be resolved, the host portion of the returned
   address will be INADDR_NONE, and the function will return -1.
   If 'host' is NULL, the resolved host will be set to INADDR_ANY.
 *}
  INADDR_ANY = $00000000;
{$EXTERNALSYM INADDR_ANY}

  INADDR_NONE = $FFFFFFFF;
{$EXTERNALSYM INADDR_NONE}

{***********************************************************************}
{* UDP network API                                                     *}
{***********************************************************************}
{* The maximum channels on a a UDP socket *}
  SDLNET_MAX_UDPCHANNELS = 32;
{$EXTERNALSYM SDLNET_MAX_UDPCHANNELS}
{* The maximum addresses bound to a single UDP socket channel *}
  SDLNET_MAX_UDPADDRESSES = 4;
{$EXTERNALSYM SDLNET_MAX_UDPADDRESSES}

type
  // SDL_net.h types
  {***********************************************************************}
  {* IPv4 hostname resolution API                                        *}
  {***********************************************************************}
  PIPAddress = ^TIPAddress;
  TIPAddress = record
    host : Uint32; // 32-bit IPv4 host address */
    port : Uint16; // 16-bit protocol port */
  end;

  {***********************************************************************}
  {* TCP network API                                                     *}
  {***********************************************************************}
  PTCPSocket = ^TTCPSocket;
  TTCPSocket = record
    ready : integer;
{$IFDEF windows}
    channel : integer;
{$ENDIF}
{$IFDEF UNIX}
    channel : integer;
{$ENDIF}
    remoteAddress : TIPaddress;
    localAddress : TIPaddress;
    sflag : integer;
  end;

  {***********************************************************************}
  {* UDP network API                                                     *}
  {***********************************************************************}
  PUDP_Channel = ^TUDP_Channel;
  TUDP_Channel = record
    numbound : integer;
    address : array[ 0..SDLNET_MAX_UDPADDRESSES - 1 ] of TIPAddress;
  end;

  PUDPSocket = ^TUDPSocket;
  TUDPSocket = record
    ready : integer;
{$IFDEF windows}
    channel : integer;
{$ENDIF}
{$IFDEF UNIX}
    channel : integer;
{$ENDIF}
    address : TIPAddress;
    binding : array[ 0..SDLNET_MAX_UDPCHANNELS - 1 ] of TUDP_Channel;
  end;

  PUDPpacket = ^TUDPpacket;
  PPUDPpacket = ^PUDPpacket;
  TUDPpacket = record
    channel : integer; {* The src/dst channel of the packet *}
    data : PUint8; {* The packet data *}
    len : integer; {* The length of the packet data *}
    maxlen : integer; {* The size of the data buffer *}
    status : integer; {* packet status after sending *}
    address : TIPAddress; {* The source/dest address of an incoming/outgoing packet *}
  end;

  {***********************************************************************}
  {* Hooks for checking sockets for available data                       *}
  {***********************************************************************}
  PSDLNet_Socket = ^TSDLNet_Socket;
  TSDLNet_Socket = record
    ready : integer;
{$IFDEF windows}
    channel : integer;
{$ENDIF}
{$IFDEF UNIX}
    channel : integer;
{$ENDIF}
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  PPSDLNet_Socket = ^PSDLNet_Socket;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  PSDLNet_SocketSet = ^TSDLNet_SocketSet;
  TSDLNet_SocketSet = record
    numsockets : integer;
    maxsockets : integer;
<<<<<<< HEAD
<<<<<<< HEAD
    sockets : PPSDLNet_Socket;
=======
    sockets : PSDLNet_Socket;
>>>>>>> graemeg/fixes_2_2
=======
    sockets : PSDLNet_Socket;
>>>>>>> origin/fixes_2_2
  end;

  {* Any network socket can be safely cast to this socket type *}
  PSDLNet_GenericSocket = ^TSDLNet_GenericSocket;
  TSDLNet_GenericSocket = record
    ready : integer;
  end;

{ This macro can be used to fill a version structure with the compile-time
  version of the SDL_net library. }
procedure SDL_NET_VERSION( var X : TSDL_version );
{$EXTERNALSYM SDL_NET_VERSION}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
{$INCLUDE powersdl_net.inc}
{$ELSE MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{* Initialize/Cleanup the network API
   SDL must be initialized before calls to functions in this library,
   because this library uses utility functions from the SDL library.
*}
function SDLNet_Init : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_Init'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_Init}

procedure SDLNet_Quit;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_Quit'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_Quit}

 {* Resolve a host name and port to an IP address in network form.
   If the function succeeds, it will return 0.
   If the host couldn't be resolved, the host portion of the returned
   address will be INADDR_NONE, and the function will return -1.
   If 'host' is NULL, the resolved host will be set to INADDR_ANY.
 *}
function SDLNet_ResolveHost( var address : TIPaddress; host : PCHar; port : Uint16 ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_ResolveHost'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_ResolveHost}

{* Resolve an ip address to a host name in canonical form.
   If the ip couldn't be resolved, this function returns NULL,
   otherwise a pointer to a static buffer containing the hostname
   is returned.  Note that this function is not thread-safe.
*}
function SDLNet_ResolveIP( var ip : TIPaddress ) : PChar;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_ResolveIP'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_ResolveIP}

  {***********************************************************************}
  {* TCP network API                                                     *}
  {***********************************************************************}

{* Open a TCP network socket
   If ip.host is INADDR_NONE, this creates a local server socket on the
   given port, otherwise a TCP connection to the remote host and port is
   attempted.  The address passed in should already be swapped to network
   byte order (addresses returned from SDLNet_ResolveHost() are already
   in the correct form).
   The newly created socket is returned, or NULL if there was an error.
*}
function SDLNet_TCP_Open( var ip : TIPaddress ) : PTCPSocket;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_TCP_Open'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_TCP_Open}

{* Accept an incoming connection on the given server socket.
   The newly created socket is returned, or NULL if there was an error.
*}
function SDLNet_TCP_Accept( server : PTCPsocket ) : PTCPSocket;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_TCP_Accept'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_TCP_Accept}

{* Get the IP address of the remote system associated with the socket.
   If the socket is a server socket, this function returns NULL.
*}
function SDLNet_TCP_GetPeerAddress( sock : PTCPsocket ) : PIPAddress;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_TCP_GetPeerAddress'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_TCP_GetPeerAddress}

{* Send 'len' bytes of 'data' over the non-server socket 'sock'
   This function returns the actual amount of data sent.  If the return value
   is less than the amount of data sent, then either the remote connection was
   closed, or an unknown socket error occurred.
*}
function SDLNet_TCP_Send( sock : PTCPsocket; data : Pointer; len : integer ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_TCP_Send'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_TCP_Send}

{* Receive up to 'maxlen' bytes of data over the non-server socket 'sock',
   and store them in the buffer pointed to by 'data'.
   This function returns the actual amount of data received.  If the return
   value is less than or equal to zero, then either the remote connection was
   closed, or an unknown socket error occurred.
*}
function SDLNet_TCP_Recv( sock : PTCPsocket; data : Pointer; maxlen : integer ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_TCP_Recv'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_TCP_Recv}

{* Close a TCP network socket *}
procedure SDLNet_TCP_Close( sock : PTCPsocket );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_TCP_Close'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_TCP_Close}


  {***********************************************************************}
  {* UDP network API                                                     *}
  {***********************************************************************}

  {* Allocate/resize/free a single UDP packet 'size' bytes long.
   The new packet is returned, or NULL if the function ran out of memory.
 *}
function SDLNet_AllocPacket( size : integer ) : PUDPpacket;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_AllocPacket'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_AllocPacket}

function SDLNet_ResizePacket( packet : PUDPpacket; newsize : integer ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_ResizePacket'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_ResizePacket}

procedure SDLNet_FreePacket( packet : PUDPpacket );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_FreePacket'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_FreePacket}

{* Allocate/Free a UDP packet vector (array of packets) of 'howmany' packets,
   each 'size' bytes long.
   A pointer to the first packet in the array is returned, or NULL if the
   function ran out of memory.
 *}
function SDLNet_AllocPacketV( howmany : integer; size : integer ) : PUDPpacket;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_AllocPacketV'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_AllocPacketV}

procedure SDLNet_FreePacketV( packetV : PUDPpacket );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_FreePacketV'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_FreePacketV}

{* Open a UDP network socket
   If 'port' is non-zero, the UDP socket is bound to a local port.
   This allows other systems to send to this socket via a known port.
*}
function SDLNet_UDP_Open( port : Uint16 ) : PUDPsocket;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_Open'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_Open}

{* Bind the address 'address' to the requested channel on the UDP socket.
   If the channel is -1, then the first unbound channel will be bound with
   the given address as it's primary address.
   If the channel is already bound, this new address will be added to the
   list of valid source addresses for packets arriving on the channel.
   If the channel is not already bound, then the address becomes the primary
   address, to which all outbound packets on the channel are sent.
   This function returns the channel which was bound, or -1 on error.
*}
function SDLNet_UDP_Bind( sock : PUDPsocket; channel : integer; var address : TIPaddress ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_Bind'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_Bind}

{* Unbind all addresses from the given channel *}
procedure SDLNet_UDP_Unbind( sock : PUDPsocket; channel : integer );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_Unbind'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_Unbind}

{* Get the primary IP address of the remote system associated with the
   socket and channel.  If the channel is -1, then the primary IP port
   of the UDP socket is returned -- this is only meaningful for sockets
   opened with a specific port.
   If the channel is not bound and not -1, this function returns NULL.
 *}
function SDLNet_UDP_GetPeerAddress( sock : PUDPsocket; channel : integer ) : PIPAddress;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_GetPeerAddress'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_GetPeerAddress}

{* Send a vector of packets to the the channels specified within the packet.
   If the channel specified in the packet is -1, the packet will be sent to
   the address in the 'src' member of the packet.
   Each packet will be updated with the status of the packet after it has
   been sent, -1 if the packet send failed.
   This function returns the number of packets sent.
*}
function SDLNet_UDP_SendV( sock : PUDPsocket; packets : PPUDPpacket; npackets : integer ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_SendV'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_SendV}

{* Send a single packet to the specified channel.
   If the channel specified in the packet is -1, the packet will be sent to
   the address in the 'src' member of the packet.
   The packet will be updated with the status of the packet after it has
   been sent.
   This function returns 1 if the packet was sent, or 0 on error.
*}
function SDLNet_UDP_Send( sock : PUDPsocket; channel : integer; packet : PUDPpacket ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_Send'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_Send}

{* Receive a vector of pending packets from the UDP socket.
   The returned packets contain the source address and the channel they arrived
   on.  If they did not arrive on a bound channel, the the channel will be set
   to -1.
   The channels are checked in highest to lowest order, so if an address is
   bound to multiple channels, the highest channel with the source address
   bound will be returned.
   This function returns the number of packets read from the network, or -1
   on error.  This function does not block, so can return 0 packets pending.
*}
function SDLNet_UDP_RecvV( sock : PUDPsocket; packets : PPUDPpacket ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_RecvV'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_RecvV}

{* Receive a single packet from the UDP socket.
   The returned packet contains the source address and the channel it arrived
   on.  If it did not arrive on a bound channel, the the channel will be set
   to -1.
   The channels are checked in highest to lowest order, so if an address is
   bound to multiple channels, the highest channel with the source address
   bound will be returned.
   This function returns the number of packets read from the network, or -1
   on error.  This function does not block, so can return 0 packets pending.
*}
function SDLNet_UDP_Recv( sock : PUDPsocket; packet : PUDPpacket ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_Recv'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_Recv}

{* Close a UDP network socket *}
procedure SDLNet_UDP_Close( sock : PUDPsocket );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_UDP_Close'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_UDP_Close}

  {***********************************************************************}
  {* Hooks for checking sockets for available data                       *}
  {***********************************************************************}

{* Allocate a socket set for use with SDLNet_CheckSockets()
   This returns a socket set for up to 'maxsockets' sockets, or NULL if
   the function ran out of memory.
 *}
function SDLNet_AllocSocketSet( maxsockets : integer ) : PSDLNet_SocketSet;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_AllocSocketSet'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_AllocSocketSet}

{* Add a socket to a set of sockets to be checked for available data *}
function SDLNet_AddSocket( set_ : PSDLNet_SocketSet; sock : PSDLNet_GenericSocket ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_AddSocket'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_AddSocket}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}
function SDLNet_TCP_AddSocket( set_ : PSDLNet_SocketSet; sock : PTCPSocket ) : integer;

function SDLNet_UDP_AddSocket( set_ : PSDLNet_SocketSet; sock : PUDPSocket ) : integer;
{$IFNDEF MORPHOS}
=======
=======
>>>>>>> origin/fixes_2_2
function SDLNet_TCP_AddSocket( set_ : PSDLNet_SocketSet; sock : PTCPSocket ) : integer;

function SDLNet_UDP_AddSocket( set_ : PSDLNet_SocketSet; sock : PUDPSocket ) : integer;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{* Remove a socket from a set of sockets to be checked for available data *}
function SDLNet_DelSocket( set_ : PSDLNet_SocketSet; sock : PSDLNet_GenericSocket ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_DelSocket'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_DelSocket}

function SDLNet_TCP_DelSocket( set_ : PSDLNet_SocketSet; sock : PTCPSocket ) : integer; // SDLNet_DelSocket(set, (SDLNet_GenericSocket)sock)

function SDLNet_UDP_DelSocket( set_ : PSDLNet_SocketSet; sock : PUDPSocket ) : integer; //SDLNet_DelSocket(set, (SDLNet_GenericSocket)sock)

{* This function checks to see if data is available for reading on the
   given set of sockets.  If 'timeout' is 0, it performs a quick poll,
   otherwise the function returns when either data is available for
   reading, or the timeout in milliseconds has elapsed, which ever occurs
   first.  This function returns the number of sockets ready for reading,
   or -1 if there was an error with the select() system call.
*}
function SDLNet_CheckSockets( set_ : PSDLNet_SocketSet; timeout : Sint32 ) : integer;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_CheckSockets'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_CheckSockets}

{* After calling SDLNet_CheckSockets(), you can use this function on a
   socket that was in the socket set, to find out if data is available
   for reading.
*}
<<<<<<< HEAD
<<<<<<< HEAD

{$ENDIF MORPHOS}
function SDLNet_SocketReady( sock : PSDLNet_GenericSocket ) : boolean;
{$IFNDEF MORPHOS}
=======
function SDLNet_SocketReady( sock : PSDLNet_GenericSocket ) : boolean;
>>>>>>> graemeg/fixes_2_2
=======
function SDLNet_SocketReady( sock : PSDLNet_GenericSocket ) : boolean;
>>>>>>> origin/fixes_2_2

{* Free a set of sockets allocated by SDL_NetAllocSocketSet() *}
procedure SDLNet_FreeSocketSet( set_ : PSDLNet_SocketSet );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_FreeSocketSet'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_FreeSocketSet}

{***********************************************************************}
{* Platform-independent data conversion functions                      *}
{***********************************************************************}

{* Write a 16/32 bit value to network packet buffer *}
procedure SDLNet_Write16( value : Uint16; area : Pointer );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_Write16'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_Write16}

procedure SDLNet_Write32( value : Uint32; area : Pointer );
cdecl; external{$IFDEF __GPC__}name 'SDLNet_Write32'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_Write32}

{* Read a 16/32 bit value from network packet buffer *}
function SDLNet_Read16( area : Pointer ) : Uint16;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_Read16'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_Read16}

function SDLNet_Read32( area : Pointer ) : Uint32;
cdecl; external{$IFDEF __GPC__}name 'SDLNet_Read32'{$ELSE}SDLNetLibName{$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_Read32}

{***********************************************************************}
{* Error reporting functions                                           *}
{***********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{* We'll use SDL's functions for error reporting *}
procedure SDLNet_SetError( fmt : PChar );
function SDLNet_GetError : PChar;

(* I'm eventually going to try to disentangle SDL_net from SDL, thus making
   SDL_net an independent X-platform networking toolkit.  Not today though....

proceudre SDLNet_SetError(const char *fmt, ...);
cdecl; external {$IFDEF __GPC__}name 'SDLNet_SetError'{$ELSE} SDLNetLibName {$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_SetError}

function SDLNet_GetError : PChar;
cdecl; external {$IFDEF __GPC__}name 'SDLNet_GetError'{$ELSE} SDLNetLibName {$ENDIF __GPC__};
{$EXTERNALSYM SDLNet_GetError}
*)

implementation

{$IFDEF __GPC__}
{$L 'sdl_net'} { link sdl_net.dll.a or libsdl_net.so or libsdl_net.a }
{$ENDIF}

procedure SDL_NET_VERSION( var X : TSDL_version );
begin
  X.major := SDL_NET_MAJOR_VERSION;
  X.minor := SDL_NET_MINOR_VERSION;
  X.patch := SDL_NET_PATCHLEVEL;
end;

function SDLNet_TCP_AddSocket( set_ : PSDLNet_SocketSet; sock : PTCPSocket ) : integer;
begin
  result := SDLNet_AddSocket( set_, PSDLNet_GenericSocket( sock ) );
end;

function SDLNet_UDP_AddSocket( set_ : PSDLNet_SocketSet; sock : PUDPSocket ) : integer;
begin
  result := SDLNet_AddSocket( set_, PSDLNet_GenericSocket( sock ) );
end;

function SDLNet_TCP_DelSocket( set_ : PSDLNet_SocketSet; sock : PTCPSocket ) : integer;
begin
  result := SDLNet_DelSocket( set_, PSDLNet_GenericSocket( sock ) );
end;

function SDLNet_UDP_DelSocket( set_ : PSDLNet_SocketSet; sock : PUDPSocket ) : integer;
begin
  result := SDLNet_DelSocket( set_, PSDLNet_GenericSocket( sock ) );
end;

{* After calling SDLNet_CheckSockets(), you can use this function on a
   socket that was in the socket set, to find out if data is available
   for reading.
*}

function SDLNet_SocketReady( sock : PSDLNet_GenericSocket ) : boolean;
begin
  result := ( ( sock <> nil ) and ( sock^.ready = 1 ) );
end;

procedure SDLNet_SetError( fmt : PChar );
begin
  SDL_SetError( fmt );
end;

function SDLNet_GetError : PChar;
begin
  result := SDL_GetError;
end;

end.

