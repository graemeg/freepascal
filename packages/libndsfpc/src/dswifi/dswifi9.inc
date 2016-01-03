(*
  $Id: dswifi9.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  DSWifi Project - Arm9 Library Header File (dswifi9.h)
  (C) 2005-2006 Stephen Stair - sgstair@akkit.org - http://www.akkit.org
  ****************************************************************************** 
  DSWifi Lib and test materials are licenced under the MIT open source licence:
  Copyright (c) 2005-2006 Stephen Stair
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do
  so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  ------------------------------------------------------------------------------

    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  Comments:

*)


{$include dswifi_version.inc}

{$include sys/socket.inc}
{$include netinet/in.inc}
{$include netdb.inc}

{$ifdef NDS_INTERFACE}

const
  WIFIINIT_OPTION_USELED          = $0002;

  WIFIINIT_OPTION_USEHEAP_128     = $0000;
  WIFIINIT_OPTION_USEHEAP_64      = $1000;
  WIFIINIT_OPTION_USEHEAP_256     = $2000;
  WIFIINIT_OPTION_USEHEAP_512     = $3000;
  WIFIINIT_OPTION_USECUSTOMALLOC  = $4000;
  WIFIINIT_OPTION_HEAPMASK        = $F000;

  WFLAG_PACKET_DATA               = $0001;
  WFLAG_PACKET_MGT                = $0002;
  WFLAG_PACKET_BEACON             = $0004;
  WFLAG_PACKET_CTRL               = $0008;

  WFLAG_PACKET_ALL                = $FFFF;


  WFLAG_APDATA_ADHOC              = $0001;
  WFLAG_APDATA_WEP                = $0002;
  WFLAG_APDATA_WPA                = $0004;
  WFLAG_APDATA_COMPATIBLE         = $0008;
  WFLAG_APDATA_EXTCOMPATIBLE      = $0010;
  WFLAG_APDATA_SHORTPREAMBLE      = $0020;
  WFLAG_APDATA_ACTIVE             = $8000;

type
  TWIFI_RETURN = (
    WIFI_RETURN_OK          =	0, 
    WIFI_RETURN_LOCKFAILED  = 1, 
    WIFI_RETURN_ERROR       =	2, 
    WIFI_RETURN_PARAMERROR  = 3  
  );

  TWIFI_STATS = (
    WSTAT_RXQUEUEDPACKETS, 
    WSTAT_TXQUEUEDPACKETS, 
    WSTAT_RXQUEUEDBYTES, 
    WSTAT_TXQUEUEDBYTES, 
    WSTAT_RXQUEUEDLOST, 
    WSTAT_TXQUEUEDREJECTED, 
    WSTAT_RXPACKETS,
    WSTAT_RXBYTES,
    WSTAT_RXDATABYTES,
    WSTAT_TXPACKETS,
    WSTAT_TXBYTES,
    WSTAT_TXDATABYTES,

    WSTAT_HW_1B0,WSTAT_HW_1B1,WSTAT_HW_1B2,WSTAT_HW_1B3,WSTAT_HW_1B4,WSTAT_HW_1B5,WSTAT_HW_1B6,WSTAT_HW_1B7,	
    WSTAT_HW_1B8,WSTAT_HW_1B9,WSTAT_HW_1BA,WSTAT_HW_1BB,WSTAT_HW_1BC,WSTAT_HW_1BD,WSTAT_HW_1BE,WSTAT_HW_1BF,	
    WSTAT_HW_1C0,WSTAT_HW_1C1,WSTAT_HW_1C4,WSTAT_HW_1C5,
    WSTAT_HW_1D0,WSTAT_HW_1D1,WSTAT_HW_1D2,WSTAT_HW_1D3,WSTAT_HW_1D4,WSTAT_HW_1D5,WSTAT_HW_1D6,WSTAT_HW_1D7,	
    WSTAT_HW_1D8,WSTAT_HW_1D9,WSTAT_HW_1DA,WSTAT_HW_1DB,WSTAT_HW_1DC,WSTAT_HW_1DD,WSTAT_HW_1DE,WSTAT_HW_1DF,	
    
    NUM_WIFI_STATS
  );

  TWIFI_MODE = (
    WIFIMODE_DISABLED,
    WIFIMODE_NORMAL,
    WIFIMODE_SCAN,
    WIFIMODE_ASSOCIATE,
    WIFIMODE_ASSOCIATED,
    WIFIMODE_DISASSOCIATE,
    WIFIMODE_CANNOTASSOCIATE
  );

  TWIFI_AUTHLEVEL = (
    WIFI_AUTHLEVEL_DISCONNECTED,
    WIFI_AUTHLEVEL_AUTHENTICATED,
    WIFI_AUTHLEVEL_ASSOCIATED,
    WIFI_AUTHLEVEL_DEASSOCIATED
  );

  TWIFIGETDATA = (
    WIFIGETDATA_MACADDRESS,			
    WIFIGETDATA_NUMWFCAPS,			
    MAX_WIFIGETDATA
  );


  TWEPMODES = (
    WEPMODE_NONE    = 0,
    WEPMODE_40BIT   = 1,
    WEPMODE_128BIT  = 2
  );
  
  TWIFI_ASSOCSTATUS = (
    ASSOCSTATUS_DISCONNECTED, 
    ASSOCSTATUS_SEARCHING, 
    ASSOCSTATUS_AUTHENTICATING, 
    ASSOCSTATUS_ASSOCIATING, 
    ASSOCSTATUS_ACQUIRINGDHCP, 
    ASSOCSTATUS_ASSOCIATED,	
    ASSOCSTATUS_CANNOTCONNECT 
  );

  WIFI_TXHEADER =  record
    enable_flags: cuint16;
    unknown: cuint16;
    countup: cuint16;
    beaconfreq: cuint16;
    tx_rate: cuint16;
    tx_length: cuint16;
  end;
  TWIFI_TXHEADER = WIFI_TXHEADER;
	PWIFI_TXHEADER = ^WIFI_TXHEADER;

  WIFI_RXHEADER =  record
    a: cuint16;
    b: cuint16;
    c: cuint16;
    d: cuint16;
    byteLength: cuint16;
    rssi_: cuint16;
  end;
  TWIFI_RXHEADER = WIFI_RXHEADER;
  PWIFI_RXHEADER = ^WIFI_RXHEADER;

  WIFI_ACCESSPOINT = packed record
    ssid: array [0..32] of cchar; 
    ssid_len: cchar; 
    bssid: array [0..5] of cuint8; 
    macaddr: array [0..5] of cuint8; 
    maxrate: cuint16; 
    timectr: cuint32; 
    rssi: cuint16; 
    flags: cuint16; 
    spinlock: cuint32; 
    channel: cuint8; 
    rssi_past: array [0..7] of cuint8; 
    base_rates: array [0..15] of cuint8; 
  end;
  TWIFI_ACCESSPOINT = WIFI_ACCESSPOINT;
  PWIFI_ACCESSPOINT = ^WIFI_ACCESSPOINT;

  WifiPacketHandler = procedure(packetID, packetlength: cint);

  WifiSyncHandler = procedure;


function Wifi_Init(initflags: cint): culong; cdecl; external;
function Wifi_CheckInit(): cint; cdecl; external;
procedure Wifi_DisableWifi(); cdecl; external;
procedure Wifi_EnableWifi(); cdecl; external;
procedure Wifi_SetPromiscuousMode(enable: cint); cdecl; external;
procedure Wifi_ScanMode(); cdecl; external;
procedure Wifi_SetChannel(channel: cint); cdecl; external;
function Wifi_GetNumAP(): cint; cdecl; external;
function Wifi_GetAPData(apnum: cint; apdata: PWifi_AccessPoint): cint; cdecl; external;
function Wifi_FindMatchingAP(numaps: cint; apdata: PWifi_AccessPoint; match_dest: PWifi_AccessPoint): cint; cdecl; external;
function Wifi_ConnectAP(apdata: PWifi_AccessPoint; wepmode, wepkeyid: cint; wepkey: pcuchar): cint; cdecl; external;
procedure Wifi_AutoConnect(); cdecl; external;
function Wifi_AssocStatus(): cint; cdecl; external;
function Wifi_DisconnectAP(): cint; cdecl; external;
procedure Wifi_Timer(num_ms: cint); cdecl; external;
function Wifi_GetIP(): cuint32; cdecl; external;
function Wifi_GetIPInfo(pGateway, pSnmask, pDns1, pDns2: pcuint32): cuint32; cdecl; external;
procedure Wifi_SetIP(IPaddr, gateway, subnetmask, dns1, dns2: u32); cdecl; external;
function Wifi_GetData(datatype, bufferlen: cint; buffer: pcuchar): cint; cdecl; external;
function Wifi_GetStats(statnum: cint): cuint32; cdecl; external;
function Wifi_RawTxFrame(datalen, rate: cuint16; data: pcuint16): cint; cdecl; external;
procedure Wifi_RawSetPacketHandler(wphfunc: WifiPacketHandler); cdecl; external;
function Wifi_RxRawReadPacket(packetID, readlength: cint32; data: pcuint16): cint; cdecl; external;
procedure Wifi_Update(); cdecl; external;
procedure Wifi_Sync(); cdecl; external;
procedure Wifi_SetSyncHandler(sh: WifiSyncHandler); cdecl; external;
//procedure Wifi_SetSyncHandler(sh: pointer); cdecl; external;

{$endif NDS_INTERFACE}
