{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ********************************************************************** }

//
// Module Name:
//
//     connmgr.h
//
// DESCRIPTION from MSDN:
// The Connection Manager API is used to centralize and automate the establishment
// and management of the network connections on a Windows Mobile-based device.
// Mobile device applications use Connection Manager to establish or schedule a
// network connection, and Connection Manager handles the details of the connection.
// The application simply informs Connection Manager of the network type to use
// for the connection (the Internet, for example).
// When an application requests a network connection, Connection Manager first
// retrieves all of the possible connections from a set of Connection Service
// Providers (CSPs). Connection Manager then associates a set of costs with these
// routes and ultimately determines the optimal connection based on cost, latency,
// bandwidth, and other factors. Finally, Connection Manager queues the requested
// connection and uses the CSPs to establish the connection at the appropriate
// time.
//

//
//  Microsoft Windows Mobile 6.0 for PocketPC SDK.
//

unit connmgr;

{$CALLING cdecl}

interface

uses Windows, WinSock2;

const
      CellcoreDLL = 'cellcore.dll';

// - connmgr_conntypes.h

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Connection types | Describes possible connection types and subtypes
//
// @comm none
//
// -----------------------------------------------------------------------------
const
      CM_CONNTYPE_UNKNOWN     = 0;
      CM_CONNTYPE_CELLULAR    = 1;
      CM_CONNTYPE_NIC         = 2;
      CM_CONNTYPE_BLUETOOTH   = 3;
      CM_CONNTYPE_UNIMODEM    = 4;
      CM_CONNTYPE_VPN         = 5;
      CM_CONNTYPE_PROXY       = 6;
      CM_CONNTYPE_PC          = 7;
      CM_CONNTYPE_MAX         = 8;

      CM_CONNSUBTYPE_UNKNOWN  = 0;

      CM_CONNSUBTYPE_CELLULAR_UNKNOWN = 0;
      CM_CONNSUBTYPE_CELLULAR_CSD     = 1;
      CM_CONNSUBTYPE_CELLULAR_GPRS    = 2;
      CM_CONNSUBTYPE_CELLULAR_1XRTT   = 3;   // @constdefine Not distinct from CSD.
      CM_CONNSUBTYPE_CELLULAR_1XEVDO  = 4;   // @constdefine Not distinct from CSD.
      CM_CONNSUBTYPE_CELLULAR_1XEVDV  = 5;   // @constdefine Not distinct from CSD.
      CM_CONNSUBTYPE_CELLULAR_EDGE    = 6;   // @constdefine Not distinct from GPRS.
      CM_CONNSUBTYPE_CELLULAR_UMTS    = 7;   // @constdefine Not distinct from GPRS.
      CM_CONNSUBTYPE_CELLULAR_VOICE   = 8;
      CM_CONNSUBTYPE_CELLULAR_PTT     = 9;   // @constdefine Push-to-Talk, not supported.
      CM_CONNSUBTYPE_CELLULAR_HSDPA   = 10;  // @constdefine High-Speed Downlink Packet Access (3.5G).
      CM_CONNSUBTYPE_CELLULAR_MAX     = 11;

      CM_CONNSUBTYPE_NIC_UNKNOWN  = 0;
      CM_CONNSUBTYPE_NIC_ETHERNET = 1;
      CM_CONNSUBTYPE_NIC_WIFI     = 2;
      CM_CONNSUBTYPE_NIC_MAX      = 3;

      CM_CONNSUBTYPE_BLUETOOTH_UNKNOWN    = 0;
      CM_CONNSUBTYPE_BLUETOOTH_RAS        = 1;
      CM_CONNSUBTYPE_BLUETOOTH_PAN        = 2;
      CM_CONNSUBTYPE_BLUETOOTH_MAX        = 3;

      CM_CONNSUBTYPE_UNIMODEM_UNKNOWN         = 0;
      CM_CONNSUBTYPE_UNIMODEM_CSD             = 1;
      CM_CONNSUBTYPE_UNIMODEM_OOB_CSD         = 2;
//
// Derived from unimodem device types
//
const
      CM_CONNSUBTYPE_UNIMODEM_NULL_MODEM      = 3;   // Direct Cable Connect (DCC)
      CM_CONNSUBTYPE_UNIMODEM_EXTERNAL_MODEM  = 4;   // Serial port attached modem
      CM_CONNSUBTYPE_UNIMODEM_INTERNAL_MODEM  = 5;
      CM_CONNSUBTYPE_UNIMODEM_PCMCIA_MODEM    = 6;
      CM_CONNSUBTYPE_UNIMODEM_IRCOMM_MODEM    = 7;   // DCC over Irda
      CM_CONNSUBTYPE_UNIMODEM_DYNAMIC_MODEM   = 8;   // Bluetooth modem
      CM_CONNSUBTYPE_UNIMODEM_DYNAMIC_PORT    = 9;   // DCC over Bluetooth
      CM_CONNSUBTYPE_UNIMODEM_MAX             = 10;


      CM_CONNSUBTYPE_VPN_UNKNOWN  = 0;
      CM_CONNSUBTYPE_VPN_L2TP     = 1;
      CM_CONNSUBTYPE_VPN_PPTP     = 2;
      CM_CONNSUBTYPE_VPN_MAX      = 3;

      CM_CONNSUBTYPE_PROXY_UNKNOWN    = 0;
      CM_CONNSUBTYPE_PROXY_NULL       = 1;
      CM_CONNSUBTYPE_PROXY_HTTP       = 2;
      CM_CONNSUBTYPE_PROXY_WAP        = 3;
      CM_CONNSUBTYPE_PROXY_SOCKS4     = 4;
      CM_CONNSUBTYPE_PROXY_SOCKS5     = 5;
      CM_CONNSUBTYPE_PROXY_MAX        = 6;

      CM_CONNSUBTYPE_PC_UNKNOWN            = 0;
      CM_CONNSUBTYPE_PC_DESKTOPPASSTHROUGH = 1;
      CM_CONNSUBTYPE_PC_IR                 = 2;
      CM_CONNSUBTYPE_PC_MODEM_LINK         = 3;
      CM_CONNSUBTYPE_PC_MAX                = 4;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Connection options | Describes possible connection options
//
// @comm none
//
// -----------------------------------------------------------------------------
const
      CM_DSF_BILLBYTIME       = $00000001;  // @constdefine Connection is billed by time.
      CM_DSF_ALWAYSON         = $00000002;  // @constdefine Connection is always on.
      CM_DSF_SUSPENDRESUME    = $00000004;  // @constdefine Connection is suspend/resume capable.

// - end of connmgr_conntypes.h


// - connmgr_status.h

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct CONNMGR_CONNECTION_IPADDR | Address information for an active connection
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     _CONNMGR_CONNECTION_IPADDR = record
       cIPAddr:DWORD;                  // @field Count of addresses in array.
       IPAddr:array[0..0] of SOCKADDR_STORAGE;     // @field Array of IPv4 and/or IPv6 addresses.
     end;
     CONNMGR_CONNECTION_IPADDR = _CONNMGR_CONNECTION_IPADDR;
     LPCONNMGR_CONNECTION_IPADDR = ^_CONNMGR_CONNECTION_IPADDR;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct CONNMGR_CONNECTION_DETAILED_STATUS | Information about available connections
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      CONNMGRDETAILEDSTATUS_VERSION               = 1;

      CONNMGRDETAILEDSTATUS_PARAM_TYPE            = $00000001;
      CONNMGRDETAILEDSTATUS_PARAM_SUBTYPE         = $00000002;
      CONNMGRDETAILEDSTATUS_PARAM_DESTNET         = $00000004;
      CONNMGRDETAILEDSTATUS_PARAM_SOURCENET       = $00000008;
      CONNMGRDETAILEDSTATUS_PARAM_FLAGS           = $00000010;
      CONNMGRDETAILEDSTATUS_PARAM_SECURE          = $00000020;
      CONNMGRDETAILEDSTATUS_PARAM_DESCRIPTION     = $00000040;
      CONNMGRDETAILEDSTATUS_PARAM_ADAPTERNAME     = $00000080;
      CONNMGRDETAILEDSTATUS_PARAM_CONNSTATUS      = $00000100;
      CONNMGRDETAILEDSTATUS_PARAM_LASTCONNECT     = $00000200;
      CONNMGRDETAILEDSTATUS_PARAM_SIGNALQUALITY   = $00000400;
      CONNMGRDETAILEDSTATUS_PARAM_IPADDR          = $00000800;

type
     LPCONNMGR_CONNECTION_DETAILED_STATUS = ^_CONNMGR_CONNECTION_DETAILED_STATUS;
     _CONNMGR_CONNECTION_DETAILED_STATUS = record
       pNext:LPCONNMGR_CONNECTION_DETAILED_STATUS;

       dwVer:DWORD;                // @field Structure version; current is CONNMGRDETAILEDSTATUS_VERSION.
       dwParams:DWORD;             // @field Combination of CONNMGRDETAILEDSTATUS_PARAM_* values.

       dwType:DWORD;               // @field One of CM_CONNTYPE_* values.
       dwSubtype:DWORD;            // @field One of CM_CONNSUBTYPE_* values.

       dwFlags:DWORD;              // @field Combination of CM_DSF_* flags.
       dwSecure:DWORD;             // @field Secure level (0 == not-secure) of connection.

       guidDestNet:GUID;           // @field GUID of destination network.
       guidSourceNet:GUID;         // @field GUID of source network.

       szDescription:PTCHAR;       // @field Name of connection, 0-terminated string or NULL if N/A.
<<<<<<< HEAD
<<<<<<< HEAD
       szAdapterName:PTCHAR;       // @field Name of adapter, 0-terminated or NULL if N/A.
=======
       szAdapterName:GUID;       // @field Name of adapter, 0-terminated or NULL if N/A.
>>>>>>> graemeg/fixes_2_2
=======
       szAdapterName:GUID;       // @field Name of adapter, 0-terminated or NULL if N/A.
>>>>>>> origin/fixes_2_2

       dwConnectionStatus:DWORD;   // @field One of CONNMGR_STATUS_*.
       LastConnectTime:SYSTEMTIME; // @field Time connection was last established.
       dwSignalQuality:DWORD;      // @field Signal quality normalized in the range 0-255.

       pIPAddr:LPCONNMGR_CONNECTION_IPADDR; // @field Available IP addrs, or NULL if N/A.
     end;
     CONNMGR_CONNECTION_DETAILED_STATUS = _CONNMGR_CONNECTION_DETAILED_STATUS;
// End of version 1 fields.


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func ConnMgrQueryDetailedStatus | Returns detailed information about all existing connections.
//
// @comm Standard semantics here, call once to find out buffer size, then call
// again to get array of CONNMGR_CONNECTION_DETAILED_STATUS structures.
// Note: since connections can change often, make sure to allow for possibility
// that between first and second call, the necessary amount of space needed for
// the return structure changes.
//
// -----------------------------------------------------------------------------

function ConnMgrQueryDetailedStatus(pStatusBuffer:LPCONNMGR_CONNECTION_DETAILED_STATUS; // @parm Pointer to buffer to be filled status info.
                                    pcbBufferSize:LPDWORD                               // @parm IN: Size of buffer.  OUT: size of buffer needed.
                                   ):HRESULT; external CellcoreDLL name 'ConnMgrQueryDetailedStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func ConnMgrRegisterForStatusNotification | Registers a window handle for connection status notifications.
//
// @comm Will post a notification when,
// 1) an inactive connection is created or deleted, ex. via user input.
// 2) any of of the active connections changes connection state, i.e.
//    active to inactive, or inactive to active.
// Note: Notifications will not be generated for other changes, ex. change in
// signal quality.
//
// -----------------------------------------------------------------------------

const
      CONNMGR_STATUS_CHANGE_NOTIFICATION_MSG = 'CONNMGR_STATUS_CHANGE_NOTIFICATION_MSG';

function ConnMgrRegisterForStatusChangeNotification(fEnable:BOOL;   // @parm Turn on or turn off notifications
                                                    _hWnd:HWND      // @parm Window to post status change notifications to
                                                   ):HRESULT; external CellcoreDLL name 'ConnMgrRegisterForStatusChangeNotification';

// - end of connmgr_status.h


// - connmgr_proxy.h

// Proxy CSP config/settings
const
      IID_ConnPrv_IProxyExtension:TIID = '{AF96B0BD-A481-482C-A094-A8448767A0C0}';

const
      CMPROXY_PROXYSERVER_MAXSIZE   = 256;
      CMPROXY_USERNAME_MAXSIZE      = 32;
      CMPROXY_PASSWORD_MAXSIZE      = 32;
      CMPROXY_EXTRAINFO_MAXSIZE     = 256;
      CMPROXY_PROXYOVERRIDE_MAXSIZE = 64;

type
     _PROXY_CONFIG = record
       dwType:DWORD;
       dwEnable:DWORD;
       szProxyServer:array[0..CMPROXY_PROXYSERVER_MAXSIZE-1] of TCHAR;
       szUsername:array[0..CMPROXY_USERNAME_MAXSIZE-1] of TCHAR;
       szPassword:array[0..CMPROXY_PASSWORD_MAXSIZE-1] of TCHAR;
       szProxyOverride:array[0..CMPROXY_PROXYOVERRIDE_MAXSIZE-1] of TCHAR;
       szExtraInfo:array[0..CMPROXY_EXTRAINFO_MAXSIZE-1] of TCHAR;
     end;
     PROXY_CONFIG = _PROXY_CONFIG;
     LPPROXY_CONFIG = ^_PROXY_CONFIG;

// - end of connmgr_proxy.h

// - connmgr.h

const
      IID_DestNetInternet:TIID = '{436EF144-B4FB-4863-A041-8F905A62C572}';

      IID_DestNetCorp:TIID = '{A1182988-0D73-439e-87AD-2A5B369F808B}';

      IID_DestNetWAP = '{7022E968-5A97-4051-BC1C-C578E2FBA5D9}';

      IID_DestNetSecureWAP:TIID = '{F28D1F74-72BE-4394-A4A7-4E296219390C}';

//
// Connection Manager
//

//
// Client API
//

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Valid Params | Defines valid parameters in CONNMGR_CONNECTIONINFO
//
// @comm The following flags in the dwParams field of the CONNMGR_CONNECTIONINFO structure
// define which optional parameters are valid.
//
// -----------------------------------------------------------------------------
const
      CONNMGR_PARAM_GUIDDESTNET       = $01; // @constdefine guidDestNet field is valid
      CONNMGR_PARAM_MAXCOST           = $02; // @constdefine MaxCost field is valid
      CONNMGR_PARAM_MINRCVBW          = $04; // @constdefine MinRcvBw field is valid
      CONNMGR_PARAM_MAXCONNLATENCY    = $08; // @constdefine MaxConnLatency field is valid

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Proxy flags | Defines which proxies the application supports
//
// @comm The following flags in the dwFlags field of the CONNMGR_CONNECTIONINFO structure
// define special connection properties supported by client apps, typically which proxies are
// supported by each app. For example, each CONNMGR_FLAG_PROXY_* specifies that this connection
// can be specified by configuring an appropriate proxy server.  e.g. HTTP this is appropriate
// for Internet Explorer and other HTTP based protocols, including HTTP-DAV synchronization.
// If none of these flags are specified, then only a direct IP connection (or tunnel) will be attempted.
//
// -----------------------------------------------------------------------------
const
      CONNMGR_FLAG_PROXY_HTTP   = $01; // @constdefine HTTP Proxy supported
      CONNMGR_FLAG_PROXY_WAP    = $02; // @constdefine WAP Proxy (gateway) supported
      CONNMGR_FLAG_PROXY_SOCKS4 = $04; // @constdefine SOCKS4 Proxy supported
      CONNMGR_FLAG_PROXY_SOCKS5 = $08; // @constdefine SOCKS5 Proxy supported

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Control flags | Defines connection properties
//
// @comm The following flags in the dwFlags field of the CONNMGR_CONNECTIONINFO structure
// define special connection properties supported by client apps,
//
// -----------------------------------------------------------------------------
const
      CONNMGR_FLAG_SUSPEND_AWARE    = $10; // @constdefine suspended connections supported
      CONNMGR_FLAG_REGISTERED_HOME  = $20; // @constdefine only dial out if we're registered on the home network
      CONNMGR_FLAG_NO_ERROR_MSGS    = $40; // @constdefine don't show any error messages for failed connections
      CONNMGR_FLAG_WAKE_ON_INCOMING = $80; // @constdefine to satisfy request use only those interfaces that can wake the system on incoming traffic

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Priority | Defines which priority of the applications connections
//
// @comm The following flags in the dwPriority field of the CONNMGR_CONNECTIONINFO structure
// define the priority of the connection. Connection manager arbitrates among multiple connections,
// satisfying as many as possible at a time but giving preference to higher priorities
//
// -----------------------------------------------------------------------------
const
      CONNMGR_PRIORITY_VOICE                = $00020000;
// @constdefine Voice, highest priority, reserved for internal use only.

      CONNMGR_PRIORITY_USERINTERACTIVE      = $00008000;
// @constdefine User initiated action caused this request, and UI is
// currently pending on the creation of this connection.
// This is appropriate for an interactive browsing session,
// or if the user selects "MORE" at the bottom of a truncated
// mail message, etc.


      CONNMGR_PRIORITY_USERBACKGROUND       = $00002000;
// @constdefine User initiated connection which has recently become idle.
// A connection should be marked as idle when it is no
// longer the user's current task.

      CONNMGR_PRIORITY_USERIDLE             = $00000800;
// @constdefine Interactive user task which has been idle for an application
// specified time.  The application should toggle the state
// between CONNMGR_PRIORITY_USERIDLE and CONNMGR_PRIORITY_USERINTERACTIVE as the user
// uses the application.  This helps ConnectionManager
// optimize responsiveness to the interactive application,
// while sharing the connection with background applications.

      CONNMGR_PRIORITY_HIPRIBKGND         = $00000200;
// @constdefine High priority background connection

      CONNMGR_PRIORITY_IDLEBKGND          = $00000080;
// @constdefine Idle priority background connection

      CONNMGR_PRIORITY_EXTERNALINTERACTIVE = $00000020;
// @constdefine Connection is requested on behalf of an external entity, but
// is an interactive session (e.g. AT Command Iterpreter)

      CONNMGR_PRIORITY_LOWBKGND            = $00000008;
// @constdefine Lowest priority. Only connects if another higher priority client is already using the same path.

      CONNMGR_PRIORITY_CACHED              = $00000002;
// @constdefine Cached connection, reserved for internal use only.

      CONNMGR_PRIORITY_ALWAYS_ON           = $00000001;
// @constdefine Always on  connection, reserved for internal use only.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct CONNMGR_CONNECTIONINFO | Information about connection request
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     _CONNMGR_CONNECTIONINFO = record
       cbSize:DWORD;       // @field Size of this structure
       dwParams:DWORD;     // @field Valid parms, set of CONNMGR_PARAM_*
       dwFlags:DWORD;      // @field Connection flags, set of CONNMGR_FLAG_*
       dwPriority:DWORD;   // @field Priority, one of CONNMGR_PRIORITY_*
       bExclusive:BOOL;    // @field Connection is exclusive, see comments
       bDisabled:BOOL;     // @field Don't actually connect
       guidDestNet:GUID;   // @field GUID of network to connect to
       _hWnd:HWND;          // @field hWnd to post status change messages to
       uMsg:UINT;          // @field Msg to use when posting status changes
       lParam:LPARAM;      // @field lParam to use when posting status changes
       ulMaxCost:ULONG;    // @field Max acceptable cost of connection
       ulMinRcvBw:ULONG;   // @field Min acceptable receive bandwidth of connection
       ulMaxConnLatency:ULONG; // @field Max acceptable connect latency
     end;
     CONNMGR_CONNECTIONINFO = _CONNMGR_CONNECTIONINFO;
     LPCONNMGR_CONNECTIONINFO = ^_CONNMGR_CONNECTIONINFO;

// @comm bExclusive: If false, the connection is shared among all applications, and other
// applications with an interest in a connection to this network will be notified that
// the connection is available.  If true, then this connection can not be shared with other
// applications, and no other applications will be notified, and any application requesting
// a connection to the same network will be treated as a contender for
// the same resource, and not permitted to share the existing connection.  A decision will be made
// between this connection and the others based on connection priority.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Creates a connection request.
//
// @comm Return Value:  S_OK if success, error code otherwise
//
// -----------------------------------------------------------------------------
function ConnMgrEstablishConnection(pConnInfo:LPCONNMGR_CONNECTIONINFO;  // @parm Params describing requested connection
                                    phConnection:LPHANDLE                // @parm Returned connection handle
                                   ):HRESULT; external CellcoreDLL name 'ConnMgrEstablishConnection';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Creates a connection request.
//
// @comm Return Value:  Same as ConnMgrEstablishConnection, but doesn't return
// until connection has either been established or failed.
//
// -----------------------------------------------------------------------------
function ConnMgrEstablishConnectionSync(pConnInfo:LPCONNMGR_CONNECTIONINFO;  // @parm Params describing requested connection
                                        phConnection:LPHANDLE;               // @parm Returned connection handle
                                        dwTimeout:DWORD;                    // @parm Timeout
                                        pdwStatus:LPDWORD                    // @parm Final status value, one of CONNMGR_STATUS_*
                                       ):HRESULT; external CellcoreDLL name 'ConnMgrEstablishConnectionSync';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Status values | Describes the current status of the connection
//
// @comm none
//
// -----------------------------------------------------------------------------
const
      CONNMGR_STATUS_UNKNOWN                    = $00;  // @constdefine Unknown status

      CONNMGR_STATUS_CONNECTED                  = $10;  // @constdefine Connection is up
      CONNMGR_STATUS_SUSPENDED                  = $11;  // @constdefine Connection is up but suspended

      CONNMGR_STATUS_DISCONNECTED               = $20;  // @constdefine Connection is disconnected
      CONNMGR_STATUS_CONNECTIONFAILED           = $21;  // @constdefine Connection failed and cannot not be reestablished
      CONNMGR_STATUS_CONNECTIONCANCELED         = $22;  // @constdefine User aborted connection
      CONNMGR_STATUS_CONNECTIONDISABLED         = $23;  // @constdefine Connection is ready to connect but disabled
      CONNMGR_STATUS_NOPATHTODESTINATION        = $24;  // @constdefine No path could be found to destination
      CONNMGR_STATUS_WAITINGFORPATH             = $25;  // @constdefine Waiting for a path to the destination
      CONNMGR_STATUS_WAITINGFORPHONE            = $26;  // @constdefine Voice call is in progress
      CONNMGR_STATUS_PHONEOFF                   = $27;  // @constdefine Phone resource needed and phone is off
      CONNMGR_STATUS_EXCLUSIVECONFLICT          = $28;  // @constdefine the connection could not be established because it would multi-home an exclusive connection
      CONNMGR_STATUS_NORESOURCES                = $29;  // @constdefine Failed to allocate resources to make the connection.
      CONNMGR_STATUS_CONNECTIONLINKFAILED       = $2A;  // @constdefine Connection link disconnected prematurely.
      CONNMGR_STATUS_AUTHENTICATIONFAILED       = $2B;  // @constdefine Failed to authenticate user.
      CONNMGR_STATUS_NOPATHWITHPROPERTY         = $2C;  // @constdefine Path with connection having requested property, ex. WAKE_ON_INCOMING, is not available.

      CONNMGR_STATUS_WAITINGCONNECTION          = $40;  // @constdefine Attempting to connect
      CONNMGR_STATUS_WAITINGFORRESOURCE         = $41;  // @constdefine Resource is in use by another connection
      CONNMGR_STATUS_WAITINGFORNETWORK          = $42;  // @constdefine Network is used by higher priority thread or device is roaming.

      CONNMGR_STATUS_WAITINGDISCONNECTION       = $80;  // @constdefine Connection is being brought down
      CONNMGR_STATUS_WAITINGCONNECTIONABORT     = $81;  // @constdefine Aborting connection attempt


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Returns status about the current connection.
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrConnectionStatus(hConnection:HANDLE;    // @parm Handle to connection, returned from ConnMgrEstablishConnection
                                 pdwStatus:LPDWORD       // @parm Returns current connection status, one of CONNMGR_STATUS_*
                                ):HRESULT; external CellcoreDLL name 'ConnMgrConnectionStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deletes specified connection request, potentially dropping the physical connection.
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrReleaseConnection(hConnection:HANDLE;    // @parm Handle to connection, returned from ConnMgrEstablishConnection
                                  bCache:BOOL            // @parm ConnMgr can cache connection
                                 ):HRESULT; external CellcoreDLL name 'ConnMgrReleaseConnection';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Changes a connection's priority.
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrSetConnectionPriority(hConnection:HANDLE;    // @parm Handle to connection, returned from ConnMgrEstablishConnection
                                      dwPriority:DWORD       // @parm New priority
                                     ):HRESULT; external CellcoreDLL name 'ConnMgrSetConnectionPriority';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func General purpose (backdoor) API for exchanging information with planner or providers.
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrProviderMessage(hConnection:HANDLE;         // @parm Optional, Handle to connection
                                pguidProvider:LPGUID;  // @parm Provider GUID
                                pdwIndex:LPDWORD;            // @parm Optional index, used to address multiple providers associated with connection
                                dwMsg1:DWORD;               // @parm General param 1
                                dwMsg2:DWORD;               // @parm General param 2
                                pParams:PBYTE;              // @param Pointer to params structure
                                cbParamSize:ULONG           // @param size of params structure
                               ):HRESULT; external CellcoreDLL name 'ConnMgrProviderMessage';

const
      CONNMGR_MAX_DESC = 128;    // @constdefine Max size of a network description

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct CONNMGR_DESTINATION_INFO | Information about a specific network
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     _CONNMGR_DESTINATION_INFO = record
       _guid:GUID;  // @field GUID associated with network
       szDescription:array[0..CONNMGR_MAX_DESC-1] of TCHAR;  // @field Description of network
       fSecure:BOOL; // @field Is it OK to allow multi-homing on the network
     end;
     CONNMGR_DESTINATION_INFO = _CONNMGR_DESTINATION_INFO;
     LPCONNMGR_DESTINATION_INFO = ^_CONNMGR_DESTINATION_INFO;


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enumerates available networks.
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrEnumDestinations(nIndex:longint;                            // @param Index of network
                                 pDestInfo:LPCONNMGR_DESTINATION_INFO    // @param ptr to structure to fill in with info about network
                                ):HRESULT; external CellcoreDLL name 'ConnMgrEnumDestinations';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct SCHEDULEDCONNECTIONINFO | Information about a scheduled connection
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     _SCHEDULEDCONNECTIONINFO = record
       guidDest:GUID;                  // @field Guid of network
       uiStartTime:UINT64;             // @field Starting time, same ref as filetime
       uiEndTime:UINT64;               // @field Ending time, same ref as filetime
       uiPeriod:UINT64;                // @field Period between schedule attempts
       szAppName:array[0..MAX_PATH-1] of TCHAR;    // @field App name to execute when scheduled
       szCmdLine:array[0..MAX_PATH-1] of TCHAR;    // @field Cmd line to execute when scheduled
       szToken:array[0..31] of TCHAR;            // @field Unique token identifying this scheduled connection
       bPiggyback:BOOL;                // @field If true, execute app whenever network is available
     end;
     SCHEDULEDCONNECTIONINFO = _SCHEDULEDCONNECTIONINFO;
     LPSCHEDULEDCONNECTIONINFO = ^_SCHEDULEDCONNECTIONINFO;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Registers a scheduled connection
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrRegisterScheduledConnection(pSCI:SCHEDULEDCONNECTIONINFO  // @param Ptr to struct describing scheduled connection
                                           ):HRESULT; external CellcoreDLL name 'ConnMgrRegisterScheduledConnection';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Unregisters a scheduled connection
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrUnregisterScheduledConnection(pwszToken:LPCTSTR  // @param Token of scheduled connection to unregister
                                              ):HRESULT; external CellcoreDLL name 'ConnMgrUnregisterScheduledConnection';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Maps a URL to a destination network GUID
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrMapURL(pwszURL:LPCTSTR; // @parm URL to map
                       pguid:LPGUID;     // @parm Returned network GUID
                       pdwIndex:LPDWORD  // @parm Index in table for next search
                      ):HRESULT; external CellcoreDLL name 'ConnMgrMapURL';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Returns a handle to an event which becomes signaled when the ConnMgr API
// is ready to be used. Caller is responsible for calling CloseHandle on the returned event.
//
// @comm none
//
// -----------------------------------------------------------------------------
function ConnMgrApiReadyEvent:HANDLE; external CellcoreDLL name 'ConnMgrApiReadyEvent';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Defines the type of a connection reference
//
// @comm none
//
// -----------------------------------------------------------------------------

{$PACKENUM 4}
type
     _ConnMgrConRefTypeEnum = (ConRefType_NAP := 0,     // @constdefine NAP connection reference
                               ConRefType_PROXY        // @constdefine PROXY connection reference
                              );
     ConnMgrConRefTypeEnum = _ConnMgrConRefTypeEnum;
{$PACKENUM DEFAULT}

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Maps a connection reference to its corresponding GUID
//
// @comm none
//
// -----------------------------------------------------------------------------

function ConnMgrMapConRef(e:ConnMgrConRefTypeEnum;        // @parm Specify type of connection reference
                          szConRef:LPCTSTR;               // @parm Connection reference to map
                          pGUID:LPGUID                     // @parm Returned connection reference GUID
                         ):HRESULT; external CellcoreDLL name 'ConnMgrMapConRef';


// - end of connmgr.h

implementation

end.
