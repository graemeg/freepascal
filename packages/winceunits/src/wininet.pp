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
//    wininet.h
//
// Abstract:
//
//    Contains manifests, macros, types and prototypes for Microsoft Windows
//    Internet Extensions
//

//
//  Microsoft Windows Mobile 6.0 for PocketPC SDK.
//

unit wininet;

{$IFNDEF NO_SMART_LINK}
{$SMARTLINK ON}
{$ENDIF}

{$CALLING cdecl}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
interface

uses Windows;

// *
// * Set up Structure Packing to be 4 bytes
// * for all wininet structures
// *
{$PACKRECORDS 4}
{
#if defined(_WIN64)
#include <pshpack8.h>
#else
#include <pshpack4.h>
#endif
}

//
// internet types
//
type
     HINTERNET = LPVOID;
     LPHINTERNET = ^HINTERNET;

     INTERNET_PORT = word;
     LPINTERNET_PORT = ^INTERNET_PORT;

//
// Internet APIs
//

//
// manifests
//
const
      INTERNET_INVALID_PORT_NUMBER    = 0;           // use the protocol-specific default

      INTERNET_DEFAULT_FTP_PORT       = 21;          // default for FTP servers
      INTERNET_DEFAULT_GOPHER_PORT    = 70;          //    "     "  gopher "
      INTERNET_DEFAULT_HTTP_PORT      = 80;          //    "     "  HTTP   "
      INTERNET_DEFAULT_HTTPS_PORT     = 443;         //    "     "  HTTPS  "
      INTERNET_DEFAULT_SOCKS_PORT     = 1080;        // default for SOCKS firewall servers.


//
// maximum field lengths (arbitrary)
//
const
      INTERNET_MAX_HOST_NAME_LENGTH   = 256;
      INTERNET_MAX_USER_NAME_LENGTH   = 128;
      INTERNET_MAX_PASSWORD_LENGTH    = 128;
      INTERNET_MAX_PORT_NUMBER_LENGTH = 5;           // INTERNET_PORT is unsigned short
      INTERNET_MAX_PORT_NUMBER_VALUE  = 65535;       // maximum unsigned short value
      INTERNET_MAX_PATH_LENGTH        = 2048;
      INTERNET_MAX_SCHEME_LENGTH      = 32;          // longest protocol name length

      INTERNET_MAX_URL_LENGTH         = INTERNET_MAX_SCHEME_LENGTH + Length('://') + INTERNET_MAX_PATH_LENGTH;
{ Was declared as
    INTERNET_MAX_URL_LENGTH         (INTERNET_MAX_SCHEME_LENGTH  + sizeof("://") + INTERNET_MAX_PATH_LENGTH)
}


//
// values returned by InternetQueryOption() with INTERNET_OPTION_KEEP_CONNECTION:
//
const
      INTERNET_KEEP_ALIVE_UNKNOWN     = DWORD(-1);
      INTERNET_KEEP_ALIVE_ENABLED     = 1;
      INTERNET_KEEP_ALIVE_DISABLED    = 0;

//
// flags returned by InternetQueryOption() with INTERNET_OPTION_REQUEST_FLAGS
//
const
      INTERNET_REQFLAG_FROM_CACHE     = $00000001;  // response came from cache
      INTERNET_REQFLAG_ASYNC          = $00000002;  // request was made asynchronously
      INTERNET_REQFLAG_VIA_PROXY      = $00000004;  // request was made via a proxy
      INTERNET_REQFLAG_NO_HEADERS     = $00000008;  // orginal response contained no headers
      INTERNET_REQFLAG_PASSIVE        = $00000010;  // FTP: passive-mode connection
      INTERNET_REQFLAG_CACHE_WRITE_DISABLED = $00000040;  // HTTPS: this request not cacheable
      INTERNET_REQFLAG_NET_TIMEOUT    = $00000080;  // w/ _FROM_CACHE: net request timed out

//
// flags common to open functions (not InternetOpen()):
//
const
      INTERNET_FLAG_RELOAD            = $80000000;  // retrieve the original item

//
// flags for InternetOpenUrl():
//
const
      INTERNET_FLAG_RAW_DATA          = $40000000;  // FTP/gopher find: receive the item as raw (structured) data
      INTERNET_FLAG_EXISTING_CONNECT  = $20000000;  // FTP: use existing InternetConnect handle for server if possible

//
// flags for InternetOpen():
//
const
      INTERNET_FLAG_ASYNC             = $10000000;  // this request is asynchronous (where supported)

//
// protocol-specific flags:
//
const
      INTERNET_FLAG_PASSIVE           = $08000000;  // used for FTP connections

//
// additional cache flags
//
const
      INTERNET_FLAG_NO_CACHE_WRITE    = $04000000;  // don't write this item to the cache
      INTERNET_FLAG_DONT_CACHE        = INTERNET_FLAG_NO_CACHE_WRITE;
      INTERNET_FLAG_MAKE_PERSISTENT   = $02000000;  // make this item persistent in cache
      INTERNET_FLAG_FROM_CACHE        = $01000000;  // use offline semantics
      INTERNET_FLAG_OFFLINE           = INTERNET_FLAG_FROM_CACHE;

//
// additional flags
//
const
      INTERNET_FLAG_SECURE            = $00800000;  // use PCT/SSL if applicable (HTTP)
      INTERNET_FLAG_KEEP_CONNECTION   = $00400000;  // use keep-alive semantics
      INTERNET_FLAG_NO_AUTO_REDIRECT  = $00200000;  // don't handle redirections automatically
      INTERNET_FLAG_READ_PREFETCH     = $00100000;  // do background read prefetch
      INTERNET_FLAG_NO_COOKIES        = $00080000;  // no automatic cookie handling
      INTERNET_FLAG_NO_AUTH           = $00040000;  // no automatic authentication handling
      INTERNET_FLAG_RESTRICTED_ZONE   = $00020000;  // apply restricted zone policies for cookies, auth
      INTERNET_FLAG_CACHE_IF_NET_FAIL = $00010000;  // return cache file if net request fails

//
// Security Ignore Flags, Allow HttpOpenRequest to overide
//  Secure Channel (SSL/PCT) failures of the following types.
//
const
      INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTP   = $00008000; // ex: https:// to http://
      INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS  = $00004000; // ex: http:// to https://
      INTERNET_FLAG_IGNORE_CERT_DATE_INVALID  = $00002000; // expired X509 Cert.
      INTERNET_FLAG_IGNORE_CERT_CN_INVALID    = $00001000; // bad common name in X509 Cert.

//
// more caching flags
//
const
      INTERNET_FLAG_RESYNCHRONIZE     = $00000800;  // asking wininet to update an item if it is newer
      INTERNET_FLAG_HYPERLINK         = $00000400;  // asking wininet to do hyperlinking semantic which works right for scripts
      INTERNET_FLAG_NO_UI             = $00000200;  // no cookie popup
      INTERNET_FLAG_PRAGMA_NOCACHE    = $00000100;  // asking wininet to add "pragma: no-cache"
      INTERNET_FLAG_CACHE_ASYNC       = $00000080;  // ok to perform lazy cache-write
      INTERNET_FLAG_FORMS_SUBMIT      = $00000040;  // this is a forms submit
      INTERNET_FLAG_FWD_BACK          = $00000020;  // fwd-back button op
      INTERNET_FLAG_NEED_FILE         = $00000010;  // need a file for this request
      INTERNET_FLAG_MUST_CACHE_REQUEST = INTERNET_FLAG_NEED_FILE;

      INTERNET_FLAG_BGUPDATE           = $00000008; // Undocumented flag.


//
// FTP manifests
//
const
      FTP_TRANSFER_TYPE_UNKNOWN   = $00000000;
      FTP_TRANSFER_TYPE_ASCII     = $00000001;
      FTP_TRANSFER_TYPE_BINARY    = $00000002;

      FTP_TRANSFER_TYPE_MASK      = FTP_TRANSFER_TYPE_ASCII or FTP_TRANSFER_TYPE_BINARY;

//
// flags for FTP
//
const
      INTERNET_FLAG_TRANSFER_ASCII    = FTP_TRANSFER_TYPE_ASCII;     // 0x00000001
      INTERNET_FLAG_TRANSFER_BINARY   = FTP_TRANSFER_TYPE_BINARY;    // 0x00000002

//
// flags field masks
//
const
      SECURITY_INTERNET_MASK  = INTERNET_FLAG_IGNORE_CERT_CN_INVALID or
                                INTERNET_FLAG_IGNORE_CERT_DATE_INVALID or
                                INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS or
                                INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTP;

      INTERNET_FLAGS_MASK     = INTERNET_FLAG_RELOAD or
                                INTERNET_FLAG_RAW_DATA or
                                INTERNET_FLAG_EXISTING_CONNECT or
                                INTERNET_FLAG_ASYNC or
                                INTERNET_FLAG_PASSIVE or
                                INTERNET_FLAG_NO_CACHE_WRITE or
                                INTERNET_FLAG_MAKE_PERSISTENT or
                                INTERNET_FLAG_FROM_CACHE or
                                INTERNET_FLAG_SECURE or
                                INTERNET_FLAG_KEEP_CONNECTION or
                                INTERNET_FLAG_NO_AUTO_REDIRECT or
                                INTERNET_FLAG_READ_PREFETCH or
                                INTERNET_FLAG_NO_COOKIES or
                                INTERNET_FLAG_NO_AUTH or
                                INTERNET_FLAG_CACHE_IF_NET_FAIL or
                                SECURITY_INTERNET_MASK or
                                INTERNET_FLAG_RESYNCHRONIZE or
                                INTERNET_FLAG_HYPERLINK or
                                INTERNET_FLAG_NO_UI or
                                INTERNET_FLAG_PRAGMA_NOCACHE or
                                INTERNET_FLAG_CACHE_ASYNC or
                                INTERNET_FLAG_FORMS_SUBMIT or
                                INTERNET_FLAG_NEED_FILE or
                                INTERNET_FLAG_RESTRICTED_ZONE or
                                INTERNET_FLAG_TRANSFER_BINARY or
                                INTERNET_FLAG_TRANSFER_ASCII or
                                INTERNET_FLAG_FWD_BACK or
                                INTERNET_FLAG_BGUPDATE;


      INTERNET_ERROR_MASK_INSERT_CDROM                    = $01;
      INTERNET_ERROR_MASK_COMBINED_SEC_CERT               = $02;
      INTERNET_ERROR_MASK_NEED_MSN_SSPI_PKG               = $04;
      INTERNET_ERROR_MASK_LOGIN_FAILURE_DISPLAY_ENTITY_BODY = $08;

      INTERNET_OPTIONS_MASK   = DWORD(not INTERNET_FLAGS_MASK);

//
// common per-API flags (new APIs)
//
const
      WININET_API_FLAG_ASYNC          = $00000001;  // force async operation
      WININET_API_FLAG_SYNC           = $00000004;  // force sync operation
      WININET_API_FLAG_USE_CONTEXT    = $00000008;  // use value supplied in dwContext (even if 0)

//
// INTERNET_NO_CALLBACK - if this value is presented as the dwContext parameter
// then no call-backs will be made for that API
//
const
      INTERNET_NO_CALLBACK            = 0;


//
// structures/types
//

//
// INTERNET_SCHEME - enumerated URL scheme type
//

type
     INTERNET_SCHEME = (INTERNET_SCHEME_PARTIAL := -2,
                         INTERNET_SCHEME_UNKNOWN := -1,
                         INTERNET_SCHEME_DEFAULT := 0,
                         INTERNET_SCHEME_FTP,
                         INTERNET_SCHEME_GOPHER,
                         INTERNET_SCHEME_HTTP,
                         INTERNET_SCHEME_HTTPS,
                         INTERNET_SCHEME_FILE,
                         INTERNET_SCHEME_NEWS,
                         INTERNET_SCHEME_MAILTO,
                         INTERNET_SCHEME_SOCKS,
                         INTERNET_SCHEME_JAVASCRIPT,
                         INTERNET_SCHEME_VBSCRIPT,
                         INTERNET_SCHEME_RES);

const
      INTERNET_SCHEME_FIRST = INTERNET_SCHEME_FTP;
      INTERNET_SCHEME_LAST = INTERNET_SCHEME_RES;

type
     LPINTERNET_SCHEME = ^INTERNET_SCHEME;

//
// INTERNET_ASYNC_RESULT - this structure is returned to the application via
// the callback with INTERNET_STATUS_REQUEST_COMPLETE. It is not sufficient to
// just return the result of the async operation. If the API failed then the
// app cannot call GetLastError() because the thread context will be incorrect.
// Both the value returned by the async API and any resultant error code are
// made available. The app need not check dwError if dwResult indicates that
// the API succeeded (in this case dwError will be ERROR_SUCCESS)
//

type
     INTERNET_ASYNC_RESULT = record
       //
       // dwResult - the HINTERNET, DWORD or BOOL return code from an async API
       //
        dwResult:DWORD_PTR;
      //
       // dwError - the error code if the API failed
       //
        dwError:DWORD;
     end;
     LPINTERNET_ASYNC_RESULT = ^INTERNET_ASYNC_RESULT;


//
// INTERNET_PROXY_INFO - structure supplied with INTERNET_OPTION_PROXY to get/
// set proxy information on a InternetOpen() handle
//

type
     INTERNET_PROXY_INFO = record
      //
      // dwAccessType - INTERNET_OPEN_TYPE_DIRECT, INTERNET_OPEN_TYPE_PROXY, or
      // INTERNET_OPEN_TYPE_PRECONFIG (set only)
      //
       dwAccessType:DWORD;

      //
      // lpszProxy - proxy server list
      //
       lpszProxy:LPCTSTR;

      //
      // lpszProxyBypass - proxy bypass list
      //
       lpszProxyBypass:LPCTSTR;
     end;
     LPINTERNET_PROXY_INFO = ^INTERNET_PROXY_INFO;

//
// INTERNET_PER_CONN_OPTION_LIST - set per-connection options such as proxy
// and autoconfig info
//
// Set and queried using Internet[Set|Query]Option with
// INTERNET_OPTION_PER_CONNECTION_OPTION
//
type
     INTERNET_PER_CONN_OPTIONA = record
        dwOption:DWORD;      // option to be queried or set
       case DWORD of
         0: (dwValue:DWORD);     // dword value for the option
         1: (pszValue:LPSTR);    // pointer to string value for the option
           2: (ftValue:FILETIME);    // file-time value for the option
     end;
     LPINTERNET_PER_CONN_OPTIONA = ^INTERNET_PER_CONN_OPTIONA;

type
     INTERNET_PER_CONN_OPTIONW = record
        dwOption:DWORD;      // option to be queried or set
       case DWORD of
         0: (dwValue:DWORD);     // dword value for the option
         1: (pszValue:LPWSTR);     // pointer to string value for the option
           2: (ftValue:FILETIME);    // file-time value for the option
     end;
     LPINTERNET_PER_CONN_OPTIONW = ^INTERNET_PER_CONN_OPTIONW;

{$IFDEF UNICODE}
type
     INTERNET_PER_CONN_OPTION = INTERNET_PER_CONN_OPTIONW;
     LPINTERNET_PER_CONN_OPTION = LPINTERNET_PER_CONN_OPTIONW;
{$ELSE UNICODE}
type
     INTERNET_PER_CONN_OPTION = INTERNET_PER_CONN_OPTIONA;
     LPINTERNET_PER_CONN_OPTION = LPINTERNET_PER_CONN_OPTIONA;
{$ENDIF UNICODE}

type
     INTERNET_PER_CONN_OPTION_LISTA = record
       dwSize:DWORD;        // size of the INTERNET_PER_CONN_OPTION_LIST struct
        pszConnection:LPSTR;    // connection name to set/query options
        dwOptionCount:DWORD;    // number of options to set/query
        dwOptionError:DWORD;    // on error, which option failed
        pOptions:LPINTERNET_PER_CONN_OPTIONA; // array of options to set/query
     end;
     LPINTERNET_PER_CONN_OPTION_LISTA = ^INTERNET_PER_CONN_OPTION_LISTA;

type
     INTERNET_PER_CONN_OPTION_LISTW = record
       dwSize:DWORD;        // size of the INTERNET_PER_CONN_OPTION_LIST struct
        pszConnection:LPWSTR;   // connection name to set/query options
        dwOptionCount:DWORD;    // number of options to set/query
        dwOptionError:DWORD;    // on error, which option failed
        pOptions:LPINTERNET_PER_CONN_OPTIONW; // array of options to set/query
     end;
     LPINTERNET_PER_CONN_OPTION_LISTW = ^INTERNET_PER_CONN_OPTION_LISTW;

{$IFDEF UNICODE}
type
     INTERNET_PER_CONN_OPTION_LIST = INTERNET_PER_CONN_OPTION_LISTW;
     LPINTERNET_PER_CONN_OPTION_LIST = LPINTERNET_PER_CONN_OPTION_LISTW;
{$ELSE UNICODE}
type
     INTERNET_PER_CONN_OPTION_LIST = INTERNET_PER_CONN_OPTION_LISTA;
     LPINTERNET_PER_CONN_OPTION_LIST = LPINTERNET_PER_CONN_OPTION_LISTA;
{$ENDIF UNICODE}

//
// Options used in INTERNET_PER_CONN_OPTON struct
//
const
      INTERNET_PER_CONN_FLAGS                         = 1;
      INTERNET_PER_CONN_PROXY_SERVER                  = 2;
      INTERNET_PER_CONN_PROXY_BYPASS                  = 3;
      INTERNET_PER_CONN_AUTOCONFIG_URL                = 4;
      INTERNET_PER_CONN_AUTODISCOVERY_FLAGS           = 5;
      INTERNET_PER_CONN_AUTOCONFIG_SECONDARY_URL      = 6;
      INTERNET_PER_CONN_AUTOCONFIG_RELOAD_DELAY_MINS  = 7;
      INTERNET_PER_CONN_AUTOCONFIG_LAST_DETECT_TIME   = 8;
      INTERNET_PER_CONN_AUTOCONFIG_LAST_DETECT_URL    = 9;

//
// PER_CONN_FLAGS
//
const
      PROXY_TYPE_DIRECT                               = $00000001;   // direct to net
      PROXY_TYPE_PROXY                                = $00000002;   // via named proxy
      PROXY_TYPE_AUTO_PROXY_URL                       = $00000004;   // autoproxy URL
      PROXY_TYPE_AUTO_DETECT                          = $00000008;   // use autoproxy detection

//
// PER_CONN_AUTODISCOVERY_FLAGS
//
      AUTO_PROXY_FLAG_USER_SET                        = $00000001;   // user changed this setting
      AUTO_PROXY_FLAG_ALWAYS_DETECT                   = $00000002;   // force detection even when its not needed
      AUTO_PROXY_FLAG_DETECTION_RUN                   = $00000004;   // detection has been run
      AUTO_PROXY_FLAG_MIGRATED                        = $00000008;   // migration has just been done
      AUTO_PROXY_FLAG_DONT_CACHE_PROXY_RESULT         = $00000010;   // don't cache result of host=proxy name
      AUTO_PROXY_FLAG_CACHE_INIT_RUN                  = $00000020;   // don't initalize and run unless URL expired
      AUTO_PROXY_FLAG_DETECTION_SUSPECT               = $00000040;   // if we're on a LAN & Modem, with only one IP, bad?!?

//
// INTERNET_VERSION_INFO - version information returned via
// InternetQueryOption(..., INTERNET_OPTION_VERSION, ...)
//
type
     INTERNET_VERSION_INFO = record
        dwMajorVersion:DWORD;
        dwMinorVersion:DWORD;
     end;
     LPINTERNET_VERSION_INFO = ^INTERNET_VERSION_INFO;

//
// HTTP_VERSION_INFO - query or set global HTTP version (1.0 or 1.1)
//
type
     HTTP_VERSION_INFO = record
        dwMajorVersion:DWORD;
        dwMinorVersion:DWORD;
     end;
     LPHTTP_VERSION_INFO = ^HTTP_VERSION_INFO;

//
// INTERNET_CONNECTED_INFO - information used to set the global connected state
//

type
     INTERNET_CONNECTED_INFO = record
       //
       // dwConnectedState - new connected/disconnected state.
       // See INTERNET_STATE_CONNECTED, etc.
       //
        dwConnectedState:DWORD;
      //
       // dwFlags - flags controlling connected->disconnected (or disconnected->
       // connected) transition. See below
       //
        dwFlags:DWORD;
     end;
     LPINTERNET_CONNECTED_INFO = ^INTERNET_CONNECTED_INFO;


//
// flags for INTERNET_CONNECTED_INFO dwFlags
//

//
// ISO_FORCE_DISCONNECTED - if set when putting Wininet into disconnected mode,
// all outstanding requests will be aborted with a cancelled error
//
const
      ISO_FORCE_DISCONNECTED  = $00000001;


//
// URL_COMPONENTS - the constituent parts of an URL. Used in InternetCrackUrl()
// and InternetCreateUrl()
//
// For InternetCrackUrl(), if a pointer field and its corresponding length field
// are both 0 then that component is not returned. If the pointer field is NULL
// but the length field is not zero, then both the pointer and length fields are
// returned if both pointer and corresponding length fields are non-zero then
// the pointer field points to a buffer where the component is copied. The
// component may be un-escaped, depending on dwFlags
//
// For InternetCreateUrl(), the pointer fields should be NULL if the component
// is not required. If the corresponding length field is zero then the pointer
// field is the address of a zero-terminated string. If the length field is not
// zero then it is the string length of the corresponding pointer field
//

// #pragma warning( disable : 4121 )   // disable alignment warning

type
     URL_COMPONENTSA = record
       dwStructSize:DWORD;    // size of this structure. Used in version check
       lpszScheme:LPSTR;      // pointer to scheme name
       dwSchemeLength:DWORD;    // length of scheme name
       nScheme:INTERNET_SCHEME; // enumerated scheme type (if known)
       lpszHostName:LPSTR;    // pointer to host name
       dwHostNameLength:DWORD;  // length of host name
       nPort:INTERNET_PORT;   // converted port number
       lpszUserName:LPSTR;    // pointer to user name
       dwUserNameLength:DWORD;  // length of user name
       lpszPassword:LPSTR;    // pointer to password
       dwPasswordLength:DWORD;  // length of password
       lpszUrlPath:LPSTR;   // pointer to URL-path
       dwUrlPathLength:DWORD; // length of URL-path
       lpszExtraInfo:LPSTR;   // pointer to extra information (e.g. ?foo or #foo)
       dwExtraInfoLength:DWORD; // length of extra information
     end;
     LPURL_COMPONENTSA = ^URL_COMPONENTSA;

type
     URL_COMPONENTSW = record
       dwStructSize:DWORD;    // size of this structure. Used in version check
       lpszScheme:LPWSTR;     // pointer to scheme name
       dwSchemeLength:DWORD;    // length of scheme name
       nScheme:INTERNET_SCHEME; // enumerated scheme type (if known)
       lpszHostName:LPWSTR;   // pointer to host name
       dwHostNameLength:DWORD;  // length of host name
       nPort:INTERNET_PORT;   // converted port number
       lpszUserName:LPWSTR;   // pointer to user name
       dwUserNameLength:DWORD;  // length of user name
       lpszPassword:LPWSTR;   // pointer to password
       dwPasswordLength:DWORD;  // length of password
       lpszUrlPath:LPWSTR;    // pointer to URL-path
       dwUrlPathLength:DWORD; // length of URL-path
       lpszExtraInfo:LPWSTR;    // pointer to extra information (e.g. ?foo or #foo)
       dwExtraInfoLength:DWORD; // length of extra information
     end;
     LPURL_COMPONENTSW = ^URL_COMPONENTSW;

{$IFDEF UNICODE}
type
     URL_COMPONENTS = URL_COMPONENTSW;
     LPURL_COMPONENTS = LPURL_COMPONENTSW;
{$ELSE UNICODE}
type
     URL_COMPONENTS = URL_COMPONENTSA;
     LPURL_COMPONENTS = LPURL_COMPONENTSA;
{$ENDIF UNICODE}

// #pragma warning( default : 4121 )   // restore alignment warning

//
// INTERNET_CERTIFICATE_INFO lpBuffer - contains the certificate returned from
// the server
//

type
     INTERNET_CERTIFICATE_INFO = record
      //
      // ftExpiry - date the certificate expires.
      //
       ftExpiry:FILETIME;

      //
      // ftStart - date the certificate becomes valid.
      //
      ftStart:FILETIME;

      //
      // lpszSubjectInfo - the name of organization, site, and server
      //   the cert. was issued for.
      //
       lpszSubjectInfo:LPTSTR;

      //
      // lpszIssuerInfo - the name of orgainzation, site, and server
      //   the cert was issues by.
      //
       lpszIssuerInfo:LPTSTR;

      //
      // lpszProtocolName - the name of the protocol used to provide the secure
      //   connection.
      //
       lpszProtocolName:LPTSTR;

      //
      // lpszSignatureAlgName - the name of the algorithm used for signing
      //  the certificate.
      //
       lpszSignatureAlgName:LPTSTR;

      //
      // lpszEncryptionAlgName - the name of the algorithm used for
      //  doing encryption over the secure channel (SSL/PCT) connection.
      //
       lpszEncryptionAlgName:LPTSTR;

      //
      // dwKeySize - size of the key.
      //
       dwKeySize:DWORD;
     end;
     LPINTERNET_CERTIFICATE_INFO = ^INTERNET_CERTIFICATE_INFO;


//
// INTERNET_BUFFERS - combines headers and data. May be chained for e.g. file
// upload or scatter/gather operations. For chunked read/write, lpcszHeader
// contains the chunked-ext
//
type
     LPINTERNET_BUFFERSA = ^_INTERNET_BUFFERSA;
     _INTERNET_BUFFERSA = record
       dwStructSize:DWORD;          // used for API versioning. Set to sizeof(INTERNET_BUFFERS)
       Next:LPINTERNET_BUFFERSA;  // chain of buffers
       lpcszHeader:LPCSTR;        // pointer to headers (may be NULL)
       dwHeadersLength:DWORD;       // length of headers if not NULL
       dwHeadersTotal:DWORD;        // size of headers if not enough buffer
       lpvBuffer:LPVOID;          // pointer to data buffer (may be NULL)
       dwBufferLength:DWORD;        // length of data buffer if not NULL
       dwBufferTotal:DWORD;       // total size of chunk, or content-length if not chunked
       dwOffsetLow:DWORD;         // used for read-ranges (only used in HttpSendRequest2)
       dwOffsetHigh:DWORD;
     end;
     INTERNET_BUFFERSA = _INTERNET_BUFFERSA;

type
     LPINTERNET_BUFFERSW = ^_INTERNET_BUFFERSW;
     _INTERNET_BUFFERSW = record
       dwStructSize:DWORD;          // used for API versioning. Set to sizeof(INTERNET_BUFFERS)
       Next:LPINTERNET_BUFFERSW;  // chain of buffers
       lpcszHeader:LPCSTR;        // pointer to headers (may be NULL)
       dwHeadersLength:DWORD;       // length of headers if not NULL
       dwHeadersTotal:DWORD;        // size of headers if not enough buffer
       lpvBuffer:LPVOID;          // pointer to data buffer (may be NULL)
       dwBufferLength:DWORD;        // length of data buffer if not NULL
       dwBufferTotal:DWORD;       // total size of chunk, or content-length if not chunked
       dwOffsetLow:DWORD;         // used for read-ranges (only used in HttpSendRequest2)
       dwOffsetHigh:DWORD;
     end;
     INTERNET_BUFFERSW = _INTERNET_BUFFERSW;

{$IFDEF UNICODE}
type
     INTERNET_BUFFERS = INTERNET_BUFFERSW;
     LPINTERNET_BUFFERS = LPINTERNET_BUFFERSW;
{$ELSE UNICODE}
type
     INTERNET_BUFFERS = INTERNET_BUFFERSA;
     LPINTERNET_BUFFERS = LPINTERNET_BUFFERSA;
{$ENDIF UNICODE}

//
// prototypes
//
const
      WinInetDLL = 'wininet.dll';

function InternetTimeFromSystemTimeA(pst:LPSYSTEMTIME;  // input GMT time
                                             dwRFC:DWORD;        // RFC format
                                             lpszTime:LPSTR;       // output string buffer
                                             cbTime:DWORD        // output buffer size
                                            ):BOOL; external WinInetDLL name 'InternetTimeFromSystemTimeA';

function InternetTimeFromSystemTimeW(pst:LPSYSTEMTIME;  // input GMT time
                                             dwRFC:DWORD;        // RFC format
                                             lpszTime:LPWSTR;      // output string buffer
                                             cbTime:DWORD        // output buffer size
                                            ):BOOL; external WinInetDLL name 'InternetTimeFromSystemTimeW';

{$IFDEF UNICODE}
function InternetTimeFromSystemTime(pst:LPSYSTEMTIME;  // input GMT time
                                            dwRFC:DWORD;         // RFC format
                                            lpszTime:LPWSTR;       // output string buffer
                                            cbTime:DWORD         // output buffer size
                                           ):BOOL; external WinInetDLL name 'InternetTimeFromSystemTimeW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function InternetTimeFromSystemTime(pst:LPSYSTEMTIME;  // input GMT time
                                            dwRFC:DWORD;         // RFC format
                                    lpszTime:LPSTR;      // output string buffer
                                    cbTime:DWORD         // output buffer size
                                   ):BOOL; external WinInetDLL name 'InternetTimeFromSystemTimeA';
{$ELSE WIN32}
function InternetTimeFromSystemTime(pst:LPSYSTEMTIME;  // input GMT time
                                            dwRFC:DWORD;         // RFC format
                                    lpszTime:LPSTR;      // output string buffer
                                    cbTime:DWORD         // output buffer size
                                   ):BOOL; external WinInetDLL name 'InternetTimeFromSystemTime';
{$ENDIF WIN32}
{$ENDIF UNICODE}


//
// constants for InternetTimeFromSystemTime
//
const
      INTERNET_RFC1123_FORMAT     = 0;
      INTERNET_RFC1123_BUFSIZE    = 30;

function InternetTimeToSystemTimeA(lpszTime:LPCSTR;     // NULL terminated string
                                   pst:LPSYSTEMTIME;      // output in GMT time
                                           dwReserved:DWORD
                                  ):BOOL; external WinInetDLL name 'InternetTimeToSystemTimeA';

function InternetTimeToSystemTimeW(lpszTime:LPCWSTR;      // NULL terminated string
                                   pst:LPSYSTEMTIME;      // output in GMT time
                                           dwReserved:DWORD
                                  ):BOOL; external WinInetDLL name 'InternetTimeToSystemTimeW';

{$IFDEF UNICODE}
function InternetTimeToSystemTime(lpszTime:LPCWSTR;     // NULL terminated string
                                  pst:LPSYSTEMTIME;     // output in GMT time
                                          dwReserved:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetTimeToSystemTimeW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function InternetTimeToSystemTime(lpszTime:LPCSTR;      // NULL terminated string
                                  pst:LPSYSTEMTIME;     // output in GMT time
                                          dwReserved:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetTimeToSystemTimeA';
{$ELSE WIN32}
function InternetTimeToSystemTime(lpszTime:LPCSTR;      // NULL terminated string
                                  pst:LPSYSTEMTIME;     // output in GMT time
                                  dwReserved:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetTimeToSystemTime';
{$ENDIF WIN32}
{$ENDIF UNICODE}

function InternetCrackUrlA(lpszUrl:LPCSTR;
                           dwUrlLength:DWORD;
                                 dwFlags:DWORD;
                                 lpUrlComponents:LPURL_COMPONENTSA
                                ):BOOL; external WinInetDLL name 'InternetCrackUrlA';

function InternetCrackUrlW(lpszUrl:LPCWSTR;
                           dwUrlLength:DWORD;
                                 dwFlags:DWORD;
                                 lpUrlComponents:LPURL_COMPONENTSW
                                ):BOOL; external WinInetDLL name 'InternetCrackUrlW';

{$IFDEF UNICODE}
function InternetCrackUrl(lpszUrl:LPCWSTR;
                          dwUrlLength:DWORD;
                                dwFlags:DWORD;
                                lpUrlComponents:LPURL_COMPONENTSW
                               ):BOOL; external WinInetDLL name 'InternetCrackUrlW';
{$ELSE UNICODE}
function InternetCrackUrl(lpszUrl:LPCSTR;
                          dwUrlLength:DWORD;
                                dwFlags:DWORD;
                                lpUrlComponents:LPURL_COMPONENTSA
                               ):BOOL; external WinInetDLL name 'InternetCrackUrlA';
{$ENDIF UNICODE}

function InternetCreateUrlA(lpUrlComponents:LPURL_COMPONENTSA;
                                  dwFlags:DWORD;
                                  lpszUrl:LPSTR;
                                  lpdwUrlLength:LPDWORD
                                 ):BOOL; external WinInetDLL name 'InternetCreateUrlA';

function InternetCreateUrlW(lpUrlComponents:LPURL_COMPONENTSW;
                                  dwFlags:DWORD;
                                  lpszUrl:LPWSTR;
                                  lpdwUrlLength:LPDWORD
                                 ):BOOL; external WinInetDLL name 'InternetCreateUrlW';

{$IFDEF UNICODE}
function InternetCreateUrl(lpUrlComponents:LPURL_COMPONENTSW;
                                 dwFlags:DWORD;
                                 lpszUrl:LPWSTR;
                                 lpdwUrlLength:LPDWORD
                                ):BOOL; external WinInetDLL name 'InternetCreateUrlW';
{$ELSE UNICODE}
function InternetCreateUrl(lpUrlComponents:LPURL_COMPONENTSA;
                                 dwFlags:DWORD;
                                 lpszUrl:LPSTR;
                                 lpdwUrlLength:LPDWORD
                                ):BOOL; external WinInetDLL name 'InternetCreateUrlA';
{$ENDIF UNICODE}


function InternetCanonicalizeUrlA(lpszUrl:LPCSTR;
                                          lpszBuffer:LPSTR;
                                          lpdwBufferLength:LPDWORD;
                                          dwFlags:DWORD
                                         ):BOOL; external WinInetDLL name 'InternetCanonicalizeUrlA';

function InternetCanonicalizeUrlW(lpszUrl:LPCWSTR;
                                          lpszBuffer:LPWSTR;
                                          lpdwBufferLength:LPDWORD;
                                          dwFlags:DWORD
                                         ):BOOL; external WinInetDLL name 'InternetCanonicalizeUrlW';

{$IFDEF UNICODE}
function InternetCanonicalizeUrl(lpszUrl:LPCWSTR;
                                         lpszBuffer:LPWSTR;
                                         lpdwBufferLength:LPDWORD;
                                         dwFlags:DWORD
                                        ):BOOL; external WinInetDLL name 'InternetCanonicalizeUrlW';
{$ELSE UNICODE}
function InternetCanonicalizeUrl(lpszUrl:LPCSTR;
                                         lpszBuffer:LPSTR;
                                         lpdwBufferLength:LPDWORD;
                                         dwFlags:DWORD
                                        ):BOOL; external WinInetDLL name 'InternetCanonicalizeUrlA';
{$ENDIF UNICODE}

function InternetCombineUrlA(lpszBaseUrl:LPCSTR;
                                   lpszRelativeUrl:LPCSTR;
                                   lpszBuffer:LPSTR;
                                   lpdwBufferLength:LPDWORD;
                                   dwFlags:DWORD
                            ):BOOL; external WinInetDLL name 'InternetCombineUrlA';

function InternetCombineUrlW(lpszBaseUrl:LPCWSTR;
                                   lpszRelativeUrl:LPCWSTR;
                                   lpszBuffer:LPWSTR;
                                   lpdwBufferLength:LPDWORD;
                                   dwFlags:DWORD
                            ):BOOL; external WinInetDLL name 'InternetCombineUrlW';

{$IFDEF UNICODE}
function InternetCombineUrl(lpszBaseUrl:LPCWSTR;
                                  lpszRelativeUrl:LPCWSTR;
                                  lpszBuffer:LPWSTR;
                                  lpdwBufferLength:LPDWORD;
                                  dwFlags:DWORD
                           ):BOOL; external WinInetDLL name 'InternetCombineUrlW';
{$ELSE UNICODE}
function InternetCombineUrl(lpszBaseUrl:LPCSTR;
                                  lpszRelativeUrl:LPCSTR;
                                  lpszBuffer:LPSTR;
                                  lpdwBufferLength:LPDWORD;
                                  dwFlags:DWORD
                           ):BOOL; external WinInetDLL name 'InternetCombineUrlA';
{$ENDIF UNICODE}

//
// flags for InternetCrackUrl() and InternetCreateUrl()
//
const
      ICU_ESCAPE              = $80000000;  // (un)escape URL characters
      ICU_USERNAME            = $40000000;  // use internal username & password
      ICU_ESCAPE_AUTHORITY    = $00002000;  // causes InternetCreateUrlA to escape chars in authority components (user, pwd, host)


//
// flags for InternetCanonicalizeUrl() and InternetCombineUrl()
//
const
      ICU_NO_ENCODE          = $20000000;  // Don't convert unsafe characters to escape sequence
      ICU_DECODE             = $10000000;  // Convert %XX escape sequences to characters
      ICU_NO_META            = $08000000;  // Don't convert .. etc. meta path sequences
      ICU_ENCODE_SPACES_ONLY = $04000000;  // Encode spaces only
      ICU_BROWSER_MODE       = $02000000; // Special encode/decode rules for browser
      ICU_ENCODE_PERCENT     = $00001000; // Encode any percent (ASCII25)
                                          // signs encountered, default is to not encode percent.

function InternetOpenA(lpszAgent:LPCSTR;
                                dwAccessType:DWORD;
                                lpszProxy:LPCSTR;
                                lpszProxyBypass:LPCSTR;
                                dwFlags:DWORD
                               ):HINTERNET; external WinInetDLL name 'InternetOpenA';

function InternetOpenW(lpszAgent:LPCWSTR;
                                dwAccessType:DWORD;
                                lpszProxy:LPCWSTR;
                                lpszProxyBypass:LPCWSTR;
                                dwFlags:DWORD
                               ):HINTERNET; external WinInetDLL name 'InternetOpenW';

{$IFDEF UNICODE}
function InternetOpen(lpszAgent:LPCWSTR;
                               dwAccessType:DWORD;
                               lpszProxy:LPCWSTR;
                               lpszProxyBypass:LPCWSTR;
                               dwFlags:DWORD
                              ):HINTERNET; external WinInetDLL name 'InternetOpenW';
{$ELSE UNICODE}
function InternetOpen(lpszAgent:LPCSTR;
                               dwAccessType:DWORD;
                               lpszProxy:LPCSTR;
                               lpszProxyBypass:LPCSTR;
                               dwFlags:DWORD
                              ):HINTERNET; external WinInetDLL name 'InternetOpenA';
{$ENDIF UNICODE}

//
// access types for InternetOpen()
//
const
      INTERNET_OPEN_TYPE_PRECONFIG                    = 0;   // use registry configuration
      INTERNET_OPEN_TYPE_DIRECT                       = 1;   // direct to net
      INTERNET_OPEN_TYPE_PROXY                        = 3;   // via named proxy
      INTERNET_OPEN_TYPE_PRECONFIG_WITH_NO_AUTOPROXY  = 4;   // prevent using java/script/INS

//
// old names for access types
//
const
      PRE_CONFIG_INTERNET_ACCESS  = INTERNET_OPEN_TYPE_PRECONFIG;
      LOCAL_INTERNET_ACCESS       = INTERNET_OPEN_TYPE_DIRECT;
      CERN_PROXY_INTERNET_ACCESS  = INTERNET_OPEN_TYPE_PROXY;

function InternetCloseHandle(_hInternet:HINTERNET):BOOL; external WinInetDLL name 'InternetCloseHandle';

function InternetConnectA(_hInternet:HINTERNET;
                                    lpszServerName:LPCSTR;
                                    nServerPort:INTERNET_PORT;
                                    lpszUserName:LPCSTR;
                                    lpszPassword:LPCSTR;
                                    dwService:DWORD;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'InternetConnectA';

function InternetConnectW(_hInternet:HINTERNET;
                                    lpszServerName:LPCWSTR;
                                    nServerPort:INTERNET_PORT;
                                    lpszUserName:LPCWSTR;
                                    lpszPassword:LPCWSTR;
                                    dwService:DWORD;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'InternetConnectW';

{$IFDEF UNICODE}
function InternetConnect(_hInternet:HINTERNET;
                                   lpszServerName:LPCWSTR;
                                   nServerPort:INTERNET_PORT;
                                   lpszUserName:LPCWSTR;
                                   lpszPassword:LPCWSTR;
                                   dwService:DWORD;
                                   dwFlags:DWORD;
                                   dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'InternetConnectW';
{$ELSE UNICODE}
function InternetConnect(_hInternet:HINTERNET;
                                   lpszServerName:LPCSTR;
                                   nServerPort:INTERNET_PORT;
                                   lpszUserName:LPCSTR;
                                   lpszPassword:LPCSTR;
                                   dwService:DWORD;
                                   dwFlags:DWORD;
                                   dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'InternetConnectA';
{$ENDIF UNICODE}

//
// service types for InternetConnect()
//
const
      INTERNET_SERVICE_FTP    = 1;
      INTERNET_SERVICE_GOPHER = 2;
      INTERNET_SERVICE_HTTP   = 3;


function InternetOpenUrlA(_hInternet:HINTERNET;
                                    lpszUrl:LPCSTR;
                                    lpszHeaders:LPCSTR;
                                    dwHeadersLength:DWORD;
                                    dwFlags:DWORD;
                          dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'InternetOpenUrlA';

function InternetOpenUrlW(_hInternet:HINTERNET;
                                    lpszUrl:LPCWSTR;
                                    lpszHeaders:LPCWSTR;
                                    dwHeadersLength:DWORD;
                                    dwFlags:DWORD;
                          dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'InternetOpenUrlW';

{$IFDEF UNICODE}
function InternetOpenUrl(_hInternet:HINTERNET;
                                   lpszUrl:LPCWSTR;
                                   lpszHeaders:LPCWSTR;
                                   dwHeadersLength:DWORD;
                                   dwFlags:DWORD;
                         dwContext:DWORD_PTR
                                  ):HINTERNET; external WinInetDLL name 'InternetOpenUrlW';
{$ELSE UNICODE}
function InternetOpenUrl(_hInternet:HINTERNET;
                                   lpszUrl:LPCSTR;
                                   lpszHeaders:LPCSTR;
                                   dwHeadersLength:DWORD;
                                   dwFlags:DWORD;
                         dwContext:DWORD_PTR
                                  ):HINTERNET; external WinInetDLL name 'InternetOpenUrlA';
{$ENDIF UNICODE}

function InternetReadFile(_hFile:HINTERNET;
                                lpBuffer:LPVOID;
                                dwNumberOfBytesToRead:DWORD;
                                lpdwNumberOfBytesRead:LPDWORD
<<<<<<< HEAD
<<<<<<< HEAD
                               ):BOOL; external WinInetDLL name 'InternetReadFile';
=======
                               ):BOOL; external WinInetDLL name 'InternetREadFile';
>>>>>>> graemeg/fixes_2_2
=======
                               ):BOOL; external WinInetDLL name 'InternetREadFile';
>>>>>>> origin/fixes_2_2

function InternetReadFileExA(_hFile:HINTERNET;
                                      lpBuffersOut:LPINTERNET_BUFFERSA;
                                      dwFlags:DWORD;
                                      dwContext:DWORD_PTR
                                     ):BOOL; external WinInetDLL name 'InternetReadFileExA';

function InternetReadFileExW(_hFile:HINTERNET;
                                      lpBuffersOut:LPINTERNET_BUFFERSW;
                                      dwFlags:DWORD;
                                      dwContext:DWORD_PTR
                                     ):BOOL; external WinInetDLL name 'InternetReadFileExW';

{$IFDEF UNICODE}
function InternetReadFileEx(_hFile:HINTERNET;
                                     lpBuffersOut:LPINTERNET_BUFFERSW;
                                     dwFlags:DWORD;
                                     dwContext:DWORD_PTR
                                    ):BOOL; external WinInetDLL name 'InternetReadFileExW';
{$ELSE UNICODE}
function InternetReadFileEx(_hFile:HINTERNET;
                                     lpBuffersOut:LPINTERNET_BUFFERSA;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):BOOL; external WinInetDLL name 'InternetReadFileExA';
{$ENDIF UNICODE}


//
// flags for InternetReadFileEx()
//
const
      IRF_ASYNC       = WININET_API_FLAG_ASYNC;
      IRF_SYNC        = WININET_API_FLAG_SYNC;
      IRF_USE_CONTEXT = WININET_API_FLAG_USE_CONTEXT;
      IRF_NO_WAIT     = $00000008;

function InternetSetFilePointer(_hFile:HINTERNET;
                                          lDistanceToMove:LONG;
                                          pReserved:PVOID;
                                          dwMoveMethod:DWORD;
                                          dwContext:DWORD_PTR
                                         ):DWORD; external WinInetDLL name 'InternetSetFilePointer';

function InternetWriteFile(_hFile:HINTERNET;
                                 lpBuffer:LPCVOID;
                                 dwNumberOfBytesToWrite:DWORD;
                                 lpdwNumberOfBytesWritten:LPDWORD
                                ):BOOL; external WinInetDLL name 'InternetWriteFile';


function InternetQueryDataAvailable(_hFile:HINTERNET;
                                            lpdwNumberOfBytesAvailable:LPDWORD;
                                            dwFlags:DWORD;
                                            dwContext:DWORD_PTR
                                           ):BOOL; external WinInetDLL name 'InternetQueryDataAvailable';

function InternetFindNextFileA(hFind:HINTERNET;
                                      lpvFindData:LPVOID
                                     ):BOOL; external WinInetDLL name 'InternetFindNextFileA';

function InternetFindNextFileW(hFind:HINTERNET;
                                      lpvFindData:LPVOID
                                     ):BOOL; external WinInetDLL name 'InternetFindNextFileW';

{$IFDEF UNICODE}
function InternetFindNextFile(hFind:HINTERNET;
                                     lpvFindData:LPVOID
                                    ):BOOL; external WinInetDLL name 'InternetFindNextFileW';
{$ELSE UNICODE}
function InternetFindNextFile(hFind:HINTERNET;
                                    lpvFindData:LPVOID
                                   ):BOOL; external WinInetDLL name 'InternetFindNextFileA';
{$ENDIF UNICODE}

function InternetQueryOptionA(_hInternet:HINTERNET;
                                     dwOption:DWORD;
                                     lpBuffer:LPVOID;
                                     lpdwBufferLength:LPDWORD
                                    ):BOOL; external WinInetDLL name 'InternetQueryOptionA';

function InternetQueryOptionW(_hInternet:HINTERNET;
                                     dwOption:DWORD;
                                     lpBuffer:LPVOID;
                                     lpdwBufferLength:LPDWORD
                                    ):BOOL; external WinInetDLL name 'InternetQueryOptionW';

{$IFDEF UNICODE}
function InternetQueryOption(_hInternet:HINTERNET;
                                    dwOption:DWORD;
                                    lpBuffer:LPVOID;
                                    lpdwBufferLength:LPDWORD
                                   ):BOOL; external WinInetDLL name 'InternetQueryOptionW';
{$ELSE UNICODE}
function InternetQueryOption(_hInternet:HINTERNET;
                                    dwOption:DWORD;
                                    lpBuffer:LPVOID;
                                    lpdwBufferLength:LPDWORD
                                   ):BOOL; external WinInetDLL name 'InternetQueryOptionA';
{$ENDIF UNICODE}

function InternetSetOptionA(_hInternet:HINTERNET;
                                  dwOption:DWORD;
                                  lpBuffer:LPVOID;
                                  dwBufferLength:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetSetOptionA';

function InternetSetOptionW(_hInternet:HINTERNET;
                                  dwOption:DWORD;
                                  lpBuffer:LPVOID;
                                  dwBufferLength:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetSetOptionW';

{$IFDEF UNICODE}
function InternetSetOption(_hInternet:HINTERNET;
                                 dwOption:DWORD;
                                 lpBuffer:LPVOID;
                                 dwBufferLength:DWORD
                                ):BOOL; external WinInetDLL name 'InternetSetOptionW';
{$ELSE UNICODE}
function InternetSetOption(_hInternet:HINTERNET;
                                 dwOption:DWORD;
                                 lpBuffer:LPVOID;
                                 dwBufferLength:DWORD
                                ):BOOL; external WinInetDLL name 'InternetSetOptionA';
{$ENDIF UNICODE}

function InternetSetOptionExA(_hInternet:HINTERNET;
                              dwOption:DWORD;
                                     lpBuffer:LPVOID;
                                     dwBufferLength:DWORD;
                                     dwFlags:DWORD
                                    ):BOOL; external WinInetDLL name 'InternetSetOptionExA';

function InternetSetOptionExW(_hInternet:HINTERNET;
                              dwOption:DWORD;
                                     lpBuffer:LPVOID;
                                     dwBufferLength:DWORD;
                                     dwFlags:DWORD
                                    ):BOOL; external WinInetDLL name 'InternetSetOptionExW';

{$IFDEF UNICODE}
function InternetSetOptionEx(_hInternet:HINTERNET;
                             dwOption:DWORD;
                             lpBuffer:LPVOID;
                             dwBufferLength:DWORD;
                            dwFlags:DWORD
                            ):BOOL; external WinInetDLL name 'InternetSetOptionExW';
{$ELSE UNICODE}
function InternetSetOptionEx(_hInternet:HINTERNET;
                             dwOption:DWORD;
                             lpBuffer:LPVOID;
                             dwBufferLength:DWORD;
                             dwFlags:DWORD
                            ):BOOL; external WinInetDLL name 'InternetSetOptionExA';
{$ENDIF UNICODE}


function InternetLockRequestFile(_hInternet:HINTERNET;
                                 lphLockRequestInfo:LPHANDLE
                                ):BOOL; external WinInetDLL name 'InternetLockRequestFile';

function InternetUnlockRequestFile(hLockRequestInfo:HANDLE):BOOL; external WinInetDLL name 'InternetUnlockRequestFile';

//
// flags for InternetSetOptionEx()
//
const
      ISO_GLOBAL      = $00000001;  // modify option globally
      ISO_REGISTRY    = $00000002;  // write option to registry (where applicable)

      ISO_VALID_FLAGS = ISO_GLOBAL or ISO_REGISTRY;

//
// options manifests for Internet{Query|Set}Option
//
const
      INTERNET_OPTION_CALLBACK                = 1;
      INTERNET_OPTION_CONNECT_TIMEOUT         = 2;
      INTERNET_OPTION_CONNECT_RETRIES         = 3;
      INTERNET_OPTION_CONNECT_BACKOFF         = 4;
      INTERNET_OPTION_SEND_TIMEOUT            = 5;
      INTERNET_OPTION_CONTROL_SEND_TIMEOUT    = INTERNET_OPTION_SEND_TIMEOUT;
      INTERNET_OPTION_RECEIVE_TIMEOUT         = 6;
      INTERNET_OPTION_CONTROL_RECEIVE_TIMEOUT = INTERNET_OPTION_RECEIVE_TIMEOUT;
      INTERNET_OPTION_DATA_SEND_TIMEOUT       = 7;
      INTERNET_OPTION_DATA_RECEIVE_TIMEOUT    = 8;
      INTERNET_OPTION_HANDLE_TYPE             = 9;
      INTERNET_OPTION_LISTEN_TIMEOUT          = 11;
      INTERNET_OPTION_READ_BUFFER_SIZE        = 12;
      INTERNET_OPTION_WRITE_BUFFER_SIZE       = 13;

      INTERNET_OPTION_ASYNC_ID                = 15;
      INTERNET_OPTION_ASYNC_PRIORITY          = 16;

      INTERNET_OPTION_PARENT_HANDLE           = 21;
      INTERNET_OPTION_KEEP_CONNECTION         = 22;
      INTERNET_OPTION_REQUEST_FLAGS           = 23;
      INTERNET_OPTION_EXTENDED_ERROR          = 24;

      INTERNET_OPTION_FLUSH_CACHE             = 25;
      INTERNET_OPTION_OFFLINE_MODE            = 26;
      INTERNET_OPTION_CACHE_STREAM_HANDLE     = 27;
      INTERNET_OPTION_USERNAME                = 28;
      INTERNET_OPTION_PASSWORD                = 29;
      INTERNET_OPTION_ASYNC                   = 30;
      INTERNET_OPTION_SECURITY_FLAGS          = 31;
      INTERNET_OPTION_SECURITY_CERTIFICATE_STRUCT = 32;
      INTERNET_OPTION_DATAFILE_NAME           = 33;
      INTERNET_OPTION_URL                     = 34;
      INTERNET_OPTION_SECURITY_CERTIFICATE    = 35;
      INTERNET_OPTION_SECURITY_KEY_BITNESS    = 36;
      INTERNET_OPTION_REFRESH                 = 37;
      INTERNET_OPTION_PROXY                   = 38;
      INTERNET_OPTION_SETTINGS_CHANGED        = 39;
      INTERNET_OPTION_VERSION                 = 40;
      INTERNET_OPTION_USER_AGENT              = 41;
      INTERNET_OPTION_END_BROWSER_SESSION     = 42;
      INTERNET_OPTION_PROXY_USERNAME          = 43;
      INTERNET_OPTION_PROXY_PASSWORD          = 44;
      INTERNET_OPTION_CONTEXT_VALUE           = 45;
      INTERNET_OPTION_CONNECT_LIMIT           = 46;
      INTERNET_OPTION_SECURITY_SELECT_CLIENT_CERT = 47;
      INTERNET_OPTION_POLICY                  = 48;
      INTERNET_OPTION_DISCONNECTED_TIMEOUT    = 49;
      INTERNET_OPTION_CONNECTED_STATE         = 50;
      INTERNET_OPTION_IDLE_STATE              = 51;
      INTERNET_OPTION_OFFLINE_SEMANTICS       = 52;
      INTERNET_OPTION_SECONDARY_CACHE_KEY     = 53;
      INTERNET_OPTION_CALLBACK_FILTER         = 54;
      INTERNET_OPTION_CONNECT_TIME            = 55;
      INTERNET_OPTION_SEND_THROUGHPUT         = 56;
      INTERNET_OPTION_RECEIVE_THROUGHPUT      = 57;
      INTERNET_OPTION_REQUEST_PRIORITY        = 58;
      INTERNET_OPTION_HTTP_VERSION            = 59;
      INTERNET_OPTION_RESET_URLCACHE_SESSION  = 60;
      INTERNET_OPTION_ERROR_MASK              = 62;
      INTERNET_OPTION_FROM_CACHE_TIMEOUT      = 63;
      INTERNET_OPTION_BYPASS_EDITED_ENTRY     = 64;
      INTERNET_OPTION_CODEPAGE                = 68;
      INTERNET_OPTION_CACHE_TIMESTAMPS        = 69;
      INTERNET_OPTION_DISABLE_AUTODIAL        = 70;
      INTERNET_OPTION_MAX_CONNS_PER_SERVER    = 73;
      INTERNET_OPTION_MAX_CONNS_PER_1_0_SERVER = 74;
      INTERNET_OPTION_PER_CONNECTION_OPTION    = 75;
      INTERNET_OPTION_DIGEST_AUTH_UNLOAD       = 76;
      INTERNET_OPTION_IGNORE_OFFLINE           = 77;
      INTERNET_OPTION_IDENTITY                 = 78;
      INTERNET_OPTION_REMOVE_IDENTITY          = 79;
      INTERNET_OPTION_ALTER_IDENTITY           = 80;
      INTERNET_OPTION_SUPPRESS_BEHAVIOR        = 81;
      INTERNET_OPTION_AUTODIAL_MODE            = 82;
      INTERNET_OPTION_AUTODIAL_CONNECTION      = 83;
      INTERNET_OPTION_CLIENT_CERT_CONTEXT      = 84;
      INTERNET_OPTION_AUTH_FLAGS               = 85;
      INTERNET_OPTION_COOKIES_3RD_PARTY        = 86;
      INTERNET_OPTION_DISABLE_PASSPORT_AUTH    = 87;
      INTERNET_OPTION_SEND_UTF8_SERVERNAME_TO_PROXY         = 88;
      INTERNET_OPTION_EXEMPT_CONNECTION_LIMIT  = 89;
      INTERNET_OPTION_ENABLE_PASSPORT_AUTH     = 90;

      INTERNET_OPTION_HIBERNATE_INACTIVE_WORKER_THREADS       = 91;
      INTERNET_OPTION_ACTIVATE_WORKER_THREADS                 = 92;
      INTERNET_OPTION_RESTORE_WORKER_THREAD_DEFAULTS          = 93;
      INTERNET_OPTION_SOCKET_SEND_BUFFER_LENGTH               = 94;
      INTERNET_OPTION_PROXY_SETTINGS_CHANGED                  = 95;
      INTERNET_OPTION_SERVER_CERT_CONTEXT                     = 96;

      INTERNET_OPTION_SOCKET_LINGER_TIME                      = 97;


      INTERNET_FIRST_OPTION                   = INTERNET_OPTION_CALLBACK;
      INTERNET_LAST_OPTION                    = INTERNET_OPTION_PROXY_SETTINGS_CHANGED;

//
// values for INTERNET_OPTION_PRIORITY
//
const
      INTERNET_PRIORITY_FOREGROUND            = 1000;

//
// handle types
//
const
      INTERNET_HANDLE_TYPE_INTERNET           = 1;
      INTERNET_HANDLE_TYPE_CONNECT_FTP        = 2;
      INTERNET_HANDLE_TYPE_CONNECT_GOPHER     = 3;
      INTERNET_HANDLE_TYPE_CONNECT_HTTP       = 4;
      INTERNET_HANDLE_TYPE_FTP_FIND           = 5;
      INTERNET_HANDLE_TYPE_FTP_FIND_HTML      = 6;
      INTERNET_HANDLE_TYPE_FTP_FILE           = 7;
      INTERNET_HANDLE_TYPE_FTP_FILE_HTML      = 8;
      INTERNET_HANDLE_TYPE_GOPHER_FIND        = 9;
      INTERNET_HANDLE_TYPE_GOPHER_FIND_HTML   = 10;
      INTERNET_HANDLE_TYPE_GOPHER_FILE        = 11;
      INTERNET_HANDLE_TYPE_GOPHER_FILE_HTML   = 12;
      INTERNET_HANDLE_TYPE_HTTP_REQUEST       = 13;
      INTERNET_HANDLE_TYPE_FILE_REQUEST       = 14;


//
// values for INTERNET_OPTION_AUTH_FLAGS
//
const
      AUTH_FLAG_DISABLE_NEGOTIATE             = $00000001;
      AUTH_FLAG_ENABLE_NEGOTIATE              = $00000002;

//
// values for INTERNET_OPTION_SECURITY_FLAGS
//

// query only
const
      SECURITY_FLAG_SECURE                    = $00000001; // can query only
      SECURITY_FLAG_STRENGTH_WEAK             = $10000000;
      SECURITY_FLAG_STRENGTH_MEDIUM           = $40000000;
      SECURITY_FLAG_STRENGTH_STRONG           = $20000000;
      SECURITY_FLAG_UNKNOWNBIT                = $80000000;
      SECURITY_FLAG_FORTEZZA                  = $08000000;
      SECURITY_FLAG_NORMALBITNESS             = SECURITY_FLAG_STRENGTH_WEAK;



// The following are unused
const
      SECURITY_FLAG_SSL                       = $00000002;
      SECURITY_FLAG_SSL3                      = $00000004;
      SECURITY_FLAG_PCT                       = $00000008;
      SECURITY_FLAG_PCT4                      = $00000010;
      SECURITY_FLAG_IETFSSL4                  = $00000020;

// The following are for backwards compatability only.
const
      SECURITY_FLAG_40BIT                     = SECURITY_FLAG_STRENGTH_WEAK;
      SECURITY_FLAG_128BIT                    = SECURITY_FLAG_STRENGTH_STRONG;
      SECURITY_FLAG_56BIT                     = SECURITY_FLAG_STRENGTH_MEDIUM;

// setable flags
const
      SECURITY_FLAG_IGNORE_REVOCATION         = $00000080;
      SECURITY_FLAG_IGNORE_UNKNOWN_CA         = $00000100;
      SECURITY_FLAG_IGNORE_WRONG_USAGE        = $00000200;

      SECURITY_FLAG_IGNORE_CERT_CN_INVALID    = INTERNET_FLAG_IGNORE_CERT_CN_INVALID;
      SECURITY_FLAG_IGNORE_CERT_DATE_INVALID  = INTERNET_FLAG_IGNORE_CERT_DATE_INVALID;


      SECURITY_FLAG_IGNORE_REDIRECT_TO_HTTPS  = INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS;
      SECURITY_FLAG_IGNORE_REDIRECT_TO_HTTP   = INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTP;



      SECURITY_SET_MASK       = SECURITY_FLAG_IGNORE_REVOCATION or
                                SECURITY_FLAG_IGNORE_UNKNOWN_CA or
                                SECURITY_FLAG_IGNORE_CERT_CN_INVALID or
                                SECURITY_FLAG_IGNORE_CERT_DATE_INVALID or
                                SECURITY_FLAG_IGNORE_WRONG_USAGE;

// valid autodial modes
const
      AUTODIAL_MODE_NEVER                     = 1;
      AUTODIAL_MODE_ALWAYS                    = 2;
      AUTODIAL_MODE_NO_NETWORK_PRESENT        = 4;


function InternetGetLastResponseInfoA(lpdwError:LPDWORD;
                                      lpszBuffer:LPSTR;
                                      lpdwBufferLength:LPDWORD
                                     ):BOOL; external WinInetDLL name 'InternetGetLastResponseInfoA';

function InternetGetLastResponseInfoW(lpdwError:LPDWORD;
                                      lpszBuffer:LPWSTR;
                                      lpdwBufferLength:LPDWORD
                                     ):BOOL; external WinInetDLL name 'InternetGetLastResponseInfoW';

{$IFDEF UNICODE}
function InternetGetLastResponseInfo(lpdwError:LPDWORD;
                                     lpszBuffer:LPWSTR;
                                     lpdwBufferLength:LPDWORD
                                    ):BOOL; external WinInetDLL name 'InternetGetLastResponseInfoW';
{$ELSE UNICODE}
function InternetGetLastResponseInfo(lpdwError:LPDWORD;
                                     lpszBuffer:LPSTR;
                                     lpdwBufferLength:LPDWORD
                                    ):BOOL; external WinInetDLL name 'InternetGetLastResponseInfoA';
{$ENDIF UNICODE}


//
// callback function for InternetSetStatusCallback
//

type
     INTERNET_STATUS_CALLBACK = procedure(_hInternet:HINTERNET;
                                          dwContext:DWORD_PTR;
                                          dwInternetStatus:DWORD;
                                          lpvStatusInformation:LPVOID;
                                          dwStatusInformationLength:DWORD); cdecl;


     LPINTERNET_STATUS_CALLBACK = INTERNET_STATUS_CALLBACK;


function InternetSetStatusCallbackA(_hInternet:HINTERNET;
                                    lpfnInternetCallback:INTERNET_STATUS_CALLBACK
                                   ):INTERNET_STATUS_CALLBACK; external WinInetDLL name 'InternetSetStatusCallbackA';

function InternetSetStatusCallbackW(_hInternet:HINTERNET;
                                    lpfnInternetCallback:INTERNET_STATUS_CALLBACK
                                   ):INTERNET_STATUS_CALLBACK; external WinInetDLL name 'InternetSetStatusCallbackW';

{$IFDEF UNICODE}
function InternetSetStatusCallback(_hInternet:HINTERNET;
                                   lpfnInternetCallback:INTERNET_STATUS_CALLBACK
                                  ):INTERNET_STATUS_CALLBACK; external WinInetDLL name 'InternetSetStatusCallbackW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function InternetSetStatusCallback(_hInternet:HINTERNET;
                                   lpfnInternetCallback:INTERNET_STATUS_CALLBACK
                                  ):INTERNET_STATUS_CALLBACK; external WinInetDLL name 'InternetSetStatusCallbackA';
{$ELSE WIN32}
function InternetSetStatusCallback(_hInternet:HINTERNET;
                                   lpfnInternetCallback:INTERNET_STATUS_CALLBACK
                                  ):INTERNET_STATUS_CALLBACK; external WinInetDLL name 'InternetSetStatusCallback';
{$ENDIF WIN32}
{$ENDIF UNICODE}

//
// status manifests for Internet status callback
//
const
      INTERNET_STATUS_RESOLVING_NAME          = 10;
      INTERNET_STATUS_NAME_RESOLVED           = 11;
      INTERNET_STATUS_CONNECTING_TO_SERVER    = 20;
      INTERNET_STATUS_CONNECTED_TO_SERVER     = 21;
      INTERNET_STATUS_SENDING_REQUEST         = 30;
      INTERNET_STATUS_REQUEST_SENT            = 31;
      INTERNET_STATUS_RECEIVING_RESPONSE      = 40;
      INTERNET_STATUS_RESPONSE_RECEIVED       = 41;
      INTERNET_STATUS_CTL_RESPONSE_RECEIVED   = 42;
      INTERNET_STATUS_PREFETCH                = 43;
      INTERNET_STATUS_CLOSING_CONNECTION      = 50;
      INTERNET_STATUS_CONNECTION_CLOSED       = 51;
      INTERNET_STATUS_HANDLE_CREATED          = 60;
      INTERNET_STATUS_HANDLE_CLOSING          = 70;
      INTERNET_STATUS_DETECTING_PROXY         = 80;
      INTERNET_STATUS_REQUEST_COMPLETE        = 100;
      INTERNET_STATUS_REDIRECT                = 110;
      INTERNET_STATUS_INTERMEDIATE_RESPONSE   = 120;
      INTERNET_STATUS_USER_INPUT_REQUIRED     = 140;
      INTERNET_STATUS_STATE_CHANGE            = 200;
      INTERNET_STATUS_COOKIE_SENT             = 320;
      INTERNET_STATUS_COOKIE_RECEIVED         = 321;
      INTERNET_STATUS_COOKIE_STATE            = 322;
      INTERNET_STATUS_COOKIE_SUPPRESSED       = 323;
      INTERNET_STATUS_PRIVACY_IMPACTED        = 324;
      INTERNET_STATUS_P3P_HEADER              = 325;
      INTERNET_STATUS_P3P_POLICYREF           = 326;
      INTERNET_STATUS_COOKIE_HISTORY          = 327;
      INTERNET_STATUS_SSL_NEGOTIATION_COMPLETE  = 401;

//
// the following can be indicated in a state change notification:
//
const
      INTERNET_STATE_CONNECTED                = $00000001;  // connected state (mutually exclusive with disconnected)
      INTERNET_STATE_DISCONNECTED             = $00000002;  // disconnected from network
      INTERNET_STATE_DISCONNECTED_BY_USER     = $00000010;  // disconnected by user request
      INTERNET_STATE_IDLE                     = $00000100;  // no network requests being made (by Wininet)
      INTERNET_STATE_BUSY                     = $00000200;  // network requests being made (by Wininet)

//
// the following values are used for cookie state:
//
type
     InternetCookieState = (COOKIE_STATE_UNKNOWN := $00,
                            COOKIE_STATE_ACCEPT  := $01,
                             COOKIE_STATE_PROMPT  := $02,
                             COOKIE_STATE_LEASH   := $03,
                             COOKIE_STATE_DOWNGRADE := $04,
                             COOKIE_STATE_REJECT    := $05);

const
      COOKIE_STATE_MAX   = COOKIE_STATE_REJECT;

type
     IncomingCookieState = record
       cSession:longint;      // Session cookies received
       cPersistent:longint;   // Persistent cookies received

       cAccepted:longint;     // Number of cookies accepted
       cLeashed:longint;      //               ... leashed
       cDowngraded:longint;   //               ... converted to session-cookies
       cBlocked:longint;      //               ... rejected

       pszLocation:PChar;   // Optional: URL associated with reported cookie events
                            // This can be used to override request URL
     end;

type
     OutgoingCookieState = record
       cSent:longint;
       cSuppressed:longint;

       pszLocation:PChar;   // Optional: URL associated with reported cookie events
                            // This can be used to override request URL
     end;

type
     InternetCookieHistory = record
       fAccepted:BOOL;
        fLeashed:BOOL;
        fDowngraded:BOOL;
        fRejected:BOOL;
     end;

type
     CookieDecision = record
       dwCookieState:DWORD;
        fAllowSession:BOOL;
     end;

//
// if the following value is returned by InternetSetStatusCallback, then
// probably an invalid (non-code) address was supplied for the callback
//
const
      INTERNET_INVALID_STATUS_CALLBACK        = pointer(-1); // INTERNET_STATUS_CALLBACK(longint(-1));

//
// FTP
//

//
// prototypes
//

function FtpFindFirstFileA(hConnect:HINTERNET;
                           lpszSearchFile:LPCSTR;
                           lpFindFileData:LPWIN32_FIND_DATAA;
                           dwFlags:DWORD;
                           dwContext:DWORD_PTR
                          ):HINTERNET; external WinInetDLL name 'FtpFindFirstFileA';

function FtpFindFirstFileW(hConnect:HINTERNET;
                           lpszSearchFile:LPCWSTR;
                           lpFindFileData:LPWIN32_FIND_DATAW;
                           dwFlags:DWORD;
                           dwContext:DWORD_PTR
                          ):HINTERNET; external WinInetDLL name 'FtpFindFirstFileW';

{$IFDEF UNICODE}
function FtpFindFirstFile(hConnect:HINTERNET;
                          lpszSearchFile:LPCWSTR;
                          lpFindFileData:LPWIN32_FIND_DATAW;
                          dwFlags:DWORD;
                          dwContext:DWORD_PTR
                         ):HINTERNET; external WinInetDLL name 'FtpFindFirstFileW';
{$ELSE UNICODE}
function FtpFindFirstFile(hConnect:HINTERNET;
                          lpszSearchFile:LPCSTR;
                          lpFindFileData:LPWIN32_FIND_DATAA;
                          dwFlags:DWORD;
                          dwContext:DWORD_PTR
                         ):HINTERNET; external WinInetDLL name 'FtpFindFirstFileA';
{$ENDIF UNICODE}

function FtpGetFileA(hConnect:HINTERNET;
                     lpszRemoteFile:LPCSTR;
                     lpszNewFile:LPCSTR;
                     fFailIfExists:BOOL;
                     dwFlagsAndAttributes:DWORD;
                     dwFlags:DWORD;
                     dwContext:DWORD_PTR
                    ):BOOL; external WinInetDLL name 'FtpGetFileA';

function FtpGetFileW(hConnect:HINTERNET;
                     lpszRemoteFile:LPCWSTR;
                     lpszNewFile:LPCWSTR;
                     fFailIfExists:BOOL;
                     dwFlagsAndAttributes:DWORD;
                     dwFlags:DWORD;
                     dwContext:DWORD_PTR
                    ):BOOL; external WinInetDLL name 'FtpGetFileW';

{$IFDEF UNICODE}
function FtpGetFile(hConnect:HINTERNET;
                    lpszRemoteFile:LPCWSTR;
                    lpszNewFile:LPCWSTR;
                    fFailIfExists:BOOL;
                    dwFlagsAndAttributes:DWORD;
                    dwFlags:DWORD;
                    dwContext:DWORD_PTR
                   ):BOOL; external WinInetDLL name 'FtpGetFileW';
{$ELSE UNICODE}
function FtpGetFile(hConnect:HINTERNET;
                    lpszRemoteFile:LPCSTR;
                    lpszNewFile:LPCSTR;
                    fFailIfExists:BOOL;
                    dwFlagsAndAttributes:DWORD;
                    dwFlags:DWORD;
                    dwContext:DWORD_PTR
                   ):BOOL; external WinInetDLL name 'FtpGetFileA';
{$ENDIF UNICODE}

function FtpPutFileA(hConnect:HINTERNET;
                     lpszLocalFile:LPCSTR;
                     lpszNewRemoteFile:LPCSTR;
                     dwFlags:DWORD;
                     dwContext:DWORD_PTR
                    ):BOOL; external WinInetDLL name 'FtpPutFileA';

function FtpPutFileW(hConnect:HINTERNET;
                     lpszLocalFile:LPCWSTR;
                     lpszNewRemoteFile:LPCWSTR;
                     dwFlags:DWORD;
                     dwContext:DWORD_PTR
                    ):BOOL; external WinInetDLL name 'FtpPutFileW';

{$IFDEF UNICODE}
function FtpPutFile(hConnect:HINTERNET;
                    lpszLocalFile:LPCWSTR;
                    lpszNewRemoteFile:LPCWSTR;
                    dwFlags:DWORD;
                    dwContext:DWORD_PTR
                   ):BOOL; external WinInetDLL name 'FtpPutFileW';
{$ELSE UNICODE}
function FtpPutFile(hConnect:HINTERNET;
                    lpszLocalFile:LPCSTR;
                    lpszNewRemoteFile:LPCSTR;
                    dwFlags:DWORD;
                    dwContext:DWORD_PTR
                   ):BOOL; external WinInetDLL name 'FtpPutFileA';
{$ENDIF UNICODE}

function FtpGetFileEx(hFtpSession:HINTERNET;
                      lpszRemoteFile:LPCSTR;
                      lpszNewFile:LPCWSTR;
                      fFailIfExists:BOOL;
                      dwFlagsAndAttributes:DWORD;
                      dwFlags:DWORD;
                      dwContext:DWORD_PTR
                     ):BOOL; external WinInetDLL name 'FtpGetFileEx';

function FtpPutFileEx(hFtpSession:HINTERNET;
                      lpszLocalFile:LPCWSTR;
                      lpszNewRemoteFile:LPCSTR;
                      dwFlags:DWORD;
                      dwContext:DWORD_PTR
                     ):BOOL; external WinInetDLL name 'FtpPutFileEx';

function FtpDeleteFileA(hConnect:HINTERNET;
                        lpszFileName:LPCSTR
                       ):BOOL; external WinInetDLL name 'FtpDeleteFileA';

function FtpDeleteFileW(hConnect:HINTERNET;
                        lpszFileName:LPCWSTR
                       ):BOOL; external WinInetDLL name 'FtpDeleteFileW';

{$IFDEF UNICODE}
function FtpDeleteFile(hConnect:HINTERNET;
                       lpszFileName:LPCWSTR
                      ):BOOL; external WinInetDLL name 'FtpDeleteFileW';
{$ELSE UNICODE}
function FtpDeleteFile(hConnect:HINTERNET;
                       lpszFileName:LPCSTR
                      ):BOOL; external WinInetDLL name 'FtpDeleteFileA';
{$ENDIF UNICODE}

function FtpRenameFileA(hConnect:HINTERNET;
                        lpszExisting:LPCSTR;
                        lpszNew:LPCSTR
                       ):BOOL; external WinInetDLL name 'FtpRenameFileA';

function FtpRenameFileW(hConnect:HINTERNET;
                        lpszExisting:LPCWSTR;
                        lpszNew:LPCWSTR
                       ):BOOL; external WinInetDLL name 'FtpRenameFileW';
{$IFDEF UNICODE}
function FtpRenameFile(hConnect:HINTERNET;
                       lpszExisting:LPCWSTR;
                       lpszNew:LPCWSTR
                      ):BOOL; external WinInetDLL name 'FtpRenameFileW';
{$ELSE UNICODE}
function FtpRenameFile(hConnect:HINTERNET;
                       lpszExisting:LPCSTR;
                       lpszNew:LPCSTR
                      ):BOOL; external WinInetDLL name 'FtpRenameFileA';
{$ENDIF UNICODE}

function FtpOpenFileA(hConnect:HINTERNET;
                      lpszFileName:LPCSTR;
                      dwAccess:DWORD;
                      dwFlags:DWORD;
                      dwContext:DWORD_PTR
                     ):HINTERNET; external WinInetDLL name 'FtpOpenFileA';

function FtpOpenFileW(hConnect:HINTERNET;
                      lpszFileName:LPCWSTR;
                      dwAccess:DWORD;
                      dwFlags:DWORD;
                      dwContext:DWORD_PTR
                     ):HINTERNET; external WinInetDLL name 'FtpOpenFileW';

{$IFDEF UNICODE}
function FtpOpenFile(hConnect:HINTERNET;
                     lpszFileName:LPCWSTR;
                     dwAccess:DWORD;
                     dwFlags:DWORD;
                     dwContext:DWORD_PTR
                    ):HINTERNET; external WinInetDLL name 'FtpOpenFileW';
{$ELSE UNICODE}
function FtpOpenFile(hConnect:HINTERNET;
                     lpszFileName:LPCSTR;
                     dwAccess:DWORD;
                     dwFlags:DWORD;
                     dwContext:DWORD_PTR
                    ):HINTERNET; external WinInetDLL name 'FtpOpenFileA';
{$ENDIF UNICODE}

function FtpCreateDirectoryA(hConnect:HINTERNET;
                             lpszDirectory:LPCSTR
                            ):BOOL; external WinInetDLL name 'FtpCreateDirectoryA';

function FtpCreateDirectoryW(hConnect:HINTERNET;
                             lpszDirectory:LPCWSTR
                            ):BOOL; external WinInetDLL name 'FtpCreateDirectoryW';

{$IFDEF UNICODE}
function FtpCreateDirectory(hConnect:HINTERNET;
                            lpszDirectory:LPCWSTR
                           ):BOOL; external WinInetDLL name 'FtpCreateDirectoryW';
{$ELSE UNICODE}
function FtpCreateDirectory(hConnect:HINTERNET;
                            lpszDirectory:LPCSTR
                           ):BOOL; external WinInetDLL name 'FtpCreateDirectoryA';
{$ENDIF UNICODE}

function FtpRemoveDirectoryA(hConnect:HINTERNET;
                             lpszDirectory:LPCSTR
                            ):BOOL; external WinInetDLL name 'FtpRemoveDirectoryA';

function FtpRemoveDirectoryW(hConnect:HINTERNET;
                             lpszDirectory:LPCWSTR
                            ):BOOL; external WinInetDLL name 'FtpRemoveDirectoryW';

{$IFDEF UNICODE}
function FtpRemoveDirectory(hConnect:HINTERNET;
                            lpszDirectory:LPCWSTR
                           ):BOOL; external WinInetDLL name 'FtpRemoveDirectoryW';
{$ELSE UNICODE}
function FtpRemoveDirectory(hConnect:HINTERNET;
                            lpszDirectory:LPCSTR
                           ):BOOL; external WinInetDLL name 'FtpRemoveDirectoryA';
{$ENDIF UNICODE}

function FtpSetCurrentDirectoryA(hConnect:HINTERNET;
                                      lpszDirectory:LPCSTR
                                       ):BOOL; external WinInetDLL name 'FtpSetCurrentDirectoryA';

function FtpSetCurrentDirectoryW(hConnect:HINTERNET;
                                      lpszDirectory:LPCWSTR
                                       ):BOOL; external WinInetDLL name 'FtpSetCurrentDirectoryW';

{$IFDEF UNICODE}
function FtpSetCurrentDirectory(hConnect:HINTERNET;
                                     lpszDirectory:LPCWSTR
                                      ):BOOL; external WinInetDLL name 'FtpSetCurrentDirectoryW';
{$ELSE UNICODE}
function FtpSetCurrentDirectory(hConnect:HINTERNET;
                                     lpszDirectory:LPCSTR
                                      ):BOOL; external WinInetDLL name 'FtpSetCurrentDirectoryA';
{$ENDIF UNICODE}

function FtpGetCurrentDirectoryA(hConnect:HINTERNET;
                                        lpszCurrentDirectory:LPSTR;
                                        lpdwCurrentDirectory:LPDWORD
                                       ):BOOL; external WinInetDLL name 'FtpGetCurrentDirectoryA';

function FtpGetCurrentDirectoryW(hConnect:HINTERNET;
                                        lpszCurrentDirectory:LPWSTR;
                                        lpdwCurrentDirectory:LPDWORD
                                       ):BOOL; external WinInetDLL name 'FtpGetCurrentDirectoryW';

{$IFDEF UNICODE}
function FtpGetCurrentDirectory(hConnect:HINTERNET;
                                       lpszCurrentDirectory:LPWSTR;
                                       lpdwCurrentDirectory:LPDWORD
                                      ):BOOL; external WinInetDLL name 'FtpGetCurrentDirectoryW';
{$ELSE UNICODE}
function FtpGetCurrentDirectory(hConnect:HINTERNET;
                                       lpszCurrentDirectory:LPSTR;
                                       lpdwCurrentDirectory:LPDWORD
                                      ):BOOL; external WinInetDLL name 'FtpGetCurrentDirectoryA';
{$ENDIF UNICODE}

function FtpCommandA(hConnect:HINTERNET;
                         fExpectResponse:BOOL;
                         dwFlags:DWORD;
                         lpszCommand:LPCSTR;
                         dwContext:DWORD_PTR;
                         phFtpCommand:LPHINTERNET
                        ):BOOL; external WinInetDLL name 'FtpCommandA';

function FtpCommandW(hConnect:HINTERNET;
                         fExpectResponse:BOOL;
                         dwFlags:DWORD;
                         lpszCommand:LPCWSTR;
                         dwContext:DWORD_PTR;
                         phFtpCommand:LPHINTERNET
                        ):BOOL; external WinInetDLL name 'FtpCommandW';

{$IFDEF UNICODE}
function FtpCommand(hConnect:HINTERNET;
                        fExpectResponse:BOOL;
                        dwFlags:DWORD;
                        lpszCommand:LPCWSTR;
                        dwContext:DWORD_PTR;
                        phFtpCommand:LPHINTERNET
                       ):BOOL; external WinInetDLL name 'FtpCommandW';
{$ELSE UNICODE}
function FtpCommand(hConnect:HINTERNET;
                        fExpectResponse:BOOL;
                        dwFlags:DWORD;
                        lpszCommand:LPCSTR;
                        dwContext:DWORD_PTR;
                        phFtpCommand:LPHINTERNET
                       ):BOOL; external WinInetDLL name 'FtpCommandA';
{$ENDIF UNICODE}

function FtpGetFileSize(_hFile:HINTERNET;
                                lpdwFileSizeHigh:LPDWORD
                               ):DWORD; external WinInetDLL name 'FtpGetFileSize';




//
// Gopher
//

//
// manifests
//

//
// string field lengths (in characters, not bytes)
//
const
       MAX_GOPHER_DISPLAY_TEXT     = 128;
       MAX_GOPHER_SELECTOR_TEXT    = 256;
       MAX_GOPHER_HOST_NAME        = INTERNET_MAX_HOST_NAME_LENGTH;
       MAX_GOPHER_LOCATOR_LENGTH   = 1 +
                                     MAX_GOPHER_DISPLAY_TEXT +
                                     1 +
                                     MAX_GOPHER_SELECTOR_TEXT +
                                     1 +
                                     MAX_GOPHER_HOST_NAME +
                                     1 +
                                     INTERNET_MAX_PORT_NUMBER_LENGTH +
                                     1 +
                                     1 +
                                     2;


//
// structures/types
//

//
// GOPHER_FIND_DATA - returns the results of a GopherFindFirstFile()/
// InternetFindNextFile() request
//

type
     GOPHER_FIND_DATAA = record
        DisplayString:array[0..MAX_GOPHER_DISPLAY_TEXT] of char;
        GopherType:DWORD; // GOPHER_TYPE_, if known
        SizeLow:DWORD;
        SizeHigh:DWORD;
        LastModificationTime:FILETIME;
        Locator:array[0..MAX_GOPHER_LOCATOR_LENGTH] of char;
     end;
     LPGOPHER_FIND_DATAA = ^GOPHER_FIND_DATAA;

     GOPHER_FIND_DATAW = record
        DisplayString:array[0..MAX_GOPHER_DISPLAY_TEXT] of WCHAR;
        GopherType:DWORD; // GOPHER_TYPE_, if known
        SizeLow:DWORD;
        SizeHigh:DWORD;
        LastModificationTime:FILETIME;
        Locator:array[0..MAX_GOPHER_LOCATOR_LENGTH] of WCHAR;
     end;
     LPGOPHER_FIND_DATAW = ^GOPHER_FIND_DATAW;


{$IFDEF UNICODE}
type
     GOPHER_FIND_DATA = GOPHER_FIND_DATAW;
     LPGOPHER_FIND_DATA = LPGOPHER_FIND_DATAW;
{$ELSE UNICODE}
type
     GOPHER_FIND_DATA = GOPHER_FIND_DATAA;
     LPGOPHER_FIND_DATA = LPGOPHER_FIND_DATAA;
{$ENDIF UNICODE}

//
// manifests for GopherType
//
const
       GOPHER_TYPE_TEXT_FILE       = $00000001;
       GOPHER_TYPE_DIRECTORY       = $00000002;
       GOPHER_TYPE_CSO             = $00000004;
       GOPHER_TYPE_ERROR           = $00000008;
       GOPHER_TYPE_MAC_BINHEX      = $00000010;
       GOPHER_TYPE_DOS_ARCHIVE     = $00000020;
       GOPHER_TYPE_UNIX_UUENCODED  = $00000040;
       GOPHER_TYPE_INDEX_SERVER    = $00000080;
       GOPHER_TYPE_TELNET          = $00000100;
       GOPHER_TYPE_BINARY          = $00000200;
       GOPHER_TYPE_REDUNDANT       = $00000400;
       GOPHER_TYPE_TN3270          = $00000800;
       GOPHER_TYPE_GIF             = $00001000;
       GOPHER_TYPE_IMAGE           = $00002000;
       GOPHER_TYPE_BITMAP          = $00004000;
       GOPHER_TYPE_MOVIE           = $00008000;
       GOPHER_TYPE_SOUND           = $00010000;
       GOPHER_TYPE_HTML            = $00020000;
       GOPHER_TYPE_PDF             = $00040000;
       GOPHER_TYPE_CALENDAR        = $00080000;
       GOPHER_TYPE_INLINE          = $00100000;
       GOPHER_TYPE_UNKNOWN         = $20000000;
       GOPHER_TYPE_ASK             = $40000000;
       GOPHER_TYPE_GOPHER_PLUS     = $80000000;

//
// gopher type macros
//
function IS_GOPHER_FILE(_type:DWORD):BOOL;
function IS_GOPHER_DIRECTORY(_type:DWORD):BOOL;
function IS_GOPHER_PHONE_SERVER(_type:DWORD):BOOL;
function IS_GOPHER_ERROR(_type:DWORD):BOOL;
function IS_GOPHER_INDEX_SERVER(_type:DWORD):BOOL;
function IS_GOPHER_TELNET_SESSION(_type:DWORD):BOOL;
function IS_GOPHER_BACKUP_SERVER(_type:DWORD):BOOL;
function IS_GOPHER_TN3270_SESSION(_type:DWORD):BOOL;
function IS_GOPHER_ASK(_type:DWORD):BOOL;
function IS_GOPHER_PLUS(_type:DWORD):BOOL;
function IS_GOPHER_TYPE_KNOWN(_type:DWORD):BOOL;

//
// GOPHER_TYPE_FILE_MASK - use this to determine if a locator identifies a
// (known) file type
//
const
      GOPHER_TYPE_FILE_MASK       = GOPHER_TYPE_TEXT_FILE or
                                    GOPHER_TYPE_MAC_BINHEX or
                                    GOPHER_TYPE_DOS_ARCHIVE or
                                    GOPHER_TYPE_UNIX_UUENCODED or
                                    GOPHER_TYPE_BINARY or
                                    GOPHER_TYPE_GIF or
                                    GOPHER_TYPE_IMAGE or
                                    GOPHER_TYPE_BITMAP or
                                    GOPHER_TYPE_MOVIE or
                                    GOPHER_TYPE_SOUND or
                                    GOPHER_TYPE_HTML or
                                    GOPHER_TYPE_PDF or
                                    GOPHER_TYPE_CALENDAR or
                                    GOPHER_TYPE_INLINE;


//
// structured gopher attributes (as defined in gopher+ protocol document)
//
type
     GOPHER_ADMIN_ATTRIBUTE_TYPE = record
        Comment:LPCTSTR;
        EmailAddress:LPCTSTR;
     end;
     LPGOPHER_ADMIN_ATTRIBUTE_TYPE = ^GOPHER_ADMIN_ATTRIBUTE_TYPE;

type
     GOPHER_MOD_DATE_ATTRIBUTE_TYPE = record
        DateAndTime:FILETIME;
     end;
     LPGOPHER_MOD_DATE_ATTRIBUTE_TYPE = ^GOPHER_MOD_DATE_ATTRIBUTE_TYPE;

type
     GOPHER_TTL_ATTRIBUTE_TYPE = record
        Ttl:DWORD;
     end;
     LPGOPHER_TTL_ATTRIBUTE_TYPE = ^GOPHER_TTL_ATTRIBUTE_TYPE;

type
     GOPHER_SCORE_ATTRIBUTE_TYPE = record
        Score:longint;
     end;
     LPGOPHER_SCORE_ATTRIBUTE_TYPE = ^GOPHER_SCORE_ATTRIBUTE_TYPE;

type
     GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE = record
        LowerBound:longint;
        UpperBound:longint;
     end;
     LPGOPHER_SCORE_RANGE_ATTRIBUTE_TYPE = ^GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE;

type
     GOPHER_SITE_ATTRIBUTE_TYPE = record
        Site:LPCTSTR;
     end;
     LPGOPHER_SITE_ATTRIBUTE_TYPE = ^GOPHER_SITE_ATTRIBUTE_TYPE;

type
     GOPHER_ORGANIZATION_ATTRIBUTE_TYPE = record
        Organization:LPCTSTR;
     end;
     LPGOPHER_ORGANIZATION_ATTRIBUTE_TYPE = ^GOPHER_ORGANIZATION_ATTRIBUTE_TYPE;

type
     GOPHER_LOCATION_ATTRIBUTE_TYPE = record
        Location:LPCTSTR;
     end;
     LPGOPHER_LOCATION_ATTRIBUTE_TYPE = ^GOPHER_LOCATION_ATTRIBUTE_TYPE;

type
     GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE = record
        DegreesNorth:longint;
        MinutesNorth:longint;
        SecondsNorth:longint;
        DegreesEast:longint;
        MinutesEast:longint;
        SecondsEast:longint;
     end;
     LPGOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE = ^GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE;

type
     GOPHER_TIMEZONE_ATTRIBUTE_TYPE = record
        Zone:longint;
     end;
     LPGOPHER_TIMEZONE_ATTRIBUTE_TYPE = ^GOPHER_TIMEZONE_ATTRIBUTE_TYPE;

type
      GOPHER_PROVIDER_ATTRIBUTE_TYPE = record
         Provider:LPCTSTR;
      end;
      LPGOPHER_PROVIDER_ATTRIBUTE_TYPE = ^GOPHER_PROVIDER_ATTRIBUTE_TYPE;

type
     GOPHER_VERSION_ATTRIBUTE_TYPE = record
        Version:LPCTSTR;
     end;
     LPGOPHER_VERSION_ATTRIBUTE_TYPE = ^GOPHER_VERSION_ATTRIBUTE_TYPE;

type
     GOPHER_ABSTRACT_ATTRIBUTE_TYPE = record
        ShortAbstract:LPCTSTR;
        AbstractFile:LPCTSTR;
     end;
     LPGOPHER_ABSTRACT_ATTRIBUTE_TYPE = ^GOPHER_ABSTRACT_ATTRIBUTE_TYPE;

type
     GOPHER_VIEW_ATTRIBUTE_TYPE = record
        ContentType:LPCTSTR;
        Language:LPCTSTR;
        Size:DWORD;
     end;
     LPGOPHER_VIEW_ATTRIBUTE_TYPE = ^GOPHER_VIEW_ATTRIBUTE_TYPE;

type
     GOPHER_VERONICA_ATTRIBUTE_TYPE = record
        TreeWalk:BOOL;
     end;
     LPGOPHER_VERONICA_ATTRIBUTE_TYPE = ^GOPHER_VERONICA_ATTRIBUTE_TYPE;

type
     GOPHER_ASK_ATTRIBUTE_TYPE = record
        QuestionType:LPCTSTR;
        QuestionText:LPCTSTR;
     end;
     LPGOPHER_ASK_ATTRIBUTE_TYPE = ^GOPHER_ASK_ATTRIBUTE_TYPE;

//
// GOPHER_UNKNOWN_ATTRIBUTE_TYPE - this is returned if we retrieve an attribute
// that is not specified in the current gopher/gopher+ documentation. It is up
// to the application to parse the information
//

type
      GOPHER_UNKNOWN_ATTRIBUTE_TYPE = record
         Text:LPCTSTR;
      end;
      LPGOPHER_UNKNOWN_ATTRIBUTE_TYPE = ^GOPHER_UNKNOWN_ATTRIBUTE_TYPE;

//
// GOPHER_ATTRIBUTE_TYPE - returned in the user's buffer when an enumerated
// GopherGetAttribute call is made
//

type
     GOPHER_ATTRIBUTE_TYPE = record
        CategoryId:DWORD; // e.g. GOPHER_CATEGORY_ID_ADMIN
        AttributeId:DWORD;  // e.g. GOPHER_ATTRIBUTE_ID_ADMIN
       case longint of // AttributeType
           0: (Admin:GOPHER_ADMIN_ATTRIBUTE_TYPE);
           1: (ModDate:GOPHER_MOD_DATE_ATTRIBUTE_TYPE);
           2: (Ttl:GOPHER_TTL_ATTRIBUTE_TYPE);
           3: (Score:GOPHER_SCORE_ATTRIBUTE_TYPE);
           4: (ScoreRange:GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE);
           5: (Site:GOPHER_SITE_ATTRIBUTE_TYPE);
           6: (Organization:GOPHER_ORGANIZATION_ATTRIBUTE_TYPE);
           7: (Location:GOPHER_LOCATION_ATTRIBUTE_TYPE);
           8: (GeographicalLocation:GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE);
           9: (TimeZone:GOPHER_TIMEZONE_ATTRIBUTE_TYPE);
           10: (Provider:GOPHER_PROVIDER_ATTRIBUTE_TYPE);
           11: (Version:GOPHER_VERSION_ATTRIBUTE_TYPE);
           12: (_Abstract:GOPHER_ABSTRACT_ATTRIBUTE_TYPE);
           13: (View:GOPHER_VIEW_ATTRIBUTE_TYPE);
           14: (Veronica:GOPHER_VERONICA_ATTRIBUTE_TYPE);
           15: (Ask:GOPHER_ASK_ATTRIBUTE_TYPE);
           16: (Unknown:GOPHER_UNKNOWN_ATTRIBUTE_TYPE);
     end;
     LPGOPHER_ATTRIBUTE_TYPE = ^GOPHER_ATTRIBUTE_TYPE;

const
      MAX_GOPHER_CATEGORY_NAME    = 128;     // arbitrary
      MAX_GOPHER_ATTRIBUTE_NAME   = 128;     //     "
      MIN_GOPHER_ATTRIBUTE_LENGTH = 256;     //     "

//
// known gopher attribute categories. See below for ordinals
//
const
      GOPHER_INFO_CATEGORY        = '+INFO';
      GOPHER_ADMIN_CATEGORY       = '+ADMIN';
      GOPHER_VIEWS_CATEGORY       = '+VIEWS';
      GOPHER_ABSTRACT_CATEGORY    = '+ABSTRACT';
      GOPHER_VERONICA_CATEGORY    = '+VERONICA';

//
// known gopher attributes. These are the attribute names as defined in the
// gopher+ protocol document
//
const
      GOPHER_ADMIN_ATTRIBUTE      = 'Admin';
      GOPHER_MOD_DATE_ATTRIBUTE   = 'Mod-Date';
      GOPHER_TTL_ATTRIBUTE        = 'TTL';
      GOPHER_SCORE_ATTRIBUTE      = 'Score';
      GOPHER_RANGE_ATTRIBUTE      = 'Score-range';
      GOPHER_SITE_ATTRIBUTE       = 'Site';
      GOPHER_ORG_ATTRIBUTE        = 'Org';
      GOPHER_LOCATION_ATTRIBUTE   = 'Loc';
      GOPHER_GEOG_ATTRIBUTE       = 'Geog';
      GOPHER_TIMEZONE_ATTRIBUTE   = 'TZ';
      GOPHER_PROVIDER_ATTRIBUTE   = 'Provider';
      GOPHER_VERSION_ATTRIBUTE    = 'Version';
      GOPHER_ABSTRACT_ATTRIBUTE   = 'Abstract';
      GOPHER_VIEW_ATTRIBUTE       = 'View';
      GOPHER_TREEWALK_ATTRIBUTE   = 'treewalk';

//
// identifiers for attribute strings
//
const
      GOPHER_ATTRIBUTE_ID_BASE        = $ABCCCC00;

      GOPHER_CATEGORY_ID_ALL          = GOPHER_ATTRIBUTE_ID_BASE + 1;

      GOPHER_CATEGORY_ID_INFO         = GOPHER_ATTRIBUTE_ID_BASE + 2;
      GOPHER_CATEGORY_ID_ADMIN        = GOPHER_ATTRIBUTE_ID_BASE + 3;
      GOPHER_CATEGORY_ID_VIEWS        = GOPHER_ATTRIBUTE_ID_BASE + 4;
      GOPHER_CATEGORY_ID_ABSTRACT     = GOPHER_ATTRIBUTE_ID_BASE + 5;
      GOPHER_CATEGORY_ID_VERONICA     = GOPHER_ATTRIBUTE_ID_BASE + 6;
      GOPHER_CATEGORY_ID_ASK          = GOPHER_ATTRIBUTE_ID_BASE + 7;

      GOPHER_CATEGORY_ID_UNKNOWN      = GOPHER_ATTRIBUTE_ID_BASE + 8;

      GOPHER_ATTRIBUTE_ID_ALL         = GOPHER_ATTRIBUTE_ID_BASE + 9;

      GOPHER_ATTRIBUTE_ID_ADMIN       = GOPHER_ATTRIBUTE_ID_BASE + 10;
      GOPHER_ATTRIBUTE_ID_MOD_DATE    = GOPHER_ATTRIBUTE_ID_BASE + 11;
      GOPHER_ATTRIBUTE_ID_TTL         = GOPHER_ATTRIBUTE_ID_BASE + 12;
      GOPHER_ATTRIBUTE_ID_SCORE       = GOPHER_ATTRIBUTE_ID_BASE + 13;
      GOPHER_ATTRIBUTE_ID_RANGE       = GOPHER_ATTRIBUTE_ID_BASE + 14;
      GOPHER_ATTRIBUTE_ID_SITE        = GOPHER_ATTRIBUTE_ID_BASE + 15;
      GOPHER_ATTRIBUTE_ID_ORG         = GOPHER_ATTRIBUTE_ID_BASE + 16;
      GOPHER_ATTRIBUTE_ID_LOCATION    = GOPHER_ATTRIBUTE_ID_BASE + 17;
      GOPHER_ATTRIBUTE_ID_GEOG        = GOPHER_ATTRIBUTE_ID_BASE + 18;
      GOPHER_ATTRIBUTE_ID_TIMEZONE    = GOPHER_ATTRIBUTE_ID_BASE + 19;
      GOPHER_ATTRIBUTE_ID_PROVIDER    = GOPHER_ATTRIBUTE_ID_BASE + 20;
      GOPHER_ATTRIBUTE_ID_VERSION     = GOPHER_ATTRIBUTE_ID_BASE + 21;
      GOPHER_ATTRIBUTE_ID_ABSTRACT    = GOPHER_ATTRIBUTE_ID_BASE + 22;
      GOPHER_ATTRIBUTE_ID_VIEW        = GOPHER_ATTRIBUTE_ID_BASE + 23;
      GOPHER_ATTRIBUTE_ID_TREEWALK    = GOPHER_ATTRIBUTE_ID_BASE + 24;

      GOPHER_ATTRIBUTE_ID_UNKNOWN     = GOPHER_ATTRIBUTE_ID_BASE + 25;

//
// prototypes
//

function GopherCreateLocatorA(lpszHost:LPCSTR;
                                     nServerPort:INTERNET_PORT;
                                     lpszDisplayString:INTERNET_PORT;
                                     lpszSelectorString:LPCSTR;
                                     dwGopherType:DWORD;
                                     lpszLocator:LPSTR;
                                     lpdwBufferLength:LPDWORD):BOOL; external WinInetDLL name 'GopherCreateLocatorA';

function GopherCreateLocatorW(lpszHost:LPCWSTR;
                                     nServerPort:INTERNET_PORT;
                                     lpszDisplayString:INTERNET_PORT;
                                     lpszSelectorString:LPCWSTR;
                                     dwGopherType:DWORD;
                                     lpszLocator:LPWSTR;
                                     lpdwBufferLength:LPDWORD):BOOL; external WinInetDLL name 'GopherCreateLocatorW';

{$IFDEF UNICODE}
function GopherCreateLocator(lpszHost:LPCWSTR;
                                    nServerPort:INTERNET_PORT;
                                    lpszDisplayString:INTERNET_PORT;
                                    lpszSelectorString:LPCWSTR;
                                    dwGopherType:DWORD;
                                    lpszLocator:LPWSTR;
                                    lpdwBufferLength:LPDWORD):BOOL; external WinInetDLL name 'GopherCreateLocatorW';
{$ELSE UNICODE}
function GopherCreateLocator(lpszHost:LPCSTR;
                                    nServerPort:INTERNET_PORT;
                                    lpszDisplayString:INTERNET_PORT;
                                    lpszSelectorString:LPCSTR;
                                    dwGopherType:DWORD;
                                    lpszLocator:LPSTR;
                                    lpdwBufferLength:LPDWORD):BOOL; external WinInetDLL name 'GopherCreateLocatorA';
{$ENDIF UNICODE}

function GopherGetLocatorTypeA(lpszLocator:LPCSTR;
                                      lpdwGopherType:LPDWORD):BOOL; external WinInetDLL name 'GopherGetLocatorTypeA';

function GopherGetLocatorTypeW(lpszLocator:LPCWSTR;
                                      lpdwGopherType:LPDWORD):BOOL; external WinInetDLL name 'GopherGetLocatorTypeW';

{$IFDEF UNICODE}
function GopherGetLocatorType(lpszLocator:LPCWSTR;
                                     lpdwGopherType:LPDWORD):BOOL; external WinInetDLL name 'GopherGetLocatorTypeW';
{$ELSE UNICODE}
function GopherGetLocatorType(lpszLocator:LPCSTR;
                                     lpdwGopherType:LPDWORD):BOOL; external WinInetDLL name 'GopherGetLocatorTypeA';
{$ENDIF UNICODE}

function GopherFindFirstFileA(hConnect:HINTERNET;
                                         lpszLocator:LPCSTR;
                                         lpszSearchString:LPCSTR;
                                         lpFindData:LPGOPHER_FIND_DATAA;
                                         dwFlags:DWORD;
                                         dwContext:DWORD_PTR
                                        ):HINTERNET; external WinInetDLL name 'GopherFindFirstFileA';

function GopherFindFirstFileW(hConnect:HINTERNET;
                                         lpszLocator:LPCWSTR;
                                         lpszSearchString:LPCWSTR;
                                         lpFindData:LPGOPHER_FIND_DATAW;
                                         dwFlags:DWORD;
                                         dwContext:DWORD_PTR
                                        ):HINTERNET; external WinInetDLL name 'GopherFindFirstFileW';

{$IFDEF UNICODE}
function GopherFindFirstFile(hConnect:HINTERNET;
                                        lpszLocator:LPCWSTR;
                                        lpszSearchString:LPCWSTR;
                                        lpFindData:LPGOPHER_FIND_DATAW;
                                        dwFlags:DWORD;
                                        dwContext:DWORD_PTR
                                       ):HINTERNET; external WinInetDLL name 'GopherFindFirstFileW';
{$ELSE UNICODE}
function GopherFindFirstFile(hConnect:HINTERNET;
                                        lpszLocator:LPCSTR;
                                        lpszSearchString:LPCSTR;
                                        lpFindData:LPGOPHER_FIND_DATAA;
                                        dwFlags:DWORD;
                                        dwContext:DWORD_PTR
                                       ):HINTERNET; external WinInetDLL name 'GopherFindFirstFileA';
{$ENDIF UNICODE}

function GopherOpenFileA(hConnect:HINTERNET;
                                  lpszLocator:LPCSTR;
                                  lpszView:LPCSTR;
                                  dwFlags:DWORD;
                                  dwContext:DWORD_PTR
                                 ):HINTERNET; external WinInetDLL name 'GopherOpenFileA';

function GopherOpenFileW(hConnect:HINTERNET;
                                  lpszLocator:LPCWSTR;
                                  lpszView:LPCWSTR;
                                  dwFlags:DWORD;
                                  dwContext:DWORD_PTR
                                 ):HINTERNET; external WinInetDLL name 'GopherOpenFileW';

{$IFDEF UNICODE}
function GopherOpenFile(hConnect:HINTERNET;
                                 lpszLocator:LPCWSTR;
                                 lpszView:LPCWSTR;
                                 dwFlags:DWORD;
                                 dwContext:DWORD_PTR
                                ):HINTERNET; external WinInetDLL name 'GopherOpenFileW';
{$ELSE UNICODE}
function GopherOpenFile(hConnect:HINTERNET;
                                 lpszLocator:LPCSTR;
                        lpszView:LPCSTR;
                                 dwFlags:DWORD;
                                 dwContext:DWORD_PTR
                                ):HINTERNET; external WinInetDLL name 'GopherOpenFileA';
{$ENDIF UNICODE}

type
     GOPHER_ATTRIBUTE_ENUMERATOR = function(lpAttributeInfo:LPGOPHER_ATTRIBUTE_TYPE;
                                            dwError:DWORD):BOOL; cdecl;

function GopherGetAttributeA(hConnect:HINTERNET;
                                   lpszLocator:LPCSTR;
                                   lpszAttributeName:LPCSTR;
                                   lpBuffer:LPBYTE;
                                   dwBufferLength:DWORD;
                                   lpdwCharactersReturned:LPDWORD;
                                   lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR;
                                   dwContext:DWORD_PTR
                                  ):BOOL; external WinInetDLL name 'GopherGetAttributeA';

function GopherGetAttributeW(hConnect:HINTERNET;
                                   lpszLocator:LPCWSTR;
                                   lpszAttributeName:LPCWSTR;
                                   lpBuffer:LPBYTE;
                                   dwBufferLength:DWORD;
                                   lpdwCharactersReturned:LPDWORD;
                                   lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR;
                                   dwContext:DWORD_PTR
                                  ):BOOL; external WinInetDLL name 'GopherGetAttributeW';

{$IFDEF UNICODE}
function GopherGetAttribute(hConnect:HINTERNET;
                                  lpszLocator:LPCWSTR;
                                  lpszAttributeName:LPCWSTR;
                                  lpBuffer:LPBYTE;
                                  dwBufferLength:DWORD;
                                  lpdwCharactersReturned:LPDWORD;
                                  lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR;
                                  dwContext:DWORD_PTR
                                 ):BOOL; external WinInetDLL name 'GopherGetAttributeW';
{$ELSE UNICODE}
function GopherGetAttribute(hConnect:HINTERNET;
                                  lpszLocator:LPCSTR;
                                  lpszAttributeName:LPCSTR;
                                  lpBuffer:LPBYTE;
                                  dwBufferLength:DWORD;
                                  lpdwCharactersReturned:LPDWORD;
                                  lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR;
                                  dwContext:DWORD_PTR
                                 ):BOOL; external WinInetDLL name 'GopherGetAttributeA';
{$ENDIF UNICODE}


//
// HTTP
//

//
// manifests
//

//
// the default major/minor HTTP version numbers
//
const
      HTTP_MAJOR_VERSION      = 1;
      HTTP_MINOR_VERSION      = 0;

      HTTP_VERSIONA            = 'HTTP/1.0';
      HTTP_VERSIONW:WideString = 'HTTP/1.0';

{$IFDEF UNICODE}
      HTTP_VERSION:WideString = 'HTTP/1.0'; // HTTP_VERSIONW
{$ELSE UNICODE}
      HTTP_VERSION             = HTTP_VERSIONA;
{$ENDIF UNICODE}

//
// HttpQueryInfo info levels. Generally, there is one info level
// for each potential RFC822/HTTP/MIME header that an HTTP server
// may send as part of a request response.
//
// The HTTP_QUERY_RAW_HEADERS info level is provided for clients
// that choose to perform their own header parsing.
//
const
      HTTP_QUERY_MIME_VERSION                 = 0;
      HTTP_QUERY_CONTENT_TYPE                 = 1;
      HTTP_QUERY_CONTENT_TRANSFER_ENCODING    = 2;
      HTTP_QUERY_CONTENT_ID                   = 3;
      HTTP_QUERY_CONTENT_DESCRIPTION          = 4;
      HTTP_QUERY_CONTENT_LENGTH               = 5;
      HTTP_QUERY_CONTENT_LANGUAGE             = 6;
      HTTP_QUERY_ALLOW                        = 7;
      HTTP_QUERY_PUBLIC                       = 8;
      HTTP_QUERY_DATE                         = 9;
      HTTP_QUERY_EXPIRES                      = 10;
      HTTP_QUERY_LAST_MODIFIED                = 11;
      HTTP_QUERY_MESSAGE_ID                   = 12;
      HTTP_QUERY_URI                          = 13;
      HTTP_QUERY_DERIVED_FROM                 = 14;
      HTTP_QUERY_COST                         = 15;
      HTTP_QUERY_LINK                         = 16;
      HTTP_QUERY_PRAGMA                       = 17;
      HTTP_QUERY_VERSION                      = 18;  // special: part of status line
      HTTP_QUERY_STATUS_CODE                  = 19;  // special: part of status line
      HTTP_QUERY_STATUS_TEXT                  = 20;  // special: part of status line
      HTTP_QUERY_RAW_HEADERS                  = 21;  // special: all headers as ASCIIZ
      HTTP_QUERY_RAW_HEADERS_CRLF             = 22;  // special: all headers
      HTTP_QUERY_CONNECTION                   = 23;
      HTTP_QUERY_ACCEPT                       = 24;
      HTTP_QUERY_ACCEPT_CHARSET               = 25;
      HTTP_QUERY_ACCEPT_ENCODING              = 26;
      HTTP_QUERY_ACCEPT_LANGUAGE              = 27;
      HTTP_QUERY_AUTHORIZATION                = 28;
      HTTP_QUERY_CONTENT_ENCODING             = 29;
      HTTP_QUERY_FORWARDED                    = 30;
      HTTP_QUERY_FROM                         = 31;
      HTTP_QUERY_IF_MODIFIED_SINCE            = 32;
      HTTP_QUERY_LOCATION                     = 33;
      HTTP_QUERY_ORIG_URI                     = 34;
      HTTP_QUERY_REFERER                      = 35;
      HTTP_QUERY_RETRY_AFTER                  = 36;
      HTTP_QUERY_SERVER                       = 37;
      HTTP_QUERY_TITLE                        = 38;
      HTTP_QUERY_USER_AGENT                   = 39;
      HTTP_QUERY_WWW_AUTHENTICATE             = 40;
      HTTP_QUERY_PROXY_AUTHENTICATE           = 41;
      HTTP_QUERY_ACCEPT_RANGES                = 42;
      HTTP_QUERY_SET_COOKIE                   = 43;
      HTTP_QUERY_COOKIE                       = 44;
      HTTP_QUERY_REQUEST_METHOD               = 45;  // special: GET/POST etc.
      HTTP_QUERY_REFRESH                      = 46;
      HTTP_QUERY_CONTENT_DISPOSITION          = 47;

//
// HTTP 1.1 defined headers
//

      HTTP_QUERY_AGE                          = 48;
      HTTP_QUERY_CACHE_CONTROL                = 49;
      HTTP_QUERY_CONTENT_BASE                 = 50;
      HTTP_QUERY_CONTENT_LOCATION             = 51;
      HTTP_QUERY_CONTENT_MD5                  = 52;
      HTTP_QUERY_CONTENT_RANGE                = 53;
      HTTP_QUERY_ETAG                         = 54;
      HTTP_QUERY_HOST                         = 55;
      HTTP_QUERY_IF_MATCH                     = 56;
      HTTP_QUERY_IF_NONE_MATCH                = 57;
      HTTP_QUERY_IF_RANGE                     = 58;
      HTTP_QUERY_IF_UNMODIFIED_SINCE          = 59;
      HTTP_QUERY_MAX_FORWARDS                 = 60;
      HTTP_QUERY_PROXY_AUTHORIZATION          = 61;
      HTTP_QUERY_RANGE                        = 62;
      HTTP_QUERY_TRANSFER_ENCODING            = 63;
      HTTP_QUERY_UPGRADE                      = 64;
      HTTP_QUERY_VARY                         = 65;
      HTTP_QUERY_VIA                          = 66;
      HTTP_QUERY_WARNING                      = 67;
      HTTP_QUERY_EXPECT                       = 68;
      HTTP_QUERY_PROXY_CONNECTION             = 69;
      HTTP_QUERY_UNLESS_MODIFIED_SINCE        = 70;



      HTTP_QUERY_ECHO_REQUEST                 = 71;
      HTTP_QUERY_ECHO_REPLY                   = 72;

// These are the set of headers that should be added back to a request when
// re-doing a request after a RETRY_WITH response.
      HTTP_QUERY_ECHO_HEADERS                 = 73;
      HTTP_QUERY_ECHO_HEADERS_CRLF            = 74;

      HTTP_QUERY_PROXY_SUPPORT                = 75;
      HTTP_QUERY_AUTHENTICATION_INFO          = 76;
      HTTP_QUERY_PASSPORT_URLS                = 77;
      HTTP_QUERY_PASSPORT_CONFIG              = 78;

      HTTP_QUERY_MAX                          = 78;

//
// HTTP_QUERY_CUSTOM - if this special value is supplied as the dwInfoLevel
// parameter of HttpQueryInfo() then the lpBuffer parameter contains the name
// of the header we are to query
//

      HTTP_QUERY_CUSTOM                       = 65535;

//
// HTTP_QUERY_FLAG_REQUEST_HEADERS - if this bit is set in the dwInfoLevel
// parameter of HttpQueryInfo() then the request headers will be queried for the
// request information
//

      HTTP_QUERY_FLAG_REQUEST_HEADERS         = $80000000;

//
// HTTP_QUERY_FLAG_SYSTEMTIME - if this bit is set in the dwInfoLevel parameter
// of HttpQueryInfo() AND the header being queried contains date information,
// e.g. the "Expires:" header then lpBuffer will contain a SYSTEMTIME structure
// containing the date and time information converted from the header string
//

      HTTP_QUERY_FLAG_SYSTEMTIME              = $40000000;

//
// HTTP_QUERY_FLAG_NUMBER - if this bit is set in the dwInfoLevel parameter of
// HttpQueryInfo(), then the value of the header will be converted to a number
// before being returned to the caller, if applicable
//

      HTTP_QUERY_FLAG_NUMBER                  = $20000000;

//
// HTTP_QUERY_FLAG_COALESCE - combine the values from several headers of the
// same name into the output buffer
//

      HTTP_QUERY_FLAG_COALESCE                = $10000000;


      HTTP_QUERY_MODIFIER_FLAGS_MASK          = HTTP_QUERY_FLAG_REQUEST_HEADERS or
                                                HTTP_QUERY_FLAG_SYSTEMTIME or
                                                HTTP_QUERY_FLAG_NUMBER or
                                                HTTP_QUERY_FLAG_COALESCE;


      HTTP_QUERY_HEADER_MASK                  = (not HTTP_QUERY_MODIFIER_FLAGS_MASK);

//
// HTTP Response Status Codes:
//

      HTTP_STATUS_CONTINUE            = 100; // OK to continue with request
      HTTP_STATUS_SWITCH_PROTOCOLS    = 101; // server has switched protocols in upgrade header

      HTTP_STATUS_OK                  = 200; // request completed
      HTTP_STATUS_CREATED             = 201; // object created, reason = new URI
      HTTP_STATUS_ACCEPTED            = 202; // async completion (TBS)
      HTTP_STATUS_PARTIAL             = 203; // partial completion
      HTTP_STATUS_NO_CONTENT          = 204; // no info to return
      HTTP_STATUS_RESET_CONTENT       = 205; // request completed, but clear form
      HTTP_STATUS_PARTIAL_CONTENT     = 206; // partial GET furfilled

      HTTP_STATUS_AMBIGUOUS           = 300; // server couldn't decide what to return
      HTTP_STATUS_MOVED               = 301; // object permanently moved
      HTTP_STATUS_REDIRECT            = 302; // object temporarily moved
      HTTP_STATUS_REDIRECT_METHOD     = 303; // redirection w/ new access method
      HTTP_STATUS_NOT_MODIFIED        = 304; // if-modified-since was not modified
      HTTP_STATUS_USE_PROXY           = 305; // redirection to proxy, location header specifies proxy to use
      HTTP_STATUS_REDIRECT_KEEP_VERB  = 307; // HTTP/1.1: keep same verb

      HTTP_STATUS_BAD_REQUEST         = 400; // invalid syntax
      HTTP_STATUS_DENIED              = 401; // access denied
      HTTP_STATUS_PAYMENT_REQ         = 402; // payment required
      HTTP_STATUS_FORBIDDEN           = 403; // request forbidden
      HTTP_STATUS_NOT_FOUND           = 404; // object not found
      HTTP_STATUS_BAD_METHOD          = 405; // method is not allowed
      HTTP_STATUS_NONE_ACCEPTABLE     = 406; // no response acceptable to client found
      HTTP_STATUS_PROXY_AUTH_REQ      = 407; // proxy authentication required
      HTTP_STATUS_REQUEST_TIMEOUT     = 408; // server timed out waiting for request
      HTTP_STATUS_CONFLICT            = 409; // user should resubmit with more info
      HTTP_STATUS_GONE                = 410; // the resource is no longer available
      HTTP_STATUS_LENGTH_REQUIRED     = 411; // the server refused to accept request w/o a length
      HTTP_STATUS_PRECOND_FAILED      = 412; // precondition given in request failed
      HTTP_STATUS_REQUEST_TOO_LARGE   = 413; // request entity was too large
      HTTP_STATUS_URI_TOO_LONG        = 414; // request URI too long
      HTTP_STATUS_UNSUPPORTED_MEDIA   = 415; // unsupported media type
      HTTP_STATUS_RETRY_WITH          = 449; // retry after doing the appropriate action.

      HTTP_STATUS_SERVER_ERROR        = 500; // internal server error
      HTTP_STATUS_NOT_SUPPORTED       = 501; // required not supported
      HTTP_STATUS_BAD_GATEWAY         = 502; // error response received from gateway
      HTTP_STATUS_SERVICE_UNAVAIL     = 503; // temporarily overloaded
      HTTP_STATUS_GATEWAY_TIMEOUT     = 504; // timed out waiting for gateway
      HTTP_STATUS_VERSION_NOT_SUP     = 505; // HTTP version not supported

      HTTP_STATUS_FIRST               = HTTP_STATUS_CONTINUE;
      HTTP_STATUS_LAST                = HTTP_STATUS_VERSION_NOT_SUP;

//
// prototypes
//

function HttpOpenRequestA(hConnect:HINTERNET;
                                    lpszVerb:LPCSTR;
                                    lpszObjectName:LPCSTR;
                                    lpszVersion:LPCSTR;
                                    lpszReferrer:LPCSTR;
                                    lplpszAcceptTypes:PLPSTR;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'HttpOpenRequestA';

function HttpOpenRequestW(hConnect:HINTERNET;
                                    lpszVerb:LPCWSTR;
                                    lpszObjectName:LPCWSTR;
                                    lpszVersion:LPCWSTR;
                                    lpszReferrer:LPCWSTR;
                                    lplpszAcceptTypes:PLPWSTR;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):HINTERNET; external WinInetDLL name 'HttpOpenRequestW';

{$IFDEF UNICODE}
function HttpOpenRequest(hConnect:HINTERNET;
                                   lpszVerb:LPCWSTR;
                         lpszObjectName:LPCWSTR;
                         lpszVersion:LPCWSTR;
                         lpszReferrer:LPCWSTR;
                         lplpszAcceptTypes:PLPWSTR;
                         dwFlags:DWORD;
                         dwContext:DWORD_PTR
                        ):HINTERNET; external WinInetDLL name 'HttpOpenRequestW';
{$ELSE UNICODE}
function HttpOpenRequest(hConnect:HINTERNET;
                                   lpszVerb:LPCSTR;
                         lpszObjectName:LPCSTR;
                         lpszVersion:LPCSTR;
                         lpszReferrer:LPCSTR;
                         lplpszAcceptTypes:PLPSTR;
                         dwFlags:DWORD;
                         dwContext:DWORD_PTR
                        ):HINTERNET; external WinInetDLL name 'HttpOpenRequestA';
{$ENDIF UNICODE}

function HttpAddRequestHeadersA(hRequest:HINTERNET;
                                       lpszHeaders:LPCSTR;
                                       dwHeadersLength:DWORD;
                                       dwModifiers:DWORD
                                      ):BOOL; external WinInetDLL name 'HttpAddRequestHeadersA';

function HttpAddRequestHeadersW(hRequest:HINTERNET;
                                       lpszHeaders:LPCWSTR;
                                       dwHeadersLength:DWORD;
                                       dwModifiers:DWORD
                                      ):BOOL; external WinInetDLL name 'HttpAddRequestHeadersW';

{$IFDEF UNICODE}
function HttpAddRequestHeaders(hRequest:HINTERNET;
                                      lpszHeaders:LPCWSTR;
                                      dwHeadersLength:DWORD;
                                      dwModifiers:DWORD
                                     ):BOOL; external WinInetDLL name 'HttpAddRequestHeadersW';
{$ELSE UNICODE}
function HttpAddRequestHeaders(hRequest:HINTERNET;
                                      lpszHeaders:LPCSTR;
                                      dwHeadersLength:DWORD;
                                      dwModifiers:DWORD
                                     ):BOOL; external WinInetDLL name 'HttpAddRequestHeadersA';
{$ENDIF UNICODE}

//
// values for dwModifiers parameter of HttpAddRequestHeaders()
//
const
      HTTP_ADDREQ_INDEX_MASK      = $0000FFFF;
      HTTP_ADDREQ_FLAGS_MASK      = $FFFF0000;

//
// HTTP_ADDREQ_FLAG_ADD_IF_NEW - the header will only be added if it doesn't
// already exist
//
const
      HTTP_ADDREQ_FLAG_ADD_IF_NEW = $10000000;

//
// HTTP_ADDREQ_FLAG_ADD - if HTTP_ADDREQ_FLAG_REPLACE is set but the header is
// not found then if this flag is set, the header is added anyway, so long as
// there is a valid header-value
//
const
      HTTP_ADDREQ_FLAG_ADD        = $20000000;

//
// HTTP_ADDREQ_FLAG_COALESCE - coalesce headers with same name. e.g.
// "Accept: text/*" and "Accept: audio/*" with this flag results in a single
// header: "Accept: text/*, audio/*"
//
const
      HTTP_ADDREQ_FLAG_COALESCE_WITH_COMMA       = $40000000;
      HTTP_ADDREQ_FLAG_COALESCE_WITH_SEMICOLON   = $01000000;
      HTTP_ADDREQ_FLAG_COALESCE                  = HTTP_ADDREQ_FLAG_COALESCE_WITH_COMMA;

//
// HTTP_ADDREQ_FLAG_REPLACE - replaces the specified header. Only one header can
// be supplied in the buffer. If the header to be replaced is not the first
// in a list of headers with the same name, then the relative index should be
// supplied in the low 8 bits of the dwModifiers parameter. If the header-value
// part is missing, then the header is removed
//
const
      HTTP_ADDREQ_FLAG_REPLACE    = $80000000;

function HttpSendRequestA(hRequest:HINTERNET;
                                lpszHeaders:LPCSTR;
                                dwHeadersLength:DWORD;
                                lpOptional:LPVOID;
                                dwOptionalLength:DWORD
                               ):BOOL; external WinInetDLL name 'HttpSendRequestA';

function HttpSendRequestW(hRequest:HINTERNET;
                                lpszHeaders:LPCWSTR;
                                dwHeadersLength:DWORD;
                                lpOptional:LPVOID;
                                dwOptionalLength:DWORD
                               ):BOOL; external WinInetDLL name 'HttpSendRequestW';

{$IFDEF UNICODE}
function HttpSendRequest(hRequest:HINTERNET;
                               lpszHeaders:LPCWSTR;
                               dwHeadersLength:DWORD;
                               lpOptional:LPVOID;
                               dwOptionalLength:DWORD
                              ):BOOL; external WinInetDLL name 'HttpSendRequestW';
{$ELSE UNICODE}
function HttpSendRequest(hRequest:HINTERNET;
                               lpszHeaders:LPCSTR;
                               dwHeadersLength:DWORD;
                               lpOptional:LPVOID;
                               dwOptionalLength:DWORD
                              ):BOOL; external WinInetDLL name 'HttpSendRequestA';
{$ENDIF UNICODE}

function HttpSendRequestExA(hRequest:HINTERNET;
                                     lpBuffersIn:LPINTERNET_BUFFERSA;
                                     lpBuffersOut:LPINTERNET_BUFFERSA;
                                     dwFlags:DWORD;
                                     dwContext:DWORD_PTR
                                    ):BOOL; external WinInetDLL name 'HttpSendRequestExA';

function HttpSendRequestExW(hRequest:HINTERNET;
                                     lpBuffersIn:LPINTERNET_BUFFERSW;
                                     lpBuffersOut:LPINTERNET_BUFFERSW;
                                     dwFlags:DWORD;
                                     dwContext:DWORD_PTR
                                    ):BOOL; external WinInetDLL name 'HttpSendRequestExW';

{$IFDEF UNICODE}
function HttpSendRequestEx(hRequest:HINTERNET;
                                    lpBuffersIn:LPINTERNET_BUFFERS;
                                    lpBuffersOut:LPINTERNET_BUFFERS;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):BOOL; external WinInetDLL name 'HttpSendRequestExW';
{$ELSE UNICODE}
function HttpSendRequestEx(hRequest:HINTERNET;
                                    lpBuffersIn:LPINTERNET_BUFFERS;
                                    lpBuffersOut:LPINTERNET_BUFFERS;
                                    dwFlags:DWORD;
                                    dwContext:DWORD_PTR
                                   ):BOOL; external WinInetDLL name 'HttpSendRequestExA';
{$ENDIF UNICODE}

//
// flags for HttpSendRequestEx(), HttpEndRequest()
//
const
      HSR_ASYNC       = WININET_API_FLAG_ASYNC;          // force async
      HSR_SYNC        = WININET_API_FLAG_SYNC;           // force sync
      HSR_USE_CONTEXT = WININET_API_FLAG_USE_CONTEXT;    // use dwContext value
      HSR_INITIATE    = $00000008;                      // iterative operation (completed by HttpEndRequest)
      HSR_DOWNLOAD    = $00000010;                      // download to file
      HSR_CHUNKED     = $00000020;                      // operation is send of chunked data

function HttpEndRequestA(hRequest:HINTERNET;
                                 lpBuffersOut:LPINTERNET_BUFFERSA;
                                 dwFlags:DWORD;
                                 dwContext:DWORD_PTR
                                ):BOOL; external WinInetDLL name 'HttpEndRequestA';

function HttpEndRequestW(hRequest:HINTERNET;
                                 lpBuffersOut:LPINTERNET_BUFFERSW;
                                 dwFlags:DWORD;
                                 dwContext:DWORD_PTR
                                ):BOOL; external WinInetDLL name 'HttpEndRequestW';

{$IFDEF UNICODE}
function HttpEndRequest(hRequest:HINTERNET;
                                lpBuffersOut:LPINTERNET_BUFFERS;
                               dwFlags:DWORD;
                               dwContext:DWORD_PTR
                              ):BOOL; external WinInetDLL name 'HttpEndRequestW';
{$ELSE UNICODE}
function HttpEndRequest(hRequest:HINTERNET;
                                lpBuffersOut:LPINTERNET_BUFFERS;
                               dwFlags:DWORD;
                               dwContext:DWORD_PTR
                              ):BOOL; external WinInetDLL name 'HttpEndRequestA';
{$ENDIF UNICODE}

function HttpQueryInfoA(hRequest:HINTERNET;
                             dwInfoLevel:DWORD;
                             lpBuffer:LPVOID;
                             lpdwBufferLength:LPDWORD;
                             lpdwIndex:LPDWORD
                            ):BOOL; external WinInetDLL name 'HttpQueryInfoA';

function HttpQueryInfoW(hRequest:HINTERNET;
                             dwInfoLevel:DWORD;
                             lpBuffer:LPVOID;
                             lpdwBufferLength:LPDWORD;
                             lpdwIndex:LPDWORD
                            ):BOOL; external WinInetDLL name 'HttpQueryInfoW';

{$IFDEF UNICODE}
function HttpQueryInfo(hRequest:HINTERNET;
                            dwInfoLevel:DWORD;
                            lpBuffer:LPVOID;
                            lpdwBufferLength:LPDWORD;
                            lpdwIndex:LPDWORD
                           ):BOOL; external WinInetDLL name 'HttpQueryInfoW';
{$ELSE UNICODE}
function HttpQueryInfo(hRequest:HINTERNET;
                            dwInfoLevel:DWORD;
                            lpBuffer:LPVOID;
                            lpdwBufferLength:LPDWORD;
                            lpdwIndex:LPDWORD
                           ):BOOL; external WinInetDLL name 'HttpQueryInfoA';
{$ENDIF UNICODE}


//
// Cookie APIs
//
const
      INTERNET_COOKIE_IS_SECURE       = $01;
      INTERNET_COOKIE_IS_SESSION      = $02;
      INTERNET_COOKIE_THIRD_PARTY     = $10;
      INTERNET_COOKIE_PROMPT_REQUIRED = $20;
      INTERNET_COOKIE_EVALUATE_P3P    = $40;
      INTERNET_COOKIE_APPLY_P3P       = $80;

      INTERNET_COOKIE_P3P_ENABLED     = $100;
      INTERNET_COOKIE_IS_RESTRICTED   = $200;
      INTERNET_COOKIE_IE6             = $400;
      INTERNET_COOKIE_IS_LEGACY       = $800;

function InternetSetCookieA(lpszUrl:LPCSTR;
                                  lpszCookieName:LPCSTR;
                                  lpszCookieData:LPCSTR
                                 ):BOOL; external WinInetDLL name 'InternetSetCookieA';

function InternetSetCookieW(lpszUrl:LPCWSTR;
                                  lpszCookieName:LPCWSTR;
                                  lpszCookieData:LPCWSTR
                                 ):BOOL; external WinInetDLL name 'InternetSetCookieW';

{$IFDEF UNICODE}
function InternetSetCookie(lpszUrl:LPCWSTR;
                                 lpszCookieName:LPCWSTR;
                                 lpszCookieData:LPCWSTR
                                ):BOOL; external WinInetDLL name 'InternetSetCookieW';
{$ELSE UNICODE}
function InternetSetCookie(lpszUrl:LPCSTR;
                                 lpszCookieName:LPCSTR;
                                 lpszCookieData:LPCSTR
                                ):BOOL; external WinInetDLL name 'InternetSetCookieA';
{$ENDIF UNICODE}

function InternetGetCookieA(lpszUrl:LPCSTR;
                                  lpszCookieName:LPCSTR;
                                  lpCookieData:LPCSTR;
                                  lpdwSize:LPDWORD
                                 ):BOOL; external WinInetDLL name 'InternetGetCookieA';

function InternetGetCookieW(lpszUrl:LPCWSTR;
                                  lpszCookieName:LPCWSTR;
                                  lpCookieData:LPCWSTR;
                                  lpdwSize:LPDWORD
                                 ):BOOL; external WinInetDLL name 'InternetGetCookieW';

{$IFDEF UNICODE}
function InternetGetCookie(lpszUrl:LPCWSTR;
                                 lpszCookieName:LPCWSTR;
                                 lpCookieData:LPCWSTR;
                                 lpdwSize:LPDWORD
                                ):BOOL; external WinInetDLL name 'InternetGetCookieW';
{$ELSE UNICODE}
function InternetGetCookie(lpszUrl:LPCSTR;
                                 lpszCookieName:LPCSTR;
                                 lpCookieData:LPCSTR;
                                 lpdwSize:LPDWORD
                                ):BOOL; external WinInetDLL name 'InternetGetCookieA';
{$ENDIF UNICODE}

function InternetSetCookieExA(lpszUrl:LPCSTR;
                                        lpszCookieName:LPCSTR;
                                        lpszCookieData:LPCSTR;
                                        dwFlags:DWORD;
                                        dwReserved:DWORD_PTR
                                       ):DWORD; external WinInetDLL name 'InternetSetCookieExA';

function InternetSetCookieExW(lpszUrl:LPCWSTR;
                                        lpszCookieName:LPCWSTR;
                                        lpszCookieData:LPCWSTR;
                                        dwFlags:DWORD;
                                        dwReserved:DWORD_PTR
                                       ):DWORD; external WinInetDLL name 'InternetSetCookieExW';

{$IFDEF UNICODE}
function InternetSetCookieEx(lpszUrl:LPCWSTR;
                                       lpszCookieName:LPCWSTR;
                                       lpszCookieData:LPCWSTR;
                                       dwFlags:DWORD;
                                       dwReserved:DWORD_PTR
                                      ):DWORD; external WinInetDLL name 'InternetSetCookieExW';
{$ELSE UNICODE}
function InternetSetCookieEx(lpszUrl:LPCSTR;
                                       lpszCookieName:LPCSTR;
                                       lpszCookieData:LPCSTR;
                                       dwFlags:DWORD;
                                       dwReserved:DWORD_PTR
                                      ):DWORD; external WinInetDLL name 'InternetSetCookieExA';
{$ENDIF UNICODE}


//
// offline browsing
//

function InternetAttemptConnect(dwReserved:DWORD):DWORD; external WinInetDLL name 'InternetAttemptConnect';

function InternetCheckConnectionA(lpszUrl:LPCSTR;
                                          dwFlags:DWORD;
                                          dwReserved:DWORD
                                         ):BOOL; external WinInetDLL name 'InternetCheckConnectionA';

function InternetCheckConnectionW(lpszUrl:LPCWSTR;
                                          dwFlags:DWORD;
                                          dwReserved:DWORD
                                         ):BOOL; external WinInetDLL name 'InternetCheckConnectionW';

{$IFDEF UNICODE}
function InternetCheckConnection(lpszUrl:LPCWSTR;
                                         dwFlags:DWORD;
                                         dwReserved:DWORD
                                        ):BOOL; external WinInetDLL name 'InternetCheckConnectionW';
{$ELSE UNICODE}
function InternetCheckConnection(lpszUrl:LPCSTR;
                                         dwFlags:DWORD;
                                         dwReserved:DWORD
                                        ):BOOL; external WinInetDLL name 'InternetCheckConnectionA';
{$ENDIF UNICODE}

const
      FLAG_ICC_FORCE_CONNECTION       = $00000001;

//
// Internet UI
//

//
// InternetErrorDlg - Provides UI for certain Errors.
//
const
      FLAGS_ERROR_UI_FILTER_FOR_ERRORS        = $01;
      FLAGS_ERROR_UI_FLAGS_CHANGE_OPTIONS     = $02;
      FLAGS_ERROR_UI_FLAGS_GENERATE_DATA      = $04;
      FLAGS_ERROR_UI_FLAGS_NO_UI              = $08;
      FLAGS_ERROR_UI_SERIALIZE_DIALOGS        = $10;

//
// If SERIALIZE_DIALOGS flag set, client should implement thread-safe non-blocking callback...
//

type
     PFN_AUTH_NOTIFY = function(dwContext:DWORD_PTR; // as passed to InternetErrorDlg
                                dwReturn:DWORD;      // error code: success, resend, or cancel
                                lpResrved:LPVOID     // reserved: will be set to null
                               ):DWORD; cdecl;

//
// ... and last parameter of InternetErrorDlg should point to...
//

type
     INTERNET_AUTH_NOTIFY_DATA = record
       cbStruct:DWORD;    // size of this structure
        dwOptions:DWORD;    // reserved: must set to 0
        pfnNotify:PFN_AUTH_NOTIFY;    // notification callback to retry InternetErrorDlg
       dwContext:DWORD_PTR;   // context to pass to to notification function
     end;


function ResumeSuspendedDownload(hRequest:HINTERNET;
                                           dwResultCode:DWORD
                                          ):BOOL; external WinInetDLL name 'ResumeSuspendedDownload';

function InternetErrorDlg(_hWnd:HWND;
                                   hRequest:HINTERNET;
                                   dwError:DWORD;
                                   dwFlags:DWORD;
                                   lppvData:PPVOID
                                  ):DWORD; external WinInetDLL name 'InternetErrorDlg';

function InternetConfirmZoneCrossingA(_hWnd:HWND;
                                                  szUrlPrev:LPSTR;
                                                  szUrlNew:LPSTR;
                                                  bPost:BOOL
                                                 ):DWORD; external WinInetDLL name 'InternetConfirmZoneCrossingA';

function InternetConfirmZoneCrossingW(_hWnd:HWND;
                                                  szUrlPrev:LPWSTR;
                                                  szUrlNew:LPWSTR;
                                                  bPost:BOOL
                                                 ):DWORD; external WinInetDLL name 'InternetConfirmZoneCrossingW';

{$IFDEF UNICODE}
function InternetConfirmZoneCrossing(_hWnd:HWND;
                                                 szUrlPrev:LPWSTR;
                                                 szUrlNew:LPWSTR;
                                                 bPost:BOOL
                                                ):DWORD; external WinInetDLL name 'InternetConfirmZoneCrossingW';
{$ELSE UNICODE}
{$IFDEF WINX32}
function InternetConfirmZoneCrossing(_hWnd:HWND;
                                                 szUrlPrev:LPSTR;
                                                 szUrlNew:LPSTR;
                                                 bPost:BOOL
                                                ):DWORD; external WinInetDLL name 'InternetConfirmZoneCrossingA';
{$ELSE WINX32}
function InternetConfirmZoneCrossing(_hWnd:HWND;
                                                 szUrlPrev:LPSTR;
                                                 szUrlNew:LPSTR;
                                                 bPost:BOOL
                                                ):DWORD; external WinInetDLL name 'InternetConfirmZoneCrossing';
{$ENDIF WINX32}
{$ENDIF UNICODE}

//
// Internet API error returns
//
const
      INTERNET_ERROR_BASE                     = 12000;

      ERROR_INTERNET_OUT_OF_HANDLES           = INTERNET_ERROR_BASE + 1;
      ERROR_INTERNET_TIMEOUT                  = INTERNET_ERROR_BASE + 2;
      ERROR_INTERNET_EXTENDED_ERROR           = INTERNET_ERROR_BASE + 3;
      ERROR_INTERNET_INTERNAL_ERROR           = INTERNET_ERROR_BASE + 4;
      ERROR_INTERNET_INVALID_URL              = INTERNET_ERROR_BASE + 5;
      ERROR_INTERNET_UNRECOGNIZED_SCHEME      = INTERNET_ERROR_BASE + 6;
      ERROR_INTERNET_NAME_NOT_RESOLVED        = INTERNET_ERROR_BASE + 7;
      ERROR_INTERNET_PROTOCOL_NOT_FOUND       = INTERNET_ERROR_BASE + 8;
      ERROR_INTERNET_INVALID_OPTION           = INTERNET_ERROR_BASE + 9;
      ERROR_INTERNET_BAD_OPTION_LENGTH        = INTERNET_ERROR_BASE + 10;
      ERROR_INTERNET_OPTION_NOT_SETTABLE      = INTERNET_ERROR_BASE + 11;
      ERROR_INTERNET_SHUTDOWN                 = INTERNET_ERROR_BASE + 12;
      ERROR_INTERNET_INCORRECT_USER_NAME      = INTERNET_ERROR_BASE + 13;
      ERROR_INTERNET_INCORRECT_PASSWORD       = INTERNET_ERROR_BASE + 14;
      ERROR_INTERNET_LOGIN_FAILURE            = INTERNET_ERROR_BASE + 15;
      ERROR_INTERNET_INVALID_OPERATION        = INTERNET_ERROR_BASE + 16;
      ERROR_INTERNET_OPERATION_CANCELLED      = INTERNET_ERROR_BASE + 17;
      ERROR_INTERNET_INCORRECT_HANDLE_TYPE    = INTERNET_ERROR_BASE + 18;
      ERROR_INTERNET_INCORRECT_HANDLE_STATE   = INTERNET_ERROR_BASE + 19;
      ERROR_INTERNET_NOT_PROXY_REQUEST        = INTERNET_ERROR_BASE + 20;
      ERROR_INTERNET_REGISTRY_VALUE_NOT_FOUND = INTERNET_ERROR_BASE + 21;
      ERROR_INTERNET_BAD_REGISTRY_PARAMETER   = INTERNET_ERROR_BASE + 22;
      ERROR_INTERNET_NO_DIRECT_ACCESS         = INTERNET_ERROR_BASE + 23;
      ERROR_INTERNET_NO_CONTEXT               = INTERNET_ERROR_BASE + 24;
      ERROR_INTERNET_NO_CALLBACK              = INTERNET_ERROR_BASE + 25;
      ERROR_INTERNET_REQUEST_PENDING          = INTERNET_ERROR_BASE + 26;
      ERROR_INTERNET_INCORRECT_FORMAT         = INTERNET_ERROR_BASE + 27;
      ERROR_INTERNET_ITEM_NOT_FOUND           = INTERNET_ERROR_BASE + 28;
      ERROR_INTERNET_CANNOT_CONNECT           = INTERNET_ERROR_BASE + 29;
      ERROR_INTERNET_CONNECTION_ABORTED       = INTERNET_ERROR_BASE + 30;
      ERROR_INTERNET_CONNECTION_RESET         = INTERNET_ERROR_BASE + 31;
      ERROR_INTERNET_FORCE_RETRY              = INTERNET_ERROR_BASE + 32;
      ERROR_INTERNET_INVALID_PROXY_REQUEST    = INTERNET_ERROR_BASE + 33;
      ERROR_INTERNET_NEED_UI                  = INTERNET_ERROR_BASE + 34;

      ERROR_INTERNET_HANDLE_EXISTS            = INTERNET_ERROR_BASE + 36;
      ERROR_INTERNET_SEC_CERT_DATE_INVALID    = INTERNET_ERROR_BASE + 37;
      ERROR_INTERNET_SEC_CERT_CN_INVALID      = INTERNET_ERROR_BASE + 38;
      ERROR_INTERNET_HTTP_TO_HTTPS_ON_REDIR   = INTERNET_ERROR_BASE + 39;
      ERROR_INTERNET_HTTPS_TO_HTTP_ON_REDIR   = INTERNET_ERROR_BASE + 40;
      ERROR_INTERNET_MIXED_SECURITY           = INTERNET_ERROR_BASE + 41;
      ERROR_INTERNET_CHG_POST_IS_NON_SECURE   = INTERNET_ERROR_BASE + 42;
      ERROR_INTERNET_POST_IS_NON_SECURE       = INTERNET_ERROR_BASE + 43;
      ERROR_INTERNET_CLIENT_AUTH_CERT_NEEDED  = INTERNET_ERROR_BASE + 44;
      ERROR_INTERNET_INVALID_CA               = INTERNET_ERROR_BASE + 45;
      ERROR_INTERNET_CLIENT_AUTH_NOT_SETUP    = INTERNET_ERROR_BASE + 46;
      ERROR_INTERNET_ASYNC_THREAD_FAILED      = INTERNET_ERROR_BASE + 47;
      ERROR_INTERNET_REDIRECT_SCHEME_CHANGE   = INTERNET_ERROR_BASE + 48;
      ERROR_INTERNET_DIALOG_PENDING           = INTERNET_ERROR_BASE + 49;
      ERROR_INTERNET_RETRY_DIALOG             = INTERNET_ERROR_BASE + 50;
      ERROR_INTERNET_HTTPS_HTTP_SUBMIT_REDIR  = INTERNET_ERROR_BASE + 52;
      ERROR_INTERNET_INSERT_CDROM             = INTERNET_ERROR_BASE + 53;
      ERROR_INTERNET_FORTEZZA_LOGIN_NEEDED    = INTERNET_ERROR_BASE + 54;
      ERROR_INTERNET_SEC_CERT_ERRORS          = INTERNET_ERROR_BASE + 55;
      ERROR_INTERNET_SEC_CERT_NO_REV          = INTERNET_ERROR_BASE + 56;
      ERROR_INTERNET_SEC_CERT_REV_FAILED      = INTERNET_ERROR_BASE + 57;

//
// FTP API errors
//
const
      ERROR_FTP_TRANSFER_IN_PROGRESS          = INTERNET_ERROR_BASE + 110;
      ERROR_FTP_DROPPED                       = INTERNET_ERROR_BASE + 111;
      ERROR_FTP_NO_PASSIVE_MODE               = INTERNET_ERROR_BASE + 112;

//
// gopher API errors
//
const
      ERROR_GOPHER_PROTOCOL_ERROR             = INTERNET_ERROR_BASE + 130;
      ERROR_GOPHER_NOT_FILE                   = INTERNET_ERROR_BASE + 131;
      ERROR_GOPHER_DATA_ERROR                 = INTERNET_ERROR_BASE + 132;
      ERROR_GOPHER_END_OF_DATA                = INTERNET_ERROR_BASE + 133;
      ERROR_GOPHER_INVALID_LOCATOR            = INTERNET_ERROR_BASE + 134;
      ERROR_GOPHER_INCORRECT_LOCATOR_TYPE     = INTERNET_ERROR_BASE + 135;
      ERROR_GOPHER_NOT_GOPHER_PLUS            = INTERNET_ERROR_BASE + 136;
      ERROR_GOPHER_ATTRIBUTE_NOT_FOUND        = INTERNET_ERROR_BASE + 137;
      ERROR_GOPHER_UNKNOWN_LOCATOR            = INTERNET_ERROR_BASE + 138;

//
// HTTP API errors
//
const
      ERROR_HTTP_HEADER_NOT_FOUND             = INTERNET_ERROR_BASE + 150;
      ERROR_HTTP_DOWNLEVEL_SERVER             = INTERNET_ERROR_BASE + 151;
      ERROR_HTTP_INVALID_SERVER_RESPONSE      = INTERNET_ERROR_BASE + 152;
      ERROR_HTTP_INVALID_HEADER               = INTERNET_ERROR_BASE + 153;
      ERROR_HTTP_INVALID_QUERY_REQUEST        = INTERNET_ERROR_BASE + 154;
      ERROR_HTTP_HEADER_ALREADY_EXISTS        = INTERNET_ERROR_BASE + 155;
      ERROR_HTTP_REDIRECT_FAILED              = INTERNET_ERROR_BASE + 156;
      ERROR_HTTP_NOT_REDIRECTED               = INTERNET_ERROR_BASE + 160;
      ERROR_HTTP_COOKIE_NEEDS_CONFIRMATION    = INTERNET_ERROR_BASE + 161;
      ERROR_HTTP_COOKIE_DECLINED              = INTERNET_ERROR_BASE + 162;
      ERROR_HTTP_REDIRECT_NEEDS_CONFIRMATION  = INTERNET_ERROR_BASE + 168;

//
// additional Internet API error codes
//
const
      ERROR_INTERNET_SECURITY_CHANNEL_ERROR   = INTERNET_ERROR_BASE + 157;
      ERROR_INTERNET_UNABLE_TO_CACHE_FILE     = INTERNET_ERROR_BASE + 158;
      ERROR_INTERNET_TCPIP_NOT_INSTALLED      = INTERNET_ERROR_BASE + 159;
      ERROR_INTERNET_DISCONNECTED             = INTERNET_ERROR_BASE + 163;
      ERROR_INTERNET_SERVER_UNREACHABLE       = INTERNET_ERROR_BASE + 164;
      ERROR_INTERNET_PROXY_SERVER_UNREACHABLE = INTERNET_ERROR_BASE + 165;

      ERROR_INTERNET_BAD_AUTO_PROXY_SCRIPT    = INTERNET_ERROR_BASE + 166;
      ERROR_INTERNET_UNABLE_TO_DOWNLOAD_SCRIPT = INTERNET_ERROR_BASE + 167;
      ERROR_INTERNET_SEC_INVALID_CERT         = INTERNET_ERROR_BASE + 169;
      ERROR_INTERNET_SEC_CERT_REVOKED         = INTERNET_ERROR_BASE + 170;

// InternetAutodial specific errors
const
      ERROR_INTERNET_FAILED_DUETOSECURITYCHECK  = INTERNET_ERROR_BASE + 171;
      ERROR_INTERNET_NOT_INITIALIZED            = INTERNET_ERROR_BASE + 172;
      ERROR_INTERNET_NEED_MSN_SSPI_PKG          = INTERNET_ERROR_BASE + 173;
      ERROR_INTERNET_LOGIN_FAILURE_DISPLAY_ENTITY_BODY   = INTERNET_ERROR_BASE + 174;


      INTERNET_ERROR_LAST                     = ERROR_INTERNET_LOGIN_FAILURE_DISPLAY_ENTITY_BODY;


//
// URLCACHE APIs
//

//
// datatype definitions.
//

//
// cache entry type flags.
//
const
      NORMAL_CACHE_ENTRY              = $00000001;
      STICKY_CACHE_ENTRY              = $00000004;
      EDITED_CACHE_ENTRY              = $00000008;
      TRACK_OFFLINE_CACHE_ENTRY       = $00000010;
      TRACK_ONLINE_CACHE_ENTRY        = $00000020;
      SPARSE_CACHE_ENTRY              = $00010000;
      COOKIE_CACHE_ENTRY              = $00100000;
      URLHISTORY_CACHE_ENTRY          = $00200000;


      URLCACHE_FIND_DEFAULT_FILTER    = NORMAL_CACHE_ENTRY or
                                        COOKIE_CACHE_ENTRY or
                                        URLHISTORY_CACHE_ENTRY or
                                        TRACK_OFFLINE_CACHE_ENTRY or
<<<<<<< HEAD
<<<<<<< HEAD
                                        TRACK_ONLINE_CACHE_ENTRY or
=======
                                        TRACK_ONLINE_CACHE_ENTRY or       
>>>>>>> graemeg/fixes_2_2
=======
                                        TRACK_ONLINE_CACHE_ENTRY or       
>>>>>>> origin/fixes_2_2
                                        STICKY_CACHE_ENTRY;



//
// INTERNET_CACHE_ENTRY_INFO -
//

type
     _INTERNET_CACHE_ENTRY_INFOA = record
        dwStructSize:DWORD;     // version of cache system.
        lpszSourceUrlName:LPSTR;    // embedded pointer to the URL name string.
        lpszLocalFileName:LPSTR;  // embedded pointer to the local file name.
        CacheEntryType:DWORD;   // cache type bit mask.
        dwUseCount:DWORD;     // current users count of the cache entry.
        dwHitRate:DWORD;      // num of times the cache entry was retrieved.
        dwSizeLow:DWORD;      // low DWORD of the file size.
        dwSizeHigh:DWORD;     // high DWORD of the file size.
        LastModifiedTime:FILETIME;  // last modified time of the file in GMT format.
        ExpireTime:FILETIME;    // expire time of the file in GMT format
        LastAccessTime:FILETIME;  // last accessed time in GMT format
        LastSyncTime:FILETIME;    // last time the URL was synchronized with the source
        lpHeaderInfo:LPSTR;    // embedded pointer to the header info.
        dwHeaderInfoSize:DWORD;   // size of the above header.
        lpszFileExtension:LPSTR;  // File extension used to retrive the urldata as a file.
       case longint of // Exemption delta from last access time.
           0: (dwReserved:DWORD);
         1: (dwExemptDelta:DWORD);
     end;
     INTERNET_CACHE_ENTRY_INFOA = _INTERNET_CACHE_ENTRY_INFOA;
     LPINTERNET_CACHE_ENTRY_INFOA = ^_INTERNET_CACHE_ENTRY_INFOA;

     _INTERNET_CACHE_ENTRY_INFOW = record
        dwStructSize:DWORD;     // version of cache system.
        lpszSourceUrlName:LPWSTR;   // embedded pointer to the URL name string.
        lpszLocalFileName:LPWSTR; // embedded pointer to the local file name.
        CacheEntryType:DWORD;   // cache type bit mask.
        dwUseCount:DWORD;     // current users count of the cache entry.
        dwHitRate:DWORD;      // num of times the cache entry was retrieved.
        dwSizeLow:DWORD;      // low DWORD of the file size.
        dwSizeHigh:DWORD;     // high DWORD of the file size.
        LastModifiedTime:FILETIME;  // last modified time of the file in GMT format.
        ExpireTime:FILETIME;    // expire time of the file in GMT format
        LastAccessTime:FILETIME;  // last accessed time in GMT format
        LastSyncTime:FILETIME;    // last time the URL was synchronized with the source
        lpHeaderInfo:LPWSTR;     // embedded pointer to the header info.
        dwHeaderInfoSize:DWORD;   // size of the above header.
        lpszFileExtension:LPWSTR; // File extension used to retrive the urldata as a file.
       case longint of // Exemption delta from last access time.
           0: (dwReserved:DWORD);
         1: (dwExemptDelta:DWORD);
     end;
     INTERNET_CACHE_ENTRY_INFOW = _INTERNET_CACHE_ENTRY_INFOW;
     LPINTERNET_CACHE_ENTRY_INFOW = ^_INTERNET_CACHE_ENTRY_INFOW;

{$IFDEF UNICODE}
type
     INTERNET_CACHE_ENTRY_INFO = INTERNET_CACHE_ENTRY_INFOW;
     LPINTERNET_CACHE_ENTRY_INFO = LPINTERNET_CACHE_ENTRY_INFOW;
{$ELSE UNICODE}
type
     INTERNET_CACHE_ENTRY_INFO = INTERNET_CACHE_ENTRY_INFOA;
     LPINTERNET_CACHE_ENTRY_INFO = LPINTERNET_CACHE_ENTRY_INFOA;
{$ENDIF UNICODE}

type
     _INTERNET_CACHE_TIMESTAMPS = record
       ftExpires:FILETIME;
        ftLastModified:FILETIME;
     end;
     INTERNET_CACHE_TIMESTAMPS = _INTERNET_CACHE_TIMESTAMPS;
     LPINTERNET_CACHE_TIMESTAMPS = ^_INTERNET_CACHE_TIMESTAMPS;



//
// Cache Group
//
type
     GROUPID = LONGLONG;
     LPGROUPID = ^GROUPID;


//
// Cache Group Flags
//
const
      CACHEGROUP_ATTRIBUTE_GET_ALL        = $ffffffff;
      CACHEGROUP_ATTRIBUTE_BASIC          = $00000001;
      CACHEGROUP_ATTRIBUTE_FLAG           = $00000002;
      CACHEGROUP_ATTRIBUTE_TYPE           = $00000004;
      CACHEGROUP_ATTRIBUTE_QUOTA          = $00000008;
      CACHEGROUP_ATTRIBUTE_GROUPNAME      = $00000010;
      CACHEGROUP_ATTRIBUTE_STORAGE        = $00000020;

      CACHEGROUP_FLAG_NONPURGEABLE        = $00000001;
      CACHEGROUP_FLAG_GIDONLY             = $00000004;

      CACHEGROUP_FLAG_FLUSHURL_ONDELETE   = $00000002;

      CACHEGROUP_SEARCH_ALL               = $00000000;
      CACHEGROUP_SEARCH_BYURL             = $00000001;

      CACHEGROUP_TYPE_INVALID             = $00000001;


//
// updatable cache group fields
//
const
      CACHEGROUP_READWRITE_MASK = CACHEGROUP_ATTRIBUTE_TYPE or
                                  CACHEGROUP_ATTRIBUTE_QUOTA or
                                  CACHEGROUP_ATTRIBUTE_GROUPNAME or
                                  CACHEGROUP_ATTRIBUTE_STORAGE;

//
// INTERNET_CACHE_GROUP_INFO
//
const
       GROUPNAME_MAX_LENGTH       = 120;
       GROUP_OWNER_STORAGE_SIZE   = 4;

type
     _INTERNET_CACHE_GROUP_INFOA = record
       dwGroupSize:DWORD;
        dwGroupFlags:DWORD;
        dwGroupType:DWORD;
        dwDiskUsage:DWORD;  // in KB
        dwDiskQuota:DWORD;  // in KB
        dwOwnerStorage:array[0..GROUP_OWNER_STORAGE_SIZE-1] of DWORD;
        szGroupName:array[0..GROUPNAME_MAX_LENGTH-1] of char;
     end;
     INTERNET_CACHE_GROUP_INFOA = _INTERNET_CACHE_GROUP_INFOA;
     LPINTERNET_CACHE_GROUP_INFOA = ^INTERNET_CACHE_GROUP_INFOA;

     _INTERNET_CACHE_GROUP_INFOW = record
       dwGroupSize:DWORD;
        dwGroupFlags:DWORD;
        dwGroupType:DWORD;
        dwDiskUsage:DWORD;  // in KB
        dwDiskQuota:DWORD;  // in KB
        dwOwnerStorage:array[0..GROUP_OWNER_STORAGE_SIZE-1] of DWORD;
        szGroupName:array[0..GROUPNAME_MAX_LENGTH-1] of WCHAR;
     end;
     INTERNET_CACHE_GROUP_INFOW = _INTERNET_CACHE_GROUP_INFOW;
     LPINTERNET_CACHE_GROUP_INFOW = ^INTERNET_CACHE_GROUP_INFOW;


{$IFDEF UNICODE}
type
     INTERNET_CACHE_GROUP_INFO = INTERNET_CACHE_GROUP_INFOW;
     LPINTERNET_CACHE_GROUP_INFO = LPINTERNET_CACHE_GROUP_INFOW;
{$ELSE UNICODE}
type
     INTERNET_CACHE_GROUP_INFO = INTERNET_CACHE_GROUP_INFOA;
     LPINTERNET_CACHE_GROUP_INFO = LPINTERNET_CACHE_GROUP_INFOA;
{$ENDIF UNICODE}


//
// Cache APIs
//

function CreateUrlCacheEntryA(lpszUrlName:LPCSTR;
                                     dwExpectedFileSize:DWORD;
                                     lpszFileExtension:LPCSTR;
                                     lpszFileName:LPSTR;
                                     dwReserved:DWORD
                                    ):BOOL; external WinInetDLL name 'CreateUrlCacheEntryA';

function CreateUrlCacheEntryW(lpszUrlName:LPCWSTR;
                                     dwExpectedFileSize:DWORD;
                                     lpszFileExtension:LPCWSTR;
                                     lpszFileName:LPWSTR;
                                     dwReserved:DWORD
                                    ):BOOL; external WinInetDLL name 'CreateUrlCacheEntryW';

{$IFDEF UNICODE}
function CreateUrlCacheEntry(lpszUrlName:LPCWSTR;
                                    dwExpectedFileSize:DWORD;
                                    lpszFileExtension:LPCWSTR;
                                    lpszFileName:LPWSTR;
                                    dwReserved:DWORD
                                   ):BOOL; external WinInetDLL name 'CreateUrlCacheEntryW';
{$ELSE UNICODE}
function CreateUrlCacheEntry(lpszUrlName:LPCSTR;
                                    dwExpectedFileSize:DWORD;
                                    lpszFileExtension:LPCSTR;
                                    lpszFileName:LPSTR;
                                    dwReserved:DWORD
                                   ):BOOL; external WinInetDLL name 'CreateUrlCacheEntryA';
{$ENDIF UNICODE}

{$IFNDEF USE_FIXED_COMMIT_URL_CACHE_ENTRY}
// Temporary state of affairs until we reconcile our apis.

// Why are we doing this? HeaderInfo _should_ be string data.
// However, one group is passing binary data instead. For the
// unicode api, we've decided to disallow this, but this
// brings up an inconsistency between the u and a apis, which
// is undesirable.

// For Beta 1, we'll go with this behaviour, but in future releases
// we want to make these apis consistent.

function CommitUrlCacheEntryA(lpszUrlName:LPCSTR;
                                     lpszLocalFileName:LPCSTR;
                                     ExpireTime:FILETIME;
                              LastModifiedTime:FILETIME;
                                     CacheEntryType:DWORD;
                                     lpHeaderInfo:LPBYTE;
                                     dwHeaderSize:DWORD;
                                     lpszFileExtension:LPCSTR;
                                     lpszOriginalUrl:LPCSTR
                                    ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryA';

function CommitUrlCacheEntryW(lpszUrlName:LPCWSTR;
                                     lpszLocalFileName:LPCWSTR;
                                     ExpireTime:FILETIME;
                              LastModifiedTime:FILETIME;
                                     CacheEntryType:DWORD;
                                     lpHeaderInfo:LPWSTR;
                                     dwHeaderSize:DWORD;
                                     lpszFileExtension:LPCWSTR;
                                     lpszOriginalUrl:LPCWSTR
                                    ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryW';

{$IFDEF UNICODE}
function CommitUrlCacheEntry(lpszUrlName:LPCWSTR;
                             lpszLocalFileName:LPCWSTR;
                             ExpireTime:FILETIME;
                             LastModifiedTime:FILETIME;
                             CacheEntryType:DWORD;
                             lpHeaderInfo:LPWSTR;
                             dwHeaderSize:DWORD;
                             lpszFileExtension:LPCWSTR;
                             lpszOriginalUrl:LPCWSTR
                            ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryW';
{$ELSE UNICODE}
function CommitUrlCacheEntry(lpszUrlName:LPCSTR;
                             lpszLocalFileName:LPCSTR;
                             ExpireTime:FILETIME;
                             LastModifiedTime:FILETIME;
                             CacheEntryType:DWORD;
                             lpHeaderInfo:LPBYTE;
                             dwHeaderSize:DWORD;
                             lpszFileExtension:LPCSTR;
                             lpszOriginalUrl:LPCSTR
                            ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryA';
{$ENDIF UNICODE}
{$ELSE USE_FIXED_COMMIT_URL_CACHE_ENTRY}
function CommitUrlCacheEntryA(lpszUrlName:LPCSTR;
                                     lpszLocalFileName:LPCSTR;
                                     ExpireTime:FILETIME;
                              LastModifiedTime:FILETIME;
                                     CacheEntryType:DWORD;
                                     lpHeaderInfo:LPCSTR;
                                     dwHeaderSize:DWORD;
                                     lpszFileExtension:LPCSTR;
                                     lpszOriginalUrl:LPCSTR
                                    ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryA';

function CommitUrlCacheEntryW(lpszUrlName:LPCWSTR;
                                     lpszLocalFileName:LPCWSTR;
                                     ExpireTime:FILETIME;
                              LastModifiedTime:FILETIME;
                                     CacheEntryType:DWORD;
                                     lpHeaderInfo:LPCWSTR;
                                     dwHeaderSize:DWORD;
                                     lpszFileExtension:LPCWSTR;
                                     lpszOriginalUrl:LPCWSTR
                                    ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryW';

{$IFDEF UNICODE}
function CommitUrlCacheEntry(lpszUrlName:LPCWSTR;
                             lpszLocalFileName:LPCWSTR;
                             ExpireTime:FILETIME;
                             LastModifiedTime:FILETIME;
                             CacheEntryType:DWORD;
                             lpHeaderInfo:LPCWSTR;
                             dwHeaderSize:DWORD;
                             lpszFileExtension:LPCWSTR;
                             lpszOriginalUrl:LPCWSTR
                            ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryW';
{$ELSE UNICODE}
function CommitUrlCacheEntry(lpszUrlName:LPCSTR;
                             lpszLocalFileName:LPCSTR;
                             ExpireTime:FILETIME;
                             LastModifiedTime:FILETIME;
                             CacheEntryType:DWORD;
                             lpHeaderInfo:LPCSTR;
                             dwHeaderSize:DWORD;
                             lpszFileExtension:LPCSTR;
                             lpszOriginalUrl:LPCSTR
                            ):BOOL; external WinInetDLL name 'CommitUrlCacheEntryA';
{$ENDIF UNICODE}
{$ENDIF USE_FIXED_COMMIT_URL_CACHE_ENTRY}

function RetrieveUrlCacheEntryFileA(lpszUrlName:LPCSTR;
                                            lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                            lpdwCacheEntryInfoBufferSize:LPDWORD;
                                            dwReserved:DWORD
                                           ):BOOL; external WinInetDLL name 'RetrieveUrlCacheEntryFileA';

function RetrieveUrlCacheEntryFileW(lpszUrlName:LPCWSTR;
                                            lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                            lpdwCacheEntryInfoBufferSize:LPDWORD;
                                            dwReserved:DWORD
                                           ):BOOL; external WinInetDLL name 'RetrieveUrlCacheEntryFileW';

{$IFDEF UNICODE}
function RetrieveUrlCacheEntryFile(lpszUrlName:LPCWSTR;
                                           lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                           lpdwCacheEntryInfoBufferSize:LPDWORD;
                                           dwReserved:DWORD
                                          ):BOOL; external WinInetDLL name 'RetrieveUrlCacheEntryFileW';
{$ELSE UNICODE}
function RetrieveUrlCacheEntryFile(lpszUrlName:LPCSTR;
                                           lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                           lpdwCacheEntryInfoBufferSize:LPDWORD;
                                           dwReserved:DWORD
                                          ):BOOL; external WinInetDLL name 'RetrieveUrlCacheEntryFileA';
{$ENDIF UNICODE}

function UnlockUrlCacheEntryFileA(lpszUrlName:LPCSTR;
                                          dwReserved:DWORD
                                         ):BOOL; external WinInetDLL name 'UnlockUrlCacheEntryFileA';

function UnlockUrlCacheEntryFileW(lpszUrlName:LPCWSTR;
                                          dwReserved:DWORD
                                         ):BOOL; external WinInetDLL name 'UnlockUrlCacheEntryFileW';

{$IFDEF UNICODE}
function UnlockUrlCacheEntryFile(lpszUrlName:LPCWSTR;
                                         dwReserved:DWORD
                                        ):BOOL; external WinInetDLL name 'UnlockUrlCacheEntryFileW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function UnlockUrlCacheEntryFile(lpszUrlName:LPCSTR;
                                         dwReserved:DWORD
                                        ):BOOL; external WinInetDLL name 'UnlockUrlCacheEntryFileA';
{$ELSE WIN32}
function UnlockUrlCacheEntryFile(lpszUrlName:LPCSTR;
                                         dwReserved:DWORD
                                        ):BOOL; external WinInetDLL name 'UnlockUrlCacheEntryFile';
{$ENDIF WIN32}
{$ENDIF UNICODE}

function RetrieveUrlCacheEntryStreamA(lpszUrlName:LPCSTR;
                                                  lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                                  lpdwCacheEntryInfoBufferSize:LPDWORD;
                                                  fRandomRead:BOOL;
                                                  dwReserved:DWORD
                                                ):HANDLE; external WinInetDLL name 'RetrieveUrlCacheEntryStreamA';

function RetrieveUrlCacheEntryStreamW(lpszUrlName:LPCWSTR;
                                                  lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                                  lpdwCacheEntryInfoBufferSize:LPDWORD;
                                                  fRandomRead:BOOL;
                                                  dwReserved:DWORD
                                                ):HANDLE; external WinInetDLL name 'RetrieveUrlCacheEntryStreamW';

{$IFDEF UNICODE}
function RetrieveUrlCacheEntryStream(lpszUrlName:LPCWSTR;
                                                 lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                                 lpdwCacheEntryInfoBufferSize:LPDWORD;
                                                 fRandomRead:BOOL;
                                                 dwReserved:DWORD
                                               ):HANDLE; external WinInetDLL name 'RetrieveUrlCacheEntryStreamW';
{$ELSE UNICODE}
function RetrieveUrlCacheEntryStream(lpszUrlName:LPCSTR;
                                                 lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                                 lpdwCacheEntryInfoBufferSize:LPDWORD;
                                                 fRandomRead:BOOL;
                                                 dwReserved:DWORD
                                               ):HANDLE; external WinInetDLL name 'RetrieveUrlCacheEntryStreamA';
{$ENDIF UNICODE}

function ReadUrlCacheEntryStream(hUrlCacheStream:HANDLE;
                                        dwLocation:DWORD;
                                        lpBuffer:LPVOID;
                                        lpdwLen:LPDWORD;
                                        Reserved:DWORD
                                       ):BOOL; external WinInetDLL name 'ReadUrlCacheEntryStream';

function UnlockUrlCacheEntryStream(hUrlCacheStream:HANDLE;
                                           Reserved:DWORD
                                          ):BOOL; external WinInetDLL name 'UnlockUrlCacheEntryStream';

function GetUrlCacheEntryInfoA(lpszUrlName:LPCSTR;
                                         lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                         lpdwCacheEntryInfoBufferSize:LPDWORD
                                        ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoA';

function GetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR;
                                         lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                         lpdwCacheEntryInfoBufferSize:LPDWORD
                                        ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoW';

{$IFDEF UNICODE}
function GetUrlCacheEntryInfo(lpszUrlName:LPCWSTR;
                                        lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                        lpdwCacheEntryInfoBufferSize:LPDWORD
                                       ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoW';
{$ELSE UNICODE}
function GetUrlCacheEntryInfo(lpszUrlName:LPCSTR;
                                        lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                        lpdwCacheEntryInfoBufferSize:LPDWORD
                                       ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoA';
{$ENDIF UNICODE}

function FindFirstUrlCacheGroup(dwFlags:DWORD;
                                          dwFilter:DWORD;
                                          lpSearchCondition:LPVOID;
                                          dwSearchCondition:DWORD;
                                          lpGroupId:LPGROUPID;
                                          lpReserved:LPVOID
                                         ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheGroup';

function FindNextUrlCacheGroup(hFind:HANDLE;
                                         lpGroupId:LPGROUPID;
                                         lpReserved:LPVOID
                                        ):BOOL; external WinInetDLL name 'FindNextUrlCacheGroup';


function GetUrlCacheGroupAttributeA(gid:GROUPID;
                                    dwFlags:DWORD;
                                               dwAttributes:DWORD;
                                               lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA;
                                               lpdwGroupInfo:LPDWORD;
                                               lpReserved:LPVOID
                                              ):BOOL; external WinInetDLL name 'GetUrlCacheGroupAttributeA';

function GetUrlCacheGroupAttributeW(gid:GROUPID;
                                    dwFlags:DWORD;
                                               dwAttributes:DWORD;
                                               lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW;
                                               lpdwGroupInfo:LPDWORD;
                                               lpReserved:LPVOID
                                              ):BOOL; external WinInetDLL name 'GetUrlCacheGroupAttributeW';

{$IFDEF UNICODE}
function GetUrlCacheGroupAttribute(gid:GROUPID;
                                   dwFlags:DWORD;
                                              dwAttributes:DWORD;
                                              lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW;
                                              lpdwGroupInfo:LPDWORD;
                                              lpReserved:LPVOID
                                             ):BOOL; external WinInetDLL name 'GetUrlCacheGroupAttributeW';
{$ELSE UNICODE}
function GetUrlCacheGroupAttribute(gid:GROUPID;
                                   dwFlags:DWORD;
                                              dwAttributes:DWORD;
                                              lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA;
                                              lpdwGroupInfo:LPDWORD;
                                              lpReserved:LPVOID
                                             ):BOOL; external WinInetDLL name 'GetUrlCacheGroupAttributeA';
{$ENDIF UNICODE}

function SetUrlCacheGroupAttributeA(gid:GROUPID;
                                               dwFlags:DWORD;
                                               dwAttributes:DWORD;
                                               lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA;
                                               lpReserved:LPVOID
                                              ):BOOL; external WinInetDLL name 'SetUrlCacheGroupAttributeA';

function SetUrlCacheGroupAttributeW(gid:GROUPID;
                                               dwFlags:DWORD;
                                               dwAttributes:DWORD;
                                               lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW;
                                               lpReserved:LPVOID
                                              ):BOOL; external WinInetDLL name 'SetUrlCacheGroupAttributeW';

{$IFDEF UNICODE}
function SetUrlCacheGroupAttribute(gid:GROUPID;
                                              dwFlags:DWORD;
                                              dwAttributes:DWORD;
                                              lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW;
                                              lpReserved:LPVOID
                                             ):BOOL; external WinInetDLL name 'SetUrlCacheGroupAttributeW';
{$ELSE UNICODE}
function SetUrlCacheGroupAttribute(gid:GROUPID;
                                              dwFlags:DWORD;
                                              dwAttributes:DWORD;
                                              lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA;
                                              lpReserved:LPVOID
                                             ):BOOL; external WinInetDLL name 'SetUrlCacheGroupAttributeA';
{$ENDIF UNICODE}


function GetUrlCacheEntryInfoExA(lpszUrl:LPCSTR;
                                        lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                        lpdwCacheEntryInfoBufSize:LPDWORD;
                                        lpszReserved:LPSTR;  // must pass null
                                        lpdwReserved:LPDWORD;  // must pass null
                                        lpReserved:LPVOID;   // must pass null
                                        dwFlags:DWORD    // reserved
                                       ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoExA';

function GetUrlCacheEntryInfoExW(lpszUrl:LPCWSTR;
                                        lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                        lpdwCacheEntryInfoBufSize:LPDWORD;
                                        lpszReserved:LPWSTR;  // must pass null
                                        lpdwReserved:LPDWORD;  // must pass null
                                        lpReserved:LPVOID;   // must pass null
                                        dwFlags:DWORD    // reserved
                                       ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoExW';

{$IFDEF UNICODE}
function GetUrlCacheEntryInfoEx(lpszUrl:LPCWSTR;
                                lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                lpdwCacheEntryInfoBufSize:LPDWORD;
                                lpszReserved:LPWSTR;  // must pass null
                                lpdwReserved:LPDWORD;  // must pass null
                                lpReserved:LPVOID;   // must pass null
                                dwFlags:DWORD    // reserved
                               ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoExW';
{$ELSE UNICODE}
function GetUrlCacheEntryInfoEx(lpszUrl:LPCSTR;
                                lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                lpdwCacheEntryInfoBufSize:LPDWORD;
                                lpszReserved:LPSTR;  // must pass null
                                lpdwReserved:LPDWORD;  // must pass null
                                lpReserved:LPVOID;   // must pass null
                                dwFlags:DWORD    // reserved
                               ):BOOL; external WinInetDLL name 'GetUrlCacheEntryInfoExA';
{$ENDIF UNICODE}

const
      CACHE_ENTRY_ATTRIBUTE_FC    = $00000004;
      CACHE_ENTRY_HITRATE_FC      = $00000010;
      CACHE_ENTRY_MODTIME_FC      = $00000040;
      CACHE_ENTRY_EXPTIME_FC      = $00000080;
      CACHE_ENTRY_ACCTIME_FC      = $00000100;
      CACHE_ENTRY_SYNCTIME_FC     = $00000200;
      CACHE_ENTRY_HEADERINFO_FC   = $00000400;
      CACHE_ENTRY_EXEMPT_DELTA_FC = $00000800;

function SetUrlCacheEntryInfoA(lpszUrlName:LPCSTR;
                                      lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                      dwFieldControl:DWORD
                                     ):BOOL; external WinInetDLL name 'SetUrlCacheEntryInfoA';

function SetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR;
                                      lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                      dwFieldControl:DWORD
                                     ):BOOL; external WinInetDLL name 'SetUrlCacheEntryInfoW';

{$IFDEF UNICODE}
function SetUrlCacheEntryInfo(lpszUrlName:LPCWSTR;
                                    lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                    dwFieldControl:DWORD
                                   ):BOOL; external WinInetDLL name 'SetUrlCacheEntryInfoW';
{$ELSE UNICODE}
function SetUrlCacheEntryInfo(lpszUrlName:LPCSTR;
                                    lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                    dwFieldControl:DWORD
                                   ):BOOL; external WinInetDLL name 'SetUrlCacheEntryInfoA';
{$ENDIF UNICODE}


//
// Cache Group Functions
//

function CreateUrlCacheGroup(dwFlags:DWORD;
                                       lpReserved:LPVOID  // must pass NULL
                                      ):GROUPID; external WinInetDLL name 'CreateUrlCacheGroup';

function DeleteUrlCacheGroup(GroupId:GROUPID;
                                   dwFlags:DWORD;     // must pass 0
                                   lpReserved:LPVOID    // must pass NULL
                                  ):BOOL; external WinInetDLL name 'DeleteUrlCacheGroup';

// Flags for SetUrlCacheEntryGroup
const
      INTERNET_CACHE_GROUP_ADD      = 0;
      INTERNET_CACHE_GROUP_REMOVE   = 1;

function SetUrlCacheEntryGroupA(lpszUrlName:LPCSTR;
                                dwFlags:DWORD;
                                GroupId:GROUPID;
                                pbGroupAttributes:LPBYTE; // must pass NULL
                                cbGroupAttributes:DWORD; // must pass 0
                                       lpReserved:LPVOID     // must pass NULL
                                      ):BOOL; external WinInetDLL name 'SetUrlCacheEntryGroupA';

function SetUrlCacheEntryGroupW(lpszUrlName:LPCWSTR;
                                dwFlags:DWORD;
                                GroupId:GROUPID;
                                pbGroupAttributes:LPBYTE; // must pass NULL
                                cbGroupAttributes:DWORD; // must pass 0
                                       lpReserved:LPVOID     // must pass NULL
                                      ):BOOL; external WinInetDLL name 'SetUrlCacheEntryGroupW';

{$IFDEF UNICODE}
function SetUrlCacheEntryGroup(lpszUrlName:LPCWSTR;
                               dwFlags:DWORD;
                               GroupId:GROUPID;
                               pbGroupAttributes:LPBYTE; // must pass NULL
                               cbGroupAttributes:DWORD; // must pass 0
                               lpReserved:LPVOID     // must pass NULL
                              ):BOOL; external WinInetDLL name 'SetUrlCacheEntryGroupW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function SetUrlCacheEntryGroup(lpszUrlName:LPCSTR;
                               dwFlags:DWORD;
                               GroupId:GROUPID;
                               pbGroupAttributes:LPBYTE; // must pass NULL
                               cbGroupAttributes:DWORD; // must pass 0
                               lpReserved:LPVOID     // must pass NULL
                              ):BOOL; external WinInetDLL name 'SetUrlCacheEntryGroupA';
{$ELSE WIN32}
function SetUrlCacheEntryGroup(lpszUrlName:LPCSTR;
                               dwFlags:DWORD;
                               GroupId:GROUPID;
                               pbGroupAttributes:LPBYTE; // must pass NULL
                               cbGroupAttributes:DWORD; // must pass 0
                               lpReserved:LPVOID     // must pass NULL
                              ):BOOL; external WinInetDLL name 'SetUrlCacheEntryGroup';
{$ENDIF WIN32}
{$ENDIF UNICODE}

function FindFirstUrlCacheEntryExA(lpszUrlSearchPattern:LPCSTR;
                                   dwFlags:DWORD;
                                              dwFilter:DWORD;
                                              GroupId:GROUPID;
                                              lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                              lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                              lpReserved:LPVOID;     // must pass NULL
                                              pcbReserved2:LPDWORD;   // must pass NULL
                                              lpReserved3:LPVOID     // must pass NULL
                                             ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryExA';

function FindFirstUrlCacheEntryExW(lpszUrlSearchPattern:LPCWSTR;
                                   dwFlags:DWORD;
                                              dwFilter:DWORD;
                                              GroupId:GROUPID;
                                              lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                              lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                              lpReserved:LPVOID;     // must pass NULL
                                              pcbReserved2:LPDWORD;   // must pass NULL
                                              lpReserved3:LPVOID     // must pass NULL
                                             ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryExW';

{$IFDEF UNICODE}
function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCWSTR;
                                  dwFlags:DWORD;
                                  dwFilter:DWORD;
                                  GroupId:GROUPID;
                                  lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                  lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                  lpReserved:LPVOID;     // must pass NULL
                                  pcbReserved2:LPDWORD;   // must pass NULL
                                  lpReserved3:LPVOID     // must pass NULL
                                 ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryExW';
{$ELSE UNICODE}
function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCSTR;
                                  dwFlags:DWORD;
                                  dwFilter:DWORD;
                                  GroupId:GROUPID;
                                  lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                  lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                  lpReserved:LPVOID;     // must pass NULL
                                  pcbReserved2:LPDWORD;   // must pass NULL
                                  lpReserved3:LPVOID     // must pass NULL
                                 ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryExA';
{$ENDIF UNICODE}

function FindNextUrlCacheEntryExA(hEnumHandle:HANDLE;
                                          lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                  lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                          lpReserved:LPVOID;   // must pass NULL
                                          pcbReserved2:LPDWORD;  // must pass NULL
                                          lpReserved3:LPVOID   // must pass NULL
                                         ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryExA';

function FindNextUrlCacheEntryExW(hEnumHandle:HANDLE;
                                          lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                  lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                          lpReserved:LPVOID;   // must pass NULL
                                          pcbReserved2:LPDWORD;  // must pass NULL
                                          lpReserved3:LPVOID   // must pass NULL
                                         ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryExW';

{$IFDEF UNICODE}
function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE;
                                 lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                 lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                 lpReserved:LPVOID;  // must pass NULL
                                 pcbReserved2:LPDWORD;   // must pass NULL
                                 lpReserved3:LPVOID  // must pass NULL
                                ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryExW';
{$ELSE UNICODE}
function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE;
                                 lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                 lpdwFirstCacheEntryInfoBufferSize:LPDWORD;
                                 lpReserved:LPVOID;  // must pass NULL
                                 pcbReserved2:LPDWORD;   // must pass NULL
                                 lpReserved3:LPVOID  // must pass NULL
                                ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryExA';
{$ENDIF UNICODE}

function FindFirstUrlCacheEntryA(lpszUrlSearchPattern:LPCSTR;
                                            lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                            lpdwFirstCacheEntryInfoBufferSize:LPDWORD
                                           ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryA';

function FindFirstUrlCacheEntryW(lpszUrlSearchPattern:LPCWSTR;
                                            lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                            lpdwFirstCacheEntryInfoBufferSize:LPDWORD
                                           ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryW';

{$IFDEF UNICODE}
function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCWSTR;
                                lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                lpdwFirstCacheEntryInfoBufferSize:LPDWORD
                               ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryW';
{$ELSE UNICODE}
function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCSTR;
                                lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                lpdwFirstCacheEntryInfoBufferSize:LPDWORD
                               ):HANDLE; external WinInetDLL name 'FindFirstUrlCacheEntryA';
{$ENDIF UNICODE}

function FindNextUrlCacheEntryA(hEnumHandle:HANDLE;
                                       lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                       lpdwNextCacheEntryInfoBufferSize:LPDWORD
                                      ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryA';

function FindNextUrlCacheEntryW(hEnumHandle:HANDLE;
                                       lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                       lpdwNextCacheEntryInfoBufferSize:LPDWORD
                                      ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryW';

{$IFDEF UNICODE}
function FindNextUrlCacheEntry(hEnumHandle:HANDLE;
                                      lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
                                      lpdwNextCacheEntryInfoBufferSize:LPDWORD
                                     ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryW';
{$ELSE UNICODE}
function FindNextUrlCacheEntry(hEnumHandle:HANDLE;
                                      lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
                                      lpdwNextCacheEntryInfoBufferSize:LPDWORD
                                     ):BOOL; external WinInetDLL name 'FindNextUrlCacheEntryA';
{$ENDIF UNICODE}


function FindCloseUrlCache(hEnumHandle:HANDLE):BOOL; external WinInetDLL name 'FindCloseUrlCache';

function DeleteUrlCacheEntryA(lpszUrlName:LPCSTR):BOOL; external WinInetDLL name 'DeleteUrlCacheEntryA';

function DeleteUrlCacheEntryW(lpszUrlName:LPCWSTR):BOOL; external WinInetDLL name 'DeleteUrlCacheEntryW';

{$IFDEF UNICODE}
function DeleteUrlCacheEntry(lpszUrlName:LPCWSTR):BOOL; external WinInetDLL name 'DeleteUrlCacheEntryW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function DeleteUrlCacheEntry(lpszUrlName:LPCSTR):BOOL; external WinInetDLL name 'DeleteUrlCacheEntryA';
{$ELSE WIN32}
function DeleteUrlCacheEntry(lpszUrlName:LPCSTR):BOOL; external WinInetDLL name 'DeleteUrlCacheEntry';
{$ENDIF WIN32}
{$ENDIF UNICODE}



//
// Autodial APIs
//

{$IFNDEF WINCE}
// Despite the functions
// InternetDial
// InternetHangUp
// InternetAutodial
// InternetAutodialHangup
// InternetInitializeAutoProxyDll
// are declared in wininet.h for both WIN32 and WINCE there are no such
// functions exported by wininet.dll under CE.
function InternetDialA(hwndParent:HWND;
                       lpszConnectoid:LPSTR;
                       dwFlags:DWORD;
                               lpdwConnection:PDWORD_PTR;
                               dwReserved:DWORD
                              ):DWORD; external WinInetDLL name 'InternetDialA';

function InternetDialW(hwndParent:HWND;
                       lpszConnectoid:LPWSTR;
                       dwFlags:DWORD;
                               lpdwConnection:PDWORD_PTR;
                               dwReserved:DWORD
                              ):DWORD; external WinInetDLL name 'InternetDialW';

{$IFDEF UNICODE}
function InternetDial(hwndParent:HWND;
                      lpszConnectoid:LPWSTR;
                      dwFlags:DWORD;
                              lpdwConnection:PDWORD_PTR;
                              dwReserved:DWORD
                             ):DWORD; external WinInetDLL name 'InternetDialW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function InternetDial(hwndParent:HWND;
                      lpszConnectoid:LPSTR;
                      dwFlags:DWORD;
                              lpdwConnection:PDWORD_PTR;
                              dwReserved:DWORD
                             ):DWORD; external WinInetDLL name 'InternetDialA';
{$ELSE WIN32}
function InternetDial(hwndParent:HWND;
                      lpszConnectoid:LPSTR;
                      dwFlags:DWORD;
                              lpdwConnection:PDWORD_PTR;
                              dwReserved:DWORD
                             ):DWORD; external WinInetDLL name 'InternetDial';
{$ENDIF WIN32}
{$ENDIF UNICODE}

// Flags for InternetDial - must not conflict with InternetAutodial flags
//                          as they are valid here also.
const
      INTERNET_DIAL_FORCE_PROMPT     = $2000;
      INTERNET_DIAL_SHOW_OFFLINE     = $4000;
      INTERNET_DIAL_UNATTENDED       = $8000;

function InternetHangUp(dwConnection:DWORD_PTR;
                                dwReserved:DWORD
                       ):DWORD; external WinInetDLL name 'InternetHangUp';

function InternetAutodial(dwFlags:DWORD;
                                  hwndParent:HWND
                         ):BOOL; external WinInetDLL name 'InternetAutodial';

// Flags for InternetAutodial
const
      INTERNET_AUTODIAL_FORCE_ONLINE          = 1;
      INTERNET_AUTODIAL_FORCE_UNATTENDED      = 2;
      INTERNET_AUTODIAL_FAILIFSECURITYCHECK   = 4;
      INTERNET_AUTODIAL_OVERRIDE_NET_PRESENT  = 8;

      INTERNET_AUTODIAL_FLAGS_MASK = INTERNET_AUTODIAL_FORCE_ONLINE or
                                     INTERNET_AUTODIAL_FORCE_UNATTENDED or
                                     INTERNET_AUTODIAL_FAILIFSECURITYCHECK or
                                     INTERNET_AUTODIAL_OVERRIDE_NET_PRESENT;

function InternetAutodialHangup(dwReserved:DWORD):BOOL; external WinInetDLL name 'InternetAutodialHangup';

function InternetInitializeAutoProxyDll(dwReserved:DWORD):BOOL; external WinInetDLL name 'InternetInitializeAutoProxyDll';
{$ENDIF WINCE}

const
      INTERENT_GOONLINE_REFRESH = $00000001;
      INTERENT_GOONLINE_MASK    = $00000001;

function InternetGoOnlineA(lpszURL:LPSTR;
                                    hwndParent:HWND;
                           dwFlags:DWORD
                                   ):BOOL; external WinInetDLL name 'InternetGoOnlineA';

function InternetGoOnlineW(lpszURL:LPWSTR;
                                    hwndParent:HWND;
                           dwFlags:DWORD
                                   ):BOOL; external WinInetDLL name 'InternetGoOnlineW';

{$IFDEF UNICODE}
function InternetGoOnline(lpszURL:LPWSTR;
                                  hwndParent:HWND;
                          dwFlags:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetGoOnlineW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function InternetGoOnline(lpszURL:LPSTR;
                                  hwndParent:HWND;
                          dwFlags:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetGoOnlineA';
{$ELSE WIN32}
function InternetGoOnline(lpszURL:LPSTR;
                                  hwndParent:HWND;
                          dwFlags:DWORD
                                 ):BOOL; external WinInetDLL name 'InternetGoOnline';
{$ENDIF WIN32}
{$ENDIF UNICODE}


function InternetGetConnectedState(lpdwFlags:LPDWORD;
                                              dwReserved:DWORD
                                  ):BOOL; external WinInetDLL name 'InternetGetConnectedState';


function InternetGetConnectedStateExA(lpdwFlags:LPDWORD;
                                                 lpszConnectionName:LPSTR;
                                      dwNameLen:DWORD;
                                      dwReserved:DWORD
                                                ):BOOL; external WinInetDLL name 'InternetGetConnectedStateExA';

function InternetGetConnectedStateExW(lpdwFlags:LPDWORD;
                                                 lpszConnectionName:LPWSTR;
                                      dwNameLen:DWORD;
                                      dwReserved:DWORD
                                                ):BOOL; external WinInetDLL name 'InternetGetConnectedStateExW';

{$IFDEF UNICODE}
function InternetGetConnectedStateEx(lpdwFlags:LPDWORD;
                                                lpszConnectionName:LPWSTR;
                                     dwNameLen:DWORD;
                                     dwReserved:DWORD
                                               ):BOOL; external WinInetDLL name 'InternetGetConnectedStateExW';
{$ELSE UNICODE}
{$IFDEF WIN32}
function InternetGetConnectedStateEx(lpdwFlags:LPDWORD;
                                                lpszConnectionName:LPSTR;
                                     dwNameLen:DWORD;
                                     dwReserved:DWORD
                                               ):BOOL; external WinInetDLL name 'InternetGetConnectedStateExA';
{$ELSE WIN32}
function InternetGetConnectedStateEx(lpdwFlags:LPDWORD;
                                                lpszConnectionName:LPSTR;
                                     dwNameLen:DWORD;
                                     dwReserved:DWORD
                                               ):BOOL; external WinInetDLL name 'InternetGetConnectedStateEx';
{$ENDIF WIN32}
{$ENDIF UNICODE}

// Flags for InternetGetConnectedState and Ex
const
      INTERNET_CONNECTION_MODEM           = $01;
      INTERNET_CONNECTION_LAN             = $02;
      INTERNET_CONNECTION_PROXY           = $04;
<<<<<<< HEAD
<<<<<<< HEAD
      INTERNET_CONNECTION_MODEM_BUSY      = $08;  // no longer used
=======
      INTERNET_CONNECTION_MODEM_BUSY      = $08;  // no longer used 
>>>>>>> graemeg/fixes_2_2
=======
      INTERNET_CONNECTION_MODEM_BUSY      = $08;  // no longer used 
>>>>>>> origin/fixes_2_2
      INTERNET_RAS_INSTALLED              = $10;
      INTERNET_CONNECTION_OFFLINE         = $20;
      INTERNET_CONNECTION_CONFIGURED      = $40;

//
// Custom dial handler functions
//

// Custom dial handler prototype
type
     PFN_DIAL_HANDLER = function(param1:HWND; param2:LPCSTR; param3:DWORD; param4:LPDWORD):DWORD; cdecl;

// Flags for custom dial handler
const
      INTERNET_CUSTOMDIAL_CONNECT         = 0;
      INTERNET_CUSTOMDIAL_UNATTENDED      = 1;
      INTERNET_CUSTOMDIAL_DISCONNECT      = 2;
      INTERNET_CUSTOMDIAL_SHOWOFFLINE     = 4;

// Custom dial handler supported functionality flags
const
      INTERNET_CUSTOMDIAL_SAFE_FOR_UNATTENDED = 1;
      INTERNET_CUSTOMDIAL_WILL_SUPPLY_STATE   = 2;
      INTERNET_CUSTOMDIAL_CAN_HANGUP          = 4;

{
  The obsolete functions InternetSetDialStateA and InternetSetDialStateW and
  constants for them are left out.
}


function InternetSetPerSiteCookieDecisionA(pchHostName:LPCSTR;
                                           dwDecision:DWORD
                                          ):BOOL; external WinInetDLL name 'InternetSetPerSiteCookieDecisionA';

function InternetSetPerSiteCookieDecisionW(pchHostName:LPCWSTR;
                                           dwDecision:DWORD
                                          ):BOOL; external WinInetDLL name 'InternetSetPerSiteCookieDecisionW';

{$IFDEF UNICODE}
function InternetSetPerSiteCookieDecision(pchHostName:LPCWSTR;
                                          dwDecision:DWORD
                                         ):BOOL; external WinInetDLL name 'InternetSetPerSiteCookieDecisionW';
{$ELSE UNICODE}
function InternetSetPerSiteCookieDecision(pchHostName:LPCSTR;
                                          dwDecision:DWORD
                                         ):BOOL; external WinInetDLL name 'InternetSetPerSiteCookieDecisionA';
{$ENDIF UNICODE}

function InternetGetPerSiteCookieDecisionA(pchHostName:LPCSTR;
                                           pResult:PULONG
                                          ):BOOL; external WinInetDLL name 'InternetGetPerSiteCookieDecisionA';

function InternetGetPerSiteCookieDecisionW(pchHostName:LPCWSTR;
                                           pResult:PULONG
                                          ):BOOL; external WinInetDLL name 'InternetGetPerSiteCookieDecisionW';

{$IFDEF UNICODE}
function InternetGetPerSiteCookieDecision(pchHostName:LPCWSTR;
                                          pResult:PULONG
                                         ):BOOL; external WinInetDLL name 'InternetGetPerSiteCookieDecisionW';
{$ELSE UNICODE}
function InternetGetPerSiteCookieDecision(pchHostName:LPCSTR;
                                          pResult:PULONG
                                         ):BOOL; external WinInetDLL name 'InternetGetPerSiteCookieDecisionA';
{$ENDIF UNICODE}

function InternetClearAllPerSiteCookieDecisions:BOOL; external WinInetDLL name 'InternetClearAllPerSiteCookieDecisions';


function InternetEnumPerSiteCookieDecisionA(pszSiteName:LPSTR;
                                            pcSiteNameSize:PULONG;
                                            pdwDecision:PULONG;
                                            dwIndex:ULONG
                                           ):BOOL; external WinInetDLL name 'InternetEnumPerSiteCookieDecisionA';

function InternetEnumPerSiteCookieDecisionW(pszSiteName:LPWSTR;
                                            pcSiteNameSize:PULONG;
                                            pdwDecision:PULONG;
                                            dwIndex:ULONG
                                           ):BOOL; external WinInetDLL name 'InternetEnumPerSiteCookieDecisionW';

{$IFDEF UNICODE}
function InternetEnumPerSiteCookieDecision(pszSiteName:LPWSTR;
                                           pcSiteNameSize:PULONG;
                                           pdwDecision:PULONG;
                                           dwIndex:ULONG
                                          ):BOOL; external WinInetDLL name 'InternetEnumPerSiteCookieDecisionW';
{$ELSE UNICODE}
function InternetEnumPerSiteCookieDecision(pszSiteName:LPSTR;
                                           pcSiteNameSize:PULONG;
                                           pdwDecision:PULONG;
                                           dwIndex:ULONG
                                          ):BOOL; external WinInetDLL name 'InternetEnumPerSiteCookieDecisionA';
{$ENDIF UNICODE}

const
      INTERNET_IDENTITY_FLAG_PRIVATE_CACHE        = $01;
      INTERNET_IDENTITY_FLAG_SHARED_CACHE         = $02;
      INTERNET_IDENTITY_FLAG_CLEAR_DATA           = $04;
      INTERNET_IDENTITY_FLAG_CLEAR_COOKIES        = $08;
      INTERNET_IDENTITY_FLAG_CLEAR_HISTORY        = $10;
      INTERNET_IDENTITY_FLAG_CLEAR_CONTENT        = $20;

      INTERNET_SUPPRESS_RESET_ALL                 = $00;
      INTERNET_SUPPRESS_COOKIE_POLICY             = $01;
      INTERNET_SUPPRESS_COOKIE_POLICY_RESET       = $02;

//
// Privacy settings values and APIs
//
const
      PRIVACY_TEMPLATE_NO_COOKIES     = 0;
      PRIVACY_TEMPLATE_HIGH           = 1;
      PRIVACY_TEMPLATE_MEDIUM_HIGH    = 2;
      PRIVACY_TEMPLATE_MEDIUM         = 3;
      PRIVACY_TEMPLATE_MEDIUM_LOW     = 4;
      PRIVACY_TEMPLATE_LOW            = 5;
      PRIVACY_TEMPLATE_CUSTOM         = 100;
      PRIVACY_TEMPLATE_ADVANCED       = 101;

      PRIVACY_TEMPLATE_MAX            = PRIVACY_TEMPLATE_LOW;

      PRIVACY_TYPE_FIRST_PARTY        = 0;
      PRIVACY_TYPE_THIRD_PARTY        = 1;

function PrivacySetZonePreferenceW(dwZone:DWORD;
                                         dwType:DWORD;
                                         dwTemplate:DWORD;
                                   pszPreference:LPCWSTR
                                        ):DWORD; external WinInetDLL name 'PrivacySetZonePreferenceW';
function PrivacySetZonePreference(dwZone:DWORD;
                                        dwType:DWORD;
                                        dwTemplate:DWORD;
                                  pszPreference:LPCWSTR
                                       ):DWORD; external WinInetDLL name 'PrivacySetZonePreferenceW';

function PrivacyGetZonePreferenceW(dwZone:DWORD;
                                         dwType:DWORD;
                                         pdwTemplate:LPDWORD;
                                   pszBuffer:LPWSTR;
                                   pdwBufferLength:LPDWORD
                                        ):DWORD; external WinInetDLL name 'PrivacyGetZonePreferenceW';
function PrivacyGetZonePreference(dwZone:DWORD;
                                        dwType:DWORD;
                                        pdwTemplate:LPDWORD;
                                  pszBuffer:LPWSTR;
                                  pdwBufferLength:LPDWORD
                                       ):DWORD; external WinInetDLL name 'PrivacyGetZonePreferenceW';

// *
// * Return packing to whatever it was before we
// * entered this file
// *
{$PACKRECORDS DEFAULT} // #include <poppack.h>

implementation

//
// gopher type macros
//
function IS_GOPHER_FILE(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_FILE:=(_type and GOPHER_TYPE_FILE_MASK)<>0;
end;

function IS_GOPHER_DIRECTORY(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_DIRECTORY:=(_type and GOPHER_TYPE_DIRECTORY)<>0;
end;

function IS_GOPHER_PHONE_SERVER(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_PHONE_SERVER:=(_type and GOPHER_TYPE_CSO)<>0;
end;

function IS_GOPHER_ERROR(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_ERROR:=(_type and GOPHER_TYPE_ERROR)<>0;
end;

function IS_GOPHER_INDEX_SERVER(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_INDEX_SERVER:=(_type and GOPHER_TYPE_INDEX_SERVER)<>0;
end;

function IS_GOPHER_TELNET_SESSION(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_TELNET_SESSION:=(_type and GOPHER_TYPE_TELNET)<>0;
end;

function IS_GOPHER_BACKUP_SERVER(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_BACKUP_SERVER:=(_type and GOPHER_TYPE_REDUNDANT)<>0;
end;

function IS_GOPHER_TN3270_SESSION(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_TN3270_SESSION:=(_type and GOPHER_TYPE_TN3270)<>0;
end;

function IS_GOPHER_ASK(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_ASK:=(_type and GOPHER_TYPE_ASK)<>0;
end;

function IS_GOPHER_PLUS(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_PLUS:=(_type and GOPHER_TYPE_GOPHER_PLUS)<>0;
end;

function IS_GOPHER_TYPE_KNOWN(_type:DWORD):BOOL; inline;
begin
  IS_GOPHER_TYPE_KNOWN:=(_type and GOPHER_TYPE_UNKNOWN)=0;
end;

end.
