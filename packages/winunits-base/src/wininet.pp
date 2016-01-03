unit wininet;
//+-------------------------------------------------------------------------
//
//  Microsoft Windows
//  Copyright (c) Microsoft Corporation. All rights reserved.
//
//  File: wininet.h
//
//  Header translation by Marco van de Voort for Free Pascal
//  Platform SDK "winsdk6.0" downloaded february 2008
//
//--------------------------------------------------------------------------
{$Mode objfpc}

{$ifndef NO_SMART_LINK}
{$smartlink on}
{$endif}

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

Uses Windows;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef win64}
  {$packrecords 8}
{$else}
  {$packrecords 4}
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//
// Internet APIs
//

Type
     HINTERNET = LPVOID;
     LPHINTERNET = HINTERNET;
	 PHINTERNET = ^HINTERNET;
     INTERNET_PORT = WORD;
     LPINTERNET_PORT = INTERNET_PORT;
	 PINTERNET_PORT = ^INTERNET_PORT;
	 PGROUPID = ^GROUPID;
     GROUPID = LONGLONG;
	
Const
  WININETLIBNAME = 'wininet.dll';
{ wininenti constants}

     MAX_CACHE_ENTRY_INFO_SIZE       = 4096;
     INTERNET_FLAG_BGUPDATE          = $00000008;
     INTERNET_FLAG_UNUSED_4          = $00000004;

     INTERNET_INVALID_PORT_NUMBER    = 0;           // use the protocol-specific default
     INTERNET_DEFAULT_FTP_PORT       = 21;          // default for FTP servers
     INTERNET_DEFAULT_GOPHER_PORT    = 70;          //    "     "  gopher "
     INTERNET_DEFAULT_HTTP_PORT      = 80;          //    "     "  HTTP   "
     INTERNET_DEFAULT_HTTPS_PORT     = 443;         //    "     "  HTTPS  "
     INTERNET_DEFAULT_SOCKS_PORT     = 1080;        // default for SOCKS firewall servers.
//
// maximum field lengths (arbitrary)
//
     INTERNET_MAX_HOST_NAME_LENGTH   = 256;
     INTERNET_MAX_USER_NAME_LENGTH   = 128;
     INTERNET_MAX_PASSWORD_LENGTH    = 128;
     INTERNET_MAX_PORT_NUMBER_LENGTH = 5;           // INTERNET_PORT is unsigned short
     INTERNET_MAX_PORT_NUMBER_VALUE  = 65535;       // maximum unsigned short value
     INTERNET_MAX_PATH_LENGTH        = 2048;
     INTERNET_MAX_SCHEME_LENGTH      = 32;          // longest protocol name length
     INTERNET_MAX_URL_LENGTH         = (INTERNET_MAX_SCHEME_LENGTH + length('://') + INTERNET_MAX_PATH_LENGTH);

//
// values returned by InternetQueryOption() with INTERNET_OPTION_KEEP_CONNECTION:
//

     INTERNET_KEEP_ALIVE_UNKNOWN     = DWORD(-1);
     INTERNET_KEEP_ALIVE_ENABLED     = 1;
     INTERNET_KEEP_ALIVE_DISABLED    = 0;

//
// flags returned by InternetQueryOption() with INTERNET_OPTION_REQUEST_FLAGS
//

     INTERNET_REQFLAG_FROM_CACHE     = $00000001;  // response came from cache
     INTERNET_REQFLAG_ASYNC          = $00000002;  // request was made asynchronously
     INTERNET_REQFLAG_VIA_PROXY      = $00000004;  // request was made via a proxy
     INTERNET_REQFLAG_NO_HEADERS     = $00000008;  // orginal response contained no headers
     INTERNET_REQFLAG_PASSIVE        = $00000010;  // FTP: passive-mode connection
     INTERNET_REQFLAG_CACHE_WRITE_DISABLED = $00000040;  // HTTPS: this request not cacheable
     INTERNET_REQFLAG_NET_TIMEOUT    = $00000080;  // w/ _FROM_CACHE: net request timed out

//
// flags for IDN enable/disable via INTERNET_OPTION_IDN
//
     INTERNET_FLAG_IDN_DIRECT        = $00000001;  // IDN enabled for direct connections
     INTERNET_FLAG_IDN_PROXY         = $00000002;  // IDN enabled for proxy
//
// flags common to open functions (not InternetOpen()):
//

     INTERNET_FLAG_RELOAD            = $80000000;  // retrieve the original item

//
// flags for InternetOpenUrl():
//

     INTERNET_FLAG_RAW_DATA          = $40000000;  // FTP/gopher find: receive the item as raw (structured) data
     INTERNET_FLAG_EXISTING_CONNECT  = $20000000;  // FTP: use existing InternetConnect handle for server if possible

//
// flags for InternetOpen():
//

     INTERNET_FLAG_ASYNC             = $10000000;  // this request is asynchronous (where supported)

//
// protocol-specific flags:
//

     INTERNET_FLAG_PASSIVE           = $08000000;  // used for FTP connections

//
// additional cache flags
//

     INTERNET_FLAG_NO_CACHE_WRITE    = $04000000;  // don't write this item to the cache
     INTERNET_FLAG_DONT_CACHE        = INTERNET_FLAG_NO_CACHE_WRITE;
     INTERNET_FLAG_MAKE_PERSISTENT   = $02000000;  // make this item persistent in cache
     INTERNET_FLAG_FROM_CACHE        = $01000000;  // use offline semantics
     INTERNET_FLAG_OFFLINE           = INTERNET_FLAG_FROM_CACHE;

//
// additional flags
//

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

     INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTP   = $00008000; // ex: https:// to http://
     INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS  = $00004000; // ex: http:// to https://
     INTERNET_FLAG_IGNORE_CERT_DATE_INVALID  = $00002000; // expired X509 Cert.
     INTERNET_FLAG_IGNORE_CERT_CN_INVALID    = $00001000; // bad common name in X509 Cert.

//
// more caching flags
//

     INTERNET_FLAG_RESYNCHRONIZE     = $00000800;  // asking wininet to update an item if it is newer
     INTERNET_FLAG_HYPERLINK         = $00000400;  // asking wininet to do hyperlinking semantic which works right for scripts
     INTERNET_FLAG_NO_UI             = $00000200;  // no cookie popup
     INTERNET_FLAG_PRAGMA_NOCACHE    = $00000100;  // asking wininet to add "pragma: no-cache"
     INTERNET_FLAG_CACHE_ASYNC       = $00000080;  // ok to perform lazy cache-write
     INTERNET_FLAG_FORMS_SUBMIT      = $00000040;  // this is a forms submit
     INTERNET_FLAG_FWD_BACK          = $00000020;  // fwd-back button op
     INTERNET_FLAG_NEED_FILE         = $00000010;  // need a file for this request
     INTERNET_FLAG_MUST_CACHE_REQUEST = INTERNET_FLAG_NEED_FILE;

//
// flags for FTP
//	
	 FTP_TRANSFER_TYPE_UNKNOWN   = $00000000;
     FTP_TRANSFER_TYPE_ASCII     = $00000001;
     FTP_TRANSFER_TYPE_BINARY    = $00000002;
	
     INTERNET_FLAG_TRANSFER_ASCII    = FTP_TRANSFER_TYPE_ASCII;     // = $00000001
     INTERNET_FLAG_TRANSFER_BINARY   = FTP_TRANSFER_TYPE_BINARY;    // = $00000002

// setable flags
     SECURITY_FLAG_IGNORE_REVOCATION         = $00000080;
     SECURITY_FLAG_IGNORE_UNKNOWN_CA         = $00000100;
     SECURITY_FLAG_IGNORE_WRONG_USAGE        = $00000200;

     SECURITY_FLAG_IGNORE_CERT_CN_INVALID    = INTERNET_FLAG_IGNORE_CERT_CN_INVALID;
     SECURITY_FLAG_IGNORE_CERT_DATE_INVALID  = INTERNET_FLAG_IGNORE_CERT_DATE_INVALID;


     SECURITY_FLAG_IGNORE_REDIRECT_TO_HTTPS  = INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS;
     SECURITY_FLAG_IGNORE_REDIRECT_TO_HTTP   = INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTP;



     SECURITY_SET_MASK        = (SECURITY_FLAG_IGNORE_REVOCATION or
                                 SECURITY_FLAG_IGNORE_UNKNOWN_CA or
                                 SECURITY_FLAG_IGNORE_CERT_CN_INVALID or
                                 SECURITY_FLAG_IGNORE_CERT_DATE_INVALID or
                                 SECURITY_FLAG_IGNORE_WRONG_USAGE);
	
	
//
// flags field masks
//

     SECURITY_INTERNET_MASK  = (INTERNET_FLAG_IGNORE_CERT_CN_INVALID    or
                                 INTERNET_FLAG_IGNORE_CERT_DATE_INVALID  or
                                 INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTPS  or
                                 INTERNET_FLAG_IGNORE_REDIRECT_TO_HTTP   );

     SECURITY_IGNORE_ERROR_MASK  = (INTERNET_FLAG_IGNORE_CERT_CN_INVALID   or
                                     INTERNET_FLAG_IGNORE_CERT_DATE_INVALID or
                                     SECURITY_FLAG_IGNORE_UNKNOWN_CA        or
                                     SECURITY_FLAG_IGNORE_REVOCATION    );

     INTERNET_FLAGS_MASK     = (INTERNET_FLAG_RELOAD
                                or INTERNET_FLAG_RAW_DATA
                                or INTERNET_FLAG_EXISTING_CONNECT
                                or INTERNET_FLAG_ASYNC
                                or INTERNET_FLAG_PASSIVE
                                or INTERNET_FLAG_NO_CACHE_WRITE
                                or INTERNET_FLAG_MAKE_PERSISTENT
                                or INTERNET_FLAG_FROM_CACHE
                                or INTERNET_FLAG_SECURE
                                or INTERNET_FLAG_KEEP_CONNECTION
                                or INTERNET_FLAG_NO_AUTO_REDIRECT
                                or INTERNET_FLAG_READ_PREFETCH
                                or INTERNET_FLAG_NO_COOKIES
                                or INTERNET_FLAG_NO_AUTH
                                or INTERNET_FLAG_CACHE_IF_NET_FAIL
                                or SECURITY_INTERNET_MASK
                                or INTERNET_FLAG_RESYNCHRONIZE
                                or INTERNET_FLAG_HYPERLINK
                                or INTERNET_FLAG_NO_UI
                                or INTERNET_FLAG_PRAGMA_NOCACHE
                                or INTERNET_FLAG_CACHE_ASYNC
                                or INTERNET_FLAG_FORMS_SUBMIT
                                or INTERNET_FLAG_NEED_FILE
                                or INTERNET_FLAG_RESTRICTED_ZONE
                                or INTERNET_FLAG_TRANSFER_BINARY
                                or INTERNET_FLAG_TRANSFER_ASCII
                                or INTERNET_FLAG_FWD_BACK
                                or INTERNET_FLAG_BGUPDATE
                                );

     INTERNET_ERROR_MASK_INSERT_CDROM                    = $1;
     INTERNET_ERROR_MASK_COMBINED_SEC_CERT               = $2;
     INTERNET_ERROR_MASK_NEED_MSN_SSPI_PKG               = $4;
     INTERNET_ERROR_MASK_LOGIN_FAILURE_DISPLAY_ENTITY_BODY = $8;

     INTERNET_OPTIONS_MASK   = (NOT INTERNET_FLAGS_MASK);

//
// common per-API flags (new APIs)
//

     WININET_API_FLAG_ASYNC          = $00000001;  // force async operation
     WININET_API_FLAG_SYNC           = $00000004;  // force sync operation
     WININET_API_FLAG_USE_CONTEXT    = $00000008;  // use value supplied in dwContext (even if 0)

//
// INTERNET_NO_CALLBACK - if this value is presented as the dwContext parameter
// then no call-backs will be made for that API
//

     INTERNET_NO_CALLBACK            = 0;

//
// structures/types
//

//
// INTERNET_SCHEME - enumerated URL scheme type
//

Type
  INTERNET_SCHEME = (
    INTERNET_SCHEME_PARTIAL = -2,
    INTERNET_SCHEME_UNKNOWN = -1,
    INTERNET_SCHEME_DEFAULT = 0,
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
    INTERNET_SCHEME_RES
	);	
  LPINTERNET_SCHEME =  ^INTERNET_SCHEME;

Const
    INTERNET_SCHEME_FIRST = INTERNET_SCHEME_FTP;
    INTERNET_SCHEME_LAST = INTERNET_SCHEME_RES;

//
// INTERNET_DIAGNOSTIC_SOCKET_INFO.Flags definitions
//

     IDSI_FLAG_KEEP_ALIVE    = $00000001;  // set if from keep-alive pool
     IDSI_FLAG_SECURE        = $00000002;  // set if secure connection
     IDSI_FLAG_PROXY         = $00000004;  // set if using proxy
     IDSI_FLAG_TUNNEL        = $00000008;  // set if tunnelling through proxy

//
// Options used in INTERNET_PER_CONN_OPTON struct
//
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
// flags for INTERNET_CONNECTED_INFO dwFlags
//

//
// ISO_FORCE_DISCONNECTED - if set when putting Wininet into disconnected mode,
// all outstanding requests will be aborted with a cancelled error
//

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


//
// constants for InternetTimeFromSystemTime
//

     INTERNET_RFC1123_FORMAT    = 0;
     INTERNET_RFC1123_BUFSIZE   = 30;

//
// flags for InternetCrackUrl() and InternetCreateUrl()
//
     ICU_ESCAPE      = $80000000;  // (un)escape URL characters
     ICU_USERNAME    = $40000000;  // use internal username & password
//
// flags for InternetCanonicalizeUrl() and InternetCombineUrl()
//
     ICU_NO_ENCODE   = $20000000;  // Don't convert unsafe characters to escape sequence
     ICU_DECODE      = $10000000;  // Convert %XX escape sequences to characters
     ICU_NO_META     = $08000000;  // Don't convert .. etc. meta path sequences
     ICU_ENCODE_SPACES_ONLY = $04000000;  // Encode spaces only
     ICU_BROWSER_MODE = $02000000; // Special encode/decode rules for browser
     ICU_ENCODE_PERCENT      = $00001000;      // Encode any percent (ASCII25)
//
// access types for InternetOpen()
//
     INTERNET_OPEN_TYPE_PRECONFIG                    = 0;  // use registry configuration
     INTERNET_OPEN_TYPE_DIRECT                       = 1;   // direct to net
     INTERNET_OPEN_TYPE_PROXY                        = 3;   // via named proxy
     INTERNET_OPEN_TYPE_PRECONFIG_WITH_NO_AUTOPROXY  = 4;   // prevent using java/script/INS
//
// old names for access types
//
     PRE_CONFIG_INTERNET_ACCESS  = INTERNET_OPEN_TYPE_PRECONFIG;
     LOCAL_INTERNET_ACCESS       = INTERNET_OPEN_TYPE_DIRECT;
     CERN_PROXY_INTERNET_ACCESS  = INTERNET_OPEN_TYPE_PROXY;
//
// service types for InternetConnect()
//
     INTERNET_SERVICE_FTP    = 1;
     INTERNET_SERVICE_GOPHER = 2;
     INTERNET_SERVICE_HTTP   = 3;
//
// flags for InternetReadFileEx()
//
     IRF_ASYNC       = WININET_API_FLAG_ASYNC;
     IRF_SYNC        = WININET_API_FLAG_SYNC;
     IRF_USE_CONTEXT = WININET_API_FLAG_USE_CONTEXT;
     IRF_NO_WAIT     = $00000008;
//
// flags for InternetSetOptionEx()
//

     ISO_GLOBAL      = $00000001;  // modify option globally
     ISO_REGISTRY    = $00000002;  // write option to registry (where applicable)

     ISO_VALID_FLAGS = (ISO_GLOBAL or ISO_REGISTRY);

//
// options manifests for Internet{QueryorSet}Option
//

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
     INTERNET_OPTION_OFFLINE_MODE            = 26;
     INTERNET_OPTION_CACHE_STREAM_HANDLE     = 27;
     INTERNET_OPTION_USERNAME                = 28;
     INTERNET_OPTION_PASSWORD                = 29;
     INTERNET_OPTION_ASYNC                   = 30;
     INTERNET_OPTION_SECURITY_FLAGS          = 31;
     INTERNET_OPTION_SECURITY_CERTIFICATE_STRUCT= 32;
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
     INTERNET_OPTION_SECURITY_SELECT_CLIENT_CERT= 47;
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
     INTERNET_OPTION_HTTP_DECODING           = 65;
     INTERNET_OPTION_DIAGNOSTIC_SOCKET_INFO  = 67;
     INTERNET_OPTION_CODEPAGE                = 68;
     INTERNET_OPTION_CACHE_TIMESTAMPS        = 69;
     INTERNET_OPTION_DISABLE_AUTODIAL        = 70;
     INTERNET_OPTION_MAX_CONNS_PER_SERVER    = 73;
     INTERNET_OPTION_MAX_CONNS_PER_1_0_SERVER= 74;
     INTERNET_OPTION_PER_CONNECTION_OPTION   = 75;
     INTERNET_OPTION_DIGEST_AUTH_UNLOAD      = 76;
     INTERNET_OPTION_IGNORE_OFFLINE          = 77;
     INTERNET_OPTION_IDENTITY                = 78;
     INTERNET_OPTION_REMOVE_IDENTITY         = 79;
     INTERNET_OPTION_ALTER_IDENTITY          = 80;
     INTERNET_OPTION_SUPPRESS_BEHAVIOR       = 81;
     INTERNET_OPTION_AUTODIAL_MODE           = 82;
     INTERNET_OPTION_AUTODIAL_CONNECTION     = 83;
     INTERNET_OPTION_CLIENT_CERT_CONTEXT     = 84;
     INTERNET_OPTION_AUTH_FLAGS              = 85;
     INTERNET_OPTION_COOKIES_3RD_PARTY       = 86;
     INTERNET_OPTION_DISABLE_PASSPORT_AUTH   = 87;
     INTERNET_OPTION_SEND_UTF8_SERVERNAME_TO_PROXY           = 88;
     INTERNET_OPTION_EXEMPT_CONNECTION_LIMIT  = 89;
     INTERNET_OPTION_ENABLE_PASSPORT_AUTH     = 90;
     INTERNET_OPTION_HIBERNATE_INACTIVE_WORKER_THREADS       = 91;
     INTERNET_OPTION_ACTIVATE_WORKER_THREADS                 = 92;
     INTERNET_OPTION_RESTORE_WORKER_THREAD_DEFAULTS          = 93;
     INTERNET_OPTION_SOCKET_SEND_BUFFER_LENGTH               = 94;
     INTERNET_OPTION_PROXY_SETTINGS_CHANGED                  = 95;
     INTERNET_OPTION_DATAFILE_EXT                            = 96;
     INTERNET_OPTION_CODEPAGE_PATH       =          100;
     INTERNET_OPTION_CODEPAGE_EXTRA      =          101;
     INTERNET_OPTION_IDN                 =          102;
     INTERNET_FIRST_OPTION               =     INTERNET_OPTION_CALLBACK;
     INTERNET_LAST_OPTION                =     INTERNET_OPTION_IDN;
//
// values for INTERNET_OPTION_PRIORITY
//
     INTERNET_PRIORITY_FOREGROUND            = 1000;
//
// handle types
//
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
     AUTH_FLAG_DISABLE_NEGOTIATE             = $00000001;
     AUTH_FLAG_ENABLE_NEGOTIATE              = $00000002;
     AUTH_FLAG_DISABLE_BASIC_CLEARCHANNEL    = $00000004;
//
// values for INTERNET_OPTION_SECURITY_FLAGS
//
// query only
     SECURITY_FLAG_SECURE                    = $00000001; // can query only
     SECURITY_FLAG_STRENGTH_WEAK             = $10000000;
     SECURITY_FLAG_STRENGTH_MEDIUM           = $40000000;
     SECURITY_FLAG_STRENGTH_STRONG           = $20000000;
     SECURITY_FLAG_UNKNOWNBIT                = $80000000;
     SECURITY_FLAG_FORTEZZA                  = $08000000;
     SECURITY_FLAG_NORMALBITNESS             = SECURITY_FLAG_STRENGTH_WEAK;

// The following are unused
     SECURITY_FLAG_SSL                       = $00000002;
     SECURITY_FLAG_SSL3                      = $00000004;
     SECURITY_FLAG_PCT                       = $00000008;
     SECURITY_FLAG_PCT4                      = $00000010;
     SECURITY_FLAG_IETFSSL4                  = $00000020;

// The following are for backwards compatability only.
     SECURITY_FLAG_40BIT                     = SECURITY_FLAG_STRENGTH_WEAK;
     SECURITY_FLAG_128BIT                    = SECURITY_FLAG_STRENGTH_STRONG;
     SECURITY_FLAG_56BIT                     = SECURITY_FLAG_STRENGTH_MEDIUM;


// valid autodial modes
     AUTODIAL_MODE_NEVER                     = 1;
     AUTODIAL_MODE_ALWAYS                    = 2;
     AUTODIAL_MODE_NO_NETWORK_PRESENT        = 4;

//
// status manifests for Internet status callback
//

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
     INTERNET_STATUS_PRIVACY_IMPACTED        = 324;
     INTERNET_STATUS_P3P_HEADER              = 325;
     INTERNET_STATUS_P3P_POLICYREF           = 326;
     INTERNET_STATUS_COOKIE_HISTORY          = 327;

//
// the following can be indicated in a state change notification:
//

     INTERNET_STATE_CONNECTED                = $00000001;  // connected state (mutually exclusive with disconnected)
     INTERNET_STATE_DISCONNECTED             = $00000002;  // disconnected from network
     INTERNET_STATE_DISCONNECTED_BY_USER     = $00000010;  // disconnected by user request
     INTERNET_STATE_IDLE                     = $00000100;  // no network requests being made (by Wininet)
     INTERNET_STATE_BUSY                     = $00000200;  // network requests being made (by Wininet)

//
// the following values are used for cookie state:
//

Type
   InternetCookieState = (
    COOKIE_STATE_UNKNOWN        =  $0,

    COOKIE_STATE_ACCEPT         =  $1,
    COOKIE_STATE_PROMPT         =  $2,
    COOKIE_STATE_LEASH          =  $3,
    COOKIE_STATE_DOWNGRADE      =  $4,
    COOKIE_STATE_REJECT         =  $5
	);


Const
    COOKIE_STATE_MAX            = COOKIE_STATE_REJECT;

//
// if the following value is returned by InternetSetStatusCallback, then
// probably an invalid (non-code) address was supplied for the callback
//
   INTERNET_INVALID_STATUS_CALLBACK        = {INTERNET_STATUS_CALLBACK} pointer(-1);
//
// FTP
//

     FTP_TRANSFER_TYPE_MASK      = (FTP_TRANSFER_TYPE_ASCII or FTP_TRANSFER_TYPE_BINARY);
//
// Gopher
//

//
// string field lengths (in characters, not bytes)
//

     MAX_GOPHER_DISPLAY_TEXT     = 128;
     MAX_GOPHER_SELECTOR_TEXT    = 256;
     MAX_GOPHER_HOST_NAME        = INTERNET_MAX_HOST_NAME_LENGTH;
     MAX_GOPHER_LOCATOR_LENGTH   = (1
                                    + MAX_GOPHER_DISPLAY_TEXT
                                    + 1
                                    + MAX_GOPHER_SELECTOR_TEXT
                                    + 1
                                    + MAX_GOPHER_HOST_NAME
                                    + 1
                                    + INTERNET_MAX_PORT_NUMBER_LENGTH
                                    + 1
                                    + 1
                                    + 2
                                    );

//
// manifests for GopherType
//

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
{
     IS_GOPHER_FILE(type)            (BOOL)(((type) & GOPHER_TYPE_FILE_MASK) ? TRUE : FALSE)
     IS_GOPHER_DIRECTORY(type)       (BOOL)(((type) & GOPHER_TYPE_DIRECTORY) ? TRUE : FALSE)
     IS_GOPHER_PHONE_SERVER(type)    (BOOL)(((type) & GOPHER_TYPE_CSO) ? TRUE : FALSE)
     IS_GOPHER_ERROR(type)           (BOOL)(((type) & GOPHER_TYPE_ERROR) ? TRUE : FALSE)
     IS_GOPHER_INDEX_SERVER(type)    (BOOL)(((type) & GOPHER_TYPE_INDEX_SERVER) ? TRUE : FALSE)
     IS_GOPHER_TELNET_SESSION(type)  (BOOL)(((type) & GOPHER_TYPE_TELNET) ? TRUE : FALSE)
     IS_GOPHER_BACKUP_SERVER(type)   (BOOL)(((type) & GOPHER_TYPE_REDUNDANT) ? TRUE : FALSE)
     IS_GOPHER_TN3270_SESSION(type)  (BOOL)(((type) & GOPHER_TYPE_TN3270) ? TRUE : FALSE)
     IS_GOPHER_ASK(type)             (BOOL)(((type) & GOPHER_TYPE_ASK) ? TRUE : FALSE)
     IS_GOPHER_PLUS(type)            (BOOL)(((type) & GOPHER_TYPE_GOPHER_PLUS) ? TRUE : FALSE)

     IS_GOPHER_TYPE_KNOWN(type)      (BOOL)(((type) & GOPHER_TYPE_UNKNOWN) ? FALSE : TRUE)
}
//
// GOPHER_TYPE_FILE_MASK - use this to determine if a locator identifies a
// (known) file type
//

     GOPHER_TYPE_FILE_MASK       = (GOPHER_TYPE_TEXT_FILE
                                    or GOPHER_TYPE_MAC_BINHEX
                                    or GOPHER_TYPE_DOS_ARCHIVE
                                    or GOPHER_TYPE_UNIX_UUENCODED
                                    or GOPHER_TYPE_BINARY
                                    or GOPHER_TYPE_GIF
                                    or GOPHER_TYPE_IMAGE
                                    or GOPHER_TYPE_BITMAP
                                    or GOPHER_TYPE_MOVIE
                                    or GOPHER_TYPE_SOUND
                                    or GOPHER_TYPE_HTML
                                    or GOPHER_TYPE_PDF
                                    or GOPHER_TYPE_CALENDAR
                                    or GOPHER_TYPE_INLINE
                                    );

     MAX_GOPHER_CATEGORY_NAME    = 128;     // arbitrary
     MAX_GOPHER_ATTRIBUTE_NAME   = 128;     //     "
     MIN_GOPHER_ATTRIBUTE_LENGTH = 256;     //     "

//
// known gopher attribute categories. See below for ordinals
//

     GOPHER_INFO_CATEGORY        = '+INFO';
     GOPHER_ADMIN_CATEGORY       = '+ADMIN';
     GOPHER_VIEWS_CATEGORY       = '+VIEWS';
     GOPHER_ABSTRACT_CATEGORY    = '+ABSTRACT';
     GOPHER_VERONICA_CATEGORY    = '+VERONICA';

//
// known gopher attributes. These are the attribute names as defined in the
// gopher+ protocol document
//

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

     GOPHER_ATTRIBUTE_ID_BASE        = $abcccc00;
     GOPHER_CATEGORY_ID_ALL          = (GOPHER_ATTRIBUTE_ID_BASE + 1);
     GOPHER_CATEGORY_ID_INFO         = (GOPHER_ATTRIBUTE_ID_BASE + 2);
     GOPHER_CATEGORY_ID_ADMIN        = (GOPHER_ATTRIBUTE_ID_BASE + 3);
     GOPHER_CATEGORY_ID_VIEWS        = (GOPHER_ATTRIBUTE_ID_BASE + 4);
     GOPHER_CATEGORY_ID_ABSTRACT     = (GOPHER_ATTRIBUTE_ID_BASE + 5);
     GOPHER_CATEGORY_ID_VERONICA     = (GOPHER_ATTRIBUTE_ID_BASE + 6);
     GOPHER_CATEGORY_ID_ASK          = (GOPHER_ATTRIBUTE_ID_BASE + 7);
     GOPHER_CATEGORY_ID_UNKNOWN      = (GOPHER_ATTRIBUTE_ID_BASE + 8);
     GOPHER_ATTRIBUTE_ID_ALL         = (GOPHER_ATTRIBUTE_ID_BASE + 9);
     GOPHER_ATTRIBUTE_ID_ADMIN       = (GOPHER_ATTRIBUTE_ID_BASE + 10);
     GOPHER_ATTRIBUTE_ID_MOD_DATE    = (GOPHER_ATTRIBUTE_ID_BASE + 11);
     GOPHER_ATTRIBUTE_ID_TTL         = (GOPHER_ATTRIBUTE_ID_BASE + 12);
     GOPHER_ATTRIBUTE_ID_SCORE       = (GOPHER_ATTRIBUTE_ID_BASE + 13);
     GOPHER_ATTRIBUTE_ID_RANGE       = (GOPHER_ATTRIBUTE_ID_BASE + 14);
     GOPHER_ATTRIBUTE_ID_SITE        = (GOPHER_ATTRIBUTE_ID_BASE + 15);
     GOPHER_ATTRIBUTE_ID_ORG         = (GOPHER_ATTRIBUTE_ID_BASE + 16);
     GOPHER_ATTRIBUTE_ID_LOCATION    = (GOPHER_ATTRIBUTE_ID_BASE + 17);
     GOPHER_ATTRIBUTE_ID_GEOG        = (GOPHER_ATTRIBUTE_ID_BASE + 18);
     GOPHER_ATTRIBUTE_ID_TIMEZONE    = (GOPHER_ATTRIBUTE_ID_BASE + 19);
     GOPHER_ATTRIBUTE_ID_PROVIDER    = (GOPHER_ATTRIBUTE_ID_BASE + 20);
     GOPHER_ATTRIBUTE_ID_VERSION     = (GOPHER_ATTRIBUTE_ID_BASE + 21);
     GOPHER_ATTRIBUTE_ID_ABSTRACT    = (GOPHER_ATTRIBUTE_ID_BASE + 22);
     GOPHER_ATTRIBUTE_ID_VIEW        = (GOPHER_ATTRIBUTE_ID_BASE + 23);
     GOPHER_ATTRIBUTE_ID_TREEWALK    = (GOPHER_ATTRIBUTE_ID_BASE + 24);
     GOPHER_ATTRIBUTE_ID_UNKNOWN     = (GOPHER_ATTRIBUTE_ID_BASE + 25);

//
// HTTP
//

//
// the default major/minor HTTP version numbers
//

     HTTP_MAJOR_VERSION      = 1;
     HTTP_MINOR_VERSION      = 0;

     HTTP_VERSIONA           = 'HTTP/1.0';
<<<<<<< HEAD
<<<<<<< HEAD
     HTTP_VERSIONW           = 'HTTP/1.0'#$0000; // force unicode url.
=======
     HTTP_VERSIONW           : widestring = 'HTTP/1.0';
>>>>>>> graemeg/fixes_2_2
=======
     HTTP_VERSIONW           : widestring = 'HTTP/1.0';
>>>>>>> origin/fixes_2_2
     {$ifdef UNICODE}
       HTTP_VERSION = HTTP_VERSIONW;
     {$ELSE}
       HTTP_VERSION = HTTP_VERSIONA;
     {$ENDIF}


//
// HttpQueryInfo info levels. Generally, there is one info level
// for each potential RFC822/HTTP/MIME header that an HTTP server
// may send as part of a request response.
//
// The HTTP_QUERY_RAW_HEADERS info level is provided for clients
// that choose to perform their own header parsing.
//

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


     HTTP_QUERY_MODIFIER_FLAGS_MASK          = (HTTP_QUERY_FLAG_REQUEST_HEADERS
                                                or HTTP_QUERY_FLAG_SYSTEMTIME
                                                or HTTP_QUERY_FLAG_NUMBER
                                                or HTTP_QUERY_FLAG_COALESCE
                                                );

     HTTP_QUERY_HEADER_MASK                  = ( not HTTP_QUERY_MODIFIER_FLAGS_MASK);

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
// values for dwModifiers parameter of HttpAddRequestHeaders()
//

     HTTP_ADDREQ_INDEX_MASK      = $0000FFFF;
     HTTP_ADDREQ_FLAGS_MASK      = $FFFF0000;

//
// HTTP_ADDREQ_FLAG_ADD_IF_NEW - the header will only be added if it doesn't
// already exist
//

     HTTP_ADDREQ_FLAG_ADD_IF_NEW = $10000000;

//
// HTTP_ADDREQ_FLAG_ADD - if HTTP_ADDREQ_FLAG_REPLACE is set but the header is
// not found then if this flag is set, the header is added anyway, so long as
// there is a valid header-value
//

     HTTP_ADDREQ_FLAG_ADD        = $20000000;

//
// HTTP_ADDREQ_FLAG_COALESCE - coalesce headers with same name. e.g.
// "Accept: text/*" and "Accept: audio/*" with this flag results in a single
// header: "Accept: text/*, audio/*"
//

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

     HTTP_ADDREQ_FLAG_REPLACE    = $80000000;


//
// flags for HttpSendRequestEx(), HttpEndRequest()
//

     HSR_ASYNC       = WININET_API_FLAG_ASYNC;          // force async
     HSR_SYNC        = WININET_API_FLAG_SYNC;           // force sync
     HSR_USE_CONTEXT = WININET_API_FLAG_USE_CONTEXT;    // use dwContext value
     HSR_INITIATE    = $00000008;                      // iterative operation (completed by HttpEndRequest)
     HSR_DOWNLOAD    = $00000010;                      // download to file
     HSR_CHUNKED     = $00000020;                      // operation is send of chunked data

//
// Cookie APIs
//


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

     FLAG_ICC_FORCE_CONNECTION       = $00000001;

//
// Internet UI
//

//
// InternetErrorDlg - Provides UI for certain Errors.
//

     FLAGS_ERROR_UI_FILTER_FOR_ERRORS        = $01;
     FLAGS_ERROR_UI_FLAGS_CHANGE_OPTIONS     = $02;
     FLAGS_ERROR_UI_FLAGS_GENERATE_DATA      = $04;
     FLAGS_ERROR_UI_FLAGS_NO_UI              = $08;
     FLAGS_ERROR_UI_SERIALIZE_DIALOGS        = $10;

//
// If SERIALIZE_DIALOGS flag set, client should implement thread-safe non-blocking callback...
//

//#if !defined(_WINERROR_)

//
// Internet API error returns
//

     INTERNET_ERROR_BASE                     = 12000;
     ERROR_INTERNET_OUT_OF_HANDLES           = (INTERNET_ERROR_BASE + 1);
     ERROR_INTERNET_TIMEOUT                  = (INTERNET_ERROR_BASE + 2);
     ERROR_INTERNET_EXTENDED_ERROR           = (INTERNET_ERROR_BASE + 3);
     ERROR_INTERNET_INTERNAL_ERROR           = (INTERNET_ERROR_BASE + 4);
     ERROR_INTERNET_INVALID_URL              = (INTERNET_ERROR_BASE + 5);
     ERROR_INTERNET_UNRECOGNIZED_SCHEME      = (INTERNET_ERROR_BASE + 6);
     ERROR_INTERNET_NAME_NOT_RESOLVED        = (INTERNET_ERROR_BASE + 7);
     ERROR_INTERNET_PROTOCOL_NOT_FOUND       = (INTERNET_ERROR_BASE + 8);
     ERROR_INTERNET_INVALID_OPTION           = (INTERNET_ERROR_BASE + 9);
     ERROR_INTERNET_BAD_OPTION_LENGTH        = (INTERNET_ERROR_BASE + 10);
     ERROR_INTERNET_OPTION_NOT_SETTABLE      = (INTERNET_ERROR_BASE + 11);
     ERROR_INTERNET_SHUTDOWN                 = (INTERNET_ERROR_BASE + 12);
     ERROR_INTERNET_INCORRECT_USER_NAME      = (INTERNET_ERROR_BASE + 13);
     ERROR_INTERNET_INCORRECT_PASSWORD       = (INTERNET_ERROR_BASE + 14);
     ERROR_INTERNET_LOGIN_FAILURE            = (INTERNET_ERROR_BASE + 15);
     ERROR_INTERNET_INVALID_OPERATION        = (INTERNET_ERROR_BASE + 16);
     ERROR_INTERNET_OPERATION_CANCELLED      = (INTERNET_ERROR_BASE + 17);
     ERROR_INTERNET_INCORRECT_HANDLE_TYPE    = (INTERNET_ERROR_BASE + 18);
     ERROR_INTERNET_INCORRECT_HANDLE_STATE   = (INTERNET_ERROR_BASE + 19);
     ERROR_INTERNET_NOT_PROXY_REQUEST        = (INTERNET_ERROR_BASE + 20);
     ERROR_INTERNET_REGISTRY_VALUE_NOT_FOUND = (INTERNET_ERROR_BASE + 21);
     ERROR_INTERNET_BAD_REGISTRY_PARAMETER   = (INTERNET_ERROR_BASE + 22);
     ERROR_INTERNET_NO_DIRECT_ACCESS         = (INTERNET_ERROR_BASE + 23);
     ERROR_INTERNET_NO_CONTEXT               = (INTERNET_ERROR_BASE + 24);
     ERROR_INTERNET_NO_CALLBACK              = (INTERNET_ERROR_BASE + 25);
     ERROR_INTERNET_REQUEST_PENDING          = (INTERNET_ERROR_BASE + 26);
     ERROR_INTERNET_INCORRECT_FORMAT         = (INTERNET_ERROR_BASE + 27);
     ERROR_INTERNET_ITEM_NOT_FOUND           = (INTERNET_ERROR_BASE + 28);
     ERROR_INTERNET_CANNOT_CONNECT           = (INTERNET_ERROR_BASE + 29);
     ERROR_INTERNET_CONNECTION_ABORTED       = (INTERNET_ERROR_BASE + 30);
     ERROR_INTERNET_CONNECTION_RESET         = (INTERNET_ERROR_BASE + 31);
     ERROR_INTERNET_FORCE_RETRY              = (INTERNET_ERROR_BASE + 32);
     ERROR_INTERNET_INVALID_PROXY_REQUEST    = (INTERNET_ERROR_BASE + 33);
     ERROR_INTERNET_NEED_UI                  = (INTERNET_ERROR_BASE + 34);

     ERROR_INTERNET_HANDLE_EXISTS            = (INTERNET_ERROR_BASE + 36);
     ERROR_INTERNET_SEC_CERT_DATE_INVALID    = (INTERNET_ERROR_BASE + 37);
     ERROR_INTERNET_SEC_CERT_CN_INVALID      = (INTERNET_ERROR_BASE + 38);
     ERROR_INTERNET_HTTP_TO_HTTPS_ON_REDIR   = (INTERNET_ERROR_BASE + 39);
     ERROR_INTERNET_HTTPS_TO_HTTP_ON_REDIR   = (INTERNET_ERROR_BASE + 40);
     ERROR_INTERNET_MIXED_SECURITY           = (INTERNET_ERROR_BASE + 41);
     ERROR_INTERNET_CHG_POST_IS_NON_SECURE   = (INTERNET_ERROR_BASE + 42);
     ERROR_INTERNET_POST_IS_NON_SECURE       = (INTERNET_ERROR_BASE + 43);
     ERROR_INTERNET_CLIENT_AUTH_CERT_NEEDED  = (INTERNET_ERROR_BASE + 44);
     ERROR_INTERNET_INVALID_CA               = (INTERNET_ERROR_BASE + 45);
     ERROR_INTERNET_CLIENT_AUTH_NOT_SETUP    = (INTERNET_ERROR_BASE + 46);
     ERROR_INTERNET_ASYNC_THREAD_FAILED      = (INTERNET_ERROR_BASE + 47);
     ERROR_INTERNET_REDIRECT_SCHEME_CHANGE   = (INTERNET_ERROR_BASE + 48);
     ERROR_INTERNET_DIALOG_PENDING           = (INTERNET_ERROR_BASE + 49);
     ERROR_INTERNET_RETRY_DIALOG             = (INTERNET_ERROR_BASE + 50);
     ERROR_INTERNET_HTTPS_HTTP_SUBMIT_REDIR  = (INTERNET_ERROR_BASE + 52);
     ERROR_INTERNET_INSERT_CDROM             = (INTERNET_ERROR_BASE + 53);
     ERROR_INTERNET_FORTEZZA_LOGIN_NEEDED    = (INTERNET_ERROR_BASE + 54);
     ERROR_INTERNET_SEC_CERT_ERRORS          = (INTERNET_ERROR_BASE + 55);
     ERROR_INTERNET_SEC_CERT_NO_REV          = (INTERNET_ERROR_BASE + 56);
     ERROR_INTERNET_SEC_CERT_REV_FAILED      = (INTERNET_ERROR_BASE + 57);

//
// FTP API errors
//

     ERROR_FTP_TRANSFER_IN_PROGRESS          = (INTERNET_ERROR_BASE + 110);
     ERROR_FTP_DROPPED                       = (INTERNET_ERROR_BASE + 111);
     ERROR_FTP_NO_PASSIVE_MODE               = (INTERNET_ERROR_BASE + 112);

//
// gopher API errors
//

     ERROR_GOPHER_PROTOCOL_ERROR             = (INTERNET_ERROR_BASE + 130);
     ERROR_GOPHER_NOT_FILE                   = (INTERNET_ERROR_BASE + 131);
     ERROR_GOPHER_DATA_ERROR                 = (INTERNET_ERROR_BASE + 132);
     ERROR_GOPHER_END_OF_DATA                = (INTERNET_ERROR_BASE + 133);
     ERROR_GOPHER_INVALID_LOCATOR            = (INTERNET_ERROR_BASE + 134);
     ERROR_GOPHER_INCORRECT_LOCATOR_TYPE     = (INTERNET_ERROR_BASE + 135);
     ERROR_GOPHER_NOT_GOPHER_PLUS            = (INTERNET_ERROR_BASE + 136);
     ERROR_GOPHER_ATTRIBUTE_NOT_FOUND        = (INTERNET_ERROR_BASE + 137);
     ERROR_GOPHER_UNKNOWN_LOCATOR            = (INTERNET_ERROR_BASE + 138);

//
// HTTP API errors
//

     ERROR_HTTP_HEADER_NOT_FOUND             = (INTERNET_ERROR_BASE + 150);
     ERROR_HTTP_DOWNLEVEL_SERVER             = (INTERNET_ERROR_BASE + 151);
     ERROR_HTTP_INVALID_SERVER_RESPONSE      = (INTERNET_ERROR_BASE + 152);
     ERROR_HTTP_INVALID_HEADER               = (INTERNET_ERROR_BASE + 153);
     ERROR_HTTP_INVALID_QUERY_REQUEST        = (INTERNET_ERROR_BASE + 154);
     ERROR_HTTP_HEADER_ALREADY_EXISTS        = (INTERNET_ERROR_BASE + 155);
     ERROR_HTTP_REDIRECT_FAILED              = (INTERNET_ERROR_BASE + 156);
     ERROR_HTTP_NOT_REDIRECTED               = (INTERNET_ERROR_BASE + 160);
     ERROR_HTTP_COOKIE_NEEDS_CONFIRMATION    = (INTERNET_ERROR_BASE + 161);
     ERROR_HTTP_COOKIE_DECLINED              = (INTERNET_ERROR_BASE + 162);
     ERROR_HTTP_REDIRECT_NEEDS_CONFIRMATION  = (INTERNET_ERROR_BASE + 168);

//
// additional Internet API error codes
//

     ERROR_INTERNET_SECURITY_CHANNEL_ERROR   = (INTERNET_ERROR_BASE + 157);
     ERROR_INTERNET_UNABLE_TO_CACHE_FILE     = (INTERNET_ERROR_BASE + 158);
     ERROR_INTERNET_TCPIP_NOT_INSTALLED      = (INTERNET_ERROR_BASE + 159);
     ERROR_INTERNET_DISCONNECTED             = (INTERNET_ERROR_BASE + 163);
     ERROR_INTERNET_SERVER_UNREACHABLE       = (INTERNET_ERROR_BASE + 164);
     ERROR_INTERNET_PROXY_SERVER_UNREACHABLE = (INTERNET_ERROR_BASE + 165);

     ERROR_INTERNET_BAD_AUTO_PROXY_SCRIPT    = (INTERNET_ERROR_BASE + 166);
     ERROR_INTERNET_UNABLE_TO_DOWNLOAD_SCRIPT = (INTERNET_ERROR_BASE + 167);
     ERROR_INTERNET_SEC_INVALID_CERT         = (INTERNET_ERROR_BASE + 169);
     ERROR_INTERNET_SEC_CERT_REVOKED         = (INTERNET_ERROR_BASE + 170);

// InternetAutodial specific errors

     ERROR_INTERNET_FAILED_DUETOSECURITYCHECK  = (INTERNET_ERROR_BASE + 171);
     ERROR_INTERNET_NOT_INITIALIZED          = (INTERNET_ERROR_BASE + 172);
     ERROR_INTERNET_NEED_MSN_SSPI_PKG          = (INTERNET_ERROR_BASE + 173);
     ERROR_INTERNET_LOGIN_FAILURE_DISPLAY_ENTITY_BODY   = (INTERNET_ERROR_BASE + 174);

// Decoding/Decompression specific errors

     ERROR_INTERNET_DECODING_FAILED          = (INTERNET_ERROR_BASE + 175);


     INTERNET_ERROR_LAST                     = ERROR_INTERNET_DECODING_FAILED;



//
// URLCACHE APIs
//

//
// datatype definitions.
//

//
// cache entry type flags.
//

     NORMAL_CACHE_ENTRY              = $00000001;
     STICKY_CACHE_ENTRY              = $00000004;
     EDITED_CACHE_ENTRY              = $00000008;
     TRACK_OFFLINE_CACHE_ENTRY       = $00000010;
     TRACK_ONLINE_CACHE_ENTRY        = $00000020;
     SPARSE_CACHE_ENTRY              = $00010000;
     COOKIE_CACHE_ENTRY              = $00100000;
     URLHISTORY_CACHE_ENTRY          = $00200000;


     URLCACHE_FIND_DEFAULT_FILTER    = NORMAL_CACHE_ENTRY
                                    or   COOKIE_CACHE_ENTRY
                                    or   URLHISTORY_CACHE_ENTRY
                                    or   TRACK_OFFLINE_CACHE_ENTRY
                                    or   TRACK_ONLINE_CACHE_ENTRY
                                    or   STICKY_CACHE_ENTRY;



//
// INTERNET_CACHE_ENTRY_INFO -
//

//
// Cache Group Flags
//
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
     CACHEGROUP_READWRITE_MASK                   =
            CACHEGROUP_ATTRIBUTE_TYPE
        or   CACHEGROUP_ATTRIBUTE_QUOTA
		or   CACHEGROUP_ATTRIBUTE_GROUPNAME
        or   CACHEGROUP_ATTRIBUTE_STORAGE;
//
// INTERNET_CACHE_GROUP_INFO
//
      GROUPNAME_MAX_LENGTH       = 120;
      GROUP_OWNER_STORAGE_SIZE   = 4;

     CACHE_ENTRY_ATTRIBUTE_FC    = $00000004;
     CACHE_ENTRY_HITRATE_FC      = $00000010;
     CACHE_ENTRY_MODTIME_FC      = $00000040;
     CACHE_ENTRY_EXPTIME_FC      = $00000080;
     CACHE_ENTRY_ACCTIME_FC      = $00000100;
     CACHE_ENTRY_SYNCTIME_FC     = $00000200;
     CACHE_ENTRY_HEADERINFO_FC   = $00000400;
     CACHE_ENTRY_EXEMPT_DELTA_FC = $00000800;


// Flags for SetUrlCacheEntryGroup
     INTERNET_CACHE_GROUP_ADD      = 0;
     INTERNET_CACHE_GROUP_REMOVE   = 1;

// Flags for InternetDial - must not conflict with InternetAutodial flags
//                          as they are valid here also.
     INTERNET_DIAL_FORCE_PROMPT     = $2000;
     INTERNET_DIAL_SHOW_OFFLINE     = $4000;
     INTERNET_DIAL_UNATTENDED       = $8000;

     INTERENT_GOONLINE_REFRESH = $00000001;
     INTERENT_GOONLINE_MASK = $00000001;

// Flags for InternetAutodial
     INTERNET_AUTODIAL_FORCE_ONLINE          = 1;
     INTERNET_AUTODIAL_FORCE_UNATTENDED      = 2;
     INTERNET_AUTODIAL_FAILIFSECURITYCHECK   = 4;
     INTERNET_AUTODIAL_OVERRIDE_NET_PRESENT  = 8;

     INTERNET_AUTODIAL_FLAGS_MASK = (INTERNET_AUTODIAL_FORCE_ONLINE or INTERNET_AUTODIAL_FORCE_UNATTENDED or INTERNET_AUTODIAL_FAILIFSECURITYCHECK or INTERNET_AUTODIAL_OVERRIDE_NET_PRESENT);

      PROXY_AUTO_DETECT_TYPE_DHCP    = 1;
      PROXY_AUTO_DETECT_TYPE_DNS_A   = 2;

// Flags for InternetGetConnectedState and Ex
     INTERNET_CONNECTION_MODEM           = $01;
     INTERNET_CONNECTION_LAN             = $02;
     INTERNET_CONNECTION_PROXY           = $04;
     INTERNET_CONNECTION_MODEM_BUSY      = $08;  {no longer used }
     INTERNET_RAS_INSTALLED              = $10;
     INTERNET_CONNECTION_OFFLINE         = $20;
     INTERNET_CONNECTION_CONFIGURED      = $40;

// Flags for custom dial handler
     INTERNET_CUSTOMDIAL_CONNECT         = 0;
     INTERNET_CUSTOMDIAL_UNATTENDED      = 1;
     INTERNET_CUSTOMDIAL_DISCONNECT      = 2;
     INTERNET_CUSTOMDIAL_SHOWOFFLINE     = 4;

// Custom dial handler supported functionality flags
     INTERNET_CUSTOMDIAL_SAFE_FOR_UNATTENDED = 1;
     INTERNET_CUSTOMDIAL_WILL_SUPPLY_STATE   = 2;
     INTERNET_CUSTOMDIAL_CAN_HANGUP          = 4;

// States for InternetSetDialState
     INTERNET_DIALSTATE_DISCONNECTED     = 1;

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
	
Type	
	
<<<<<<< HEAD
<<<<<<< HEAD
     INTERNET_ASYNC_RESULT = record
=======
     INTERNET_ASYNC_RESULT = packed record
>>>>>>> graemeg/fixes_2_2
=======
     INTERNET_ASYNC_RESULT = packed record
>>>>>>> origin/fixes_2_2
          dwResult : DWORD_PTR;
          dwError : DWORD;
       end;
     TINTERNET_ASYNC_RESULT = INTERNET_ASYNC_RESULT;
     LPINTERNET_ASYNC_RESULT = ^INTERNET_ASYNC_RESULT;
     PINTERNET_ASYNC_RESULT = LPINTERNET_ASYNC_RESULT;
     TINTERNETASYNCRESULT = TINTERNET_ASYNC_RESULT;
     LPINTERNETASYNCRESULT =LPINTERNET_ASYNC_RESULT;
     PINTERNETASYNCRESULT = PINTERNET_ASYNC_RESULT;	

  { INTERNET_DIAGNOSTIC_SOCKET_INFO - info about the socket in use }
<<<<<<< HEAD
<<<<<<< HEAD
     INTERNET_DIAGNOSTIC_SOCKET_INFO = record
=======
     INTERNET_DIAGNOSTIC_SOCKET_INFO =packed  record
>>>>>>> graemeg/fixes_2_2
=======
     INTERNET_DIAGNOSTIC_SOCKET_INFO =packed  record
>>>>>>> origin/fixes_2_2
          Socket : DWORD_PTR;
          SourcePort : DWORD;
          DestPort : DWORD;
          Flags : DWORD;
       end;
	 TINTERNET_DIAGNOSTIC_SOCKET_INFO = INTERNET_DIAGNOSTIC_SOCKET_INFO;
     LPINTERNET_DIAGNOSTIC_SOCKET_INFO = ^INTERNET_DIAGNOSTIC_SOCKET_INFO;
     PINTERNET_DIAGNOSTIC_SOCKET_INFO = LPINTERNET_DIAGNOSTIC_SOCKET_INFO;	

<<<<<<< HEAD
<<<<<<< HEAD
    INTERNET_PREFETCH_STATUS = record
=======
    INTERNET_PREFETCH_STATUS = packed record
>>>>>>> graemeg/fixes_2_2
=======
    INTERNET_PREFETCH_STATUS = packed record
>>>>>>> origin/fixes_2_2
    			dwStatus,
			dwSize : DWord;
                        end;
    TINTERNET_PREFETCH_STATUS= INTERNET_PREFETCH_STATUS; 
    LPINTERNET_PREFETCH_STATUS= ^INTERNET_PREFETCH_STATUS; 
    PINTERNET_PREFETCH_STATUS= LPINTERNET_PREFETCH_STATUS; 
    TINTERNETPREFETCHSTATUS= TINTERNET_PREFETCH_STATUS; 
    PINTERNETPREFETCHSTATUS= PINTERNET_PREFETCH_STATUS; 
    LPINTERNETPREFETCHSTATUS= PINTERNET_PREFETCH_STATUS; 

<<<<<<< HEAD
<<<<<<< HEAD
     INTERNET_PROXY_INFO =  record
=======
     INTERNET_PROXY_INFO = packed record
>>>>>>> graemeg/fixes_2_2
=======
     INTERNET_PROXY_INFO = packed record
>>>>>>> origin/fixes_2_2
          dwAccessType : DWORD;
          lpszProxy : LPCTSTR;
          lpszProxyBypass : LPCTSTR;
       end;
     TINTERNET_PROXY_INFO = INTERNET_PROXY_INFO ;
     LPINTERNET_PROXY_INFO = ^INTERNET_PROXY_INFO;
     PINTERNET_PROXY_INFO = LPINTERNET_PROXY_INFO;
     TINTERNETPROXYINFO =   TINTERNET_PROXY_INFO;
     LPINTERNETPROXYINFO =  LPINTERNET_PROXY_INFO;
     PINTERNETPROXYINFO =   PINTERNET_PROXY_INFO;

     INTERNET_PER_CONN_OPTIONA = record
          dwOption : DWORD;
          Value : record
              case longint of
                 0 : ( dwValue : DWORD );
                 1 : ( pszValue : LPSTR );
                 2 : ( ftValue : FILETIME );
              end;
       end;
	 TINTERNET_PER_CONN_OPTIONA   = INTERNET_PER_CONN_OPTIONA;
     LPINTERNET_PER_CONN_OPTIONA = ^INTERNET_PER_CONN_OPTIONA;
     PINTERNET_PER_CONN_OPTIONA = LPINTERNET_PER_CONN_OPTIONA;	
     INTERNET_PER_CONN_OPTIONW = record
          dwOption : DWORD;
          Value : record
              case longint of
                 0 : ( dwValue : DWORD );
                 1 : ( pszValue : LPWSTR );
                 2 : ( ftValue : FILETIME );
              end;
       end;
	 TINTERNET_PER_CONN_OPTIONW = INTERNET_PER_CONN_OPTIONW;
     LPINTERNET_PER_CONN_OPTIONW = ^INTERNET_PER_CONN_OPTIONW;	
     PINTERNET_PER_CONN_OPTIONW = LPINTERNET_PER_CONN_OPTIONW;
	
{$ifdef UNICODE}
     INTERNET_PER_CONN_OPTION = INTERNET_PER_CONN_OPTIONW;
     LPINTERNET_PER_CONN_OPTION = LPINTERNET_PER_CONN_OPTIONW;
	 TINTERNET_PER_CONN_OPTION = INTERNET_PER_CONN_OPTIONW;
     PINTERNET_PER_CONN_OPTION = LPINTERNET_PER_CONN_OPTIONW;
{$else}
     INTERNET_PER_CONN_OPTION = INTERNET_PER_CONN_OPTIONA;
     LPINTERNET_PER_CONN_OPTION = LPINTERNET_PER_CONN_OPTIONA;
     TINTERNET_PER_CONN_OPTION = INTERNET_PER_CONN_OPTIONA;
     PINTERNET_PER_CONN_OPTION = LPINTERNET_PER_CONN_OPTIONA;	
{$endif}

     INTERNET_PER_CONN_OPTION_LISTA = record
          dwSize : DWORD;
          pszConnection : LPSTR;
          dwOptionCount : DWORD;
          dwOptionError : DWORD;
          pOptions : LPINTERNET_PER_CONN_OPTIONA;
       end;
	 TINTERNET_PER_CONN_OPTION_LISTA  = INTERNET_PER_CONN_OPTION_LISTA ;
     LPINTERNET_PER_CONN_OPTION_LISTA = ^INTERNET_PER_CONN_OPTION_LISTA;
     PINTERNET_PER_CONN_OPTION_LISTA  = LPINTERNET_PER_CONN_OPTION_LISTA;	

     INTERNET_PER_CONN_OPTION_LISTW = record
          dwSize : DWORD;
          pszConnection : LPWSTR;
          dwOptionCount : DWORD;
          dwOptionError : DWORD;
          pOptions : LPINTERNET_PER_CONN_OPTIONW;
       end;
     TINTERNET_PER_CONN_OPTION_LISTW  = INTERNET_PER_CONN_OPTION_LISTW;
     LPINTERNET_PER_CONN_OPTION_LISTW = ^INTERNET_PER_CONN_OPTION_LISTW;
	 PINTERNET_PER_CONN_OPTION_LISTW  = LPINTERNET_PER_CONN_OPTION_LISTW;
{$ifdef UNICODE}
     INTERNET_PER_CONN_OPTION_LIST = INTERNET_PER_CONN_OPTION_LISTW;
     LPINTERNET_PER_CONN_OPTION_LIST = LPINTERNET_PER_CONN_OPTION_LISTW;
     TINTERNET_PER_CONN_OPTION_LIST = INTERNET_PER_CONN_OPTION_LISTW;
     PINTERNET_PER_CONN_OPTION_LIST = LPINTERNET_PER_CONN_OPTION_LISTW;	
{$else}
     INTERNET_PER_CONN_OPTION_LIST = INTERNET_PER_CONN_OPTION_LISTA;
     LPINTERNET_PER_CONN_OPTION_LIST = LPINTERNET_PER_CONN_OPTION_LISTA;
     TINTERNET_PER_CONN_OPTION_LIST = INTERNET_PER_CONN_OPTION_LISTA;
     PINTERNET_PER_CONN_OPTION_LIST = LPINTERNET_PER_CONN_OPTION_LISTA;
{$endif}

     INTERNET_VERSION_INFO = record
          dwMajorVersion : DWORD;
          dwMinorVersion : DWORD;
       end;
     LPINTERNET_VERSION_INFO = ^INTERNET_VERSION_INFO;
     PINTERNET_VERSION_INFO = LPINTERNET_VERSION_INFO;	
     TINTERNET_VERSION_INFO = INTERNET_VERSION_INFO;
     LPINTERNETVERSIONINFO = LPINTERNET_VERSION_INFO;
     PINTERNETVERSIONINFO =  PINTERNET_VERSION_INFO;
     TINTERNETVERSIONINFO =  TINTERNET_VERSION_INFO;

     HTTP_VERSION_INFO = record
          dwMajorVersion : DWORD;
          dwMinorVersion : DWORD;
       end;
     THTTP_VERSION_INFO = HTTP_VERSION_INFO;
     LPHTTP_VERSION_INFO = ^HTTP_VERSION_INFO;
     PHTTP_VERSION_INFO = LPHTTP_VERSION_INFO;	
     LPHTTPVERSIONINFO = LPHTTP_VERSION_INFO;
     PHTTPVERSIONINFO =  PHTTP_VERSION_INFO;
     THTTPVERSIONINFO =  THTTP_VERSION_INFO;

     INTERNET_CONNECTED_INFO = record
          dwConnectedState : DWORD;
          dwFlags : DWORD;
       end;
     TINTERNET_CONNECTED_INFO = INTERNET_CONNECTED_INFO;
     LPINTERNET_CONNECTED_INFO = ^INTERNET_CONNECTED_INFO;
     PINTERNET_CONNECTED_INFO = LPINTERNET_CONNECTED_INFO;
     TINTERNETCONNECTEDINFO =  TINTERNET_CONNECTED_INFO;
     LPINTERNETCONNECTEDINFO = LPINTERNET_CONNECTED_INFO;
     PINTERNETCONNECTEDINFO =  PINTERNET_CONNECTED_INFO;

     URL_COMPONENTSA = record
          dwStructSize : DWORD;
          lpszScheme : LPSTR;
          dwSchemeLength : DWORD;
          nScheme : INTERNET_SCHEME;
          lpszHostName : LPSTR;
          dwHostNameLength : DWORD;
          nPort : INTERNET_PORT;
          lpszUserName : LPSTR;
          dwUserNameLength : DWORD;
          lpszPassword : LPSTR;
          dwPasswordLength : DWORD;
          lpszUrlPath : LPSTR;
          dwUrlPathLength : DWORD;
          lpszExtraInfo : LPSTR;
          dwExtraInfoLength : DWORD;
       end;
     LPURL_COMPONENTSA = ^URL_COMPONENTSA;
     PURL_COMPONENTSA = LPURL_COMPONENTSA;	
     TURL_COMPONENTSA = URL_COMPONENTSA;

     URL_COMPONENTSW = record
          dwStructSize : DWORD;
          lpszScheme : LPWSTR;
          dwSchemeLength : DWORD;
          nScheme : INTERNET_SCHEME;
          lpszHostName : LPWSTR;
          dwHostNameLength : DWORD;
          nPort : INTERNET_PORT;
          lpszUserName : LPWSTR;
          dwUserNameLength : DWORD;
          lpszPassword : LPWSTR;
          dwPasswordLength : DWORD;
          lpszUrlPath : LPWSTR;
          dwUrlPathLength : DWORD;
          lpszExtraInfo : LPWSTR;
          dwExtraInfoLength : DWORD;
       end;
	 TURL_COMPONENTSW = URL_COMPONENTSW;
     LPURL_COMPONENTSW = ^URL_COMPONENTSW;
     PURL_COMPONENTSW = LPURL_COMPONENTSW;	
	
{$ifdef UNICODE}
     URL_COMPONENTS = URL_COMPONENTSW;
     LPURL_COMPONENTS = LPURL_COMPONENTSW;
     TURL_COMPONENTS = URL_COMPONENTSW;
     PURL_COMPONENTS = LPURL_COMPONENTSW;
{$else}
     URL_COMPONENTS = URL_COMPONENTSA;
     LPURL_COMPONENTS = LPURL_COMPONENTSA;
     TURL_COMPONENTS = URL_COMPONENTSA;
     PURL_COMPONENTS = LPURL_COMPONENTSA;
{$endif}
     TURLComponents  = TURL_COMPONENTS;
     LPURLCOMPONENTS = LPURL_COMPONENTS;
     PURLCOMPONENTS =  PURL_COMPONENTS;
     
     INTERNET_CERTIFICATE_INFO = record
          ftExpiry : FILETIME;
          ftStart : FILETIME;
          lpszSubjectInfo : LPTSTR;
          lpszIssuerInfo : LPTSTR;
          lpszProtocolName : LPTSTR;
          lpszSignatureAlgName : LPTSTR;
          lpszEncryptionAlgName : LPTSTR;
          dwKeySize : DWORD;
       end;
     TINTERNET_CERTIFICATE_INFO = INTERNET_CERTIFICATE_INFO;
     LPINTERNET_CERTIFICATE_INFO = ^INTERNET_CERTIFICATE_INFO;
     PINTERNET_CERTIFICATE_INFO = LPINTERNET_CERTIFICATE_INFO;
     TINTERNETCERTIFICATEINFO =  TINTERNET_CERTIFICATE_INFO;
     LPINTERNETCERTIFICATEINFO = LPINTERNET_CERTIFICATE_INFO;
     PINTERNETCERTIFICATEINFO =  PINTERNET_CERTIFICATE_INFO;

     LPINTERNET_BUFFERSA = ^_INTERNET_BUFFERSA;
     _INTERNET_BUFFERSA = record
          dwStructSize : DWORD;
          Next : LPINTERNET_BUFFERSA;
          lpcszHeader : LPCSTR;
          dwHeadersLength : DWORD;
          dwHeadersTotal : DWORD;
          lpvBuffer : LPVOID;
          dwBufferLength : DWORD;
          dwBufferTotal : DWORD;
          dwOffsetLow : DWORD;
          dwOffsetHigh : DWORD;
       end;
     INTERNET_BUFFERSA = _INTERNET_BUFFERSA;

     TINTERNET_BUFFERSA = _INTERNET_BUFFERSA;
     PINTERNET_BUFFERSA = LPINTERNET_BUFFERSA;
     TINTERNETBUFFERSA = TINTERNET_BUFFERSA;
     PINTERNETBUFFERSA = PINTERNET_BUFFERSA; 

     LPINTERNET_BUFFERSW = ^_INTERNET_BUFFERSW;
     _INTERNET_BUFFERSW = record
          dwStructSize : DWORD;
          Next : LPINTERNET_BUFFERSW;
          lpcszHeader : LPCWSTR;
          dwHeadersLength : DWORD;
          dwHeadersTotal : DWORD;
          lpvBuffer : LPVOID;
          dwBufferLength : DWORD;
          dwBufferTotal : DWORD;
          dwOffsetLow : DWORD;
          dwOffsetHigh : DWORD;
       end;
     INTERNET_BUFFERSW = _INTERNET_BUFFERSW;
     TINTERNET_BUFFERSW = _INTERNET_BUFFERSW;
     PINTERNET_BUFFERSW = LPINTERNET_BUFFERSW;
     TINTERNETBUFFERSW = TINTERNET_BUFFERSW;
     PINTERNETBUFFERSW = PINTERNET_BUFFERSW;

{$ifdef UNICODE}
     INTERNET_BUFFERS = INTERNET_BUFFERSW;
     LPINTERNET_BUFFERS = LPINTERNET_BUFFERSW;
     TINTERNET_BUFFERS = INTERNET_BUFFERSW;
     PINTERNET_BUFFERS = LPINTERNET_BUFFERSW;	
{$else}
     INTERNET_BUFFERS = INTERNET_BUFFERSA;
     LPINTERNET_BUFFERS = LPINTERNET_BUFFERSA;
     TINTERNET_BUFFERS = INTERNET_BUFFERSA;
     PINTERNET_BUFFERS = LPINTERNET_BUFFERSA;
{$endif}
     TINTERNETBUFFERS = TINTERNET_BUFFERS;
     PINTERNETBUFFERS = PINTERNET_BUFFERS;    

     IncomingCookieState = record
          cSession : longint;
          cPersistent : longint;
          cAccepted : longint;
          cLeashed : longint;
          cDowngraded : longint;
          cBlocked : longint;
          pszLocation : ^char;
       end;
     TIncomingCookieState = IncomingCookieState;
	 PIncomingCookieState = ^IncomingCookieState;
	 LPIncomingCookieState = PIncomingCookieState;
	
     OutgoingCookieState = record
          cSent : longint;
          cSuppressed : longint;
          pszLocation : ^char;
       end;
     ToutgoingCookieState = outgoingCookieState;
	 PoutgoingCookieState = ^outgoingCookieState;
	 LPoutgoingCookieState = PoutgoingCookieState;
	
     InternetCookieHistory = record
          fAccepted : BOOL;
          fLeashed : BOOL;
          fDowngraded : BOOL;
          fRejected : BOOL;
       end;
     TInternetCookieHistory = InternetCookieHistory;
	 PInternetCookieHistory = ^InternetCookieHistory;
	 LPInternetCookieHistory = PInternetCookieHistory;
	
     CookieDecision = record
          dwCookieState : DWORD;
          fAllowSession : BOOL;
       end;
     TCookieDecision = CookieDecision;
	 PCookieDecision = ^CookieDecision;
	 LPCookieDecision = PCookieDecision;
     GOPHER_FIND_DATAA = record
          DisplayString : array[0..(MAX_GOPHER_DISPLAY_TEXT+1)-1] of CHAR;
          GopherType : DWORD;
          SizeLow : DWORD;
          SizeHigh : DWORD;
          LastModificationTime : FILETIME;
          Locator : array[0..(MAX_GOPHER_LOCATOR_LENGTH+1)-1] of CHAR;
       end;
     LPGOPHER_FIND_DATAA = ^GOPHER_FIND_DATAA;
	 TGOPHER_FIND_DATAA = GOPHER_FIND_DATAA;
	 PGOPHER_FIND_DATAA = LPGOPHER_FIND_DATAA;

     GOPHER_FIND_DATAW = record
          DisplayString : array[0..(MAX_GOPHER_DISPLAY_TEXT+1)-1] of WCHAR;
          GopherType : DWORD;
          SizeLow : DWORD;
          SizeHigh : DWORD;
          LastModificationTime : FILETIME;
          Locator : array[0..(MAX_GOPHER_LOCATOR_LENGTH+1)-1] of WCHAR;
       end;

	 LPGOPHER_FIND_DATAW = ^GOPHER_FIND_DATAW;
	 TGOPHER_FIND_DATAW = GOPHER_FIND_DATAW;
	 PGOPHER_FIND_DATAW = LPGOPHER_FIND_DATAW;

{$ifdef UNICODE}
     GOPHER_FIND_DATA = GOPHER_FIND_DATAW;
     LPGOPHER_FIND_DATA = LPGOPHER_FIND_DATAW;
     TGOPHER_FIND_DATA = GOPHER_FIND_DATAW;
     PGOPHER_FIND_DATA = LPGOPHER_FIND_DATAW;
{$else}
     GOPHER_FIND_DATA = GOPHER_FIND_DATAA;
     LPGOPHER_FIND_DATA = LPGOPHER_FIND_DATAA;
     TGOPHER_FIND_DATA = GOPHER_FIND_DATAA;
     PGOPHER_FIND_DATA = LPGOPHER_FIND_DATAA;
{$endif}

     GOPHER_ADMIN_ATTRIBUTE_TYPE = record
          Comment : LPCTSTR;
          EmailAddress : LPCTSTR;
       end;
	 TGOPHER_ADMIN_ATTRIBUTE_TYPE  = GOPHER_ADMIN_ATTRIBUTE_TYPE;
     LPGOPHER_ADMIN_ATTRIBUTE_TYPE = ^GOPHER_ADMIN_ATTRIBUTE_TYPE;
	 PGOPHER_ADMIN_ATTRIBUTE_TYPE  = LPGOPHER_ADMIN_ATTRIBUTE_TYPE;

     GOPHER_MOD_DATE_ATTRIBUTE_TYPE = record
          DateAndTime : FILETIME;
       end;
	 TGOPHER_MOD_DATE_ATTRIBUTE_TYPE = GOPHER_MOD_DATE_ATTRIBUTE_TYPE;
     LPGOPHER_MOD_DATE_ATTRIBUTE_TYPE = ^GOPHER_MOD_DATE_ATTRIBUTE_TYPE;
	 PGOPHER_MOD_DATE_ATTRIBUTE_TYPE = LPGOPHER_MOD_DATE_ATTRIBUTE_TYPE;

     GOPHER_TTL_ATTRIBUTE_TYPE = record
          Ttl : DWORD;
       end;
	 TGOPHER_TTL_ATTRIBUTE_TYPE = GOPHER_TTL_ATTRIBUTE_TYPE;
	 LPGOPHER_TTL_ATTRIBUTE_TYPE = ^GOPHER_TTL_ATTRIBUTE_TYPE;
	 PGOPHER_TTL_ATTRIBUTE_TYPE = LPGOPHER_TTL_ATTRIBUTE_TYPE;

     GOPHER_SCORE_ATTRIBUTE_TYPE = record
          Score : WINT;
       end;
     TGOPHER_SCORE_ATTRIBUTE_TYPE = GOPHER_SCORE_ATTRIBUTE_TYPE;
     LPGOPHER_SCORE_ATTRIBUTE_TYPE = ^GOPHER_SCORE_ATTRIBUTE_TYPE;
     PGOPHER_SCORE_ATTRIBUTE_TYPE = LPGOPHER_SCORE_ATTRIBUTE_TYPE;

     GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE = record
          LowerBound : WINT;
          UpperBound : WINT;
       end;
     TGOPHER_SCORE_RANGE_ATTRIBUTE_TYPE = GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE;
     LPGOPHER_SCORE_RANGE_ATTRIBUTE_TYPE = ^GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE;
     PGOPHER_SCORE_RANGE_ATTRIBUTE_TYPE = LPGOPHER_SCORE_RANGE_ATTRIBUTE_TYPE;

     GOPHER_SITE_ATTRIBUTE_TYPE = record
          Site : LPCTSTR;
       end;
     TGOPHER_SITE_ATTRIBUTE_TYPE = GOPHER_SITE_ATTRIBUTE_TYPE;
     LPGOPHER_SITE_ATTRIBUTE_TYPE = ^GOPHER_SITE_ATTRIBUTE_TYPE;
     PGOPHER_SITE_ATTRIBUTE_TYPE = LPGOPHER_SITE_ATTRIBUTE_TYPE;

     GOPHER_ORGANIZATION_ATTRIBUTE_TYPE = record
          Organization : LPCTSTR;
       end;
     TGOPHER_ORGANIZATION_ATTRIBUTE_TYPE = GOPHER_ORGANIZATION_ATTRIBUTE_TYPE;
     LPGOPHER_ORGANIZATION_ATTRIBUTE_TYPE = ^GOPHER_ORGANIZATION_ATTRIBUTE_TYPE;
     PGOPHER_ORGANIZATION_ATTRIBUTE_TYPE = LPGOPHER_ORGANIZATION_ATTRIBUTE_TYPE;

     GOPHER_LOCATION_ATTRIBUTE_TYPE = record
          Location : LPCTSTR;
       end;
     TGOPHER_LOCATION_ATTRIBUTE_TYPE = GOPHER_LOCATION_ATTRIBUTE_TYPE;
     LPGOPHER_LOCATION_ATTRIBUTE_TYPE = ^GOPHER_LOCATION_ATTRIBUTE_TYPE;
     PGOPHER_LOCATION_ATTRIBUTE_TYPE = LPGOPHER_LOCATION_ATTRIBUTE_TYPE;

     GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE = record
          DegreesNorth : WINT;
          MinutesNorth : WINT;
          SecondsNorth : WINT;
          DegreesEast : WINT;
          MinutesEast : WINT;
          SecondsEast : WINT;
       end;
     TGOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE = GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE;
     LPGOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE = ^GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE;
     PGOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE = LPGOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE;

     GOPHER_TIMEZONE_ATTRIBUTE_TYPE = record
          Zone : WINT;
       end;
     TGOPHER_TIMEZONE_ATTRIBUTE_TYPE = GOPHER_TIMEZONE_ATTRIBUTE_TYPE;
     LPGOPHER_TIMEZONE_ATTRIBUTE_TYPE = ^GOPHER_TIMEZONE_ATTRIBUTE_TYPE;
     PGOPHER_TIMEZONE_ATTRIBUTE_TYPE = LPGOPHER_TIMEZONE_ATTRIBUTE_TYPE;

     GOPHER_PROVIDER_ATTRIBUTE_TYPE = record
          Provider : LPCTSTR;
       end;
     TGOPHER_PROVIDER_ATTRIBUTE_TYPE = GOPHER_PROVIDER_ATTRIBUTE_TYPE;
     LPGOPHER_PROVIDER_ATTRIBUTE_TYPE = ^GOPHER_PROVIDER_ATTRIBUTE_TYPE;
     PGOPHER_PROVIDER_ATTRIBUTE_TYPE = LPGOPHER_PROVIDER_ATTRIBUTE_TYPE;

     GOPHER_VERSION_ATTRIBUTE_TYPE = record
          Version : LPCTSTR;
       end;
     TGOPHER_VERSION_ATTRIBUTE_TYPE = GOPHER_VERSION_ATTRIBUTE_TYPE;
     LPGOPHER_VERSION_ATTRIBUTE_TYPE = ^GOPHER_VERSION_ATTRIBUTE_TYPE;
     PGOPHER_VERSION_ATTRIBUTE_TYPE = LPGOPHER_VERSION_ATTRIBUTE_TYPE;

     GOPHER_ABSTRACT_ATTRIBUTE_TYPE = record
          ShortAbstract : LPCTSTR;
          AbstractFile : LPCTSTR;
       end;
     TGOPHER_ABSTRACT_ATTRIBUTE_TYPE = GOPHER_ABSTRACT_ATTRIBUTE_TYPE;
     LPGOPHER_ABSTRACT_ATTRIBUTE_TYPE = ^GOPHER_ABSTRACT_ATTRIBUTE_TYPE;
     PGOPHER_ABSTRACT_ATTRIBUTE_TYPE = LPGOPHER_ABSTRACT_ATTRIBUTE_TYPE;

     GOPHER_VIEW_ATTRIBUTE_TYPE = record
          ContentType : LPCTSTR;
          Language : LPCTSTR;
          Size : DWORD;
       end;
     TGOPHER_VIEW_ATTRIBUTE_TYPE = GOPHER_VIEW_ATTRIBUTE_TYPE;
     LPGOPHER_VIEW_ATTRIBUTE_TYPE = ^GOPHER_VIEW_ATTRIBUTE_TYPE;
     PGOPHER_VIEW_ATTRIBUTE_TYPE = LPGOPHER_VIEW_ATTRIBUTE_TYPE;

     GOPHER_VERONICA_ATTRIBUTE_TYPE = record
          TreeWalk : BOOL;
       end;
     TGOPHER_VERONICA_ATTRIBUTE_TYPE = GOPHER_VERONICA_ATTRIBUTE_TYPE;
     LPGOPHER_VERONICA_ATTRIBUTE_TYPE = ^GOPHER_VERONICA_ATTRIBUTE_TYPE;
     PGOPHER_VERONICA_ATTRIBUTE_TYPE = LPGOPHER_VERONICA_ATTRIBUTE_TYPE;

     GOPHER_ASK_ATTRIBUTE_TYPE = record
          QuestionType : LPCTSTR;
          QuestionText : LPCTSTR;
       end;
     TGOPHER_ASK_ATTRIBUTE_TYPE = GOPHER_ASK_ATTRIBUTE_TYPE;
     LPGOPHER_ASK_ATTRIBUTE_TYPE = ^GOPHER_ASK_ATTRIBUTE_TYPE;
     PGOPHER_ASK_ATTRIBUTE_TYPE = LPGOPHER_ASK_ATTRIBUTE_TYPE;

     GOPHER_UNKNOWN_ATTRIBUTE_TYPE = record
          Text : LPCTSTR;
       end;
     TGOPHER_UNKNOWN_ATTRIBUTE_TYPE = GOPHER_UNKNOWN_ATTRIBUTE_TYPE;
     LPGOPHER_UNKNOWN_ATTRIBUTE_TYPE = ^GOPHER_UNKNOWN_ATTRIBUTE_TYPE;
     PGOPHER_UNKNOWN_ATTRIBUTE_TYPE = LPGOPHER_UNKNOWN_ATTRIBUTE_TYPE;

     GOPHER_ATTRIBUTE_TYPE = record
          CategoryId : DWORD;
          AttributeId : DWORD;
          AttributeType : record
              case longint of
                 0 : ( Admin : GOPHER_ADMIN_ATTRIBUTE_TYPE );
                 1 : ( ModDate : GOPHER_MOD_DATE_ATTRIBUTE_TYPE );
                 2 : ( Ttl : GOPHER_TTL_ATTRIBUTE_TYPE );
                 3 : ( Score : GOPHER_SCORE_ATTRIBUTE_TYPE );
                 4 : ( ScoreRange : GOPHER_SCORE_RANGE_ATTRIBUTE_TYPE );
                 5 : ( Site : GOPHER_SITE_ATTRIBUTE_TYPE );
                 6 : ( Organization : GOPHER_ORGANIZATION_ATTRIBUTE_TYPE );
                 7 : ( Location : GOPHER_LOCATION_ATTRIBUTE_TYPE );
                 8 : ( GeographicalLocation : GOPHER_GEOGRAPHICAL_LOCATION_ATTRIBUTE_TYPE );
                 9 : ( TimeZone : GOPHER_TIMEZONE_ATTRIBUTE_TYPE );
                 10 : ( Provider : GOPHER_PROVIDER_ATTRIBUTE_TYPE );
                 11 : ( Version : GOPHER_VERSION_ATTRIBUTE_TYPE );
                 12 : ( Abstract : GOPHER_ABSTRACT_ATTRIBUTE_TYPE );
                 13 : ( View : GOPHER_VIEW_ATTRIBUTE_TYPE );
                 14 : ( Veronica : GOPHER_VERONICA_ATTRIBUTE_TYPE );
                 15 : ( Ask : GOPHER_ASK_ATTRIBUTE_TYPE );
                 16 : ( Unknown : GOPHER_UNKNOWN_ATTRIBUTE_TYPE );
              end;
       end;
     TGOPHER_ATTRIBUTE_TYPE = GOPHER_ATTRIBUTE_TYPE;
     LPGOPHER_ATTRIBUTE_TYPE = ^GOPHER_ATTRIBUTE_TYPE;
     PGOPHER_ATTRIBUTE_TYPE = LPGOPHER_ATTRIBUTE_TYPE;

     INTERNET_STATUS_CALLBACK = procedure (hInternet:HINTERNET; dwContext:DWORD_PTR; dwInternetStatus:DWORD; lpvStatusInformation:LPVOID; dwStatusInformationLength:DWORD);stdcall;
     LPINTERNET_STATUS_CALLBACK = INTERNET_STATUS_CALLBACK; // ??
	 GOPHER_ATTRIBUTE_ENUMERATOR = function (lpAttributeInfo:LPGOPHER_ATTRIBUTE_TYPE; dwError:DWORD):BOOL;stdcall;
     PFN_AUTH_NOTIFY = function (dwContext:DWORD_PTR; dwReturn:DWORD; lpreserved:LPVOID):DWORD;stdcall;
	 InternetAuthNotifyCallback = PFN_AUTH_NOTIFY;

<<<<<<< HEAD
<<<<<<< HEAD
     _INTERNET_CACHE_ENTRY_INFOA = record
=======
     _INTERNET_CACHE_ENTRY_INFOA = packed record
>>>>>>> graemeg/fixes_2_2
=======
     _INTERNET_CACHE_ENTRY_INFOA = packed record
>>>>>>> origin/fixes_2_2
          dwStructSize : DWORD;
          lpszSourceUrlName : LPSTR;
          lpszLocalFileName : LPSTR;
          CacheEntryType : DWORD;
          dwUseCount : DWORD;
          dwHitRate : DWORD;
          dwSizeLow : DWORD;
          dwSizeHigh : DWORD;
          LastModifiedTime : FILETIME;
          ExpireTime : FILETIME;
          LastAccessTime : FILETIME;
          LastSyncTime : FILETIME;
          lpHeaderInfo : LPSTR;
          dwHeaderInfoSize : DWORD;
          lpszFileExtension : LPSTR;
          case longint of
                 0 : ( dwReserved : DWORD );
                 1 : ( dwExemptDelta : DWORD );
          end;
	 TINTERNET_CACHE_ENTRY_INFOA = _INTERNET_CACHE_ENTRY_INFOA;
	 INTERNET_CACHE_ENTRY_INFOA = _INTERNET_CACHE_ENTRY_INFOA;
     PINTERNET_CACHE_ENTRY_INFOA = ^INTERNET_CACHE_ENTRY_INFOA;
     LPINTERNET_CACHE_ENTRY_INFOA = PINTERNET_CACHE_ENTRY_INFOA;
     PLPINTERNET_CACHE_ENTRY_INFOA = ^LPINTERNET_CACHE_ENTRY_INFOA;

<<<<<<< HEAD
<<<<<<< HEAD
     _INTERNET_CACHE_ENTRY_INFOW = record
=======
     _INTERNET_CACHE_ENTRY_INFOW = packed record
>>>>>>> graemeg/fixes_2_2
=======
     _INTERNET_CACHE_ENTRY_INFOW = packed record
>>>>>>> origin/fixes_2_2
          dwStructSize : DWORD;
          lpszSourceUrlName : LPWSTR;
          lpszLocalFileName : LPWSTR;
          CacheEntryType : DWORD;
          dwUseCount : DWORD;
          dwHitRate : DWORD;
          dwSizeLow : DWORD;
          dwSizeHigh : DWORD;
          LastModifiedTime : FILETIME;
          ExpireTime : FILETIME;
          LastAccessTime : FILETIME;
          LastSyncTime : FILETIME;
          lpHeaderInfo : LPWSTR;
          dwHeaderInfoSize : DWORD;
          lpszFileExtension : LPWSTR;
          case longint of
                 0 : ( dwReserved : DWORD );
                 1 : ( dwExemptDelta : DWORD );
          end;
     INTERNET_CACHE_ENTRY_INFOW  = _INTERNET_CACHE_ENTRY_INFOW;
     TINTERNET_CACHE_ENTRY_INFOW = _INTERNET_CACHE_ENTRY_INFOW;
     PINTERNET_CACHE_ENTRY_INFOW = ^INTERNET_CACHE_ENTRY_INFOW;
     LPINTERNET_CACHE_ENTRY_INFOW = PINTERNET_CACHE_ENTRY_INFOW;
     PLPINTERNET_CACHE_ENTRY_INFOW = ^LPINTERNET_CACHE_ENTRY_INFOW;
{$ifdef UNICODE}
       INTERNET_CACHE_ENTRY_INFO = INTERNET_CACHE_ENTRY_INFOW;
       LPINTERNET_CACHE_ENTRY_INFO = LPINTERNET_CACHE_ENTRY_INFOW;
       TINTERNET_CACHE_ENTRY_INFO = INTERNET_CACHE_ENTRY_INFOW;
       PINTERNET_CACHE_ENTRY_INFO = LPINTERNET_CACHE_ENTRY_INFOW;
{$else}
       INTERNET_CACHE_ENTRY_INFO = INTERNET_CACHE_ENTRY_INFOA;
       LPINTERNET_CACHE_ENTRY_INFO = LPINTERNET_CACHE_ENTRY_INFOA;
       TINTERNET_CACHE_ENTRY_INFO = INTERNET_CACHE_ENTRY_INFOA;
       PINTERNET_CACHE_ENTRY_INFO = LPINTERNET_CACHE_ENTRY_INFOA;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
     PINTERNETCACHEENTRYINFO = PINTERNET_CACHE_ENTRY_INFO;
     TINTERNETCACHEENTRYINFO = TINTERNET_CACHE_ENTRY_INFO;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

     INTERNET_AUTH_NOTIFY_DATA = record
          cbStruct : DWORD;
          dwOptions : DWORD;
          pfnNotify : PFN_AUTH_NOTIFY;
          dwContext : DWORD_PTR;
       end;
     TINTERNET_AUTH_NOTIFY_DATA = INTERNET_AUTH_NOTIFY_DATA;
	 PINTERNET_AUTH_NOTIFY_DATA = ^INTERNET_AUTH_NOTIFY_DATA;
	 LPINTERNET_AUTH_NOTIFY_DATA = PINTERNET_AUTH_NOTIFY_DATA;
	
       _INTERNET_CACHE_TIMESTAMPS = record
            ftExpires : FILETIME;
            ftLastModified : FILETIME;
         end;
       INTERNET_CACHE_TIMESTAMPS = _INTERNET_CACHE_TIMESTAMPS;
       LPINTERNET_CACHE_TIMESTAMPS = ^_INTERNET_CACHE_TIMESTAMPS;
       TINTERNET_CACHE_TIMESTAMPS = _INTERNET_CACHE_TIMESTAMPS;
       PINTERNET_CACHE_TIMESTAMPS = LPINTERNET_CACHE_TIMESTAMPS;

       _INTERNET_CACHE_GROUP_INFOA = record
            dwGroupSize : DWORD;
            dwGroupFlags : DWORD;
            dwGroupType : DWORD;
            dwDiskUsage : DWORD;
            dwDiskQuota : DWORD;
            dwOwnerStorage : array[0..(GROUP_OWNER_STORAGE_SIZE)-1] of DWORD;
            szGroupName : array[0..(GROUPNAME_MAX_LENGTH)-1] of CHAR;
         end;
       INTERNET_CACHE_GROUP_INFOA = _INTERNET_CACHE_GROUP_INFOA;
       LPINTERNET_CACHE_GROUP_INFOA = ^_INTERNET_CACHE_GROUP_INFOA;
	   TINTERNET_CACHE_GROUP_INFOA = _INTERNET_CACHE_GROUP_INFOA;
       PINTERNET_CACHE_GROUP_INFOA = LPINTERNET_CACHE_GROUP_INFOA;
       _INTERNET_CACHE_GROUP_INFOW = record
            dwGroupSize : DWORD;
            dwGroupFlags : DWORD;
            dwGroupType : DWORD;
            dwDiskUsage : DWORD;
            dwDiskQuota : DWORD;
            dwOwnerStorage : array[0..(GROUP_OWNER_STORAGE_SIZE)-1] of DWORD;
            szGroupName : array[0..(GROUPNAME_MAX_LENGTH)-1] of WCHAR;
         end;
       INTERNET_CACHE_GROUP_INFOW = _INTERNET_CACHE_GROUP_INFOW;
       LPINTERNET_CACHE_GROUP_INFOW = ^_INTERNET_CACHE_GROUP_INFOW;
       TINTERNET_CACHE_GROUP_INFOW = _INTERNET_CACHE_GROUP_INFOW;
       PINTERNET_CACHE_GROUP_INFOW = LPINTERNET_CACHE_GROUP_INFOW;
{$ifdef UNICODE}
       INTERNET_CACHE_GROUP_INFO = INTERNET_CACHE_GROUP_INFOW;
       LPINTERNET_CACHE_GROUP_INFO = LPINTERNET_CACHE_GROUP_INFOW;
       TINTERNET_CACHE_GROUP_INFO = INTERNET_CACHE_GROUP_INFOW;
       PINTERNET_CACHE_GROUP_INFO = LPINTERNET_CACHE_GROUP_INFOW;
{$else}
       INTERNET_CACHE_GROUP_INFO = INTERNET_CACHE_GROUP_INFOA;
       LPINTERNET_CACHE_GROUP_INFO = LPINTERNET_CACHE_GROUP_INFOA;
       TINTERNET_CACHE_GROUP_INFO = INTERNET_CACHE_GROUP_INFOA;
       PINTERNET_CACHE_GROUP_INFO = LPINTERNET_CACHE_GROUP_INFOA;
{$endif}

       AUTO_PROXY_SCRIPT_BUFFER = record
            dwStructSize : DWORD;
            lpszScriptBuffer : LPSTR;
            dwScriptBufferSize : DWORD;
         end;

	   LPAUTO_PROXY_SCRIPT_BUFFER = ^AUTO_PROXY_SCRIPT_BUFFER;
	   TAUTO_PROXY_SCRIPT_BUFFER = AUTO_PROXY_SCRIPT_BUFFER;
       PAUTO_PROXY_SCRIPT_BUFFER = LPAUTO_PROXY_SCRIPT_BUFFER;


     TIsResolvable = function (lpszHost:LPSTR):BOOL;stdcall;
     TGetIPAddress = function (lpszIPAddress:LPSTR; lpdwIPAddressSize:LPDWORD):DWORD; stdcall;
     TResolveHostName = function (lpszHostName:LPSTR; lpszIPAddress:LPSTR; lpdwIPAddressSize:LPDWORD):DWORD; stdcall;
     TIsInNet = function (lpszIPAddress:LPSTR; lpszDest:LPSTR; lpszMask:LPSTR):BOOL; stdcall;
     TIsResolvableEx = function (lpszHost:LPSTR):BOOL; stdcall;
     TGetIPAddressEx = function (lpszIPAddress:LPSTR; lpdwIPAddressSize:LPDWORD):DWORD; stdcall;
     TResolveHostNameEx = function (lpszHostName:LPSTR; lpszIPAddress:LPSTR; lpdwIPAddressSize:LPDWORD):DWORD; stdcall;
     TIsInNetEx = function (lpszIPAddress:LPSTR; lpszIPPrefix:LPSTR):BOOL; stdcall;
     TSortIpList = function (lpszIPAddressList:LPSTR; lpszIPSortedList:LPSTR; lpdwIPSortedListSize:LPDWORD):DWORD; stdcall;


  PAutoProxyHelperVtbl = ^AutoProxyHelperVtbl;
<<<<<<< HEAD
<<<<<<< HEAD
  AutoProxyHelperVtbl =  record
=======
  AutoProxyHelperVtbl = packed record
>>>>>>> graemeg/fixes_2_2
=======
  AutoProxyHelperVtbl = packed record
>>>>>>> origin/fixes_2_2
          IsResolvable                : TIsResolvable;
          GetIPAddress                : TGetIPAddress;
          ResolveHostName             : TResolveHostName;
          IsInNet                     : TIsInNet;
          IsResolvableEx              : TIsResolvableEx;
          GetIPAddressEx              : TGetIPAddressEx;
          ResolveHostNameEx           : TResolveHostNameEx;
          IsInNetEx                   : TIsInNetEx;
          SortIpList                  : TSortIpList;
       end;
  PAutoProxyHelperFunctions = ^AutoProxyHelperFunctions;
  AutoProxyHelperFunctions = record
            lpVtbl : PAutoProxyHelperVtbl;
			end;
  pfnInternetInitializeAutoProxyDll = function (dwVersion:DWORD; lpszDownloadedTempFile:LPSTR; lpszMime:LPSTR; lpAutoProxyCallbacks:pAutoProxyHelperFunctions; lpAutoProxyScriptBuffer:LPAUTO_PROXY_SCRIPT_BUFFER):BOOL;stdcall;
  pfnInternetDeInitializeAutoProxyDll = function (lpszMime:LPSTR; dwReserved:DWORD):BOOL;stdcall;
  pfnInternetGetProxyInfo = function (lpszUrl:LPCSTR; dwUrlLength:DWORD; lpszUrlHostName:LPSTR; dwUrlHostNameLength:DWORD; lplpszProxyHostName:pLPSTR;
                    lpdwProxyHostNameLength:LPDWORD):BOOL;stdcall;
  PFN_DIAL_HANDLER = function (_para1:HWND; _para2:LPCSTR; _para3:DWORD; _para4:LPDWORD):DWORD;stdcall;

  function InternetTimeFromSystemTimeA(pst:PSYSTEMTIME; dwRFC:DWORD; lpszTime:LPSTR; cbTime:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeFromSystemTimeA';
  function InternetTimeFromSystemTimeW(pst:PSYSTEMTIME; dwRFC:DWORD; lpszTime:LPWSTR; cbTime:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeFromSystemTimeW';
  function InternetTimeToSystemTimeA(lpszTime:LPCSTR; pst:PSYSTEMTIME; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeToSystemTimeA';
  function InternetTimeToSystemTimeW(lpszTime:LPCWSTR; pst:PSYSTEMTIME; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeToSystemTimeW';
  function InternetCanonicalizeUrlA(lpszUrl:LPCSTR; lpszBuffer:LPSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlA';
  function InternetCanonicalizeUrlW(lpszUrl:LPCWSTR; lpszBuffer:LPWSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlW';
  function InternetCombineUrlA(lpszBaseUrl:LPCSTR; lpszRelativeUrl:LPCSTR; lpszBuffer:LPSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlA';
  function InternetCombineUrlW(lpszBaseUrl:LPCWSTR; lpszRelativeUrl:LPCWSTR; lpszBuffer:LPWSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlW';
  function InternetOpenA(lpszAgent:LPCSTR; dwAccessType:DWORD; lpszProxy:LPCSTR; lpszProxyBypass:LPCSTR; dwFlags:DWORD):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenA';
  function InternetOpenW(lpszAgent:LPCWSTR; dwAccessType:DWORD; lpszProxy:LPCWSTR; lpszProxyBypass:LPCWSTR; dwFlags:DWORD):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenW';	
  function InternetCloseHandle(hInternet:HINTERNET):BOOL;stdcall;external WININETLIBNAME name 'InternetCloseHandle';
  function InternetConnectA(hInternet:HINTERNET; lpszServerName:LPCSTR; nServerPort:INTERNET_PORT; lpszUserName:LPCSTR; lpszPassword:LPCSTR;
             dwService:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetConnectA';
  function InternetConnectW(hInternet:HINTERNET; lpszServerName:LPCWSTR; nServerPort:INTERNET_PORT; lpszUserName:LPCWSTR; lpszPassword:LPCWSTR;
             dwService:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetConnectW';
  function InternetOpenUrlA(hInternet:HINTERNET; lpszUrl:LPCSTR; lpszHeaders:LPCSTR; dwHeadersLength:DWORD; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenUrlA';
  function InternetOpenUrlW(hInternet:HINTERNET; lpszUrl:LPCWSTR; lpszHeaders:LPCWSTR; dwHeadersLength:DWORD; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenUrlW';
  function InternetReadFile(hFile:HINTERNET; lpBuffer:LPVOID; dwNumberOfBytesToRead:DWORD; lpdwNumberOfBytesRead:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetReadFile';

  function InternetReadFileExA(hFile:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSA; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'InternetReadFileExA';
  function InternetReadFileExW(hFile:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSW; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'InternetReadFileExW';
  function InternetSetFilePointer(hFile:HINTERNET; lDistanceToMove:LONG; pReserved:PVOID; dwMoveMethod:DWORD; dwContext:DWORD_PTR):DWORD;stdcall;external WININETLIBNAME name 'InternetSetFilePointer';
  function InternetWriteFile(hFile:HINTERNET; lpBuffer:LPCVOID; dwNumberOfBytesToWrite:DWORD; lpdwNumberOfBytesWritten:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetWriteFile';
  function InternetQueryDataAvailable(hFile:HINTERNET; lpdwNumberOfBytesAvailable:LPDWORD; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryDataAvailable';
  function InternetFindNextFileA(hFind:HINTERNET; lpvFindData:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetFindNextFileA';
  function InternetFindNextFileW(hFind:HINTERNET; lpvFindData:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetFindNextFileW';
  function InternetQueryOptionA(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionA';
  function InternetQueryOptionW(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionW';
  function InternetSetOptionA(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionA';
  function InternetSetOptionW(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionW';
  function InternetSetOptionExA(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionExA';
  function InternetSetOptionExW(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionExW';
  function InternetLockRequestFile(hInternet:HINTERNET; lphLockRequestInfo:PHANDLE):BOOL;stdcall;external WININETLIBNAME name 'InternetLockRequestFile';
  function InternetUnlockRequestFile(hLockRequestInfo:HANDLE):BOOL;stdcall;external WININETLIBNAME name 'InternetUnlockRequestFile';
  function InternetGetLastResponseInfoA(lpdwError:LPDWORD; lpszBuffer:LPSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoA';
  function InternetGetLastResponseInfoW(lpdwError:LPDWORD; lpszBuffer:LPWSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoW';
  function InternetSetStatusCallbackA(hInternet:HINTERNET; lpfnInternetCallback:INTERNET_STATUS_CALLBACK):INTERNET_STATUS_CALLBACK;stdcall;external WININETLIBNAME name 'InternetSetStatusCallbackA';
  function InternetSetStatusCallbackW(hInternet:HINTERNET; lpfnInternetCallback:INTERNET_STATUS_CALLBACK):INTERNET_STATUS_CALLBACK;stdcall;external WININETLIBNAME name 'InternetSetStatusCallbackW';
  function FtpFindFirstFileA(hConnect:HINTERNET; lpszSearchFile:LPCSTR; lpFindFileData:LPWIN32_FIND_DATA; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileA';
  function FtpFindFirstFileW(hConnect:HINTERNET; lpszSearchFile:LPCWSTR; lpFindFileData:LPWIN32_FIND_DATAW; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileW';
  function FtpGetFileA(hConnect:HINTERNET; lpszRemoteFile:LPCSTR; lpszNewFile:LPCSTR; fFailIfExists:BOOL; dwFlagsAndAttributes:DWORD;
             dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpGetFileA';
  function FtpGetFileW(hConnect:HINTERNET; lpszRemoteFile:LPCWSTR; lpszNewFile:LPCWSTR; fFailIfExists:BOOL; dwFlagsAndAttributes:DWORD;
             dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpGetFileW';
  function FtpPutFileA(hConnect:HINTERNET; lpszLocalFile:LPCSTR; lpszNewRemoteFile:LPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpPutFileA';
  function FtpPutFileW(hConnect:HINTERNET; lpszLocalFile:LPCWSTR; lpszNewRemoteFile:LPCWSTR; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpPutFileW';
  function FtpGetFileEx(hFtpSession:HINTERNET; lpszRemoteFile:LPCSTR; lpszNewFile:LPCWSTR; fFailIfExists:BOOL; dwFlagsAndAttributes:DWORD;
             dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpGetFileEx';
  function FtpPutFileEx(hFtpSession:HINTERNET; lpszLocalFile:LPCWSTR; lpszNewRemoteFile:LPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpPutFileEx';
  function FtpDeleteFileA(hConnect:HINTERNET; lpszFileName:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpDeleteFileA';
  function FtpDeleteFileW(hConnect:HINTERNET; lpszFileName:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpDeleteFileW';
  function FtpRenameFileA(hConnect:HINTERNET; lpszExisting:LPCSTR; lpszNew:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRenameFileA';
  function FtpRenameFileW(hConnect:HINTERNET; lpszExisting:LPCWSTR; lpszNew:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRenameFileW';
  function FtpOpenFileA(hConnect:HINTERNET; lpszFileName:LPCSTR; dwAccess:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpOpenFileA';
  function FtpOpenFileW(hConnect:HINTERNET; lpszFileName:LPCWSTR; dwAccess:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpOpenFileW';
  function FtpCreateDirectoryA(hConnect:HINTERNET; lpszDirectory:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpCreateDirectoryA';
  function FtpCreateDirectoryW(hConnect:HINTERNET; lpszDirectory:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpCreateDirectoryW';
  function FtpRemoveDirectoryA(hConnect:HINTERNET; lpszDirectory:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRemoveDirectoryA';
  function FtpRemoveDirectoryW(hConnect:HINTERNET; lpszDirectory:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRemoveDirectoryW';
  function FtpSetCurrentDirectoryA(hConnect:HINTERNET; lpszDirectory:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpSetCurrentDirectoryA';
  function FtpSetCurrentDirectoryW(hConnect:HINTERNET; lpszDirectory:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpSetCurrentDirectoryW';
  function FtpGetCurrentDirectoryA(hConnect:HINTERNET; lpszCurrentDirectory:LPSTR; lpdwCurrentDirectory:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryA';
  function FtpGetCurrentDirectoryW(hConnect:HINTERNET; lpszCurrentDirectory:LPWSTR; lpdwCurrentDirectory:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryW';
  function FtpCommandA(hConnect:HINTERNET; fExpectResponse:BOOL; dwFlags:DWORD; lpszCommand:LPCSTR; dwContext:DWORD_PTR;
             phFtpCommand:PHINTERNET):BOOL;stdcall;external WININETLIBNAME name 'FtpCommandA';
  function FtpCommandW(hConnect:HINTERNET; fExpectResponse:BOOL; dwFlags:DWORD; lpszCommand:LPCWSTR; dwContext:DWORD_PTR;
             phFtpCommand:PHINTERNET):BOOL;stdcall;external WININETLIBNAME name 'FtpCommandW';
  function FtpGetFileSize(hFile:HINTERNET; lpdwFileSizeHigh:LPDWORD):DWORD;stdcall;external WININETLIBNAME name 'FtpGetFileSize';
  function GopherCreateLocatorA(lpszHost:LPCSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCSTR; lpszSelectorString:LPCSTR; dwGopherType:DWORD;
             lpszLocator:LPSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorA';
  function GopherCreateLocatorW(lpszHost:LPCWSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCWSTR; lpszSelectorString:LPCWSTR; dwGopherType:DWORD;
             lpszLocator:LPWSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorW';
  function GopherGetLocatorTypeA(lpszLocator:LPCSTR; lpdwGopherType:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeA';
  function GopherGetLocatorTypeW(lpszLocator:LPCWSTR; lpdwGopherType:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeW';
  function GopherFindFirstFileA(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszSearchString:LPCSTR; lpFindData:LPGOPHER_FIND_DATAA; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileA';
  function GopherFindFirstFileW(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszSearchString:LPCWSTR; lpFindData:LPGOPHER_FIND_DATAW; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileW';
  function GopherOpenFileA(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszView:LPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherOpenFileA';
  function GopherOpenFileW(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszView:LPCWSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherOpenFileW';
  function GopherGetAttributeA(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszAttributeName:LPCSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             lpdwCharactersReturned:LPDWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeA';
  function GopherGetAttributeW(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszAttributeName:LPCWSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             lpdwCharactersReturned:LPDWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeW';
  function HttpOpenRequestA(hConnect:HINTERNET; lpszVerb:LPCSTR; lpszObjectName:LPCSTR; lpszVersion:LPCSTR; lpszReferrer:LPCSTR;
             lplpszAcceptTypes:LPPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'HttpOpenRequestA';
  function HttpOpenRequestW(hConnect:HINTERNET; lpszVerb:LPCWSTR; lpszObjectName:LPCWSTR; lpszVersion:LPCWSTR; lpszReferrer:LPCWSTR;
             lplpszAcceptTypes:LPPCWSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'HttpOpenRequestW';
  function HttpAddRequestHeadersA(hRequest:HINTERNET; lpszHeaders:LPCSTR; dwHeadersLength:DWORD; dwModifiers:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpAddRequestHeadersA';
  function HttpAddRequestHeadersW(hRequest:HINTERNET; lpszHeaders:LPCWSTR; dwHeadersLength:DWORD; dwModifiers:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpAddRequestHeadersW';
  function HttpSendRequestA(hRequest:HINTERNET; lpszHeaders:LPCSTR; dwHeadersLength:DWORD; lpOptional:LPVOID; dwOptionalLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestA';
  function HttpSendRequestW(hRequest:HINTERNET; lpszHeaders:LPCWSTR; dwHeadersLength:DWORD; lpOptional:LPVOID; dwOptionalLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestW';
  function HttpSendRequestExA(hRequest:HINTERNET; lpBuffersIn:LPINTERNET_BUFFERSA; lpBuffersOut:LPINTERNET_BUFFERSA; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestExA';
  function HttpSendRequestExW(hRequest:HINTERNET; lpBuffersIn:LPINTERNET_BUFFERSW; lpBuffersOut:LPINTERNET_BUFFERSW; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestExW';
  function HttpEndRequestA(hRequest:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSA; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpEndRequestA';
  function HttpEndRequestW(hRequest:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSW; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpEndRequestW';
  function HttpQueryInfoA(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD; lpdwIndex:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoA';
  function HttpQueryInfoW(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD; lpdwIndex:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoW';
  function InternetSetCookieA(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'InternetSetCookieA';
  function InternetSetCookieW(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'InternetSetCookieW';
  function InternetGetCookieA(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR; lpdwSize:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieA';
  function InternetGetCookieW(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR; lpdwSize:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieW';
  function InternetSetCookieExA(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPCSTR; dwFlags:DWORD; dwReserved:DWORD_PTR):DWORD;stdcall;external WININETLIBNAME name 'InternetSetCookieExA';
  function InternetSetCookieExW(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPCWSTR; dwFlags:DWORD; dwReserved:DWORD_PTR):DWORD;stdcall;external WININETLIBNAME name 'InternetSetCookieExW';
  function InternetGetCookieExA(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR; lpdwSize:LPDWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExA';
  function InternetGetCookieExW(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR; lpdwSize:LPDWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExW';
  function InternetAttemptConnect(dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetAttemptConnect';
  function InternetCheckConnectionA(lpszUrl:LPCSTR; dwFlags:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCheckConnectionA';
  function InternetCheckConnectionW(lpszUrl:LPCWSTR; dwFlags:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCheckConnectionW';
  function ResumeSuspendedDownload(hRequest:HINTERNET; dwResultCode:DWORD):BOOL;stdcall;external WININETLIBNAME name 'ResumeSuspendedDownload';
  function InternetErrorDlg(hWnd:HWND; hRequest:HINTERNET; dwError:DWORD; dwFlags:DWORD; lppvData:ppointer):DWORD;stdcall;external WININETLIBNAME name 'InternetErrorDlg';
  function InternetConfirmZoneCrossingA(hWnd:HWND; szUrlPrev:LPSTR; szUrlNew:LPSTR; bPost:BOOL):DWORD;stdcall;external WININETLIBNAME name 'InternetConfirmZoneCrossingA';
  function InternetConfirmZoneCrossingW(hWnd:HWND; szUrlPrev:LPWSTR; szUrlNew:LPWSTR; bPost:BOOL):DWORD;stdcall;external WININETLIBNAME name 'InternetConfirmZoneCrossingW';
  function CreateUrlCacheEntryA(lpszUrlName:LPCSTR; dwExpectedFileSize:DWORD; lpszFileExtension:LPCSTR; lpszFileName:LPSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'CreateUrlCacheEntryA';
  function CreateUrlCacheEntryW(lpszUrlName:LPCWSTR; dwExpectedFileSize:DWORD; lpszFileExtension:LPCWSTR; lpszFileName:LPWSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'CreateUrlCacheEntryW';
  function CommitUrlCacheEntryA(lpszUrlName:LPCSTR; lpszLocalFileName:LPCSTR; ExpireTime:FILETIME; LastModifiedTime:FILETIME; CacheEntryType:DWORD;
             lpHeaderInfo:LPCSTR; dwHeaderSize:DWORD; lpszFileExtension:LPCSTR; lpszOriginalUrl:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'CommitUrlCacheEntryA';
  function CommitUrlCacheEntryW(lpszUrlName:LPCWSTR; lpszLocalFileName:LPCWSTR; ExpireTime:FILETIME; LastModifiedTime:FILETIME; CacheEntryType:DWORD;
             lpHeaderInfo:LPCWSTR; dwHeaderSize:DWORD; lpszFileExtension:LPCWSTR; lpszOriginalUrl:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'CommitUrlCacheEntryW';
  function RetrieveUrlCacheEntryFileA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileA';
  function RetrieveUrlCacheEntryFileW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileW';
  function UnlockUrlCacheEntryFileA(lpszUrlName:LPCSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'UnlockUrlCacheEntryFileA';
  function UnlockUrlCacheEntryFileW(lpszUrlName:LPCWSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'UnlockUrlCacheEntryFileW';
  function RetrieveUrlCacheEntryStreamA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamA';
  function RetrieveUrlCacheEntryStreamW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamW';
  function ReadUrlCacheEntryStream(hUrlCacheStream:HANDLE; dwLocation:DWORD; lpBuffer:LPVOID; lpdwLen:LPDWORD; Reserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'ReadUrlCacheEntryStream';
  function UnlockUrlCacheEntryStream(hUrlCacheStream:HANDLE; Reserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'UnlockUrlCacheEntryStream';
  function GetUrlCacheEntryInfoA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoA';
  function GetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoW';
  function FindFirstUrlCacheGroup(dwFlags:DWORD; dwFilter:DWORD; lpSearchCondition:LPVOID; dwSearchCondition:DWORD; lpGroupId:PGROUPID;
             lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheGroup';
  function FindNextUrlCacheGroup(hFind:HANDLE; lpGroupId:PGROUPID; lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheGroup';
  function GetUrlCacheGroupAttributeA(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA; lpdwGroupInfo:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeA';
  function GetUrlCacheGroupAttributeW(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW; lpdwGroupInfo:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeW';
  function SetUrlCacheGroupAttributeA(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheGroupAttributeA';
  function SetUrlCacheGroupAttributeW(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW; lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheGroupAttributeW';
  function GetUrlCacheEntryInfoExA(lpszUrl:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; lpszRedirectUrl:LPSTR; lpcbRedirectUrl:LPDWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExA';
  function GetUrlCacheEntryInfoExW(lpszUrl:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; lpszRedirectUrl:LPWSTR; lpcbRedirectUrl:LPDWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExW';
  function SetUrlCacheEntryInfoA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoA';
  function SetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoW';
  function CreateUrlCacheGroup(dwFlags:DWORD; lpReserved:LPVOID):GROUPID;stdcall;external WININETLIBNAME name 'CreateUrlCacheGroup';
  function DeleteUrlCacheGroup(GroupId:GROUPID; dwFlags:DWORD; lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'DeleteUrlCacheGroup';
  function SetUrlCacheEntryGroupA(lpszUrlName:LPCSTR; dwFlags:DWORD; GroupId:GROUPID; pbGroupAttributes:LPBYTE; cbGroupAttributes:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryGroupA';
  function SetUrlCacheEntryGroupW(lpszUrlName:LPCWSTR; dwFlags:DWORD; GroupId:GROUPID; pbGroupAttributes:LPBYTE; cbGroupAttributes:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryGroupW';
  function FindFirstUrlCacheEntryExA(lpszUrlSearchPattern:LPCSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExA';
  function FindFirstUrlCacheEntryExW(lpszUrlSearchPattern:LPCWSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExW';
  function FindNextUrlCacheEntryExA(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExA';
  function FindNextUrlCacheEntryExW(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExW';
  function FindFirstUrlCacheEntryA(lpszUrlSearchPattern:LPCSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
  function FindFirstUrlCacheEntryW(lpszUrlSearchPattern:LPCWSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
  function FindNextUrlCacheEntryA(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
  function FindNextUrlCacheEntryW(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
  function FindCloseUrlCache(hEnumHandle:HANDLE):BOOL;stdcall;external WININETLIBNAME name 'FindCloseUrlCache';
  function DeleteUrlCacheEntryA(lpszUrlName:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'DeleteUrlCacheEntryA';
  function DeleteUrlCacheEntryW(lpszUrlName:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'DeleteUrlCacheEntryW';
  function InternetDialA(hwndParent:HWND; lpszConnectoid:LPSTR; dwFlags:DWORD; lpdwConnection:PDWORD_PTR; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialA';
  function InternetDialW(hwndParent:HWND; lpszConnectoid:LPWSTR; dwFlags:DWORD; lpdwConnection:PDWORD_PTR; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialW';
  function InternetHangUp(dwConnection:DWORD_PTR; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetHangUp';
  function InternetGoOnlineA(lpszURL:LPSTR; hwndParent:HWND; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGoOnlineA';
  function InternetGoOnlineW(lpszURL:LPWSTR; hwndParent:HWND; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGoOnlineW';
  function InternetAutodial(dwFlags:DWORD; hwndParent:HWND):BOOL;stdcall;external WININETLIBNAME name 'InternetAutodial';
  function InternetAutodialHangup(dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetAutodialHangup';
  function InternetGetConnectedState(lpdwFlags:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedState';
  function InternetGetConnectedStateExA(lpdwFlags:LPDWORD; lpszConnectionName:LPSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExA';
  function InternetGetConnectedStateExW(lpdwFlags:LPDWORD; lpszConnectionName:LPWSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExW';
  function InternetInitializeAutoProxyDll(dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetInitializeAutoProxyDll';
  function DetectAutoProxyUrl(lpszAutoProxyUrl:LPSTR; dwAutoProxyUrlLength:DWORD; dwDetectFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'DetectAutoProxyUrl';
  function CreateMD5SSOHash(pszChallengeInfo:LPWSTR; pwszRealm:LPWSTR; pwszTarget:LPWSTR; pbHexHash:PBYTE):BOOL;stdcall;external WININETLIBNAME name 'CreateMD5SSOHash';
  function InternetSetDialStateA(lpszConnectoid:LPCSTR; dwState:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetDialStateA';
  function InternetSetDialStateW(lpszConnectoid:LPCWSTR; dwState:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetDialStateW';
  function InternetSetPerSiteCookieDecisionA(pchHostName:LPCSTR; dwDecision:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetPerSiteCookieDecisionA';
  function InternetSetPerSiteCookieDecisionW(pchHostName:LPCWSTR; dwDecision:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetPerSiteCookieDecisionW';
  function InternetGetPerSiteCookieDecisionA(pchHostName:LPCSTR; pResult:Pdword):BOOL;stdcall;external WININETLIBNAME name 'InternetGetPerSiteCookieDecisionA';
  function InternetGetPerSiteCookieDecisionW(pchHostName:LPCWSTR; pResult:Pdword):BOOL;stdcall;external WININETLIBNAME name 'InternetGetPerSiteCookieDecisionW';
  function InternetEnumPerSiteCookieDecisionA(pszSiteName:LPSTR; pcSiteNameSize:Pdword; pdwDecision:Pdword; dwIndex:dword):BOOL;stdcall;external WININETLIBNAME name 'InternetEnumPerSiteCookieDecisionA';
  function InternetEnumPerSiteCookieDecisionW(pszSiteName:LPWSTR; pcSiteNameSize:Pdword; pdwDecision:Pdword; dwIndex:dword):BOOL;stdcall;external WININETLIBNAME name 'InternetEnumPerSiteCookieDecisionW';
  function PrivacySetZonePreferenceW(dwZone:DWORD; dwType:DWORD; dwTemplate:DWORD; pszPreference:LPCWSTR):DWORD;stdcall;external WININETLIBNAME name 'PrivacySetZonePreferenceW';
  function PrivacyGetZonePreferenceW(dwZone:DWORD; dwType:DWORD; pdwTemplate:LPDWORD; pszBuffer:LPWSTR; pdwBufferLength:LPDWORD):DWORD;stdcall;external WININETLIBNAME name 'PrivacyGetZonePreferenceW';
  function InternetClearAllPerSiteCookieDecisions:BOOL;stdcall;external WININETLIBNAME name 'InternetClearAllPerSiteCookieDecisions';

  { Delphi overloads, see bug 10576 and 11226}

  function InternetCreateUrlA(lpUrlComponents:LPURL_COMPONENTSA; dwFlags:DWORD; lpszUrl:LPSTR; var lpdwUrlLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlA';
  function InternetCreateUrlW(lpUrlComponents:LPURL_COMPONENTSW; dwFlags:DWORD; lpszUrl:LPWSTR;var lpdwUrlLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlW';
  function InternetCanonicalizeUrlA(lpszUrl:LPCSTR; lpszBuffer:LPSTR; var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlA';
  function InternetCanonicalizeUrlW(lpszUrl:LPCWSTR; lpszBuffer:LPWSTR; var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlW';
  function InternetCombineUrlA(lpszBaseUrl:LPCSTR; lpszRelativeUrl:LPCSTR; lpszBuffer:LPSTR; var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlA';
  function InternetCombineUrlW(lpszBaseUrl:LPCWSTR; lpszRelativeUrl:LPCWSTR; lpszBuffer:LPWSTR;var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlW';
  function InternetQueryDataAvailable(hFile:HINTERNET; var lpdwNumberOfBytesAvailable:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryDataAvailable';
  function InternetQueryOptionA(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionA';
  function InternetReadFile(hFile:HINTERNET; lpBuffer:LPVOID; dwNumberOfBytesToRead:DWORD; var lpdwNumberOfBytesRead:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetReadFile';
  function InternetWriteFile(hFile:HINTERNET; lpBuffer:LPCVOID; dwNumberOfBytesToWrite:DWORD; var lpdwNumberOfBytesWritten:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetWriteFile';
  function InternetQueryOptionW(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionW';
  function InternetGetLastResponseInfoA(var dwError:DWORD; lpszBuffer:LPSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoA';
  function InternetGetLastResponseInfoW(var dwError:DWORD; lpszBuffer:LPWSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoW';
  function FtpGetFileSize(hFile:HINTERNET; var lpdwFileSizeHigh:DWORD):DWORD;stdcall;external WININETLIBNAME name 'FtpGetFileSize';
  function GopherCreateLocatorA(lpszHost:LPCSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCSTR; lpszSelectorString:LPCSTR; dwGopherType:DWORD;
             lpszLocator:LPSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorA';
  function GopherCreateLocatorW(lpszHost:LPCWSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCWSTR; lpszSelectorString:LPCWSTR; dwGopherType:DWORD;
             lpszLocator:LPWSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorW';
  function GopherGetLocatorTypeA(lpszLocator:LPCSTR; var lpdwGopherType:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeA';
  function GopherGetLocatorTypeW(lpszLocator:LPCWSTR; var lpdwGopherType:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeW';
  function HttpQueryInfoA(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD; var lpdwIndex:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoA';
  function HttpQueryInfoW(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD; var lpdwIndex:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoW';
  function InternetGetCookieA(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR;var lpdwSize:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieA';
  function InternetGetCookieW(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR;var lpdwSize:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieW';
  function InternetGetCookieExA(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR; var lpdwSize:DWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExA';
  function InternetGetCookieExW(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR; var lpdwSize:DWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExW';
  function RetrieveUrlCacheEntryFileA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileA';
  function RetrieveUrlCacheEntryFileW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileW';
  function RetrieveUrlCacheEntryStreamA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamA';
  function RetrieveUrlCacheEntryStreamW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamW';
  function ReadUrlCacheEntryStream(hUrlCacheStream:HANDLE; dwLocation:DWORD; lpBuffer:LPVOID; var lpdwLen:DWORD; Reserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'ReadUrlCacheEntryStream';
  function GetUrlCacheEntryInfoA(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoA';
  function GetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoW';
  function GetUrlCacheGroupAttributeA(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA; var lpdwGroupInfo:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeA';
  function GetUrlCacheGroupAttributeW(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW; var lpdwGroupInfo:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeW';
  function GetUrlCacheEntryInfoExA(lpszUrl:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD; lpszRedirectUrl:LPSTR; var lpcbRedirectUrl:DWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExA';
  function GetUrlCacheEntryInfoExW(lpszUrl:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD; lpszRedirectUrl:LPWSTR; var lpcbRedirectUrl:DWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExW';
  function FindFirstUrlCacheEntryExA(lpszUrlSearchPattern:LPCSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
             var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; var lpcbGroupAttributes:DWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExA';
  function FindFirstUrlCacheEntryExW(lpszUrlSearchPattern:LPCWSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
             var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; var lpcbGroupAttributes:DWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExW';

  function FindNextUrlCacheEntryExA(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExA';
  function FindNextUrlCacheEntryExW(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExW';
  function FindFirstUrlCacheEntryA(lpszUrlSearchPattern:LPCSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
  function FindFirstUrlCacheEntryW(lpszUrlSearchPattern:LPCWSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
  function FindNextUrlCacheEntryA(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
  function FindNextUrlCacheEntryW(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
  function InternetDialA(hwndParent:HWND; lpszConnectoid:LPSTR; dwFlags:DWORD; var lpdwConnection:DWORD; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialA';
  function InternetDialW(hwndParent:HWND; lpszConnectoid:LPWSTR; dwFlags:DWORD; var lpdwConnection:DWORD; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialW';
  function InternetGetConnectedState(var lpdwFlags:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedState';
  function InternetGetConnectedStateExA(var lpdwFlags:DWORD; lpszConnectionName:LPSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExA';
  function InternetGetConnectedStateExW(var lpdwFlags:DWORD; lpszConnectionName:LPWSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExW';
  function PrivacyGetZonePreferenceW(dwZone:DWORD; dwType:DWORD; var pdwTemplate:DWORD; pszBuffer:LPWSTR; var pdwBufferLength:DWORD):DWORD;stdcall;external WININETLIBNAME name 'PrivacyGetZonePreferenceW';

  function FtpGetCurrentDirectoryA(hConnect:HINTERNET; lpszCurrentDirectory:LPSTR; var lpdwCurrentDirectory:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryA';
  function FtpGetCurrentDirectoryW(hConnect:HINTERNET; lpszCurrentDirectory:LPWSTR; var lpdwCurrentDirectory:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryW';
  function FtpFindFirstFileA(hConnect:HINTERNET; lpszSearchFile:LPCSTR; var lpFindFileData:WIN32_FIND_DATA; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileA';
  function FtpFindFirstFileW(hConnect:HINTERNET; lpszSearchFile:LPCWSTR; var lpFindFileData:WIN32_FIND_DATAW; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileW';
  function GopherFindFirstFileA(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszSearchString:LPCSTR; var lpFindData:GOPHER_FIND_DATAA; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileA';
  function GopherFindFirstFileW(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszSearchString:LPCWSTR; var lpFindData:GOPHER_FIND_DATAW; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileW';
  function GopherGetAttributeA(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszAttributeName:LPCSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             var lpdwCharactersReturned:DWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeA';
  function GopherGetAttributeW(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszAttributeName:LPCWSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             var lpdwCharactersReturned:DWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeW';

  function InternetErrorDlg(hWnd:HWND; hRequest:HINTERNET; dwError:DWORD; dwFlags:DWORD; var lppvData:pointer):DWORD;stdcall;external WININETLIBNAME name 'InternetErrorDlg';
  function RetrieveUrlCacheEntryFileA(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileA';
  function RetrieveUrlCacheEntryFileW(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileW';
  function RetrieveUrlCacheEntryStreamA(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamA';
  function RetrieveUrlCacheEntryStreamW(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamW';
  function ReadUrlCacheEntryStream(hUrlCacheStream:HANDLE; dwLocation:DWORD; var lpBuffer; lpdwLen:LPDWORD; Reserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'ReadUrlCacheEntryStream';
  function GetUrlCacheEntryInfoA(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoA';
  function GetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoW';
  function SetUrlCacheEntryInfoA(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoA';
  function SetUrlCacheEntryInfoW(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoW';
  function FindFirstUrlCacheEntryExA(lpszUrlSearchPattern:LPCSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExA';
  function FindFirstUrlCacheEntryExW(lpszUrlSearchPattern:LPCWSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExW';
  function FindNextUrlCacheEntryExA(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExA';
  function FindNextUrlCacheEntryExW(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExW';
  function FindFirstUrlCacheEntryA(lpszUrlSearchPattern:LPCSTR; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
  function FindFirstUrlCacheEntryW(lpszUrlSearchPattern:LPCWSTR; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
  function FindNextUrlCacheEntryA(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
  function FindNextUrlCacheEntryW(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';

  function InternetCrackUrlW(lpszUrl:LPCWSTR; dwUrlLength:DWORD; dwFlags:DWORD; var lpUrlComponents:URL_COMPONENTSW):BOOL;stdcall;external WININETLIBNAME name 'InternetCrackUrlW';
  function InternetCreateUrlW(var lpUrlComponents:URL_COMPONENTSW; dwFlags:DWORD; lpszUrl:LPWSTR; lpdwUrlLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlW';
  function InternetCrackUrlA(lpszUrl:LPCSTR; dwUrlLength:DWORD; dwFlags:DWORD; var lpUrlComponents:URL_COMPONENTSA):BOOL;stdcall;external WININETLIBNAME name 'InternetCrackUrlA';
  function InternetCreateUrlA(var lpUrlComponents:URL_COMPONENTSA; dwFlags:DWORD; lpszUrl:LPSTR; lpdwUrlLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlA';


{$ifndef UNICODE}

  function InternetTimeFromSystemTime(pst:PSYSTEMTIME; dwRFC:DWORD; lpszTime:LPSTR; cbTime:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeFromSystemTimeA';
  function InternetTimeToSystemTime(lpszTime:LPCSTR; pst:PSYSTEMTIME; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeToSystemTimeA';
  function InternetCrackUrl(lpszUrl:LPCSTR; dwUrlLength:DWORD; dwFlags:DWORD; lpUrlComponents:LPURL_COMPONENTSA):BOOL;stdcall;external WININETLIBNAME name 'InternetCrackUrlA';
  function InternetCreateUrl(lpUrlComponents:LPURL_COMPONENTSA; dwFlags:DWORD; lpszUrl:LPSTR; lpdwUrlLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlA';
  function InternetCanonicalizeUrl(lpszUrl:LPCSTR; lpszBuffer:LPSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlA';
  function InternetCombineUrl(lpszBaseUrl:LPCSTR; lpszRelativeUrl:LPCSTR; lpszBuffer:LPSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlA';
  function InternetOpen(lpszAgent:LPCSTR; dwAccessType:DWORD; lpszProxy:LPCSTR; lpszProxyBypass:LPCSTR; dwFlags:DWORD):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenA';
  function InternetConnect(hInternet:HINTERNET; lpszServerName:LPCSTR; nServerPort:INTERNET_PORT; lpszUserName:LPCSTR; lpszPassword:LPCSTR;
             dwService:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetConnectA';
  function InternetReadFileEx(hFile:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSA; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'InternetReadFileExA';			
  function InternetOpenUrl(hInternet:HINTERNET; lpszUrl:LPCSTR; lpszHeaders:LPCSTR; dwHeadersLength:DWORD; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenUrlA';
  function InternetFindNextFile(hFind:HINTERNET; lpvFindData:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetFindNextFileA';
  function InternetQueryOption(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionA';
  function InternetSetOption(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionA';
  function InternetSetOptionEx(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionExA';
  function InternetGetLastResponseInfo(lpdwError:LPDWORD; lpszBuffer:LPSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoA';
  function InternetSetStatusCallback(hInternet:HINTERNET; lpfnInternetCallback:INTERNET_STATUS_CALLBACK):INTERNET_STATUS_CALLBACK;stdcall;external WININETLIBNAME name 'InternetSetStatusCallbackA';
  function FtpFindFirstFile(hConnect:HINTERNET; lpszSearchFile:LPCSTR; lpFindFileData:LPWIN32_FIND_DATA; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileA';
  function FtpGetFile(hConnect:HINTERNET; lpszRemoteFile:LPCSTR; lpszNewFile:LPCSTR; fFailIfExists:BOOL; dwFlagsAndAttributes:DWORD;
             dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpGetFileA';
  function FtpPutFile(hConnect:HINTERNET; lpszLocalFile:LPCSTR; lpszNewRemoteFile:LPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpPutFileA';
  function FtpDeleteFile(hConnect:HINTERNET; lpszFileName:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpDeleteFileA';
  function FtpRenameFile(hConnect:HINTERNET; lpszExisting:LPCSTR; lpszNew:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRenameFileA';
  function FtpOpenFile(hConnect:HINTERNET; lpszFileName:LPCSTR; dwAccess:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpOpenFileA';
  function FtpCreateDirectory(hConnect:HINTERNET; lpszDirectory:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpCreateDirectoryA';
  function FtpRemoveDirectory(hConnect:HINTERNET; lpszDirectory:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRemoveDirectoryA';
  function FtpSetCurrentDirectory(hConnect:HINTERNET; lpszDirectory:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpSetCurrentDirectoryA';
  function FtpGetCurrentDirectory(hConnect:HINTERNET; lpszCurrentDirectory:LPSTR; lpdwCurrentDirectory:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryA';
  function FtpCommand(hConnect:HINTERNET; fExpectResponse:BOOL; dwFlags:DWORD; lpszCommand:LPCSTR; dwContext:DWORD_PTR;
             phFtpCommand:PHINTERNET):BOOL;stdcall;external WININETLIBNAME name 'FtpCommandA';
  function GopherCreateLocator(lpszHost:LPCSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCSTR; lpszSelectorString:LPCSTR; dwGopherType:DWORD;
             lpszLocator:LPSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorA';
  function GopherGetLocatorType(lpszLocator:LPCSTR; lpdwGopherType:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeA';
  function GopherFindFirstFile(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszSearchString:LPCSTR; lpFindData:LPGOPHER_FIND_DATAA; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileA';	
  function GopherOpenFile(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszView:LPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherOpenFileA';
  function GopherGetAttribute(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszAttributeName:LPCSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             lpdwCharactersReturned:LPDWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeA';
  function HttpOpenRequest(hConnect:HINTERNET; lpszVerb:LPCSTR; lpszObjectName:LPCSTR; lpszVersion:LPCSTR; lpszReferrer:LPCSTR;
             lplpszAcceptTypes:LPPCSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'HttpOpenRequestA';
  function HttpAddRequestHeaders(hRequest:HINTERNET; lpszHeaders:LPCSTR; dwHeadersLength:DWORD; dwModifiers:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpAddRequestHeadersA';
  function HttpSendRequest(hRequest:HINTERNET; lpszHeaders:LPCSTR; dwHeadersLength:DWORD; lpOptional:LPVOID; dwOptionalLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestA';
  function HttpSendRequestEx(hRequest:HINTERNET; lpBuffersIn:LPINTERNET_BUFFERSA; lpBuffersOut:LPINTERNET_BUFFERSA; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestExA';
  function HttpEndRequest(hRequest:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSA; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpEndRequestA';
  function HttpQueryInfo(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD; lpdwIndex:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoA';
  function InternetSetCookie(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'InternetSetCookieA';
  function InternetGetCookie(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR; lpdwSize:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieA';
  function InternetSetCookieEx(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPCSTR; dwFlags:DWORD; dwReserved:DWORD_PTR):DWORD;stdcall;external WININETLIBNAME name 'InternetSetCookieExA';
  function InternetGetCookieEx(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR; lpdwSize:LPDWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExA';
  function InternetCheckConnection(lpszUrl:LPCSTR; dwFlags:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCheckConnectionA';
  function InternetConfirmZoneCrossing(hWnd:HWND; szUrlPrev:LPSTR; szUrlNew:LPSTR; bPost:BOOL):DWORD;stdcall;external WININETLIBNAME name 'InternetConfirmZoneCrossingA';
  function CreateUrlCacheEntry(lpszUrlName:LPCSTR; dwExpectedFileSize:DWORD; lpszFileExtension:LPCSTR; lpszFileName:LPSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'CreateUrlCacheEntryA';
  function CommitUrlCacheEntry(lpszUrlName:LPCSTR; lpszLocalFileName:LPCSTR; ExpireTime:FILETIME; LastModifiedTime:FILETIME; CacheEntryType:DWORD;
             lpHeaderInfo:LPCSTR; dwHeaderSize:DWORD; lpszFileExtension:LPCSTR; lpszOriginalUrl:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'CommitUrlCacheEntryA';
  function RetrieveUrlCacheEntryFile(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileA';
  function UnlockUrlCacheEntryFile(lpszUrlName:LPCSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'UnlockUrlCacheEntryFileA';
  function RetrieveUrlCacheEntryStream(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamA';
  function GetUrlCacheEntryInfo(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoA';
  function GetUrlCacheGroupAttribute(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA; lpdwGroupInfo:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeA';
  function SetUrlCacheGroupAttribute(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheGroupAttributeA';
  function GetUrlCacheEntryInfoEx(lpszUrl:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; lpszRedirectUrl:LPSTR; lpcbRedirectUrl:LPDWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExA';
  function SetUrlCacheEntryInfo(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoA';
  function SetUrlCacheEntryGroup(lpszUrlName:LPCSTR; dwFlags:DWORD; GroupId:GROUPID; pbGroupAttributes:LPBYTE; cbGroupAttributes:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryGroupA';
  function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExA';
  function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExA';
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
  function DeleteUrlCacheEntry(lpszUrlName:LPCSTR):BOOL;stdcall;external WININETLIBNAME name 'DeleteUrlCacheEntryA';
  function InternetDial(hwndParent:HWND; lpszConnectoid:LPSTR; dwFlags:DWORD; lpdwConnection:PDWORD_PTR; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialA';
  function InternetGoOnline(lpszURL:LPSTR; hwndParent:HWND; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGoOnlineA';
  function InternetGetConnectedStateEx(lpdwFlags:LPDWORD; lpszConnectionName:LPSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExA';
  function InternetSetDialState(lpszConnectoid:LPCSTR; dwState:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetDialStateA';
  function InternetSetPerSiteCookieDecision(pchHostName:LPCSTR; dwDecision:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetPerSiteCookieDecisionA';
  function InternetGetPerSiteCookieDecision(pchHostName:LPCSTR; pResult:Pdword):BOOL;stdcall;external WININETLIBNAME name 'InternetGetPerSiteCookieDecisionA';
  function InternetEnumPerSiteCookieDecision(pszSiteName:LPSTR; pcSiteNameSize:Pdword; pdwDecision:Pdword; dwIndex:dword):BOOL;stdcall;external WININETLIBNAME name 'InternetEnumPerSiteCookieDecisionA';

  function InternetCrackUrl(lpszUrl:LPCSTR; dwUrlLength:DWORD; dwFlags:DWORD; var lpUrlComponents:URL_COMPONENTSA):BOOL;stdcall;external WININETLIBNAME name 'InternetCrackUrlA';
  function InternetCreateUrl(var lpUrlComponents:URL_COMPONENTSA; dwFlags:DWORD; lpszUrl:LPSTR; lpdwUrlLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlA';
  function InternetCreateUrl(lpUrlComponents:LPURL_COMPONENTSA; dwFlags:DWORD; lpszUrl:LPSTR; var lpdwUrlLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlA';
  function InternetCanonicalizeUrl(lpszUrl:LPCSTR; lpszBuffer:LPSTR; var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlA';
  function InternetCombineUrl(lpszBaseUrl:LPCSTR; lpszRelativeUrl:LPCSTR; lpszBuffer:LPSTR; var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlA';
  function InternetQueryOption(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionA';
  function InternetGetLastResponseInfo(lpdwError:LPDWORD; lpszBuffer:LPSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoA';
  function GopherCreateLocator(lpszHost:LPCSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCSTR; lpszSelectorString:LPCSTR; dwGopherType:DWORD;
             lpszLocator:LPSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorA';
  function GopherGetLocatorType(lpszLocator:LPCSTR; var lpdwGopherType:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeA';
  function HttpQueryInfo(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD; var lpdwIndex:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoA';
  function InternetGetCookie(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR;var lpdwSize:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieA';
  function InternetGetCookieEx(lpszUrl:LPCSTR; lpszCookieName:LPCSTR; lpszCookieData:LPSTR; var lpdwSize:DWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExA';
  function RetrieveUrlCacheEntryFile(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileA';
  function RetrieveUrlCacheEntryStream(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamA';
  function GetUrlCacheEntryInfo(lpszUrlName:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoA';
  function GetUrlCacheGroupAttribute(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOA; var lpdwGroupInfo:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeA';
  function GetUrlCacheEntryInfoEx(lpszUrl:LPCSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD; lpszRedirectUrl:LPSTR; var lpcbRedirectUrl:DWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExA';
  function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA;
             var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; var lpcbGroupAttributes:DWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExA';
  function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExA';
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
  function InternetDial(hwndParent:HWND; lpszConnectoid:LPSTR; dwFlags:DWORD; var lpdwConnection:DWORD; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialA';
  function InternetGetConnectedStateEx(var lpdwFlags:DWORD; lpszConnectionName:LPSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExA';
  function FtpGetCurrentDirectory(hConnect:HINTERNET; lpszCurrentDirectory:LPSTR; var lpdwCurrentDirectory:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryA';
  function FtpFindFirstFile(hConnect:HINTERNET; lpszSearchFile:LPCSTR; var lpFindFileData:WIN32_FIND_DATA; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileA';
  function GopherFindFirstFile(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszSearchString:LPCSTR; var lpFindData:GOPHER_FIND_DATAA; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileA';
  function GopherGetAttribute(hConnect:HINTERNET; lpszLocator:LPCSTR; lpszAttributeName:LPCSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             var lpdwCharactersReturned:DWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeA';
  function RetrieveUrlCacheEntryFile(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileA';
  function RetrieveUrlCacheEntryStream(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamA';
  function GetUrlCacheEntryInfo(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoA';
  function SetUrlCacheEntryInfo(lpszUrlName:LPCSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoA';
  function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExA';
  function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExA';
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCSTR; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
<<<<<<< HEAD
<<<<<<< HEAD
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCSTR; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:LongWord):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryA';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; var lpcbCacheEntryInfo:LongWord):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
=======
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
>>>>>>> graemeg/fixes_2_2
=======
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOA; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryA';
>>>>>>> origin/fixes_2_2

{$ELSE}

  function InternetTimeFromSystemTime(pst:PSYSTEMTIME; dwRFC:DWORD; lpszTime:LPWSTR; cbTime:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeFromSystemTimeW';
  function InternetTimeToSystemTime(lpszTime:LPCWSTR; pst:PSYSTEMTIME; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetTimeToSystemTimeW';
  function InternetCrackUrl(lpszUrl:LPCWSTR; dwUrlLength:DWORD; dwFlags:DWORD; lpUrlComponents:LPURL_COMPONENTSW):BOOL;stdcall;external WININETLIBNAME name 'InternetCrackUrlW';
  function InternetCreateUrl(lpUrlComponents:LPURL_COMPONENTSW; dwFlags:DWORD; lpszUrl:LPWSTR; lpdwUrlLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlW';
  function InternetCanonicalizeUrl(lpszUrl:LPCWSTR; lpszBuffer:LPWSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlW';
  function InternetCombineUrl(lpszBaseUrl:LPCWSTR; lpszRelativeUrl:LPCWSTR; lpszBuffer:LPWSTR; lpdwBufferLength:LPDWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlW';
  function InternetOpen(lpszAgent:LPCWSTR; dwAccessType:DWORD; lpszProxy:LPCWSTR; lpszProxyBypass:LPCWSTR; dwFlags:DWORD):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenW';	
  function InternetConnect(hInternet:HINTERNET; lpszServerName:LPCWSTR; nServerPort:INTERNET_PORT; lpszUserName:LPCWSTR; lpszPassword:LPCWSTR;
             dwService:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetConnectW';
  function InternetOpenUrl(hInternet:HINTERNET; lpszUrl:LPCWSTR; lpszHeaders:LPCWSTR; dwHeadersLength:DWORD; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'InternetOpenUrlW';			
  function InternetReadFileEx(hFile:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSW; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'InternetReadFileExW';
    function InternetFindNextFile(hFind:HINTERNET; lpvFindData:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetFindNextFileW';
  function InternetQueryOption(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionW';
  function InternetSetOption(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionW';
  function InternetSetOptionEx(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; dwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetOptionExW';
  function InternetGetLastResponseInfo(lpdwError:LPDWORD; lpszBuffer:LPWSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoW';
  function InternetSetStatusCallback(hInternet:HINTERNET; lpfnInternetCallback:INTERNET_STATUS_CALLBACK):INTERNET_STATUS_CALLBACK;stdcall;external WININETLIBNAME name 'InternetSetStatusCallbackW';
  function FtpFindFirstFile(hConnect:HINTERNET; lpszSearchFile:LPCWSTR; lpFindFileData:LPWIN32_FIND_DATAW; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileW';
  function FtpGetFile(hConnect:HINTERNET; lpszRemoteFile:LPCWSTR; lpszNewFile:LPCWSTR; fFailIfExists:BOOL; dwFlagsAndAttributes:DWORD;
             dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpGetFileW';
  function FtpPutFile(hConnect:HINTERNET; lpszLocalFile:LPCWSTR; lpszNewRemoteFile:LPCWSTR; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'FtpPutFileW';
  function FtpDeleteFile(hConnect:HINTERNET; lpszFileName:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpDeleteFileW';
  function FtpRenameFile(hConnect:HINTERNET; lpszExisting:LPCWSTR; lpszNew:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRenameFileW';
  function FtpOpenFile(hConnect:HINTERNET; lpszFileName:LPCWSTR; dwAccess:DWORD; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpOpenFileW';
  function FtpCreateDirectory(hConnect:HINTERNET; lpszDirectory:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpCreateDirectoryW';
  function FtpRemoveDirectory(hConnect:HINTERNET; lpszDirectory:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpRemoveDirectoryW';
  function FtpSetCurrentDirectory(hConnect:HINTERNET; lpszDirectory:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'FtpSetCurrentDirectoryW';
  function FtpGetCurrentDirectory(hConnect:HINTERNET; lpszCurrentDirectory:LPWSTR; lpdwCurrentDirectory:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryW';
  function FtpCommand(hConnect:HINTERNET; fExpectResponse:BOOL; dwFlags:DWORD; lpszCommand:LPCWSTR; dwContext:DWORD_PTR;
             phFtpCommand:PHINTERNET):BOOL;stdcall;external WININETLIBNAME name 'FtpCommandW';
  function GopherCreateLocator(lpszHost:LPCWSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCWSTR; lpszSelectorString:LPCWSTR; dwGopherType:DWORD;
             lpszLocator:LPWSTR; lpdwBufferLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorW';
  function GopherGetLocatorType(lpszLocator:LPCWSTR; lpdwGopherType:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeW';
  function GopherFindFirstFile(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszSearchString:LPCWSTR; lpFindData:LPGOPHER_FIND_DATAW; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileW';		
  function GopherOpenFile(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszView:LPCWSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherOpenFileW';
  function GopherGetAttribute(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszAttributeName:LPCWSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             lpdwCharactersReturned:LPDWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeW';
  function HttpOpenRequest(hConnect:HINTERNET; lpszVerb:LPCWSTR; lpszObjectName:LPCWSTR; lpszVersion:LPCWSTR; lpszReferrer:LPCWSTR;
             lplpszAcceptTypes:LPPCWSTR; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'HttpOpenRequestW';
  function HttpAddRequestHeaders(hRequest:HINTERNET; lpszHeaders:LPCWSTR; dwHeadersLength:DWORD; dwModifiers:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpAddRequestHeadersW';
  function HttpSendRequest(hRequest:HINTERNET; lpszHeaders:LPCWSTR; dwHeadersLength:DWORD; lpOptional:LPVOID; dwOptionalLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestW';
  function HttpSendRequestEx(hRequest:HINTERNET; lpBuffersIn:LPINTERNET_BUFFERSW; lpBuffersOut:LPINTERNET_BUFFERSW; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpSendRequestExW';
  function HttpEndRequest(hRequest:HINTERNET; lpBuffersOut:LPINTERNET_BUFFERSW; dwFlags:DWORD; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'HttpEndRequestW';
  function HttpQueryInfo(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; lpdwBufferLength:LPDWORD; lpdwIndex:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoW';
  function InternetSetCookie(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'InternetSetCookieW';
  function InternetGetCookie(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR; lpdwSize:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieW';
  function InternetSetCookieEx(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPCWSTR; dwFlags:DWORD; dwReserved:DWORD_PTR):DWORD;stdcall;external WININETLIBNAME name 'InternetSetCookieExW';
  function InternetGetCookieEx(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR; lpdwSize:LPDWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExW';
  function InternetCheckConnection(lpszUrl:LPCWSTR; dwFlags:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCheckConnectionW';
  function InternetConfirmZoneCrossing(hWnd:HWND; szUrlPrev:LPWSTR; szUrlNew:LPWSTR; bPost:BOOL):DWORD;stdcall;external WININETLIBNAME name 'InternetConfirmZoneCrossingW';
  function CreateUrlCacheEntry(lpszUrlName:LPCWSTR; dwExpectedFileSize:DWORD; lpszFileExtension:LPCWSTR; lpszFileName:LPWSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'CreateUrlCacheEntryW';
  function CommitUrlCacheEntry(lpszUrlName:LPCWSTR; lpszLocalFileName:LPCWSTR; ExpireTime:FILETIME; LastModifiedTime:FILETIME; CacheEntryType:DWORD;
             lpHeaderInfo:LPCWSTR; dwHeaderSize:DWORD; lpszFileExtension:LPCWSTR; lpszOriginalUrl:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'CommitUrlCacheEntryW';
  function RetrieveUrlCacheEntryFile(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileW';
  function UnlockUrlCacheEntryFile(lpszUrlName:LPCWSTR; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'UnlockUrlCacheEntryFileW';
  function RetrieveUrlCacheEntryStream(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamW';
  function GetUrlCacheEntryInfo(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoW';

  function GetUrlCacheGroupAttribute(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW; lpdwGroupInfo:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeW';
  function SetUrlCacheGroupAttribute(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW; lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheGroupAttributeW';
  function GetUrlCacheEntryInfoEx(lpszUrl:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; lpszRedirectUrl:LPWSTR; lpcbRedirectUrl:LPDWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExW';
  function SetUrlCacheEntryInfo(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoW';
  function SetUrlCacheEntryGroup(lpszUrlName:LPCWSTR; dwFlags:DWORD; GroupId:GROUPID; pbGroupAttributes:LPBYTE; cbGroupAttributes:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryGroupW';
  function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCWSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExW';
  function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExW';
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCWSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
<<<<<<< HEAD
<<<<<<< HEAD
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCWSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:LongWord):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:LongWord):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
//  function FindCloseUrlCache(hEnumHandle:HANDLE):BOOL;stdcall;external WININETLIBNAME name 'FindCloseUrlCache';
=======
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
  function FindCloseUrlCache(hEnumHandle:HANDLE):BOOL;stdcall;external WININETLIBNAME name 'FindCloseUrlCache';
>>>>>>> graemeg/fixes_2_2
=======
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
  function FindCloseUrlCache(hEnumHandle:HANDLE):BOOL;stdcall;external WININETLIBNAME name 'FindCloseUrlCache';
>>>>>>> origin/fixes_2_2
  function DeleteUrlCacheEntry(lpszUrlName:LPCWSTR):BOOL;stdcall;external WININETLIBNAME name 'DeleteUrlCacheEntryW';
  function InternetDial(hwndParent:HWND; lpszConnectoid:LPWSTR; dwFlags:DWORD; lpdwConnection:PDWORD_PTR; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialW';

  function InternetGoOnline(lpszURL:LPWSTR; hwndParent:HWND; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGoOnlineW';
  function InternetGetConnectedStateEx(lpdwFlags:LPDWORD; lpszConnectionName:LPWSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExW';
  function InternetSetDialState(lpszConnectoid:LPCWSTR; dwState:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetDialStateW';
  function InternetSetPerSiteCookieDecision(pchHostName:LPCWSTR; dwDecision:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetSetPerSiteCookieDecisionW';
  function InternetGetPerSiteCookieDecision(pchHostName:LPCWSTR; pResult:Pdword):BOOL;stdcall;external WININETLIBNAME name 'InternetGetPerSiteCookieDecisionW';
  function InternetEnumPerSiteCookieDecision(pszSiteName:LPWSTR; pcSiteNameSize:Pdword; pdwDecision:Pdword; dwIndex:dword):BOOL;stdcall;external WININETLIBNAME name 'InternetEnumPerSiteCookieDecisionW';
  function PrivacySetZonePreference(dwZone:DWORD; dwType:DWORD; dwTemplate:DWORD; pszPreference:LPCWSTR):DWORD;stdcall;external WININETLIBNAME name 'PrivacySetZonePreferenceW';
  function PrivacyGetZonePreference(dwZone:DWORD; dwType:DWORD; pdwTemplate:LPDWORD; pszBuffer:LPWSTR; pdwBufferLength:LPDWORD):DWORD;stdcall;external WININETLIBNAME name 'PrivacyGetZonePreferenceW';

  {wide overloads}
  function InternetCrackUrl(lpszUrl:LPCWSTR; dwUrlLength:DWORD; dwFlags:DWORD; var lpUrlComponents:URL_COMPONENTSW):BOOL;stdcall;external WININETLIBNAME name 'InternetCrackUrlW';
  function InternetCreateUrl(var lpUrlComponents:URL_COMPONENTSW; dwFlags:DWORD; lpszUrl:LPWSTR; lpdwUrlLength:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlW';

  function InternetCreateUrl(lpUrlComponents:LPURL_COMPONENTSW; dwFlags:DWORD; lpszUrl:LPWSTR;var lpdwUrlLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCreateUrlW';
  function InternetCanonicalizeUrl(lpszUrl:LPCWSTR; lpszBuffer:LPWSTR; var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCanonicalizeUrlW';
  function InternetCombineUrl(lpszBaseUrl:LPCWSTR; lpszRelativeUrl:LPCWSTR; lpszBuffer:LPWSTR;var lpdwBufferLength:DWORD; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetCombineUrlW';
  function InternetQueryOption(hInternet:HINTERNET; dwOption:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetQueryOptionW';
  function InternetGetLastResponseInfo(lpdwError:LPDWORD; lpszBuffer:LPWSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetLastResponseInfoW';
  function GopherCreateLocator(lpszHost:LPCWSTR; nServerPort:INTERNET_PORT; lpszDisplayString:LPCWSTR; lpszSelectorString:LPCWSTR; dwGopherType:DWORD;
             lpszLocator:LPWSTR; var lpdwBufferLength:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherCreateLocatorW';
  function GopherGetLocatorType(lpszLocator:LPCWSTR; var lpdwGopherType:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GopherGetLocatorTypeW';
  function HttpQueryInfo(hRequest:HINTERNET; dwInfoLevel:DWORD; lpBuffer:LPVOID; var lpdwBufferLength:DWORD; var lpdwIndex:DWORD):BOOL;stdcall;external WININETLIBNAME name 'HttpQueryInfoW';
  function InternetGetCookie(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR;var lpdwSize:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieW';
  function InternetGetCookieEx(lpszUrl:LPCWSTR; lpszCookieName:LPCWSTR; lpszCookieData:LPWSTR; var lpdwSize:DWORD; dwFlags:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'InternetGetCookieExW';
  function RetrieveUrlCacheEntryFile(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileW';
  function RetrieveUrlCacheEntryStream(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamW';
  function GetUrlCacheEntryInfo(lpszUrlName:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoW';
  function GetUrlCacheGroupAttribute(gid:GROUPID; dwFlags:DWORD; dwAttributes:DWORD; lpGroupInfo:LPINTERNET_CACHE_GROUP_INFOW; var lpdwGroupInfo:DWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheGroupAttributeW';
  function GetUrlCacheEntryInfoEx(lpszUrl:LPCWSTR; lpCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD; lpszRedirectUrl:LPWSTR; var lpcbRedirectUrl:DWORD;
             lpReserved:LPVOID; dwFlags:DWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoExW';
  function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCWSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW;
             var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; var lpcbGroupAttributes:DWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExW';
  function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbEntryInfo:DWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExW';
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCWSTR; lpFirstCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; lpNextCacheEntryInfo:LPINTERNET_CACHE_ENTRY_INFOW; var lpcbCacheEntryInfo:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';
  function InternetDial(hwndParent:HWND; lpszConnectoid:LPWSTR; dwFlags:DWORD; var lpdwConnection:DWORD; dwReserved:DWORD):DWORD;stdcall;external WININETLIBNAME name 'InternetDialW';
  function InternetGetConnectedStateEx(var lpdwFlags:DWORD; lpszConnectionName:LPWSTR; dwBufLen:DWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'InternetGetConnectedStateExW';

  function FtpGetCurrentDirectory(hConnect:HINTERNET; lpszCurrentDirectory:LPWSTR; var lpdwCurrentDirectory:DWORD):BOOL;stdcall;external WININETLIBNAME name 'FtpGetCurrentDirectoryW';
  function FtpFindFirstFile(hConnect:HINTERNET; lpszSearchFile:LPCWSTR; var lpFindFileData:WIN32_FIND_DATAW; dwFlags:DWORD; dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'FtpFindFirstFileW';
  function GopherFindFirstFile(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszSearchString:LPCWSTR; var lpFindData:GOPHER_FIND_DATAW; dwFlags:DWORD;
             dwContext:DWORD_PTR):HINTERNET;stdcall;external WININETLIBNAME name 'GopherFindFirstFileW';
  function GopherGetAttribute(hConnect:HINTERNET; lpszLocator:LPCWSTR; lpszAttributeName:LPCWSTR; lpBuffer:LPBYTE; dwBufferLength:DWORD;
             var lpdwCharactersReturned:DWORD; lpfnEnumerator:GOPHER_ATTRIBUTE_ENUMERATOR; dwContext:DWORD_PTR):BOOL;stdcall;external WININETLIBNAME name 'GopherGetAttributeW';

  function RetrieveUrlCacheEntryFile(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; dwReserved:DWORD):BOOL;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryFileW';
  function RetrieveUrlCacheEntryStream(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD; fRandomRead:BOOL; dwReserved:DWORD):HANDLE;stdcall;external WININETLIBNAME name 'RetrieveUrlCacheEntryStreamW';
  function GetUrlCacheEntryInfo(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'GetUrlCacheEntryInfoW';
  function SetUrlCacheEntryInfo(lpszUrlName:LPCWSTR; var lpCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; dwFieldControl:DWORD):BOOL;stdcall;external WININETLIBNAME name 'SetUrlCacheEntryInfoW';
  function FindFirstUrlCacheEntryEx(lpszUrlSearchPattern:LPCWSTR; dwFlags:DWORD; dwFilter:DWORD; GroupId:GROUPID; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW;
             lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD; lpReserved:LPVOID):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryExW';
  function FindNextUrlCacheEntryEx(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbEntryInfo:LPDWORD; lpGroupAttributes:LPVOID; lpcbGroupAttributes:LPDWORD;
             lpReserved:LPVOID):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryExW';
  function FindFirstUrlCacheEntry(lpszUrlSearchPattern:LPCWSTR; var lpFirstCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):HANDLE;stdcall;external WININETLIBNAME name 'FindFirstUrlCacheEntryW';
  function FindNextUrlCacheEntry(hEnumHandle:HANDLE; var lpNextCacheEntryInfo:INTERNET_CACHE_ENTRY_INFOW; lpcbCacheEntryInfo:LPDWORD):BOOL;stdcall;external WININETLIBNAME name 'FindNextUrlCacheEntryW';


{$endif}
    

 function IS_GOPHER_TEXT_FILE(gtype:DWORD):BOOL; inline;           
 function IS_GOPHER_DIRECTORY(gtype:DWORD):BOOL; inline;       
 function IS_GOPHER_CSO(gtype:DWORD):BOOL; inline;
 function IS_GOPHER_ERROR(gtype:DWORD):BOOL; inline;           
 function IS_GOPHER_MAC_BINHEX(gtype:DWORD):BOOL; inline;      
 function IS_GOPHER_DOS_ARCHIVE(gtype:DWORD):BOOL; inline;     
 function IS_GOPHER_UNIX_UUENCODED(gtype:DWORD):BOOL; inline;  
 function IS_GOPHER_INDEX_SERVER(gtype:DWORD):BOOL; inline;    
 function IS_GOPHER_TELNET(gtype:DWORD):BOOL; inline;          
 function IS_GOPHER_BINARY(gtype:DWORD):BOOL; inline;          
 function IS_GOPHER_REDUNDANT(gtype:DWORD):BOOL; inline;       
 function IS_GOPHER_TN3270(gtype:DWORD):BOOL; inline;          
 function IS_GOPHER_GIF(gtype:DWORD):BOOL; inline;             
 function IS_GOPHER_IMAGE(gtype:DWORD):BOOL; inline;           
 function IS_GOPHER_BITMAP(gtype:DWORD):BOOL; inline;          
 function IS_GOPHER_MOVIE(gtype:DWORD):BOOL; inline;           
 function IS_GOPHER_SOUND(gtype:DWORD):BOOL; inline;           
 function IS_GOPHER_HTML(gtype:DWORD):BOOL; inline;            
 function IS_GOPHER_PDF(gtype:DWORD):BOOL; inline;             
 function IS_GOPHER_CALENDAR(gtype:DWORD):BOOL; inline;        
 function IS_GOPHER_INLINE(gtype:DWORD):BOOL; inline;          
 function IS_GOPHER_UNKNOWN(gtype:DWORD):BOOL; inline;         
 function IS_GOPHER_ASK(gtype:DWORD):BOOL; inline;             
 function IS_GOPHER_GOPHER_PLUS(gtype:DWORD):BOOL; inline;

implementation

 function IS_GOPHER_TEXT_FILE(gtype:DWORD):BOOL;            
    begin
      result:=(gtype and GOPHER_TYPE_TEXT_FILE)=0;
    end;
 function IS_GOPHER_DIRECTORY(gtype:DWORD):BOOL;        
    begin
      result:=(gtype and GOPHER_TYPE_DIRECTORY)=0;
    end;
 function IS_GOPHER_CSO(gtype:DWORD):BOOL; 
    begin
      result:=(gtype and GOPHER_TYPE_CSO)=0;
    end;
 function IS_GOPHER_ERROR(gtype:DWORD):BOOL;            
    begin
      result:=(gtype and GOPHER_TYPE_ERROR)=0;
    end;
 function IS_GOPHER_MAC_BINHEX(gtype:DWORD):BOOL;       
    begin
      result:=(gtype and GOPHER_TYPE_MAC_BINHEX)=0;
    end;
 function IS_GOPHER_DOS_ARCHIVE(gtype:DWORD):BOOL;      
    begin
      result:=(gtype and GOPHER_TYPE_DOS_ARCHIVE)=0;
    end;
 function IS_GOPHER_UNIX_UUENCODED(gtype:DWORD):BOOL;   
    begin
      result:=(gtype and GOPHER_TYPE_UNIX_UUENCODED)=0;
    end;
 function IS_GOPHER_INDEX_SERVER(gtype:DWORD):BOOL;     
    begin
      result:=(gtype and GOPHER_TYPE_INDEX_SERVER)=0;
    end;
 function IS_GOPHER_TELNET(gtype:DWORD):BOOL;           
    begin
      result:=(gtype and GOPHER_TYPE_TELNET)=0;
    end;
 function IS_GOPHER_BINARY(gtype:DWORD):BOOL;           
    begin
      result:=(gtype and GOPHER_TYPE_BINARY)=0;
    end;
 function IS_GOPHER_REDUNDANT(gtype:DWORD):BOOL;        
    begin
      result:=(gtype and GOPHER_TYPE_REDUNDANT)=0;
    end;
 function IS_GOPHER_TN3270(gtype:DWORD):BOOL;           
    begin
      result:=(gtype and GOPHER_TYPE_TN3270)=0;
    end;
 function IS_GOPHER_GIF(gtype:DWORD):BOOL;              
    begin
      result:=(gtype and GOPHER_TYPE_GIF)=0;
    end;
 function IS_GOPHER_IMAGE(gtype:DWORD):BOOL;            
    begin
      result:=(gtype and GOPHER_TYPE_IMAGE)=0;
    end;
 function IS_GOPHER_BITMAP(gtype:DWORD):BOOL;           
    begin
      result:=(gtype and GOPHER_TYPE_BITMAP)=0;
    end;
 function IS_GOPHER_MOVIE(gtype:DWORD):BOOL;            
    begin
      result:=(gtype and GOPHER_TYPE_MOVIE)=0;
    end;
 function IS_GOPHER_SOUND(gtype:DWORD):BOOL;            
    begin
      result:=(gtype and GOPHER_TYPE_SOUND)=0;
    end;
 function IS_GOPHER_HTML(gtype:DWORD):BOOL;             
    begin
      result:=(gtype and GOPHER_TYPE_HTML)=0;
    end;
 function IS_GOPHER_PDF(gtype:DWORD):BOOL;              
    begin
      result:=(gtype and GOPHER_TYPE_PDF)=0;
    end;  
 function IS_GOPHER_CALENDAR(gtype:DWORD):BOOL;         
    begin
      result:=(gtype and GOPHER_TYPE_CALENDAR)=0;
    end;  
 function IS_GOPHER_INLINE(gtype:DWORD):BOOL;           
    begin
      result:=(gtype and GOPHER_TYPE_INLINE)=0;
    end;     
 function IS_GOPHER_UNKNOWN(gtype:DWORD):BOOL;          
    begin
      result:=(gtype and GOPHER_TYPE_UNKNOWN)=0;
    end;  
 function IS_GOPHER_ASK(gtype:DWORD):BOOL;              
    begin
      result:=(gtype and GOPHER_TYPE_ASK)=0;
    end;  
 function IS_GOPHER_GOPHER_PLUS(gtype:DWORD):BOOL; 
    begin
      result:=(gtype and GOPHER_TYPE_GOPHER_PLUS)=0;
    end;  

<<<<<<< HEAD
<<<<<<< HEAD
end.
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
