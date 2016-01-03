{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Pascal packages
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
 {
   the curl library is governed by its own copyright, see the curl
   website for this. 
 }
{$mode objfpc}
unit libcurl;

interface

uses unixtype;

{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}

{ Automatically converted by H2Pas 1.0.0 from curl.h
  The following command line parameters were used:
    -D -l libcurl -p curl.h }

const
  External_library='libcurl'; {Setup as you need}

Type

  Pchar  = ^char;
  Pcurl_calloc_callback  = ^curl_calloc_callback;
  Pcurl_closepolicy  = ^curl_closepolicy;
  Pcurl_forms  = ^curl_forms;
  Pcurl_ftpauth  = ^curl_ftpauth;
  Pcurl_ftpmethod  = ^curl_ftpmethod;
  Pcurl_ftpssl  = ^curl_ftpssl;
  PCURL_HTTP_VERSION  = ^CURL_HTTP_VERSION;
  Pcurl_httppost  = ^curl_httppost;
  PPcurl_httppost = ^Pcurl_httppost;
  Pcurl_infotype  = ^curl_infotype;
  Pcurl_lock_access  = ^curl_lock_access;
  Pcurl_lock_data  = ^curl_lock_data;
  Pcurl_malloc_callback  = ^curl_malloc_callback;
  PCURL_NETRC_OPTION  = ^CURL_NETRC_OPTION;
  Pcurl_off_t  = ^curl_off_t;
  Pcurl_proxytype  = ^curl_proxytype;
  Pcurl_realloc_callback  = ^curl_realloc_callback;
  Pcurl_slist  = ^curl_slist;
  Pcurl_socket_t  = ^curl_socket_t;
  PCURL_SSL_VERSION  = ^CURL_SSL_VERSION;
  Pcurl_strdup_callback  = ^curl_strdup_callback;
  PCURL_TIMECOND  = ^CURL_TIMECOND;
  Pcurl_version_info_data  = ^curl_version_info_data;
  PCURLcode  = ^CURLcode;
  PCURLFORMcode  = ^CURLFORMcode;
  PCURLformoption  = ^CURLformoption;
  PCURLINFO  = ^CURLINFO;
  Pcurliocmd  = ^curliocmd;
  Pcurlioerr  = ^curlioerr;
  PCURLM  = ^CURLM;
  PCURLMcode  = ^CURLMcode;
  PCURLMoption  = ^CURLMoption;
  PCURLMSG  = ^CURLMSG;
  PCURLoption  = ^CURLoption;
  PCURLSH  = ^CURLSH;
  PCURLSHcode  = ^CURLSHcode;
  PCURLSHoption  = ^CURLSHoption;
  PCURLversion  = ^CURLversion;
  Pfd_set = pointer;

  PCURL = ^CURL;
  CURL = pointer;
  curl_off_t = off_t;

  curl_httppost = record
    next : Pcurl_httppost;
    name : Pchar;
    namelength : longint;
    contents : Pchar;
    contentslength : longint;
    buffer : Pchar;
    bufferlength : longint;
    contenttype : Pchar;
    contentheader : Pcurl_slist;
    more : Pcurl_httppost;
    flags : longint;
    showfilename : Pchar;
  end;


  curl_progress_callback = function (clientp:pointer; dltotal:double; dlnow:double; ultotal:double; ulnow:double):longint;cdecl;
  curl_write_callback = function (buffer:Pchar; size:size_t; nitems:size_t; outstream:pointer):size_t;cdecl;
  curl_read_callback = function (buffer:Pchar; size:size_t; nitems:size_t; instream:pointer):size_t;cdecl;
  curl_passwd_callback = function (clientp:pointer; prompt:Pchar; buffer:Pchar; buflen:longint):longint;cdecl;


  curlioerr = (CURLIOE_OK, CURLIOE_UNKNOWNCMD, CURLIOE_FAILRESTART, CURLIOE_LAST);


  curliocmd = (CURLIOCMD_NOP, CURLIOCMD_RESTARTREAD, CURLIOCMD_LAST);

  curl_ioctl_callback = function (handle:PCURL; cmd:longint; clientp:pointer):curlioerr;cdecl;
  curl_malloc_callback = function(size: size_t) : pointer; cdecl;
  curl_free_callback = procedure (ptr:pointer); cdecl;
  curl_realloc_callback = function(ptr : pointer; size:size_t) : pointer; cdecl;
  curl_strdup_callback = function(str : pchar) : pchar; cdecl;
  curl_calloc_callback = function(nmemb : size_t; size : size_t) : pointer;

  curl_infotype = (CURLINFO_TEXT := 0,CURLINFO_HEADER_IN,
                   CURLINFO_HEADER_OUT,CURLINFO_DATA_IN,
                   CURLINFO_DATA_OUT,CURLINFO_SSL_DATA_IN,
                   CURLINFO_SSL_DATA_OUT,CURLINFO_END);

  curl_debug_callback = function (handle:PCURL; _type:curl_infotype; data:Pchar; size:size_t; userptr:pointer):longint;cdecl;

  CURLcode = (CURLE_OK := 0,CURLE_UNSUPPORTED_PROTOCOL,
    CURLE_FAILED_INIT,CURLE_URL_MALFORMAT,
    CURLE_URL_MALFORMAT_USER,CURLE_COULDNT_RESOLVE_PROXY,
    CURLE_COULDNT_RESOLVE_HOST,CURLE_COULDNT_CONNECT,
    CURLE_FTP_WEIRD_SERVER_REPLY,CURLE_FTP_ACCESS_DENIED,
    CURLE_FTP_USER_PASSWORD_INCORRECT,CURLE_FTP_WEIRD_PASS_REPLY,
    CURLE_FTP_WEIRD_USER_REPLY,CURLE_FTP_WEIRD_PASV_REPLY,
    CURLE_FTP_WEIRD_227_FORMAT,CURLE_FTP_CANT_GET_HOST,
    CURLE_FTP_CANT_RECONNECT,CURLE_FTP_COULDNT_SET_BINARY,
    CURLE_PARTIAL_FILE,CURLE_FTP_COULDNT_RETR_FILE,
    CURLE_FTP_WRITE_ERROR,CURLE_FTP_QUOTE_ERROR,
    CURLE_HTTP_RETURNED_ERROR,CURLE_WRITE_ERROR,
    CURLE_MALFORMAT_USER,CURLE_FTP_COULDNT_STOR_FILE,
    CURLE_READ_ERROR,CURLE_OUT_OF_MEMORY,
    CURLE_OPERATION_TIMEOUTED,CURLE_FTP_COULDNT_SET_ASCII,
    CURLE_FTP_PORT_FAILED,CURLE_FTP_COULDNT_USE_REST,
    CURLE_FTP_COULDNT_GET_SIZE,CURLE_HTTP_RANGE_ERROR,
    CURLE_HTTP_POST_ERROR,CURLE_SSL_CONNECT_ERROR,
    CURLE_BAD_DOWNLOAD_RESUME,CURLE_FILE_COULDNT_READ_FILE,
    CURLE_LDAP_CANNOT_BIND,CURLE_LDAP_SEARCH_FAILED,
    CURLE_LIBRARY_NOT_FOUND,CURLE_FUNCTION_NOT_FOUND,
    CURLE_ABORTED_BY_CALLBACK,CURLE_BAD_FUNCTION_ARGUMENT,
    CURLE_BAD_CALLING_ORDER,CURLE_INTERFACE_FAILED,
    CURLE_BAD_PASSWORD_ENTERED,CURLE_TOO_MANY_REDIRECTS,
    CURLE_UNKNOWN_TELNET_OPTION,CURLE_TELNET_OPTION_SYNTAX,
    CURLE_OBSOLETE,CURLE_SSL_PEER_CERTIFICATE,
    CURLE_GOT_NOTHING,CURLE_SSL_ENGINE_NOTFOUND,
    CURLE_SSL_ENGINE_SETFAILED,CURLE_SEND_ERROR,
    CURLE_RECV_ERROR,CURLE_SHARE_IN_USE,
    CURLE_SSL_CERTPROBLEM,CURLE_SSL_CIPHER,
    CURLE_SSL_CACERT,CURLE_BAD_CONTENT_ENCODING,
    CURLE_LDAP_INVALID_URL,CURLE_FILESIZE_EXCEEDED,
    CURLE_FTP_SSL_FAILED,CURLE_SEND_FAIL_REWIND,
    CURLE_SSL_ENGINE_INITFAILED,CURLE_LOGIN_DENIED,
    CURLE_TFTP_NOTFOUND,CURLE_TFTP_PERM,
    CURLE_TFTP_DISKFULL,CURLE_TFTP_ILLEGAL,
    CURLE_TFTP_UNKNOWNID,CURLE_TFTP_EXISTS,
    CURLE_TFTP_NOSUCHUSER,CURLE_CONV_FAILED,
    CURLE_CONV_REQD,CURL_LAST);
 
  curl_conv_callback = function (buffer:Pchar; length:size_t):CURLcode;cdecl;
  curl_ssl_ctx_callback = function (curl:PCURL; ssl_ctx:pointer; userptr:pointer):CURLcode;cdecl;

  curl_proxytype = (CURLPROXY_HTTP := 0,CURLPROXY_SOCKS4 := 4, CURLPROXY_SOCKS5 := 5);

  curl_ftpssl = (CURLFTPSSL_NONE,CURLFTPSSL_TRY,CURLFTPSSL_CONTROL, CURLFTPSSL_ALL,CURLFTPSSL_LAST);

  curl_ftpauth = (CURLFTPAUTH_DEFAULT,CURLFTPAUTH_SSL, CURLFTPAUTH_TLS,CURLFTPAUTH_LAST);

  curl_ftpmethod = (CURLFTPMETHOD_DEFAULT,CURLFTPMETHOD_MULTICWD,  CURLFTPMETHOD_NOCWD,CURLFTPMETHOD_SINGLECWD,
                    CURLFTPMETHOD_LAST);

  CURLoption = (CURLOPT_FILE := 10000+1,CURLOPT_URL := 10000+2,
     CURLOPT_PORT := 0+3,CURLOPT_PROXY := 10000+4,
     CURLOPT_USERPWD := 10000+5,CURLOPT_PROXYUSERPWD := 10000+6,
     CURLOPT_RANGE := 10000+7,CURLOPT_INFILE := 10000+9,
     CURLOPT_ERRORBUFFER := 10000+10,CURLOPT_WRITEFUNCTION := 20000+11,
     CURLOPT_READFUNCTION := 20000+12,CURLOPT_TIMEOUT := 0+13,
     CURLOPT_INFILESIZE := 0+14,CURLOPT_POSTFIELDS := 10000+15,
     CURLOPT_REFERER := 10000+16,CURLOPT_FTPPORT := 10000+17,
     CURLOPT_USERAGENT := 10000+18,CURLOPT_LOW_SPEED_LIMIT := 0+19,
     CURLOPT_LOW_SPEED_TIME := 0+20,CURLOPT_RESUME_FROM := 0+21,
     CURLOPT_COOKIE := 10000+22,CURLOPT_HTTPHEADER := 10000+23,
     CURLOPT_HTTPPOST := 10000+24,CURLOPT_SSLCERT := 10000+25,
     CURLOPT_SSLCERTPASSWD := 10000+26,CURLOPT_SSLKEYPASSWD := 10000+26,
     CURLOPT_CRLF := 0+27,CURLOPT_QUOTE := 10000+28,
     CURLOPT_WRITEHEADER := 10000+29,CURLOPT_COOKIEFILE := 10000+31,
     CURLOPT_SSLVERSION := 0+32,CURLOPT_TIMECONDITION := 0+33,
     CURLOPT_TIMEVALUE := 0+34,CURLOPT_CUSTOMREQUEST := 10000+36,
     CURLOPT_STDERR := 10000+37,CURLOPT_POSTQUOTE := 10000+39,
     CURLOPT_WRITEINFO := 10000+40,CURLOPT_VERBOSE := 0+41,
     CURLOPT_HEADER := 0+42,CURLOPT_NOPROGRESS := 0+43,
     CURLOPT_NOBODY := 0+44,CURLOPT_FAILONERROR := 0+45,
     CURLOPT_UPLOAD := 0+46,CURLOPT_POST := 0+47,
     CURLOPT_FTPLISTONLY := 0+48,CURLOPT_FTPAPPEND := 0+50,
     CURLOPT_NETRC := 0+51,CURLOPT_FOLLOWLOCATION := 0+52,
     CURLOPT_TRANSFERTEXT := 0+53,CURLOPT_PUT := 0+54,
     CURLOPT_PROGRESSFUNCTION := 20000+56,CURLOPT_PROGRESSDATA := 10000+57,
     CURLOPT_AUTOREFERER := 0+58,CURLOPT_PROXYPORT := 0+59,
     CURLOPT_POSTFIELDSIZE := 0+60,CURLOPT_HTTPPROXYTUNNEL := 0+61,
     CURLOPT_INTERFACE := 10000+62,CURLOPT_KRB4LEVEL := 10000+63,
     CURLOPT_SSL_VERIFYPEER := 0+64,CURLOPT_CAINFO := 10000+65,
     CURLOPT_MAXREDIRS := 0+68,CURLOPT_FILETIME := 0+69,
     CURLOPT_TELNETOPTIONS := 10000+70,CURLOPT_MAXCONNECTS := 0+71,
     CURLOPT_CLOSEPOLICY := 0+72,CURLOPT_FRESH_CONNECT := 0+74,
     CURLOPT_FORBID_REUSE := 0+75,CURLOPT_RANDOM_FILE := 10000+76,
     CURLOPT_EGDSOCKET := 10000+77,CURLOPT_CONNECTTIMEOUT := 0+78,
     CURLOPT_HEADERFUNCTION := 20000+79,CURLOPT_HTTPGET := 0+80,
     CURLOPT_SSL_VERIFYHOST := 0+81,CURLOPT_COOKIEJAR := 10000+82,
     CURLOPT_SSL_CIPHER_LIST := 10000+83,CURLOPT_HTTP_VERSION := 0+84,
     CURLOPT_FTP_USE_EPSV := 0+85,CURLOPT_SSLCERTTYPE := 10000+86,
     CURLOPT_SSLKEY := 10000+87,CURLOPT_SSLKEYTYPE := 10000+88,
     CURLOPT_SSLENGINE := 10000+89,CURLOPT_SSLENGINE_DEFAULT := 0+90,
     CURLOPT_DNS_USE_GLOBAL_CACHE := 0+91,
     CURLOPT_DNS_CACHE_TIMEOUT := 0+92,CURLOPT_PREQUOTE := 10000+93,
     CURLOPT_DEBUGFUNCTION := 20000+94,CURLOPT_DEBUGDATA := 10000+95,
     CURLOPT_COOKIESESSION := 0+96,CURLOPT_CAPATH := 10000+97,
     CURLOPT_BUFFERSIZE := 0+98,CURLOPT_NOSIGNAL := 0+99,
     CURLOPT_SHARE := 10000+100,CURLOPT_PROXYTYPE := 0+101,
     CURLOPT_ENCODING := 10000+102,CURLOPT_PRIVATE := 10000+103,
     CURLOPT_HTTP200ALIASES := 10000+104,CURLOPT_UNRESTRICTED_AUTH := 0+105,
     CURLOPT_FTP_USE_EPRT := 0+106,CURLOPT_HTTPAUTH := 0+107,
     CURLOPT_SSL_CTX_FUNCTION := 20000+108,CURLOPT_SSL_CTX_DATA := 10000+109,
     CURLOPT_FTP_CREATE_MISSING_DIRS := 0+110,
     CURLOPT_PROXYAUTH := 0+111,CURLOPT_FTP_RESPONSE_TIMEOUT := 0+112,
     CURLOPT_IPRESOLVE := 0+113,CURLOPT_MAXFILESIZE := 0+114,
     CURLOPT_INFILESIZE_LARGE := 30000+115,CURLOPT_RESUME_FROM_LARGE := 30000+116,
     CURLOPT_MAXFILESIZE_LARGE := 30000+117,CURLOPT_NETRC_FILE := 10000+118,
     CURLOPT_FTP_SSL := 0+119,CURLOPT_POSTFIELDSIZE_LARGE := 30000+120,
     CURLOPT_TCP_NODELAY := 0+121,CURLOPT_SOURCE_USERPWD := 10000+123,
     CURLOPT_SOURCE_PREQUOTE := 10000+127,CURLOPT_SOURCE_POSTQUOTE := 10000+128,
     CURLOPT_FTPSSLAUTH := 0+129,CURLOPT_IOCTLFUNCTION := 20000+130,
     CURLOPT_IOCTLDATA := 10000+131,CURLOPT_SOURCE_URL := 10000+132,
     CURLOPT_SOURCE_QUOTE := 10000+133,CURLOPT_FTP_ACCOUNT := 10000+134,
     CURLOPT_COOKIELIST := 10000+135,CURLOPT_IGNORE_CONTENT_LENGTH := 0+136,
     CURLOPT_FTP_SKIP_PASV_IP := 0+137,CURLOPT_FTP_FILEMETHOD := 0+138,
     CURLOPT_LOCALPORT := 0+139,CURLOPT_LOCALPORTRANGE := 0+140,
     CURLOPT_CONNECT_ONLY := 0+141,CURLOPT_CONV_FROM_NETWORK_FUNCTION := 20000+142,
     CURLOPT_CONV_TO_NETWORK_FUNCTION := 20000+143,
     CURLOPT_CONV_FROM_UTF8_FUNCTION := 20000+144,
     CURLOPT_MAX_SEND_SPEED_LARGE := 30000+145,
     CURLOPT_MAX_RECV_SPEED_LARGE := 30000+146,
     CURLOPT_FTP_ALTERNATIVE_TO_USER := 10000+147,
     CURLOPT_LASTENTRY);

  CURL_HTTP_VERSION = (CURL_HTTP_VERSION_NONE,CURL_HTTP_VERSION_1_0,
                       CURL_HTTP_VERSION_1_1,CURL_HTTP_VERSION_LAST);

  CURL_NETRC_OPTION = (CURL_NETRC_IGNORED,CURL_NETRC_OPTIONAL,
                       CURL_NETRC_REQUIRED,CURL_NETRC_LAST);

  CURL_SSL_VERSION = (CURL_SSLVERSION_DEFAULT,CURL_SSLVERSION_TLSv1,
                      CURL_SSLVERSION_SSLv2,CURL_SSLVERSION_SSLv3,
                      CURL_SSLVERSION_LAST);


  CURL_TIMECOND = (CURL_TIMECOND_NONE,CURL_TIMECOND_IFMODSINCE,
                   CURL_TIMECOND_IFUNMODSINCE,CURL_TIMECOND_LASTMOD,
                   CURL_TIMECOND_LAST);

  CURLformoption = (CURLFORM_NOTHING,CURLFORM_COPYNAME,CURLFORM_PTRNAME,
                    CURLFORM_NAMELENGTH,CURLFORM_COPYCONTENTS,
                    CURLFORM_PTRCONTENTS,CURLFORM_CONTENTSLENGTH,
                    CURLFORM_FILECONTENT,CURLFORM_ARRAY,
                    CURLFORM_OBSOLETE,CURLFORM_FILE,CURLFORM_BUFFER,
                    CURLFORM_BUFFERPTR,CURLFORM_BUFFERLENGTH,
                    CURLFORM_CONTENTTYPE,CURLFORM_CONTENTHEADER,
                    CURLFORM_FILENAME,CURLFORM_END,CURLFORM_OBSOLETE2,
                    CURLFORM_LASTENTRY);

  curl_forms = record
    option : CURLformoption;
    value : Pchar;
  end;

  CURLFORMcode = (CURL_FORMADD_OK,CURL_FORMADD_MEMORY,
                  CURL_FORMADD_OPTION_TWICE,CURL_FORMADD_NULL,
                  CURL_FORMADD_UNKNOWN_OPTION,CURL_FORMADD_INCOMPLETE,
                  CURL_FORMADD_ILLEGAL_ARRAY,CURL_FORMADD_DISABLED,
                 CURL_FORMADD_LAST);
  curl_formget_callback = function (arg:pointer; buf:Pchar; len:size_t):size_t;cdecl;

  curl_slist = record
    data : Pchar;
    next : Pcurl_slist;
  end;

  CURLINFO = (CURLINFO_NONE,CURLINFO_EFFECTIVE_URL := $100000+1,
    CURLINFO_RESPONSE_CODE := $200000+2,CURLINFO_TOTAL_TIME := $300000+3,
    CURLINFO_NAMELOOKUP_TIME := $300000+4,CURLINFO_CONNECT_TIME := $300000+5,
    CURLINFO_PRETRANSFER_TIME := $300000+6,CURLINFO_SIZE_UPLOAD := $300000+7,
    CURLINFO_SIZE_DOWNLOAD := $300000+8,CURLINFO_SPEED_DOWNLOAD := $300000+9,
    CURLINFO_SPEED_UPLOAD := $300000+10,CURLINFO_HEADER_SIZE := $200000+11,
    CURLINFO_REQUEST_SIZE := $200000+12,CURLINFO_SSL_VERIFYRESULT := $200000+13,
    CURLINFO_FILETIME := $200000+14,CURLINFO_CONTENT_LENGTH_DOWNLOAD := $300000+15,
    CURLINFO_CONTENT_LENGTH_UPLOAD := $300000+16,
    CURLINFO_STARTTRANSFER_TIME := $300000+17,CURLINFO_CONTENT_TYPE := $100000+18,
    CURLINFO_REDIRECT_TIME := $300000+19,CURLINFO_REDIRECT_COUNT := $200000+20,
    CURLINFO_PRIVATE := $100000+21,CURLINFO_HTTP_CONNECTCODE := $200000+22,
    CURLINFO_HTTPAUTH_AVAIL := $200000+23,CURLINFO_PROXYAUTH_AVAIL := $200000+24,
    CURLINFO_OS_ERRNO := $200000+25,CURLINFO_NUM_CONNECTS := $200000+26,
    CURLINFO_SSL_ENGINES := $400000+27,CURLINFO_COOKIELIST := $400000+28,
    CURLINFO_LASTSOCKET := $200000+29,CURLINFO_FTP_ENTRY_PATH := $100000+30,
    CURLINFO_LASTONE := 30);
 
  curl_closepolicy = (CURLCLOSEPOLICY_NONE,CURLCLOSEPOLICY_OLDEST,
                      CURLCLOSEPOLICY_LEAST_RECENTLY_USED,
                      CURLCLOSEPOLICY_LEAST_TRAFFIC,CURLCLOSEPOLICY_SLOWEST,
                      CURLCLOSEPOLICY_CALLBACK,CURLCLOSEPOLICY_LAST);
 
  curl_lock_data = (CURL_LOCK_DATA_NONE := 0,CURL_LOCK_DATA_SHARE,
                    CURL_LOCK_DATA_COOKIE,CURL_LOCK_DATA_DNS,
                    CURL_LOCK_DATA_SSL_SESSION,CURL_LOCK_DATA_CONNECT,
                    CURL_LOCK_DATA_LAST);
  
  curl_lock_access = (CURL_LOCK_ACCESS_NONE := 0,
                      CURL_LOCK_ACCESS_SHARED := 1,
                      CURL_LOCK_ACCESS_SINGLE := 2,
                      CURL_LOCK_ACCESS_LAST);
 
  curl_lock_function = procedure (handle:PCURL; data:curl_lock_data; locktype:curl_lock_access; userptr:pointer);cdecl;
  curl_unlock_function = procedure (handle:PCURL; data:curl_lock_data; userptr:pointer);cdecl;
 
  CURLSH = pointer;
 
  CURLSHcode = (CURLSHE_OK,CURLSHE_BAD_OPTION,CURLSHE_IN_USE,
                CURLSHE_INVALID,CURLSHE_NOMEM,CURLSHE_LAST);
 
  CURLSHoption = (CURLSHOPT_NONE,CURLSHOPT_SHARE,CURLSHOPT_UNSHARE,
                  CURLSHOPT_LOCKFUNC,CURLSHOPT_UNLOCKFUNC,
                  CURLSHOPT_USERDATA,CURLSHOPT_LAST);

  CURLversion = (CURLVERSION_FIRST,CURLVERSION_SECOND,
                 CURLVERSION_THIRD,CURLVERSION_LAST);

  curl_version_info_data = record
    age : CURLversion;
    version : Pchar;
    version_num : dword;
    host : Pchar;
    features : longint;
    ssl_version : Pchar;
    ssl_version_num : longint;
    libz_version : Pchar;
    protocols : ^Pchar;
    ares : Pchar;
    ares_num : longint;
    libidn : Pchar;
    iconv_ver_num : longint;
  end;
  CURLM = pointer;
 
  curl_socket_t = longint;
 
  CURLMcode = (CURLM_CALL_MULTI_PERFORM := -(1),CURLM_OK,
               CURLM_BAD_HANDLE,CURLM_BAD_EASY_HANDLE,
               CURLM_OUT_OF_MEMORY,CURLM_INTERNAL_ERROR,
               CURLM_BAD_SOCKET,CURLM_UNKNOWN_OPTION, CURLM_LAST);
 
  TCURLMSG = (CURLMSG_NONE,CURLMSG_DONE,CURLMSG_LAST);

  CURLMsg = record
    msg : TCURLMSG;
    easy_handle : PCURL;
    data : record
      case longint of
        0 : ( whatever : pointer );
        1 : ( result : CURLcode );
    end;
  end;
  curl_socket_callback = function (easy:PCURL; s:curl_socket_t; what:longint; userp:pointer; socketp:pointer):longint;cdecl;
  CURLMoption = (CURLMOPT_SOCKETFUNCTION := 20000+1,CURLMOPT_SOCKETDATA := 10000+2, CURLMOPT_LASTENTRY);

const
  CURLAUTH_ANY =  not (0);     
  CURLAUTH_BASIC = 1 shl 0;     
  CURLAUTH_ANYSAFE =  not (CURLAUTH_BASIC);     
  CURLAUTH_DIGEST = 1 shl 1;     
  CURLAUTH_GSSNEGOTIATE = 1 shl 2;     
  CURLAUTH_NONE = 0;     
  CURLAUTH_NTLM = 1 shl 3;     
  CURLE_ALREADY_COMPLETE = 99999;     
  CURLE_FTP_BAD_DOWNLOAD_RESUME = CURLE_BAD_DOWNLOAD_RESUME;     
  CURLE_FTP_PARTIAL_FILE = CURLE_PARTIAL_FILE;     
  CURLE_HTTP_NOT_FOUND = CURLE_HTTP_RETURNED_ERROR;     
  CURLE_HTTP_PORT_FAILED = CURLE_INTERFACE_FAILED;     
  CURLE_OPERATION_TIMEDOUT = CURLE_OPERATION_TIMEOUTED;     
  CURL_ERROR_SIZE = 256;     
  CURL_FORMAT_OFF_T = '%ld';     
  CURL_GLOBAL_NOTHING = 0;     
  CURL_GLOBAL_SSL = 1 shl 0;     
  CURL_GLOBAL_WIN32 = 1 shl 1;     
  CURL_GLOBAL_ALL = CURL_GLOBAL_SSL or CURL_GLOBAL_WIN32;     
  CURL_GLOBAL_DEFAULT = CURL_GLOBAL_ALL;     
  CURLINFO_DOUBLE = $300000;     
  CURLINFO_HTTP_CODE = CURLINFO_RESPONSE_CODE;     
  CURLINFO_LONG = $200000;     
  CURLINFO_MASK = $0fffff;     
  CURLINFO_SLIST = $400000;     
  CURLINFO_STRING = $100000;     
  CURLINFO_TYPEMASK = $f00000;     
  CURL_IPRESOLVE_V4 = 1;     
  CURL_IPRESOLVE_V6 = 2;     
  CURL_IPRESOLVE_WHATEVER = 0;     
  CURL_MAX_WRITE_SIZE = 16384;     
  CURLM_CALL_MULTI_SOCKET = CURLM_CALL_MULTI_PERFORM;     
  CURLOPT_CLOSEFUNCTION = -(5);     
  CURLOPT_FTPASCII = CURLOPT_TRANSFERTEXT;     
  CURLOPT_HEADERDATA = CURLOPT_WRITEHEADER;     
  CURLOPT_HTTPREQUEST = -(1);     
  CURLOPT_MUTE = -(2);     
  CURLOPT_PASSWDDATA = -(4);     
  CURLOPT_PASSWDFUNCTION = -(3);     
  CURLOPT_PASV_HOST = -(9);     
  CURLOPT_READDATA = CURLOPT_INFILE;     
  CURLOPT_SOURCE_HOST = -(6);     
  CURLOPT_SOURCE_PATH = -(7);     
  CURLOPT_SOURCE_PORT = -(8);     
  CURLOPTTYPE_FUNCTIONPOINT = 20000;     
  CURLOPTTYPE_LONG = 0;     
  CURLOPTTYPE_OBJECTPOINT = 10000;     
  CURLOPTTYPE_OFF_T = 30000;     
  CURLOPT_WRITEDATA = CURLOPT_FILE;     
  CURL_POLL_IN = 1;     
  CURL_POLL_INOUT = 3;     
  CURL_POLL_NONE = 0;     
  CURL_POLL_OUT = 2;     
  CURL_POLL_REMOVE = 4;     
  CURL_READFUNC_ABORT = $10000000;     
  CURL_SOCKET_BAD = -(1);      
  CURL_SOCKET_TIMEOUT = CURL_SOCKET_BAD;     
  CURL_VERSION_ASYNCHDNS = 1 shl 7;     
  CURL_VERSION_CONV = 1 shl 12;     
  CURL_VERSION_DEBUG = 1 shl 6;     
  CURL_VERSION_GSSNEGOTIATE = 1 shl 5;     
  CURL_VERSION_IDN = 1 shl 10;     
  CURL_VERSION_IPV6 = 1 shl 0;     
  CURL_VERSION_KERBEROS4 = 1 shl 1;     
  CURL_VERSION_LARGEFILE = 1 shl 9;     
  CURL_VERSION_LIBZ = 1 shl 3;     
  CURLVERSION_NOW = CURLVERSION_THIRD;     
  CURL_VERSION_NTLM = 1 shl 4;     
  CURL_VERSION_SPNEGO = 1 shl 8;     
  CURL_VERSION_SSL = 1 shl 2;     
  CURL_VERSION_SSPI = 1 shl 11;     
  _FILE_OFFSET_BITS = 0;     
  FILESIZEBITS = 0;     
  FUNCTIONPOINT = CURLOPTTYPE_FUNCTIONPOINT;     
  HTTPPOST_BUFFER = 1 shl 4;     
  HTTPPOST_FILENAME = 1 shl 0;     
  HTTPPOST_PTRBUFFER = 1 shl 5;     
  HTTPPOST_PTRCONTENTS = 1 shl 3;     
  HTTPPOST_PTRNAME = 1 shl 2;     
  HTTPPOST_READFILE = 1 shl 1;     
  LIBCURL_VERSION = '7.15.5';     
  LIBCURL_VERSION_MAJOR = 7;     
  LIBCURL_VERSION_MINOR = 15;     
  LIBCURL_VERSION_NUM = $070f05;     
  LIBCURL_VERSION_PATCH = 5;     
 
function curl_strequal(s1:Pchar; s2:Pchar):longint;cdecl;external External_library name 'curl_strequal';
function curl_strnequal(s1:Pchar; s2:Pchar; n:size_t):longint;cdecl;external External_library name 'curl_strnequal';

function curl_formadd(httppost:PPcurl_httppost; last_post:PPcurl_httppost; args:array of const):CURLFORMcode;cdecl;external External_library name 'curl_formadd';
function curl_formadd(httppost:PPcurl_httppost; last_post:PPcurl_httppost):CURLFORMcode;cdecl;external External_library name 'curl_formadd';

function curl_formget(form:Pcurl_httppost; arg:pointer; append:curl_formget_callback):longint;cdecl;external External_library name 'curl_formget';
procedure curl_formfree(form:Pcurl_httppost);cdecl;external External_library name 'curl_formfree';
function curl_getenv(variable:Pchar):Pchar;cdecl;external External_library name 'curl_getenv';
function curl_version:Pchar;cdecl;external External_library name 'curl_version';
function curl_easy_escape(handle:PCURL; _string:Pchar; length:longint):Pchar;cdecl;external External_library name 'curl_easy_escape';
function curl_escape(_string:Pchar; length:longint):Pchar;cdecl;external External_library name 'curl_escape';
function curl_easy_unescape(handle:PCURL; _string:Pchar; length:longint; outlength:Plongint):Pchar;cdecl;external External_library name 'curl_easy_unescape';
function curl_unescape(_string:Pchar; length:longint):Pchar;cdecl;external External_library name 'curl_unescape';
procedure curl_free(p:pointer);cdecl;external External_library name 'curl_free';
function curl_global_init(flags:longint):CURLcode;cdecl;external External_library name 'curl_global_init';
function curl_global_init_mem(flags:longint; m:curl_malloc_callback; f:curl_free_callback; r:curl_realloc_callback; s:curl_strdup_callback; 
             c:curl_calloc_callback):CURLcode;cdecl;external External_library name 'curl_global_init_mem';

procedure curl_global_cleanup;cdecl;external External_library name 'curl_global_cleanup';
function curl_slist_append (curl_slist : Pcurl_slist; P : PChar) : Pcurl_slist; cdecl; external External_library name 'curl_slist_append';
procedure curl_slist_free_all(_para1:Pcurl_slist);cdecl;external External_library name 'curl_slist_free_all';
function curl_getdate(p:Pchar; unused:Ptime_t):time_t;cdecl;external External_library name 'curl_getdate';

function curl_share_init:PCURLSH;cdecl;external External_library name 'curl_share_init';
function curl_share_setopt(_para1:PCURLSH; option:CURLSHoption; args:array of const):CURLSHcode;cdecl;external External_library name 'curl_share_setopt';
function curl_share_setopt(_para1:PCURLSH; option:CURLSHoption):CURLSHcode;cdecl;external External_library name 'curl_share_setopt';
function curl_share_cleanup(_para1:PCURLSH):CURLSHcode;cdecl;external External_library name 'curl_share_cleanup';

function curl_version_info(_para1:CURLversion):Pcurl_version_info_data;cdecl;external External_library name 'curl_version_info';
function curl_easy_strerror(_para1:CURLcode):Pchar;cdecl;external External_library name 'curl_easy_strerror';
function curl_share_strerror(_para1:CURLSHcode):Pchar;cdecl;external External_library name 'curl_share_strerror';
function curl_easy_init:PCURL;cdecl;external External_library name 'curl_easy_init';
function curl_easy_setopt(curl:PCURL; option:CURLoption; args:array of const):CURLcode;cdecl;external External_library name 'curl_easy_setopt';
function curl_easy_setopt(curl:PCURL; option:CURLoption):CURLcode;cdecl;external External_library name 'curl_easy_setopt';
function curl_easy_perform(curl:PCURL):CURLcode;cdecl;external External_library name 'curl_easy_perform';
procedure curl_easy_cleanup(curl:PCURL);cdecl;external External_library name 'curl_easy_cleanup';
function curl_easy_getinfo(curl:PCURL; info:CURLINFO; args:array of const):CURLcode;cdecl;external External_library name 'curl_easy_getinfo';
function curl_easy_getinfo(curl:PCURL; info:CURLINFO):CURLcode;cdecl;external External_library name 'curl_easy_getinfo';
function curl_easy_duphandle(curl:PCURL):PCURL;cdecl;external External_library name 'curl_easy_duphandle';
procedure curl_easy_reset(curl:PCURL);cdecl;external External_library name 'curl_easy_reset';

function curl_multi_init:PCURLM;cdecl;external External_library name 'curl_multi_init';
function curl_multi_add_handle(multi_handle:PCURLM; curl_handle:PCURL):CURLMcode;cdecl;external External_library name 'curl_multi_add_handle';
function curl_multi_remove_handle(multi_handle:PCURLM; curl_handle:PCURL):CURLMcode;cdecl;external External_library name 'curl_multi_remove_handle';
function curl_multi_fdset(multi_handle:PCURLM; read_fd_set:Pfd_set; write_fd_set:Pfd_set; exc_fd_set:Pfd_set; max_fd:Plongint):CURLMcode;cdecl;external External_library name 'curl_multi_fdset';
function curl_multi_perform(multi_handle:PCURLM; running_handles:Plongint):CURLMcode;cdecl;external External_library name 'curl_multi_perform';
function curl_multi_cleanup(multi_handle:PCURLM):CURLMcode;cdecl;external External_library name 'curl_multi_cleanup';
function curl_multi_info_read(multi_handle:PCURLM; msgs_in_queue:Plongint):PCURLMsg;cdecl;external External_library name 'curl_multi_info_read';
function curl_multi_strerror(_para1:CURLMcode):Pchar;cdecl;external External_library name 'curl_multi_strerror';

function curl_multi_socket(multi_handle:PCURLM; s:curl_socket_t; running_handles:Plongint):CURLMcode;cdecl;external External_library name 'curl_multi_socket';
function curl_multi_socket_all(multi_handle:PCURLM; running_handles:Plongint):CURLMcode;cdecl;external External_library name 'curl_multi_socket_all';
function curl_multi_timeout(multi_handle:PCURLM; milliseconds:Plongint):CURLMcode;cdecl;external External_library name 'curl_multi_timeout';

function curl_multi_setopt(multi_handle:PCURLM; option:CURLMoption; args:array of const):CURLMcode;cdecl;external External_library name 'curl_multi_setopt';
function curl_multi_setopt(multi_handle:PCURLM; option:CURLMoption):CURLMcode;cdecl;external External_library name 'curl_multi_setopt';
function curl_multi_assign(multi_handle:PCURLM; sockfd:curl_socket_t; sockp:pointer):CURLMcode;cdecl;external External_library name 'curl_multi_assign';

implementation

end.
