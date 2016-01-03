{
 * @file httpd.h
 * @brief HTTP Daemon routines
}

{ XXX - We need to push more stuff to other .h files, or even .c files, to
 * make this file smaller
}

// Headers in which EVERYONE has an interest...
{$include ap_config.inc}
{$include ap_mmn.inc}

{$include ap_release.inc}

{#include "apr_general.h"}
{#include "apr_time.h"
#include "apr_network_io.h"}

{$ifdef windows}
  {$include apr\apr_buckets.inc}
{$else}
  {$include apr/apr_buckets.inc}
{$endif}

{#include "os.h"}

{$include ap_regex.inc}

// Note: util_uri.h is also included, see below

{ ----------------------------- config dir ------------------------------ }

{ Define this to be the default server home dir. Most things later in this
 * file with a relative pathname will have this added.
}
const
  {$ifdef OS2}
    { Set default for OS/2 file system }
    HTTPD_ROOT = '/os2httpd';
  {$else}{$ifdef WINDOWS}
    { Set default for Windows file system }
    HTTPD_ROOT = '/apache';
  {$else}{$ifdef BEOS}
    { Set the default for BeOS }
    HTTPD_ROOT = '/boot/home/apache';
  {$else}{$ifdef NETWARE}
    { Set the default for NetWare }
    HTTPD_ROOT = '/apache';
  {$else}
    { Set for all other OSes }
    HTTPD_ROOT = '/usr/local/apache';
  {$endif}
  {$endif}
  {$endif}
  {$endif}

{
 * --------- You shouldn't have to edit anything below this line ----------
 *
 * Any modifications to any defaults not defined above should be done in the
 * respective configuration file.
 *
}
const

{ Default location of documents.  Can be overridden by the DocumentRoot
 * directive.
}
  DOCUMENT_LOCATION = HTTPD_ROOT + '/htdocs';

// Maximum number of dynamically loaded modules
 DYNAMIC_MODULE_LIMIT = 128;

// Default administrator's address
 DEFAULT_ADMIN  = '[no address given]';

// The name of the log files
{$if defined(OS2) or defined(WINDOWS)}
  DEFAULT_ERRORLOG = 'logs/error.log';
{$else}
  DEFAULT_ERRORLOG = 'logs/error_log';
{$endif}

{ Define this to be what your per-directory security files are called }
  DEFAULT_ACCESS_FNAME = '.htaccess';

{ The name of the server config file }
  SERVER_CONFIG_FILE = 'conf/httpd.conf';

{ The default path for CGI scripts if none is currently set }
  DEFAULT_PATH = '/bin:/usr/bin:/usr/ucb:/usr/bsd:/usr/local/bin';

{ The path to the suExec wrapper, can be overridden in Configuration }
  SUEXEC_BIN  = HTTPD_ROOT + '/bin/suexec';

{ The timeout for waiting for messages }
  DEFAULT_TIMEOUT = 300 ;

{ The timeout for waiting for keepalive timeout until next request }
  DEFAULT_KEEPALIVE_TIMEOUT = 15;

{ The number of requests to entertain per connection }
  DEFAULT_KEEPALIVE = 100;

{ Limits on the size of various request items.  These limits primarily
 * exist to prevent simple denial-of-service attacks on a server based
 * on misuse of the protocol.  The recommended values will depend on the
 * nature of the server resources -- CGI scripts and database backends
 * might require large values, but most servers could get by with much
 * smaller limits than we use below.  The request message body size can
 * be limited by the per-dir config directive LimitRequestBody.
 *
 * Internal buffer sizes are two bytes more than the DEFAULT_LIMIT_REQUEST_LINE
 * and DEFAULT_LIMIT_REQUEST_FIELDSIZE below, which explains the 8190.
 * These two limits can be lowered (but not raised) by the server config
 * directives LimitRequestLine and LimitRequestFieldsize, respectively.
 *
 * DEFAULT_LIMIT_REQUEST_FIELDS can be modified or disabled (set = 0) by
 * the server config directive LimitRequestFields.
}
  DEFAULT_LIMIT_REQUEST_LINE = 8190;

  DEFAULT_LIMIT_REQUEST_FIELDSIZE = 8190;

  DEFAULT_LIMIT_REQUEST_FIELDS = 100;


{
 * The default default character set name to add if AddDefaultCharset is
 * enabled.  Overridden with AddDefaultCharsetName.
}
  DEFAULT_ADD_DEFAULT_CHARSET_NAME = 'iso-8859-1';

{ default HTTP Server protocol }
  AP_SERVER_PROTOCOL = 'HTTP/1.1';


{ ------------------ stuff that modules are allowed to look at ----------- }

  AP_DEFAULT_INDEX = 'index.html';


{
 * Define this to be what type you'd like returned for files with unknown
 * suffixes.
 * @warning MUST be all lower case.
}
  DEFAULT_CONTENT_TYPE = 'text/plain';

{ The name of the MIME types file }
  AP_TYPES_CONFIG_FILE = 'conf/mime.types';

{
 * Define the HTML doctype strings centrally.
}
{ HTML 2.0 Doctype }
  DOCTYPE_HTML_2_0 = '<!DOCTYPE HTML PUBLIC "-//IETF//' +
                          'DTD HTML 2.0//EN">' + LineEnding;
{ HTML 3.2 Doctype }
  DOCTYPE_HTML_3_2 = '<!DOCTYPE HTML PUBLIC "-//W3C//' +
                          'DTD HTML 3.2 Final//EN">' + LineEnding;
{ HTML 4.0 Strict Doctype }
  DOCTYPE_HTML_4_0S = '<!DOCTYPE HTML PUBLIC "-//W3C//' +
                          'DTD HTML 4.0//EN"' + LineEnding +
                          '"http://www.w3.org/TR/REC-html40/strict.dtd">' + LineEnding;
{ HTML 4.0 Transitional Doctype }
  DOCTYPE_HTML_4_0T = '<!DOCTYPE HTML PUBLIC "-//W3C//' +
                          'DTD HTML 4.0 Transitional//EN"' + LineEnding +
                          '"http://www.w3.org/TR/REC-html40/loose.dtd">' + LineEnding;
{ HTML 4.0 Frameset Doctype }
  DOCTYPE_HTML_4_0F = '<!DOCTYPE HTML PUBLIC "-//W3C//' +
                          'DTD HTML 4.0 Frameset//EN"' + LineEnding +
                          '"http://www.w3.org/TR/REC-html40/frameset.dtd">' + LineEnding;
{ XHTML 1.0 Strict Doctype }
  DOCTYPE_XHTML_1_0S = '<!DOCTYPE html PUBLIC "-//W3C//' +
                           'DTD XHTML 1.0 Strict//EN"' + LineEnding +
                           '"http://www.w3.org/TR/xhtml1/DTD/' +
                           'xhtml1-strict.dtd\">' + LineEnding;
{ XHTML 1.0 Transitional Doctype }
  DOCTYPE_XHTML_1_0T = '<!DOCTYPE html PUBLIC "-//W3C//' +
                           'DTD XHTML 1.0 Transitional//EN"' + LineEnding +
                           '"http://www.w3.org/TR/xhtml1/DTD/' +
                           'xhtml1-transitional.dtd">' + LineEnding;
{ XHTML 1.0 Frameset Doctype }
  DOCTYPE_XHTML_1_0F = '<!DOCTYPE html PUBLIC "-//W3C//' +
                           'DTD XHTML 1.0 Frameset//EN"' + LineEnding +
                           '"http://www.w3.org/TR/xhtml1/DTD/' +
                           'xhtml1-frameset.dtd">' + LineEnding;

{ Internal representation for a HTTP protocol number, e.g., HTTP/1.1 }

function HTTP_VERSION(major, minor: Integer): Integer;
{ Major part of HTTP protocol }
function HTTP_VERSION_MAJOR(number: Integer): Integer;
{ Minor part of HTTP protocol }
function HTTP_VERSION_MINOR(number: Integer): Integer;

{ -------------- Port number for server running standalone --------------- }

const
{ default HTTP Port }
  DEFAULT_HTTP_PORT =	80;
{ default HTTPS Port }
  DEFAULT_HTTPS_PORT =	443;
{
 * Check whether @a port is the default port for the request @a r.
 * @param port The port number
 * @param r The request
 * @see #ap_default_port
}
//#define ap_is_default_port(port,r)	((port) == ap_default_port(r))
{
 * Get the default port for a request (which depends on the scheme).
 * @param r The request
}
//#define ap_default_port(r)	ap_run_default_port(r)
{
 * Get the scheme for a request.
 * @param r The request
 * @bug This should be called ap_http_scheme!
}
//#define ap_http_scheme(r)	ap_run_http_scheme(r)

{ The default string lengths }
//  MAX_STRING_LEN = HUGE_STRING_LEN;
  HUGE_STRING_LEN = 8192;

{ The size of the server's internal read-write buffers }
  AP_IOBUFSIZE = 8192;

{ The max number of regex captures that can be expanded by ap_pregsub }
  AP_MAX_REG_MATCH = 10;

{
 * APR_HAS_LARGE_FILES introduces the problem of spliting sendfile into
 * mutiple buckets, no greater than MAX(apr_size_t), and more granular
 * than that in case the brigade code/filters attempt to read it directly.
 * ### 16mb is an invention, no idea if it is reasonable.
 }
  AP_MAX_SENDFILE  = 16777216;  { 2^24 }

{
 * Special Apache error codes. These are basically used
 *  in http_main.c so we can keep track of various errors.
 *
 }
{ a normal exit }
  APEXIT_OK =		$0;
{ A fatal error arising during the server's init sequence }
  APEXIT_INIT =		$2;
{  The child died during its init sequence }
  APEXIT_CHILDINIT =	$3;
{
 *   The child exited due to a resource shortage.
 *   The parent should limit the rate of forking until
 *   the situation is resolved.
}
  APEXIT_CHILDSICK =      $7;
{
 *     A fatal error, resulting in the whole server aborting.
 *     If a child exits with this error, the parent process
 *     considers this a server-wide fatal error and aborts.
}
  APEXIT_CHILDFATAL =	$f;

{
 * Stuff marked #AP_DECLARE is part of the API, and intended for use
 * by modules. Its purpose is to allow us to add attributes that
 * particular platforms or compilers require to every exported function.
}
// define AP_DECLARE(type)    type

{
 * Stuff marked #AP_DECLARE_NONSTD is part of the API, and intended for
 * use by modules.  The difference between #AP_DECLARE and
 * #AP_DECLARE_NONSTD is that the latter is required for any functions
 * which use varargs or are used via indirect function call.  This
 * is to accomodate the two calling conventions in windows dlls.
}
//# define AP_DECLARE_NONSTD(type)    type

{$define AP_DECLARE_DATA}


//# define AP_MODULE_DECLARE(type)    type
//# define AP_MODULE_DECLARE_NONSTD(type)  type

{$define AP_MODULE_DECLARE_DATA}

{
 * @internal
 * modules should not used functions marked AP_CORE_DECLARE
}
//  AP_CORE_DECLARE =	AP_DECLARE;

{
 * @internal
 * modules should not used functions marked AP_CORE_DECLARE_NONSTD
}

//  AP_CORE_DECLARE_NONSTD =	AP_DECLARE_NONSTD;

{
 * The numeric version information is broken out into fields within this
 * structure.
 }
type
  ap_version_t = record
    major: Integer;              {< major number }
    minor: Integer;              {< minor number }
    patch: Integer;              {< patch number }
    add_string: PChar; {< additional string like "-dev" }
  end;
  
  Pap_version_t = ^ap_version_t;

{
 * Return httpd's version information in a numeric form.
 *
 *  @param version Pointer to a version structure for returning the version
 *                 information.
 }
procedure ap_get_server_revision(version: Pap_version_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_server_revision' + LibSuff4;

{
 * Get the server version string
 * @return The server version string
 }
function ap_get_server_version: PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_server_version' + LibSuff0;

{
 * Add a component to the version string
 * @param pconf The pool to allocate the component from
 * @param component The string to add
}
procedure ap_add_version_component(pconf: Papr_pool_t; const component: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_add_version_component' + LibSuff8;

{
 * Get the date a time that the server was built
 * @return The server build time string
}
function ap_get_server_built: PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_server_built' + LibSuff0;

const
  DECLINED = -1;		{< Module declines to handle }
  DONE = -2;			{< Module has served the response completely
				 *  - it's safe to die() with no more output
				 }
  OK = 0;			{< Module has handled this stage. }


{
 * @defgroup HTTP_Status HTTP Status Codes
 * @


 * The size of the static array in http_protocol.c for storing
 * all of the potential response status-lines (a sparse table).
 * A future version should dynamically generate the apr_table_t at startup.
}
  RESPONSE_CODES = 57;

  HTTP_CONTINUE =                      100;
  HTTP_SWITCHING_PROTOCOLS =           101;
  HTTP_PROCESSING =                    102;
  HTTP_OK =                            200;
  HTTP_CREATED =                       201;
  HTTP_ACCEPTED =                      202;
  HTTP_NON_AUTHORITATIVE =             203;
  HTTP_NO_CONTENT =                    204;
  HTTP_RESET_CONTENT =                 205;
  HTTP_PARTIAL_CONTENT =               206;
  HTTP_MULTI_STATUS =                  207;
  HTTP_MULTIPLE_CHOICES =              300;
  HTTP_MOVED_PERMANENTLY =             301;
  HTTP_MOVED_TEMPORARILY =             302;
  HTTP_SEE_OTHER =                     303;
  HTTP_NOT_MODIFIED =                  304;
  HTTP_USE_PROXY =                     305;
  HTTP_TEMPORARY_REDIRECT =            307;
  HTTP_BAD_REQUEST =                   400;
  HTTP_UNAUTHORIZED =                  401;
  HTTP_PAYMENT_REQUIRED =              402;
  HTTP_FORBIDDEN =                     403;
  HTTP_NOT_FOUND =                     404;
  HTTP_METHOD_NOT_ALLOWED =            405;
  HTTP_NOT_ACCEPTABLE =                406;
  HTTP_PROXY_AUTHENTICATION_REQUIRED = 407;
  HTTP_REQUEST_TIME_OUT =              408;
  HTTP_CONFLICT =                      409;
  HTTP_GONE =                          410;
  HTTP_LENGTH_REQUIRED =               411;
  HTTP_PRECONDITION_FAILED =           412;
  HTTP_REQUEST_ENTITY_TOO_LARGE =      413;
  HTTP_REQUEST_URI_TOO_LARGE =         414;
  HTTP_UNSUPPORTED_MEDIA_TYPE =        415;
  HTTP_RANGE_NOT_SATISFIABLE =         416;
  HTTP_EXPECTATION_FAILED =            417;
  HTTP_UNPROCESSABLE_ENTITY =          422;
  HTTP_LOCKED =                        423;
  HTTP_FAILED_DEPENDENCY =             424;
  HTTP_UPGRADE_REQUIRED =              426;
  HTTP_INTERNAL_SERVER_ERROR =         500;
  HTTP_NOT_IMPLEMENTED =               501;
  HTTP_BAD_GATEWAY =                   502;
  HTTP_SERVICE_UNAVAILABLE =           503;
  HTTP_GATEWAY_TIME_OUT =              504;
  HTTP_VERSION_NOT_SUPPORTED =         505;
  HTTP_VARIANT_ALSO_VARIES =           506;
  _INSUFFICIENT_STORAGE =              507;
  HTTP_NOT_EXTENDED =                  510;

{ is the status code informational }
//#define ap_is_HTTP_INFO(x)         (((x) >= 100)&&((x) < 200))
{ is the status code OK ?}
//#define ap_is_HTTP_SUCCESS(x)      (((x) >= 200)&&((x) < 300))
{ is the status code a redirect }
//#define ap_is_HTTP_REDIRECT(x)     (((x) >= 300)&&((x) < 400))
{ is the status code a error (client or server) }
//#define ap_is_HTTP_ERROR(x)        (((x) >= 400)&&((x) < 600))
{ is the status code a client error  }
//#define ap_is_HTTP_CLIENT_ERROR(x) (((x) >= 400)&&((x) < 500))
{ is the status code a server error  }
//#define ap_is_HTTP_SERVER_ERROR(x) (((x) >= 500)&&((x) < 600))

{ should the status code drop the connection }
{#define ap_status_drops_connection(x) \
                                   (((x) == HTTP_BAD_REQUEST)           || \
                                    ((x) == HTTP_REQUEST_TIME_OUT)      || \
                                    ((x) == HTTP_LENGTH_REQUIRED)       || \
                                    ((x) == HTTP_REQUEST_ENTITY_TOO_LARGE) || \
                                    ((x) == HTTP_REQUEST_URI_TOO_LARGE) || \
                                    ((x) == HTTP_INTERNAL_SERVER_ERROR) || \
                                    ((x) == HTTP_SERVICE_UNAVAILABLE) || \
				    ((x) == HTTP_NOT_IMPLEMENTED))}
{
 * @defgroup Methods List of Methods recognized by the server
 * @


 * Methods recognized (but not necessarily handled) by the server.
 * These constants are used in bit shifting masks of size int, so it is
 * unsafe to have more methods than bits in an int.  HEAD == M_GET.
 * This list must be tracked by the list in http_protocol.c in routine
 * ap_method_name_of().
}
const
  M_GET =                   0;       // RFC 2616: HTTP
  M_PUT =                   1;       //  :
  M_POST =                  2;
  M_DELETE =                3;
  M_CONNECT =               4;
  M_OPTIONS =               5;
  M_TRACE =                 6;       // RFC 2616: HTTP }
  M_PATCH =                 7;       // no rfc(!)  ### remove this one? }
  M_PROPFIND =              8;       // RFC 2518: WebDAV }
  M_PROPPATCH =             9;       //  :               }
  M_MKCOL =                 10;
  M_COPY =                  11;
  M_MOVE =                  12;
  M_LOCK =                  13;
  M_UNLOCK =                14;      // RFC 2518: WebDAV }
  M_VERSION_CONTROL =       15;      // RFC 3253: WebDAV Versioning }
  M_CHECKOUT =              16;      //  :                          }
  M_UNCHECKOUT =            17;
  M_CHECKIN =               18;
  M_UPDATE =                19;
  M_LABEL =                 20;
  M_REPORT =                21;
  M_MKWORKSPACE =           22;
  M_MKACTIVITY =            23;
  M_BASELINE_CONTROL =      24;
  M_MERGE =                 25;
  M_INVALID =               26;      // RFC 3253: WebDAV Versioning }

{
 * METHODS needs to be equal to the number of bits
 * we are using for limit masks.
}
  METHODS = 64;

{
 * The method mask bit to shift for anding with a bitmask.
}
  AP_METHOD_BIT = apr_int64_t(1);


{
 * Structure for handling HTTP methods.  Methods known to the server are
 * accessed via a bitmask shortcut; extension methods are handled by
 * an array.
}
type
  ap_method_list_t = record
    { The bitmask used for known methods }
    method_mask: apr_int64_t;
    { the array used for extension methods }
//    method_list: ^apr_array_header_t;
  end;
{
 * @defgroup module_magic Module Magic mime types
 * @
 }
{ Magic for mod_cgi[d] }
const
  CGI_MAGIC_TYPE = 'application/x-httpd-cgi';
{ Magic for mod_include }
  INCLUDES_MAGIC_TYPE = 'text/x-server-parsed-html';
{ Magic for mod_include }
  INCLUDES_MAGIC_TYPE3 = 'text/x-server-parsed-html3';
{ Magic for mod_dir }
  DIR_MAGIC_TYPE = 'httpd/unix-directory';

{ Just in case your linefeed isn't the one the other end is expecting. }
{ linefeed }
  LF = 10;
{ carrige return }
  CR = 13;
{ carrige return /Line Feed Combo }
  CRLF = #015#012;

{
 * @defgroup values_request_rec_body Possible values for request_rec.read_body
 * @
 * Possible values for request_rec.read_body (set by handling module):
 }

{ Send 413 error if message has any body }
  REQUEST_NO_BODY =         0;
{ Send 411 error if body without Content-Length }
  REQUEST_CHUNKED_ERROR =   1;
{ If chunked, remove the chunks for me. }
  REQUEST_CHUNKED_DECHUNK = 2;


{
 * @defgroup values_request_rec_used_path_info Possible values for request_rec.used_path_info
 * @
 * Possible values for request_rec.used_path_info:
 }

{ Accept the path_info from the request }
  AP_REQ_ACCEPT_PATH_INFO =   0;
{ Return a 404 error if path_info was given }
  AP_REQ_REJECT_PATH_INFO =   1;
{ Module may chose to use the given path_info }
  AP_REQ_DEFAULT_PATH_INFO =  2;
{ @}

{
 * Things which may vary per file-lookup WITHIN a request ---
 * e.g., state of MIME config.  Basically, the name of an object, info
 * about the object, and any other info we may ahve which may need to
 * change as we go poking around looking for it (e.g., overridden by
 * .htaccess files).
 *
 * Note how the default state of almost all these things is properly
 * zero, so that allocating it with pcalloc does the right thing without
 * a whole lot of hairy initialization... so long as we are willing to
 * make the (fairly) portable assumption that the bit pattern of a NULL
 * pointer is, in fact, zero.
}

{
 * This represents the result of calling htaccess; these are cached for
 * each request.
}
type
  Phtaccess_result = ^htaccess_result;
  htaccess_result = record
    { the directory to which this applies }
    dir: PChar;
    { the overrides allowed for the .htaccess file }
    override_: Integer;
    { the configuration directives }
    htaccess: Pap_conf_vector_t;
    { the next one, or NULL if no more; N.B. never change this }
    next: Phtaccess_result;
  end;

{ The following four types define a hierarchy of activities, so that
 * given a request_rec r you can write r->connection->server->process
 * to get to the process_rec.  While this reduces substantially the
 * number of arguments that various hooks require beware that in
 * threaded versions of the server you must consider multiplexing
 * issues.  }


{ ### would be nice to not include this from httpd.h ... }
{ This comes after we have defined the request_rec type }
//#include "apr_uri.h"

type
  { Forward declarations of pointer to record types}
  Pconn_rec = ^conn_rec;
  Prequest_rec = ^request_rec;
  Pserver_rec = ^server_rec;
  PPserver_rec = ^Pserver_rec;
  Pserver_addr_rec = ^server_addr_rec;
  Pprocess_rec = ^process_rec;

  { A structure that represents one process }
  process_rec = record
    { Global pool. Cleared upon normal exit }
    pool: Papr_pool_t;
    { Configuration pool. Cleared upon restart }
    pconf: Papr_pool_t;
    { Number of command line arguments passed to the program }
    argc: Integer;
    { The command line arguments }
    argv: PChar;
    { The program name used to execute the program }
    short_name: PChar;
  end;

  { A structure that represents the current request }
  request_rec = record
    { The pool associated with the request }
    pool: Papr_pool_t;
    { The connection to the client }
    connection: Pconn_rec;
    { The virtual host for this request }
    server: Pserver_rec;

    { Pointer to the redirected request if this is an external redirect }
    next: Prequest_rec;
    { Pointer to the previous request if this is an internal redirect }
    prev: Prequest_rec;

    { Pointer to the main request if this is a sub-request
     * (see http_request.h) }
    main: Prequest_rec;

    { Info about the request itself... we begin with stuff that only
     * protocol.c should ever touch...
    }
    { First line of request }
    the_request: PChar;
    { HTTP/0.9, "simple" request (e.g. GET /foo\n w/no headers) }
    assbackwards: Integer;
    { A proxy request (calculated during post_read_request/translate_name)
     *  possible values PROXYREQ_NONE, PROXYREQ_PROXY, PROXYREQ_REVERSE,
     *                  PROXYREQ_RESPONSE
     }
    proxyreq: Integer;
    { HEAD request, as opposed to GET }
    header_only: Integer;
    { Protocol string, as given to us, or HTTP/0.9 }
    protocol: PChar;
    { Protocol version number of protocol; 1.1 = 1001 }
    proto_num: Integer;
    { Host, as set by full URI or Host: }
    hostname: PChar;

    { Time when the request started }
    request_time: apr_time_t;

    { Status line, if set by script }
    status_line: PChar;
    { Status line }
    status: Integer;

    { Request method, two ways; also, protocol, etc..  Outside of protocol.c,
     * look, but don't touch.
     }

    { Request method (eg. GET, HEAD, POST, etc.) }
    method: PChar;
    { M_GET, M_POST, etc. }
    method_number: Integer;

    {
     *  'allowed' is a bitvector of the allowed methods.
     *
     *  A handler must ensure that the request method is one that
     *  it is capable of handling.  Generally modules should DECLINE
     *  any request methods they do not handle.  Prior to aborting the
     *  handler like this the handler should set r->allowed to the list
     *  of methods that it is willing to handle.  This bitvector is used
     *  to construct the "Allow:" header required for OPTIONS requests,
     *  and HTTP_METHOD_NOT_ALLOWED and HTTP_NOT_IMPLEMENTED status codes.
     *
     *  Since the default_handler deals with OPTIONS, all modules can
     *  usually decline to deal with OPTIONS.  TRACE is always allowed,
     *  modules don't need to set it explicitly.
     *
     *  Since the default_handler will always handle a GET, a
     *  module which does *not* implement GET should probably return
     *  HTTP_METHOD_NOT_ALLOWED.  Unfortunately this means that a Script GET
     *  handler can't be installed by mod_actions.
     }
    allowed: apr_int64_t;
    { Array of extension methods }
    allowed_xmethods: Papr_array_header_t;
    { List of allowed methods }
    allowed_methods: Pap_method_list_t;

    { byte count in stream is for body }
    sent_bodyct: apr_off_t;
    { body byte count, for easy access }
    bytes_sent: apr_off_t;
    { Last modified time of the requested resource }
    mtime: apr_time_t;

    { HTTP/1.1 connection-level features }

    { sending chunked transfer-coding }
    chunked: Integer;
    { The Range: header }
    range: PChar;
    { The "real" content length }
    clength: apr_off_t;

    { Remaining bytes left to read from the request body }
    remaining: apr_off_t;
    { Number of bytes that have been read  from the request body }
    read_length: apr_off_t;
    { Method for reading the request body
     * (eg. REQUEST_CHUNKED_ERROR, REQUEST_NO_BODY,
     *  REQUEST_CHUNKED_DECHUNK, etc...) }
    read_body: Integer;
    { reading chunked transfer-coding }
    read_chunked: Integer;
    { is client waiting for a 100 response? }
    expecting_100: Cardinal;

    { MIME header environments, in and out.  Also, an array containing
     * environment variables to be passed to subprocesses, so people can
     * write modules to add to that environment.
     *
     * The difference between headers_out and err_headers_out is that the
     * latter are printed even on error, and persist across internal redirects
     * (so the headers printed for ErrorDocument handlers will have them).
     *
     * The 'notes' apr_table_t is for notes from one module to another, with no
     * other set purpose in mind...
     }

    { MIME header environment from the request }
    headers_in: Papr_table_t;
    { MIME header environment for the response }
    headers_out: Papr_table_t;
    { MIME header environment for the response, printed even on errors and
     * persist across internal redirects }
    err_headers_out: Papr_table_t;
    { Array of environment variables to be used for sub processes }
    subprocess_env: Papr_table_t;
    { Notes from one module to another }
    notes: Papr_table_t;

    { content_type, handler, content_encoding, and all content_languages
     * MUST be lowercased strings.  They may be pointers to static strings;
     * they should not be modified in place.
     }
    { The content-type for the current request }
    content_type: PChar;	// Break these out --- we dispatch on 'em
    { The handler string that we use to call a handler function }
    handler: PChar;	// What we *really* dispatch on

    { How to encode the data }
    content_encoding: PChar;
    { Array of strings representing the content languages }
    content_languages: Papr_array_header_t;

    { variant list validator (if negotiated) }
    vlist_validator: PChar;

    { If an authentication check was made, this gets set to the user name. }
    user: PChar;
    { If an authentication check was made, this gets set to the auth type. }
    ap_auth_type: PChar;

    { This response can not be cached }
    no_cache: Integer;
    { There is no local copy of this response }
    no_local_copy: Integer;

    { What object is being requested (either directly, or via include
     * or content-negotiation mapping).
     }

    { The URI without any parsing performed }
    unparsed_uri: PChar;
    { The path portion of the URI }
    uri: PChar;
    { The filename on disk corresponding to this response }
    filename: PChar;
    { XXX: What does this mean? Please define "canonicalize" -aaron }
    { The true filename, we canonicalize r->filename if these don't match }
    canonical_filename: PChar;
    { The PATH_INFO extracted from this request }
    path_info: PChar;
    { The QUERY_ARGS extracted from this request }
    args: PChar;
    {  finfo.protection (st_mode) set to zero if no such file }
    finfo: apr_finfo_t;
    { A struct containing the components of URI }
    parsed_uri: apr_uri_t;

    {
     * Flag for the handler to accept or reject path_info on
     * the current request.  All modules should respect the
     * AP_REQ_ACCEPT_PATH_INFO and AP_REQ_REJECT_PATH_INFO
     * values, while AP_REQ_DEFAULT_PATH_INFO indicates they
     * may follow existing conventions.  This is set to the
     * user's preference upon HOOK_VERY_FIRST of the fixups.
     }
    used_path_info: Integer;

    { Various other config info which may change with .htaccess files
     * These are config vectors, with one void* pointer for each module
     * (the thing pointed to being the module's business).
     }

    { Options set in config files, etc.}
    per_dir_config: Pap_conf_vector_t;
    { Notes on *this* request }
    request_config: Pap_conf_vector_t;

    {
     * A linked list of the .htaccess configuration directives
     * accessed by this request.
     * N.B. always add to the head of the list, _never_ to the end.
     * that way, a sub request's list can (temporarily) point to a parent's list
     }
    htaccess: Phtaccess_result;

    { A list of output filters to be used for this request }
    output_filters: Pap_filter_t;
    { A list of input filters to be used for this request }
    input_filters: Pap_filter_t;

    { A list of protocol level output filters to be used for this
     *  request }
    proto_output_filters: Pap_filter_t;
    { A list of protocol level input filters to be used for this
     *  request }
    proto_input_filters: Pap_filter_t;

    { A flag to determine if the eos bucket has been sent yet }
    eos_sent: Integer;

{ Things placed at the end of the record to avoid breaking binary
 * compatibility.  It would be nice to remember to reorder the entire
 * record to improve 64bit alignment the next time we need to break
 * binary compatibility for some other reason.
}
  end;

{
 * @defgroup ProxyReq Proxy request types
 *
 * Possible values of request_rec->proxyreq. A request could be normal,
 *  proxied or reverse proxied. Normally proxied and reverse proxied are
 *  grouped together as just "proxied", but sometimes it's necessary to
 *  tell the difference between the two, such as for authentication.
}

  ap_conn_keepalive_e = (AP_CONN_UNKNOWN, AP_CONN_CLOSE, AP_CONN_KEEPALIVE);

  Pconn_state_t = ^conn_state_t;

{ Structure to store things which are per connection }
  conn_rec = record
    { Pool associated with this connection }
    pool: Papr_pool_t;
    { Physical vhost this conn came in on }
    base_server: Pserver_rec;
    { used by http_vhost.c }
    vhost_lookup_data: Pointer;

    { Information about the connection itself }
    { local address }
    local_addr: Papr_sockaddr_t;
    { remote address }
    remote_addr: Papr_sockaddr_t;

    { Client's IP address }
    remote_ip: PChar;
    { Client's DNS name, if known.  NULL if DNS hasn't been checked,
     *  "" if it has and no address was found.  N.B. Only access this though
     * get_remote_host() }
    remote_host: PChar;
    { Only ever set if doing rfc1413 lookups.  N.B. Only access this through
     *  get_remote_logname() }
    remote_logname: PChar;

    { Are we still talking? }
    flags: Cardinal;

    (* Useless bitset variables to make the code obscure

     Are we still talking?
    unsigned aborted:1;

     Are we going to keep the connection alive for another request?
     * @see ap_conn_keepalive_e
    signed int keepalive:2;

     have we done double-reverse DNS? -1 yes/failure, 0 not yet,
     *  1 yes/success
    signed int double_reverse:2;
    *)

    { How many times have we used it? }
    keepalives: Integer;
    { server IP address }
    local_ip: PChar;
    { used for ap_get_server_name when UseCanonicalName is set to DNS
     *  (ignores setting of HostnameLookups) }
    local_host: PChar;

    { ID of this connection; unique at any point in time }
    id: clong;
    { Config vector containing pointers to connections per-server
     *  config structures. }
    conn_config: Pap_conf_vector_t;
    { Notes on *this* connection: send note from one module to
     *  another. must remain valid for all requests on this conn }
    notes: Papr_table_t;
    { A list of input filters to be used for this connection }
    input_filters: Pap_filter_t;
    { A list of output filters to be used for this connection }
    output_filters: Pap_filter_t;
    { handle to scoreboard information for this connection }
    sbh: Pointer;
    { The bucket allocator to use for all bucket/brigade creations }
    bucket_alloc: Papr_bucket_alloc_t;
    { The current state of this connection }
    cs: Pconn_state_t;
    { Is there data pending in the input filters? }
    data_in_input_filters: cint;
  end;

{
 * Enumeration of connection states 
 }
  conn_state_e = (
    CONN_STATE_CHECK_REQUEST_LINE_READABLE,
    CONN_STATE_READ_REQUEST_LINE,
    CONN_STATE_LINGER
  );

{
 * @brief A structure to contain connection state information 
 }
  APR_RING_ENTRY_conn_state_t = record
    next: Pconn_state_t;
    prev: Pconn_state_t;
  end;
 
  conn_state_t = record
    { APR_RING of expiration timeouts }
    timeout_list: APR_RING_ENTRY_conn_state_t;
    { the expiration time of the next keepalive timeout }
    expiration_time: apr_time_t;
    { Current state of the connection }
    state: conn_state_e;
    { connection record this struct refers to }
    c: Pconn_rec;
    { memory pool to allocate from }
    p: Papr_pool_t;
    { bucket allocator }
    bucket_alloc: Papr_bucket_alloc_t;
    { poll file decriptor information }
    pfd: apr_pollfd_t;
  end;


{ Per-vhost config... }

{ A structure to be used for Per-vhost config }
  server_addr_rec = record
    { The next server in the list }
    next: Pserver_addr_rec;
    { The bound address, for this server }
    host_addr: Papr_sockaddr_t;
    { The bound port, for this server }
    host_port: apr_port_t;
    { The name given in <VirtualHost> }
    virthost: PChar;
  end;

  { A structure to store information for each virtual server }
  server_rec = record
    { The process this server is running in }
    process: Pprocess_rec;
    { The next server in the list }
    next: Pserver_rec;

    { The name of the server }
    defn_name: PChar;
    { The line of the config file that the server was defined on }
    defn_line_number: Integer;

    { Contact information }

    { The admin's contact information }
    server_admin: PChar;
    { The server hostname }
    server_hostname: PChar;
    { for redirects, etc. }
    port: apr_port_t;

    { Log files --- note that transfer log is now in the modules... }

    { The name of the error log }
    error_fname: PChar;
    { A file descriptor that references the error log }
    error_log: Papr_file_t;
    { The log level for this server }
    loglevel: Integer;

    { Module-specific configuration for server, and defaults... }

    { true if this is the virtual server }
    is_virtual: Integer;
    { Config vector containing pointers to modules' per-server config
     *  structures. }
    module_config: Pap_conf_vector_t;
    { MIME type info, etc., before we start checking per-directory info }
    lookup_defaults: Pap_conf_vector_t;

    { Transaction handling }

    { I haven't got a clue }
    addrs: Pserver_addr_rec;
    { Timeout, as an apr interval, before we give up }
    timeout: apr_interval_time_t;
    { The apr interval we will wait for another request }
    keep_alive_timeout: apr_interval_time_t;
    { Maximum requests per connection }
    keep_alive_max: Integer;
    { Use persistent connections? }
    keep_alive: Integer;

    { Pathname for ServerPath }
    path: PChar;
    { Length of path }
    pathlen: Integer;

    { Normal names for ServerAlias servers }
    names: Papr_array_header_t;
    { Wildcarded names for ServerAlias servers }
    wild_names: Papr_array_header_t;

    { limit on size of the HTTP request line    }
    limit_req_line: Integer;
    { limit on size of any request header field }
    limit_req_fieldsize: Integer;
    { limit on number of request header fields  }
    limit_req_fields: Integer;

    { The server request scheme for redirect responses }
    server_scheme: PChar;
  end;

type
  core_output_filter_ctx = record
    b: Papr_bucket_brigade;
    deferred_write_pool: Papr_pool_t; { subpool of c->pool used for resources
                                      * which may outlive the request         }
  end;

  core_filter_ctx = record
    b: Papr_bucket_brigade;
    tmpbb: Papr_bucket_brigade;
  end; // core_ctx_t
  
  core_ctx_t = core_filter_ctx;

  Pcore_ctx_t = ^core_ctx_t;

type
  core_net_rec = record
    { Connection to the client }
    client_socket: Papr_socket_t;

    { connection record }
    c: Pconn_rec;

    out_ctx: Pcore_output_filter_ctx_t;
    in_ctx: Pcore_ctx_t;
  end; // core_net_rec;

  Pcore_net_rec = ^core_net_rec;

{
  The constants are on the bottom because the structures need to be on the same type block
}
const
  PROXYREQ_NONE = 0;		{< No proxy }
  PROXYREQ_PROXY = 1;	{< Standard proxy }
  PROXYREQ_REVERSE = 2;	{< Reverse proxy }
  PROXYREQ_RESPONSE = 3; {< Origin response }

{
 * The address 255.255.255.255, when used as a virtualhost address,
 * will become the "default" server when the ip doesn't match other vhosts.
}
const DEFAULT_VHOST_ADDR = $ffffffff;//ul

{
 * Examine a field value (such as a media-/content-type) string and return
 * it sans any parameters; e.g., strip off any ';charset=foo' and the like.
 * @param p Pool to allocate memory from
 * @param intype The field to examine
 * @return A copy of the field minus any parameters
}
function ap_field_noparam(p: Papr_pool_t; const intype: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_field_noparam' + LibSuff8;

{
 * Convert a time from an integer into a string in a specified format
 * @param p The pool to allocate memory from
 * @param t The time to convert
 * @param fmt The format to use for the conversion
 * @param gmt Convert the time for GMT?
 * @return The string that represents the specified time
}
function ap_ht_time(p: Papr_pool_t; t: apr_time_t; const fmt: PChar; gmt: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_ht_time' + LibSuff20;

{ String handling. The *_nc variants allow you to use non-const char **s as
   arguments (unfortunately C won't automatically convert a char ** to a const
   char **) }

{
 * Get the characters until the first occurance of a specified character
 * @param p The pool to allocate memory from
 * @param line The string to get the characters from
 * @param stop The character to stop at
 * @return A copy of the characters up to the first stop character
}
function ap_getword(p: Papr_pool_t; const line: PPChar; stop: Char): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword' + LibSuff12;

{
 * Get the characters until the first occurance of a specified character
 * @param p The pool to allocate memory from
 * @param line The string to get the characters from
 * @param stop The character to stop at
 * @return A copy of the characters up to the first stop character
 * @note This is the same as ap_getword(), except it doesn't use const char **.
}
function ap_getword_nc(p: Papr_pool_t; const line: PPChar; stop: Char): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_nc' + LibSuff12;

{
 * Get the first word from a given string.  A word is defined as all characters
 * up to the first whitespace.
 * @param p The pool to allocate memory from
 * @param line The string to traverse
 * @return The first word in the line
}
function ap_getword_white(p: Papr_pool_t; const line: PPChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_white' + LibSuff8;

{
 * Get the first word from a given string.  A word is defined as all characters
 * up to the first whitespace.
 * @param p The pool to allocate memory from
 * @param line The string to traverse
 * @return The first word in the line
 * @note The same as ap_getword_white(), except it doesn't use const char **.
}
function ap_getword_white_nc(p: Papr_pool_t; const line: PPChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_white_nc' + LibSuff8;

{
 * Get all characters from the first occurance of @a stop to the first '\0'
 * @param p The pool to allocate memory from
 * @param line The line to traverse
 * @param stop The character to start at
 * @return A copy of all caracters after the first occurance of the specified
 *         character
}
function ap_getword_nulls(p: Papr_pool_t; const line: PPChar; stop: Char): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_nulls' + LibSuff12;

{
 * Get all characters from the first occurance of @a stop to the first '\0'
 * @param p The pool to allocate memory from
 * @param line The line to traverse
 * @param stop The character to start at
 * @return A copy of all caracters after the first occurance of the specified
 *         character
 * @note The same as ap_getword_nulls(), except it doesn't use const char **.
}
function ap_getword_nulls_nc(p: Papr_pool_t; const line: PPChar; stop: Char): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_nulls_nc' + LibSuff12;

{
 * Get the second word in the string paying attention to quoting
 * @param p The pool to allocate from
 * @param line The line to traverse
 * @return A copy of the string
}
function ap_getword_conf(p: Papr_pool_t; const line: PPChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_conf' + LibSuff8;

{
 * Get the second word in the string paying attention to quoting
 * @param p The pool to allocate from
 * @param line The line to traverse
 * @return A copy of the string
 * @note The same as ap_getword_conf(), except it doesn't use const char **.
}
function ap_getword_conf_nc(p: Papr_pool_t; const line: PPChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getword_conf_nc' + LibSuff8;

{
 * Check a string for any $ENV environment variable construct and replace
 * each them by the value of that environment variable, if it exists. If the
 * environment value does not exist, leave the $ENV construct alone; it
 * means something else.
 * @param p The pool to allocate from
 * @param word The string to check
 * @return The string with the replaced environment variables
}
function ap_resolve_env(p: Papr_pool_t; const word_: PChar; accept_white: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_resolve_env' + LibSuff8;

{
 * Size an HTTP header field list item, as separated by a comma.
 * @param field The field to size
 * @param len The length of the field
 * @return The return value is a pointer to the beginning of the non-empty
 * list item within the original string (or NULL if there is none) and the
 * address of field is shifted to the next non-comma, non-whitespace
 * character.  len is the length of the item excluding any beginning whitespace.
}
function ap_size_list_item(const field: PPChar; len: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_size_list_item' + LibSuff8;

{
 * Retrieve an HTTP header field list item, as separated by a comma,
 * while stripping insignificant whitespace and lowercasing anything not in
 * a quoted string or comment.
 * @param p The pool to allocate from
 * @param field The field to retrieve
 * @return The return value is a new string containing the converted list
 *         item (or NULL if none) and the address pointed to by field is
 *         shifted to the next non-comma, non-whitespace.
}
function ap_get_list_item(p: Papr_pool_t; const field: PPChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_list_item' + LibSuff8;

{
 * Find an item in canonical form (lowercase, no extra spaces) within
 * an HTTP field value list.
 * @param p The pool to allocate from
 * @param line The field value list to search
 * @param tok The token to search for
 * @return 1 if found, 0 if not found.
}
function ap_find_list_item(p: Papr_pool_t; const line, tok: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_find_list_item' + LibSuff12;

{
 * Retrieve a token, spacing over it and returning a pointer to
 * the first non-white byte afterwards.  Note that these tokens
 * are delimited by semis and commas and can also be delimited
 * by whitespace at the caller's option.
 * @param p The pool to allocate from
 * @param accept_line The line to retrieve the token from
 * @param accept_white Is it delimited by whitespace
 * @return the first non-white byte after the token
}
function ap_get_token(p: Papr_pool_t; const accept_line: PPChar; accept_white: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_token' + LibSuff12;

{
 * Find http tokens, see the definition of token from RFC2068
 * @param p The pool to allocate from
 * @param line The line to find the token
 * @param tok The token to find
 * @return 1 if the token is found, 0 otherwise
}
function ap_find_token(p: Papr_pool_t; const line, tok: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_find_token' + LibSuff12;

{
 * find http tokens from the end of the line
 * @param p The pool to allocate from
 * @param line The line to find the token
 * @param tok The token to find
 * @return 1 if the token is found, 0 otherwise
}
function ap_find_last_token(p: Papr_pool_t; const line, tok: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_find_last_token' + LibSuff12;

{
 * Check for an Absolute URI syntax
 * @param u The string to check
 * @return 1 if URI, 0 otherwise
}
function ap_is_url(const u: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_is_url' + LibSuff4;

{
 * Unescape a URL
 * @param url The url to unescape
 * @return 0 on success, non-zero otherwise
}
function ap_unescape_url(url: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_unescape_url' + LibSuff4;

{
 * Unescape a URL, but leaving %2f (slashes) escaped
 * @param url The url to unescape
 * @return 0 on success, non-zero otherwise
}
function ap_unescape_url_keep2f(url: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_unescape_url_keep2f' + LibSuff4;

{
 * Convert all double slashes to single slashes
 * @param name The string to convert
}
procedure ap_no2slash(name: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_no2slash' + LibSuff4;

{
 * Remove all ./ and xx/../ substrings from a file name. Also remove
 * any leading ../ or /../ substrings.
 * @param name the file name to parse
}
procedure ap_getparents(name: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getparents' + LibSuff4;

{
 * Escape a path segment, as defined in RFC 1808
 * @param p The pool to allocate from
 * @param s The path to convert
 * @return The converted URL
}
function ap_escape_path_segment(p: Papr_pool_t; const s: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_escape_path_segment' + LibSuff8;

{
 * convert an OS path to a URL in an OS dependant way.
 * @param p The pool to allocate from
 * @param path The path to convert
 * @param partial if set, assume that the path will be appended to something
 *        with a '/' in it (and thus does not prefix "./")
 * @return The converted URL
}
function ap_os_escape_path(p: Papr_pool_t; const path: PChar; partial: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_os_escape_path' + LibSuff12;

{ @see ap_os_escape_path }
function ap_escape_uri(p: Papr_pool_t; const path: PChar): PChar;

{
 * Escape an html string
 * @param p The pool to allocate from
 * @param s The html to escape
 * @return The escaped string
 }
function ap_escape_html(p: Papr_pool_t; const s: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_escape_html' + LibSuff8;

{
 * Escape a string for logging
 * @param p The pool to allocate from
 * @param str The string to escape
 * @return The escaped string
 }
function ap_escape_logitem(p: Papr_pool_t; const str: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_escape_logitem' + LibSuff8;

{
 * Escape a string for logging into the error log (without a pool)
 * @param dest The buffer to write to
 * @param source The string to escape
 * @param buflen The buffer size for the escaped string (including \0)
 * @return The len of the escaped string (always < maxlen)
 }
function ap_escape_errorlog_item(dest, source: PChar;
 buflen: apr_size_t): apr_size_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_escape_errorlog_item' + LibSuff12;

{
 * Construct a full hostname
 * @param p The pool to allocate from
 * @param hostname The hostname of the server
 * @param port The port the server is running on
 * @param r The current request
 * @return The server's hostname
 }
function ap_construct_server(p: Papr_pool_t; const hostname: PChar;
 port: Papr_port_t; const r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_construct_server' + LibSuff16;

{
 * Escape a shell command
 * @param p The pool to allocate from
 * @param s The command to escape
 * @return The escaped shell command
 }
function ap_escape_shell_cmd(p: Papr_pool_t; const s: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_escape_shell_cmd' + LibSuff8;

{
 * Count the number of directories in a path
 * @param path The path to count
 * @return The number of directories
 }
function ap_count_dirs(const path: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_count_dirs' + LibSuff4;

{
 * Copy at most @a n leading directories of @a s into @a d. @a d
 * should be at least as large as @a s plus 1 extra byte
 *
 * @param d The location to copy to
 * @param s The location to copy from
 * @param n The number of directories to copy
 * @return value is the ever useful pointer to the trailing \0 of d
 * @note on platforms with drive letters, n = 0 returns the "/" root,
 * whereas n = 1 returns the "d:/" root.  On all other platforms, n = 0
 * returns the empty string.  }
function ap_make_dirstr_prefix(const d, s: PChar; n: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_make_dirstr_prefix' + LibSuff12;

{
 * Return the parent directory name (including trailing /) of the file
 * @a s
 * @param p The pool to allocate from
 * @param s The file to get the parent of
 * @return A copy of the file's parent directory
 }
function ap_make_dirstr_parent(p: Papr_pool_t; const s: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_make_dirstr_parent' + LibSuff8;

{
 * Given a directory and filename, create a single path from them.  This
 * function is smart enough to ensure that there is a sinlge '/' between the
 * directory and file names
 * @param a The pool to allocate from
 * @param dir The directory name
 * @param f The filename
 * @return A copy of the full path
 * @tip Never consider using this function if you are dealing with filesystem
 * names that need to remain canonical, unless you are merging an apr_dir_read
 * path and returned filename.  Otherwise, the result is not canonical.
 }
function ap_make_full_path(a: Papr_pool_t; const dir, f: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_make_full_path' + LibSuff12;

{
 * Test if the given path has an an absolute path.
 * @param p The pool to allocate from
 * @param dir The directory name
 * @tip The converse is not necessarily true, some OS's (Win32/OS2/Netware) have
 * multiple forms of absolute paths.  This only reports if the path is absolute
 * in a canonical sense.
 }
function ap_os_is_path_absolute(p: Papr_pool_t; const dir: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_os_is_path_absolute' + LibSuff8;

{
 * Does the provided string contain wildcard characters?  This is useful
 * for determining if the string should be passed to strcmp_match or to strcmp.
 * The only wildcard characters recognized are '?' and '*'
 * @param str The string to check
 * @return 1 if the string has wildcards, 0 otherwise
 }
function ap_is_matchexp(const str: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_is_matchexp' + LibSuff4;

{
 * Determine if a string matches a patterm containing the wildcards '?' or '*'
 * @param str The string to check
 * @param expected The pattern to match against
 * @return 1 if the two strings match, 0 otherwise
 }
function ap_strcmp_match(const str, expected: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_strcmp_match' + LibSuff8;

{
 * Determine if a string matches a patterm containing the wildcards '?' or '*',
 * ignoring case
 * @param str The string to check
 * @param expected The pattern to match against
 * @return 1 if the two strings match, 0 otherwise
 }
function ap_strcasecmp_match(const str, expected: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_strcasecmp_match' + LibSuff8;

{
 * Find the first occurrence of the substring s2 in s1, regardless of case
 * @param s1 The string to search
 * @param s2 The substring to search for
 * @return A pointer to the beginning of the substring
 * @remark See apr_strmatch() for a faster alternative
 }
function ap_strcasestr(const s1, s2: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_strcasestr' + LibSuff8;

{
 * Return a pointer to the location inside of bigstring immediately after prefix
 * @param bigstring The input string
 * @param prefix The prefix to strip away
 * @return A pointer relative to bigstring after prefix
 }
function ap_stripprefix(const bigstring, prefix: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_stripprefix' + LibSuff8;

{
 * Decode a base64 encoded string into memory allocated from a pool
 * @param p The pool to allocate from
 * @param bufcoded The encoded string
 * @return The decoded string
 }
function ap_pbase64decode(p: Papr_pool_t; const bufcoded: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_pbase64decode' + LibSuff8;

{
 * Encode a string into memory allocated from a pool in base 64 format
 * @param p The pool to allocate from
 * @param strin The plaintext string
 * @return The encoded string
 }
function ap_pbase64encode(p: Papr_pool_t; string_: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_pbase64encode' + LibSuff8;

{
 * Compile a regular expression to be used later
 * @param p The pool to allocate from
 * @param pattern the regular expression to compile
 * @param cflags The bitwise or of one or more of the following:
 *   @li #REG_EXTENDED - Use POSIX extended Regular Expressions
 *   @li #REG_ICASE    - Ignore case
 *   @li #REG_NOSUB    - Support for substring addressing of matches
 *       not required
 *   @li #REG_NEWLINE  - Match-any-character operators don't match new-line
 * @return The compiled regular expression
 }
function ap_pregcomp(p: Papr_pool_t; const pattern: PChar; cflags: Integer): Pap_regex_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_pregcomp' + LibSuff12;

{
 * Free the memory associated with a compiled regular expression
 * @param p The pool the regex was allocated from
 * @param reg The regular expression to free
 }
procedure ap_pregfree(p: Papr_pool_t; reg: Pap_regex_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_pregfree' + LibSuff8;

{
 * After performing a successful regex match, you may use this function to
 * perform a series of string substitutions based on subexpressions that were
 * matched during the call to ap_regexec
 * @param p The pool to allocate from
 * @param input An arbitrary string containing $1 through $9.  These are
 *              replaced with the corresponding matched sub-expressions
 * @param source The string that was originally matched to the regex
 * @param nmatch the nmatch returned from ap_pregex
 * @param pmatch the pmatch array returned from ap_pregex
}
function ap_pregsub(p: Papr_pool_t; const input, source: PChar;
 nmatch: size_t; pmatch: array of ap_regmatch_t): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_pregsub' + LibSuff20;

{
 * We want to downcase the type/subtype for comparison purposes
 * but nothing else because ;parameter=foo values are case sensitive.
 * @param s The content-type to convert to lowercase
}
procedure ap_content_type_tolower(s: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_content_type_tolower' + LibSuff4;

{
 * convert a string to all lowercase
 * @param s The string to convert to lowercase
}
procedure ap_str_tolower(s: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_str_tolower' + LibSuff4;

{
 * Search a string from left to right for the first occurrence of a
 * specific character
 * @param str The string to search
 * @param c The character to search for
 * @return The index of the first occurrence of c in str
}
function ap_ind(str: PChar; c: Char): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_ind' + LibSuff8;

{
 * Search a string from right to left for the first occurrence of a
 * specific character
 * @param str The string to search
 * @param c The character to search for
 * @return The index of the first occurrence of c in str
}
function ap_rind(str: PChar; c: Char): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_rind' + LibSuff8;

{
 * Given a string, replace any bare " with \" .
 * @param p The pool to allocate memory from
 * @param instring The string to search for "
 * @return A copy of the string with escaped quotes
}
function ap_escape_quotes(p: Papr_pool_t; const instring: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_escape_quotes' + LibSuff8;

{
 * Given a string, append the PID deliminated by delim.
 * Usually used to create a pid-appended filepath name
 * (eg: /a/b/foo -> /a/b/foo.6726). A function, and not
 * a macro, to avoid unistd.h dependency
 * @param p The pool to allocate memory from
 * @param string The string to append the PID to
 * @param delim The string to use to deliminate the string from the PID
 * @return A copy of the string with the PID appended 
 }
function ap_append_pid(p: Papr_pool_t; const string_, delim: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_append_pid' + LibSuff12;

{ Misc system hackery }
{
 * Given the name of an object in the file system determine if it is a directory
 * @param p The pool to allocate from
 * @param name The name of the object to check
 * @return 1 if it is a directory, 0 otherwise
}
function ap_is_rdirectory(p: Papr_pool_t; const name: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_is_rdirectory' + LibSuff8;

{
 * Given the name of an object in the file system determine if it is a directory - this version is symlink aware
 * @param p The pool to allocate from
 * @param name The name of the object to check
 * @return 1 if it is a directory, 0 otherwise
}
function ap_is_directory(p: Papr_pool_t; const name: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_is_directory' + LibSuff8;

{#ifdef _OSD_POSIX
extern int os_init_job_environment(server_rec *s, const char *user_name, int one_process);
#endif} { _OSD_POSIX }

{
 * Determine the local host name for the current machine
 * @param p The pool to allocate from
 * @return A copy of the local host name
}
//char *ap_get_local_host(apr_pool_t *p);

{
 * Log an assertion to the error log
 * @param szExp The assertion that failed
 * @param szFile The file the assertion is in
 * @param nLine The line the assertion is defined on
}
procedure ap_log_assert(szExp, szFile: PChar; nLine: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_log_assert' + LibSuff12;

{ @internal }
//#define ap_assert(exp) ((exp) ? (void)0 : ap_log_assert(#exp,__FILE__,__LINE__))

{
 * Redefine assert() to something more useful for an Apache...
 *
 * Use ap_assert() if the condition should always be checked.
 * Use AP_DEBUG_ASSERT() if the condition should only be checked when AP_DEBUG
 * is defined.
}

{#ifdef AP_DEBUG
#define AP_DEBUG_ASSERT(exp) ap_assert(exp)
#else
#define AP_DEBUG_ASSERT(exp) ((void)0)
#endif}

{
 * @defgroup stopsignal flags which indicate places where the sever should stop for debugging.
 * @
 * A set of flags which indicate places where the server should raise(SIGSTOP).
 * This is useful for debugging, because you can then attach to that process
 * with gdb and continue.  This is important in cases where one_process
 * debugging isn't possible.
}
{ stop on a Detach }
//  SIGSTOP_DETACH = 1;
{ stop making a child process }
//  SIGSTOP_MAKE_CHILD = 2;
{ stop spawning a child process }
//  SIGSTOP_SPAWN_CHILD = 4;
{ stop spawning a child process with a piped log }
//  SIGSTOP_PIPED_LOG_SPAWN = 8;
{ stop spawning a CGI child process }
//  SIGSTOP_CGI_CHILD = 16;

{ Macro to get GDB started }
{#ifdef DEBUG_SIGSTOP
extern int raise_sigstop_flags;
#define RAISE_SIGSTOP(x)	do begin
	if (raise_sigstop_flags & SIGSTOP_##x) raise(SIGSTOP);\
    end while (0)
#else
#define RAISE_SIGSTOP(x)
#endif }

{
 * Get HTML describing the address and (optionally) admin of the server.
 * @param prefix Text which is prepended to the return value
 * @param r The request_rec
 * @return HTML describing the server, allocated in @a r's pool.
 }
function ap_psignature(prefix: PChar; r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_psignature' + LibSuff8;

{const
  AP_NORESTART = APR_OS_START_USEERR + 1;}

