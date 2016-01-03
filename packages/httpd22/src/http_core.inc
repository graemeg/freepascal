{ Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 }

//#include "apr.h"
{$include apr/apr_hash.inc}
{#include "apr_optional.h"}
{$include util_filter.inc}

{#if APR_HAVE_STRUCT_RLIMIT
#include <sys/time.h>
#include <sys/resource.h>
#endif}

{
 * @package CORE HTTP Daemon
 }

{ ****************************************************************
 *
 * The most basic server code is encapsulated in a single module
 * known as the core, which is just *barely* functional enough to
 * serve documents, though not terribly well.
 *
 * Largely for NCSA back-compatibility reasons, the core needs to
 * make pieces of its config structures available to other modules.
 * The accessors are declared here, along with the interpretation
 * of one of them (allow_options).
 }

const
{ No directives }
  OPT_NONE = 0;
{ Indexes directive }
  OPT_INDEXES = 1;
{  Includes directive }
  OPT_INCLUDES = 2;
{  FollowSymLinks directive }
  OPT_SYM_LINKS = 4;
{  ExecCGI directive }
  OPT_EXECCGI = 8;
{  directive unset }
  OPT_UNSET = 16;
{  IncludesNOEXEC directive }
  OPT_INCNOEXEC = 32;
{ SymLinksIfOwnerMatch directive }
  OPT_SYM_OWNER = 64;
{ MultiViews directive }
  OPT_MULTI = 128;
{  All directives }
  OPT_ALL = (OPT_INDEXES or OPT_INCLUDES or OPT_SYM_LINKS or OPT_EXECCGI);

{
 * @defgroup get_remote_host Remote Host Resolution
 * @ingroup APACHE_CORE_HTTPD
 }
{ REMOTE_HOST returns the hostname, or NULL if the hostname
 * lookup fails.  It will force a DNS lookup according to the
 * HostnameLookups setting.
 }
  REMOTE_HOST = (0);

{ REMOTE_NAME returns the hostname, or the dotted quad if the
 * hostname lookup fails.  It will force a DNS lookup according
 * to the HostnameLookups setting.
 }
  REMOTE_NAME = (1);

{ REMOTE_NOLOOKUP is like REMOTE_NAME except that a DNS lookup is
 * never forced.
 }
  REMOTE_NOLOOKUP = (2);

{ REMOTE_DOUBLE_REV will always force a DNS lookup, and also force
 * a double reverse lookup, regardless of the HostnameLookups
 * setting.  The result is the (double reverse checked) hostname,
 * or NULL if any of the lookups fail.
 }
  REMOTE_DOUBLE_REV = (3);

{ all of the requirements must be met }
  SATISFY_ALL = 0;
{  any of the requirements must be met }
  SATISFY_ANY = 1;
{ There are no applicable satisfy lines }
  SATISFY_NOSPEC = 2;

{ Make sure we don't write less than 8000 bytes at any one time.
 }
  AP_MIN_BYTES_TO_WRITE  = 8000;

{ default maximum of internal redirects }
  AP_DEFAULT_MAX_INTERNAL_REDIRECTS = 10;

{ default maximum subrequest nesting level }
  AP_DEFAULT_MAX_SUBREQ_DEPTH = 10;

{
 * Retrieve the value of Options for this request
 * @param r The current request
 * @return the Options bitmask
 * @deffunc int ap_allow_options(request_rec *r)
 }
function ap_allow_options(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_allow_options' + LibSuff4;

{
 * Retrieve the value of the AllowOverride for this request
 * @param r The current request
 * @return the overrides bitmask
 * @deffunc int ap_allow_overrides(request_rec *r)
 }
function ap_allow_overrides(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_allow_overrides' + LibSuff4;

{
 * Retrieve the value of the DefaultType directive, or text/plain if not set
 * @param r The current request
 * @return The default type
 * @deffunc const char *ap_default_type(request_rec *r)
 }
function ap_default_type(r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_default_type' + LibSuff4;

{
 * Retrieve the document root for this server
 * @param r The current request
 * @warning Don't use this!  If your request went through a Userdir, or 
 * something like that, it'll screw you.  But it's back-compatible...
 * @return The document root
 * @deffunc const char *ap_document_root(request_rec *r)
 }
function ap_document_root(r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_document_root' + LibSuff4;

{
 * Lookup the remote client's DNS name or IP address
 * @param conn The current connection
 * @param dir_config The directory config vector from the request
 * @param type The type of lookup to perform.  One of:
 * <pre>
 *     REMOTE_HOST returns the hostname, or NULL if the hostname
 *                 lookup fails.  It will force a DNS lookup according to the
 *                 HostnameLookups setting.
 *     REMOTE_NAME returns the hostname, or the dotted quad if the
 *                 hostname lookup fails.  It will force a DNS lookup according
 *                 to the HostnameLookups setting.
 *     REMOTE_NOLOOKUP is like REMOTE_NAME except that a DNS lookup is
 *                     never forced.
 *     REMOTE_DOUBLE_REV will always force a DNS lookup, and also force
 *                   a double reverse lookup, regardless of the HostnameLookups
 *                   setting.  The result is the (double reverse checked) 
 *                   hostname, or NULL if any of the lookups fail.
 * </pre>
 * @param str_is_ip unless NULL is passed, this will be set to non-zero on output when an IP address 
 *        string is returned
 * @return The remote hostname
 * @deffunc const char *ap_get_remote_host(conn_rec *conn, void *dir_config, int type, int *str_is_ip)
 }
function ap_get_remote_host(conn: Pconn_rec; dir_config: Pointer;
  _type: Integer; str_is_ip: PInteger): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_remote_host' + LibSuff16;

{
 * Retrieve the login name of the remote user.  Undef if it could not be
 * determined
 * @param r The current request
 * @return The user logged in to the client machine
 * @deffunc const char *ap_get_remote_logname(request_rec *r)
 }
function ap_get_remote_logname(r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_remote_logname' + LibSuff4;

{ Used for constructing self-referencing URLs, and things like SERVER_PORT,
 * and SERVER_NAME.
 }
{
 * build a fully qualified URL from the uri and information in the request rec
 * @param p The pool to allocate the URL from
 * @param uri The path to the requested file
 * @param r The current request
 * @return A fully qualified URL
 * @deffunc char *ap_construct_url(apr_pool_t *p, const char *uri, request_rec *r)
 }
function ap_construct_url(p: Papr_pool_t; const uri: PChar; r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_construct_url' + LibSuff12;

{
 * Get the current server name from the request
 * @param r The current request
 * @return the server name
 * @deffunc const char *ap_get_server_name(request_rec *r)
 }
function ap_get_server_name(r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_server_name' + LibSuff4;

{
 * Get the current server port
 * @param The current request
 * @return The server's port
 * @deffunc apr_port_t ap_get_server_port(const request_rec *r)
 }
function ap_get_server_port(r: Prequest_rec): apr_port_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_server_port' + LibSuff4;

{
 * Return the limit on bytes in request msg body 
 * @param r The current request
 * @return the maximum number of bytes in the request msg body
 * @deffunc apr_off_t ap_get_limit_req_body(const request_rec *r)
 }
function ap_get_limit_req_body(r: Prequest_rec): apr_off_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_limit_req_body' + LibSuff4;

{
 * Return the limit on bytes in XML request msg body
 * @param r The current request
 * @return the maximum number of bytes in XML request msg body
 * @deffunc size_t ap_get_limit_xml_body(const request_rec *r)
 }
function ap_get_limit_xml_body(r: Prequest_rec): size_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_limit_xml_body' + LibSuff4;

{
 * Install a custom response handler for a given status
 * @param r The current request
 * @param status The status for which the custom response should be used
 * @param string The custom response.  This can be a static string, a file
 *               or a URL
 }
procedure ap_custom_response(r: Prequest_rec; status: Integer; const str: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_custom_response' + LibSuff12;

{
 * Check if the current request is beyond the configured max. number of redirects or subrequests
 * @param r The current request
 * @return true (is exceeded) or false
 * @deffunc int ap_is_recursion_limit_exceeded(const request_rec *r)
 }
function ap_is_recursion_limit_exceeded(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_is_recursion_limit_exceeded' + LibSuff4;

{
 * Check for a definition from the server command line
 * @param name The define to check for
 * @return 1 if defined, 0 otherwise
 * @deffunc int ap_exists_config_define(const char *name)
 }
function ap_exists_config_define(name: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_exists_config_define' + LibSuff4;

{ FIXME! See STATUS about how }
function ap_core_translate(r: Prequest_rec): Integer;
 cdecl; external LibHTTPD name 'ap_core_translate';

{ Authentication stuff.  This is one of the places where compatibility
 * with the old config files *really* hurts; they don't discriminate at
 * all between different authentication schemes, meaning that we need
 * to maintain common state for all of them in the core, and make it
 * available to the other modules through interfaces.
 }

{ A structure to keep track of authorization requirements }
type
  require_line = record
    { Where the require line is in the config file. }
    method_mask: apr_int64_t;
    { The complete string from the command line }
    requirement: PChar;
  end;
     
{
 * Return the type of authorization required for this request
 * @param r The current request
 * @return The authorization required
 * @deffunc const char *ap_auth_type(request_rec *r)
 }
function ap_auth_type(r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_auth_type' + LibSuff4;

{
 * Return the current Authorization realm
 * @param r The current request
 * @return The current authorization realm
 * @deffunc const char *ap_auth_name(request_rec *r)
 }
function ap_auth_name(r: Prequest_rec): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_auth_name' + LibSuff4;

{
 * How the requires lines must be met.
 * @param r The current request
 * @return How the requirements must be met.  One of:
 * <pre>
 *      SATISFY_ANY    -- any of the requirements must be met.
 *      SATISFY_ALL    -- all of the requirements must be met.
 *      SATISFY_NOSPEC -- There are no applicable satisfy lines
 * </pre>
 * @deffunc int ap_satisfies(request_rec *r)
 }
function ap_satisfies(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_satisfies' + LibSuff4;

{
 * Retrieve information about all of the requires directives for this request
 * @param r The current request
 * @return An array of all requires directives for this request
 * @deffunc const apr_array_header_t *ap_requires(request_rec *r)
 }
function ap_requires(p: Papr_array_header_t): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_requires' + LibSuff4;

//#ifdef CORE_PRIVATE

{
 * Core is also unlike other modules in being implemented in more than
 * one file... so, data structures are declared here, even though most of
 * the code that cares really is in http_core.c.  Also, another accessor.
 }

//AP_DECLARE_DATA extern module core_module;

{ Per-request configuration }

type
  core_request_config = record
    { bucket brigade used by getline for look-ahead and
     * ap_get_client_block for holding left-over request body }
    bb: Papr_bucket_brigade;

    { an array of per-request working data elements, accessed
     * by ID using ap_get_request_note()
     * (Use ap_register_request_note() during initialization
     * to add elements)
     }
    notes: PPointer;

    { There is a script processor installed on the output filter chain,
     * so it needs the default_handler to deliver a (script) file into
     * the chain so it can process it. Normally, default_handler only
     * serves files on a GET request (assuming the file is actual content),
     * since other methods are not content-retrieval. This flag overrides
     * that behavior, stating that the "content" is actually a script and
     * won't actually be delivered as the response for the non-GET method.
     }
    deliver_script: Integer;

    { Custom response strings registered via ap_custom_response(),
     * or NULL; check per-dir config if nothing found here
     }
    response_code_strings: PPChar; { from ap_custom_response(), not from
                                   * ErrorDocument
                                   }
    { Should addition of charset= be suppressed for this request?
     }
    suppress_charset: Integer;
  end;

{ Standard entries that are guaranteed to be accessible via
 * ap_get_request_note() for each request (additional entries
 * can be added with ap_register_request_note())
 }
const
  AP_NOTE_DIRECTORY_WALK = 0;
  AP_NOTE_LOCATION_WALK =  1;
  AP_NOTE_FILE_WALK =      2;
  AP_NUM_STD_NOTES =       3;

{
 * Reserve an element in the core_request_config->notes array
 * for some application-specific data
 * @return An integer key that can be passed to ap_get_request_note()
 *         during request processing to access this element for the
 *         current request.
 }
function ap_register_request_note: apr_size_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_register_request_note' + LibSuff0;

{
 * Retrieve a pointer to an element in the core_request_config->notes array
 * @param r The request
 * @param note_num  A key for the element: either a value obtained from
 *        ap_register_request_note() or one of the predefined AP_NOTE_*
 *        values.
 * @return NULL if the note_num is invalid, otherwise a pointer to the
 *         requested note element.
 * @remark At the start of a request, each note element is NULL.  The
 *         handle provided by ap_get_request_note() is a pointer-to-pointer
 *         so that the caller can point the element to some app-specific
 *         data structure.  The caller should guarantee that any such
 *         structure will last as long as the request itself.
 }
function ap_get_request_note(r: Prequest_rec; note_num: apr_size_t): PPointer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_request_note' + LibSuff8;

{ Per-directory configuration }

type
  allow_options_t = cuchar;
  overrides_t = cuchar;

{
 * Bits of info that go into making an ETag for a file
 * document.  Why a long?  Because char historically
 * proved too short for Options, and int can be different
 * sizes on different platforms.
 }
  etag_components_t = culong;

const
  ETAG_UNSET = 0;
  ETAG_NONE  = (1 shl 0);
  ETAG_MTIME = (1 shl 1);
  ETAG_INODE = (1 shl 2);
  ETAG_SIZE  = (1 shl 3);
  ETAG_BACKWARD = (ETAG_MTIME or ETAG_INODE or ETAG_SIZE);
  ETAG_ALL   = (ETAG_MTIME or ETAG_INODE or ETAG_SIZE);

  { Hostname resolution etc }
  
  HOSTNAME_LOOKUP_OFF = 0;
  HOSTNAME_LOOKUP_ON = 1;
  HOSTNAME_LOOKUP_DOUBLE = 2;
  OSTNAME_LOOKUP_UNSET = 3;

  { Hostname resolution etc }

  USE_CANONICAL_NAME_OFF   = (0);
  USE_CANONICAL_NAME_ON    = (1);
  USE_CANONICAL_NAME_DNS   = (2);
  USE_CANONICAL_NAME_UNSET = (3);

  { should we force a charset on any outgoing parameterless content-type?
   * if so, which charset?
   }
  ADD_DEFAULT_CHARSET_OFF   = (0);
  ADD_DEFAULT_CHARSET_ON    = (1);
  ADD_DEFAULT_CHARSET_UNSET = (2);

  {
   * Run-time performance tuning
   }
  ENABLE_MMAP_OFF   = (0);
  ENABLE_MMAP_ON    = (1);
  ENABLE_MMAP_UNSET = (2);

  ENABLE_SENDFILE_OFF   = (0);
  ENABLE_SENDFILE_ON    = (1);
  ENABLE_SENDFILE_UNSET = (2);

  USE_CANONICAL_PHYS_PORT_OFF   = (0);
  USE_CANONICAL_PHYS_PORT_ON    = (1);
  USE_CANONICAL_PHYS_PORT_UNSET = (2);

type
  server_signature_e = (
    srv_sig_unset,
    srv_sig_off,
    srv_sig_on,
    srv_sig_withmail
  );

  core_dir_config = record

    { path of the directory/regex/etc. see also d_is_fnmatch/absolute below }
    d: PChar;
    { the number of slashes in d }
    d_components: Cardinal;

    { If (opts & OPT_UNSET) then no absolute assignment to options has
     * been made.
     * invariant: (opts_add & opts_remove) == 0
     * Which said another way means that the last relative (options + or -)
     * assignment made to each bit is recorded in exactly one of opts_add
     * or opts_remove.
     }
    opts: allow_options_t;
    opts_add: allow_options_t;
    opts_remove: allow_options_t;
    override_: overrides_t;
    override_opts: allow_options_t;
    
    { MIME typing --- the core doesn't do anything at all with this,
     * but it does know what to slap on a request for a document which
     * goes untyped by other mechanisms before it slips out the door...
     }
    
    ap_default_type: PChar;
  
    { Authentication stuff.  Groan... }
    
    satisfy: PInteger; { for every method one }
    ap_auth_type: PChar;
    ap_auth_name: PChar;
    ap_requires: Papr_array_header_t;

    { Custom response config. These can contain text or a URL to redirect to.
     * if response_code_strings is NULL then there are none in the config,
     * if it's not null then it's allocated to sizeof(char*)*RESPONSE_CODES.
     * This lets us do quick merges in merge_core_dir_configs().
     }
  
    response_code_strings: PPChar; { from ErrorDocument, not from
                                   * ap_custom_response() }

    { Hostname resolution etc }
{    unsigned int hostname_lookups : 4; }

{    signed int content_md5 : 2;  }{ calculate Content-MD5? }

{    unsigned use_canonical_name : 2; }

    { since is_fnmatch(conf->d) was being called so frequently in
     * directory_walk() and its relatives, this field was created and
     * is set to the result of that call.
     }
{    unsigned d_is_fnmatch : 1; }

    { should we force a charset on any outgoing parameterless content-type?
     * if so, which charset?
     }
{    unsigned add_default_charset : 2; }
    add_default_charset_name: PChar;

    { System Resource Control }
{$ifdef RLIMIT_CPU}
    limit_cpu: Prlimit;
{$endif}
{$if defined(RLIMIT_DATA) or defined (RLIMIT_VMEM) or defined(RLIMIT_AS)}
    limit_mem: Prlimit;
{$endif}
{$ifdef RLIMIT_NPROC}
    limit_nproc: Prlimit;
{$endif}
    limit_req_body: apr_off_t;      { limit on bytes in request msg body }
    limit_xml_body: cLong;           { limit on bytes in XML request msg body }

    { logging options }

    server_signature: server_signature_e;

    loglevel: Integer;
    
    { Access control }
    sec_file: Papr_array_header_t;
    r: Pap_regex_t;

    mime_type: PChar;       { forced with ForceType  }
    handler: PChar;         { forced with SetHandler }
    output_filters: PChar;  { forced with SetOutputFilters }
    input_filters: PChar;   { forced with SetInputFilters }
    accept_path_info: Integer;        { forced with AcceptPathInfo }

    ct_output_filters: Papr_hash_t; { added with AddOutputFilterByType }

    {
     * What attributes/data should be included in ETag generation?
     }
    etag_bits: etag_components_t;
    etag_add: etag_components_t;
    etag_remove: etag_components_t;

    {
     * Run-time performance tuning
     }
{    unsigned int enable_mmap : 2;  }{ whether files in this dir can be mmap'ed }

{    unsigned int enable_sendfile : 2;  }{ files in this dir can be mmap'ed }
{    unsigned int allow_encoded_slashes : 1; }{ URLs may contain %2f w/o being
                                             * pitched indiscriminately }
{    unsigned use_canonical_phys_port : 2;}
  end;

{ Per-server core configuration }

const
  { TRACE control }

  AP_TRACE_UNSET    = -1;
  AP_TRACE_DISABLE  =  0;
  AP_TRACE_ENABLE   =  1;
  AP_TRACE_EXTENDED =  2;

type
   core_server_config = record
  
{$ifdef GPROF}
    gprof_dir: PChar;
{$endif}

    { Name translations --- we want the core to be able to do *something*
     * so it's at least a minimally functional web server on its own (and
     * can be tested that way).  But let's keep it to the bare minimum:
     }
    ap_document_root: PChar;
  
    { Access control }

    access_name: PChar;
    sec_dir: Papr_array_header_t;
    sec_url: Papr_array_header_t;

    { recursion backstopper }
    redirect_limit: Integer; { maximum number of internal redirects }
    subreq_limit: Integer;   { maximum nesting level of subrequests }

    protocol: PChar;
    accf_map: Papr_table_t;

    { TRACE control }
    trace_enable: Integer;
  end;

{ for AddOutputFiltersByType in core.c }
//void ap_add_output_filters_by_type(request_rec *r);

{ for http_config.c }
//void ap_core_reorder_directories(apr_pool_t *, server_rec *);

{ for mod_perl }
{AP_CORE_DECLARE(void) ap_add_per_dir_conf(server_rec *s, void *dir_config);
AP_CORE_DECLARE(void) ap_add_per_url_conf(server_rec *s, void *url_config);
AP_CORE_DECLARE(void) ap_add_file_conf(core_dir_config *conf, void *url_config);
AP_CORE_DECLARE_NONSTD(const char *) ap_limit_section(cmd_parms *cmd, void *dummy, const char *arg);}

{ Core filters; not exported. }
{int ap_core_input_filter(ap_filter_t *f, apr_bucket_brigade *b,
                         ap_input_mode_t mode, apr_read_type_e block,
                         apr_off_t readbytes);
apr_status_t ap_core_output_filter(ap_filter_t *f, apr_bucket_brigade *b);

#endif} { CORE_PRIVATE }

//AP_DECLARE(const char*) ap_get_server_protocol(server_rec* s);
//AP_DECLARE(void) ap_set_server_protocol(server_rec* s, const char* proto);


{ ----------------------------------------------------------------------
 *
 * Runtime status/management
 }

type
  ap_mgmt_type_e = (
    ap_mgmt_type_string,
    ap_mgmt_type_long,
    ap_mgmt_type_hash
  );

  ap_mgmt_value = record
    case Integer of
     0: (s_value: PChar);
     1: (i_value: cLong);
     2: (h_value: Papr_hash_t);
  end;

  ap_mgmt_item_t = record
    description: PChar;
    name: PChar;
    vtype: ap_mgmt_type_e;
    v: ap_mgmt_value;
  end;

{ Handles for core filters }
{extern AP_DECLARE_DATA ap_filter_rec_t *ap_subreq_core_filter_handle;
extern AP_DECLARE_DATA ap_filter_rec_t *ap_core_output_filter_handle;
extern AP_DECLARE_DATA ap_filter_rec_t *ap_content_length_filter_handle;
extern AP_DECLARE_DATA ap_filter_rec_t *ap_core_input_filter_handle;}

{
 * This hook provdes a way for modules to provide metrics/statistics about
 * their operational status.
 *
 * @param p A pool to use to create entries in the hash table
 * @param val The name of the parameter(s) that is wanted. This is
 *            tree-structured would be in the form ('*' is all the tree,
 *            'module.*' all of the module , 'module.foo.*', or
 *            'module.foo.bar' )
 * @param ht The hash table to store the results. Keys are item names, and
 *           the values point to ap_mgmt_item_t structures.
 * @ingroup hooks
 }
type
  ap_HOOK_get_mgmt_items_t = function(p: Papr_pool_t; const val: PChar;
    ht: Papr_hash_t): Integer; cdecl;

procedure ap_hook_get_mgmt_items(pf: ap_HOOK_get_mgmt_items_t;
 const aszPre: PPChar; const aszSucc: PPChar; nOrder: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_hook_get_mgmt_items' + LibSuff16;

{ ---------------------------------------------------------------------- }

{ ----------------------------------------------------------------------
 *
 * I/O logging with mod_logio
 }

{APR_DECLARE_OPTIONAL_FN(void, ap_logio_add_bytes_out,
                        (conn_rec *c, apr_off_t bytes));}

{ ----------------------------------------------------------------------
 *
 * ident lookups with mod_ident
 }

{APR_DECLARE_OPTIONAL_FN(const char *, ap_ident_lookup,
                        (request_rec *r));}

