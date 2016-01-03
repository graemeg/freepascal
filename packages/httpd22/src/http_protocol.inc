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

{#include "httpd.h"
#include "apr_hooks.h"
#include "apr_portable.h"
#include "apr_mmap.h"
#include "apr_buckets.h"
#include "util_filter.h"}

{
 * @package HTTP protocol handling
 }

{
 * This hook allows modules to insert filters for the current error response
 * @param r the current request
 * @ingroup hooks
 }
type
  ap_HOOK_insert_error_filter_t = procedure(r: Prequest_rec); cdecl;

procedure ap_hook_insert_error_filter(pf: ap_HOOK_insert_error_filter_t; const aszPre: PPChar;
 const aszSucc: PPChar; nOrder: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_hook_insert_error_filter' + LibSuff16;

{ This is an optimization.  We keep a record of the filter_rec that
 * stores the old_write filter, so that we can avoid strcmp's later.
 }
//AP_DECLARE_DATA extern ap_filter_rec_t *ap_old_write_func;

{
 * Prototypes for routines which either talk directly back to the user,
 * or control the ones that eventually do.
 }

{
 * Read a request and fill in the fields.
 * @param c The current connection
 * @return The new request_rec
 }
//request_rec *ap_read_request(conn_rec *c);

{
 * Read the mime-encoded headers.
 * @param r The current request
 }
procedure ap_get_mime_headers(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_mime_headers' + LibSuff4;

{
 * Optimized version of ap_get_mime_headers() that requires a
 * temporary brigade to work with
 * @param r The current request
 * @param bb temp brigade
 }
procedure ap_get_mime_headers_core(r: Prequest_rec; bb: Papr_bucket_brigade);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_mime_headers_core' + LibSuff8;

{ Finish up stuff after a request }

{
 * Called at completion of sending the response.  It sends the terminating
 * protocol information.
 * @param r The current request
 * @deffunc void ap_finalize_request_protocol(request_rec *r)
 }
procedure ap_finalize_request_protocol(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_finalize_request_protocol' + LibSuff4;

{
 * Send error back to client.
 * @param r The current request
 * @param recursive_error last arg indicates error status in case we get 
 *      an error in the process of trying to deal with an ErrorDocument 
 *      to handle some other error.  In that case, we print the default 
 *      report for the first thing that went wrong, and more briefly report 
 *      on the problem with the ErrorDocument.
 * @deffunc void ap_send_error_response(request_rec *r, int recursive_error)
 }
procedure ap_send_error_response(r: Prequest_rec; recursive_error: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_send_error_response' + LibSuff8;

{ Set last modified header line from the lastmod date of the associated file.
 * Also, set content length.
 *
 * May return an error status, typically HTTP_NOT_MODIFIED (that when the
 * permit_cache argument is set to one).
 }

{
 * Set the content length for this request
 * @param r The current request
 * @param length The new content length
 * @deffunc void ap_set_content_length(request_rec *r, apr_off_t length)
 }
procedure ap_set_content_length(r: Prequest_rec; length: apr_off_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_set_content_length' + LibSuff12;

{
 * Set the keepalive status for this request
 * @param r The current request
 * @return 1 if keepalive can be set, 0 otherwise
 * @deffunc int ap_set_keepalive(request_rec *r)
 }
function ap_set_keepalive(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_set_keepalive' + LibSuff4;

{
 * Return the latest rational time from a request/mtime pair.  Mtime is 
 * returned unless it's in the future, in which case we return the current time.
 * @param r The current request
 * @param mtime The last modified time
 * @return the latest rational time.
 * @deffunc apr_time_t ap_rationalize_mtime(request_rec *r, apr_time_t mtime)
 }
function ap_rationalize_mtime(r: Prequest_rec; mtime: apr_time_t): apr_time_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_rationalize_mtime' + LibSuff12;

{
 * Build the content-type that should be sent to the client from the
 * content-type specified.  The following rules are followed:
 *    - if type is NULL, type is set to ap_default_type(r)
 *    - if charset adding is disabled, stop processing and return type.
 *    - then, if there are no parameters on type, add the default charset
 *    - return type
 * @param r The current request
 * @return The content-type
 * @deffunc const char *ap_make_content_type(request_rec *r, const char *type);
 }
function ap_make_content_type(r: Prequest_rec; type_: PChar): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_make_content_type' + LibSuff8;

//#ifdef CORE_PRIVATE
{
 * Precompile metadata structures used by ap_make_content_type()
 * @param r The pool to use for allocations
 * @deffunc void ap_setup_make_content_type(apr_pool_t *pool)
 }
procedure ap_setup_make_content_type(pool: Papr_pool_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_setup_make_content_type' + LibSuff4;

//#endif { CORE_PRIVATE }

{
 * Construct an entity tag from the resource information.  If it's a real
 * file, build in some of the file characteristics.
 * @param r The current request
 * @param force_weak Force the entity tag to be weak - it could be modified
 *                   again in as short an interval.
 * @return The entity tag
 * @deffunc char *ap_make_etag(request_rec *r, int force_weak)
 }
function ap_make_etag(r: Prequest_rec; force_weak: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_make_etag' + LibSuff8;

{
 * Set the E-tag outgoing header
 * @param The current request
 * @deffunc void ap_set_etag(request_rec *r)
 }
procedure ap_set_etag(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_set_etag' + LibSuff4;

{
 * Set the last modified time for the file being sent
 * @param r The current request
 * @deffunc void ap_set_last_modified(request_rec *r)
 }
procedure ap_set_last_modified(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_set_last_modified' + LibSuff4;

{
 * Implements condition GET rules for HTTP/1.1 specification.  This function
 * inspects the client headers and determines if the response fulfills 
 * the requirements specified.
 * @param r The current request
 * @return OK if the response fulfills the condition GET rules, some
 *         other status code otherwise
 * @deffunc int ap_meets_conditions(request_rec *r)
 }
function ap_meets_conditions(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_meets_conditions' + LibSuff4;

{ Other ways to send stuff at the client.  All of these keep track
 * of bytes_sent automatically.  This indirection is intended to make
 * it a little more painless to slide things like HTTP-NG packetization
 * underneath the main body of the code later.  In the meantime, it lets
 * us centralize a bit of accounting (bytes_sent).
 *
 * These also return the number of bytes written by the call.
 * They should only be called with a timeout registered, for obvious reaasons.
 * (Ditto the send_header stuff).
 }

{
 * Send an entire file to the client, using sendfile if supported by the 
 * current platform
 * @param fd The file to send.
 * @param r The current request
 * @param offset Offset into the file to start sending.
 * @param length Amount of data to send
 * @param nbytes Amount of data actually sent
 * @deffunc apr_status_t ap_send_fd(apr_file_t *fd, request_rec *r, apr_off_t offset, apr_size_t length, apr_size_t *nbytes);
 }
function ap_send_fd(fd: Papr_file_t; r: Prequest_rec; offset: apr_off_t;
 length: apr_size_t; nbytes: Papr_size_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_send_fd' + LibSuff24;

{$ifdef APR_HAS_MMAP}
{
 * Send an MMAP'ed file to the client
 * @param mm The MMAP'ed file to send
 * @param r The current request
 * @param offset The offset into the MMAP to start sending
 * @param length The amount of data to send
 * @return The number of bytes sent
 * @deffunc size_t ap_send_mmap(apr_mmap_t *mm, request_rec *r, size_t offset, size_t length)
 }
function ap_send_mmap(mm: Papr_mmap_t; r: Prequest_rec; offset, length: size_t): size_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_send_mmap' + LibSuff20;

{$endif}


{
 * Register a new request method, and return the offset that will be
 * associated with that method.
 *
 * @param p        The pool to create registered method numbers from.
 * @param methname The name of the new method to register.
 * @return         Ab int value representing an offset into a bitmask.
 }
function ap_method_register(p: Papr_pool_t; methname: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_register' + LibSuff8;

{
 * Initialize the method_registry and allocate memory for it.
 *
 * @param p Pool to allocate memory for the registry from.
 }
procedure ap_method_registry_init(p: Papr_pool_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_registry_init' + LibSuff4;

{
 * This is a convenience macro to ease with checking a mask
 * against a method name.
 }
{#define AP_METHOD_CHECK_ALLOWED(mask, methname) \
    ((mask) & (AP_METHOD_BIT << ap_method_number_of((methname))))}

{
 * Create a new method list with the specified number of preallocated
 * slots for extension methods.
 *
 * @param   p       Pointer to a pool in which the structure should be
 *                  allocated.
 * @param   nelts   Number of preallocated extension slots
 * @return  Pointer to the newly created structure.
 * @deffunc ap_method_list_t ap_make_method_list(apr_pool_t *p, int nelts)
 }
function ap_make_method_list(p: Papr_pool_t; nelts: Integer): Pap_method_list_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_make_method_list' + LibSuff8;

{
 * Copy a method list
 *
 * @param   dest List to copy to
 * @param   src  List to copy from
 }
procedure ap_copy_method_list(dest, src: Pap_method_list_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_copy_method_list' + LibSuff8;

{
 * Search for an HTTP method name in an ap_method_list_t structure, and
 * return true if found.
 *
 * @param   method  String containing the name of the method to check.
 * @param   l       Pointer to a method list, such as cmd->methods_limited.
 * @return  1 if method is in the list, otherwise 0
 * @deffunc int ap_method_in_list(const char *method, ap_method_list_t *l)
 }
function ap_method_in_list(l: Pap_method_list_t; const method: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_in_list' + LibSuff8;

{
 * Add an HTTP method name to an ap_method_list_t structure if it isn't
 * already listed.
 *
 * @param   method  String containing the name of the method to check.
 * @param   l       Pointer to a method list, such as cmd->methods_limited.
 * @return  None.
 * @deffunc void ap_method_in_list(ap_method_list_t *l, const char *method)
 }
procedure ap_method_list_add(l: Pap_method_list_t; const method: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_list_add' + LibSuff8;

{
 * Remove an HTTP method name from an ap_method_list_t structure.
 *
 * @param   l       Pointer to a method list, such as cmd->methods_limited.
 * @param   method  String containing the name of the method to remove.
 * @return  None.
 * @deffunc void ap_method_list_remove(ap_method_list_t *l, const char *method)
 }
procedure ap_method_list_remove(l: Pap_method_list_t; const method: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_list_remove' + LibSuff8;

{
 * Reset a method list to be completely empty.
 *
 * @param   l       Pointer to a method list, such as cmd->methods_limited.
 * @return  None.
 * @deffunc void ap_clear_method_list(ap_method_list_t *l)
 }
procedure ap_clear_method_list(l: Pap_method_list_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_clear_method_list' + LibSuff4;

{
 * Set the content type for this request (r->content_type). 
 * @param r The current request
 * @param ct The new content type
 * @deffunc void ap_set_content_type(request_rec *r, const char* ct)
 * @warning This function must be called to set r->content_type in order 
 * for the AddOutputFilterByType directive to work correctly.
 }
procedure ap_set_content_type(r: Prequest_rec; const ct: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_set_content_type' + LibSuff8;

{ Hmmm... could macrofy these for now, and maybe forever, though the
 * definitions of the macros would get a whole lot hairier.
 }

{
 * Output one character for this request
 * @param c the character to output
 * @param r the current request
 * @return The number of bytes sent
 * @deffunc int ap_rputc(int c, request_rec *r)
 }
function ap_rputc(c: Integer; r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_rputc' + LibSuff8;

{
 * Output a string for the current request
 * @param str The string to output
 * @param r The current request
 * @return The number of bytes sent
 * @deffunc int ap_rputs(const char *str, request_rec *r)
 }
function ap_rputs(const str: PChar; r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_rputs' + LibSuff8;

{
 * Write a buffer for the current request
 * @param buf The buffer to write
 * @param nbyte The number of bytes to send from the buffer
 * @param r The current request
 * @return The number of bytes sent
 * @deffunc int ap_rwrite(const void *buf, int nbyte, request_rec *r)
 }
function ap_rwrite(const buf: Pointer; nbyte: Integer; r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_rwrite' + LibSuff12;

{
 * Write an unspecified number of strings to the request
 * @param r The current request
 * @param ... The strings to write
 * @return The number of bytes sent
 * @deffunc int ap_rvputs(request_rec *r, ...)
 }
function ap_rvputs(r: Prequest_rec; others: array of const): Integer;
 cdecl; external LibHTTPD name 'ap_rvputs';

//AP_DECLARE_NONSTD(int) ap_rvputs(request_rec *r,...);

{
 * Output data to the client in a printf format
 * @param r The current request
 * @param fmt The format string
 * @param vlist The arguments to use to fill out the format string
 * @return The number of bytes sent
 * @deffunc int ap_vrprintf(request_rec *r, const char *fmt, va_list vlist)
 }
function ap_vrprintf(r: Prequest_rec; const fmt: PChar; vlist: va_list): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_vrprintf' + LibSuff12;

{
 * Output data to the client in a printf format
 * @param r The current request
 * @param fmt The format string
 * @param ... The arguments to use to fill out the format string
 * @return The number of bytes sent
 * @deffunc int ap_rprintf(request_rec *r, const char *fmt, ...)
 }
function ap_rprintf(r: Prequest_rec; const fmt: PChar; others: array of const): Integer;
 cdecl; external LibHTTPD name 'ap_rprintf';

//AP_DECLARE_NONSTD(int) ap_rprintf(request_rec *r, const char *fmt,...)
//				__attribute__((format(printf,2,3)));

{
 * Flush all of the data for the current request to the client
 * @param r The current request
 * @return The number of bytes sent
 * @deffunc int ap_rflush(request_rec *r)
 }
function ap_rflush(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_rflush' + LibSuff4;

{
 * Index used in custom_responses array for a specific error code
 * (only use outside protocol.c is in getting them configured).
 * @param status HTTP status code
 * @return The index of the response
 * @deffunc int ap_index_of_response(int status)
 }
function ap_index_of_response(status: Integer): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_index_of_response' + LibSuff4;

{
 * Return the Status-Line for a given status code (excluding the
 * HTTP-Version field). If an invalid or unknown status code is
 * passed, "500 Internal Server Error" will be returned. 
 * @param status The HTTP status code
 * @return The Status-Line
 * @deffunc const char *ap_get_status_line(int status)
 }
function ap_get_status_line(status: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_status_line' + LibSuff4;

{ Reading a block of data from the client connection (e.g., POST arg) }

{
 * Setup the client to allow Apache to read the request body.
 * @param r The current request
 * @param read_policy How the server should interpret a chunked 
 *                    transfer-encoding.  One of: <pre>
 *    REQUEST_NO_BODY          Send 413 error if message has any body
 *    REQUEST_CHUNKED_ERROR    Send 411 error if body without Content-Length
 *    REQUEST_CHUNKED_DECHUNK  If chunked, remove the chunks for me.
 * </pre>
 * @return either OK or an error code
 * @deffunc int ap_setup_client_block(request_rec *r, int read_policy)
 }
function ap_setup_client_block(r: Prequest_rec; read_policy: Integer): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_setup_client_block' + LibSuff8;

{
 * Determine if the client has sent any data.  This also sends a 
 * 100 Continue response to HTTP/1.1 clients, so modules should not be called
 * until the module is ready to read content.
 * @warning Never call this function more than once.
 * @param r The current request
 * @return 0 if there is no message to read, 1 otherwise
 * @deffunc int ap_should_client_block(request_rec *r)
 }
function ap_should_client_block(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_should_client_block' + LibSuff4;

{
 * Call this in a loop.  It will put data into a buffer and return the length
 * of the input block
 * @param r The current request
 * @param buffer The buffer in which to store the data
 * @param bufsiz The size of the buffer
 * @return Number of bytes inserted into the buffer.  When done reading, 0
 *         if EOF, or -1 if there was an error
 * @deffunc long ap_get_client_block(request_rec *r, char *buffer, apr_size_t bufsiz)
 }
function ap_get_client_block(r: Prequest_rec; buffer: PChar; bufsiz: apr_size_t): cLong;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_client_block' + LibSuff12;

{
 * In HTTP/1.1, any method can have a body.  However, most GET handlers
 * wouldn't know what to do with a request body if they received one.
 * This helper routine tests for and reads any message body in the request,
 * simply discarding whatever it receives.  We need to do this because
 * failing to read the request body would cause it to be interpreted
 * as the next request on a persistent connection.
 * @param r The current request
 * @return error status if request is malformed, OK otherwise 
 * @deffunc int ap_discard_request_body(request_rec *r)
 }
function ap_discard_request_body(r: Prequest_rec): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_discard_request_body' + LibSuff4;

{
 * Setup the output headers so that the client knows how to authenticate
 * itself the next time, if an authentication request failed.  This function
 * works for both basic and digest authentication
 * @param r The current request
 * @deffunc void ap_note_auth_failure(request_rec *r)
 }
procedure ap_note_auth_failure(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_note_auth_failure' + LibSuff4;

{
 * Setup the output headers so that the client knows how to authenticate
 * itself the next time, if an authentication request failed.  This function
 * works only for basic authentication
 * @param r The current request
 * @deffunc void ap_note_basic_auth_failure(request_rec *r)
 }
procedure ap_note_basic_auth_failure(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_note_basic_auth_failure' + LibSuff4;

{
 * Setup the output headers so that the client knows how to authenticate
 * itself the next time, if an authentication request failed.  This function
 * works only for digest authentication
 * @param r The current request
 * @deffunc void ap_note_digest_auth_failure(request_rec *r)
 }
procedure ap_note_digest_auth_failure(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_note_digest_auth_failure' + LibSuff4;

{
 * Get the password from the request headers
 * @param r The current request
 * @param pw The password as set in the headers
 * @return 0 (OK) if it set the 'pw' argument (and assured
 *         a correct value in r->user); otherwise it returns 
 *         an error code, either HTTP_INTERNAL_SERVER_ERROR if things are 
 *         really confused, HTTP_UNAUTHORIZED if no authentication at all 
 *         seemed to be in use, or DECLINED if there was authentication but 
 *         it wasn't Basic (in which case, the caller should presumably 
 *         decline as well).
 * @deffunc int ap_get_basic_auth_pw(request_rec *r, const char **pw)
 }
function ap_get_basic_auth_pw(r: Prequest_rec; pw: PPChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_get_basic_auth_pw' + LibSuff8;

{
 * parse_uri: break apart the uri
 * @warning Side Effects: <pre>
 *    - sets r->args to rest after '?' (or NULL if no '?')
 *    - sets r->uri to request uri (without r->args part)
 *    - sets r->hostname (if not set already) from request (scheme://host:port)
 * </pre>
 * @param r The current request
 * @param uri The uri to break apart
 * @deffunc void ap_parse_uri(request_rec *r, const char *uri)
 }
procedure ap_parse_uri(r: Prequest_rec; const uri: PChar);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_parse_uri' + LibSuff8;

{
 * Get the next line of input for the request
 * @param s The buffer into which to read the line
 * @param n The size of the buffer
 * @param r The request
 * @param fold Whether to merge continuation lines
 * @return The length of the line, if successful
 *         n, if the line is too big to fit in the buffer
 *         -1 for miscellaneous errors
 * @deffunc int ap_method_number_of(const char *method)
 }
function ap_getline(s: PChar; n: Integer; r: Prequest_rec; fold: Integer): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_getline' + LibSuff16;

{
 * Get the next line of input for the request
 *
 * Note: on ASCII boxes, ap_rgetline is a macro which simply calls 
 *       ap_rgetline_core to get the line of input.
 * 
 *       on EBCDIC boxes, ap_rgetline is a wrapper function which
 *       translates ASCII protocol lines to the local EBCDIC code page
 *       after getting the line of input.
 *       
 * @param s Pointer to the pointer to the buffer into which the line
 *          should be read; if *s==NULL, a buffer of the necessary size
 *          to hold the data will be allocated from the request pool
 * @param n The size of the buffer
 * @param read The length of the line.
 * @param r The request
 * @param fold Whether to merge continuation lines
 * @param bb Working brigade to use when reading buckets
 * @return APR_SUCCESS, if successful
 *         APR_ENOSPC, if the line is too big to fit in the buffer
 *         Other errors where appropriate
 }
{#if APR_CHARSET_EBCDIC
AP_DECLARE(apr_status_t) ap_rgetline(char **s, apr_size_t n, 
                                     apr_size_t *read,
                                     request_rec *r, int fold,
                                     apr_bucket_brigade *bb);
#else }{ ASCII box }
{#define ap_rgetline(s, n, read, r, fold, bb) \
        ap_rgetline_core((s), (n), (read), (r), (fold), (bb))
#endif}

{AP_DECLARE(apr_status_t) ap_rgetline_core(char **s, apr_size_t n,
                                          apr_size_t *read,
                                          request_rec *r, int fold,
                                          apr_bucket_brigade *bb);}

{
 * Get the method number associated with the given string, assumed to
 * contain an HTTP method.  Returns M_INVALID if not recognized.
 * @param method A string containing a valid HTTP method
 * @return The method number
 }
function ap_method_number_of(const method: PChar): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_number_of' + LibSuff4;

{
 * Get the method name associated with the given internal method
 * number.  Returns NULL if not recognized.
 * @param p A pool to use for temporary allocations.
 * @param methnum An integer value corresponding to an internal method number
 * @return The name corresponding to the method number
 }
function ap_method_name_of(p: Papr_pool_t; methnum: Integer): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_method_name_of' + LibSuff8;


  { Hooks }
  {
   * post_read_request --- run right after read_request or internal_redirect,
   *                  and not run during any subrequests.
   }
{
 * This hook allows modules to affect the request immediately after the request
 * has been read, and before any other phases have been processes.  This allows
 * modules to make decisions based upon the input header fields
 * @param r The current request
 * @return OK or DECLINED
 * @deffunc ap_run_post_read_request(request_rec *r)
 }
type
  ap_HOOK_post_read_request_t = function(r: Prequest_rec): Integer; cdecl;

procedure ap_hook_post_read_request(pf: ap_HOOK_post_read_request_t;
 const aszPre: PPChar; const aszSucc: PPChar; nOrder: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_hook_post_read_request' + LibSuff16;

{
 * This hook allows modules to perform any module-specific logging activities
 * over and above the normal server things.
 * @param r The current request
 * @return OK, DECLINED, or HTTP_...
 * @deffunc int ap_run_log_transaction(request_rec *r)
 }
type
  ap_HOOK_log_transaction_t = function(r: Prequest_rec): Integer; cdecl;

procedure ap_hook_log_transaction(pf: ap_HOOK_log_transaction_t;
 const aszPre: PPChar; const aszSucc: PPChar; nOrder: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_hook_log_transaction' + LibSuff16;

{
 * This hook allows modules to retrieve the http scheme for a request.  This
 * allows Apache modules to easily extend the scheme that Apache understands
 * @param r The current request
 * @return The http method from the request
 }
type
  ap_HOOK_http_method_t = function(const r: Prequest_rec): PChar; cdecl;

procedure ap_hook_http_scheme(pf: ap_HOOK_http_method_t;
 const aszPre: PPChar; const aszSucc: PPChar; nOrder: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_hook_http_scheme' + LibSuff16;

{
 * Return the default port from the current request
 * @param r The current request
 * @return The current port
 * @deffunc apr_port_t ap_run_default_port(const request_rec *r)
 }
type
  ap_HOOK_default_port_t = function(const r: Prequest_rec): apr_port_t; cdecl;

procedure ap_hook_default_port(pf: ap_HOOK_default_port_t;
 const aszPre: PPChar; const aszSucc: PPChar; nOrder: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_hook_default_port' + LibSuff16;

{
 * A bucket referring to an HTTP error
 * This bucket can be passed down the filter stack to indicate that an
 * HTTP error occurred while running a filter.  In order for this bucket
 * to be used successfully, it MUST be sent as the first bucket in the
 * first brigade to be sent from a given filter.
 }
type
  ap_bucket_error = record
    { Number of buckets using this memory }
    refcount: apr_bucket_refcount;
    { The error code }
    status: Integer;
    { The error string }
    data: PChar;
  end;
  Pap_bucket_error = ^ap_bucket_error;

//AP_DECLARE_DATA extern const apr_bucket_type_t ap_bucket_type_error;

{
 * Determine if a bucket is an error bucket
 * @param e The bucket to inspect
 * @return true or false
 }
//#define AP_BUCKET_IS_ERROR(e)         (e->type == &ap_bucket_type_error)

{
 * Make the bucket passed in an error bucket
 * @param b The bucket to make into an error bucket
 * @param error The HTTP error code to put in the bucket. 
 * @param buf An optional error string to put in the bucket.
 * @param p A pool to allocate out of.
 * @return The new bucket, or NULL if allocation failed
 * @deffunc apr_bucket *ap_bucket_error_make(apr_bucket *b, int error, const char *buf, apr_pool_t *p)
 }
function ap_bucket_error_make(b: Papr_bucket; error: Integer;
 const buf: PChar; p: Papr_pool_t): Papr_bucket;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_bucket_error_make' + LibSuff16;

{
 * Create a bucket referring to an HTTP error.
 * @param error The HTTP error code to put in the bucket. 
 * @param buf An optional error string to put in the bucket.
 * @param p A pool to allocate the error string out of.
 * @param list The bucket allocator from which to allocate the bucket
 * @return The new bucket, or NULL if allocation failed
 * @deffunc apr_bucket *ap_bucket_error_create(int error, const char *buf, apr_pool_t *p, apr_bucket_alloc_t *list)
 }
function ap_bucket_error_create(error: Integer; const buf: PChar;
 p: Papr_pool_t; list: Papr_bucket_alloc_t): Papr_bucket;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_bucket_error_create' + LibSuff16;

function ap_byterange_filter(f: Pap_filter_t; b: Papr_bucket_brigade): apr_status_t;
 cdecl; external LibHTTPD name 'ap_byterange_filter';

function ap_http_header_filter(f: Pap_filter_t; b: Papr_bucket_brigade): apr_status_t;
 cdecl; external LibHTTPD name 'ap_http_header_filter';

function ap_content_length_filter(f: Pap_filter_t; b: Papr_bucket_brigade): apr_status_t;
 cdecl; external LibHTTPD name 'ap_content_length_filter';

function ap_old_write_filter(f: Pap_filter_t; b: Papr_bucket_brigade): apr_status_t;
 cdecl; external LibHTTPD name 'ap_old_write_filter';

{
 * Sett up the protocol fields for subsidiary requests
 * @param rnew New Sub Request
 * @param r current request
 }
procedure ap_set_sub_req_protocol(rnew: Prequest_rec; const r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_set_sub_req_protocol' + LibSuff8;

{
 * A wrapup function to keep the internal accounting straight.
 * Indicates that there is no more content coming.
 * @param sub_r Subrequest that is now compete
 }
procedure ap_finalize_sub_req_protocol(sub_r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibHTTPD name LibNamePrefix + 'ap_finalize_sub_req_protocol' + LibSuff4;

