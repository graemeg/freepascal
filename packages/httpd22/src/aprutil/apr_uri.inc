{ Copyright 2000-2005 The Apache Software Foundation or its licensors, as
 * applicable.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 }

{
 * apr_uri.h: External Interface of apr_uri.c
 }

{
 * @file apr_uri.h
 * @brief APR-UTIL URI Routines
 }

{#include "apu.h"

#include "apr_network_io.h"}

{
 * @defgroup APR_Util_URI URI
 * @ingroup APR_Util
 }
const
  APR_URI_FTP_DEFAULT_PORT        = 21; {< default FTP port }
  APR_URI_SSH_DEFAULT_PORT        = 22; {< default SSH port }
  APR_URI_TELNET_DEFAULT_PORT     = 23; {< default telnet port }
  APR_URI_GOPHER_DEFAULT_PORT     = 70; {< default Gopher port }
  APR_URI_HTTP_DEFAULT_PORT       = 80; {< default HTTP port }
  APR_URI_POP_DEFAULT_PORT       = 110; {< default POP port }
  APR_URI_NNTP_DEFAULT_PORT      = 119; {< default NNTP port }
  APR_URI_IMAP_DEFAULT_PORT      = 143; {< default IMAP port }
  APR_URI_PROSPERO_DEFAULT_PORT  = 191; {< default Prospero port }
  APR_URI_WAIS_DEFAULT_PORT      = 210; {< default WAIS port }
  APR_URI_LDAP_DEFAULT_PORT      = 389; {< default LDAP port }
  APR_URI_HTTPS_DEFAULT_PORT     = 443; {< default HTTPS port }
  APR_URI_RTSP_DEFAULT_PORT      = 554; {< default RTSP port }
  APR_URI_SNEWS_DEFAULT_PORT     = 563; {< default SNEWS port }
  APR_URI_ACAP_DEFAULT_PORT      = 674; {< default ACAP port }
  APR_URI_NFS_DEFAULT_PORT      = 2049; {< default NFS port }
  APR_URI_TIP_DEFAULT_PORT      = 3372; {< default TIP port }
  APR_URI_SIP_DEFAULT_PORT      = 5060; {< default SIP port }

{ Flags passed to unparse_uri_components(): }
{ suppress "scheme://user\@site:port" }
  APR_URI_UNP_OMITSITEPART    = (1 shl 0);
{ Just omit user }
  APR_URI_UNP_OMITUSER        = (1 shl 1);
{ Just omit password }
  APR_URI_UNP_OMITPASSWORD    = (1 shl 2);
{ omit "user:password\@" part }
  APR_URI_UNP_OMITUSERINFO    = (APR_URI_UNP_OMITUSER or APR_URI_UNP_OMITPASSWORD);
{ Show plain text password (default: show XXXXXXXX) }
  APR_URI_UNP_REVEALPASSWORD  = (1 shl 3);
{ Show "scheme://user\@site:port" only }
  APR_URI_UNP_OMITPATHINFO    = (1 shl 4);
{ Omit the "?queryarg" from the path }
  APR_URI_UNP_OMITQUERY       = (1 shl 5);

type
{ @see apr_uri_t }
  Papr_uri_t = ^apr_uri_t;

{
 * A structure to encompass all of the fields in a uri
 }
  apr_uri_t = record
    { scheme ("http"/"ftp"/...) }
    scheme: PChar;
    { combined [user[:password]\@]host[:port] }
    hostinfo: PChar;
    { user name, as in http://user:passwd\@host:port/ }
    user: PChar;
    { password, as in http://user:passwd\@host:port/ }
    password: PChar;
    { hostname from URI (or from Host: header) }
    hostname: PChar;
    { port string (integer representation is in "port") }
    port_str: PChar;
    { the request path (or "/" if only scheme://host was given) }
    path: PChar;
    { Everything after a '?' in the path, if present }
    query: PChar;
    { Trailing "#fragment" string, if present }
    fragment: PChar;

    { structure returned from gethostbyname() }
    hostent: Pointer;

    { The port number, numeric, valid only if port_str != NULL }
    port: apr_port_t;
    
    { has the structure been initialized }
//    unsigned is_initialized:1;

    { has the DNS been looked up yet }
//    unsigned dns_looked_up:1;
    { has the dns been resolved yet }
//    unsigned dns_resolved:1;
  end;

{ apr_uri.c }
{
 * Return the default port for a given scheme.  The schemes recognized are
 * http, ftp, https, gopher, wais, nntp, snews, and prospero
 * @param scheme_str The string that contains the current scheme
 * @return The default port for this scheme
 }
function apr_uri_port_of_scheme(const scheme_str: PChar): apr_port_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPRUtil name LibNamePrefix + 'apr_uri_port_of_scheme' + LibSuff4;

{
 * Unparse a apr_uri_t structure to an URI string.  Optionally 
 * suppress the password for security reasons.
 * @param p The pool to allocate out of
 * @param uptr All of the parts of the uri
 * @param flags How to unparse the uri.  One of:
 * <PRE>
 *    APR_URI_UNP_OMITSITEPART        Suppress "scheme://user\@site:port" 
 *    APR_URI_UNP_OMITUSER            Just omit user 
 *    APR_URI_UNP_OMITPASSWORD        Just omit password 
 *    APR_URI_UNP_OMITUSERINFO        Omit "user:password\@" part
 *    APR_URI_UNP_REVEALPASSWORD      Show plain text password (default: show XXXXXXXX)
 *    APR_URI_UNP_OMITPATHINFO        Show "scheme://user\@site:port" only 
 *    APR_URI_UNP_OMITQUERY           Omit "?queryarg" or "#fragment" 
 * </PRE>
 * @return The uri as a string
 }
function apr_uri_unparse(p: Papr_pool_t; const uptr: Papr_uri_t;
 flags: cuint): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPRUtil name LibNamePrefix + 'apr_uri_unparse' + LibSuff12;

{
 * Parse a given URI, fill in all supplied fields of a apr_uri_t
 * structure. This eliminates the necessity of extracting host, port,
 * path, query info repeatedly in the modules.
 * @param p The pool to allocate out of
 * @param uri The uri to parse
 * @param uptr The apr_uri_t to fill out
 * @return APR_SUCCESS for success or error code
 }
function apr_uri_parse(p: Papr_pool_t; const uri: PChar;
 uptr: Papr_uri_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPRUtil name LibNamePrefix + 'apr_uri_parse' + LibSuff12;

{
 * Special case for CONNECT parsing: it comes with the hostinfo part only
 * @param p The pool to allocate out of
 * @param hostinfo The hostinfo string to parse
 * @param uptr The apr_uri_t to fill out
 * @return APR_SUCCESS for success or error code
 }
function apr_uri_parse_hostinfo(p: Papr_pool_t;
 const hostinfo: PChar; uptr: Papr_uri_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPRUtil name LibNamePrefix + 'apr_uri_parse_hostinfo' + LibSuff12;

