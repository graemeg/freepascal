unit cNetDB;

{*-
 * Copyright (c) 1980, 1983, 1988, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * -
 * Portions Copyright (c) 1993 by Digital Equipment Corporation.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies, and that
 * the name of Digital Equipment Corporation not be used in advertising or
 * publicity pertaining to distribution of the document or software without
 * specific, written prior permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND DIGITAL EQUIPMENT CORP. DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS.   IN NO EVENT SHALL DIGITAL EQUIPMENT
 * CORPORATION BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 * SOFTWARE.
 * -
 * --Copyright--
 *}

{*
 *      @(#)netdb.h	8.1 (Berkeley) 6/2/93
 *      From: Id: netdb.h,v 8.9 1996/11/19 08:39:29 vixie Exp $
 * $FreeBSD: src/include/netdb.h,v 1.38.2.1 2005/07/22 20:17:30 ume Exp $
 *}

{$mode objfpc}{$H+}

interface

uses
  BaseUnix, Sockets,initc;
  
const
  LIB_C = clib; // use initc's idea of what libc is called. In the future overrides might be necessary here.
                // (older BSD convention is to have a separate libresolve that contains these functions)

// Error return codes from gethostbyname() and gethostbyaddr()
// (left in h_errno).

const
   NETDB_INTERNAL  = -(1);	{ see errno }	
   NETDB_SUCCESS   = 0;		{ no problem  }                                       	
   HOST_NOT_FOUND  = 1;		{ Authoritative Answer Host not found  }              	
   TRY_AGAIN 	   = 2;		{ Non-Authoritative Host not found, or SERVERFAIL  }  	
   NO_RECOVERY     = 3;		{ Non recoverable errors, FORMERR, REFUSED, NOTIMP  }   	
   NO_DATA 	   = 4;		{ Valid name, no data record of requested type  }     	
   NO_ADDRESS = NO_DATA;        { no address, look for MX record  }
{
  return codes from getaddrinfo()
}

   EAI_AGAIN      = 2;          { address family for hostname not supported }
   EAI_BADFLAGS   = 3;          { invalid value for ai_flags  }
   EAI_FAIL       = 4;		{ non-recoverable failure in name resolution  }
   EAI_FAMILY     = 5;          { ai_family not supported  }
   EAI_MEMORY     = 6;          { memory allocation failure  }
   EAI_NONAME     = 8;          { hostname nor servname provided, or not known  }
   EAI_SERVICE    = 9;          { servname not supported for ai_socktype  } 
   EAI_SOCKTYPE   = 10;         { ai_socktype not supported  }
   EAI_SYSTEM     = 11;         { system error returned in errno  }
   EAI_BADHINTS   = 12;
   EAI_PROTOCOL   = 13;
   EAI_MAX = 14;
{
 * Flag values for getaddrinfo()
  }
{ get address to use bind()  }
   AI_PASSIVE = $00000001;
{ fill ai_canonname  }
   AI_CANONNAME = $00000002;
{ prevent host name resolution  }
   AI_NUMERICHOST = $00000004;
{ prevent service name resolution  }
   AI_NUMERICSERV = $00000008;
{ IPv6 and IPv4-mapped (with AI_V4MAPPED)  }
   AI_ALL = $00000100;
{ accept IPv4-mapped if kernel supports  }
   AI_V4MAPPED_CFG = $00000200;
{ only if any address is assigned  }
   AI_ADDRCONFIG = $00000400;
{ accept IPv4-mapped IPv6 address  }
   AI_V4MAPPED = $00000800;
{ special recommended flags for getipnodebyname  }
   AI_DEFAULT = AI_V4MAPPED_CFG or AI_ADDRCONFIG;
{ valid flags for addrinfo (not a standard def, apps should not use it)  }
   AI_MASK = AI_PASSIVE or AI_CANONNAME or AI_NUMERICHOST or AI_NUMERICSERV or AI_ADDRCONFIG;

{
 * Constants for getnameinfo()
  }
   NI_MAXHOST = 1025;
   NI_MAXSERV = 32;
{
 * Flag values for getnameinfo()
  }
   NI_NOFQDN = $00000001;
   NI_NUMERICHOST = $00000002;
   NI_NAMEREQD = $00000004;
   NI_NUMERICSERV = $00000008;
   NI_DGRAM = $00000010;
{
 * Scope delimit character
  }

const
   SCOPE_DELIMITER = '%';
   
//#define	h_addr	h_addr_list[0]	/* address, for backward compatibility */

type
{*
 * Structures returned by network data base library.  All addresses are
 * supplied in host order, and returned in network order (suitable for
 * use in system calls).
 *}
  hostent = record
    h_name: PChar;      {/* official name of host *}
    h_aliases: PPChar;  {* alias list *}
    h_addrtype: cInt;   {* host address type *}
    h_length: cInt;     {* length of address *}
    h_addr_list: PPChar;{* list of addresses from name server *}
  end;

  THostEnt = hostent;
  PHostEnt = ^THostEnt;
  PPHostEnt = ^PHostEnt;

  netent = record
    n_name: PChar;      {* official name of net *}
    n_aliases: PPChar;  {* alias list *}
    n_addrtype: cInt;   {* net address type *}
    n_net: cuInt32;     {* network # *}
  end;
  TNetEnt = netent;
  PNetEnt = ^TNetEnt;
  PPNetEnt = ^PNetEnt;

  servent = record
    s_name: PChar;    {* official service name *}
    s_aliases: PPChar;{* alias list *}
    s_port: cInt;     {* port # *}
    s_proto: PChar;   {* protocol to use *}
  end;
  TServEnt = servent;
  PServEnt = ^TServEnt;
  PPServEnt = ^PServEnt;

  protoent = record
    p_name: PChar; {* official protocol name *}
    p_aliases: PPChar;  {* alias list *}
    p_proto: cInt;      {* protocol # *}
  end;
  TProtoEnt = protoent;
  PProtoEnt = ^TProtoEnt;
  PPProtoEnt = ^PProtoEnt;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(LINUX) or defined(OPENBSD)}
{$define FIRST_ADDR_THEN_CANONNAME}
{$endif}
{$if defined(FREEBSD) or defined(NETBSD) or defined(DRAGONFLY) or defined(SOLARIS)}
{$define FIRST_CANONNAME_THEN_ADDR}
{$endif}
{$if not defined(FIRST_CANONNAME_THEN_ADDR) and not defined(FIRST_ADDR_THEN_CANONNAME)}
{$error fatal 'Please consult the netdb.h file for your system to determine the order of ai_addr and ai_canonname'}
{$endif} 

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  PAddrInfo = ^addrinfo;
  addrinfo = record
    ai_flags: cInt;     {* AI_PASSIVE, AI_CANONNAME, AI_NUMERICHOST *}
    ai_family: cInt;    {* PF_xxx *}
    ai_socktype: cInt;  {* SOCK_xxx *}
    ai_protocol: cInt;  {* 0 or IPPROTO_xxx for IPv4 and IPv6 *}
    ai_addrlen: TSocklen;  {* length of ai_addr *}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FIRST_CANONNAME_THEN_ADDR}
    ai_canonname: PChar;   {* canonical name for hostname *}
    ai_addr: psockaddr;	   {* binary address *}
{$endif}
{$ifdef FIRST_ADDR_THEN_CANONNAME}
    ai_addr: psockaddr;	   {* binary address *}
    ai_canonname: PChar;   {* canonical name for hostname *}
{$endif}
=======
    ai_canonname: PChar;   {* canonical name for hostname *}
    ai_addr: psockaddr;	   {* binary address *}
>>>>>>> graemeg/cpstrnew
=======
    ai_canonname: PChar;   {* canonical name for hostname *}
    ai_addr: psockaddr;	   {* binary address *}
>>>>>>> graemeg/cpstrnew
=======
    ai_canonname: PChar;   {* canonical name for hostname *}
    ai_addr: psockaddr;	   {* binary address *}
>>>>>>> graemeg/cpstrnew
=======
    ai_canonname: PChar;   {* canonical name for hostname *}
    ai_addr: psockaddr;	   {* binary address *}
>>>>>>> origin/cpstrnew
    ai_next: PAddrInfo;	   {* next structure in linked list *}
  end;
  TAddrInfo = addrinfo;
  PPAddrInfo = ^PAddrInfo;
   

procedure endhostent; cdecl; external LIB_C name 'endhostent';
procedure endnetent; cdecl; external LIB_C name 'endnetent';
procedure endnetgrent; cdecl; external LIB_C name 'endnetgrent';
procedure endprotoent; cdecl; external LIB_C name 'endprotoent';
procedure endservent; cdecl; external LIB_C name 'endservent';
procedure freehostent(ptr: PHostEnt); cdecl; external LIB_C name 'freehostent';
function  gethostbyaddr(addr: PChar; len, typ: cInt): PHostEnt; cdecl; external LIB_C name 'gethostbyaddr';

function  gethostbyname(name: PChar): PHostEnt; cdecl; external LIB_C name 'gethostbyname';

function  gethostbyname2(name: PChar; af: cInt): PHostEnt; cdecl; external LIB_C name 'gethostbyname2';

function  gethostent: PHostEnt; cdecl; external LIB_C name 'gethostent';

function getipnodebyaddr(src: Pointer; len: size_t; af: cInt; error_num: PcInt): PHostEnt; cdecl; external LIB_C name 'getipnodebyaddr';

function getipnodebyname(name: PChar; af, flags: cInt; error_num: PcInt): PHostEnt; cdecl; external LIB_C name 'getipnodebyname';

function getnetbyaddr(net: cint32; typ: cInt): PNetEnt; cdecl; external LIB_C name 'getnetbyaddr';

function getnetbyname(name: PChar): PNetEnt; cdecl; external LIB_C name 'getnetbyname';

function getnetent: PNetEnt; cdecl; external LIB_C name 'getnetent';

function getnetgrent(host, user, domain: PPChar): cInt; cdecl; external LIB_C name 'getnetgrent';

function getprotobyname(name: PChar): PProtoEnt; cdecl; external LIB_C name 'getprotobyname';

function getprotobynumber(proto: cInt): PProtoEnt; cdecl; external LIB_C name 'getprotobynumber';

function getprotoent: PProtoEnt; cdecl; external LIB_C name 'getprotoent';

function getservbyname(name, proto: PChar): PServEnt; cdecl; external LIB_C name 'getservbyname';

function getservbyport(port: cInt; proto: PChar): PServEnt; cdecl; external LIB_C name 'getservbyport';

function  getservent: PServEnt; cdecl; external LIB_C name 'getservent';
procedure herror(erstring: PChar); cdecl; external LIB_C name 'herror';
function  hstrerror(err: cInt): PChar; cdecl; external LIB_C name 'hstrerror';
function  innetgr(netgroup, host, user, domain: PChar): cInt; cdecl; external LIB_C name 'innetgr';
procedure sethostent(i: cInt); cdecl; external LIB_C name 'sethostent';

{* void		sethostfile(const char *); *}

procedure setnetent(stayopen: cInt); cdecl; external LIB_C name 'setnetent';
procedure setprotoent(stayopen: cInt); cdecl; external LIB_C name 'setprotoent';
function  getaddrinfo(hostname, servname: PChar;
                     hints: PAddrInfo; res: PPAddrInfo): cInt; cdecl; external LIB_C name 'getaddrinfo';
function  getnameinfo(sa: PSockAddr; salen: TSockLen; host: PChar; hostlen: TSize;
                     serv: PChar; servlen: TSize; flags: cInt): cInt; cdecl; external LIB_C name 'getnameinfo';
procedure freeaddrinfo(ai: PAddrInfo); cdecl; external LIB_C name 'freeaddrinfo';
function  gai_strerror(ecode: cInt): PChar; cdecl; external LIB_C name 'gai_strerror';
procedure setnetgrent(netgroup: PChar); cdecl; external LIB_C name 'setnetgrent';
procedure setservent(stayopen: cInt); cdecl; external LIB_C name 'setservent';


implementation

end.

