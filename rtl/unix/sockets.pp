{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Sockets;
Interface

{$ifdef Unix}
Uses baseunix,UnixType;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
{$i osdefs.inc}       { Compile time defines }

{$if 
     defined(FreeBSD) or 
     defined(Darwin) or 
     defined(Haiku)
}
{$DEFINE SOCK_HAS_SINLEN}               // BSD definition of socketaddr
{$endif}

Type 
 TSockLen = BaseUnix.TSocklen;
=======
=======
{$i osdefs.inc}       { Compile time defines }

>>>>>>> origin/fixes_2.4
{$ifdef FreeBSD}
{$DEFINE SOCK_HAS_SINLEN}               // BSD definition of socketaddr
{$endif}

{$ifdef Darwin}
{$DEFINE SOCK_HAS_SINLEN}               // BSD definition of socketaddr
{$endif}
>>>>>>> graemeg/fixes_2_2

{$i unxsockh.inc}
{$i socketsh.inc}

type
  TUnixSockAddr = packed Record
                 {$ifdef SOCK_HAS_SINLEN}
                    sa_len     : cuchar;
                 {$endif}
                  family       : sa_family_t;
                  path:array[0..107] of char;    //104 total for freebsd.
                  end;

const
  EsockEINTR            = EsysEINTR;   
  EsockEBADF            = EsysEBADF;
  EsockEFAULT           = EsysEFAULT;
  EsockEINVAL           = EsysEINVAL;
  EsockEACCESS          = ESysEAcces;
  EsockEMFILE           = ESysEmfile;
{$ifndef beos}
  EsockEMSGSIZE         = ESysEMsgSize;
{$endif beos}
  EsockENOBUFS          = ESysENoBufs;
  EsockENOTCONN         = ESysENotConn;
{$ifndef beos}  
  EsockENOTSOCK         = ESysENotSock;
{$endif beos}
  EsockEPROTONOSUPPORT  = ESysEProtoNoSupport;
  EsockEWOULDBLOCK      = ESysEWouldBlock;


{ unix socket specific functions }
Procedure Str2UnixSockAddr(const addr:string;var t:TUnixSockAddr;var len:longint); deprecated;
Function Bind(Sock:longint;const addr:string):boolean; deprecated;
Function Connect(Sock:longint;const addr:string;var SockIn,SockOut:text):Boolean; deprecated;
Function Connect(Sock:longint;const addr:string;var SockIn,SockOut:file):Boolean; deprecated;
Function Accept(Sock:longint;var addr:string;var SockIn,SockOut:text):Boolean;    deprecated;
Function Accept(Sock:longint;var addr:string;var SockIn,SockOut:File):Boolean;    deprecated;

//function  fpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint; maybelibc
//function  fpbind      (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;  maybelibc
//function  fpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;  maybelibc

Implementation

Uses {$ifndef FPC_USE_LIBC}SysCall{$else}initc{$endif};

threadvar internal_socketerror : cint;

{ Include filerec and textrec structures }
{$i filerec.inc}
{$i textrec.inc}
{******************************************************************************
                          Kernel Socket Callings
******************************************************************************}

function socketerror:cint;

begin
  socketerror:=internal_socketerror;
end;

{$ifndef FPC_USE_LIBC}
{$i unixsock.inc}
{$else}
{$i stdsock.inc}
{$endif}
{$i sockovl.inc}
{$i sockets.inc}
end.
