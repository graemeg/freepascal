{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2008 by the Free Pascal development team

    This file implements dyn. lib calls calls for Unix

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit dl;

interface

const
{$ifdef BSD}   // dlopen is in libc on FreeBSD.
  LibDL = 'c';
{$else}
  {$ifdef HAIKU}
    LibDL = 'root';
  {$else}
    LibDL = 'dl';
  {$endif}
{$endif}

{$if defined(linux) or defined(freebsd) or defined(openbsd) or defined(dragonfly)}
  {$define ELF} // ELF symbol versioning.
{$endif}

{$if defined(linux)}
    { if libc is not linked explicitly, FPC might chose the wrong startup code, as
      libdl depends on libc on linux, this does not hurt }
    {$linklib c}
{$endif}

{$ifdef aix}
  RTLD_LAZY         = $004;
  RTLD_NOW          = $002;
  RTLD_BINDING_MASK = $006;
  RTLD_LOCAL        = 0;
  RTLD_GLOBAL       = $10000;
  RTLD_MEMBER       = $40000;
  RTLD_NEXT         = pointer(-3);
  RTLD_DEFAULT      = pointer(-1);
{$else}
  RTLD_LAZY         = $001;
  RTLD_NOW          = $002;
  RTLD_BINDING_MASK = $003;
  {$ifdef DARWIN}
  RTLD_LOCAL        = $004;
  RTLD_GLOBAL       = $008;
  RTLD_NOLOAD       = $010;
  RTLD_NODELETE     = $080;
  RTLD_FIRST        = $100;
  RTLD_NEXT         = pointer(-1);
  RTLD_DEFAULT      = pointer(-2);
  RTLD_SELF         = pointer(-3);
  RTLD_MAIN_ONLY    = pointer(-5);
  {$else}
  RTLD_LOCAL        = 0;
  RTLD_GLOBAL       = $100;
  RTLD_NEXT         = pointer(-1);
  {$ifdef LINUX}
  RTLD_DEFAULT      = nil;
  RTLD_NOLOAD = $00004; // GLIBC 2.2 and above
  RTLD_DI_LINKMAP = 2;
  {$endif}
  {$ifdef BSD}
  RTLD_DEFAULT      = pointer(-2);
  RTLD_MODEMASK     = RTLD_BINDING_MASK;
  {$endif}
  {$endif}          // DARWIN
{$endif}

type
  Pdl_info = ^dl_info;
  dl_info = record
    dli_fname      : Pchar;
    dli_fbase      : pointer;
    dli_sname      : Pchar;
    dli_saddr      : pointer;
  end;

function dlopen(Name : PChar; Flags : longint) : Pointer; cdecl; external libdl;
function dlsym(Lib : Pointer; Name : Pchar) : Pointer; cdecl; external Libdl;
{$ifdef ELF}
function dlvsym(Lib : Pointer; Name : Pchar; Version: Pchar) : Pointer; cdecl; external Libdl;
{$endif}
function dlclose(Lib : Pointer) : Longint; cdecl; external libdl;
function dlerror() : Pchar; cdecl; external libdl;

{ overloaded for compatibility with hmodule }
function dlsym(Lib : PtrInt; Name : Pchar) : Pointer; cdecl; external Libdl;
function dlclose(Lib : PtrInt) : Longint; cdecl; external libdl;
function dladdr(Lib: pointer; info: Pdl_info): Longint; cdecl; {$if not defined(aix) and not defined(android)} external; {$endif}

type
  plink_map = ^link_map;
  link_map = record
    l_addr:pointer;  { Difference between the address in the ELF file and the address in memory }
    l_name:Pchar;  { Absolute pathname where object was found }
    l_ld:pointer;    { Dynamic section of the shared object }
    l_next, l_prev:^link_map; { Chain of loaded objects }
    {Plus additional fields private to the implementation }
  end;

{$if defined(BSD) or defined(LINUX)}
function dlinfo(Lib:pointer;request:longint;info:pointer):longint;cdecl;external Libdl;
{$else}
{ Fortunately investigating the sources of open source projects brought the understanding, that
  `handle` is just a `struct link_map*` that contains full library name.}
{$endif}


implementation

uses
  ctypes;

  function PosLastSlash(const s : string) : longint;
    var
      i : longint;
    begin 
      PosLastSlash:=0;
      for i:=1 to length(s) do
        if s[i]='/' then
          PosLastSlash:=i;
    end;
    
    
  function SimpleExtractFilename(const s : string) : string;
    begin
      SimpleExtractFilename:=Copy(s,PosLastSlash(s)+1,Length(s)-PosLastSlash(s));
    end;


  procedure UnixGetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: openstring);
    var
      dlinfo: dl_info;
    begin
      baseaddr:=nil;
      FillChar(dlinfo, sizeof(dlinfo), 0);
      dladdr(addr, @dlinfo);
      baseaddr:=dlinfo.dli_fbase;
      filename:=String(dlinfo.dli_fname);
    end;

{$ifdef aix}
{$i dlaix.inc}
{$endif}

{$ifdef android}
{$i dlandroid.inc}
{$endif}

begin
  UnixGetModuleByAddrHook:=@UnixGetModuleByAddr;
end.
