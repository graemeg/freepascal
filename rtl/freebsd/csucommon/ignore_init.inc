{
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright 2012 Konstantin Belousov <kib@FreeBSD.org>
 * Copyright (c) 2018 The FreeBSD Foundation
 *
 * Parts of this software was developed by Konstantin Belousov
 * <kib@FreeBSD.org> under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Translated into Pascal by Marco van de Voort 
}

//#include <sys/cdefs.h>
//__FBSDID("$FreeBSD: releng/12.1/lib/csu/common/ignore_init.c 339351 2018-10-13 23:52:55Z kib $");

//#include <sys/param.h>
//#include <sys/elf.h>
//#include <sys/elf_common.h>

//#include "notes.h"

// replaced by pascalmain in si_c.pp
//function cmain(argc:longint;argv:ppchar;env:ppchar):longint; cdecl; external name '_main';


Type TInitProc     = procedure(argc:longint;argv:ppchar;env:ppchar); cdecl;
     PInitProc     = ^TInitProc;
     TCleanupProc  = procedure; cdecl;
     PCleanupProc  = ^TCleanupProc;

procedure atexit(prc:TCleanupProc); cdecl external name 'atexit';
procedure cleanup(prc:TCleanupProc); cdecl external name 'cleanup';

var
  environ : ppchar; cvar; public  name '__environ';
  _Dynamic  : longint; weakexternal name '_DYNAMIC';

var
    preinit_array_start  : PInitProc; external name '__preinit_array_start';
    preinit_array_end    : PInitProc; external name '__preinit_array_end';
    init_array_start     : PInitProc; external name '__init_array_start';
    init_array_end       : PInitProc; external name '__init_array_end';
    fini_array_start     : PCleanupProc; external name '__fini_array_start';
    fini_array_end       : PCleanupProc; external name '__fini_array_end';


procedure _fini; cdecl; external name '_fini';
procedure _init; cdecl; external name '_init';
procedure libc_exit(exitcode:longint);cdecl; external name 'exit';

Type
     // I only doublechecked these for don't know how these records are defined, but they are resp. 24 and 16 byte on 64-bit
     Elf_Rela = record
      	r_offset : uint64 {Elf64_Addr};	 { Location to be relocated. }
      	r_info   : uint64 {Elf64_Xword};	 { Relocation type and symbol index. }
      	r_addend : uint64 {Elf64_Sxword}; { Addend. }
     end;
     PElf_Rela = ^Elf_Rela;
     Elf_Rel  = record
      	r_offset : uint64 {Elf64_Addr};	 { Location to be relocated. }
      	r_info   : uint64 {Elf64_Xword};	 { Relocation type and symbol index. }
       end;
     PElf_Rel = ^Elf_Rel;

{$if defined(CRT_IRELOC_RELA)}
var
  rela_iplt_start : Elf_rela; weakexternal name '__rela_iplt_start';
  rela_iplt_end   : Elf_rela; weakexternal name '__rela_iplt_end';

{$include reloc.inc}

procedure process_irelocs; cdecl;
var p,p2 : pElf_Rela;
begin
  p:=@rela_iplt_start;
  p2:=@rela_iplt_end;
  while (p<p2) do
    begin
       crt1_handle_rela(p);
       inc(p);
    end;
end;
{$elseif defined(CRT_IRELOC_REL)}

var
  rel_iplt_start : Elf_Rel; weakexternal name '__rel_iplt_start';
  rel_iplt_end   : Elf_Rel; weakexternal name '__rel_iplt_end';

  {$include reloc.inc}

procedure process_irelocs; cdecl;
var p,p2 : pElf_Rel;
begin
  p:=@rel_iplt_start;
  p2:=@rel_iplt_end;
  while (p<p2) do
    begin
       crt1_handle_rel(p)
       inc(p);
    end;
end;

{$elseif defined(CRT_IRELOC_SUPPRESS)}
{$else}
{$error 'Define platform reloc type'}
{$endif}


procedure finalizer; cdecl;
var
  fn : TCleanupProc;
  n,array_size : ptruint; // actually: size_t;
begin
  array_size:= fini_array_end - fini_array_start;
  n:=array_size;
  while n>0 do
    begin
      fn := fini_array_start[n - 1];
      if assigned(fn) and (ptrint(fn)<>-1) then
        fn();
      dec(n);
    end;
 _fini();
end;

procedure handle_static_init(argc:longint;argv:ppchar;env:ppchar);  cdecl;
var fn : TInitProc;
    n,array_size : ptruint; // actually: size_t;
begin
   if assigned(@_dynamic) then
     exit;
   atexit(@finalizer);
   array_size := preinit_array_end - preinit_array_start;
   n:=0;
   while n<array_size do
     begin
       fn := preinit_array_start[n];
       if assigned(fn) and (ptrint(fn)<>-1) then
         fn(argc,argv,env);
       inc(n);
     end;
   _init();
   n:=0;
   while n<array_size do
     begin
       fn := init_array_start[n];
       if assigned(fn) and (ptrint(fn)<>-1) then
         fn(argc,argv,env);
       inc(n);
     end;
end;

procedure handle_argv(argc:longint;argv:ppchar;env:ppchar); inline;
var
  s: pchar;
begin
   if assigned(environ) then
      environ:=env;
   if (argc>0) and assigned(argv[0]) then
     begin
       progname:=argv[0];
       s:=progname;
       while s^<>#0 do
          begin
            if s^='/' then
              progname:=@s[1];
          inc(s);
       end;
     end;
end;

(*
static const struct {
	int32_t	namesz;
	int32_t	descsz;
	int32_t	type;
	char	name[sizeof(NOTE_FREEBSD_VENDOR)];
	uint32_t desc;
} crt_noinit_tag __attribute__ ((section (NOTE_SECTION),
    aligned(4))) __used = {
	.namesz = sizeof(NOTE_FREEBSD_VENDOR),
	.descsz = sizeof(uint32_t),
	.type = NT_FREEBSD_NOINIT_TAG,
	.name = NOTE_FREEBSD_VENDOR,
	.desc = 0
};

*)
