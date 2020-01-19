{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2017 by Sven Barth
    member of the Free Pascal development team.

    Interface unit for the Foreign Function Interface library.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

(* -------------------------------------------------------------------
   The basic API is described in the README file.

   The raw API is designed to bypass some of the argument packing
   and unpacking on architectures for which it can be avoided.

   The closure API allows interpreted functions to be packaged up
   inside a C function pointer, so that they can be called as C functions,
   with no understanding on the client side that they are interpreted.
   It can also be used in other cases in which it is necessary to package
   up a user specified parameter and a function pointer as a single
   function pointer.

   The closure API must be implemented in order to get its functionality,
   e.g. for use by gij.  Routines are provided to emulate the raw API
   if the underlying platform doesn't allow faster implementation.

   More details on the raw and cloure API can be found in:

   http://gcc.gnu.org/ml/java/1999-q3/msg00138.html

   and

   http://gcc.gnu.org/ml/java/1999-q3/msg00174.html
   -------------------------------------------------------------------- *)

unit ffi;

interface

uses
  ctypes;

{
   from the various ffitarget.h
}

{ ToDo: we need defines for the MIPS ABI }

const
{$if defined(CPUMIPS) or defined(CPU64)}
  FFI_SIZEOF_ARG = 8;
{$else}
  FFI_SIZEOF_ARG = 4;
{$endif}

  FFI_SIZEOF_JAVA_ARG = FFI_SIZEOF_ARG;

{$if defined(CPUPOWERPC) and not defined(AIX) and not defined(DARWIN)}
  {$if defined(CPUPOWERPC32)}
  FFI_SYSV_SOFT_FLOAT = 1;
  FFI_SYSV_STRUCT_RET = 2;
  FFI_SYSV_IBM_LONG_DOUBLE = 4;
  FFI_SYSV_LONG_DOUBLE_128 = 16;
  {$elseif defined(CPUPOWERPC64)}
  FFI_LINUX_STRUCT_ALIGN = 1;
  FFI_LINUX_LONG_DOUBLE_128 = 2;
  {$endif}
{$endif}

type
{$ifdef WIN64}
  ffi_arg = QWord;
  ffi_sarg = Int64;
{$else}
  ffi_arg = culong;
  ffi_sarg = cslong;
{$endif}

  ffi_abi = (
    FFI_FIRST_ABI,
{$if not defined(CPUMIPS) and not defined(CPUX86_64) and not defined(CPUPOWERPC) and not defined(CPUSPARCGEN)}
    FFI_SYSV,
{$endif}
{$if defined(CPUARM)}
    FFI_VFP,
{$endif}
{$if defined(CPUMIPS)}
    FFI_O32,
    FFI_N32,
    FFI_N64,
    FFI_O32_SOFT_FLOAT,
    FFI_N32_SOFT_FLOAT,
    FFI_N64_SOFT_FLOAT,
{$endif}
{$if defined(CPUX86_64)}
  {$ifndef WIN64}
    FFI_UNIX64,
  {$endif}
    FFI_WIN64,
{$endif}
{$if defined(CPUI386)}
  {$ifdef WIN32}
    FFI_STDCALL,
  {$endif}
    FFI_THISCALL = 3,
    FFI_FASTCALL,
  {$ifdef WIN32}
    FFI_MS_CDECL,
  {$else}
    FFI_STDCALL,
  {$endif}
    FFI_PASCAL,
    FFI_REGISTER,
{$endif}
{$if defined(CPUSPARC32)}
    FFI_V8,
{$endif}
{$if defined(CPUSPARC64)}
    FFI_V9,
{$endif}
{$if defined(CPUPOWERPC)}
    { this one is getting ugly... }
  {$if defined(AIX) or defined(DARWIN)}
    FFI_AIX,
    FFI_DARWIN,
  {$else}
    FFI_COMPAT_SYSV,
    FFI_COMPAT_GCC_SYSV,
    FFI_COMPAT_LINUX64,
    FFI_COMPAT_LINUX,
    FFI_COMPAT_LINUX_SOFT_FLOAT,
    {$if defined(CPUPOWERPC64)}
    FFI_LINUX = 8,
    {$define NO_LAST_ABI}
    FFI_LAST_ABI = 12
    {$elseif defined(CPUPOWERPC32)}
    FFI_SYSV = 8,
    {$define NO_LAST_ABI}
    FFI_LAST_ABI = 32
    {$endif}
  {$endif}
{$endif}
{$ifndef NO_LAST_ABI}
    FFI_LAST_ABI
{$endif}
  );

{ alias values }
const
{$if defined(CPUX86_64) and not defined(WIN64)}
  FFI_EFI64 = FFI_WIN64;
{$endif}
{$if defined(CPUARMHF)}
  FFI_DEFAULT_ABI = FFI_VFP;
{$elseif defined(CPUMIPS)}
  { ToDo: needs define for ABI }
  FFI_DEFAULT_ABI = FFI_N32;
{$elseif defined(CPUX86_64)}
  {$ifdef WIN64}
  FFI_DEFAULT_ABI = FFI_WIN64;
  {$else}
  FFI_DEFAULT_ABI = FFI_UNIX64;
  {$endif}
{$elseif defined(CPUSPARC32)}
  FFI_DEFAULT_ABI = FFI_V8;
{$elseif defined(CPUSPARC64)}
  FFI_DEFAULT_ABI = FFI_V9;
{$elseif defined(CPUPOWERPC)}
  {$if defined(AIX)}
  FFI_DEFAULT_ABI = FFI_AIX;
  {$elseif defined(DARWIN)}
  FFI_DEFAULT_ABI = FFI_DARWIN;
  {$elseif defined(CPUPOWERPC64)}
  { ToDo: find out what needs to be set }
  FFI_DEFAULT_ABI = ffi_abi(Ord(FFI_LINUX) or FFI_LINUX_STRUCT_ALIGN or FFI_LINUX_LONG_DOUBLE_128);
  {$elseif defined(CPUPOWERPC)}
  { ToDo: find out what needs to be set }
  FFI_DEFAULT_ABI = ffi_abi(Ord(FFI_SYSV) {$ifdef FREEBSD}or FFI_SYSV_STRUCT_RET{$endif});
  {$endif}
{$else}
  {$ifdef WIN32}
    FFI_DEFAULT_ABI = FFI_STDCALL;
  {$else}
    FFI_DEFAULT_ABI = FFI_SYSV;
  {$endif}
{$endif}

const
{$if defined(CPUPOWERPC)}
  FFI_TARGET_HAS_COMPLEX_TYPE = False;
{$else}
  FFI_TARGET_HAS_COMPLEX_TYPE = True;
{$endif}

(* ---- Definitions for closures ----------------------------------------- *)

const
  FFI_CLOSURES = true;
{$if defined(DARWIN) and (defined(CPUARM) or defined(CPUAARCH64))}
  FFI_EXEC_TRAMPOLINE_TABLE = True;
{$else}
  FFI_EXEC_TRAMPOLINE_TABLE = False;
{$endif}
{$if defined(CPUI386)}
  FFI_NATIVE_RAW_API = True;
{$else}
  FFI_NATIVE_RAW_API = False;
{$endif}

{$if defined(CPUARM) or defined(CPUX86_64) or defined(CPUI386) or defined(CPUSPARCGEN) or (defined(CPUAARCH64) and not defined(DARWIN)) or (defined(CPUPOWERPC) and not defined(DARWIN))}
  FFI_GO_CLOSURES = True;
{$else}
  FFI_GO_CLOSURES = False;
{$endif}

{$if defined(CPUARM)}
  {$if defined(DARWIN)}
  FFI_TRAMPOLINE_SIZE = 12;
  FFI_TRAMPOLINE_CLOSURE_OFFSET = 8;
  {$elseif FFI_EXEC_TRAMPOLINE_TABLE}
    {$error 'No trampoline table implementation'}
  {$else}
  FFI_TRAMPOLINE_SIZE = 12;
  FFI_TRAMPOLINE_CLOSURE_OFFSET = FFI_TRAMPOLINE_SIZE;
  {$endif}
{$elseif defined(CPUAARCH64)}
  {$if defined(DARWIN)}
  FFI_TRAMPOLINE_SIZE =16;
  FFI_TRAMPOLINE_CLOSURE_OFFSET = 16;
  {$elseif FFI_EXEC_TRAMPOLINE_TABLE}
    {$error 'No trampoline table implementation'}
  {$else}
  FFI_TRAMPOLINE_SIZE = 24;
  FFI_TRAMPOLINE_CLOSURE_OFFSET = FFI_TRAMPOLINE_SIZE;
  {$endif}
{$elseif defined(CPUPOWERPC)}
  { ToDo: check for ELFv2? }
  {$ifdef ELF_V2}
  FFI_TRAMPOLINE_SIZE = 32;
  {$else}
    {$if defined(CPUPOWERPC64) or defined(AIX)}
      {$if defined(DARWIN)}
  FFI_TRAMPOLINE_SIZE = 48;
      {$else}
  FFI_TRAMPOLINE_SIZE = 25;
      {$endif}
    {$else}
  FFI_TRAMPOLINE_SIZE = 40;
    {$endif}
  {$endif}
{$elseif defined(CPUSPARC32)}
  FFI_TRAMPOLINE_SIZE = 16;
{$elseif defined(CPUSPARC64)}
  FFI_TRAMPOLINE_SIZE = 24;
{$elseif defined(CPUX86_64)}
  FFI_TRAMPOLINE_SIZE = 24;
{$elseif defined(CPUI386)}
  FFI_TRAMPOLINE_SIZE = 12;
{$elseif defined(CPUM68K)}
  FFI_TRAMPOLINE_SIZE = 16;
{$elseif defined(CPURISCV32)}
  FFI_TRAMPOLINE_SIZE = 24;
{$elseif defined(CPURISCV64)}
  FFI_TRAMPOLINE_SIZE = 24;
{$elseif defined(CPUMIPS32)}
  FFI_TRAMPOLINE_SIZE = 20;
{$elseif defined(CPUMIPS64)}
  FFI_TRAMPOLINE_SIZE = 56;
#endif

{$endif}

{
   from ffi.h
}

const
  ffilibrary = 'ffi';

{$if defined(CPUI8086) or defined(CPUI386) or (defined(CPUX86_64) and not defined(WIN64))}
  { Note: we can not use FPC_HAS_TYPE_EXTENDED here as libffi won't have the
          corresponding type no matter what }
  {$define HAVE_LONG_DOUBLE}
{$endif}

type
  pffi_type = ^ffi_type;
  ppffi_type = ^pffi_type;
  ffi_type = record
    size: csize_t;
    alignment: cushort;
    _type: cushort;
    elements: ppffi_type;
  end;

var
  ffi_type_void: ffi_type; cvar; external ffilibrary;
  ffi_type_uint8: ffi_type; cvar; external ffilibrary;
  ffi_type_sint8: ffi_type; cvar; external ffilibrary;
  ffi_type_uint16: ffi_type; cvar; external ffilibrary;
  ffi_type_sint16: ffi_type; cvar; external ffilibrary;
  ffi_type_uint32: ffi_type; cvar; external ffilibrary;
  ffi_type_sint32: ffi_type; cvar; external ffilibrary;
  ffi_type_uint64: ffi_type; cvar; external ffilibrary;
  ffi_type_sint64: ffi_type; cvar; external ffilibrary;
  ffi_type_float: ffi_type; cvar; external ffilibrary;
  ffi_type_double: ffi_type; cvar; external ffilibrary;
  ffi_type_pointer: ffi_type; cvar; external ffilibrary;
{$ifdef HAVE_LONG_DOUBLE}
  ffi_type_longdouble: ffi_type; cvar; external ffilibrary;
{$else}
  ffi_type_longdouble: ffi_type absolute ffi_type_double;
{$endif}

{$if FFI_TARGET_HAS_COMPLEX_TYPE}
  ffi_type_complex_single: ffi_type; cvar; external ffilibrary;
  ffi_type_complex_double: ffi_type; cvar; external ffilibrary;
  {$ifdef HAVE_LONG_DOUBLE}
  ffi_type_complex_longdouble: ffi_type; cvar; external ffilibrary;
  {$endif}
{$endif}

  { type aliases }

  ffi_type_uchar: ffi_type absolute ffi_type_uint8;
  ffi_type_schar: ffi_type absolute ffi_type_sint8;

  { ToDo: check when C's short isn't 2 byte }
  ffi_type_ushort: ffi_type absolute ffi_type_uint16;
  ffi_type_sshort: ffi_type absolute ffi_type_sint16;

  { ToDo: check when C's int isn't 4 byte }
  ffi_type_uint: ffi_type absolute ffi_type_uint32;
  ffi_type_sint: ffi_type absolute ffi_type_sint32;

{$if defined(CPU64) and not defined(WIN64)}
  ffi_type_ulong: ffi_type absolute ffi_type_uint64;
  ffi_type_slong: ffi_type absolute ffi_type_sint64;
{$else}
  ffi_type_ulong: ffi_type absolute ffi_type_uint32;
  ffi_type_slong: ffi_type absolute ffi_type_sint32;
{$endif}

type
  ffi_status = (
    FFI_OK,
    FFI_BAD_TYPEDEF,
    FFI_BAD_ABI
  );

  pffi_cif = ^ffi_cif;
  ffi_cif = record
    abi: ffi_abi;
    nargs: cunsigned;
    arg_type: ppffi_type;
    rtype: pffi_type;
    bytes: cunsigned;
    flags: cunsigned;
{$if defined(CPUARM)}
    vfp_used: cint;
    vfp_reg_free: cushort;
    vfp_nargs: cushort;
    vfp_args: array[0..15] of cchar;
{$elseif defined(CPUAARCH64)}
  {$ifdef DARWIN}
    aarch64_nfixedargs: cuint;
  {$endif}
{$elseif defined(CPUSPARC64)}
    nfixedargs: cuint;
{$elseif defined(CPUPOWERPC)}
  {$ifndef DARWIN}
    nfixedargs: cuint;
  {$endif}
{$endif}
  end;

  pffi_raw = ^ffi_raw;
  ffi_raw = record
    case longint of
      0: (sint: ffi_sarg);
      1: (uint: ffi_arg);
      2: (flt: cfloat);
      3: (data: array[0..FFI_SIZEOF_ARG] of cchar);
      4: (ptr: Pointer);
  end;

{$if (FFI_SIZEOF_JAVA_ARG = 4) and (FFI_SIZEOF_ARG = 8)}
  (* This is a special case for mips64/n32 ABI (and perhaps others) where
     sizeof(void * ) is 4 and FFI_SIZEOF_ARG is 8.  *)
  ffi_java_raw = record
    case longint of
      0: (sint: ffi_sarg);
      1: (uint: ffi_arg);
      2: (flt: cfloat);
      3: (data: array[0..FFI_SIZEOF_JAVA_ARG] of cchar);
      4: (ptr: Pointer);
  end;
{$else}
  ffi_java_raw = ffi_raw;
{$endif}
  pffi_java_raw = ^ffi_java_raw;

  ffi_fn = procedure;

procedure ffi_raw_call(cif: pffi_cif;
                       fn: ffi_fn;
                       rvalue: Pointer;
                       avalue: pffi_raw); cdecl; external ffilibrary name 'ffi_raw_call';

procedure ffi_ptrarray_to_raw(cif: pffi_cif; args: PPointer; raw: pffi_raw); cdecl; external ffilibrary name 'ffi_ptrarray_to_raw';
procedure ffi_raw_to_ptrarray(cif: pffi_cif; raw: pffi_raw; args: PPointer); cdecl; external ffilibrary name 'ffi_raw_to_ptrarray';
function ffi_raw_size(cif: pffi_cif): csize_t; cdecl; external ffilibrary name 'ffi_raw_size';

(* This is analogous to the raw API, except it uses Java parameter
   packing, even on 64-bit machines.  I.e. on 64-bit machines longs
   and doubles are followed by an empty 64-bit word.  *)

procedure ffi_java_raw_call(cif: pffi_cif;
                            fn: ffi_fn;
                            rvalue: Pointer;
                            avalue: pffi_java_raw); cdecl; external ffilibrary name 'ffi_java_raw_call';

procedure ffi_java_ptrarray_to_raw(cif: pffi_cif; args: PPointer; raw: pffi_java_raw); cdecl; external ffilibrary name 'ffi_java_ptrarray_to_raw';
procedure ffi_java_raw_to_ptrarray(cif: pffi_cif; raw: pffi_java_raw; args: PPointer); cdecl; external ffilibrary name 'ffi_java_raw_to_ptrarray';
function ffi_java_raw_size(cif: pffi_cif): csize_t; cdecl; external ffilibrary name 'ffi_java_raw_size';

(* ---- Definitions for closures ----------------------------------------- *)

{$if FFI_CLOSURES}

type
  ffi_closure_fun = procedure(cif: pffi_cif; arg1: Pointer; arg2: PPointer; arg3: Pointer); cdecl;

  { ToDo: align 8 }
  ffi_closure = record
{$if FFI_EXEC_TRAMPOLINE_TABLE}
    trampoline_table: Pointer;
    trampoline_table_entry: Pointer;
{$else}
    tramp: array[0..FFI_TRAMPOLINE_SIZE] of cchar;
{$endif}
    cif: pffi_cif;
    fun: ffi_closure_fun;
    user_data: Pointer;
  end;
  pffi_closure = ^ffi_closure;

function ffi_closure_alloc(size: csize_t; code: PPointer): Pointer; cdecl; external ffilibrary name 'ffi_closure_alloc';
procedure ffi_closure_free(clo: Pointer); cdecl; external ffilibrary name 'ffi_closure_free';

function ffi_prep_closure(clo: pffi_closure;
                          cif: pffi_cif;
                          fun: ffi_closure_fun;
                          user_data: Pointer): ffi_status; cdecl; external ffilibrary name 'ffi_prep_closure'; deprecated 'use ffi_prep_closure_loc instead';

function ffi_prep_closure_loc(clo: pffi_closure;
                              cif: pffi_cif;
                              fun: ffi_closure_fun;
                              user_data: Pointer;
                              codeloc: Pointer): ffi_status; cdecl; external ffilibrary name 'ffi_prep_closure_loc';

type
  ffi_raw_closure_fun = procedure(cif: pffi_cif; arg1: Pointer; arg2: pffi_raw; arg3: Pointer); cdecl;
  ffi_java_raw_closure_fun = procedure(cif: pffi_cif; arg1: Pointer; arg2: pffi_java_raw; arg3: Pointer); cdecl;

  { ToDo: pack 8 for __sgi aka MIPS? }
  ffi_raw_closure = record
{$if FFI_EXEC_TRAMPOLINE_TABLE}
    trampoline_table: Pointer;
    trampoline_table_entry: Pointer;
{$else}
    tramp: array[0..FFI_TRAMPOLINE_SIZE] of cchar;
{$endif}
    cif: pffi_cif;
{$if not FFI_NATIVE_RAW_API}
    (* If this is enabled, then a raw closure has the same layout
       as a regular closure.  We use this to install an intermediate
       handler to do the transaltion, void** -> ffi_raw*.  *)
    translate_args: ffi_closure_fun;
    this_closure: Pointer;
{$endif}
    fun: ffi_raw_closure_fun;
    user_data: Pointer;
  end;
  pffi_raw_closure = ^ffi_raw_closure;

  { ToDo: pack 8 for __sgi aka MIPS? }
  ffi_java_raw_closure = record
{$if FFI_EXEC_TRAMPOLINE_TABLE}
    trampoline_table: Pointer;
    trampoline_table_entry: Pointer;
{$else}
    tramp: array[0..FFI_TRAMPOLINE_SIZE] of cchar;
{$endif}
    cif: pffi_cif;
{$if not FFI_NATIVE_RAW_API}
    (* If this is enabled, then a raw closure has the same layout
       as a regular closure.  We use this to install an intermediate
       handler to do the transaltion, void** -> ffi_raw*.  *)
    translate_args: ffi_closure_fun;
    this_closure: Pointer;
{$endif}
    fun: ffi_java_raw_closure_fun;
    user_data: Pointer;
  end;
  pffi_java_raw_closure = ^ffi_java_raw_closure;

function ffi_prep_raw_closure(clo: pffi_raw_closure;
                              cif: pffi_cif;
                              fun: ffi_raw_closure_fun;
                              user_data: Pointer): ffi_status; cdecl; external ffilibrary name 'ffi_prep_raw_closure';

function ffi_prep_raw_closure_loc(clo: pffi_raw_closure;
                                  cif: pffi_cif;
                                  fun: ffi_raw_closure_fun;
                                  user_data: Pointer;
                                  codeloc: Pointer): ffi_status; cdecl; external ffilibrary name 'ffi_prep_raw_closure_loc';

function ffi_prep_java_raw_closure(clo: pffi_java_raw_closure;
                              cif: pffi_cif;
                              fun: ffi_java_raw_closure_fun;
                              user_data: Pointer): ffi_status; cdecl; external ffilibrary name 'ffi_prep_java_raw_closure';

function ffi_prep_java_raw_closure_loc(clo: pffi_java_raw_closure;
                                  cif: pffi_cif;
                                  fun: ffi_java_raw_closure_fun;
                                  user_data: Pointer;
                                  codeloc: Pointer): ffi_status; cdecl; external ffilibrary name 'ffi_prep_java_raw_closure_loc';

{$endif}

{$if FFI_GO_CLOSURES}
type
  ffi_go_closure = record
    tramp: Pointer;
    cif: pffi_cif;
    fun: ffi_closure_fun;
  end;
  pffi_go_closure = ^ffi_go_closure;

function ffi_prep_go_closure(clo: pffi_go_closure; cif: pffi_cif; fun: ffi_closure_fun): ffi_status; cdecl; external ffilibrary name 'ffi_prep_go_closure';

procedure ffi_call_go(cif: pffi_cif; fn: ffi_fn; rvalue: Pointer; avalue: PPointer; closure: Pointer); cdecl; external ffilibrary name 'ffi_call_go';

{$endif}

(* ---- Public interface definition -------------------------------------- *)

function ffi_prep_cif(cif: pffi_cif;
                      abi: ffi_abi;
                      nargs: cuint;
                      rtype: pffi_type;
                      atypes: ppffi_type): ffi_status; cdecl; external ffilibrary name 'ffi_prep_cif';

function ffi_prep_cif_var(cif: pffi_cif;
                          abi: ffi_abi;
                          nfixedargs: cuint;
                          ntotalargs: cuint;
                          rtype: pffi_type;
                          atypes: ppffi_type): ffi_status; cdecl; external ffilibrary name 'ffi_prep_cif_var';

procedure ffi_call(cif: pffi_cif;
                   fn: ffi_fn;
                   rvalue: Pointer;
                   avalue: PPointer); cdecl; external ffilibrary name 'ffi_call';

function ffi_get_struct_offsets(abi: ffi_abi; struct_type: pffi_type;
                                offsets: pcsize_t): ffi_status; cdecl; external ffilibrary name 'ffi_get_struct_offsets';

const
  _FFI_TYPE_VOID       = 0;
  _FFI_TYPE_INT        = 1;
  _FFI_TYPE_FLOAT      = 2;
  _FFI_TYPE_DOUBLE     = 3;
{$ifdef HAVE_LONG_DOUBLE}
  _FFI_TYPE_LONGDOUBLE = 4;
{$else}
  _FFI_TYPE_LONGDOUBLE = _FFI_TYPE_DOUBLE;
{$endif}
  _FFI_TYPE_UINT8      = 5;
  _FFI_TYPE_SINT8      = 6;
  _FFI_TYPE_UINT16     = 7;
  _FFI_TYPE_SINT16     = 8;
  _FFI_TYPE_UINT32     = 9;
  _FFI_TYPE_SINT32     = 10;
  _FFI_TYPE_UINT64     = 11;
  _FFI_TYPE_SINT64     = 12;
  _FFI_TYPE_STRUCT     = 13;
  _FFI_TYPE_POINTER    = 14;
  _FFI_TYPE_COMPLEX    = 15;

  _FFI_TYPE_LAST       = _FFI_TYPE_COMPLEX;

  { ToDo: can we do without the platform specific types? }

implementation

end.
