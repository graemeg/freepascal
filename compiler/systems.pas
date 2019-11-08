{
    Copyright (c) 1998-2008 by Florian Klaempfl

    This unit contains information about the target systems supported
    (these are not processor specific)

    This program is free software; you can redistribute it and/or modify
    iu under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge- MA 02139, USA.

 ****************************************************************************
}
unit systems;

{$i fpcdefs.inc}

interface

{$i systems.inc}

{*****************************************************************************
                               Structures
*****************************************************************************}

     type
       TAbstractResourceFile = class
         constructor create(const fn : ansistring);virtual;abstract;
       end;
       TAbstractResourceFileClass = class of TAbstractResourceFile;


       palignmentinfo = ^talignmentinfo;
       { this is written to ppus during token recording for generics so it must be packed }
       talignmentinfo = packed record
         procalign,
         loopalign,
         { alignment for labels after unconditional jumps, this must be a power of two }
         jumpalign,
         { max. alignment for labels after unconditional jumps:
           the compiler tries to align jumpalign, however, to do so it inserts at maximum jumpalignskipmax bytes or uses
           the next smaller power of two of jumpalign }
         jumpalignskipmax,
         { alignment for labels where two flows of the program flow coalesce, this must be a power of two }
         coalescealign,
         { max. alignment for labels where two flows of the program flow coalesce
           the compiler tries to align to coalescealign, however, to do so it inserts at maximum coalescealignskipmax bytes or uses
           the next smaller power of two of coalescealign }
         coalescealignskipmax,
         constalignmin,
         constalignmax,
         varalignmin,
         varalignmax,
         localalignmin,
         localalignmax,
         recordalignmin,
         recordalignmax,
         maxCrecordalign : longint;
       end;

       tasmflags = (af_none
         ,af_outputbinary
         ,af_needar
         ,af_smartlink_sections
         ,af_labelprefix_only_inside_procedure
         ,af_supports_dwarf
         ,af_no_debug
         ,af_stabs_use_function_absolute_addresses
         ,af_no_stabs
       );

       pasminfo = ^tasminfo;
       tasminfo = record
          id          : tasm;
          idtxt       : string[12];
          asmbin      : string[16];
          asmcmd      : string[70];
          supported_targets : set of tsystem;
          flags        : set of tasmflags;
          labelprefix : string[3];
          comment     : string[3];
          { set to '$' if that character is allowed in symbol names, otherwise
            to alternate character by which '$' should be replaced }
          dollarsign  : char;
       end;

       parinfo = ^tarinfo;
       tarinfo = record
          id          : tar;
          addfilecmd  : string[10];
          arfirstcmd  : string[50];
          arcmd       : string[50];
          arfinishcmd : string[10];
       end;

       presinfo = ^tresinfo;
       tresinfo = record
          id      : tres;
          { Compiler for resource (.rc or .res) to obj }
          resbin  : string[10];
          rescmd  : string[50];
          { Optional compiler for resource script (.rc) to binary resource (.res). }
          { If it is not provided resbin and rescmd will be used.                 }
          rcbin   : string[10];
          rccmd   : string[50];
          resourcefileclass : TAbstractResourceFileClass;
          resflags : set of tresinfoflags;
       end;

       pdbginfo = ^tdbginfo;
       tdbginfo = record
          id      : tdbg;
          idtxt   : string[12];
       end;

       tsystemflags = (tf_none,
            tf_under_development,
            tf_need_export,
            tf_needs_isconsole,
            tf_code_small,
            tf_static_reg_based,
            tf_needs_symbol_size,
            tf_smartlink_sections,
            tf_smartlink_library,
            tf_needs_dwarf_cfi,
            tf_use_8_3,
            tf_pic_uses_got,
            tf_library_needs_pic,
            tf_needs_symbol_type,
            tf_section_threadvars,
            tf_files_case_sensitive,
            tf_files_case_aware,
            tf_p_ext_support,
            tf_has_dllscanner,
            tf_use_function_relative_addresses,
            tf_winlikewidestring,
            tf_dwarf_relative_addresses,         // use offsets where the Dwarf spec requires this instead of absolute addresses (the latter is needed by Linux binutils)
            tf_dwarf_only_local_labels,          // only use local labels inside the Dwarf debug_info section (needed for e.g. Darwin)
            tf_requires_proper_alignment,
            tf_no_pic_supported,
            tf_pic_default,
            { the os does some kind of stack checking and it can be converted into a rte 202 }
            tf_no_generic_stackcheck,
            tf_emit_stklen,                     // Means that the compiler should emit a _stklen variable with the stack size, even if tf_no_generic_stackcheck is specified
            tf_has_winlike_resources,
            tf_safecall_clearstack,             // With this flag set, after safecall calls the caller cleans up the stack
            tf_safecall_exceptions,             // Exceptions in safecall calls are not raised, but passed to the caller as an ordinal (hresult) in the function result.
                                                // The original result (if it exists) is passed as an extra parameter
            tf_no_backquote_support,
            { do not generate an object file when smartlinking is turned on,
              this is usefull for architectures which require a small code footprint }
            tf_no_objectfiles_when_smartlinking,
            { indicates that the default value of the ts_cld target switch is 'on' for this target }
            tf_cld,
            { indicates that the default value of the ts_x86_far_procs_push_odd_bp target switch is 'on' for this target }
            tf_x86_far_procs_push_odd_bp,
            { indicates that this target can use dynamic packages otherwise an
              error will be generated if a package file is compiled }
            tf_supports_packages,
            { use PSABI/Dwarf-based "zero cost" exception handling }
            tf_use_psabieh,
            { use high level cfi directives to generate call frame information }
            tf_use_hlcfi,
            { supports symbol order file (to ensure symbols in vectorised sections are kept in the correct order) }
            tf_supports_symbolorderfile,
            { supports hidden/private extern symbols: visible across object files, but local/private in exe/library }
            tf_supports_hidden_symbols
       );

       psysteminfo = ^tsysteminfo;
       { using packed causes bus errors on processors which require alignment }
       tsysteminfo = record
          system       : tsystem;
          name         : string[34];
          shortname    : string[9];
          flags        : set of tsystemflags;
          cpu          : tsystemcpu;
          unit_env     : string[16];
          extradefines : string[40];
          exeext,
          defext,
          scriptext,
          smartext,
          unitext,
          unitlibext,
          asmext       : string[4];
          objext       : string[6];
          resext       : string[4];
          resobjext    : string[7];
          sharedlibext : string[10];
          staticlibext,
          staticlibprefix : string[4];
          sharedlibprefix : string[4];
          sharedClibext : string[10];
          staticClibext,
          staticClibprefix : string[4];
          sharedClibprefix : string[4];
          importlibprefix : string[10];
          importlibext : string[4];
          Cprefix      : string[2];
          newline      : string[2];
          dirsep       : char;
          assem        : tasm;
          assemextern  : tasm; { external assembler, used by -a }
          link         : tlink;
          linkextern   : tlink;  { external linker, used by -s }
          ar           : tar;
          res          : tres;
          dbg          : tdbg;
          script       : tscripttype;
          endian       : tendian;
          alignment    : talignmentinfo;
          {
            Offset from the argument pointer register to the first
            argument's address. On some machines it may depend on
            the data type of the function.
            (see also FIRST_PARM_OFFSET in GCC source)
          }
          first_parm_offset : longint;
          stacksize    : longint;
          { stack alignment }
          stackalign   : byte;
          abi          : tabi;
          { llvm -- varies wildly in length and is empty for many targets ->
            ansistring instead of shortstring; tsysteminfo records aren't
            copied very often anyway. These strings come from the file
            lib/Basic/Targets.cpp in the clang (cfe 3.3) source tree, sometimes
            adapted to match our (custom) stack alignment requirements }
          llvmdatalayout: ansistring;
       end;

    tabiinfo = record
      name: string[13];
      supported: boolean;
    end;

    const
       { alias for supported_target field in tasminfo }
       system_any = system_none;

       systems_wince = [system_arm_wince,system_i386_wince];
       systems_android = [system_arm_android, system_aarch64_android, system_i386_android, system_x86_64_android, system_mipsel_android];
       systems_linux = [system_i386_linux,system_x86_64_linux,system_powerpc_linux,system_powerpc64_linux,
                       system_arm_linux,system_sparc_linux,system_sparc64_linux,system_m68k_linux,
                       system_x86_6432_linux,system_mipseb_linux,system_mipsel_linux,system_aarch64_linux,
                       system_riscv32_linux,system_riscv64_linux];
       systems_dragonfly = [system_x86_64_dragonfly];
       systems_freebsd = [system_i386_freebsd,
                          system_x86_64_freebsd];
       systems_netbsd  = [system_i386_netbsd,
                          system_m68k_netbsd,
                          system_powerpc_netbsd,
                          system_x86_64_netbsd,
                          system_arm_netbsd];
       systems_openbsd = [system_i386_openbsd,
                          system_x86_64_openbsd];

       systems_bsd = systems_freebsd + systems_netbsd + systems_openbsd + systems_dragonfly;

       systems_aix = [system_powerpc_aix,system_powerpc64_aix];

       { all real windows systems, no cripple ones like win16, wince, wdosx et. al. }
       systems_windows = [system_i386_win32,system_x86_64_win64];

       { all windows systems }
       systems_all_windows = [system_i386_win32,system_x86_64_win64,
                             system_arm_wince,system_i386_wince,
                             system_i8086_win16];

       { all darwin systems }
       systems_darwin = [system_powerpc_darwin,system_i386_darwin,
                         system_powerpc64_darwin,system_x86_64_darwin,
                         system_arm_darwin,system_i386_iphonesim,
                         system_aarch64_darwin,system_x86_64_iphonesim];

       {all solaris systems }
       systems_solaris = [system_sparc_solaris, system_i386_solaris,
                          system_x86_64_solaris];

       { all embedded systems }
       systems_embedded = [system_i386_embedded,system_m68k_embedded,
                           system_powerpc_embedded,
                           system_sparc_embedded,obsolete_system_vm_embedded,
                           obsolete_system_ia64_embedded,system_x86_64_embedded,
                           system_mips_embedded,system_arm_embedded,
                           system_powerpc64_embedded,system_avr_embedded,
                           system_jvm_java32,system_mipseb_embedded,system_mipsel_embedded,
                           system_i8086_embedded,system_riscv32_embedded,system_riscv64_embedded];

       { all systems that allow section directive }
       systems_allow_section = systems_embedded;

       { systems that uses dotted function names as descriptors }
       systems_dotted_function_names = [system_powerpc64_linux]+systems_aix;

       systems_allow_section_no_semicolon = systems_allow_section
{$ifndef DISABLE_TLS_DIRECTORY}
       + systems_windows
{$endif not DISABLE_TLS_DIRECTORY}
       ;

       { systems that allow external far variables }
       systems_allow_external_far_var = [system_i8086_msdos,system_i8086_win16,system_i8086_embedded];

       { all symbian systems }
       systems_symbian = [system_i386_symbian,system_arm_symbian];

       { all classic Mac OS targets }
       systems_macos = [system_m68k_macos,system_powerpc_macos];

       { all OS/2 targets }
       systems_os2 = [system_i386_OS2,system_i386_emx];

       { AROS systems }
       systems_aros = [system_i386_aros,system_x86_64_aros,system_arm_aros];

       { all amiga like systems }
       systems_amigalike = [system_m68k_amiga,system_powerpc_morphos,system_powerpc_amiga]+systems_aros;

       { all native nt systems }
       systems_nativent = [system_i386_nativent];

       { systems supporting Objective-C }
       systems_objc_supported = systems_darwin;

       { systems using the non-fragile Objective-C ABI }
       systems_objc_nfabi = [system_powerpc64_darwin,system_x86_64_darwin,system_arm_darwin,system_i386_iphonesim,system_aarch64_darwin,system_x86_64_iphonesim];

       { systems supporting "blocks" }
       systems_blocks_supported = systems_darwin;

       { all systems supporting exports from programs or units }
       systems_unit_program_exports = [system_i386_win32,
                                         system_i386_wdosx,
                                         system_i386_Netware,
                                         system_i386_netwlibc,
                                         system_arm_wince,
                                         system_x86_64_win64,
                                         system_i8086_win16]+systems_linux+systems_android;

       { all systems that reference symbols in other binaries using indirect imports }
       systems_indirect_var_imports = systems_all_windows+[system_i386_nativent];

       { all systems that support indirect entry information }
       systems_indirect_entry_information = systems_darwin+[system_i386_win32,system_x86_64_win64,system_x86_64_linux];

       { all systems for which weak linking has been tested/is supported }
       systems_weak_linking = systems_darwin + systems_solaris + systems_linux + systems_android + systems_openbsd;

       systems_internal_sysinit = [system_i386_win32,system_x86_64_win64,
                                   system_i386_linux,system_powerpc64_linux,system_sparc64_linux,system_x86_64_linux,
                                   system_m68k_atari,system_m68k_palmos,
                                   system_i386_haiku,system_x86_64_haiku,
                                   system_i386_openbsd,system_x86_64_openbsd,
                                   system_riscv32_linux,system_riscv64_linux
                                  ]+systems_darwin+systems_amigalike;

       { all systems that use garbage collection for reference-counted types }
       systems_garbage_collected_managed_types = [
         system_jvm_java32,
         system_jvm_android32
       ];

       { all systems that use a managed vm (-> no real pointers, internal VMT
         format, ...) }
       systems_managed_vm = [
         system_jvm_java32,
         system_jvm_android32
       ];

       { all systems based on the JVM }
       systems_jvm = [
         system_jvm_java32,
         system_jvm_android32
       ];

       { all systems where typed constants have to be translated into node
         trees that initialise the data instead of into data sections }
       systems_typed_constants_node_init = [
         system_jvm_java32,
         system_jvm_android32
       ];

       { all systems that don't use a built-in framepointer for accessing nested
         variables, but emulate it by wrapping nested variables in records
         whose address is passed around }
       systems_fpnestedstruct = [
{$ifndef llvm}
         system_jvm_java32,
         system_jvm_android32
{$else not llvm}
         low(tsystem)..high(tsystem)
{$endif not llvm}
       ];

       { all systems where a value parameter passed by reference must be copied
         on the caller side rather than on the callee side }
       systems_caller_copy_addr_value_para = [system_aarch64_darwin,system_aarch64_linux];

       { pointer checking (requires special code in FPC_CHECKPOINTER,
         and can never work for libc-based targets or any other program
         linking to an external library)
       }
       systems_support_checkpointer = systems_linux
                             + [system_i386_win32]
                             + [system_i386_GO32V2]
                             + [system_i386_os2]
                             + [system_i386_beos,system_i386_haiku]
                             + [system_powerpc_morphos];

       cpu2str : array[TSystemCpu] of string[10] =
            ('','i386','m68k','alpha','powerpc','sparc','vm','ia64','x86_64',
             'mips','arm', 'powerpc64', 'avr', 'mipsel','jvm', 'i8086',
             'aarch64', 'wasm', 'sparc64','riscv32','riscv64');

       abiinfo : array[tabi] of tabiinfo = (
         (name: 'DEFAULT'; supported: true),
         (name: 'SYSV'   ; supported:{$if defined(powerpc) or defined(powerpc64)}true{$else}false{$endif}),
         (name: 'AIX'    ; supported:{$if defined(powerpc) or defined(powerpc64)}true{$else}false{$endif}),
         (name: 'DARWIN'    ; supported:{$if defined(powerpc) or defined(powerpc64)}true{$else}false{$endif}),
         (name: 'ELFV2'  ; supported:{$if defined(powerpc64)}true{$else}false{$endif}),
         (name: 'EABI'   ; supported:{$ifdef FPC_ARMEL}true{$else}false{$endif}),
         (name: 'ARMEB'  ; supported:{$ifdef FPC_ARMEB}true{$else}false{$endif}),
         (name: 'EABIHF' ; supported:{$ifdef FPC_ARMHF}true{$else}false{$endif}),
         (name: 'OLDWIN32GNU'; supported:{$ifdef I386}true{$else}false{$endif}),
         (name: 'AARCH64IOS'; supported:{$ifdef aarch64}true{$else}false{$endif}),
         (name: 'RISCVHF'; supported:{$if defined(riscv32) or defined(riscv64)}true{$else}false{$endif}),
         (name: 'LINUX386_SYSV'; supported:{$if defined(i386)}true{$else}false{$endif})
       );

       { x86 asm modes with an Intel-style syntax }
       asmmodes_x86_intel = [
{$ifdef i8086}
         asmmode_standard,
{$endif i8086}
         asmmode_i8086_intel,
         asmmode_i386_intel,
         asmmode_x86_64_intel
       ];

       { x86 asm modes with an AT&T-style syntax }
       asmmodes_x86_att = [
{$if defined(i386) or defined(x86_64)}
         asmmode_standard,
{$endif}
         asmmode_i8086_att,
         asmmode_i386_att,
         asmmode_x86_64_att,
         asmmode_x86_64_gas
       ];

    var
       targetinfos   : array[tsystem] of psysteminfo;
       arinfos       : array[tar] of parinfo;
       resinfos      : array[tres] of presinfo;
       asminfos      : array[tasm] of pasminfo;
       dbginfos      : array[tdbg] of pdbginfo;

       source_info : tsysteminfo;
       target_cpu  : tsystemcpu;
       target_info : tsysteminfo;
       target_asm  : tasminfo;
       target_ar   : tarinfo;
       target_res  : tresinfo;
       target_dbg  : tdbginfo;
       target_cpu_string,
       target_os_string   : string[12]; { for rtl/<X>/,fcl/<X>/, etc. }
       target_full_string : string[24];

    function set_target(t:tsystem):boolean;
    function set_target_asm(t:tasm):boolean;
    function set_target_ar(t:tar):boolean;
    function set_target_res(t:tres):boolean;
    function set_target_dbg(t:tdbg):boolean;

    function find_system_by_string(const s : string) : tsystem;
    function find_asm_by_string(const s : string) : tasm;
    function find_dbg_by_string(const s : string) : tdbg;

    procedure set_source_info(const ti : tsysteminfo);

    function UpdateAlignment(var d:talignmentinfo;const s:talignmentinfo) : boolean;

    procedure RegisterTarget(const r:tsysteminfo);
    procedure RegisterRes(const r:tresinfo; rcf : TAbstractResourceFileClass);
    procedure RegisterAr(const r:tarinfo);

    procedure InitSystems;

    {$ifdef FreeBSD}
        function GetOSRelDate:Longint;
    {$endif}

implementation

    uses
      cutils{$ifdef FreeBSD},SysCtl,BaseUnix{$endif};

{****************************************************************************
           OS runtime version detection utility routine
****************************************************************************}

{$ifdef FreeBSD}
function GetOSRelDate:Longint;

{ FPSysCtl first argument was of type pchar
  up to commit 35566 from 2017/03/11 
  and corrected to pcint in that commit.
  But the following code needs to work with
  both old 3.0.X definition and new definition using pcint type.
  Problem solved using a special type called
  FPSysCtlFirstArgType. }
{$if defined(VER3_0_0) or defined(VER3_0_2)}  
type
  FPSysCtlFirstArgType = PChar;
{$else}
type
  FPSysCtlFirstArgType = pcint;
{$endif}  

var
        mib  : array[0..1] of cint;
        rval : cint;
        len  : size_t;
        i    : longint;
        v    : longint;
        oerrno : cint;
        S    : AnsiString;

Begin
        s:='ab';
        SetLength(S,50);
        mib[0] := CTL_KERN;
        mib[1] := KERN_OSRELDATE;
        len    := 4;
        oerrno:= fpgeterrno;
        if (FPsysctl(FPSysCtlFirstArgType(@mib), 2, pchar(@v), @len, NIL, 0) = -1) Then
             Begin
                if (fpgeterrno = ESysENOMEM) Then
                        fpseterrno(oerrno);
                GetOSRelDate:=0;
           End
        else
         GetOSRelDate:=v;
End;
{$endif}


{****************************************************************************
                              Target setting
****************************************************************************}

function set_target(t:tsystem):boolean;
begin
  set_target:=false;
  if assigned(targetinfos[t]) then
   begin
     target_info:=targetinfos[t]^;
     set_target_asm(target_info.assem);
     set_target_ar(target_info.ar);
     set_target_res(target_info.res);
     set_target_dbg(target_info.dbg);
     target_cpu:=target_info.cpu;
     target_os_string:=lower(target_info.shortname);
     target_cpu_string:=cpu2str[target_cpu];
     target_full_string:=target_cpu_string+'-'+target_os_string;
     set_target:=true;
     exit;
   end;
end;


function set_target_asm(t:tasm):boolean;
begin
  set_target_asm:=false;
  if assigned(asminfos[t]) and
    ((target_info.system in asminfos[t]^.supported_targets) or
     (system_any in asminfos[t]^.supported_targets)) then
   begin
     target_asm:=asminfos[t]^;
     set_target_asm:=true;
     exit;
   end;
end;


function set_target_ar(t:tar):boolean;
begin
  result:=false;
  if assigned(arinfos[t]) then
   begin
     target_ar:=arinfos[t]^;
     result:=true;
     exit;
   end;
end;


function set_target_res(t:tres):boolean;
begin
  result:=false;
  if assigned(resinfos[t]) then
   begin
     target_res:=resinfos[t]^;
     result:=true;
     exit;
   end
  else
   FillByte(target_res,sizeof(target_res),0);
end;


function set_target_dbg(t:tdbg):boolean;
begin
  result:=false;
{ no debugging support for llvm yet }
{$ifndef llvm}
  if assigned(dbginfos[t]) then
   begin
     target_dbg:=dbginfos[t]^;
     result:=true;
     exit;
   end;
{$endif}
end;


function find_system_by_string(const s : string) : tsystem;
var
  hs : string;
  t  : tsystem;
begin
  result:=system_none;
  hs:=upper(s);
  for t:=low(tsystem) to high(tsystem) do
   if assigned(targetinfos[t]) and
      (upper(targetinfos[t]^.shortname)=hs) then
    begin
      result:=t;
      exit;
    end;
end;


function find_asm_by_string(const s : string) : tasm;
var
  hs : string;
  t  : tasm;
begin
  result:=as_none;
  hs:=upper(s);
  for t:=low(tasm) to high(tasm) do
   if assigned(asminfos[t]) and
      (asminfos[t]^.idtxt=hs) then
    begin
      result:=t;
      exit;
    end;
end;


function find_dbg_by_string(const s : string) : tdbg;
var
  hs : string;
  t  : tdbg;
begin
  result:=dbg_none;
  hs:=upper(s);
  for t:=low(tdbg) to high(tdbg) do
   if assigned(dbginfos[t]) and
      (dbginfos[t]^.idtxt=hs) then
    begin
      result:=t;
      exit;
    end;
end;


function UpdateAlignment(var d:talignmentinfo;const s:talignmentinfo) : boolean;
begin
  result:=true;
  with d do
   begin
     if (s.procalign in [1,2,4,8,16,32,64,128]) or (s.procalign=256) then
       procalign:=s.procalign
     else if s.procalign<>0 then
       result:=false;
     if (s.loopalign in [1,2,4,8,16,32,64,128]) or (s.loopalign=256) then
       loopalign:=s.loopalign
     else if s.loopalign<>0 then
       result:=false;
     if (s.jumpalign in [1,2,4,8,16,32,64,128]) or (s.jumpalign=256) then
       jumpalign:=s.jumpalign
     else if s.jumpalign<>0 then
       result:=false;
     if (s.coalescealign in [1,2,4,8,16,32,64,128]) or (s.coalescealign=256) then
       coalescealign:=s.coalescealign
     else if s.coalescealign<>0 then
       result:=false;
     if s.jumpalignskipmax>0 then
       jumpalignskipmax:=s.jumpalignskipmax;
     if s.coalescealign>0 then
       coalescealignskipmax:=s.coalescealignskipmax;
     { general update rules:
       minimum: if higher then update
       maximum: if lower then update or if undefined then update }
     if s.constalignmin>constalignmin then
      constalignmin:=s.constalignmin;
     if (constalignmax=0) or
        ((s.constalignmax>0) and (s.constalignmax<constalignmax)) then
      constalignmax:=s.constalignmax;
     if s.varalignmin>varalignmin then
      varalignmin:=s.varalignmin;
     if (varalignmax=0) or
        ((s.varalignmax>0) and (s.varalignmax<varalignmax)) then
      varalignmax:=s.varalignmax;
     if s.localalignmin>localalignmin then
      localalignmin:=s.localalignmin;
     if (localalignmax=0) or
        ((s.localalignmax>0) and (s.localalignmax<localalignmax)) then
      localalignmax:=s.localalignmax;
     if s.recordalignmin>recordalignmin then
      recordalignmin:=s.recordalignmin;
     if (recordalignmax=0) or
        ((s.recordalignmax>0) and (s.recordalignmax<recordalignmax)) then
      recordalignmax:=s.recordalignmax;
     if (maxCrecordalign=0) or
        ((s.maxCrecordalign>0) and (s.maxCrecordalign<maxCrecordalign)) then
      maxCrecordalign:=s.maxCrecordalign;
   end;
end;


{****************************************************************************
                              Target registration
****************************************************************************}

procedure RegisterTarget(const r:tsysteminfo);
var
  t : tsystem;
begin
  t:=r.system;
  if assigned(targetinfos[t]) then
   writeln('Warning: Target is already registered!')
  else
   new(targetinfos[t]);
  targetinfos[t]^:=r;
end;


procedure RegisterRes(const r:tresinfo; rcf : TAbstractResourceFileClass);
var
  t : tres;
begin
  t:=r.id;
  if not assigned(resinfos[t]) then
    new(resinfos[t]);
  resinfos[t]^:=r;
  resinfos[t]^.resourcefileclass:=rcf;
end;


procedure RegisterAr(const r:tarinfo);
var
  t : tar;
begin
  t:=r.id;
  if assigned(arinfos[t]) then
    writeln('Warning: ar is already registered!')
  else
    new(arinfos[t]);
  arinfos[t]^:=r;
end;



procedure DeregisterInfos;
var
  assem   : tasm;
  target  : tsystem;
  ar      : tar;
  res     : tres;
  dbg     : tdbg;
begin
  for target:=low(tsystem) to high(tsystem) do
   if assigned(targetinfos[target]) then
    begin
      freemem(targetinfos[target],sizeof(tsysteminfo));
      targetinfos[target]:=nil;
    end;
  for assem:=low(tasm) to high(tasm) do
   if assigned(asminfos[assem]) then
    begin
      freemem(asminfos[assem],sizeof(tasminfo));
      asminfos[assem]:=nil;
    end;
  for ar:=low(tar) to high(tar) do
   if assigned(arinfos[ar]) then
    begin
      freemem(arinfos[ar],sizeof(tarinfo));
      arinfos[ar]:=nil;
    end;
  for res:=low(tres) to high(tres) do
   if assigned(resinfos[res]) then
    begin
      freemem(resinfos[res],sizeof(tresinfo));
      resinfos[res]:=nil;
    end;
  for dbg:=low(tdbg) to high(tdbg) do
   if assigned(dbginfos[dbg]) then
    begin
      freemem(dbginfos[dbg],sizeof(tdbginfo));
      dbginfos[dbg]:=nil;
    end;
end;


{****************************************************************************
                      Initialization of default target
****************************************************************************}

procedure default_target(t:tsystem);
begin
  set_target(t);
  if source_info.name='' then
    source_info:=target_info;
end;


procedure set_source_info(const ti : tsysteminfo);
begin
{ can't use message() here (PFV) }
  if source_info.name<>'' then
    Writeln('Warning: Source OS Redefined!');
  source_info:=ti;
end;


procedure InitSystems;
begin
{ Now default target, this is dependent on the target cpu define,
  when the define is the same as the source cpu then we use the source
  os, else we pick a default }
{$ifdef i386}
  {$ifdef cpui386}
    default_target(source_info.system);
    {$define default_target_set}
  {$else cpui386}
   {$ifdef linux}
    default_target(system_i386_linux);
    {$define default_target_set}
   {$endif}
   {$ifdef freebsd}
    default_target(system_i386_freebsd);
    {$define default_target_set}
   {$endif}
   {$ifdef openbsd}
    default_target(system_i386_openbsd);
    {$define default_target_set}
   {$endif}
   {$ifdef netbsd}
    default_target(system_i386_netbsd);
    {$define default_target_set}
   {$endif}
   {$ifdef darwin}
    default_target(system_i386_darwin);
    {$define default_target_set}
   {$endif}
   {$ifdef android}
    {$define default_target_set}
    default_target(system_i386_android);
   {$endif}
   {$ifdef solaris}
    {$define default_target_set}
    default_target(system_i386_solaris);
   {$endif}
  {$endif cpui386}
  { default is linux }
  {$ifndef default_target_set}
   default_target(system_i386_linux);
  {$endif default_target_set}
{$endif i386}

{$ifdef x86_64}
  {$ifdef cpux86_64}
    default_target(source_info.system);
    {$define default_target_set}
  {$else cpux86_64}
   {$ifdef MSWindows}
    default_target(system_x86_64_win64);
    {$define default_target_set}
   {$endif}
   {$ifdef linux}
    default_target(system_x86_64_linux);
    {$define default_target_set}
   {$endif}
   {$ifdef dragonfly}
    default_target(system_x86_64_dragonfly);
    {$define default_target_set}
   {$endif}
   {$ifdef freebsd}
    default_target(system_x86_64_freebsd);
    {$define default_target_set}
   {$endif}
   {$ifdef openbsd}
    default_target(system_x86_64_openbsd);
    {$define default_target_set}
   {$endif}
   {$ifdef netbsd}
    default_target(system_x86_64_netbsd);
    {$define default_target_set}
   {$endif}
   {$ifdef solaris}
    default_target(system_x86_64_solaris);
    {$define default_target_set}
   {$endif}
   {$ifdef darwin}
    default_target(system_x86_64_darwin);
    {$define default_target_set}
   {$endif}
  {$endif cpux86_64}
  { default is linux }
  {$ifndef default_target_set}
   default_target(system_x86_64_linux);
  {$endif default_target_set}
{$endif x86_64}

{$ifdef m68k}
  {$ifdef cpu68}
    default_target(source_info.system);
  {$else cpu68}
    default_target(system_m68k_linux);
  {$endif cpu68}
{$endif m68k}

{$ifdef powerpc}
  {$ifdef cpupowerpc32}
    default_target(source_info.system);
    {$define default_target_set}
  {$else cpupowerpc}
   {$ifdef linux}
    default_target(system_powerpc_linux);
    {$define default_target_set}
   {$endif}
   {$ifdef darwin}
    default_target(system_powerpc_darwin);
    {$define default_target_set}
   {$endif}
  {$endif cpupowerpc}
  {$ifdef aix}
   default_target(system_powerpc_aix);
   {$define default_target_set}
  {$endif}
  {$ifdef android}
   {$define default_target_set}
   default_target(system_x86_64_android);
  {$endif}
  {$ifndef default_target_set}
    default_target(system_powerpc_linux);
  {$endif default_target_set}
{$endif powerpc}

{$ifdef POWERPC64}
  {$ifdef cpupowerpc64}
    default_target(source_info.system);
    {$define default_target_set}
  {$else cpupowerpc64}
    {$ifdef darwin}
     default_target(system_powerpc64_darwin);
     {$define default_target_set}
    {$endif}
    {$ifdef linux}
     default_target(system_powerpc64_linux);
     {$define default_target_set}
    {$endif}
    {$ifdef aix}
     default_target(system_powerpc64_aix);
     {$define default_target_set}
    {$endif}
  {$endif cpupowerpc64}
  {$ifndef default_target_set}
    default_target(system_powerpc64_linux);
    {$define default_target_set}
  {$endif}
{$endif POWERPC64}

{$ifdef sparc}
  {$ifdef cpusparc}
    default_target(source_info.system);
  {$else cpusparc}
   {$ifdef solaris}
    {$define default_target_set}
    default_target(system_sparc_solaris);
   {$endif}
    {$ifndef default_target_set}
    default_target(system_sparc_linux);
    {$endif ndef default_target_set}
  {$endif cpusparc}
{$endif sparc}

{$ifdef sparc64}
  {$ifdef cpusparc64}
    default_target(source_info.system);
  {$else cpusparc64}
   {$ifdef solaris}
    {$define default_target_set}
    default_target(system_sparc64_solaris);
   {$endif}
    {$ifndef default_target_set}
    default_target(system_sparc64_linux);
    {$endif ndef default_target_set}
  {$endif cpusparc64}
{$endif sparc64}

{$ifdef arm}
  {$ifdef cpuarm}
    default_target(source_info.system);
  {$else cpuarm}
    {$ifdef WINDOWS}
      {$define default_target_set}
      default_target(system_arm_wince);
    {$endif}
    {$ifdef linux}
      {$define default_target_set}
      default_target(system_arm_linux);
    {$endif}
    {$ifdef netbsd}
      {$define default_target_set}
      default_target(system_arm_netbsd);
    {$endif}
    {$ifdef android}
      {$define default_target_set}
      default_target(system_arm_android);
    {$endif}
    {$ifdef darwin}
      {$define default_target_set}
      default_target(system_arm_darwin);
    {$endif}
    {$ifndef default_target_set}
      default_target(system_arm_linux);
      {$define default_target_set}
    {$endif}
  {$endif cpuarm}
{$endif arm}

{$ifdef avr}
  default_target(system_avr_embedded);
{$endif avr}

{$ifdef mips}
{$ifdef mipsel}
  {$ifdef cpumipsel}
    default_target(source_info.system);
  {$else cpumipsel}
    default_target(system_mipsel_linux);
  {$endif cpumipsel}
{$else mipsel}
  default_target(system_mipseb_linux);
{$endif mipsel}
{$endif mips}

{$ifdef jvm}
  default_target(system_jvm_java32);
{$endif jvm}

{$ifdef i8086}
  default_target(system_i8086_msdos);
{$endif i8086}

{$ifdef aarch64}
  {$ifdef cpuaarch64}
    default_target(source_info.system);
  {$else cpuaarch64}
    {$ifdef darwin}
      {$define default_target_set}
      default_target(system_aarch64_darwin);
    {$endif darwin}
    {$ifdef android}
      {$define default_target_set}
      default_target(system_aarch64_android);
    {$endif android}
    {$ifndef default_target_set}
      default_target(system_aarch64_linux);
      {$define default_target_set}
    {$endif}
  {$endif cpuaarch64}
{$endif aarch64}

{$ifdef wasm}
  default_target(system_wasm_wasm32);
{$endif}

{$ifdef riscv32}
  default_target(system_riscv32_linux);
{$endif riscv32}

{$ifdef riscv64}
  default_target(system_riscv64_linux);
{$endif riscv64}
end;


initialization
   source_info.name:='';
finalization
  DeregisterInfos;
end.
