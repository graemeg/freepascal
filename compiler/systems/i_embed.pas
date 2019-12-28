{
    This unit implements support information structures for the FPC Embedded target

    Copyright (c) 1998-2006 by Peter Vreman

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ****************************************************************************
}
unit i_embed;

{$i fpcdefs.inc}

{$ifdef go32v2}
  { As wlib uses a different Dos-Extender, long-command line
    encoding for DJGPP does not work here.
    Put all inside a script file instead }
  {$define USE_SCRIPTED_WLIB}
{$endif}

  interface

    uses
       systems;

    const
       system_arm_embedded_info : tsysteminfo =
          (
            system       : system_arm_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_files_case_sensitive,tf_requires_proper_alignment,
                            tf_smartlink_sections,tf_init_final_units_by_calls];
            cpu          : cpu_arm;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 4;
                varalignmin     : 0;
                varalignmax     : 4;
                localalignmin   : 4;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 262144;
            stackalign   : 4;
            abi : abi_default;
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:64:128-a0:0:64-n32-S32';
          );

       system_avr_embedded_info : tsysteminfo =
          (
            system       : system_avr_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_files_case_sensitive,
                            tf_smartlink_sections,tf_init_final_units_by_calls];
            cpu          : cpu_avr;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_dwarf3;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 1;
                loopalign       : 1;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 1;
                varalignmin     : 0;
                varalignmax     : 1;
                localalignmin   : 0;
                localalignmax   : 1;
                recordalignmin  : 0;
                recordalignmax  : 1;
                maxCrecordalign : 1
              );
            first_parm_offset : 0;
            stacksize    : 1024;
            stackalign   : 1;
            abi : abi_default;
            llvmdatalayout : 'todo';
          );

       system_mipsel_embedded_info : tsysteminfo =
          (
            system       : system_mipsel_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_needs_symbol_type,tf_files_case_sensitive,
                            tf_smartlink_sections];
            cpu          : cpu_mipsel;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 4;
                varalignmin     : 0;
                varalignmax     : 4;
                localalignmin   : 4;
                localalignmax   : 8;
                recordalignmin  : 0;
                recordalignmax  : 8;
                maxCrecordalign : 4
              );
            first_parm_offset : 0;
            stacksize    : 262144;
            stackalign   : 8;
            abi : abi_default;
            llvmdatalayout : 'todo';
          );

       system_i386_embedded_info : tsysteminfo =
          (
            system       : system_i386_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_files_case_sensitive,
                            tf_smartlink_sections];
            cpu          : cpu_i386;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_i386_elf32;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 16;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 16;
                varalignmin     : 0;
                varalignmax     : 16;
                localalignmin   : 4;
                localalignmax   : 8;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 4096;
            stackalign   : 4;
            abi : abi_default;
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32-S32';
          );

       system_x86_64_embedded_info : tsysteminfo =
          (
            system       : system_x86_64_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_files_case_sensitive,
                            tf_smartlink_sections];
            cpu          : cpu_x86_64;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_x86_64_elf64;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 16;
                loopalign       : 8;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 16;
                varalignmin     : 0;
                varalignmax     : 16;
                localalignmin   : 4;
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 16
              );
            first_parm_offset : 16;
            stacksize    : 8*1024*1024;
            stackalign   : 16;
            abi : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
         );

       system_i8086_embedded_info : tsysteminfo =
          (
            system       : system_i8086_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_use_8_3,
{$ifdef I8086_SMARTLINK_SECTIONS}
                            tf_smartlink_sections,
{$else I8086_SMARTLINK_SECTIONS}
                            tf_smartlink_library,
                            tf_no_objectfiles_when_smartlinking,
{$endif I8086_SMARTLINK_SECTIONS}
                            tf_cld,
                            tf_no_generic_stackcheck,tf_emit_stklen];
            cpu          : cpu_i8086;
            unit_env     : '';
            extradefines : '';
            exeext       : '.exe';
            defext       : '.def';
            scriptext    : '.bat';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.dll';
            staticlibext : '.a';
            staticlibprefix : '';
            sharedlibprefix : '';
            sharedClibext : '.dll';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : '';
            importlibext : '.al';
            Cprefix      : '_';
            newline      : #13#10;
            dirsep       : '\';
            assem        : as_i8086_omf;
            assemextern  : as_i8086_nasmobj;
            link         : ld_int_msdos;
            linkextern   : ld_msdos;
{$ifdef USE_SCRIPTED_WLIB}
            ar           : ar_watcom_wlib_omf_scripted;
{$else}
            ar           : ar_watcom_wlib_omf;
{$endif}
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_dos;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 1;
                loopalign       : 1;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 2;
                varalignmin     : 0;
                varalignmax     : 2;
                localalignmin   : 0;
                localalignmax   : 2;
                recordalignmin  : 0;
                recordalignmax  : 2;
                maxCrecordalign : 2
              );
            first_parm_offset : 4;
            stacksize    : 0;
            stackalign   : 2;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );

       system_m68k_embedded_info : tsysteminfo =
          (
            system       : system_m68k_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_under_development,tf_needs_symbol_size,tf_files_case_sensitive,tf_requires_proper_alignment,
                            tf_smartlink_sections];
            cpu          : cpu_m68k;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_big;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 4;
                varalignmin     : 0;
                varalignmax     : 4;
                localalignmin   : 4;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 32768;
            stackalign   : 4;
            abi : abi_default;
            llvmdatalayout : 'TODO';
          );

       system_riscv32_embedded_info : tsysteminfo =
          (
            system       : system_riscv32_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_files_case_sensitive,tf_requires_proper_alignment,
                            tf_smartlink_sections];
            cpu          : cpu_riscv32;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 4;
                varalignmin     : 0;
                varalignmax     : 4;
                localalignmin   : 4;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 262144;
            stackalign   : 4;
            abi : abi_default;
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:64:128-a0:0:64-n32-S32';
          );

       system_riscv64_embedded_info : tsysteminfo =
          (
            system       : system_riscv64_embedded;
            name         : 'Embedded';
            shortname    : 'embedded';
            flags        : [tf_needs_symbol_size,tf_files_case_sensitive,tf_requires_proper_alignment,
                            tf_smartlink_sections];
            cpu          : cpu_riscv64;
            unit_env     : '';
            extradefines : '';
            exeext       : '';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.so';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : 'lib';
            sharedClibext : '.so';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : 'lib';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_embedded;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 0;
                constalignmax   : 4;
                varalignmin     : 0;
                varalignmax     : 4;
                localalignmin   : 4;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 4
              );
            first_parm_offset : 16;
            stacksize    : 262144;
            stackalign   : 8;
            abi : abi_default;
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:64:128-a0:0:64-n32-S32';
          );

  implementation

initialization
{$ifdef CPUARM}
  {$ifdef embedded}
    set_source_info(system_arm_embedded_info);
  {$endif embedded}
{$endif CPUARM}
{$ifdef CPUAVR}
  {$ifdef embedded}
    set_source_info(system_avr_embedded_info);
  {$endif embedded}
{$endif CPUAVR}
{$ifdef CPUMIPSEL}
  {$ifdef embedded}
    set_source_info(system_mipsel_embedded_info);
  {$endif embedded}
{$endif CPUMIPSEL}
{$ifdef CPUI386}
  {$ifdef embedded}
    set_source_info(system_i386_embedded_info);
  {$endif embedded}
{$endif CPUI386}
{$ifdef CPUX86_64}
  {$ifdef embedded}
    set_source_info(system_x86_64_embedded_info);
  {$endif embedded}
{$endif CPUX86_64}
{$ifdef cpu8086}
  {$ifdef embedded}
    set_source_info(system_i8086_embedded_info);
  {$endif embedded}
{$endif cpu8086}
{$ifdef cpum68k}
  {$ifdef embedded}
    set_source_info(system_m68k_embedded_info);
  {$endif embedded}
{$endif cpum68k}
{$ifdef cpuriscv32}
  {$ifdef embedded}
    set_source_info(system_riscv32_embedded_info);
  {$endif embedded}
{$endif cpuriscv32}
{$ifdef cpuriscv64}
  {$ifdef embedded}
    set_source_info(system_riscv64_embedded_info);
  {$endif embedded}
{$endif cpuriscv64}
end.
