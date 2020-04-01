{
    Copyright (c) 1998-2008 by Peter Vreman

    This unit implements support information structures for FreeBSD/NetBSD
    and OpenBSD

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
{ This unit implements support information structures for FreeBSD/NetBSD.
  OpenBSD is only added for i386 for now, though it exists for most
  other common CPU's too}

unit i_bsd;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_freebsd_info : tsysteminfo =
          (
            system       : system_i386_FreeBSD;
            name         : 'FreeBSD/ELF for i386';
            shortname    : 'FreeBSD';
            flags        : [tf_pic_uses_got,tf_files_case_sensitive,
{$ifdef tls_threadvars}
                            tf_section_threadvars,
{$endif tls_threadvars}
                            tf_needs_symbol_type,tf_needs_symbol_size
                            ,tf_smartlink_sections,tf_has_winlike_resources
                            ,tf_supports_hidden_symbols];
            cpu          : cpu_i386;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;BSD;HASUNIX';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_stabs;
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
                localalignmin   : 0;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 2;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize   : 262144;
            stackalign   : 4;
            abi          : abi_default;
            { note: default LLVM stack alignment is 16 bytes for this target }
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32-S32';
          );


       system_x86_64_freebsd_info : tsysteminfo =
          (
            system       : system_x86_64_freebsd;
            name         : 'FreeBSD for x86-64';
            shortname    : 'FreeBSD';
            flags        : [tf_needs_symbol_size,tf_needs_dwarf_cfi,tf_library_needs_pic,tf_needs_symbol_type,
                            tf_files_case_sensitive,
                            tf_dwarf_only_local_labels,
                            {tf_pic_uses_got,}tf_smartlink_sections,tf_has_winlike_resources,tf_supports_hidden_symbols];
            cpu          : cpu_x86_64;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;HASUNIX;BSD';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_dwarf2;            //dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 8;
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
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 16
              );
            first_parm_offset : 16;
            stacksize    : 256*1024;
            stackalign   : 16;
            abi          : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
          );


       system_x86_64_dragonfly_info : tsysteminfo =
          (
            system       : system_x86_64_dragonfly;
            name         : 'DragonFly for x86-64';
            shortname    : 'DragonFly';
            flags        : [tf_needs_symbol_size,tf_needs_dwarf_cfi,tf_library_needs_pic,tf_needs_symbol_type,
                            tf_files_case_sensitive,
                            tf_dwarf_only_local_labels,
                            {tf_pic_uses_got,}tf_smartlink_sections,tf_has_winlike_resources,tf_supports_hidden_symbols];
            cpu          : cpu_x86_64;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;HASUNIX;BSD';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_dwarf2;            //dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 8;
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
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 16
              );
            first_parm_offset : 16;
            stacksize    : 256*1024;
            stackalign   : 16;
            abi          : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
          );


       system_i386_netbsd_info : tsysteminfo =
          (
            system       : system_i386_NetBSD;
            name         : 'NetBSD for i386';
            shortname    : 'NetBSD';
            flags        : [tf_pic_uses_got,tf_under_development,tf_files_case_sensitive,tf_smartlink_library,tf_has_winlike_resources,
                            tf_supports_hidden_symbols];
            cpu          : cpu_i386;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;BSD;HASUNIX';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
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
            stacksize   : 262144;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32-S32';
          );

       system_i386_openbsd_info : tsysteminfo =
          (
            system       : system_i386_OpenBSD;
            name         : 'OpenBSD for i386';
            shortname    : 'OpenBSD';
            flags        : [tf_pic_default,tf_pic_uses_got,tf_under_development,tf_files_case_sensitive,tf_smartlink_sections,tf_has_winlike_resources,
                            tf_supports_hidden_symbols];
            cpu          : cpu_i386;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;BSD;HASUNIX';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
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
            stacksize   : 262144;
            stackalign   : 4;
            abi          : abi_default;
            { note: default LLVM stack alignment is 16 bytes for this target }
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32-S32';
          );

       system_x86_64_openbsd_info : tsysteminfo =
          (
            system       : system_x86_64_openbsd;
            name         : 'OpenBSD for x86-64';
            shortname    : 'OpenBSD';
            flags        : [tf_needs_symbol_size,tf_needs_dwarf_cfi,tf_library_needs_pic,tf_needs_symbol_type,
                            tf_files_case_sensitive, tf_under_development,
                            tf_dwarf_only_local_labels, tf_pic_default,
                            { tf_pic_uses_got,}tf_smartlink_sections,tf_has_winlike_resources,tf_supports_hidden_symbols];
            cpu          : cpu_x86_64;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;HASUNIX;BSD';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_dwarf2;            //dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 8;
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
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 16
              );
            first_parm_offset : 16;
            stacksize    : 256*1024;
            stackalign   : 16;
            abi          : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
          );

       system_m68k_netbsd_info : tsysteminfo =
          (
            system       : system_m68k_NetBSD;
            name         : 'NetBSD for m68k';
            shortname    : 'NetBSD';
            flags        : [tf_under_development,tf_needs_symbol_size,tf_needs_symbol_type,tf_requires_proper_alignment,
                            tf_files_case_sensitive,tf_smartlink_sections,tf_has_winlike_resources,tf_supports_hidden_symbols];
            cpu          : cpu_m68k;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;BSD;HASUNIX';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_stabs;
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
                constalignmax   : 1;
                varalignmin     : 0;
                varalignmax     : 1;
                localalignmin   : 0;
                localalignmax   : 1;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 8
              );
            first_parm_offset : 8;
            stacksize   : 262144;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );

       system_powerpc_netbsd_info : tsysteminfo =
          (
            system       : system_powerpc_netbsd;
            name         : 'NetBSD for PowerPC';
            shortname    : 'NetBSD';
            flags        : [tf_under_development,tf_files_case_sensitive,tf_smartlink_library,tf_has_winlike_resources,tf_supports_hidden_symbols];
            cpu          : cpu_powerpc;
            unit_env     : '';
            extradefines : 'UNIX;BSD;HASUNIX';
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
            staticlibext : '.s';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_stabs;
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
                localalignmin   : 0;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 2;
                maxCrecordalign : 4     // should be 8 probably
              );
            first_parm_offset : 8;
            stacksize    : 32*1024*1024;
            stackalign   : 16;
            { abi_powerpc_sysv doesn't work yet }
            abi : abi_powerpc_aix;
            llvmdatalayout : 'E-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v128:128:128-n32';
          );

       system_x86_64_netbsd_info : tsysteminfo =
          (
            system       : system_x86_64_netbsd;
            name         : 'NetBSD for x86-64';
            shortname    : 'NetBSD';
            flags        : [tf_needs_symbol_size,tf_needs_dwarf_cfi,tf_library_needs_pic,tf_needs_symbol_type,
                            tf_files_case_sensitive, tf_under_development,
                            tf_dwarf_only_local_labels,
                            { tf_pic_uses_got,}tf_smartlink_sections,tf_has_winlike_resources,tf_supports_hidden_symbols];
            cpu          : cpu_x86_64;
            unit_env     : 'BSDUNITS';
            extradefines : 'UNIX;HASUNIX;BSD';
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
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_dwarf2;            //dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 8;
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
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 16
              );
            first_parm_offset : 16;
            stacksize    : 256*1024;
            stackalign   : 16;
            abi          : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
          );

       system_arm_netbsd_info : tsysteminfo =
          (
            system       : system_arm_netbsd;
            name         : 'NetBSD for ARMHF';
            shortname    : 'NetBSD';
            flags        : [tf_under_development,tf_requires_proper_alignment,tf_files_case_sensitive,tf_smartlink_sections,tf_has_winlike_resources,
                            tf_supports_hidden_symbols];
            cpu          : cpu_arm;
            unit_env     : '';
            extradefines : 'UNIX;BSD;HASUNIX';
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
            staticlibext : '.s';
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
            assem        : as_gas; {as_arm_elf32;}
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_bsd;
            ar           : ar_gnu_ar;
            res          : res_elf;
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
                localalignmin   : 0;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 2;
                maxCrecordalign : 4     // should be 8 probably
              );
            first_parm_offset : 8;
            stacksize    : 32*1024*1024;
            stackalign   : 16;
            abi : abi_eabihf;
            llvmdatalayout : 'todo';
          );



  implementation

initialization
{$ifdef cpui386}
  {$ifdef FreeBSD}
     set_source_info(system_i386_FreeBSD_info);
  {$endif}
  {$ifdef NetBSD}
     set_source_info(system_i386_NetBSD_info);
  {$endif}
  {$ifdef OpenBSD}
     set_source_info(system_i386_OpenBSD_info);
  {$endif}
{$endif cpui386}
{$ifdef cpux86_64}
   {$ifdef FreeBSD}
     set_source_info(system_x86_64_FreeBSD_info);
   {$endif}
   {$ifdef DragonFly}
     set_source_info(system_x86_64_DragonFly_info);
   {$endif}
   {$ifdef OpenBSD}
     set_source_info(system_x86_64_OpenBSD_info);
   {$endif}
   {$ifdef NetBSD}
     set_source_info(system_x86_64_NetBSD_info);
   {$endif}
{$endif}
{$ifdef cpu68}
  {$ifdef NetBSD}
     set_source_info(system_m68k_NetBSD_info);
  {$endif NetBSD}
{$endif cpu68}
{$ifdef cpupowerpc32}
  {$ifdef NetBSD}
     set_source_info(system_powerpc_netbsd_info);
  {$endif}
{$endif cpupowerpc32}
{$ifdef cpuarm}
  {$ifdef NetBSD}
     set_source_info(system_arm_netbsd_info);
  {$endif}
{$endif cpuarm}
end.
