{
    Copyright (c) 1998-2008 by Peter Vreman

    This unit implements support information structures for Darwin
    (Mac OS X/OS X/macOS/iOS/iPhoneSimulator/...)

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
unit i_darwin;

{$i fpcdefs.inc}

interface

uses
   systems;

const
   res_macho_info : tresinfo =
       (
         id     : res_macho;
         resbin : 'fpcres';
         rescmd : '-o $OBJ -a $ARCH -s $SUBARCH -of mach-o $DBG';
         rcbin  : 'windres';
         rccmd  : '--include $INC -O res -D FPC -o $RES $RC';
         resourcefileclass : nil;
         resflags : [];
       );
   res_macosx_ext_info : tresinfo =
      (
         id     : res_ext;
         resbin : 'fpcres';
         rescmd : '-o $OBJ -a $ENDIAN -of external $DBG';
         rcbin  : 'windres';
         rccmd  : '--include $INC -O res -D FPC -o $RES $RC';
         resourcefileclass : nil;
         resflags : [res_external_file,res_arch_in_file_name];
      );

   system_powerpc_darwin_info  : tsysteminfo =
      (
        system       : system_powerpc_darwin;
        name         : 'Darwin for PowerPC';
        shortname    : 'Darwin';
        flags        : [tf_p_ext_support,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,tf_dwarf_only_local_labels,tf_pic_default,tf_has_winlike_resources,
                        tf_supports_hidden_symbols];
        cpu          : cpu_powerpc;
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_darwin;
        assemextern  : as_darwin;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_stabs;
        script       : script_unix;
        endian       : endian_big;
        alignment    :
          (
            procalign       : 16;
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
            recordalignmax  : 4;
            maxCrecordalign : 4
          );
        first_parm_offset : 24;
        stacksize   : 262144;
        stackalign   : 16;
        abi : abi_powerpc_darwin;
        llvmdatalayout : 'E-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:64:64-v128:128:128-n32';
      );



   system_i386_darwin_info  : tsysteminfo =
      (
        system       : system_i386_darwin;
        name         : 'Darwin for i386';
        shortname    : 'Darwin';
        flags        : [tf_p_ext_support,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,tf_dwarf_only_local_labels,tf_pic_uses_got,
                        tf_pic_default,tf_has_winlike_resources,tf_use_hlcfi,tf_supports_symbolorderfile,tf_supports_hidden_symbols];
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_clang;
        assemextern  : as_clang;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
        script       : script_unix;
        endian       : endian_little;
        alignment    :
          (
            procalign       : 16;
            loopalign       : 8;
            jumpalign       : 16;
            jumpalignskipmax    : 10;
            coalescealign   : 0;
            coalescealignskipmax: 0;
            constalignmin   : 0;
            constalignmax   : 16;
            varalignmin     : 0;
            varalignmax     : 16;
            localalignmin   : 0;
            localalignmax   : 8;
            recordalignmin  : 0;
            recordalignmax  : 16;
            maxCrecordalign : 16
          );
        first_parm_offset : 8;
        stacksize   : 262144;
        stackalign   : 16;
        abi         : abi_i386_dynalignedstack;
        llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:128:128-n8:16:32-S128';
      );



   system_i386_iphonesim_info  : tsysteminfo =
      (
        system       : system_i386_iphonesim;
        name         : 'Darwin/iPhoneSim for i386';
        shortname    : 'iPhoneSim';
        flags        : [tf_p_ext_support,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,tf_dwarf_only_local_labels,
                        tf_pic_uses_got,tf_pic_default,tf_has_winlike_resources,tf_use_hlcfi,tf_supports_hidden_symbols];
        cpu          : cpu_i386;
        unit_env     : 'BSDUNITS';
        extradefines : 'UNIX;BSD;HASUNIX;DARWIN'; // also define darwin for code compatibility
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_clang;
        assemextern  : as_clang;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
        script       : script_unix;
        endian       : endian_little;
        alignment    :
          (
            procalign       : 16;
            loopalign       : 8;
            jumpalign       : 16;
            jumpalignskipmax    : 10;
            coalescealign   : 0;
            coalescealignskipmax: 0;
            constalignmin   : 0;
            constalignmax   : 16;
            varalignmin     : 0;
            varalignmax     : 16;
            localalignmin   : 0;
            localalignmax   : 16;
            recordalignmin  : 0;
            recordalignmax  : 16;
            maxCrecordalign : 16
          );
        first_parm_offset : 8;
        stacksize   : 262144;
        stackalign   : 16;
        abi         : abi_i386_dynalignedstack;
        llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:128:128-n8:16:32-S128';
      );



   system_powerpc64_darwin_info  : tsysteminfo =
      (
        system       : system_powerpc64_darwin;
        name         : 'Darwin for PowerPC64';
        shortname    : 'Darwin';
        flags        : [tf_p_ext_support,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,tf_dwarf_only_local_labels,
                        tf_pic_default,tf_has_winlike_resources,tf_supports_symbolorderfile,tf_supports_hidden_symbols];
        cpu          : cpu_powerpc64;
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_darwin;
        assemextern  : as_darwin;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
        script       : script_unix;
        endian       : endian_big;
        alignment    :
          (
            procalign       : 16;
            loopalign       : 4;
            jumpalign       : 0;
            jumpalignskipmax    : 0;
            coalescealign   : 0;
            coalescealignskipmax: 0;
            constalignmin   : 4;
            constalignmax   : 8;
            varalignmin     : 4;
            varalignmax     : 8;
            localalignmin   : 4;
            localalignmax   : 8;
            recordalignmin  : 0;
            recordalignmax  : 8;
            maxCrecordalign : 4
          );
        first_parm_offset : 48;
        stacksize   : 262144;
        stackalign   : 16;
        abi : abi_powerpc_darwin;
        llvmdatalayout : 'E-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v128:128:128-n32:64';
      );



   system_x86_64_darwin_info  : tsysteminfo =
      (
        system       : system_x86_64_darwin;
        name         : 'Darwin for x86_64';
        shortname    : 'Darwin';
        flags        : [tf_p_ext_support,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,tf_dwarf_only_local_labels,tf_pic_default,tf_has_winlike_resources,tf_use_hlcfi
                        {$ifdef llvm},tf_use_psabieh{$endif},tf_supports_symbolorderfile,tf_supports_hidden_symbols];
        cpu          : cpu_x86_64;
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_clang;
        assemextern  : as_clang;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
        script       : script_unix;
        endian       : endian_little;
        alignment    :
          (
            procalign       : 16;
            loopalign       : 8;
            jumpalign       : 16;
            jumpalignskipmax    : 10;
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
        stacksize   : 262144;
        stackalign   : 16;
        abi : abi_default;
        llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
      );


   system_x86_64_iphonesim_info  : tsysteminfo =
      (
        system       : system_x86_64_iphonesim;
        name         : 'Darwin/iPhoneSim for x86_64';
        shortname    : 'iPhoneSim';
        flags        : [tf_p_ext_support,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,tf_dwarf_only_local_labels,
                        tf_pic_default,tf_has_winlike_resources,tf_use_hlcfi,tf_supports_hidden_symbols];
        cpu          : cpu_x86_64;
        unit_env     : 'BSDUNITS';
        extradefines : 'UNIX;BSD;HASUNIX;DARWIN'; // also define darwin for code compatibility
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_clang;
        assemextern  : as_clang;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
        script       : script_unix;
        endian       : endian_little;
        alignment    :
          (
            procalign       : 16;
            loopalign       : 8;
            jumpalign       : 16;
            jumpalignskipmax    : 10;
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
        stacksize   : 262144;
        stackalign   : 16;
        abi : abi_default;
        llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
      );


   system_arm_darwin_info : tsysteminfo =
      (
        system       : system_arm_darwin;
        name         : 'Darwin for ARM';
        shortname    : 'Darwin';
        flags        : [tf_p_ext_support,tf_requires_proper_alignment,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,
                        tf_dwarf_only_local_labels,tf_has_winlike_resources,tf_pic_default,tf_supports_symbolorderfile,tf_supports_hidden_symbols];
        cpu          : cpu_arm;
        unit_env     : 'BSDUNITS';
        extradefines : 'UNIX;BSD;HASUNIX;CPUARMEL';
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_clang;
        assemextern  : as_clang;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
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
            constalignmax   : 8;
            varalignmin     : 0;
            varalignmax     : 8;
            localalignmin   : 4;
            localalignmax   : 8;
            recordalignmin  : 0;
            recordalignmax  : 8;
            maxCrecordalign : 8
          );
        first_parm_offset : 8;
        stacksize    : 262144;
        stackalign   : 4;
        abi : abi_default;
        { note: default LLVM stack alignment is 8 bytes for this target }
        llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:64:128-a0:0:64-n32-S32';
      );


   system_aarch64_darwin_info  : tsysteminfo =
      (
        system       : system_aarch64_darwin;
        name         : 'Darwin for AArch64';
        shortname    : 'Darwin';
        flags        : [tf_p_ext_support,tf_requires_proper_alignment,tf_files_case_sensitive,tf_smartlink_sections,tf_dwarf_relative_addresses,
                        tf_dwarf_only_local_labels,tf_pic_default,tf_has_winlike_resources,tf_supports_symbolorderfile,tf_supports_hidden_symbols];
        cpu          : cpu_aarch64;
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
        sharedlibext : '.dylib';
        staticlibext : '.a';
        staticlibprefix : 'libp';
        sharedlibprefix : 'lib';
        sharedClibext : '.dylib';
        staticClibext : '.a';
        staticClibprefix : 'lib';
        sharedClibprefix : 'lib';
        importlibprefix : 'libimp';
        importlibext : '.a';
        Cprefix      : '_';
        newline      : #10;
        dirsep       : '/';
        assem        : as_clang;
        assemextern  : as_clang;
        link         : ld_none;
        linkextern   : ld_darwin;
        ar           : ar_gnu_ar;
        res          : res_macho;
        dbg          : dbg_dwarf2;
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
            localalignmax   : 16;
            recordalignmin  : 0;
            recordalignmax  : 16;
            maxCrecordalign : 16
          );
        first_parm_offset : 16;
        stacksize   : 8*1024*1024;
        stackalign   : 16;
        abi : abi_aarch64_darwin;
        llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-n32:64-S128'
      );

implementation

initialization
  {$ifdef cpui386}
    {$ifdef Darwin}
      set_source_info(system_i386_Darwin_info);
    {$endif Darwin}
  {$endif cpui386}
  {$ifdef cpux86_64}
    {$ifdef Darwin}
      set_source_info(system_x86_64_darwin_info);
    {$endif}
  {$endif cpux86_64}
  {$ifdef cpupowerpc32}
    {$ifdef Darwin}
      set_source_info(system_powerpc_darwin_info);
    {$endif Darwin}
  {$endif cpupowerpc32}
  {$ifdef cpupowerpc64}
    {$ifdef Darwin}
      set_source_info(system_powerpc64_darwin_info);
    {$endif Darwin}
  {$endif powerpc64}
  {$ifdef cpuarm}
    {$ifdef Darwin}
      set_source_info(system_arm_darwin_info);
    {$endif Darwin}
  {$endif cpuarm}
  {$ifdef cpuaarch64}
    {$ifdef Darwin}
      set_source_info(system_aarch64_darwin_info);
    {$endif Darwin}
  {$endif cpuaarch64}

end.

