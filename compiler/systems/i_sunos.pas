{
    Copyright (c) 1998-2008 by Peter Vreman

    This unit implements support information structures for solaris

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
{ This unit implements support information structures for solaris. }
unit i_sunos;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_solaris_info : tsysteminfo =
          (
            system       : system_i386_solaris;
            name         : 'Solaris for i386';
            shortname    : 'solaris';
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_under_development,tf_needs_symbol_size,
                            tf_files_case_sensitive,tf_requires_proper_alignment,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            tf_pic_uses_got,tf_library_needs_pic,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            tf_smartlink_library,tf_has_winlike_resources];
=======
            flags        : [tf_under_development,tf_files_case_sensitive,tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_under_development,tf_files_case_sensitive,tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : 'SOLARISUNITS';
            extradefines : 'UNIX;LIBC;SUNOS;HASUNIX';
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
            assemextern  : as_ggas;
            link         : ld_none;
            linkextern   : ld_solaris;
            ar           : ar_gnu_gar;
            res          : res_elf;
            dbg          : dbg_stabs;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 16;
                loopalign       : 4;
                jumpalign       : 0;
                constalignmin   : 0;
                constalignmax   : 8;
                varalignmin     : 0;
                varalignmax     : 16;
                localalignmin   : 4;
                localalignmax   : 8;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 262144;
            stackalign   : 4;
            abi          : abi_default;
            { note: default LLVM stack alignment is 16 bytes for this target }
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32-S32';
          );


       system_x86_64_solaris_info : tsysteminfo =
          (
            system       : system_x86_64_solaris;
            name         : 'Solaris for x86-64';
            shortname    : 'solaris';
            flags        : [tf_needs_symbol_size,tf_needs_symbol_type,
<<<<<<< HEAD
                            tf_under_development,
=======
			    tf_under_development,
>>>>>>> graemeg/cpstrnew
                            tf_files_case_sensitive,
                            tf_requires_proper_alignment,tf_smartlink_library,tf_library_needs_pic,
                            tf_has_winlike_resources];
            cpu          : cpu_x86_64;
            unit_env     : 'SOLARISUNITS';
            extradefines : 'UNIX;LIBC;SUNOS;HASUNIX';
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
<<<<<<< HEAD
            assem        : as_x86_64_elf64;
            assemextern  : as_ggas;
            link         : ld_none;
            linkextern   : ld_solaris;
            ar           : ar_gnu_gar;
=======
            assem        : as_ggas{as_x86_64_elf64};
            assemextern  : as_ggas;
            link         : nil;
            linkextern   : nil;
            ar           : ar_gnu_ar;
>>>>>>> graemeg/cpstrnew
            res          : res_elf;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 8;
                loopalign       : 4;
                jumpalign       : 0;
                constalignmin   : 0;
                constalignmax   : 8;
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
<<<<<<< HEAD
            stackalign   : 16;
            abi : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
=======
            abi : abi_default
>>>>>>> graemeg/cpstrnew
          );


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       system_x86_64_solaris_info : tsysteminfo =
          (
            system       : system_x86_64_solaris;
            name         : 'Solaris for x86-64';
            shortname    : 'solaris';
            flags        : [tf_needs_symbol_size,tf_needs_symbol_type,
			    tf_under_development,
                            tf_files_case_sensitive,
                            tf_requires_proper_alignment,tf_smartlink_library,tf_library_needs_pic,
                            tf_has_winlike_resources];
            cpu          : cpu_x86_64;
            unit_env     : 'SOLARISUNITS';
            extradefines : 'UNIX;LIBC;SUNOS;HASUNIX';
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
            assem        : as_ggas{as_x86_64_elf64};
            assemextern  : as_ggas;
            link         : nil;
            linkextern   : nil;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 8;
                loopalign       : 4;
                jumpalign       : 0;
                constalignmin   : 0;
                constalignmax   : 8;
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
            abi : abi_default
          );


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       system_sparc_solaris_info : tsysteminfo =
          (
            system       : system_sparc_solaris;
            name         : 'Solaris for SPARC';
            shortname    : 'solaris';
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_needs_symbol_size,tf_under_development,
                            tf_files_case_sensitive,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            tf_pic_uses_got,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            tf_requires_proper_alignment,tf_smartlink_library,
                            tf_has_winlike_resources];
=======
            flags        : [tf_needs_symbol_size,tf_under_development,tf_files_case_sensitive,tf_use_function_relative_addresses,
                            tf_requires_proper_alignment,tf_smartlink_library];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_needs_symbol_size,tf_under_development,tf_files_case_sensitive,tf_use_function_relative_addresses,
                            tf_requires_proper_alignment,tf_smartlink_library];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_SPARC;
            unit_env     : 'SOLARISUNITS';
            extradefines : 'UNIX;LIBC;SUNOS;HASUNIX';
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
            assem        : as_ggas;
            assemextern  : as_ggas;
            link         : ld_none;
            linkextern   : ld_solaris;
            ar           : ar_gnu_gar;
            res          : res_elf;
            dbg          : dbg_stabs;
            script       : script_unix;
            endian       : endian_big;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                constalignmin   : 4;
                constalignmax   : 8;
                varalignmin     : 4;
                varalignmax     : 8;
                localalignmin   : 4;
                localalignmax   : 8;
                recordalignmin  : 0;
                recordalignmax  : 8;
                maxCrecordalign : 8
              );
            first_parm_offset : 92;
            stacksize    : 262144;
            stackalign   : 8;
            abi          : abi_default;
            llvmdatalayout : 'E-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-n32-S64';
          );

  implementation

initialization
{$ifdef CPUI386}
  {$ifdef solaris}
    set_source_info(system_i386_solaris_info);
  {$endif solaris}
<<<<<<< HEAD
{$endif CPUI386}
=======
{$endif CPU86}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef CPUX86_64}
  {$ifdef solaris}
    set_source_info(system_x86_64_solaris_info);
  {$endif solaris}
{$endif CPUX86_64}
{$ifdef CPUSparc}
  {$ifdef solaris}
    set_source_info(system_sparc_solaris_info);
  {$endif solaris}
{$endif CPUSparc}

end.
