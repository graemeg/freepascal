{
    Copyright (c) 1998-2008 by Peter Vreman

    This unit implements support information structures for win32

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
{ This unit implements support information structures for win32. }
unit i_win;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_win32_info : tsysteminfo =
          (
            system       : system_i386_WIN32;
            name         : 'Win32 for i386';
            shortname    : 'Win32';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_files_case_aware,tf_has_dllscanner
=======
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_smartlink_library
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_smartlink_library
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_smartlink_library
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_smartlink_library
>>>>>>> origin/cpstrnew
                            ,tf_smartlink_sections{,tf_section_threadvars}{,tf_needs_dwarf_cfi},
                            tf_winlikewidestring,tf_no_pic_supported,
                            tf_no_generic_stackcheck,tf_has_winlike_resources,
                            tf_dwarf_only_local_labels,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            tf_safecall_exceptions,tf_no_backquote_support];
=======
                            tf_safecall_exceptions];
>>>>>>> graemeg/cpstrnew
=======
                            tf_safecall_exceptions];
>>>>>>> graemeg/cpstrnew
=======
                            tf_safecall_exceptions];
>>>>>>> graemeg/cpstrnew
=======
                            tf_safecall_exceptions];
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_use_function_relative_addresses,tf_smartlink_library
                            ,tf_smartlink_sections{,tf_section_threadvars}{,tf_needs_dwarf_cfi},
                            tf_winlikewidestring,tf_no_pic_supported,
                            tf_no_generic_stackcheck];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : 'WIN32UNITS';
            extradefines : 'MSWINDOWS;WINDOWS';
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
            staticlibprefix : 'libp';
            sharedlibprefix : '';
            sharedClibext : '.dll';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '_';
            newline      : #13#10;
            dirsep       : '\';
            assem        : as_i386_pecoff;
            assemextern  : as_gas;
            link         : ld_int_windows;
            linkextern   : ld_windows;
            ar           : ar_gnu_ar;
            res          : res_gnu_windres;
            dbg          : dbg_stabs;
            script       : script_dos;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 16;
                loopalign       : 4;
                jumpalign       : 0;
                constalignmin   : 0;
                constalignmax   : 16;
                varalignmin     : 0;
                varalignmax     : 16;
                localalignmin   : 4;
                localalignmax   : 8;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 16
              );
            first_parm_offset : 8;
            stacksize    : 16*1024*1024;
<<<<<<< HEAD
<<<<<<< HEAD
            stackalign   : 4;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            abi          : abi_default;
            llvmdatalayout : 'e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-f80:128:128-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32-S32';
          );

       system_x64_win64_info : tsysteminfo =
          (
            system       : system_x86_64_win64;
            name         : 'Win64 for x64';
            shortname    : 'Win64';
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_files_case_aware,tf_has_dllscanner,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            tf_smartlink_sections,
                            tf_winlikewidestring,tf_no_pic_supported,
                            tf_dwarf_only_local_labels,
                            tf_no_generic_stackcheck,tf_has_winlike_resources,
                            tf_safecall_exceptions,tf_no_backquote_support];
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            tf_smartlink_sections,tf_smartlink_library,
                            tf_winlikewidestring,tf_no_pic_supported,
                            tf_dwarf_only_local_labels,
                            tf_no_generic_stackcheck,tf_has_winlike_resources];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_use_function_relative_addresses,
                            tf_smartlink_sections,tf_smartlink_library,tf_winlikewidestring,tf_no_pic_supported,
                            tf_no_generic_stackcheck];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_files_case_aware,tf_has_dllscanner,tf_use_function_relative_addresses,
                            tf_smartlink_sections,tf_smartlink_library,tf_winlikewidestring,tf_no_pic_supported,
                            tf_no_generic_stackcheck];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_x86_64;
            unit_env     : 'WIN64UNITS';
            extradefines : 'MSWINDOWS;WINDOWS';
            exeext       : '.exe';
            defext       : '.def';
            scriptext    : '.bat';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.obj';
            sharedlibext : '.dll';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : '';
            sharedClibext : '.dll';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #13#10;
            dirsep       : '\';
            assem        : as_x86_64_pecoff;
            assemextern  : as_gas;
            link         : ld_int_windows;
            linkextern   : ld_windows;
            ar           : ar_gnu_ar;
<<<<<<< HEAD
<<<<<<< HEAD
            res          : res_gnu_windres;
            dbg          : dbg_dwarf2;
=======
=======
>>>>>>> origin/fixes_2_2
            res          : res_win64_gorc;
            dbg          : dbg_stabs;
>>>>>>> graemeg/fixes_2_2
            script       : script_dos;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 16;
                loopalign       : 8;
                jumpalign       : 0;
                constalignmin   : 0;
                constalignmax   : 16;
                varalignmin     : 0;
                varalignmax     : 16;
                localalignmin   : 8;
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 8;
                maxCrecordalign : 16
              );
            first_parm_offset : 16;
            stacksize    : 16*1024*1024;
<<<<<<< HEAD
<<<<<<< HEAD
            stackalign   : 16;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            abi          : abi_default;
            llvmdatalayout : 'e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128';
          );

       system_arm_wince_info : tsysteminfo =
          (
            system       : system_arm_wince;
            name         : 'WinCE for ARM';
            shortname    : 'WinCE';
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_files_case_aware{,tf_winlikewidestring},
                            tf_smartlink_sections,tf_requires_proper_alignment,tf_no_pic_supported,
                            tf_has_winlike_resources,
                            tf_safecall_exceptions,tf_no_backquote_support];
=======
            flags        : [tf_files_case_aware,tf_use_function_relative_addresses{,tf_winlikewidestring},
                            tf_smartlink_sections,tf_requires_proper_alignment,tf_no_pic_supported];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_files_case_aware,tf_use_function_relative_addresses{,tf_winlikewidestring},
                            tf_smartlink_sections,tf_requires_proper_alignment,tf_no_pic_supported];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_arm;
            unit_env     : '';
            extradefines : 'UNDER_CE;WINDOWS;FPC_OS_UNICODE';
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
            staticlibprefix : 'libp';
            sharedlibprefix : '';
            sharedClibext : '.dll';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #13#10;
            dirsep       : '\';
            assem        : as_arm_pecoffwince;
            assemextern  : as_gas;
            link         : ld_int_windows;
            linkextern   : ld_windows;
            ar           : ar_gnu_ar_scripted;
            res          : res_gnu_windres;
            dbg          : dbg_stabs;
            script       : script_dos;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
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
            first_parm_offset : 8;
            stacksize    : 262144;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );

       system_i386_wince_info : tsysteminfo =
          (
            system       : system_i386_wince;
            name         : 'WinCE for i386';
            shortname    : 'WinCE';
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_files_case_aware
                            {,tf_winlikewidestring},tf_smartlink_sections,tf_no_pic_supported,
                            tf_has_winlike_resources,
                            tf_safecall_exceptions,tf_no_backquote_support];
=======
            flags        : [tf_files_case_aware,tf_use_function_relative_addresses
                            {,tf_winlikewidestring},tf_smartlink_sections,tf_no_pic_supported];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_files_case_aware,tf_use_function_relative_addresses
                            {,tf_winlikewidestring},tf_smartlink_sections,tf_no_pic_supported];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : '';
            extradefines : 'UNDER_CE;WINDOWS;FPC_OS_UNICODE';
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
            staticlibprefix : 'libp';
            sharedlibprefix : '';
            sharedClibext : '.dll';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '_';
            newline      : #13#10;
            dirsep       : '\';
            assem        : as_i386_pecoffwince;
            assemextern  : as_gas;
            link         : ld_int_windows;
            linkextern   : ld_windows;
            ar           : ar_gnu_ar_scripted;
            res          : res_gnu_windres;
            dbg          : dbg_stabs;
            script       : script_dos;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                constalignmin   : 0;
                constalignmax   : 4;
                varalignmin     : 0;
                varalignmax     : 4;
                localalignmin   : 4;
                localalignmax   : 4;
                recordalignmin  : 0;
                recordalignmax  : 4;
                maxCrecordalign : 16
              );
            first_parm_offset : 8;
            stacksize    : 262144;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );


  implementation

initialization
{$ifdef CPUI386}
  {$ifdef WIN32}
    {$ifndef WDOSX}
      set_source_info(system_i386_win32_info);
    {$endif WDOSX}
  {$endif WIN32}
  {$ifdef WINCE}
    set_source_info(system_i386_wince_info);
  {$endif WINCE}
{$endif CPUI386}

{$ifdef CPUX86_64}
  {$ifdef WIN64}
    {$ifndef WDOSX}
      set_source_info(system_x64_win64_info);
    {$endif WDOSX}
  {$endif WIN64}
{$endif CPUX86_64}

{$ifdef CPUARM}
  {$ifdef WINCE}
    set_source_info(system_arm_wince_info);
  {$endif WINCE}
{$endif CPUARM}
end.
