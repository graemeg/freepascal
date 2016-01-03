{
    Copyright (c) 2011 by Francesco Lombardi

    This unit implements support information structures for Wii

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
{ This unit implements support information structures for the Nintendo Wii. }
unit i_wii;

  interface

    uses
       systems;

    const
       system_powerpc_wii_info : tsysteminfo =
          (
            system       : system_powerpc_wii;
            name         : 'Wii';
            shortname    : 'Wii';
            flags        : [tf_under_development,tf_needs_symbol_size,tf_files_case_aware,
                            tf_use_function_relative_addresses,tf_needs_symbol_type,
                            tf_smartlink_library];
            cpu          : cpu_powerpc;
            unit_env     : '';
            extradefines : '';
            exeext       : '.dol';
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            link         : ld_none;
            linkextern   : ld_wii;
=======
            link         : nil;
            linkextern   : nil;
>>>>>>> graemeg/cpstrnew
=======
            link         : nil;
            linkextern   : nil;
>>>>>>> graemeg/cpstrnew
=======
            link         : nil;
            linkextern   : nil;
>>>>>>> graemeg/cpstrnew
=======
            link         : nil;
            linkextern   : nil;
>>>>>>> origin/cpstrnew
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_stabs;
            script       : script_unix;
            endian       : endian_big;
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
                maxCrecordalign : 8
              );
            first_parm_offset : 8;
            stacksize    : 131072;  // 128 kb 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            stackalign   : 16;
            abi : abi_powerpc_sysv;
            llvmdatalayout : 'E-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v128:128:128-n32';
=======
            abi : abi_powerpc_sysv;
>>>>>>> graemeg/cpstrnew
=======
            abi : abi_powerpc_sysv;
>>>>>>> graemeg/cpstrnew
=======
            abi : abi_powerpc_sysv;
>>>>>>> graemeg/cpstrnew
=======
            abi : abi_powerpc_sysv;
>>>>>>> origin/cpstrnew
          );

  implementation

initialization
{$ifdef CPUPOWERPC}
  {$ifdef WII}
    set_source_info(system_powerpc_wii_info);
  {$endif WII}
{$endif CPUPOWERPC}
end.
