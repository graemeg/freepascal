{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by contributors of the Free Pascal Compiler

    This unit implements support information structures for symbian os

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
unit i_symbian;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_symbian_info : tsysteminfo =
          (
            system       : system_i386_symbian;
            name         : 'Symbian OS for i386';
            shortname    : 'Symbian';
            flags        : [tf_files_case_aware, tf_has_dllscanner,
<<<<<<< HEAD
<<<<<<< HEAD
                            tf_smartlink_library];
=======
                            tf_smartlink_library,tf_use_function_relative_addresses];
>>>>>>> graemeg/fixes_2_2
=======
                            tf_smartlink_library,tf_use_function_relative_addresses];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : 'SYMBIANUNITS';
            extradefines : 'SYMBIAN';
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
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
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
            stacksize    : 262144;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );

       system_arm_symbian_info : tsysteminfo =
          (
            system       : system_arm_symbian;
            name         : 'Symbian OS for ARM';
            shortname    : 'Symbian';
            flags        : [tf_files_case_aware, tf_has_dllscanner,
                            tf_requires_proper_alignment,tf_no_pic_supported];
            cpu          : cpu_arm;
            unit_env     : 'SYMBIANUNITS';
            extradefines : 'SYMBIAN';
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
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_windows;
            linkextern   : ld_none;
            ar           : ar_gnu_ar;
            res          : res_none;
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


implementation

initialization

{$ifdef CPUI386}
  {$ifdef Symbian}
  set_source_info(system_i386_symbian_info);
  {$endif Symbian}
{$endif CPUI386}

{$ifdef CPUARM}
  {$ifdef Symbian}
  set_source_info(system_arm_symbian_info);
  {$endif Symbian}
{$endif CPUARM}

end.
