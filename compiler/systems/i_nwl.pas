{
    Copyright (c) 1998-2004 by Peter Vreman

    This unit implements support information structures for Netware (libc) modules

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
 ****************************************************************************}

{ This unit implements support information structures for Netware libc modules. }
unit i_nwl;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_netwlibc_info : tsysteminfo =
          (
            system       : system_i386_netwlibc;
            name         : 'Netware for i386(libc)';
            shortname    : 'Netwlibc';
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_smartlink_library];
=======
            flags        : [tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : 'NETWLIBCUNITS';
            extradefines : 'NETWARE;NETWARE_LIBC';
            exeext       : '.nlm';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.s';
            objext       : '.o';
            resext       : '.res';
            resobjext    : '.or';
            sharedlibext : '.nlm';
            staticlibext : '.a';
            staticlibprefix : '';
            sharedlibprefix : '';
            sharedClibext : '.nlm';
            staticClibext : '.a';
            staticClibprefix : '';
            sharedClibprefix : '';
            importlibprefix : 'imp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #13#10;
            dirsep       : '/';
            assem        : as_i386_elf32;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_netwlibc;
            ar           : ar_gnu_ar;
            res          : res_none;
            dbg          : dbg_stabs;
            script       : script_unix;
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
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 16384;
<<<<<<< HEAD
<<<<<<< HEAD
            stackalign   : 4;
            abi : abi_default;
            llvmdatalayout : 'todo';
=======
            abi : abi_default
>>>>>>> graemeg/fixes_2_2
=======
            abi : abi_default
>>>>>>> origin/fixes_2_2
          );

  implementation

initialization
{$ifdef CPUI386}
  {$ifdef netwlibc}
    set_source_info(system_i386_netwlibc_info);
  {$endif netwlibc}
{$endif CPUI386}
end.
