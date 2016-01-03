{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit implements support information structures for Netware modules

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
{ This unit implements support information structures for Netware modules. }
unit i_nwm;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_netware_info : tsysteminfo =
          (
            system       : system_i386_netware;
            name         : 'Netware for i386(clib)';
            shortname    : 'Netware';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_smartlink_sections,tf_dwarf_only_local_labels];
=======
            flags        : [tf_smartlink_library,tf_smartlink_sections,tf_dwarf_only_local_labels];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_smartlink_library,tf_smartlink_sections,tf_dwarf_only_local_labels];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_smartlink_library,tf_smartlink_sections,tf_dwarf_only_local_labels];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_smartlink_library,tf_smartlink_sections,tf_dwarf_only_local_labels];
>>>>>>> origin/cpstrnew
=======
            flags        : [tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : 'NETWAREUNITS';
            extradefines : 'NETWARE_CLIB';
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
            staticlibprefix : 'libp';
            sharedlibprefix : '';
            sharedClibext : '.nlm';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #13#10;
            dirsep       : '/';
            assem        : as_i386_nlmcoff; // as_i386_elf32;
            assemextern  : as_gas;
            link         : ld_int_netware;
            linkextern   : ld_netware;
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
  {$ifdef netware}
    set_source_info(system_i386_netware_info);
  {$endif netware}
{$endif CPUI386}
end.
