{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit implements support information structures for go32v2

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
{ This unit implements support information structures for go32v2. }
unit i_go32v2;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_i386_go32v2_info : tsysteminfo =
          (
            system       : system_i386_GO32V2;
            name         : 'GO32 V2 DOS extender';
            shortname    : 'Go32v2';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            flags        : [tf_use_8_3,tf_smartlink_sections];
=======
            flags        : [tf_use_8_3,tf_smartlink_library];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_use_8_3,tf_smartlink_library];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_use_8_3,tf_smartlink_library];
>>>>>>> graemeg/cpstrnew
=======
            flags        : [tf_use_8_3,tf_smartlink_library];
>>>>>>> origin/cpstrnew
=======
            flags        : [tf_use_8_3,tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> graemeg/fixes_2_2
=======
            flags        : [tf_use_8_3,tf_use_function_relative_addresses,tf_smartlink_library];
>>>>>>> origin/fixes_2_2
            cpu          : cpu_i386;
            unit_env     : 'GO32V2UNITS';
            extradefines : 'DPMI';
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
            assem        : as_i386_coff;
            assemextern  : as_gas;
            link         : ld_int_go32v2;
            linkextern   : ld_go32v2;
            ar           : ar_gnu_ar_scripted;
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
                recordalignmax  : 2;
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
{$ifdef cpui386}
  {$ifdef go32v2}
    set_source_info(system_i386_go32v2_info);
  {$endif go32v2}
{$endif cpui386}
end.
