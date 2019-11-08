{
    Copyright (c) 2004 by Free Pascal Development Team

    This unit implements support information structures for MorphOS

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
{ This unit implements support information structures for the MorphOS. }
unit i_morph;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_powerpc_MorphOS_info : tsysteminfo =
          (
            system       : system_powerpc_MorphOS;
            name         : 'MorphOS';
            shortname    : 'MorphOS';
            flags        : [tf_files_case_aware,tf_requires_proper_alignment,tf_smartlink_sections,
                            tf_has_winlike_resources,tf_needs_symbol_type,tf_needs_symbol_size];
            cpu          : cpu_powerpc;
            unit_env     : 'MORPHOSUNITS';
            extradefines : 'HASAMIGA';
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
            sharedlibext : '.library';
            staticlibext : '.a';
            staticlibprefix : 'libp';
            sharedlibprefix : '';
            sharedClibext : '.library';
            staticClibext : '.a';
            staticClibprefix : 'lib';
            sharedClibprefix : '';
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_gas;
            assemextern  : as_gas;
            link         : ld_none;
            linkextern   : ld_morphos;
            ar           : ar_gnu_ar;
            res          : res_elf;
            dbg          : dbg_stabs;
            script       : script_amiga;
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
                recordalignmax  : 4;
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 262144;
            stackalign   : 16;
            abi : abi_powerpc_sysv;
            llvmdatalayout : 'E-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v128:128:128-n32';
          );

  implementation

initialization
{$ifdef CPUPOWERPC}
  {$ifdef MORPHOS}
    set_source_info(system_powerpc_MorphOS_info);
  {$endif MORPHOS}
{$endif CPUPOWERPC}
end.
