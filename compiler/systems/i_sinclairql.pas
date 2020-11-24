{
    Copyright (c) 2020 by Karoly Balogh

    This unit implements support information structures for the Sinclair QL

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
{ This unit implements support information structures for the Sinclair QL. }
unit i_sinclairql;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       system_m68k_sinclairql_info : tsysteminfo =
          (
            system       : system_m68k_sinclairql;
            name         : 'Sinclair QL';
            shortname    : 'sinclairql';
            flags        : [tf_use_8_3,tf_requires_proper_alignment,
                            tf_smartlink_sections,tf_under_development];
            cpu          : cpu_m68k;
            unit_env     : '';
            extradefines : '';
            exeext       : '.exe';
            defext       : '';
            scriptext    : '';
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
            importlibprefix : 'libimp';
            importlibext : '.a';
            Cprefix      : '_';
            newline      : #10;
            dirsep       : '/'; { ... the underlying tools (binutils/vlink/vasm) prefer Unix paths }
            assem        : as_m68k_vasm;
            assemextern  : as_m68k_vasm;
            link         : ld_none;
            linkextern   : ld_sinclairql;
            ar           : ar_gnu_ar;
            res          : res_ext;
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
                maxCrecordalign : 4
              );
            first_parm_offset : 8;
            stacksize    : 8192;
            stackalign   : 2;
            abi : abi_default;
            llvmdatalayout : 'todo';
          );

  implementation

initialization
{$ifdef cpu68}
  {$ifdef atari}
    set_source_info(system_m68k_sinclairql_info);
  {$endif atari}
{$endif cpu68}
end.
