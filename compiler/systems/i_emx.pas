{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit implements support information structures for OS/2 via EMX

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
{ This unit implements support information structures for OS/2 via EMX. }
unit i_emx;

{$i fpcdefs.inc}

  interface

    uses
       systems;

    const
       res_wrc_os2_info : tresinfo =
          (
             id     : res_watcom_wrc_os2;
             resbin : '';
             rescmd : '';
             rcbin  : 'wrc';
             rccmd  : '-r -zm -q -bt=os2 -dFPC -fo=$RES $RC';
             resourcefileclass : nil;
             resflags : [res_single_file];
          );

       system_i386_emx_info : tsysteminfo =
          (
            system       : system_i386_EMX;
            name         : 'OS/2 via EMX';
            shortname    : 'EMX';
            flags        : [tf_need_export,tf_use_8_3];
            cpu          : cpu_i386;
            unit_env     : 'EMXUNITS';
            extradefines : 'OS2';
            exeext       : '.exe';
            defext       : '.def';
            scriptext    : '.cmd';
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
            staticClibprefix : '';
            sharedClibprefix : '';
            importlibprefix : '';
            importlibext : '.a';
            Cprefix      : '_';
            newline      : #13#10;
            dirsep       : '\';
            assem        : as_i386_as_aout;
            assemextern  : as_i386_as_aout;
            link         : ld_none;
            linkextern   : ld_emx;
            ar           : ar_gnu_ar;
            res          : res_watcom_wrc_os2;
            dbg          : dbg_stabs;
            script       : script_dos;
            endian       : endian_little;
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
            stacksize    : 256*1024;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );


  implementation

initialization
{$ifdef CPUI386}
  {$ifdef EMX}
      set_source_info(system_i386_emx_info);
      { OS/2 via EMX can be run under DOS as well }
      if (OS_Mode=osDOS) or (OS_Mode=osDPMI) then
        source_info.scriptext := '.bat';
  {$endif EMX}
{$endif CPUI386}
end.
