{
    Copyright (c) 2019 by Dmitry Boyarintsev

    This unit implements support information structures for WebAssembly

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

unit i_wasi;

{$i fpcdefs.inc}

  interface

    uses
       systems,rescmn;

    const
       res_wasmraw_info : tresinfo =
           (
             id     : res_none; // todo: not implemented. but could be as memory
             resbin : 'fpcwasmres';
             rescmd : '-o $OBJ $DBG';
             rcbin  : '';
             rccmd  : '';
             resourcefileclass : nil;
             resflags : [res_no_compile];
           );

        system_wasm32_wasi_info : tsysteminfo =
          (
            system       : system_wasm32_wasi;
            name         : 'The WebAssembly System Interface (WASI)';
            shortname    : 'Wasi';
            flags        : [tf_under_development,tf_needs_symbol_size,tf_needs_symbol_type,
                            tf_files_case_sensitive,tf_no_generic_stackcheck,
                            tf_smartlink_sections,
                            { avoid the creation of threadvar tables }
                            tf_section_threadvars];
            cpu          : cpu_wasm32;
            unit_env     : '';
            extradefines : '';
            exeext       : '.wasm';
            defext       : '.def';
            scriptext    : '.sh';
            smartext     : '.sl';
            unitext      : '.ppu';
            unitlibext   : '.ppl';
            asmext       : '.wat';
            objext       : '.o';
            resext       : '';
            resobjext    : '.o';
            sharedlibext : ''; // keep it empty! The sharedlibext drives the export module name
                               // if this is populated, then the name should be cleared when generating import
            staticlibext : '.a';
            staticlibprefix : '';
            sharedlibprefix : '';
            sharedClibext : '.wasm';
            staticClibext : '.wasm';
            staticClibprefix : '';
            sharedClibprefix : '';
            importlibprefix : '';
            importlibext : '.wasm';
            Cprefix      : '';
            newline      : #10;
            dirsep       : '/';
            assem        : as_wasm32_llvm_mc;
            assemextern  : as_wasm32_llvm_mc;
            link         : ld_none;
            linkextern   : ld_wasi; // there's no linker, only object files for WASM
            ar           : ar_none;
            res          : res_none;
            dbg          : dbg_dwarf2;
            script       : script_unix;
            endian       : endian_little;
            alignment    :
              (
                procalign       : 4;
                loopalign       : 4;
                jumpalign       : 0;
                jumpalignskipmax    : 0;
                coalescealign   : 0;
                coalescealignskipmax: 0;
                constalignmin   : 4;
                constalignmax   : 16;
                varalignmin     : 4;
                varalignmax     : 16;
                localalignmin   : 4;
                localalignmax   : 16;
                recordalignmin  : 0;
                recordalignmax  : 16;
                maxCrecordalign : 16
              );
            first_parm_offset : 0;
            stacksize   : 262144;
            stackalign   : 4;
            abi          : abi_default;
            llvmdatalayout : 'todo';
          );


  implementation

initialization
{$ifdef CPUWASM32}
  {$ifdef wasi}
    set_source_info(system_wasm32_wasi_info);
  {$endif wasi}
{$endif CPUWASM32}
end.
