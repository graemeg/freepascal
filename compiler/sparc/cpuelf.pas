{
    Copyright (c) 1998-2006 by Peter Vreman

    Includes ELF-related code specific to SPARC

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
unit cpuelf;

{$i fpcdefs.inc}

interface

implementation

  uses
    verbose,elfbase,
    systems,ogbase,ogelf,assemble;

  const
    { Relocation types }
    R_SPARC_NONE = 0;
    R_SPARC_8 = 1;
    R_SPARC_16 = 2;
    R_SPARC_32 = 3;
    R_SPARC_DISP8 = 4;
    R_SPARC_DISP16 = 5;
    R_SPARC_DISP32 = 6;
    R_SPARC_WDISP30 = 7;
    R_SPARC_WDISP22 = 8;
    R_SPARC_HI22 = 9;
    R_SPARC_22 = 10;
    R_SPARC_13 = 11;
    R_SPARC_LO10 = 12;
    R_SPARC_GOT10 = 13;
    R_SPARC_GOT13 = 14;
    R_SPARC_GOT22 = 15;
    R_SPARC_PC10 = 16;
    R_SPARC_PC22 = 17;
    R_SPARC_WPLT30 = 18;
    R_SPARC_COPY = 19;
    R_SPARC_GLOB_DAT = 20;
    R_SPARC_JMP_SLOT = 21;
    R_SPARC_RELATIVE = 22;
    R_SPARC_UA32 = 23;
    R_SPARC_GNU_VTINHERIT = 250;
    R_SPARC_GNU_VTENTRY = 251;


{****************************************************************************
                               ELF Target methods
****************************************************************************}

   function elf_sparc_encodereloc(objrel:TObjRelocation):byte;
     begin
       case objrel.typ of
         RELOC_NONE :
           result:=R_SPARC_NONE;
         RELOC_ABSOLUTE :
           result:=R_SPARC_32;
         { TODO }
       else
         result:=0;
         InternalError(2012082303);
       end;
     end;


   function elf_sparc_relocname(reltyp:byte):string;
     begin
       result:='TODO';
     end;


   procedure elf_sparc.loadreloc(objrel:TObjRelocation);
     begin
     end;



{*****************************************************************************
                                    Initialize
*****************************************************************************}

  const
    elf_target_sparc: TElfTarget =
      (
        max_page_size:     $8000; // fixme
        exe_image_base:    $8000; // fixme
        machine_code:      EM_SPARC;
        relocs_use_addend: false;
        dyn_reloc_codes: (
          R_SPARC_RELATIVE,
          R_SPARC_GLOB_DAT,
          R_SPARC_JUMP_SLOT,
          R_SPARC_COPY,
          0      // IRELATIVE is absent(?)
        );
        relocname:         @elf_sparc_relocName;
        encodereloc:       @elf_sparc_encodeReloc;
        loadreloc:         @elf_sparc_loadReloc;
        loadsection:       nil;
        encodeflags:       nil;
      );

    as_sparc_elf32_info : tasminfo =
       (
         id     : as_sparc_elf32;
         idtxt  : 'ELF';
         asmbin : '';
         asmcmd : '';
         supported_targets : [system_sparc_linux, system_sparc_solaris,
                              system_sparc_embedded];
//            flags : [af_outputbinary,af_smartlink_sections];
         flags : [af_outputbinary,af_supports_dwarf];
         labelprefix : '.L';
         labelmaxlen : -1;
         comment : '';
         dollarsign: '$';
       );

initialization
  RegisterAssembler(as_sparc_elf32_info,TElfAssembler);
  ElfTarget:=elf_target_sparc;
end.

