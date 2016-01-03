unit BFD;

{$mode Delphi}
interface

{
  Object Pascal Translation of bfd.h,
  used in the Binary File Descriptor library
  found in the GNU Binutils

  This translation itself is in the public domain, but don't ignore
  the copyright of the BFD lib itself (see 'original comments' below)

  History:
  07.08.2001: utessel@gmx.de (UT) (started translation)
    - the structs, defines and functions are separated to different
      blocks to have one large type block (since type forwards are not
      possible across different type blocks)
    - Maybe some comments were removed or are at wrong places: while
      the codelines were sorted I did not check everything
    - I did a lot of Search and Replace, so don't wonder if comments
      were changed a bit, too.
    - I renamed bfd to TBFD and a few other minor changes to keep the
      namespace a bit cleaner

}

{
  Original comments and copyrights:
}

(* Main header file for the bfd library -- portable access to object files.
   Copyright 1990, 91, 92, 93, 94, 95, 96, 97, 98, 99, 2000
   Free Software Foundation, Inc.
   Contributed by Cygnus Support.

** NOTE: bfd.h and bfd-in2.h are GENERATED files.  Don't change them;
** instead, change bfd-in.h or the other BFD source files processed to
** generate these files.

This file is part of BFD, the Binary File Descriptor library.

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
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  *)

(* bfd.h -- The only header file required by users of the bfd library

The bfd.h file is generated from bfd-in.h and various .c files; if you
change it, your changes will probably be lost.

All the prototypes and definitions following the comment "THE FOLLOWING
IS EXTRACTED FROM THE SOURCE" are extracted from the source files for
BFD.  If you change it, someone oneday will extract it from the source
again, and your changes will be lost.  To save yourself from this bind,
change the definitions in the source in the bfd directory.  Type "make
docs" and then "make headers" in that directory, and magically this file
will change to reflect your changes.

If you don't have the tools to perform the extraction, then you are
safe from someone on your system trampling over your header files.
You should still maintain the equivalence between the source and this
file though; every change you make to the .c file should be reflected
here.  *)

{UT: these settings create matching records: }
{$packrecords 4}
{$ALIGN 4}
{$MINENUMSIZE 4}



const
  BFD_VERSION   = '2.10.91';
  BFD_ARCH_SIZE = 32;
  BFD_LIB_NAME = 'libbfd-'+BFD_VERSION+'.dll';

type
(* forward declaration *)
  PBFD = ^TBFD;
  PPsymbol_cache_entry = ^Psymbol_cache_entry;
  Psymbol_cache_entry = ^symbol_cache_entry;
  PSec = ^sec;
  PPSec = ^PSec;
  Pbfd_target = ^bfd_target;
  Pbfd_arch_info = ^bfd_arch_info;
  PPreloc_cache_entry = ^Preloc_cache_entry;
  Preloc_cache_entry = ^reloc_cache_entry;
  Palent = ^alent;
  Pbfd_comdat_info = ^bfd_comdat_info;
  Prelent_chain = ^relent_chain;
  Pbfd_window = ^bfd_window;
  Pasymbol = ^asymbol;
  Pbfd_size_type = ^bfd_size_type;
  Porl = ^orl;
  Psymbol_info = ^symbol_info;
  PParelent = ^Parelent;
  Parelent = ^arelent;
  Pasection = ^asection;
  PPreloc_howto = ^Preloc_howto;
  Preloc_howto = ^reloc_howto;

  Pbfd_link_info = pointer; //^bfd_link_info;
  Pbfd_link_hash_table = pointer; //^bfd_link_hash_table;

(* A pointer to a position in a file.  *)
  file_ptr = LongInt;

(* Represent a target address.  Also used as a generic unsigned type
   which is guaranteed to be big enough to hold any arithmetic types
   we need to deal with.  *)
  bfd_vma = LongWord;

(* A generic signed type which is guaranteed to be big enough to hold any
   arithmetic types we need to deal with.  Can be assumed to be compatible
   with bfd_vma in the same way that signed and LongWords are compatible
   (as parameters, in assignment, etc).  *)
  bfd_signed_vma = LongInt;

  symvalue = LongWord;
  bfd_size_type = LongWord;

  flagword = LongWord;  (* 32 bits of flags *)

<<<<<<< HEAD
<<<<<<< HEAD
=======
  pppchar = ^ppchar;
>>>>>>> graemeg/fixes_2_2
=======
  pppchar = ^ppchar;
>>>>>>> origin/fixes_2_2
  Pbfd_byte = ^bfd_byte;
  bfd_byte = Byte;
  bfd_boolean = LongBool;

(** File formats *)
(* MvdV: FPC 1.0 doesn't support the construct below, but the first ordinal
is in practice already 0 *)

  bfd_format = (
    bfd_unknown  = 0,  (* file format is unknown *)
    bfd_object,     (* linker/assember/compiler output *)
    bfd_archive,    (* object archive file *)
    bfd_core        (* core dump *)
    //UT: to use this enum in arrays the following is not needed:
    //bfd_type_end  (* marks the end; don't use it! *)
  );

(* symbols and relocation *)

(* A count of carsyms (canonical archive symbols).  *)
  symindex = LongWord;

(* A canonical archive symbol.  *)
(* This is a type pun with struct ranlib on purpose! *)
  carsym = record
    name: pchar;
    file_offset: file_ptr;  (* look here to find the file *)
  end;                      (* to make these you call a carsymogen *)

(* Used in generating armaps (archive tables of contents).
   Perhaps just a forward definition would do? *)
  orl = record                  (* output ranlib *)
    name:    ppchar;            (* symbol name *)
    pos:     file_ptr;          (* TBFD* or file position *)
    nameidx: integer;           (* index into string table *)
  end;

(* Linenumber stuff *)
  lineno_cache_entry = record
    line_number: LongWord;      (* Linenumber from start of function*)
    u: record
      case integer of
        0: (sym:    Psymbol_cache_entry); (* Function name *)
        1: (offset: LongWord);            (* Offset into section *)
      end;
  end;
  alent = lineno_cache_entry;

  Pstat = pointer; // todo?

  bfd_print_symbol = (
    bfd_print_symbol_name,
    bfd_print_symbol_more,
    bfd_print_symbol_all
  );

(* Information about a symbol that nm needs.  *)

  symbol_info = record
    value:     symvalue;
    typ:       ShortInt;
    name:      pchar;    (* Symbol name.  *)
    stab_type: Byte;     (* Stab type.  *)
    stab_other:ShortInt; (* Stab other.  *)
    stab_desc: SmallInt; (* Stab desc.  *)
    stab_name: pchar;    (* String for stab type.  *)
  end;

(* Hash table routines.  There is no way to free up a hash table.  *)

(* An element in the hash table.  Most uses will actually use a larger
   structure, and an instance of this will be the first field.  *)

   Pbfd_hash_entry = ^bfd_hash_entry;
   bfd_hash_entry = record
     (* Next entry for this hash code.  *)
     next: Pbfd_hash_entry;
     (* String being hashed.  *)
     strng: pchar;
     (* Hash code.  This is the full hash code, not the index into the
        table.  *)
     hash: LongWord;
   end;

(* A hash table.  *)
  Pbfd_hash_table = ^bfd_hash_table;

  bfd_hash_table_newfunc = function( entry: Pbfd_hash_entry;
    table: Pbfd_hash_table; v: pchar): Pbfd_hash_entry;

  bfd_hash_table = record
    (* The hash array.  *)
    table: ^Pbfd_hash_entry;
    (* The number of slots in the hash table.  *)
    size: LongWord ;
    (* A function used to create new elements in the hash table.  The
       first entry is itself a pointer to an element.  When this
       function is first invoked, this pointer will be NULL.  However,
       having the pointer permits a hierarchy of method functions to be
       built each of which calls the function in the superclass.  Thus
       each function should be written to allocate a new block of memory
       only if the argument is NULL.  *)
    newfunc: bfd_hash_table_newfunc;
     (* An objalloc for this hash table.  This is a struct objalloc *,
       but we use pointer to avoid requiring the inclusion of objalloc.h.  *)
    memory: pointer;
  end;

  Pbfd_window_internal = pointer;

  bfd_window = record
    (* What the user asked for.  *)
    data: pointer;
    size: bfd_size_type;
    (* The actual window used by BFD.  Small user-requested read-only
       regions sharing a page may share a single window into the object
       file.  Read-write versions shouldn't until I've fixed things to
       keep track of which portions have been claimed by the
       application; don't want to give the same region back when the
       application wants two writable copies!  *)
    i: Pbfd_window_internal;
  end;

  Pbfd_link_needed_lists = ^bfd_link_needed_lists;
  bfd_link_needed_lists = record
    next: Pbfd_link_needed_lists;
    by: PBFD;
    name: pchar;
  end;

  bfd_direction = (no_direction    = 0,
                   read_direction  = 1,
                   write_direction = 2,
                   both_direction  = 3);

  TBFD = record
    (* The filename the application opened the BFD with.  *)
     filename: pchar;

    (* A pointer to the target jump table.             *)
    xvec: Pbfd_target;

    (* To avoid dragging too many header files into every file that
       includes `<<bfd.h>>', IOSTREAM has been declared as a "ShortInt
       *", and MTIME as a "LongInt".  Their correct types, to which they
       are cast when used, are "FILE *" and "time_t".    The iostream
       is the result of an fopen on the filename.  However, if the
       BFD_IN_MEMORY flag is set, then iostream is actually a pointer
       to a bfd_in_memory struct.  *)
    iostream: pointer ;

    (* Is the file descriptor being cached?  That is, can it be closed as
       needed, and re-opened when accessed later?  *)

    cacheable: bfd_boolean ;

    (* Marks whether there was a default target specified when the
       BFD was opened. This is used to select which matching algorithm
       to use to choose the back end. *)

    target_defaulted: bfd_boolean ;

    (* The caching routines use these to maintain a
       least-recently-used list of BFDs *)

    lru_prev, lru_next: PBFD;

    (* When a file is closed by the caching routines, BFD retains
       state information on the file here: *)

    where: file_ptr;

    (* and here: (``once'' means at least once) *)

    opened_once: bfd_boolean;

    (* Set if we have a locally maintained mtime value, rather than
       getting it from the file each time: *)

    mtime_set: bfd_boolean;

    (* File modified time, if mtime_set is true: *)

    mtime: LongInt;

    (* Reserved for an unimplemented file locking extension.*)

    ifd: integer ;

    (* The format which belongs to the BFD. (object, core, etc.) *)

    format: bfd_format;

    (* The direction the BFD was opened with*)

    direction: bfd_direction;

    (* Format_specific flags*)

    flags: flagword;

    (* Currently my_archive is tested before adding origin to
       anything. I believe that this can become always an add of
       origin, with origin set to 0 for non archive files.   *)

    origin: file_ptr;

    (* Remember when output has begun, to stop strange things
       from happening. *)
    output_has_begun: bfd_boolean ;

    (* Pointer to linked list of sections*)
    sections: Psec;

    (* The number of sections *)
    section_count: LongWord ;

    (* Stuff only useful for object files:
       The start address. *)
    start_address: bfd_vma ;

    (* Used for input and output*)
    symcount: LongWord ;

    (* Symbol table for output BFD (with symcount entries) *)
    outsymbols: PPsymbol_cache_entry;

    (* Pointer to structure which contains architecture information*)
    arch_info: Pbfd_arch_info;

    (* Stuff only useful for archives:*)
    arelt_data: pointer ;
    my_archive: PBFD ;     (* The containing archive BFD.  *)
    next: PBFD ;           (* The next BFD in the archive.  *)
    archive_head: PBFD ;   (* The first BFD in the archive.  *)
    has_armap: bfd_boolean ;

    (* A chain of BFD structures involved in a link.  *)
    link_next: PBFD ;

    (* A field used by _bfd_generic_link_add_archive_symbols.  This will
       be used only for archive elements.  *)
    archive_pass: integer ;

    (* Used by the back end to hold private data. *)

    tdata: record
    case integer of
    { TODO: (?)
      struct aout_data_struct *aout_data;
      struct artdata *aout_ar_data;
      struct _oasys_data *oasys_obj_data;
      struct _oasys_ar_data *oasys_ar_data;
      struct coff_tdata *coff_obj_data;
      struct pe_tdata *pe_obj_data;
      struct xcoff_tdata *xcoff_obj_data;
      struct ecoff_tdata *ecoff_obj_data;
      struct ieee_data_struct *ieee_data;
      struct ieee_ar_data_struct *ieee_ar_data;
      struct srec_data_struct *srec_data;
      struct ihex_data_struct *ihex_data;
      struct tekhex_data_struct *tekhex_data;
      struct elf_obj_tdata *elf_obj_data;
      struct nlm_obj_tdata *nlm_obj_data;
      struct bout_data_struct *bout_data;
      struct sun_core_struct *sun_core_data;
      struct sco5_core_struct *sco5_core_data;
      struct trad_core_struct *trad_core_data;
      struct som_data_struct *som_data;
      struct hpux_core_struct *hpux_core_data;
      struct hppabsd_core_struct *hppabsd_core_data;
      struct sgi_core_struct *sgi_core_data;
      struct lynx_core_struct *lynx_core_data;
      struct osf_core_struct *osf_core_data;
      struct cisco_core_struct *cisco_core_data;
      struct versados_data_struct *versados_data;
      struct netbsd_core_struct *netbsd_core_data;}
      -1:(any: pointer);
    end;

    (* Used by the application to hold private data*)
    usrdata: pointer ;

  (* Where all the allocated stuff under this BFD goes.  This is a
     struct objalloc *, but we use pointer to avoid requiring the inclusion of
     objalloc.h.  *)
    memory: pointer ;
  end; { TBFD }

  symbol_cache_entry = record
       (* A pointer to the BFD which owns the symbol. This information
          is necessary so that a back end can work out what additional
          information (invisible to the application writer) is carried
          with the symbol.

          This field is *almost* redundant, since you can use section->owner
          instead, except that some symbols point to the global sections
          bfd_{abs,com,und}_section.  This could be fixed by making
          these globals be per-bfd (or per-target-flavor).  FIXME. *)

    the_bfd: PBFD; (* Use bfd_asymbol_bfd(sym) to access this field. *)

       (* The text of the symbol. The name is left alone, and not copied; the
          application may not alter it. *)
    name: pchar;

       (* The value of the symbol.  This really should be a union of a
          numeric value with a pointer, since some flags indicate that
          a pointer to another symbol is stored here.  *)
    value: symvalue;

    flags: flagword;

       (* A pointer to the section to which this symbol is
          relative.  This will always be non NULL, there are special
          sections for undefined and absolute symbols.  *)
    section: Psec;

    (* Back end special data.  *)
     udata: record
     case bfd_boolean of
       false: (p: pointer);
       true:  (i: bfd_vma);
     end;
  end; { symbol_cache_entry }
  asymbol = symbol_cache_entry;

  Pbfd_link_order = pointer; // todo?

  sec = record
    (* The name of the section; the name isn't a copy, the pointer is
     the same as that passed to bfd_make_section.  *)
    name: pchar;

    (* A unique sequence number.  *)
    id: integer;

    (* Which section is it; 0..nth.  *)
    index: integer;

    (* The next section in the list belonging to the BFD, or NULL.  *)
    next: Psec ;

    (* The field flags contains attributes of the section. Some
       flags are read in from the object file, and some are
       synthesized from other information.  *)
    flags: flagword;

    (* Some internal packed bfd_boolean fields.  *)

    { UT: I replaced this bitfield with a Pascal 'set'!
      the code was
        user_set_vma: LongWord : 1;
        reloc_done: LongWord : 1;
        linker_mark: LongWord : 1;
        gc_mark: LongWord : 1;
        segment_mark: LongWord : 1; }
    bitfield: set of (
      user_set_vma, (* See the vma field.  *)
      reloc_done,   (* Whether relocations have been processed.  *)
      linker_mark,  (* A mark flag used by some of the linker backends.  *)
      gc_mark,      (* A mark flag used by some linker backends for garbage collection.  *)
      segment_mark  (* Used by the ELF code to mark sections which have been allocated to segments.  *)
    );
    (* End of internal packed bfd_boolean fields.  *)

    (*  The virtual memory address of the section - where it will be
        at run time.  The symbols are relocated against this.  The
        user_set_vma flag is maintained by bfd; if it's not set, the
        backend can assign addresses (for example, in <<a.out>>, where
        the default address for <<.data>> is dependent on the specific
        target and various flags).  *)
    vma: bfd_vma;

    (*  The load address of the section - where it would be in a
        rom image; really only used for writing section header
        information. *)
    lma: bfd_vma;

    (* The size of the section in octets, as it will be output.
       Contains a value even if the section has no contents (e.g., the
       size of <<.bss>>).  This will be filled in after relocation.  *)
    _cooked_size: bfd_size_type;

    (* The original size on disk of the section, in octets.  Normally this
       value is the same as the size, but if some relaxing has
       been done, then this value will be bigger.  *)
    _raw_size: bfd_size_type;

    (* If this section is going to be output, then this value is the
       offset in *bytes* into the output section of the first byte in the
       input section (byte ==> smallest addressable unit on the
       target).  In most cases, if this was going to start at the
       100th octet (8-bit quantity) in the output section, this value
       would be 100.  However, if the target byte size is 16 bits
       (bfd_octets_per_byte is "2"), this value would be 50.  *)
    output_offset: bfd_vma;

    (* The output section through which to map on output.  *)
    output_section: Psec ;

    (* The alignment requirement of the section, as an exponent of 2 -
       e.g., 3 aligns to 2^3 (or 8).  *)
    alignment_power: LongWord;

    (* If an input section, a pointer to a vector of relocation
       records for the data in this section.  *)
    relocation: Preloc_cache_entry;

    (* If an output section, a pointer to a vector of pointers to
       relocation records for the data in this section.  *)
    orelocation: PPreloc_cache_entry;

    (* The number of relocation records in one of the above  *)
    reloc_count: LongWord;

    (* Information below is back end specific - and not always used
       or updated.  *)

    (* File position of section data.  *)
    filepos: file_ptr;

    (* File position of relocation info.  *)
    rel_filepos: file_ptr;

    (* File position of line data.  *)
    line_filepos: file_ptr;

    (* Pointer to data for applications.  *)
    userdata: pointer;

    (* If the SEC_IN_MEMORY flag is set, this points to the actual
       contents.  *)
    contents: PByte;

    (* Attached line number information.  *)
    lineno: Palent;

    (* Number of line number records.  *)
    lineno_count: LongWord;

    (* Optional information about a COMDAT entry; NULL if not COMDAT.  *)
    comdat: Pbfd_comdat_info;

    (* Points to the kept section if this section is a link-once section,
       and is discarded.  *)
    kept_section: Psec;

    (* When a section is being output, this value changes as more
       linenumbers are written out.  *)
    moving_line_filepos: file_ptr;

    (* What the section number is in the target world.  *)
    target_index: integer;

    used_by_bfd: pointer;

    (* If this is a constructor section then here is a list of the
       relocations created to relocate items within it.  *)
    constructor_chain: Prelent_chain;

    (* The BFD which owns the section.  *)
    owner: PBFD;

    (* A symbol which points at this section only *)
    symbol: Psymbol_cache_entry;
    symbol_ptr_ptr: PPsymbol_cache_entry;

    link_order_head: Pbfd_link_order;
    link_order_tail: Pbfd_link_order;
  end; { sec }
  asection = sec;

  bfd_flavour = (
    bfd_target_unknown_flavour,
    bfd_target_aout_flavour,
    bfd_target_coff_flavour,
    bfd_target_ecoff_flavour,
    bfd_target_xcoff_flavour,
    bfd_target_elf_flavour,
    bfd_target_ieee_flavour,
    bfd_target_nlm_flavour,
    bfd_target_oasys_flavour,
    bfd_target_tekhex_flavour,
    bfd_target_srec_flavour,
    bfd_target_ihex_flavour,
    bfd_target_som_flavour,
    bfd_target_os9k_flavour,
    bfd_target_versados_flavour,
    bfd_target_msdos_flavour,
    bfd_target_ovax_flavour,
    bfd_target_evax_flavour
  ); { bfd_flavour }

  bfd_endian = ( BFD_ENDIAN_BIG, BFD_ENDIAN_LITTLE, BFD_ENDIAN_UNKNOWN );

  bfd_reloc_code_real = (
    _dummy_first_bfd_reloc_code_real,

    (* Basic absolute relocations of N bits. *)
    BFD_RELOC_64,
    BFD_RELOC_32,
    BFD_RELOC_26,
    BFD_RELOC_24,
    BFD_RELOC_16,
    BFD_RELOC_14,
    BFD_RELOC_8,

  (* PC-relative relocations.  Sometimes these are relative to the address
  of the relocation itself; sometimes they are relative to the start of
  the section containing the relocation.  It depends on the specific target.

  The 24-bit relocation is used in some Intel 960 configurations. *)
    BFD_RELOC_64_PCREL,
    BFD_RELOC_32_PCREL,
    BFD_RELOC_24_PCREL,
    BFD_RELOC_16_PCREL,
    BFD_RELOC_12_PCREL,
    BFD_RELOC_8_PCREL,

  (* For ELF. *)
    BFD_RELOC_32_GOT_PCREL,
    BFD_RELOC_16_GOT_PCREL,
    BFD_RELOC_8_GOT_PCREL,
    BFD_RELOC_32_GOTOFF,
    BFD_RELOC_16_GOTOFF,
    BFD_RELOC_LO16_GOTOFF,
    BFD_RELOC_HI16_GOTOFF,
    BFD_RELOC_HI16_S_GOTOFF,
    BFD_RELOC_8_GOTOFF,
    BFD_RELOC_32_PLT_PCREL,
    BFD_RELOC_24_PLT_PCREL,
    BFD_RELOC_16_PLT_PCREL,
    BFD_RELOC_8_PLT_PCREL,
    BFD_RELOC_32_PLTOFF,
    BFD_RELOC_16_PLTOFF,
    BFD_RELOC_LO16_PLTOFF,
    BFD_RELOC_HI16_PLTOFF,
    BFD_RELOC_HI16_S_PLTOFF,
    BFD_RELOC_8_PLTOFF,

  (* Relocations used by 68K ELF. *)
    BFD_RELOC_68K_GLOB_DAT,
    BFD_RELOC_68K_JMP_SLOT,
    BFD_RELOC_68K_RELATIVE,

  (* Linkage-table relative. *)
    BFD_RELOC_32_BASEREL,
    BFD_RELOC_16_BASEREL,
    BFD_RELOC_LO16_BASEREL,
    BFD_RELOC_HI16_BASEREL,
    BFD_RELOC_HI16_S_BASEREL,
    BFD_RELOC_8_BASEREL,
    BFD_RELOC_RVA,

  (* Absolute 8-bit relocation, but used to form an address like $FFnn. *)
    BFD_RELOC_8_FFnn,

  (* These PC-relative relocations are stored as word displacements --
  i.e., byte displacements shifted right two bits.  The 30-bit word
  displacement (<<32_PCREL_S2>> -- 32 bits, shifted 2) is used on the
  SPARC.  (SPARC tools generally refer to this as <<WDISP30>>.)  The
  signed 16-bit displacement is used on the MIPS, and the 23-bit
  displacement is used on the Alpha. *)
    BFD_RELOC_32_PCREL_S2,
    BFD_RELOC_16_PCREL_S2,
    BFD_RELOC_23_PCREL_S2,

  (* High 22 bits and low 10 bits of 32-bit value, placed into lower bits of
  the target word.  These are used on the SPARC. *)
    BFD_RELOC_HI22,
    BFD_RELOC_LO10,

  (* For systems that allocate a Global Pointer register, these are
  displacements off that register.  These relocation types are
  handled specially, because the value the register will have is
  decided relatively late. *)
    BFD_RELOC_GPREL16,
    BFD_RELOC_GPREL32,

  (* Reloc types used for i960/b.out. *)
    BFD_RELOC_I960_CALLJ,

  (* SPARC ELF relocations.  There is probably some overlap with other
  relocation types already defined. *)
    BFD_RELOC_NONE,
    BFD_RELOC_SPARC_WDISP22,
    BFD_RELOC_SPARC22,
    BFD_RELOC_SPARC13,
    BFD_RELOC_SPARC_GOT10,
    BFD_RELOC_SPARC_GOT13,
    BFD_RELOC_SPARC_GOT22,
    BFD_RELOC_SPARC_PC10,
    BFD_RELOC_SPARC_PC22,
    BFD_RELOC_SPARC_WPLT30,
    BFD_RELOC_SPARC_COPY,
    BFD_RELOC_SPARC_GLOB_DAT,
    BFD_RELOC_SPARC_JMP_SLOT,
    BFD_RELOC_SPARC_RELATIVE,
    BFD_RELOC_SPARC_UA32,

  (* I think these are specific to SPARC a.out (e.g., Sun 4). *)
    BFD_RELOC_SPARC_BASE13,
    BFD_RELOC_SPARC_BASE22,

  (* SPARC64 relocations *)
    BFD_RELOC_SPARC_10,
    BFD_RELOC_SPARC_11,
    BFD_RELOC_SPARC_OLO10,
    BFD_RELOC_SPARC_HH22,
    BFD_RELOC_SPARC_HM10,
    BFD_RELOC_SPARC_LM22,
    BFD_RELOC_SPARC_PC_HH22,
    BFD_RELOC_SPARC_PC_HM10,
    BFD_RELOC_SPARC_PC_LM22,
    BFD_RELOC_SPARC_WDISP16,
    BFD_RELOC_SPARC_WDISP19,
    BFD_RELOC_SPARC_7,
    BFD_RELOC_SPARC_6,
    BFD_RELOC_SPARC_5,
    BFD_RELOC_SPARC_PLT64,
    BFD_RELOC_SPARC_HIX22,
    BFD_RELOC_SPARC_LOX10,
    BFD_RELOC_SPARC_H44,
    BFD_RELOC_SPARC_M44,
    BFD_RELOC_SPARC_L44,
    BFD_RELOC_SPARC_REGISTER,

  (* SPARC little endian relocation *)
    BFD_RELOC_SPARC_REV32,

  (* Alpha ECOFF and ELF relocations.  Some of these treat the symbol or
  "addend" in some special way.
  For GPDISP_HI16 ("gpdisp") relocations, the symbol is ignored when
  writing; when reading, it will be the absolute section symbol.  The
  addend is the displacement in bytes of the "lda" instruction from
  the "ldah" instruction (which is at the address of this reloc). *)
    BFD_RELOC_ALPHA_GPDISP_HI16,

  (* For GPDISP_LO16 ("ignore") relocations, the symbol is handled as
  with GPDISP_HI16 relocs.  The addend is ignored when writing the
  relocations out, and is filled in with the file's GP value on
  reading, for convenience. *)
    BFD_RELOC_ALPHA_GPDISP_LO16,

  (* The ELF GPDISP relocation is exactly the same as the GPDISP_HI16
  relocation except that there is no accompanying GPDISP_LO16
  relocation. *)
    BFD_RELOC_ALPHA_GPDISP,

  (* The Alpha LITERAL/LITUSE relocs are produced by a symbol reference;
  the assembler turns it into a LDQ instruction to load the address of
  the symbol, and then fills in a register in the real instruction.

  The LITERAL reloc, at the LDQ instruction, refers to the .lita
  section symbol.  The addend is ignored when writing, but is filled
  in with the file's GP value on reading, for convenience, as with the
  GPDISP_LO16 reloc.

  The ELF_LITERAL reloc is somewhere between 16_GOTOFF and GPDISP_LO16.
  It should refer to the symbol to be referenced, as with 16_GOTOFF,
  but it generates output not based on the position within the .got
  section, but relative to the GP value chosen for the file during the
  final link stage.

  The LITUSE reloc, on the instruction using the loaded address, gives
  information to the linker that it might be able to use to optimize
  away some literal section references.  The symbol is ignored (read
  as the absolute section symbol), and the "addend" indicates the type
  of instruction using the register:
  1 - "memory" fmt insn
  2 - byte-manipulation (byte offset reg)
  3 - jsr (target of branch)

  The GNU linker currently doesn't do any of this optimizing. *)
    BFD_RELOC_ALPHA_LITERAL,
    BFD_RELOC_ALPHA_ELF_LITERAL,
    BFD_RELOC_ALPHA_LITUSE,

  (* The BFD_RELOC_ALPHA_USER_* relocations are used by the assembler to
  process the explicit !<reloc>!sequence relocations, and are mapped
  into the normal relocations at the end of processing. *)
    BFD_RELOC_ALPHA_USER_LITERAL,
    BFD_RELOC_ALPHA_USER_LITUSE_BASE,
    BFD_RELOC_ALPHA_USER_LITUSE_BYTOFF,
    BFD_RELOC_ALPHA_USER_LITUSE_JSR,
    BFD_RELOC_ALPHA_USER_GPDISP,
    BFD_RELOC_ALPHA_USER_GPRELHIGH,
    BFD_RELOC_ALPHA_USER_GPRELLOW,

  (* The HINT relocation indicates a value that should be filled into the
  "hint" field of a jmp/jsr/ret instruction, for possible branch-
  prediction logic which may be provided on some processors. *)
    BFD_RELOC_ALPHA_HINT,

  (* The LINKAGE relocation outputs a linkage pair in the object file,
  which is filled by the linker. *)
    BFD_RELOC_ALPHA_LINKAGE,

  (* The CODEADDR relocation outputs a STO_CA in the object file,
  which is filled by the linker. *)
    BFD_RELOC_ALPHA_CODEADDR,

  (* Bits 27..2 of the relocation address shifted right 2 bits;
  simple reloc otherwise. *)
    BFD_RELOC_MIPS_JMP,

  (* The MIPS16 jump instruction. *)
    BFD_RELOC_MIPS16_JMP,

  (* MIPS16 GP relative reloc. *)
    BFD_RELOC_MIPS16_GPREL,

  (* High 16 bits of 32-bit value; simple reloc. *)
    BFD_RELOC_HI16,

  (* High 16 bits of 32-bit value but the low 16 bits will be sign
  extended and added to form the final result.  If the low 16
  bits form a negative number, we need to add one to the high value
  to compensate for the borrow when the low bits are added. *)
    BFD_RELOC_HI16_S,

  (* Low 16 bits. *)
    BFD_RELOC_LO16,

  (* Like BFD_RELOC_HI16_S, but PC relative. *)
    BFD_RELOC_PCREL_HI16_S,

  (* Like BFD_RELOC_LO16, but PC relative. *)
    BFD_RELOC_PCREL_LO16,

  (* Relocation relative to the global pointer. *)

  (* Relocation against a MIPS literal section. *)
    BFD_RELOC_MIPS_LITERAL,

  (* MIPS ELF relocations. *)
    BFD_RELOC_MIPS_GOT16,
    BFD_RELOC_MIPS_CALL16,
    BFD_RELOC_MIPS_GOT_HI16,
    BFD_RELOC_MIPS_GOT_LO16,
    BFD_RELOC_MIPS_CALL_HI16,
    BFD_RELOC_MIPS_CALL_LO16,
    BFD_RELOC_MIPS_SUB,
    BFD_RELOC_MIPS_GOT_PAGE,
    BFD_RELOC_MIPS_GOT_OFST,
    BFD_RELOC_MIPS_GOT_DISP,

  (* i386/elf relocations *)
    BFD_RELOC_386_GOT32,
    BFD_RELOC_386_PLT32,
    BFD_RELOC_386_COPY,
    BFD_RELOC_386_GLOB_DAT,
    BFD_RELOC_386_JUMP_SLOT,
    BFD_RELOC_386_RELATIVE,
    BFD_RELOC_386_GOTOFF,
    BFD_RELOC_386_GOTPC,

  (* x86-64/elf relocations *)
    BFD_RELOC_X86_64_GOT32,
    BFD_RELOC_X86_64_PLT32,
    BFD_RELOC_X86_64_COPY,
    BFD_RELOC_X86_64_GLOB_DAT,
    BFD_RELOC_X86_64_JUMP_SLOT,
    BFD_RELOC_X86_64_RELATIVE,
    BFD_RELOC_X86_64_GOTPCREL,
    BFD_RELOC_X86_64_32S,

  (* ns32k relocations *)
    BFD_RELOC_NS32K_IMM_8,
    BFD_RELOC_NS32K_IMM_16,
    BFD_RELOC_NS32K_IMM_32,
    BFD_RELOC_NS32K_IMM_8_PCREL,
    BFD_RELOC_NS32K_IMM_16_PCREL,
    BFD_RELOC_NS32K_IMM_32_PCREL,
    BFD_RELOC_NS32K_DISP_8,
    BFD_RELOC_NS32K_DISP_16,
    BFD_RELOC_NS32K_DISP_32,
    BFD_RELOC_NS32K_DISP_8_PCREL,
    BFD_RELOC_NS32K_DISP_16_PCREL,
    BFD_RELOC_NS32K_DISP_32_PCREL,

  (* Picojava relocs.  Not all of these appear in object files. *)
    BFD_RELOC_PJ_CODE_HI16,
    BFD_RELOC_PJ_CODE_LO16,
    BFD_RELOC_PJ_CODE_DIR16,
    BFD_RELOC_PJ_CODE_DIR32,
    BFD_RELOC_PJ_CODE_REL16,
    BFD_RELOC_PJ_CODE_REL32,

  (* Power(rs6000) and PowerPC relocations. *)
    BFD_RELOC_PPC_B26,
    BFD_RELOC_PPC_BA26,
    BFD_RELOC_PPC_TOC16,
    BFD_RELOC_PPC_B16,
    BFD_RELOC_PPC_B16_BRTAKEN,
    BFD_RELOC_PPC_B16_BRNTAKEN,
    BFD_RELOC_PPC_BA16,
    BFD_RELOC_PPC_BA16_BRTAKEN,
    BFD_RELOC_PPC_BA16_BRNTAKEN,
    BFD_RELOC_PPC_COPY,
    BFD_RELOC_PPC_GLOB_DAT,
    BFD_RELOC_PPC_JMP_SLOT,
    BFD_RELOC_PPC_RELATIVE,
    BFD_RELOC_PPC_LOCAL24PC,
    BFD_RELOC_PPC_EMB_NADDR32,
    BFD_RELOC_PPC_EMB_NADDR16,
    BFD_RELOC_PPC_EMB_NADDR16_LO,
    BFD_RELOC_PPC_EMB_NADDR16_HI,
    BFD_RELOC_PPC_EMB_NADDR16_HA,
    BFD_RELOC_PPC_EMB_SDAI16,
    BFD_RELOC_PPC_EMB_SDA2I16,
    BFD_RELOC_PPC_EMB_SDA2REL,
    BFD_RELOC_PPC_EMB_SDA21,
    BFD_RELOC_PPC_EMB_MRKREF,
    BFD_RELOC_PPC_EMB_RELSEC16,
    BFD_RELOC_PPC_EMB_RELST_LO,
    BFD_RELOC_PPC_EMB_RELST_HI,
    BFD_RELOC_PPC_EMB_RELST_HA,
    BFD_RELOC_PPC_EMB_BIT_FLD,
    BFD_RELOC_PPC_EMB_RELSDA,

  (* IBM 370/390 relocations *)
    BFD_RELOC_I370_D12,

  (* The type of reloc used to build a contructor table - at the moment
  probably a 32 bit wide absolute relocation, but the target can choose.
  It generally does map to one of the other relocation types. *)
    BFD_RELOC_CTOR,

  (* ARM 26 bit pc-relative branch.  The lowest two bits must be zero and are
  not stored in the instruction. *)
    BFD_RELOC_ARM_PCREL_BRANCH,

  (* ARM 26 bit pc-relative branch.  The lowest bit must be zero and is
  not stored in the instruction.  The 2nd lowest bit comes from a 1 bit
  field in the instruction. *)
    BFD_RELOC_ARM_PCREL_BLX,

  (* Thumb 22 bit pc-relative branch.  The lowest bit must be zero and is
  not stored in the instruction.  The 2nd lowest bit comes from a 1 bit
  field in the instruction. *)
    BFD_RELOC_THUMB_PCREL_BLX,

  (* These relocs are only used within the ARM assembler.  They are not
  (at present) written to any object files. *)
    BFD_RELOC_ARM_IMMEDIATE,
    BFD_RELOC_ARM_ADRL_IMMEDIATE,
    BFD_RELOC_ARM_OFFSET_IMM,
    BFD_RELOC_ARM_SHIFT_IMM,
    BFD_RELOC_ARM_SWI,
    BFD_RELOC_ARM_MULTI,
    BFD_RELOC_ARM_CP_OFF_IMM,
    BFD_RELOC_ARM_ADR_IMM,
    BFD_RELOC_ARM_LDR_IMM,
    BFD_RELOC_ARM_LITERAL,
    BFD_RELOC_ARM_IN_POOL,
    BFD_RELOC_ARM_OFFSET_IMM8,
    BFD_RELOC_ARM_HWLITERAL,
    BFD_RELOC_ARM_THUMB_ADD,
    BFD_RELOC_ARM_THUMB_IMM,
    BFD_RELOC_ARM_THUMB_SHIFT,
    BFD_RELOC_ARM_THUMB_OFFSET,
    BFD_RELOC_ARM_GOT12,
    BFD_RELOC_ARM_GOT32,
    BFD_RELOC_ARM_JUMP_SLOT,
    BFD_RELOC_ARM_COPY,
    BFD_RELOC_ARM_GLOB_DAT,
    BFD_RELOC_ARM_PLT32,
    BFD_RELOC_ARM_RELATIVE,
    BFD_RELOC_ARM_GOTOFF,
    BFD_RELOC_ARM_GOTPC,

  (* Hitachi SH relocs.  Not all of these appear in object files. *)
    BFD_RELOC_SH_PCDISP8BY2,
    BFD_RELOC_SH_PCDISP12BY2,
    BFD_RELOC_SH_IMM4,
    BFD_RELOC_SH_IMM4BY2,
    BFD_RELOC_SH_IMM4BY4,
    BFD_RELOC_SH_IMM8,
    BFD_RELOC_SH_IMM8BY2,
    BFD_RELOC_SH_IMM8BY4,
    BFD_RELOC_SH_PCRELIMM8BY2,
    BFD_RELOC_SH_PCRELIMM8BY4,
    BFD_RELOC_SH_SWITCH16,
    BFD_RELOC_SH_SWITCH32,
    BFD_RELOC_SH_USES,
    BFD_RELOC_SH_COUNT,
    BFD_RELOC_SH_ALIGN,
    BFD_RELOC_SH_CODE,
    BFD_RELOC_SH_DATA,
    BFD_RELOC_SH_LABEL,
    BFD_RELOC_SH_LOOP_START,
    BFD_RELOC_SH_LOOP_END,
    BFD_RELOC_SH_COPY,
    BFD_RELOC_SH_GLOB_DAT,
    BFD_RELOC_SH_JMP_SLOT,
    BFD_RELOC_SH_RELATIVE,
    BFD_RELOC_SH_GOTPC,

  (* Thumb 23-, 12- and 9-bit pc-relative branches.  The lowest bit must
  be zero and is not stored in the instruction. *)
    BFD_RELOC_THUMB_PCREL_BRANCH9,
    BFD_RELOC_THUMB_PCREL_BRANCH12,
    BFD_RELOC_THUMB_PCREL_BRANCH23,

  (* ARC Cores relocs.
  ARC 22 bit pc-relative branch.  The lowest two bits must be zero and are
  not stored in the instruction.  The high 20 bits are installed in bits 26
  through 7 of the instruction. *)
    BFD_RELOC_ARC_B22_PCREL,

  (* ARC 26 bit absolute branch.  The lowest two bits must be zero and are not
  stored in the instruction.  The high 24 bits are installed in bits 23
  through 0. *)
    BFD_RELOC_ARC_B26,

  (* Mitsubishi D10V relocs.
  This is a 10-bit reloc with the right 2 bits
  assumed to be 0. *)
    BFD_RELOC_D10V_10_PCREL_R,

  (* Mitsubishi D10V relocs.
  This is a 10-bit reloc with the right 2 bits
  assumed to be 0.  This is the same as the previous reloc
  except it is in the left container, i.e.,
  shifted left 15 bits. *)
    BFD_RELOC_D10V_10_PCREL_L,

  (* This is an 18-bit reloc with the right 2 bits
  assumed to be 0. *)
    BFD_RELOC_D10V_18,

  (* This is an 18-bit reloc with the right 2 bits
  assumed to be 0. *)
    BFD_RELOC_D10V_18_PCREL,

  (* Mitsubishi D30V relocs.
  This is a 6-bit absolute reloc. *)
    BFD_RELOC_D30V_6,

  (* This is a 6-bit pc-relative reloc with
  the right 3 bits assumed to be 0. *)
    BFD_RELOC_D30V_9_PCREL,

  (* This is a 6-bit pc-relative reloc with
  the right 3 bits assumed to be 0. Same
  as the previous reloc but on the right side
  of the container. *)
    BFD_RELOC_D30V_9_PCREL_R,

  (* This is a 12-bit absolute reloc with the
  right 3 bitsassumed to be 0. *)
    BFD_RELOC_D30V_15,

  (* This is a 12-bit pc-relative reloc with
  the right 3 bits assumed to be 0. *)
    BFD_RELOC_D30V_15_PCREL,

  (* This is a 12-bit pc-relative reloc with
  the right 3 bits assumed to be 0. Same
  as the previous reloc but on the right side
  of the container. *)
    BFD_RELOC_D30V_15_PCREL_R,

  (* This is an 18-bit absolute reloc with
  the right 3 bits assumed to be 0. *)
    BFD_RELOC_D30V_21,

  (* This is an 18-bit pc-relative reloc with
  the right 3 bits assumed to be 0. *)
    BFD_RELOC_D30V_21_PCREL,

  (* This is an 18-bit pc-relative reloc with
  the right 3 bits assumed to be 0. Same
  as the previous reloc but on the right side
  of the container. *)
    BFD_RELOC_D30V_21_PCREL_R,

  (* This is a 32-bit absolute reloc. *)
    BFD_RELOC_D30V_32,

  (* This is a 32-bit pc-relative reloc. *)
    BFD_RELOC_D30V_32_PCREL,

  (* Mitsubishi M32R relocs.
  This is a 24 bit absolute address. *)
    BFD_RELOC_M32R_24,

  (* This is a 10-bit pc-relative reloc with the right 2 bits assumed to be 0. *)
    BFD_RELOC_M32R_10_PCREL,

  (* This is an 18-bit reloc with the right 2 bits assumed to be 0. *)
    BFD_RELOC_M32R_18_PCREL,

  (* This is a 26-bit reloc with the right 2 bits assumed to be 0. *)
    BFD_RELOC_M32R_26_PCREL,

  (* This is a 16-bit reloc containing the high 16 bits of an address
  used when the lower 16 bits are treated as unsigned. *)
    BFD_RELOC_M32R_HI16_ULO,

  (* This is a 16-bit reloc containing the high 16 bits of an address
  used when the lower 16 bits are treated as signed. *)
    BFD_RELOC_M32R_HI16_SLO,

  (* This is a 16-bit reloc containing the lower 16 bits of an address. *)
    BFD_RELOC_M32R_LO16,

  (* This is a 16-bit reloc containing the small data area offset for use in
  add3, load, and store instructions. *)
    BFD_RELOC_M32R_SDA16,

  (* This is a 9-bit reloc *)
    BFD_RELOC_V850_9_PCREL,

  (* This is a 22-bit reloc *)
    BFD_RELOC_V850_22_PCREL,

  (* This is a 16 bit offset from the SmallInt data area pointer. *)
    BFD_RELOC_V850_SDA_16_16_OFFSET,

  (* This is a 16 bit offset (of which only 15 bits are used) from the
  SmallInt data area pointer. *)
    BFD_RELOC_V850_SDA_15_16_OFFSET,

  (* This is a 16 bit offset from the zero data area pointer. *)
    BFD_RELOC_V850_ZDA_16_16_OFFSET,

  (* This is a 16 bit offset (of which only 15 bits are used) from the
  zero data area pointer. *)
    BFD_RELOC_V850_ZDA_15_16_OFFSET,

  (* This is an 8 bit offset (of which only 6 bits are used) from the
  tiny data area pointer. *)
    BFD_RELOC_V850_TDA_6_8_OFFSET,

  (* This is an 8bit offset (of which only 7 bits are used) from the tiny
  data area pointer. *)
    BFD_RELOC_V850_TDA_7_8_OFFSET,

  (* This is a 7 bit offset from the tiny data area pointer. *)
    BFD_RELOC_V850_TDA_7_7_OFFSET,

  (* This is a 16 bit offset from the tiny data area pointer. *)
    BFD_RELOC_V850_TDA_16_16_OFFSET,

  (* This is a 5 bit offset (of which only 4 bits are used) from the tiny
  data area pointer. *)
    BFD_RELOC_V850_TDA_4_5_OFFSET,

  (* This is a 4 bit offset from the tiny data area pointer. *)
    BFD_RELOC_V850_TDA_4_4_OFFSET,

  (* This is a 16 bit offset from the SmallInt data area pointer, with the
  bits placed non-contigously in the instruction. *)
    BFD_RELOC_V850_SDA_16_16_SPLIT_OFFSET,

  (* This is a 16 bit offset from the zero data area pointer, with the
  bits placed non-contigously in the instruction. *)
    BFD_RELOC_V850_ZDA_16_16_SPLIT_OFFSET,

  (* This is a 6 bit offset from the call table base pointer. *)
    BFD_RELOC_V850_CALLT_6_7_OFFSET,

  (* This is a 16 bit offset from the call table base pointer. *)
    BFD_RELOC_V850_CALLT_16_16_OFFSET,


  (* This is a 32bit pcrel reloc for the mn10300, offset by two bytes in the
  instruction. *)
    BFD_RELOC_MN10300_32_PCREL,

  (* This is a 16bit pcrel reloc for the mn10300, offset by two bytes in the
  instruction. *)
    BFD_RELOC_MN10300_16_PCREL,

  (* This is a 8bit DP reloc for the tms320c30, where the most
  significant 8 bits of a 24 bit word are placed into the least
  significant 8 bits of the opcode. *)
    BFD_RELOC_TIC30_LDP,

  (* This is a 7bit reloc for the tms320c54x, where the least
  significant 7 bits of a 16 bit word are placed into the least
  significant 7 bits of the opcode. *)
    BFD_RELOC_TIC54X_PARTLS7,

  (* This is a 9bit DP reloc for the tms320c54x, where the most
  significant 9 bits of a 16 bit word are placed into the least
  significant 9 bits of the opcode. *)
    BFD_RELOC_TIC54X_PARTMS9,

  (* This is an extended address 23-bit reloc for the tms320c54x. *)
    BFD_RELOC_TIC54X_23,

  (* This is a 16-bit reloc for the tms320c54x, where the least
  significant 16 bits of a 23-bit extended address are placed into
  the opcode. *)
    BFD_RELOC_TIC54X_16_OF_23,

  (* This is a reloc for the tms320c54x, where the most
  significant 7 bits of a 23-bit extended address are placed into
  the opcode. *)
    BFD_RELOC_TIC54X_MS7_OF_23,

  (* This is a 48 bit reloc for the FR30 that stores 32 bits. *)
    BFD_RELOC_FR30_48,

  (* This is a 32 bit reloc for the FR30 that stores 20 bits split up into
  two sections. *)
    BFD_RELOC_FR30_20,

  (* This is a 16 bit reloc for the FR30 that stores a 6 bit word offset in
  4 bits. *)
    BFD_RELOC_FR30_6_IN_4,

  (* This is a 16 bit reloc for the FR30 that stores an 8 bit byte offset
  into 8 bits. *)
    BFD_RELOC_FR30_8_IN_8,

  (* This is a 16 bit reloc for the FR30 that stores a 9 bit SmallInt offset
  into 8 bits. *)
    BFD_RELOC_FR30_9_IN_8,

  (* This is a 16 bit reloc for the FR30 that stores a 10 bit word offset
  into 8 bits. *)
    BFD_RELOC_FR30_10_IN_8,

  (* This is a 16 bit reloc for the FR30 that stores a 9 bit pc relative
  SmallInt offset into 8 bits. *)
    BFD_RELOC_FR30_9_PCREL,

  (* This is a 16 bit reloc for the FR30 that stores a 12 bit pc relative
  SmallInt offset into 11 bits. *)
    BFD_RELOC_FR30_12_PCREL,

  (* Motorola Mcore relocations. *)
    BFD_RELOC_MCORE_PCREL_IMM8BY4,
    BFD_RELOC_MCORE_PCREL_IMM11BY2,
    BFD_RELOC_MCORE_PCREL_IMM4BY2,
    BFD_RELOC_MCORE_PCREL_32,
    BFD_RELOC_MCORE_PCREL_JSR_IMM11BY2,
    BFD_RELOC_MCORE_RVA,

  (* This is a 16 bit reloc for the AVR that stores 8 bit pc relative
  SmallInt offset into 7 bits. *)
    BFD_RELOC_AVR_7_PCREL,

  (* This is a 16 bit reloc for the AVR that stores 13 bit pc relative
  SmallInt offset into 12 bits. *)
    BFD_RELOC_AVR_13_PCREL,

  (* This is a 16 bit reloc for the AVR that stores 17 bit value (usually
  program memory address) into 16 bits. *)
    BFD_RELOC_AVR_16_PM,

  (* This is a 16 bit reloc for the AVR that stores 8 bit value (usually
  data memory address) into 8 bit immediate value of LDI insn. *)
    BFD_RELOC_AVR_LO8_LDI,

  (* This is a 16 bit reloc for the AVR that stores 8 bit value (high 8 bit
  of data memory address) into 8 bit immediate value of LDI insn. *)
    BFD_RELOC_AVR_HI8_LDI,

  (* This is a 16 bit reloc for the AVR that stores 8 bit value (most high 8 bit
  of program memory address) into 8 bit immediate value of LDI insn. *)
    BFD_RELOC_AVR_HH8_LDI,

  (* This is a 16 bit reloc for the AVR that stores negated 8 bit value
  (usually data memory address) into 8 bit immediate value of SUBI insn. *)
    BFD_RELOC_AVR_LO8_LDI_NEG,

  (* This is a 16 bit reloc for the AVR that stores negated 8 bit value
  (high 8 bit of data memory address) into 8 bit immediate value of
  SUBI insn. *)
    BFD_RELOC_AVR_HI8_LDI_NEG,

  (* This is a 16 bit reloc for the AVR that stores negated 8 bit value
  (most high 8 bit of program memory address) into 8 bit immediate value
  of LDI or SUBI insn. *)
    BFD_RELOC_AVR_HH8_LDI_NEG,

  (* This is a 16 bit reloc for the AVR that stores 8 bit value (usually
  command address) into 8 bit immediate value of LDI insn. *)
    BFD_RELOC_AVR_LO8_LDI_PM,

  (* This is a 16 bit reloc for the AVR that stores 8 bit value (high 8 bit
  of command address) into 8 bit immediate value of LDI insn. *)
    BFD_RELOC_AVR_HI8_LDI_PM,

  (* This is a 16 bit reloc for the AVR that stores 8 bit value (most high 8 bit
  of command address) into 8 bit immediate value of LDI insn. *)
    BFD_RELOC_AVR_HH8_LDI_PM,

  (* This is a 16 bit reloc for the AVR that stores negated 8 bit value
  (usually command address) into 8 bit immediate value of SUBI insn. *)
    BFD_RELOC_AVR_LO8_LDI_PM_NEG,

  (* This is a 16 bit reloc for the AVR that stores negated 8 bit value
  (high 8 bit of 16 bit command address) into 8 bit immediate value
  of SUBI insn. *)
    BFD_RELOC_AVR_HI8_LDI_PM_NEG,

  (* This is a 16 bit reloc for the AVR that stores negated 8 bit value
  (high 6 bit of 22 bit command address) into 8 bit immediate
  value of SUBI insn. *)
    BFD_RELOC_AVR_HH8_LDI_PM_NEG,

  (* This is a 32 bit reloc for the AVR that stores 23 bit value
  into 22 bits. *)
    BFD_RELOC_AVR_CALL,

  (* These two relocations are used by the linker to determine which of
  the entries in a C++ virtual function table are actually used.  When
  the --gc-sections option is given, the linker will zero out the entries
  that are not used, so that the code for those functions need not be
  included in the output.

  VTABLE_INHERIT is a zero-space relocation used to describe to the
  linker the inheritence tree of a C++ virtual function table.  The
  relocation's symbol should be the parent class' vtable, and the
  relocation should be located at the child vtable.

  VTABLE_ENTRY is a zero-space relocation that describes the use of a
  virtual function table entry.  The reloc's symbol should refer to the
  table of the class mentioned in the code.  Off of that base, an offset
  describes the entry that is being used.  For Rela hosts, this offset
  is stored in the reloc's addend.  For Rel hosts, we are forced to put
  this offset in the reloc's section offset. *)
    BFD_RELOC_VTABLE_INHERIT,
    BFD_RELOC_VTABLE_ENTRY,

  (* Intel IA64 Relocations. *)
    BFD_RELOC_IA64_IMM14,
    BFD_RELOC_IA64_IMM22,
    BFD_RELOC_IA64_IMM64,
    BFD_RELOC_IA64_DIR32MSB,
    BFD_RELOC_IA64_DIR32LSB,
    BFD_RELOC_IA64_DIR64MSB,
    BFD_RELOC_IA64_DIR64LSB,
    BFD_RELOC_IA64_GPREL22,
    BFD_RELOC_IA64_GPREL64I,
    BFD_RELOC_IA64_GPREL32MSB,
    BFD_RELOC_IA64_GPREL32LSB,
    BFD_RELOC_IA64_GPREL64MSB,
    BFD_RELOC_IA64_GPREL64LSB,
    BFD_RELOC_IA64_LTOFF22,
    BFD_RELOC_IA64_LTOFF64I,
    BFD_RELOC_IA64_PLTOFF22,
    BFD_RELOC_IA64_PLTOFF64I,
    BFD_RELOC_IA64_PLTOFF64MSB,
    BFD_RELOC_IA64_PLTOFF64LSB,
    BFD_RELOC_IA64_FPTR64I,
    BFD_RELOC_IA64_FPTR32MSB,
    BFD_RELOC_IA64_FPTR32LSB,
    BFD_RELOC_IA64_FPTR64MSB,
    BFD_RELOC_IA64_FPTR64LSB,
    BFD_RELOC_IA64_PCREL21B,
    BFD_RELOC_IA64_PCREL21BI,
    BFD_RELOC_IA64_PCREL21M,
    BFD_RELOC_IA64_PCREL21F,
    BFD_RELOC_IA64_PCREL22,
    BFD_RELOC_IA64_PCREL60B,
    BFD_RELOC_IA64_PCREL64I,
    BFD_RELOC_IA64_PCREL32MSB,
    BFD_RELOC_IA64_PCREL32LSB,
    BFD_RELOC_IA64_PCREL64MSB,
    BFD_RELOC_IA64_PCREL64LSB,
    BFD_RELOC_IA64_LTOFF_FPTR22,
    BFD_RELOC_IA64_LTOFF_FPTR64I,
    BFD_RELOC_IA64_LTOFF_FPTR64MSB,
    BFD_RELOC_IA64_LTOFF_FPTR64LSB,
    BFD_RELOC_IA64_SEGREL32MSB,
    BFD_RELOC_IA64_SEGREL32LSB,
    BFD_RELOC_IA64_SEGREL64MSB,
    BFD_RELOC_IA64_SEGREL64LSB,
    BFD_RELOC_IA64_SECREL32MSB,
    BFD_RELOC_IA64_SECREL32LSB,
    BFD_RELOC_IA64_SECREL64MSB,
    BFD_RELOC_IA64_SECREL64LSB,
    BFD_RELOC_IA64_REL32MSB,
    BFD_RELOC_IA64_REL32LSB,
    BFD_RELOC_IA64_REL64MSB,
    BFD_RELOC_IA64_REL64LSB,
    BFD_RELOC_IA64_LTV32MSB,
    BFD_RELOC_IA64_LTV32LSB,
    BFD_RELOC_IA64_LTV64MSB,
    BFD_RELOC_IA64_LTV64LSB,
    BFD_RELOC_IA64_IPLTMSB,
    BFD_RELOC_IA64_IPLTLSB,
    BFD_RELOC_IA64_COPY,
    BFD_RELOC_IA64_TPREL22,
    BFD_RELOC_IA64_TPREL64MSB,
    BFD_RELOC_IA64_TPREL64LSB,
    BFD_RELOC_IA64_LTOFF_TP22,
    BFD_RELOC_IA64_LTOFF22X,
    BFD_RELOC_IA64_LDXMOV,

  (* Motorola 68HC11 reloc.
  This is the 8 bits high part of an absolute address. *)
    BFD_RELOC_M68HC11_HI8,

  (* Motorola 68HC11 reloc.
  This is the 8 bits low part of an absolute address. *)
    BFD_RELOC_M68HC11_LO8,

  (* Motorola 68HC11 reloc.
  This is the 3 bits of a value. *)
    BFD_RELOC_M68HC11_3B,

  (* These relocs are only used within the CRIS assembler.  They are not
  (at present) written to any object files. *)
    BFD_RELOC_CRIS_BDISP8,
    BFD_RELOC_CRIS_UNSIGNED_5,
    BFD_RELOC_CRIS_SIGNED_6,
    BFD_RELOC_CRIS_UNSIGNED_6,
    BFD_RELOC_CRIS_UNSIGNED_4,

  (* Intel i860 Relocations. *)
    BFD_RELOC_860_COPY,
    BFD_RELOC_860_GLOB_DAT,
    BFD_RELOC_860_JUMP_SLOT,
    BFD_RELOC_860_RELATIVE,
    BFD_RELOC_860_PC26,
    BFD_RELOC_860_PLT26,
    BFD_RELOC_860_PC16,
    BFD_RELOC_860_LOW0,
    BFD_RELOC_860_SPLIT0,
    BFD_RELOC_860_LOW1,
    BFD_RELOC_860_SPLIT1,
    BFD_RELOC_860_LOW2,
    BFD_RELOC_860_SPLIT2,
    BFD_RELOC_860_LOW3,
    BFD_RELOC_860_LOGOT0,
    BFD_RELOC_860_SPGOT0,
    BFD_RELOC_860_LOGOT1,
    BFD_RELOC_860_SPGOT1,
    BFD_RELOC_860_LOGOTOFF0,
    BFD_RELOC_860_SPGOTOFF0,
    BFD_RELOC_860_LOGOTOFF1,
    BFD_RELOC_860_SPGOTOFF1,
    BFD_RELOC_860_LOGOTOFF2,
    BFD_RELOC_860_LOGOTOFF3,
    BFD_RELOC_860_LOPC,
    BFD_RELOC_860_HIGHADJ,
    BFD_RELOC_860_HAGOT,
    BFD_RELOC_860_HAGOTOFF,
    BFD_RELOC_860_HAPC,
    BFD_RELOC_860_HIGH,
    BFD_RELOC_860_HIGOT,
    BFD_RELOC_860_HIGOTOFF,
    BFD_RELOC_UNUSED
  ); { bfd_reloc_code_real }

  bfd_architecture = (
    bfd_arch_unknown,   (* File arch not known *)
    bfd_arch_obscure,   (* Arch known, not one of these *)
    bfd_arch_m68k,      (* Motorola 68xxx *)
    bfd_arch_vax,       (* DEC Vax *)
    bfd_arch_i960,      (* Intel 960 *)
      (* The order of the following is important.
         lower number indicates a machine type that
         only accepts a subset of the instructions
         available to machines with higher numbers.
         The exception is the "ca", which is
         incompatible with all other machines except
         "core". *)

    bfd_arch_a29k,      (* AMD 29000 *)
    bfd_arch_sparc,     (* SPARC *)
  (* The difference between v8plus and v9 is that v9 is a true 64 bit env.  *)
  (* Nonzero if MACH has the v9 instruction set.  *)
    bfd_arch_mips,      (* MIPS Rxxxx *)
    bfd_arch_i386,      (* Intel 386 *)
    bfd_arch_we32k,     (* AT&T WE32xxx *)
    bfd_arch_tahoe,     (* CCI/Harris Tahoe *)
    bfd_arch_i860,      (* Intel 860 *)
    bfd_arch_i370,      (* IBM 360/370 Mainframes *)
    bfd_arch_romp,      (* IBM ROMP PC/RT *)
    bfd_arch_alliant,   (* Alliant *)
    bfd_arch_convex,    (* Convex *)
    bfd_arch_m88k,      (* Motorola 88xxx *)
    bfd_arch_pyramid,   (* Pyramid Technology *)
    bfd_arch_h8300,     (* Hitachi H8/300 *)
    bfd_arch_powerpc,   (* PowerPC *)
    bfd_arch_rs6000,    (* IBM RS/6000 *)
    bfd_arch_hppa,      (* HP PA RISC *)
    bfd_arch_d10v,      (* Mitsubishi D10V *)
    bfd_arch_d30v,      (* Mitsubishi D30V *)
    bfd_arch_m68hc11,   (* Motorola 68HC11 *)
    bfd_arch_m68hc12,   (* Motorola 68HC12 *)
    bfd_arch_z8k,       (* Zilog Z8000 *)
    bfd_arch_h8500,     (* Hitachi H8/500 *)
    bfd_arch_sh,        (* Hitachi SH *)
    bfd_arch_alpha,     (* Dec Alpha *)
    bfd_arch_arm,       (* Advanced Risc Machines ARM *)
    bfd_arch_ns32k,     (* National Semiconductors ns32000 *)
    bfd_arch_w65,       (* WDC 65816 *)
    bfd_arch_tic30,     (* Texas Instruments TMS320C30 *)
    bfd_arch_tic54x,    (* Texas Instruments TMS320C54X *)
    bfd_arch_tic80,     (* TI TMS320c80 (MVP) *)
    bfd_arch_v850,      (* NEC V850 *)
    bfd_arch_arc,       (* ARC Cores *)
    bfd_arch_m32r,      (* Mitsubishi M32R/D *)
    bfd_arch_mn10200,   (* Matsushita MN10200 *)
    bfd_arch_mn10300,   (* Matsushita MN10300 *)
    bfd_arch_fr30,
    bfd_arch_mcore,
    bfd_arch_ia64,      (* HP/Intel ia64 *)
    bfd_arch_pj,
    bfd_arch_avr,       (* Atmel AVR microcontrollers *)
    bfd_arch_cris,      (* Axis CRIS *)
    bfd_arch_last
  ); { bfd_architecture }

  check_format_proc    = function (a:PBFD):pbfd_target;cdecl;
  set_format_proc      = function (a:PBFD):bfd_boolean;cdecl;
  write_contents_proc = function (a:PBFD):bfd_boolean;cdecl;

  bfd_target = record
    name: pchar;
    flavour: bfd_flavour;
    byteorder: bfd_endian;
    header_byteorder: bfd_endian;
    object_flags: flagword;
    section_flags: flagword;
    symbol_leading_char: ShortInt;
    ar_pad_char: ShortInt;
    ar_max_namelen: Word;
    bfd_getx64:           function (const Pbfd_byte): bfd_vma; cdecl;
    bfd_getx_signed_64:   function (const Pbfd_byte):bfd_signed_vma; cdecl;
    bfd_putx64:           procedure(a: bfd_vma; b: Pbfd_byte); cdecl;
    bfd_getx32:           function (const Pbfd_byte):bfd_vma; cdecl;
    bfd_getx_signed_32:   function (const Pbfd_byte):bfd_signed_vma; cdecl;
    bfd_putx32:           procedure(a: bfd_vma; b: Pbfd_byte); cdecl;
    bfd_getx16:           function (const Pbfd_byte):bfd_vma; cdecl;
    bfd_getx_signed_16:   function (const Pbfd_byte):bfd_signed_vma; cdecl;
    bfd_putx16:           procedure(a: bfd_vma; b: Pbfd_byte); cdecl;
    bfd_h_getx64:         function (const Pbfd_byte):bfd_vma; cdecl;
    bfd_h_getx_signed_64: function (const Pbfd_byte):bfd_signed_vma; cdecl;
    bfd_h_putx64:         procedure(a: bfd_vma; b: Pbfd_byte); cdecl;
    bfd_h_getx32:         function (const Pbfd_byte):bfd_vma; cdecl;
    bfd_h_getx_signed_32: function (const Pbfd_byte):bfd_signed_vma; cdecl;
    bfd_h_putx32:         procedure(a: bfd_vma; b: Pbfd_byte); cdecl;
    bfd_h_getx16:         function (const Pbfd_byte):bfd_vma; cdecl;
    bfd_h_getx_signed_16: function (const Pbfd_byte):bfd_signed_vma; cdecl;
    bfd_h_putx16:         procedure(a: bfd_vma; b: Pbfd_byte); cdecl;
    _bfd_check_format:   array [bfd_format] of check_format_proc;
    _bfd_set_format:     array [bfd_format] of set_format_proc;
    _bfd_write_contents: array [bfd_format] of write_contents_proc;

    (* Generic entry points.  *)

    (* Called when the BFD is being closed to do any necessary cleanup.  *)
    _close_and_cleanup:     function(a: PBFD  ):bfd_boolean;cdecl;
    (* Ask the BFD to free all cached information.  *)
    _bfd_free_cached_info:  function(a: PBFD  ):bfd_boolean;cdecl;
    (* Called when a new section is created.  *)
    _new_section_hook:      function(a: PBFD; b: PSec):bfd_boolean;cdecl;
    (* Read the contents of a section.  *)
    _bfd_get_section_contents:  function(a: PBFD; b: PSec; c: pointer; d: file_ptr; e: bfd_size_type):bfd_boolean;cdecl;
    _bfd_get_section_contents_in_window: function(a: PBFD; b: PSec; c: Pbfd_window; d:file_ptr; e: bfd_size_type):bfd_boolean;cdecl;

    (* Entry points to copy private data.  *)

    (* Called to copy BFD general private data from one object file
       to another.  *)
    _bfd_copy_private_bfd_data:  function(a: PBFD; b: PBFD  ):bfd_boolean;cdecl;
    (* Called to merge BFD general private data from one object file
       to a common output file when linking.  *)
    _bfd_merge_private_bfd_data:  function(a: PBFD; b: PBFD  ):bfd_boolean;cdecl;
    (* Called to copy BFD private section data from one object file
       to another.  *)
    _bfd_copy_private_section_data: function(a: PBFD; b:PSec; c: PBFD; d: PSec):bfd_boolean;cdecl;
    (* Called to copy BFD private symbol data from one symbol
       to another.  *)
    _bfd_copy_private_symbol_data: function(a: PBFD; b: Pasymbol; c: PBFD; d: Pasymbol):bfd_boolean;cdecl;
    (* Called to set private backend flags *)
    _bfd_set_private_flags: function(a: PBFD; b:flagword):bfd_boolean; cdecl;

    (* Called to print private BFD data *)
    _bfd_print_private_bfd_data: function(a: PBFD; b: pointer):bfd_boolean; cdecl;

    (* Core file entry points.  *)

    (* Archive entry points.  *)

    _bfd_slurp_armap: function(a: PBFD):bfd_boolean;cdecl;
    _bfd_slurp_extended_name_table: function(a: PBFD  ):bfd_boolean;cdecl;
    _bfd_construct_extended_name_table: function(a: PBFD; b: Ppchar; c: Pbfd_size_type; d: Ppchar):bfd_boolean;cdecl;
    _bfd_truncate_arname: procedure (a: PBFD; b: pchar; c: pchar);cdecl;
    write_armap: function(arch: PBFD; elength: LongWord; map: Porl; orl_count: LongWord; stridx: integer):bfd_boolean;cdecl;
    _bfd_read_ar_hdr_fn: function(a: PBFD  ):pointer;cdecl;
    openr_next_archived_file: function(arch: PBFD; prev: PBFD):PBFD;cdecl;


    _bfd_get_elt_at_index: function(a: PBFD; b: symindex):PBFD;cdecl;
    _bfd_stat_arch_elt: function(a: PBFD; b: Pstat ):integer;cdecl;
    _bfd_update_armap_timestamp: function(a: PBFD  ):bfd_boolean;cdecl;

    (* Entry points used for symbols.  *)

    _bfd_get_symtab_upper_bound: function(a: PBFD  ):LongInt;cdecl;
    _bfd_canonicalize_symtab: function(a: PBFD; b: PPsymbol_cache_entry):LongInt;cdecl;


    _bfd_make_empty_symbol: function(a: PBFD  ): PPsymbol_cache_entry;cdecl;

    _bfd_print_symbol: procedure(a: PBFD; b: pointer; c: Psymbol_cache_entry; d: bfd_print_symbol);cdecl;
    _bfd_get_symbol_info: procedure (a: PBFD; b: Psymbol_cache_entry; c:Psymbol_info);cdecl;
    _bfd_is_local_label_name: function(a: PBFD; b: pchar):bfd_boolean;cdecl;

    _get_lineno: function( a: PBFD; b: Psymbol_cache_entry):Palent;cdecl;
    _bfd_find_nearest_line: function(abfd: PBFD; section: Psec; symbols: PPsymbol_cache_entry; offset: bfd_vma; fil: pchar; func: Ppchar;  lin: PLongWord):bfd_boolean;cdecl;
   (* Back-door to allow format-aware applications to create debug symbols
      while using BFD for everything else.  Currently used by the assembler
      when creating COFF files.  *)
    _bfd_make_debug_symbol: function( abfd: PBFD ; ptr: pointer; size: LongWord ):Pasymbol;cdecl;
    _read_minisymbols:   function(a: PBFD; b: bfd_boolean; c: PPointer; d: PLongWord):LongInt;cdecl;
    _minisymbol_to_symbol: function(a: PBFD; b:bfd_boolean; c: pointer; d: Pasymbol):Pasymbol;cdecl;

    (* Routines for relocs.  *)

    _get_reloc_upper_bound: function(a: PBFD; b: PSec):LongInt;cdecl;
    _bfd_canonicalize_reloc: function(a: PBFD; b: PSec;c: PParelent; d: PPsymbol_cache_entry ):LongInt;cdecl;
    (* See documentation on reloc types.  *)

    reloc_type_lookup: function(abfd: PBFD ; code: bfd_reloc_code_real): PPreloc_howto;cdecl;

    (* Routines used when writing an object file.  *)

    _bfd_set_arch_mach: function( a: PBFD; b:bfd_architecture; c:LongWord): bfd_boolean;cdecl;

    _bfd_set_section_contents: function(a: PBFD; b: PSec; c: pointer; d: file_ptr; e: bfd_size_type):bfd_boolean;cdecl;

    (* Routines used by the linker.  *)

    _bfd_sizeof_headers: function(a: PBFD ; b: bfd_boolean):integer;cdecl;
    _bfd_get_relocated_section_contents: function(a: PBFD ; b: Pbfd_link_info; c: Pbfd_link_order; data: Pbfd_byte; relocateable: bfd_boolean; ce: PPsymbol_cache_entry):Pbfd_byte;cdecl;

    _bfd_relax_section: function(a: PBFD ; b: Psec; c: Pbfd_link_info; again: Pboolean):bfd_boolean;cdecl;

    (* Create a hash table for the linker.  Different backends store
       different information in this table.  *)
    _bfd_link_hash_table_create: function( a: PBFD  ):Pbfd_link_hash_table;cdecl;

    (* Add symbols from this object file into the hash table.  *)
    _bfd_link_add_symbols: function(a: PBFD ; b: Pbfd_link_info):bfd_boolean;cdecl;

    (* Do a link based on the link_order structures attached to each
       section of the BFD.  *)
    _bfd_final_link: function(a: PBFD; b: Pbfd_link_info):bfd_boolean;cdecl;

    (* Should this section be split up into smaller pieces during linking.  *)
    _bfd_link_split_section: function(a: PBFD; b: Psec):bfd_boolean;cdecl;

    (* Remove sections that are not referenced from the output.  *)
    _bfd_gc_sections: function(a: PBFD ; b: Pbfd_link_info):bfd_boolean;cdecl;

    (* Routines to handle dynamic symbols and relocs.  *)

    (* Get the amount of memory required to hold the dynamic symbols. *)
    _bfd_get_dynamic_symtab_upper_bound: function(a: PBFD  ):LongInt;cdecl;
    (* Read in the dynamic symbols.  *)
    _bfd_canonicalize_dynamic_symtab: function(a: PBFD ; b: PPsymbol_cache_entry):LongInt;cdecl;
    (* Get the amount of memory required to hold the dynamic relocs.  *)
    _bfd_get_dynamic_reloc_upper_bound: function(a: PBFD  ):LongInt;cdecl;
    (* Read in the dynamic relocs.  *)
    _bfd_canonicalize_dynamic_reloc: function(a: PBFD ; b: PParelent; c: PPsymbol_cache_entry):LongInt;cdecl;

    (* Opposite endian version of this target.  *)
    alternative_target: Pbfd_target;

    backend_data: pointer;
  end; { bfd_target }

  complain_overflow = (
       (* Do not complain on overflow. *)
    complain_overflow_dont,

       (* Complain if the bitfield overflows, whether it is considered
          as signed or unsigned. *)
    complain_overflow_bitfield,

       (* Complain if the value overflows when considered as signed
          number. *)
    complain_overflow_signed,

       (* Complain if the value overflows when considered as an
          unsigned number. *)
    complain_overflow_unsigned
  ); { complain_overflow }

  bfd_reloc_status = (
         (* No errors detected *)
    bfd_reloc_ok,

         (* The relocation was performed, but there was an overflow. *)
    bfd_reloc_overflow,

         (* The address to relocate was not within the section supplied. *)
    bfd_reloc_outofrange,

         (* Used by special functions *)
    bfd_reloc_continue,

         (* Unsupported relocation size requested. *)
    bfd_reloc_notsupported,

         (* Unused *)
    bfd_reloc_other,

         (* The symbol to relocate against was undefined. *)
    bfd_reloc_undefined,

         (* The relocation was performed, but may not be ok - presently
            generated only when linking i960 coff files with i960 b.out
            symbols.  If this type is returned, the error_message argument
            to bfd_perform_relocation will be set.  *)
    bfd_reloc_dangerous
  ); { bfd_reloc_status }


  reloc_howto = record
         (*  The type field has mainly a documentary use - the back end can
             do what it wants with it, though normally the back end's
             external idea of what a reloc number is stored
             in this field. For example, a PC relative word relocation
             in a coff environment has the type 023 - because that's
             what the outside world calls a R_PCRWORD reloc. *)
    typ: LongWord;

         (*  The value the final relocation is shifted right by. This drops
             unwanted data from the relocation.  *)
    rightshift: LongWord;

         (*  The size of the item to be relocated.  This is *not* a
             power-of-two measure.  To get the number of bytes operated
             on by a type of relocation, use bfd_get_reloc_size.  *)
    size: integer;

         (*  The number of bits in the item to be relocated.  This is used
             when doing overflow checking.  *)
    bitsize: LongWord;

         (*  Notes that the relocation is relative to the location in the
             data section of the addend. The relocation function will
             subtract from the relocation value the address of the location
             being relocated. *)
    pc_relative: bfd_boolean;

         (*  The bit position of the reloc value in the destination.
             The relocated value is left shifted by this amount. *)
    bitpos: LongWord;

         (* What type of overflow error should be checked for when
            relocating. *)
    complain_on_overflow: complain_overflow;

         (* If this field is non null, then the supplied function is
            called rather than the normal function. This allows really
            strange relocation methods to be accomodated (e.g., i960 callj
            instructions). *)
    special_function: function( abfd: PBFD;
                                reloc_entry: Parelent;
                                symbol: Psymbol_cache_entry;
                                data: pointer;
                                input_section: Pasection;
                                output_bfd: PBFD;
                                error_message: ppchar):  bfd_reloc_status;cdecl;

         (* The textual name of the relocation type. *)
    name: pchar;

         (* Some formats record a relocation addend in the section contents
            rather than with the relocation.  For ELF formats this is the
            distinction between USE_REL and USE_RELA (though the code checks
            for USE_REL == 1/0).  The value of this field is TRUE if the
            addend is recorded with the section contents; when performing a
            partial link (ld -r) the section contents (the data) will be
            modified.  The value of this field is FALSE if addends are
            recorded with the relocation (in arelent.addend); when performing
            a partial link the relocation will be modified.
            All relocations for all ELF USE_RELA targets should set this field
            to FALSE (values of TRUE should be looked on with suspicion).
            However, the converse is not true: not all relocations of all ELF
            USE_REL targets set this field to TRUE.  Why this is so is peculiar
            to each particular target.  For relocs that aren't used in partial
            links (e.g. GOT stuff) it doesn't matter what this is set to.  *)
    partial_inplace: bfd_boolean;

         (* The src_mask selects which parts of the read in data
            are to be used in the relocation sum.  E.g., if this was an 8 bit
            byte of data which we read and relocated, this would be
            $000000ff. When we have relocs which have an addend, such as
            sun4 extended relocs, the value in the offset part of a
            relocating field is garbage so we never use it. In this case
            the mask would be $00000000. *)
    src_mask: bfd_vma;

         (* The dst_mask selects which parts of the instruction are replaced
            into the instruction. In most cases src_mask == dst_mask,
            except in the above special case, where dst_mask would be
            $000000ff, and src_mask would be $00000000.   *)
    dst_mask: bfd_vma;

         (* When some formats create PC relative instructions, they leave
            the value of the pc of the place being relocated in the offset
            slot of the instruction, so that a PC relative relocation can
            be made just by adding in an ordinary offset (e.g., sun3 a.out).
            Some formats leave the displacement part of an instruction
            empty (e.g., m88k bcs); this flag signals the fact.*)
    pcrel_offset: bfd_boolean;

  end; { reloc_howto }

  bfd_arch_info = record
    bits_per_word: integer;
    bits_per_address: integer;
    bits_per_byte: integer;
    arch: bfd_architecture;
    mach: LongWord;
    arch_name: pchar;
    printable_name: pchar;
    section_align_power: LongWord;
    (* True if this is the default machine for the architecture.  *)
    the_default: bfd_boolean;

    compatible: function(a: Pbfd_arch_info; b: Pbfd_arch_info):Pbfd_arch_info;cdecl;
    scan: function(a: Pbfd_arch_info; b: pchar): bfd_boolean;cdecl;

    next: Pbfd_arch_info;
  end; { bfd_arch_info }

  reloc_cache_entry = record
         (* A pointer into the canonical table of pointers  *)
    sym_ptr_ptr: Psymbol_cache_entry;

         (* offset in section *)
    address: bfd_size_type;

         (* addend for relocation value *)
    addend: bfd_vma;

         (* Pointer to how to perform the required relocation *)
    howto: reloc_howto;
  end; { reloc_cache_entry }
  arelent = reloc_cache_entry;


(* This structure is used for a comdat section, as in PE.  A comdat
   section is associated with a particular symbol.  When the linker
   sees a comdat section, it keeps only one of the sections with a
   given name and associated with a given symbol.  *)

   bfd_comdat_info = record
    (* The name of the symbol associated with a comdat section.  *)
      name: pchar;

    (* The local symbol table index of the symbol associated with a
       comdat section.  This is only meaningful to the object file format
       specific code; it is not an index into the list returned by
       bfd_canonicalize_symtab.  *)
    symbol: LongInt;
  end; { bfd_comdat_info }

  relent_chain = record
    relent: arelent;
    next: Prelent_chain;
  end;
  arelent_chain = relent_chain;

  bfd_error = (
    bfd_error_no_error = 0 ,
    bfd_error_system_call,
    bfd_error_invalid_target,
    bfd_error_wrong_format,
    bfd_error_invalid_operation,
    bfd_error_no_memory,
    bfd_error_no_symbols,
    bfd_error_no_armap,
    bfd_error_no_more_archived_files,
    bfd_error_malformed_archive,
    bfd_error_file_not_recognized,
    bfd_error_file_ambiguously_recognized,
    bfd_error_no_contents,
    bfd_error_nonrepresentable_section,
    bfd_error_no_debug_section,
    bfd_error_bad_value,
    bfd_error_file_truncated,
    bfd_error_file_too_big,
    bfd_error_invalid_error_code
  ); { bfd_error }

const
  BFD_NO_MORE_SYMBOLS: symindex = symindex(not symindex(0));

  bfd_mach_m68000 = 1;
  bfd_mach_m68008 = 2;
  bfd_mach_m68010 = 3;
  bfd_mach_m68020 = 4;
  bfd_mach_m68030 = 5;
  bfd_mach_m68040 = 6;
  bfd_mach_m68060 = 7;
  bfd_mach_cpu32  = 8;
  bfd_mach_i960_core      = 1;
  bfd_mach_i960_ka_sa     = 2;
  bfd_mach_i960_kb_sb     = 3;
  bfd_mach_i960_mc        = 4;
  bfd_mach_i960_xa        = 5;
  bfd_mach_i960_ca        = 6;
  bfd_mach_i960_jx        = 7;
  bfd_mach_i960_hx        = 8;
  bfd_mach_sparc                 = 1;
  bfd_mach_sparc_sparclet        = 2;
  bfd_mach_sparc_sparclite       = 3;
  bfd_mach_sparc_v8plus          = 4;
  bfd_mach_sparc_v8plusa         = 5 (* with ultrasparc add'ns *);
  bfd_mach_sparc_sparclite_le    = 6;
  bfd_mach_sparc_v9              = 7;
  bfd_mach_sparc_v9a             = 8 (* with ultrasparc add'ns *);
  bfd_mach_sparc_v8plusb         = 9 (* with cheetah add'ns *);
  bfd_mach_sparc_v9b             = 10 (* with cheetah add'ns *);
  bfd_mach_mips3000              = 3000;
  bfd_mach_mips3900              = 3900;
  bfd_mach_mips4000              = 4000;
  bfd_mach_mips4010              = 4010;
  bfd_mach_mips4100              = 4100;
  bfd_mach_mips4111              = 4111;
  bfd_mach_mips4300              = 4300;
  bfd_mach_mips4400              = 4400;
  bfd_mach_mips4600              = 4600;
  bfd_mach_mips4650              = 4650;
  bfd_mach_mips5000              = 5000;
  bfd_mach_mips6000              = 6000;
  bfd_mach_mips8000              = 8000;
  bfd_mach_mips10000             = 10000;
  bfd_mach_mips16                = 16;
  bfd_mach_mips32                = 32;
  bfd_mach_mips32_4k             = 3204113 (* 32, 04, octal 'K' *);
  bfd_mach_mips5                 = 5;
  bfd_mach_mips64                = 64;
  bfd_mach_mips_sb1              = 12310201 (* octal 'SB', 01 *);
  bfd_mach_i386_i386 = 0;
  bfd_mach_i386_i8086 = 1;
  bfd_mach_i386_i386_intel_syntax = 2;
  bfd_mach_x86_64 = 3;
  bfd_mach_x86_64_intel_syntax = 4;
  bfd_mach_h8300   = 1;
  bfd_mach_h8300h  = 2;
  bfd_mach_h8300s  = 3;
  bfd_mach_ppc           = 0;
  bfd_mach_ppc_403       = 403;
  bfd_mach_ppc_403gc     = 4030;
  bfd_mach_ppc_505       = 505;
  bfd_mach_ppc_601       = 601;
  bfd_mach_ppc_602       = 602;
  bfd_mach_ppc_603       = 603;
  bfd_mach_ppc_ec603e    = 6031;
  bfd_mach_ppc_604       = 604;
  bfd_mach_ppc_620       = 620;
  bfd_mach_ppc_630       = 630;
  bfd_mach_ppc_750       = 750;
  bfd_mach_ppc_860       = 860;
  bfd_mach_ppc_a35       = 35;
  bfd_mach_ppc_rs64ii    = 642;
  bfd_mach_ppc_rs64iii   = 643;
  bfd_mach_ppc_7400      = 7400;
  bfd_mach_rs6k          = 0;
  bfd_mach_rs6k_rs1      = 6001;
  bfd_mach_rs6k_rsc      = 6003;
  bfd_mach_rs6k_rs2      = 6002;
  bfd_mach_d10v          = 0;
  bfd_mach_d10v_ts2      = 2;
  bfd_mach_d10v_ts3      = 3;
  bfd_mach_z8001         = 1;
  bfd_mach_z8002         = 2;
  bfd_mach_sh            = 0;
  bfd_mach_sh2        = $20;
  bfd_mach_sh_dsp     = $2d;
  bfd_mach_sh3        = $30;
  bfd_mach_sh3_dsp    = $3d;
  bfd_mach_sh3e       = $3e;
  bfd_mach_sh4        = $40;
  bfd_mach_alpha_ev4  = $10;
  bfd_mach_alpha_ev5  = $20;
  bfd_mach_alpha_ev6  = $30;
  bfd_mach_arm_2         = 1;
  bfd_mach_arm_2a        = 2;
  bfd_mach_arm_3         = 3;
  bfd_mach_arm_3M        = 4;
  bfd_mach_arm_4         = 5;
  bfd_mach_arm_4T        = 6;
  bfd_mach_arm_5         = 7;
  bfd_mach_arm_5T        = 8;
  bfd_mach_arm_5TE       = 9;
  bfd_mach_arm_XScale    = 10;
  bfd_mach_v850          = 0;
  bfd_mach_v850e         = ord('E');
  bfd_mach_v850ea        = ord('A');
  bfd_mach_arc_5         = 0;
  bfd_mach_arc_6         = 1;
  bfd_mach_arc_7         = 2;
  bfd_mach_arc_8         = 3;
  bfd_mach_m32r          = 0 (* backwards compatibility *);
  bfd_mach_m32rx         = ord('x');
  bfd_mach_mn10300       = 300;
  bfd_mach_am33          = 330;
  bfd_mach_fr30          = $46523330;
  bfd_mach_ia64_elf64    = 0;
  bfd_mach_ia64_elf32    = 1;
  bfd_mach_avr1          = 1;
  bfd_mach_avr2          = 2;
  bfd_mach_avr3          = 3;
  bfd_mach_avr4          = 4;
  bfd_mach_avr5          = 5;


  BFD_RELOC_SPARC_64 = BFD_RELOC_64;
  BFD_RELOC_SPARC_DISP64 = BFD_RELOC_64_PCREL;
  BFD_RELOC_MIPS_GPREL = BFD_RELOC_GPREL16;
  BFD_RELOC_MIPS_GPREL32 = BFD_RELOC_GPREL32;


  SEC_NO_FLAGS   = $000;

  (* Tells the OS to allocate space for this section when loading.
     This is clear for a section containing debug information only.  *)
  SEC_ALLOC      = $001;

  (* Tells the OS to load the section from the file when loading.
     This is clear for a .bss section.  *)
  SEC_LOAD       = $002;

  (* The section contains data still to be relocated, so there is
     some relocation information too.  *)
  SEC_RELOC      = $004;

{  (* Obsolete ? *)
  SEC_BALIGN     = $008;
}

  (* A signal to the OS that the section contains read only data.  *)
  SEC_READONLY   = $010;

  (* The section contains code only.  *)
  SEC_CODE       = $020;

  (* The section contains data only.  *)
  SEC_DATA       = $040;

  (* The section will reside in ROM.  *)
  SEC_ROM        = $080;

  (* The section contains constructor information. This section
     type is used by the linker to create lists of constructors and
     destructors used by <<g++>>. When a back end sees a symbol
     which should be used in a constructor list, it creates a new
     section for the type of name (e.g., <<__CTOR_LIST__>>), attaches
     the symbol to it, and builds a relocation. To build the lists
     of constructors, all the linker has to do is catenate all the
     sections called <<__CTOR_LIST__>> and relocate the data
     contained within - exactly the operations it would peform on
     standard data.  *)
  SEC_CONSTRUCTOR = $100;

  (* The section is a constructor, and should be placed at the
     end of the text, data, or bss section(?).  *)
  SEC_CONSTRUCTOR_TEXT = $1100;
  SEC_CONSTRUCTOR_DATA = $2100;
  SEC_CONSTRUCTOR_BSS  = $3100;

  (* The section has contents - a data section could be
     <<SEC_ALLOC>> | <<SEC_HAS_CONTENTS>>; a debug section could be
     <<SEC_HAS_CONTENTS>>  *)
  SEC_HAS_CONTENTS = $200;

  (* An instruction to the linker to not output the section
     even if it has information which would normally be written.  *)
  SEC_NEVER_LOAD = $400;

  (* The section is a COFF shared library section.  This flag is
     only for the linker.  If this type of section appears in
     the input file, the linker must copy it to the output file
     without changing the vma or size.  FIXME: Although this
     was originally intended to be general, it really is COFF
     specific (and the flag was renamed to indicate this).  It
     might be cleaner to have some more general mechanism to
     allow the back end to control what the linker does with
     sections.  *)
  SEC_COFF_SHARED_LIBRARY = $800;

  (* The section has GOT references.  This flag is only for the
     linker, and is currently only used by the elf32-1 back end.
     It will be set if global offset table references were detected
     in this section, which indicate to the linker that the section
     contains PIC code, and must be handled specially when doing a
     static link.  *)
  SEC_HAS_GOT_REF = $4000;

  (* The section contains common symbols (symbols may be defined
     multiple times, the value of a symbol is the amount of
     space it requires, and the largest symbol value is the one
     used).  Most targets have exactly one of these (which we
     translate to bfd_com_section_ptr), but ECOFF has two.  *)
  SEC_IS_COMMON = $8000;

  (* The section contains only debugging information.  For
     example, this is set for ELF .debug and .stab sections.
     strip tests this flag to see if a section can be
     discarded.  *)
  SEC_DEBUGGING = $10000;

  (* The contents of this section are held in memory pointed to
     by the contents field.  This is checked by bfd_get_section_contents,
     and the data is retrieved from memory if appropriate.  *)
  SEC_IN_MEMORY = $20000;

  (* The contents of this section are to be excluded by the
     linker for executable and shared objects unless those
     objects are to be further relocated.  *)
  SEC_EXCLUDE = $40000;

  (* The contents of this section are to be sorted by the
     based on the address specified in the associated symbol
     table.  *)
  SEC_SORT_ENTRIES = $80000;

  (* When linking, duplicate sections of the same name should be
     discarded, rather than being combined into a single section as
     is usually done.  This is similar to how common symbols are
     handled.  See SEC_LINK_DUPLICATES below.  *)
  SEC_LINK_ONCE = $100000;

  (* If SEC_LINK_ONCE is set, this bitfield describes how the linker
     should handle duplicate sections.  *)
  SEC_LINK_DUPLICATES = $600000;

  (* This value for SEC_LINK_DUPLICATES means that duplicate
     sections with the same name should simply be discarded.  *)
  SEC_LINK_DUPLICATES_DISCARD = $0;

  (* This value for SEC_LINK_DUPLICATES means that the linker
     should warn if there are any duplicate sections, although
     it should still only link one copy.  *)
  SEC_LINK_DUPLICATES_ONE_ONLY = $200000;

  (* This value for SEC_LINK_DUPLICATES means that the linker
     should warn if any duplicate sections are a different size.  *)
  SEC_LINK_DUPLICATES_SAME_SIZE = $400000;

  (* This value for SEC_LINK_DUPLICATES means that the linker
     should warn if any duplicate sections contain different
     contents.  *)
  SEC_LINK_DUPLICATES_SAME_CONTENTS = $600000;

  (* This section was created by the linker as part of dynamic
     relocation or other arcane processing.  It is skipped when
     going through the first-pass output, trusting that someone
     else up the line will take care of it later.  *)
  SEC_LINKER_CREATED = $800000;

  (* This section should not be subject to garbage collection.  *)
  SEC_KEEP = $1000000;

  (* This section contains "SmallInt" data, and should be placed
     "near" the GP.  *)
  SEC_SMALL_DATA = $2000000;

  (* This section contains data which may be shared with other
     executables or shared objects.  *)
  SEC_SHARED = $4000000;

  (* When a section with this flag is being linked, then if the size of
     the input section is less than a page, it should not cross a page
     boundary.  If the size of the input section is one page or more, it
     should be aligned on a page boundary.  *)
  SEC_BLOCK = $8000000;

  (* Conditionally link this section; do not link if there are no
     references found to any symbol in the section.  *)
  SEC_CLINK = $10000000;

  (*  End of section flags.  *)


 (* Attributes of a symbol: *)

  BSF_NO_FLAGS    = $00;

       (* The symbol has local scope; <<static>> in <<C>>. The value
          is the offset into the section of the data. *)
  BSF_LOCAL      = $01;

       (* The symbol has global scope; initialized data in <<C>>. The
          value is the offset into the section of the data. *)
  BSF_GLOBAL     = $02;

       (* The symbol has global scope and is exported. The value is
          the offset into the section of the data. *)
  BSF_EXPORT     = BSF_GLOBAL (* no real difference *);

       (* A normal C symbol would be one of:
          <<BSF_LOCAL>>, <<BSF_FORT_COMM>>,  <<BSF_UNDEFINED>> or
          <<BSF_GLOBAL>> *)

       (* The symbol is a debugging record. The value has an arbitary
          meaning, unless BSF_DEBUGGING_RELOC is also set.  *)
  BSF_DEBUGGING  = $08;

       (* The symbol denotes a function entry point.  Used in ELF,
          perhaps others someday.  *)
  BSF_FUNCTION    = $10;

       (* Used by the linker. *)
  BSF_KEEP        = $20;
  BSF_KEEP_G      = $40;

       (* A weak global symbol, overridable without warnings by
          a regular global symbol of the same name.  *)
  BSF_WEAK        = $80;

       (* This symbol was created to point to a section, e.g. ELF's
          STT_SECTION symbols.  *)
  BSF_SECTION_SYM = $100;

       (* The symbol used to be a common symbol, but now it is
          allocated. *)
  BSF_OLD_COMMON  = $200;

       (* The default value for common data. *)
  BFD_FORT_COMM_DEFAULT_VALUE = 0;

       (* In some files the type of a symbol sometimes alters its
          location in an output file - ie in coff a <<ISFCN>> symbol
          which is also <<C_EXT>> symbol appears where it was
          declared and not at the end of a section.  This bit is set
          by the target BFD part to convey this information. *)

  BSF_NOT_AT_END    = $400;

       (* Signal that the symbol is the label of constructor section. *)
  BSF_CONSTRUCTOR   = $800;

       (* Signal that the symbol is a warning symbol.  The name is a
          warning.  The name of the next symbol is the one to warn about;
          if a reference is made to a symbol with the same name as the next
          symbol, a warning is issued by the linker. *)
  BSF_WARNING       = $1000;

       (* Signal that the symbol is indirect.  This symbol is an indirect
          pointer to the symbol with the same name as the next symbol. *)
  BSF_INDIRECT      = $2000;

       (* BSF_FILE marks symbols that contain a file name.  This is used
          for ELF STT_FILE symbols.  *)
  BSF_FILE          = $4000;

       (* Symbol is from dynamic linking information.  *)
  BSF_DYNAMIC       = $8000;

       (* The symbol denotes a data object.  Used in ELF, and perhaps
          others someday.  *)
  BSF_OBJECT        = $10000;

       (* This symbol is a debugging symbol.  The value is the offset
          into the section of the data.  BSF_DEBUGGING should be set
          as well.  *)
  BSF_DEBUGGING_RELOC = $20000;


(* User program access to BFD facilities *)

(* Direct I/O routines, for programs which know more about the object
   file than BFD does.  Use higher level routines if possible.  *)

function bfd_read  (var data; size: bfd_size_type; nitems: bfd_size_type; abfd: PBFD): bfd_size_type;cdecl; external BFD_LIB_NAME;
function bfd_write (const data; size: bfd_size_type; nitems: bfd_size_type; abfd: PBFD): bfd_size_type;cdecl; external BFD_LIB_NAME;
function bfd_seek  (abfd: PBFD; fp: file_ptr; direction: integer): integer;cdecl; external BFD_LIB_NAME;
function bfd_tell  (abfd: PBFD): LongInt;cdecl; external BFD_LIB_NAME;
function bfd_flush (abfd: PBFD): integer;cdecl; external BFD_LIB_NAME;
function bfd_stat  (abfd: PBFD; a: Pstat): integer;cdecl; external BFD_LIB_NAME;

(* Cast from const pchar to pchar so that caller can assign to
   a pchar without a warning.  *)

function bfd_get_filename(abfd: PBFD): pchar;

function bfd_record_phdr(abfd: PBFD; a: LongWord; b: bfd_boolean; c: flagword; d: bfd_boolean; e: bfd_vma;
           f: bfd_boolean; g: bfd_boolean; h: LongWord; i: PPsec):bfd_boolean;cdecl; external BFD_LIB_NAME;

(* Byte swapping routines.  *)

function bfd_getb64         (const val):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_getl64         (const val):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_getb_signed_64 (const val):bfd_signed_vma;cdecl; external BFD_LIB_NAME;
function bfd_getl_signed_64 (const val):bfd_signed_vma;cdecl; external BFD_LIB_NAME;
function bfd_getb32         (const val):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_getl32         (const val):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_getb_signed_32 (const val):bfd_signed_vma;cdecl; external BFD_LIB_NAME;
function bfd_getl_signed_32 (const val):bfd_signed_vma;cdecl; external BFD_LIB_NAME;
function bfd_getb16         (const val):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_getl16         (const val):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_getb_signed_16 (const val):bfd_signed_vma;cdecl; external BFD_LIB_NAME;
function bfd_getl_signed_16 (const val):bfd_signed_vma;cdecl; external BFD_LIB_NAME;
procedure bfd_putb64(vma: bfd_vma; var val);cdecl; external BFD_LIB_NAME;
procedure bfd_putl64(vma: bfd_vma; var val);cdecl; external BFD_LIB_NAME;
procedure bfd_putb32(vma: bfd_vma; var val);cdecl; external BFD_LIB_NAME;
procedure bfd_putl32(vma: bfd_vma; var val);cdecl; external BFD_LIB_NAME;
procedure bfd_putb16(vma: bfd_vma; var val);cdecl; external BFD_LIB_NAME;
procedure bfd_putl16(vma: bfd_vma; var val);cdecl; external BFD_LIB_NAME;

(* Byte swapping routines which take size and endiannes as arguments.  *)

function  bfd_get_bits (a: Pbfd_byte; b:integer; c: bfd_boolean): bfd_vma;cdecl; external BFD_LIB_NAME;
procedure bfd_put_bits (a: bfd_vma; b:Pbfd_byte; c:integer; d:bfd_boolean);cdecl; external BFD_LIB_NAME;

(* Externally visible ECOFF routines.  *)

type
  Pecoff_debug_info  = pointer;
  Pecoff_debug_swap  = pointer;

function bfd_ecoff_get_gp_value(abfd: PBFD ):bfd_vma;cdecl; external BFD_LIB_NAME;
function bfd_ecoff_set_gp_value(abfd: PBFD; gp_value: bfd_vma):bfd_boolean ;cdecl; external BFD_LIB_NAME;
function bfd_ecoff_set_regmasks(abfd: PBFD; gprmask: LongWord; fprmask: LongWord;cprmask: PLongWord):bfd_boolean;cdecl; external BFD_LIB_NAME;
function bfd_ecoff_debug_init(output_bfd: PBFD; output_debug: Pecoff_debug_info; output_swap: Pecoff_debug_swap; li: Pbfd_link_info):pointer;cdecl; external BFD_LIB_NAME;
procedure bfd_ecoff_debug_free(handle: pointer; output_bfd: PBFD; output_debug: Pecoff_debug_info; output_swap: Pecoff_debug_swap;li: Pbfd_link_info);cdecl; external BFD_LIB_NAME;
function bfd_ecoff_debug_accumulate(
           handle: pointer; output_bfd: PBFD; output_debug: Pecoff_debug_info;
           output_swap: Pecoff_debug_swap;
           input_bfd: PBFD; input_debug: Pecoff_debug_info;
           input_swap: Pecoff_debug_swap;
           li: Pbfd_link_info):bfd_boolean ;cdecl; external BFD_LIB_NAME;

(*todo: function bfd_ecoff_debug_accumulate_other(
           pointer handle;
           PBFD output_bfd;
           Pecoff_debug_info output_debug;
           Pecoff_debug_swap output_swap;
           PBFD input_bfd;
           li: Pbfd_link_info
           ):bfd_boolean ;cdecl; external BFD_LIB_NAME;
*)

(*todo: function bfd_ecoff_debug_externals(
           PBFD abfd;
           Pecoff_debug_info debug;
           Pecoff_debug_swap swap;
           bfd_boolean relocateable;
           bfd_boolean ( *get_extr) (struct symbol_cache_entry *;
                                struct ecoff_extr * );
           void ( *set_index) (struct symbol_cache_entry *;
                              bfd_size_type)
           ):bfd_boolean ;cdecl; external BFD_LIB_NAME;
*)

(*todo: function bfd_ecoff_debug_one_external(
           PBFD abfd,
           Pecoff_debug_info debug,
           Pecoff_debug_swap swap,
           const ShortInt *name,
           struct ecoff_extr *esym
           ):bfd_boolean ;cdecl; external BFD_LIB_NAME;
*)

(*todo: function bfd_ecoff_debug_size(
           PBFD abfd,
           Pecoff_debug_info debug,
           Pecoff_debug_swap swap
           ):bfd_size_type;cdecl; external BFD_LIB_NAME;
*)

(*todo: function bfd_ecoff_write_debug(
           PBFD abfd,
           Pecoff_debug_info debug,
           Pecoff_debug_swap swap,
           file_ptr where
           ):bfd_boolean;cdecl; external BFD_LIB_NAME;
*)

(*todo: function bfd_ecoff_write_accumulated_debug(
           pointer handle,
           PBFD abfd,
           Pecoff_debug_info debug,
           Pecoff_debug_swap swa,
           struct bfd_link_info *info, file_ptr where)
           ):bfd_boolean;cdecl; external BFD_LIB_NAME;
*)

(*todo: function bfd_mips_ecoff_create_embedded_relocs(
           PBFD ,
           struct bfd_link_info *,
           struct sec *,
           struct sec *,
           pchar*
           ):bfd_boolean;cdecl; external BFD_LIB_NAME;
*)

(* Externally visible ELF routines.  *)

//todo:function bfd_elf32_record_link_assignment(PBFD ; struct bfd_link_info *; const pchar; bfd_boolean):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_elf64_record_link_assignment(PBFD ; struct bfd_link_info *; const pchar; bfd_boolean):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_link_needed_list *bfd_elf_get_needed_list(PBFD ; struct bfd_link_info *):struct;cdecl; external BFD_LIB_NAME;
//todo:function bfd_elf_get_bfd_needed_list(PBFD ; struct bfd_link_needed_list **):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_elf32_size_dynamic_sections (PBFD ; const pchar; const pchar; bfd_boolean; const pchar;      const pchar const *; struct bfd_link_info *; struct sec **;     struct bfd_elf_version_tree *):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_elf64_size_dynamic_sections (PBFD ; const pchar; const pchar; bfd_boolean; const pchar;      const pchar const *; struct bfd_link_info *; struct sec **;     struct bfd_elf_version_tree *):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_elf_set_dt_needed_name(PBFD ; const pchar):void;cdecl; external BFD_LIB_NAME;
//todo:function bfd_elf_set_dt_needed_soname(PBFD ; const pchar):void;cdecl; external BFD_LIB_NAME;
//todo:function const ShortInt *bfd_elf_get_dt_soname (PBFD );cdecl; external BFD_LIB_NAME;
//todo:function struct bfd_link_needed_list *bfd_elf_get_runpath_list (PBFD ; struct bfd_link_info *);cdecl; external BFD_LIB_NAME;

(* Return an upper bound on the number of bytes required to store a
   copy of ABFD's program header table entries.  Return -1 if an error
   occurs; bfd_get_error will return an appropriate code.  *)
//todo:function bfd_get_elf_phdr_upper_bound(PBFD abfd):LongInt;cdecl; external BFD_LIB_NAME;

(* Copy ABFD's program header table entries to *PHDRS.  The entries
   will be stored as an array of Elf_Internal_Phdr structures, as
   defined in include/elf/internal.h.  To find out how large the
   buffer needs to be, call bfd_get_elf_phdr_upper_bound.

   Return the number of program header table entries read, or -1 if an
   error occurs; bfd_get_error will return an appropriate code.  *)
//todo:function bfd_get_elf_phdrs(PBFD abfd, void *phdrs):integer;cdecl; external BFD_LIB_NAME;

(* Return the arch_size field of an elf bfd, or -1 if not elf.  *)
//todo:function bfd_get_arch_size(PBFD  ):integer;cdecl; external BFD_LIB_NAME;

(* Return true if address "naturally" sign extends, or -1 if not elf.  *)
//todo:function bfd_get_sign_extend_vma(PBFD  ):integer;cdecl; external BFD_LIB_NAME;

//todo:function bfd_m68k_elf32_create_embedded_relocs(PBFD , struct bfd_link_info *, struct sec *, struct sec *,           pchar*):bfd_boolean;cdecl; external BFD_LIB_NAME;

(* SunOS shared library support routines for the linker.  *)

//todo:function *bfd_sunos_get_needed_list(PBFD , struct bfd_link_info *):bfd_link_needed_list;cdecl; external BFD_LIB_NAME;
//todo:function bfd_sunos_record_link_assignment(PBFD , struct bfd_link_info *, const pchar):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_sunos_size_dynamic_sections (PBFD , struct bfd_link_info *, struct sec **, struct sec **, struct sec **):bfd_boolean;cdecl; external BFD_LIB_NAME;

(* Linux shared library support routines for the linker.  *)

//todo:function bfd_i386linux_size_dynamic_sections(PBFD , struct bfd_link_info *):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_m68klinux_size_dynamic_sections(PBFD , struct bfd_link_info *):bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:function bfd_sparclinux_size_dynamic_sections(PBFD , struct bfd_link_info *):bfd_boolean;cdecl; external BFD_LIB_NAME;

//todo:function bfd_init_window(bfd_window *):void;cdecl; external BFD_LIB_NAME;
//todo:function bfd_free_window(bfd_window *):void;cdecl; external BFD_LIB_NAME;
//todo:function bfd_get_file_window(PBFD , file_ptr, bfd_size_type, bfd_window *, bfd_boolean):bfd_boolean;cdecl; external BFD_LIB_NAME;

(* XCOFF support routines for the linker.  *)
//todo:function bfd_boolean bfd_xcoff_link_record_set (PBFD , struct bfd_link_info *, struct bfd_link_hash_entry *,        bfd_size_type));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_xcoff_import_symbol (PBFD , struct bfd_link_info *, struct bfd_link_hash_entry *,          bfd_vma, const pchar, const pchar, const pchar));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_xcoff_export_symbol  (PBFD , struct bfd_link_info *, struct bfd_link_hash_entry *,         bfd_boolean));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_xcoff_link_count_reloc  (PBFD , struct bfd_link_info *, const pchar));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_xcoff_record_link_assignment  (PBFD , struct bfd_link_info *, const pchar));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_xcoff_size_dynamic_sections  (PBFD , struct bfd_link_info *, const pchar, const pchar,     LongWord, LongWord, LongWord, bfd_boolean,      integer, bfd_boolean, bfd_boolean, struct sec **));cdecl; external BFD_LIB_NAME;

(* Externally visible COFF routines.  *)

//todo:function bfd_boolean bfd_coff_get_syment (PBFD , struct symbol_cache_entry *, struct internal_syment *));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_coff_get_auxent (PBFD , struct symbol_cache_entry *, integer, union internal_auxent *));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_coff_set_symbol_class (PBFD , struct symbol_cache_entry *, LongWord));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_m68k_coff_create_embedded_relocs (PBFD , struct bfd_link_info *, struct sec *, struct sec *,       pchar*));cdecl; external BFD_LIB_NAME;

(* ARM Interworking support.  Called from linker.  *)
//todo:function bfd_boolean bfd_arm_allocate_interworking_sections (struct bfd_link_info *));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_arm_process_before_allocation (PBFD , struct bfd_link_info *, integer));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_arm_get_bfd_for_interworking  (PBFD , struct bfd_link_info *));cdecl; external BFD_LIB_NAME;

(* PE ARM Interworking support.  Called from linker.  *)
//todo:function bfd_boolean bfd_arm_pe_allocate_interworking_sections  (struct bfd_link_info *));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_arm_pe_process_before_allocation (PBFD , struct bfd_link_info *, integer));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_arm_pe_get_bfd_for_interworking   (PBFD , struct bfd_link_info *));cdecl; external BFD_LIB_NAME;

(* ELF ARM Interworking support.  Called from linker.  *)
//todo:function bfd_boolean bfd_elf32_arm_allocate_interworking_sections  (struct bfd_link_info *));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_elf32_arm_process_before_allocation  (PBFD , struct bfd_link_info *, integer));cdecl; external BFD_LIB_NAME;
//todo:function bfd_boolean bfd_elf32_arm_get_bfd_for_interworking (PBFD , struct bfd_link_info *));cdecl; external BFD_LIB_NAME;

(* TI COFF load page support.  *)
//todo:function void bfd_ticoff_set_section_load_page (struct sec *, integer));cdecl; external BFD_LIB_NAME;
//todo:function integer bfd_ticoff_get_section_load_page  (struct sec *));cdecl; external BFD_LIB_NAME;

(* And more from the source.  *)
procedure bfd_init;cdecl; external BFD_LIB_NAME;
function bfd_openr( filename: pchar ;  target: pchar ):PBFD;cdecl; external BFD_LIB_NAME;
function bfd_fdopenr( filename: pchar ;  target: pchar ; fd: integer):PBFD;cdecl; external BFD_LIB_NAME;
function bfd_openstreamr(a: pchar; b: pchar; c: pointer):PBFD;cdecl; external BFD_LIB_NAME;
function bfd_openw( filename: pchar ;  target: pchar):PBFD;cdecl; external BFD_LIB_NAME;
function bfd_close(abfd: PBFD):bfd_boolean;cdecl; external BFD_LIB_NAME;
function bfd_close_all_done(abfd: PBFD  ):bfd_boolean;cdecl; external BFD_LIB_NAME;
function bfd_create( filename: pchar ; templ: PBFD):PBFD;cdecl; external BFD_LIB_NAME;
function bfd_make_writable(abfd: PBFD ):bfd_boolean;cdecl; external BFD_LIB_NAME;
function bfd_make_readable(abfd: PBFD ):bfd_boolean;cdecl; external BFD_LIB_NAME;

(* General purpose part of a symbol X;
   target specific parts are in libcoff.h, libaout.h, etc.  *)

//todo:#define bfd_get_section(x) ((x)->section)
//todo:#define bfd_get_output_section(x) ((x)->section->output_section)
//todo:#define bfd_set_section(x,y) ((x)->section) = (y)
//todo:#define bfd_asymbol_base(x) ((x)->section->vma)
//todo:#define bfd_asymbol_value(x) (bfd_asymbol_base(x) + (x)->value)
//todo:#define bfd_asymbol_name(x) ((x)->name)
(*Perhaps future: #define bfd_asymbol_bfd(x) ((x)->section->owner)*)
//todo:#define bfd_asymbol_bfd(x) ((x)->the_bfd)
//todo:#define bfd_asymbol_flavour(x) (bfd_asymbol_bfd(x)->xvec->flavour)

//todo:#define  align_power(addr, align) ((addr) + ((1<<(align))-1)) & (-1 << (align)))
//todo:#define bfd_get_section_name(bfd, ptr) ((ptr)->name + 0)
//todo:#define bfd_get_section_vma(bfd, ptr) ((ptr)->vma + 0)
//todo:#define bfd_get_section_alignment(bfd, ptr) ((ptr)->alignment_power + 0)
//todo:#define bfd_section_name(bfd, ptr) ((ptr)->name)
//todo:#define bfd_section_size(bfd, ptr) (bfd_get_section_size_before_reloc(ptr))
//todo:#define bfd_section_vma(bfd, ptr) ((ptr)->vma)
//todo:#define bfd_section_lma(bfd, ptr) ((ptr)->lma)
//todo:#define bfd_section_alignment(bfd, ptr) ((ptr)->alignment_power)
//todo:#define bfd_get_section_flags(bfd, ptr) ((ptr)->flags + 0)
//todo:#define bfd_get_section_userdata(bfd, ptr) ((ptr)->userdata)
//todo:#define bfd_is_com_section(ptr) (((ptr)->flags & SEC_IS_COMMON) != 0)
//todo:#define bfd_set_section_vma(bfd, ptr, val) (((ptr)->vma = (ptr)->lma= (val)), ((ptr)->user_set_vma = (bfd_boolean)true), true)
//todo:#define bfd_set_section_alignment(bfd, ptr, val) (((ptr)->alignment_power = (val)),true)
//todo:#define bfd_set_section_userdata(bfd, ptr, val) (((ptr)->userdata = (val)),true)
//todo:#define bfd_mach_sparc_v9_p(mach) ((mach) >= bfd_mach_sparc_v8plus && (mach) <= bfd_mach_sparc_v9b && (mach) != bfd_mach_sparc_sparclite_le)
//todo:#define bfd_get_elt_at_index(b,i) BFD_SEND(b, _bfd_get_elt_at_index, (b,i))
//todo:#define bfd_print_symbol(b,p,s,e) BFD_SEND(b, _bfd_print_symbol, (b,p,s,e))
//todo:#define bfd_get_symbol_info(b,p,e) BFD_SEND(b, _bfd_get_symbol_info, (b,p,e))
//todo:#define bfd_read_minisymbols(b, d, m, s)   BFD_SEND (b, _read_minisymbols, (b, d, m, s))
//todo:#define bfd_minisymbol_to_symbol(b, d, m, f)   BFD_SEND (b, _minisymbol_to_symbol, (b, d, m, f))
//todo:#define COFF_SWAP_TABLE (pointer) &bfd_coff_std_swap_table
//todo:#define bfd_get_cacheable(abfd) ((abfd)->cacheable)
//todo:#define bfd_get_format(abfd) ((abfd)->format)
//todo:#define bfd_get_target(abfd) ((abfd)->xvec->name)
//todo:#define bfd_get_flavour(abfd) ((abfd)->xvec->flavour)
//todo:#define bfd_family_coff(abfd) (bfd_get_flavour (abfd) == bfd_target_coff_flavour || bfd_get_flavour (abfd) == bfd_target_xcoff_flavour)
//todo:#define bfd_big_endian(abfd) ((abfd)->xvec->byteorder == BFD_ENDIAN_BIG)
//todo:#define bfd_little_endian(abfd) ((abfd)->xvec->byteorder == BFD_ENDIAN_LITTLE)
//todo:#define bfd_header_big_endian(abfd) ((abfd)->xvec->header_byteorder == BFD_ENDIAN_BIG)
//todo:#define bfd_header_little_endian(abfd) ((abfd)->xvec->header_byteorder == BFD_ENDIAN_LITTLE)
//todo:#define bfd_get_file_flags(abfd) ((abfd)->flags)
//todo:#define bfd_applicable_file_flags(abfd) ((abfd)->xvec->object_flags)
//todo:#define bfd_applicable_section_flags(abfd) ((abfd)->xvec->section_flags)
//todo:#define bfd_my_archive(abfd) ((abfd)->my_archive)
//todo:#define bfd_has_map(abfd) ((abfd)->has_armap)
//todo:#define bfd_valid_reloc_types(abfd) ((abfd)->xvec->valid_reloc_types)
//todo:#define bfd_usrdata(abfd) ((abfd)->usrdata)
//todo:#define bfd_get_start_address(abfd) ((abfd)->start_address)
//todo:#define bfd_get_symcount(abfd) ((abfd)->symcount)
//todo:#define bfd_get_outsymbols(abfd) ((abfd)->outsymbols)
//todo:#define bfd_count_sections(abfd) ((abfd)->section_count)
//todo:#define bfd_get_symbol_leading_char(abfd) ((abfd)->xvec->symbol_leading_char)
//todo:#define bfd_set_cacheable(abfd,bool) (((abfd)->cacheable = (bfd_boolean) (bool)), true)
//todo:#define bfd_put_8(abfd, val, ptr)  ((void) (*((byte * ) (ptr)) = (Byte) (val)))
//todo:#define bfd_put_signed_8  bfd_put_8
//todo:#define bfd_get_8(abfd, ptr)  ( *(Byte * ) (ptr))
//todo:#define bfd_get_signed_8(abfd, ptr)  (( *(byte * ) (ptr) ^ $80) - $80)

//todo:#define bfd_put_16(abfd, val, ptr)  BFD_SEND(abfd, bfd_putx16, ((val),(ptr)))
//todo:#define bfd_put_signed_16  bfd_put_16
//todo:#define bfd_get_16(abfd, ptr)   BFD_SEND(abfd, bfd_getx16, (ptr))
//todo:#define bfd_get_signed_16(abfd, ptr)  BFD_SEND (abfd, bfd_getx_signed_16, (ptr))

//todo:#define bfd_put_32(abfd, val, ptr)    BFD_SEND(abfd, bfd_putx32, ((val),(ptr)))
//todo:#define bfd_put_signed_32             bfd_put_32
//todo:#define bfd_get_32(abfd, ptr)         BFD_SEND(abfd, bfd_getx32, (ptr))
//todo:#define bfd_get_signed_32(abfd, ptr)  BFD_SEND(abfd, bfd_getx_signed_32, (ptr))

//todo:#define bfd_put_64(abfd, val, ptr)   BFD_SEND(abfd, bfd_putx64, ((val), (ptr)))
//todo:#define bfd_put_signed_64            bfd_put_64
//todo:#define bfd_get_64(abfd, ptr)        BFD_SEND(abfd, bfd_getx64, (ptr))
//todo:#define bfd_get_signed_64(abfd, ptr) BFD_SEND(abfd, bfd_getx_signed_64, (ptr))

//todo:#define bfd_get(bits, abfd, ptr) ((bits) == 8 ? bfd_get_8 (abfd, ptr) : (bits) == 16 ? bfd_get_16 (abfd, ptr) : (bits) == 32 ? bfd_get_32 (abfd, ptr) : (bits) == 64 ? bfd_get_64 (abfd, ptr) : (abort (), (bfd_vma) - 1))
//todo:#define bfd_put(bits, abfd, val, ptr) ((bits) == 8 ? bfd_put_8 (abfd, val, ptr) : (bits) == 16 ? bfd_put_16 (abfd, val, ptr) : (bits) == 32 ? bfd_put_32 (abfd, val, ptr) : (bits) == 64 ? bfd_put_64 (abfd, val, ptr) : (abort (), (void) 0))

(* Byte swapping macros for file header data.  *)

//todo:#define bfd_h_put_8(abfd, val, ptr)    bfd_put_8 (abfd, val, ptr)
//todo:#define bfd_h_put_signed_8(abfd, val, ptr) bfd_put_8 (abfd, val, ptr)
//todo:#define bfd_h_get_8(abfd, ptr)         bfd_get_8 (abfd, ptr)
//todo:#define bfd_h_get_signed_8(abfd, ptr)  bfd_get_signed_8 (abfd, ptr)

//todo:#define bfd_h_put_16(abfd, val, ptr)  BFD_SEND(abfd, bfd_h_putx16,(val,ptr))
//todo:#define bfd_h_put_signed_16           bfd_h_put_16
//todo:#define bfd_h_get_16(abfd, ptr)       BFD_SEND(abfd, bfd_h_getx16,(ptr))
//todo:#define bfd_h_get_signed_16(abfd, ptr) BFD_SEND(abfd, bfd_h_getx_signed_16, (ptr))

//todo:#define bfd_h_put_32(abfd, val, ptr)  BFD_SEND(abfd, bfd_h_putx32,(val,ptr))
//todo:#define bfd_h_put_signed_32           bfd_h_put_32
//todo:#define bfd_h_get_32(abfd, ptr)       BFD_SEND(abfd, bfd_h_getx32,(ptr))
//todo:#define bfd_h_get_signed_32(abfd, ptr) BFD_SEND(abfd, bfd_h_getx_signed_32, (ptr))

//todo:#define bfd_h_put_64(abfd, val, ptr) BFD_SEND(abfd, bfd_h_putx64,(val, ptr))
//todo:#define bfd_h_put_signed_64          bfd_h_put_64
//todo:#define bfd_h_get_64(abfd, ptr)      BFD_SEND(abfd, bfd_h_getx64,(ptr))
//todo:#define bfd_h_get_signed_64(abfd, ptr) BFD_SEND(abfd, bfd_h_getx_signed_64, (ptr))

(* These sections are global, and are managed by BFD.  The application
   and target back end are not permitted to change the values in
   these sections.  New code should use the section_ptr macros rather
   than referring directly to the const sections.  The const sections
   may eventually vanish.  *)
const
  BFD_ABS_SECTION_NAME = '*ABS*';
  BFD_UND_SECTION_NAME = '*UND*';
  BFD_COM_SECTION_NAME = '*COM*';
  BFD_IND_SECTION_NAME = '*IND*';

(* the absolute section *)
//todo:_BFD_IMPORT function const asection bfd_abs_section;
//todo:#define bfd_abs_section_ptr ((asection *) &bfd_abs_section)
//todo:#define bfd_is_abs_section(sec) ((sec) == bfd_abs_section_ptr)
(* Pointer to the undefined section *)
//todo:_BFD_IMPORT function const asection bfd_und_section;
//todo:#define bfd_und_section_ptr ((asection *) &bfd_und_section)
//todo:#define bfd_is_und_section(sec) ((sec) == bfd_und_section_ptr)
(* Pointer to the common section *)
//todo:_BFD_IMPORT function const asection bfd_com_section;
//todo:#define bfd_com_section_ptr ((asection *) &bfd_com_section)
(* Pointer to the indirect section *)
//todo:_BFD_IMPORT function const asection bfd_ind_section;
//todo:#define bfd_ind_section_ptr ((asection *) &bfd_ind_section)
//todo:#define bfd_is_ind_section(sec) ((sec) == bfd_ind_section_ptr)

//todo:function const struct symbol_cache_entry * const bfd_abs_symbol;
//todo:function const struct symbol_cache_entry * const bfd_com_symbol;
//todo:function const struct symbol_cache_entry * const bfd_und_symbol;
//todo:function const struct symbol_cache_entry * const bfd_ind_symbol;
//todo:#define bfd_get_section_size_before_reloc(section) ((section)->reloc_done ? (abort (), (bfd_size_type) 1) : (section)->_raw_size)
//todo:#define bfd_get_section_size_after_reloc(section)    ((section)->reloc_done ? (section)->_cooked_size : (abort (), (bfd_size_type) 1))
//todo:asection * bfd_get_section_by_name (PBFD abfd, const ShortInt *name));

//todo:pchar bfd_get_unique_section_name (PBFD abfd, const ShortInt *templat,  integer *count));

//todo:asection * bfd_make_section_old_way (PBFD abfd, const ShortInt *name));

//todo:asection * bfd_make_section_anyway (PBFD abfd, const ShortInt *name));

//todo:asection * bfd_make_section (PBFD , const ShortInt *name));

//todo:bfd_boolean bfd_set_section_flags (PBFD abfd, asection *sec, flagword flags));

type
  bfd_map_over_sections_proc = procedure( abfd: PBFD; sect: Pasection; obj: pointer ); cdecl;

procedure bfd_map_over_sections( abfd: PBFD; func: bfd_map_over_sections_proc; obj: pointer ); cdecl; external BFD_LIB_NAME;

//todo:bfd_boolean bfd_set_section_size (PBFD abfd, asection *sec, bfd_size_type val));

//todo:bfd_boolean bfd_set_section_contents (PBFD abfd, asection *section, pointer data, file_ptr offset,  bfd_size_type count));

function bfd_get_section_contents(abfd: PBFD; section: Pasection; location: pointer; offset: file_ptr; count: bfd_size_type): bfd_boolean; cdecl; external BFD_LIB_NAME;

//todo:bfd_boolean bfd_copy_private_section_data (PBFD ibfd, asection *isec, PBFD obfd, asection *osec));

//todo:#define bfd_copy_private_section_data(ibfd, isection, obfd, osection)   BFD_SEND (obfd, _bfd_copy_private_section_data,   (ibfd, isection, obfd, osection))
//todo:void _bfd_strip_section_from_output (struct bfd_link_info *info, asection *section));


//todo:const pchar bfd_printable_name (PBFD abfd));

//todo:const bfd_arch_info_type * bfd_scan_arch (const ShortInt *string));

//todo:const pchar* bfd_arch_list (void));

//todo:const bfd_arch_info_type * bfd_arch_get_compatible ( const PBFD abfd, const PBFD bbfd));

//todo:void bfd_set_arch_info (PBFD abfd, const bfd_arch_info_type *arg));

function bfd_get_arch( abfd: PBFD ): bfd_architecture; cdecl; external BFD_LIB_NAME;

function bfd_get_mach( abfd: PBFD ): LongWord; cdecl; external BFD_LIB_NAME;

//todo:LongWord bfd_arch_bits_per_byte (PBFD abfd));

//todo:LongWord bfd_arch_bits_per_address (PBFD abfd));

//todo:const bfd_arch_info_type * bfd_get_arch_info (PBFD abfd));

//todo:const bfd_arch_info_type * bfd_lookup_arch  (enum bfd_architecture arch, LongWord machine));

function bfd_printable_arch_mach(arch: bfd_architecture; machine: LongWord): pchar; cdecl; external BFD_LIB_NAME;

//todo:LongWord bfd_octets_per_byte (PBFD abfd));
//todo:LongWord bfd_arch_mach_octets_per_byte (enum bfd_architecture arch, LongWord machine));

//todo:#define HOWTO(C, R,S,B, P, BI, O, SF, NAME, INPLACE, MASKSRC, MASKDST, PC)  {(unsigned)C,R,S,B, P, BI, O,SF,NAME,INPLACE,MASKSRC,MASKDST,PC}
//todo:#define NEWHOWTO( FUNCTION, NAME,SIZE,REL,IN) HOWTO(0,0,SIZE,0,REL,0,complain_overflow_dont,FUNCTION, NAME,false,0,0,IN)
//todo:#define EMPTY_HOWTO(C) HOWTO((C),0,0,0,false,0,complain_overflow_dont,NULL,NULL,false,0,0,false)

//Todo:#define HOWTO_PREPARE(relocation, symbol){ if (symbol != (asymbol * )NULL) { if (bfd_is_com_section (symbol->section)) {relocation = 0;}else{ relocation = symbol->value;}}}

//todo:LongWord bfd_get_reloc_size  (reloc_howto_type *));

{todo:
  relent_chain = record
    arelent relent;
    struct relent_chain *next;
  end;
  arelent_chain = relent_chaing;
}

//todo:bfd_reloc_status_type bfd_check_overflow (enum complain_overflow how, LongWord bitsize, LongWord rightshift, LongWord addrsize,  bfd_vma relocation));

//todo:bfd_reloc_status_type bfd_perform_relocation (PBFD abfd,  arelent *reloc_entry, pointer data, asection *input_section, PBFD output_bfd, pchar*error_message));

//todo:bfd_reloc_status_type bfd_install_relocation (PBFD abfd, arelent *reloc_entry, pointer data, bfd_vma data_start, asection *input_section,  pchar*error_message));

//todo:typedef enum bfd_reloc_code_real bfd_reloc_code_real_type;

//todo:reloc_howto_type * bfd_reloc_type_lookup  (PBFD abfd, bfd_reloc_code_real_type code));

//todo:const pchar bfd_get_reloc_code_name  (bfd_reloc_code_real_type code));

//todo:#define bfd_get_symtab_upper_bound(abfd)  BFD_SEND (abfd, _bfd_get_symtab_upper_bound, (abfd))
//todo:bfd_boolean bfd_is_local_label (PBFD abfd, asymbol *sym));

//todo:bfd_boolean bfd_is_local_label_name (PBFD abfd, const ShortInt *name));

//todo:#define bfd_is_local_label_name(abfd, name)  BFD_SEND (abfd, _bfd_is_local_label_name, (abfd, name))
//todo:#define bfd_canonicalize_symtab(abfd, location) BFD_SEND (abfd, _bfd_canonicalize_symtab, (abfd, location))
//todo:bfd_boolean bfd_set_symtab  (PBFD abfd, asymbol **location, LongWord count));

//todo:void bfd_print_symbol_vandf (pointer file, asymbol *symbol));

//todo:#define bfd_make_empty_symbol(abfd)  BFD_SEND (abfd, _bfd_make_empty_symbol, (abfd))
//todo:#define bfd_make_debug_symbol(abfd,ptr,size) BFD_SEND (abfd, _bfd_make_debug_symbol, (abfd, ptr, size))
//todo:integer bfd_decode_symclass (asymbol *symbol));

//todo:bfd_boolean bfd_is_undefined_symclass  (integer symclass));

//todo:void bfd_symbol_info (asymbol *symbol, symbol_info *ret));

//todo:bfd_boolean bfd_copy_private_symbol_data (PBFD ibfd, asymbol *isym, PBFD obfd, asymbol *osym));

//todo:#define bfd_copy_private_symbol_data(ibfd, isymbol, obfd, osymbol)  BFD_SEND (obfd, _bfd_copy_private_symbol_data, (ibfd, isymbol, obfd, osymbol))


function bfd_get_error: bfd_error; cdecl; external BFD_LIB_NAME;

//todo:void bfd_set_error  (bfd_error error_tag));

function bfd_errmsg( error_tag: bfd_error ): pchar; cdecl; external BFD_LIB_NAME;

//todo:void bfd_perror  ( ShortInt *message));

type
  bfd_error_handler_type = procedure( a: pchar { VARARGS! } ); cdecl;

function bfd_set_error_handler( handler: bfd_error_handler_type ):bfd_error_handler_type; cdecl; external BFD_LIB_NAME;

//todo:void bfd_set_error_program_name  (const pchar));

//todo:bfd_error_handler_type bfd_get_error_handler  (void));

//todo:LongInt bfd_get_reloc_upper_bound (PBFD abfd, asection *sect));

//todo:LongInt bfd_canonicalize_reloc (PBFD abfd, asection *sec, arelent **loc, asymbol **syms));

//todo:void bfd_set_reloc (PBFD abfd, asection *sec, arelent **rel, LongWord count) );

//todo:bfd_boolean bfd_set_file_flags (PBFD abfd, flagword flags));

//todo:integer bfd_get_arch_size  (PBFD abfd));

//todo:integer bfd_get_sign_extend_vma  (PBFD abfd));

//todo:bfd_boolean bfd_set_start_address (PBFD abfd, bfd_vma vma));

//todo:LongInt bfd_get_mtime (PBFD abfd));

//todo:LongInt bfd_get_size (PBFD abfd));

//todo:integer bfd_get_gp_size (PBFD abfd));

//todo:void bfd_set_gp_size (PBFD abfd, integer i));

//todo:bfd_vma bfd_scan_vma ( ShortInt *string,  pchar*end, integer base));

//todo:bfd_boolean bfd_copy_private_bfd_data (PBFD ibfd, PBFD obfd));

//todo: #define bfd_copy_private_bfd_data(ibfd, obfd)      BFD_SEND (obfd, _bfd_copy_private_bfd_data,                (ibfd, obfd))
//todo:bfd_boolean bfd_merge_private_bfd_data (PBFD ibfd, PBFD obfd));

//todo:#define bfd_merge_private_bfd_data(ibfd, obfd)   BFD_SEND (obfd, _bfd_merge_private_bfd_data,(ibfd, obfd))
//todo:bfd_boolean bfd_set_private_flags (PBFD abfd, flagword flags));

//todo:#define bfd_set_private_flags(abfd, flags) BFD_SEND (abfd, _bfd_set_private_flags, (abfd, flags))
//todo:#define bfd_sizeof_headers(abfd, reloc)  BFD_SEND (abfd, _bfd_sizeof_headers, (abfd, reloc))

//todo:#define bfd_find_nearest_line(abfd, sec, syms, off, file, func, line) BFD_SEND (abfd, _bfd_find_nearest_line,  (abfd, sec, syms, off, file, func, line))

       (* Do these three do anything useful at all, for any back end?  *)
//todo:#define bfd_debug_info_start(abfd)       BFD_SEND (abfd, _bfd_debug_info_start, (abfd))

//todo:#define bfd_debug_info_end(abfd)         BFD_SEND (abfd, _bfd_debug_info_end, (abfd))

//todo:#define bfd_debug_info_accumulate(abfd, section)         BFD_SEND (abfd, _bfd_debug_info_accumulate, (abfd, section))


//todo:#define bfd_stat_arch_elt(abfd, stat)       BFD_SEND (abfd, _bfd_stat_arch_elt,(abfd, stat))

//todo:#define bfd_update_armap_timestamp(abfd)    BFD_SEND (abfd, _bfd_update_armap_timestamp, (abfd))

//todo:#define bfd_set_arch_mach(abfd, arch, mach) BFD_SEND ( abfd, _bfd_set_arch_mach, (abfd, arch, mach))

//todo:#define bfd_relax_section(abfd, section, link_info, again)        BFD_SEND (abfd, _bfd_relax_section, (abfd, section, link_info, again))

//todo:#define bfd_gc_sections(abfd, link_info)    BFD_SEND (abfd, _bfd_gc_sections, (abfd, link_info))

//todo:#define bfd_link_hash_table_create(abfd)    BFD_SEND (abfd, _bfd_link_hash_table_create, (abfd))

//todo:#define bfd_link_add_symbols(abfd, info)    BFD_SEND (abfd, _bfd_link_add_symbols, (abfd, info))

//todo:#define bfd_final_link(abfd, info)          BFD_SEND (abfd, _bfd_final_link, (abfd, info))

//todo:#define bfd_free_cached_info(abfd)          BFD_SEND (abfd, _bfd_free_cached_info, (abfd))

//todo:#define bfd_get_dynamic_symtab_upper_bound(abfd) BFD_SEND (abfd, _bfd_get_dynamic_symtab_upper_bound, (abfd))

//todo:#define bfd_print_private_bfd_data(abfd, file)   BFD_SEND (abfd, _bfd_print_private_bfd_data, (abfd, file))

//todo:#define bfd_canonicalize_dynamic_symtab(abfd, asymbols) BFD_SEND (abfd, _bfd_canonicalize_dynamic_symtab, (abfd, asymbols))

//todo:#define bfd_get_dynamic_reloc_upper_bound(abfd)         BFD_SEND (abfd, _bfd_get_dynamic_reloc_upper_bound, (abfd))

//todo:#define bfd_canonicalize_dynamic_reloc(abfd, arels, asyms) BFD_SEND (abfd, _bfd_canonicalize_dynamic_reloc, (abfd, arels, asyms))

//todo:function Pbfd_byte bfd_get_relocated_section_contents (PBFD , struct bfd_link_info *, struct bfd_link_order *, Pbfd_byte , bfd_boolean, asymbol **));cdecl; external BFD_LIB_NAME;
//todo:symindex bfd_get_next_mapent (PBFD abfd, symindex previous, carsym **sym));cdecl; external BFD_LIB_NAME;
//todo:bfd_boolean bfd_set_archive_head (PBFD output, PBFD new_head));cdecl; external BFD_LIB_NAME;
//todo:PBFD bfd_openr_next_archived_file (PBFD archive, PBFD previous));cdecl; external BFD_LIB_NAME;
//todo:pchar bfd_core_file_failing_command (PBFD abfd));cdecl; external BFD_LIB_NAME;
//todo:integer bfd_core_file_failing_signal (PBFD abfd));cdecl; external BFD_LIB_NAME;
//todo:bfd_boolean core_file_matches_executable_p (PBFD core_bfd, PBFD exec_bfd));cdecl; external BFD_LIB_NAME;

//todo:#define BFD_SEND(bfd, message, arglist)    (( *((bfd)->xvec->message)) arglist)
//todo:#define BFD_SEND_FMT(bfd, message, arglist)(((bfd)->xvec->message[(integer) ((bfd)->format)]) arglist)

//todo:bfd_boolean bfd_set_default_target  (const ShortInt *name));cdecl; external BFD_LIB_NAME;
//todo:const bfd_target * bfd_find_target ( ShortInt *target_name, PBFD abfd));cdecl; external BFD_LIB_NAME;
//todo:const pchar* bfd_target_list (void));cdecl; external BFD_LIB_NAME;
//todo:const bfd_target * bfd_search_for_target  (integer ( * search_func) (const bfd_target *, void * ), void * ));cdecl; external BFD_LIB_NAME;

function bfd_check_format(abfd: PBFD; format: bfd_format): bfd_boolean;cdecl; external BFD_LIB_NAME;
function bfd_check_format_matches(abfd: PBFD; format: bfd_format; matching: pppchar): bfd_boolean;cdecl; external BFD_LIB_NAME;
//todo:bfd_boolean bfd_set_format (PBFD abfd, bfd_format format));cdecl; external BFD_LIB_NAME;
//todo:pchar bfd_format_string (bfd_format format));cdecl; external BFD_LIB_NAME;

(* Values that may appear in the flags field of a BFD.  These also
   appear in the object_flags field of the bfd_target structure, where
   they indicate the set of flags used by that backend (not all flags
   are meaningful for all object file formats) (FIXME: at the moment,
   the object_flags values have mostly just been copied from backend
   to another, and are not necessarily correct).  *)

(* No flags.  *)
const
  BFD_NO_FLAGS  = $00;

(* BFD contains relocation entries.  *)
  HAS_RELOC     = $01;

(* BFD is directly executable.  *)
  EXEC_P        = $02;

(* BFD has line number information (basically used for F_LNNO in a
   COFF header).  *)
  HAS_LINENO    = $04;

(* BFD has debugging information.  *)
  HAS_DEBUG     = $08;

(* BFD has symbols.  *)
  HAS_SYMS      = $10;

(* BFD has local symbols (basically used for F_LSYMS in a COFF
   header).  *)
  HAS_LOCALS    = $20;

(* BFD is a dynamic object.  *)
  DYNAMIC       = $40;

(* Text section is write protected (if D_PAGED is not set, this is
   like an a.out NMAGIC file) (the linker sets this by default, but
   clears it for -r or -N).  *)
  WP_TEXT       = $80;

(* BFD is dynamically paged (this is like an a.out ZMAGIC file) (the
   linker sets this by default, but clears it for -r or -n or -N).  *)
  D_PAGED       = $100;

(* BFD is relaxable (this means that bfd_relax_section may be able to
   do something) (sometimes bfd_relax_section can do something even if
   this is not set).  *)
  BFD_IS_RELAXABLE = $200;

(* This may be set before writing out a BFD to request using a
   traditional format.  For example, this is used to request that when
   writing out an a.out object the symbols not be hashed to eliminate
   duplicates.  *)
  BFD_TRADITIONAL_FORMAT = $400;

(* This flag indicates that the BFD contents are actually cached in
   memory.  If this is set, iostream points to a bfd_in_memory struct.  *)
  BFD_IN_MEMORY = $800;

(* Get the name of a stabs type code.  *)

function bfd_get_stab_name(a:integer):pchar;cdecl; external BFD_LIB_NAME;

(* Initialize a hash table.  *)
//todo:function bfd_boolean bfd_hash_table_init  (struct bfd_hash_table *,   struct bfd_hash_entry *( * ) (struct bfd_hash_entry *, struct bfd_hash_table *, const pchar)));cdecl; external BFD_LIB_NAME;

(* Initialize a hash table specifying a size.  *)
//todo:function bfd_boolean bfd_hash_table_init_n (struct bfd_hash_table *, struct bfd_hash_entry *( * ) (struct bfd_hash_entry *,  struct bfd_hash_table *, const pchar),  LongWord size));cdecl; external BFD_LIB_NAME;

(* Free up a hash table.  *)
//todo:function void bfd_hash_table_free (struct bfd_hash_table *));cdecl; external BFD_LIB_NAME;

(* Look up a string in a hash table.  If CREATE is true, a new entry
   will be created for this string if one does not already exist.  The
   COPY argument must be true if this routine should copy the string
   into newly allocated memory when adding an entry.  *)
//todo:function struct bfd_hash_entry *bfd_hash_lookup (struct bfd_hash_table *, const pchar, bfd_boolean create,  bfd_boolean copy));cdecl; external BFD_LIB_NAME;

(* Replace an entry in a hash table.  *)
//todo:function void bfd_hash_replace (struct bfd_hash_table *, struct bfd_hash_entry *old, struct bfd_hash_entry *nw));cdecl; external BFD_LIB_NAME;

(* Base method for creating a hash table entry.  *)
//todo:function struct bfd_hash_entry *bfd_hash_newfunc (struct bfd_hash_entry *, struct bfd_hash_table *,  const pchar));cdecl; external BFD_LIB_NAME;

(* Grab some space for a hash table entry.  *)
//todo:function pointer bfd_hash_allocate (struct bfd_hash_table *, LongWord));cdecl; external BFD_LIB_NAME;

(* Traverse a hash table in a random order, calling a function on each
   element.  If the function returns false, the traversal stops.  The
   INFO argument is passed to the function.  *)
//todo:function void bfd_hash_traverse (struct bfd_hash_table *,  bfd_boolean ( * ) (struct bfd_hash_entry *, pointer), pointer info));cdecl; external BFD_LIB_NAME;

(* Print a bfd_vma x on stream s.  *)
//todo:#define fprintf_vma(s,x) fprintf (s, "%08lx", x)
//todo:#define sprintf_vma(s,x) sprintf (s, "%08lx", x)

//todo:#define printf_vma(x) fprintf_vma(stdout,x)

implementation

// Makros:

function bfd_get_filename(abfd: PBFD): pchar;
begin
  result:=abfd.filename;
end;

initialization
  bfd_init;
end.
