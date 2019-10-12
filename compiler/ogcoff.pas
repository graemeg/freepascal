{
    Copyright (c) 1998-2006 by Peter Vreman

    Contains the binary coff/PE reader and writer

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
unit ogcoff;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cclasses,globtype,
       { target }
       systems,
       { assembler }
       aasmbase,assemble,
       { output }
       ogbase,
       owbase;

    const
       PE_DATADIR_ENTRIES = 16;

    type
       tcoffpedatadir = packed record
         vaddr : longword;
         size  : longword;
       end;
       tcoffheader = packed record
         mach   : word;
         nsects : word;
         time   : longword;
         sympos : longword;
         syms   : longword;
         opthdr : word;
         flag   : word;
       end;
       tcoffbigobjheader = packed record
         Sig1 : word;
         Sig2 : word;
         Version : word;
         Machine : word;
         TimeDateStame : longword;
         UUID : array[0..15] of byte;
         unused : array[0..3] of longword;
         NumberOfSections : longword;
         PointerToSymbolTable : longword;
         NumberOfSymbols : longword;
       end;
       tcoffpeoptheader = packed record
         Magic : word;
         MajorLinkerVersion : byte;
         MinorLinkerVersion : byte;
         tsize : longword;
         dsize : longword;
         bsize : longword;
         entry : longword;
         text_start : longword;
{$ifndef cpu64bitaddr}
         data_start : longword;
{$endif cpu64bitaddr}
         ImageBase : aword;
         SectionAlignment : longword;
         FileAlignment : longword;
         MajorOperatingSystemVersion : word;
         MinorOperatingSystemVersion : word;
         MajorImageVersion : word;
         MinorImageVersion : word;
         MajorSubsystemVersion : word;
         MinorSubsystemVersion : word;
         Win32Version : longword;
         SizeOfImage : longword;
         SizeOfHeaders : longword;
         CheckSum : longword;
         Subsystem : word;
         DllCharacteristics : word;
         SizeOfStackReserve : aword;
         SizeOfStackCommit : aword;
         SizeOfHeapReserve : aword;
         SizeOfHeapCommit : aword;
         LoaderFlags : longword;          { This field is obsolete }
         NumberOfRvaAndSizes : longword;
         DataDirectory : array[0..PE_DATADIR_ENTRIES-1] of tcoffpedatadir;
       end;
       tcoffsechdr = packed record
         name     : array[0..7] of char;
         vsize    : longword;
         rvaofs   : longword;
         datasize : longword;
         datapos  : longword;
         relocpos : longword;
         lineno1  : longword;
         nrelocs  : word;
         lineno2  : word;
         flags    : longword;
       end;

       TCoffObjSection = class(TObjSection)
       private
         orgmempos,
         coffrelocs,
         coffrelocpos : aword;
       public
         constructor create(AList:TFPHashObjectList;const Aname:string;Aalign:longint;Aoptions:TObjSectionOptions);override;
         procedure writereloc_internal(aTarget:TObjSection;offset:aword;len:byte;reltype:TObjRelocationType);override;
       end;

       TCoffObjData = class(TObjData)
       private
         win32      : boolean;
{$ifdef arm}
         eVCobj     : boolean;
{$endif arm}
       public
         constructor createcoff(const n:string;awin32:boolean;acObjSection:TObjSectionClass);
         procedure CreateDebugSections;override;
         function  sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;override;
         function  sectiontype2options(atype:TAsmSectiontype):TObjSectionOptions;override;
         procedure writereloc(data:aint;len:aword;p:TObjSymbol;reloctype:TObjRelocationType);override;
       end;

       TDJCoffObjData = class(TCoffObjData)
         constructor create(const n:string);override;
       end;

       TPECoffObjData = class(TCoffObjData)
         constructor create(const n:string);override;
       end;

       TCoffObjOutput = class(tObjOutput)
       private
         win32   : boolean;
         bigobj  : boolean;
         symidx  : longint;
         FCoffSyms,
         FCoffStrs : tdynamicarray;
         procedure write_symbol(const name:string;value:aword;section:longint;typ,aux:byte);
         procedure section_write_symbol(p:TObject;arg:pointer);
         procedure section_write_relocs(p:TObject;arg:pointer);
         procedure create_symbols(data:TObjData);
         procedure section_set_reloc_datapos(p:TCoffObjSection;var datapos:aword);
         procedure section_write_header(p:TObject;arg:pointer);
       protected
         function writedata(data:TObjData):boolean;override;
       public
         constructor createcoff(AWriter:TObjectWriter;awin32:boolean);
         destructor destroy;override;
       end;

       TDJCoffObjOutput = class(TCoffObjOutput)
         constructor create(AWriter:TObjectWriter);override;
       end;

       TPECoffObjOutput = class(TCoffObjOutput)
         constructor create(AWriter:TObjectWriter);override;
       end;

       TCoffObjInput = class(tObjInput)
       private
         FCoffsyms : tdynamicarray;
         FCoffStrs : PChar;
         FCoffStrSize: longword;
         { Convert symidx -> TObjSymbol }
         FSymTbl   : ^TObjSymbolArray;
         { Convert secidx -> TObjSection }
         FSecCount : Longint;
         FSecTbl   : ^TObjSectionArray;
         win32     : boolean;
         bigobj    : boolean;
         function  GetSection(secidx:longint):TObjSection;
         function  Read_str(strpos:longword):string;
         procedure read_relocs(s:TCoffObjSection);
         procedure read_symbols(objdata:TObjData);
         procedure ObjSections_read_relocs(p:TObject;arg:pointer);
       public
         constructor createcoff(awin32:boolean);
         destructor destroy;override;
         function  ReadObjData(AReader:TObjectreader;out objdata:TObjData):boolean;override;
       end;

       TDJCoffObjInput = class(TCoffObjInput)
         constructor create;override;
       end;

       TPECoffObjInput = class(TCoffObjInput)
         constructor create;override;
       end;

       TCoffexeoutput = class(texeoutput)
       private
         FCoffStrs : tdynamicarray;
         win32     : boolean;
         nsects    : word;
         nsyms,
         sympos    : aword;
         datapos_offset: longword;
         function  totalheadersize:longword;
         procedure ExeSectionList_pass2_header(p:TObject;arg:pointer);
         procedure write_symbol(const name:string;value:aword;section:smallint;typ,aux:byte);
         procedure globalsyms_write_symbol(p:TObject;arg:pointer);
         procedure ExeSectionList_write_header(p:TObject;arg:pointer);
       protected
         function writedata:boolean;override;
         procedure Order_ObjSectionList(ObjSectionList : TFPObjectList;const aPattern:string);override;
         procedure DoRelocationFixup(objsec:TObjSection);override;
       public
         constructor createcoff(awin32:boolean);
         procedure MemPos_Header;override;
         procedure DataPos_Header;override;
         procedure DataPos_Symbols;override;
       end;

       TDJCoffexeoutput = class(TCoffexeoutput)
         constructor create;override;
         procedure MemPos_Header;override;
       end;

       TPECoffexeoutput = class(TCoffexeoutput)
       private
         FImports: TFPHashObjectList;
         textobjsection,
         idata2objsection,
         idata4objsection,
         idata5objsection,
         idata6objsection,
         idata7objsection : TObjSection;
         FRelocsGenerated : boolean;
         procedure GenerateRelocs;
       public
         constructor create;override;
         procedure MarkTargetSpecificSections(WorkList:TFPObjectList);override;
         procedure AfterUnusedSectionRemoval;override;
         procedure GenerateLibraryImports(ImportLibraryList:TFPHashObjectList);override;
         procedure MemPos_Start;override;
         procedure MemPos_ExeSection(const aname:string);override;
       end;

       TObjSymbolArray = array[0..high(word)] of TObjSymbol;
       TObjSectionArray = array[0..high(smallint)] of TObjSection;

       TDJCoffAssembler = class(tinternalassembler)
         constructor create(info: pasminfo; smart:boolean);override;
       end;

       TPECoffassembler = class(tinternalassembler)
         constructor create(info: pasminfo; smart:boolean);override;
       end;


    type
      Treaddllproc = procedure(const dllname,funcname:string) of object;

    const
{$ifdef i386}
       COFF_MAGIC       = $14c;
       COFF_OPT_MAGIC   = $10b;
       TLSDIR_SIZE      = $18;
{$endif i386}
{$ifdef arm}
       COFF_OPT_MAGIC   = $10b;
       TLSDIR_SIZE      = $18;

       function COFF_MAGIC: word;

     const
{$endif arm}
{$ifdef x86_64}
       COFF_MAGIC       = $8664;
       COFF_OPT_MAGIC   = $20b;
       TLSDIR_SIZE      = $28;
{$endif x86_64}
       COFF_BIG_OBJ_MAGIC: array[0..15] of byte = ($C7, $A1, $BA, $D1, $EE, $BA, $A9, $4B, $AF, $20, $FA, $F6, $6A, $A4, $DC, $B8);
       COFF_BIG_OBJ_VERSION = 2;

    function ReadDLLImports(const dllname:string;readdllproc:Treaddllproc):boolean;

implementation

    uses
{$ifdef win32}
       Windows,
{$endif win32}
       SysUtils,
       cutils,verbose,globals,
       cpubase,cpuinfo,
       fmodule,
       ogmap,
       owar,
       version
       ;

    const
       COFF_FLAG_NORELOCS = $0001;
       COFF_FLAG_EXE      = $0002;
       COFF_FLAG_NOLINES  = $0004;
       COFF_FLAG_NOLSYMS  = $0008;
       COFF_FLAG_AR16WR   = $0080; { 16bit little endian }
       COFF_FLAG_AR32WR   = $0100; { 32bit little endian }
       COFF_FLAG_AR32W    = $0200; { 32bit big endian }
       COFF_FLAG_DLL      = $2000;

       COFF_SYM_GLOBAL   = 2;
       COFF_SYM_LOCAL    = 3;
       COFF_SYM_LABEL    = 6;
       COFF_SYM_FUNCTION = 101;
       COFF_SYM_FILE     = 103;
       COFF_SYM_SECTION  = 104;

       COFF_STYP_REG    = $0000; { "regular": allocated, relocated, loaded }
       COFF_STYP_DSECT  = $0001; { "dummy":  relocated only }
       COFF_STYP_NOLOAD = $0002; { "noload": allocated, relocated, not loaded }
       COFF_STYP_GROUP  = $0004; { "grouped": formed of input sections }
       COFF_STYP_PAD    = $0008;
       COFF_STYP_COPY   = $0010;
       COFF_STYP_TEXT   = $0020;
       COFF_STYP_DATA   = $0040;
       COFF_STYP_BSS    = $0080;
       COFF_STYP_INFO   = $0200;
       COFF_STYP_OVER   = $0400;
       COFF_STYP_LIB    = $0800;

       PE_SUBSYSTEM_NATIVE         = 1;
       PE_SUBSYSTEM_WINDOWS_GUI    = 2;
       PE_SUBSYSTEM_WINDOWS_CUI    = 3;
       PE_SUBSYSTEM_WINDOWS_CE_GUI = 9;

       PE_FILE_RELOCS_STRIPPED         = $0001;
       PE_FILE_EXECUTABLE_IMAGE        = $0002;
       PE_FILE_LINE_NUMS_STRIPPED      = $0004;
       PE_FILE_LOCAL_SYMS_STRIPPED     = $0008;
       PE_FILE_AGGRESSIVE_WS_TRIM      = $0010;
       PE_FILE_LARGE_ADDRESS_AWARE     = $0020;
       PE_FILE_16BIT_MACHINE           = $0040;
       PE_FILE_BYTES_REVERSED_LO       = $0080;
       PE_FILE_32BIT_MACHINE           = $0100;
       PE_FILE_DEBUG_STRIPPED          = $0200;
       PE_FILE_REMOVABLE_RUN_FROM_SWAP = $0400;
       PE_FILE_NET_RUN_FROM_SWAP       = $0800;
       PE_FILE_SYSTEM                  = $1000;
       PE_FILE_DLL                     = $2000;
       PE_FILE_UP_SYSTEM_ONLY          = $4000;
       PE_FILE_BYTES_REVERSED_HI       = $8000;

       PE_SCN_CNT_CODE               = $00000020; { Section contains code. }
       PE_SCN_CNT_INITIALIZED_DATA   = $00000040; { Section contains initialized data. }
       PE_SCN_CNT_UNINITIALIZED_DATA = $00000080; { Section contains uninitialized data. }
       PE_SCN_LNK_OTHER              = $00000100; { Reserved. }
       PE_SCN_LNK_INFO               = $00000200; { Section contains comments or some other type of information. }
       PE_SCN_LNK_REMOVE             = $00000800; { Section contents will not become part of image. }
       PE_SCN_LNK_COMDAT             = $00001000; { Section contents comdat. }
       PE_SCN_MEM_FARDATA            = $00008000;
       PE_SCN_MEM_PURGEABLE          = $00020000;
       PE_SCN_MEM_16BIT              = $00020000;
       PE_SCN_MEM_LOCKED             = $00040000;
       PE_SCN_MEM_PRELOAD            = $00080000;
       PE_SCN_ALIGN_MASK             = $00f00000;
       PE_SCN_ALIGN_1BYTES           = $00100000;
       PE_SCN_ALIGN_2BYTES           = $00200000;
       PE_SCN_ALIGN_4BYTES           = $00300000;
       PE_SCN_ALIGN_8BYTES           = $00400000;
       PE_SCN_ALIGN_16BYTES          = $00500000; { Default alignment if no others are specified. }
       PE_SCN_ALIGN_32BYTES          = $00600000;
       PE_SCN_ALIGN_64BYTES          = $00700000;
       PE_SCN_ALIGN_128BYTES         = $00800000;
       PE_SCN_ALIGN_256BYTES         = $00900000;
       PE_SCN_ALIGN_512BYTES         = $00A00000;
       PE_SCN_ALIGN_1024BYTES        = $00B00000;
       PE_SCN_ALIGN_2048BYTES        = $00C00000;
       PE_SCN_ALIGN_4096BYTES        = $00D00000;
       PE_SCN_ALIGN_8192BYTES        = $00E00000;
       PE_SCN_LNK_NRELOC_OVFL        = $01000000; { Section contains extended relocations. }
       PE_SCN_MEM_NOT_CACHED         = $04000000; { Section is not cachable.               }
       PE_SCN_MEM_NOT_PAGED          = $08000000; { Section is not pageable.               }
       PE_SCN_MEM_SHARED             = $10000000; { Section is shareable.                  }
       PE_SCN_MEM_DISCARDABLE        = $02000000;
       PE_SCN_MEM_EXECUTE            = $20000000;
       PE_SCN_MEM_READ               = $40000000;
       PE_SCN_MEM_WRITE              = $80000000;

       PE_DATADIR_EDATA = 0;
       PE_DATADIR_IDATA = 1;
       PE_DATADIR_RSRC = 2;
       PE_DATADIR_PDATA = 3;
       PE_DATADIR_SECURITY = 4;
       PE_DATADIR_RELOC = 5;
       PE_DATADIR_DEBUG = 6;
       PE_DATADIR_DESCRIPTION = 7;
       PE_DATADIR_SPECIAL = 8;
       PE_DATADIR_TLS = 9;
       PE_DATADIR_LOADCFG = 10;
       PE_DATADIR_BOUNDIMPORT = 11;
       PE_DATADIR_IMPORTADDRESSTABLE = 12;
       PE_DATADIR_DELAYIMPORT = 13;

{$ifdef x86_64}
       IMAGE_REL_AMD64_ABSOLUTE    = $0000;  { Reference is absolute, no relocation is necessary }
       IMAGE_REL_AMD64_ADDR64      = $0001;  { 64-bit address (VA). }
       IMAGE_REL_AMD64_ADDR32      = $0002;  { 32-bit address (VA). }
       IMAGE_REL_AMD64_ADDR32NB    = $0003;  { 32-bit address w/o image base (RVA). }
       IMAGE_REL_AMD64_REL32       = $0004;  { 32-bit relative address from byte following reloc }
       IMAGE_REL_AMD64_REL32_1     = $0005;  { 32-bit relative address from byte distance 1 from reloc }
       IMAGE_REL_AMD64_REL32_2     = $0006;  { 32-bit relative address from byte distance 2 from reloc }
       IMAGE_REL_AMD64_REL32_3     = $0007;  { 32-bit relative address from byte distance 3 from reloc }
       IMAGE_REL_AMD64_REL32_4     = $0008;  { 32-bit relative address from byte distance 4 from reloc }
       IMAGE_REL_AMD64_REL32_5     = $0009;  { 32-bit relative address from byte distance 5 from reloc }
       IMAGE_REL_AMD64_SECTION     = $000A;  { Section index }
       IMAGE_REL_AMD64_SECREL      = $000B;  { 32 bit offset from base of section containing target }
       IMAGE_REL_AMD64_SECREL7     = $000C;  { 7 bit unsigned offset from base of section containing target }
       IMAGE_REL_AMD64_TOKEN       = $000D;  { 32 bit metadata token }
       IMAGE_REL_AMD64_SREL32      = $000E;  { 32 bit signed span-dependent value emitted into object }
       IMAGE_REL_AMD64_PAIR        = $000F;
       IMAGE_REL_AMD64_SSPAN32     = $0010;  { 32 bit signed span-dependent value applied at link time }
      { Direct 32 bit sign extended,
        win64 mingw GNU compiler
        also generates this type
        inside coff objects
        We assume they are equivalent to
        IMAGE_REL_AMD64_ADDR32  PM 2010-11-27 }
       R_X86_64_32S =           $11;

{$endif x86_64}

{$ifdef arm}
       IMAGE_REL_ARM_ABSOLUTE      = $0000;     { No relocation required }
       IMAGE_REL_ARM_ADDR32        = $0001;     { 32 bit address }
       IMAGE_REL_ARM_ADDR32NB      = $0002;     { 32 bit address w/o image base }
       IMAGE_REL_ARM_BRANCH24      = $0003;     { 24 bit offset << 2 & sign ext. }
       IMAGE_REL_ARM_BRANCH11      = $0004;     { Thumb: 2 11 bit offsets }
       IMAGE_REL_ARM_TOKEN         = $0005;     { clr token }
       IMAGE_REL_ARM_GPREL12       = $0006;     { GP-relative addressing (ARM) }
       IMAGE_REL_ARM_GPREL7        = $0007;     { GP-relative addressing (Thumb) }
       IMAGE_REL_ARM_BLX24         = $0008;
       IMAGE_REL_ARM_BLX11         = $0009;
       IMAGE_REL_ARM_SECTION       = $000E;     { Section table index }
       IMAGE_REL_ARM_SECREL        = $000F;     { Offset within section }
       IMAGE_REL_ARM_MOV32A        = $0010;     { 32-bit VA applied to MOVW+MOVT pair, added to existing imm (ARM) }
       IMAGE_REL_ARM_MOV32T        = $0011;     { 32-bit VA applied to MOVW+MOVT pair, added to existing imm (THUMB) }
       IMAGE_REL_ARM_BRANCH20T     = $0012;     { Thumb: 20 most significant bits of 32 bit B cond instruction }
       IMAGE_REL_ARM_BRANCH24T     = $0014;     { Thumb: 24 most significant bits of 32 bit B uncond instruction }
       IMAGE_REL_ARM_BLX23T        = $0015;     { 23 most significant bits of 32 bit BL/BLX instruction. Transformed to BLX if target is Thumb }
{$endif arm}

{$ifdef i386}
       IMAGE_REL_I386_DIR32 = 6;
       IMAGE_REL_I386_IMAGEBASE = 7;
       IMAGE_REL_I386_SECREL32 = 11;
       IMAGE_REL_I386_PCRLONG = 20;
{$endif i386}

       { .reloc section fixup types }
       IMAGE_REL_BASED_HIGHLOW     = 3;  { Applies the delta to the 32-bit field at Offset. }
       IMAGE_REL_BASED_DIR64       = 10; { Applies the delta to the 64-bit field at Offset. }

       { values for coffsectionrec.select }
       IMAGE_COMDAT_SELECT_NODUPLICATES = 1;
       IMAGE_COMDAT_SELECT_ANY          = 2;
       IMAGE_COMDAT_SELECT_SAME_SIZE    = 3;
       IMAGE_COMDAT_SELECT_EXACT_MATCH  = 4;
       IMAGE_COMDAT_SELECT_ASSOCIATIVE  = 5;
       IMAGE_COMDAT_SELECT_LARGEST      = 6;

    type
       coffdjoptheader=packed record
         magic  : word;
         vstamp : word;
         tsize  : longint;
         dsize  : longint;
         bsize  : longint;
         entry  : longint;
         text_start : longint;
         data_start : longint;
       end;
       coffsectionrec=packed record
         len     : longword;
         nrelocs : word;
         nlines  : word;
         checksum: longword;
         assoc   : word;
         select  : byte;
         empty   : array[0..2] of char;
       end;
       pcoffsectionrec=^coffsectionrec;
       coffreloc=packed record
         address  : longword;
         sym      : longword;
         reloctype : word;
       end;
       strtableoffset=packed record
         Zeroes : longword;
         Offset : longword;
       end;
       coffsymbol=packed record
         name    : array[0..3] of char; { real is [0..7], which overlaps the strpos ! }
         strpos  : longword;
         value   : longword;
         section : smallint;
         empty   : word;                { actually type, $20: function, 0: not a function }
         typ     : byte;
         aux     : byte;
       end;
       coffbigobjsymbol=packed record
         Name               : record
                                case boolean of
                                  True: (ShortName : array[0..7] of char);
                                  False: (Offset : strtableoffset)
                              end;
         Value              : longword;
         SectionNumber      : longword;
         _Type              : word;
         StorageClass       : byte;
         NumberOfAuxSymbols : byte;
       end;

       { This is defined in rtl/win/sysos.inc source }
       tlsdirectory=packed record
         data_start, data_end : PUInt;
         index_pointer, callbacks_pointer : PUInt;
         zero_fill_size : dword;
         flags : dword;
       end;

     const
       SymbolMaxGrow = 200*sizeof(coffsymbol);
       StrsMaxGrow   = 8192;

       coffsecnames : array[TAsmSectiontype] of string[length('__DATA, __datacoal_nt,coalesced')] = ('','',
          '.text','.data','.rdata','.rdata','.bss','.tls',
          '.pdata',{pdata}
          '.text', {stub}
          '.data',
          '.data',
          '.data',
          '.data',
          '.stab','.stabstr',
          '.idata$2','.idata$4','.idata$5','.idata$6','.idata$7','.edata',
          '.eh_frame',
          '.debug_frame','.debug_info','.debug_line','.debug_abbrev','.debug_aranges','.debug_ranges',
          '.fpc',
          '',
          '.init',
          '.fini',
          '.objc_class',
          '.objc_meta_class',
          '.objc_cat_cls_meth',
          '.objc_cat_inst_meth',
          '.objc_protocol',
          '.objc_string_object',
          '.objc_cls_meth',
          '.objc_inst_meth',
          '.objc_cls_refs',
          '.objc_message_refs',
          '.objc_symbols',
          '.objc_category',
          '.objc_class_vars',
          '.objc_instance_vars',
          '.objc_module_info',
          '.objc_class_names',
          '.objc_meth_var_types',
          '.objc_meth_var_names',
          '.objc_selector_strs',
          '.objc_protocol_ext',
          '.objc_class_ext',
          '.objc_property',
          '.objc_image_info',
          '.objc_cstring_object',
          '.objc_sel_fixup',
          '__DATA,__objc_data',
          '__DATA,__objc_const',
          '.objc_superrefs',
          '__DATA, __datacoal_nt,coalesced',
          '.objc_classlist',
          '.objc_nlclasslist',
          '.objc_catlist',
          '.obcj_nlcatlist',
          '.objc_protolist',
          '.stack',
          '.heap',
          '.gcc_except_table',
          '.ARM.attributes'
        );

const go32v2stub : array[0..2047] of byte=(
  $4D,$5A,$00,$00,$04,$00,$00,$00,$20,$00,$27,$00,$FF,$FF,$00,
  $00,$60,$07,$00,$00,$54,$00,$00,$00,$00,$00,$00,$00,$0D,$0A,
  $73,$74,$75,$62,$2E,$68,$20,$67,$65,$6E,$65,$72,$61,$74,$65,
  $64,$20,$66,$72,$6F,$6D,$20,$73,$74,$75,$62,$2E,$61,$73,$6D,
  $20,$62,$79,$20,$64,$6A,$61,$73,$6D,$2C,$20,$6F,$6E,$20,$54,
  $68,$75,$20,$44,$65,$63,$20,$20,$39,$20,$31,$30,$3A,$35,$39,
  $3A,$33,$31,$20,$31,$39,$39,$39,$0D,$0A,$54,$68,$65,$20,$53,
  $54,$55,$42,$2E,$45,$58,$45,$20,$73,$74,$75,$62,$20,$6C,$6F,
  $61,$64,$65,$72,$20,$69,$73,$20,$43,$6F,$70,$79,$72,$69,$67,
  $68,$74,$20,$28,$43,$29,$20,$31,$39,$39,$33,$2D,$31,$39,$39,
  $35,$20,$44,$4A,$20,$44,$65,$6C,$6F,$72,$69,$65,$2E,$20,$0D,
  $0A,$50,$65,$72,$6D,$69,$73,$73,$69,$6F,$6E,$20,$67,$72,$61,
  $6E,$74,$65,$64,$20,$74,$6F,$20,$75,$73,$65,$20,$66,$6F,$72,
  $20,$61,$6E,$79,$20,$70,$75,$72,$70,$6F,$73,$65,$20,$70,$72,
  $6F,$76,$69,$64,$65,$64,$20,$74,$68,$69,$73,$20,$63,$6F,$70,
  $79,$72,$69,$67,$68,$74,$20,$0D,$0A,$72,$65,$6D,$61,$69,$6E,
  $73,$20,$70,$72,$65,$73,$65,$6E,$74,$20,$61,$6E,$64,$20,$75,
  $6E,$6D,$6F,$64,$69,$66,$69,$65,$64,$2E,$20,$0D,$0A,$54,$68,
  $69,$73,$20,$6F,$6E,$6C,$79,$20,$61,$70,$70,$6C,$69,$65,$73,
  $20,$74,$6F,$20,$74,$68,$65,$20,$73,$74,$75,$62,$2C,$20,$61,
  $6E,$64,$20,$6E,$6F,$74,$20,$6E,$65,$63,$65,$73,$73,$61,$72,
  $69,$6C,$79,$20,$74,$68,$65,$20,$77,$68,$6F,$6C,$65,$20,$70,
  $72,$6F,$67,$72,$61,$6D,$2E,$0A,$0D,$0A,$24,$49,$64,$3A,$20,
  $73,$74,$75,$62,$2E,$61,$73,$6D,$20,$62,$75,$69,$6C,$74,$20,
  $31,$32,$2F,$30,$39,$2F,$39,$39,$20,$31,$30,$3A,$35,$39,$3A,
  $33,$31,$20,$62,$79,$20,$64,$6A,$61,$73,$6D,$20,$24,$0A,$0D,
  $0A,$40,$28,$23,$29,$20,$73,$74,$75,$62,$2E,$61,$73,$6D,$20,
  $62,$75,$69,$6C,$74,$20,$31,$32,$2F,$30,$39,$2F,$39,$39,$20,
  $31,$30,$3A,$35,$39,$3A,$33,$31,$20,$62,$79,$20,$64,$6A,$61,
  $73,$6D,$0A,$0D,$0A,$1A,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$67,$6F,$33,$32,$73,$74,$75,$62,$2C,$20,$76,$20,$32,
  $2E,$30,$32,$54,$00,$00,$00,$00,$00,$08,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$40,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$43,$57,$53,$44,$50,
  $4D,$49,$2E,$45,$58,$45,$00,$00,$00,$00,$00,$0E,$1F,$8C,$1E,
  $24,$00,$8C,$06,$60,$07,$FC,$B4,$30,$CD,$21,$3C,$03,$73,$08,
  $B0,$6D,$BA,$A7,$05,$E9,$D4,$03,$A2,$69,$08,$BE,$20,$00,$8B,
  $04,$09,$C0,$75,$02,$B4,$FE,$BB,$70,$08,$39,$C3,$73,$02,$89,
  $C3,$89,$1C,$FE,$C7,$B9,$04,$FF,$D3,$EB,$B4,$4A,$CD,$21,$73,
  $08,$D3,$E3,$FE,$CF,$89,$1C,$EB,$D8,$26,$8E,$06,$2C,$00,$31,
  $FF,$30,$C0,$A9,$F2,$AE,$26,$81,$3D,$50,$41,$75,$15,$AF,$26,
  $81,$3D,$54,$48,$75,$0D,$AF,$26,$80,$3D,$3D,$75,$06,$47,$89,
  $3E,$8C,$04,$4F,$AE,$75,$DF,$AF,$B4,$3E,$BB,$13,$00,$CD,$21,
  $B4,$3E,$BB,$12,$00,$CD,$21,$06,$57,$31,$C9,$74,$12,$B0,$6E,
  $BA,$7E,$05,$E9,$5E,$03,$09,$C9,$75,$F4,$41,$E8,$A1,$03,$72,
  $EE,$B8,$87,$16,$CD,$2F,$09,$C0,$75,$ED,$80,$E3,$01,$74,$E8,
  $89,$3E,$00,$06,$8C,$06,$02,$06,$89,$36,$04,$06,$5F,$07,$E8,
  $D3,$02,$89,$3E,$2A,$00,$89,$36,$62,$07,$80,$3E,$2C,$00,$00,
  $74,$23,$B9,$08,$00,$BF,$2C,$00,$8A,$05,$47,$08,$C0,$74,$05,
  $88,$07,$43,$E2,$F4,$66,$C7,$07,$2E,$45,$58,$45,$83,$C3,$04,
  $C6,$07,$00,$89,$1E,$62,$07,$B8,$00,$3D,$BA,$64,$07,$CD,$21,
  $0F,$82,$B3,$02,$A3,$06,$06,$89,$C3,$B9,$06,$00,$BA,$B5,$07,
  $B4,$3F,$CD,$21,$31,$D2,$31,$C9,$A1,$B5,$07,$3D,$4C,$01,$74,
  $1B,$3D,$4D,$5A,$0F,$85,$98,$02,$8B,$16,$B9,$07,$C1,$E2,$09,
  $8B,$1E,$B7,$07,$09,$DB,$74,$05,$80,$EE,$02,$01,$DA,$89,$16,
  $BB,$07,$89,$0E,$BD,$07,$B8,$00,$42,$8B,$1E,$06,$06,$CD,$21,
  $B9,$A8,$00,$BA,$BF,$07,$B4,$3F,$CD,$21,$3D,$A8,$00,$75,$06,
  $81,$3E,$BF,$07,$4C,$01,$0F,$85,$61,$02,$66,$A1,$E3,$07,$66,
  $A3,$10,$06,$66,$8B,$0E,$BB,$07,$66,$A1,$03,$08,$66,$01,$C8,
  $66,$A3,$08,$06,$66,$A1,$2B,$08,$66,$01,$C8,$66,$A3,$0C,$06,
  $66,$8B,$1E,$4B,$08,$66,$A1,$4F,$08,$66,$01,$C3,$66,$B8,$01,
  $00,$01,$00,$66,$39,$C3,$73,$03,$66,$89,$C3,$66,$81,$C3,$FF,
  $FF,$00,$00,$31,$DB,$66,$89,$1E,$1C,$00,$E8,$F5,$02,$8B,$1E,
  $04,$06,$09,$DB,$74,$0A,$B4,$48,$CD,$21,$0F,$82,$15,$02,$8E,
  $C0,$E8,$08,$03,$B8,$01,$00,$FF,$1E,$00,$06,$0F,$82,$0F,$02,
  $8C,$06,$26,$00,$8C,$0E,$28,$00,$8C,$D8,$A3,$22,$00,$8E,$C0,
  $31,$C0,$B9,$01,$00,$CD,$31,$72,$07,$A3,$14,$06,$31,$C0,$CD,
  $31,$0F,$82,$F3,$01,$A3,$16,$06,$66,$8B,$0E,$1C,$00,$B8,$01,
  $05,$8B,$1E,$1E,$00,$CD,$31,$0F,$82,$E5,$01,$89,$1E,$1A,$06,
  $89,$0E,$18,$06,$89,$36,$1A,$00,$89,$3E,$18,$00,$B8,$07,$00,
  $8B,$1E,$14,$06,$8B,$0E,$1A,$06,$8B,$16,$18,$06,$CD,$31,$B8,
  $09,$00,$8C,$C9,$83,$E1,$03,$C1,$E1,$05,$51,$81,$C9,$9B,$C0,
  $CD,$31,$B8,$08,$00,$8B,$0E,$1E,$00,$49,$BA,$FF,$FF,$CD,$31,
  $B8,$07,$00,$8B,$1E,$16,$06,$8B,$0E,$1A,$06,$8B,$16,$18,$06,
  $CD,$31,$B8,$09,$00,$59,$81,$C9,$93,$C0,$CD,$31,$B8,$08,$00,
  $8B,$0E,$1E,$00,$49,$BA,$FF,$FF,$CD,$31,$B8,$00,$01,$BB,$00,
  $0F,$CD,$31,$73,$10,$3D,$08,$00,$0F,$85,$73,$01,$B8,$00,$01,
  $CD,$31,$0F,$82,$6A,$01,$A3,$1C,$06,$89,$16,$1E,$06,$C1,$E3,
  $04,$89,$1E,$20,$06,$66,$8B,$36,$08,$06,$66,$8B,$3E,$FB,$07,
  $66,$8B,$0E,$FF,$07,$E8,$49,$00,$66,$8B,$36,$0C,$06,$66,$8B,
  $3E,$23,$08,$66,$8B,$0E,$27,$08,$E8,$37,$00,$8E,$06,$16,$06,
  $66,$8B,$3E,$4B,$08,$66,$8B,$0E,$4F,$08,$66,$31,$C0,$66,$C1,
  $E9,$02,$67,$F3,$66,$AB,$B4,$3E,$8B,$1E,$06,$06,$CD,$21,$B8,
  $01,$01,$8B,$16,$1E,$06,$CD,$31,$1E,$0F,$A1,$8E,$1E,$16,$06,
  $66,$64,$FF,$2E,$10,$06,$66,$89,$F0,$66,$25,$FF,$01,$00,$00,
  $66,$01,$C1,$29,$C6,$66,$29,$C7,$66,$89,$0E,$26,$06,$66,$89,
  $3E,$22,$06,$E8,$0F,$01,$89,$36,$3E,$06,$66,$C1,$EE,$10,$89,
  $36,$42,$06,$8B,$1E,$06,$06,$89,$1E,$3A,$06,$C7,$06,$46,$06,
  $00,$42,$E8,$03,$01,$A1,$1C,$06,$A3,$4E,$06,$C7,$06,$3E,$06,
  $00,$00,$C6,$06,$47,$06,$3F,$A1,$28,$06,$09,$C0,$75,$09,$A1,
  $26,$06,$3B,$06,$20,$06,$76,$03,$A1,$20,$06,$A3,$42,$06,$E8,
  $D9,$00,$66,$31,$C9,$8B,$0E,$46,$06,$66,$8B,$3E,$22,$06,$66,
  $01,$0E,$22,$06,$66,$29,$0E,$26,$06,$66,$31,$F6,$C1,$E9,$02,
  $1E,$06,$8E,$06,$16,$06,$8E,$1E,$1E,$06,$67,$F3,$66,$A5,$07,
  $1F,$66,$03,$0E,$26,$06,$75,$AF,$C3,$3C,$3A,$74,$06,$3C,$2F,
  $74,$02,$3C,$5C,$C3,$BE,$64,$07,$89,$F3,$26,$8A,$05,$47,$88,
  $04,$38,$E0,$74,$0E,$08,$C0,$74,$0A,$46,$E8,$DE,$FF,$75,$EC,
  $89,$F3,$74,$E8,$C3,$B0,$66,$BA,$48,$05,$EB,$0C,$B0,$67,$BA,
  $55,$05,$EB,$05,$B0,$68,$BA,$5F,$05,$52,$8B,$1E,$62,$07,$C6,
  $07,$24,$BB,$64,$07,$EB,$28,$E8,$F5,$00,$B0,$69,$BA,$99,$05,
  $EB,$1A,$B0,$6A,$BA,$B2,$05,$EB,$13,$B0,$6B,$BA,$C4,$05,$EB,
  $0C,$B0,$6C,$BA,$D6,$05,$EB,$05,$B0,$69,$BA,$99,$05,$52,$BB,
  $3B,$05,$E8,$15,$00,$5B,$E8,$11,$00,$BB,$67,$04,$E8,$0B,$00,
  $B4,$4C,$CD,$21,$43,$50,$B4,$02,$CD,$21,$58,$8A,$17,$80,$FA,
  $24,$75,$F2,$C3,$0D,$0A,$24,$50,$51,$57,$31,$C0,$BF,$2A,$06,
  $B9,$19,$00,$F3,$AB,$5F,$59,$58,$C3,$B8,$00,$03,$BB,$21,$00,
  $31,$C9,$66,$BF,$2A,$06,$00,$00,$CD,$31,$C3,$00,$00,$30,$E4,
  $E8,$4E,$FF,$89,$DE,$8B,$3E,$8C,$04,$EB,$17,$B4,$3B,$E8,$41,
  $FF,$81,$FE,$64,$07,$74,$12,$8A,$44,$FF,$E8,$2A,$FF,$74,$04,
  $C6,$04,$5C,$46,$E8,$03,$00,$72,$E4,$C3,$E8,$34,$00,$BB,$44,
  $00,$8A,$07,$88,$04,$43,$46,$08,$C0,$75,$F6,$06,$57,$1E,$07,
  $E8,$9B,$FF,$BB,$2A,$06,$8C,$5F,$04,$89,$5F,$02,$BA,$64,$07,
  $B8,$00,$4B,$CD,$21,$5F,$07,$72,$09,$B4,$4D,$CD,$21,$2D,$00,
  $03,$F7,$D8,$EB,$28,$80,$3E,$69,$08,$05,$72,$20,$B8,$00,$58,
  $CD,$21,$A2,$67,$08,$B8,$02,$58,$CD,$21,$A2,$68,$08,$B8,$01,
  $58,$BB,$80,$00,$CD,$21,$B8,$03,$58,$BB,$01,$00,$CD,$21,$C3,
  $9C,$80,$3E,$69,$08,$05,$72,$1A,$50,$53,$B8,$03,$58,$8A,$1E,
  $68,$08,$30,$FF,$CD,$21,$B8,$01,$58,$8A,$1E,$67,$08,$30,$FF,
  $CD,$21,$5B,$58,$9D,$C3,$4C,$6F,$61,$64,$20,$65,$72,$72,$6F,
  $72,$3A,$20,$24,$3A,$20,$63,$61,$6E,$27,$74,$20,$6F,$70,$65,
  $6E,$24,$3A,$20,$6E,$6F,$74,$20,$45,$58,$45,$24,$3A,$20,$6E,
  $6F,$74,$20,$43,$4F,$46,$46,$20,$28,$43,$68,$65,$63,$6B,$20,
  $66,$6F,$72,$20,$76,$69,$72,$75,$73,$65,$73,$29,$24,$6E,$6F,
  $20,$44,$50,$4D,$49,$20,$2D,$20,$47,$65,$74,$20,$63,$73,$64,
  $70,$6D,$69,$2A,$62,$2E,$7A,$69,$70,$24,$6E,$6F,$20,$44,$4F,
  $53,$20,$6D,$65,$6D,$6F,$72,$79,$24,$6E,$65,$65,$64,$20,$44,
  $4F,$53,$20,$33,$24,$63,$61,$6E,$27,$74,$20,$73,$77,$69,$74,
  $63,$68,$20,$6D,$6F,$64,$65,$24,$6E,$6F,$20,$44,$50,$4D,$49,
  $20,$73,$65,$6C,$65,$63,$74,$6F,$72,$73,$24,$6E,$6F,$20,$44,
  $50,$4D,$49,$20,$6D,$65,$6D,$6F,$72,$79,$24,$90,$90,$90,$90,
  $90,$90,$90,$90,$90,$90,$90,$90,$90,$90,$90,$90,$90,$90,$90,
  $90,$90,$90,$90,$90,$90,$90,$90);

const win32stub : array[0..127] of byte=(
  $4D,$5A,$90,$00,$03,$00,$00,$00,$04,$00,$00,$00,$FF,$FF,$00,$00,
  $B8,$00,$00,$00,$00,$00,$00,$00,$40,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,
  $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$80,$00,$00,$00,
  $0E,$1F,$BA,$0E,$00,$B4,$09,$CD,$21,$B8,$01,$4C,$CD,$21,$54,$68,
  $69,$73,$20,$70,$72,$6F,$67,$72,$61,$6D,$20,$63,$61,$6E,$6E,$6F,
  $74,$20,$62,$65,$20,$72,$75,$6E,$20,$69,$6E,$20,$44,$4F,$53,$20,
  $6D,$6F,$64,$65,$2E,$0D,$0D,$0A,$24,$00,$00,$00,$00,$00,$00,$00);

const pemagic : array[0..3] of byte = (
  $50,$45,$00,$00);

{****************************************************************************
                                 Helpers
****************************************************************************}

    function djencodesechdrflags(aoptions:TObjSectionOptions):longword;
      begin
        if (oso_load in aoptions) then
          begin
            if oso_executable in aoptions then
              result:=COFF_STYP_TEXT
            else if not(oso_data in aoptions) then
              result:=COFF_STYP_BSS
            else
              result:=COFF_STYP_DATA;
          end
        else if oso_debug in aoptions then
          result:=COFF_STYP_INFO
        else
          result:=COFF_STYP_REG;
      end;


    function djdecodesechdrflags(const aname:string;flags:longword):TObjSectionOptions;
      begin
        result:=[];
        if flags and COFF_STYP_TEXT<>0 then
          result:=[oso_data,oso_load,oso_executable]
        else if flags and COFF_STYP_BSS<>0 then
          result:=[oso_load]
        else if flags and COFF_STYP_DATA<>0 then
          result:=[oso_data,oso_load]
        else if flags and COFF_STYP_INFO<>0 then
          result:=[oso_data,oso_debug]
        else
          result:=[oso_data]
      end;


    function peencodesechdrflags(aoptions:TObjSectionOptions;aalign:longint):longword;
      begin
        if oso_executable in aoptions then
          result:=PE_SCN_CNT_CODE or PE_SCN_MEM_EXECUTE
        else
          begin
            if (oso_data in aoptions) then
              result:=PE_SCN_CNT_INITIALIZED_DATA
            else
              result:=PE_SCN_CNT_UNINITIALIZED_DATA;
          end;
        if oso_write in aoptions then
          result:=result or PE_SCN_MEM_WRITE or PE_SCN_MEM_READ
        else
          result:=result or PE_SCN_MEM_READ;
        if not (oso_load in aoptions) then
          result:=result or PE_SCN_MEM_DISCARDABLE;
        case aalign of
           1 : result:=result or PE_SCN_ALIGN_1BYTES;
           2 : result:=result or PE_SCN_ALIGN_2BYTES;
           4 : result:=result or PE_SCN_ALIGN_4BYTES;
           8 : result:=result or PE_SCN_ALIGN_8BYTES;
          16 : result:=result or PE_SCN_ALIGN_16BYTES;
          32 : result:=result or PE_SCN_ALIGN_32BYTES;
          64 : result:=result or PE_SCN_ALIGN_64BYTES;
         128 : result:=result or PE_SCN_ALIGN_128BYTES;
         256 : result:=result or PE_SCN_ALIGN_256BYTES;
         512 : result:=result or PE_SCN_ALIGN_512BYTES;
        1024 : result:=result or PE_SCN_ALIGN_1024BYTES;
        2048 : result:=result or PE_SCN_ALIGN_2048BYTES;
        4096 : result:=result or PE_SCN_ALIGN_4096BYTES;
        8192 : result:=result or PE_SCN_ALIGN_8192BYTES;
          else result:=result or PE_SCN_ALIGN_16BYTES;
        end;
      end;


    procedure pedecodesechdrflags(const aname:string;flags:longword;out aoptions:TObjSectionOptions;out aalign:longint);
      var
        alignflag : longword;
      begin
        aoptions:=[];
        if flags and PE_SCN_CNT_CODE<>0 then
          include(aoptions,oso_executable);
        if flags and PE_SCN_MEM_DISCARDABLE<>0 then
          include(aoptions,oso_debug);
        if flags and PE_SCN_CNT_UNINITIALIZED_DATA=0 then
          include(aoptions,oso_data);
        if (flags and (PE_SCN_LNK_REMOVE or PE_SCN_MEM_DISCARDABLE)=0) then
          include(aoptions,oso_load);
        if flags and PE_SCN_LNK_COMDAT<>0 then
          include(aoptions,oso_comdat);
        { read/write }
        if flags and PE_SCN_MEM_WRITE<>0 then
          include(aoptions,oso_write);
        { alignment }
        alignflag:=flags and PE_SCN_ALIGN_MASK;
        if alignflag=PE_SCN_ALIGN_8192BYTES then
          aalign:=8192
        else if alignflag=PE_SCN_ALIGN_4096BYTES then
          aalign:=4096
        else if alignflag=PE_SCN_ALIGN_2048BYTES then
          aalign:=2048
        else if alignflag=PE_SCN_ALIGN_1024BYTES then
          aalign:=1024
        else if alignflag=PE_SCN_ALIGN_512BYTES then
          aalign:=512
        else if alignflag=PE_SCN_ALIGN_256BYTES then
          aalign:=256
        else if alignflag=PE_SCN_ALIGN_128BYTES then
          aalign:=128
        else if alignflag=PE_SCN_ALIGN_64BYTES then
          aalign:=64
        else if alignflag=PE_SCN_ALIGN_32BYTES then
          aalign:=32
        else if alignflag=PE_SCN_ALIGN_16BYTES then
          aalign:=16
        else if alignflag=PE_SCN_ALIGN_8BYTES then
          aalign:=8
        else if alignflag=PE_SCN_ALIGN_4BYTES then
          aalign:=4
        else if alignflag=PE_SCN_ALIGN_2BYTES then
          aalign:=2
        else if alignflag=PE_SCN_ALIGN_1BYTES then
          aalign:=1
        else if alignflag=0 then
          aalign:=0
        else
          Internalerror(2009050401);
      end;


{****************************************************************************
                               TCoffObjSection
****************************************************************************}

    constructor TCoffObjSection.create(AList:TFPHashObjectList;const aname:string;aalign:longint;aoptions:TObjSectionOptions);
      begin
        inherited create(AList,aname,aalign,aoptions);
      end;


    procedure TCoffObjSection.writereloc_internal(aTarget:TObjSection;offset:aword;len:byte;reltype:TObjRelocationType);
      begin
        AddSectionReloc(size,aTarget,reltype);
        write(offset,len);
      end;

{ We don't want overflow nor range checks here,
  wrapping is accepted in the address computation below }
{$r-}
{$q-}

    procedure TCoffExeOutput.DoRelocationFixup(objsec:TObjSection);
      var
        i,zero,address_size : longint;
        objreloc : TObjRelocation;
        address,
        relocval : aint;
{$ifdef arm}
        addend   : aint;
{$endif arm}
        relocsec : TObjSection;
{$ifdef cpu64bitaddr}
        s        : string;
{$endif cpu64bitaddr}
        data     : TDynamicArray;
      begin
        data:=objsec.data;
        for i:=0 to objsec.ObjRelocations.Count-1 do
          begin
            objreloc:=TObjRelocation(objsec.ObjRelocations[i]);
            address_size:=4;
            case objreloc.typ of
              RELOC_NONE:
                continue;
              RELOC_ZERO:
                begin
                  data.Seek(objreloc.dataoffset);
                  zero:=0;
                  data.Write(zero,4);
                  continue;
                end;
{$ifdef cpu64bitaddr}
              RELOC_ABSOLUTE:
                address_size:=8;
{$endif cpu64bitaddr}
              else
                ;
            end;

            address:=0;
            data.Seek(objreloc.dataoffset);
            data.Read(address,address_size);
            if assigned(objreloc.symbol) then
              begin
                relocsec:=objreloc.symbol.objsection;
                relocval:=objreloc.symbol.address;
              end
            else
              if assigned(objreloc.objsection) then
                begin
                  relocsec:=objreloc.objsection;
                  relocval:=objreloc.objsection.mempos
                end
            else
              internalerror(200205183);
            { Only debug sections are allowed to have relocs pointing to unused sections }
            if not relocsec.used and not (oso_debug in objsec.secoptions) then
              internalerror(200603061);

            if relocsec.used then
              case objreloc.typ of
                RELOC_RELATIVE  :
                  begin
                    address:=address-objsec.mempos+relocval;
                    if win32 then
                      dec(address,objreloc.dataoffset+4);
                  end;
                RELOC_RVA:
                  begin
                    { fixup address when the symbol was known in defined object }
                    if (relocsec.objdata=objsec.objdata) then
                      dec(address,TCoffObjSection(relocsec).orgmempos);
{$ifdef arm}
                    if (relocsec.objdata=objsec.objdata) and not TCoffObjData(objsec.objdata).eVCobj then
                      inc(address, relocsec.MemPos)
                    else
{$endif arm}
                      inc(address,relocval);
                  end;
                RELOC_SECREL32 :
                  begin
                    { fixup address when the symbol was known in defined object }
                    if (relocsec.objdata=objsec.objdata) then
                      dec(address,relocsec.ExeSection.MemPos);
                    inc(address,relocval);
                  end;
{$ifdef arm}
                RELOC_RELATIVE_24,
                RELOC_RELATIVE_CALL:
                  begin
                    addend:=sarlongint(((address and $ffffff) shl 8),6); // Sign-extend while shifting left twice
                    relocval:=longint(relocval - objsec.mempos - objreloc.dataoffset + addend) shr 2;
                    address:=(address and $ff000000) or (relocval and $ffffff);
                    relocval:=relocval shr 24;
                    if (relocval<>$3f) and (relocval<>0) then
                      internalerror(200606085);  { offset overflow }
                  end;
                RELOC_RELATIVE_24_THUMB,
                RELOC_RELATIVE_CALL_THUMB:
                  begin
                    addend:=sarlongint(((address and $ffffff) shl 8),6); // Sign-extend while shifting left twice, the assembler never sets the H bit
                    relocval:=longint(relocval - objsec.mempos - objreloc.dataoffset + addend) shr 1;
                    address:=(address and $ff000000) or ((relocval shr 1) and $ffffff) or ((relocval and 1) shl 24);
                    relocval:=relocval shr 25;
                    if (relocval<>$3f) and (relocval<>0) then
                      internalerror(200606085);  { offset overflow }
                  end;
{$endif arm}
{$ifdef x86_64}
                { 64 bit coff only }
                RELOC_RELATIVE_1:
                  begin
                    address:=address-objsec.mempos+relocval;
                    dec(address,objreloc.dataoffset+5);
                  end;
                RELOC_RELATIVE_2:
                  begin
                    address:=address-objsec.mempos+relocval;
                    dec(address,objreloc.dataoffset+6);
                  end;
                RELOC_RELATIVE_3:
                  begin
                    address:=address-objsec.mempos+relocval;
                    dec(address,objreloc.dataoffset+7);
                  end;
                RELOC_RELATIVE_4:
                  begin
                    address:=address-objsec.mempos+relocval;
                    dec(address,objreloc.dataoffset+8);
                  end;
                RELOC_RELATIVE_5:
                  begin
                    address:=address-objsec.mempos+relocval;
                    dec(address,objreloc.dataoffset+9);
                  end;
                RELOC_ABSOLUTE32,
{$endif x86_64}
                RELOC_ABSOLUTE :
                  begin
                    if (not win32) and assigned(objreloc.symbol) and
                      (objreloc.symbol.bind=AB_COMMON) then
                      begin
                        dec(address,objreloc.symbol.size);
                      end
                    else
                      begin
                        { fixup address when the symbol was known in defined object }
                        if (relocsec.objdata=objsec.objdata) then
                          dec(address,TCoffObjSection(relocsec).orgmempos);
                      end;
{$ifdef arm}
                    if (relocsec.objdata=objsec.objdata) and not TCoffObjData(objsec.objdata).eVCobj then
                      inc(address, relocsec.MemPos)
                    else
{$endif arm}
                      inc(address,relocval);
                    inc(address,imagebase);
                  end;
                else
                  internalerror(200604014);
              end
            else
              address:=0;  { Relocation in debug section points to unused section, which is eliminated by linker }

            data.Seek(objreloc.dataoffset);
            data.Write(address,address_size);
{$ifdef cpu64bitaddr}
            if (objreloc.typ = RELOC_ABSOLUTE32) and (objsec.name <> '.stab') then
              begin
                if assigned(objreloc.symbol) then
                  s:=objreloc.symbol.Name
                else
                  s:=objreloc.objsection.Name;
                Message2(link_w_32bit_absolute_reloc, objsec.ObjData.Name, s);
              end;
{$endif cpu64bitaddr}
          end;
      end;



{****************************************************************************
                                TCoffObjData
****************************************************************************}

    constructor TCoffObjData.createcoff(const n:string;awin32:boolean;acObjSection:TObjSectionClass);
      begin
        inherited create(n);
        CObjSection:=ACObjSection;
        win32:=awin32;
      end;


    function TCoffObjData.sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;
      var
        sep     : string[3];
        secname : string;
      begin
        { section type user gives the user full controll on the section name }
        if atype=sec_user then
          result:=aname
        else
          begin
            { non-PECOFF targets lack rodata support }
            if (atype in [sec_rodata,sec_rodata_norel]) and
               not (target_info.system in systems_all_windows) then
              atype:=sec_data;
            secname:=coffsecnames[atype];
            if create_smartlink_sections and
               (aname<>'') then
              begin
                case aorder of
                  secorder_begin :
                    sep:='.b_';
                  secorder_end :
                    sep:='.z_';
                  else
                    sep:='.n_';
                end;
                result:=secname+sep+aname
              end
            else
              result:=secname;
          end;
      end;


    function TCoffObjData.sectiontype2options(aType:TAsmSectionType): TObjSectionOptions;
      begin
        if (aType in [sec_rodata,sec_rodata_norel]) then
          begin
            if (target_info.system in systems_all_windows) then
              aType:=sec_rodata_norel
            else
              aType:=sec_data;
          end;
        result:=inherited sectiontype2options(aType);
      end;


    procedure TCoffObjData.CreateDebugSections;
      begin
        if target_dbg.id=dbg_stabs then
          begin
            stabssec:=createsection(sec_stab);
            stabstrsec:=createsection(sec_stabstr);
          end;
      end;


    procedure TCoffObjData.writereloc(data:aint;len:aword;p:TObjSymbol;reloctype:TObjRelocationType);
      var
        curraddr,
        symaddr : aword;
      begin
        if CurrObjSec=nil then
          internalerror(200403072);
        if assigned(p) then
          begin
            { current address }
            curraddr:=CurrObjSec.mempos+CurrObjSec.Size;
            { external/common symbols don't have a fixed memory position yet }
            if (p.bind=AB_COMMON) then
              begin
                { For go32v2 we need to use the size as address }
                if not win32 then
                  symaddr:=p.size
                else
                  symaddr:=0;
              end
            else
              symaddr:=p.address;
            { no symbol relocation need inside a section }
            if (p.objsection=CurrObjSec) and
               (p.bind<>AB_COMMON) then
              begin
                case reloctype of
                  RELOC_ABSOLUTE :
                    begin
                      CurrObjSec.addsectionreloc(curraddr,CurrObjSec,RELOC_ABSOLUTE);
                      inc(data,symaddr);
                    end;
{$ifdef cpu64bitaddr}
                  RELOC_ABSOLUTE32 :
                    begin
                      CurrObjSec.addsectionreloc(curraddr,CurrObjSec,RELOC_ABSOLUTE32);
                      inc(data,symaddr);
                    end;
{$endif cpu64bitaddr}
                  RELOC_RELATIVE :
                    begin
                      //inc(data,symaddr-len-CurrObjSec.Size);
                      data:=data+symaddr-len-CurrObjSec.Size;
                    end;
{$ifdef ARM}
                  RELOC_RELATIVE_24,
                  RELOC_RELATIVE_CALL:
                    begin
                      data:=(data and $ff000000) or (((((data and $ffffff) shl 2)+(symaddr-CurrObjSec.Size)) shr 2) and $FFFFFF); // TODO: Check overflow
                    end;
{$endif ARM}
                  RELOC_RVA,
                  RELOC_SECREL32 :
                    begin
                      CurrObjSec.addsectionreloc(curraddr,CurrObjSec,reloctype);
                      inc(data,symaddr);
                    end;
                  else
                    internalerror(200604013);
                end;
              end
            else
              begin
                if (p.objsection<>nil) and
                   (p.bind<>AB_COMMON) then
                  CurrObjSec.addsectionreloc(curraddr,p.objsection,reloctype)
                else
                  CurrObjSec.addsymreloc(curraddr,p,reloctype);
                if (not win32) or
                   (p.objsection<>nil) then
                  inc(data,symaddr);
                if reloctype=RELOC_RELATIVE then
                  begin
                    if win32 then
                      dec(data,len-4)
                    else
                      dec(data,len+CurrObjSec.Size);
                  end;
              end;
          end
        else
          begin
            if reloctype=RELOC_RVA then
              internalerror(200603033);
          end;
        CurrObjSec.write(data,len);
      end;


{****************************************************************************
                                TDJCoffObjData
****************************************************************************}

    constructor TDJCoffObjData.create(const n:string);
      begin
        inherited createcoff(n,false,TCoffObjSection);
      end;


{****************************************************************************
                                TPECoffObjData
****************************************************************************}

    constructor TPECoffObjData.create(const n:string);
      begin
        inherited createcoff(n,true,TCoffObjSection);
      end;


{****************************************************************************
                                TCoffObjOutput
****************************************************************************}

    constructor TCoffObjOutput.createcoff(AWriter:TObjectWriter;awin32:boolean);
      begin
        inherited create(AWriter);
        win32:=awin32;
      end;


    destructor TCoffObjOutput.destroy;
      begin
        FCoffSyms.free;
        FCoffStrs.free;
        inherited destroy;
      end;


    procedure TCoffObjOutput.write_symbol(const name:string;value:aword;section:longint;typ,aux:byte);
      var
        sym : coffsymbol;
        bosym : coffbigobjsymbol;
        strpos : longword;
      begin
        { symbolname }
        if length(name)>8 then
          begin
            strpos:=FCoffStrs.size+4;
            FCoffStrs.writestr(name);
            FCoffStrs.writestr(#0);
          end
        else
          strpos:=0;

        if bigobj then
          begin
            fillchar(bosym,sizeof(bosym),0);
            if length(name)>8 then
              bosym.name.offset.offset:=strpos
            else
              move(name[1],bosym.name.shortname,length(name));
            bosym.value:=value;
            bosym.SectionNumber:=longword(section);
            bosym.StorageClass:=typ;
            bosym.NumberOfAuxSymbols:=aux;
            inc(symidx);
            FCoffSyms.write(bosym,sizeof(bosym));
          end
        else
          begin
            if section>$7fff then
              internalerror(2017020302);
            FillChar(sym,sizeof(sym),0);
            if length(name)>8 then
              sym.strpos:=strpos
            else
              move(name[1],sym.name,length(name));
            sym.value:=value;
            sym.section:=section;
            sym.typ:=typ;
            sym.aux:=aux;
            inc(symidx);
            FCoffSyms.write(sym,sizeof(sym));
          end;
      end;


    procedure TCoffObjOutput.section_write_symbol(p:TObject;arg:pointer);
      var
        secrec : coffsectionrec;
        padding : word;
      begin
        with TCoffObjSection(p) do
          begin
            Inc(plongword(arg)^);
            index:=plongword(arg)^;

            secsymidx:=symidx;
            { Both GNU and Microsoft toolchains write section symbols using
              storage class 3 (STATIC).
              No reason to use COFF_SYM_SECTION, it is silently converted to 3 by
              PE binutils and causes warnings with DJGPP binutils. }
            write_symbol(name,mempos,index,COFF_SYM_LOCAL,1);
            { AUX }
            fillchar(secrec,sizeof(secrec),0);
            secrec.len:=Size;
            if ObjRelocations.count<65535 then
              secrec.nrelocs:=ObjRelocations.count
            else
              secrec.nrelocs:=65535;
            inc(symidx);
            FCoffSyms.write(secrec,sizeof(secrec));
            { aux recs have the same size as symbols, so we need to add two
              Byte of padding in case of a Big Obj Coff }
            padding:=0;
            if bigobj then
              FCoffSyms.write(padding,sizeof(padding));
          end;
      end;


    procedure TCoffObjOutput.section_write_relocs(p:TObject;arg:pointer);
      var
        i    : longint;
        rel  : coffreloc;
        objreloc : TObjRelocation;
      begin
        if (TObjSection(p).ObjRelocations.Count>65535) then
          begin
            rel.address:=TObjSection(p).ObjRelocations.Count+1;
            rel.sym:=0;
            rel.reloctype:=0;
            FWriter.Write(rel,sizeof(rel));
          end;
        for i:=0 to TObjSection(p).ObjRelocations.Count-1 do
          begin
            objreloc:=TObjRelocation(TObjSection(p).ObjRelocations[i]);
            rel.address:=objreloc.dataoffset;
            if assigned(objreloc.symbol) then
              begin
                if (objreloc.symbol.bind=AB_LOCAL) then
                  rel.sym:=objreloc.symbol.objsection.secsymidx
                else
                  begin
                    if objreloc.symbol.symidx=-1 then
                      internalerror(200602233);
                    rel.sym:=objreloc.symbol.symidx;
                  end;
              end
            else
              begin
                if objreloc.objsection<>nil then
                  rel.sym:=objreloc.objsection.secsymidx
                else
                  rel.sym:=0;
              end;
            case objreloc.typ of
{$ifdef arm}
              RELOC_ABSOLUTE :
                rel.reloctype:=IMAGE_REL_ARM_ADDR32;

              { I've no idea if this is correct (FK):
              RELOC_RELATIVE :
                rel.reloctype:=IMAGE_REL_ARM_GPREL12;
              }

              RELOC_RVA :
                rel.reloctype:=IMAGE_REL_ARM_ADDR32NB;
              RELOC_SECREL32 :
                rel.reloctype:=IMAGE_REL_ARM_SECREL;
              RELOC_RELATIVE_24 :
                rel.reloctype:=IMAGE_REL_ARM_BRANCH24;
              RELOC_RELATIVE_CALL :
                rel.reloctype:=IMAGE_REL_ARM_BLX24;
              RELOC_RELATIVE_24_THUMB:
                rel.reloctype:=IMAGE_REL_ARM_BLX23T;
{$endif arm}
{$ifdef i386}
              RELOC_RELATIVE :
                rel.reloctype:=IMAGE_REL_I386_PCRLONG;
              RELOC_ABSOLUTE :
                rel.reloctype:=IMAGE_REL_I386_DIR32;
              RELOC_RVA :
                rel.reloctype:=IMAGE_REL_I386_IMAGEBASE;
              RELOC_SECREL32 :
                rel.reloctype:=IMAGE_REL_I386_SECREL32;
{$endif i386}
{$ifdef x86_64}
              RELOC_NONE :
                rel.reloctype:=IMAGE_REL_AMD64_ABSOLUTE;
              RELOC_RELATIVE :
                rel.reloctype:=IMAGE_REL_AMD64_REL32;
              RELOC_ABSOLUTE32 :
                rel.reloctype:=IMAGE_REL_AMD64_ADDR32;
              RELOC_ABSOLUTE :
                rel.reloctype:=IMAGE_REL_AMD64_ADDR64;
              RELOC_RVA :
                rel.reloctype:=IMAGE_REL_AMD64_ADDR32NB;
              RELOC_RELATIVE_1 :
                rel.reloctype:=IMAGE_REL_AMD64_REL32_1;
              RELOC_RELATIVE_2 :
                rel.reloctype:=IMAGE_REL_AMD64_REL32_2;
              RELOC_RELATIVE_3 :
                rel.reloctype:=IMAGE_REL_AMD64_REL32_3;
              RELOC_RELATIVE_4 :
                rel.reloctype:=IMAGE_REL_AMD64_REL32_4;
              RELOC_RELATIVE_5 :
                rel.reloctype:=IMAGE_REL_AMD64_REL32_5;
              RELOC_SECREL32 :
                rel.reloctype:=IMAGE_REL_AMD64_SECREL;
{$endif x86_64}
              else
                internalerror(200905071);
            end;
            FWriter.write(rel,sizeof(rel));
          end;
      end;


    procedure TCoffObjOutput.create_symbols(data:TObjData);
      var
        filename   : string[20];
        filenamelen : longint;
        sectionval : word;
        globalval  : byte;
        i          : longint;
        value      : aword;
        objsym     : TObjSymbol;
        secidx     : longword;
      begin
        with TCoffObjData(data) do
         begin
           symidx:=0;
           { The `.file' record, and the file name auxiliary record }
           write_symbol('.file', 0, -2, COFF_SYM_FILE, 1);
           fillchar(filename,sizeof(filename),0);
           filename:=ExtractFileName(current_module.mainsource);
           inc(symidx);
           if bigobj then
             filenamelen:=sizeof(coffbigobjsymbol)
           else
             filenamelen:=sizeof(coffsymbol);
           FCoffSyms.write(filename[1],filenamelen);
           { Sections }
           secidx:=0;
           ObjSectionList.ForEachCall(@section_write_symbol,@secidx);
           { ObjSymbols }
           for i:=0 to ObjSymbolList.Count-1 do
             begin
               objsym:=TObjSymbol(ObjSymbolList[i]);
               if (objsym.bind=AB_LOCAL) then
                 continue;
               case objsym.bind of
                 AB_GLOBAL,
                 AB_PRIVATE_EXTERN:
                   begin
                     globalval:=COFF_SYM_GLOBAL;
                     sectionval:=objsym.objsection.index;
                     value:=objsym.address;
                   end;
                 AB_LOCAL :
                   begin
                     globalval:=COFF_SYM_LOCAL;
                     sectionval:=objsym.objsection.index;
                     value:=objsym.address;
                   end;
                 else
                   begin
                     globalval:=COFF_SYM_GLOBAL;
                     sectionval:=0;
                     value:=objsym.size;
                   end;
               end;
               { symbolname }
               objsym.symidx:=symidx;
               write_symbol(objsym.name,value,sectionval,globalval,0);
             end;
         end;
      end;


    procedure TCoffObjOutput.section_set_reloc_datapos(p:TCoffObjSection;var datapos:aword);
      begin
        p.coffrelocpos:=datapos;
        inc(datapos,sizeof(coffreloc)*p.ObjRelocations.count);
        if p.ObjRelocations.count>65535 then
          begin
            if win32 then
              inc(datapos,sizeof(coffreloc))
            else
              Message1(asmw_f_too_many_relocations,p.fullname);
          end;
      end;


    procedure TCoffObjOutput.section_write_header(p:TObject;arg:pointer);
      var
        sechdr   : tcoffsechdr;
        s        : string;
        strpos   : aword;
      begin
        with TCoffObjSection(p) do
          begin
            fillchar(sechdr,sizeof(sechdr),0);
            s:=name;
            if length(s)>8 then
             begin
               strpos:=FCoffStrs.size+4;
               FCoffStrs.writestr(s);
               FCoffStrs.writestr(#0);
               s:='/'+ToStr(strpos);
             end;
            move(s[1],sechdr.name,length(s));
            if not win32 then
              begin
                sechdr.rvaofs:=mempos;
                sechdr.vsize:=mempos;
              end
            else
              begin
                if not(oso_data in secoptions) then
                  sechdr.vsize:=Size;
              end;
            sechdr.DataSize:=size;
            if (Size>0) and
               (oso_data in secoptions) then
              sechdr.datapos:=datapos;
            if ObjRelocations.count<65535 then
              sechdr.nrelocs:=ObjRelocations.count
            else
              sechdr.nrelocs:=65535;
            sechdr.relocpos:=coffrelocpos;
            if win32 then
              begin
                sechdr.flags:=peencodesechdrflags(secoptions,secalign);
                if ObjRelocations.count>65535 then
                  sechdr.flags:=sechdr.flags or PE_SCN_LNK_NRELOC_OVFL;
              end
            else
              sechdr.flags:=djencodesechdrflags(secoptions);
            FWriter.write(sechdr,sizeof(sechdr));
          end;
      end;


    function TCoffObjOutput.writedata(data:TObjData):boolean;
      var
        datapos,
        sympos   : aword;
        i        : longint;
        header   : tcoffheader;
        boheader : tcoffbigobjheader;
      begin
        result:=false;
        FCoffSyms:=TDynamicArray.Create(SymbolMaxGrow);
        FCoffStrs:=TDynamicArray.Create(StrsMaxGrow);
        with TCoffObjData(data) do
         begin
           bigobj:=(ObjSectionList.Count>$7fff) and win32;

           { Create Symbol Table }
           create_symbols(data);

           { Calculate the filepositions }
           if bigobj then
             datapos:=sizeof(tcoffbigobjheader)
           else
             datapos:=sizeof(tcoffheader);
           datapos:=datapos+sizeof(tcoffsechdr)*ObjSectionList.Count;
           { Sections first }
           layoutsections(datapos);
           { relocs }
           for i:=0 to ObjSectionList.Count-1 do
             section_set_reloc_datapos(TCoffObjSection(ObjSectionList[i]),datapos);
           { Symbols }
           sympos:=datapos;

           { Generate COFF header }
           if bigobj then
             begin
               fillchar(boheader,sizeof(boheader),0);
               boheader.Sig1:=0;
               boheader.Sig2:=$ffff;
               boheader.Machine:=COFF_MAGIC;
               boheader.Version:=COFF_BIG_OBJ_VERSION;
               boheader.NumberOfSections:=longword(ObjSectionList.Count);
               boheader.NumberOfSymbols:=longword(symidx);
               boheader.PointerToSymbolTable:=sympos;
               Move(COFF_BIG_OBJ_MAGIC,boheader.UUID,length(boheader.UUID));
               FWriter.write(boheader,sizeof(boheader));
             end
           else
             begin
               fillchar(header,sizeof(tcoffheader),0);
               header.mach:=COFF_MAGIC;
               header.nsects:=ObjSectionList.Count;
               header.sympos:=sympos;
               header.syms:=symidx;
               if win32 then
                 begin
{$ifndef x86_64}
                   header.flag:=PE_FILE_32BIT_MACHINE or
                                PE_FILE_LINE_NUMS_STRIPPED or PE_FILE_LOCAL_SYMS_STRIPPED;
{$else x86_64}
                   header.flag:=PE_FILE_LINE_NUMS_STRIPPED or PE_FILE_LOCAL_SYMS_STRIPPED;
{$endif x86_64}
                 end
               else
                 header.flag:=COFF_FLAG_AR32WR or COFF_FLAG_NOLINES or COFF_FLAG_NOLSYMS;
               FWriter.write(header,sizeof(header));
             end;
           { Section headers }
           ObjSectionList.ForEachCall(@section_write_header,nil);
           { ObjSections }
           WriteSectionContent(data);
           { Relocs }
           ObjSectionList.ForEachCall(@section_write_relocs,nil);
           { ObjSymbols }
           if Sympos<>FWriter.ObjSize then
             internalerror(200603051);
           FWriter.writearray(FCoffSyms);
           { Strings }
           i:=FCoffStrs.size+4;
           FWriter.write(i,4);
           FWriter.writearray(FCoffStrs);
         end;
        FCoffStrs.Free;
        FCoffStrs:=nil;
        FCoffSyms.Free;
        FCoffSyms:=nil;
      end;


    constructor TDJCoffObjOutput.create(AWriter:TObjectWriter);
      begin
        inherited createcoff(AWriter,false);
        cobjdata:=TDJCoffObjData;
      end;


    constructor TPECoffObjOutput.create(AWriter:TObjectWriter);
      begin
        inherited createcoff(AWriter,true);
        cobjdata:=TPECoffObjData;
      end;


{****************************************************************************
                                TCoffObjInput
****************************************************************************}

    constructor TCoffObjInput.createcoff(awin32:boolean);
      begin
        inherited create;
        win32:=awin32;
        bigobj:=false;
        FSymTbl:=nil;
      end;


    destructor TCoffObjInput.destroy;
      begin
        FCoffSyms.free;
        if assigned(FCoffStrs) then
          freemem(FCoffStrs);
        if assigned(FSymTbl) then
          freemem(FSymTbl);
        if assigned(FSecTbl) then
          freemem(FSecTbl);
        inherited destroy;
      end;


    function TCoffObjInput.GetSection(secidx:longint):TObjSection;
      begin
        result:=nil;
        if (secidx<1) or (secidx>FSecCount) then
          begin
            InputError('Failed reading coff file, invalid section index');
            exit;
          end;
        result:=FSecTbl^[secidx];
      end;


    function TCoffObjInput.Read_str(strpos:longword):string;
      begin
        if (FCoffStrs=nil) or (strpos>=FCoffStrSize) or (FCoffStrs[strpos]=#0) then
          Internalerror(200205172);
        result:=string(PChar(@FCoffStrs[strpos]));
      end;


    procedure TCoffObjInput.read_relocs(s:TCoffObjSection);
      var
        rel       : coffreloc;
        rel_type  : TObjRelocationType;
        i         : longint;
        p         : TObjSymbol;
      begin
        if s.coffrelocs=high(aword) then
          begin
            { If number of relocations exceeds 65535, it is stored in address field
              of the first record, and includes this first fake relocation. }
            FReader.read(rel,sizeof(rel));
            s.coffrelocs:=rel.address-1;
            if s.coffrelocs<=65535 then
              InternalError(2013012503);
          end;
        for i:=1 to s.coffrelocs do
         begin
           FReader.read(rel,sizeof(rel));
           case rel.reloctype of
{$ifdef arm}
             IMAGE_REL_ARM_ABSOLUTE:
               rel_type:=RELOC_NONE;
             IMAGE_REL_ARM_ADDR32:
               rel_type:=RELOC_ABSOLUTE;
             IMAGE_REL_ARM_ADDR32NB:
               rel_type:=RELOC_RVA;
             IMAGE_REL_ARM_BRANCH24:
               rel_type:=RELOC_RELATIVE_24;
             IMAGE_REL_ARM_BLX24:
               rel_type:=RELOC_RELATIVE_CALL;
             IMAGE_REL_ARM_SECREL:
               rel_type:=RELOC_SECREL32;
             IMAGE_REL_ARM_BLX23T:
               rel_type:=RELOC_RELATIVE_24_THUMB;
{$endif arm}
{$ifdef i386}
             IMAGE_REL_I386_PCRLONG :
               rel_type:=RELOC_RELATIVE;
             IMAGE_REL_I386_DIR32 :
               rel_type:=RELOC_ABSOLUTE;
             IMAGE_REL_I386_IMAGEBASE :
               rel_type:=RELOC_RVA;
             IMAGE_REL_I386_SECREL32 :
               rel_type:=RELOC_SECREL32;
{$endif i386}
{$ifdef x86_64}
             IMAGE_REL_AMD64_ABSOLUTE:
               rel_type:=RELOC_NONE;
             IMAGE_REL_AMD64_REL32:
               rel_type:=RELOC_RELATIVE;
             IMAGE_REL_AMD64_ADDR32,
             R_X86_64_32S:
               rel_type:=RELOC_ABSOLUTE32;
             IMAGE_REL_AMD64_ADDR64:
               rel_type:=RELOC_ABSOLUTE;
             IMAGE_REL_AMD64_ADDR32NB:
               rel_type:=RELOC_RVA;
             IMAGE_REL_AMD64_REL32_1:
               rel_type:=RELOC_RELATIVE_1;
             IMAGE_REL_AMD64_REL32_2:
               rel_type:=RELOC_RELATIVE_2;
             IMAGE_REL_AMD64_REL32_3:
               rel_type:=RELOC_RELATIVE_3;
             IMAGE_REL_AMD64_REL32_4:
               rel_type:=RELOC_RELATIVE_4;
             IMAGE_REL_AMD64_REL32_5:
               rel_type:=RELOC_RELATIVE_5;
             IMAGE_REL_AMD64_SECREL:
               rel_type:=RELOC_SECREL32;
{$endif x86_64}
           else
             begin
               InputError('Failed reading coff file, illegal reloctype $'+system.hexstr(rel.reloctype,4));
               exit;
             end;
           end;

           p:=FSymTbl^[rel.sym];
           if assigned(p) then
             s.addsymreloc(rel.address-s.mempos,p,rel_type)
           else
            begin
              InputError('Failed reading coff file, can''t resolve symbol of relocation');
              exit;
            end;
         end;
      end;


    procedure TCoffObjInput.read_symbols(objdata:TObjData);
      var
        size,
        address,
        nsyms,
        symidx    : aint;
        i         : longint;
        sym       : coffsymbol;
        bosym     : coffbigobjsymbol;
        objsym    : TObjSymbol;
        bind      : Tasmsymbind;
        strname   : string;
        auxrec    : array[0..sizeof(coffsymbol)-1] of byte;
        boauxrec  : array[0..sizeof(coffbigobjsymbol)-1] of byte;
        secrec    : pcoffsectionrec;
        objsec    : TObjSection;
        secidx    : longint;
        symvalue  : longword;
        auxcount  : byte;
        symcls    : byte;
        comdatsel : TObjSectionComdatSelection;

        { keeps string manipulations out of main routine }
        procedure UnsupportedSymbolType;
          begin
            Comment(V_Fatal,'Unsupported COFF symbol type '+tostr(symcls)+' at index '+tostr(symidx)+' while reading '+InputFileName);
          end;

      begin
        with TCoffObjData(objdata) do
         begin
           if bigobj then
             nsyms:=FCoffSyms.Size div sizeof(coffbigobjsymbol)
           else
             nsyms:=FCoffSyms.Size div sizeof(CoffSymbol);
           { Allocate memory for symidx -> TObjSymbol table }
           FSymTbl:=AllocMem(nsyms*sizeof(TObjSymbol));
           { Load the Symbols }
           FCoffSyms.Seek(0);
           symidx:=0;
           while (symidx<nsyms) do
            begin
              if bigobj then
                begin
                  FCoffSyms.Read(bosym,sizeof(bosym));
                  if bosym.Name.Offset.Zeroes<>0 then
                    begin
                      move(bosym.Name.ShortName,strname[1],8);
                      strname[9]:=#0;
                      strname[0]:=chr(strlen(@strname[1]));
                      if strname='' then
                        internalerror(2017020301);
                    end
                  else
                    strname:=Read_str(bosym.Name.Offset.Offset);
                  secidx:=longint(bosym.SectionNumber);
                  symvalue:=bosym.Value;
                  auxcount:=bosym.NumberOfAuxSymbols;
                  symcls:=bosym.StorageClass;
                end
              else
                begin
                  FCoffSyms.Read(sym,sizeof(sym));
                  if plongint(@sym.name)^<>0 then
                    begin
                      move(sym.name,strname[1],8);
                      strname[9]:=#0;
                      strname[0]:=chr(strlen(@strname[1]));
                      if strname='' then
                        Internalerror(200205171);
                    end
                  else
                    strname:=Read_str(sym.strpos);
                  secidx:=sym.section;
                  symvalue:=sym.value;
                  auxcount:=sym.aux;
                  symcls:=sym.typ;
                end;
              bind:=AB_EXTERNAL;
              size:=0;
              address:=0;
              objsym:=nil;
              objsec:=nil;
              case symcls of
                COFF_SYM_GLOBAL :
                  begin
                    if secidx=0 then
                     begin
                       if symvalue=0 then
                        bind:=AB_EXTERNAL
                       else
                        begin
                          bind:=AB_COMMON;
                          size:=symvalue;
                        end;
                     end
                    else
                     begin
                       bind:=AB_GLOBAL;
                       objsec:=GetSection(secidx);
                       if symvalue>=objsec.mempos then
                         address:=symvalue-objsec.mempos;
                     end;
                    objsym:=CreateSymbol(strname);
                    objsym.bind:=bind;
                    objsym.typ:=AT_FUNCTION;
                    objsym.objsection:=objsec;
                    objsym.offset:=address;
                    objsym.size:=size;
                  end;
                COFF_SYM_LABEL,
                COFF_SYM_LOCAL :
                  begin
                    { do not add constants (section=-1) }
                    if secidx<>-1 then
                     begin
                       objsec:=GetSection(secidx);
                       if symvalue>=objsec.mempos then
                         address:=symvalue-objsec.mempos;
                       objsym:=CreateSymbol(strname);
                       objsym.bind:=AB_LOCAL;
                       objsym.typ:=AT_FUNCTION;
                       objsym.objsection:=objsec;
                       objsym.offset:=address;
                       objsym.size:=size;
                     end;
                  end;
                COFF_SYM_SECTION :
                  begin
                    { GetSection checks that index is in range }
                    objsec:=GetSection(secidx);
                    if assigned(objsec) then
                      begin
                        if symvalue>=objsec.mempos then
                          address:=symvalue-objsec.mempos;
                        objsym:=CreateSymbol(strname);
                        objsym.bind:=AB_LOCAL;
                        objsym.typ:=AT_FUNCTION;
                        objsym.objsection:=objsec;
                        objsym.offset:=address;
                        objsym.size:=size;
                      end;
                  end;
                COFF_SYM_FUNCTION,
                COFF_SYM_FILE :
                  ;
                else
                  UnsupportedSymbolType;
              end;
              FSymTbl^[symidx]:=objsym;
              { read aux records }

              { handle COMDAT symbols }
              if (symcls=COFF_SYM_LOCAL) and (auxcount=1) and (symvalue=0) and (oso_comdat in objsym.objsection.SecOptions) then
                begin
                  if bigobj then
                    begin
                      FCoffSyms.Read(boauxrec,sizeof(boauxrec));
                      secrec:=pcoffsectionrec(@boauxrec[0]);
                    end
                  else
                    begin
                      FCoffSyms.Read(auxrec,sizeof(auxrec));
                      secrec:=pcoffsectionrec(@auxrec);
                    end;

                  case secrec^.select of
                    IMAGE_COMDAT_SELECT_NODUPLICATES:
                      comdatsel:=oscs_none;
                    IMAGE_COMDAT_SELECT_ANY:
                      comdatsel:=oscs_any;
                    IMAGE_COMDAT_SELECT_SAME_SIZE:
                      comdatsel:=oscs_same_size;
                    IMAGE_COMDAT_SELECT_EXACT_MATCH:
                      comdatsel:=oscs_exact_match;
                    IMAGE_COMDAT_SELECT_ASSOCIATIVE:
                      comdatsel:=oscs_associative;
                    IMAGE_COMDAT_SELECT_LARGEST:
                      comdatsel:=oscs_largest;
                    else begin
                      comdatsel:=oscs_none;
                      Message2(link_e_comdat_select_unsupported,inttostr(secrec^.select),objsym.objsection.name);
                    end;
                  end;

                  if comdatsel in [oscs_associative,oscs_exact_match] then
                    { only temporary }
                    Comment(V_Error,'Associative or exact match COMDAT sections are not yet supported (symbol: '+objsym.objsection.Name+')')
                  else if (comdatsel=oscs_associative) and (secrec^.assoc=0) then
                    Message1(link_e_comdat_associative_section_expected,objsym.objsection.name)
                  else if (objsym.objsection.ComdatSelection<>oscs_none) and (comdatsel<>oscs_none) and (objsym.objsection.ComdatSelection<>comdatsel) then
                    Message2(link_e_comdat_not_matching,objsym.objsection.Name,objsym.Name)
                  else
                    begin
                      objsym.objsection.ComdatSelection:=comdatsel;

                      if (secrec^.assoc<>0) and not assigned(objsym.objsection.AssociativeSection) then
                        begin
                          objsym.objsection.AssociativeSection:=GetSection(secrec^.assoc);
                          if not assigned(objsym.objsection.AssociativeSection) then
                            Message1(link_e_comdat_associative_section_not_found,objsym.objsection.Name);
                        end;
                    end;

                  dec(auxcount);
                  inc(symidx);
                end;

              for i:=1 to auxcount do
               begin
                 if bigobj then
                   FCoffSyms.Read(boauxrec,sizeof(boauxrec))
                 else
                   FCoffSyms.Read(auxrec,sizeof(auxrec));
                 inc(symidx);
               end;
              inc(symidx);
            end;
         end;
      end;


    procedure TCoffObjInput.ObjSections_read_relocs(p:TObject;arg:pointer);
      begin
        with TCoffObjSection(p) do
          begin
            { Skip debug sections }
            if (oso_debug in secoptions)  and
               (cs_link_strip in current_settings.globalswitches) and
               not(cs_link_separate_dbg_file in current_settings.globalswitches) then
              exit;

            if coffrelocs>0 then
              begin
                FReader.Seek(coffrelocpos);
                read_relocs(TCoffObjSection(p));
              end;
          end;
      end;


    function  TCoffObjInput.ReadObjData(AReader:TObjectreader;out objdata:TObjData):boolean;
      var
        secalign : longint;
        secofs,
        strpos,
        i        : longint;
        sympos,
        symcount,
        symsize,
        code     : longint;
        objsec   : TCoffObjSection;
        secoptions : TObjSectionOptions;
        header   : tcoffheader;
        boheader : tcoffbigobjheader;
        sechdr   : tcoffsechdr;
        secname  : string;
        secnamebuf : array[0..15] of char;
      begin
        FReader:=AReader;
        InputFileName:=AReader.FileName;
        objdata:=CObjData.Create(InputFileName);
        result:=false;
        boheader:=default(tcoffbigobjheader);
        FCoffSyms:=TDynamicArray.Create(SymbolMaxGrow);
        with TCoffObjData(objdata) do
         begin
           { Read COFF header }
           if not AReader.read(header,sizeof(tcoffheader)) then
             begin
               InputError('Can''t read COFF Header');
               exit;
             end;
           if (header.mach=0) and (header.nsects=$ffff) then
             begin
               { either a library or big obj COFF }
               AReader.seek(0);
               if not AReader.read(boheader,sizeof(boheader)) then
                 begin
                   InputError('Can''t read Big Obj COFF Header');
                   exit;
                 end;
               if CompareByte(boheader.UUID,COFF_BIG_OBJ_MAGIC,length(boheader.uuid))<>0 then
                 begin
                   { ToDo: this should be treated as a library }
                   InputError('Illegal Big Obj COFF Magic');
                   exit;
                 end;
               if boheader.Version<>COFF_BIG_OBJ_VERSION then
                 begin
                   InputError('Illegal Big Obj COFF Version');
                   exit;
                 end;
               if boheader.Machine<>COFF_MAGIC then
                 begin
                   InputError('Illegal COFF Machine type');
                   exit;
                 end;
               bigobj:=true;
             end
           else if header.mach<>COFF_MAGIC then
             begin
               InputError('Illegal COFF Magic');
               exit;
             end;
{$ifdef arm}
           eVCobj:=header.flag=$100;
{$endif arm}
           { ObjSymbols }
           if bigobj then
             begin
               sympos:=longint(boheader.PointerToSymbolTable);
               symcount:=longint(boheader.NumberOfSymbols);
               symsize:=sizeof(CoffBigObjSymbol);
             end
           else
             begin
               sympos:=longint(header.sympos);
               symcount:=longint(header.syms);
               symsize:=sizeof(CoffSymbol);
             end;
           AReader.Seek(sympos);
           if not AReader.ReadArray(FCoffSyms,symcount*symsize) then
             begin
               InputError('Error reading coff symbol table');
               exit;
           end;
           { Strings }
           if not AReader.Read(FCoffStrSize,4) then
             begin
               InputError('Error reading COFF string table');
               exit;
             end;
           if (FCoffStrSize>4) then
             begin
               { allocate an extra byte and null-terminate }
               GetMem(FCoffStrs,FCoffStrSize+1);
               FCoffStrs[FCoffStrSize]:=#0;
               for i:=0 to 3 do
                 FCoffStrs[i]:=#0;
               if not AReader.Read(FCoffStrs[4],FCoffStrSize-4) then
                 begin
                   InputError('Error reading COFF string table');
                   exit;
                 end;
             end;
           { Section headers }
           { Allocate SecIdx -> TObjSection table, secidx is 1-based }
           if bigobj then
             FSecCount:=longint(boheader.NumberOfSections)
           else
             FSecCount:=header.nsects;
           FSecTbl:=AllocMem((FSecCount+1)*sizeof(TObjSection));
           if bigobj then
             secofs:=sizeof(tcoffbigobjheader)
           else
             secofs:=sizeof(tcoffheader)+header.opthdr;
           AReader.Seek(secofs);
           for i:=1 to FSecCount do
             begin
               if not AReader.read(sechdr,sizeof(sechdr)) then
                begin
                  InputError('Error reading COFF Section Headers');
                  exit;
                end;
               move(sechdr.name,secnamebuf,8);
               secnamebuf[8]:=#0;
               secname:=strpas(secnamebuf);
               if secname[1]='/' then
                 begin
                   Val(Copy(secname,2,8),strpos,code);
                   if code=0 then
                     secname:=Read_str(strpos)
                   else
                     begin
                       InputError('Error reading COFF Section Headers');
                       secname:='error';
                     end;
                 end;
               if win32 then
                 pedecodesechdrflags(secname,sechdr.flags,secoptions,secalign)
               else
                 begin
                   secoptions:=djdecodesechdrflags(secname,sechdr.flags);
                   secalign:=sizeof(pint);
                 end;
               if (Length(secname)>3) and (secname[2] in ['e','f','i','p','r']) then
                 begin
                   if (Pos('.edata',secname)=1) or
                      (Pos('.rsrc',secname)=1) or
{$ifndef x86_64}
                      (Pos('.pdata',secname)=1) or
{$endif}
                      (Pos('.fpc',secname)=1) then
                     include(secoptions,oso_keep);
                   if (Pos('.idata',secname)=1) then
                     begin
  { TODO: idata keep can maybe replaced with grouping of text and idata}
                       include(secoptions,oso_keep);
                       secname:=secname + '.' + ExtractFileName(InputFileName);
                     end;
                 end;
               objsec:=TCoffObjSection(createsection(secname,secalign,secoptions,false));
               FSecTbl^[i]:=objsec;
               if not win32 then
                 objsec.mempos:=sechdr.rvaofs;
               objsec.orgmempos:=sechdr.rvaofs;
               objsec.coffrelocs:=sechdr.nrelocs;
               if win32 then
                 begin
                   if (sechdr.flags and PE_SCN_LNK_NRELOC_OVFL)<>0 then
                     objsec.coffrelocs:=high(aword);
                 end;
               objsec.coffrelocpos:=sechdr.relocpos;
               objsec.datapos:=sechdr.datapos;
               objsec.Size:=sechdr.dataSize;
             end;
           { Insert all ObjSymbols }
           read_symbols(objdata);
           { Section Data }
           ReadSectionContent(objdata);
           { Relocs }
           ObjSectionList.ForEachCall(@objsections_read_relocs,nil);
         end;
        if assigned(FCoffStrs) then
          freemem(FCoffStrs);
        FCoffStrs:=nil;
        FCoffSyms.Free;
        FCoffSyms:=nil;
        result:=true;
      end;


    constructor TDJCoffObjInput.create;
      begin
        inherited createcoff(false);
        cobjdata:=TDJCoffObjData;
      end;


    constructor TPECoffObjInput.create;
      begin
        inherited createcoff(true);
        cobjdata:=TPECoffObjData;
      end;


{****************************************************************************
                              TCoffexeoutput
****************************************************************************}

    constructor TCoffexeoutput.createcoff(awin32:boolean);
      begin
        inherited create;
        win32:=awin32;
        if target_info.system in [system_x86_64_win64] then
          MaxMemPos:=$FFFFFFFF
        else
          if target_info.system in systems_wince then
            MaxMemPos:=$1FFFFFF
          else
            MaxMemPos:=$7FFFFFFF;
      end;


    procedure TCoffexeoutput.write_symbol(const name:string;value:aword;section:smallint;typ,aux:byte);
      var
        sym : coffsymbol;
      begin
        FillChar(sym,sizeof(sym),0);
        if length(name)>8 then
          begin
             sym.strpos:=FCoffStrs.size+4;
             FCoffStrs.writestr(name);
             FCoffStrs.writestr(#0);
          end
        else
          move(name[1],sym.name,length(name));
        sym.value:=value;
        sym.section:=section;
        sym.typ:=typ;
        sym.aux:=aux;
        FWriter.write(sym,sizeof(sym));
      end;


    procedure TCoffexeoutput.globalsyms_write_symbol(p:TObject;arg:pointer);
      var
        secval,
        value  : aint;
        globalval : byte;
        exesec : TExeSection;
      begin
        if not assigned(texesymbol(p).objsymbol) then
          internalerror(200603053);
        with texesymbol(p).objsymbol do
          begin
            if assigned(objsection) then
              exesec:=TExeSection(objsection.exesection)
            else
              exesec:=nil;
            { There is no exesection defined for special internal symbols
              like __image_base__ }
            if assigned(exesec) then
              begin
                secval:=exesec.secsymidx;
                if win32 then
                  value:=address-exesec.mempos
                else
                  value:=address;
              end
            else
              begin
                secval:=-1;
                value:=address;
              end;
            if bind=AB_LOCAL then
              globalval:=COFF_SYM_LOCAL
            else
              globalval:=COFF_SYM_GLOBAL;
            { reloctype address to the section in the executable }
            write_symbol(name,value,secval,globalval,0);
          end;
      end;


    procedure TCoffexeoutput.ExeSectionList_write_header(p:TObject;arg:pointer);
      var
        sechdr   : tcoffsechdr;
        s        : string;
        strpos   : aword;
      begin
        with tExeSection(p) do
          begin
            fillchar(sechdr,sizeof(sechdr),0);
            s:=name;
            if length(s)>8 then
             begin
               strpos:=FCoffStrs.size+4;
               FCoffStrs.writestr(s);
               FCoffStrs.writestr(#0);
               s:='/'+ToStr(strpos);
             end;
            move(s[1],sechdr.name,length(s));
            if win32 then
              begin
                sechdr.rvaofs:=mempos;
                sechdr.vsize:=Size;
                { sechdr.dataSize is size of initialized data, rounded up to FileAlignment
                  (so it can be greater than VirtualSize). Must be zero for sections that
                  do not contain initialized data. }
                if (oso_data in SecOptions) then
                  sechdr.datasize:=Align(Size,SectionDataAlign);
              end
            else
              begin
                if not (oso_debug in SecOptions) then
                  begin
                    sechdr.rvaofs:=mempos;
                    sechdr.vsize:=mempos;
                  end;
                sechdr.datasize:=Size;
              end;
            if (Size>0) then
              sechdr.datapos:=datapos-datapos_offset;
            sechdr.nrelocs:=0;
            sechdr.relocpos:=0;
            if win32 then
              begin
                if (target_info.system in systems_nativent) and
                   (apptype = app_native) then
                  sechdr.flags:=peencodesechdrflags(SecOptions,SecAlign) or PE_SCN_MEM_NOT_PAGED
                else
                  sechdr.flags:=peencodesechdrflags(SecOptions,SecAlign);
                { some flags are invalid in executables, reset them }
                sechdr.flags:=sechdr.flags and
                  not(PE_SCN_LNK_INFO or PE_SCN_LNK_REMOVE or
                      PE_SCN_LNK_COMDAT or PE_SCN_ALIGN_MASK);
              end
            else
              sechdr.flags:=djencodesechdrflags(SecOptions);
            FWriter.write(sechdr,sizeof(sechdr));
          end;
      end;


    procedure TCoffexeoutput.ExeSectionList_pass2_header(p:TObject;arg:pointer);
      begin
        with TExeSection(p) do
          begin
            { The debuginfo sections should already be stripped }
{            if (ExeWriteMode=ewm_exeonly) and
               (oso_debug in SecOptions) then
              internalerror(200801161); }
            inc(plongint(arg)^);
            secsymidx:=plongint(arg)^;
          end;
      end;


    function tcoffexeoutput.totalheadersize:longword;
      var
        stubsize,
        optheadersize : longword;
      begin
        if win32 then
          begin
            stubsize:=sizeof(win32stub)+sizeof(pemagic);
            optheadersize:=sizeof(tcoffpeoptheader);
          end
        else
          begin
            stubsize:=sizeof(go32v2stub);
            optheadersize:=sizeof(coffdjoptheader);
          end;
        result:=stubsize+sizeof(tcoffheader)+optheadersize;
      end;


    procedure tcoffexeoutput.MemPos_Header;
      begin
        { calculate start positions after the headers }
        currmempos:=totalheadersize+sizeof(tcoffsechdr)*longword(ExeSectionList.Count-2);
      end;


    procedure tcoffexeoutput.DataPos_Header;
      begin
        { retrieve amount of sections }
        nsects:=0;
        ExeSectionList.ForEachCall(@ExeSectionList_pass2_header,@nsects);
        { calculate start positions after the headers }
        currdatapos:=totalheadersize+longword(nsects)*sizeof(tcoffsechdr);
      end;


    procedure tcoffexeoutput.DataPos_Symbols;
      begin
        inherited DataPos_Symbols;
        { Calculating symbols position and size }
        nsyms:=ExeSymbolList.Count;
        sympos:=Align(CurrDataPos,SectionDataAlign);
        inc(CurrDataPos,sizeof(coffsymbol)*nsyms);
      end;


    function TCoffexeoutput.writedata:boolean;
      var
        i           : longword;
        header      : tcoffheader;
        djoptheader : coffdjoptheader;
        peoptheader : tcoffpeoptheader;
        textExeSec,
        dataExeSec,
        bssExeSec,
        idataExeSec : TExeSection;
        hassymbols,
        writeDbgStrings : boolean;

        procedure UpdateDataDir(const secname:string;idx:longint);
        var
          exesec : TExeSection;
        begin
          exesec:=FindExeSection(secname);
          if assigned(exesec) then
            begin
              peoptheader.DataDirectory[idx].vaddr:=exesec.mempos;
              peoptheader.DataDirectory[idx].size:=exesec.Size;
           end;
        end;

        procedure UpdateImports;
        var
          exesec: TExeSection;
          objsec, iat_start, iat_end, ilt_start: TObjSection;
          i: longint;
        begin
          exesec:=FindExeSection('.idata');
          if exesec=nil then
            exit;
          iat_start:=nil;
          iat_end:=nil;
          ilt_start:=nil;
          for i:=0 to exesec.ObjSectionList.Count-1 do
            begin
              objsec:=TObjSection(exesec.ObjSectionList[i]);
              if (ilt_start=nil) and (Pos('.idata$4',objsec.Name)=1) then
                ilt_start:=objsec;
              if Pos('.idata$5',objsec.Name)=1 then
                begin
                  if iat_start=nil then
                    iat_start:=objsec;
                end
              else
                if Assigned(iat_start) then
                  begin
                    iat_end:=objsec;
                    Break;
                  end;
            end;

          peoptheader.DataDirectory[PE_DATADIR_IDATA].vaddr:=exesec.mempos;
          if Assigned(ilt_start) then
            peoptheader.DataDirectory[PE_DATADIR_IDATA].size:=ilt_start.mempos-exesec.mempos
          else  { should not happen }
            peoptheader.DataDirectory[PE_DATADIR_IDATA].size:=exesec.Size;

          if Assigned(iat_start) and Assigned(iat_end) then
            begin
              peoptheader.DataDirectory[PE_DATADIR_IMPORTADDRESSTABLE].vaddr:=iat_start.mempos;
              peoptheader.DataDirectory[PE_DATADIR_IMPORTADDRESSTABLE].size:=iat_end.mempos-iat_start.mempos;
            end;
        end;

        procedure UpdateTlsDataDir;
        var
          {callbacksection : TExeSection;}
          tlsexesymbol: TExeSymbol;
          tlssymbol: TObjSymbol;
          callbackexesymbol: TExeSymbol;
          //callbacksymbol: TObjSymbol;
        begin
          { according to GNU ld,
            the callback routines should be placed into .CRT$XL*
            sections, and the thread local variables in .tls
            __tls_start__ and __tls_end__ symbols
            should be used for the initialized part,
            which we do not support yet. }
          { For now, we only pass the address of the __tls_used
            asm symbol into PE_DATADIR_TLS with the correct
            size of this table (different for win32/win64 }
          tlsexesymbol:=texesymbol(ExeSymbolList.Find(
            target_info.Cprefix+'_tls_used'));
          if assigned(tlsexesymbol) then
            begin
              tlssymbol:=tlsexesymbol.ObjSymbol;
              peoptheader.DataDirectory[PE_DATADIR_TLS].vaddr:=tlssymbol.address;
              { sizeof(TlsDirectory) is different on host and target when cross-compiling }
              peoptheader.DataDirectory[PE_DATADIR_TLS].size:=TLSDIR_SIZE;
              if IsSharedLibrary then
                begin
                  { Here we should reset __FPC_tls_callbacks value to nil }
                  callbackexesymbol:=texesymbol(ExeSymbolList.Find(
                                        '__FPC_tls_callbacks'));
                  if assigned (callbackexesymbol) then
                    begin
                      //callbacksymbol:=callbackexesymbol.ObjSymbol;

                    end;
                end;

           end;
        end;

      begin
        result:=false;
        FCoffStrs:=TDynamicArray.Create(StrsMaxGrow);
        textExeSec:=FindExeSection('.text');
        dataExeSec:=FindExeSection('.data');
        bssExeSec:=FindExeSection('.bss');
        if not assigned(TextExeSec) or
           not assigned(DataExeSec) then
          internalerror(200602231);
        { do we need to write symbols? }
        hassymbols:=(ExeWriteMode=ewm_dbgonly) or
                    (
                     (ExeWriteMode=ewm_exefull) and
                     not(cs_link_strip in current_settings.globalswitches)
                    );
        writeDbgStrings:=hassymbols or ((ExeWriteMode=ewm_exeonly) and (cs_link_separate_dbg_file in current_settings.globalswitches));
        { Stub }
        if win32 then
          begin
            FWriter.write(win32stub,sizeof(win32stub));
            FWriter.write(pemagic,sizeof(pemagic));
          end
        else
          FWriter.write(go32v2stub,sizeof(go32v2stub));
        { Initial header, will be updated later }
        fillchar(header,sizeof(header),0);
        header.mach:=COFF_MAGIC;
        header.nsects:=nsects;
        if writeDbgStrings then
          header.sympos:=sympos-datapos_offset;
        if hassymbols then
          header.syms:=nsyms;
        if win32 then
          header.opthdr:=sizeof(tcoffpeoptheader)
        else
          header.opthdr:=sizeof(coffdjoptheader);
        if win32 then
          begin
            header.flag:=PE_FILE_EXECUTABLE_IMAGE or PE_FILE_LINE_NUMS_STRIPPED;
            if target_info.system in [system_x86_64_win64] then
              header.flag:=header.flag or PE_FILE_LARGE_ADDRESS_AWARE
            else
              header.flag:=header.flag or PE_FILE_32BIT_MACHINE;
            if IsSharedLibrary then
              header.flag:=header.flag or PE_FILE_DLL;
            if FindExeSection('.reloc')=nil then
              header.flag:=header.flag or PE_FILE_RELOCS_STRIPPED;
            if (FindExeSection('.stab')=nil) and
               (FindExeSection('.debug_info')=nil) and
               (FindExeSection('.gnu_debuglink')=nil) then
              header.flag:=header.flag or PE_FILE_DEBUG_STRIPPED;
            if not hassymbols then
              header.flag:=header.flag or PE_FILE_LOCAL_SYMS_STRIPPED;
            if SetPEFlagsSetExplicity then
              header.flag:=header.flag or peflags;
          end
        else
          header.flag:=COFF_FLAG_AR32WR or COFF_FLAG_EXE or COFF_FLAG_NORELOCS or COFF_FLAG_NOLINES;
        FWriter.write(header,sizeof(header));
        { Optional COFF Header }
        if win32 then
          begin
            fillchar(peoptheader,sizeof(peoptheader),0);
            peoptheader.magic:=COFF_OPT_MAGIC;
            peoptheader.MajorLinkerVersion:=ord(version_nr)-ord('0');
            peoptheader.MinorLinkerVersion:=(ord(release_nr)-ord('0'))*10 + (ord(patch_nr)-ord('0'));
            peoptheader.tsize:=TextExeSec.Size;
            peoptheader.dsize:=DataExeSec.Size;
            if assigned(BSSExeSec) then
              peoptheader.bsize:=BSSExeSec.Size;
            peoptheader.text_start:=TextExeSec.mempos;
{$ifndef cpu64bitaddr}
            peoptheader.data_start:=DataExeSec.mempos;
{$endif cpu64bitaddr}
            peoptheader.entry:=EntrySym.Address;
            peoptheader.ImageBase:=ImageBase;
            peoptheader.SectionAlignment:=SectionMemAlign;
            peoptheader.FileAlignment:=SectionDataAlign;
            if SetPEOSVersionSetExplicitely then
              begin
                peoptheader.MajorOperatingSystemVersion:=peosversionmajor;
                peoptheader.MinorOperatingSystemVersion:=peosversionminor;
              end
            else
              begin
                peoptheader.MajorOperatingSystemVersion:=4;
                peoptheader.MinorOperatingSystemVersion:=0;
              end;
            if SetPEUserVersionSetExplicitely then
              begin
                peoptheader.MajorImageVersion:=peuserversionmajor;
                peoptheader.MinorImageVersion:=peuserversionminor;
              end
            else
              begin
                peoptheader.MajorImageVersion:=dllmajor;
                peoptheader.MinorImageVersion:=dllminor;
              end;
            if SetPESubSysVersionSetExplicitely then
              begin
                peoptheader.MajorSubsystemVersion:=pesubsysversionmajor;
                peoptheader.MinorSubsystemVersion:=pesubsysversionminor;
              end
            else
              begin
                if target_info.system in systems_wince then
                  peoptheader.MajorSubsystemVersion:=3
                else
                  peoptheader.MajorSubsystemVersion:=4;
                peoptheader.MinorSubsystemVersion:=0;
              end;
            peoptheader.Win32Version:=0;
            peoptheader.SizeOfImage:=Align(CurrMemPos,SectionMemAlign);
            peoptheader.SizeOfHeaders:=textExeSec.DataPos;
            peoptheader.CheckSum:=0;
            if (target_info.system in systems_nativent) and (not IsSharedLibrary or (apptype = app_native)) then
              { Although I did not really test this, it seems that Subsystem is
                not checked in DLLs except for maybe drivers}
              peoptheader.Subsystem:=PE_SUBSYSTEM_NATIVE
            else
              if target_info.system in systems_wince then
                peoptheader.Subsystem:=PE_SUBSYSTEM_WINDOWS_CE_GUI
              else
                if apptype=app_gui then
                  peoptheader.Subsystem:=PE_SUBSYSTEM_WINDOWS_GUI
                else
                  peoptheader.Subsystem:=PE_SUBSYSTEM_WINDOWS_CUI;

            if SetPEOptFlagsSetExplicity then
              peoptheader.DllCharacteristics:=peoptflags
            else
              peoptheader.DllCharacteristics:=0;

            peoptheader.SizeOfStackReserve:=stacksize;
            peoptheader.SizeOfStackCommit:=$1000;
            if MinStackSizeSetExplicity then
              peoptheader.SizeOfStackCommit:=minstacksize;
            if MaxStackSizeSetExplicity then
              peoptheader.SizeOfStackReserve:=maxstacksize;
            peoptheader.SizeOfHeapReserve:=$100000;
            peoptheader.SizeOfHeapCommit:=$1000;
            peoptheader.NumberOfRvaAndSizes:=PE_DATADIR_ENTRIES;
            UpdateImports;
            UpdateTlsDataDir;
            UpdateDataDir('.edata',PE_DATADIR_EDATA);
            UpdateDataDir('.rsrc',PE_DATADIR_RSRC);
            UpdateDataDir('.pdata',PE_DATADIR_PDATA);
            UpdateDataDir('.reloc',PE_DATADIR_RELOC);
            FWriter.write(peoptheader,sizeof(peoptheader));
          end
        else
          begin
            fillchar(djoptheader,sizeof(djoptheader),0);
            djoptheader.magic:=COFF_OPT_MAGIC;
            djoptheader.tsize:=TextExeSec.Size;
            djoptheader.dsize:=DataExeSec.Size;
            if assigned(BSSExeSec) then
              djoptheader.bsize:=BSSExeSec.Size;
            djoptheader.text_start:=TextExeSec.mempos;
            djoptheader.data_start:=DataExeSec.mempos;
            djoptheader.entry:=EntrySym.Address;
            FWriter.write(djoptheader,sizeof(djoptheader));
          end;

        { For some unknown reason WM 6.1 requires .idata section to be read only.
          Otherwise it refuses to load DLLs greater than 64KB.
          Earlier versions of WinCE load DLLs regardless of .idata flags. }
        if target_info.system in systems_wince then
          begin
            idataExeSec:=FindExeSection('.idata');
            if idataExeSec<>nil then
              idataExeSec.SecOptions:=idataExeSec.SecOptions - [oso_write];
          end;

        { Section headers }
        ExeSectionList.ForEachCall(@ExeSectionList_write_header,nil);
        { Section data }
        WriteExeSectionContent;
        { Align after the last section }
        FWriter.Writezeros(Align(FWriter.Size,SectionDataAlign)-FWriter.Size);

        { Optional Symbols }
        if SymPos<>FWriter.Size then
          internalerror(200602252);
        if hassymbols then
          ExeSymbolList.ForEachCall(@globalsyms_write_symbol,nil);
        if writeDbgStrings then
          begin
            { Strings }
            i:=FCoffStrs.size+4;
            FWriter.write(i,4);
            FWriter.writearray(FCoffStrs);
          end;
        { Release }
        FCoffStrs.Free;
        result:=true;
      end;


    function IdataObjSectionCompare(Item1, Item2: Pointer): Integer;
      var
        I1 : TObjSection absolute Item1;
        I2 : TObjSection absolute Item2;
      begin
        Result:=CompareStr(I1.Name,I2.Name);
      end;

    procedure TCoffexeoutput.Order_ObjSectionList(ObjSectionList: TFPObjectList;const aPattern:string);
      begin
        { Sort sections having '$' in the name, that's how PECOFF documentation
          tells to handle them. However, look for '$' in the pattern, not in section
          names, because the latter often get superfluous '$' due to mangling. }
        if Pos('$',aPattern)>0 then
          ObjSectionList.Sort(@IdataObjSectionCompare);
      end;


    constructor TDJCoffexeoutput.create;
      begin
        inherited createcoff(false);
        datapos_offset:=sizeof(go32v2stub);
        CExeSection:=TExeSection;
        CObjData:=TDJCoffObjData;
      end;


    procedure TDJCoffexeoutput.MemPos_Header;
      begin
        { Headers are not loaded, first 4K page is reserved }
        CurrMemPos:=$1000;
      end;

    constructor TPECoffexeoutput.create;
      begin
        inherited createcoff(true);
        CExeSection:=TExeSection;
        CObjData:=TPECoffObjData;
      end;


    procedure TPECoffexeoutput.MarkTargetSpecificSections(WorkList:TFPObjectList);
      var
        exesec:TExeSection;
        objsec,textsec:TObjSection;
        objreloc:TObjRelocation;
        i,j:longint;
      begin
        if target_info.system<>system_x86_64_win64 then
          exit;
        exesec:=FindExeSection('.pdata');
        if exesec=nil then
          exit;
        for i:=0 to exesec.ObjSectionList.Count-1 do
          begin
            objsec:=TObjSection(exesec.ObjSectionList[i]);
            if objsec.Used then
              continue;
            j:=0;
            while j<objsec.ObjRelocations.Count do
              begin
                objreloc:=TObjRelocation(objsec.ObjRelocations[j]);
                if objreloc.symbol=nil then
                  InternalError(2013041201);
                textsec:=objreloc.symbol.objsection;
                if textsec.used then
                  begin
                    WorkList.Add(objsec);
                    objsec.used:=true;
                    { The exact algorithm for non-smartlinked .pdata sections
                      is subject for refinement. Extreme cases are:
                      - several disjoint .pdata entries for a function, if function
                        is complex or if compiler splits it into chunks,
                      - single .pdata section referencing several .text sections,
                        may need to remove irrelevant parts like BFD does for
                        .eh_frame sections. }
                    break;
                  end;
                inc(j,3);
              end;
          end;
      end;


    procedure TPECoffexeoutput.AfterUnusedSectionRemoval;
      var
        basedllname : string;

        procedure StartImport(const dllname:string);
        begin
          if assigned(exemap) then
            begin
              exemap.Add('');
              exemap.Add('Importing from DLL '+dllname);
            end;
          basedllname:=ExtractFileName(dllname);
          { idata2 }
          idata2objsection.writereloc_internal(idata4objsection,idata4objsection.size,sizeof(longint),RELOC_RVA);
          idata2objsection.writezeros(2*sizeof(longint));
          idata2objsection.writereloc_internal(idata7objsection,idata7objsection.size,sizeof(longint),RELOC_RVA);
          idata2objsection.writereloc_internal(idata5objsection,idata5objsection.size,sizeof(longint),RELOC_RVA);
          { idata7 }
          idata7objsection.writestr(basedllname);
        end;

        procedure EndImport;
        begin
          { idata4 }
          idata4objsection.writezeros(sizeof(longint));
          if target_info.system=system_x86_64_win64 then
            idata4objsection.writezeros(sizeof(longint));
          { idata5 }
          idata5objsection.writezeros(sizeof(longint));
          if target_info.system=system_x86_64_win64 then
            idata5objsection.writezeros(sizeof(longint));
        end;

        function AddImport(const afuncname,amangledname:string; AOrdNr:longint;isvar:boolean):TObjSymbol;
        const
  {$ifdef arm}
          jmpopcode : array[0..7] of byte = (
            $00,$c0,$9f,$e5,    // ldr ip, [pc, #0]
            $00,$f0,$9c,$e5     // ldr pc, [ip]
          );
  {$else arm}
          jmpopcode : array[0..1] of byte = (
            $ff,$25
          );
  {$endif arm}
          nopopcodes : array[0..1] of byte = (
            $90,$90
          );
        var
          ordint: dword;

          procedure WriteTableEntry(objsec:TObjSection);
          begin
            if AOrdNr <= 0 then
              begin
                objsec.writereloc_internal(idata6objsection,idata6objsection.size,sizeof(longint),RELOC_RVA);
                if target_info.system=system_x86_64_win64 then
                  objsec.writezeros(sizeof(longint));
              end
            else
              begin
                { import by ordinal }
                ordint:=AOrdNr;
                if target_info.system=system_x86_64_win64 then
                  begin
                    objsec.write(ordint,sizeof(ordint));
                    ordint:=$80000000;
                    objsec.write(ordint,sizeof(ordint));
                  end
                else
                  begin
                    ordint:=ordint or $80000000;
                    objsec.write(ordint,sizeof(ordint));
                  end;
              end;
          end;

        begin
          result:=nil;
          if assigned(exemap) then
            begin
              if AOrdNr <= 0 then
                exemap.Add(' Importing Function '+afuncname)
              else
                exemap.Add(' Importing Function '+afuncname+' (OrdNr='+tostr(AOrdNr)+')');
            end;

          { idata4, import lookup table }
          WriteTableEntry(idata4objsection);
          { idata5, import address table }
          internalobjdata.SetSection(idata5objsection);
          if isvar then
            result:=internalobjdata.SymbolDefine(amangledname,AB_GLOBAL,AT_DATA)
          else
            begin
              internalobjdata.SetSection(textobjsection);
              textobjsection.writezeros(align_aword(textobjsection.size,16)-textobjsection.size);
              result:=internalobjdata.SymbolDefine('_'+amangledname,AB_GLOBAL,AT_FUNCTION);
              textobjsection.write(jmpopcode,sizeof(jmpopcode));
{$ifdef x86_64}
              textobjsection.writereloc_internal(idata5objsection,idata5objsection.size,4,RELOC_RELATIVE);
{$else}
              textobjsection.writereloc_internal(idata5objsection,idata5objsection.size,4,RELOC_ABSOLUTE32);
{$endif x86_64}

              textobjsection.write(nopopcodes,align(textobjsection.size,qword(sizeof(nopopcodes)))-textobjsection.size);
            end;
          { idata5 section data }
          WriteTableEntry(idata5objsection);
          if (AOrdNr<=0) then
            begin
              { index hint, function name, null terminator and align }
              ordint:=abs(AOrdNr);
              idata6objsection.write(ordint,2);
              idata6objsection.writestr(afuncname);
              idata6objsection.writezeros(align(idata6objsection.size,2)-idata6objsection.size);
            end;
        end;

      var
        i,j : longint;
        ImportLibrary : TImportLibrary;
        ImportSymbol  : TImportSymbol;
        exesym     : TExeSymbol;
        newdll : boolean;
      begin
        for i:=0 to FImports.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(FImports[i]);
            newdll:=False;
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                exesym:=ImportSymbol.CachedExeSymbol;
                if assigned(exesym) and
                   exesym.Used then
                  begin
                    if (not newdll) then
                      StartImport(ImportLibrary.Name);
                    newdll:=True;
                    exesym.objsymbol:=AddImport(ImportSymbol.Name,ImportSymbol.MangledName,ImportSymbol.OrdNr,ImportSymbol.IsVar);
                  end;
              end;
            if newdll then
              EndImport;
          end;
        FixupSymbols;
      end;


    procedure TPECoffexeoutput.GenerateLibraryImports(ImportLibraryList:TFPHashObjectList);
      var
        i,j: longint;
        ImportLibrary: TImportLibrary;
        ImportSymbol: TImportSymbol;
        exesym: TExeSymbol;
      begin
        { Here map import symbols to exe symbols and create necessary sections.
          Actual import generation is done after unused sections (and symbols) are removed. }
        FImports:=ImportLibraryList;
        textobjsection:=internalObjData.CreateSection(sec_code);
        textobjsection.SecOptions:=[oso_keep];
        idata2objsection:=internalObjData.CreateSection(sec_idata2);
        idata2objsection.SecOptions:=[oso_keep];
        idata4objsection:=internalObjData.CreateSection(sec_idata4);
        idata4objsection.SecOptions:=[oso_keep];
        idata5objsection:=internalObjData.CreateSection(sec_idata5);
        idata5objsection.SecOptions:=[oso_keep];
        idata6objsection:=internalObjData.CreateSection(sec_idata6);
        idata6objsection.SecOptions:=[oso_keep];
        idata7objsection:=internalObjData.CreateSection(sec_idata7);
        idata7objsection.SecOptions:=[oso_keep];

        for i:=0 to ImportLibraryList.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(ImportLibraryList[i]);
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                exesym:=TExeSymbol(ExeSymbolList.Find(ImportSymbol.MangledName));
                if assigned(exesym) and
                   (exesym.State<>symstate_defined) then
                  begin
                    ImportSymbol.CachedExeSymbol:=exesym;
                    exesym.State:=symstate_defined;
                  end;
              end;
          end;
        PackUnresolvedExeSymbols('after DLL imports');
      end;


    procedure TPECoffexeoutput.GenerateRelocs;
      var
        pgaddr, hdrpos : longword;

      procedure FinishBlock;
      var
        p,len : longint;
      begin
        if hdrpos = longword(-1) then
          exit;
        p:=0;
        internalobjdata.writebytes(p,align(internalobjdata.CurrObjSec.size,4)-internalobjdata.CurrObjSec.size);
        p:=internalObjData.CurrObjSec.Data.Pos;
        internalObjData.CurrObjSec.Data.seek(hdrpos+4);
        len:=p-hdrpos;
        internalObjData.CurrObjSec.Data.write(len,4);
        internalObjData.CurrObjSec.Data.seek(p);
        hdrpos:=longword(-1);
      end;

      var
        exesec : TExeSection;
        objsec : TObjSection;
        objreloc : TObjRelocation;
        i,j,k : longint;
        offset : longword;
        w: word;
      begin
        if not RelocSection or FRelocsGenerated then
          exit;
        exesec:=FindExeSection('.reloc');
        if exesec=nil then
          exit;
        objsec:=internalObjData.createsection('.reloc',0,[oso_data,oso_keep]);
        exesec.AddObjSection(objsec);
        pgaddr:=longword(-1);
        hdrpos:=longword(-1);
        for i:=0 to ExeSectionList.Count-1 do
          begin
            exesec:=TExeSection(ExeSectionList[i]);
            for j:=0 to exesec.ObjSectionList.count-1 do
              begin
                objsec:=TObjSection(exesec.ObjSectionList[j]);
                { create relocs only for sections which are loaded in memory }
                if not (oso_load in objsec.SecOptions) then
                  continue;
                for k:=0 to objsec.ObjRelocations.Count-1 do
                  begin
                    objreloc:=TObjRelocation(objsec.ObjRelocations[k]);
                    if not (objreloc.typ in [{$ifdef cpu64bitaddr}RELOC_ABSOLUTE32,{$endif cpu64bitaddr}RELOC_ABSOLUTE]) then
                      continue;
                    offset:=objsec.MemPos+objreloc.dataoffset;
                    if (offset<pgaddr) and (pgaddr<>longword(-1)) then
                      Internalerror(2007062701);
                    if (offset-pgaddr>=4096) or (pgaddr=longword(-1)) then
                      begin
                        FinishBlock;
                        pgaddr:=(offset div 4096)*4096;
                        hdrpos:=internalObjData.CurrObjSec.Data.Pos;
                        internalObjData.writebytes(pgaddr,4);
                        { Reserving space for block size. The size will be written later in FinishBlock }
                        internalObjData.writebytes(k,4);
                      end;
{$ifdef cpu64bitaddr}
                    if objreloc.typ = RELOC_ABSOLUTE then
                      w:=IMAGE_REL_BASED_DIR64
                    else
{$endif cpu64bitaddr}
                      w:=IMAGE_REL_BASED_HIGHLOW;
                    w:=(w shl 12) or (offset-pgaddr);
                    internalObjData.writebytes(w,2);
                  end;
              end;
          end;
        FinishBlock;
        FRelocsGenerated:=true;
      end;


    procedure TPECoffexeoutput.MemPos_Start;
      var
        exesec : TExeSection;
      begin
        if RelocSection then
          begin
            exesec:=FindExeSection('.reloc');
            if exesec=nil then
              InternalError(2012072401);
            exesec.Disabled:=false;
          end;
        inherited;
      end;


      procedure TPECoffexeoutput.MemPos_ExeSection(const aname:string);
        begin
          if aname='.reloc' then
            GenerateRelocs;
          inherited;
        end;



{****************************************************************************
                                 TDJCoffAssembler
****************************************************************************}

    constructor TDJCoffAssembler.Create(info: pasminfo; smart:boolean);
      begin
        inherited;
        CObjOutput:=TDJCoffObjOutput;
        CInternalAr:=tarobjectwriter;
      end;


{****************************************************************************
                               TPECoffAssembler
****************************************************************************}

    constructor TPECoffAssembler.Create(info: pasminfo; smart:boolean);
      begin
        inherited;
        CObjOutput:=TPECoffObjOutput;
        CInternalAr:=tarobjectwriter;
      end;


{*****************************************************************************
                                   DLLReader
*****************************************************************************}

{$ifdef win32}
    var
      Wow64DisableWow64FsRedirection : function (var OldValue : pointer) : boolean;stdcall;
      Wow64RevertWow64FsRedirection : function (OldValue : pointer) : boolean;stdcall;
{$endif win32}

    function ReadDLLImports(const dllname:string;readdllproc:Treaddllproc):boolean;
      type
       TPECoffExpDir=packed record
         flag,
         stamp      : cardinal;
         Major,
         Minor      : word;
         Name,
         Base,
         NumFuncs,
         NumNames,
         AddrFuncs,
         AddrNames,
         AddrOrds   : cardinal;
       end;
      var
        DLLReader : TObjectReader;
        DosHeader : array[0..$7f] of byte;
        PEMagic   : array[0..3] of byte;
        Header    : TCoffHeader;
        peheader  : tcoffpeoptheader;
        NameOfs,
        newheaderofs : longword;
        FuncName  : string;
        expdir    : TPECoffExpDir;
        i         : longint;
        found     : boolean;
        sechdr    : tCoffSecHdr;
{$ifdef win32}
        p : pointer;
{$endif win32}
      begin
        result:=false;
        fillchar(sechdr,sizeof(sechdr),0);
{$ifdef win32}
        if (target_info.system=system_x86_64_win64) and
          assigned(Wow64DisableWow64FsRedirection) then
          Wow64DisableWow64FsRedirection(p);
{$endif win32}
        DLLReader:=TObjectReader.Create;
        DLLReader.OpenFile(dllname);
{$ifdef win32}
        if (target_info.system=system_x86_64_win64) and
          assigned(Wow64RevertWow64FsRedirection) then
          Wow64RevertWow64FsRedirection(p);
{$endif win32}
        if not DLLReader.Read(DosHeader,sizeof(DosHeader)) or
           (DosHeader[0]<>$4d) or (DosHeader[1]<>$5a) then
          begin
            Comment(V_Error,'Invalid DLL '+dllname+', Dos Header invalid');
            exit;
          end;
        newheaderofs:=cardinal(DosHeader[$3c]) or (DosHeader[$3d] shl 8) or (DosHeader[$3e] shl 16) or (DosHeader[$3f] shl 24);
        DLLReader.Seek(newheaderofs);
        if not DLLReader.Read(PEMagic,sizeof(PEMagic)) or
           (PEMagic[0]<>$50) or (PEMagic[1]<>$45) or (PEMagic[2]<>$00) or (PEMagic[3]<>$00) then
          begin
            Comment(V_Error,'Invalid DLL '+dllname+': invalid magic code');
            exit;
          end;
        if not DLLReader.Read(Header,sizeof(TCoffHeader)) or
           (Header.mach<>COFF_MAGIC) or
           (Header.opthdr<>sizeof(tcoffpeoptheader)) then
          begin
            Comment(V_Error,'Invalid DLL '+dllname+', invalid header size');
            exit;
          end;
        { Read optheader }
        DLLreader.Read(peheader,sizeof(tcoffpeoptheader));
        { Section headers }
        found:=false;
        for i:=1 to header.nsects do
          begin
            if not DLLreader.read(sechdr,sizeof(sechdr)) then
              begin
                Comment(V_Error,'Error reading coff file '+DLLName);
                exit;
              end;
            if (sechdr.rvaofs<=peheader.DataDirectory[PE_DATADIR_EDATA].vaddr) and
               (peheader.DataDirectory[PE_DATADIR_EDATA].vaddr<sechdr.rvaofs+sechdr.vsize) then
              begin
                found:=true;
                break;
              end;
          end;
        if not found then
          begin
            Comment(V_Warning,'DLL '+DLLName+' does not contain any exports');
            exit;
          end;
        { Process edata }
        DLLReader.Seek(sechdr.datapos+peheader.DataDirectory[PE_DATADIR_EDATA].vaddr-sechdr.rvaofs);
        DLLReader.Read(expdir,sizeof(expdir));
        for i:=0 to expdir.NumNames-1 do
          begin
            DLLReader.Seek(sechdr.datapos+expdir.AddrNames-sechdr.rvaofs+i*4);
            DLLReader.Read(NameOfs,4);
            Dec(NameOfs,sechdr.rvaofs);
            if {(NameOfs<0) or}
               (NameOfs>sechdr.vsize) then
              begin
                Comment(V_Error,'DLL does contains invalid exports');
                break;
              end;
            { Read Function name from DLL, prepend _ and terminate with #0 }
            DLLReader.Seek(sechdr.datapos+NameOfs);
            DLLReader.Read((@FuncName[1])^,sizeof(FuncName)-3);
            FuncName[sizeof(FuncName)-1]:=#0;
            FuncName[0]:=chr(Strlen(@FuncName[1]));
            readdllproc(DLLName,FuncName);
          end;
        DLLReader.Free;
      end;

{$ifdef arm}
    function COFF_MAGIC: word;
      begin
        if GenerateThumb2Code and (current_settings.cputype>=cpu_armv7) then
          COFF_MAGIC:=$1c4 // IMAGE_FILE_MACHINE_ARMNT
        else
          COFF_MAGIC:=$1c0; // IMAGE_FILE_MACHINE_ARM
      end;
{$endif arm}

{*****************************************************************************
                                  Initialize
*****************************************************************************}

{$ifdef i386}
    const
       as_i386_coff_info : tasminfo =
          (
            id     : as_i386_coff;
            idtxt  : 'COFF';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_i386_go32v2];
            flags : [af_outputbinary,af_smartlink_sections];
            labelprefix : '.L';
            comment : '';
            dollarsign: '$';
          );

       as_i386_pecoff_info : tasminfo =
          (
            id     : as_i386_pecoff;
            idtxt  : 'PECOFF';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_i386_win32,system_i386_nativent];
            flags : [af_outputbinary,af_smartlink_sections];
            labelprefix : '.L';
            comment : '';
            dollarsign: '$';
          );

       as_i386_pecoffwdosx_info : tasminfo =
          (
            id     : as_i386_pecoffwdosx;
            idtxt  : 'PEWDOSX';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_i386_wdosx];
            flags : [af_outputbinary];
            labelprefix : '.L';
            comment : '';
            dollarsign: '$';
          );

       as_i386_pecoffwince_info : tasminfo =
          (
            id     : as_i386_pecoffwince;
            idtxt  : 'PECOFFWINCE';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_i386_wince];
            flags : [af_outputbinary,af_smartlink_sections];
            labelprefix : '.L';
            comment : '';
            dollarsign: '$';
          );
{$endif i386}
{$ifdef x86_64}
    const
       as_x86_64_pecoff_info : tasminfo =
          (
            id     : as_x86_64_pecoff;
            idtxt  : 'PECOFF';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_x86_64_win64];
            flags : [af_outputbinary,af_smartlink_sections];
            labelprefix : '.L';
            comment : '';
            dollarsign: '$';
          );
{$endif x86_64}
{$ifdef arm}
    const
       as_arm_pecoffwince_info : tasminfo =
          (
            id     : as_arm_pecoffwince;
            idtxt  : 'PECOFFWINCE';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_arm_wince];
            flags : [af_outputbinary,af_smartlink_sections];
            labelprefix : '.L';
            comment : '';
            dollarsign: '$';
          );
{$endif arm}


{$ifdef win32}
  procedure SetupProcVars;
    var
      hinstLib : THandle;
    begin
      Wow64DisableWow64FsRedirection:=nil;
      Wow64RevertWow64FsRedirection:=nil;
      hinstLib:=LoadLibrary('kernel32.dll');
      if hinstLib<>0 then
        begin
          pointer(Wow64DisableWow64FsRedirection):=GetProcAddress(hinstLib,'Wow64DisableWow64FsRedirection');
          pointer(Wow64RevertWow64FsRedirection):=GetProcAddress(hinstLib,'Wow64RevertWow64FsRedirection');
          FreeLibrary(hinstLib);
        end;
    end;
{$endif win32}


initialization
{$ifdef i386}
  RegisterAssembler(as_i386_coff_info,TDJCoffAssembler);
  RegisterAssembler(as_i386_pecoff_info,TPECoffAssembler);
  RegisterAssembler(as_i386_pecoffwdosx_info,TPECoffAssembler);
  RegisterAssembler(as_i386_pecoffwince_info,TPECoffAssembler);
{$endif i386}
{$ifdef x86_64}
  RegisterAssembler(as_x86_64_pecoff_info,TPECoffAssembler);
{$endif x86_64}
{$ifdef arm}
  RegisterAssembler(as_arm_pecoffwince_info,TPECoffAssembler);
{$endif arm}
{$ifdef win32}
  SetupProcVars;
{$endif win32}
end.
