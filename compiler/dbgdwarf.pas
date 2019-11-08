{
    Copyright (c) 2003-2006 by Peter Vreman and Florian Klaempfl

    This units contains support for DWARF debug info generation

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
{
  This units contains support for DWARF debug info generation.

  Currently a lot of code looks like being mergable with dbgstabs. This might
  change however when improved dwarf info is generated, so the stuff shouldn't be
  merged yet. (FK)

  The easiest way to debug dwarf debug info generation is the usage of
  readelf --debug-dump <executable>
  This works only with elf targets though.

  There is a similar utility called dwarfdump which is not elf-specific and
  which has been ported to most systems.
}
unit dbgdwarf;

{$i fpcdefs.inc}

interface

    uses
      cclasses,globtype,
      cgbase,
      aasmbase,aasmtai,aasmdata,
      symbase,symconst,symtype,symdef,symsym,
      finput,
      DbgBase;

    type
      {$ifdef avr}
      // re-map to larger types because of offsets required to distinguish different memory spaces
      puint = cardinal;
      pint = longint;
      {$endif avr}

      { Tag names and codes.   }
      tdwarf_tag = (DW_TAG_padding := $00,DW_TAG_array_type := $01,
        DW_TAG_class_type := $02,DW_TAG_entry_point := $03,
        DW_TAG_enumeration_type := $04,DW_TAG_formal_parameter := $05,
        DW_TAG_imported_declaration := $08,DW_TAG_label := $0a,
        DW_TAG_lexical_block := $0b,DW_TAG_member := $0d,
        DW_TAG_pointer_type := $0f,DW_TAG_reference_type := $10,
        DW_TAG_compile_unit := $11,DW_TAG_stringtypee := $12,
        DW_TAG_structure_type := $13,DW_TAG_subroutine_type := $15,
        DW_TAG_typedef := $16,DW_TAG_union_type := $17,
        DW_TAG_unspecified_parameters := $18,
        DW_TAG_variant := $19,DW_TAG_common_block := $1a,
        DW_TAG_common_inclusion := $1b,DW_TAG_inheritance := $1c,
        DW_TAG_inlined_subroutine := $1d,DW_TAG_module := $1e,
        DW_TAG_ptr_to_member_type := $1f,DW_TAG_set_type := $20,
        DW_TAG_subrange_type := $21,DW_TAG_with_stmt := $22,
        DW_TAG_access_declaration := $23,DW_TAG_base_type := $24,
        DW_TAG_catch_block := $25,DW_TAG_const_type := $26,
        DW_TAG_constant := $27,DW_TAG_enumerator := $28,
        DW_TAG_file_type := $29,DW_TAG_friend := $2a,
        DW_TAG_namelist := $2b,DW_TAG_namelist_item := $2c,
        DW_TAG_packed_type := $2d,DW_TAG_subprogram := $2e,
        DW_TAG_template_type_param := $2f,DW_TAG_template_value_param := $30,
        DW_TAG_thrown_type := $31,DW_TAG_try_block := $32,
        DW_TAG_variant_part := $33,DW_TAG_variable := $34,
        DW_TAG_volatile_type := $35,
        { DWARF 3.   }
        DW_TAG_dwarf_procedure := $36,
        DW_TAG_restrict_type := $37,DW_TAG_interface_type := $38,
        DW_TAG_namespace := $39,DW_TAG_imported_module := $3a,
        DW_TAG_unspecified_type := $3b,DW_TAG_partial_unit := $3c,
        DW_TAG_imported_unit := $3d,
        DW_TAG_condition := $3f,
        DW_TAG_shared_type := $40,

        { DWARF 4 }
        DW_TAG_type_unit := $41,
        DW_TAG_rvalue_reference_type := $42,
        DW_TAG_template_alias := $43,


        { SGI/MIPS Extensions.   }
        DW_TAG_MIPS_loop := $4081,

        { HP extensions.  See: ftp://ftp.hp.com/pub/lang/tools/WDB/wdb-4.0.tar.gz .   }
        DW_TAG_HP_array_descriptor := $4090,

        { GNU extensions.   }
        { For FORTRAN 77 and Fortran 90.   }
        DW_TAG_format_label := $4101,
        { For C++.   }
        DW_TAG_function_template := $4102,DW_TAG_class_template := $4103,

        DW_TAG_GNU_BINCL := $4104,DW_TAG_GNU_EINCL := $4105,
        { Extensions for UPC.  See: http://upc.gwu.edu/~upc.   }
        DW_TAG_upc_shared_type := $8765,DW_TAG_upc_strict_type := $8766,
        DW_TAG_upc_relaxed_type := $8767,

        { PGI (STMicroelectronics) extensions.  No documentation available.   }
        DW_TAG_PGI_kanji_type := $A000,
        DW_TAG_PGI_interface_block := $A020
      );

{$push} 
{$notes off}
      { Attribute names and codes.   }
      tdwarf_attribute = (DW_AT_sibling := $01,DW_AT_location := $02,
        DW_AT_name := $03,DW_AT_ordering := $09,
        DW_AT_subscr_data := $0a,DW_AT_byte_size := $0b,
        DW_AT_bit_offset := $0c,DW_AT_bit_size := $0d,
        DW_AT_element_list := $0f,DW_AT_stmt_list := $10,
        DW_AT_low_pc := $11,DW_AT_high_pc := $12,
        DW_AT_language := $13,DW_AT_member := $14,
        DW_AT_discr := $15,DW_AT_discr_value := $16,
        DW_AT_visibility := $17,DW_AT_import := $18,
        DW_AT_string_length := $19,DW_AT_common_reference := $1a,
        DW_AT_comp_dir := $1b,DW_AT_const_value := $1c,
        DW_AT_containing_type := $1d,DW_AT_default_value := $1e,
        DW_AT_inline := $20,DW_AT_is_optional := $21,
        DW_AT_lower_bound := $22,DW_AT_producer := $25,
        DW_AT_prototyped := $27,DW_AT_return_addr := $2a,
        DW_AT_start_scope := $2c,DW_AT_stride_size := $2e,
        DW_AT_upper_bound := $2f,DW_AT_abstract_origin := $31,
        DW_AT_accessibility := $32,DW_AT_address_class := $33,
        DW_AT_artificial := $34,DW_AT_base_types := $35,
        DW_AT_calling_convention := $36,DW_AT_count := $37,
        DW_AT_data_member_location := $38,DW_AT_decl_column := $39,
        DW_AT_decl_file := $3a,DW_AT_decl_line := $3b,
        DW_AT_declaration := $3c,DW_AT_discr_list := $3d,
        DW_AT_encoding := $3e,DW_AT_external := $3f,
        DW_AT_frame_base := $40,DW_AT_friend := $41,
        DW_AT_identifier_case := $42,DW_AT_macro_info := $43,
        DW_AT_namelist_items := $44,DW_AT_priority := $45,
        DW_AT_segment := $46,DW_AT_specification := $47,
        DW_AT_static_link := $48,DW_AT_type := $49,
        DW_AT_use_location := $4a,DW_AT_variable_parameter := $4b,
        DW_AT_virtuality := $4c,DW_AT_vtable_elem_location := $4d,

        { DWARF 3 values.   }
        DW_AT_allocated := $4e,DW_AT_associated := $4f,
        DW_AT_data_location := $50,DW_AT_byte_stride := $51,
        DW_AT_entry_pc := $52,DW_AT_use_UTF8 := $53,
        DW_AT_extension := $54,DW_AT_ranges := $55,
        DW_AT_trampoline := $56,DW_AT_call_column := $57,
        DW_AT_call_file := $58,DW_AT_call_line := $59,
        DW_AT_description := $5a,     { string }
        DW_AT_binary_scale := $5b,    { constant }
        DW_AT_decimal_scale := $5c,   { constant }
        DW_AT_small := $5d,           { reference }
        DW_AT_decimal_sign := $5e,    { constant }
        DW_AT_digit_count := $5f,     { constant }
        DW_AT_picture_string := $60,  { string }
        DW_AT_mutable := $61,         { flag }
        DW_AT_threads_scaled := $62,  { flag }
        DW_AT_explicit := $63,        { flag }
        DW_AT_object_pointer := $64,  { reference }
        DW_AT_endianity := $65,       { constant }
        DW_AT_elemental := $66,       { flag }
        DW_AT_pure := $67,            { flag }
        DW_AT_recursive := $68,       { flag }

        { DWARF 4 values }
        DW_AT_signature := $69,       { reference }
        DW_AT_main_subprogram := $6a, { flag }
        DW_AT_data_bit_offset := $6b, { constant }
        DW_AT_const_expr := $6c,      { flag }
        DW_AT_enum_class := $6d,      { flag }
        DW_AT_linkage_name := $6e,    { string }


        { SGI/MIPS extensions.   }
        DW_AT_MIPS_fde := $2001,DW_AT_MIPS_loop_begin := $2002,
        DW_AT_MIPS_tail_loop_begin := $2003,DW_AT_MIPS_epilog_begin := $2004,
        DW_AT_MIPS_loop_unroll_factor := $2005,
        DW_AT_MIPS_software_pipeline_depth := $2006,
        DW_AT_MIPS_linkage_name := $2007,DW_AT_MIPS_stride := $2008,
        DW_AT_MIPS_abstract_name := $2009,DW_AT_MIPS_clone_origin := $200a,
        DW_AT_MIPS_has_inlines := $200b,

        { HP extensions.   }
        DW_AT_HP_block_index := $2000,
        DW_AT_HP_unmodifiable := $2001,DW_AT_HP_actuals_stmt_list := $2010,
        DW_AT_HP_proc_per_section := $2011,DW_AT_HP_raw_data_ptr := $2012,
        DW_AT_HP_pass_by_reference := $2013,DW_AT_HP_opt_level := $2014,
        DW_AT_HP_prof_version_id := $2015,DW_AT_HP_opt_flags := $2016,
        DW_AT_HP_cold_region_low_pc := $2017,DW_AT_HP_cold_region_high_pc := $2018,
        DW_AT_HP_all_variables_modifiable := $2019,
        DW_AT_HP_linkage_name := $201a,DW_AT_HP_prof_flags := $201b,

        { WATCOM extensions. }
        DW_AT_WATCOM_memory_model := $2082,
        DW_AT_WATCOM_references_start := $2083,
        DW_AT_WATCOM_parm_entry := $2084,

        { GNU extensions.   }
        DW_AT_sf_names := $2101,DW_AT_src_info := $2102,
        DW_AT_mac_info := $2103,DW_AT_src_coords := $2104,
        DW_AT_body_begin := $2105,DW_AT_body_end := $2106,
        DW_AT_GNU_vector := $2107,

        { VMS extensions.  }
        DW_AT_VMS_rtnbeg_pd_address := $2201,

        { UPC extension.   }
        DW_AT_upc_threads_scaled := $3210,

        { PGI (STMicroelectronics) extensions.   }
        DW_AT_PGI_lbase := $3a00,
        DW_AT_PGI_soffset := $3a01,DW_AT_PGI_lstride := $3a02,

        { Apple extensions }
        DW_AT_APPLE_optimized := $3fe1,
        DW_AT_APPLE_flags := $3fe2,
        DW_AT_APPLE_major_runtime_vers := $3fe5,
        DW_AT_APPLE_runtime_class := $3fe6
      );
{$pop}

      { Form names and codes.   }
      Tdwarf_form = (DW_FORM_addr := $01,DW_FORM_block2 := $03,
        DW_FORM_block4 := $04,DW_FORM_data2 := $05,
        DW_FORM_data4 := $06,DW_FORM_data8 := $07,
        DW_FORM_string := $08,DW_FORM_block := $09,
        DW_FORM_block1 := $0a,DW_FORM_data1 := $0b,
        DW_FORM_flag := $0c,DW_FORM_sdata := $0d,
        DW_FORM_strp := $0e,DW_FORM_udata := $0f,
        DW_FORM_ref_addr := $10,DW_FORM_ref1 := $11,
        DW_FORM_ref2 := $12,DW_FORM_ref4 := $13,
        DW_FORM_ref8 := $14,DW_FORM_ref_udata := $15,
        DW_FORM_indirect := $16,

        { DWARF 4 }
        DW_FORM_sec_offset := $17,   { lineptr, loclistptr, macptr, rangelistptr }
        DW_FORM_exprloc := $18,      { exprloc }
        DW_FORM_flag_present := $19, { flag }
        DW_FORM_ref_sig8 := $20      { reference }
        );

      { values of DW_AT_address_class }
      Tdwarf_addr = (
        DW_ADDR_none := 0,
        DW_ADDR_near16 := 1,
        DW_ADDR_far16 := 2,
        DW_ADDR_huge16 := 3,
        DW_ADDR_near32 := 4,
        DW_ADDR_far32 := 5
      );

      { values of DW_AT_WATCOM_memory_model }
      Tdwarf_watcom_memory_model = (
        DW_WATCOM_MEMORY_MODEL_none := 0,
        DW_WATCOM_MEMORY_MODEL_flat := 1,
        DW_WATCOM_MEMORY_MODEL_small := 2,
        DW_WATCOM_MEMORY_MODEL_medium := 3,
        DW_WATCOM_MEMORY_MODEL_compact := 4,
        DW_WATCOM_MEMORY_MODEL_large := 5,
        DW_WATCOM_MEMORY_MODEL_huge := 6
      );

      TDwarfFile = record
        Index: integer;
        Name: PChar;
      end;

      { flags for emitting variables/parameters }
      tdwarfvarsymflag =
        { force the sym to be emitted as a local variable regardless of its
          type; used for "absolute" local variables referring to parameters.
        }
        (dvf_force_local_var
        );
      tdwarfvarsymflags = set of tdwarfvarsymflag;

      pAbbrevSearchTreeItem = ^tAbbrevSearchTreeItem;
      tAbbrevSearchTreeItem = record
        value: QWord;
        Abbrev: longint;
        // When this item does not match the abbrev-value, look for it
        // in the next SearchItem
        SearchItem: pAbbrevSearchTreeItem;
        // Next and prior item of the abbrev-section
        NextItem: pAbbrevSearchTreeItem;
        PriorItem: pAbbrevSearchTreeItem;
        bit8: boolean;
      end;

      TDwarfHashSetItem = record
        HashSetItem: THashSetItem;
        lab, ref_lab: tasmsymbol;
        struct_lab: tasmsymbol;
      end;
      PDwarfHashSetItem = ^TDwarfHashSetItem;

      TDwarfLabHashSet = class(THashSet)
        class function SizeOfItem: Integer; override;
      end;

      { TDebugInfoDwarf }

      TDebugInfoDwarf = class(TDebugInfo)
      private
        currabbrevnumber : longint;

        { use this defs to create info for variants and file handles }
        { unused (MWE)
        filerecdef,
        textrecdef : tdef;
        }

        dirlist: TFPHashObjectList;
        filesequence: Integer;
        loclist: tdynamicarray;
        asmline: TAsmList;

        { lookup table for def -> DWARF-labels }
        dwarflabels: TDwarfLabHashSet;

        // The current entry in dwarf_info with the link to the abbrev-section
        dwarf_info_abbref_tai: tai_const;
        // Empty start-item of the abbrev-searchtree
        AbbrevSearchTree: pAbbrevSearchTreeItem;
        // The current abbrev-item
        CurrentSearchTreeItem: pAbbrevSearchTreeItem;
        // Is true when the abbrev-section is newly created
        NewAbbrev: boolean;
        procedure StartAbbrevSearch;
        procedure AddConstToAbbrev(Value: QWord; bit8:boolean=false);
        procedure StartAbbrevSectionFromSearchtree;
        procedure WriteSearchItemToAbbrevSection(SI: pAbbrevSearchTreeItem);
        function FinishAbbrevSearch: longint;

        function def_dwarf_lab(def:tdef) : tasmsymbol;
        function def_dwarf_ref_lab(def:tdef) : tasmsymbol;
        function def_dwarf_class_struct_lab(def:tobjectdef) : tasmsymbol;
        function get_file_index(afile: tinputfile): Integer;
        function relative_dwarf_path(const s:tcmdstr):tcmdstr;
      protected
        // set if we should use 64bit headers (dwarf3 and up)
        _use_64bit_headers: Boolean;
        // set to ait_const32bit if use_64bit_headers is false, otherwise
        // to ait_const64bit
        offsetreltype,
        offsetabstype : taiconst_type;
        // set if we generated any lineinfo at all. If not, we have to terminate
        // when insertmoduleinfo is called.
        generated_lineinfo: boolean;

        vardatadef: trecorddef;

        procedure set_use_64bit_headers(state: boolean);
        property use_64bit_headers: Boolean read _use_64bit_headers write set_use_64bit_headers;

        function get_def_dwarf_labs(def:tdef): PDwarfHashSetItem;

        function is_fbreg(reg:tregister):boolean;

        { Convenience version of the method below, so the compiler creates the
          tvarrec for us (must only pass one element in the last parameter).  }
        procedure append_attribute(attr: tdwarf_attribute; form: tdwarf_form; const values: array of const);
        procedure append_attribute(attr: tdwarf_attribute; form: tdwarf_form; const value: tvarrec);
        procedure append_entry(tag : tdwarf_tag;has_children : boolean;data : array of const);
        procedure append_block1(attr: tdwarf_attribute; size: aint);
        procedure append_labelentry(attr : tdwarf_attribute;sym : tasmsymbol);
        procedure append_labelentry_addr_ref(attr : tdwarf_attribute;sym : tasmsymbol); virtual;
        procedure append_labelentry_ref(attr : tdwarf_attribute;sym : tasmsymbol);
        procedure append_labelentry_dataptr_abs(attr : tdwarf_attribute;sym : tasmsymbol);
        procedure append_labelentry_dataptr_rel(attr : tdwarf_attribute;sym,endsym : tasmsymbol);
        procedure append_labelentry_dataptr_common(attr : tdwarf_attribute);
        procedure append_pointerclass(list:TAsmList;def:tpointerdef);
        procedure append_proc_frame_base(list:TAsmList;def:tprocdef);
{$ifdef i8086}
        procedure append_seg_name(const name:string);
        procedure append_seg_reg(const segment_register:tregister);
{$endif i8086}

        procedure beforeappenddef(list:TAsmList;def:tdef);override;
        procedure afterappenddef(list:TAsmList;def:tdef);override;
        procedure appenddef_ord(list:TAsmList;def:torddef);override;
        procedure appenddef_float(list:TAsmList;def:tfloatdef);override;
        procedure appenddef_enum(list:TAsmList;def:tenumdef);override;
        procedure appenddef_array(list:TAsmList;def:tarraydef);override;
        procedure appenddef_record_named(list:TAsmList;def:trecorddef;const name: shortstring);
        procedure appenddef_record(list:TAsmList;def:trecorddef);override;
        procedure appenddef_pointer(list:TAsmList;def:tpointerdef);override;
        procedure appenddef_string(list:TAsmList;def:tstringdef);override;
        procedure appenddef_procvar(list:TAsmList;def:tprocvardef);override;
        procedure appendprocdef(list:TAsmList;def:tprocdef);override;

        function  get_symlist_sym_offset(symlist: ppropaccesslistitem; out sym: tabstractvarsym; out offset: pint): boolean;
        procedure appendsym_var(list:TAsmList;sym:tabstractnormalvarsym);
        procedure appendsym_var_with_name_type_offset(list:TAsmList; sym:tabstractnormalvarsym; const name: string; def: tdef; offset: pint; const flags: tdwarfvarsymflags);
        { used for fields and properties mapped to fields }
        procedure appendsym_fieldvar_with_name_offset(list:TAsmList;sym: tfieldvarsym;const name: string; def: tdef; offset: pint);
        procedure appendsym_const_member(list:TAsmList;sym:tconstsym;ismember:boolean);

        procedure beforeappendsym(list:TAsmList;sym:tsym);override;
        procedure appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);override;
        procedure appendsym_paravar(list:TAsmList;sym:tparavarsym);override;
        procedure appendsym_localvar(list:TAsmList;sym:tlocalvarsym);override;
        procedure appendsym_fieldvar(list:TAsmList;sym:tfieldvarsym);override;
        procedure appendsym_const(list:TAsmList;sym:tconstsym);override;
        procedure appendsym_type(list:TAsmList;sym:ttypesym);override;
        procedure appendsym_label(list:TAsmList;sym:tlabelsym);override;
        procedure appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);override;
        procedure appendsym_property(list:TAsmList;sym:tpropertysym);override;

        function symdebugname(sym:tsym): String; virtual;
        function symname(sym: tsym; manglename: boolean): String; virtual;
        procedure append_visibility(vis: tvisibility);

        procedure enum_membersyms_callback(p:TObject;arg:pointer);

        procedure finish_children;
        procedure finish_entry;
        procedure finish_lineinfo;
      public
        constructor Create;override;
        destructor Destroy;override;
        procedure insertmoduleinfo;override;
        procedure inserttypeinfo;override;
        procedure referencesections(list:TAsmList);override;
        procedure insertlineinfo(list:TAsmList);override;
        function  dwarf_version: Word; virtual; abstract;
      end;

      { TDebugInfoDwarf2 }

      TDebugInfoDwarf2 = class(TDebugInfoDwarf)
      private
      protected
        procedure appenddef_set_intern(list:TAsmList;def:tsetdef; force_tag_set: boolean);
        procedure append_object_struct(def: tobjectdef; const createlabel: boolean; const objectname: PShortString);

        procedure appenddef_file(list:TAsmList;def:tfiledef); override;
        procedure appenddef_formal(list:TAsmList;def:tformaldef); override;
        procedure appenddef_object(list:TAsmList;def:tobjectdef); override;
        procedure appenddef_set(list:TAsmList;def:tsetdef); override;
        procedure appenddef_undefined(list:TAsmList;def:tundefineddef); override;
        procedure appenddef_variant(list:TAsmList;def:tvariantdef); override;
      public
        function  dwarf_version: Word; override;
      end;

      { TDebugInfoDwarf3 }

      TDebugInfoDwarf3 = class(TDebugInfoDwarf2)
      private
      protected
        procedure append_labelentry_addr_ref(attr : tdwarf_attribute;sym : tasmsymbol); override;
        procedure appenddef_array(list:TAsmList;def:tarraydef); override;
        procedure appenddef_string(list:TAsmList;def:tstringdef);override;
        procedure appenddef_file(list:TAsmList;def:tfiledef); override;
        procedure appenddef_formal(list:TAsmList;def:tformaldef); override;
        procedure appenddef_object(list:TAsmList;def:tobjectdef); override;
        procedure appenddef_set(list:TAsmList;def: tsetdef); override;
        procedure appenddef_undefined(list:TAsmList;def:tundefineddef); override;
        procedure appenddef_variant(list:TAsmList;def:tvariantdef); override;

        function symdebugname(sym:tsym): String; override;
      public
        function  dwarf_version: Word; override;
      end;


      TDebugInfoDwarf4 = class(TDebugInfoDwarf3)
      public
        function  dwarf_version: Word; override;
      end;


implementation

    uses
      sysutils,cutils,cfileutl,constexp,
      version,globals,verbose,systems,
      cpubase,cpuinfo,paramgr,
      fmodule,
      defutil,symtable,symcpu,ppu
{$ifdef OMFOBJSUPPORT}
      ,dbgcodeview
{$endif OMFOBJSUPPORT}
      ;

    const
      LINE_BASE   = 1;
      OPCODE_BASE = 13;

    const
      DW_TAG_lo_user = $4080;
      DW_TAG_hi_user = $ffff;

      { Flag that tells whether entry has a child or not.   }
      DW_children_no = 0;
      DW_children_yes = 1;

    const
      { Implementation-defined range start.   }
      DW_AT_lo_user = $2000;
      { Implementation-defined range end.   }
      DW_AT_hi_user = $3ff0;

    type
      { Source language names and codes.   }
      tdwarf_source_language = (DW_LANG_C89 := $0001,DW_LANG_C := $0002,DW_LANG_Ada83 := $0003,
        DW_LANG_C_plus_plus := $0004,DW_LANG_Cobol74 := $0005,
        DW_LANG_Cobol85 := $0006,DW_LANG_Fortran77 := $0007,
        DW_LANG_Fortran90 := $0008,DW_LANG_Pascal83 := $0009,
        DW_LANG_Modula2 := $000a,DW_LANG_Java := $000b,

        { DWARF 3.   }
        DW_LANG_C99 := $000c,DW_LANG_Ada95 := $000d,
        DW_LANG_Fortran95 := $000e,

        { Objective-C }
        DW_LANG_ObjC := $10,

        { MIPS.   }
        DW_LANG_Mips_Assembler := $8001,

        { UPC.   }
        DW_LANG_Upc := $8765
      );

    const
      { Implementation-defined range start.   }
      DW_LANG_lo_user = $8000;

      { Implementation-defined range start.   }
      DW_LANG_hi_user = $ffff;

      {$ifdef avr}
      // More space required to include memory type offset
      aitconst_ptr_unaligned = aitconst_32bit_unaligned;
      {$endif avr}

    type
      { Names and codes for macro information.   }
      tdwarf_macinfo_record_type = (DW_MACINFO_define := 1,DW_MACINFO_undef := 2,
        DW_MACINFO_start_file := 3,DW_MACINFO_end_file := 4,
        DW_MACINFO_vendor_ext := 255);


    type
      { Type encodings.   }
      Tdwarf_type = (DW_ATE_void := $0,DW_ATE_address := $1,
        DW_ATE_boolean := $2,DW_ATE_complex_float := $3,
        DW_ATE_float := $4,DW_ATE_signed := $5,
        DW_ATE_signed_char := $6,DW_ATE_unsigned := $7,
        DW_ATE_unsigned_char := $8,DW_ATE_imaginary_float := $9,

        { HP extensions.   }
        DW_ATE_HP_float80 := $80,DW_ATE_HP_complex_float80 := $81,
        DW_ATE_HP_float128 := $82,DW_ATE_HP_complex_float128 := $83,
        DW_ATE_HP_floathpintel := $84,DW_ATE_HP_imaginary_float80 := $85,
        DW_ATE_HP_imaginary_float128 := $86
        );


    const
      DW_ATE_lo_user = $80;
      DW_ATE_hi_user = $ff;


    type
      Tdwarf_array_dim_ordering = (DW_ORD_row_major := 0,DW_ORD_col_major := 1
        );

      { Access attribute.   }
      Tdwarf_access_attribute = (DW_ACCESS_public := 1,DW_ACCESS_protected := 2,
        DW_ACCESS_private := 3);

      { Visibility.   }
      Tdwarf_visibility_attribute = (DW_VIS_local := 1,DW_VIS_exported := 2,
        DW_VIS_qualified := 3);

      { Virtuality.   }
      Tdwarf_virtuality_attribute = (DW_VIRTUALITY_none := 0,DW_VIRTUALITY_virtual := 1,
        DW_VIRTUALITY_pure_virtual := 2);

      { Case sensitivity.   }
      Tdwarf_id_case = (DW_ID_case_sensitive := 0,DW_ID_up_case := 1,
        DW_ID_down_case := 2,DW_ID_case_insensitive := 3
        );

      { Calling convention.   }
      Tdwarf_calling_convention = (DW_CC_normal := $1,DW_CC_program := $2,
        DW_CC_nocall := $3,DW_CC_GNU_renesas_sh := $40, DW_CC_GNU_borland_fastcall_i386 := $41
        );
{$push}
{$notes off}
      { Location atom names and codes.   }
      Tdwarf_location_atom = (DW_OP_addr := $03,DW_OP_deref := $06,DW_OP_const1u := $08,
        DW_OP_const1s := $09,DW_OP_const2u := $0a,
        DW_OP_const2s := $0b,DW_OP_const4u := $0c,
        DW_OP_const4s := $0d,DW_OP_const8u := $0e,
        DW_OP_const8s := $0f,DW_OP_constu := $10,
        DW_OP_consts := $11,DW_OP_dup := $12,DW_OP_drop := $13,
        DW_OP_over := $14,DW_OP_pick := $15,DW_OP_swap := $16,
        DW_OP_rot := $17,DW_OP_xderef := $18,DW_OP_abs := $19,
        DW_OP_and := $1a,DW_OP_div := $1b,DW_OP_minus := $1c,
        DW_OP_mod := $1d,DW_OP_mul := $1e,DW_OP_neg := $1f,
        DW_OP_not := $20,DW_OP_or := $21,DW_OP_plus := $22,
        DW_OP_plus_uconst := $23,DW_OP_shl := $24,
        DW_OP_shr := $25,DW_OP_shra := $26,DW_OP_xor := $27,
        DW_OP_bra := $28,DW_OP_eq := $29,DW_OP_ge := $2a,
        DW_OP_gt := $2b,DW_OP_le := $2c,DW_OP_lt := $2d,
        DW_OP_ne := $2e,DW_OP_skip := $2f,DW_OP_lit0 := $30,
        DW_OP_lit1 := $31,DW_OP_lit2 := $32,DW_OP_lit3 := $33,
        DW_OP_lit4 := $34,DW_OP_lit5 := $35,DW_OP_lit6 := $36,
        DW_OP_lit7 := $37,DW_OP_lit8 := $38,DW_OP_lit9 := $39,
        DW_OP_lit10 := $3a,DW_OP_lit11 := $3b,
        DW_OP_lit12 := $3c,DW_OP_lit13 := $3d,
        DW_OP_lit14 := $3e,DW_OP_lit15 := $3f,
        DW_OP_lit16 := $40,DW_OP_lit17 := $41,
        DW_OP_lit18 := $42,DW_OP_lit19 := $43,
        DW_OP_lit20 := $44,DW_OP_lit21 := $45,
        DW_OP_lit22 := $46,DW_OP_lit23 := $47,
        DW_OP_lit24 := $48,DW_OP_lit25 := $49,
        DW_OP_lit26 := $4a,DW_OP_lit27 := $4b,
        DW_OP_lit28 := $4c,DW_OP_lit29 := $4d,
        DW_OP_lit30 := $4e,DW_OP_lit31 := $4f,
        DW_OP_reg0 := $50,DW_OP_reg1 := $51,DW_OP_reg2 := $52,
        DW_OP_reg3 := $53,DW_OP_reg4 := $54,DW_OP_reg5 := $55,
        DW_OP_reg6 := $56,DW_OP_reg7 := $57,DW_OP_reg8 := $58,
        DW_OP_reg9 := $59,DW_OP_reg10 := $5a,DW_OP_reg11 := $5b,
        DW_OP_reg12 := $5c,DW_OP_reg13 := $5d,
        DW_OP_reg14 := $5e,DW_OP_reg15 := $5f,
        DW_OP_reg16 := $60,DW_OP_reg17 := $61,
        DW_OP_reg18 := $62,DW_OP_reg19 := $63,
        DW_OP_reg20 := $64,DW_OP_reg21 := $65,
        DW_OP_reg22 := $66,DW_OP_reg23 := $67,
        DW_OP_reg24 := $68,DW_OP_reg25 := $69,
        DW_OP_reg26 := $6a,DW_OP_reg27 := $6b,
        DW_OP_reg28 := $6c,DW_OP_reg29 := $6d,
        DW_OP_reg30 := $6e,DW_OP_reg31 := $6f,
        DW_OP_breg0 := $70,DW_OP_breg1 := $71,
        DW_OP_breg2 := $72,DW_OP_breg3 := $73,
        DW_OP_breg4 := $74,DW_OP_breg5 := $75,
        DW_OP_breg6 := $76,DW_OP_breg7 := $77,
        DW_OP_breg8 := $78,DW_OP_breg9 := $79,
        DW_OP_breg10 := $7a,DW_OP_breg11 := $7b,
        DW_OP_breg12 := $7c,DW_OP_breg13 := $7d,
        DW_OP_breg14 := $7e,DW_OP_breg15 := $7f,
        DW_OP_breg16 := $80,DW_OP_breg17 := $81,
        DW_OP_breg18 := $82,DW_OP_breg19 := $83,
        DW_OP_breg20 := $84,DW_OP_breg21 := $85,
        DW_OP_breg22 := $86,DW_OP_breg23 := $87,
        DW_OP_breg24 := $88,DW_OP_breg25 := $89,
        DW_OP_breg26 := $8a,DW_OP_breg27 := $8b,
        DW_OP_breg28 := $8c,DW_OP_breg29 := $8d,
        DW_OP_breg30 := $8e,DW_OP_breg31 := $8f,
        DW_OP_regx := $90,DW_OP_fbreg := $91,DW_OP_bregx := $92,
        DW_OP_piece := $93,DW_OP_deref_size := $94,
        DW_OP_xderef_size := $95,DW_OP_nop := $96,

        { DWARF 3 extensions.   }
        DW_OP_push_object_address := $97,DW_OP_call2 := $98,
        DW_OP_call4 := $99,DW_OP_call_ref := $9a,

        { DWARF 4 extensions.   }
        DW_OP_implicit_value := $9e, DW_OP_stack_value := $9f,

        { GNU extensions.   }
        DW_OP_GNU_push_tls_address := $e0,

        { HP extensions.   }
        DW_OP_HP_unknown := $e0,
        DW_OP_HP_is_value := $e1,DW_OP_HP_fltconst4 := $e2,
        DW_OP_HP_fltconst8 := $e3,DW_OP_HP_mod_range := $e4,
        DW_OP_HP_unmod_range := $e5,DW_OP_HP_tls := $e6
        );
{$pop}

    const
      { Implementation-defined range start.   }
      DW_OP_lo_user = $e0;
      { Implementation-defined range end.   }
      DW_OP_hi_user = $ff;


    const
      DW_LNS_extended_op     = $00;

      { next copied from cfidwarf, need to go to something shared }
      DW_LNS_copy            = $01;
      DW_LNS_advance_pc      = $02;
      DW_LNS_advance_line    = $03;
      DW_LNS_set_file        = $04;
      DW_LNS_set_column      = $05;
      DW_LNS_negate_stmt     = $06;
      DW_LNS_set_basic_block = $07;
      DW_LNS_const_add_pc    = $08;

      DW_LNS_fixed_advance_pc   = $09;
      DW_LNS_set_prologue_end   = $0a;
      DW_LNS_set_epilogue_begin = $0b;
      DW_LNS_set_isa            = $0c;

      DW_LNE_end_sequence = $01;
      DW_LNE_set_address  = $02;
      DW_LNE_define_file  = $03;
      { DW_LNE_set_segment is a non-standard Open Watcom extension. It might
        create conflicts with future versions of the DWARF standard. }
      DW_LNE_set_segment  = $04;
      DW_LNE_lo_user      = $80;
      DW_LNE_hi_user      = $ff;

    type
      { TDirIndexItem }

      TDirIndexItem = class(TFPHashObject)
      private
        FFiles: TFPHashObjectList;
      public
        IndexNr : Integer;
        constructor Create(AList:TFPHashObjectList;const AName: String; AIndex: Integer);
        destructor  Destroy;override;
        property Files: TFPHashObjectList read FFiles;
      end;

      { TFileIndexItem }

      TFileIndexItem = class(TFPHashObject)
      private
        FDirIndex: Integer;
      public
        IndexNr : Integer;
        constructor Create(AList:TFPHashObjectList;const AName: String; ADirIndex, AIndex: Integer);
        property DirIndex: Integer read FDirIndex;
      end;


{****************************************************************************
                              procs
****************************************************************************}

    function DirListSortCompare(AItem1, AItem2: Pointer): Integer;
      begin
        Result := TDirIndexItem(AItem1).IndexNr - TDirIndexItem(AItem2).IndexNr;
      end;


    function FileListSortCompare(AItem1, AItem2: Pointer): Integer;
      begin
        Result := TFileIndexItem(AItem1).IndexNr - TFileIndexItem(AItem2).IndexNr;
      end;


    function AllocateNewAiSearchItem: pAbbrevSearchTreeItem;
      begin
        new(result);
        FillChar(result^,sizeof(result^),#0);
      end;

    procedure FreeSearchItem(SI: pAbbrevSearchTreeItem);
      begin
        if assigned(SI^.NextItem) then
          FreeSearchItem(SI^.NextItem);
        if assigned(SI^.SearchItem) then
          FreeSearchItem(SI^.SearchItem);
        Dispose(SI);
      end;


{****************************************************************************
                              TDwarfLabHashSet
****************************************************************************}

    class function TDwarfLabHashSet.SizeOfItem: Integer;
      begin
        Result:=sizeof(TDwarfHashSetItem);
      end;

{****************************************************************************
                              TDirIndexItem
****************************************************************************}

    constructor TDirIndexItem.Create(AList:TFPHashObjectList;const AName: String; AIndex: Integer);
      begin
        inherited Create(AList,AName);
        FFiles := TFPHashObjectList.Create;
        IndexNr := AIndex;
      end;


    destructor TDirIndexItem.Destroy;
      begin
        FFiles.Free;
        inherited Destroy;
      end;


{****************************************************************************
                              TFileIndexItem
****************************************************************************}

    constructor TFileIndexItem.Create(AList:TFPHashObjectList;const AName: String; ADirIndex, AIndex: Integer);
    begin
      inherited Create(AList,Aname);
      FDirIndex := ADirIndex;
      IndexNr := AIndex;
    end;


{****************************************************************************
                              TDebugInfoDwarf
****************************************************************************}

    procedure TDebugInfoDwarf.StartAbbrevSearch;
      begin
        CurrentSearchTreeItem:=AbbrevSearchTree;
      end;


    procedure TDebugInfoDwarf.WriteSearchItemToAbbrevSection(SI: pAbbrevSearchTreeItem);
      begin
        if SI^.bit8 then
          current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_const.Create_8bit(SI^.value))
        else
          current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_const.Create_uleb128bit(SI^.value));
      end;


    procedure TDebugInfoDwarf.StartAbbrevSectionFromSearchtree;

      procedure AddCurrentAndPriorItemsToAbrev(SI: pAbbrevSearchTreeItem);
        begin
          if assigned(SI^.PriorItem) then
            AddCurrentAndPriorItemsToAbrev(SI^.PriorItem);
          WriteSearchItemToAbbrevSection(SI);
        end;

      begin
        NewAbbrev:=true;
        inc(currabbrevnumber);
        current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_comment.Create(strpnew('Abbrev '+tostr(currabbrevnumber))));
        current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_const.create_uleb128bit(currabbrevnumber));

        if CurrentSearchTreeItem<>AbbrevSearchTree then
          AddCurrentAndPriorItemsToAbrev(CurrentSearchTreeItem);
      end;


    function TDebugInfoDwarf.FinishAbbrevSearch: longint;

      procedure FinalizeAbbrevSection;
        begin
          current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_const.create_8bit(0));
          current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_const.create_8bit(0));
          CurrentSearchTreeItem^.Abbrev:=currabbrevnumber;
          NewAbbrev := false;
        end;

      begin
        if NewAbbrev then
          FinalizeAbbrevSection;
        result := CurrentSearchTreeItem^.Abbrev;
        if result=0 then
          begin
            // In this case the abbrev-section equals an existing longer abbrev section.
            // So a new abbrev-section has to be made which ends on the current
            // searchtree item
            StartAbbrevSectionFromSearchtree;
            FinalizeAbbrevSection;
            result := CurrentSearchTreeItem^.Abbrev;
          end;
      end;


    procedure TDebugInfoDwarf.AddConstToAbbrev(Value: QWord; bit8:boolean);

        procedure AddCurrentItemToAbbrev;
          begin
            CurrentSearchTreeItem^.value:=value;
            CurrentSearchTreeItem^.bit8:=bit8;
            WriteSearchItemToAbbrevSection(CurrentSearchTreeItem);
          end;

      var si: pAbbrevSearchTreeItem;
      begin
        // Instead of adding this value directly to the ai-tree, search if an
        // abbrev section with the same values already exist, and use the existing
        // one or create one.
        if NewAbbrev then
          begin
          // The current abbrev-section is new, so add the value to the abbrev-section
          // and add it to the search-list.
          CurrentSearchTreeItem^.NextItem:=AllocateNewAiSearchItem;
          CurrentSearchTreeItem^.NextItem^.PriorItem:=CurrentSearchTreeItem;
          CurrentSearchTreeItem := CurrentSearchTreeItem^.NextItem;
          AddCurrentItemToAbbrev;
          end
        else
          begin
          // Search for the value which is added in the next sections of the
          // searchtree for a match
          si := CurrentSearchTreeItem^.NextItem;
          while assigned(si) do
            begin
              if (SI^.value=Value) and (si^.bit8=bit8) then
                begin
                // If a match is found, set the current searchtree item to the next item
                CurrentSearchTreeItem:=SI;
                Exit;
                end
              else if si^.SearchItem=nil then
                begin
                // If no match is found, add a new item to the searchtree and write
                // a new abbrev-section.
                StartAbbrevSectionFromSearchtree;

                si^.SearchItem:=AllocateNewAiSearchItem;
                if currentsearchtreeitem<>AbbrevSearchTree then
                  si^.SearchItem^.PriorItem:=CurrentSearchTreeItem;
                CurrentSearchTreeItem := si^.SearchItem;

                AddCurrentItemToAbbrev;
                Exit;
                end;
              Si := SI^.SearchItem;
            end;
          // The abbrev section we are looking for is longer than the one
          // which is already in the search-tree. So expand the searchtree with
          // the new value and write a new abbrev section
          StartAbbrevSectionFromSearchtree;

          CurrentSearchTreeItem^.NextItem:=AllocateNewAiSearchItem;
          if currentsearchtreeitem^.PriorItem<>AbbrevSearchTree then
            CurrentSearchTreeItem^.NextItem^.PriorItem:=CurrentSearchTreeItem;
          CurrentSearchTreeItem := CurrentSearchTreeItem^.NextItem;

          AddCurrentItemToAbbrev;
          end;
      end;


    function TDebugInfoDwarf.relative_dwarf_path(const s:tcmdstr):tcmdstr;
      begin
        { Make a clean path for gdb. Remove trailing / and ./ prefixes and
	      use always a / }
         result:=BsToSlash(ExcludeTrailingPathDelimiter(ExtractRelativePath(GetCurrentDir,FixFileName(ExpandFileName(s)))));
      end;


    procedure TDebugInfoDwarf.set_use_64bit_headers(state: boolean);
      begin
         _use_64bit_headers:=state;
         if not(state) then
           begin
             if (target_info.system in systems_windows+systems_wince) then
               offsetabstype:=aitconst_secrel32_symbol
             else
               offsetabstype:=aitconst_32bit_unaligned;
             if (target_info.system in systems_darwin) then
                offsetreltype:=aitconst_darwin_dwarf_delta32
              else
                offsetreltype:=aitconst_32bit_unaligned;
           end
         else
           begin
             if (target_info.system in systems_darwin) then
                offsetreltype:=aitconst_darwin_dwarf_delta64
             else
               offsetreltype:=aitconst_64bit_unaligned;
             offsetabstype:=aitconst_64bit_unaligned;
           end;
      end;


    function TDebugInfoDwarf.get_def_dwarf_labs(def:tdef): PDwarfHashSetItem;
      var
        needstructdeflab: boolean;
      begin
        { Keep track of used dwarf entries, this info is only useful for dwarf entries
          referenced by the symbols. Definitions will always include all
          required stabs }
        if def.dbg_state=dbg_state_unused then
          def.dbg_state:=dbg_state_used;
        { Need a new label? }
        result:=PDwarfHashSetItem(dwarflabels.FindOrAdd(@def,sizeof(def)));
        { the other fields besides  Data are not initialised }
        if not assigned(result^.HashSetItem.Data) then
          begin
            { Mark as initialised }
            result^.HashSetItem.Data:=self;
            needstructdeflab:=is_implicit_pointer_object_type(def);
            if not(tf_dwarf_only_local_labels in target_info.flags) then
              begin
                if (ds_dwarf_dbg_info_written in def.defstates) then
                  begin
                    if not assigned(def.typesym) then
                      internalerror(200610011);
                    result^.lab:=current_asmdata.RefAsmSymbol(make_mangledname('DBG',def.owner,symname(def.typesym, true)),AT_METADATA);
                    result^.ref_lab:=current_asmdata.RefAsmSymbol(make_mangledname('DBGREF',def.owner,symname(def.typesym, true)),AT_METADATA);
                    if needstructdeflab then
                      result^.struct_lab:=current_asmdata.RefAsmSymbol(make_mangledname('DBG2',def.owner,symname(def.typesym, true)),AT_METADATA);
                    def.dbg_state:=dbg_state_written;
                  end
                else
                  begin
                    { Create an exported DBG symbol if we are generating a def defined in the
                      globalsymtable of the current unit }
                    if assigned(def.typesym) and
                       (def.owner.symtabletype=globalsymtable) and
                       (def.owner.iscurrentunit) then
                      begin
                        result^.lab:=current_asmdata.DefineAsmSymbol(make_mangledname('DBG',def.owner,symname(def.typesym, true)),AB_GLOBAL,AT_METADATA,voidpointertype);
                        result^.ref_lab:=current_asmdata.DefineAsmSymbol(make_mangledname('DBGREF',def.owner,symname(def.typesym, true)),AB_GLOBAL,AT_METADATA,voidpointertype);
                        if needstructdeflab then
                          result^.struct_lab:=current_asmdata.DefineAsmSymbol(make_mangledname('DBG2',def.owner,symname(def.typesym, true)),AB_GLOBAL,AT_METADATA,voidpointertype);
                        include(def.defstates,ds_dwarf_dbg_info_written);
                      end
                    else
                      begin
                        { The pointer typecast is needed to prevent a problem with range checking
                          on when the typecast is changed to 'as' }
                        current_asmdata.getglobaldatalabel(TAsmLabel(pointer(result^.lab)));
                        current_asmdata.getglobaldatalabel(TAsmLabel(pointer(result^.ref_lab)));
                        if needstructdeflab then
                          current_asmdata.getglobaldatalabel(TAsmLabel(pointer(result^.struct_lab)));
                      end;
                  end;
              end
            else
              begin
                { The pointer typecast is needed to prevent a problem with range checking
                  on when the typecast is changed to 'as' }
                { addrlabel instead of datalabel because it must be a local one }
                current_asmdata.getaddrlabel(TAsmLabel(pointer(result^.lab)));
                current_asmdata.getaddrlabel(TAsmLabel(pointer(result^.ref_lab)));
                if needstructdeflab then
                  current_asmdata.getaddrlabel(TAsmLabel(pointer(result^.struct_lab)));
              end;
            if def.dbg_state=dbg_state_used then
              deftowritelist.Add(def);
            defnumberlist.Add(def);
          end;
      end;

    function TDebugInfoDwarf.is_fbreg(reg: tregister): boolean;
      begin
{$ifdef i8086}
        result:=reg=NR_BP;
{$else i8086}
        { always return false, because we don't emit DW_AT_frame_base attributes yet }
        result:=false;
{$endif i8086}
      end;

    function TDebugInfoDwarf.def_dwarf_lab(def: tdef): tasmsymbol;
      begin
        result:=get_def_dwarf_labs(def)^.lab;
      end;

    function TDebugInfoDwarf.def_dwarf_class_struct_lab(def: tobjectdef): tasmsymbol;
      begin
        result:=get_def_dwarf_labs(def)^.struct_lab;
      end;

    function TDebugInfoDwarf.def_dwarf_ref_lab(def: tdef): tasmsymbol;
      begin
        result:=get_def_dwarf_labs(def)^.ref_lab;
      end;

    constructor TDebugInfoDwarf.Create;
      begin
        inherited Create;
        { 64bit headers are only supported for dwarf3 and up, so default off }
        use_64bit_headers := false;
        { we haven't generated any lineinfo yet }
        generated_lineinfo := false;

        dirlist := TFPHashObjectList.Create;
        { add current dir as first item (index=0) }
        TDirIndexItem.Create(dirlist,'.', 0);
        asmline := TAsmList.create;
        loclist := tdynamicarray.Create(4096);

        AbbrevSearchTree:=AllocateNewAiSearchItem;

        vardatadef := nil;
      end;


    destructor TDebugInfoDwarf.Destroy;
      begin
        dirlist.Free;
        if assigned(AbbrevSearchTree) then
          FreeSearchItem(AbbrevSearchTree);
        dirlist := nil;
        asmline.free;
        asmline:=nil;
        loclist.Free;
        loclist := nil;
        inherited Destroy;
      end;


    procedure TDebugInfoDwarf.enum_membersyms_callback(p:TObject; arg: pointer);
      begin
        case tsym(p).typ of
          fieldvarsym:
            appendsym_fieldvar(TAsmList(arg),tfieldvarsym(p));
          propertysym:
            appendsym_property(TAsmList(arg),tpropertysym(p));
          constsym:
            appendsym_const_member(TAsmList(arg),tconstsym(p),true);
          else
            ;
        end;
      end;


    function TDebugInfoDwarf.get_file_index(afile: tinputfile): Integer;
      var
        dirname: String;
        diritem: TDirIndexItem;
        diridx: Integer;
        fileitem: TFileIndexItem;
      begin
        if afile.path = '' then
          dirname := '.'
        else
          begin
            { add the canonical form here already to avoid problems with }
            { paths such as './' etc                                     }
            dirname := relative_dwarf_path(afile.path);
            if dirname = '' then
              dirname := '.';
          end;
        diritem := TDirIndexItem(dirlist.Find(dirname));
        if diritem = nil then
          diritem := TDirIndexItem.Create(dirlist,dirname, dirlist.Count);
        diridx := diritem.IndexNr;

        fileitem := TFileIndexItem(diritem.files.Find(afile.name));
        if fileitem = nil then
          begin
            Inc(filesequence);
            fileitem := TFileIndexItem.Create(diritem.files,afile.name, diridx, filesequence);
          end;
        Result := fileitem.IndexNr;
      end;


    procedure TDebugInfoDwarf.append_attribute(attr: tdwarf_attribute; form: tdwarf_form; const values: array of const);
      begin
        if length(values)<>1 then
          internalerror(2009040402);
        append_attribute(attr,form,values[0]);
      end;


    procedure TDebugInfoDwarf.append_attribute(attr: tdwarf_attribute; form: tdwarf_form; const value: tvarrec);
      begin
        { attribute }
        AddConstToAbbrev(cardinal(attr));

        { form }
        AddConstToAbbrev(cardinal(form));

        { info itself }
        case form of
          DW_FORM_string:
            case value.VType of
              vtChar:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_string.create(value.VChar));
              vtString:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_string.create(value.VString^));
              vtAnsistring:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_string.create(Ansistring(value.VAnsiString)));
              else
                internalerror(200601264);
            end;

          DW_FORM_flag:
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(byte(value.VBoolean)));

          DW_FORM_data1:
             case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(value.VQWord^));
              else
                internalerror(200602143);
            end;

          DW_FORM_data2:
             case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(value.VQWord^));
              else
                internalerror(200602144);
            end;

          DW_FORM_data4:
             case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_32bit_unaligned(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_32bit_unaligned(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_32bit_unaligned(value.VQWord^));
              else
                internalerror(200602145);
            end;

          DW_FORM_data8:
             case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_64bit_unaligned(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_64bit_unaligned(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_64bit_unaligned(value.VQWord^));
              else
                internalerror(200602146);
            end;

          DW_FORM_sdata:
            case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_sleb128bit(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_sleb128bit(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_sleb128bit(value.VQWord^));
              else
                internalerror(200601285);
            end;

          DW_FORM_udata:
            case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(value.VQWord^));
              else
                internalerror(200601284);
            end;

          { block gets only the size, the rest is appended manually by the caller }
          DW_FORM_block1:
             case value.VType of
              vtInteger:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(value.VInteger));
              vtInt64:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(value.VInt64^));
              vtQWord:
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(value.VQWord^));
              else
                internalerror(200602141);
            end;
          else
            internalerror(200601263);
        end;
      end;


    { writing the data through a few simply procedures allows to create easily extra information
      for debugging of debug info }
    procedure TDebugInfoDwarf.append_entry(tag : tdwarf_tag;has_children : boolean;data : array of const);
      var
        i : longint;
      begin
        { abbrev number }
        // Store the ai with the reference to the abbrev number and start a search
        // to find the right abbrev-section. (Or create one)
        dwarf_info_abbref_tai := tai_const.create_uleb128bit(currabbrevnumber);
        current_asmdata.asmlists[al_dwarf_info].concat(dwarf_info_abbref_tai);
        StartAbbrevSearch;

        { tag }
        AddConstToAbbrev(ord(tag));

        { children? }
        AddConstToAbbrev(ord(has_children),true);

        i:=0;
        while i<=high(data) do
          begin
            if (i+2 > high(data)) then
              internalerror(2009040401);
            if data[i].VType<>vtInteger then
              internalerror(200601261);
            if data[i+1].VType<>vtInteger then
              internalerror(200601261);
            append_attribute(tdwarf_attribute(data[i].VInteger),tdwarf_form(data[i+1].VInteger),data[i+2]);
            inc(i,3);
          end;
      end;


    procedure TDebugInfoDwarf.append_block1(attr: tdwarf_attribute; size: aint);
      begin
        AddConstToAbbrev(ord(attr));
        AddConstToAbbrev(ord(DW_FORM_block1));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(size));
      end;


    procedure TDebugInfoDwarf.append_labelentry(attr : tdwarf_attribute;sym : tasmsymbol);
      begin
        AddConstToAbbrev(ord(attr));
        AddConstToAbbrev(ord(DW_FORM_addr));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_type_sym(aitconst_ptr_unaligned,sym));
      end;

    procedure TDebugInfoDwarf.append_labelentry_addr_ref(attr : tdwarf_attribute;sym : tasmsymbol);
      begin
        AddConstToAbbrev(ord(DW_FORM_ref_addr));
{$ifdef i8086}
        { DW_FORM_ref_addr is treated as 32-bit by Open Watcom on i8086 }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.Create_type_sym(aitconst_32bit_unaligned,sym));
{$else i8086}
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_type_sym(aitconst_ptr_unaligned,sym));
{$endif i8086}
      end;

    procedure TDebugInfoDwarf.append_labelentry_ref(attr : tdwarf_attribute;sym : tasmsymbol);
      begin
        AddConstToAbbrev(ord(attr));
        if not(tf_dwarf_only_local_labels in target_info.flags) then
          append_labelentry_addr_ref(attr, sym)
        else
          begin
            AddConstToAbbrev(ord(DW_FORM_ref4));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_rel_sym(offsetreltype,current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_info0',AB_LOCAL,AT_METADATA,voidpointertype),sym));
          end;
      end;


    procedure TDebugInfoDwarf.append_labelentry_dataptr_common(attr : tdwarf_attribute);
      begin
        AddConstToAbbrev(ord(attr));
        if use_64bit_headers then
          AddConstToAbbrev(ord(DW_FORM_data8))
        else
          AddConstToAbbrev(ord(DW_FORM_data4));
      end;

    procedure TDebugInfoDwarf.append_pointerclass(list: TAsmList;
      def: tpointerdef);
      begin
{$ifdef i8086}
        case tcpupointerdef(def).x86pointertyp of
          x86pt_near,
          { todo: is there a way to specify these somehow? }
          x86pt_near_cs,x86pt_near_ds,x86pt_near_ss,
          x86pt_near_es,x86pt_near_fs,x86pt_near_gs:
            append_attribute(DW_AT_address_class,DW_FORM_data1,[DW_ADDR_near16]);
          x86pt_far:
            append_attribute(DW_AT_address_class,DW_FORM_data1,[DW_ADDR_far16]);
          x86pt_huge:
            append_attribute(DW_AT_address_class,DW_FORM_data1,[DW_ADDR_huge16]);
        end;
{$else i8086}
        { Theoretically, we could do this, but it might upset some debuggers, }
        { even though it's part of the DWARF standard. }
        { append_attribute(DW_AT_address_class,DW_FORM_data1,[DW_ADDR_none]); }
{$endif i8086}
      end;

    procedure TDebugInfoDwarf.append_proc_frame_base(list: TAsmList;
      def: tprocdef);
{$ifdef i8086}
      var
        dreg: longint;
        blocksize: longint;
        templist: TAsmList;
      begin
        dreg:=dwarf_reg(NR_BP);
        templist:=TAsmList.create;
        if dreg<=31 then
          begin
            templist.concat(tai_const.create_8bit(ord(DW_OP_reg0)+dreg));
            blocksize:=1;
          end
        else
          begin
            templist.concat(tai_const.create_8bit(ord(DW_OP_regx)));
            templist.concat(tai_const.create_uleb128bit(dreg));
            blocksize:=1+Lengthuleb128(dreg);
          end;
        append_block1(DW_AT_frame_base,blocksize);
        current_asmdata.asmlists[al_dwarf_info].concatlist(templist);
        templist.free;
      end;
{$else i8086}
      begin
        { problem: base reg isn't known here
          DW_AT_frame_base,DW_FORM_block1,1
        }
      end;
{$endif i8086}


{$ifdef i8086}
    procedure TDebugInfoDwarf.append_seg_name(const name:string);
      begin
        append_block1(DW_AT_segment,3);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_const2u)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.Create_seg_name(name));
      end;

    procedure TDebugInfoDwarf.append_seg_reg(const segment_register: tregister);
      var
        dreg: longint;
        blocksize: longint;
        templist: TAsmList;
      begin
        dreg:=dwarf_reg(segment_register);
        templist:=TAsmList.create;
        if dreg<=31 then
          begin
            templist.concat(tai_const.create_8bit(ord(DW_OP_reg0)+dreg));
            blocksize:=1;
          end
        else
          begin
            templist.concat(tai_const.create_8bit(ord(DW_OP_regx)));
            templist.concat(tai_const.create_uleb128bit(dreg));
            blocksize:=1+Lengthuleb128(dreg);
          end;
        append_block1(DW_AT_segment,blocksize);
        current_asmdata.asmlists[al_dwarf_info].concatlist(templist);
        templist.free;
      end;
{$endif i8086}


    procedure TDebugInfoDwarf.append_labelentry_dataptr_abs(attr : tdwarf_attribute;sym : tasmsymbol);
      begin
        {
          used for writing dwarf lineptr, loclistptr, macptr and rangelistptr classes as FORM_dataN
          The size of these depend on the header format
          Must be relative to another symbol on tf_dwarf_relative_addresses
          targets
        }
        if (tf_dwarf_relative_addresses in target_info.flags) then
          { use append_labelentry_dataptr_rel instead }
          internalerror(2007020210);
        append_labelentry_dataptr_common(attr);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_type_sym(offsetabstype,sym))
      end;


    procedure TDebugInfoDwarf.append_labelentry_dataptr_rel(attr : tdwarf_attribute;sym,endsym : tasmsymbol);
      begin
        {
          used for writing dwarf lineptr, loclistptr, macptr and rangelistptr classes as FORM_dataN
          The size of these depend on the header format
          Must be relative to another symbol on tf_dwarf_relative_addresses
          targets
        }
        append_labelentry_dataptr_common(attr);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_rel_sym(offsetreltype,sym,endsym));
      end;


    procedure TDebugInfoDwarf.finish_entry;
      begin
        dwarf_info_abbref_tai.value:=FinishAbbrevSearch;
      end;


    procedure TDebugInfoDwarf.finish_children;
      begin
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(0));
      end;

    procedure TDebugInfoDwarf.appenddef_ord(list:TAsmList;def:torddef);
      var
        basedef      : tdef;
        sign         : tdwarf_type;
        signform     : tdwarf_form;
        fullbytesize : byte;
        ordtype      : tordtype;
      begin
        ordtype:=def.ordtype;
        if ordtype=customint then
          ordtype:=range_to_basetype(def.low,def.high);
        case ordtype of
          s8bit,
          s16bit,
          s32bit,
          u8bit,
          u16bit,
          u32bit :
            begin
              { generate proper signed/unsigned info for types like 0..3 }
              { these are s8bit, but should be identified as unsigned    }
              { because otherwise they are interpreted wrongly when used }
              { in a bitpacked record                                    }
              if (def.low<0) then
                begin
                  sign:=DW_ATE_signed;
                  signform:=DW_FORM_sdata
                end
              else
                begin
                  sign:=DW_ATE_unsigned;
                  signform:=DW_FORM_udata
                end;
              fullbytesize:=def.size;
              case fullbytesize of
                1:
                  if (sign=DW_ATE_signed) then
                    basedef:=s8inttype
                  else
                    basedef:=u8inttype;
                2:
                  if (sign=DW_ATE_signed) then
                    basedef:=s16inttype
                  else
                    basedef:=u16inttype;
                3,4:
                  if (sign=DW_ATE_signed) then
                    basedef:=s32inttype
                  else
                    basedef:=u32inttype;
                else
                  internalerror(2008032201);
              end;

              if (def.low=torddef(basedef).low) and
                 (def.high=torddef(basedef).high) then
                { base type such as byte/shortint/word/... }
                if assigned(def.typesym) then
                  append_entry(DW_TAG_base_type,false,[
                    DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                    DW_AT_encoding,DW_FORM_data1,sign,
                    DW_AT_byte_size,DW_FORM_data1,fullbytesize])
                else
                  append_entry(DW_TAG_base_type,false,[
                    DW_AT_encoding,DW_FORM_data1,sign,
                    DW_AT_byte_size,DW_FORM_data1,fullbytesize])
              else
                begin
                  { subrange type }
                  { note: don't do this 64 bit int types, they appear    }
                  {       to be always clipped to s32bit for some reason }
                  if assigned(def.typesym) then
                    append_entry(DW_TAG_subrange_type,false,[
                      DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                      DW_AT_lower_bound,signform,int64(def.low),
                      DW_AT_upper_bound,signform,int64(def.high)
                      ])
                  else
                    append_entry(DW_TAG_subrange_type,false,[
                      DW_AT_lower_bound,signform,int64(def.low),
                      DW_AT_upper_bound,signform,int64(def.high)
                      ]);
                  append_labelentry_ref(DW_AT_type,def_dwarf_lab(basedef));
                end;

              finish_entry;
            end;
          uvoid :
            begin
              { gdb 6.4 doesn't support DW_TAG_unspecified_type so we
                replace it with a unsigned type with size 0 (FK)
              }
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Void'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
                DW_AT_byte_size,DW_FORM_data1,0
              ]);
              finish_entry;
            end;
          uchar :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Char'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned_char,
                DW_AT_byte_size,DW_FORM_data1,1
                ]);
              finish_entry;
            end;
          uwidechar :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'WideChar'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned_char,
                DW_AT_byte_size,DW_FORM_data1,2
                ]);
              finish_entry;
            end;
          pasbool1 :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Boolean'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,1
                ]);
              finish_entry;
            end;
          pasbool8 :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Boolean8'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,1
                ]);
              finish_entry;
            end;
          bool8bit :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'ByteBool'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,1
                ]);
              finish_entry;
            end;
          pasbool16 :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Boolean16'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,2
                ]);
              finish_entry;
            end;
          bool16bit :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'WordBool'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,2
                ]);
              finish_entry;
            end;
          pasbool32 :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Boolean32'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,4
                ]);
              finish_entry;
            end;
          bool32bit :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'LongBool'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,4
                ]);
              finish_entry;
            end;
          pasbool64 :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Boolean64'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
              finish_entry;
            end;
          bool64bit :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'QWordBool'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_boolean,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
              finish_entry;
            end;
          u64bit :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'QWord'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
              finish_entry;
            end;
          scurrency :
            begin
              { we should use DW_ATE_signed_fixed, however it isn't supported yet by GDB (FK) }
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Currency'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
              finish_entry;
            end;
          s64bit :
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Int64'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
              finish_entry;
            end;
          u128bit:
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Int128'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
                DW_AT_byte_size,DW_FORM_data1,16
                ]);
              finish_entry;
            end;
          s128bit:
            begin
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,'Int128'#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,16
                ]);
              finish_entry;
            end;
          else
            internalerror(200601287);
        end;
      end;

    procedure TDebugInfoDwarf.appenddef_float(list:TAsmList;def:tfloatdef);
      begin
        case def.floattype of
          s32real,
          s64real,
          s80real,
          sc80real:
            if assigned(def.typesym) then
              begin
                append_entry(DW_TAG_base_type,false,[
                  DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                  DW_AT_encoding,DW_FORM_data1,DW_ATE_float,
                  DW_AT_byte_size,DW_FORM_data1,def.size
                  ]);
                if (def.floattype in [s80real,sc80real]) and
                   (def.size<>10) then
                  begin
                    append_attribute(DW_AT_bit_size,DW_FORM_data1,[10*8]);
                    { "The bit offset attribute describes the offset in bits
                        of the high order bit of a value of the given type
                        from the high order bit of the storage unit used to
                        contain that value." }
                    if target_info.endian=endian_little then
                      append_attribute(DW_AT_bit_offset,DW_FORM_data1,[(def.size-10)*8]);
                  end;
              end
            else
              append_entry(DW_TAG_base_type,false,[
                DW_AT_encoding,DW_FORM_data1,DW_ATE_float,
                DW_AT_byte_size,DW_FORM_data1,def.size
                ]);
          s64currency:
            { we should use DW_ATE_signed_fixed, however it isn't supported yet by GDB (FK) }
            if assigned(def.typesym) then
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,8
                ])
            else
              append_entry(DW_TAG_base_type,false,[
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
          s64comp:
            if assigned(def.typesym) then
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,8
                ])
            else
              append_entry(DW_TAG_base_type,false,[
                DW_AT_encoding,DW_FORM_data1,DW_ATE_signed,
                DW_AT_byte_size,DW_FORM_data1,8
                ]);
          else
            internalerror(200601289);
        end;
        finish_entry;
      end;


    procedure TDebugInfoDwarf.appenddef_enum(list:TAsmList;def:tenumdef);
      var
        hp : tenumsym;
        i  : integer;
      begin
        if assigned(def.typesym) then
          append_entry(DW_TAG_enumeration_type,true,[
            DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
            DW_AT_byte_size,DW_FORM_data1,def.size
            ])
        else
          append_entry(DW_TAG_enumeration_type,true,[
            DW_AT_byte_size,DW_FORM_data1,def.size
            ]);
        if assigned(def.basedef) then
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.basedef));
        finish_entry;

        { write enum symbols }
        for i := 0 to def.symtable.SymList.Count - 1 do
          begin
            hp:=tenumsym(def.symtable.SymList[i]);
            if hp.value<def.minval then
              continue
            else
            if hp.value>def.maxval then
              break;
            append_entry(DW_TAG_enumerator,false,[
              DW_AT_name,DW_FORM_string,symname(hp, false)+#0,
              DW_AT_const_value,DW_FORM_data4,hp.value
            ]);
            finish_entry;
          end;

        finish_children;
      end;


    procedure TDebugInfoDwarf.appenddef_array(list:TAsmList;def:tarraydef);
      var
        size : PInt;
        elesize : PInt;
        elestrideattr : tdwarf_attribute;
        labsym: tasmlabel;
      begin
        if is_dynamic_array(def) then
          begin
            { It's a pointer to the actual array }
            current_asmdata.getaddrlabel(labsym);
            append_entry(DW_TAG_pointer_type,false,[]);
            append_labelentry_ref(DW_AT_type,labsym);
            finish_entry;
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(labsym,0));
          end;

        if not is_packed_array(def) then
          begin
            elestrideattr:=DW_AT_byte_stride;
            elesize:=def.elesize;
          end
        else
          begin
            elestrideattr:=DW_AT_stride_size;
            elesize:=def.elepackedbitsize;
          end;

        if is_special_array(def) then
          begin
            { no known size, no known upper bound }
            if assigned(def.typesym) then
              append_entry(DW_TAG_array_type,true,[
                DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                elestrideattr,DW_FORM_udata,elesize
                ])
            else
              append_entry(DW_TAG_array_type,true,[
              elestrideattr,DW_FORM_udata,elesize
              ]);
            append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.elementdef));
            finish_entry;
            { a missing upper bound means "unknown"/default }
            append_entry(DW_TAG_subrange_type,false,[
              DW_AT_lower_bound,DW_FORM_sdata,def.lowrange
              ]);
          end
        else
          begin
            size:=def.size;
            if assigned(def.typesym) then
              append_entry(DW_TAG_array_type,true,[
                DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                DW_AT_byte_size,DW_FORM_udata,size,
                elestrideattr,DW_FORM_udata,elesize
                ])
            else
              append_entry(DW_TAG_array_type,true,[
                DW_AT_byte_size,DW_FORM_udata,size,
                elestrideattr,DW_FORM_udata,elesize
                ]);
            append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.elementdef));
            finish_entry;
            { to simplify things, we don't write a multidimensional array here }
            append_entry(DW_TAG_subrange_type,false,[
              DW_AT_lower_bound,DW_FORM_sdata,def.lowrange,
              DW_AT_upper_bound,DW_FORM_sdata,def.highrange
              ]);
          end;
        append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.rangedef));
        finish_entry;
        finish_children;
      end;


    procedure TDebugInfoDwarf.appenddef_record(list:TAsmList;def:trecorddef);
      begin
        if assigned(def.objname) then
          appenddef_record_named(list,def,def.objname^)
        else
          appenddef_record_named(list,def,'');
      end;


    procedure TDebugInfoDwarf.appenddef_record_named(list:TAsmList;def:trecorddef;const name: shortstring);
      begin
        if (name<>'') then
          append_entry(DW_TAG_structure_type,true,[
            DW_AT_name,DW_FORM_string,name+#0,
            DW_AT_byte_size,DW_FORM_udata,def.size
            ])
        else
          append_entry(DW_TAG_structure_type,true,[
            DW_AT_byte_size,DW_FORM_udata,def.size
            ]);
        finish_entry;
        def.symtable.symList.ForEachCall(@enum_membersyms_callback,nil);
        { don't know whether external record declaration is allow but if it so then
          do the same as we do for other object types - skip procdef info generation
          for external defs (Paul Ishenin) }
        if not(oo_is_external in def.objectoptions) then
          write_symtable_procdefs(current_asmdata.asmlists[al_dwarf_info],def.symtable);
        finish_children;
      end;


    procedure TDebugInfoDwarf.appenddef_pointer(list:TAsmList;def:tpointerdef);
      begin
        append_entry(DW_TAG_pointer_type,false,[]);
        append_pointerclass(list,def);
        if not(is_voidpointer(def)) then
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.pointeddef));
        finish_entry;
      end;


    procedure TDebugInfoDwarf.appenddef_string(list:TAsmList;def:tstringdef);

      procedure addnormalstringdef(const name: shortstring; lendef: tdef; maxlen: asizeuint);
        var
          { maxlen can be > high(int64) }
          slen : asizeuint;
          arr : tasmlabel;
        begin
          { fix length of openshortstring }
          slen:=aword(def.len);
          if (slen=0) or
             (slen>maxlen) then
            slen:=maxlen;

          { create a structure with two elements }
          if not(tf_dwarf_only_local_labels in target_info.flags) then
            current_asmdata.getglobaldatalabel(arr)
          else
            current_asmdata.getaddrlabel(arr);
          append_entry(DW_TAG_structure_type,true,[
            DW_AT_name,DW_FORM_string,name+#0,
            DW_AT_byte_size,DW_FORM_udata,qword(lendef.size)+slen
            ]);
          finish_entry;

          { length entry }
          append_entry(DW_TAG_member,false,[
            DW_AT_name,DW_FORM_string,'length'#0,
            DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(0)
            ]);
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(0));
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(lendef));
          finish_entry;

          { string data entry }
          append_entry(DW_TAG_member,false,[
            DW_AT_name,DW_FORM_string,'st'#0,
            DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(1)
            ]);
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(lendef.size));
          append_labelentry_ref(DW_AT_type,arr);
          finish_entry;

          finish_children;

          { now the data array }
          if arr.bind=AB_GLOBAL then
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create_global(arr,0))
          else
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(arr,0));
          append_entry(DW_TAG_array_type,true,[
            DW_AT_byte_size,DW_FORM_udata,def.size,
            DW_AT_byte_stride,DW_FORM_udata,1
            ]);
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(cansichartype));
          finish_entry;
          append_entry(DW_TAG_subrange_type,false,[
            DW_AT_lower_bound,DW_FORM_udata,1,
            DW_AT_upper_bound,DW_FORM_udata,qword(slen)
            ]);
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(lendef));
          finish_entry;
          finish_children;
        end;

      begin
        case def.stringtype of
          st_shortstring:
            begin
              addnormalstringdef('ShortString',u8inttype,255);
            end;
          st_longstring:
            begin
              { a) we don't actually support variables of this type currently
                b) this type is only used as the type for constant strings
                   > 255 characters
                c) in such a case, gdb will allocate and initialise enough
                   memory to hold the maximum size for such a string
                -> don't use high(qword)/high(cardinal) as maximum, since that
                 will cause exhausting the VM space, but some "reasonably high"
                 number that should be enough for most constant strings
              }
{$ifdef cpu64bitaddr}
              addnormalstringdef('LongString',u64inttype,qword(1024*1024));
{$endif cpu64bitaddr}
{$ifdef cpu32bitaddr}
              addnormalstringdef('LongString',u32inttype,cardinal(1024*1024));
{$endif cpu32bitaddr}
{$ifdef cpu16bitaddr}
              addnormalstringdef('LongString',u16inttype,cardinal(1024));
{$endif cpu16bitaddr}
           end;
         st_ansistring:
           begin
             { looks like a pchar }
             append_entry(DW_TAG_pointer_type,false,[]);
             append_labelentry_ref(DW_AT_type,def_dwarf_lab(cansichartype));
             finish_entry;
           end;
         st_unicodestring,
         st_widestring:
           begin
             { looks like a pwidechar }
             append_entry(DW_TAG_pointer_type,false,[]);
             append_labelentry_ref(DW_AT_type,def_dwarf_lab(cwidechartype));
             finish_entry;
           end;
        end;
      end;

    procedure TDebugInfoDwarf.appenddef_procvar(list:TAsmList;def:tprocvardef);

      procedure doappend;
        var
          i : longint;
        begin
          if assigned(def.typesym) then
            append_entry(DW_TAG_subroutine_type,true,[
              DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
              DW_AT_prototyped,DW_FORM_flag,true
            ])
          else
            append_entry(DW_TAG_subroutine_type,true,[
              DW_AT_prototyped,DW_FORM_flag,true
            ]);
          if not(is_void(tprocvardef(def).returndef)) then
            append_labelentry_ref(DW_AT_type,def_dwarf_lab(tprocvardef(def).returndef));
          finish_entry;

          { write parameters }
          for i:=0 to def.paras.count-1 do
            begin
              append_entry(DW_TAG_formal_parameter,false,[
                DW_AT_name,DW_FORM_string,symname(tsym(def.paras[i]), false)+#0
              ]);
              append_labelentry_ref(DW_AT_type,def_dwarf_lab(tparavarsym(def.paras[i]).vardef));
              finish_entry;
            end;

          finish_children;
        end;

      var
        proc : tasmlabel;

      begin
        if not def.is_addressonly then
          begin
            { create a structure with two elements }
            if not(tf_dwarf_only_local_labels in target_info.flags) then
              current_asmdata.getglobaldatalabel(proc)
            else
              current_asmdata.getaddrlabel(proc);
            append_entry(DW_TAG_structure_type,true,[
              DW_AT_byte_size,DW_FORM_data1,2*sizeof(pint)
            ]);
            finish_entry;

            { proc entry }
            append_entry(DW_TAG_member,false,[
              DW_AT_name,DW_FORM_string,'Proc'#0,
              DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(0)
              ]);
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(0));
            append_labelentry_ref(DW_AT_type,proc);
            finish_entry;

            { self entry }
            append_entry(DW_TAG_member,false,[
              DW_AT_name,DW_FORM_string,'Self'#0,
              DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(sizeof(pint))
              ]);
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(sizeof(pint)));
            append_labelentry_ref(DW_AT_type,def_dwarf_lab(class_tobject));
            finish_entry;

            finish_children;

            if proc.bind=AB_GLOBAL then
              current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create_global(proc,0))
            else
              current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(proc,0));
            doappend;
          end
        else
          doappend;
      end;


    procedure TDebugInfoDwarf.beforeappenddef(list:TAsmList;def:tdef);
      var
        labsym : tasmsymbol;
      begin
        current_asmdata.asmlists[al_dwarf_info].concat(tai_comment.Create(strpnew('Definition '+def.typename)));

        labsym:=def_dwarf_lab(def);
        case labsym.bind of
          AB_GLOBAL:
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create_global(labsym,0));
          AB_LOCAL:
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(labsym,0));
          else
            internalerror(2013082001);
        end;

        { On Darwin, dwarf info is not linked in the final binary,
          but kept in the individual object files. This allows for
          faster linking, but means that you have to keep the object
          files for debugging and also that gdb only loads in the
          debug info of a particular object file once you step into
          or over a procedure in it.

          To solve this, there is a tool called dsymutil which can
          extract all the dwarf info from a program's object files.
          This utility however performs "smart linking" on the dwarf
          info and throws away all unreferenced dwarf entries. Since
          variables' types always point to the dwarfinfo for a tdef
          and never to that for a typesym, this means all debug
          entries generated for typesyms are thrown away.

          The problem with that is that we translate typesyms into
          DW_TAG_typedef, and gdb's dwarf-2 reader only makes types
          globally visibly if they are defined using a DW_TAG_typedef.
          So as a result, before running dsymutil types only become
          available once you stepped into/over a function in the object
          file where they are declared, and after running dsymutil they
          are all gone (printing variables still works because the
          tdef dwarf info is still available, but you cannot typecast
          anything outside the declaring units because the type names
          are not known there).

          The solution: if a tdef has an associated typesym, let the
          debug label for the tdef point to a DW_TAG_typedef instead
          of directly to the tdef itself. And don't write anything
          special for the typesym itself.

          Update: we now also do this for other platforms, because
          otherwise if you compile unit A without debug info and
          use one of its types in unit B, then no typedef will be
          generated and hence gdb will not be able to give a definition
          of the type.
        }

        if is_objc_class_or_protocol(def) then
          begin
            { for Objective-C classes, the typedef must refer to the
              struct itself, not to the pointer of the struct; Objective-C
              classes are not implicit pointers in Objective-C itself, only
              in FPC. So make the def label point to a pointer to the
              typedef, which in turn refers to the actual struct (for Delphi-
              style classes, the def points to the typedef, which refers to
              a pointer to the actual struct) }

            { implicit pointer }
            current_asmdata.getaddrlabel(TAsmLabel(pointer(labsym)));
            append_entry(DW_TAG_pointer_type,false,[]);
            append_labelentry_ref(DW_AT_type,labsym);
            finish_entry;
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(labsym,0));
          end;

        if assigned(def.typesym) and
           not(df_generic in def.defoptions) then
          begin
            current_asmdata.getaddrlabel(TAsmLabel(pointer(labsym)));
            append_entry(DW_TAG_typedef,false,[
              DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0
            ]);
            append_labelentry_ref(DW_AT_type,labsym);
            finish_entry;
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(labsym,0));
          end
      end;


    procedure TDebugInfoDwarf.afterappenddef(list:TAsmList;def:tdef);
      var
        labsym : tasmsymbol;
      begin
        { create a derived reference type for pass-by-reference parameters }
        { (gdb doesn't support DW_AT_variable_parameter yet)               }
        labsym:=def_dwarf_ref_lab(def);
        case labsym.bind of
          AB_GLOBAL:
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create_global(labsym,0));
          AB_LOCAL:
            current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(labsym,0));
          else
            internalerror(2013082002);
        end;
        append_entry(DW_TAG_reference_type,false,[]);
        append_labelentry_ref(DW_AT_type,def_dwarf_lab(def));
        finish_entry;
      end;


    procedure TDebugInfoDwarf.appendprocdef(list:TAsmList; def:tprocdef);

      function dwarf_calling_convention(def: tprocdef): Tdwarf_calling_convention;
        begin
          case def.proccalloption of
            pocall_register:
              result:=DW_CC_GNU_borland_fastcall_i386;
            pocall_cdecl,
            pocall_stdcall,
            pocall_cppdecl,
            pocall_mwpascal:
              result:=DW_CC_normal;
            else
              result:=DW_CC_nocall;
          end
        end;

      var
        procendlabel   : tasmlabel;
        procentry,s    : string;
        cc             : Tdwarf_calling_convention;
        st             : tsymtable;
        vmtoffset      : pint;
        in_currentunit : boolean;
      begin
        { only write debug info for procedures defined in the current module,
          except in case of methods (gcc-compatible)
        }
        in_currentunit:=def.in_currentunit;

        if not in_currentunit and
          not (def.owner.symtabletype in [objectsymtable,recordsymtable]) then
          exit;

        { happens for init procdef of units without init section }
        if in_currentunit and
           not assigned(def.procstarttai) then
          exit;

        { Procdefs are not handled by the regular def writing code, so
          dbg_state is not set/checked for them. Do it here.  }
        if (def.dbg_state in [dbg_state_writing,dbg_state_written]) then
          exit;
        defnumberlist.Add(def);

        { Write methods and only in the scope of their parent objectdefs.  }
        if (def.owner.symtabletype in [objectsymtable,recordsymtable]) then
          begin
            { this code can also work for nested procdefs, but is not yet
              activated for those because there is no clear advantage yet to
              limiting the scope of nested procedures to that of their parent,
              and it makes it impossible to set breakpoints in them by
              referring to their name.  }
            st:=def.owner;
            while assigned(st.defowner) and
                  (tdef(st.defowner).typ = procdef) do
              st:=tprocdef(st.defowner).owner;
            if assigned(st) and
               (tdef(st.defowner).dbg_state<>dbg_state_writing) then
              exit;
         end;

        def.dbg_state:=dbg_state_writing;

        current_asmdata.asmlists[al_dwarf_info].concat(tai_comment.Create(strpnew('Procdef '+def.fullprocname(true))));
        if not is_objc_class_or_protocol(def.struct) then
          append_entry(DW_TAG_subprogram,true,
            [DW_AT_name,DW_FORM_string,symname(def.procsym, false)+#0])
        else
          append_entry(DW_TAG_subprogram,true,
            [DW_AT_name,DW_FORM_string,def.mangledname+#0]);

        if (ds_dwarf_cpp in current_settings.debugswitches) and (def.owner.symtabletype in [objectsymtable,recordsymtable]) then
          begin
            { If C++ emulation is enabled, add DW_AT_linkage_name attribute for methods.
              LLDB uses it to display fully qualified method names.
              Add a simple C++ mangled name without params to achieve at least "Class::Method()"
              instead of just "Method" in LLDB. }
            s:=tabstractrecorddef(def.owner.defowner).objrealname^;
            procentry:=Format('_ZN%d%s', [Length(s), s]);
            s:=symname(def.procsym, false);
            procentry:=Format('%s%d%sEv'#0, [procentry, Length(s), s]);
            append_attribute(DW_AT_linkage_name,DW_FORM_string, [procentry]);
          end;

        append_proc_frame_base(list,def);

        { Append optional flags. }

        { All Pascal procedures are prototyped }
        append_attribute(DW_AT_prototyped,DW_FORM_flag,[true]);
        { Calling convention.  }
        cc:=dwarf_calling_convention(def);
        if (cc<>DW_CC_normal) then
          append_attribute(DW_AT_calling_convention,DW_FORM_data1,[ord(cc)]);
{$ifdef i8086}
        { Call model (near or far). Open Watcom compatible. }
        if tcpuprocdef(def).is_far then
          append_attribute(DW_AT_address_class,DW_FORM_data1,[DW_ADDR_far16])
        else
          append_attribute(DW_AT_address_class,DW_FORM_data1,[DW_ADDR_none]);
{$endif i8086}
        { Externally visible.  }
        if (po_global in def.procoptions) and
           (def.parast.symtablelevel<=normal_function_level) then
          append_attribute(DW_AT_external,DW_FORM_flag,[true]);
        { Abstract or virtual/overriding method.  }
        if (([po_abstractmethod, po_virtualmethod, po_overridingmethod] * def.procoptions) <> []) and
           not is_objc_class_or_protocol(def.struct) and
           not is_objectpascal_helper(def.struct) then
          begin
            if not(po_abstractmethod in def.procoptions) then
              append_attribute(DW_AT_virtuality,DW_FORM_data1,[ord(DW_VIRTUALITY_virtual)])
            else
              append_attribute(DW_AT_virtuality,DW_FORM_data1,[ord(DW_VIRTUALITY_pure_virtual)]);
            { Element number in the vmt (needs to skip stuff coming before the
              actual method addresses in the vmt, so we use vmtmethodoffset()
              and then divide by sizeof(pint)).  }
            vmtoffset:=tobjectdef(def.owner.defowner).vmtmethodoffset(def.extnumber);
            append_attribute(DW_AT_vtable_elem_location,DW_FORM_block1,[3+LengthUleb128(vmtoffset)]);
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_constu)));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.Create_uleb128bit(vmtoffset));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus)));
          end;

        { accessibility: public/private/protected }
        if (def.owner.symtabletype in [objectsymtable,recordsymtable]) then
          append_visibility(def.visibility);

        { Return type.  }
        if not(is_void(tprocdef(def).returndef)) then
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(tprocdef(def).returndef));

        { we can only write the start/end if this procedure is implemented in
          this module
        }
        if in_currentunit then
          begin
            { mark end of procedure }
            current_asmdata.getlabel(procendlabel,alt_dbgtype);
            current_asmdata.asmlists[al_procedures].insertbefore(tai_label.create(procendlabel),def.procendtai);

            if use_dotted_functions then
              procentry := '.' + def.mangledname
            else
              procentry := def.mangledname;

{$ifdef i8086}
            append_seg_name(procentry);
{$endif i8086}
            append_labelentry(DW_AT_low_pc,current_asmdata.RefAsmSymbol(procentry,AT_FUNCTION));
            append_labelentry(DW_AT_high_pc,procendlabel);

            if not(target_info.system in systems_darwin) then
              begin
                current_asmdata.asmlists[al_dwarf_aranges].Concat(
                  tai_const.create_type_sym(aitconst_ptr_unaligned,current_asmdata.RefAsmSymbol(procentry,AT_FUNCTION)));
{$ifdef i8086}
                { bits 16..31 of the offset }
                current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_16bit_unaligned(0));
                { segment }
                current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_seg_name(procentry));
{$endif i8086}
                current_asmdata.asmlists[al_dwarf_aranges].Concat(
                  tai_const.Create_rel_sym(aitconst_ptr_unaligned,current_asmdata.RefAsmSymbol(procentry,AT_FUNCTION),procendlabel));
{$ifdef i8086}
                { bits 16..31 of length }
                current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_16bit_unaligned(0));
{$endif i8086}
              end;
          end;

        { Don't write the funcretsym explicitly, it's also in the
          localsymtable and/or parasymtable.
        }
        finish_entry;

        if assigned(def.parast) then
          begin
            { First insert self, because gdb uses the fact whether or not the
              first parameter of a method is artificial to distinguish static
              from regular methods.  }

            { fortunately, self is the always the first parameter in the
              paralist, since it has the lowest paranr. Note that this is not
              true for Objective-C, but those methods are detected in
              another way (by reading the ObjC run time information)  }
            write_symtable_parasyms(current_asmdata.asmlists[al_dwarf_info],def.paras);
          end;
        { local type defs and vars should not be written
          inside the main proc }
        if in_currentunit and
           assigned(def.localst) and
           (def.localst.symtabletype=localsymtable) then
          write_symtable_syms(current_asmdata.asmlists[al_dwarf_info],def.localst);

        { last write the types from this procdef }
        if assigned(def.parast) then
          write_symtable_defs(current_asmdata.asmlists[al_dwarf_info],def.parast);
        { only try to write the localst if the routine is implemented here }
        if in_currentunit and
           assigned(def.localst) and
           (def.localst.symtabletype=localsymtable) then
          begin
            write_symtable_defs(current_asmdata.asmlists[al_dwarf_info],def.localst);
            { Write nested procedures -- disabled, see scope check at the
              beginning; currently, these are still written in the global
              scope.  }
            // write_symtable_procdefs(current_asmdata.asmlists[al_dwarf_info],def.localst);
          end;

        finish_children;
      end;


    function TDebugInfoDwarf.get_symlist_sym_offset(symlist: ppropaccesslistitem; out sym: tabstractvarsym; out offset: pint): boolean;
      var
        elesize : pint;
        currdef : tdef;
        indirection: boolean;
      begin
        result:=false;
        if not assigned(symlist) then
          exit;
        sym:=nil;
        offset:=0;
        currdef:=nil;
        indirection:=false;
        repeat
          case symlist^.sltype of
            sl_load:
              begin
                if assigned(sym) then
                  internalerror(2009031203);
                if not(symlist^.sym.typ in [paravarsym,localvarsym,staticvarsym,fieldvarsym]) then
                  { can't handle... }
                  exit;
                sym:=tabstractvarsym(symlist^.sym);
                currdef:=tabstractvarsym(sym).vardef;
                if ((sym.typ=paravarsym) and
                    paramanager.push_addr_param(tparavarsym(sym).varspez,sym.vardef,tprocdef(sym.owner.defowner).proccalloption)) then
                  indirection:=true;
              end;
            sl_subscript:
              begin
                if not assigned(currdef) then
                  internalerror(2009031301);
                if (symlist^.sym.typ<>fieldvarsym) then
                  internalerror(2009031202);
                { can't handle offsets with indirections yet }
                if indirection then
                  exit;
                if is_packed_record_or_object(currdef) then
                  begin
                    { can't calculate the address of a non-byte aligned field }
                    if (tfieldvarsym(symlist^.sym).fieldoffset mod 8) <> 0 then
                      exit;
                    inc(offset,tfieldvarsym(symlist^.sym).fieldoffset div 8)
                  end
                else
                  inc(offset,tfieldvarsym(symlist^.sym).fieldoffset);
                currdef:=tfieldvarsym(symlist^.sym).vardef;
              end;
            sl_absolutetype,
            sl_typeconv:
              begin
                currdef:=symlist^.def;
                { ignore, these don't change the address }
              end;
            sl_vec:
              begin
                if not assigned(currdef) or
                   (currdef.typ<>arraydef) then
                  internalerror(2009031201);
                { can't handle offsets with indirections yet }
                if indirection then
                  exit;
                if not is_packed_array(currdef) then
                  elesize:=tarraydef(currdef).elesize
                else
                  begin
                    elesize:=tarraydef(currdef).elepackedbitsize;
                    { can't calculate the address of a non-byte aligned element }
                    if (elesize mod 8)<>0 then
                      exit;
                    elesize:=elesize div 8;
                  end;
                inc(offset,(symlist^.value.svalue-tarraydef(currdef).lowrange)*elesize);
                currdef:=tarraydef(currdef).elementdef;
              end;
            else
              internalerror(2009031401);
          end;
          symlist:=symlist^.next;
        until not assigned(symlist);
        if not assigned(sym) then
          internalerror(2009031205);
        result:=true;
      end;


    procedure TDebugInfoDwarf.appendsym_var(list:TAsmList;sym:tabstractnormalvarsym);
      begin
        appendsym_var_with_name_type_offset(list,sym,symname(sym, false),sym.vardef,0,[]);
      end;


    procedure TDebugInfoDwarf.appendsym_var_with_name_type_offset(list:TAsmList; sym:tabstractnormalvarsym; const name: string; def: tdef; offset: pint; const flags: tdwarfvarsymflags);
      var
        templist : TAsmList;
        blocksize,size_of_int : longint;
        tag : tdwarf_tag;
        has_high_reg : boolean;
        dreg,dreghigh : shortint;
{$ifdef i8086}
        has_segment_sym_name : boolean=false;
        segment_sym_name : TSymStr='';
        segment_reg: TRegister=NR_NO;
{$endif i8086}
      begin
        blocksize:=0;
        dreghigh:=0;
        { external symbols can't be resolved at link time, so we
          can't generate stabs for them

          not sure if this applies to dwarf as well (FK)
        }
        if vo_is_external in sym.varoptions then
          exit;

        { There is no space allocated for not referenced locals }
        if (sym.owner.symtabletype=localsymtable) and (sym.refs=0) then
          exit;

        templist:=TAsmList.create;

        case sym.localloc.loc of
          LOC_REGISTER,
          LOC_CREGISTER,
          LOC_MMREGISTER,
          LOC_CMMREGISTER,
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
              { dwarf_reg_no_error might return -1
                in case the register variable has been optimized out }
              dreg:=dwarf_reg_no_error(sym.localloc.register);
              has_high_reg:=(sym.localloc.loc in [LOC_REGISTER,LOC_CREGISTER]) and (sym.localloc.registerhi<>NR_NO);
              if has_high_reg then
                dreghigh:=dwarf_reg_no_error(sym.localloc.registerhi);
              if dreghigh=-1 then
                has_high_reg:=false;
              if (sym.localloc.loc in [LOC_REGISTER,LOC_CREGISTER]) and
                 (sym.typ=paravarsym) and
                  paramanager.push_addr_param(sym.varspez,sym.vardef,tprocdef(sym.owner.defowner).proccalloption) and
                  not(vo_has_local_copy in sym.varoptions) and
                  not is_open_string(sym.vardef) and (dreg>=0) then
                begin
                  templist.concat(tai_const.create_8bit(ord(DW_OP_bregx)));
                  templist.concat(tai_const.create_uleb128bit(dreg));
                  templist.concat(tai_const.create_sleb128bit(0));
                  blocksize:=1+Lengthuleb128(dreg)+LengthSleb128(0);
                end
              else
                begin
                  if has_high_reg then
                    begin
                      templist.concat(tai_comment.create(strpnew('high:low reg pair variable')));
                      size_of_int:=sizeof(aint);
                      templist.concat(tai_const.create_8bit(ord(DW_OP_regx)));
                      templist.concat(tai_const.create_uleb128bit(dreg));
                      blocksize:=1+Lengthuleb128(dreg);
                      templist.concat(tai_const.create_8bit(ord(DW_OP_piece)));
                      templist.concat(tai_const.create_uleb128bit(size_of_int));
                      blocksize:=blocksize+1+Lengthuleb128(size_of_int);
                      templist.concat(tai_const.create_8bit(ord(DW_OP_regx)));
                      templist.concat(tai_const.create_uleb128bit(dreghigh));
                      blocksize:=blocksize+1+Lengthuleb128(dreghigh);
                      templist.concat(tai_const.create_8bit(ord(DW_OP_piece)));
                      templist.concat(tai_const.create_uleb128bit(size_of_int));
                      blocksize:=blocksize+1+Lengthuleb128(size_of_int);
                    end
                  else if (dreg>=0) then
                    begin
                      templist.concat(tai_const.create_8bit(ord(DW_OP_regx)));
                      templist.concat(tai_const.create_uleb128bit(dreg));
                      blocksize:=1+Lengthuleb128(dreg);
                    end;
                 end;
            end;
          else
            begin
              case sym.typ of
                staticvarsym:
                  begin
                    if vo_is_thread_var in sym.varoptions then
                      begin
                        if tf_section_threadvars in target_info.flags then
                          begin
                            case sizeof(puint) of
                              2:
                                templist.concat(tai_const.create_8bit(ord(DW_OP_const2u)));
                              4:
                                templist.concat(tai_const.create_8bit(ord(DW_OP_const4u)));
                              8:
                                templist.concat(tai_const.create_8bit(ord(DW_OP_const8u)));
                              else
                                Internalerror(2019100501);
                            end;
{$push}
{$warn 6018 off}            { Unreachable code due to compile time evaluation }
                            templist.concat(tai_const.Create_type_name(aitconst_dtpoff,sym.mangledname,0));
                            { so far, aitconst_dtpoff is solely 32 bit }
                            if (sizeof(puint)=8) and (target_info.endian=endian_little) then
                              templist.concat(tai_const.create_32bit(0));
                            templist.concat(tai_const.create_8bit(ord(DW_OP_GNU_push_tls_address)));
                            if (sizeof(puint)=8) and (target_info.endian=endian_big) then
                              templist.concat(tai_const.create_32bit(0));
{$pop}

                            blocksize:=2+sizeof(puint);
                          end
                        else
                          begin
                            { TODO: !!! FIXME: dwarf for thread vars !!!}
                            { This is only a minimal change to at least be able to get a value
                              in only one thread is present PM 2014-11-21, like for stabs format }
                            templist.concat(tai_const.create_8bit(ord(DW_OP_addr)));
                            templist.concat(tai_const.Create_type_name(aitconst_ptr_unaligned,sym.mangledname,
                              offset+sizeof(pint)));
                            blocksize:=1+sizeof(puint);
                          end;
                      end
                    else
                      begin
                        templist.concat(tai_const.create_8bit(ord(DW_OP_addr)));
                        templist.concat(tai_const.Create_type_name(aitconst_ptr_unaligned,sym.mangledname,offset));
                        blocksize:=1+sizeof(puint);
{$ifdef i8086}
                        segment_sym_name:=sym.mangledname;
                        has_segment_sym_name:=true;
{$endif i8086}
                      end;
                  end;
                paravarsym,
                localvarsym:
                  begin
                    { Happens when writing debug info for paras of procdefs not
                      implemented in the current module. Can't add a general check
                      for LOC_INVALID above, because staticvarsyms may also have it.
                    }
                    if sym.localloc.loc<> LOC_INVALID then
                      begin
                        if is_fbreg(sym.localloc.reference.base) then
                          begin
                            templist.concat(tai_const.create_8bit(ord(DW_OP_fbreg)));
                            templist.concat(tai_const.create_sleb128bit(sym.localloc.reference.offset+offset));
                            blocksize:=1+Lengthsleb128(sym.localloc.reference.offset+offset);
                          end
                        else
                          begin
                            dreg:=dwarf_reg(sym.localloc.reference.base);
                            if dreg<=31 then
                              begin
                                templist.concat(tai_const.create_8bit(ord(DW_OP_breg0)+dreg));
                                templist.concat(tai_const.create_sleb128bit(sym.localloc.reference.offset+offset));
                                blocksize:=1+Lengthsleb128(sym.localloc.reference.offset+offset);
                              end
                            else
                              begin
                                templist.concat(tai_const.create_8bit(ord(DW_OP_bregx)));
                                templist.concat(tai_const.create_uleb128bit(dreg));
                                templist.concat(tai_const.create_sleb128bit(sym.localloc.reference.offset+offset));
                                blocksize:=1+Lengthuleb128(dreg)+LengthSleb128(sym.localloc.reference.offset+offset);
                              end;
                          end;
{$ifdef i8086}
                        segment_reg:=sym.localloc.reference.segment;
{$endif i8086}
{$ifndef gdb_supports_DW_AT_variable_parameter}
                        { Parameters which are passed by reference. (var and the like)
                          Hide the reference-pointer and dereference the pointer
                          in the DW_AT_location block.
                        }
                        if (sym.typ=paravarsym) and
                            paramanager.push_addr_param(sym.varspez,sym.vardef,tprocdef(sym.owner.defowner).proccalloption) and
                            not(vo_has_local_copy in sym.varoptions) and
                            not is_open_string(sym.vardef) then
                          begin
                            templist.concat(tai_const.create_8bit(ord(DW_OP_deref)));
                            inc(blocksize);
                          end
{$endif not gdb_supports_DW_AT_variable_parameter}
                      end;
                  end
                else
                  internalerror(200601288);
              end;
            end;
        end;

        { function results must not be added to the parameter list,
          as they are not part of the signature of the function
          (gdb automatically adds them according to the ABI specifications
           when calling the function)
        }
        if (sym.typ=paravarsym) and
           not(dvf_force_local_var in flags) and
           not(vo_is_funcret in sym.varoptions) then
          tag:=DW_TAG_formal_parameter
        else
          tag:=DW_TAG_variable;

        { must be parasym of externally implemented procdef, but
          the parasymtable can con also contain e.g. absolutevarsyms
          -> check symtabletype}
        if (sym.owner.symtabletype=parasymtable) and
           (sym.localloc.loc=LOC_INVALID) then
          begin
            if (sym.owner.symtabletype<>parasymtable) then
              internalerror(2009101001);
            append_entry(tag,false,[
              DW_AT_name,DW_FORM_string,name+#0
              {
              DW_AT_decl_file,DW_FORM_data1,0,
              DW_AT_decl_line,DW_FORM_data1,
              }
              ])
          end
        else if not(sym.localloc.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_MMREGISTER,
                                 LOC_CMMREGISTER,LOC_FPUREGISTER,LOC_CFPUREGISTER]) and
           ((sym.owner.symtabletype = globalsymtable) or
            (sp_static in sym.symoptions) or
            (vo_is_public in sym.varoptions)) then
          append_entry(tag,false,[
            DW_AT_name,DW_FORM_string,name+#0,
            {
            DW_AT_decl_file,DW_FORM_data1,0,
            DW_AT_decl_line,DW_FORM_data1,
            }
            DW_AT_external,DW_FORM_flag,true,
            { data continues below }
            DW_AT_location,DW_FORM_block1,blocksize
            ])
{$ifdef gdb_supports_DW_AT_variable_parameter}
        else if (sym.typ=paravarsym) and
            paramanager.push_addr_param(sym.varspez,sym.vardef,tprocdef(sym.owner.defowner).proccalloption) and
            not(vo_has_local_copy in sym.varoptions) and
            not is_open_string(sym.vardef) then
          append_entry(tag,false,[
            DW_AT_name,DW_FORM_string,name+#0,
            DW_AT_variable_parameter,DW_FORM_flag,true,
            {
            DW_AT_decl_file,DW_FORM_data1,0,
            DW_AT_decl_line,DW_FORM_data1,
            }
            { data continues below }
            DW_AT_location,DW_FORM_block1,blocksize
            ])
{$endif gdb_supports_DW_AT_variable_parameter}
        else
          append_entry(tag,false,[
            DW_AT_name,DW_FORM_string,name+#0,
            {
            DW_AT_decl_file,DW_FORM_data1,0,
            DW_AT_decl_line,DW_FORM_data1,
            }
            { data continues below }
            DW_AT_location,DW_FORM_block1,blocksize
            ]);
        { append block data }
        current_asmdata.asmlists[al_dwarf_info].concatlist(templist);
        { Mark self as artificial for methods, because gdb uses the fact
          whether or not the first parameter of a method is artificial to
          distinguish regular from static methods (since there are no
          no vo_is_self parameters for static methods, we don't have to check
          that).  }
        if (vo_is_self in sym.varoptions) then
          append_attribute(DW_AT_artificial,DW_FORM_flag,[true]);
        append_labelentry_ref(DW_AT_type,def_dwarf_lab(def));
{$ifdef i8086}
        if has_segment_sym_name then
          append_seg_name(segment_sym_name)
        else if segment_reg<>NR_NO then
          append_seg_reg(segment_reg);
{$endif i8086}

        templist.free;

        finish_entry;
      end;


    procedure TDebugInfoDwarf.appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);
      begin
        appendsym_var(list,sym);
      end;


    procedure TDebugInfoDwarf.appendsym_localvar(list:TAsmList;sym:tlocalvarsym);
      begin
        appendsym_var(list,sym);
      end;


    procedure TDebugInfoDwarf.appendsym_paravar(list:TAsmList;sym:tparavarsym);
      begin
        appendsym_var(list,sym);
      end;


    procedure TDebugInfoDwarf.appendsym_fieldvar(list:TAsmList;sym: tfieldvarsym);
      begin
        appendsym_fieldvar_with_name_offset(list,sym,symname(sym, false),sym.vardef,0);
      end;


    procedure TDebugInfoDwarf.appendsym_fieldvar_with_name_offset(list:TAsmList;sym: tfieldvarsym;const name: string; def: tdef; offset: pint);
      var
        bitoffset,
        fieldoffset,
        fieldnatsize: asizeint;
      begin
        if (sp_static in sym.symoptions) or
           (sym.visibility=vis_hidden) then
          exit;

        if (tabstractrecordsymtable(sym.owner).usefieldalignment<>bit_alignment) or
           { only ordinals are bitpacked }
           not is_ordinal(sym.vardef) then
          begin
            { other kinds of fields can however also appear in a bitpacked   }
            { record, and then their offset is also specified in bits rather }
            { than in bytes                                                  }
            if (tabstractrecordsymtable(sym.owner).usefieldalignment<>bit_alignment) then
              fieldoffset:=sym.fieldoffset
            else
              fieldoffset:=sym.fieldoffset div 8;
            inc(fieldoffset,offset);
            append_entry(DW_TAG_member,false,[
              DW_AT_name,DW_FORM_string,name+#0,
              DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(fieldoffset)
              ]);
          end
        else
          begin
            if (sym.vardef.packedbitsize > 255) then
              internalerror(2007061201);

            { we don't bitpack according to the ABI, but as close as }
            { possible, i.e., equivalent to gcc's                    }
            { __attribute__((__packed__)), which is also what gpc    }
            { does.                                                  }
            fieldnatsize:=max(sizeof(pint),sym.vardef.size);
            fieldoffset:=(sym.fieldoffset div (fieldnatsize*8)) * fieldnatsize;
            inc(fieldoffset,offset);
            bitoffset:=sym.fieldoffset mod (fieldnatsize*8);
            if (target_info.endian=endian_little) then
              bitoffset:=(fieldnatsize*8)-bitoffset-sym.vardef.packedbitsize;
            append_entry(DW_TAG_member,false,[
              DW_AT_name,DW_FORM_string,symname(sym, false)+#0,
              { gcc also generates both a bit and byte size attribute }
              { we don't support ordinals >= 256 bits }
              DW_AT_byte_size,DW_FORM_data1,fieldnatsize,
              { nor >= 256 bits (not yet, anyway, see IE above) }
              DW_AT_bit_size,DW_FORM_data1,sym.vardef.packedbitsize,
              { data1 and data2 are unsigned, bitoffset can also be negative }
              DW_AT_bit_offset,DW_FORM_data4,bitoffset,
              DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(fieldoffset)
              ]);
          end;
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(fieldoffset));
        if (sym.owner.symtabletype in [objectsymtable,recordsymtable]) then
          append_visibility(sym.visibility);

        append_labelentry_ref(DW_AT_type,def_dwarf_lab(def));
        finish_entry;
      end;

    procedure TDebugInfoDwarf.appendsym_const(list:TAsmList;sym:tconstsym);
    begin
      appendsym_const_member(list,sym,false);
    end;

    procedure TDebugInfoDwarf.appendsym_const_member(list:TAsmList;sym:tconstsym;ismember:boolean);
      var
        i,
        size: aint;
        usedef: tdef;
      begin
        { These are default values of parameters. These should be encoded
          via DW_AT_default_value, not as a separate sym. Moreover, their
          type is not available when writing the debug info for external
          procedures.
        }
        if (sym.owner.symtabletype=parasymtable) then
          exit;

        if ismember then
          append_entry(DW_TAG_member,false,[
            DW_AT_name,DW_FORM_string,symname(sym, false)+#0,
          { The DW_AT_declaration tag is invalid according to the DWARF specifications.
            But gcc adds this to static const members and gdb checks
            for this flag. So we have to set it also.
          }
            DW_AT_declaration,DW_FORM_flag,true,
            DW_AT_external,DW_FORM_flag,true
            ])
        else
          append_entry(DW_TAG_variable,false,[
            DW_AT_name,DW_FORM_string,symname(sym, false)+#0
            ]);
        { for string constants, constdef isn't set because they have no real type }
        case sym.consttyp of
          conststring:
            begin
              { if DW_FORM_string is used below one day, this usedef should
                probably become nil }
              { note: < 255 instead of <= 255 because we have to store the
                entire length of the string as well, and 256 does not fit in
                a byte }
              if (sym.value.len<255) then
                usedef:=cshortstringtype
              else
                usedef:=clongstringtype;
            end;
          constresourcestring,
          constwstring:
            usedef:=nil;
          else
            usedef:=sym.constdef;
          end;
        if assigned(usedef) then
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(usedef));
        AddConstToAbbrev(ord(DW_AT_const_value));
        case sym.consttyp of
          conststring:
            begin
              { DW_FORM_string isn't supported yet by the Pascal value printer
                -> create a string using raw bytes }
              if (sym.value.len<255) then
                begin
                  AddConstToAbbrev(ord(DW_FORM_block1));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(sym.value.len+1));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(sym.value.len));
                end
              else
                begin
                  AddConstToAbbrev(ord(DW_FORM_block));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(sym.value.len+sizesinttype.size));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.Create_sizeint_unaligned(sym.value.len));
                end;
              i:=0;
              size:=sym.value.len;
              while(i<size) do
                begin
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit((pbyte(sym.value.valueptr+i)^)));
                  inc(i);
                end;
            end;
          constguid,
          constset:
            begin
              AddConstToAbbrev(ord(DW_FORM_block1));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(usedef.size));
              i:=0;
              size:=sym.constdef.size;
              while (i<size) do
                begin
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit((pbyte(sym.value.valueptr+i)^)));
                  inc(i);
                end;
            end;
          constwstring,
          constresourcestring:
            begin
              { write dummy for now }
              AddConstToAbbrev(ord(DW_FORM_string));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_string.create(''));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(0));
            end;
          constord:
            begin
              if (sym.value.valueord<0) then
                begin
                  AddConstToAbbrev(ord(DW_FORM_sdata));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_sleb128bit(sym.value.valueord.svalue));
                end
              else
                begin
                  AddConstToAbbrev(ord(DW_FORM_udata));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(sym.value.valueord.uvalue));
                end;
            end;
          constnil:
            begin
{$ifdef cpu64bitaddr}
              AddConstToAbbrev(ord(DW_FORM_data8));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_64bit_unaligned(0));
{$else cpu64bitaddr}
              AddConstToAbbrev(ord(DW_FORM_data4));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_32bit_unaligned(0));
{$endif cpu64bitaddr}
            end;
          constpointer:
            begin
{$ifdef cpu64bitaddr}
              AddConstToAbbrev(ord(DW_FORM_data8));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_64bit_unaligned(int64(sym.value.valueordptr)));
{$else cpu64bitaddr}
              AddConstToAbbrev(ord(DW_FORM_data4));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_32bit_unaligned(longint(sym.value.valueordptr)));
{$endif cpu64bitaddr}
            end;
          constreal:
            begin
              AddConstToAbbrev(ord(DW_FORM_block1));
              case tfloatdef(sym.constdef).floattype of
                s32real:
                  begin
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(4));
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_realconst.create_s32real(pbestreal(sym.value.valueptr)^));
                  end;
                s64real:
                  begin
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(8));
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_realconst.create_s64real(pbestreal(sym.value.valueptr)^));
                  end;
                s64comp,
                s64currency:
                  begin
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(8));
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_64bit_unaligned(trunc(pbestreal(sym.value.valueptr)^)));
                  end;
                s80real,
                sc80real:
                  begin
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(sym.constdef.size));
                    current_asmdata.asmlists[al_dwarf_info].concat(tai_realconst.create_s80real(pextended(sym.value.valueptr)^,sym.constdef.size));
                  end;
                else
                  internalerror(200601291);
              end;
            end;
          else
            internalerror(200601292);
        end;
        finish_entry;
      end;


    procedure TDebugInfoDwarf.appendsym_label(list:TAsmList;sym: tlabelsym);
      begin
        { ignore label syms for now, the problem is that a label sym
          can have more than one label associated e.g. in case of
          an inline procedure expansion }
      end;


    procedure TDebugInfoDwarf.appendsym_property(list:TAsmList;sym: tpropertysym);
      var
        symlist: ppropaccesslistitem;
        tosym: tabstractvarsym;
        offset: pint;
      begin
        if assigned(sym.propaccesslist[palt_read]) and
           not assigned(sym.propaccesslist[palt_read].procdef) then
          symlist:=sym.propaccesslist[palt_read].firstsym
        else
          { can't handle }
          exit;

        if not get_symlist_sym_offset(symlist,tosym,offset) then
          exit;

        if not (tosym.owner.symtabletype in [objectsymtable,recordsymtable]) then
          begin
            if (tosym.typ=fieldvarsym) then
              internalerror(2009031404);
            appendsym_var_with_name_type_offset(list,tabstractnormalvarsym(tosym),symname(sym, false),sym.propdef,offset,[])
          end
        else
          appendsym_fieldvar_with_name_offset(list,tfieldvarsym(tosym),symname(sym, false),sym.propdef,offset)
      end;


    function TDebugInfoDwarf.symdebugname(sym: tsym): String;
    begin
      result := sym.name;
    end;


    procedure TDebugInfoDwarf.appendsym_type(list:TAsmList;sym: ttypesym);
      begin
        { just queue the def if needed, beforeappenddef will
          emit the typedef if necessary }
        def_dwarf_lab(sym.typedef);
      end;


    procedure TDebugInfoDwarf.appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);
      var
        templist : TAsmList;
        blocksize : longint;
        symlist : ppropaccesslistitem;
        tosym: tabstractvarsym;
        offset: pint;
        flags: tdwarfvarsymflags;
      begin
        templist:=TAsmList.create;
        case tabsolutevarsym(sym).abstyp of
          toaddr :
            begin
               { MWE: replaced ifdef i368 }
               (*
               if target_cpu = cpu_i386 then
                 begin
                  { in theory, we could write a DW_AT_segment entry here for sym.absseg,
                    however I doubt that gdb supports this (FK) }
                 end;
               *)
               templist.concat(tai_const.create_8bit(3));
               {$ifdef avr}
               // Add $800000 to indicate that the address is in memory space
               templist.concat(tai_const.create_int_dataptr_unaligned(sym.addroffset + $800000, aitconst_ptr_unaligned));
               {$else}
               templist.concat(tai_const.create_int_dataptr_unaligned(sym.addroffset));
               {$endif}
               blocksize:=1+sizeof(puint);
            end;
          toasm :
            begin
              templist.concat(tai_const.create_8bit(3));
              templist.concat(tai_const.create_type_name(aitconst_ptr_unaligned,sym.mangledname,0));
              blocksize:=1+sizeof(puint);
            end;
          tovar:
            begin
              symlist:=tabsolutevarsym(sym).ref.firstsym;
              if get_symlist_sym_offset(symlist,tosym,offset) then
                begin
                  if (tosym.typ=fieldvarsym) then
                    internalerror(2009031402);
                  flags:=[];
                  if (sym.owner.symtabletype=localsymtable) then
                    include(flags,dvf_force_local_var);
                  appendsym_var_with_name_type_offset(list,tabstractnormalvarsym(tosym),symname(sym, false),tabstractvarsym(sym).vardef,offset,flags);
                end;
              templist.free;
              exit;
            end;
        end;

        append_entry(DW_TAG_variable,false,[
          DW_AT_name,DW_FORM_string,symname(sym, false)+#0,
          {
          DW_AT_decl_file,DW_FORM_data1,0,
          DW_AT_decl_line,DW_FORM_data1,
          }
          DW_AT_external,DW_FORM_flag,true,
          { data continues below }
          DW_AT_location,DW_FORM_block1,blocksize
          ]);
        { append block data }
        current_asmdata.asmlists[al_dwarf_info].concatlist(templist);
        append_labelentry_ref(DW_AT_type,def_dwarf_lab(sym.vardef));

        templist.free;

        finish_entry;
      end;


    procedure TDebugInfoDwarf.beforeappendsym(list:TAsmList;sym:tsym);
      begin
        current_asmdata.asmlists[al_dwarf_info].concat(tai_comment.Create(strpnew('Symbol '+symname(sym, true))));
      end;


    procedure TDebugInfoDwarf.insertmoduleinfo;
      var
        templist: TAsmList;
        linelist: TAsmList;
        lbl   : tasmlabel;
        n,m   : Integer;
        ditem : TDirIndexItem;
        fitem : TFileIndexItem;
        flist : TFPList;
        dbgname : String;
      begin
        { insert DEBUGSTART and DEBUGEND labels }
        dbgname:=make_mangledname('DEBUGSTART',current_module.localsymtable,'');
        { Darwin's linker does not like two global labels both pointing to the
          end of a section, which can happen in case of units without code ->
          make them local; we don't need the debugtable stuff there either,
          so it doesn't matter that they are not global.
        }
        if (target_info.system in systems_darwin) then
          dbgname:='L'+dbgname;
        new_section(current_asmdata.asmlists[al_start],sec_code,dbgname,0,secorder_begin);
        if not(target_info.system in systems_darwin) then
          current_asmdata.asmlists[al_start].concat(tai_symbol.Createname_global(dbgname,AT_METADATA,0,voidpointertype))
        else
          current_asmdata.asmlists[al_start].concat(tai_symbol.Createname(dbgname,AT_METADATA,0,voidpointertype));

        dbgname:=make_mangledname('DEBUGEND',current_module.localsymtable,'');
        { See above. }
        if (target_info.system in systems_darwin) then
          dbgname:='L'+dbgname;
        new_section(current_asmdata.asmlists[al_end],sec_code,dbgname,0,secorder_end);
        if not(target_info.system in systems_darwin) then
          current_asmdata.asmlists[al_end].concat(tai_symbol.Createname_global(dbgname,AT_METADATA,0,voidpointertype))
        else
          current_asmdata.asmlists[al_end].concat(tai_symbol.Createname(dbgname,AT_METADATA,0,voidpointertype));

        { insert .Ldebug_abbrev0 label }
        templist:=TAsmList.create;
        new_section(templist,sec_debug_abbrev,'',0);
        templist.concat(tai_symbol.createname(target_asm.labelprefix+'debug_abbrevsection0',AT_METADATA,0,voidpointertype));
        { add any extra stuff which needs to be in the abbrev section, but before    }
        { the actual abbreviations, in between the symbol above and below, i.e. here }
        templist.concat(tai_symbol.createname(target_asm.labelprefix+'debug_abbrev0',AT_METADATA,0,voidpointertype));
        current_asmdata.asmlists[al_start].insertlist(templist);
        templist.free;

        { insert .Ldebug_line0 label }
        templist:=TAsmList.create;
        new_section(templist,sec_debug_line,'',0);
        templist.concat(tai_symbol.createname(target_asm.labelprefix+'debug_linesection0',AT_METADATA,0,voidpointertype));
        { add any extra stuff which needs to be in the line section, but before  }
        { the actual line info, in between the symbol above and below, i.e. here }
        templist.concat(tai_symbol.createname(target_asm.labelprefix+'debug_line0',AT_METADATA,0,voidpointertype));
        current_asmdata.asmlists[al_start].insertlist(templist);
        templist.free;

        { finalize line info if the unit doesn't contain any function/ }
        { procedure/init/final code                                    }
        finish_lineinfo;

        { debug line header }
        linelist := current_asmdata.asmlists[al_dwarf_line];
        new_section(linelist,sec_debug_line,'',0);
        linelist.concat(tai_comment.Create(strpnew('=== header start ===')));

        { size }
        current_asmdata.getlabel(lbl,alt_dbgfile);
        if use_64bit_headers then
          linelist.concat(tai_const.create_32bit_unaligned(longint($FFFFFFFF)));
        linelist.concat(tai_const.create_rel_sym(offsetreltype,
          lbl,current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'edebug_line0',AB_LOCAL,AT_METADATA,voidpointertype)));
        linelist.concat(tai_label.create(lbl));

        { version }
        linelist.concat(tai_const.create_16bit_unaligned(dwarf_version));

        { header length }
        current_asmdata.getlabel(lbl,alt_dbgfile);
        linelist.concat(tai_const.create_rel_sym(offsetreltype,
          lbl,current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'ehdebug_line0',AB_LOCAL,AT_METADATA,voidpointertype)));
        linelist.concat(tai_label.create(lbl));

        { minimum_instruction_length }
        linelist.concat(tai_const.create_8bit(1));

        { default_is_stmt }
        linelist.concat(tai_const.create_8bit(1));

        { line_base }
        linelist.concat(tai_const.create_8bit(LINE_BASE));

        { line_range }
        { only line increase, no adress }
        linelist.concat(tai_const.create_8bit(255));

        { opcode_base }
        linelist.concat(tai_const.create_8bit(OPCODE_BASE));

        { standard_opcode_lengths }
        { MWE: sigh... why adding the default lengths (and make those sizes sense with LEB encoding) }
          { DW_LNS_copy }
        linelist.concat(tai_const.create_8bit(0));
          { DW_LNS_advance_pc }
        linelist.concat(tai_const.create_8bit(1));
          { DW_LNS_advance_line }
        linelist.concat(tai_const.create_8bit(1));
          { DW_LNS_set_file }
        linelist.concat(tai_const.create_8bit(1));
          { DW_LNS_set_column }
        linelist.concat(tai_const.create_8bit(1));
          { DW_LNS_negate_stmt }
        linelist.concat(tai_const.create_8bit(0));
          { DW_LNS_set_basic_block }
        linelist.concat(tai_const.create_8bit(0));
          { DW_LNS_const_add_pc }
        linelist.concat(tai_const.create_8bit(0));
          { DW_LNS_fixed_advance_pc }
        linelist.concat(tai_const.create_8bit(1));
          { DW_LNS_set_prologue_end }
        linelist.concat(tai_const.create_8bit(0));
          { DW_LNS_set_epilogue_begin }
        linelist.concat(tai_const.create_8bit(0));
          { DW_LNS_set_isa }
        linelist.concat(tai_const.create_8bit(1));

        { Create single list of filenames sorted in IndexNr }
        flist:=TFPList.Create;
        for n := 0 to dirlist.Count - 1 do
          begin
            ditem := TDirIndexItem(dirlist[n]);
            for m := 0 to ditem.Files.Count - 1 do
              flist.Add(ditem.Files[m]);
          end;
        flist.Sort(@FileListSortCompare);

        { include_directories }
        linelist.concat(tai_comment.Create(strpnew('include_directories')));
        for n := 0 to dirlist.Count - 1 do
          begin
            ditem := TDirIndexItem(dirlist[n]);
            if ditem.Name = '.' then
              Continue;
            { Write without trailing path delimiter and also don't prefix with ./ for current dir (already done while adding to dirlist }

            linelist.concat(tai_string.create(ditem.Name+#0));
          end;
        linelist.concat(tai_const.create_8bit(0));

        { file_names }
        linelist.concat(tai_comment.Create(strpnew('file_names')));
        for n := 0 to flist.Count - 1 do
          begin
            fitem := TFileIndexItem(flist[n]);
            { file name }
            linelist.concat(tai_string.create(fitem.Name+#0));
            { directory index }
            linelist.concat(tai_const.create_uleb128bit(fitem.DirIndex));
            { last modification }
            linelist.concat(tai_const.create_uleb128bit(0));
            { file length }
            linelist.concat(tai_const.create_uleb128bit(0));
          end;
        linelist.concat(tai_const.create_8bit(0));

        { end of debug line header }
        linelist.concat(tai_symbol.createname(target_asm.labelprefix+'ehdebug_line0',AT_METADATA,0,voidpointertype));
        linelist.concat(tai_comment.Create(strpnew('=== header end ===')));

        { add line program }
        linelist.concatList(asmline);

        { end of debug line table }
        linelist.concat(tai_symbol.createname(target_asm.labelprefix+'edebug_line0',AT_METADATA,0,voidpointertype));

        flist.free;
      end;


    procedure TDebugInfoDwarf.inserttypeinfo;


      var
        storefilepos  : tfileposinfo;
        lenstartlabel,arangestartlabel: tasmlabel;
        i : longint;
        def: tdef;
        dbgname: string;
        vardatatype: ttypesym;
        bind: tasmsymbind;
        lang: tdwarf_source_language;
      begin
        include(current_module.moduleflags,mf_has_dwarf_debuginfo);
        storefilepos:=current_filepos;
        current_filepos:=current_module.mainfilepos;

        if assigned(dwarflabels) then
          internalerror(2015100301);
        { one item per def, plus some extra space in case of nested types,
          externally used types etc (it will grow further if necessary) }
        i:=current_module.localsymtable.DefList.count*4;
        if assigned(current_module.globalsymtable) then
          inc(i,current_module.globalsymtable.DefList.count*2);
        dwarflabels:=TDwarfLabHashSet.Create(i,true,false);

        currabbrevnumber:=0;

        defnumberlist:=TFPObjectList.create(false);
        deftowritelist:=TFPObjectList.create(false);

        { not exported (FK)
            FILEREC
            TEXTREC
        }
        vardatatype:=try_search_system_type('TVARDATA');
        if assigned(vardatatype) then
          vardatadef:=trecorddef(vardatatype.typedef);

        { write start labels }
        new_section(current_asmdata.asmlists[al_dwarf_info],sec_debug_info,'',0);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.createname(target_asm.labelprefix+'debug_info0',AT_METADATA,0,voidpointertype));

        { start abbrev section }
        new_section(current_asmdata.asmlists[al_dwarf_abbrev],sec_debug_abbrev,'',0);

        if not(target_info.system in systems_darwin) then
          begin
            { start aranges section }
            new_section(current_asmdata.asmlists[al_dwarf_aranges],sec_debug_aranges,'',0);

            current_asmdata.getlabel(arangestartlabel,alt_dbgfile);

            if use_64bit_headers then
              current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_32bit_unaligned(longint($FFFFFFFF)));
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_rel_sym(offsetreltype,
              arangestartlabel,current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'earanges0',AB_LOCAL,AT_METADATA,voidpointertype)));

            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_label.create(arangestartlabel));

            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_16bit_unaligned(2));

            if not(tf_dwarf_relative_addresses in target_info.flags) then
              current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_type_sym(offsetabstype,
                current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_info0',AB_LOCAL,AT_METADATA,voidpointertype)))
            else
              current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_rel_sym(offsetreltype,
                current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_infosection0',AB_LOCAL,AT_METADATA,voidpointertype),
                current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_info0',AB_LOCAL,AT_METADATA,voidpointertype)));

{$ifdef i8086}
            { address_size }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_8bit(4));
            { segment_size }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_8bit(2));
            { no alignment/padding bytes on i8086 for Open Watcom compatibility }
{$else i8086}
            { address_size }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_8bit(sizeof(pint)));
            { segment_size }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_8bit(0));
            { alignment }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.create_32bit_unaligned(0));
{$endif i8086}

            { start ranges section }
            new_section(current_asmdata.asmlists[al_dwarf_ranges],sec_debug_ranges,'',0);
          end;

        { debug info header }
        current_asmdata.getlabel(lenstartlabel,alt_dbgfile);
        { size }
        if use_64bit_headers then
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_32bit_unaligned(longint($FFFFFFFF)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_rel_sym(offsetreltype,
          lenstartlabel,current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'edebug_info0',AB_LOCAL,AT_METADATA,voidpointertype)));

        current_asmdata.asmlists[al_dwarf_info].concat(tai_label.create(lenstartlabel));
        { version }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(dwarf_version));
        { abbrev table (=relative from section start)}
        if not(tf_dwarf_relative_addresses in target_info.flags) then
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_type_sym(offsetabstype,
            current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_abbrev0',AB_LOCAL,AT_METADATA,voidpointertype)))
        else
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_rel_sym(offsetreltype,
            current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_abbrevsection0',AB_LOCAL,AT_METADATA,voidpointertype),
            current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_abbrev0',AB_LOCAL,AT_METADATA,voidpointertype)));

        { address size }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(sizeof(pint)));

        if (ds_dwarf_cpp in current_settings.debugswitches) then
          lang:=DW_LANG_C_plus_plus
        else
          lang:=DW_LANG_Pascal83;
        { first manadatory compilation unit TAG }
        append_entry(DW_TAG_compile_unit,true,[
          DW_AT_name,DW_FORM_string,relative_dwarf_path(current_module.sourcefiles.get_file(1).path+current_module.sourcefiles.get_file(1).name)+#0,
          DW_AT_producer,DW_FORM_string,'Free Pascal '+full_version_string+' '+date_string+#0,
          DW_AT_comp_dir,DW_FORM_string,BSToSlash(FixPath(GetCurrentDir,false))+#0,
          DW_AT_language,DW_FORM_data1,lang,
          DW_AT_identifier_case,DW_FORM_data1,DW_ID_case_insensitive]);

{$ifdef i8086}
        case current_settings.x86memorymodel of
          mm_tiny,
          mm_small:
            append_attribute(DW_AT_WATCOM_memory_model,DW_FORM_data1,[DW_WATCOM_MEMORY_MODEL_small]);
          mm_medium:
            append_attribute(DW_AT_WATCOM_memory_model,DW_FORM_data1,[DW_WATCOM_MEMORY_MODEL_medium]);
          mm_compact:
            append_attribute(DW_AT_WATCOM_memory_model,DW_FORM_data1,[DW_WATCOM_MEMORY_MODEL_compact]);
          mm_large:
            append_attribute(DW_AT_WATCOM_memory_model,DW_FORM_data1,[DW_WATCOM_MEMORY_MODEL_large]);
          mm_huge:
            append_attribute(DW_AT_WATCOM_memory_model,DW_FORM_data1,[DW_WATCOM_MEMORY_MODEL_huge]);
        end;
{$endif i8086}

        { reference to line info section }
        if not(tf_dwarf_relative_addresses in target_info.flags) then
          append_labelentry_dataptr_abs(DW_AT_stmt_list,current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_line0',AB_LOCAL,AT_METADATA,voidpointertype))
        else
          append_labelentry_dataptr_rel(DW_AT_stmt_list,
            current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_linesection0',AB_LOCAL,AT_METADATA,voidpointertype),
            current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'debug_line0',AB_LOCAL,AT_METADATA,voidpointertype));

        if (m_objectivec1 in current_settings.modeswitches) then
          append_attribute(DW_AT_APPLE_major_runtime_vers,DW_FORM_data1,[1]);

        dbgname:=make_mangledname('DEBUGSTART',current_module.localsymtable,'');
        if (target_info.system in systems_darwin) then
          begin
            bind:=AB_LOCAL;
            dbgname:='L'+dbgname;
          end
        else
          bind:=AB_GLOBAL;
        append_labelentry(DW_AT_low_pc,current_asmdata.DefineAsmSymbol(dbgname,bind,AT_METADATA,voidpointertype));
        dbgname:=make_mangledname('DEBUGEND',current_module.localsymtable,'');
        if (target_info.system in systems_darwin) then
          dbgname:='L'+dbgname;
        append_labelentry(DW_AT_high_pc,current_asmdata.DefineAsmSymbol(dbgname,bind,AT_METADATA,voidpointertype));

        finish_entry;

        { write all global/local variables. This will flag all required tdefs  }
        if assigned(current_module.globalsymtable) then
          write_symtable_syms(current_asmdata.asmlists[al_dwarf_info],current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_syms(current_asmdata.asmlists[al_dwarf_info],current_module.localsymtable);

        { write all procedures and methods. This will flag all required tdefs }
        if assigned(current_module.globalsymtable) then
          write_symtable_procdefs(current_asmdata.asmlists[al_dwarf_info],current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_procdefs(current_asmdata.asmlists[al_dwarf_info],current_module.localsymtable);

        { reset unit type info flag }
        reset_unit_type_info;

        { write used types from the used units }
        write_used_unit_type_info(current_asmdata.asmlists[al_dwarf_info],current_module);

        { last write the types from this unit }
        if assigned(current_module.globalsymtable) then
          write_symtable_defs(current_asmdata.asmlists[al_dwarf_info],current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_defs(current_asmdata.asmlists[al_dwarf_info],current_module.localsymtable);

        { write defs not written yet }
        write_remaining_defs_to_write(current_asmdata.asmlists[al_dwarf_info]);

        { close compilation unit entry }
        finish_children;

        { end of debug info table }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.createname(target_asm.labelprefix+'edebug_info0',AT_METADATA,0,voidpointertype));

        { end of abbrev table }
        current_asmdata.asmlists[al_dwarf_abbrev].concat(tai_const.create_8bit(0));

        if not(target_info.system in systems_darwin) then
          begin
            { end of aranges table }
{$ifdef i8086}
            { 32-bit offset }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_32bit_unaligned(0));
            { 16-bit segment }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_16bit_unaligned(0));
            { 32-bit length }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_32bit_unaligned(0));
{$else i8086}
            { offset }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_aint(0));
            { length }
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_const.Create_aint(0));
{$endif i8086}
            current_asmdata.asmlists[al_dwarf_aranges].concat(tai_symbol.createname(target_asm.labelprefix+'earanges0',AT_METADATA,0,voidpointertype));
          end;

        { reset all def debug states }
        for i:=0 to defnumberlist.count-1 do
          begin
            def := tdef(defnumberlist[i]);
            if assigned(def) then
              def.dbg_state:=dbg_state_unused;
          end;
        dwarflabels.free;
        dwarflabels:=nil;

        defnumberlist.free;
        defnumberlist:=nil;
        deftowritelist.free;
        deftowritelist:=nil;

        current_filepos:=storefilepos;
      end;


    procedure TDebugInfoDwarf.referencesections(list:TAsmList);
      var
        hp : tmodule;
      begin
        { Reference all DEBUGINFO sections from the main .fpc section }
        { to prevent eliminating them by smartlinking                 }
        if (target_info.system in ([system_powerpc_macos]+systems_darwin)) then
          exit;
        new_section(list,sec_fpc,'links',0);

        { include reference to all debuginfo sections of used units }
        hp:=tmodule(loaded_units.first);
        while assigned(hp) do
          begin
            If (mf_has_dwarf_debuginfo in hp.moduleflags) and not assigned(hp.package) then
              begin
                list.concat(Tai_const.Createname(make_mangledname('DEBUGSTART',hp.localsymtable,''),0));
                list.concat(Tai_const.Createname(make_mangledname('DEBUGEND',hp.localsymtable,''),0));
              end;
            hp:=tmodule(hp.next);
          end;
      end;


    function TDebugInfoDwarf.symname(sym: tsym; manglename: boolean): String;
      begin
        if (sym.typ=paravarsym) and
           (vo_is_self in tparavarsym(sym).varoptions) then
          { We use 'this' for regular methods because that's what gdb triggers
            on to automatically search fields. Don't do this for class methods,
            because search class fields is not supported, and gdb 7.0+ fails
            in this case because "this" is not a record in that case (it's a
            pointer to a vmt) }
          if not is_objc_class_or_protocol(tdef(sym.owner.defowner.owner.defowner)) and
             not(po_classmethod in tabstractprocdef(sym.owner.defowner).procoptions) then
            result:='this'
          else
            result:='self'
        else if (sym.typ=typesym) and
                is_objc_class_or_protocol(ttypesym(sym).typedef) then
          result:=tobjectdef(ttypesym(sym).typedef).objextname^
        else if (ds_dwarf_method_class_prefix in current_settings.debugswitches) and
                (sym.typ=procsym) and
                (tprocsym(sym).owner.symtabletype in [objectsymtable,recordsymtable]) then
          begin
            result:=tprocsym(sym).owner.name^+'__';
            if manglename then
              result := result + sym.name
            else
              result := result + symdebugname(sym);
          end
        else
          begin
            if manglename then
              result := sym.name
            else
              result := symdebugname(sym);
          end;
      end;


        procedure TDebugInfoDwarf.append_visibility(vis: tvisibility);
      begin
        case vis of
          vis_hidden,
          vis_private,
          vis_strictprivate:
            append_attribute(DW_AT_accessibility,DW_FORM_data1,[ord(DW_ACCESS_private)]);
          vis_protected,
          vis_strictprotected:
            append_attribute(DW_AT_accessibility,DW_FORM_data1,[ord(DW_ACCESS_protected)]);
          vis_published,
          vis_public:
            { default };
          vis_none:
            internalerror(2019050720);
        end;
      end;


    procedure TDebugInfoDwarf.insertlineinfo(list:TAsmList);
      var
        currfileinfo,
        lastfileinfo : tfileposinfo;
        currfuncname : pshortstring;
        currstatement: boolean;
        currsectype  : TAsmSectiontype;
        hp, hpend : tai;
        infile : tinputfile;
        prevcolumn,
        diffline,
        prevline,
        prevfileidx,
        currfileidx,
        nolineinfolevel : Integer;
        prevlabel,
        currlabel     : tasmlabel;
      begin
{$ifdef OMFOBJSUPPORT}
        if ds_dwarf_omf_linnum in current_settings.debugswitches then
          dbgcodeview.InsertLineInfo_OMF_LINNUM_MsLink(list);
{$endif OMFOBJSUPPORT}
        { this function will always terminate the lineinfo block }
        generated_lineinfo := true;
        { if this unit only contains code without debug info (implicit init
          or final etc), make sure the file table contains at least one entry
          (the main source of the unit), because normally this table gets
          populated via calls to get_file_index and that won't happen in this
          case }
        get_file_index(current_module.sourcefiles.get_file(1));
        FillChar(lastfileinfo,sizeof(lastfileinfo),0);
        currfuncname:=nil;
        currsectype:=sec_code;
        hp:=Tai(list.first);
        currstatement:=true;
        prevcolumn := 0;
        prevline := 1;
        prevfileidx := 1;
        prevlabel := nil;
        nolineinfolevel:=0;
        while assigned(hp) do
          begin
            case hp.typ of
              ait_section :
                currsectype:=tai_section(hp).sectype;
              ait_function_name :
                begin
                  currfuncname:=tai_function_name(hp).funcname;
                  asmline.concat(tai_comment.Create(strpnew('function: '+currfuncname^)));
                end;
              ait_force_line :
                begin
                  lastfileinfo.line:=-1;
                end;
              ait_marker :
                begin
                  case tai_marker(hp).kind of
                    mark_NoLineInfoStart:
                      inc(nolineinfolevel);
                    mark_NoLineInfoEnd:
                      dec(nolineinfolevel);
                    else
                      ;
                  end;
                end;
              else
                ;
            end;

            if (currsectype=sec_code) and
               (hp.typ=ait_instruction) then
              begin
                currfileinfo:=tailineinfo(hp).fileinfo;
                { file changed ? (must be before line info) }
                if (currfileinfo.fileindex<>0) and
                   ((lastfileinfo.fileindex<>currfileinfo.fileindex) or
                    (lastfileinfo.moduleindex<>currfileinfo.moduleindex)) then
                  begin
                    infile:=get_module(currfileinfo.moduleindex).sourcefiles.get_file(currfileinfo.fileindex);
                    if assigned(infile) then
                      begin
                        currfileidx := get_file_index(infile);
                        if prevfileidx <> currfileidx then
                          begin
                            list.insertbefore(tai_comment.Create(strpnew('path: '+infile.path)), hp);
                            list.insertbefore(tai_comment.Create(strpnew('file: '+infile.name)), hp);
                            list.insertbefore(tai_comment.Create(strpnew('indx: '+tostr(currfileidx))), hp);

                            { set file }
                            asmline.concat(tai_comment.Create(strpnew('path: '+infile.path)));
                            asmline.concat(tai_comment.Create(strpnew('file: '+infile.name)));
                            asmline.concat(tai_const.create_8bit(DW_LNS_set_file));
                            asmline.concat(tai_const.create_uleb128bit(currfileidx));

                            prevfileidx := currfileidx;
                          end;
                        { force new line info }
                        lastfileinfo.line:=-1;
                      end;
                  end;

                { Set the line-nr to 0 if the code does not corresponds to a particular line  }
                if nolineinfolevel>0 then
                  currfileinfo.line := 0;

                { line changed ? }
                if (lastfileinfo.line<>currfileinfo.line) and ((currfileinfo.line<>0) or (nolineinfolevel>0)) then
                  begin
                    { set address }
                    current_asmdata.getlabel(currlabel, alt_dbgline);
                    list.insertbefore(tai_label.create(currlabel), hp);

                    asmline.concat(tai_comment.Create(strpnew('['+tostr(currfileinfo.line)+':'+tostr(currfileinfo.column)+']')));

                    if (prevlabel = nil) or
                       { darwin's assembler cannot create an uleb128 of the difference
                         between to symbols
                         same goes for Solaris native assembler
                         ... and riscv }

                       (target_info.system in systems_darwin+[system_riscv32_linux,system_riscv64_linux]) or
                       (target_asm.id=as_solaris_as) then
                      begin
                        asmline.concat(tai_const.create_8bit(DW_LNS_extended_op));
                        asmline.concat(tai_const.create_uleb128bit(1+sizeof(pint)));
                        asmline.concat(tai_const.create_8bit(DW_LNE_set_address));
                        asmline.concat(tai_const.create_type_sym(aitconst_ptr_unaligned,currlabel));
{$ifdef i8086}
                        { on i8086 we also emit an Open Watcom-specific 'set segment' op }
                        asmline.concat(tai_const.create_8bit(DW_LNS_extended_op));
                        asmline.concat(tai_const.create_uleb128bit(3));
                        asmline.concat(tai_const.create_8bit(DW_LNE_set_segment));
                        asmline.concat(tai_const.Create_seg_name(currlabel.Name));
{$endif i8086}
                      end
                    else
                      begin
                        asmline.concat(tai_const.create_8bit(DW_LNS_advance_pc));
                        asmline.concat(tai_const.create_rel_sym(aitconst_uleb128bit, prevlabel, currlabel));
                      end;
                    prevlabel := currlabel;

                    { set column }
                    if prevcolumn <> currfileinfo.column then
                      begin
                        asmline.concat(tai_const.create_8bit(DW_LNS_set_column));
                        asmline.concat(tai_const.create_uleb128bit(currfileinfo.column));
                        prevcolumn := currfileinfo.column;
                      end;

                    { set statement }
                    if (currfileinfo.line=0) and currstatement then
                      begin
                        currstatement := false;
                        asmline.concat(tai_const.create_8bit(DW_LNS_negate_stmt));
                      end;

                    if not currstatement and (currfileinfo.line>0) then
                      begin
                        currstatement := true;
                        asmline.concat(tai_const.create_8bit(DW_LNS_negate_stmt));
                      end;

                    { set line }
                    diffline := currfileinfo.line - prevline;
                    if (diffline >= LINE_BASE) and (OPCODE_BASE + diffline - LINE_BASE <= 255) then
                      begin
                        { use special opcode, this also adds a row }
                        asmline.concat(tai_const.create_8bit(OPCODE_BASE + diffline - LINE_BASE));
                      end
                    else
                      begin
                        if diffline <> 0 then
                          begin
                            asmline.concat(tai_const.create_8bit(DW_LNS_advance_line));
                            asmline.concat(tai_const.create_sleb128bit(diffline));
                          end;
                        { no row added yet, do it manually }
                        asmline.concat(tai_const.create_8bit(DW_LNS_copy));
                      end;
                    prevline := currfileinfo.line;
                  end;

                lastfileinfo:=currfileinfo;
              end;

            hpend:=hp;
            hp:=tai(hp.next);
          end;

        if assigned(hpend) then
          begin
           { set address for end (see appendix 3 of dwarf 2 specs) }
            current_asmdata.getlabel(currlabel, alt_dbgline);
            list.insertafter(tai_label.create(currlabel), hpend);
            asmline.concat(tai_const.create_8bit(DW_LNS_extended_op));
            asmline.concat(tai_const.create_uleb128bit(1+sizeof(pint)));
            asmline.concat(tai_const.create_8bit(DW_LNE_set_address));
            asmline.concat(tai_const.create_type_sym(aitconst_ptr_unaligned,currlabel));
          end;

        { end sequence }
        asmline.concat(tai_const.Create_8bit(DW_LNS_extended_op));
        asmline.concat(tai_const.Create_8bit(1));
        asmline.concat(tai_const.Create_8bit(DW_LNE_end_sequence));
        asmline.concat(tai_comment.Create(strpnew('###################')));
      end;


    procedure TDebugInfoDwarf.finish_lineinfo;
      var
        infile: tinputfile;
      begin
        { only needed if no line info at all has been generated }
        if generated_lineinfo then
          begin
            { reset for next module compilation }
            generated_lineinfo:=false;
            exit;
          end;
        { at least the Darwin linker is annoyed if you do not }
        { finish the lineinfo section, or if it doesn't       }
        { contain at least one file name and set_address      }
        infile:=current_module.sourcefiles.get_file(1);
        if not assigned(infile) then
          internalerror(2006020211);
        asmline.concat(tai_const.create_8bit(DW_LNS_set_file));
        asmline.concat(tai_const.create_uleb128bit(get_file_index(infile)));

        asmline.concat(tai_const.create_8bit(DW_LNS_extended_op));
        asmline.concat(tai_const.create_uleb128bit(1+sizeof(pint)));
        asmline.concat(tai_const.create_8bit(DW_LNE_set_address));
        asmline.concat(tai_const.create_type_sym(aitconst_ptr_unaligned,nil));
        asmline.concat(tai_const.create_8bit(DW_LNS_extended_op));
        asmline.concat(tai_const.Create_8bit(1));
        asmline.concat(tai_const.Create_8bit(DW_LNE_end_sequence));
        asmline.concat(tai_comment.Create(strpnew('###################')));
      end;

{****************************************************************************
                              TDebugInfoDwarf2
****************************************************************************}

    procedure TDebugInfoDwarf2.appenddef_file(list:TAsmList;def: tfiledef);
      begin
        { gdb 6.4 doesn't support files so far so we use some fake recorddef
          file recs. are less than 1k so using data2 is enough }
        if assigned(def.typesym) then
          append_entry(DW_TAG_structure_type,false,[
           DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
           DW_AT_byte_size,DW_FORM_udata,def.size
          ])
        else
          append_entry(DW_TAG_structure_type,false,[
           DW_AT_byte_size,DW_FORM_udata,def.size
          ]);
        finish_entry;
      end;

    procedure TDebugInfoDwarf2.appenddef_formal(list:TAsmList;def: tformaldef);
      begin
        { gdb 6.4 doesn't support DW_TAG_unspecified_type so we
          replace it with a unsigned type with size 0 (FK)
        }
        append_entry(DW_TAG_base_type,false,[
          DW_AT_name,DW_FORM_string,'FormalDef'#0,
          DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
          DW_AT_byte_size,DW_FORM_data1,0
        ]);
        finish_entry;
      end;

    procedure TDebugInfoDwarf2.append_object_struct(def: tobjectdef; const createlabel: boolean; const objectname: PShortString);
      begin
        if createlabel then
          begin
            if not(tf_dwarf_only_local_labels in target_info.flags) then
              current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create_global(def_dwarf_class_struct_lab(def),0))
            else
              current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(def_dwarf_class_struct_lab(def),0));
          end;
        if assigned(objectname) then
          append_entry(DW_TAG_class_type,true,[
            DW_AT_name,DW_FORM_string,objectname^+#0,
            DW_AT_byte_size,DW_FORM_udata,tobjectsymtable(def.symtable).datasize
            ])
        else
          append_entry(DW_TAG_class_type,true,[
            DW_AT_byte_size,DW_FORM_udata,tobjectsymtable(def.symtable).datasize
            ]);
        { Apple-specific tag that identifies it as an Objective-C class }
        if (def.objecttype=odt_objcclass) then
          append_attribute(DW_AT_APPLE_runtime_class,DW_FORM_data1,[DW_LANG_ObjC]);

        finish_entry;
        if assigned(def.childof) then
          begin
            append_entry(DW_TAG_inheritance,false,[
              DW_AT_accessibility,DW_FORM_data1,DW_ACCESS_public,
              DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(0)
            ]);
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(0));
            if (def.childof.dbg_state=dbg_state_unused) then
              def.childof.dbg_state:=dbg_state_used;
            if is_implicit_pointer_object_type(def) then
              append_labelentry_ref(DW_AT_type,def_dwarf_class_struct_lab(def.childof))
            else
              append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.childof));
            finish_entry;
          end;
        if (oo_has_vmt in def.objectoptions) and
           (not assigned(def.childof) or
            not(oo_has_vmt in def.childof.objectoptions)) then
          begin
            { vmt field }
            append_entry(DW_TAG_member,false,[
                DW_AT_artificial,DW_FORM_flag,true,
                DW_AT_name,DW_FORM_string,'_vptr$'+def.objname^+#0,
                DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(def.vmt_offset)
            ]);
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
            current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(def.vmt_offset));
            { should be changed into a pointer to a function returning an }
            { int and with TAG_unspecified_parameters                     }
            if (voidpointertype.dbg_state=dbg_state_unused) then
              voidpointertype.dbg_state:=dbg_state_used;
            append_labelentry_ref(DW_AT_type,def_dwarf_lab(voidpointertype));
            finish_entry;
          end;

        def.symtable.symList.ForEachCall(@enum_membersyms_callback,nil);
        { Write the methods in the scope of the class/object, except for Objective-C.  }
        if is_objc_class_or_protocol(def) then
          finish_children;
        { don't write procdefs of externally defined classes, gcc doesn't
          either (info is probably gotten from ObjC runtime)  }
        if not(oo_is_external in def.objectoptions) then
          write_symtable_procdefs(current_asmdata.asmlists[al_dwarf_info],def.symtable);
        if not is_objc_class_or_protocol(def) then
          finish_children;
      end;


    procedure TDebugInfoDwarf2.appenddef_object(list:TAsmList;def: tobjectdef);

      begin
        case def.objecttype of
          odt_cppclass,
          odt_object:
            append_object_struct(def,false,def.objname);
          odt_interfacecom,
          odt_interfacecorba,
          odt_dispinterface,
          odt_helper,
          odt_class:
            begin
              { implicit pointer }
              append_entry(DW_TAG_pointer_type,false,[]);
              append_labelentry_ref(DW_AT_type,def_dwarf_class_struct_lab(def));
              finish_entry;

              append_object_struct(def,true,def.objname);
            end;
          odt_objcclass:
            { Objective-C class: same as regular class, except for
                a) Apple-specific tag that identifies it as an Objective-C class
                b) use extname^ instead of objname
            }
            append_object_struct(def,true,def.objextname);
          odt_objcprotocol:
            begin
              append_entry(DW_TAG_pointer_type,false,[]);
              append_labelentry_ref(DW_AT_type,def_dwarf_lab(voidpointertype));
              finish_entry;
            end;
          else
            internalerror(200602041);
        end;
      end;

    procedure TDebugInfoDwarf2.appenddef_set_intern(list:TAsmList;def: tsetdef; force_tag_set: boolean);
      var
        lab: tasmlabel;
      begin
        if force_tag_set or
           (ds_dwarf_sets in current_settings.debugswitches) then
          begin
            { current (20070704 -- patch was committed on 20060513) gdb cvs supports set types }

            if assigned(def.typesym) then
              append_entry(DW_TAG_set_type,false,[
                DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                DW_AT_byte_size,DW_FORM_data2,def.size
                ])
            else
              append_entry(DW_TAG_set_type,false,[
                DW_AT_byte_size,DW_FORM_data2,def.size
                ]);
            if assigned(def.elementdef) then
              begin
                if not(tf_dwarf_only_local_labels in target_info.flags) then
                  current_asmdata.getglobaldatalabel(lab)
                else
                  current_asmdata.getaddrlabel(lab);
                append_labelentry_ref(DW_AT_type,lab);
                finish_entry;
                if lab.bind=AB_GLOBAL then
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create_global(lab,0))
                else
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(lab,0));
                { Sets of e.g. [1..5] are actually stored as a set of [0..7],
                  so write the exact boundaries of the set here. Let's hope no
                  debugger ever rejects this because this "subrange" type can
                  actually have a larger range than the original one.  }
                append_entry(DW_TAG_subrange_type,false,[
                  DW_AT_lower_bound,DW_FORM_sdata,def.setbase,
                  DW_AT_upper_bound,DW_FORM_sdata,get_max_value(def.elementdef).svalue
                  ]);
                append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.elementdef))
              end
          end
        else
          begin
            { gdb versions which don't support sets refuse to load the debug }
            { info of modules that contain set tags                          }
            if assigned(def.typesym) then
              append_entry(DW_TAG_base_type,false,[
                DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
                DW_AT_byte_size,DW_FORM_data2,def.size
                ])
            else
              append_entry(DW_TAG_base_type,false,[
                DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
                DW_AT_byte_size,DW_FORM_data2,def.size
                ]);
          end;
        finish_entry;
      end;

    procedure TDebugInfoDwarf2.appenddef_set(list:TAsmList;def: tsetdef);
      begin
        appenddef_set_intern(list,def,false);
      end;

    procedure TDebugInfoDwarf2.appenddef_undefined(list:TAsmList;def: tundefineddef);
      begin
        { gdb 6.4 doesn't support DW_TAG_unspecified_type so we
          replace it with a unsigned type with size 0 (FK)
        }
        append_entry(DW_TAG_base_type,false,[
          DW_AT_name,DW_FORM_string,'FormalDef'#0,
          DW_AT_encoding,DW_FORM_data1,DW_ATE_unsigned,
          DW_AT_byte_size,DW_FORM_data1,0
        ]);
        finish_entry;
      end;

    procedure TDebugInfoDwarf2.appenddef_variant(list:TAsmList;def: tvariantdef);
      begin
        { variants aren't known to dwarf2 but writting tvardata should be enough }
        if assigned(vardatadef) then
          appenddef_record_named(list,trecorddef(vardatadef),'Variant');
      end;

    function TDebugInfoDwarf2.dwarf_version: Word;
      begin
        Result:=2;
      end;

{****************************************************************************
                              TDebugInfoDwarf3
****************************************************************************}

    procedure TDebugInfoDwarf3.append_labelentry_addr_ref(attr : tdwarf_attribute;sym : tasmsymbol);
      begin
        AddConstToAbbrev(ord(DW_FORM_ref_addr));
        { Since Dwarf 3 the length of a DW_FORM_ref_addr entry is not dependent on the pointer size of the
          target platform, but on the used Dwarf-format (32 bit or 64 bit) for the current compilation section. }
        if use_64bit_headers then
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.Create_type_sym(aitconst_64bit_unaligned,sym))
        else
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.Create_type_sym(aitconst_32bit_unaligned,sym));
      end;

    procedure tdebuginfodwarf3.appenddef_array(list: tasmlist; def: tarraydef);
      begin
        if not is_dynamic_array(def) then
          begin
            inherited appenddef_array(list,def);
            exit;
          end;

        if assigned(def.typesym) then
          append_entry(DW_TAG_array_type,true,[
            DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
            DW_AT_byte_stride,DW_FORM_udata,def.elesize,
            DW_AT_data_location,DW_FORM_block1,2
            ])
        else
          append_entry(DW_TAG_array_type,true,[
            DW_AT_byte_stride,DW_FORM_udata,def.elesize,
            DW_AT_data_location,DW_FORM_block1,2
            ]);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));

        append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.elementdef));
        finish_entry;
        { to simplify things, we don't write a multidimensional array here }
        append_entry(DW_TAG_subrange_type,false,[
          DW_AT_lower_bound,DW_FORM_udata,0,
          DW_AT_upper_bound,DW_FORM_block1,14
          ]);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_dup)));
        { pointer = nil? }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_bra)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(5));
        { yes -> length = 0 }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_const1s)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(byte(-1)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_skip)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(3));
        { no -> load length }
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_lit0)+sizesinttype.size));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_minus)));
        current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
        append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.rangedef));
        finish_entry;

        finish_children;
      end;


    procedure tdebuginfodwarf3.appenddef_string(list: tasmlist; def: tstringdef);

      procedure addstringdef(const name: shortstring; chardef: tdef; deref: boolean; lensize: aint);
        var
          upperopcodes: longint;
        begin
          { deref=true -> ansi/unicde/widestring; deref = false -> short/longstring }
          if assigned(def.typesym) then
            append_entry(DW_TAG_array_type,true,[
              DW_AT_name,DW_FORM_string,name+#0,
              DW_AT_data_location,DW_FORM_block1,2+ord(not(deref))
              ])
          else
            append_entry(DW_TAG_array_type,true,[
              DW_AT_data_location,DW_FORM_block1,2+ord(not(deref))
              ]);

          { in all cases we start with the address of the string }
          current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
          if deref then
            begin
              { ansi/unicode/widestring -> dereference the address of the string, and then
                we point to address of the string
              }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
            end
          else
            begin
              { shortstring characters begin at string[1], so add one to the string's address }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_lit0)+lensize));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus)))
            end;

          { reference to the element type of the string }
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(chardef));
          finish_entry;

          { now the information about the length of the string }
          if deref then
            begin
              if not (is_widestring(def) and (tf_winlikewidestring in target_info.flags)) then
                upperopcodes:=13
              else
                upperopcodes:=16;
              { lower bound is always 1, upper bound (length) needs to be calculated }
              append_entry(DW_TAG_subrange_type,false,[
                DW_AT_lower_bound,DW_FORM_udata,1,
                DW_AT_upper_bound,DW_FORM_block1,upperopcodes
                ]);

              { high(string) is stored sizeof(sizeint) bytes before the string data }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_dup)));
              { pointer = nil? }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_bra)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(4));
              { yes -> length = 0 }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_lit0)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_skip)));
              if upperopcodes=16 then
                { skip the extra deref_size argument and the division by two of the length }
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(6))
              else
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_16bit_unaligned(3));
              { no -> load length }
              if upperopcodes=16 then
                { for Windows WideString the size is always a DWORD }
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_lit4)))
              else
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_lit0)+sizesinttype.size));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_minus)));
              if upperopcodes=16 then
                begin
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref_size)));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(4));
                end
              else
                current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));

              { for widestrings, the length is specified in bytes, so divide by two }
              if (upperopcodes=16) then
                begin
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_lit1)));
                  current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_shr)));
                end;
            end
          else
            begin
              append_entry(DW_TAG_subrange_type,false,[
                DW_AT_lower_bound,DW_FORM_udata,1,
                DW_AT_upper_bound,DW_FORM_block1,3
                ]);
              { for shortstrings, the length is the first byte of the string }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
              { load 1 byte }
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref_size)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(lensize));
            end;
          finish_entry;

          finish_children;
        end;

      begin
        if (ds_dwarf_cpp in current_settings.debugswitches) then
          begin
            // At least LLDB 6.0.0 does not like this implementation of string types.
            // Call the inherited DWARF 2 implementation, which works fine.
            inherited;
            exit;
          end;
        case def.stringtype of
          st_shortstring:
            begin
              addstringdef('ShortString',cansichartype,false,1);
            end;
          st_longstring:
            begin
{$ifdef cpu64bitaddr}
              addstringdef('LongString',cansichartype,false,8);
{$else cpu64bitaddr}
              addstringdef('LongString',cansichartype,false,4);
{$endif cpu64bitaddr}
           end;
         st_ansistring:
           begin
             addstringdef('AnsiString',cansichartype,true,-1);
           end;
         st_unicodestring:
           begin
             addstringdef('UnicodeString',cwidechartype,true,-1);
           end;
         st_widestring:
           begin
             addstringdef('WideString',cwidechartype,true,-1)
           end;
        end;
      end;

    procedure TDebugInfoDwarf3.appenddef_file(list:TAsmList;def: tfiledef);
      begin
        if assigned(def.typesym) then
          append_entry(DW_TAG_file_type,false,[
            DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0,
            DW_AT_byte_size,DW_FORM_data2,def.size
            ])
        else
          append_entry(DW_TAG_file_type,false,[
            DW_AT_byte_size,DW_FORM_data2,def.size
            ]);
        if tfiledef(def).filetyp=ft_typed then
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(tfiledef(def).typedfiledef));
        finish_entry;
      end;

    procedure TDebugInfoDwarf3.appenddef_formal(list:TAsmList;def: tformaldef);
      begin
        if (ds_dwarf_cpp in current_settings.debugswitches) then
          begin
            // Do not use DW_TAG_unspecified_type for C++ simulation.
            // At least LLDB 3.9.0 crashes in such case.
            // Call the inherited DWARF 2 implementation, which works fine.
            inherited;
            exit;
          end;

        append_entry(DW_TAG_unspecified_type,false,[]);
        finish_entry;
      end;

    procedure TDebugInfoDwarf3.appenddef_object(list:TAsmList;def: tobjectdef);

      procedure dostruct(tag: tdwarf_tag);
        begin
          if assigned(def.objname) then
            append_entry(tag,true,[
              DW_AT_name,DW_FORM_string,def.objrealname^+#0
              ])
          else
            append_entry(DW_TAG_structure_type,true,[]);
          append_attribute(DW_AT_byte_size,DW_FORM_udata,[tobjectsymtable(def.symtable).datasize]);
          { an old style object and a cpp class are accessed directly, so we do not need DW_AT_allocated and DW_AT_data_location tags,
            see issue #36017 }
          if not(is_object(def) or is_cppclass(def)) then
            begin
              { The pointer to the class-structure is hidden. The debug-information
                does not contain an implicit pointer, but the data-adress is dereferenced here.
                In case of a nil-pointer, report the class as being unallocated.
              }
              append_block1(DW_AT_allocated,2);
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
              append_block1(DW_AT_data_location,2);
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_push_object_address)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_deref)));
            end;
          finish_entry;
        end;

      procedure doimplicitpointer;
        var
          obj : tasmlabel;
        begin
          if not(tf_dwarf_only_local_labels in target_info.flags) then
            current_asmdata.getglobaldatalabel(obj)
          else
            current_asmdata.getaddrlabel(obj);
          { implicit pointer }
          append_entry(DW_TAG_pointer_type,false,[]);
          append_labelentry_ref(DW_AT_type,obj);
          finish_entry;

          current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(obj,0));
        end;

      procedure doparent(isinterface: boolean);
        begin
          if not assigned(def.childof) then
            exit;

          if isinterface then
            begin
              append_entry(DW_TAG_inheritance,false,[]);
            end
          else
            begin
              append_entry(DW_TAG_inheritance,false,[
                DW_AT_accessibility,DW_FORM_data1,DW_ACCESS_public,
                DW_AT_data_member_location,DW_FORM_block1,1+lengthuleb128(0)
              ]);
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_8bit(ord(DW_OP_plus_uconst)));
              current_asmdata.asmlists[al_dwarf_info].concat(tai_const.create_uleb128bit(0));
            end;
          append_labelentry_ref(DW_AT_type,def_dwarf_lab(def.childof));
          finish_entry;
        end;

      var
        n: integer;

      begin
        case def.objecttype of
          odt_cppclass,
          odt_object:
            begin
              dostruct(DW_TAG_structure_type);
              doparent(false);
            end;
          odt_interfacecom,
          odt_interfacecorba,
          odt_dispinterface:
            begin
              dostruct(DW_TAG_interface_type);
              doparent(true);
            end;
          odt_helper,
          odt_class:
            begin
              //dostruct(DW_TAG_class_type);
              //doparent(false);
              append_entry(DW_TAG_pointer_type,false,[]);
              append_labelentry_ref(DW_AT_type,def_dwarf_class_struct_lab(def));
              finish_entry;

              append_object_struct(def,true,def.objrealname);
              Exit;
            end;
        else
          internalerror(200609171);
        end;

        { add implemented interfaces }
        if assigned(def.ImplementedInterfaces) then
          for n := 0 to def.ImplementedInterfaces.count-1 do
            begin
              append_entry(DW_TAG_inheritance,false,[]);
              append_labelentry_ref(DW_AT_type,def_dwarf_lab(TImplementedInterface(def.ImplementedInterfaces[n]).IntfDef));
              finish_entry;
            end;

        { add members }
        def.symtable.symList.ForEachCall(@enum_membersyms_callback,nil);
        finish_children;
      end;

    procedure TDebugInfoDwarf3.appenddef_set(list:TAsmList;def: tsetdef);
      begin
        appenddef_set_intern(list,def,true);
      end;

    procedure TDebugInfoDwarf3.appenddef_undefined(list:TAsmList;def: tundefineddef);
      begin
        { ??? can a undefined def have a typename ? }
        if assigned(def.typesym) then
          append_entry(DW_TAG_unspecified_type,false,[
            DW_AT_name,DW_FORM_string,symname(def.typesym, false)+#0
            ])
        else
          append_entry(DW_TAG_unspecified_type,false,[
            ]);
        finish_entry;
      end;

    procedure TDebugInfoDwarf3.appenddef_variant(list:TAsmList;def: tvariantdef);
      const
        VARIANTS: array[1..27] of record Value: Word; Name: String end = (
          (value:0;     name:''),
          (value:1;     name:''),
          (value:2;     name:'VSMALLINT'),
          (value:3;     name:'VINTEGER'),
          (value:4;     name:'VSINGLE'),
          (value:5;     name:'VDOUBLE'),
          (value:6;     name:'VCURRENCY'),
          (value:7;     name:'VDATE'),
          (value:8;     name:'VOLESTR'),
          (value:9;     name:'VDISPATCH'),
          (value:10;    name:'VERROR'),
          (value:11;    name:'VBOOLEAN'),
          (value:12;    name:''),
          (value:13;    name:'VUNKNOWN'),
          (value:14;    name:''),
          (value:16;    name:'VSHORTINT'),
          (value:17;    name:'VBYTE'),
          (value:18;    name:'VWORD'),
          (value:19;    name:'VLONGWORD'),
          (value:20;    name:'VINT64'),
          (value:21;    name:'VQWORD'),
          (value:36;    name:'VRECORD'),
          (value:$48;   name:''),
          (value:$100;  name:'VSTRING'),
          (value:$101;  name:'VANY'),
          (value:$2000; name:'VARRAY'),
          (value:$4000; name:'VPOINTER')
        );
      var
        fs: tfieldvarsym;
        lbl: tasmlabel;
        idx: integer;
      begin
        { it could be done with DW_TAG_variant for the union part (if that info was available)
          now we do it manually for variants (MWE) }

        { struct }
        append_entry(DW_TAG_structure_type,true,[
          DW_AT_name,DW_FORM_string,'Variant'#0,
          DW_AT_byte_size,DW_FORM_udata,vardatadef.size
          ]);
        finish_entry;

        append_entry(DW_TAG_variant_part,true,[
          ]);
        current_asmdata.getaddrlabel(lbl);
        append_labelentry_ref(DW_AT_discr,lbl);
        finish_entry;

        { discriminant }
        fs := tfieldvarsym(vardatadef.symtable.Find('VTYPE'));
        if (fs = nil) or (fs.typ <> fieldvarsym) then
          internalerror(200609271);
        current_asmdata.asmlists[al_dwarf_info].concat(tai_symbol.create(lbl,0));
        appendsym_fieldvar(list,fs);

        { variants }
        for idx := Low(VARIANTS) to High(VARIANTS) do
          begin
            append_entry(DW_TAG_variant,true,[
              DW_AT_discr_value,DW_FORM_udata,VARIANTS[idx].value
              ]);
            finish_entry;

            if VARIANTS[idx].name <> '' then
              begin
                fs := tfieldvarsym(vardatadef.symtable.Find(VARIANTS[idx].name));
                if (fs = nil) or (fs.typ <> fieldvarsym) then
                  internalerror(20060927200+idx);
                appendsym_fieldvar(list,fs);
              end;

            finish_children; { variant }
          end;


        finish_children; { variant part }

        finish_children; { struct }
      end;

    function TDebugInfoDwarf3.dwarf_version: Word;
      begin
        Result:=3;
      end;

    function TDebugInfoDwarf3.symdebugname(sym: tsym): String;
      begin
        Result:=sym.realname;
      end;


    { TDebugInfoDwarf4 }

    function TDebugInfoDwarf4.dwarf_version: Word;
    begin
      Result:=4;
    end;


{****************************************************************************
****************************************************************************}
    const
      dbg_dwarf2_info : tdbginfo =
         (
           id     : dbg_dwarf2;
           idtxt  : 'DWARF2';
         );

      dbg_dwarf3_info : tdbginfo =
         (
           id     : dbg_dwarf3;
           idtxt  : 'DWARF3';
         );

      dbg_dwarf4_info : tdbginfo =
         (
           id     : dbg_dwarf4;
           idtxt  : 'DWARF4';
         );


initialization
  RegisterDebugInfo(dbg_dwarf2_info,TDebugInfoDwarf2);
  RegisterDebugInfo(dbg_dwarf3_info,TDebugInfoDwarf3);
  RegisterDebugInfo(dbg_dwarf4_info,TDebugInfoDwarf4);

end.
