{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements an abstract asmoutput class for all processor types

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
{ @abstract(This unit implements an abstract asm output class for all processor types)
  This unit implements an abstract assembler output class for all processors, these
  are then overridden for each assembler writer to actually write the data in these
  classes to an assembler file.
}

unit aasmbase;

{$i fpcdefs.inc}

interface

    uses
       cutils,cclasses,
       globtype,globals,systems
       ;

    type
       TAsmsymbind=(
         AB_NONE,AB_EXTERNAL,AB_COMMON,AB_LOCAL,AB_GLOBAL,AB_WEAK_EXTERNAL,
         { global in the current program/library, but not visible outside it
           (= "hidden" in ELF) }
         AB_PRIVATE_EXTERN,
         AB_LAZY,AB_IMPORT,
         { a symbol that's internal to the compiler and used as a temp }
         AB_TEMP,
         { a global symbol that points to another global symbol and is only used
           to allow indirect loading in case of packages and indirect imports }
         AB_INDIRECT,AB_EXTERNAL_INDIRECT);

       TAsmsymtype=(
         AT_NONE,AT_FUNCTION,AT_DATA,AT_SECTION,AT_LABEL,
         {
           the address of this code label is taken somewhere in the code
           so it must be taken care of it when creating pic
         }
         AT_ADDR,
         { Label for debug or other non-program information }
         AT_METADATA,
         { label for data that must always be accessed indirectly, because it
           is handled explcitely in the system unit or (e.g. RTTI and threadvar
           tables) -- never seen in an assembler/assembler writer, always
           changed to AT_DATA }
         AT_DATA_FORCEINDIRECT,
         { don't generate an implicit indirect symbol as that might be provided
           by other means (e.g. the typed const builder) to ensure a correct
           section name }
         AT_DATA_NOINDIRECT,
         { Thread-local symbol (ELF targets) }
         AT_TLS,
         { GNU indirect function (ELF targets) }
         AT_GNU_IFUNC
         );

       { is the label only there for getting an DataOffset (e.g. for i/o
         checks -> alt_addr) or is it a jump target (alt_jump), for debug
         info alt_dbgline and alt_dbgfile, etc. }
       TAsmLabelType = (alt_jump,alt_addr,alt_data,alt_dbgline,alt_dbgfile,alt_dbgtype,alt_dbgframe,alt_eh_begin,alt_eh_end);

    const
       asmlabeltypeprefix : array[TAsmLabeltype] of string[2] = ('j','a','d','l','f','t','c','eb','ee');
       asmsymbindname : array[TAsmsymbind] of string[23] = ('none', 'external','common',
       'local','global','weak external','private external','lazy','import','internal temp',
       'indirect','external indirect');
       asmsymbindindirect = [AB_INDIRECT,AB_EXTERNAL_INDIRECT];

    type
       TAsmSectiontype=(sec_none,
         { this section type allows to define a user named section }
         sec_user,
         sec_code,
         sec_data,
         { read-only, but may contain relocations }
         sec_rodata,
         { read-only and cannot contain relocations }
         sec_rodata_norel,
         sec_bss,
         sec_threadvar,
         { used for wince exception handling }
         sec_pdata,
         { used for darwin import stubs }
         sec_stub,
         sec_data_nonlazy,
         sec_data_lazy,
         sec_init_func,
         sec_term_func,
         { stabs }
         sec_stab,sec_stabstr,
         { win32 }
         sec_idata2,sec_idata4,sec_idata5,sec_idata6,sec_idata7,sec_edata,
         { C++ exception handling unwinding (uses dwarf) }
         sec_eh_frame,
         { dwarf }
         sec_debug_frame,
         sec_debug_info,
         sec_debug_line,
         sec_debug_abbrev,
         sec_debug_aranges,
         sec_debug_ranges,
         { Yury: "sec_fpc is intended for storing fpc specific data
                  which must be recognized and processed specially by linker.
                  Currently fpc version string, dummy links to stab sections
                  and elf resources are stored in .fpc sections."
                  "If special .fpc section cannot be used on some target,
                  .text can be used instead." }
         sec_fpc,
         { Table of contents section }
         sec_toc,
         sec_init,
         sec_fini,
         {Objective-C common and fragile ABI }
         sec_objc_class,
         sec_objc_meta_class,
         sec_objc_cat_cls_meth,
         sec_objc_cat_inst_meth,
         sec_objc_protocol,
         sec_objc_string_object,
         sec_objc_cls_meth,
         sec_objc_inst_meth,
         sec_objc_cls_refs,
         sec_objc_message_refs,
         sec_objc_symbols,
         sec_objc_category,
         sec_objc_class_vars,
         sec_objc_instance_vars,
         sec_objc_module_info,
         sec_objc_class_names,
         sec_objc_meth_var_types,
         sec_objc_meth_var_names,
         sec_objc_selector_strs,
         sec_objc_protocol_ext,
         sec_objc_class_ext,
         sec_objc_property,
         sec_objc_image_info,
         sec_objc_cstring_object,
         sec_objc_sel_fixup,
         { Objective-C non-fragile ABI }
         sec_objc_data,
         sec_objc_const,
         sec_objc_sup_refs,
         sec_data_coalesced,
         sec_objc_classlist,
         sec_objc_nlclasslist,
         sec_objc_catlist,
         sec_objc_nlcatlist,
         sec_objc_protolist,
         { stack segment for 16-bit DOS }
         sec_stack,
         { initial heap segment for 16-bit DOS }
         sec_heap,
         { dwarf based/gcc style exception handling }
         sec_gcc_except_table,
         sec_arm_attribute
       );

       TObjCAsmSectionType = sec_objc_class..sec_objc_protolist;

       TAsmSectionOrder = (secorder_begin,secorder_default,secorder_end);

       TSectionFlag = (SF_A,SF_W,SF_X);
       TSectionFlags = set of TSectionFlag;
       TSectionProgbits = (SPB_None,SPB_PROGBITS,SPB_NOBITS,SPB_NOTE,SPB_ARM_ATTRIBUTES);

       TAsmSymbol = class(TFPHashObject)
       private
         { this need to be incremented with every symbol loading into the
           TAsmList with loadsym/loadref/const_symbol (PFV) }
         refs       : longint;
       public
         { on avr the compiler needs to replace cond. jumps with too large offsets
           so we have to store an offset somewhere to calculate jump distances }
{$ifdef AVR}
         offset     : longint;
{$endif AVR}
         bind       : TAsmsymbind;
         typ        : TAsmsymtype;
{$ifdef llvm}
         { have we generated a declaration for this symbol? }
         declared   : boolean;
{$endif llvm}
         { Alternate symbol which can be used for 'renaming' needed for
           asm inlining. Also used for external and common solving during linking }
         altsymbol  : TAsmSymbol;
         { Cached objsymbol }
         cachedobjsymbol : TObject;
         constructor Create(AList:TFPHashObjectList;const s:TSymStr;_bind:TAsmsymbind;_typ:Tasmsymtype);
         function getaltcopy(AList:TFPHashObjectList;altnr: longint): TAsmSymbol; virtual;
         function  is_used:boolean;
         procedure increfs;
         procedure decrefs;
         function getrefs: longint;
       end;
       TAsmSymbolClass = class of TAsmSymbol;

       TAsmLabel = class(TAsmSymbol)
       protected
         function getname:TSymStr;override;
         {$push}{$warnings off}
         { new visibility section to let "warnings off" take effect }
       protected
         { this constructor is only supposed to be used internally by
           createstatoc/createlocal -> disable warning that constructors should
           be public }
         constructor create_non_global(AList: TFPHashObjectList; nr: longint; ltyp: TAsmLabelType; const prefix: TSymStr);
       public
         {$pop}
         labelnr   : longint;
         labeltype : TAsmLabelType;
         is_set    : boolean;
         is_public : boolean;
         defined_in_asmstatement : boolean;
         constructor Createlocal(AList: TFPHashObjectList; nr: longint; ltyp: TAsmLabelType);
         constructor Createstatic(AList: TFPHashObjectList; nr: longint; ltyp: TAsmLabelType);
         constructor Createglobal(AList: TFPHashObjectList; const modulename: TSymStr; nr: longint; ltyp: TAsmLabelType);
         function getaltcopy(AList:TFPHashObjectList;altnr: longint): TAsmSymbol; override;
       end;

    function create_smartlink_sections:boolean;inline;
    function create_smartlink_library:boolean;inline;
    function create_smartlink:boolean;inline;

    function ReplaceForbiddenAsmSymbolChars(const s: ansistring): ansistring;

    { dummy default noop callback }
    procedure default_global_used;

    type
      { Procedure variable to allow for special handling of
        the occurence of use of a global variable,
        used by PIC code generation to request GOT loading }
      TGlobalUsedProcedure = procedure;

  const
    global_used : TGlobalUsedProcedure = @default_global_used;

implementation

    uses
      verbose;


    function create_smartlink_sections:boolean;inline;
      begin
        result:=(af_smartlink_sections in target_asm.flags) and
                (tf_smartlink_sections in target_info.flags);
      end;


    function create_smartlink_library:boolean;inline;
      begin
        result:=(cs_Create_smart in current_settings.moduleswitches) and
                (tf_smartlink_library in target_info.flags) and
                not create_smartlink_sections;
      end;


    function create_smartlink:boolean;inline;
      begin
        result:=(
                 (af_smartlink_sections in target_asm.flags) and
                 (tf_smartlink_sections in target_info.flags)
                ) or
                (
                 (cs_Create_smart in current_settings.moduleswitches) and
                 (tf_smartlink_library in target_info.flags)
                );
      end;


    function ReplaceForbiddenAsmSymbolChars(const s: ansistring): ansistring;
      var
        i : longint;
        rchar: char;
      begin
        Result:=s;
        rchar:=target_asm.dollarsign;
        for i:=1 to Length(Result) do
          if Result[i]='$' then
            Result[i]:=rchar;
      end;


{*****************************************************************************
                                 TAsmSymbol
*****************************************************************************}

    constructor TAsmSymbol.Create(AList:TFPHashObjectList;const s:TSymStr;_bind:TAsmsymbind;_typ:Tasmsymtype);
      begin;
        inherited Create(AList,s);
        bind:=_bind;
        typ:=_typ;
        { used to remove unused labels from the al_procedures }
        refs:=0;
      end;


    function TAsmSymbol.getaltcopy(AList:TFPHashObjectList;altnr: longint): TAsmSymbol;
      begin
        result := TAsmSymbol(TAsmSymbolClass(classtype).Create(AList,name+'_'+tostr(altnr),bind,typ));
      end;


    function TAsmSymbol.is_used:boolean;
      begin
        is_used:=(refs>0);
      end;


    procedure TAsmSymbol.increfs;
      begin
        inc(refs);
      end;


    procedure TAsmSymbol.decrefs;
      begin
        dec(refs);
        if refs<0 then
          internalerror(200211121);
      end;


    function TAsmSymbol.getrefs: longint;
      begin
        getrefs := refs;
      end;


{*****************************************************************************
                                 TAsmLabel
*****************************************************************************}

    constructor TAsmLabel.Createlocal(AList: TFPHashObjectList; nr: longint; ltyp: TAsmLabelType);
      begin
        create_non_global(AList,nr,ltyp,target_asm.labelprefix);
      end;


    constructor TAsmLabel.Createstatic(AList:TFPHashObjectList;nr:longint;ltyp:TAsmLabelType);
      begin
        create_non_global(AList,nr,ltyp,'_$$fpclocal$_l');
      end;


    constructor TAsmLabel.Createglobal(AList:TFPHashObjectList;const modulename:TSymStr;nr:longint;ltyp:TAsmLabelType);
      begin
        inherited Create(AList,'_$'+modulename+'$_L'+asmlabeltypeprefix[ltyp]+tostr(nr),AB_GLOBAL,AT_DATA);
        labelnr:=nr;
        labeltype:=ltyp;
        is_set:=false;
        { write it always }
        increfs;
        global_used;
      end;


    function TAsmLabel.getaltcopy(AList:TFPHashObjectList;altnr: longint): TAsmSymbol;
      begin;
        result := inherited getaltcopy(AList,altnr);
        TAsmLabel(result).labelnr:=labelnr;
        TAsmLabel(result).labeltype:=labeltype;
        TAsmLabel(result).is_set:=false;
        case bind of
          AB_GLOBAL,
          AB_PRIVATE_EXTERN:
            result.increfs;
          AB_LOCAL:
            ;
          else
            internalerror(2006053101);
        end;
      end;


    function TAsmLabel.getname:TSymStr;
      begin
        getname:=inherited getname;
        increfs;
      end;


    constructor TAsmLabel.create_non_global(AList: TFPHashObjectList; nr: longint; ltyp: TAsmLabelType; const prefix: TSymStr);
      var
        asmtyp: TAsmsymtype;
      begin
        case ltyp of
          alt_addr:
            asmtyp:=AT_ADDR;
          alt_data:
            asmtyp:=AT_DATA;
          else
            asmtyp:=AT_LABEL;
        end;
        inherited Create(AList,prefix+asmlabeltypeprefix[ltyp]+tostr(nr),AB_LOCAL,asmtyp);
        labelnr:=nr;
        labeltype:=ltyp;
        is_set:=false;
      end;


    procedure default_global_used;
      begin
      end;

end.
