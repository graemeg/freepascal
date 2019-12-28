{
    Copyright (c) 1998-2002 by Florian Klaempfl, Pierre Muller

    This unit handles the symbol tables

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
unit symtable;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cutils,cclasses,globtype,tokens,
       { symtable }
       symconst,symbase,symtype,symdef,symsym;


{****************************************************************************
                             Symtable types
****************************************************************************}

    type
       tstoredsymtable = class(TSymtable)
       private
          init_final_check_done : boolean;
          procedure _needs_init_final(sym:TObject;arg:pointer);
          procedure do_init_final_check;
          procedure check_forward(sym:TObject;arg:pointer);
          procedure check_block_valid(def: TObject;arg:pointer);
          procedure register_defs(def:tobject;arg:pointer);
          procedure register_syms(sym:tobject;arg:pointer);
          procedure labeldefined(sym:TObject;arg:pointer);
          procedure varsymbolused(sym:TObject;arg:pointer);
          procedure TestPrivate(sym:TObject;arg:pointer);
          procedure objectprivatesymbolused(sym:TObject;arg:pointer);
          procedure loaddefs(ppufile:tcompilerppufile);
          procedure loadsyms(ppufile:tcompilerppufile);
          procedure writedefs(ppufile:tcompilerppufile);
          procedure writesyms(ppufile:tcompilerppufile);
       public
          constructor create(const s:string);
          procedure insert(sym:TSymEntry;checkdup:boolean=true);override;
          procedure delete(sym:TSymEntry);override;
          { load/write }
          procedure ppuload(ppufile:tcompilerppufile);virtual;
          procedure ppuwrite(ppufile:tcompilerppufile);virtual;
          procedure buildderef;
          procedure buildderefimpl;
          { buildderef but only for (recursively) used symbols/defs }
          procedure buildderef_registered;
          procedure deref(only_registered: boolean);virtual;
          procedure derefimpl(only_registered: boolean);virtual;
          function  checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;override;
          procedure allsymbolsused;
          procedure allprivatesused;
          procedure check_forwards;
          procedure checklabels;
          function  needs_init_final : boolean; virtual;
          function  has_non_trivial_init:boolean;virtual;
          procedure testfordefaultproperty(sym:TObject;arg:pointer);
          procedure register_children;
       end;

{$ifdef llvm}
      tllvmshadowsymtableentry = class
        constructor create(def: tdef; fieldoffset: aint);
       private
         ffieldoffset: aint;
         fdef: tdef;
       public
         property fieldoffset: aint read ffieldoffset;
         property def: tdef read fdef;
       end;

       tllvmshadowsymtable = class;
{$endif llvm}

       tmanagementoperator_offset_entry = record
         pd : tprocdef;
         offset : asizeint;
       end;
       pmanagementoperator_offset_entry = ^tmanagementoperator_offset_entry;

       tabstractrecordsymtable = class(tstoredsymtable)
{$ifdef llvm}
       private
         fllvmst: tllvmshadowsymtable;
         function getllvmshadowsymtabll: tllvmshadowsymtable;
{$endif llvm}
       public
          usefieldalignment,     { alignment to use for fields (PACKRECORDS value), C_alignment is C style }
          recordalignment,       { alignment desired when inserting this record }
          fieldalignment,        { alignment current alignment used when fields are inserted }
          padalignment : shortint;   { size to a multiple of which the symtable has to be rounded up }
          recordalignmin: shortint; { local equivalentsof global settings, so that records can be created with custom settings internally }
          has_fields_with_mop : tmanagementoperators; { whether any of the fields has the need for a management operator (or one of the field's fields) }
          constructor create(const n:string;usealign,recordminalign:shortint);
          destructor destroy;override;
          procedure ppuload(ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure alignrecord(fieldoffset:asizeint;varalign:shortint);
          procedure addfield(sym:tfieldvarsym;vis:tvisibility);
          procedure addfieldlist(list: tfpobjectlist; maybereorder: boolean);
          { returns the field closest to this offset (may not be exact because
            of padding; internalerrors for variant records, assumes fields are
            ordered by increasing offset) }
          function findfieldbyoffset(offset:asizeint): tfieldvarsym;
          procedure addalignmentpadding;
          procedure insertdef(def:TDefEntry);override;
          function is_packed: boolean;
          function has_single_field(out def:tdef): boolean;
          function get_unit_symtable: tsymtable;
          { collects all management operators of the specified type in list (which
            is not cleared); the entries are copies and thus must be freed by the
            caller }
          procedure get_managementoperator_offset_list(mop:tmanagementoperator;list:tfplist);
        protected
          { size in bytes including padding }
          _datasize      : asizeint;
          { size in bits of the data in case of bitpacked record. Only important during construction, }
          { no need to save in/restore from ppu file. datasize is always (databitsize+7) div 8.       }
          databitsize    : asizeint;
          { size in bytes of padding }
          _paddingsize   : word;
          { array of tmanagementoperator_offset_entry lists; only assigned if
            they had been queried once by get_management_operator_list }
          mop_list : array[tmanagementoperator] of tfplist;
          procedure setdatasize(val: asizeint);
          function getfieldoffset(sym: tfieldvarsym; base: asizeint; var globalfieldalignment: shortint): asizeint;
          procedure do_get_managementoperator_offset_list(data:tobject;arg:pointer);
        public
          function iscurrentunit: boolean; override;
          property datasize : asizeint read _datasize write setdatasize;
          property paddingsize: word read _paddingsize write _paddingsize;
{$ifdef llvm}
          property llvmst: tllvmshadowsymtable read getllvmshadowsymtabll;
{$endif llvm}
       end;

       trecordsymtable = class(tabstractrecordsymtable)
       public
          { maybe someday is worth to move managementoperators to              }
          { tabstractrecordsymtable to perform management class operators for  }
          { object/classes. In XE5 and newer is possible to use class operator }
          { for classes (like for Delphi .NET before) only for Delphi NEXTGEN  }
          managementoperators : tmanagementoperators;
          constructor create(const n:string;usealign,recordminalign:shortint);
          procedure insertunionst(unionst : trecordsymtable;offset : asizeint);
          procedure includemanagementoperator(mop:tmanagementoperator);
       end;

       tObjectSymtable = class(tabstractrecordsymtable)
       public
          constructor create(adefowner:tdef;const n:string;usealign,recordminalign:shortint);
          function  checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;override;
       end;

{$ifdef llvm}
       { llvm record definitions cannot contain variant/union parts, }
       { you have to flatten them first. the tllvmshadowsymtable     }
       { contains a flattened version of a record/object symtable    }
       tllvmshadowsymtable = class
        private
         equivst: tabstractrecordsymtable;
         curroffset: aint;
         function get(f: tfieldvarsym): tllvmshadowsymtableentry;
         function get_by_llvm_index(index: longint): tllvmshadowsymtableentry;
        public
         symdeflist: TFPObjectList;

         constructor create(st: tabstractrecordsymtable);
         destructor destroy; override;

         property entries[index: tfieldvarsym]: tllvmshadowsymtableentry read get; default;
         { warning: do not call this with field.llvmfieldnr, as
             field.llvmfieldnr will only be initialised when the llvm shadow
             symtable is accessed for the first time. Use the default/entries
             property instead in this case }
         property entries_by_llvm_index[index: longint]: tllvmshadowsymtableentry read get_by_llvm_index;
        private
         // generate the table
         procedure generate;
         // helpers
         procedure appenddefoffset(vardef:tdef; fieldoffset: aint; derefclass: boolean);
         procedure findvariantstarts(variantstarts: tfplist);
         procedure addalignmentpadding(finalsize: aint);
         procedure buildmapping(variantstarts: tfplist);
         procedure buildtable(variantstarts: tfplist);
       end;
{$endif llvm}

       { tabstractsubsymtable }

       tabstractsubsymtable = class(tstoredsymtable)
       public
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       { tabstractlocalsymtable }

       tabstractlocalsymtable = class(tabstractsubsymtable)
       public
          function count_locals:longint;
          function iscurrentunit: boolean; override;
       end;

       tlocalsymtable = class(tabstractlocalsymtable)
       public
          constructor create(adefowner:tdef;level:byte);
          function checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;override;
       end;

       { tparasymtable }

       tparasymtable = class(tabstractlocalsymtable)
       public
          readonly: boolean;
          constructor create(adefowner:tdef;level:byte);
          function checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;override;
          procedure insertdef(def:TDefEntry);override;
       end;

       tabstractuniTSymtable = class(tstoredsymtable)
       public
          constructor create(const n : string;id:word);
          function checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;override;
          function findnamespace(const n:string):TSymEntry;virtual;
          function iscurrentunit:boolean;override;
          function needs_init_final: boolean; override;
          procedure insertunit(sym:TSymEntry);
          function has_class_condestructors: boolean;
       end;

       tglobalsymtable = class(tabstractuniTSymtable)
       public
          unittypecount : word;
          constructor create(const n : string;id:word);
          procedure ppuload(ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       tstaticsymtable = class(tabstractuniTSymtable)
       public
          constructor create(const n : string;id:word);
          procedure ppuload(ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;override;
          function findnamespace(const n:string):TSymEntry;override;
       end;

       tspecializesymtable = class(tglobalsymtable)
       public
          constructor create(const n : string;id:word);
          function iscurrentunit:boolean;override;
       end;

       twithsymtable = class(TSymtable)
          withrefnode : tobject; { tnode }
          constructor create(aowner:tdef;ASymList:TFPHashObjectList;refnode:tobject{tnode});
          destructor  destroy;override;
          procedure clear;override;
          procedure insertdef(def:TDefEntry);override;
        end;

       tstt_excepTSymtable = class(TSymtable)
       public
          constructor create;
       end;

       tmacrosymtable = class(tstoredsymtable)
       public
          constructor create(exported: boolean);
       end;

       { tenumsymtable }

       tenumsymtable = class(tabstractsubsymtable)
       public
          procedure insert(sym: TSymEntry; checkdup: boolean = true); override;
          constructor create(adefowner:tdef);
       end;

       { tarraysymtable }

       tarraysymtable = class(tabstractsubsymtable)
       public
          procedure insertdef(def:TDefEntry);override;
          constructor create(adefowner:tdef);
       end;

    var
       systemunit     : tglobalsymtable; { pointer to the system unit }

    type
       tsymbol_search_flag = (
         ssf_search_option,
         ssf_search_helper,
         ssf_has_inherited,
         ssf_no_addsymref
       );
       tsymbol_search_flags = set of tsymbol_search_flag;


{****************************************************************************
                             Functions
****************************************************************************}

{*** Misc ***}
    function  FullTypeName(def,otherdef:tdef):string;
    function generate_nested_name(symtable:tsymtable;delimiter:string):string;
    { def is the extended type of a helper }
    function generate_objectpascal_helper_key(def:tdef):string;
    procedure incompatibletypes(def1,def2:tdef);
    procedure hidesym(sym:TSymEntry);
    procedure duplicatesym(var hashedid: THashedIDString; dupsym, origsym:TSymEntry; warn: boolean);
    function handle_generic_dummysym(sym:TSymEntry;var symoptions:tsymoptions):boolean;
    { writes all declarations for the specified system unit symbol }
    procedure write_system_parameter_lists(const name:string);

{*** Search ***}
    procedure addsymref(sym:tsym);inline;
    procedure addsymref(sym:tsym;def:tdef);
    function  is_owned_by(nesteddef,ownerdef:tdef):boolean;
    function  sym_is_owned_by(childsym:tsym;symtable:tsymtable):boolean;
    function  defs_belong_to_same_generic(def1,def2:tdef):boolean;
    function  get_generic_in_hierarchy_by_name(srsym:tsym;def:tdef):tdef;
    function  return_specialization_of_generic(nesteddef,genericdef:tdef;out resultdef:tdef):boolean;
    function  is_visible_for_object(symst:tsymtable;symvisibility:tvisibility;contextobjdef:tabstractrecorddef):boolean;
    function  is_visible_for_object(pd:tprocdef;contextobjdef:tabstractrecorddef):boolean;
    function  is_visible_for_object(sym:tsym;contextobjdef:tabstractrecorddef):boolean;
    function  searchsym(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
    function  searchsym_with_flags(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags):boolean;
    function  searchsym_maybe_with_symoption(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags;option:tsymoption):boolean;
    { searches for a symbol with the given name that has the given option in
      symoptions set }
    function  searchsym_with_symoption(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;option:tsymoption):boolean;
    function  searchsym_type(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
    function  searchsym_in_module(pm:pointer;const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
    function  searchsym_in_named_module(const unitname, symname: TIDString; out srsym: tsym; out srsymtable: tsymtable): boolean;
    function  searchsym_in_class(classh: tobjectdef; contextclassh:tabstractrecorddef;const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags):boolean;
    function  searchsym_in_record(recordh:tabstractrecorddef;const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
    function  searchsym_in_class_by_msgint(classh:tobjectdef;msgid:longint;out srdef : tdef;out srsym:tsym;out srsymtable:TSymtable):boolean;
    function  searchsym_in_class_by_msgstr(classh:tobjectdef;const s:string;out srsym:tsym;out srsymtable:TSymtable):boolean;
    { searches symbols inside of a helper's implementation }
    function  searchsym_in_helper(classh,contextclassh:tobjectdef;const s: TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags):boolean;
    function  search_system_type(const s: TIDString): ttypesym;
    function  try_search_system_type(const s: TIDString): ttypesym;
    function  try_search_current_module_type(const s: TIDString): ttypesym;
    function  search_system_proc(const s: TIDString): tprocdef;
    function  search_named_unit_globaltype(const unitname, typename: TIDString; throwerror: boolean): ttypesym;
    function  search_struct_member(pd : tabstractrecorddef;const s : string):tsym;
    function  search_struct_member_no_helper(pd : tabstractrecorddef;const s : string):tsym;
    function  search_assignment_operator(from_def,to_def:Tdef;explicit:boolean):Tprocdef;
    function  search_enumerator_operator(from_def,to_def:Tdef):Tprocdef;
    function  search_management_operator(mop:tmanagementoperator;pd:Tdef):Tprocdef;
    { searches for the helper definition that's currently active for pd }
    function  search_last_objectpascal_helper(pd : tdef;contextclassh : tabstractrecorddef;out odef : tobjectdef):boolean;
    { searches whether the symbol s is available in the currently active }
    { helper for pd }
    function  search_objectpascal_helper(pd : tdef;contextclassh : tabstractrecorddef;const s : string; out srsym: tsym; out srsymtable: tsymtable):boolean;
    function  search_objc_helper(pd : tobjectdef;const s : string; out srsym: tsym; out srsymtable: tsymtable):boolean;
    function  search_objc_method(const s : string; out srsym: tsym; out srsymtable: tsymtable):boolean;
    {Looks for macro s (must be given in upper case) in the macrosymbolstack, }
    {and returns it if found. Returns nil otherwise.}
    function  search_macro(const s : string):tsym;
    { Additionally to searching for a macro, also checks whether it's still }
    { actually defined (could be disable using "undef")                     }
    function  defined_macro(const s : string):boolean;
    { Look for a system procedure (no overloads supported) }
    { returns a list of helpers in the current module for the def }
    function get_objectpascal_helpers(pd : tdef):TFPObjectList;

{*** Object Helpers ***}
    function search_default_property(pd : tabstractrecorddef) : tpropertysym;
    function maybe_find_real_class_definition(pd: tdef; erroronfailure: boolean): tdef;
    function find_real_class_definition(pd: tobjectdef; erroronfailure: boolean): tobjectdef;

{*** Macro Helpers ***}
    {If called initially, the following procedures manipulate macros in }
    {initialmacrotable, otherwise they manipulate system macros local to a module.}
    {Name can be given in any case (it will be converted to upper case).}
    procedure def_system_macro(const name : string);
    procedure set_system_macro(const name, value : string);
    procedure set_system_compvar(const name, value : string);
    procedure undef_system_macro(const name : string);

{*** symtable stack ***}
{ $ifdef DEBUG
    procedure test_symtablestack;
    procedure list_symtablestack;
 $endif DEBUG}

{$ifdef UNITALIASES}
    type
       punit_alias = ^tunit_alias;
       tunit_alias = object(TNamedIndexItem)
          newname : pshortstring;
          constructor init(const n:string);
          destructor  done;virtual;
       end;
    var
       unitaliases : pdictionary;

    procedure addunitalias(const n:string);
    function getunitalias(const n:string):string;
{$endif UNITALIASES}

{*** Init / Done ***}
    procedure IniTSymtable;
    procedure DoneSymtable;

    const
      overloaded_names : array [NOTOKEN..last_overloaded] of string[16] = (
    { NOTOKEN        }  'error',
    { _PLUS          }  'plus',
    { _MINUS         }  'minus',
    { _STAR          }  'star',
    { _SLASH         }  'slash',
    { _EQ            }  'equal',
    { _GT            }  'greater',
    { _LT            }  'lower',
    { _GTE           }  'greater_or_equal',
    { _LTE           }  'lower_or_equal',
    { _NE            }  'not_equal',
    { _SYMDIF        }  'sym_diff',
    { _STARSTAR      }  'starstar',
    { _OP_AS         }  'as',
    { _OP_IN         }  'in',
    { _OP_IS         }  'is',
    { _OP_OR         }  'or',
    { _OP_AND        }  'and',
    { _OP_DIV        }  'div',
    { _OP_MOD        }  'mod',
    { _OP_NOT        }  'not',
    { _OP_SHL        }  'shl',
    { _OP_SHR        }  'shr',
    { _OP_XOR        }  'xor',
    { _ASSIGNMENT    }  'assign',
    { _OP_EXPLICIT   }  'explicit',
    { _OP_ENUMERATOR }  'enumerator',
    { _OP_INITIALIZE }  'initialize',
    { _OP_FINALIZE   }  'finalize',
    { _OP_ADDREF     }  'addref',
    { _OP_COPY       }  'copy',
    { _OP_INC        }  'inc',
    { _OP_DEC        }  'dec');

      managementoperator2tok:array[tmanagementoperator] of ttoken = (
    { mop_none       }  NOTOKEN,
    { mop_initialize }  _OP_INITIALIZE,
    { mop_finalize   }  _OP_FINALIZE,
    { mop_addref     }  _OP_ADDREF,
    { mop_copy       }  _OP_COPY
    );



implementation

    uses
      { global }
      verbose,globals,systems,
      { symtable }
      symutil,defutil,defcmp,objcdef,
      { module }
      fmodule,
      { codegen }
      procinfo,
      { ppu }
      entfile,ppu,
      { parser }
      scanner
      ;


    var
      dupnr : longint; { unique number for duplicate symbols }

{*****************************************************************************
                             TStoredSymtable
*****************************************************************************}

    constructor tstoredsymtable.create(const s:string);
      begin
        inherited create(s);
        { Note: this happens for the initial macro symtable, so no error here }
        if not assigned(current_module) then
          comment(v_debug,'Current module not available for module id')
        else
          moduleid:=current_module.moduleid;
      end;


    procedure tstoredsymtable.insert(sym:TSymEntry;checkdup:boolean=true);
      begin
        inherited insert(sym,checkdup);
        init_final_check_done:=false;
      end;


    procedure tstoredsymtable.delete(sym:TSymEntry);
      begin
        inherited delete(sym);
        init_final_check_done:=false;
      end;


    procedure tstoredsymtable.ppuload(ppufile:tcompilerppufile);
      begin
        { load the table's flags }
        if ppufile.readentry<>ibsymtableoptions then
          Message(unit_f_ppu_read_error);
        ppufile.getset(tppuset1(tableoptions));

        { load definitions }
        loaddefs(ppufile);

        { load symbols }
        loadsyms(ppufile);

        init_final_check_done:=true;
      end;


    procedure tstoredsymtable.ppuwrite(ppufile:tcompilerppufile);
      begin
         { ensure that we have the sto_needs_init_final flag set if needed }
         if not init_final_check_done then
           needs_init_final;

         { write the table's flags }
         ppufile.putset(tppuset1(tableoptions));
         ppufile.writeentry(ibsymtableoptions);

         { write definitions }
         writedefs(ppufile);

         { write symbols }
         writesyms(ppufile);
      end;


    procedure tstoredsymtable.loaddefs(ppufile:tcompilerppufile);
      var
        def : tdef;
        b   : byte;
      begin
         { load start of definition section, which holds the amount of defs }
         if ppufile.readentry<>ibstartdefs then
           Message(unit_f_ppu_read_error);
         { read definitions }
         repeat
           b:=ppufile.readentry;
           def:=nil;
           case b of
             ibpointerdef : def:=cpointerdef.ppuload(ppufile);
             ibarraydef : def:=carraydef.ppuload(ppufile);
             iborddef : def:=corddef.ppuload(ppufile);
             ibfloatdef : def:=cfloatdef.ppuload(ppufile);
             ibprocdef : def:=cprocdef.ppuload(ppufile);
             ibshortstringdef : def:=cstringdef.loadshort(ppufile);
             iblongstringdef : def:=cstringdef.loadlong(ppufile);
             ibansistringdef : def:=cstringdef.loadansi(ppufile);
             ibwidestringdef : def:=cstringdef.loadwide(ppufile);
             ibunicodestringdef : def:=cstringdef.loadunicode(ppufile);
             ibrecorddef : def:=crecorddef.ppuload(ppufile);
             ibobjectdef : def:=cobjectdef.ppuload(ppufile);
             ibenumdef : def:=cenumdef.ppuload(ppufile);
             ibsetdef : def:=csetdef.ppuload(ppufile);
             ibprocvardef : def:=cprocvardef.ppuload(ppufile);
             ibfiledef : def:=cfiledef.ppuload(ppufile);
             ibclassrefdef : def:=cclassrefdef.ppuload(ppufile);
             ibformaldef : def:=cformaldef.ppuload(ppufile);
             ibvariantdef : def:=cvariantdef.ppuload(ppufile);
             ibundefineddef : def:=cundefineddef.ppuload(ppufile);
             ibenddefs : break;
             ibend : Message(unit_f_ppu_read_error);
           else
             Message1(unit_f_ppu_invalid_entry,tostr(b));
           end;
           if assigned(def) then
             tstoreddef(def).ppuload_subentries(ppufile);
           InsertDef(def);
         until false;
      end;


    procedure tstoredsymtable.loadsyms(ppufile:tcompilerppufile);
      var
        b   : byte;
        sym : tsym;
      begin
         { load start of definition section, which holds the amount of defs }
         if ppufile.readentry<>ibstartsyms then
          Message(unit_f_ppu_read_error);
         { now read the symbols }
         repeat
           sym:=nil;
           b:=ppufile.readentry;
           case b of
                ibtypesym : sym:=ctypesym.ppuload(ppufile);
                ibprocsym : sym:=cprocsym.ppuload(ppufile);
               ibconstsym : sym:=cconstsym.ppuload(ppufile);
           ibstaticvarsym : sym:=cstaticvarsym.ppuload(ppufile);
            iblocalvarsym : sym:=clocalvarsym.ppuload(ppufile);
             ibparavarsym : sym:=cparavarsym.ppuload(ppufile);
            ibfieldvarsym : sym:=cfieldvarsym.ppuload(ppufile);
         ibabsolutevarsym : sym:=cabsolutevarsym.ppuload(ppufile);
                ibenumsym : sym:=cenumsym.ppuload(ppufile);
            ibpropertysym : sym:=cpropertysym.ppuload(ppufile);
                ibunitsym : sym:=cunitsym.ppuload(ppufile);
               iblabelsym : sym:=clabelsym.ppuload(ppufile);
                 ibsyssym : sym:=csyssym.ppuload(ppufile);
               ibmacrosym : sym:=tmacro.ppuload(ppufile);
           ibnamespacesym : sym:=cnamespacesym.ppuload(ppufile);
                ibendsyms : break;
                    ibend : Message(unit_f_ppu_read_error);
           else
             Message1(unit_f_ppu_invalid_entry,tostr(b));
           end;
           if assigned(sym) then
             tstoredsym(sym).ppuload_subentries(ppufile);
           Insert(sym,false);
         until false;
      end;


    procedure tstoredsymtable.writedefs(ppufile:tcompilerppufile);
      var
        defcount,
        i   : longint;
        def : tstoreddef;
      begin
        defcount:=0;
        for i:=0 to DefList.Count-1 do
          if tstoreddef(DefList[i]).is_registered then
            inc(defcount);
        { each definition get a number, write then the amount of defs to the
          ibstartdef entry }
        ppufile.putlongint(defcount);
        ppufile.writeentry(ibstartdefs);
        { now write the definition }
        for i:=0 to DefList.Count-1 do
          begin
            def:=tstoreddef(DefList[i]);
            if def.is_registered then
              begin
                def.ppuwrite(ppufile);
                def.ppuwrite_subentries(ppufile);
              end;
          end;
        { write end of definitions }
        ppufile.writeentry(ibenddefs);
      end;


    procedure tstoredsymtable.writesyms(ppufile:tcompilerppufile);
      var
        symcount,
        i   : longint;
        sym : Tstoredsym;
      begin
        symcount:=0;
        for i:=0 to SymList.Count-1 do
          if tstoredsym(SymList[i]).is_registered then
            inc(symcount);
        { each definition get a number, write then the amount of syms and the
          datasize to the ibsymdef entry }
        ppufile.putlongint(symcount);
        ppufile.writeentry(ibstartsyms);
        { foreach is used to write all symbols }
        for i:=0 to SymList.Count-1 do
          begin
            sym:=tstoredsym(SymList[i]);
            if sym.is_registered then
              begin
                sym.ppuwrite(ppufile);
                sym.ppuwrite_subentries(ppufile);
              end;
          end;
        { end of symbols }
        ppufile.writeentry(ibendsyms);
      end;


    procedure tstoredsymtable.buildderef;
      var
        i   : longint;
        def : tstoreddef;
        sym : tstoredsym;
      begin
        { interface definitions }
        for i:=0 to DefList.Count-1 do
          begin
            def:=tstoreddef(DefList[i]);
            def.buildderef;
          end;
        { interface symbols }
        for i:=0 to SymList.Count-1 do
          begin
            sym:=tstoredsym(SymList[i]);
            sym.buildderef;
          end;
      end;


    procedure tstoredsymtable.buildderefimpl;
      var
        i   : longint;
        def : tstoreddef;
      begin
        { implementation definitions }
        for i:=0 to DefList.Count-1 do
          begin
            def:=tstoreddef(DefList[i]);
            def.buildderefimpl;
          end;
      end;


    procedure tstoredsymtable.buildderef_registered;
      var
        def : tstoreddef;
        sym : tstoredsym;
        i   : longint;
        defidmax,
        symidmax: longint;
        newbuiltdefderefs,
        builtdefderefs,
        builtsymderefs: array of boolean;
        changed: boolean;
      begin
        newbuiltdefderefs:=nil;
        builtdefderefs:=nil;
        builtsymderefs:=nil;
        { tdefs for which we already built the deref }
        setlength(builtdefderefs,deflist.count);
        { tdefs for which we built the deref in this iteration }
        setlength(newbuiltdefderefs,deflist.count);
        { syms for which we already built the deref }
        setlength(builtsymderefs,symlist.count);
        repeat
          { we only have to store the defs (recursively) referred by wpo info
            or inlined routines in the static symbtable }

          { current number of registered defs/syms }
          defidmax:=current_module.deflist.count;
          symidmax:=current_module.symlist.count;
          changed:=false;

          { build the derefs for the registered defs we haven't processed yet }
          for i:=0 to DefList.Count-1 do
            begin
              if not builtdefderefs[i] then
                begin
                  def:=tstoreddef(DefList[i]);
                  if def.is_registered then
                    begin
                      def.buildderef;
                      newbuiltdefderefs[i]:=true;
                      builtdefderefs[i]:=true;
                      changed:=true;
                    end;
                end;
            end;
          { same for the syms }
          for i:=0 to SymList.Count-1 do
            begin
              if not builtsymderefs[i] then
                begin
                  sym:=tstoredsym(SymList[i]);
                  if sym.is_registered then
                    begin
                      sym.buildderef;
                      builtsymderefs[i]:=true;
                      changed:=true;
                    end;
                end;
            end;
          { now buildderefimpl for the defs we processed in this iteration }
          for i:=0 to DefList.Count-1 do
            begin
              if newbuiltdefderefs[i] then
                begin
                  newbuiltdefderefs[i]:=false;
                  tstoreddef(DefList[i]).buildderefimpl;
                  changed:=true;
                end;
            end;
        { stop when no new defs or syms have been registered while processing
          the currently registered ones (defs/syms get added to the module's
          deflist/symlist when they are registered) }
        until not changed and 
          (defidmax=current_module.deflist.count) and
          (symidmax=current_module.symlist.count);
      end;


    procedure tstoredsymtable.deref(only_registered: boolean);
      var
        i   : longint;
        def : tstoreddef;
        sym : tstoredsym;
      begin
        { first deref the interface ttype symbols. This is needs
          to be done before the interface defs are derefed, because
          the interface defs can contain references to the type symbols
          which then already need to contain a resolved typedef field (PFV) }
        for i:=0 to SymList.Count-1 do
          begin
            sym:=tstoredsym(SymList[i]);
            if (sym.typ=typesym) and
               (not only_registered or
                sym.is_registered) then
              sym.deref;
          end;
        { interface definitions }
        for i:=0 to DefList.Count-1 do
          begin
            def:=tstoreddef(DefList[i]);
            if not only_registered or
               def.is_registered then
              def.deref;
          end;
        { interface symbols }
        for i:=0 to SymList.Count-1 do
          begin
            sym:=tstoredsym(SymList[i]);
            if (not only_registered or
                sym.is_registered) and
               (sym.typ<>typesym) then
              sym.deref;
          end;
      end;


    procedure tstoredsymtable.derefimpl(only_registered: boolean);
      var
        i   : longint;
        def : tstoreddef;
      begin
        { implementation definitions }
        for i:=0 to DefList.Count-1 do
          begin
            def:=tstoreddef(DefList[i]);
            if not only_registered or
               def.is_registered then
              def.derefimpl;
          end;
      end;


    function tstoredsymtable.checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;
      var
        hsym : tsym;
      begin
        hsym:=tsym(FindWithHash(hashedid));
        if assigned(hsym) then
          DuplicateSym(hashedid,sym,hsym,false);
        result:=assigned(hsym);
      end;


{**************************************
             Callbacks
**************************************}

    procedure TStoredSymtable.check_forward(sym:TObject;arg:pointer);
      begin
         if tsym(sym).typ=procsym then
           tprocsym(sym).check_forward
         { check also object method table            }
         { we needn't to test the def list          }
         { because each object has to have a type sym,
           only test objects declarations, not type renamings }
         else
          if (tsym(sym).typ=typesym) and
             assigned(ttypesym(sym).typedef) and
             (ttypesym(sym).typedef.typesym=ttypesym(sym)) and
             (ttypesym(sym).typedef.typ in [objectdef,recorddef]) then
           tabstractrecorddef(ttypesym(sym).typedef).check_forwards;
      end;


    procedure tstoredsymtable.check_block_valid(def: TObject; arg: pointer);
      var
        founderrordef: tdef;
      begin
        { all parameters passed to a block must be handled by the Objective-C
          runtime }
        if is_block(tdef(def)) and
           not objcchecktype(tdef(def),founderrordef) then
          if assigned(tdef(def).typesym) then
            MessagePos1(tdef(def).typesym.fileinfo,type_e_objc_type_unsupported,founderrordef.typename)
          else
            Message1(type_e_objc_type_unsupported,tprocvardef(def).typename)
      end;


    procedure tstoredsymtable.register_syms(sym:tobject;arg:pointer);
      begin
        tsym(sym).register_sym;
      end;


    procedure tstoredsymtable.register_defs(def:tobject;arg:pointer);
      begin
        tdef(def).register_def;
      end;


    procedure TStoredSymtable.labeldefined(sym:TObject;arg:pointer);
      begin
        if (tsym(sym).typ=labelsym) and
           not(tlabelsym(sym).defined) then
         begin
           if tlabelsym(sym).used then
            Message1(sym_e_label_used_and_not_defined,tlabelsym(sym).realname)
           else
            Message1(sym_w_label_not_defined,tlabelsym(sym).realname);
         end;
      end;


    procedure TStoredSymtable.varsymbolused(sym:TObject;arg:pointer);
      begin
         if (tsym(sym).typ in [staticvarsym,localvarsym,paravarsym,fieldvarsym]) and
            ((tsym(sym).owner.symtabletype in
             [parasymtable,localsymtable,ObjectSymtable,recordsymtable,staticsymtable])) then
           begin
            { unused symbol should be reported only if no                    }
            { error is reported                                              }
            { if the symbol is in a register it is used                      }
            { also don't count the value parameters which have local copies  }
            { also don't claim for high param of open parameters    (PM)     }
            { also don't complain about unused symbols in generic procedures }
            { and methods                                                    }
            { and neither in abstract methods                                }
            if (Errorcount<>0) or
               ([vo_is_hidden_para,vo_is_funcret] * tabstractvarsym(sym).varoptions = [vo_is_hidden_para]) or
               (sp_internal in tsym(sym).symoptions) or
               ((assigned(tsym(sym).owner.defowner) and
                (tsym(sym).owner.defowner.typ=procdef) and
                ((df_generic in tprocdef(tsym(sym).owner.defowner).defoptions) or
                 (po_abstractmethod in tprocdef(tsym(sym).owner.defowner).procoptions)))) then
              exit;
            if (tstoredsym(sym).refs=0) then
              begin
                 if (vo_is_funcret in tabstractvarsym(sym).varoptions) then
                   begin
                     { don't warn about the result of constructors }
                     { or the synthetic helper functions for class-attributes }
                     if ((tsym(sym).owner.symtabletype<>localsymtable) or
                        (tprocdef(tsym(sym).owner.defowner).proctypeoption<>potype_constructor)) and
                        not (po_noreturn in tprocdef(tsym(sym).owner.defowner).procoptions) and
                        not(cs_opt_nodedfa in current_settings.optimizerswitches) then
                       MessagePos(tsym(sym).fileinfo,sym_w_function_result_not_set)
                   end
                 else if (tsym(sym).owner.symtabletype=parasymtable) then
                   MessagePos1(tsym(sym).fileinfo,sym_h_para_identifier_not_used,tsym(sym).prettyname)
                 else if (tsym(sym).owner.symtabletype in [ObjectSymtable,recordsymtable]) then
                   MessagePos2(tsym(sym).fileinfo,sym_n_private_identifier_not_used,tabstractrecorddef(tsym(sym).owner.defowner).GetTypeName,tsym(sym).prettyname)
                 else
                   MessagePos1(tsym(sym).fileinfo,sym_n_local_identifier_not_used,tsym(sym).prettyname);
              end
            else if tabstractvarsym(sym).varstate in [vs_written,vs_initialised] then
              begin
                 if (tsym(sym).owner.symtabletype=parasymtable) then
                   begin
                     if not(tabstractvarsym(sym).varspez in [vs_var,vs_out,vs_constref]) and
                        not(vo_is_funcret in tabstractvarsym(sym).varoptions) then
                       MessagePos1(tsym(sym).fileinfo,sym_h_para_identifier_only_set,tsym(sym).prettyname)
                   end
                 else if (tsym(sym).owner.symtabletype in [ObjectSymtable,recordsymtable]) then
                   MessagePos2(tsym(sym).fileinfo,sym_n_private_identifier_only_set,tabstractrecorddef(tsym(sym).owner.defowner).GetTypeName,tsym(sym).prettyname)
                 else if tabstractvarsym(sym).varoptions*[vo_is_funcret,vo_is_public,vo_is_external]=[] then
                   MessagePos1(tsym(sym).fileinfo,sym_n_local_identifier_only_set,tsym(sym).prettyname);
              end
            else if (tabstractvarsym(sym).varstate = vs_read_not_warned) and
                    ([vo_is_public,vo_is_external] * tabstractvarsym(sym).varoptions = []) then
              MessagePos1(tsym(sym).fileinfo,sym_w_identifier_only_read,tsym(sym).prettyname)
          end
        else if ((tsym(sym).owner.symtabletype in
              [ObjectSymtable,parasymtable,localsymtable,staticsymtable,recordsymtable])) then
          begin
           if (Errorcount<>0) or
              (sp_internal in tsym(sym).symoptions) then
             exit;
           { do not claim for inherited private fields !! }
           if (tsym(sym).refs=0) and (tsym(sym).owner.symtabletype in [ObjectSymtable,recordsymtable]) then
             case tsym(sym).typ of
               typesym:
                 MessagePos2(tsym(sym).fileinfo,sym_n_private_type_not_used,tabstractrecorddef(tsym(sym).owner.defowner).GetTypeName,tsym(sym).prettyname);
               constsym:
                 MessagePos2(tsym(sym).fileinfo,sym_n_private_const_not_used,tabstractrecorddef(tsym(sym).owner.defowner).GetTypeName,tsym(sym).prettyname);
               propertysym:
                 MessagePos2(tsym(sym).fileinfo,sym_n_private_property_not_used,tabstractrecorddef(tsym(sym).owner.defowner).GetTypeName,tsym(sym).prettyname);
             else
               MessagePos2(tsym(sym).fileinfo,sym_n_private_method_not_used,tabstractrecorddef(tsym(sym).owner.defowner).GetTypeName,tsym(sym).prettyname);
             end
           { units references are problematic }
           else
            begin
              if (tsym(sym).refs=0) and
                 not(tsym(sym).typ in [enumsym,unitsym,namespacesym]) and
                 not(is_funcret_sym(tsym(sym))) and
                 { don't complain about compiler generated syms for specializations, see also #13405 }
                 not((tsym(sym).typ=typesym) and (df_specialization in ttypesym(sym).typedef.defoptions) and
                    (pos('$',ttypesym(sym).Realname)<>0)) and
                 (
                  (tsym(sym).typ<>procsym) or
                  ((tsym(sym).owner.symtabletype=staticsymtable) and
                   not current_module.is_unit)
                 ) and
                 { don't complain about alias for hidden _cmd parameter to
                   obj-c methods }
                 not((tsym(sym).typ in [localvarsym,paravarsym,absolutevarsym]) and
                     (vo_is_msgsel in tabstractvarsym(sym).varoptions)) then
                MessagePos2(tsym(sym).fileinfo,sym_h_local_symbol_not_used,SymTypeName[tsym(sym).typ],tsym(sym).prettyname);
            end;
          end;
      end;


    procedure TStoredSymtable.TestPrivate(sym:TObject;arg:pointer);
      begin
        if tsym(sym).visibility in [vis_private,vis_strictprivate] then
          varsymbolused(sym,arg);
      end;


    procedure TStoredSymtable.objectprivatesymbolused(sym:TObject;arg:pointer);
      begin
         {
           Don't test simple object aliases PM
         }
         if (tsym(sym).typ=typesym) and
            (ttypesym(sym).typedef.typ in [objectdef,recorddef]) and
            (ttypesym(sym).typedef.typesym=tsym(sym)) then
           tabstractrecorddef(ttypesym(sym).typedef).symtable.SymList.ForEachCall(@TestPrivate,nil);
      end;


   procedure tstoredsymtable.testfordefaultproperty(sym:TObject;arg:pointer);
     begin
        if (tsym(sym).typ=propertysym) and
           (ppo_defaultproperty in tpropertysym(sym).propoptions) then
          ppointer(arg)^:=sym;
     end;


   procedure tstoredsymtable.register_children;
     begin
       SymList.ForEachCall(@register_syms,nil);
       DefList.ForEachCall(@register_defs,nil);
     end;


{***********************************************
           Process all entries
***********************************************}

    { checks, if all procsyms and methods are defined }
    procedure tstoredsymtable.check_forwards;
      begin
         SymList.ForEachCall(@check_forward,nil);
         { check whether all block definitions contain valid Objective-C types
           (now that all forward definitions have been resolved) }
         DefList.ForEachCall(@check_block_valid,nil);
      end;


    procedure tstoredsymtable.checklabels;
      begin
         SymList.ForEachCall(@labeldefined,nil);
      end;


    procedure tstoredsymtable.allsymbolsused;
      begin
         SymList.ForEachCall(@varsymbolused,nil);
      end;


    procedure tstoredsymtable.allprivatesused;
      begin
         SymList.ForEachCall(@objectprivatesymbolused,nil);
      end;


    procedure TStoredSymtable._needs_init_final(sym:TObject;arg:pointer);
      begin
         if [sto_needs_init_final,sto_has_non_trivial_init] <= tableoptions then
           exit;
         { don't check static symbols - they can be present in structures only and
           always have a reference to a symbol defined on unit level }
         if sp_static in tsym(sym).symoptions then
           exit;
         case tsym(sym).typ of
           fieldvarsym,
           staticvarsym,
           localvarsym,
           paravarsym :
             begin
               if assigned(tabstractvarsym(sym).vardef) and
                  is_managed_type(tabstractvarsym(sym).vardef) then
                 include(tableoptions,sto_needs_init_final);
               if is_record((tabstractvarsym(sym).vardef)) and
                   (mop_initialize in trecordsymtable(trecorddef(tabstractvarsym(sym).vardef).symtable).managementoperators) then
                 include(tableoptions,sto_has_non_trivial_init);
             end;
           else
             ;
         end;
      end;


    procedure tstoredsymtable.do_init_final_check;
      begin
         if not init_final_check_done then
           begin
             exclude(tableoptions,sto_needs_init_final);
             exclude(tableoptions,sto_has_non_trivial_init);
             SymList.ForEachCall(@_needs_init_final,nil);
             init_final_check_done:=true;
           end;
      end;

    { returns true, if p contains data which needs init/final code }
    function tstoredsymtable.needs_init_final : boolean;
      begin
         do_init_final_check;
         result:=sto_needs_init_final in tableoptions;
      end;


    function tstoredsymtable.has_non_trivial_init:boolean;
      begin
        do_init_final_check;
        result:=sto_has_non_trivial_init in tableoptions;
      end;


{****************************************************************************
                          TAbstractRecordSymtable
****************************************************************************}

{$ifdef llvm}
    function tabstractrecordsymtable.getllvmshadowsymtabll: tllvmshadowsymtable;
      begin
        if not assigned(fllvmst) then
          fllvmst:=tllvmshadowsymtable.create(self);
        result:=fllvmst;
      end;
{$endif llvm}

    constructor tabstractrecordsymtable.create(const n:string;usealign,recordminalign:shortint);
      begin
        inherited create(n);
        _datasize:=0;
        databitsize:=0;
        recordalignment:=1;
        usefieldalignment:=usealign;
        recordalignmin:=recordminalign;
        padalignment:=1;
        { recordalign C_alignment means C record packing, that starts
          with an alignment of 1 }
        case usealign of
          C_alignment,
          bit_alignment:
            fieldalignment:=1;
          mac68k_alignment:
            fieldalignment:=2;
          else
            fieldalignment:=usealign;
        end;
      end;


    destructor tabstractrecordsymtable.destroy;

      { for some reason a compiler built with 3.3.1 fails building the libxml2
        package if the below define is not defined and thus the code snippet is
        part of the destructor itself and not a nested procedure; until that bug
        is fixed this is used as a workaround :/ }
{$define codegen_workaround}
{$ifdef codegen_workaround}
      procedure free_mop_list(mop:tmanagementoperator);
        var
          i : longint;
        begin
          if assigned(mop_list[mop]) then
            for i:=0 to mop_list[mop].count-1 do
              dispose(pmanagementoperator_offset_entry(mop_list[mop][i]));
          mop_list[mop].free;
        end;
{$endif codegen_workaround}

      var
        mop : tmanagementoperator;
{$ifndef codegen_workaround}
        i : longint;
{$endif codegen_workaround}
      begin
        if refcount>1 then
          exit;
{$ifdef llvm}
        fllvmst.free;
{$endif llvm}
        for mop:=low(tmanagementoperator) to high(tmanagementoperator) do
          begin
{$ifdef codegen_workaround}
            free_mop_list(mop);
{$else codegen_workaround}
            if assigned(mop_list[mop]) then
              for i:=0 to mop_list[mop].count-1 do
                dispose(pmanagementoperator_offset_entry(mop_list[mop][i]));
            mop_list[mop].free;
{$endif codegen_workaround}
          end;
        inherited destroy;
      end;


    procedure tabstractrecordsymtable.ppuload(ppufile:tcompilerppufile);
      begin
        if ppufile.readentry<>ibrecsymtableoptions then
          Message(unit_f_ppu_read_error);
        recordalignment:=shortint(ppufile.getbyte);
        usefieldalignment:=shortint(ppufile.getbyte);
        recordalignmin:=shortint(ppufile.getbyte);
        if (usefieldalignment=C_alignment) then
          fieldalignment:=shortint(ppufile.getbyte);
        ppufile.getset(tppuset1(has_fields_with_mop));
        inherited ppuload(ppufile);
      end;


    procedure tabstractrecordsymtable.ppuwrite(ppufile:tcompilerppufile);
      var
        oldtyp : byte;
      begin
         oldtyp:=ppufile.entrytyp;
         ppufile.entrytyp:=subentryid;
         { in case of classes using C alignment, the alignment of the parent
           affects the alignment of fields of the childs }
         ppufile.putbyte(byte(recordalignment));
         ppufile.putbyte(byte(usefieldalignment));
         ppufile.putbyte(byte(recordalignmin));
         if (usefieldalignment=C_alignment) then
           ppufile.putbyte(byte(fieldalignment));
         { it's not really a "symtableoption", but loading this from the record
           def requires storing the set in the recorddef at least between
           ppuload and deref/derefimpl }
         ppufile.putset(tppuset1(has_fields_with_mop));
         ppufile.writeentry(ibrecsymtableoptions);

         inherited ppuwrite(ppufile);

         ppufile.entrytyp:=oldtyp;
      end;


    function field2recordalignment(fieldoffs, fieldalign: asizeint): asizeint;
      begin
        { optimal alignment of the record when declaring a variable of this }
        { type is independent of the packrecords setting                    }
        if (fieldoffs mod fieldalign) = 0 then
          result:=fieldalign
        else if (fieldalign >= 16) and
                ((fieldoffs mod 16) = 0) and
                ((fieldalign mod 16) = 0) then
          result:=16
        else if (fieldalign >= 8) and
                ((fieldoffs mod 8) = 0) and
                ((fieldalign mod 8) = 0) then
          result:=8
        else if (fieldalign >= 4) and
                ((fieldoffs mod 4) = 0) and
                ((fieldalign mod 4) = 0) then
          result:=4
        else if (fieldalign >= 2) and
                ((fieldoffs mod 2) = 0) and
                ((fieldalign mod 2) = 0) then
          result:=2
        else
          result:=1;
      end;

    procedure tabstractrecordsymtable.alignrecord(fieldoffset:asizeint;varalign:shortint);
      var
        varalignrecord: shortint;
      begin
        case usefieldalignment of
          C_alignment:
            varalignrecord:=used_align(varalign,recordalignmin,current_settings.alignment.maxCrecordalign);
          mac68k_alignment:
            varalignrecord:=2;
          else
            varalignrecord:=field2recordalignment(fieldoffset,varalign);
        end;
        recordalignment:=max(recordalignment,varalignrecord);
      end;

    procedure tabstractrecordsymtable.addfield(sym:tfieldvarsym;vis:tvisibility);
      var
        l      : asizeint;
        varalign : shortint;
        vardef : tdef;
      begin
        if (sym.owner<>self) then
          internalerror(200602031);
        if sym.fieldoffset<>-1 then
          internalerror(200602032);
        { set visibility for the symbol }
        sym.visibility:=vis;
        { this symbol can't be loaded to a register }
        sym.varregable:=vr_none;
        { management operators }
        if sym.vardef.typ in [recorddef,objectdef] then
          has_fields_with_mop:=has_fields_with_mop + tabstractrecordsymtable(tabstractrecorddef(sym.vardef).symtable).has_fields_with_mop;
        if sym.vardef.typ=recorddef then
          has_fields_with_mop:=has_fields_with_mop + trecordsymtable(trecorddef(sym.vardef).symtable).managementoperators;
        { Calculate field offset }
        l:=sym.getsize;
        vardef:=sym.vardef;
        varalign:=vardef.structalignment;
        case usefieldalignment of
          bit_alignment:
            begin
              { bitpacking only happens for ordinals, the rest is aligned at }
              { 1 byte (compatible with GPC/GCC)                             }
              if is_ordinal(vardef) then
                begin
                  sym.fieldoffset:=databitsize;
                  l:=sym.getpackedbitsize;
                end
              else
                begin
                  databitsize:=_datasize*8;
                  sym.fieldoffset:=databitsize;
                  if (l>high(asizeint) div 8) then
                    Message(sym_e_segment_too_large);
                  l:=l*8;
                end;
              if varalign=0 then
                varalign:=size_2_align(l);
              recordalignment:=max(recordalignment,field2recordalignment(databitsize mod 8,varalign));
              { bit packed records are limited to high(aint) bits }
              { instead of bytes to avoid double precision        }
              { arithmetic in offset calculations                 }
              if int64(l)>high(asizeint)-sym.fieldoffset then
                begin
                  Message(sym_e_segment_too_large);
                  _datasize:=high(asizeint);
                  databitsize:=high(asizeint);
                end
              else
                begin
                  databitsize:=sym.fieldoffset+l;
                  _datasize:=(databitsize+7) div 8;
                end;
              { rest is not applicable }
              exit;
            end;
          else
            begin
              sym.fieldoffset:=getfieldoffset(sym,_datasize,fieldalignment);
              if l>high(asizeint)-sym.fieldoffset then
                begin
                  Message(sym_e_segment_too_large);
                  _datasize:=high(asizeint);
                end
              else
                _datasize:=sym.fieldoffset+l;
              { Calc alignment needed for this record }
              alignrecord(sym.fieldoffset,varalign);
            end;
        end;
      end;


    function field_alignment_compare(item1, item2: pointer): integer;
      var
        field1: tfieldvarsym absolute item1;
        field2: tfieldvarsym absolute item2;
      begin
        { we don't care about static fields, those become global variables }
        if (sp_static in field1.symoptions) or
           (sp_static in field2.symoptions) then
          exit(0);
        { sort from large to small alignment, and in case of the same alignment
          in declaration order (items declared close together are possibly
          also related and hence possibly used together -> putting them next
          to each other can improve cache behaviour) }
        result:=field2.vardef.alignment-field1.vardef.alignment;
        if result=0 then
          result:=field1.fieldoffset-field2.fieldoffset;
      end;


    procedure tabstractrecordsymtable.addfieldlist(list: tfpobjectlist; maybereorder: boolean);
      var
        fieldvs, insertfieldvs: tfieldvarsym;
        base, fieldoffset, space, insertfieldsize, insertfieldoffset, bestinsertfieldoffset, bestspaceleft: asizeint;
        i, j, bestfieldindex: longint;
        globalfieldalignment,
        prevglobalfieldalignment,
        newfieldalignment: shortint;
        changed: boolean;
      begin
        if maybereorder and
           (cs_opt_reorder_fields in current_settings.optimizerswitches) then
          begin
            { assign dummy field offsets so we can know their order in the
              sorting routine }
            for i:=0 to list.count-1 do
              tfieldvarsym(list[i]).fieldoffset:=i;
            { sort the non-class fields to minimise losses due to alignment }
            list.sort(@field_alignment_compare);
            { now fill up gaps caused by alignment skips with smaller fields
              where possible }
            repeat
              i:=0;
              base:=_datasize;
              globalfieldalignment:=fieldalignment;
              changed:=false;
              while i<list.count do
                begin
                  fieldvs:=tfieldvarsym(list[i]);
                  if sp_static in fieldvs.symoptions then
                    begin
                      inc(i);
                      continue;
                    end;
                  prevglobalfieldalignment:=globalfieldalignment;
                  fieldoffset:=getfieldoffset(fieldvs,base,globalfieldalignment);
                  newfieldalignment:=globalfieldalignment;

                  { size of the gap between the end of the previous field and
                    the start of the current one }
                  space:=fieldoffset-base;
                  bestspaceleft:=space;
                  while space>0 do
                    begin
                      bestfieldindex:=-1;
                      bestinsertfieldoffset:=-1;
                      for j:=i+1 to list.count-1 do
                        begin
                          insertfieldvs:=tfieldvarsym(list[j]);
                          if sp_static in insertfieldvs.symoptions then
                            continue;
                          insertfieldsize:=insertfieldvs.getsize;
                          { can the new field fit possibly in the gap? }
                          if insertfieldsize<=space then
                            begin
                             { restore globalfieldalignment to situation before
                               the original field was inserted }
                              globalfieldalignment:=prevglobalfieldalignment;
                              { at what offset would it be inserted? (this new
                                field has its own alignment requirements, which
                                may make it impossible to fit after all) }
                              insertfieldoffset:=getfieldoffset(insertfieldvs,base,globalfieldalignment);
                              globalfieldalignment:=prevglobalfieldalignment;
                              { taking into account the alignment, does it still
                                fit and if so, does it fit better than the
                                previously found best fit? }
                              if (insertfieldoffset+insertfieldsize<=fieldoffset) and
                                 (fieldoffset-insertfieldoffset-insertfieldsize<bestspaceleft) then
                                begin
                                  { new best fit }
                                  bestfieldindex:=j;
                                  bestinsertfieldoffset:=insertfieldoffset;
                                  bestspaceleft:=fieldoffset-insertfieldoffset-insertfieldsize;
                                  if bestspaceleft=0 then
                                    break;
                                end;
                            end;
                        end;
                      { if we didn't find any field to fit, stop trying for this
                        gap }
                      if bestfieldindex=-1 then
                        break;
                      changed:=true;
                      { we found a field to insert -> adjust the new base
                        address }
                      base:=bestinsertfieldoffset+tfieldvarsym(list[bestfieldindex]).getsize;
                      { update globalfieldalignment for this newly inserted
                        field }
                      getfieldoffset(tfieldvarsym(list[bestfieldindex]),base,globalfieldalignment);
                      { move the new field before the current one }
                      list.move(bestfieldindex,i);
                      { and skip the new field (which is now at position i) }
                      inc(i);
                      { there may be more space left -> continue }
                      space:=bestspaceleft;
                    end;
                  if base>fieldoffset then
                    internalerror(2012071302);
                  { check the next field }
                  base:=fieldoffset+fieldvs.getsize;
                  { since the original field had the same or greater alignment
                    than anything we inserted before it, the global field
                    alignment is still the same now as it was originally after
                    inserting that field }
                  globalfieldalignment:=newfieldalignment;
                  inc(i);
                end;
            { there may be small gaps left *before* inserted fields }
          until not changed;
        end;
        { reset the dummy field offsets }
        for i:=0 to list.count-1 do
          tfieldvarsym(list[i]).fieldoffset:=-1;
        { finally, set the actual field offsets }
        for i:=0 to list.count-1 do
          begin
            fieldvs:=tfieldvarsym(list[i]);
            { static data fields are already inserted in the globalsymtable }
            if not(sp_static in fieldvs.symoptions) then
              begin
                { read_record_fields already set the visibility of the fields,
                  because a single list can contain symbols with different
                  visibility }
                addfield(fieldvs,fieldvs.visibility);
              end;
          end;
      end;


    function tabstractrecordsymtable.findfieldbyoffset(offset: asizeint): tfieldvarsym;
      var
        i: longint;
        sym: tsym;
      begin
        { there could be multiple fields in case of a variant record }
        if (defowner.typ=recorddef) and
           trecorddef(defowner).isunion then
          internalerror(2014090403);
        for i:=0 to SymList.count-1 do
          begin
            sym:=tsym(symlist[i]);
            if is_normal_fieldvarsym(sym) and
               (tfieldvarsym(sym).fieldoffset>=offset) then
              begin
                result:=tfieldvarsym(sym);
                exit;
              end;
          end;
        result:=nil;
      end;


    procedure tabstractrecordsymtable.addalignmentpadding;
      var
        padded_datasize: asizeint;
      begin
        { make the record size aligned correctly so it can be
          used as elements in an array. For C records we
          use the fieldalignment, because that is updated with the
          used alignment. }
        if (padalignment = 1) then
          case usefieldalignment of
            C_alignment:
              padalignment:=fieldalignment;
            { bitpacked }
            bit_alignment:
              padalignment:=1;
            { mac68k: always round to multiple of 2 }
            mac68k_alignment:
              padalignment:=2;
            { default/no packrecords specified }
            0:
              padalignment:=recordalignment
            { specific packrecords setting -> use as upper limit }
            else
              padalignment:=min(recordalignment,usefieldalignment);
          end;
        padded_datasize:=align(_datasize,padalignment);
        _paddingsize:=padded_datasize-_datasize;
        _datasize:=padded_datasize;
      end;


    procedure tabstractrecordsymtable.insertdef(def:TDefEntry);
      begin
        { Enums must also be available outside the record scope,
          insert in the owner of this symtable }
        if def.typ=enumdef then
          defowner.owner.insertdef(def)
        else
          inherited insertdef(def);
      end;


    function tabstractrecordsymtable.is_packed: boolean;
      begin
        result:=usefieldalignment=bit_alignment;
      end;


    function tabstractrecordsymtable.has_single_field(out def:tdef): boolean;
      var
        i: longint;
        currentsymlist: TFPHashObjectList;
        currentdef: tdef;
        sym: tfieldvarsym;
      begin
        result:=false;
        { If a record contains a union, it does not contain a "single
          non-composite field" in the context of certain ABIs requiring
          special treatment for such records }
        if (defowner.typ=recorddef) and
           trecorddef(defowner).isunion then
          exit;
        { a record/object can contain other things than fields }
        currentsymlist:=symlist;
        { recurse in arrays and records }
        sym:=nil;
        repeat
          { record has one field? }
          for i:=0 to currentsymlist.Count-1 do
            begin
              if is_normal_fieldvarsym(tsym(currentsymlist[i])) then
                begin
                  if result then
                    begin
                      result:=false;
                      exit;
                    end;
                  result:=true;
                  sym:=tfieldvarsym(currentsymlist[i])
                end;
            end;
          if assigned(sym) then
            begin
              { if the field is an array, does it contain one element? }
              currentdef:=sym.vardef;
              while (currentdef.typ=arraydef) and
                    not is_special_array(currentdef) do
                begin
                  if tarraydef(currentdef).elecount<>1 then
                    begin
                      result:=false;
                      exit;
                    end;
                  currentdef:=tarraydef(currentdef).elementdef;
                end;
              { if the array element is again a record, continue descending }
              if currentdef.typ=recorddef then
                currentsymlist:=trecorddef(currentdef).symtable.SymList
              else
                begin
                  { otherwise we found the type of the single element }
                  def:=currentdef;
                  exit;
                end;
            end
          else
            exit
        until false;
      end;

    function tabstractrecordsymtable.get_unit_symtable: tsymtable;
      begin
        result:=defowner.owner;
        while assigned(result) and (result.symtabletype in [ObjectSymtable,recordsymtable]) do
          result:=result.defowner.owner;
      end;


    procedure tabstractrecordsymtable.do_get_managementoperator_offset_list(data:tobject;arg:pointer);
      var
        sym : tsym absolute data;
        fsym : tfieldvarsym absolute data;
        mop : tmanagementoperator;
        entry : pmanagementoperator_offset_entry;
        sublist : tfplist;
        i : longint;
      begin
        if not is_normal_fieldvarsym(sym) then
          exit;
        if not is_record(fsym.vardef) and not is_object(fsym.vardef) and not is_cppclass(fsym.vardef) then
          exit;
        mop:=tmanagementoperator(ptruint(arg));
        if not assigned(mop_list[mop]) then
          internalerror(2018082303);

        if is_record(fsym.vardef) then
          begin
            if mop in trecordsymtable(trecorddef(fsym.vardef).symtable).managementoperators then
              begin
                new(entry);
                entry^.pd:=search_management_operator(mop,fsym.vardef);
                if not assigned(entry^.pd) then
                  internalerror(2018082302);
                entry^.offset:=fsym.fieldoffset;
                mop_list[mop].add(entry);
              end;
          end;

        sublist:=tfplist.create;
        tabstractrecordsymtable(tabstractrecorddef(fsym.vardef).symtable).get_managementoperator_offset_list(mop,sublist);
        for i:=0 to sublist.count-1 do
          begin
            entry:=pmanagementoperator_offset_entry(sublist[i]);
            entry^.offset:=entry^.offset+fsym.fieldoffset;
            mop_list[mop].add(entry);
          end;
        { we don't need to remove the entries as they become part of list }
        sublist.free;
      end;

    procedure tabstractrecordsymtable.get_managementoperator_offset_list(mop:tmanagementoperator;list:tfplist);
      var
        i : longint;
        entry,entrycopy : pmanagementoperator_offset_entry;
      begin
        if not assigned(list) then
          internalerror(2018082301);
        if mop=mop_none then
          exit;
        if not (mop in has_fields_with_mop) then
          { none of the fields or one of the field's fields has the requested operator }
          exit;
        if not assigned(mop_list[mop]) then
          begin
            mop_list[mop]:=tfplist.create;
            SymList.ForEachCall(@do_get_managementoperator_offset_list,pointer(ptruint(mop)));
          end;
        for i:=0 to mop_list[mop].count-1 do
          begin
            entry:=pmanagementoperator_offset_entry(mop_list[mop][i]);
            New(entrycopy);
            entrycopy^:=entry^;
            list.add(entrycopy);
          end;
      end;

    procedure tabstractrecordsymtable.setdatasize(val: asizeint);
      begin
        _datasize:=val;
        if (usefieldalignment=bit_alignment) then
          { can overflow in non bitpacked records }
          databitsize:=val*8;
      end;

    function tabstractrecordsymtable.getfieldoffset(sym: tfieldvarsym; base: asizeint; var globalfieldalignment: shortint): asizeint;
      var
        l      : asizeint;
        varalignfield,
        varalign : shortint;
        vardef : tdef;
      begin
        { Calculate field offset }
        l:=sym.getsize;
        vardef:=sym.vardef;
        varalign:=vardef.structalignment;
        case usefieldalignment of
          bit_alignment:
            { has to be handled separately }
            internalerror(2012071301);
          C_alignment:
            begin
              { Calc the alignment size for C style records }
              if (varalign>4) and
                ((varalign mod 4)<>0) and
                (vardef.typ=arraydef) then
                Message1(sym_w_wrong_C_pack,vardef.typename);
              if varalign=0 then
                varalign:=l;
              if (globalfieldalignment<current_settings.alignment.maxCrecordalign) then
                begin
                  if (varalign>16) and (globalfieldalignment<32) then
                    globalfieldalignment:=32
                  else if (varalign>12) and (globalfieldalignment<16) then
                    globalfieldalignment:=16
                  { 12 is needed for long double }
                  else if (varalign>8) and (globalfieldalignment<12) then
                    globalfieldalignment:=12
                  else if (varalign>4) and (globalfieldalignment<8) then
                    globalfieldalignment:=8
                  else if (varalign>2) and (globalfieldalignment<4) then
                    globalfieldalignment:=4
                  else if (varalign>1) and (globalfieldalignment<2) then
                    globalfieldalignment:=2;
                end;
              globalfieldalignment:=min(globalfieldalignment,current_settings.alignment.maxCrecordalign);
            end;
          mac68k_alignment:
            begin
              { mac68k alignment (C description):
                 * char is aligned to 1 byte
                 * everything else (except vector) is aligned to 2 bytes
                 * vector is aligned to 16 bytes
              }
              if l>1 then
                globalfieldalignment:=2
              else
                globalfieldalignment:=1;
              varalign:=2;
            end;
        end;
        if varalign=0 then
          varalign:=size_2_align(l);
        varalignfield:=used_align(varalign,recordalignmin,globalfieldalignment);

        result:=align(base,varalignfield);
      end;

    function tabstractrecordsymtable.iscurrentunit: boolean;
      begin
        Result:=assigned(current_module)and(current_module.moduleid=moduleid);
      end;

{****************************************************************************
                              TRecordSymtable
****************************************************************************}

    constructor trecordsymtable.create(const n:string;usealign,recordminalign:shortint);
      begin
        inherited create(n,usealign,recordminalign);
        symtabletype:=recordsymtable;
      end;


   { this procedure is reserved for inserting case variant into
      a record symtable }
    { the offset is the location of the start of the variant
      and datasize and dataalignment corresponds to
      the complete size (see code in pdecl unit) PM }
    procedure trecordsymtable.insertunionst(unionst : trecordsymtable;offset : asizeint);
      var
        sym : tsym;
        def : tdef;
        i : integer;
        varalignrecord,varalign,
        storesize,storealign : asizeint;
        bitsize: tcgint;
      begin
        storesize:=_datasize;
        storealign:=fieldalignment;
        _datasize:=offset;
        if (usefieldalignment=bit_alignment) then
          databitsize:=offset*8;

        { We move the ownership of the defs and symbols to the new recordsymtable.
          The old unionsymtable keeps the references, but doesn't own the
          objects anymore }
        unionst.DefList.OwnsObjects:=false;
        unionst.SymList.OwnsObjects:=false;

        { copy symbols }
        for i:=0 to unionst.SymList.Count-1 do
          begin
            sym:=TSym(unionst.SymList[i]);
            if not is_normal_fieldvarsym(sym) then
              internalerror(200601272);
            if tfieldvarsym(sym).fieldoffset=0 then
              include(tfieldvarsym(sym).varoptions,vo_is_first_field);

            { add to this record symtable, checking for duplicate names }
//            unionst.SymList.List.List^[i].Data:=nil;
            insert(sym);
            varalign:=tfieldvarsym(sym).vardef.alignment;
            if varalign=0 then
              varalign:=size_2_align(tfieldvarsym(sym).getsize);
            { retrieve size }
            if (usefieldalignment=bit_alignment) then
              begin
                { bit packed records are limited to high(aint) bits }
                { instead of bytes to avoid double precision        }
                { arithmetic in offset calculations                 }
                if is_ordinal(tfieldvarsym(sym).vardef) then
                  bitsize:=tfieldvarsym(sym).getpackedbitsize
                else
                  begin
                    bitsize:=tfieldvarsym(sym).getsize;
                    if (bitsize>high(asizeint) div 8) then
                      Message(sym_e_segment_too_large);
                    bitsize:=bitsize*8;
                  end;
                if bitsize>high(asizeint)-databitsize then
                  begin
                    Message(sym_e_segment_too_large);
                    _datasize:=high(asizeint);
                    databitsize:=high(asizeint);
                  end
                else
                  begin
                    databitsize:=tfieldvarsym(sym).fieldoffset+offset*8;
                    _datasize:=(databitsize+7) div 8;
                  end;
                tfieldvarsym(sym).fieldoffset:=databitsize;
              varalignrecord:=field2recordalignment(tfieldvarsym(sym).fieldoffset div 8,varalign);
              end
            else
              begin
                if tfieldvarsym(sym).getsize>high(asizeint)-_datasize then
                  begin
                    Message(sym_e_segment_too_large);
                    _datasize:=high(asizeint);
                  end
                else
                  _datasize:=tfieldvarsym(sym).fieldoffset+offset;
                { update address }
                tfieldvarsym(sym).fieldoffset:=_datasize;
                varalignrecord:=field2recordalignment(tfieldvarsym(sym).fieldoffset,varalign);
              end;
            { update alignment of this record }
            if (usefieldalignment<>C_alignment) and
               (usefieldalignment<>mac68k_alignment) then
              recordalignment:=max(recordalignment,varalignrecord);
          end;
        { update alignment for C records }
        if (usefieldalignment=C_alignment) and
           (usefieldalignment<>mac68k_alignment) then
          recordalignment:=max(recordalignment,unionst.recordalignment);
        { Register defs in the new record symtable }
        for i:=0 to unionst.DefList.Count-1 do
          begin
            def:=TDef(unionst.DefList[i]);
            def.ChangeOwner(self);
          end;
        _datasize:=storesize;
        fieldalignment:=storealign;
        { If a record contains a union, it does not contain a "single
          non-composite field" in the context of certain ABIs requiring
          special treatment for such records }
        if defowner.typ=recorddef then
          trecorddef(defowner).isunion:=true;
      end;


    procedure trecordsymtable.includemanagementoperator(mop:tmanagementoperator);
      begin
        if mop in managementoperators then
          exit;
        include(managementoperators,mop);
      end;


{****************************************************************************
                              TObjectSymtable
****************************************************************************}

    constructor tObjectSymtable.create(adefowner:tdef;const n:string;usealign,recordminalign:shortint);
      begin
        inherited create(n,usealign,recordminalign);
        symtabletype:=ObjectSymtable;
        defowner:=adefowner;
      end;


    function tObjectSymtable.checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;
      var
         hsym: tsym;
         warn: boolean;
      begin
         result:=false;
         if not assigned(defowner) then
           internalerror(200602061);

         { procsym and propertysym have special code
           to override values in inherited classes. For other
           symbols check for duplicates }
         if not(sym.typ in [procsym,propertysym]) then
           begin
              { but private ids can be reused }
              hsym:=search_struct_member(tobjectdef(defowner),hashedid.id);
              if assigned(hsym) and
                 (
                  (
                   not(m_delphi in current_settings.modeswitches) and
                   is_visible_for_object(hsym,tobjectdef(defowner))
                  ) or
                  (
                   { In Delphi, you can repeat members of a parent class. You can't }
                   { do this for objects however, and you (obviouly) can't          }
                   { declare two fields with the same name in a single class        }
                   (m_delphi in current_settings.modeswitches) and
                   (
                    is_object(tdef(defowner)) or
                    (hsym.owner = self)
                   )
                  )
                 ) then
                begin
                  { only warn when a parameter/local variable in a method
                    conflicts with a category method, because this can easily
                    happen due to all possible categories being imported via
                    CocoaAll }
                  warn:=
                    (is_objccategory(tdef(hsym.owner.defowner)) or
                     is_classhelper(tdef(hsym.owner.defowner))) and
                    (sym.typ in [paravarsym,localvarsym,fieldvarsym]);
                  DuplicateSym(hashedid,sym,hsym,warn);
                  result:=true;
                end;
           end
         else
           result:=inherited checkduplicate(hashedid,sym);
      end;


{$ifdef llvm}

{****************************************************************************
                              tLlvmShadowSymtableEntry
****************************************************************************}

    constructor tllvmshadowsymtableentry.create(def: tdef; fieldoffset: aint);
      begin
        fdef:=def;
        ffieldoffset:=fieldoffset;
      end;


{****************************************************************************
                              TLlvmShadowSymtable
****************************************************************************}

   function tllvmshadowsymtable.get(f: tfieldvarsym): tllvmshadowsymtableentry;
      begin
        result:=get_by_llvm_index(f.llvmfieldnr)
      end;


   function tllvmshadowsymtable.get_by_llvm_index(index: longint): tllvmshadowsymtableentry;
     begin
       result:=tllvmshadowsymtableentry(symdeflist[index]);
     end;


    constructor tllvmshadowsymtable.create(st: tabstractrecordsymtable);
      begin
        equivst:=st;
        curroffset:=0;
        symdeflist:=tfpobjectlist.create(true);
        generate;
      end;


    destructor tllvmshadowsymtable.destroy;
      begin
        symdeflist.free;
      end;


    procedure tllvmshadowsymtable.appenddefoffset(vardef:tdef; fieldoffset: aint; derefclass: boolean);
      var
        sizectr,
        tmpsize: aint;
      begin
        case equivst.usefieldalignment of
          bit_alignment:
            begin
              { curoffset: bit address after the previous field.      }
              { llvm has no special support for bitfields in records, }
              { so we replace them with plain bytes.                  }
              { as soon as a single bit of a byte is allocated, we    }
              { allocate the byte in the llvm shadow record           }
              if (fieldoffset>curroffset) then
                curroffset:=align(curroffset,8);
              { fields in bitpacked records always start either right }
              { after the previous one, or at the next byte boundary. }
              if (curroffset<>fieldoffset) then
                internalerror(2008051002);
              if is_ordinal(vardef) then
                begin
                  tmpsize:=vardef.packedbitsize;
                  sizectr:=((curroffset+tmpsize+7) shr 3)-((curroffset+7) shr 3);
                  inc(curroffset,tmpsize);
                  tmpsize:=0;
                  while sizectr<>0 do
                    begin
                      symdeflist.add(tllvmshadowsymtableentry.create(u8inttype,fieldoffset+tmpsize*8));
                      dec(sizectr);
                      inc(tmpsize);
                    end;
                end
              else
                begin
                  symdeflist.add(tllvmshadowsymtableentry.create(vardef,fieldoffset));
                  if not(derefclass) then
                    inc(curroffset,vardef.size*8)
                  else
                    inc(curroffset,tobjectsymtable(tobjectdef(vardef).symtable).datasize*8);
               end;
            end
          else if not(df_llvm_no_struct_packing in tdef(equivst.defowner).defoptions) then
            begin
              { curoffset: address right after the previous field }
              while (fieldoffset>curroffset) do
                begin
                  symdeflist.add(tllvmshadowsymtableentry.create(u8inttype,curroffset));
                  inc(curroffset);
                end;
              symdeflist.add(tllvmshadowsymtableentry.create(vardef,fieldoffset));
              if not(derefclass) then
                inc(curroffset,vardef.size)
              else
                inc(curroffset,tobjectsymtable(tobjectdef(vardef).symtable).datasize);
            end
          else
            { default for llvm, don't add explicit padding }
            symdeflist.add(tllvmshadowsymtableentry.create(vardef,fieldoffset));
        end
      end;


    procedure tllvmshadowsymtable.addalignmentpadding(finalsize: aint);
      begin
        case equivst.usefieldalignment of
          { already correct in this case }
          bit_alignment:
            ;
          else if not(df_llvm_no_struct_packing in tdef(equivst.defowner).defoptions) then
            begin
              { add padding fields }
              while (finalsize>curroffset) do
                begin
                  symdeflist.add(tllvmshadowsymtableentry.create(u8inttype,curroffset));
                  inc(curroffset);
                end;
            end;
        end;
      end;


    procedure tllvmshadowsymtable.findvariantstarts(variantstarts: tfplist);
      var
        sym: tfieldvarsym;
        lastoffset: aint;
        newalignment: aint;
        i, j: longint;
      begin
        i:=0;
        while (i<equivst.symlist.count) do
          begin
            if not is_normal_fieldvarsym(tsym(equivst.symlist[i])) then
              begin
                inc(i);
                continue;
              end;
            sym:=tfieldvarsym(equivst.symlist[i]);
            { a "better" algorithm might be to use the largest }
            { variant in case of (bit)packing, since then      }
            { alignment doesn't matter                         }
            if (vo_is_first_field in sym.varoptions) then
              begin
                { we assume that all fields are processed in order. }
                if (variantstarts.count<>0) then
                  lastoffset:=tfieldvarsym(variantstarts[variantstarts.count-1]).fieldoffset
                else
                  lastoffset:=-1;

                { new variant at same level as last one: use if higher alignment }
                if (lastoffset=sym.fieldoffset) then
                  begin
                    if (equivst.fieldalignment<>bit_alignment) then
                      newalignment:=used_align(sym.vardef.alignment,equivst.recordalignmin,equivst.fieldalignment)
                    else
                      newalignment:=1;
                    if (newalignment>tfieldvarsym(variantstarts[variantstarts.count-1]).vardef.alignment) then
                      variantstarts[variantstarts.count-1]:=sym;
                  end
                { variant at deeper level than last one -> add }
                else if (lastoffset<sym.fieldoffset) then
                  variantstarts.add(sym)
                else
                  begin
                    { a variant at a less deep level, so backtrack }
                    j:=variantstarts.count-2;
                    while (j>=0) do
                      begin
                        if (tfieldvarsym(variantstarts[j]).fieldoffset=sym.fieldoffset) then
                          break;
                        dec(j);
                      end;
                    if (j<0) then
                      internalerror(2008051003);
                    { new variant has higher alignment? }
                    if (equivst.fieldalignment<>bit_alignment) then
                      newalignment:=used_align(sym.vardef.alignment,equivst.recordalignmin,equivst.fieldalignment)
                    else
                      newalignment:=1;
                    { yes, replace and remove previous nested variants }
                    if (newalignment>tfieldvarsym(variantstarts[j]).vardef.alignment) then
                      begin
                        variantstarts[j]:=sym;
                        variantstarts.count:=j+1;
                      end
                   { no, skip this variant }
                    else
                      begin
                        inc(i);
                        while (i<equivst.symlist.count) and
                              (not is_normal_fieldvarsym(tsym(equivst.symlist[i])) or
                               (tfieldvarsym(equivst.symlist[i]).fieldoffset>sym.fieldoffset)) do
                          inc(i);
                        continue;
                      end;
                  end;
              end;
            inc(i);
          end;
      end;


    procedure tllvmshadowsymtable.buildtable(variantstarts: tfplist);
      var
        lastvaroffsetprocessed: aint;
        i, equivcount, varcount: longint;
      begin
        { if it's an object/class, the first entry is the parent (if there is one) }
        if (equivst.symtabletype=objectsymtable) and
           assigned(tobjectdef(equivst.defowner).childof) then
          appenddefoffset(tobjectdef(equivst.defowner).childof,0,is_class_or_interface_or_dispinterface(tobjectdef(equivst.defowner).childof));
        equivcount:=equivst.symlist.count;
        varcount:=0;
        i:=0;
        lastvaroffsetprocessed:=-1;
        while (i<equivcount) do
          begin
            if not is_normal_fieldvarsym(tsym(equivst.symlist[i])) then
              begin
                inc(i);
                continue;
              end;
            { start of a new variant? }
            if (vo_is_first_field in tfieldvarsym(equivst.symlist[i]).varoptions) then
              begin
                { if we want to process the same variant offset twice, it means that we  }
                { got to the end and are trying to process the next variant part -> stop }
                if (tfieldvarsym(equivst.symlist[i]).fieldoffset<=lastvaroffsetprocessed) then
                  break;

                if (varcount>=variantstarts.count) then
                  internalerror(2008051005);
                { new variant part -> use the one with the biggest alignment }
                i:=equivst.symlist.indexof(tobject(variantstarts[varcount]));
                lastvaroffsetprocessed:=tfieldvarsym(equivst.symlist[i]).fieldoffset;
                inc(varcount);
                if (i<0) then
                  internalerror(2008051004);
              end;
            appenddefoffset(tfieldvarsym(equivst.symlist[i]).vardef,tfieldvarsym(equivst.symlist[i]).fieldoffset,false);
            inc(i);
          end;
        addalignmentpadding(equivst.datasize);
      end;


    procedure tllvmshadowsymtable.buildmapping(variantstarts: tfplist);
      var
        i, varcount: longint;
        shadowindex: longint;
        equivcount : longint;
      begin
        varcount:=0;
        shadowindex:=0;
        equivcount:=equivst.symlist.count;
        i:=0;
        while (i < equivcount) do
          begin
            if not is_normal_fieldvarsym(tsym(equivst.symlist[i])) then
              begin
                inc(i);
                continue;
              end;
            { start of a new variant? }
            if (vo_is_first_field in tfieldvarsym(equivst.symlist[i]).varoptions) then
              begin
                { back up to a less deeply nested variant level? }
                while (tfieldvarsym(equivst.symlist[i]).fieldoffset<tfieldvarsym(variantstarts[varcount]).fieldoffset) do
                  dec(varcount);
                { it's possible that some variants are more deeply nested than the
                  one we recorded in the shadowsymtable (since we recorded the one
                  with the biggest alignment, not necessarily the biggest one in size
                }
                if (tfieldvarsym(equivst.symlist[i]).fieldoffset>tfieldvarsym(variantstarts[varcount]).fieldoffset) then
                  varcount:=variantstarts.count-1
                else if (tfieldvarsym(equivst.symlist[i]).fieldoffset<>tfieldvarsym(variantstarts[varcount]).fieldoffset) then
                  internalerror(2008051006);
                { reset the shadowindex to the start of this variant. }
                { in case the llvmfieldnr is not (yet) set for this   }
                { field, shadowindex will simply be reset to zero and }
                { we'll start searching from the start of the record  }
                shadowindex:=tfieldvarsym(variantstarts[varcount]).llvmfieldnr;
                if (varcount<pred(variantstarts.count)) then
                  inc(varcount);
              end;

            { find the last shadowfield whose offset <= the current field's offset }
            while (tllvmshadowsymtableentry(symdeflist[shadowindex]).fieldoffset<tfieldvarsym(equivst.symlist[i]).fieldoffset) and
                  (shadowindex<symdeflist.count-1) and
                  (tllvmshadowsymtableentry(symdeflist[shadowindex+1]).fieldoffset<=tfieldvarsym(equivst.symlist[i]).fieldoffset) do
              inc(shadowindex);
            { set the field number and potential offset from that field (in case }
            { of overlapping variants)                                           }
            tfieldvarsym(equivst.symlist[i]).llvmfieldnr:=shadowindex;
            tfieldvarsym(equivst.symlist[i]).offsetfromllvmfield:=
              tfieldvarsym(equivst.symlist[i]).fieldoffset-tllvmshadowsymtableentry(symdeflist[shadowindex]).fieldoffset;
            inc(i);
          end;
      end;


    procedure tllvmshadowsymtable.generate;
      var
        variantstarts: tfplist;
      begin
        variantstarts:=tfplist.create;

        { first go through the entire record and }
        { store the fieldvarsyms of the variants }
        { with the highest alignment             }
        findvariantstarts(variantstarts);

        { now go through the regular fields and the selected variants, }
        { and add them to the  llvm shadow record symtable             }
        buildtable(variantstarts);

        { finally map all original fields to the llvm definition }
        buildmapping(variantstarts);

        variantstarts.free;
      end;

{$endif llvm}

{****************************************************************************
                          TAbstractSubSymtable
****************************************************************************}

   procedure tabstractsubsymtable.ppuwrite(ppufile:tcompilerppufile);
      var
        oldtyp : byte;
      begin
         oldtyp:=ppufile.entrytyp;
         ppufile.entrytyp:=subentryid;

         inherited ppuwrite(ppufile);

         ppufile.entrytyp:=oldtyp;
      end;


{****************************************************************************
                          TAbstractLocalSymtable
****************************************************************************}

    function tabstractlocalsymtable.count_locals:longint;
      var
        i   : longint;
        sym : tsym;
      begin
        result:=0;
        for i:=0 to SymList.Count-1 do
          begin
            sym:=tsym(SymList[i]);
            { Count only varsyms, but ignore the funcretsym }
            if (tsym(sym).typ in [localvarsym,paravarsym]) and
               (tsym(sym)<>current_procinfo.procdef.funcretsym) and
               (not(vo_is_parentfp in tabstractvarsym(sym).varoptions) or
                (tstoredsym(sym).refs>0)) then
              inc(result);
         end;
      end;

    function tabstractlocalsymtable.iscurrentunit: boolean;
      begin
        Result:=
          assigned(defowner) and
          defowner.owner.iscurrentunit;
      end;


{****************************************************************************
                              TLocalSymtable
****************************************************************************}

    constructor tlocalsymtable.create(adefowner:tdef;level:byte);
      begin
        inherited create('');
        defowner:=adefowner;
        symtabletype:=localsymtable;
        symtablelevel:=level;
      end;


    function tlocalsymtable.checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;
      var
        hsym : tsym;
      begin
        if not assigned(defowner) or
           (defowner.typ<>procdef) then
          internalerror(200602042);

        result:=false;
        hsym:=tsym(FindWithHash(hashedid));
        if assigned(hsym) then
          begin
            { a local and the function can have the same
              name in TP and Delphi, but RESULT not }
            if (m_duplicate_names in current_settings.modeswitches) and
               (hsym.typ in [absolutevarsym,localvarsym]) and
               (vo_is_funcret in tabstractvarsym(hsym).varoptions) and
               not((m_result in current_settings.modeswitches) and
                   (vo_is_result in tabstractvarsym(hsym).varoptions)) then
              HideSym(hsym)
            else
              DuplicateSym(hashedid,sym,hsym,false);
            result:=true;
            exit;
          end;

        { check also parasymtable, this needs to be done here because
          of the special situation with the funcret sym that needs to be
          hidden for tp and delphi modes }
        hsym:=tsym(tabstractprocdef(defowner).parast.FindWithHash(hashedid));
        if assigned(hsym) then
          begin
            { a local and the function can have the same
              name in TP and Delphi, but RESULT not }
            if (m_duplicate_names in current_settings.modeswitches) and
               (sym.typ in [absolutevarsym,localvarsym]) and
               (vo_is_funcret in tabstractvarsym(sym).varoptions) and
               not((m_result in current_settings.modeswitches) and
                   (vo_is_result in tabstractvarsym(sym).varoptions)) then
              Hidesym(sym)
            else
              DuplicateSym(hashedid,sym,hsym,false);
            result:=true;
            exit;
          end;

        { check ObjectSymtable, skip this for funcret sym because
          that will always be positive because it has the same name
          as the procsym }
        if not is_funcret_sym(sym) and
           (defowner.typ=procdef) and
           assigned(tprocdef(defowner).struct) and
           (tprocdef(defowner).owner.defowner=tprocdef(defowner).struct) and
           (
            not(m_delphi in current_settings.modeswitches) or
            is_object(tprocdef(defowner).struct)
           ) then
          result:=tprocdef(defowner).struct.symtable.checkduplicate(hashedid,sym);
      end;


{****************************************************************************
                              TParaSymtable
****************************************************************************}

    constructor tparasymtable.create(adefowner:tdef;level:byte);
      begin
        inherited create('');
        readonly:=false;
        defowner:=adefowner;
        symtabletype:=parasymtable;
        symtablelevel:=level;
      end;


    function tparasymtable.checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;
      begin
        result:=inherited checkduplicate(hashedid,sym);
        if result then
          exit;
        if not(m_duplicate_names in current_settings.modeswitches) and
           assigned(defowner) and (defowner.typ=procdef) and
           assigned(tprocdef(defowner).struct) and
           assigned(tprocdef(defowner).owner) and
           (tprocdef(defowner).owner.defowner=tprocdef(defowner).struct) and
           (
            not(m_delphi in current_settings.modeswitches) or
            is_object(tprocdef(defowner).struct)
           ) then
          result:=tprocdef(defowner).struct.symtable.checkduplicate(hashedid,sym);
      end;

    procedure tparasymtable.insertdef(def: TDefEntry);
      begin
        if readonly then
          defowner.owner.insertdef(def)
        else
          inherited insertdef(def);
      end;


{****************************************************************************
                         TAbstractUniTSymtable
****************************************************************************}

    constructor tabstractuniTSymtable.create(const n : string;id:word);
      begin
        inherited create(n);
        moduleid:=id;
      end;


    function tabstractuniTSymtable.checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;
      var
        hsym : tsym;
      begin
        result:=false;
        hsym:=tsym(FindWithHash(hashedid));
        if assigned(hsym) then
          begin
            if (sym is tstoredsym) and handle_generic_dummysym(hsym,tstoredsym(sym).symoptions) then
              exit;
            if hsym.typ=symconst.namespacesym then
              begin
                case sym.typ of
                  symconst.namespacesym:;
                  symconst.unitsym:
                    begin
                      HideSym(sym); { if we add a unit and there is a namespace with the same name then hide the unit name and not the namespace }
                      tnamespacesym(hsym).unitsym:=tsym(sym);
                    end
                else
                  HideSym(hsym);
                end;
              end
            else
            { In delphi (contrary to TP) you can have a symbol with the same name as the
              unit, the unit can then not be accessed anymore using
              <unit>.<id>, so we can hide the symbol.
              Do the same if we add a namespace and there is a unit with the same name }
            if (hsym.typ=symconst.unitsym) and
               ((m_delphi in current_settings.modeswitches) or (sym.typ=symconst.namespacesym)) then
              begin
                HideSym(hsym);
                if sym.typ=symconst.namespacesym then
                  tnamespacesym(sym).unitsym:=tsym(hsym);
              end
            { iso mode program parameters: staticvarsyms might have the same name as a program parameters,
              in this case, copy the isoindex and make the original symbol invisible }
            else if (m_isolike_program_para in current_settings.modeswitches) and (hsym.typ=programparasym) and (sym.typ=staticvarsym)
              and (tprogramparasym(hsym).isoindex<>0) then
              begin
                HideSym(hsym);
                tstaticvarsym(sym).isoindex:=tprogramparasym(hsym).isoindex;
              end
            else
              DuplicateSym(hashedid,sym,hsym,false);
            result:=true;
            exit;
          end;
      end;

    function tabstractuniTSymtable.findnamespace(const n:string):TSymEntry;
      begin
        result:=find(n);
        if assigned(result)and(result.typ<>namespacesym)then
          result:=nil;
      end;

    function tabstractuniTSymtable.iscurrentunit:boolean;
      begin
        result:=assigned(current_module) and
                (
                 (current_module.globalsymtable=self) or
                 (current_module.localsymtable=self)
                );
      end;


    function tabstractuniTSymtable.needs_init_final: boolean;
      begin
        if not init_final_check_done then
          begin
            result:=inherited needs_init_final;
            if not result then
              begin
                result:=has_class_condestructors;
                if result then
                  include(tableoptions,sto_needs_init_final);
              end;
          end;
        result:=sto_needs_init_final in tableoptions;
      end;


    procedure tabstractuniTSymtable.insertunit(sym:TSymEntry);
      var
        p:integer;
        n,ns:string;
        oldsym:TSymEntry;
      begin
        insert(sym);
        n:=sym.realname;
        p:=pos('.',n);
        ns:='';
        while p>0 do
          begin
            if ns='' then
              ns:=copy(n,1,p-1)
            else
              ns:=ns+'.'+copy(n,1,p-1);
            system.delete(n,1,p);
            oldsym:=findnamespace(upper(ns));
            if not assigned(oldsym) then
              insert(cnamespacesym.create(ns));
            p:=pos('.',n);
          end;
      end;


    procedure CheckForClassConDestructors(p:TObject;arg:pointer);
      var
        result: pboolean absolute arg;
      begin
        if result^ then
          exit;
        if (tdef(p).typ in [objectdef,recorddef]) and
           not (df_generic in tdef(p).defoptions) then
          begin
            { first check the class... }
            if ([oo_has_class_constructor,oo_has_class_destructor] * tabstractrecorddef(p).objectoptions <> []) then
              result^:=true;;
            { ... and then also check all subclasses }
            if not result^ then
              tabstractrecorddef(p).symtable.deflist.foreachcall(@CheckForClassConDestructors,arg);
          end;
      end;


    function tabstractuniTSymtable.has_class_condestructors: boolean;
      begin
        result:=false;
        deflist.foreachcall(@CheckForClassConDestructors,@result);
      end;

{****************************************************************************
                              TStaticSymtable
****************************************************************************}

    constructor tstaticsymtable.create(const n : string;id:word);
      begin
        inherited create(n,id);
        symtabletype:=staticsymtable;
        symtablelevel:=main_program_level;
        currentvisibility:=vis_private;
      end;


    procedure tstaticsymtable.ppuload(ppufile:tcompilerppufile);
      begin
        inherited ppuload(ppufile);

        { now we can deref the syms and defs }
        deref(false);
      end;


    procedure tstaticsymtable.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
      end;


    function tstaticsymtable.checkduplicate(var hashedid:THashedIDString;sym:TSymEntry):boolean;
      begin
        result:=inherited checkduplicate(hashedid,sym);

        if not result and
           (current_module.localsymtable=self) and
           assigned(current_module.globalsymtable) then
          result:=tglobalsymtable(current_module.globalsymtable).checkduplicate(hashedid,sym);
      end;

    function tstaticsymtable.findnamespace(const n:string):TSymEntry;
      begin
        result:=inherited findnamespace(n);
        if not assigned(result) and
           (current_module.localsymtable=self) and
           assigned(current_module.globalsymtable) then
          result:=tglobalsymtable(current_module.globalsymtable).findnamespace(n);
     end;


{****************************************************************************
                              TGlobalSymtable
****************************************************************************}

    constructor tglobalsymtable.create(const n : string;id:word);
      begin
         inherited create(n,id);
         symtabletype:=globalsymtable;
         symtablelevel:=main_program_level;
      end;


    procedure tglobalsymtable.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(ppufile);

         { now we can deref the syms and defs }
         deref(false);
      end;


    procedure tglobalsymtable.ppuwrite(ppufile:tcompilerppufile);
      begin
        { write the symtable entries }
        inherited ppuwrite(ppufile);
      end;


{*****************************************************************************
                             tspecializesymtable
*****************************************************************************}

    constructor tspecializesymtable.create(const n : string;id:word);
      begin
        inherited create(n,id);
        { the specialize symtable does not own the syms and defs as they are all
          moved to a different symtable before the symtable is destroyed; this
          avoids calls to "extract" }
        symlist.ownsobjects:=false;
        deflist.ownsobjects:=false;
      end;

    function tspecializesymtable.iscurrentunit: boolean;
      begin
        Result:=true;
      end;


{****************************************************************************
                              TWITHSYMTABLE
****************************************************************************}

    constructor twithsymtable.create(aowner:tdef;ASymList:TFPHashObjectList;refnode:tobject{tnode});
      begin
         inherited create('');
         symtabletype:=withsymtable;
         withrefnode:=refnode;
         { Replace SymList with the passed symlist }
         SymList.free;
         SymList:=ASymList;
         defowner:=aowner;
      end;


    destructor twithsymtable.destroy;
      begin
        if refcount>1 then
          exit;
        withrefnode.free;
        { Disable SymList because we don't Own it }
        SymList:=nil;
        inherited destroy;
      end;


    procedure twithsymtable.clear;
      begin
         { remove no entry from a withsymtable as it is only a pointer to the
           recorddef  or objectdef symtable }
      end;


    procedure twithsymtable.insertdef(def:TDefEntry);
      begin
        { Definitions can't be registered in the withsymtable
          because the withsymtable is removed after the with block.
          We can't easily solve it here because the next symtable in the
          stack is not known. }
        internalerror(200602046);
      end;

{****************************************************************************
                          TSTT_ExceptionSymtable
****************************************************************************}

    constructor tstt_excepTSymtable.create;
      begin
        inherited create('');
        symtabletype:=stt_excepTSymtable;
      end;


{****************************************************************************
                          TMacroSymtable
****************************************************************************}

    constructor tmacrosymtable.create(exported: boolean);
      begin
        inherited create('');
        if exported then
          symtabletype:=exportedmacrosymtable
        else
          symtabletype:=localmacrosymtable;
        symtablelevel:=main_program_level;
      end;

{****************************************************************************
                          TEnumSymtable
****************************************************************************}

    procedure tenumsymtable.insert(sym: TSymEntry; checkdup: boolean);
      var
        value: longint;
        def: tenumdef;
      begin
        // defowner = nil only when we are loading from ppu
        if defowner<>nil then
          begin
            { First entry? Then we need to set the minval }
            value:=tenumsym(sym).value;
            def:=tenumdef(defowner);
            if SymList.count=0 then
              begin
                if value>0 then
                  def.has_jumps:=true;
                def.setmin(value);
                def.setmax(value);
              end
            else
              begin
                { check for jumps }
                if value>def.max+1 then
                  def.has_jumps:=true;
                { update low and high }
                if def.min>value then
                  def.setmin(value);
                if def.max<value then
                  def.setmax(value);
              end;
          end;
        inherited insert(sym, checkdup);
      end;

    constructor tenumsymtable.create(adefowner: tdef);
      begin
        inherited Create('');
        symtabletype:=enumsymtable;
        defowner:=adefowner;
      end;

{****************************************************************************
                          TArraySymtable
****************************************************************************}

    procedure tarraysymtable.insertdef(def: TDefEntry);
      begin
        { Enums must also be available outside the record scope,
          insert in the owner of this symtable }
        if def.typ=enumdef then
          defowner.owner.insertdef(def)
        else
          inherited insertdef(def);
      end;

    constructor tarraysymtable.create(adefowner: tdef);
      begin
        inherited Create('');
        symtabletype:=arraysymtable;
        defowner:=adefowner;
      end;

{*****************************************************************************
                             Helper Routines
*****************************************************************************}

    function FullTypeName(def,otherdef:tdef):string;
      var
        s1,s2 : string;
      begin
        if def.typ in [objectdef,recorddef] then
          s1:=tabstractrecorddef(def).RttiName
        else
          s1:=def.typename;
        { When the names are the same try to include the unit name }
        if assigned(otherdef) and
           (def.owner.symtabletype in [globalsymtable,staticsymtable]) then
          begin
            s2:=otherdef.typename;
            if upper(s1)=upper(s2) then
              s1:=def.owner.realname^+'.'+s1;
          end;
        FullTypeName:=s1;
      end;

    function generate_nested_name(symtable:tsymtable;delimiter:string):string;
      begin
        result:='';
        while assigned(symtable) and (symtable.symtabletype in [ObjectSymtable,recordsymtable]) do
          begin
            if (result='') then
              if symtable.name<>nil then
                result:=symtable.name^
              else
            else
              if symtable.name<>nil then
                result:=symtable.name^+delimiter+result
              else
                result:=delimiter+result;
            symtable:=symtable.defowner.owner;
          end;
      end;


    function generate_objectpascal_helper_key(def:tdef):string;
      begin
        if not assigned(def) then
          internalerror(2013020501);
        if def.typ in [recorddef,objectdef] then
          result:=make_mangledname('',tabstractrecorddef(def).symtable,'')
        else
          result:=make_mangledname('',def.owner,def.typesym.name);
      end;


    procedure incompatibletypes(def1,def2:tdef);
      begin
        { When there is an errordef there is already an error message show }
        if (def2.typ=errordef) or
           (def1.typ=errordef) then
          exit;
        CGMessage2(type_e_incompatible_types,FullTypeName(def1,def2),FullTypeName(def2,def1));
      end;


    procedure hidesym(sym:TSymEntry);
      begin
        sym.realname:='$hidden'+sym.realname;
        tsym(sym).visibility:=vis_hidden;
      end;


    procedure duplicatesym(var hashedid: THashedIDString; dupsym, origsym: TSymEntry; warn: boolean);
      var
        st : TSymtable;
        filename : TIDString;
      begin
        if not warn then
          Message1(sym_e_duplicate_id,tsym(origsym).realname)
        else
         Message1(sym_w_duplicate_id,tsym(origsym).realname);
        { Write hint where the original symbol was found }
        st:=finduniTSymtable(origsym.owner);
        with tsym(origsym).fileinfo do
          begin
            if assigned(st) and
               (st.symtabletype=globalsymtable) and
               st.iscurrentunit then
              Message2(sym_h_duplicate_id_where,current_module.sourcefiles.get_file_name(fileindex),tostr(line))
            else if assigned(st.name) then
              begin
                filename:=find_module_from_symtable(st).sourcefiles.get_file_name(fileindex);
                if filename<>'' then
                  Message2(sym_h_duplicate_id_where,'unit '+st.name^+': '+filename,tostr(line))
                else
                  Message2(sym_h_duplicate_id_where,'unit '+st.name^,tostr(line))
              end;
          end;
        { Rename duplicate sym to an unreachable name, but it can be
          inserted in the symtable without errors }
        inc(dupnr);
        hashedid.id:='dup'+tostr(dupnr)+hashedid.id;
        if assigned(dupsym) then
          include(tsym(dupsym).symoptions,sp_implicitrename);
      end;

    function handle_generic_dummysym(sym:TSymEntry;var symoptions:tsymoptions):boolean;
      begin
        result:=false;
        if not assigned(sym) or not (sym is tstoredsym) then
          Internalerror(2011081101);
        { For generics a dummy symbol without the parameter count is created
          if such a symbol not yet exists so that different parts of the
          parser can find that symbol. If that symbol is still a
          undefineddef we replace the generic dummy symbol's
          name with a "dup" name and use the new symbol as the generic dummy
          symbol }
        if (sp_generic_dummy in tstoredsym(sym).symoptions) and
            (sym.typ=typesym) and (ttypesym(sym).typedef.typ=undefineddef) and
            (m_delphi in current_settings.modeswitches) then
          begin
            inc(dupnr);
            sym.Owner.SymList.Rename(upper(sym.realname),'dup_'+tostr(dupnr)+sym.realname);
            include(tsym(sym).symoptions,sp_implicitrename);
            { we need to find the new symbol now if checking for a dummy }
            include(symoptions,sp_generic_dummy);
            result:=true;
          end;
      end;


    procedure write_system_parameter_lists(const name:string);
      var
        srsym:tprocsym;
      begin
        srsym:=tprocsym(systemunit.find(name));
        if not assigned(srsym) or not (srsym.typ=procsym) then
          internalerror(2016060302);
        srsym.write_parameter_lists(nil);
      end;


{*****************************************************************************
                                  Search
*****************************************************************************}

     procedure addsymref(sym:tsym;def:tdef);
       var
         owner,procowner : tsymtable;
       begin
         { for symbols used in preprocessor expressions, we don't want to
           increase references count (for smaller final binaries) }
         if not assigned(current_scanner) then
           internalerror(2017050601);
         if current_scanner.in_preproc_comp_expr then
           exit;
         { symbol uses count }
         sym.IncRefCount;
         owner:=sym.owner;
         while owner.symtabletype in [objectsymtable,recordsymtable,enumsymtable] do
           owner:=tdef(owner.defowner).owner;
         if assigned(current_module) and
            (owner.symtabletype=globalsymtable) then
             begin
               if tglobalsymtable(owner).moduleid>=current_module.unitmapsize then
                 internalerror(200501152);
               { unit uses count }
               inc(current_module.unitmap[tglobalsymtable(owner).moduleid].refs);
               { Note: don't check the symtable directly as owner might be
                       a specialize symtable which is a globalsymtable as well }
               if (
                     assigned(current_module.globalsymtable) and
                     (current_module.globalsymtable.moduleid<>owner.moduleid)
                  ) or (
                     assigned(current_module.localsymtable) and
                     (current_module.localsymtable.moduleid<>owner.moduleid)
                  ) then
                 { symbol is imported from another unit }
                 current_module.addimportedsym(sym);
             end;
         { static symbols that are used in public functions must be exported
           for packages as well }
         if ([tf_supports_packages,tf_supports_hidden_symbols]<=target_info.flags) and
             (owner.symtabletype=staticsymtable) and
             assigned(current_procinfo) and
             (
               (
                 (sym.typ=staticvarsym) and
                 ([vo_is_public,vo_has_global_ref]*tstaticvarsym(sym).varoptions=[])
               ) or (
                 (sym.typ=localvarsym) and
                 assigned(tlocalvarsym(sym).defaultconstsym) and
                 ([vo_is_public,vo_has_global_ref]*tstaticvarsym(tlocalvarsym(sym).defaultconstsym).varoptions=[])
               ) or (
                 (sym.typ=procsym) and
                 assigned(def) and
                 (def.typ=procdef) and
                 not (df_has_global_ref in def.defoptions) and
                 not (po_public in tprocdef(def).procoptions)
               )
             ) then
           begin
             procowner:=current_procinfo.procdef.owner;
             while procowner.symtabletype in [objectsymtable,recordsymtable] do
               procowner:=tdef(procowner.defowner).owner;
             if procowner.symtabletype=globalsymtable then
               begin
                 if sym.typ=procsym then
                   current_procinfo.add_local_ref_def(def)
                 else if sym.typ=staticvarsym then
                   current_procinfo.add_local_ref_sym(sym)
                 else
                   current_procinfo.add_local_ref_sym(tlocalvarsym(sym).defaultconstsym);
               end;
           end;
       end;

     procedure addsymref(sym:tsym);
       begin
         addsymref(sym,nil);
       end;

    function is_owned_by(nesteddef,ownerdef:tdef):boolean;
      begin
        result:=nesteddef=ownerdef;
        if not result and
           { types declared locally in a record method are not defined in the
             record itself }
           not(nesteddef.owner.symtabletype in [localsymtable,parasymtable]) and
           assigned(nesteddef.owner.defowner) then
          result:=is_owned_by(tdef(nesteddef.owner.defowner),ownerdef);
      end;

    function sym_is_owned_by(childsym:tsym;symtable:tsymtable):boolean;
      begin
        result:=assigned(childsym) and (childsym.owner=symtable);
        if not result and assigned(childsym) and
            (childsym.owner.symtabletype in [objectsymtable,recordsymtable]) then
          result:=sym_is_owned_by(tabstractrecorddef(childsym.owner.defowner).typesym,symtable);
      end;

    function defs_belong_to_same_generic(def1, def2: tdef): boolean;
    begin
      result:=false;
      if not assigned(def1) or not assigned(def2) then
        exit;
      { for both defs walk to the topmost generic }
      while assigned(def1.owner.defowner) and (df_generic in tstoreddef(def1.owner.defowner).defoptions) do
        def1:=tdef(def1.owner.defowner);
      while assigned(def2.owner.defowner) and (df_generic in tstoreddef(def2.owner.defowner).defoptions) do
        def2:=tdef(def2.owner.defowner);
      result:=def1=def2;
    end;

    function get_generic_in_hierarchy_by_name(srsym: tsym; def: tdef): tdef;
      var
        uname : string;
      begin
        { TODO : check regarding arrays and records declared as their type }
        if not (def.typ in [recorddef,objectdef]) then
          internalerror(2012051501);
        uname:=upper(srsym.realname);
        repeat
          if uname=copy(tabstractrecorddef(def).objname^,1,pos('$',tabstractrecorddef(def).objname^)-1) then
            begin
              result:=def;
              exit;
            end;
          def:=tdef(def.owner.defowner);
        until not assigned(def) or not (def.typ in [recorddef,objectdef]);
        result:=nil;
      end;

    function return_specialization_of_generic(nesteddef,genericdef:tdef; out resultdef:tdef):boolean;
      begin
        { TODO : check regarding arrays and records declared as their type }
        if not (nesteddef.typ in [recorddef,objectdef]) then
          internalerror(2012051601);
        repeat
          if tstoreddef(nesteddef).genericdef=genericdef then
            begin
              resultdef:=nesteddef;
              result:=true;
              exit;
            end;
          nesteddef:=tdef(nesteddef.owner.defowner);
        until not assigned(nesteddef) or not (nesteddef.typ in [recorddef,objectdef]);
        resultdef:=nil;
        result:=false;
      end;

    { symst: symboltable that contains the symbol (-> symowner def: record/objectdef in which the symbol is defined)
      symvisibility: visibility of the symbol
      contextobjdef: via which def the symbol is accessed, e.g.:
        fieldname:=1 -> contextobjdef = current_structdef
        objfield.fieldname:=1 -> contextobjdef = def of objfield
    }
    function is_visible_for_object(symst:tsymtable;symvisibility:tvisibility;contextobjdef:tabstractrecorddef):boolean;
      var
        symownerdef : tabstractrecorddef;
        nonlocalst : tsymtable;
        isspezproc : boolean;
      begin
        result:=false;

        { Get objdectdef owner of the symtable for the is_related checks }
        if not assigned(symst) or
           not (symst.symtabletype in [objectsymtable,recordsymtable]) then
          internalerror(200810285);
        symownerdef:=tabstractrecorddef(symst.defowner);
        { specializations might belong to a localsymtable or parasymtable }
        nonlocalst:=symownerdef.owner;
        if tstoreddef(symst.defowner).is_specialization then
          while nonlocalst.symtabletype in [localsymtable,parasymtable] do
            nonlocalst:=nonlocalst.defowner.owner;
        isspezproc:=false;
        if assigned(current_procinfo) then
          begin
            if current_procinfo.procdef.is_specialization and
                assigned(current_procinfo.procdef.struct) then
              isspezproc:=true;
          end;
        case symvisibility of
          vis_private :
            begin
              { private symbols are allowed when we are in the same
                module as they are defined }
              result:=(
                       (nonlocalst.symtabletype in [globalsymtable,staticsymtable]) and
                       (nonlocalst.iscurrentunit)
                      ) or
                      ( // the case of specialize inside the generic declaration and nested types
                       (nonlocalst.symtabletype in [objectsymtable,recordsymtable]) and
                       (
                         assigned(current_structdef) and
                         (
                           (current_structdef=symownerdef) or
                           (current_structdef.owner.iscurrentunit)
                         )
                       ) or
                       (
                         not assigned(current_structdef) and
                         (symownerdef.owner.iscurrentunit)
                       ) or
                       { access from a generic method that belongs to the class
                         but that is specialized elsewere }
                       (
                         isspezproc and
                         (current_procinfo.procdef.struct=current_structdef)
                       ) or
                       { specializations may access private symbols that their
                         generics are allowed to access }
                       (
                         assigned(current_structdef) and
                         (df_specialization in current_structdef.defoptions) and
                         (symst.moduleid=current_structdef.genericdef.owner.moduleid)
                       )
                      );
            end;
          vis_strictprivate :
            begin
              result:=assigned(current_structdef) and
                      is_owned_by(current_structdef,symownerdef);
            end;
          vis_strictprotected :
            begin
               result:=(
                         { access from nested class }
                         assigned(current_structdef) and
                         is_owned_by(current_structdef,symownerdef)
                       ) or
                       (
                         { access from child class }
                         assigned(contextobjdef) and
                         assigned(current_structdef) and
                         def_is_related(contextobjdef,symownerdef) and
                         def_is_related(current_structdef,contextobjdef)
                       ) or
                       (
                         { helpers can access strict protected symbols }
                         is_objectpascal_helper(contextobjdef) and
                         def_is_related(tobjectdef(contextobjdef).extendeddef,symownerdef)
                       ) or
                       (
                         { same as above, but from context of call node inside
                           helper method }
                         is_objectpascal_helper(current_structdef) and
                         def_is_related(tobjectdef(current_structdef).extendeddef,symownerdef)
                       );
            end;
          vis_protected :
            begin
              { protected symbols are visible in the module that defines them and
                also visible to related objects. The related object must be defined
                in the current module }
              result:=(
                       (
                        (nonlocalst.symtabletype in [globalsymtable,staticsymtable]) and
                        (nonlocalst.iscurrentunit)
                       ) or
                       (
                        assigned(contextobjdef) and
                        (contextobjdef.owner.symtabletype in [globalsymtable,staticsymtable,ObjectSymtable,recordsymtable]) and
                        (contextobjdef.owner.iscurrentunit) and
                        def_is_related(contextobjdef,symownerdef)
                       ) or
                       ( // the case of specialize inside the generic declaration and nested types
                        (nonlocalst.symtabletype in [objectsymtable,recordsymtable]) and
                        (
                          assigned(current_structdef) and
                          (
                            (current_structdef=symownerdef) or
                            (current_structdef.owner.iscurrentunit)
                          )
                        ) or
                        (
                          not assigned(current_structdef) and
                          (symownerdef.owner.iscurrentunit)
                        ) or
                        (
                          { helpers can access protected symbols }
                          is_objectpascal_helper(contextobjdef) and
                          def_is_related(tobjectdef(contextobjdef).extendeddef,symownerdef)
                        )
                       ) or
                       { access from a generic method that belongs to the class
                         but that is specialized elsewere }
                       (
                         isspezproc and
                         (current_procinfo.procdef.struct=current_structdef)
                       ) or
                       { specializations may access private symbols that their
                         generics are allowed to access }
                       (
                         assigned(current_structdef) and
                         (df_specialization in current_structdef.defoptions) and
                         (symst.moduleid=current_structdef.genericdef.owner.moduleid)
                       )
                      );
            end;
          vis_public,
          vis_published :
            result:=true;
          else
            internalerror(2019050702);
        end;
      end;


    function is_visible_for_object(pd:tprocdef;contextobjdef:tabstractrecorddef):boolean;
      begin
        result:=is_visible_for_object(pd.owner,pd.visibility,contextobjdef);
      end;


    function is_visible_for_object(sym:tsym;contextobjdef:tabstractrecorddef):boolean;
      var
        i  : longint;
        pd : tprocdef;
      begin
        if sym.typ=procsym then
          begin
            { A procsym is visible, when there is at least one of the procdefs visible }
            result:=false;
            for i:=0 to tprocsym(sym).ProcdefList.Count-1 do
              begin
                pd:=tprocdef(tprocsym(sym).ProcdefList[i]);
                if (pd.owner=sym.owner) and
                   is_visible_for_object(pd,contextobjdef) then
                  begin
                    result:=true;
                    exit;
                  end;
              end;
          end
        else
          result:=is_visible_for_object(sym.owner,sym.visibility,contextobjdef);
      end;


    function  searchsym(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
      begin
        result:=searchsym_maybe_with_symoption(s,srsym,srsymtable,[],sp_none);
      end;


    function  searchsym_with_flags(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags):boolean;
      begin
        result:=searchsym_maybe_with_symoption(s,srsym,srsymtable,flags,sp_none);
      end;


    function  searchsym_maybe_with_symoption(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags;option:tsymoption):boolean;
      var
        hashedid: THashedIDString;
        contextstructdef: tabstractrecorddef;
        stackitem: psymtablestackitem;
      begin
        result:=false;
        hashedid.id:=s;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            srsymtable:=stackitem^.symtable;
            if (srsymtable.symtabletype=objectsymtable) then
              begin
                { TODO : implement the search for an option in classes as well }
                if ssf_search_option in flags then
                  begin
                    result:=false;
                    exit;
                  end;
                if searchsym_in_class(tobjectdef(srsymtable.defowner),tobjectdef(srsymtable.defowner),s,srsym,srsymtable,flags+[ssf_search_helper]) then
                  begin
                    result:=true;
                    exit;
                  end;
              end
            else if not((srsymtable.symtabletype=withsymtable) and assigned(srsymtable.defowner) and
              (srsymtable.defowner.typ=undefineddef)) then
              begin
                srsym:=tsym(srsymtable.FindWithHash(hashedid));
                { First check if it is a unit/namespace symbol.
                  They are visible only if they are from the current unit or
                  unit of generic of currently processed specialization. }
                if assigned(srsym) and
                   (
                     not(srsym.typ in [unitsym,namespacesym]) or
                     srsymtable.iscurrentunit or
                     (assigned(current_specializedef)and(current_specializedef.genericdef.owner.moduleid=srsymtable.moduleid)) or
                     (
                       assigned(current_procinfo) and
                       (df_specialization in current_procinfo.procdef.defoptions) and
                       (current_procinfo.procdef.genericdef.owner.moduleid=srsymtable.moduleid)
                     )
                   ) and
                   (not (ssf_search_option in flags) or (option in srsym.symoptions))then
                  begin
                    { use the class from withsymtable only when it is
                      defined in this unit }
                    if (srsymtable.symtabletype=withsymtable) and
                       assigned(srsymtable.defowner) and
                       (srsymtable.defowner.typ in [recorddef,objectdef]) and
                       (srsymtable.defowner.owner.symtabletype in [globalsymtable,staticsymtable,objectsymtable,recordsymtable]) and
                       (srsymtable.defowner.owner.iscurrentunit) then
                      contextstructdef:=tabstractrecorddef(srsymtable.defowner)
                    else
                      contextstructdef:=current_structdef;
                    if not(srsym.owner.symtabletype in [objectsymtable,recordsymtable]) or
                       is_visible_for_object(srsym,contextstructdef) then
                      begin
                        { we need to know if a procedure references symbols
                          in the static symtable, because then it can't be
                          inlined from outside this unit }
                        if assigned(current_procinfo) and
                           (srsym.owner.symtabletype=staticsymtable) then
                          include(current_procinfo.flags,pi_uses_static_symtable);
                        if not (ssf_no_addsymref in flags) then
                          addsymref(srsym);
                        result:=true;
                        exit;
                      end;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        srsym:=nil;
        srsymtable:=nil;
      end;

    function searchsym_with_symoption(const s: TIDString;out srsym:tsym;out
      srsymtable:TSymtable;option:tsymoption):boolean;
      begin
        result:=searchsym_maybe_with_symoption(s,srsym,srsymtable,[ssf_search_option],option);
      end;

    function searchsym_type(const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
      var
        hashedid  : THashedIDString;
        stackitem : psymtablestackitem;
        classh : tobjectdef;
      begin
        result:=false;
        hashedid.id:=s;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            {
              It is not possible to have type symbols in:
                parameters
              Exception are classes, objects, records, generic definitions and specializations
              that have the parameterized types inserted in the symtable.
            }
            srsymtable:=stackitem^.symtable;
            if (srsymtable.symtabletype=ObjectSymtable) then
              begin
                classh:=tobjectdef(srsymtable.defowner);
                while assigned(classh) do
                  begin
                    srsymtable:=classh.symtable;
                    srsym:=tsym(srsymtable.FindWithHash(hashedid));
                     if assigned(srsym) and
                        not(srsym.typ in [fieldvarsym,paravarsym,propertysym,procsym,labelsym]) and
                        is_visible_for_object(srsym,current_structdef) then
                       begin
                        addsymref(srsym);
                        result:=true;
                        exit;
                      end;
                    classh:=classh.childof;
                  end;
              end
            else
              begin
                srsym:=tsym(srsymtable.FindWithHash(hashedid));
                if assigned(srsym) and
                   (
                     not(srsym.typ in [unitsym,namespacesym]) or
                     srsymtable.iscurrentunit or
                     (assigned(current_specializedef)and(current_specializedef.genericdef.owner.moduleid=srsymtable.moduleid))
                   ) and
                   not(srsym.typ in [fieldvarsym,paravarsym,propertysym,procsym,labelsym]) and
                   (not (srsym.owner.symtabletype in [objectsymtable,recordsymtable]) or is_visible_for_object(srsym,current_structdef)) then
                  begin
                    { we need to know if a procedure references symbols
                      in the static symtable, because then it can't be
                      inlined from outside this unit }
                    if assigned(current_procinfo) and
                       (srsym.owner.symtabletype=staticsymtable) then
                      include(current_procinfo.flags,pi_uses_static_symtable);
                    addsymref(srsym);
                    result:=true;
                    exit;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        result:=false;
        srsym:=nil;
        srsymtable:=nil;
      end;


    function searchsym_in_module(pm:pointer;const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
      var
        pmod : tmodule;
      begin
        pmod:=tmodule(pm);
        result:=false;
        if assigned(pmod.globalsymtable) then
          begin
            srsym:=tsym(pmod.globalsymtable.Find(s));
            if assigned(srsym) then
              begin
                srsymtable:=pmod.globalsymtable;
                addsymref(srsym);
                result:=true;
                exit;
              end;
          end;
        { If the module is the current unit we also need
          to search the local symtable }
        if (pmod=current_module) and
           assigned(pmod.localsymtable) then
          begin
            srsym:=tsym(pmod.localsymtable.Find(s));
            if assigned(srsym) then
              begin
                srsymtable:=pmod.localsymtable;
                addsymref(srsym);
                result:=true;
                exit;
              end;
          end;
        srsym:=nil;
        srsymtable:=nil;
      end;


    function searchsym_in_named_module(const unitname, symname: TIDString; out srsym: tsym; out srsymtable: tsymtable): boolean;
      var
        stackitem  : psymtablestackitem;
      begin
        result:=false;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            srsymtable:=stackitem^.symtable;
            if (srsymtable.symtabletype=globalsymtable) and
               (srsymtable.name^=unitname) then
              begin
                srsym:=tsym(srsymtable.find(symname));
                if not assigned(srsym) then
                  break;
                result:=true;
                exit;
              end;
            stackitem:=stackitem^.next;
          end;

        { If the module is the current unit we also need
          to search the local symtable }
        if assigned(current_module.localsymtable) and
           (current_module.localsymtable.name^=unitname) then
          begin
            srsymtable:=current_module.localsymtable;
            srsym:=tsym(srsymtable.find(symname));
            if assigned(srsym) then
              begin
                result:=true;
                exit;
              end;
          end;
      end;


    function maybe_find_real_class_definition(pd: tdef; erroronfailure: boolean): tdef;
      begin
        result:=pd;
        if pd.typ<>objectdef then
          exit;
        result:=find_real_class_definition(tobjectdef(pd),erroronfailure);
      end;


    function find_real_class_definition(pd: tobjectdef; erroronfailure: boolean): tobjectdef;
      var
        hashedid   : THashedIDString;
        stackitem  : psymtablestackitem;
        srsymtable : tsymtable;
        srsym      : tsym;
        formalname,
        foundname : shortstring;
        formalnameptr,
        foundnameptr: pshortstring;
      begin
        { not a formal definition -> return it }
        if not(oo_is_formal in pd.objectoptions) then
          begin
            result:=pd;
            exit;
          end;
        hashedid.id:=pd.typesym.name;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            srsymtable:=stackitem^.symtable;
            { ObjC classes can't appear in generics or as nested class
              definitions. Java classes can. }
            if not(srsymtable.symtabletype in [recordsymtable,parasymtable]) or
               (is_java_class_or_interface(pd) and
                (srsymtable.symtabletype=ObjectSymtable)) then
              begin
                srsym:=tsym(srsymtable.FindWithHash(hashedid));
                if assigned(srsym) and
                   (srsym.typ=typesym) and
                   (ttypesym(srsym).typedef.typ=objectdef) and
                   (tobjectdef(ttypesym(srsym).typedef).objecttype=pd.objecttype) and
                   not(oo_is_formal in tobjectdef(ttypesym(srsym).typedef).objectoptions) then
                  begin
                    if not(oo_is_forward in tobjectdef(ttypesym(srsym).typedef).objectoptions) then
                      begin
                        { the external name for the formal and the real
                          definition must match }
                        if assigned(tobjectdef(ttypesym(srsym).typedef).import_lib) or
                           assigned(pd.import_lib) then
                          begin
                            if assigned(pd.import_lib) then
                              formalname:=pd.import_lib^+'.'
                            else
                              formalname:='';
                            formalname:=formalname+pd.objextname^;
                            if assigned(tobjectdef(ttypesym(srsym).typedef).import_lib) then
                              foundname:=tobjectdef(ttypesym(srsym).typedef).import_lib^+'.'
                            else
                              foundname:='';
                            foundname:=foundname+tobjectdef(ttypesym(srsym).typedef).objextname^;

                            formalnameptr:=@formalname;
                            foundnameptr:=@foundname;
                          end
                        else
                          begin
                            formalnameptr:=pd.objextname;
                            foundnameptr:=tobjectdef(ttypesym(srsym).typedef).objextname;
                          end;
                        if foundnameptr^<>formalnameptr^ then
                          begin
                            MessagePos2(pd.typesym.fileinfo,sym_e_external_class_name_mismatch1,formalnameptr^,pd.typename);
                            MessagePos1(srsym.fileinfo,sym_e_external_class_name_mismatch2,foundnameptr^);
                          end;
                      end;
                    result:=tobjectdef(ttypesym(srsym).typedef);
                    if assigned(current_procinfo) and
                       (srsym.owner.symtabletype=staticsymtable) then
                      include(current_procinfo.flags,pi_uses_static_symtable);
                    addsymref(srsym);
                    exit;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        { nothing found: optionally give an error and return the original
          (empty) one }
        if erroronfailure then
          Message1(sym_e_formal_class_not_resolved,pd.objrealname^);
        result:=pd;
      end;


    function searchsym_in_class(classh: tobjectdef;contextclassh:tabstractrecorddef;const s : TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags):boolean;
      var
        hashedid : THashedIDString;
        orgclass : tobjectdef;
        i        : longint;
      begin
        orgclass:=classh;
        { in case this is a formal class, first find the real definition }
        if assigned(classh) then
          begin
            if (oo_is_formal in classh.objectoptions) then
              classh:=find_real_class_definition(classh,true);
            { The contextclassh is used for visibility. The classh must be equal to
              or be a parent of contextclassh. E.g. for inherited searches the classh is the
              parent or a class helper. }
            if not (def_is_related(contextclassh,classh) or
                (is_classhelper(contextclassh) and
                 assigned(tobjectdef(contextclassh).extendeddef) and
                (tobjectdef(contextclassh).extendeddef.typ=objectdef) and
                def_is_related(tobjectdef(contextclassh).extendeddef,classh))) then
              internalerror(200811161);
          end;
        result:=false;
        hashedid.id:=s;
        { an Objective-C  protocol or Java interface can inherit from multiple
          other protocols/interfaces -> use ImplementedInterfaces instead }
        if is_objcprotocol(classh) or
           is_javainterface(classh) then
          begin
            srsymtable:=classh.symtable;
            srsym:=tsym(srsymtable.FindWithHash(hashedid));
            if assigned(srsym) and
               is_visible_for_object(srsym,contextclassh) then
              begin
                if not (ssf_no_addsymref in flags) then
                  addsymref(srsym);
                result:=true;
                exit;
              end;
            for i:=0 to classh.ImplementedInterfaces.count-1 do
              begin
                if searchsym_in_class(TImplementedInterface(classh.ImplementedInterfaces[i]).intfdef,contextclassh,s,srsym,srsymtable,flags-[ssf_search_helper]) then
                  begin
                    result:=true;
                    exit;
                  end;
              end;
          end
        else
        if is_objectpascal_helper(classh) then
          begin
            { helpers have their own obscure search logic... }
            result:=searchsym_in_helper(classh,tobjectdef(contextclassh),s,srsym,srsymtable,flags-[ssf_has_inherited]);
            if result then
              exit;
          end
        else
          begin
            while assigned(classh) do
              begin
                { search for a class helper method first if this is an Object
                  Pascal class and we haven't yet found a helper symbol }
                if (classh.objecttype in objecttypes_with_helpers) and
                    (ssf_search_helper in flags) then
                  begin
                    result:=search_objectpascal_helper(classh,contextclassh,s,srsym,srsymtable);
                    { an eventual overload inside the extended type's hierarchy
                      will be found by tcallcandidates }
                    if result then
                      exit;
                  end;
                srsymtable:=classh.symtable;
                srsym:=tsym(srsymtable.FindWithHash(hashedid));
                if assigned(srsym) and
                   is_visible_for_object(srsym,contextclassh) then
                  begin
                    if not (ssf_no_addsymref in flags) then
                      addsymref(srsym);
                    result:=true;
                    exit;
                  end;
                classh:=classh.childof;
              end;
          end;
        if is_objcclass(orgclass) then
          result:=search_objc_helper(orgclass,s,srsym,srsymtable)
        else
          begin
            srsym:=nil;
            srsymtable:=nil;
          end;
      end;

    function  searchsym_in_record(recordh:tabstractrecorddef;const s : TIDString;out srsym:tsym;out srsymtable:TSymtable):boolean;
      var
        hashedid : THashedIDString;
      begin
        result:=false;
        hashedid.id:=s;
        { search for a record helper method first }
        result:=search_objectpascal_helper(recordh,recordh,s,srsym,srsymtable);
        if result then
          { an eventual overload inside the extended type's hierarchy
            will be found by tcallcandidates }
          exit;
        srsymtable:=recordh.symtable;
        srsym:=tsym(srsymtable.FindWithHash(hashedid));
        if assigned(srsym) and is_visible_for_object(srsym,recordh) then
          begin
            addsymref(srsym);
            result:=true;
            exit;
          end;
        srsym:=nil;
        srsymtable:=nil;
      end;

    function searchsym_in_class_by_msgint(classh:tobjectdef;msgid:longint;out srdef : tdef;out srsym:tsym;out srsymtable:TSymtable):boolean;
      var
        def : tdef;
        i   : longint;
      begin
        { in case this is a formal class, first find the real definition }
        if assigned(classh) and
           (oo_is_formal in classh.objectoptions) then
          classh:=find_real_class_definition(classh,true);
        result:=false;
        def:=nil;
        while assigned(classh) do
          begin
            for i:=0 to classh.symtable.DefList.Count-1 do
              begin
                def:=tstoreddef(classh.symtable.DefList[i]);
                { Find also all hidden private methods to
                  be compatible with delphi, see tw6203 (PFV) }
                if (def.typ=procdef) and
                   (po_msgint in tprocdef(def).procoptions) and
                   (tprocdef(def).messageinf.i=msgid) then
                  begin
                    srdef:=def;
                    srsym:=tprocdef(def).procsym;
                    srsymtable:=classh.symtable;
                    addsymref(srsym);
                    result:=true;
                    exit;
                  end;
              end;
            classh:=classh.childof;
          end;
        srdef:=nil;
        srsym:=nil;
        srsymtable:=nil;
      end;


    function searchsym_in_class_by_msgstr(classh:tobjectdef;const s:string;out srsym:tsym;out srsymtable:TSymtable):boolean;
      var
        def : tdef;
        i   : longint;
      begin
        { in case this is a formal class, first find the real definition }
        if assigned(classh) and
           (oo_is_formal in classh.objectoptions) then
          classh:=find_real_class_definition(classh,true);
        result:=false;
        def:=nil;
        while assigned(classh) do
          begin
            for i:=0 to classh.symtable.DefList.Count-1 do
              begin
                def:=tstoreddef(classh.symtable.DefList[i]);
                { Find also all hidden private methods to
                  be compatible with delphi, see tw6203 (PFV) }
                if (def.typ=procdef) and
                   (po_msgstr in tprocdef(def).procoptions) and
                   (tprocdef(def).messageinf.str^=s) then
                  begin
                    srsym:=tprocdef(def).procsym;
                    srsymtable:=classh.symtable;
                    addsymref(srsym);
                    result:=true;
                    exit;
                  end;
              end;
            classh:=classh.childof;
          end;
        srsym:=nil;
        srsymtable:=nil;
      end;

    function search_best_objectpascal_helper(const name: string;pd : tdef;contextclassh : tabstractrecorddef;out srsym: tsym;out srsymtable: tsymtable):boolean;forward;

    function searchsym_in_helper(classh,contextclassh:tobjectdef;const s: TIDString;out srsym:tsym;out srsymtable:TSymtable;flags:tsymbol_search_flags):boolean;
      var
        hashedid      : THashedIDString;
        parentclassh  : tobjectdef;
      begin
        result:=false;
        if not is_objectpascal_helper(classh) then
          Internalerror(2011030101);
        hashedid.id:=s;
        { in a helper things are a bit more complex:
          1. search the symbol in the helper (if not "inherited")
          2. search the symbol in the extended type
          3. search the symbol in the parent helpers
          4. only classes: search the symbol in the parents of the extended type
        }
        if not (ssf_has_inherited in flags) then
          begin
            { search in the helper itself }
            srsymtable:=classh.symtable;
            srsym:=tsym(srsymtable.FindWithHash(hashedid));
            if assigned(srsym) and
               is_visible_for_object(srsym,contextclassh) then
              begin
                if not (ssf_no_addsymref in flags) then
                  addsymref(srsym);
                result:=true;
                exit;
              end;
          end;
        { now search in the extended type itself }
        { Note: the extendeddef might be Nil if we are currently parsing the
                extended type itself and the identifier was not found }
        if assigned(classh.extendeddef) and (classh.extendeddef.typ in [recorddef,objectdef]) then
          begin
            srsymtable:=tabstractrecorddef(classh.extendeddef).symtable;
            srsym:=tsym(srsymtable.FindWithHash(hashedid));
            if assigned(srsym) and
               is_visible_for_object(srsym,contextclassh) then
              begin
                if not (ssf_no_addsymref in flags) then
                  addsymref(srsym);
                result:=true;
                exit;
              end;
          end;
        { now search in the parent helpers }
        parentclassh:=classh.childof;
        while assigned(parentclassh) do
          begin
            srsymtable:=parentclassh.symtable;
            srsym:=tsym(srsymtable.FindWithHash(hashedid));
            if assigned(srsym) and
               is_visible_for_object(srsym,contextclassh) then
              begin
                if not (ssf_no_addsymref in flags) then
                  addsymref(srsym);
                result:=true;
                exit;
              end;
            parentclassh:=parentclassh.childof;
          end;
        { now search in the parents of the extended class (with helpers!) }
        if is_class(classh.extendeddef) then
          begin
            result:=searchsym_in_class(tobjectdef(classh.extendeddef).childof,contextclassh,s,srsym,srsymtable,flags+[ssf_search_helper]);
            { addsymref is already called by searchsym_in_class }
            if result then
              exit;
          end;
        { now search all helpers using the extendeddef as the starting point }
        if (m_multi_helpers in current_settings.modeswitches) and
            (
              (current_structdef<>classh) or
              assigned(classh.extendeddef)
            ) then
          begin
            { this is only allowed if classh is currently parsed }
            if not assigned(classh.extendeddef) then
              internalerror(2019110101);
            result:=search_best_objectpascal_helper(s,classh.extendeddef,contextclassh,srsym,srsymtable);
          end;
      end;

    function search_specific_assignment_operator(assignment_type:ttoken;from_def,to_def:Tdef):Tprocdef;
      var
        sym : Tprocsym;
        hashedid : THashedIDString;
        curreq,
        besteq : tequaltype;
        currpd,
        bestpd : tprocdef;
        stackitem : psymtablestackitem;
      begin
        hashedid.id:=overloaded_names[assignment_type];
        besteq:=te_incompatible;
        bestpd:=nil;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            sym:=Tprocsym(stackitem^.symtable.FindWithHash(hashedid));
            if sym<>nil then
              begin
                if sym.typ<>procsym then
                  internalerror(200402031);
                { if the source type is an alias then this is only the second choice,
                  if you mess with this code, check tw4093 }
                currpd:=sym.find_procdef_assignment_operator(from_def,to_def,curreq);
                if curreq>besteq then
                  begin
                    besteq:=curreq;
                    bestpd:=currpd;
                    if (besteq=te_exact) then
                      break;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        result:=bestpd;
      end;


    function search_assignment_operator(from_def,to_def:Tdef;explicit:boolean):Tprocdef;
      begin
        { search record/object symtable first for a suitable operator }
        if from_def.typ in [recorddef,objectdef] then
          symtablestack.push(tabstractrecorddef(from_def).symtable);
        if to_def.typ in [recorddef,objectdef] then
          symtablestack.push(tabstractrecorddef(to_def).symtable);

        { if type conversion is explicit then search first for explicit
          operator overload and if not found then use implicit operator }
        if explicit then
          result:=search_specific_assignment_operator(_OP_EXPLICIT,from_def,to_def)
        else
          result:=nil;
        if result=nil then
          result:=search_specific_assignment_operator(_ASSIGNMENT,from_def,to_def);

        { restore symtable stack }
        if to_def.typ in [recorddef,objectdef] then
          symtablestack.pop(tabstractrecorddef(to_def).symtable);
        if from_def.typ in [recorddef,objectdef] then
          symtablestack.pop(tabstractrecorddef(from_def).symtable);
      end;


    function search_enumerator_operator(from_def,to_def:Tdef): Tprocdef;
      var
        sym : Tprocsym;
        hashedid : THashedIDString;
        curreq,
        besteq : tequaltype;
        currpd,
        bestpd : tprocdef;
        stackitem : psymtablestackitem;
      begin
        hashedid.id:='enumerator';
        besteq:=te_incompatible;
        bestpd:=nil;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            sym:=Tprocsym(stackitem^.symtable.FindWithHash(hashedid));
            if sym<>nil then
              begin
                if sym.typ<>procsym then
                  internalerror(200910241);
                { if the source type is an alias then this is only the second choice,
                  if you mess with this code, check tw4093 }
                currpd:=sym.find_procdef_enumerator_operator(from_def,to_def,curreq);
                if curreq>besteq then
                  begin
                    besteq:=curreq;
                    bestpd:=currpd;
                    if (besteq=te_exact) then
                      break;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        result:=bestpd;
    end;


    function search_management_operator(mop:tmanagementoperator;pd:Tdef):Tprocdef;
      var
        sym : Tprocsym;
        hashedid : THashedIDString;
        optoken: ttoken;
      begin
        optoken := managementoperator2tok[mop];
        if (optoken<first_managment_operator) or
           (optoken>last_managment_operator) then
          internalerror(201602280);
        hashedid.id:=overloaded_names[optoken];
        if not (pd.typ in [recorddef]) then
          internalerror(201602281);
        sym:=Tprocsym(tabstractrecorddef(pd).symtable.FindWithHash(hashedid));
        if sym<>nil then
          begin
            if sym.typ<>procsym then
              internalerror(201602282);
            result:=sym.find_procdef_bytype(potype_operator);
          end
        else
          result:=nil;
      end;


    function search_system_type(const s: TIDString): ttypesym;
      var
        sym : tsym;
      begin
        sym:=tsym(systemunit.Find(s));
        if not assigned(sym) or
           (sym.typ<>typesym) then
          message1(cg_f_unknown_system_type,s);
        result:=ttypesym(sym);
      end;


    function try_search_system_type(const s: TIDString): ttypesym;
      var
        sym : tsym;
      begin
        sym:=tsym(systemunit.Find(s));
        if not assigned(sym) then
          result:=nil
        else
          begin
            if sym.typ<>typesym then
              message1(cg_f_unknown_system_type,s);
            result:=ttypesym(sym);
          end;
      end;


    function try_search_current_module_type(const s: TIDString): ttypesym;
      var
        found: boolean;
        srsymtable: tsymtable;
        srsym: tsym;
      begin
        if s[1]='$' then
          found:=searchsym_in_module(current_module,copy(s,2,length(s)),srsym,srsymtable)
        else
          found:=searchsym_in_module(current_module,s,srsym,srsymtable);
        if found then
          begin
            if (srsym.typ<>typesym) then
              internalerror(2014091207);
            result:=ttypesym(srsym);
          end
        else
          result:=nil;
      end;


    function  search_system_proc(const s: TIDString): tprocdef;
      var
        srsym: tsym;
      begin
        srsym:=tsym(systemunit.find(s));
        if not assigned(srsym) and
           (cs_compilesystem in current_settings.moduleswitches) then
          srsym:=tsym(systemunit.Find(upper(s)));
        if not assigned(srsym) or
           (srsym.typ<>procsym) then
          message1(cg_f_unknown_compilerproc,s);
        result:=tprocdef(tprocsym(srsym).procdeflist[0]);
    end;


    function search_named_unit_globaltype(const unitname, typename: TIDString; throwerror: boolean): ttypesym;
      var
        srsymtable: tsymtable;
        sym: tsym;
      begin
        sym:=nil;
        if searchsym_in_named_module(unitname,typename,sym,srsymtable) and
           (sym.typ=typesym) then
          begin
            result:=ttypesym(sym);
            exit;
          end
        else
          begin
            if throwerror then
              message2(cg_f_unknown_type_in_unit,typename,unitname);
            result:=nil;
          end;
      end;

    function search_sym_in_helperdef(const s: string;classh : tobjectdef;contextclassh : tabstractrecorddef;out srsym: tsym;out srsymtable: tsymtable): boolean;
      var
        hashedid : THashedIDString;
        pdef : tprocdef;
        i : integer;
      begin
        hashedid.id:=s;
        result:=false;
        repeat
          srsymtable:=classh.symtable;
          srsym:=tsym(srsymtable.FindWithHash(hashedid));
          if srsym<>nil then
            begin
              case srsym.typ of
                procsym:
                  begin
                    for i:=0 to tprocsym(srsym).procdeflist.count-1 do
                      begin
                        pdef:=tprocdef(tprocsym(srsym).procdeflist[i]);
                        if not is_visible_for_object(pdef.owner,pdef.visibility,contextclassh) then
                          continue;
                        srsym:=tprocdef(tprocsym(srsym).procdeflist[i]).procsym;
                        srsymtable:=srsym.owner;
                        result:=true;
                        exit;
                      end;
                  end;
                typesym,
                fieldvarsym,
                constsym,
                enumsym,
                undefinedsym,
                propertysym:
                  begin
                    result:=true;
                    exit;
                  end;
                else
                  internalerror(2014041101);
              end;
            end;

          { try the helper parent if available }
          classh:=classh.childof;
        until classh=nil;
      end;

    function get_objectpascal_helpers(pd : tdef):TFPObjectList;
      var
        s : string;
        st : tsymtable;
      begin
        result:=nil;
        { when there are no helpers active currently then we don't need to do
          anything }
        if current_module.extendeddefs.count=0 then
          exit;
        if (df_genconstraint in pd.defoptions) then
          begin
            { if we have a constraint for a class type or a single interface we
              use that to resolve helpers at declaration time of the generic,
              otherwise there can't be any helpers as the type isn't known yet }
            if pd.typ=objectdef then
              pd:=tobjectdef(pd).getparentdef
            else
              exit;
          end;
        { no helpers for anonymous types }
        if ((pd.typ in [recorddef,objectdef]) and
            (
              not assigned(tabstractrecorddef(pd).objrealname) or
              (tabstractrecorddef(pd).objrealname^='')
            )
           ) or
           not assigned(pd.typesym) then
          exit;
        { if pd is defined inside a procedure we must not use make_mangledname
          (as a helper may not be defined in a procedure this is no problem...)}
        st:=pd.owner;
        while st.symtabletype in [objectsymtable,recordsymtable] do
          st:=st.defowner.owner;
        if st.symtabletype=localsymtable then
          exit;
        { the mangled name is used as the key for tmodule.extendeddefs }
        s:=generate_objectpascal_helper_key(pd);
        result:=TFPObjectList(current_module.extendeddefs.Find(s));
      end;

    function search_best_objectpascal_helper(const name: string;pd : tdef;contextclassh : tabstractrecorddef;out srsym: tsym;out srsymtable: tsymtable):boolean;
      var
        s : string;
        list : TFPObjectList;
        i : integer;
        st : tsymtable;
        odef : tobjectdef;
      begin
        result:=false;
        list:=get_objectpascal_helpers(pd);
        if assigned(list) and (list.count>0) then
          begin
            i:=list.count-1;
            repeat
              odef:=tobjectdef(list[i]);
              result:=(odef.owner.symtabletype in [staticsymtable,globalsymtable]) or
                      is_visible_for_object(tobjectdef(list[i]).typesym,contextclassh);
              if result then
                result:=search_sym_in_helperdef(name,odef,contextclassh,srsym,srsymtable);
              dec(i);
            until result or (i<0);
          end;
      end;

    function search_last_objectpascal_helper(pd : tdef;contextclassh : tabstractrecorddef;out odef : tobjectdef):boolean;
      var
        s : string;
        list : TFPObjectList;
        i : integer;
      begin
        result:=false;
        odef:=nil;
        list:=get_objectpascal_helpers(pd);
        if assigned(list) and (list.count>0) then
          begin
            i:=list.count-1;
            repeat
              odef:=tobjectdef(list[list.count-1]);
              result:=(odef.owner.symtabletype in [staticsymtable,globalsymtable]) or
                      is_visible_for_object(tobjectdef(list[i]).typesym,contextclassh);
              dec(i);
            until result or (i<0);
            if not result then
              { just to be sure that noone uses odef }
              odef:=nil;
          end;
      end;

    function search_objectpascal_helper(pd : tdef;contextclassh : tabstractrecorddef;const s: string; out srsym: tsym; out srsymtable: tsymtable):boolean;
      var
        classh : tobjectdef;
      begin
        result:=false;

        { if there is no class helper for the class then there is no need to
          search further }
        if m_multi_helpers in current_settings.modeswitches then
          result:=search_best_objectpascal_helper(s,pd,contextclassh,srsym,srsymtable)
        else
          begin
            if search_last_objectpascal_helper(pd,contextclassh,classh) and
               search_sym_in_helperdef(s,classh,contextclassh,srsym,srsymtable) then
                result:=true;
          end;

        if result then
          begin
            { we need to know if a procedure references symbols
              in the static symtable, because then it can't be
              inlined from outside this unit }
            if (srsym.typ=procsym) and
               assigned(current_procinfo) and
               (srsym.owner.symtabletype=staticsymtable) then
              include(current_procinfo.flags,pi_uses_static_symtable);
            addsymref(srsym);
          end
        else
          begin
            srsym:=nil;
            srsymtable:=nil;
          end;
      end;

    function search_objc_helper(pd : tobjectdef;const s : string; out srsym: tsym; out srsymtable: tsymtable):boolean;
      var
        searchst   : tsymtable;
        searchsym  : tsym;
        hashedid   : THashedIDString;
        stackitem  : psymtablestackitem;
        i          : longint;
        founddefowner,
        defowner   : tobjectdef;
      begin
        hashedid.id:=class_helper_prefix+s;
        stackitem:=symtablestack.stack;
        result:=false;
        srsym:=nil;
        srsymtable:=nil;
        founddefowner:=nil;
        while assigned(stackitem) do
          begin
            searchst:=stackitem^.symtable;
            searchsym:=tsym(searchst.FindWithHash(hashedid));
            if assigned(searchsym) then
              begin
                if not(searchst.symtabletype in [globalsymtable,staticsymtable]) or
                   not(searchsym.owner.symtabletype in [globalsymtable,staticsymtable]) or
                   (searchsym.typ<>procsym) then
                  internalerror(2009111505);
                { check whether this procsym includes a helper for this particular class }
                for i:=0 to tprocsym(searchsym).procdeflist.count-1 do
                  begin
                    { does pd inherit from (or is the same as) the class
                      that this method's category extended?

                      Warning: this list contains both category and objcclass methods
                       (for id.randommethod), so only check category methods here
                    }
                    defowner:=tobjectdef(tprocdef(tprocsym(searchsym).procdeflist[i]).owner.defowner);
                    if is_objccategory(defowner) and
                       def_is_related(pd,defowner.childof) then
                      begin
                        { we need to know if a procedure references symbols
                          in the static symtable, because then it can't be
                          inlined from outside this unit }
                        if assigned(current_procinfo) and
                           (searchsym.owner.symtabletype=staticsymtable) then
                          include(current_procinfo.flags,pi_uses_static_symtable);
                        { Stop looking if this is a category that extends the specified
                          class itself. There might be other categories that extend this,
                          but that doesn't matter. If it extens a parent, keep looking
                          in case we find the symbol in a category that extends this class
                          (or a closer parent).
                        }
                        if not result or
                           def_is_related(defowner.childof,founddefowner) then
                          begin
                            founddefowner:=defowner.childof;
                            srsym:=tprocdef(tprocsym(searchsym).procdeflist[i]).procsym;
                            srsymtable:=srsym.owner;
                            result:=true;
                            if pd=founddefowner then
                              begin
                                addsymref(srsym);
                                exit;
                              end;
                          end;
                      end;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        if result then
          begin
            addsymref(srsym);
            exit;
          end;
      end;


    function search_objc_method(const s : string; out srsym: tsym; out srsymtable: tsymtable):boolean;
      var
        hashedid   : THashedIDString;
        stackitem  : psymtablestackitem;
        i          : longint;
      begin
        hashedid.id:=class_helper_prefix+s;
        stackitem:=symtablestack.stack;
        while assigned(stackitem) do
          begin
            srsymtable:=stackitem^.symtable;
            srsym:=tsym(srsymtable.FindWithHash(hashedid));
            if assigned(srsym) then
              begin
                if not(srsymtable.symtabletype in [globalsymtable,staticsymtable]) or
                   not(srsym.owner.symtabletype in [globalsymtable,staticsymtable]) or
                   (srsym.typ<>procsym) then
                  internalerror(2009112005);
                { check whether this procsym includes a helper for this particular class }
                for i:=0 to tprocsym(srsym).procdeflist.count-1 do
                  begin
                    { we need to know if a procedure references symbols
                      in the static symtable, because then it can't be
                      inlined from outside this unit }
                    if assigned(current_procinfo) and
                       (srsym.owner.symtabletype=staticsymtable) then
                      include(current_procinfo.flags,pi_uses_static_symtable);
                    { no need to keep looking. There might be other
                      methods with the same name, but that doesn't matter
                      as far as the basic procsym is concerned.
                    }
                    srsym:=tprocdef(tprocsym(srsym).procdeflist[i]).procsym;
                    { We need the symtable in which the classhelper-like sym
                      is located, not the objectdef. The reason is that the
                      callnode will climb the symtablestack until it encounters
                      this symtable to start looking for overloads (and it won't
                      find the objectsymtable in which this method sym is
                      located

                    srsymtable:=srsym.owner;
                    }
                    addsymref(srsym);
                    result:=true;
                    exit;
                  end;
              end;
            stackitem:=stackitem^.next;
          end;
        srsym:=nil;
        srsymtable:=nil;
        result:=false;
      end;


    function search_struct_member(pd : tabstractrecorddef;const s : string):tsym;
    { searches n in symtable of pd and all anchestors }
      var
        srsymtable : tsymtable;
      begin
        { in case this is a formal class, first find the real definition }
        if (oo_is_formal in pd.objectoptions) then
          pd:=find_real_class_definition(tobjectdef(pd),true);

        if search_objectpascal_helper(pd, pd, s, result, srsymtable) then
          exit;

        result:=search_struct_member_no_helper(pd,s);
        if assigned(result) then
          exit;

        { not found, now look for class helpers }
        if is_objcclass(pd) then
          search_objc_helper(tobjectdef(pd),s,result,srsymtable)
      end;


    function search_struct_member_no_helper(pd: tabstractrecorddef; const s: string): tsym;
      var
        hashedid   : THashedIDString;
        srsym      : tsym;
      begin
        hashedid.id:=s;
        while assigned(pd) do
         begin
            srsym:=tsym(pd.symtable.FindWithHash(hashedid));
            if assigned(srsym) then
              begin
                result:=srsym;
                exit;
              end;
            if pd.typ=objectdef then
              pd:=tobjectdef(pd).childof
            else
              pd:=nil;
          end;
        result:=nil;
      end;


    function search_macro(const s : string):tsym;
      var
        stackitem  : psymtablestackitem;
        hashedid   : THashedIDString;
        srsym      : tsym;
      begin
        hashedid.id:=s;

        { First search the localmacrosymtable before searching the
          global macrosymtables from the units }
        if assigned(current_module) then
          begin
            srsym:=tsym(current_module.localmacrosymtable.FindWithHash(hashedid));
            if assigned(srsym) then
              begin
                result:= srsym;
                exit;
              end;
          end;

        stackitem:=macrosymtablestack.stack;
        while assigned(stackitem) do
          begin
            srsym:=tsym(stackitem^.symtable.FindWithHash(hashedid));
            if assigned(srsym) then
              begin
                result:= srsym;
                exit;
              end;
            stackitem:=stackitem^.next;
          end;
        result:= nil;
      end;


    function defined_macro(const s : string):boolean;
      var
        mac: tmacro;
      begin
        mac:=tmacro(search_macro(s));
        if assigned(mac) then
          begin
            mac.is_used:=true;
            defined_macro:=mac.defined;
          end
        else
          defined_macro:=false;
      end;


{****************************************************************************
                              Object Helpers
****************************************************************************}

   function search_default_property(pd : tabstractrecorddef) : tpropertysym;
   { returns the default property of a class, searches also anchestors }
     var
       _defaultprop : tpropertysym;
       helperpd : tobjectdef;
     begin
        _defaultprop:=nil;
        { first search in helper's hierarchy }
        if search_last_objectpascal_helper(pd,nil,helperpd) then
          while assigned(helperpd) do
            begin
              helperpd.symtable.SymList.ForEachCall(@tstoredsymtable(helperpd.symtable).testfordefaultproperty,@_defaultprop);
              if assigned(_defaultprop) then
                break;
              helperpd:=helperpd.childof;
            end;
        if assigned(_defaultprop) then
          begin
            search_default_property:=_defaultprop;
            exit;
          end;
        { now search in the type's hierarchy itself }
        while assigned(pd) do
          begin
             pd.symtable.SymList.ForEachCall(@tstoredsymtable(pd.symtable).testfordefaultproperty,@_defaultprop);
             if assigned(_defaultprop) then
               break;
             if (pd.typ=objectdef) then
               pd:=tobjectdef(pd).childof
             else
               break;
          end;
        search_default_property:=_defaultprop;
     end;


{****************************************************************************
                              Macro Helpers
****************************************************************************}

    procedure def_system_macro(const name : string);
      var
        mac : tmacro;
        s: string;
      begin
         if name = '' then
           internalerror(2004121202);
         s:= upper(name);
         mac:=tmacro(search_macro(s));
         if not assigned(mac) then
           begin
             mac:=tmacro.create(s);
             if assigned(current_module) then
               current_module.localmacrosymtable.insert(mac)
             else
               initialmacrosymtable.insert(mac);
           end;
         Message1(parser_c_macro_defined,mac.name);
         mac.defined:=true;
      end;


    procedure set_system_macro(const name, value : string);
      var
        mac : tmacro;
        s: string;
      begin
        if name = '' then
          internalerror(2004121203);
         s:= upper(name);
         mac:=tmacro(search_macro(s));
         if not assigned(mac) then
           begin
             mac:=tmacro.create(s);
             if assigned(current_module) then
               current_module.localmacrosymtable.insert(mac)
             else
               initialmacrosymtable.insert(mac);
           end
         else
           begin
             mac.is_compiler_var:=false;
             if assigned(mac.buftext) then
               freemem(mac.buftext,mac.buflen);
           end;
         Message2(parser_c_macro_set_to,mac.name,value);
         mac.buflen:=length(value);
         getmem(mac.buftext,mac.buflen);
         move(value[1],mac.buftext^,mac.buflen);
         mac.defined:=true;
      end;


    procedure set_system_compvar(const name, value : string);
      var
        mac : tmacro;
        s: string;
      begin
        if name = '' then
          internalerror(2004121204);
         s:= upper(name);
         mac:=tmacro(search_macro(s));
         if not assigned(mac) then
           begin
             mac:=tmacro.create(s);
             mac.is_compiler_var:=true;
             if assigned(current_module) then
               current_module.localmacrosymtable.insert(mac)
             else
               initialmacrosymtable.insert(mac);
           end
         else
           begin
             mac.is_compiler_var:=true;
             if assigned(mac.buftext) then
               freemem(mac.buftext,mac.buflen);
           end;
         Message2(parser_c_macro_set_to,mac.name,value);
         mac.buflen:=length(value);
         getmem(mac.buftext,mac.buflen);
         move(value[1],mac.buftext^,mac.buflen);
         mac.defined:=true;
      end;


    procedure undef_system_macro(const name : string);
      var
        mac : tmacro;
        s: string;
      begin
         if name = '' then
           internalerror(2004121205);
         s:= upper(name);
         mac:=tmacro(search_macro(s));
         if not assigned(mac) then
           {If not found, then it's already undefined.}
         else
           begin
             Message1(parser_c_macro_undefined,mac.name);
             mac.defined:=false;
             mac.is_compiler_var:=false;
             { delete old definition }
             if assigned(mac.buftext) then
               begin
                  freemem(mac.buftext,mac.buflen);
                  mac.buftext:=nil;
               end;
           end;
      end;


{$ifdef UNITALIASES}
{****************************************************************************
                              TUNIT_ALIAS
 ****************************************************************************}

    constructor tunit_alias.create(const n:string);
      var
        i : longint;
      begin
        i:=pos('=',n);
        if i=0 then
         fail;
        inherited createname(Copy(n,1,i-1));
        newname:=stringdup(Copy(n,i+1,255));
      end;


    destructor tunit_alias.destroy;
      begin
        stringdispose(newname);
        inherited destroy;
      end;


    procedure addunitalias(const n:string);
      begin
        unitaliases^.insert(tunit_alias,init(Upper(n))));
      end;


    function getunitalias(const n:string):string;
      var
        p : punit_alias;
      begin
        p:=punit_alias(unitaliases^.Find(Upper(n)));
        if assigned(p) then
         getunitalias:=punit_alias(p).newname^
        else
         getunitalias:=n;
      end;
{$endif UNITALIASES}


{****************************************************************************
                           Init/Done Symtable
****************************************************************************}

   procedure InitSymtable;
     begin
       { Reset symbolstack }
       symtablestack:=nil;
       systemunit:=nil;
       { create error syms and def }
       generrorsym:=terrorsym.create;
       generrordef:=cerrordef.create;
       { macros }
       initialmacrosymtable:=tmacrosymtable.create(false);
       macrosymtablestack:=TSymtablestack.create;
       macrosymtablestack.push(initialmacrosymtable);
{$ifdef UNITALIASES}
       { unit aliases }
       unitaliases:=TFPHashObjectList.create;
{$endif}
       { set some global vars to nil, might be important for the ide }
       class_tobject:=nil;
       class_tcustomattribute:=nil;
       interface_iunknown:=nil;
       interface_idispatch:=nil;
       rec_tguid:=nil;
       rec_jmp_buf:=nil;
       rec_exceptaddr:=nil;
       objc_metaclasstype:=nil;
       objc_superclasstype:=nil;
       objc_idtype:=nil;
       objc_seltype:=nil;
       objc_objecttype:=nil;
       dupnr:=0;
     end;


   procedure DoneSymtable;
      begin
        generrorsym.owner:=nil;
        generrorsym.free;
        generrordef.owner:=nil;
        generrordef.free;
        initialmacrosymtable.free;
        macrosymtablestack.free;
{$ifdef UNITALIASES}
        unitaliases.free;
{$endif}
     end;

end.
