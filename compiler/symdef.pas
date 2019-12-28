{
    Symbol table implementation for the definitions

    Copyright (c) 1998-2005 by Florian Klaempfl, Pierre Muller

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
unit symdef;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cclasses,widestr,
       { global }
       globtype,globals,tokens,constexp,
       { symtable }
       symconst,symbase,symtype,
       { node }
       node,
       { aasm }
       aasmtai,
       cpuinfo,
       cgbase,
       parabase
       ;


    type
{************************************************
                    TDef
************************************************}

       tgenericconstraintdata=class
         interfaces : tfpobjectlist;
         interfacesderef : tfplist;
         flags : tgenericconstraintflags;
         constructor create;
         destructor destroy;override;
         procedure ppuload(ppufile:tcompilerppufile);
         procedure ppuwrite(ppufile:tcompilerppufile);
         procedure buildderef;
         procedure deref;
       end;

       { trtti_attribute_list }

       trtti_attribute = class
          typesym         : tsym;
          typesymderef    : tderef;
          typeconstr      : tdef;
          typeconstrderef : tderef;
          { these two are not stored in PPU }
          constructorcall : tnode;
          constructorpd   : tdef;
          paras           : array of tnode;
          constructor ppuload(ppufile:tcompilerppufile);
          procedure ppuwrite(ppufile:tcompilerppufile);
          procedure ppuload_subentries(ppufile:tcompilerppufile);
          procedure ppuwrite_subentries(ppufile:tcompilerppufile);
          destructor destroy;override;
          procedure buildderef;
          procedure deref;
       end;

       trtti_attribute_list = class
          rtti_attributes : TFPObjectList;
          { if the attribute list is bound to a def or symbol }
          is_bound : Boolean;
          class procedure bind(var dangling,owned:trtti_attribute_list);
          procedure addattribute(atypesym:tsym;typeconstr:tdef;constructorcall:tnode;constref paras:array of tnode);
          procedure addattribute(attr:trtti_attribute);
          destructor destroy; override;
          function get_attribute_count:longint;
          procedure buildderef;
          procedure deref;

          class function ppuload(ppufile:tcompilerppufile):trtti_attribute_list;
          class procedure ppuwrite(attrlist:trtti_attribute_list;ppufile:tcompilerppufile);
          class procedure ppuload_subentries(attrlist:trtti_attribute_list;ppufile:tcompilerppufile);
          class procedure ppuwrite_subentries(attrlist:trtti_attribute_list;ppufile:tcompilerppufile);
       end;

       { tstoreddef }

       tstoreddef = class(tdef)
       private
{$ifdef symansistr}
          _fullownerhierarchyname : ansistring;
{$else symansistr}
          _fullownerhierarchyname : pshortstring;
{$endif symansistr}
          procedure writeentry(ppufile: tcompilerppufile; ibnr: byte);
       protected
          typesymderef  : tderef;
          procedure ppuwrite_platform(ppufile:tcompilerppufile);virtual;
          procedure ppuload_platform(ppufile:tcompilerppufile);virtual;
          { a (possibly) reusable def is always created on the basis of another
            def, and contains a reference to this other def. If this other
            def is in a non-persistent symboltable, the new def cannot actually
            be safely reused everywhere in the current module. This routine
            abtracts that checking, and also restores the symtable stack
            (which had to be reset before creating the new def, so that the new
             def did not automatically get added to its top) }
          class procedure setup_reusable_def(origdef, newdef: tdef; res: PHashSetItem; oldsymtablestack: tsymtablestack);
       public
{$ifdef EXTDEBUG}
          fileinfo   : tfileposinfo;
{$endif}
          { generic support }
          genericdef      : tstoreddef;
          genericdefderef : tderef;
          generictokenbuf : tdynamicarray;
          { this list contains references to the symbols that make up the
            generic parameters; the symbols are not owned by this list
            Note: this list is allocated on demand! }
          genericparas    : tfphashobjectlist;
          genericparaderefs : tfplist;
          { contains additional data if this def is a generic constraint
            Note: this class is allocated on demand! }
          genconstraintdata : tgenericconstraintdata;
          { this is Nil if the def has no RTTI attributes }
          rtti_attribute_list : trtti_attribute_list;
          constructor create(dt:tdeftyp;doregister:boolean);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          destructor  destroy;override;
          function getcopy : tstoreddef;virtual;
          procedure ppuwrite(ppufile:tcompilerppufile);virtual;
          { this is called directly after ppuload }
          procedure ppuload_subentries(ppufile:tcompilerppufile);virtual;
          { this is called directly after ppuwrite }
          procedure ppuwrite_subentries(ppufile:tcompilerppufile);virtual;
          procedure buildderef;override;
          procedure buildderefimpl;override;
          procedure deref;override;
          procedure derefimpl;override;
          function  size:asizeint;override;
          function  getvardef:longint;override;
          function  alignment:shortint;override;
          function  is_publishable : boolean;override;
          function  needs_inittable : boolean;override;
          function  has_non_trivial_init_child(check_parent:boolean):boolean;override;
          function  rtti_mangledname(rt:trttitype):TSymStr;override;
          function  OwnerHierarchyName: string; override;
          function  fullownerhierarchyname(skipprocparams:boolean):TSymStr;override;
          function  needs_separate_initrtti:boolean;override;
          function  in_currentunit: boolean;
          { regvars }
          function is_intregable : boolean;
          function is_fpuregable : boolean;
          { def can be put into a register if it is const/immutable }
          function is_const_intregable : boolean;
          { generics }
          procedure initgeneric;
          { this function can be used to determine whether a def is really a
            generic declaration or just a normal type declared inside another
            generic }
          function is_generic:boolean;
          { same as above for specializations }
          function is_specialization:boolean;
          { registers this def in the unit's deflist; no-op if already registered }
          procedure register_def; override;
          { add the def to the top of the symtable stack if it's not yet owned
            by another symtable }
          procedure maybe_put_in_symtable_stack;
       private
          savesize  : asizeuint;
       end;

       tfiletyp = (ft_text,ft_typed,ft_untyped);

       tfiledef = class(tstoreddef)
          filetyp : tfiletyp;
          typedfiledef : tdef;
          typedfiledefderef : tderef;
          constructor createtext;virtual;
          constructor createuntyped;virtual;
          constructor createtyped(def : tdef);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function  GetTypeName:string;override;
          function  getmangledparaname:TSymStr;override;
          function  size:asizeint;override;
          procedure setsize;
          function alignment: shortint; override;
       end;
       tfiledefclass = class of tfiledef;

       tvariantdef = class(tstoreddef)
          varianttype : tvarianttype;
          constructor create(v : tvarianttype);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          function GetTypeName:string;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  getvardef:longint;override;
          procedure setsize;
          function is_publishable : boolean;override;
          function needs_inittable : boolean;override;
       end;
       tvariantdefclass = class of tvariantdef;

       tformaldef = class(tstoreddef)
          typed:boolean;
          constructor create(Atyped:boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
       end;
       tformaldefclass = class of tformaldef;

       tforwarddef = class(tstoreddef)
          tosymname : pshortstring;
          forwardpos : tfileposinfo;
          constructor create(const s:string;const pos:tfileposinfo);virtual;
          destructor destroy;override;
          function getcopy:tstoreddef;override;
          function GetTypeName:string;override;
       end;
       tforwarddefclass = class of tforwarddef;

       tundefineddef = class(tstoreddef)
          constructor create(doregister:boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
       end;
       tundefineddefclass = class of tundefineddef;

       terrordef = class(tstoreddef)
          constructor create;virtual;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
          function  getmangledparaname : TSymStr;override;
       end;
       terrordefclass = class of terrordef;

       tabstractpointerdef = class(tstoreddef)
          pointeddef : tdef;
          pointeddefderef : tderef;
          constructor create(dt:tdeftyp;def:tdef);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderef;override;
          procedure deref;override;
          function size:asizeint;override;
          function alignment:shortint;override;
       end;

       tpointerdef = class(tabstractpointerdef)
          has_pointer_math : boolean;
          constructor create(def:tdef);virtual;
          { returns a pointerdef for def, reusing an existing one in case it
            exists in the current module }
          class function getreusable(def: tdef): tpointerdef; virtual;
          { same as above, but in case the def must never be freed after the
            current module has been compiled -- even if the def was not written
            to the ppu file (for defs in para locations, as we don't reset them
            so we don't have to recalculate them all the time) }
          class function getreusable_no_free(def: tdef): tpointerdef;
          function size:asizeint;override;
          function getcopy:tstoreddef;override;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
          {# returns the appropriate int type for pointer arithmetic with the given pointer type.
             When adding or subtracting a number to/from a pointer, this function returns the
             int type to which that number has to be converted, before the operation can be performed.
             Normally, this is sinttype, except on i8086, where it takes into account the
             special i8086 pointer types (near, far, huge). }
          function pointer_arithmetic_int_type:tdef;virtual;
          {# the unsigned version of pointer_arithmetic_int_type. Used with inc/dec. }
          function pointer_arithmetic_uint_type:tdef;virtual;
          {# returns the int type produced when subtracting two pointers of the given type.
             Normally, this is sinttype, except on i8086, where it takes into account the
             special i8086 pointer types (near, far, huge). }
          function pointer_subtraction_result_type:tdef;virtual;
          function compatible_with_pointerdef_size(ptr: tpointerdef): boolean; virtual;
          {# the integer index type used to convert the pointer to array (i.e. denotes int_type in: ptr[int_type]) }
          function converted_pointer_to_array_range_type:tdef;virtual;
       end;
       tpointerdefclass = class of tpointerdef;

       tprocdef = class;

       tabstractrecorddef= class(tstoreddef)
       private
          rttistring     : string;
       public
          objname,
          objrealname    : PShortString;
          { for C++ classes: name of the library this class is imported from }
          { for Java classes/records: package name }
          import_lib     : PShortString;
          symtable       : TSymtable;
          cloneddef      : tabstractrecorddef;
          cloneddefderef : tderef;
          objectoptions  : tobjectoptions;
          { for targets that initialise typed constants via explicit assignments
            instead of by generating an initialised data sectino }
          tcinitcode     : tnode;
          constructor create(const n:string; dt:tdeftyp;doregister:boolean);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          destructor destroy; override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          procedure check_forwards; virtual;
          function find_procdef_bytype(pt:tproctypeoption): tprocdef;
          function GetSymtable(t:tGetSymtable):TSymtable;override;
          function is_packed:boolean;
          function RttiName: string;
          { enumerator support }
          function search_enumerator_get: tprocdef; virtual;
          function search_enumerator_move: tprocdef; virtual;
          function search_enumerator_current: tsym; virtual;
          { JVM }
          function jvm_full_typename(with_package_name: boolean): string;
          { check if the symtable contains a float field }
          function contains_float_field : boolean;
          { check if the symtable contains a field that spans an aword boundary }
          function contains_cross_aword_field: boolean;
       end;

       pvariantrecdesc = ^tvariantrecdesc;

       tvariantrecbranch = record
         { we store only single values here and no ranges because tvariantrecdesc is only needed in iso mode
           which does not support range expressions in variant record definitions }
         values : array of Tconstexprint;
         nestedvariant : pvariantrecdesc;
       end;

       ppvariantrecdesc = ^pvariantrecdesc;

       tvariantrecdesc = record
         variantselector : tsym;
         variantselectorderef : tderef;
         branches : array of tvariantrecbranch;
       end;

       trecorddef = class(tabstractrecorddef)
       public
          variantrecdesc : pvariantrecdesc;
          isunion       : boolean;
          constructor create(const n:string; p:TSymtable);virtual;
          constructor create_global_internal(const n: string; packrecords, recordalignmin: shortint); virtual;
          function add_field_by_def(const optionalname: TIDString; def: tdef): tsym;
          procedure add_fields_from_deflist(fieldtypes: tfplist);
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function  size:asizeint;override;
          function  alignment : shortint;override;
          function  padalignment: shortint;
          function  GetTypeName:string;override;
          { debug }
          function  needs_inittable : boolean;override;
          function  needs_separate_initrtti:boolean;override;
          function  has_non_trivial_init_child(check_parent:boolean):boolean;override;
       end;
       trecorddefclass = class of trecorddef;

       tobjectdef = class;

       { TImplementedInterface }

       TImplementedInterface = class
        private
         fIOffset      : longint;
         function GetIOffset: longint;
        public
         IntfDef      : tobjectdef;
         IntfDefDeref : tderef;
         IType        : tinterfaceentrytype;
         VtblImplIntf : TImplementedInterface;
         NameMappings : TFPHashList;
         ProcDefs     : TFPObjectList;
         ImplementsGetter :  tsym;
         ImplementsGetterDeref : tderef;
         ImplementsField : tsym;
         constructor create(aintf: tobjectdef);virtual;
         constructor create_deref(intfd,getterd:tderef);virtual;
         destructor  destroy; override;
         function  getcopy:TImplementedInterface;
         procedure buildderef;
         procedure deref;
         procedure AddMapping(const origname, newname: string);
         function  GetMapping(const origname: string):string;
         procedure AddImplProc(pd:tprocdef);
         function  IsImplMergePossible(MergingIntf:TImplementedInterface;out weight: longint): boolean;
         property  IOffset: longint read GetIOffset write fIOffset;
       end;
       timplementedinterfaceclass = class of timplementedinterface;

       { tvmtentry }
       tvmtentry = record
         procdef      : tprocdef;
         procdefderef : tderef;
         visibility   : tvisibility;
       end;
       pvmtentry = ^tvmtentry;

       { tobjectdef }

       tvmcallstatic = (vmcs_default, vmcs_yes, vmcs_no, vmcs_unreachable);
       pmvcallstaticinfo = ^tmvcallstaticinfo;
       tmvcallstaticinfo = array[0..1024*1024-1] of tvmcallstatic;
       tobjectdef = class(tabstractrecorddef)
       private
          fcurrent_dispid: longint;
       public
          childof        : tobjectdef;
          childofderef   : tderef;

          { for Object Pascal helpers }
          extendeddef   : tdef;
          extendeddefderef: tderef;
          helpertype : thelpertype;
          { for Objective-C: protocols and classes can have the same name there }
          objextname     : pshortstring;
          { to be able to have a variable vmt position }
          { and no vmt field for objects without virtuals }
          vmtentries     : TFPList;
          vmcallstaticinfo : pmvcallstaticinfo;
          vmt_field       : tsym;
          vmt_fieldderef  : tderef;
          iidguid        : pguid;
          iidstr         : pshortstring;
          { store implemented interfaces defs and name mappings }
          ImplementedInterfaces : TFPObjectList;
          { number of abstract methods (used by JVM target to determine whether
            or not the class should be marked as abstract: must be done if 1 or
            more abstract methods) }
          abstractcnt    : longint;
          writing_class_record_dbginfo,
          { a class of this type has been created in this module }
          created_in_current_module,
          { a loadvmtnode for this class has been created in this
            module, so if a classrefdef variable of this or a parent
            class is used somewhere to instantiate a class, then this
            class may be instantiated
          }
          maybe_created_in_current_module,
          { a "class of" this particular class has been created in
            this module
          }
          classref_created_in_current_module : boolean;
          objecttype     : tobjecttyp;
          constructor create(ot:tobjecttyp;const n:string;c:tobjectdef;doregister:boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor  destroy;override;
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function GetTypeName:string;override;
          procedure buildderef;override;
          procedure deref;override;
          procedure derefimpl;override;
          procedure copyvmtentries(objdef:tobjectdef);
          function  getparentdef:tdef;override;
          function  size : asizeint;override;
          function  alignment:shortint;override;
          function  vmtmethodoffset(index:longint):longint;
          function  members_need_inittable : boolean;
          { this should be called when this class implements an interface }
          procedure prepareguid;
          function  is_publishable : boolean;override;
          function  needs_inittable : boolean;override;
          function  needs_separate_initrtti : boolean;override;
          function  has_non_trivial_init_child(check_parent:boolean):boolean;override;
          function  rtti_mangledname(rt:trttitype):TSymStr;override;
          function  vmt_mangledname : TSymStr;
          function  vmt_def: trecorddef;
          procedure check_forwards; override;
          procedure insertvmt;
          function  vmt_offset: asizeint;
          procedure set_parent(c : tobjectdef);
          function find_destructor: tprocdef;
          function implements_any_interfaces: boolean;
          { dispinterface support }
          function get_next_dispid: longint;
          { enumerator support }
          function search_enumerator_get: tprocdef; override;
          function search_enumerator_move: tprocdef; override;
          function search_enumerator_current: tsym; override;
          { WPO }
          procedure register_created_object_type;override;
          procedure register_maybe_created_object_type;
          procedure register_created_classref_type;
          procedure register_vmt_call(index:longint);
          { ObjC }
          procedure finish_objc_data;
          function check_objc_types: boolean;
          { C++ }
          procedure finish_cpp_data;
       end;
       tobjectdefclass = class of tobjectdef;

       tclassrefdef = class(tabstractpointerdef)
          constructor create(def:tdef);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function getcopy:tstoreddef;override;
          function GetTypeName:string;override;
          function is_publishable : boolean;override;
          function rtti_mangledname(rt:trttitype):TSymStr;override;
          procedure register_created_object_type;override;
       end;
       tclassrefdefclass = class of tclassrefdef;

       tarraydef = class(tstoreddef)
          lowrange,
          highrange     : asizeint;
          rangedef      : tdef;
          rangedefderef : tderef;
          arrayoptions  : tarraydefoptions;
          symtable      : TSymtable;
       protected
          _elementdef      : tdef;
          _elementdefderef : tderef;
          procedure setelementdef(def:tdef);
          class function getreusable_intern(def: tdef; elems: asizeint; const options: tarraydefoptions): tarraydef;
       public
          function elesize : asizeint;
          function elepackedbitsize : asizeint;
          function elecount : asizeuint;
          constructor create_from_pointer(def: tpointerdef);virtual;
          constructor create(l, h: asizeint; def: tdef);virtual;
          constructor create_vector(l,h:asizeint; def:tdef);
          constructor create_openarray;virtual;
          class function getreusable(def: tdef; elems: asizeint): tarraydef;
          class function getreusable_vector(def: tdef; elems: asizeint): tarraydef;
          { same as above, but in case the def must never be freed after the
            current module has been compiled -- even if the def was not written
            to the ppu file (for defs in para locations, as we don't reset them
            so we don't have to recalculate them all the time) }
          class function getreusable_no_free(def: tdef; elems: asizeint): tarraydef;
          class function getreusable_no_free_vector(def: tdef; elems: asizeint): tarraydef;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy; override;
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
          function  getmangledparaname : TSymStr;override;
          procedure buildderef;override;
          procedure deref;override;
          function size : asizeint;override;
          function alignment : shortint;override;
          { returns the label of the range check string }
          function needs_inittable : boolean;override;
          function needs_separate_initrtti : boolean;override;
          property elementdef : tdef read _elementdef write setelementdef;
          function is_publishable : boolean;override;
          function is_hwvector: boolean;
       end;
       tarraydefclass = class of tarraydef;

       torddef = class(tstoreddef)
          low,high : TConstExprInt;
          ordtype  : tordtype;
          constructor create(t : tordtype;v,b : TConstExprInt; doregister: boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  is_publishable : boolean;override;
          function  GetTypeName:string;override;
          function alignment:shortint;override;
          procedure setsize;
          function  packedbitsize: asizeint; override;
          function getvardef : longint;override;
       end;
       torddefclass = class of torddef;

       tfloatdef = class(tstoreddef)
          floattype : tfloattype;
          constructor create(t: tfloattype; doregister: boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
          function  is_publishable : boolean;override;
          function alignment:shortint;override;
          function structalignment: shortint;override;
          procedure setsize;
          function  getvardef:longint;override;
       end;
       tfloatdefclass = class of tfloatdef;

       { tabstractprocdef }

       tprocnameoption = (pno_showhidden, pno_proctypeoption, pno_paranames,
         pno_ownername, pno_noclassmarker, pno_noleadingdollar,
         pno_mangledname, pno_noparams);
       tprocnameoptions = set of tprocnameoption;
       tproccopytyp = (pc_normal,
                       { everything except for hidden parameters }
                       pc_normal_no_hidden,
                       { always creates a top-level function, removes all
                         special parameters (self, vmt, parentfp, ...) }
                       pc_bareproc,
                       { creates a procvardef describing only the code pointer
                         of a method/netsted function/... }
                       pc_address_only
                       );

       tabstractprocdef = class(tstoreddef)
          { saves a definition to the return type }
          returndef       : tdef;
          returndefderef  : tderef;
          parast          : TSymtable;
          paras           : tparalist;
          proctypeoption  : tproctypeoption;
          proccalloption  : tproccalloption;
          procoptions     : tprocoptions;
          callerargareasize,
          calleeargareasize: pint;
{$ifdef m68k}
          exp_funcretloc : tregister;   { explicit funcretloc for AmigaOS }
{$endif}
          funcretloc : array[callerside..calleeside] of TCGPara;
          has_paraloc_info : tcallercallee; { paraloc info is available }
          { number of user visible parameters }
          maxparacount,
          minparacount    : byte;
          constructor create(dt:tdeftyp;level:byte;doregister:boolean);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          destructor destroy;override;
          procedure  ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderef;override;
          procedure deref;override;
          procedure calcparas;
          function mangledprocparanames(oldlen : longint) : string;
          function  typename_paras(pno: tprocnameoptions): ansistring;
          function  is_methodpointer:boolean;virtual;
          function  is_addressonly:boolean;virtual;
          function  no_self_node:boolean;
          { get either a copy as a procdef or procvardef }
          function  getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp; const paraprefix: string): tstoreddef; virtual;
          function  compatible_with_pointerdef_size(ptr: tpointerdef): boolean; virtual;
          procedure check_mark_as_nested;
          procedure init_paraloc_info(side: tcallercallee);
          procedure done_paraloc_info(side: tcallercallee);
          function stack_tainting_parameter(side: tcallercallee): boolean;
          function is_pushleftright: boolean;virtual;
          function address_type:tdef;virtual;
          { address type, generated for ofs() }
          function ofs_address_type:tdef;virtual;
          procedure declared_far;virtual;
          procedure declared_near;virtual;
          function generate_safecall_wrapper: boolean; virtual;
       private
          procedure count_para(p:TObject;arg:pointer);
          procedure insert_para(p:TObject;arg:pointer);
       end;

       tprocvardef = class(tabstractprocdef)
          constructor create(level:byte);virtual;
          { returns a procvardef that represents the address of a proc(var)def }
          class function getreusableprocaddr(def: tabstractprocdef): tprocvardef; virtual;
          { same as above, but in case the def must never be freed after the
            current module has been compiled -- even if the def was not written
            to the ppu file (for defs in para locations, as we don't reset them
            so we don't have to recalculate them all the time) }
          class function getreusableprocaddr_no_free(def: tabstractprocdef): tprocvardef;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetSymtable(t:tGetSymtable):TSymtable;override;
          function  size : asizeint;override;
          function  GetTypeName:string;override;
          function  is_publishable : boolean;override;
          function  is_methodpointer:boolean;override;
          function  is_addressonly:boolean;override;
          function  getmangledparaname:TSymStr;override;
          function getcopyas(newtyp: tdeftyp; copytyp: tproccopytyp; const paraprefix: string): tstoreddef; override;
       end;
       tprocvardefclass = class of tprocvardef;

       tmessageinf = record
         case integer of
           0 : (str : pshortstring);
           1 : (i : longint);
       end;

       tinlininginfo = record
          { node tree }
          code  : tnode;
          flags : tprocinfoflags;
       end;
       pinlininginfo = ^tinlininginfo;

{$ifdef oldregvars}
       { register variables }
       pregvarinfo = ^tregvarinfo;
       tregvarinfo = record
          regvars : array[1..maxvarregs] of tsym;
          regvars_para : array[1..maxvarregs] of boolean;
          regvars_refs : array[1..maxvarregs] of longint;

          fpuregvars : array[1..maxfpuvarregs] of tsym;
          fpuregvars_para : array[1..maxfpuvarregs] of boolean;
          fpuregvars_refs : array[1..maxfpuvarregs] of longint;
       end;
{$endif oldregvars}

       timplprocdefinfo = record
          resultname : pshortstring;
          parentfpstruct: tsym;
          parentfpstructptrtype: tdef;
          parentfpinitblock: tnode;
          procstarttai,
          procendtai   : tai;
          skpara: pointer;
          personality: tprocdef;
          forwarddef,
          interfacedef : boolean;
          hasforward  : boolean;
       end;
       pimplprocdefinfo = ^timplprocdefinfo;

       { tprocdef }

       tprocdef = class(tabstractprocdef)
       protected
{$ifdef symansistr}
         _mangledname : ansistring;
{$else symansistr}
         _mangledname : pshortstring;
{$endif}
         { information that is only required until the implementation of the
           procdef has been handled }
         implprocdefinfo : pimplprocdefinfo;

         function store_localst:boolean;

         function GetResultName: PShortString;
         procedure SetResultName(AValue: PShortString);
         function GetParentFPStruct: tsym;
         procedure SetParentFPStruct(AValue: tsym);
         function GetParentFPStructPtrType: tdef;
         procedure SetParentFPStructPtrType(AValue: tdef);
         function GetParentFPInitBlock: tnode;
         procedure SetParentFPInitBlock(AValue: tnode);
         function Getprocstarttai: tai;
         procedure Setprocstarttai(AValue: tai);
         function Getprocendtai: tai;
         procedure Setprocendtai(AValue: tai);
         function Getskpara: pointer;
         procedure Setskpara(AValue: pointer);
         function Getpersonality: tprocdef;
         procedure Setpersonality(AValue: tprocdef);
         function Getforwarddef: boolean;
         procedure Setforwarddef(AValue: boolean);
         function Getinterfacedef: boolean;
         procedure Setinterfacedef(AValue: boolean);virtual;
         function Gethasforward: boolean;
         procedure Sethasforward(AValue: boolean);
         function GetIsEmpty: boolean;
         procedure SetIsEmpty(AValue: boolean);
         function GetHasInliningInfo: boolean;
         procedure SetHasInliningInfo(AValue: boolean);
       public
          messageinf : tmessageinf;
          dispid : longint;
{$ifndef EXTDEBUG}
          { where is this function defined and what were the symbol
            flags, needed here because there
            is only one symbol for all overloaded functions
            EXTDEBUG has fileinfo in tdef (PFV) }
          fileinfo : tfileposinfo;
{$endif}
          symoptions : tsymoptions;
          deprecatedmsg : pshortstring;
          { generic support }
          genericdecltokenbuf : tdynamicarray;
          { symbol owning this definition }
          procsym : tsym;
          procsymderef : tderef;
          { alias names }
          aliasnames : TCmdStrList;
          { symtables }
          localst : TSymtable;
          funcretsym : tsym;
          funcretsymderef : tderef;
          struct : tabstractrecorddef;
          structderef : tderef;
          implprocoptions: timplprocoptions;
          { import info }
          import_dll,
          import_name : pshortstring;
          { info for inlining the subroutine, if this pointer is nil,
            the procedure can't be inlined }
          inlininginfo : pinlininginfo;
{$ifdef oldregvars}
          regvarinfo: pregvarinfo;
{$endif oldregvars}
          import_nr    : word;
          extnumber    : word;
          { set to a value different from tsk_none in case this procdef is for
            a routine that has to be internally generated by the compiler }
          synthetickind: tsynthetickind;
          visibility   : tvisibility;
          constructor create(level:byte;doregister:boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor  destroy;override;
          procedure   freeimplprocdefinfo;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure buildderefimpl;override;
          procedure deref;override;
          procedure derefimpl;override;
          function  GetSymtable(t:tGetSymtable):TSymtable;override;
          { warnings:
              * the symtablestack top has to be the symtable to which the copy
                should be added
              * getcopy does not create a finished/ready-to-use procdef; it
                needs to be finalised afterwards by calling
                symcreat.finish_copied_procdef() afterwards
          }
          function  getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp; const paraprefix: string): tstoreddef; override;
          function  getcopy: tstoreddef; override;
          function  GetTypeName : string;override;
          function  mangledname : TSymStr; virtual;
          procedure setmangledname(const s : TSymStr);
          function  needsglobalasmsym: boolean;
          procedure setcompilerprocname;
          function  fullprocname(showhidden:boolean):string;
          function  customprocname(pno: tprocnameoptions):ansistring;
          function  defaultmangledname: TSymStr;
          function  cplusplusmangledname : TSymStr;
          function  objcmangledname : TSymStr;
          function  is_methodpointer:boolean;override;
          function  is_addressonly:boolean;override;
          procedure make_external;
          procedure init_genericdecl;

          function get_funcretsym_info(out ressym: tsym; out resdef: tdef): boolean; virtual;
          function get_safecall_funcretsym_info(out ressym: tsym; out resdef: tdef): boolean; virtual;

          { returns whether the mangled name or any of its aliases is equal to
            s }
          function  has_alias_name(const s: TSymStr):boolean;

          { aliases to fields only required when a function is implemented in
            the current unit }
          property resultname: PShortString read GetResultName write SetResultName;
          { temporary reference to structure containing copies of all local
            variables and parameters accessed by nested routines; reference to
            this structure is passed as "parent frame pointer" on targets that
            lack this concept (at least JVM and LLVM); no need to save to/
            restore from ppu, since nested routines are always in the same
            unit }
          property parentfpstruct: tsym read GetParentFPStruct write SetParentFPStruct;
          { pointer to parentfpstruct's type (not yet valid during parsing, so
            cannot be used for $parentfp parameter) (no need to save to ppu) }
          property parentfpstructptrtype: tdef read GetParentFPStructPtrType write SetParentFPStructPtrType;
          { code to copy the parameters accessed from nested routines into the
            parentfpstruct (no need to save to ppu) }
          property parentfpinitblock: tnode read GetParentFPInitBlock write SetParentFPInitBlock;
          { First/last assembler symbol/instruction in aasmoutput list.
            Note: initialised after compiling the code for the procdef, but
              not saved to/restored from ppu. Used when inserting debug info }
          property procstarttai: tai read Getprocstarttai write Setprocstarttai;
          property procendtai: tai read Getprocendtai write Setprocendtai;
          { optional parameter for the synthetic routine generation logic }
          property skpara: pointer read Getskpara write Setskpara;
          { ABI-conformant exception handling personality function }
          property personality: tprocdef read Getpersonality write Setpersonality;
          { true, if the procedure is only declared
            (forward procedure) }
          property forwarddef: boolean read Getforwarddef write Setforwarddef;
          { true if the procedure is declared in the interface }
          property interfacedef: boolean read Getinterfacedef write Setinterfacedef;
          { true if the procedure has a forward declaration }
          property hasforward: boolean read Gethasforward write Sethasforward;
          { true if the routine's body is empty }
          property isempty: boolean read GetIsEmpty write SetIsEmpty;
          { true if all information required to inline this routine is available }
          property has_inlininginfo: boolean read GetHasInliningInfo write SetHasInliningInfo;
       end;
       tprocdefclass = class of tprocdef;

       { single linked list of overloaded procs }
       pprocdeflist = ^tprocdeflist;
       tprocdeflist = record
         def  : tprocdef;
         defderef : tderef;
         next : pprocdeflist;
       end;

       tstringdef = class(tstoreddef)
          encoding   : tstringencoding;
          stringtype : tstringtype;
          len        : asizeint;
          constructor createshort(l: byte; doregister: boolean);virtual;
          constructor loadshort(ppufile:tcompilerppufile);
          constructor createlong(l: asizeint; doregister: boolean);virtual;
          constructor loadlong(ppufile:tcompilerppufile);
          constructor createansi(aencoding: tstringencoding; doregister: boolean);virtual;
          constructor loadansi(ppufile:tcompilerppufile);
          constructor createwide(doregister: boolean);virtual;
          constructor loadwide(ppufile:tcompilerppufile);
          constructor createunicode(doregister: boolean);virtual;
          constructor loadunicode(ppufile:tcompilerppufile);virtual;
          function getcopy : tstoreddef;override;
          function  stringtypname:string;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function  GetTypeName:string;override;
          function  getmangledparaname:TSymStr;override;
          function  is_publishable : boolean;override;
          function  size:asizeint;override;
          function alignment : shortint;override;
          function  needs_inittable : boolean;override;
          function  getvardef:longint;override;
       end;
       tstringdefclass = class of tstringdef;

       { tenumdef }

       tenumdef = class(tstoreddef)
          minval,
          maxval    : asizeint;
          basedef   : tenumdef;
          basedefderef : tderef;
          symtable  : TSymtable;
          has_jumps : boolean;
          constructor create;virtual;
          constructor create_subrange(_basedef:tenumdef;_min,_max:asizeint);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function  GetTypeName:string;override;
          function  is_publishable : boolean;override;
          procedure calcsavesize(packenum: shortint);
          function  packedbitsize: asizeint; override;
          procedure setmax(_max:asizeint);
          procedure setmin(_min:asizeint);
          function  min:asizeint;
          function  max:asizeint;
          function  getfirstsym:tsym;
          function  int2enumsym(l: asizeint): tsym;
          { returns basedef if assigned, otherwise self }
          function getbasedef: tenumdef;
       end;
       tenumdefclass = class of tenumdef;

       tsetdef = class(tstoreddef)
          elementdef : tdef;
          elementdefderef : tderef;
          setbase,
          setmax   : asizeint;
          constructor create(def: tdef; low, high: asizeint; doregister: boolean);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function  GetTypeName:string;override;
          function  is_publishable : boolean;override;
          function alignment: shortint; override;
       end;
       tsetdefclass = class of tsetdef;


       tgenericdummyentry = class
         dummysym : tsym;
         resolvedsym : tsym;
       end;


       tdefawaresymtablestack = class(TSymtablestack)
       private
         procedure add_helpers_and_generics(st:tsymtable;addgenerics:boolean);
         procedure remove_helpers_and_generics(st:tsymtable);inline;
         procedure remove_helpers(st:tsymtable);
         procedure remove_generics(st:tsymtable);
         procedure pushcommon(st:tsymtable);inline;
       public
         procedure push(st: TSymtable); override;
         procedure pushafter(st,afterst:TSymtable); override;
         procedure pop(st: TSymtable); override;
       end;


    var
       current_structdef: tabstractrecorddef; { used for private functions check !! }
       current_genericdef: tstoreddef;        { used to reject declaration of generic class inside generic class }
       current_specializedef: tstoreddef;     { used to implement usage of generic class in itself }

       cfiledef: tfiledefclass;
       cvariantdef: tvariantdefclass;
       cformaldef: tformaldefclass;
       cforwarddef: tforwarddefclass;
       cundefineddef: tundefineddefclass;
       cerrordef: terrordefclass;
       cpointerdef: tpointerdefclass;
       crecorddef: trecorddefclass;
       cimplementedinterface: timplementedinterfaceclass;
       cobjectdef: tobjectdefclass;
       cclassrefdef: tclassrefdefclass;
       carraydef: tarraydefclass;
       corddef: torddefclass;
       cfloatdef: tfloatdefclass;
       cprocvardef: tprocvardefclass;
       cprocdef: tprocdefclass;
       cstringdef: tstringdefclass;
       cenumdef: tenumdefclass;
       csetdef: tsetdefclass;


    { default types }
       generrordef,               { error in definition }
       voidpointertype,           { pointer for Void-pointeddef }
       charpointertype,           { pointer for Char-pointeddef }
       widecharpointertype,       { pointer for WideChar-pointeddef }
       voidcodepointertype,       { pointer to code; corresponds to System.CodePointer }
       voidstackpointertype,      { the pointer type used for accessing parameters and local vars on the stack }
       parentfpvoidpointertype,   { void pointer with the size of the hidden parentfp parameter, passed to nested functions }
{$ifdef x86}
       voidnearpointertype,
       voidnearcspointertype,
       voidneardspointertype,
       voidnearsspointertype,
       voidnearespointertype,
       voidnearfspointertype,
       voidneargspointertype,
  {$ifdef i8086}
       voidfarpointertype,
       voidhugepointertype,
       charnearpointertype,
       charfarpointertype,
       charhugepointertype,
       bytefarpointertype,        { used for Mem[] }
       wordfarpointertype,        { used for MemW[] }
       longintfarpointertype,     { used for MemL[] }
  {$endif i8086}
{$endif x86}
       cundefinedtype,
       cformaltype,               { unique formal definition }
       ctypedformaltype,          { unique typed formal definition }
       voidtype,                  { Void (procedure) }
       cansichartype,             { Char }
       cwidechartype,             { WideChar }
       pasbool1type,              { boolean type }
       pasbool8type,
       pasbool16type,
       pasbool32type,
       pasbool64type,
       bool8type,
       bool16type,
       bool32type,
       bool64type,                { implement me }
{$ifdef llvm}
       llvmbool1type,             { LLVM i1 type }
{$endif llvm}
       u8inttype,                 { 8-Bit unsigned integer }
       s8inttype,                 { 8-Bit signed integer }
       u16inttype,                { 16-Bit unsigned integer }
       s16inttype,                { 16-Bit signed integer }
       u24inttype,                { 24-Bit unsigned integer }
       s24inttype,                { 24-Bit signed integer }
       u32inttype,                { 32-Bit unsigned integer }
       s32inttype,                { 32-Bit signed integer }
       u40inttype,                { 40-Bit unsigned integer }
       s40inttype,                { 40-Bit signed integer }
       u48inttype,                { 48-Bit unsigned integer }
       s48inttype,                { 48-Bit signed integer }
       u56inttype,                { 56-Bit unsigned integer }
       s56inttype,                { 56-Bit signed integer }
       u64inttype,                { 64-bit unsigned integer }
       s64inttype,                { 64-bit signed integer }
       u128inttype,               { 128-bit unsigned integer }
       s128inttype,               { 128-bit signed integer }
       s32floattype,              { 32 bit floating point number }
       s64floattype,              { 64 bit floating point number }
       s80floattype,              { 80 bit floating point number }
       sc80floattype,             { 80 bit floating point number but stored like in C }
       s64currencytype,           { pointer to a currency type }
       cshortstringtype,          { pointer to type of short string const   }
       clongstringtype,           { pointer to type of long string const   }
       cansistringtype,           { pointer to type of ansi string const  }
       cwidestringtype,           { pointer to type of wide string const  }
       cunicodestringtype,
       openshortstringtype,       { pointer to type of an open shortstring,
                                    needed for readln() }
       openchararraytype,         { pointer to type of an open array of char,
                                    needed for readln() }
       cfiletype,                 { get the same definition for all file }
                                  { used for stabs }
       methodpointertype,         { typecasting of methodpointers to extract self }
       nestedprocpointertype,     { typecasting of nestedprocpointers to extract parentfp }
       hresultdef,
       typekindtype,              { def of TTypeKind for correct handling of GetTypeKind parameters }
       { we use only one variant def for every variant class }
       cvarianttype,
       colevarianttype,
       { default integer type, normally s32inttype on 32 bit systems and s64bittype on 64 bit systems }
       sinttype,
       uinttype,
       { integer types corresponding to OS_SINT/OS_INT }
       ossinttype,
       osuinttype,
       { integer types corresponding to the ALU size, sizeof(aint) and the ALUSInt/ALUUInt types in the system unit }
       alusinttype,
       aluuinttype,
       { integer types corresponding to SizeInt and SizeUInt for the target platform }
       sizeuinttype,
       sizesinttype,
       { unsigned and signed ord type with the same size as a pointer }
       ptruinttype,
       ptrsinttype,
       { unsigned and signed ord type with the same size as a codepointer }
       codeptruinttype,
       codeptrsinttype,
       { several types to simulate more or less C++ objects for GDB }
       vmttype,
       vmtarraytype,
       pvmttype      : tdef;     { type of classrefs, used for stabs }

       { pointer to the anchestor of all classes }
       class_tobject : tobjectdef;
       { pointer to the base type for custom attributes }
       class_tcustomattribute : tobjectdef;
       { pointer to the ancestor of all COM interfaces }
       interface_iunknown : tobjectdef;
       { pointer to the ancestor of all dispinterfaces }
       interface_idispatch : tobjectdef;
       { pointer to the TGUID type
         of all interfaces         }
       rec_tguid : trecorddef;

       { jump buffer type, used by setjmp }
       rec_jmp_buf : trecorddef;

       { system.texceptaddr type, used by fpc_pushexceptaddr }
       rec_exceptaddr: trecorddef;

       { Objective-C base types }
       objc_metaclasstype,
       objc_superclasstype,
       objc_idtype,
       objc_seltype              : tpointerdef;
       objc_objecttype           : trecorddef;
       { base type of @protocol(protocolname) Objective-C statements }
       objc_protocoltype         : tobjectdef;
       { helper types for for-in "fast enumeration" support in Objective-C 2.0 }
       objc_fastenumeration      : tobjectdef;
       objc_fastenumerationstate : trecorddef;

{$ifdef llvm}
       { llvm types }
       { a unique def to identify any kind of metadata }
       llvm_metadatatype         : tdef;
{$endif llvm}

       { Java base types }
       { java.lang.Object }
       java_jlobject             : tobjectdef;
       { java.lang.Throwable }
       java_jlthrowable          : tobjectdef;
       { FPC base type for records }
       java_fpcbaserecordtype    : tobjectdef;
       { java.lang.String }
       java_jlstring             : tobjectdef;
       { java.lang.Enum }
       java_jlenum               : tobjectdef;
       { java.util.EnumSet }
       java_juenumset            : tobjectdef;
       { java.util.BitSet }
       java_jubitset             : tobjectdef;
       { FPC java implementation of ansistrings }
       java_ansistring           : tobjectdef;
       { FPC java implementation of shortstrings }
       java_shortstring          : tobjectdef;
       { FPC java procvar base class }
       java_procvarbase          : tobjectdef;


    function make_mangledname(const typeprefix:TSymStr;st:TSymtable;const suffix:TSymStr):TSymStr;
    function make_dllmangledname(const dllname,importname:TSymStr;
                                 import_nr : word; pco : tproccalloption):TSymStr;

    { should be in the types unit, but the types unit uses the node stuff :( }
    function is_interfacecom(def: tdef): boolean;
    function is_interfacecom_or_dispinterface(def: tdef): boolean;
    function is_any_interface_kind(def: tdef): boolean;
    function is_interfacecorba(def: tdef): boolean;
    function is_interface(def: tdef): boolean;
    function is_dispinterface(def: tdef): boolean;
    function is_object(def: tdef): boolean;
    function is_class(def: tdef): boolean;
    function is_cppclass(def: tdef): boolean;
    function is_objectpascal_helper(def: tdef): boolean;
    function is_objcclass(def: tdef): boolean;
    function is_objcclassref(def: tdef): boolean;
    function is_objcprotocol(def: tdef): boolean;
    function is_objccategory(def: tdef): boolean;
    function is_objc_class_or_protocol(def: tdef): boolean;
    function is_objc_protocol_or_category(def: tdef): boolean;
    function is_classhelper(def: tdef): boolean;
    function is_class_or_interface(def: tdef): boolean;
    function is_class_or_interface_or_objc(def: tdef): boolean;
    function is_class_or_interface_or_objc_or_java(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface_or_objc_or_java(def: tdef): boolean;
    function is_class_or_interface_or_object(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface(def: tdef): boolean;
    function is_implicit_pointer_object_type(def: tdef): boolean;
    { returns true, if def is a type which is an implicit pointer to an array (dyn. array or dyn. string) }
    function is_implicit_array_pointer(def: tdef): boolean;
    function is_class_or_object(def: tdef): boolean;
    function is_record(def: tdef): boolean;

    function is_javaclass(def: tdef): boolean;
    function is_javaclassref(def: tdef): boolean;
    function is_javainterface(def: tdef): boolean;
    function is_java_class_or_interface(def: tdef): boolean;

    procedure loadobjctypes;
    procedure maybeloadcocoatypes;

    function use_vectorfpu(def : tdef) : boolean;

    function getansistringcodepage:tstringencoding; inline;
    function getansistringdef:tstringdef;
    function getparaencoding(def:tdef):tstringencoding; inline;

    function get_threadvar_record(def: tdef; out index_field, non_mt_data_field: tsym): trecorddef;
    function get_recorddef(prefix:tinternaltypeprefix;const fields:array of tdef; packrecords:shortint): trecorddef;
    { get a table def of the form
        record
          count: countdef;
          elements: array[0..count-1] of elementdef
        end;
      Returns both the outer record and the inner arraydef
    }
    procedure get_tabledef(prefix:tinternaltypeprefix;countdef,elementdef:tdef;count:longint;packrecords:shortint;out recdef:trecorddef;out arrdef:tarraydef);

implementation

    uses
      SysUtils,
      cutils,
      { global }
      verbose,
      { target }
      systems,paramgr,
      { symtable }
      symsym,symtable,defutil,objcdef,
      { parser }
      pgenutil,
      { module }
      fmodule,ppu,
      { other }
      aasmbase,
      gendef,
      fpccrc,
      entfile
      ;

{****************************************************************************
                                  Helpers
****************************************************************************}

    function getansistringcodepage:tstringencoding; inline;
      begin
        if ([cs_explicit_codepage,cs_system_codepage]*current_settings.moduleswitches)<>[] then
          result:=current_settings.sourcecodepage
        else
          result:=0;
      end;

    function getansistringdef:tstringdef;
      var
        symtable:tsymtable;
        oldstack : tsymtablestack;
      begin
        { if a codepage is explicitly defined in this mudule we need to return
          a replacement for ansistring def }
        if ([cs_explicit_codepage,cs_system_codepage]*current_settings.moduleswitches)<>[] then
          begin
            if not assigned(current_module) then
              internalerror(2011101301);
            { codepage can be redeclared only once per unit so we don't need a list of
              redefined ansistring but only one pointer }
            if not assigned(current_module.ansistrdef) then
              begin
                { if we did not create it yet we need to do this now }
                if current_module.in_interface then
                  symtable:=current_module.globalsymtable
                else
                  symtable:=current_module.localsymtable;
                { create a temporary stack as it's not good (TM) to mess around
                  with the order if the unit contains generics or helpers; don't
                  use a def aware symtablestack though }
                oldstack:=symtablestack;
                symtablestack:=tsymtablestack.create;
                symtablestack.push(symtable);
                current_module.ansistrdef:=cstringdef.createansi(current_settings.sourcecodepage,true);
                symtablestack.pop(symtable);
                symtablestack.free;
                symtablestack:=oldstack;
              end;
            result:=tstringdef(current_module.ansistrdef);
          end
        else
          result:=tstringdef(cansistringtype);
      end;

    function getparaencoding(def:tdef):tstringencoding; inline;
      begin
        { don't pass CP_NONE encoding to internal functions
          they expect 0 encoding instead
          exception: result of string concatenation, because if you pass the
          result of a string concatenation to a rawbytestring, the result of
          that concatenation shouldn't be converted to defaultsystemcodepage
          if all strings have the same type }
        result:=tstringdef(def).encoding;
        if result=globals.CP_NONE then
          result:=0
      end;


    function get_threadvar_record(def: tdef; out index_field, non_mt_data_field: tsym): trecorddef;
      var
        typ: ttypesym;
        name: string;
      begin
        name:=internaltypeprefixName[itp_threadvar_record]+def.unique_id_str;
        typ:=try_search_current_module_type(name);
        if assigned(typ) then
          begin
            result:=trecorddef(ttypesym(typ).typedef);
            index_field:=tsym(result.symtable.symlist[0]);
            non_mt_data_field:=tsym(result.symtable.symlist[1]);
            exit;
          end;
        { set recordalinmin to sizeof(pint), so the second field gets put at
          offset = sizeof(pint) as expected }
        result:=crecorddef.create_global_internal(
          name,sizeof(pint),sizeof(pint));
{$ifdef cpu16bitaddr}
        index_field:=result.add_field_by_def('',u16inttype);
{$else cpu16bitaddr}
        index_field:=result.add_field_by_def('',u32inttype);
{$endif cpu16bitaddr}
        non_mt_data_field:=result.add_field_by_def('',def);
        { no need to add alignment padding, we won't create arrays of these }
      end;


    function get_recorddef(prefix:tinternaltypeprefix; const fields:array of tdef; packrecords:shortint): trecorddef;
      var
        fieldlist: tfplist;
        srsym: tsym;
        srsymtable: tsymtable;
        i: longint;
        name : TIDString;
      begin
        name:=copy(internaltypeprefixName[prefix],2,length(internaltypeprefixName[prefix]));
        if searchsym_type(name,srsym,srsymtable) then
          begin
            result:=trecorddef(ttypesym(srsym).typedef);
            exit
          end;
        { also always search in the current module (symtables are popped for
          RTTI related code already) }
        if searchsym_in_module(pointer(current_module),name,srsym,srsymtable) then
          begin
            result:=trecorddef(ttypesym(srsym).typedef);
            exit;
          end;
        fieldlist:=tfplist.create;
        for i:=low(fields) to high(fields) do
          fieldlist.add(fields[i]);
        result:=crecorddef.create_global_internal(internaltypeprefixName[prefix],packrecords,
          targetinfos[target_info.system]^.alignment.recordalignmin);
        result.add_fields_from_deflist(fieldlist);
        fieldlist.free;
      end;


    procedure get_tabledef(prefix:tinternaltypeprefix;countdef,elementdef:tdef;count:longint;packrecords:shortint;out recdef:trecorddef;out arrdef:tarraydef);
      var
        fields: tfplist;
        name: TIDString;
        srsym: tsym;
        srsymtable: tsymtable;
      begin
        { already created a message string table with this number of elements
          in this unit -> reuse the def }
        name:=internaltypeprefixName[prefix]+tostr(count);
        if searchsym_type(copy(name,2,length(name)),srsym,srsymtable) then
          begin
            recdef:=trecorddef(ttypesym(srsym).typedef);
            arrdef:=tarraydef(trecordsymtable(recdef.symtable).findfieldbyoffset(countdef.size).vardef);
            exit
          end;
        { also always search in the current module (symtables are popped for
          RTTI related code already) }
        if searchsym_in_module(pointer(current_module),copy(name,2,length(name)),srsym,srsymtable) then
          begin
            recdef:=trecorddef(ttypesym(srsym).typedef);
            arrdef:=tarraydef(trecordsymtable(recdef.symtable).findfieldbyoffset(countdef.size).vardef);
            exit;
          end;
        recdef:=crecorddef.create_global_internal(name,packrecords,
          targetinfos[target_info.system]^.alignment.recordalignmin);
        fields:=tfplist.create;
        fields.add(countdef);
        if count>0 then
          begin
            arrdef:=carraydef.create(0,count-1,sizeuinttype);
            arrdef.elementdef:=elementdef;
            fields.add(arrdef);
          end
        else
          arrdef:=nil;
        recdef.add_fields_from_deflist(fields);
        fields.free;
      end;

    function make_mangledname(const typeprefix:TSymStr;st:TSymtable;const suffix:TSymStr):TSymStr;
      var
        s,
        prefix : TSymStr;
        crc : dword;
      begin
        prefix:='';
        if not assigned(st) then
         internalerror(200204212);
        { sub procedures }
        while (st.symtabletype in [localsymtable,parasymtable]) do
         begin
           if st.defowner.typ<>procdef then
            internalerror(200204173);
           { Add the full mangledname of procedure to prevent
             conflicts with 2 overloads having both a nested procedure
             with the same name, see tb0314 (PFV) }
           s:=tprocdef(st.defowner).procsym.name;
           s:=s+tprocdef(st.defowner).mangledprocparanames(Length(s));
           if prefix<>'' then
             prefix:=s+'_'+prefix
           else
             prefix:=s;
           if length(prefix)>100 then
             begin
               crc:=0;
               crc:=UpdateCrc32(crc,prefix[1],length(prefix));
               prefix:='$CRC'+hexstr(crc,8);
             end;
           st:=st.defowner.owner;
         end;
        { object/classes symtable, nested type definitions in classes require the while loop }
        while st.symtabletype in [ObjectSymtable,recordsymtable] do
         begin
           if not (st.defowner.typ in [objectdef,recorddef]) then
            internalerror(200204174);
           prefix:=tabstractrecorddef(st.defowner).objname^+'_$_'+prefix;
           st:=st.defowner.owner;
         end;
        { symtable must now be static or global }
        if not(st.symtabletype in [staticsymtable,globalsymtable]) then
          internalerror(200204175);

        { The mangled name is made out of at most 4 parts:
         1) Optional typeprefix given as first parameter
            with '_$' appended if not empty
         2) Unit name or 'P$'+program name (never empty)
         3) optional prefix variable that contains a unique
            name for the local symbol table (prepended with '$_$'
            if not empty)
         4) suffix as given as third parameter,
            also optional (i.e. can be empty)
            prepended by '_$$_' if not empty }
        result:='';
        if typeprefix<>'' then
          result:=result+typeprefix+'_$';
        { Add P$ for program, which can have the same name as
          a unit }
        if (TSymtable(main_module.localsymtable)=st) and
           (not main_module.is_unit) then
          result:=result+'P$'+st.name^
        else
          result:=result+st.name^;
        if prefix<>'' then
          result:=result+'$_$'+prefix;
        if suffix<>'' then
          result:=result+'_$$_'+suffix;
        { the Darwin assembler assumes that all symbols starting with 'L' are local }
        { Further, the Mac OS X 10.5 linker does not consider symbols which do not  }
        { start with '_' as regular symbols (it does not generate N_GSYM entries    }
        { those in the debug map, leading to troubles with dsymutil). So always     }
        { add an underscore on darwin.                                              }
        if (target_info.system in systems_darwin) then
          result := '_' + result;
      end;

    function make_dllmangledname(const dllname,importname:TSymStr;import_nr : word; pco : tproccalloption):TSymStr;
       var
         crc : cardinal;
         i : longint;
         use_crc : boolean;
         dllprefix : TSymStr;
      begin
        if (target_info.system in (systems_all_windows + systems_nativent +
                           [system_i386_emx, system_i386_os2]))
            and (dllname <> '') then
          begin
            dllprefix:=lower(ExtractFileName(dllname));
            { Remove .dll suffix if present }
            if copy(dllprefix,length(dllprefix)-3,length(dllprefix))='.dll' then
              dllprefix:=copy(dllprefix,1,length(dllprefix)-4);
            use_crc:=false;
            for i:=1 to length(dllprefix) do
              if not (dllprefix[i] in ['a'..'z','A'..'Z','_','0'..'9']) then
                begin
                  use_crc:=true;
                  break;
                end;
            if use_crc then
              begin
                crc:=0;
                crc:=UpdateCrc32(crc,dllprefix[1],length(dllprefix));
                dllprefix:='_$dll$crc$'+hexstr(crc,8)+'$';
              end
            else
              dllprefix:='_$dll$'+dllprefix+'$';

            if importname<>'' then
              result:=dllprefix+importname
            else
              result:=dllprefix+'_index_'+tostr(import_nr);
            { Replace ? and @ in import name, since GNU AS does not allow these characters in symbol names. }
            { This allows to import VC++ mangled names from DLLs. }
            { Do not perform replacement, if external symbol is not imported from DLL. }
            if (dllname<>'') then
              begin
                Replace(result,'?','__q$$');
    {$ifdef arm}
                { @ symbol is not allowed in ARM assembler only }
                Replace(result,'@','__a$$');
    {$endif arm}
             end;
          end
        else
          begin
            if importname<>'' then
             begin
               if not(pco in [pocall_cdecl,pocall_cppdecl]) then
                 result:=importname
               else
                 result:=target_info.Cprefix+importname;
             end
            else
              result:='_index_'+tostr(import_nr);
          end;

      end;

{****************************************************************************
           TDEFAWARESYMTABLESTACK
           (symtablestack descendant that does some special actions on
           the pushed/popped symtables)
****************************************************************************}

    procedure tdefawaresymtablestack.add_helpers_and_generics(st:tsymtable;addgenerics:boolean);
      var
        i: integer;
        s: string;
        list: TFPObjectList;
        def: tdef;
        sym : tsym;
      begin
        { search the symtable from first to last; the helper to use will be the
          last one in the list }
        for i:=0 to st.symlist.count-1 do
          begin
            sym:=tsym(st.symlist[i]);
            if not (sym.typ in [typesym,procsym]) then
              continue;
            if sym.typ=typesym then
              def:=ttypesym(st.SymList[i]).typedef
            else
              def:=nil;
            if is_objectpascal_helper(def) then
              begin
                s:=generate_objectpascal_helper_key(tobjectdef(def).extendeddef);
                Message1(sym_d_adding_helper_for,s);
                list:=TFPObjectList(current_module.extendeddefs.Find(s));
                if not assigned(list) then
                  begin
                    list:=TFPObjectList.Create(false);
                    current_module.extendeddefs.Add(s,list);
                  end;
                list.Add(def);
              end
            else
              begin
                if addgenerics then
                  add_generic_dummysym(sym);
                { add nested helpers as well }
                if assigned(def) and
                    (def.typ in [recorddef,objectdef]) and
                    (sto_has_helper in tabstractrecorddef(def).symtable.tableoptions) then
                  add_helpers_and_generics(tabstractrecorddef(def).symtable,false);
              end;
          end;
      end;


    procedure tdefawaresymtablestack.remove_helpers_and_generics(st:tsymtable);
      begin
        if sto_has_helper in st.tableoptions then
          remove_helpers(st);
        if sto_has_generic in st.tableoptions then
          remove_generics(st);
      end;


    procedure tdefawaresymtablestack.remove_helpers(st:TSymtable);
      var
        i, j: integer;
        tmpst: TSymtable;
        list: TFPObjectList;
      begin
        for i:=current_module.extendeddefs.count-1 downto 0 do
          begin
            list:=TFPObjectList(current_module.extendeddefs[i]);
            for j:=list.count-1 downto 0 do
              begin
                if not (list[j] is tobjectdef) then
                  Internalerror(2011031501);
                tmpst:=tobjectdef(list[j]).owner;
                repeat
                  if tmpst=st then
                    begin
                      list.delete(j);
                      break;
                    end
                  else
                    begin
                      if assigned(tmpst.defowner) then
                        tmpst:=tmpst.defowner.owner
                      else
                        tmpst:=nil;
                    end;
                until not assigned(tmpst) or (tmpst.symtabletype in [globalsymtable,staticsymtable]);
              end;
            if list.count=0 then
              current_module.extendeddefs.delete(i);
          end;
      end;


    procedure tdefawaresymtablestack.remove_generics(st:tsymtable);
      var
        i,j : longint;
        entry : tgenericdummyentry;
        list : tfpobjectlist;
      begin
        for i:=current_module.genericdummysyms.count-1 downto 0 do
          begin
            list:=tfpobjectlist(current_module.genericdummysyms[i]);
            if not assigned(list) then
              continue;
            for j:=list.count-1 downto 0 do
              begin
                entry:=tgenericdummyentry(list[j]);
                if entry.dummysym.owner=st then
                  list.delete(j);
              end;
            if list.count=0 then
              current_module.genericdummysyms.delete(i);
          end;
      end;


    procedure tdefawaresymtablestack.pushcommon(st:tsymtable);
      begin
        if (sto_has_generic in st.tableoptions) or
            (
              (st.symtabletype in [globalsymtable,staticsymtable]) and
              (sto_has_helper in st.tableoptions)
            ) then
          { nested helpers will be added as well }
          add_helpers_and_generics(st,true);
      end;

    procedure tdefawaresymtablestack.push(st: TSymtable);
      begin
        pushcommon(st);
        inherited push(st);
      end;

    procedure tdefawaresymtablestack.pushafter(st,afterst:TSymtable);
      begin
        pushcommon(st);
        inherited pushafter(st,afterst);
      end;

    procedure tdefawaresymtablestack.pop(st: TSymtable);
      begin
        inherited pop(st);
        if (sto_has_generic in st.tableoptions) or
            (
              (st.symtabletype in [globalsymtable,staticsymtable]) and
              (sto_has_helper in st.tableoptions)
            ) then
          { nested helpers will be removed as well }
          remove_helpers_and_generics(st);
      end;


{****************************************************************************
                     TDEF (base class for definitions)
****************************************************************************}

    constructor tgenericconstraintdata.create;
      begin
        interfaces:=tfpobjectlist.create(false);
        interfacesderef:=tfplist.create;
      end;


    destructor tgenericconstraintdata.destroy;
      var
        i : longint;
      begin
        for i:=0 to interfacesderef.count-1 do
          dispose(pderef(interfacesderef[i]));
        interfacesderef.free;
        interfaces.free;
        inherited destroy;
      end;

    procedure tgenericconstraintdata.ppuload(ppufile: tcompilerppufile);
      var
        cnt,i : longint;
        intfderef : pderef;
      begin
        ppufile.getset(tppuset1(flags));
        cnt:=ppufile.getlongint;
        for i:=0 to cnt-1 do
          begin
            new(intfderef);
            ppufile.getderef(intfderef^);
            interfacesderef.add(intfderef);
          end;
      end;


    procedure tgenericconstraintdata.ppuwrite(ppufile: tcompilerppufile);
      var
        i : longint;
      begin
        ppufile.putset(tppuset1(flags));
        ppufile.putlongint(interfacesderef.count);
        for i:=0 to interfacesderef.count-1 do
          ppufile.putderef(pderef(interfacesderef[i])^);
      end;

    procedure tgenericconstraintdata.buildderef;
      var
        intfderef : pderef;
        i : longint;
      begin
        for i:=0 to interfaces.count-1 do
          begin
            new(intfderef);
            intfderef^.build(tobjectdef(interfaces[i]));
            interfacesderef.add(intfderef);
          end;
      end;

    procedure tgenericconstraintdata.deref;
      var
        i : longint;
      begin
        for i:=0 to interfacesderef.count-1 do
          interfaces.add(pderef(interfacesderef[i])^.resolve);
      end;


    procedure tstoreddef.writeentry(ppufile: tcompilerppufile; ibnr: byte);
      begin
        ppuwrite_platform(ppufile);
        ppufile.writeentry(ibnr);
      end;


    procedure tstoreddef.ppuwrite_platform(ppufile: tcompilerppufile);
      begin
        { by default: do nothing }
      end;


    procedure tstoreddef.ppuload_platform(ppufile: tcompilerppufile);
      begin
        { by default: do nothing }
      end;


    class procedure tstoreddef.setup_reusable_def(origdef, newdef: tdef; res: PHashSetItem; oldsymtablestack: tsymtablestack);
      var
        reusablesymtab: tsymtable;
      begin
        { must not yet belong to a symtable }
        if assigned(newdef.owner) then
          internalerror(2015111503);
        reusablesymtab:=origdef.getreusablesymtab;
        res^.Data:=newdef;
        reusablesymtab.insertdef(newdef);
        symtablestack:=oldsymtablestack;
      end;


    constructor tstoreddef.create(dt:tdeftyp;doregister:boolean);
      begin
         inherited create(dt);
         savesize := 0;
{$ifdef EXTDEBUG}
         fileinfo := current_filepos;
{$endif}
         generictokenbuf:=nil;
         genericdef:=nil;
         typesymderef.reset;
         genericdefderef.reset;

         { Don't register forwarddefs, they are disposed at the
           end of an type block }
         if (dt=forwarddef) then
           exit;
         { Register in symtable stack }
         if doregister then
           begin
             { immediately register interface defs, as they will always be
               written to the ppu, their defid inlfuences the interface crc and
               if we wait, depending on e.g. compiler defines they may get a
               different defid (e.g. when a function is called, its procdef is
               registered, so depending on whether or not, or when, an interface
               procedure is called in the implementation, that may change its
               defid otherwise) }
             if assigned(current_module) and
                current_module.in_interface then
               register_def
             else
               maybe_put_in_symtable_stack;
           end;
      end;


    destructor tstoreddef.destroy;
      var
        i : longint;
      begin
        { Direct calls are not allowed, use symtable.deletedef() }
        if assigned(owner) then
          internalerror(200612311);
        if assigned(generictokenbuf) then
          begin
            generictokenbuf.free;
            generictokenbuf:=nil;
          end;
        rtti_attribute_list.free;
        genericparas.free;
        if assigned(genericparaderefs) then
          for i:=0 to genericparaderefs.count-1 do
            dispose(pderef(genericparaderefs[i]));
        genericparaderefs.free;
        genconstraintdata.free;
{$ifndef symansistr}
        stringdispose(_fullownerhierarchyname);
{$endif not symansistr}
        inherited destroy;
      end;


    constructor tstoreddef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      var
        sizeleft,i,cnt : longint;
        buf  : array[0..255] of byte;
        symderef : pderef;
      begin
         inherited create(dt);
         DefId:=ppufile.getlongint;
         current_module.deflist[DefId]:=self;
{$ifdef EXTDEBUG}
         fillchar(fileinfo,sizeof(fileinfo),0);
{$endif}
         { load }
         ppufile.getderef(typesymderef);
         ppufile.getset(tppuset2(defoptions));
         ppufile.getset(tppuset1(defstates));
         if df_genconstraint in defoptions then
           begin
             genconstraintdata:=tgenericconstraintdata.create;
             genconstraintdata.ppuload(ppufile);
           end;
         if [df_generic,df_specialization]*defoptions<>[] then
           begin
             cnt:=ppufile.getlongint;
             if cnt>0 then
               begin
                 genericparas:=tfphashobjectlist.create(false);
                 genericparaderefs:=tfplist.create;
                 for i:=0 to cnt-1 do
                   begin
                     genericparas.add(ppufile.getstring,nil);
                     New(symderef);
                     ppufile.getderef(symderef^);
                     genericparaderefs.add(symderef);
                   end;
               end;
           end;
         if df_generic in defoptions then
           begin
             sizeleft:=ppufile.getlongint;
             initgeneric;
             while sizeleft>0 do
               begin
                 if sizeleft>sizeof(buf) then
                   i:=sizeof(buf)
                 else
                   i:=sizeleft;
                 ppufile.getdata(buf,i);
                 generictokenbuf.write(buf,i);
                 dec(sizeleft,i);
               end;
           end;
        if df_specialization in defoptions then
          ppufile.getderef(genericdefderef);
        rtti_attribute_list:=trtti_attribute_list.ppuload(ppufile);
      end;


    function tstoreddef.needs_separate_initrtti:boolean;
      begin
        result:=false;
      end;


    function tstoreddef.rtti_mangledname(rt : trttitype) : TSymStr;
      var
        prefix : string[4];
      begin
        if (rt=fullrtti) or (not needs_separate_initrtti) then
          begin
            prefix:='RTTI';
            include(defstates,ds_rtti_table_used);
          end
        else
          begin
            prefix:='INIT';
            include(defstates,ds_init_table_used);
          end;
        if assigned(typesym) and
           (owner.symtabletype in [staticsymtable,globalsymtable]) then
          result:=make_mangledname(prefix,typesym.owner,typesym.name)
        else
          result:=make_mangledname(prefix,findunitsymtable(owner),'def'+unique_id_str)
      end;


    function tstoreddef.OwnerHierarchyName: string;
      var
        tmp: tdef;
      begin
        tmp:=self;
        result:='';
        repeat
          { can be not assigned in case of a forwarddef }
          if assigned(tmp.owner) and
             (tmp.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
            tmp:=tdef(tmp.owner.defowner)
          else
            break;
          result:=tabstractrecorddef(tmp).objrealname^+'.'+result;
        until tmp=nil;
      end;

    function tstoreddef.fullownerhierarchyname(skipprocparams:boolean): TSymStr;
      var
        lastowner: tsymtable;
        tmp: tdef;
        pno: tprocnameoptions;
      begin
{$ifdef symansistr}
        if not skipprocparams and (_fullownerhierarchyname<>'') then
          exit(_fullownerhierarchyname);
{$else symansistr}
        if not skipprocparams and assigned(_fullownerhierarchyname) then
          exit(_fullownerhierarchyname^);
{$endif symansistr}
        { the def can only reside inside structured types or
          procedures/functions/methods }
        tmp:=self;
        result:='';
        repeat
          lastowner:=tmp.owner;
          { can be not assigned in case of a forwarddef }
          if not assigned(lastowner) then
            break
          else
            tmp:=tdef(lastowner.defowner);
          if not assigned(tmp) then
            break;
          if tmp.typ in [recorddef,objectdef] then
            result:=tabstractrecorddef(tmp).objrealname^+'.'+result
          else
            if tmp.typ=procdef then
              begin
                pno:=[pno_paranames,pno_proctypeoption];
                if skipprocparams then
                  include(pno,pno_noparams);
                result:=tprocdef(tmp).customprocname(pno)+'.'+result;
              end;
        until tmp=nil;
        { add the unit name }
        if assigned(lastowner) and
           assigned(lastowner.realname) then
          result:=lastowner.realname^+'.'+result;
        if not skipprocparams then
          { don't store the name in this case }
{$ifdef symansistr}
          _fullownerhierarchyname:=result;
{$else symansistr}
          _fullownerhierarchyname:=stringdup(result);
{$endif symansistr}
      end;


    function tstoreddef.in_currentunit: boolean;
      var
        st: tsymtable;
      begin
        st:=owner;
        while not(st.symtabletype in [globalsymtable,staticsymtable]) do
          st:=st.defowner.owner;
        result:=st.iscurrentunit;
      end;


    function tstoreddef.getcopy : tstoreddef;
      begin
        Message(sym_e_cant_create_unique_type);
        getcopy:=cerrordef.create;
      end;


    procedure tstoreddef.ppuwrite(ppufile:tcompilerppufile);
      var
        sizeleft,i : longint;
        buf  : array[0..255] of byte;
        oldintfcrc : boolean;
      begin
        if defid<0 then
          internalerror(2015101401);
        ppufile.putlongint(DefId);
        ppufile.putderef(typesymderef);
        ppufile.putset(tppuset2(defoptions));
        oldintfcrc:=ppufile.do_crc;
        ppufile.do_crc:=false;
        ppufile.putset(tppuset1(defstates));
        if df_genconstraint in defoptions then
          genconstraintdata.ppuwrite(ppufile);
        if [df_generic,df_specialization]*defoptions<>[] then
          begin
            if not assigned(genericparas) then
              ppufile.putlongint(0)
            else
              begin
                if not assigned(genericparaderefs) then
                  internalerror(2014052305);
                ppufile.putlongint(genericparas.count);
                for i:=0 to genericparas.count-1 do
                  begin
                    ppufile.putstring(genericparas.nameofindex(i));
                    ppufile.putderef(pderef(genericparaderefs[i])^);
                  end;
              end;
          end;
        if df_generic in defoptions then
          begin
            if assigned(generictokenbuf) then
              begin
                sizeleft:=generictokenbuf.size;
                generictokenbuf.seek(0);
              end
            else
              sizeleft:=0;
            ppufile.putlongint(sizeleft);
            while sizeleft>0 do
              begin
                if sizeleft>sizeof(buf) then
                  i:=sizeof(buf)
                else
                  i:=sizeleft;
                generictokenbuf.read(buf,i);
                ppufile.putdata(buf,i);
                dec(sizeleft,i);
              end;
          end;
        ppufile.do_crc:=oldintfcrc;
        if df_specialization in defoptions then
          ppufile.putderef(genericdefderef);
        trtti_attribute_list.ppuwrite(rtti_attribute_list,ppufile);
      end;


    procedure tstoreddef.ppuload_subentries(ppufile: tcompilerppufile);
      begin
        trtti_attribute_list.ppuload_subentries(rtti_attribute_list,ppufile);
      end;


    procedure tstoreddef.ppuwrite_subentries(ppufile: tcompilerppufile);
      begin
        trtti_attribute_list.ppuwrite_subentries(rtti_attribute_list,ppufile);
      end;


    procedure tstoreddef.buildderef;
      var
        i : longint;
        sym : tsym;
        symderef : pderef;
      begin
        if not registered then
          register_def;
        typesymderef.build(typesym);
        genericdefderef.build(genericdef);
        if assigned(rtti_attribute_list) then
          rtti_attribute_list.buildderef;
        if assigned(genconstraintdata) then
          genconstraintdata.buildderef;
        if assigned(genericparas) then
          begin
            if not assigned(genericparaderefs) then
              genericparaderefs:=tfplist.create;
            for i:=0 to genericparas.count-1 do
              begin
                sym:=tsym(genericparas.items[i]);
                new(symderef);
                symderef^.build(sym);
                genericparaderefs.add(symderef);
              end;
          end;
      end;


    procedure tstoreddef.buildderefimpl;
      begin
      end;


    procedure tstoreddef.deref;
      var
        symderef : pderef;
        i : longint;
      begin
        typesym:=ttypesym(typesymderef.resolve);
        if df_specialization in defoptions then
          genericdef:=tstoreddef(genericdefderef.resolve);
        if assigned(rtti_attribute_list) then
          rtti_attribute_list.deref;
        if assigned(genconstraintdata) then
          genconstraintdata.deref;
        if assigned(genericparas) then
          begin
            if not assigned(genericparaderefs) then
              internalerror(2014052302);
            if genericparas.count<>genericparaderefs.count then
              internalerror(2014052303);
            for i:=0 to genericparaderefs.count-1 do
              begin
                symderef:=pderef(genericparaderefs[i]);
                genericparas.items[i]:=symderef^.resolve;
              end;
          end;
      end;


    procedure tstoreddef.derefimpl;
      begin
      end;


    function tstoreddef.size : asizeint;
      begin
         size:=savesize;
      end;


    function tstoreddef.getvardef:longint;
      begin
        result:=varUndefined;
      end;


    function tstoreddef.alignment : shortint;
      begin
         { natural alignment by default }
         alignment:=size_2_align(savesize);
         { can happen if savesize = 0, e.g. for voiddef or
           an empty record
         }
         if (alignment=0) then
           alignment:=1;
      end;


    { returns true, if the definition can be published }
    function tstoreddef.is_publishable : boolean;
      begin
         is_publishable:=false;
      end;


    { needs an init table }
    function tstoreddef.needs_inittable : boolean;
      begin
         needs_inittable:=false;
      end;


    function tstoreddef.has_non_trivial_init_child(check_parent:boolean):boolean;
      begin
        result:=false;
      end;


   function tstoreddef.is_intregable : boolean;
     var
       recsize,temp: longint;
     begin
        case typ of
          orddef,
          pointerdef,
          enumdef,
          classrefdef:
            is_intregable:=true;
          procvardef :
            is_intregable:=tprocvardef(self).is_addressonly or (po_methodpointer in tprocvardef(self).procoptions);
          objectdef:
            is_intregable:=(is_implicit_pointer_object_type(self)) and not needs_inittable;
          setdef:
            is_intregable:=is_smallset(self);
          arraydef:
{$ifdef cpuhighleveltarget}
            is_intregable:=false
{$else cpuhighleveltarget}
            is_intregable:=not(is_special_array(self)) and
              (tarraydef(self).size<=8) and (tarraydef(self).size in [1,2,4,8]) and
              tstoreddef(tarraydef(self).elementdef).is_intregable
{$ifdef SUPPORT_MMX}
              and not((cs_mmx in current_settings.localswitches) and
                 is_mmx_able_array(self))
{$endif SUPPORT_MMX}
{$endif cpuhighleveltarget}
              ;
          recorddef:
            begin
{$ifdef cpuhighleveltarget}
              is_intregable:=false;
{$else cpuhighleveltarget}
              recsize:=size;
              is_intregable:=
                ispowerof2(recsize,temp) and
                ((recsize<=sizeof(aint)*2) and
                 not trecorddef(self).contains_cross_aword_field and
                 { records cannot go into registers on 16 bit targets for now }
                 (sizeof(aint)>2) and
                 (not trecorddef(self).contains_float_field) or
                  (recsize <= sizeof(aint))
                 ) and
                 not needs_inittable;
{$endif cpuhighleveltarget}
            end;
          else
           is_intregable:=false;
        end;
     end;


   function tstoreddef.is_fpuregable : boolean;
     begin
{$ifdef x86}
       result:=use_vectorfpu(self);
{$else x86}
       result:=(typ=floatdef) and not(cs_fp_emulation in current_settings.moduleswitches);
{$endif x86}
     end;


   function tstoreddef.is_const_intregable : boolean;
     begin
       case typ of
         stringdef:
           result:=tstringdef(self).stringtype in [st_ansistring,st_unicodestring,st_widestring];
         arraydef:
           result:=is_dynamic_array(self);
         objectdef:
           result:=is_interface(self);
         else
           result:=false;
       end;
     end;


   procedure tstoreddef.initgeneric;
     begin
       if assigned(generictokenbuf) then
         internalerror(200512131);
       generictokenbuf:=tdynamicarray.create(256);
     end;


   function tstoreddef.is_generic: boolean;
     var
       sym: tsym;
       i: longint;
     begin
       result:=assigned(genericparas) and
                 (genericparas.count>0) and
                 (df_generic in defoptions);
       if result then
         { if any of the type parameters does *not* belong to as (meaning it was passed
           in from outside) then we aren't a generic, but a specialization }
         for i:=0 to genericparas.count-1 do
           begin
             sym:=tsym(genericparas[i]);
             if sym.typ<>symconst.typesym then
               internalerror(2014050903);
             if sym.owner.defowner<>self then
               exit(false);
           end;
     end;


   function tstoreddef.is_specialization: boolean;
     var
       i : longint;
       sym : tsym;
     begin
       result:=assigned(genericparas) and
                 (genericparas.count>0) and
                 (df_specialization in defoptions);
       if result then
         begin
           { if at least one of the generic parameters is not owned by us (meaning it was
             passed in from outside) then we have a specialization, otherwise we have a generic }
           for i:=0 to genericparas.count-1 do
             begin
               sym:=tsym(genericparas[i]);
               if sym.typ<>symconst.typesym then
                 internalerror(2014050904);
               if sym.owner.defowner<>self then
                 exit(true);
             end;
           result:=false;
         end;
     end;


   procedure tstoreddef.register_def;
     var
       gst : tgetsymtable;
       st : tsymtable;
     begin
       if registered then
         exit;
       { Register in current_module }
       if assigned(current_module) then
         begin
           exclude(defoptions,df_not_registered_no_free);
           for gst:=low(tgetsymtable) to high(tgetsymtable) do
             begin
               st:=getsymtable(gst);
               if assigned(st) then
                 tstoredsymtable(st).register_children;
             end;
           if defid<defid_not_registered then
             defid:=deflist_index
           else
             begin
               current_module.deflist.Add(self);
               defid:=current_module.deflist.Count-1;
             end;
           maybe_put_in_symtable_stack;
         end
       else
         DefId:=defid_registered_nost;
     end;


   procedure tstoreddef.maybe_put_in_symtable_stack;
     var
       insertstack: psymtablestackitem;
     begin
       if assigned(symtablestack) and
          not assigned(self.owner) then
         begin
           insertstack:=symtablestack.stack;
           { don't insert defs in exception symtables, as they are freed before
             the module is compiled, so we can get crashes on high level targets
             if they still need it while e.g. writing assembler code }
           while assigned(insertstack) and
                 (insertstack^.symtable.symtabletype in [stt_exceptsymtable,withsymtable]) do
             insertstack:=insertstack^.next;
           if not assigned(insertstack) then
             internalerror(200602044);
           if insertstack^.symtable.sealed then
             internalerror(2015022301);
           insertstack^.symtable.insertdef(self);
         end;
     end;


{****************************************************************************
                               Tstringdef
****************************************************************************}

    constructor tstringdef.createshort(l: byte; doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_shortstring;
         encoding:=0;
         len:=l;
      end;


    constructor tstringdef.loadshort(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_shortstring;
         encoding:=0;
         len:=ppufile.getbyte;
         ppuload_platform(ppufile);
      end;


    constructor tstringdef.createlong(l: asizeint; doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_longstring;
         encoding:=0;
         len:=l;
      end;


    constructor tstringdef.loadlong(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_longstring;
         encoding:=0;
         len:=ppufile.getasizeint;
         ppuload_platform(ppufile);
      end;


    constructor tstringdef.createansi(aencoding: tstringencoding; doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_ansistring;
         encoding:=aencoding;
         len:=-1;
      end;


    constructor tstringdef.loadansi(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_ansistring;
         len:=ppufile.getasizeint;
         encoding:=ppufile.getword;
         ppuload_platform(ppufile);
      end;


    constructor tstringdef.createwide(doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_widestring;
         if target_info.endian=endian_little then
           encoding:=CP_UTF16LE
         else
           encoding:=CP_UTF16BE;
         len:=-1;
      end;


    constructor tstringdef.loadwide(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_widestring;
         if target_info.endian=endian_little then
           encoding:=CP_UTF16LE
         else
           encoding:=CP_UTF16BE;
         len:=ppufile.getasizeint;
         ppuload_platform(ppufile);
      end;


    constructor tstringdef.createunicode(doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_unicodestring;
         if target_info.endian=endian_little then
           encoding:=CP_UTF16LE
         else
           encoding:=CP_UTF16BE;
         len:=-1;
      end;


    constructor tstringdef.loadunicode(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_unicodestring;
         len:=ppufile.getasizeint;
         encoding:=ppufile.getword;
         ppuload_platform(ppufile);
      end;


    function tstringdef.getcopy : tstoreddef;
      begin
        result:=cstringdef.create(typ,true);
        result.typ:=stringdef;
        tstringdef(result).stringtype:=stringtype;
        tstringdef(result).encoding:=encoding;
        tstringdef(result).len:=len;
      end;


    function tstringdef.stringtypname:string;
      const
        typname:array[tstringtype] of string[10]=(
          'shortstr','longstr','ansistr','widestr','unicodestr'
        );
      begin
        stringtypname:=typname[stringtype];
      end;


    procedure tstringdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         if stringtype=st_shortstring then
           begin
{$ifdef extdebug}
            if len > 255 then internalerror(12122002);
{$endif}
            ppufile.putbyte(byte(len))
           end
         else
           ppufile.putasizeint(len);
         if stringtype in [st_ansistring,st_unicodestring] then
           ppufile.putword(encoding);
         case stringtype of
            st_shortstring : writeentry(ppufile,ibshortstringdef);
            st_longstring : writeentry(ppufile,iblongstringdef);
            st_ansistring : writeentry(ppufile,ibansistringdef);
            st_widestring : writeentry(ppufile,ibwidestringdef);
            st_unicodestring : writeentry(ppufile,ibunicodestringdef);
         end;
      end;


    function tstringdef.needs_inittable : boolean;
      begin
         needs_inittable:=stringtype in [st_ansistring,st_widestring,st_unicodestring];
      end;


    function tstringdef.GetTypeName : string;
      const
         names : array[tstringtype] of string[15] = (
           'ShortString','LongString','AnsiString','WideString','UnicodeString');
      begin
         GetTypeName:=names[stringtype];
      end;


    function tstringdef.getvardef : longint;
      const
        vardef : array[tstringtype] of longint = (
          varUndefined,varUndefined,varString,varOleStr,varUString);
      begin
        result:=vardef[stringtype];
      end;


    function tstringdef.alignment : shortint;
      begin
        case stringtype of
          st_unicodestring,
          st_widestring,
          st_ansistring:
            alignment:=voidpointertype.alignment;
          st_longstring,
          st_shortstring:
              { char to string accesses byte 0 and 1 with one word access }
            if (tf_requires_proper_alignment in target_info.flags) or
              { macpas needs an alignment of 2 (MetroWerks compatible) }
               (m_mac in current_settings.modeswitches) then
              alignment:=size_2_align(2)
            else
              alignment:=size_2_align(1);
        end;
      end;


    function tstringdef.getmangledparaname : TSymStr;
      begin
        getmangledparaname:='STRING';
      end;


    function tstringdef.is_publishable : boolean;
      begin
         is_publishable:=true;
      end;


    function tstringdef.size: asizeint;
      begin
        case stringtype of
          st_shortstring:
            Result:=len+1;
          st_longstring,
          st_ansistring,
          st_widestring,
          st_unicodestring:
            Result:=voidpointertype.size;
        end;
      end;


{****************************************************************************
                                 TENUMDEF
****************************************************************************}

    constructor tenumdef.create;
      begin
         inherited create(enumdef,true);
         minval:=0;
         maxval:=0;
         calcsavesize(current_settings.packenum);
         has_jumps:=false;
         basedef:=nil;
         basedefderef.reset;
         symtable:=tenumsymtable.create(self);
      end;


    constructor tenumdef.create_subrange(_basedef:tenumdef;_min,_max:asizeint);
      begin
         inherited create(enumdef,true);
         minval:=_min;
         maxval:=_max;
         basedef:=_basedef;
         calcsavesize(current_settings.packenum);
         has_jumps:=false;
         symtable:=basedef.symtable.getcopy;
         include(defoptions, df_copied_def);
       end;


    constructor tenumdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(enumdef,ppufile);
         minval:=ppufile.getaint;
         maxval:=ppufile.getaint;
         savesize:=ppufile.getaint;
         has_jumps:=false;
         if df_copied_def in defoptions then
           begin
             symtable:=nil;
             ppufile.getderef(basedefderef);
             ppuload_platform(ppufile);
           end
         else
           begin
             ppuload_platform(ppufile);
             // create with nil defowner first to prevent values changes on insert
             symtable:=tenumsymtable.create(nil);
             tenumsymtable(symtable).ppuload(ppufile);
             symtable.defowner:=self;
           end;
      end;

    destructor tenumdef.destroy;
      begin
        symtable.free;
        symtable:=nil;
        inherited destroy;
      end;


    function tenumdef.getcopy : tstoreddef;
      begin
        if assigned(basedef) then
          result:=cenumdef.create_subrange(basedef,minval,maxval)
        else
          begin
            result:=cenumdef.create;
            tenumdef(result).minval:=minval;
            tenumdef(result).maxval:=maxval;
            tenumdef(result).symtable.free;
            tenumdef(result).symtable:=symtable.getcopy;
            tenumdef(result).basedef:=self;
          end;
        tenumdef(result).has_jumps:=has_jumps;
        tenumdef(result).basedefderef:=basedefderef;
        include(tenumdef(result).defoptions,df_copied_def);
      end;


    procedure tenumdef.calcsavesize(packenum: shortint);
      begin
{$IFNDEF cpu64bitaddr} {$push}{$warnings off} {$ENDIF} //comparison always false warning
        if (packenum=8) or (int64(min)<low(longint)) or (int64(max)>high(cardinal)) then
         savesize:=8
{$IFNDEF cpu64bitaddr} {$pop} {$ENDIF}
        else
{$IFDEF cpu16bitaddr} {$push}{$warnings off} {$ENDIF} //comparison always false warning
         if (packenum=4) or (min<low(smallint)) or (max>high(word)) then
          savesize:=4
{$IFDEF cpu16bitaddr} {$pop} {$ENDIF}
        else
         if (packenum=2) or (min<low(shortint)) or (max>high(byte)) then
          savesize:=2
        else
         savesize:=1;
      end;


    function tenumdef.packedbitsize: asizeint;
      var
        sizeval: tconstexprint;
        power: longint;
      begin
        result := 0;
        if (minval >= 0) and
           (maxval <= 1) then
          result := 1
        else
          begin
            if (minval>=0) then
              sizeval:=maxval
            else
             { don't count 0 twice, but take into account that range goes from -n-1..n }
              sizeval:=(cutils.max(-minval,maxval+1)*2)-1;
            { 256 must become 512 etc. }
            nextpowerof2(sizeval+1,power);
            result := power;
          end;
      end;


    procedure tenumdef.setmax(_max:asizeint);
      begin
        maxval:=_max;
        calcsavesize(current_settings.packenum);
      end;


    procedure tenumdef.setmin(_min:asizeint);
      begin
        minval:=_min;
        calcsavesize(current_settings.packenum);
      end;


    function tenumdef.min:asizeint;
      begin
        min:=minval;
      end;


    function tenumdef.max:asizeint;
      begin
        max:=maxval;
      end;

    function tenumdef.getfirstsym: tsym;
      var
        i:integer;
      begin
        for i := 0 to symtable.SymList.Count - 1 do
          begin
            result:=tsym(symtable.SymList[i]);
            if tenumsym(result).value=minval then
              exit;
          end;
        result:=nil;
      end;


    function tenumdef.int2enumsym(l: asizeint): tsym;
      var
        i: longint;
        sym: tsym;
        bdef: tenumdef;
      begin
        result:=nil;
        if (l<minval) or
           (l>maxval) then
          exit;
        bdef:=getbasedef;
        for i:=0 to bdef.symtable.symlist.count-1 do
          begin
            sym:=tsym(bdef.symtable.symlist[i]);
            if (sym.typ=enumsym) and
               (tenumsym(sym).value=l) then
              begin
                result:=sym;
                exit;
              end;
          end;
      end;


    function tenumdef.getbasedef: tenumdef;
      begin
        if not assigned(basedef) then
          result:=self
        else
          result:=basedef;
      end;


    procedure tenumdef.buildderef;
      begin
        inherited buildderef;
        if df_copied_def in defoptions then
          basedefderef.build(basedef)
        else
          tenumsymtable(symtable).buildderef;
      end;


    procedure tenumdef.deref;
      begin
        inherited deref;
        if df_copied_def in defoptions then
          begin
            basedef:=tenumdef(basedefderef.resolve);
            symtable:=basedef.symtable.getcopy;
          end
        else
          tenumsymtable(symtable).deref(false);
      end;


    procedure tenumdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putaint(min);
         ppufile.putaint(max);
         ppufile.putaint(savesize);
         if df_copied_def in defoptions then
           ppufile.putderef(basedefderef);
         writeentry(ppufile,ibenumdef);
         if not (df_copied_def in defoptions) then
           tenumsymtable(symtable).ppuwrite(ppufile);
      end;


    function tenumdef.is_publishable : boolean;
      begin
         is_publishable:=true;
      end;


    function tenumdef.GetTypeName : string;
      begin
         GetTypeName:='<enumeration type>';
      end;

{****************************************************************************
                             TRTTI_ATTRIBUTE_LIST
****************************************************************************}


    constructor trtti_attribute.ppuload(ppufile: tcompilerppufile);
      begin
        ppufile.getderef(typesymderef);
        ppufile.getderef(typeconstrderef);
        setlength(paras,ppufile.getlongint);
      end;


    procedure trtti_attribute.ppuwrite(ppufile: tcompilerppufile);
      begin
        ppufile.putderef(typesymderef);
        ppufile.putderef(typeconstrderef);
        ppufile.putlongint(length(paras));
      end;


    procedure trtti_attribute.ppuload_subentries(ppufile: tcompilerppufile);
      var
        i : sizeint;
      begin
        for i:=0 to high(paras) do
          paras[i]:=ppuloadnodetree(ppufile);
      end;


    procedure trtti_attribute.ppuwrite_subentries(ppufile: tcompilerppufile);
      var
        i : sizeint;
      begin
        for i:=0 to high(paras) do
          ppuwritenodetree(ppufile,paras[i]);
      end;


    destructor trtti_attribute.destroy;
      var
        n : tnode;
      begin
        constructorcall.free;
        for n in paras do
          n.free;
        inherited destroy;
      end;


    procedure trtti_attribute.buildderef;
      var
        i : sizeint;
      begin
        typesymderef.build(typesym);
        typeconstrderef.build(typeconstr);
        for i:=0 to high(paras) do
          paras[i].buildderefimpl;
      end;


    procedure trtti_attribute.deref;
      var
        i : sizeint;
      begin
        typesym:=tsym(typesymderef.resolve);
        typeconstr:=tdef(typeconstrderef.resolve);
        for i:=0 to high(paras) do
          paras[i].derefimpl;
      end;


    class procedure trtti_attribute_list.bind(var dangling,owned:trtti_attribute_list);
      begin
        if assigned(owned) then
          internalerror(2019071001);
        if not assigned(dangling) then
          exit;
        if dangling.is_bound then
          internalerror(2019071002);
        current_module.used_rtti_attrs.concatlistcopy(dangling.rtti_attributes);
        dangling.is_bound:=true;
        owned:=dangling;
        dangling:=nil;
      end;


    procedure trtti_attribute_list.addattribute(atypesym:tsym;typeconstr:tdef;constructorcall:tnode;constref paras:array of tnode);
      var
        newattribute : trtti_attribute;
        i : sizeint;
      begin
        if not assigned(rtti_attributes) then
          rtti_attributes:=TFPObjectList.Create(true);
        newattribute:=trtti_attribute.Create;
        newattribute.typesym:=atypesym;
        newattribute.typeconstr:=typeconstr;
        newattribute.constructorcall:=constructorcall;
        setlength(newattribute.paras,length(paras));
        for i:=0 to high(paras) do
          newattribute.paras[i]:=paras[i];
        rtti_attributes.Add(newattribute);
      end;


    procedure trtti_attribute_list.addattribute(attr:trtti_attribute);
      begin
        if not assigned(rtti_attributes) then
          rtti_attributes:=TFPObjectList.Create(true);
        rtti_attributes.add(attr);
      end;


    destructor trtti_attribute_list.destroy;
      begin
        rtti_attributes.Free;
        inherited destroy;
      end;


    function trtti_attribute_list.get_attribute_count:longint;
      begin
        if assigned(rtti_attributes) then
          result:=rtti_attributes.Count
        else
          result:=0;
      end;


    procedure trtti_attribute_list.buildderef;
      var
        i : sizeint;
      begin
        if not assigned(rtti_attributes) then
          exit;
        for i:=0 to rtti_attributes.count-1 do
          trtti_attribute(rtti_attributes[i]).buildderef;
      end;


    procedure trtti_attribute_list.deref;
      var
        i : sizeint;
      begin
        if not assigned(rtti_attributes) then
          exit;
        for i:=0 to rtti_attributes.count-1 do
          trtti_attribute(rtti_attributes[i]).deref;
      end;


    class procedure trtti_attribute_list.ppuload_subentries(attrlist:trtti_attribute_list;ppufile:tcompilerppufile);
      var
        i : sizeint;
      begin
        if assigned(attrlist) then
          begin
            if not assigned(attrlist.rtti_attributes) then
              internalerror(2019071101);
            for i:=0 to attrlist.rtti_attributes.count-1 do
              trtti_attribute(attrlist.rtti_attributes[i]).ppuload_subentries(ppufile);
          end;
      end;


    class procedure trtti_attribute_list.ppuwrite_subentries(attrlist:trtti_attribute_list;ppufile:tcompilerppufile);
      var
        i : sizeint;
      begin
        if assigned(attrlist) and assigned(attrlist.rtti_attributes) then
          begin
            for i:=0 to attrlist.rtti_attributes.count-1 do
              trtti_attribute(attrlist.rtti_attributes[i]).ppuwrite_subentries(ppufile);
          end;
      end;


    class function trtti_attribute_list.ppuload(ppufile:tcompilerppufile):trtti_attribute_list;
      var
        cnt,i : longint;
      begin
        cnt:=ppufile.getlongint;
        if cnt>0 then
          begin
            result:=trtti_attribute_list.create;
            for i:=0 to cnt-1 do
              result.addattribute(trtti_attribute.ppuload(ppufile));
          end
        else
          result:=nil;
      end;


    class procedure trtti_attribute_list.ppuwrite(attrlist:trtti_attribute_list;ppufile:tcompilerppufile);
      var
        i : longint;
      begin
        if assigned(attrlist) and assigned(attrlist.rtti_attributes) then
          begin
            ppufile.putlongint(attrlist.rtti_attributes.count);
            for i:=0 to attrlist.rtti_attributes.count-1 do
              trtti_attribute(attrlist.rtti_attributes[i]).ppuwrite(ppufile);
          end
        else
          ppufile.putlongint(0);
      end;


{****************************************************************************
                                 TORDDEF
****************************************************************************}

    constructor torddef.create(t : tordtype;v,b : TConstExprInt; doregister: boolean);
      begin
         inherited create(orddef,doregister);
         low:=v;
         high:=b;
         ordtype:=t;
         setsize;
      end;


    constructor torddef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(orddef,ppufile);
         ordtype:=tordtype(ppufile.getbyte);
         low:=ppufile.getexprint;
         high:=ppufile.getexprint;
         setsize;
         ppuload_platform(ppufile);
      end;


    function torddef.getcopy : tstoreddef;
      begin
         result:=corddef.create(ordtype,low,high,true);
         result.typ:=orddef;
         torddef(result).low:=low;
         torddef(result).high:=high;
         torddef(result).ordtype:=ordtype;
         torddef(result).savesize:=savesize;
      end;


    function torddef.alignment:shortint;
      begin
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim,system_arm_darwin]) and
           (ordtype in [s64bit,u64bit]) then
          result := 4
        else
          result := inherited alignment;
      end;


    procedure torddef.setsize;
      const
        sizetbl : array[tordtype] of longint = (
          0,
          1,2,4,8,16,
          1,2,4,8,16,
          1,1,2,4,8,
          1,2,4,8,
          1,2,8,system.high(longint)
        );
      begin
        savesize:=sizetbl[ordtype];
        if savesize=system.high(longint) then
          savesize:=packedbitsize div 8;
      end;


    function torddef.packedbitsize: asizeint;
      var
        sizeval: tconstexprint;
        power: longint;
      begin
        result := 0;
        if ordtype = uvoid then
          exit;

{$ifndef cpu64bitalu}
        if (ordtype in [s64bit,u64bit]) then
{$else not cpu64bitalu}
        if (ordtype = u64bit) or
           ((ordtype = s64bit) and
            ((low <= (system.low(int64) div 2)) or
             (high > (system.high(int64) div 2)))) then
{$endif cpu64bitalu}
          result := 64
        else if (
            (low >= 0) and
            (high <= 1)
           ) or (
             ordtype in [pasbool1,pasbool8,pasbool16,pasbool32,pasbool64,bool8bit,bool16bit,bool32bit,bool64bit]
           ) then
          result := 1
        else
          begin
            if (low>=0) then
              sizeval:=high
            else
              { don't count 0 twice, but take into account that range goes from -n-1..n }
              sizeval:=(cutils.max(-low,high+1)*2)-1;
            { 256 must become 512 etc. }
            nextpowerof2(sizeval+1,power);
            result := power;
         end;
      end;


    function torddef.getvardef : longint;
      const
        basetype2vardef : array[tordtype] of longint = (
          varUndefined,
          varbyte,varword,varlongword,varqword,varUndefined,
          varshortint,varsmallint,varinteger,varint64,varUndefined,
          varboolean,varboolean,varboolean,varboolean,varboolean,
          varboolean,varboolean,varUndefined,varUndefined,
          varUndefined,varUndefined,varCurrency,varEmpty);
      begin
        result:=basetype2vardef[ordtype];
        if result=varEmpty then
          result:=basetype2vardef[range_to_basetype(low,high)];
      end;


    procedure torddef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(ordtype));
         ppufile.putexprint(low);
         ppufile.putexprint(high);
         writeentry(ppufile,iborddef);
      end;


    function torddef.is_publishable : boolean;
      begin
         is_publishable:=(ordtype<>uvoid);
      end;


    function torddef.GetTypeName : string;
      const
        names : array[tordtype] of string[20] = (
          'untyped',
          'Byte','Word','DWord','QWord','UInt128',
          'ShortInt','SmallInt','LongInt','Int64','Int128',
          'Boolean','Boolean8','Boolean16','Boolean32','Boolean64',
          'ByteBool','WordBool','LongBool','QWordBool',
          'Char','WideChar','Currency','CustomRange');

      begin
         GetTypeName:=names[ordtype];
      end;


{****************************************************************************
                                TFLOATDEF
****************************************************************************}

    constructor tfloatdef.create(t: tfloattype; doregister: boolean);
      begin
         inherited create(floatdef,doregister);
         floattype:=t;
         setsize;
      end;


    constructor tfloatdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(floatdef,ppufile);
         floattype:=tfloattype(ppufile.getbyte);
         setsize;
         ppuload_platform(ppufile);
      end;


    function tfloatdef.getcopy : tstoreddef;
      begin
         result:=cfloatdef.create(floattype,true);
         result.typ:=floatdef;
         tfloatdef(result).savesize:=savesize;
      end;


    function tfloatdef.alignment:shortint;
      begin
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim,system_arm_darwin]) then
          case floattype of
            sc80real,
            s80real: result:=16;
            s64real,
            s64currency,
            s64comp : result:=4;
            else
              result := inherited alignment;
          end
        else
          result := inherited alignment;
      end;


    function tfloatdef.structalignment: shortint;
      begin
        { aix is really annoying: the recommended scalar alignment for both
          int64 and double is 64 bits, but in structs int64 has to be aligned
          to 8 bytes and double to 4 bytes }
        if (target_info.system in systems_aix) and
           (floattype=s64real) then
          result:=4
        else
          result:=alignment;
      end;


    procedure tfloatdef.setsize;
      begin
         case floattype of
           s32real : savesize:=4;
           s80real : savesize:=10;
           sc80real:
             if target_info.system in [system_i386_darwin,
                  system_i386_iphonesim,system_x86_64_darwin,
                  system_x86_64_iphonesim,
                  system_x86_64_linux,system_x86_64_freebsd,
                  system_x86_64_openbsd,system_x86_64_netbsd,
                  system_x86_64_solaris,system_x86_64_embedded,
                  system_x86_64_dragonfly,system_x86_64_haiku] then
               savesize:=16
             else
               savesize:=12;
           s64real,
           s64currency,
           s64comp : savesize:=8;
         else
           savesize:=0;
         end;
      end;


    function tfloatdef.getvardef : longint;
      const
        floattype2vardef : array[tfloattype] of longint = (
          varSingle,varDouble,varUndefined,varUndefined,
          varUndefined,varCurrency,varUndefined);
      begin
        if (upper(typename)='TDATETIME') and
          assigned(owner) and
          assigned(owner.name) and
          (owner.name^='SYSTEM') then
          result:=varDate
        else
          result:=floattype2vardef[floattype];
      end;


    procedure tfloatdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(floattype));
         writeentry(ppufile,ibfloatdef);
      end;


    function tfloatdef.is_publishable : boolean;
      begin
         is_publishable:=true;
      end;


    function tfloatdef.GetTypeName : string;
      const
        names : array[tfloattype] of string[20] = (
          'Single','Double','Extended','CExtended','Comp','Currency','Float128');
      begin
         GetTypeName:=names[floattype];
      end;


{****************************************************************************
                                TFILEDEF
****************************************************************************}

    constructor tfiledef.createtext;
      begin
         inherited create(filedef,true);
         filetyp:=ft_text;
         typedfiledef:=nil;
         typedfiledefderef.reset;
      end;


    constructor tfiledef.createuntyped;
      begin
         inherited create(filedef,true);
         filetyp:=ft_untyped;
         typedfiledef:=nil;
      end;


    constructor tfiledef.createtyped(def:tdef);
      begin
         inherited create(filedef,true);
         filetyp:=ft_typed;
         typedfiledef:=def;
      end;


    constructor tfiledef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(filedef,ppufile);
         filetyp:=tfiletyp(ppufile.getbyte);
         if filetyp=ft_typed then
           ppufile.getderef(typedfiledefderef)
         else
           typedfiledef:=nil;
         ppuload_platform(ppufile);
      end;


    function tfiledef.getcopy : tstoreddef;
      begin
        case filetyp of
          ft_typed:
            result:=cfiledef.createtyped(typedfiledef);
          ft_untyped:
            result:=cfiledef.createuntyped;
          ft_text:
            result:=cfiledef.createtext;
        end;
      end;


    procedure tfiledef.buildderef;
      begin
        inherited buildderef;
        if filetyp=ft_typed then
          typedfiledefderef.build(typedfiledef);
      end;


    procedure tfiledef.deref;
      begin
        inherited deref;
        if filetyp=ft_typed then
          typedfiledef:=tdef(typedfiledefderef.resolve);
      end;


    function  tfiledef.size:asizeint;
      begin
        if savesize=0 then
          setsize;
        size:=savesize;
      end;


    procedure tfiledef.setsize;
      begin
       case filetyp of
         ft_text:
           savesize:=search_system_type('TEXTREC').typedef.size;
         ft_typed:
           begin
             savesize:=search_system_type('FILEREC').typedef.size;
             { allocate put/get buffer in iso mode }
             if m_isolike_io in current_settings.modeswitches then
               inc(savesize,typedfiledef.size);
           end;
         ft_untyped:
           savesize:=search_system_type('FILEREC').typedef.size;
         end;
      end;


    function tfiledef.alignment: shortint;
      begin
        case filetyp of
          ft_text:
            result:=search_system_type('TEXTREC').typedef.alignment;
          ft_typed,
          ft_untyped:
            result:=search_system_type('FILEREC').typedef.alignment;
          end;
      end;


    procedure tfiledef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(filetyp));
         if filetyp=ft_typed then
           ppufile.putderef(typedfiledefderef);
         writeentry(ppufile,ibfiledef);
      end;


    function tfiledef.GetTypeName : string;
      begin
         case filetyp of
           ft_untyped:
             GetTypeName:='File';
           ft_typed:
             GetTypeName:='File Of '+typedfiledef.typename;
           ft_text:
             GetTypeName:='Text'
         end;
      end;


    function tfiledef.getmangledparaname : TSymStr;
      begin
         case filetyp of
           ft_untyped:
             getmangledparaname:='FILE';
           ft_typed:
             getmangledparaname:='FILE$OF$'+typedfiledef.mangledparaname;
           ft_text:
             getmangledparaname:='TEXT'
         end;
      end;


{****************************************************************************
                               TVARIANTDEF
****************************************************************************}

    constructor tvariantdef.create(v : tvarianttype);
      begin
         inherited create(variantdef,true);
         varianttype:=v;
         setsize;
      end;


    constructor tvariantdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(variantdef,ppufile);
         varianttype:=tvarianttype(ppufile.getbyte);
         setsize;
         ppuload_platform(ppufile);
      end;


    function tvariantdef.getcopy : tstoreddef;
      begin
        result:=cvariantdef.create(varianttype);
      end;


    procedure tvariantdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(varianttype));
         writeentry(ppufile,ibvariantdef);
      end;


    function tvariantdef.getvardef : longint;
      begin
        Result:=varVariant;
      end;


    procedure tvariantdef.setsize;
      begin
{$ifdef cpu64bitaddr}
        savesize:=24;
{$else cpu64bitaddr}
        savesize:=16;
{$endif cpu64bitaddr}
      end;


    function tvariantdef.GetTypeName : string;
      begin
         case varianttype of
           vt_normalvariant:
             GetTypeName:='Variant';
           vt_olevariant:
             GetTypeName:='OleVariant';
         end;
      end;


    function tvariantdef.needs_inittable : boolean;
      begin
         needs_inittable:=true;
      end;


    function tvariantdef.is_publishable : boolean;
      begin
         is_publishable:=true;
      end;


{****************************************************************************
                            TABSTRACtpointerdef
****************************************************************************}

    constructor tabstractpointerdef.create(dt:tdeftyp;def:tdef);
      begin
        inherited create(dt,true);
        pointeddef:=def;
        pointeddefderef.reset;
        if df_generic in pointeddef.defoptions then
          include(defoptions,df_generic);
        if df_specialization in pointeddef.defoptions then
          include(defoptions,df_specialization);
      end;


    constructor tabstractpointerdef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
         inherited ppuload(dt,ppufile);
         ppufile.getderef(pointeddefderef);
      end;


    procedure tabstractpointerdef.buildderef;
      begin
        inherited buildderef;
        pointeddefderef.build(pointeddef);
      end;


    procedure tabstractpointerdef.deref;
      begin
        inherited deref;
        pointeddef:=tdef(pointeddefderef.resolve);
      end;


    function tabstractpointerdef.size: asizeint;
      begin
        Result:=voidpointertype.size;
      end;


    function tabstractpointerdef.alignment: shortint;
      begin
        alignment:=size_2_align(voidpointertype.size);
      end;


    procedure tabstractpointerdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(pointeddefderef);
      end;


{****************************************************************************
                               tpointerdef
****************************************************************************}

    constructor tpointerdef.create(def:tdef);
      begin
        inherited create(pointerdef,def);
        has_pointer_math:=cs_pointermath in current_settings.localswitches;
        if (df_specialization in tstoreddef(def).defoptions)
{$ifndef genericdef_for_nested}
           { currently, nested procdefs of generic routines get df_specialisation,
             but no genericdef }
           and assigned(tstoreddef(def).genericdef)
{$endif}
           then
          genericdef:=cpointerdef.getreusable(tstoreddef(def).genericdef);
      end;


    class function tpointerdef.getreusable(def: tdef): tpointerdef;
      var
        res: PHashSetItem;
        oldsymtablestack: tsymtablestack;
      begin
        if not assigned(current_module) then
          internalerror(2011071101);
        res:=current_module.ptrdefs.FindOrAdd(@def,sizeof(def));
        if not assigned(res^.Data) then
          begin
            { since these pointerdefs can be reused anywhere in the current
              unit, add them to the global/staticsymtable (or local symtable
              if they're a local def, because otherwise they'll be saved
              to the ppu referencing a local symtable entry that doesn't
              exist in the ppu) }
            oldsymtablestack:=symtablestack;
            { do not simply push/pop current_module.localsymtable, because
              that can have side-effects (e.g., it removes helpers) }
            symtablestack:=nil;
            result:=cpointerdef.create(def);
            setup_reusable_def(def,result,res,oldsymtablestack);
            { res^.Data may still be nil -> don't overwrite result }
            exit;
          end;
        result:=tpointerdef(res^.Data);
      end;


    class function tpointerdef.getreusable_no_free(def: tdef): tpointerdef;
      begin
        result:=getreusable(def);
        if not result.is_registered then
          include(result.defoptions,df_not_registered_no_free);
      end;


    function tpointerdef.size: asizeint;
      begin
        result:=sizeof(pint);
      end;


    constructor tpointerdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(pointerdef,ppufile);
         has_pointer_math:=(ppufile.getbyte<>0);
         ppuload_platform(ppufile);
      end;


    function tpointerdef.getcopy : tstoreddef;
      begin
        { don't use direct pointeddef if it is a forwarddef because in other case
          one of them will be destroyed on forward type resolve and the second will
          point to garbage }
        if pointeddef.typ=forwarddef then
          result:=cpointerdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=cpointerdef.create(pointeddef);
        tpointerdef(result).has_pointer_math:=has_pointer_math;
        tpointerdef(result).savesize:=savesize;
      end;


    procedure tpointerdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(has_pointer_math));
         writeentry(ppufile,ibpointerdef);
      end;


    function tpointerdef.GetTypeName : string;
      begin
        { parameter types and the resultdef of a procvardef can contain a
          pointer to this procvardef itself, resulting in endless recursion ->
          use the typesym's name instead if it exists (if it doesn't, such as
          for anynonymous procedure types in macpas/iso mode, then there cannot
          be any recursive references to it either) }
        if (pointeddef.typ<>procvardef) or
           not assigned(pointeddef.typesym) then
          GetTypeName:='^'+pointeddef.typename
        else
          GetTypeName:='^'+pointeddef.typesym.realname;
      end;


    function tpointerdef.pointer_arithmetic_int_type:tdef;
      begin
        result:=ptrsinttype;
      end;


    function tpointerdef.pointer_arithmetic_uint_type:tdef;
      begin
        result:=ptruinttype;
      end;


    function tpointerdef.pointer_subtraction_result_type:tdef;
      begin
        result:=ptrsinttype;
      end;


    function tpointerdef.compatible_with_pointerdef_size(ptr: tpointerdef): boolean;
      begin
        result:=true;
      end;


    function tpointerdef.converted_pointer_to_array_range_type:tdef;
      begin
        result:=ptrsinttype;
      end;


{****************************************************************************
                              TCLASSREFDEF
****************************************************************************}

    constructor tclassrefdef.create(def:tdef);
      begin
         inherited create(classrefdef,def);
         if df_specialization in tstoreddef(def).defoptions then
           genericdef:=cclassrefdef.create(tstoreddef(def).genericdef);
      end;


    constructor tclassrefdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(classrefdef,ppufile);
         ppuload_platform(ppufile);
      end;


    procedure tclassrefdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         writeentry(ppufile,ibclassrefdef);
      end;


    function tclassrefdef.getcopy:tstoreddef;
      begin
        if pointeddef.typ=forwarddef then
          result:=cclassrefdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=cclassrefdef.create(pointeddef);
      end;


    function tclassrefdef.GetTypeName : string;
      begin
         GetTypeName:='Class Of '+pointeddef.typename;
      end;


    function tclassrefdef.is_publishable : boolean;
      begin
         result:=true;
      end;


    function tclassrefdef.rtti_mangledname(rt: trttitype): TSymStr;
      begin
        if (tobjectdef(pointeddef).objecttype<>odt_objcclass) then
          result:=inherited rtti_mangledname(rt)
        else
          result:=tobjectdef(pointeddef).rtti_mangledname(objcmetartti);
      end;


    procedure tclassrefdef.register_created_object_type;
      begin
        tobjectdef(pointeddef).register_created_classref_type;
      end;

{***************************************************************************
                                   TSETDEF
***************************************************************************}

    constructor tsetdef.create(def: tdef; low, high: asizeint; doregister: boolean);
      var
        setallocbits: aint;
        packedsavesize: aint;
        actual_setalloc: ShortInt;
      begin
         inherited create(setdef,doregister);
         elementdef:=def;
         elementdefderef.reset;
         setmax:=high;
         actual_setalloc:=current_settings.setalloc;
{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
         if actual_setalloc=0 then
           actual_setalloc:=1;
{$endif}
         if (actual_setalloc=0) then
           begin
             setbase:=0;
             if (high<32) then
               savesize:=Sizeof(longint)
             else if (high<256) then
               savesize:=32
             else
               savesize:=(high+7) div 8
           end
         else
           begin
             setallocbits:=actual_setalloc*8;
             setbase:=low and not(setallocbits-1);
             packedsavesize:=actual_setalloc*((((high+setallocbits)-setbase)) DIV setallocbits);
             savesize:=packedsavesize;
{$if not defined(cpu8bitalu) and not defined(cpu16bitalu)}
             if savesize=3 then
               savesize:=4;
{$endif}
           end;
      end;


    constructor tsetdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(setdef,ppufile);
         ppufile.getderef(elementdefderef);
         savesize:=ppufile.getasizeint;
         setbase:=ppufile.getasizeint;
         setmax:=ppufile.getasizeint;
         ppuload_platform(ppufile);
      end;


    function tsetdef.getcopy : tstoreddef;
      begin
        result:=csetdef.create(elementdef,setbase,setmax,true);
        { the copy might have been created with a different setalloc setting }
        tsetdef(result).savesize:=savesize;
      end;


    procedure tsetdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(elementdefderef);
         ppufile.putasizeint(savesize);
         ppufile.putasizeint(setbase);
         ppufile.putasizeint(setmax);
         writeentry(ppufile,ibsetdef);
      end;


    procedure tsetdef.buildderef;
      begin
        inherited buildderef;
        elementdefderef.build(elementdef);
      end;


    procedure tsetdef.deref;
      begin
        inherited deref;
        elementdef:=tdef(elementdefderef.resolve);
      end;


    function tsetdef.is_publishable : boolean;
      begin
         is_publishable:=savesize in [1,2,4];
      end;

    function tsetdef.alignment: shortint;
      begin
        Result:=inherited;
        if result>sizeof(aint) then
          result:=sizeof(aint);
      end;


    function tsetdef.GetTypeName : string;
      begin
         if assigned(elementdef) then
          GetTypeName:='Set Of '+elementdef.typename
         else
          GetTypeName:='Empty Set';
      end;


{***************************************************************************
                                 TFORMALDEF
***************************************************************************}

    constructor tformaldef.create(Atyped:boolean);
      begin
         inherited create(formaldef,true);
         typed:=Atyped;
         savesize:=0;
      end;


    constructor tformaldef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(formaldef,ppufile);
         typed:=boolean(ppufile.getbyte);
         savesize:=0;
         ppuload_platform(ppufile);
      end;


    procedure tformaldef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(typed));
         writeentry(ppufile,ibformaldef);
      end;


    function tformaldef.GetTypeName : string;
      begin
         if typed then
           GetTypeName:='<Typed formal type>'
         else
           GetTypeName:='<Formal type>';
      end;


{***************************************************************************
                           TARRAYDEF
***************************************************************************}

    constructor tarraydef.create(l, h: asizeint; def: tdef);
      begin
         inherited create(arraydef,true);
         lowrange:=l;
         highrange:=h;
         rangedef:=def;
         rangedefderef.reset;
         _elementdef:=nil;
         _elementdefderef.reset;
         arrayoptions:=[];
         symtable:=tarraysymtable.create(self);
      end;

    constructor tarraydef.create_vector(l ,h: asizeint; def: tdef);
      begin
        self.create(l,h,def);
        include(arrayoptions,ado_IsVector);
      end;


    constructor tarraydef.create_openarray;
      begin
        self.create(0,-1,sizesinttype)
      end;


    class function tarraydef.getreusable_intern(def: tdef; elems: asizeint; const options: tarraydefoptions): tarraydef;
      var
        res: PHashSetItem;
        oldsymtablestack: tsymtablestack;
        arrdesc: packed record
          def: tdef;
          elecount: asizeint;
          options: tarraydefoptions
        end;
      begin
        if not assigned(current_module) then
          internalerror(2011081301);
        arrdesc.def:=def;
        arrdesc.elecount:=elems;
        arrdesc.options:=options;
        res:=current_module.arraydefs.FindOrAdd(@arrdesc,sizeof(arrdesc));
        if not assigned(res^.Data) then
          begin
            { since these pointerdefs can be reused anywhere in the current
              unit, add them to the global/staticsymtable (or local symtable
              if they're a local def, because otherwise they'll be saved
              to the ppu referencing a local symtable entry that doesn't
              exist in the ppu) }
            oldsymtablestack:=symtablestack;
            { do not simply push/pop current_module.localsymtable, because
              that can have side-effects (e.g., it removes helpers) }
            symtablestack:=nil;
            result:=carraydef.create(0,elems-1,sizesinttype);
            result.elementdef:=def;
            result.arrayoptions:=options;
            setup_reusable_def(def,result,res,oldsymtablestack);
            { res^.Data may still be nil -> don't overwrite result }
            exit;
          end;
        result:=tarraydef(res^.Data);
      end;


    class function tarraydef.getreusable(def: tdef; elems: asizeint): tarraydef;
      begin
        result:=getreusable_intern(def,elems,[]);
      end;


    class function tarraydef.getreusable_vector(def: tdef; elems: asizeint): tarraydef;
      begin
        result:=getreusable_intern(def,elems,[ado_IsVector]);
      end;


    class function tarraydef.getreusable_no_free(def: tdef; elems: asizeint): tarraydef;
      begin
        result:=getreusable(def,elems);
        if not result.is_registered then
          include(result.defoptions,df_not_registered_no_free);
      end;

    class function tarraydef.getreusable_no_free_vector(def: tdef; elems: asizeint): tarraydef;
      begin
        result:=getreusable_vector(def,elems);
        if not result.is_registered then
          include(result.defoptions,df_not_registered_no_free);
      end;


    destructor tarraydef.destroy;
      begin
        symtable.free;
        symtable:=nil;
        inherited;
      end;


    constructor tarraydef.create_from_pointer(def:tpointerdef);
      begin
         { divide by the element size and do -1 so the array will have a valid size,
           further, the element size might be 0 e.g. for empty records, so use max(...,1)
           to avoid a division by zero }
         self.create(0,(high(asizeint) div max(def.pointeddef.size,1))-1,
           def.converted_pointer_to_array_range_type);
         arrayoptions:=[ado_IsConvertedPointer];
         setelementdef(def.pointeddef);
      end;


    constructor tarraydef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(arraydef,ppufile);
         { the addresses are calculated later }
         ppufile.getderef(_elementdefderef);
         ppufile.getderef(rangedefderef);
         lowrange:=ppufile.getasizeint;
         highrange:=ppufile.getasizeint;
         ppufile.getset(tppuset1(arrayoptions));
         ppuload_platform(ppufile);
         symtable:=tarraysymtable.create(self);
         tarraysymtable(symtable).ppuload(ppufile)
      end;


    function tarraydef.getcopy : tstoreddef;
      begin
        result:=carraydef.create(lowrange,highrange,rangedef);
        tarraydef(result).arrayoptions:=arrayoptions;
        tarraydef(result)._elementdef:=_elementdef;
      end;


    procedure tarraydef.buildderef;
      begin
        inherited buildderef;
        tarraysymtable(symtable).buildderef;
        _elementdefderef.build(_elementdef);
        rangedefderef.build(rangedef);
      end;


    procedure tarraydef.deref;
      begin
        inherited deref;
        tarraysymtable(symtable).deref(false);
        _elementdef:=tdef(_elementdefderef.resolve);
        rangedef:=tdef(rangedefderef.resolve);
      end;


    procedure tarraydef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(_elementdefderef);
         ppufile.putderef(rangedefderef);
         ppufile.putasizeint(lowrange);
         ppufile.putasizeint(highrange);
         ppufile.putset(tppuset1(arrayoptions));
         writeentry(ppufile,ibarraydef);
         tarraysymtable(symtable).ppuwrite(ppufile);
      end;


    function tarraydef.elesize : asizeint;
      begin
        if (ado_IsBitPacked in arrayoptions) then
          internalerror(2006080101);
        if assigned(_elementdef) then
          result:=_elementdef.size
        else
          result:=0;
      end;


    function tarraydef.elepackedbitsize : asizeint;
      begin
        if not(ado_IsBitPacked in arrayoptions) then
          internalerror(2006080102);
	if assigned(_elementdef) then
          result:=_elementdef.packedbitsize
	else
	  result:=0;
      end;


    function tarraydef.elecount : asizeuint;
      var
        qhigh,qlow : qword;
      begin
        if ado_IsDynamicArray in arrayoptions then
          begin
            result:=0;
            exit;
          end;
        if (highrange>0) and (lowrange<0) then
          begin
            qhigh:=highrange;
            if lowrange=low(asizeint) then
              qlow:=high(asizeint) + 1
            else
              qlow:=qword(-lowrange);
            { prevent overflow, return 0 to indicate overflow }
            if qhigh+qlow>qword(high(asizeint)-1) then
              result:=0
            else
              result:=qhigh+qlow+1;
          end
        else
          result:=int64(highrange)-lowrange+1;
      end;


    function tarraydef.size : asizeint;
      var
        cachedelecount : asizeuint;
        cachedelesize : asizeint;
      begin
        if ado_IsDynamicArray in arrayoptions then
          begin
            size:=voidpointertype.size;
            exit;
          end;

        { Tarraydef.size may never be called for an open array! }
        if (highrange=-1) and (lowrange=0) then
          internalerror(99080501);
        if not (ado_IsBitPacked in arrayoptions) then
          cachedelesize:=elesize
        else
          cachedelesize := elepackedbitsize;
        cachedelecount:=elecount;

        if (cachedelesize = 0) then
          begin
            size := 0;
            exit;
          end;

        if (cachedelecount = 0) then
          begin
            size := -1;
            exit;
          end;

        { prevent overflow, return -1 to indicate overflow }
        { also make sure we don't need 64/128 bit arithmetic to calculate offsets }
        if (cachedelecount > asizeuint(high(asizeint))) or
           ((high(asizeint) div cachedelesize) < asizeint(cachedelecount)) or
           { also lowrange*elesize must be < high(asizeint) to prevent overflow when
             accessing the array, see ncgmem (PFV) }
           ((high(asizeint) div cachedelesize) < abs(lowrange)) then
          begin
            result:=-1;
            exit;
          end;

        result:=cachedelesize*asizeint(cachedelecount);
        if (ado_IsBitPacked in arrayoptions) then
          { can't just add 7 and divide by 8, because that may overflow }
          result:=result div 8 + ord((result mod 8)<>0);
{$ifdef cpu16bitaddr}
        if result>65535 then
          begin
            result:=-1;
            exit;
          end;
{$endif cpu16bitaddr}
      end;


    procedure tarraydef.setelementdef(def:tdef);
      begin
        _elementdef:=def;
        if not(
               (ado_IsDynamicArray in arrayoptions) or
               (ado_IsConvertedPointer in arrayoptions) or
               (ado_IsConstructor in arrayoptions) or
               (highrange<lowrange)
	      ) and
           (size=-1) then
          Message(sym_e_segment_too_large);
      end;


    function tarraydef.alignment : shortint;
      begin
        if ado_IsVector in arrayoptions then
           alignment:=size
         { alignment of dyn. arrays doesn't depend on the element size }
         else if ado_IsDynamicArray in arrayoptions then
           alignment:=voidpointertype.alignment
         { alignment is the target alignment for the used load size }
         else if (ado_IsBitPacked in arrayoptions) and
            (elementdef.typ in [enumdef,orddef]) then
           alignment:=cgsize_orddef(int_cgsize(packedbitsloadsize(elepackedbitsize))).alignment
         { alignment is the alignment of the elements }
         else
           alignment:=elementdef.alignment
      end;


    function tarraydef.needs_inittable : boolean;
      begin
         needs_inittable:=(ado_IsDynamicArray in arrayoptions) or elementdef.needs_inittable;
      end;

    function tarraydef.needs_separate_initrtti : boolean;
      begin
        if ado_IsBitPacked in arrayoptions then
          result:=false
        else
          result:=elementdef.needs_separate_initrtti;
      end;

    function tarraydef.GetTypeName : string;
      begin
         if (ado_IsConstString in arrayoptions) then
           result:='Constant String'
         else if (ado_isarrayofconst in arrayoptions) or
                 (ado_isConstructor in arrayoptions) then
           begin
             if (ado_isvariant in arrayoptions) or ((highrange=-1) and (lowrange=0)) then
               GetTypeName:='Array Of Const'
             else
               GetTypeName:='{Array Of Const/Constant Open} Array of '+elementdef.typename;
           end
         else if (ado_IsDynamicArray in arrayoptions) then
           GetTypeName:='{Dynamic} Array Of '+elementdef.typename
         else if ((highrange=-1) and (lowrange=0)) then
           GetTypeName:='{Open} Array Of '+elementdef.typename
         else
           begin
              result := '';
              if (ado_IsBitPacked in arrayoptions) then
                result:='BitPacked ';
              if rangedef.typ=enumdef then
                result:=result+'Array['+rangedef.typename+'] Of '+elementdef.typename
              else
                result:=result+'Array['+tostr(lowrange)+'..'+
                  tostr(highrange)+'] Of '+elementdef.typename
           end;
      end;


    function tarraydef.getmangledparaname : TSymStr;
      begin
         if ado_isarrayofconst in arrayoptions then
          getmangledparaname:='array_of_const'
         else
          if ((highrange=-1) and (lowrange=0)) then
           getmangledparaname:='array_of_'+elementdef.mangledparaname
         else
          internalerror(200204176);
      end;


    function tarraydef.is_publishable : boolean;
      begin
        Result:=ado_IsDynamicArray in arrayoptions;
      end;


    function tarraydef.is_hwvector: boolean;
      begin
        result:=ado_IsVector in arrayoptions;
      end;

{***************************************************************************
                              tabstractrecorddef
***************************************************************************}

    constructor tabstractrecorddef.create(const n:string; dt:tdeftyp;doregister:boolean);
      begin
        inherited create(dt,doregister);
        objname:=stringdup(upper(n));
        objrealname:=stringdup(n);
        objectoptions:=[];
        if assigned(current_module.namespace) then
          begin
            import_lib:=stringdup(current_module.namespace^);
            replace(import_lib^,'.','/');
          end;
      end;

    constructor tabstractrecorddef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
        inherited ppuload(dt,ppufile);
        objrealname:=ppufile.getpshortstring;
        objname:=stringdup(upper(objrealname^));
        import_lib:=ppufile.getpshortstring;
        { only used for external C++ classes and Java classes/records }
        if (import_lib^='') then
          stringdispose(import_lib);
        ppufile.getset(tppuset4(objectoptions));
      end;

    procedure tabstractrecorddef.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putstring(objrealname^);
        if assigned(import_lib) then
          ppufile.putstring(import_lib^)
        else
          ppufile.putstring('');
        ppufile.putset(tppuset4(objectoptions));
      end;

    destructor tabstractrecorddef.destroy;
      begin
        stringdispose(objname);
        stringdispose(objrealname);
        stringdispose(import_lib);
        tcinitcode.free;
        inherited destroy;
      end;


    procedure tabstractrecorddef.buildderefimpl;
      begin
         inherited buildderefimpl;
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).buildderefimpl;
      end;


    procedure tabstractrecorddef.derefimpl;
      begin
        inherited derefimpl;
        if not (df_copied_def in defoptions) then
          tstoredsymtable(symtable).derefimpl(false);
      end;


    procedure tabstractrecorddef.check_forwards;
      begin
        { the defs of a copied def are defined for the original type only }
        if not(df_copied_def in defoptions) then
          tstoredsymtable(symtable).check_forwards;
      end;

    function tabstractrecorddef.find_procdef_bytype(pt:tproctypeoption): tprocdef;
      var
        i: longint;
        sym: tsym;
      begin
        for i:=0 to symtable.SymList.Count-1 do
          begin
            sym:=tsym(symtable.SymList[i]);
            if sym.typ=procsym then
              begin
                result:=tprocsym(sym).find_procdef_bytype(pt);
                if assigned(result) then
                  exit;
              end;
          end;
          result:=nil;
      end;

    function tabstractrecorddef.GetSymtable(t:tGetSymtable):TSymtable;
      begin
        if t=gs_record then
          GetSymtable:=symtable
        else
          GetSymtable:=nil;
      end;


    function tabstractrecorddef.is_packed:boolean;
      begin
        result:=tabstractrecordsymtable(symtable).is_packed;
      end;

    function tabstractrecorddef.RttiName: string;

        function generate_full_paramname(maxlength:longint):string;
          const
            commacount : array[boolean] of longint = (0,1);
          var
            fullparas,
            paramname : ansistring;
            module : tmodule;
            sym : ttypesym;
            i : longint;
          begin
            { we want at least enough space for an ellipsis }
            if maxlength<3 then
              internalerror(2014121203);
            fullparas:='';
            for i:=0 to genericparas.count-1 do
              begin
                sym:=ttypesym(genericparas[i]);
                module:=find_module_from_symtable(sym.owner);
                if not assigned(module) then
                  internalerror(2014121202);
                paramname:=module.realmodulename^;
                if sym.typedef.typ in [objectdef,recorddef] then
                  paramname:=paramname+'.'+tabstractrecorddef(sym.typedef).rttiname
                else
                  paramname:=paramname+'.'+sym.typedef.typename;
                if length(fullparas)+commacount[i>0]+length(paramname)>maxlength then
                  begin
                    if i>0 then
                      fullparas:=fullparas+',...'
                    else
                      fullparas:=fullparas+'...';
                    break;
                  end;
                { could we fit an ellipsis after this parameter if it should be too long? }
                if (maxlength-(length(fullparas)+commacount[i>0]+length(paramname))<4) and (i<genericparas.count-1) then
                  begin
                    { then omit already this parameter }
                    if i>0 then
                      fullparas:=fullparas+',...'
                    else
                      fullparas:=fullparas+'...';
                    break;
                  end;
                if i>0 then
                  fullparas:=fullparas+',';
                fullparas:=fullparas+paramname;
              end;
            result:=fullparas;
          end;

      var
        nongeneric,
        basename : string;
        i,
        remlength,
        paramcount,
        crcidx : longint;
      begin
        if rttistring='' then
          begin
            if is_specialization then
              begin
                rttistring:=OwnerHierarchyName;
                { there should be two $ characters, one before the CRC and one before the count }
                crcidx:=-1;
                for i:=length(objrealname^) downto 1 do
                  if objrealname^[i]='$' then
                    begin
                      crcidx:=i;
                      break;
                    end;
                if crcidx<0 then
                  internalerror(2014121201);
                basename:=copy(objrealname^,1,crcidx-1);
                split_generic_name(basename,nongeneric,paramcount);
                rttistring:=rttistring+nongeneric+'<';
                remlength:=255-length(rttistring)-1;
                if remlength<4 then
                  rttistring:=rttistring+'>'
                else
                  rttistring:=rttistring+generate_full_paramname(remlength)+'>';
              end
            else
              if is_generic then
                begin
                  rttistring:=OwnerHierarchyName;
                  split_generic_name(objrealname^,nongeneric,paramcount);
                  rttistring:=rttistring+nongeneric+'<';
                  { we don't want any ',' if there is only one parameter }
                  for i:=0 to paramcount-0 do
                    rttistring:=rttistring+',';
                  rttistring:=rttistring+'>';
                end
              else
                rttistring:=OwnerHierarchyName+objrealname^;
          end;
        result:=rttistring;
      end;

    function tabstractrecorddef.search_enumerator_get: tprocdef;
      var
        sym : tsym;
        i : integer;
        pd : tprocdef;
        hashedid : THashedIDString;
      begin
        result:=nil;
        hashedid.id:='GETENUMERATOR';
        sym:=tsym(symtable.FindWithHash(hashedid));
        if assigned(sym) and (sym.typ=procsym) then
          begin
            for i := 0 to Tprocsym(sym).ProcdefList.Count - 1 do
            begin
              pd := tprocdef(Tprocsym(sym).ProcdefList[i]);
              if (pd.proctypeoption = potype_function) and
                 (is_class_or_interface_or_object(pd.returndef) or is_record(pd.returndef)) and
                 (pd.visibility >= vis_public) then
              begin
                result:=pd;
                exit;
              end;
            end;
          end;
      end;

    function tabstractrecorddef.search_enumerator_move: tprocdef;
      var
        sym : tsym;
        i : integer;
        pd : tprocdef;
        hashedid : THashedIDString;
      begin
        result:=nil;
        // first search for po_enumerator_movenext method modifier
        // then search for public function MoveNext: Boolean
        for i:=0 to symtable.SymList.Count-1 do
          begin
            sym:=TSym(symtable.SymList[i]);
            if (sym.typ=procsym) then
            begin
              pd:=Tprocsym(sym).find_procdef_byoptions([po_enumerator_movenext]);
              if assigned(pd) then
                begin
                  result:=pd;
                  exit;
                end;
            end;
          end;
        hashedid.id:='MOVENEXT';
        sym:=tsym(symtable.FindWithHash(hashedid));
        if assigned(sym) and (sym.typ=procsym) then
          begin
            for i:=0 to Tprocsym(sym).ProcdefList.Count-1 do
            begin
              pd := tprocdef(Tprocsym(sym).ProcdefList[i]);
              if (pd.proctypeoption = potype_function) and
                 is_boolean(pd.returndef) and
                 (pd.minparacount = 0) and
                 (pd.visibility >= vis_public) then
              begin
                result:=pd;
                exit;
              end;
            end;
          end;
      end;

    function tabstractrecorddef.search_enumerator_current: tsym;
      var
        sym: tsym;
        i: integer;
        hashedid : THashedIDString;
      begin
        result:=nil;
        // first search for ppo_enumerator_current property modifier
        // then search for public property Current
        for i:=0 to symtable.SymList.Count-1 do
          begin
            sym:=TSym(symtable.SymList[i]);
            if (sym.typ=propertysym) and (ppo_enumerator_current in tpropertysym(sym).propoptions) then
            begin
              result:=sym;
              exit;
            end;
          end;
        hashedid.id:='CURRENT';
        sym:=tsym(symtable.FindWithHash(hashedid));
        if assigned(sym) and (sym.typ=propertysym) and
           (sym.visibility >= vis_public) and not tpropertysym(sym).propaccesslist[palt_read].empty then
          begin
            result:=sym;
            exit;
          end;
      end;


    function tabstractrecorddef.jvm_full_typename(with_package_name: boolean): string;
      var
        st: tsymtable;
        enclosingdef: tdef;
      begin
        if typ=objectdef then
          result:=tobjectdef(self).objextname^
        else if assigned(typesym) then
          result:=typesym.realname
        { have to generate anonymous nested type in current unit/class/record }
        else
          internalerror(2011032601);

        { in case of specializations, add some extras to prevent name conflicts
          with nested classes }
        if df_specialization in defoptions then
          result:='$'+result+'$specialization$';

        st:=owner;
        while assigned(st) and
              (st.symtabletype in [objectsymtable,recordsymtable,localsymtable]) do
          begin
            { nested classes are named as "OuterClass$InnerClass" }
            enclosingdef:=tdef(st.defowner);
            if enclosingdef.typ=procdef then
              result:=result+tprocdef(enclosingdef).procsym.realname+'$$'+tostr(tprocdef(enclosingdef).procsym.symid)+'$'
            else if enclosingdef.typ=objectdef then
              result:=tobjectdef(enclosingdef).objextname^+'$'+result
            else if assigned(enclosingdef.typesym) then
              result:=enclosingdef.typesym.realname+'$'+result
            else
              internalerror(2011060305);
            st:=enclosingdef.owner;
          end;

        if with_package_name and
           assigned(import_lib) then
          result:=import_lib^+'/'+result;
      end;


    function tabstractrecorddef.contains_float_field: boolean;
      var
        i : longint;
      begin
        result:=true;
        for i:=0 to symtable.symlist.count-1 do
          begin
            if (tsym(symtable.symlist[i]).typ<>fieldvarsym) or
               (sp_static in tsym(symtable.symlist[i]).symoptions) then
              continue;
            if assigned(tfieldvarsym(symtable.symlist[i]).vardef) then
              begin
                if tstoreddef(tfieldvarsym(symtable.symlist[i]).vardef).is_fpuregable then
                  exit;
                { search recursively }
                if (tstoreddef(tfieldvarsym(symtable.symlist[i]).vardef).typ=recorddef) and
                  (tabstractrecorddef(tfieldvarsym(symtable.symlist[i]).vardef).contains_float_field) then
                  exit;
              end;
          end;
        result:=false;
      end;


    function tabstractrecorddef.contains_cross_aword_field: boolean;
      var
        i : longint;
        foffset, fsize: aword;
      begin
        result:=true;
        for i:=0 to symtable.symlist.count-1 do
          begin
            if (tsym(symtable.symlist[i]).typ<>fieldvarsym) or
               (sp_static in tsym(symtable.symlist[i]).symoptions) then
              continue;
            if assigned(tfieldvarsym(symtable.symlist[i]).vardef) then
              begin
                if is_packed then
                  begin
                    foffset:=tfieldvarsym(symtable.symlist[i]).fieldoffset;
                    fsize:=tfieldvarsym(symtable.symlist[i]).vardef.packedbitsize;
                  end
                else
                  begin
                    foffset:=tfieldvarsym(symtable.symlist[i]).fieldoffset*8;
                    fsize:=tfieldvarsym(symtable.symlist[i]).vardef.size*8;
                  end;
                if (fsize>0) and ((foffset div (sizeof(aword)*8)) <> ((foffset+fsize-1) div (sizeof(aword)*8))) then
                  exit;
                { search recursively }
                if (tstoreddef(tfieldvarsym(symtable.symlist[i]).vardef).typ=recorddef) and
                  (tabstractrecorddef(tfieldvarsym(symtable.symlist[i]).vardef).contains_cross_aword_field) then
                  exit;
              end;
          end;
        result:=false;
      end;


{***************************************************************************
                                  trecorddef
***************************************************************************}

    constructor trecorddef.create(const n:string; p:TSymtable);
      begin
         inherited create(n,recorddef,true);
         symtable:=p;
         { we can own the symtable only if nobody else owns a copy so far }
         if symtable.refcount=1 then
           symtable.defowner:=self;
         isunion:=false;
         cloneddefderef.reset;
      end;


    constructor trecorddef.create_global_internal(const n: string; packrecords, recordalignmin: shortint);
      var
        name : string;
        pname : pshortstring;
        oldsymtablestack: tsymtablestack;
        where : tsymtable;
        ts: ttypesym;
      begin
        { construct name }
        if n<>'' then
          pname:=@n
        else
          begin
            init_defid;
            name:='$InternalRec'+unique_id_str;
            pname:=@name;
          end;
        oldsymtablestack:=symtablestack;
        { do not simply push/pop current_module.localsymtable, because
          that can have side-effects (e.g., it removes helpers) }
        symtablestack:=nil;

        symtable:=trecordsymtable.create(pname^,packrecords,recordalignmin);
        symtable.defowner:=self;
        isunion:=false;
        inherited create(pname^,recorddef,true);
        where:=current_module.localsymtable;
        if not assigned(where) then
          where:=current_module.globalsymtable;
        where.insertdef(self);
        { if we specified a name, then we'll probably want to look up the
          type again by name too -> create typesym }
        if n<>'' then
          begin
            ts:=ctypesym.create(n,self);
            { avoid hints about unused types (these may only be used for
              typed constant data) }
            ts.increfcount;
            where.insert(ts);
          end;
        symtablestack:=oldsymtablestack;
        { don't create RTTI for internal types, these are not exported }
        defstates:=defstates+[ds_rtti_table_written,ds_init_table_written];
        include(defoptions,df_internal);
      end;


    function trecorddef.add_field_by_def(const optionalname: TIDString; def: tdef): tsym;
      var
        sym: tfieldvarsym;
        name: TIDString;
        pname: ^TIDString;
      begin
        if optionalname='' then
          begin
            name:='$f'+tostr(trecordsymtable(symtable).symlist.count);
            pname:=@name
          end
        else
          pname:=@optionalname;
        sym:=cfieldvarsym.create(pname^,vs_value,def,[]);
        symtable.insert(sym);
        trecordsymtable(symtable).addfield(sym,vis_hidden);
        result:=sym;
      end;


    procedure trecorddef.add_fields_from_deflist(fieldtypes: tfplist);
      var
        i: longint;
      begin
        for i:=0 to fieldtypes.count-1 do
          add_field_by_def('',tdef(fieldtypes[i]));
      end;


    constructor trecorddef.ppuload(ppufile:tcompilerppufile);

      procedure readvariantrecdesc(var variantrecdesc : pvariantrecdesc);
        var
          i,j : longint;
        begin
         if ppufile.getbyte=1 then
           begin
             new(variantrecdesc);
             ppufile.getderef(variantrecdesc^.variantselectorderef);
             SetLength(variantrecdesc^.branches,ppufile.getasizeint);
             for i:=0 to high(variantrecdesc^.branches) do
               begin
                 SetLength(variantrecdesc^.branches[i].values,ppufile.getasizeint);
                 for j:=0 to high(variantrecdesc^.branches[i].values) do
                   variantrecdesc^.branches[i].values[j]:=ppufile.getexprint;
                 readvariantrecdesc(variantrecdesc^.branches[i].nestedvariant);
               end;
           end
         else
           variantrecdesc:=nil;
        end;

      begin
         inherited ppuload(recorddef,ppufile);
         if df_copied_def in defoptions then
           begin
             ppufile.getderef(cloneddefderef);
             ppuload_platform(ppufile);
           end
         else
           begin
             symtable:=trecordsymtable.create(objrealname^,0,0);
             trecordsymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).padalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).usefieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignmin:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).datasize:=ppufile.getasizeint;
             trecordsymtable(symtable).paddingsize:=ppufile.getword;
             ppufile.getset(tppuset1(trecordsymtable(symtable).managementoperators));
             { position of ppuload_platform call must correspond
               to position of writeentry in ppuwrite method }
             ppuload_platform(ppufile);
             trecordsymtable(symtable).ppuload(ppufile);
             { the variantrecdesc is needed only for iso-like new statements new(prec,1,2,3 ...);
               but because iso mode supports no units, there is no need to store the variantrecdesc
               in the ppu
             }
             // readvariantrecdesc(variantrecdesc);
             { requires usefieldalignment to be set }
             symtable.defowner:=self;
           end;
         isunion:=false;
      end;


    destructor trecorddef.destroy;

      procedure free_variantrecdesc(var variantrecdesc : pvariantrecdesc);
        var
          i : longint;
        begin
         while assigned(variantrecdesc) do
           begin
             for i:=0 to high(variantrecdesc^.branches) do
               begin
                 free_variantrecdesc(variantrecdesc^.branches[i].nestedvariant);
                 SetLength(variantrecdesc^.branches[i].values,0);
               end;
             SetLength(variantrecdesc^.branches,0);
             dispose(variantrecdesc);
             variantrecdesc:=nil;
           end;
        end;

      begin
         if assigned(variantrecdesc) then
           free_variantrecdesc(variantrecdesc);
         if assigned(symtable) then
           begin
             symtable.free;
             symtable:=nil;
           end;
         inherited destroy;
      end;


    function trecorddef.getcopy : tstoreddef;
      begin
        result:=crecorddef.create(objrealname^,symtable.getcopy);
        trecorddef(result).isunion:=isunion;
        include(trecorddef(result).defoptions,df_copied_def);
        if assigned(tcinitcode) then
          trecorddef(result).tcinitcode:=tcinitcode.getcopy;
         if assigned(import_lib) then
           trecorddef(result).import_lib:=stringdup(import_lib^);
      end;


    function trecorddef.needs_inittable : boolean;
      begin
        { each record with managed field or with any management operator needs
          init table }
        needs_inittable:=(trecordsymtable(symtable).managementoperators<>[]) or
          trecordsymtable(symtable).needs_init_final
      end;

    function trecorddef.needs_separate_initrtti : boolean;
      begin
        result:=true;
      end;


    function trecorddef.has_non_trivial_init_child(check_parent:boolean):boolean;
      begin
        result:=trecordsymtable(symtable).has_non_trivial_init;
      end;


    procedure trecorddef.buildderef;
      begin
         inherited buildderef;
         if df_copied_def in defoptions then
           cloneddefderef.build(symtable.defowner)
         else
           tstoredsymtable(symtable).buildderef;
      end;


    procedure trecorddef.deref;
      begin
         inherited deref;
         { now dereference the definitions }
         if df_copied_def in defoptions then
           begin
             cloneddef:=trecorddef(cloneddefderef.resolve);
             symtable:=cloneddef.symtable.getcopy;
           end
         else
           tstoredsymtable(symtable).deref(false);

         { internal types, only load from the system unit }
         if assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           begin
             { TGUID  }
             if not assigned(rec_tguid) and
                (upper(typename)='TGUID') then
               rec_tguid:=self
             { JMP_BUF }
             else if not assigned(rec_jmp_buf) and
                (upper(typename)='JMP_BUF') then
               rec_jmp_buf:=self
             else if not assigned(rec_exceptaddr) and
                (upper(typename)='TEXCEPTADDR') then
               rec_exceptaddr:=self;
           end;
      end;


    procedure trecorddef.ppuwrite(ppufile:tcompilerppufile);

      procedure writevariantrecdesc(variantrecdesc : pvariantrecdesc);
        var
          i,j : longint;
        begin
         if assigned(variantrecdesc) then
           begin
             ppufile.putbyte(1);
             ppufile.putderef(variantrecdesc^.variantselectorderef);
             ppufile.putasizeint(length(variantrecdesc^.branches));
             for i:=0 to high(variantrecdesc^.branches) do
               begin
                 ppufile.putasizeint(length(variantrecdesc^.branches[i].values));
                 for j:=0 to high(variantrecdesc^.branches[i].values) do
                   ppufile.putexprint(variantrecdesc^.branches[i].values[j]);
                 writevariantrecdesc(variantrecdesc^.branches[i].nestedvariant);
               end;
           end
         else
           ppufile.putbyte(0);
        end;

      begin
         inherited ppuwrite(ppufile);
         if df_copied_def in defoptions then
           ppufile.putderef(cloneddefderef)
         else
           begin
             ppufile.putbyte(byte(trecordsymtable(symtable).fieldalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).recordalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).padalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).usefieldalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).recordalignmin));
             ppufile.putasizeint(trecordsymtable(symtable).datasize);
             ppufile.putword(trecordsymtable(symtable).paddingsize);
             ppufile.putset(tppuset1(trecordsymtable(symtable).managementoperators));
             { the variantrecdesc is needed only for iso-like new statements new(prec,1,2,3 ...);
               but because iso mode supports no units, there is no need to store the variantrecdesc
               in the ppu
             }
             // writevariantrecdesc(variantrecdesc);
           end;

         writeentry(ppufile,ibrecorddef);

         if not(df_copied_def in defoptions) then
           trecordsymtable(symtable).ppuwrite(ppufile);
      end;


    function trecorddef.size:asizeint;
      begin
        result:=trecordsymtable(symtable).datasize;
      end;


    function trecorddef.alignment:shortint;
      begin
        alignment:=trecordsymtable(symtable).recordalignment;
      end;


    function trecorddef.padalignment:shortint;
      begin
        padalignment := trecordsymtable(symtable).padalignment;
      end;


    function trecorddef.GetTypeName : string;
      begin
         GetTypeName:='<record type>'
      end;


{***************************************************************************
                       TABSTRACTPROCDEF
***************************************************************************}

    constructor tabstractprocdef.create(dt:tdeftyp;level:byte;doregister:boolean);
      begin
         inherited create(dt,doregister);
         parast:=tparasymtable.create(self,level);
         paras:=nil;
         minparacount:=0;
         maxparacount:=0;
         proctypeoption:=potype_none;
         proccalloption:=pocall_none;
         procoptions:=[];
         returndef:=voidtype;
         returndefderef.reset;
         savesize:=sizeof(pint);
         callerargareasize:=0;
         calleeargareasize:=0;
         has_paraloc_info:=callnoside;
         funcretloc[callerside].init;
         funcretloc[calleeside].init;
         check_mark_as_nested;
      end;


    destructor tabstractprocdef.destroy;
      begin
         if assigned(paras) then
           begin
{$ifdef MEMDEBUG}
             memprocpara.start;
{$endif MEMDEBUG}
             paras.free;
             paras:=nil;
{$ifdef MEMDEBUG}
             memprocpara.stop;
{$endif MEMDEBUG}
          end;
         if assigned(parast) then
          begin
{$ifdef MEMDEBUG}
            memprocparast.start;
{$endif MEMDEBUG}
            parast.free;
            parast:=nil;
{$ifdef MEMDEBUG}
            memprocparast.stop;
{$endif MEMDEBUG}
          end;
         funcretloc[callerside].done;
         funcretloc[calleeside].done;
         inherited destroy;
      end;


    procedure tabstractprocdef.count_para(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ<>paravarsym) then
          exit;
        inc(plongint(arg)^);
        if not(vo_is_hidden_para in tparavarsym(p).varoptions) then
         begin
           if not assigned(tparavarsym(p).defaultconstsym) then
             inc(minparacount);
           inc(maxparacount);
         end;
      end;


    procedure tabstractprocdef.insert_para(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ<>paravarsym) then
          exit;
        paras.add(p);
      end;


    procedure tabstractprocdef.calcparas;
      var
        paracount : longint;
      begin
        { This can already be assigned when
          we need to reresolve this unit (PFV) }
        if assigned(paras) then
          paras.free;
        paras:=tparalist.create(false);
        paracount:=0;
        minparacount:=0;
        maxparacount:=0;
        parast.SymList.ForEachCall(@count_para,@paracount);
        paras.capacity:=paracount;
        { Insert parameters in table }
        parast.SymList.ForEachCall(@insert_para,nil);
        { Order parameters }
        paras.sortparas;
      end;


    function tabstractprocdef.mangledprocparanames(oldlen : longint) : string;
      var
        crc  : dword;
        hp   : TParavarsym;
        hs   : TSymStr;
        newlen,
        i    : integer;
      begin
        result:='';
        hp:=nil;
        { add parameter types }
        for i:=0 to paras.count-1 do
         begin
           hp:=tparavarsym(paras[i]);
           if not(vo_is_hidden_para in hp.varoptions) then
             result:=result+'$'+hp.vardef.mangledparaname;
         end;
        { add resultdef, add $$ as separator to make it unique from a
          parameter separator }
        if not is_void(returndef) then
          result:=result+'$$'+returndef.mangledparaname;
        newlen:=length(result)+oldlen;
        { Replace with CRC if the parameter line is very long }
        if (newlen-oldlen>12) and
           ((newlen>100) or (newlen-oldlen>64)) then
          begin
            crc:=0;
            for i:=0 to paras.count-1 do
              begin
                hp:=tparavarsym(paras[i]);
                if not(vo_is_hidden_para in hp.varoptions) then
                  begin
                    hs:=hp.vardef.mangledparaname;
                    crc:=UpdateCrc32(crc,hs[1],length(hs));
                  end;
              end;
            if not is_void(returndef) then
              begin
                { add a little prefix so that x(integer; integer) is different from x(integer):integer }
                hs:='$$'+returndef.mangledparaname;
                crc:=UpdateCrc32(crc,hs[1],length(hs));
              end;
            result:='$crc'+hexstr(crc,8);
          end;
      end;


    procedure tabstractprocdef.buildderef;
      begin
         { released procdef? }
         if not assigned(parast) then
           exit;
         inherited buildderef;
         returndefderef.build(returndef);
         if po_explicitparaloc in procoptions then
           funcretloc[callerside].buildderef;
         { parast }
         tparasymtable(parast).buildderef;
      end;


    procedure tabstractprocdef.deref;
      begin
         inherited deref;
         returndef:=tdef(returndefderef.resolve);
         if po_explicitparaloc in procoptions then
           begin
             funcretloc[callerside].deref;
             has_paraloc_info:=callerside;
            end
         else
           begin
             { deref is called after loading from a ppu, but also after another
               unit has been reloaded/recompiled and all references must be
               re-resolved. Since the funcretloc contains a reference to a tdef,
               reset it so that we won't try to access the stale def }
             funcretloc[callerside].init;
             has_paraloc_info:=callnoside;
           end;
         { parast }
         tparasymtable(parast).deref(false);
         { recalculated parameters }
         calcparas;
      end;


    constructor tabstractprocdef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
         inherited ppuload(dt,ppufile);
         parast:=nil;
         Paras:=nil;
         minparacount:=0;
         maxparacount:=0;
         ppufile.getderef(returndefderef);
         proctypeoption:=tproctypeoption(ppufile.getbyte);
         proccalloption:=tproccalloption(ppufile.getbyte);
         ppufile.getset(tppuset8(procoptions));

         funcretloc[callerside].init;
         if po_explicitparaloc in procoptions then
           funcretloc[callerside].ppuload(ppufile);

         savesize:=sizeof(pint);
         if (po_explicitparaloc in procoptions) then
           has_paraloc_info:=callerside;
      end;


    procedure tabstractprocdef.ppuwrite(ppufile:tcompilerppufile);
      var
        oldintfcrc : boolean;
      begin
         { released procdef? }
         if not assigned(parast) then
           exit;
         inherited ppuwrite(ppufile);
         ppufile.putderef(returndefderef);
         oldintfcrc:=ppufile.do_interface_crc;
         ppufile.do_interface_crc:=false;
         ppufile.putbyte(ord(proctypeoption));
         ppufile.putbyte(ord(proccalloption));
         ppufile.putset(tppuset8(procoptions));
         ppufile.do_interface_crc:=oldintfcrc;

         if (po_explicitparaloc in procoptions) then
           funcretloc[callerside].ppuwrite(ppufile);
      end;


    function tabstractprocdef.typename_paras(pno: tprocnameoptions) : ansistring;
      var
        hs,s  : ansistring;
        hp    : TParavarsym;
        hpc   : tconstsym;
        first : boolean;
        i,j   : integer;
      begin
        s:='';
        first:=true;
        for i:=0 to paras.count-1 do
         begin
           hp:=tparavarsym(paras[i]);
           if not(vo_is_hidden_para in hp.varoptions) or
              (pno_showhidden in pno) then
            begin
               if first then
                begin
                  s:=s+'(';
                  first:=false;
                end
               else
                s:=s+';';
               if vo_is_hidden_para in hp.varoptions then
                 s:=s+'<';
               case hp.varspez of
                 vs_var :
                   s:=s+'var ';
                 vs_const :
                   s:=s+'const ';
                 vs_out :
                   s:=s+'out ';
                 vs_constref :
                   s:=s+'constref ';
                 else
                   ;
               end;
               if (pno_paranames in pno) then
                 s:=s+hp.realname+':';
               if hp.univpara then
                 s:=s+'univ ';
               if assigned(hp.vardef.typesym) then
                 begin
                   hs:=hp.vardef.typesym.realname;
                   if hs[1]<>'$' then
                     s:=s+hp.vardef.OwnerHierarchyName+hs
                   else
                     s:=s+hp.vardef.GetTypeName;
                 end
               else
                 s:=s+hp.vardef.GetTypeName;
               { default value }
               if assigned(hp.defaultconstsym) then
                begin
                  hpc:=tconstsym(hp.defaultconstsym);
                  hs:='';
                  case hpc.consttyp of
                    constwstring:
                      begin
                        if pcompilerwidestring(hpc.value.valueptr)^.len>0 then
                          begin
                            setlength(hs,pcompilerwidestring(hpc.value.valueptr)^.len);
                            for j:=0 to pcompilerwidestring(hpc.value.valueptr)^.len-1 do
                             begin
                               if (ord(pcompilerwidestring(hpc.value.valueptr)^.data[j])<127) and
                                  not(byte(pcompilerwidestring(hpc.value.valueptr)^.data[j]) in [0,10,13]) then
                                 hs[j+1]:=char(pcompilerwidestring(hpc.value.valueptr)^.data[j])
                               else
                                 hs[j+1]:='.';
                             end;
                          end;
                      end;
                    conststring,
                    constresourcestring :
                      begin
                        if hpc.value.len>0 then
                          begin
                            setLength(hs,hpc.value.len);
                            { don't write past the end of hs if the constant
                              is > 255 chars }
                            move(hpc.value.valueptr^,hs[1],length(hs));
                            { make sure that constant strings with newline chars
                              don't create a linebreak in the assembler code,
                              since comments are line-based. Also remove nulls
                              because the comments are written as a pchar. }
                            ReplaceCase(hs,#0,'.');
                            ReplaceCase(hs,#10,'.');
                            ReplaceCase(hs,#13,'.');
                          end;
                      end;
                    constreal :
                      str(pbestreal(hpc.value.valueptr)^,hs);
                    constpointer :
                      hs:=tostr(hpc.value.valueordptr);
                    constord :
                      begin
                        if is_boolean(hpc.constdef) then
                          begin
                            if hpc.value.valueord<>0 then
                             hs:='TRUE'
                            else
                             hs:='FALSE';
                          end
                        else
                          hs:=tostr(hpc.value.valueord);
                      end;
                    constnil :
                      hs:='nil';
                    constset :
                      hs:='<set>';
                    constguid:
                      hs:=guid2string(pguid(hpc.value.valueptr)^);
                    constnone:
                      internalerror(2019050704);
                  end;
                  if hs<>'' then
                   s:=s+'=`'+hs+'`';
                end;
               if vo_is_hidden_para in hp.varoptions then
                 s:=s+'>';
             end;
         end;
        if not first then
         s:=s+')';
        if (po_varargs in procoptions) then
         s:=s+';VarArgs';
        typename_paras:=s;
      end;


    function tabstractprocdef.is_methodpointer:boolean;
      begin
        result:=false;
      end;


    function tabstractprocdef.is_addressonly:boolean;
      begin
        result:=true;
      end;

    function tabstractprocdef.no_self_node: boolean;
      begin
        Result:=([po_staticmethod,po_classmethod]<=procoptions)or
                (proctypeoption in [potype_class_constructor,potype_class_destructor,potype_operator]);
      end;


    function tabstractprocdef.getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp; const paraprefix: string): tstoreddef;
      var
        j, nestinglevel: longint;
        pvs, npvs: tparavarsym;
      begin
        nestinglevel:=parast.symtablelevel;
        if newtyp=procdef then
          begin
            if (copytyp<>pc_bareproc) then
              result:=cprocdef.create(nestinglevel,true)
            else
              result:=cprocdef.create(normal_function_level,true);
            tprocdef(result).visibility:=vis_public;
          end
        else
          begin
            result:=cprocvardef.create(nestinglevel);
          end;
        tabstractprocdef(result).returndef:=returndef;
        tabstractprocdef(result).returndefderef:=returndefderef;
        pvs:=nil;
        npvs:=nil;
        for j:=0 to parast.symlist.count-1 do
          begin
            case tsym(parast.symlist[j]).typ of
              paravarsym:
                begin
                  pvs:=tparavarsym(parast.symlist[j]);
                  { in case of bare proc, don't copy self, vmt or framepointer
                    parameters }
                  if (copytyp in [pc_bareproc,pc_normal_no_hidden]) and
                     (([vo_is_self,vo_is_vmt,vo_is_parentfp,vo_is_result,vo_is_funcret]*pvs.varoptions)<>[]) then
                    continue;
                  if paraprefix='' then
                    npvs:=cparavarsym.create(pvs.realname,pvs.paranr,pvs.varspez,
                      pvs.vardef,pvs.varoptions)
                  else if not(vo_is_high_para in pvs.varoptions) then
                    npvs:=cparavarsym.create(paraprefix+pvs.realname,pvs.paranr,pvs.varspez,
                      pvs.vardef,pvs.varoptions)
                  else
                    npvs:=cparavarsym.create('$high'+paraprefix+copy(pvs.name,5,length(pvs.name)),pvs.paranr,pvs.varspez,
                      pvs.vardef,pvs.varoptions);
                  npvs.defaultconstsym:=pvs.defaultconstsym;
                  tabstractprocdef(result).parast.insert(npvs);
                end;
              constsym:
                begin
                  // ignore, reuse original constym. Should also be duplicated
                  // be safe though
                end;
              symconst.typesym:
                begin
                  // reuse original, part of generic declaration
                end
              else
                internalerror(201160604);
              end;
          end;
        tabstractprocdef(result).savesize:=savesize;

        if (typ<>procvardef) and
           (newtyp=procvardef) then
          begin
            { procvars can't be (class)constructures/destructors etc }
            if proctypeoption=potype_constructor then
              begin
                tabstractprocdef(result).returndef:=tdef(owner.defowner);
                if not(is_implicit_pointer_object_type(returndef) or
                   (returndef.typ<>objectdef)) then
                  tabstractprocdef(result).returndef:=cpointerdef.getreusable(tabstractprocdef(result).returndef);
                tabstractprocdef(result).proctypeoption:=potype_function;
              end
            else if is_void(returndef) then
              tabstractprocdef(result).proctypeoption:=potype_procedure
            else
              tabstractprocdef(result).proctypeoption:=potype_function;
          end
        else
          tabstractprocdef(result).proctypeoption:=proctypeoption;
        tabstractprocdef(result).proccalloption:=proccalloption;
        tabstractprocdef(result).procoptions:=procoptions;
        if (copytyp=pc_bareproc) then
          tabstractprocdef(result).procoptions:=tabstractprocdef(result).procoptions*[po_explicitparaloc,po_hascallingconvention,po_varargs,po_iocheck,po_has_importname,po_has_importdll];
        if newtyp=procvardef then
          tabstractprocdef(result).procoptions:=tabstractprocdef(result).procoptions-[po_has_importname,po_has_importdll];
        if copytyp=pc_address_only then
          include(tabstractprocdef(result).procoptions,po_addressonly);

        tabstractprocdef(result).callerargareasize:=callerargareasize;
        tabstractprocdef(result).calleeargareasize:=calleeargareasize;
        tabstractprocdef(result).maxparacount:=maxparacount;
        tabstractprocdef(result).minparacount:=minparacount;
        if po_explicitparaloc in procoptions then
          tabstractprocdef(result).funcretloc[callerside]:=funcretloc[callerside].getcopy;
        { recalculate parameter info }
        tabstractprocdef(result).has_paraloc_info:=callnoside;
{$ifdef m68k}
        tabstractprocdef(result).exp_funcretloc:=exp_funcretloc;
{$endif}
        if (typ=procdef) and
           (newtyp=procvardef) and
           (owner.symtabletype=ObjectSymtable) then
          include(tprocvardef(result).procoptions,po_methodpointer);
      end;


    function tabstractprocdef.compatible_with_pointerdef_size(ptr: tpointerdef): boolean;
      begin
        result:=is_addressonly;
      end;


    procedure tabstractprocdef.check_mark_as_nested;
      begin
         { nested procvars require that nested functions use the Delphi-style
           nested procedure calling convention }
         if (parast.symtablelevel>normal_function_level) and
            (m_nested_procvars in current_settings.modeswitches) then
           include(procoptions,po_delphi_nested_cc);
      end;


    procedure tabstractprocdef.init_paraloc_info(side: tcallercallee);
      begin
        if (side in [callerside,callbothsides]) and
           not(has_paraloc_info in [callerside,callbothsides]) then
          begin
            if not is_c_variadic(self) then
              callerargareasize:=paramanager.create_paraloc_info(self,callerside)
            else
              callerargareasize:=paramanager.create_varargs_paraloc_info(self,callerside,nil);
            if has_paraloc_info in [calleeside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=callerside;
          end;
        if (side in [calleeside,callbothsides]) and
           not(has_paraloc_info in [calleeside,callbothsides]) then
          begin
            if not is_c_variadic(self) then
              calleeargareasize:=paramanager.create_paraloc_info(self,calleeside)
            else
              callerargareasize:=paramanager.create_varargs_paraloc_info(self,calleeside,nil);
            if has_paraloc_info in [callerside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=calleeside;
          end;
      end;


    procedure tabstractprocdef.done_paraloc_info(side: tcallercallee);
      var
        i: longint;
      begin
        if (side in [callerside,callbothsides]) and
           (has_paraloc_info in [callerside,callbothsides]) then
          begin
            funcretloc[callerside].done;
            for i:=0 to paras.count-1 do
              tparavarsym(paras[i]).paraloc[callerside].done;
            if has_paraloc_info=callerside then
              has_paraloc_info:=callnoside
            else
              has_paraloc_info:=calleeside;
          end;

        if (side in [calleeside,callbothsides]) and
           (has_paraloc_info in [calleeside,callbothsides]) then
          begin
            funcretloc[calleeside].done;
            for i:=0 to paras.count-1 do
              tparavarsym(paras[i]).paraloc[calleeside].done;
            if has_paraloc_info=calleeside then
              has_paraloc_info:=callnoside
            else
              has_paraloc_info:=callerside;
          end;
      end;


    function tabstractprocdef.stack_tainting_parameter(side: tcallercallee): boolean;
      var
        p: tparavarsym;
        ploc: PCGParalocation;
        i: longint;
      begin
        result:=false;
        init_paraloc_info(side);
        for i:=0 to parast.SymList.Count-1 do
          if tsym(parast.SymList[i]).typ=paravarsym then
            begin
              p:=tparavarsym(parast.SymList[i]);
              { check if no parameter is located on the stack }
              if (is_open_array(p.vardef) or
                 is_array_of_const(p.vardef)) and (p.varspez=vs_value) then
                begin
                  result:=true;
                  exit;
                end;
              ploc:=p.paraloc[side].location;
              while assigned(ploc) do
                begin
                  if (ploc^.loc=LOC_REFERENCE) then
                    begin
                      result:=true;
                      exit
                    end;
                  ploc:=ploc^.next;
                end;
            end;
      end;

    function tabstractprocdef.is_pushleftright: boolean;
      begin
        result:=false;
      end;

    function tabstractprocdef.address_type: tdef;
      begin
        result:=voidcodepointertype;
      end;


    function tabstractprocdef.ofs_address_type:tdef;
      begin
        result:=address_type;
      end;


    procedure tabstractprocdef.declared_far;
      begin
        Message1(parser_w_proc_directive_ignored,'FAR');
      end;


    procedure tabstractprocdef.declared_near;
      begin
        Message1(parser_w_proc_directive_ignored,'NEAR');
      end;


    function tabstractprocdef.generate_safecall_wrapper: boolean;
      begin
{$ifdef SUPPORT_SAFECALL}
        result:=
          (proccalloption=pocall_safecall) and
          not(po_assembler in procoptions) and
          (tf_safecall_exceptions in target_info.flags);
{$else SUPPORT_SAFECALL}
        result:=false;
{$endif}
      end;


{***************************************************************************
                                  TPROCDEF
***************************************************************************}


    function tprocdef.store_localst: boolean;
      begin
        result:=has_inlininginfo or (df_generic in defoptions);
      end;


    function tprocdef.GetResultName: PShortString;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010301);
        result:=implprocdefinfo^.resultname;
      end;


    procedure tprocdef.SetResultName(AValue: PShortString);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010302);
        implprocdefinfo^.resultname:=AValue;
      end;


    function tprocdef.GetParentFPInitBlock: tnode;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010303);
        result:=implprocdefinfo^.parentfpinitblock;
      end;


    function tprocdef.GetParentFPStruct: tsym;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010304);
        result:=implprocdefinfo^.parentfpstruct;
      end;


    function tprocdef.GetParentFPStructPtrType: tdef;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010305);
        result:=implprocdefinfo^.parentfpstructptrtype;
      end;


    procedure tprocdef.SetParentFPInitBlock(AValue: tnode);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010306);
        implprocdefinfo^.parentfpinitblock:=AValue;
      end;


    function tprocdef.Getprocendtai: tai;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010307);
        result:=implprocdefinfo^.procendtai;
      end;


    function tprocdef.Getprocstarttai: tai;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010308);
        result:=implprocdefinfo^.procstarttai;
      end;


    procedure tprocdef.Setprocendtai(AValue: tai);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010309);
        implprocdefinfo^.procendtai:=AValue;
      end;


    function tprocdef.Getskpara: pointer;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010310);
        result:=implprocdefinfo^.skpara;
      end;


    procedure tprocdef.Setskpara(AValue: pointer);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010311);
        implprocdefinfo^.skpara:=AValue;
      end;


    function tprocdef.Getpersonality: tprocdef;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2016121701);
         result:=implprocdefinfo^.personality;
      end;


    procedure tprocdef.Setpersonality(AValue: tprocdef);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2016121702);
        implprocdefinfo^.personality:=AValue;
      end;


    function tprocdef.Getforwarddef: boolean;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010312);
        result:=implprocdefinfo^.forwarddef;
      end;


    function tprocdef.Gethasforward: boolean;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010313);
        result:=implprocdefinfo^.hasforward;
      end;


    function tprocdef.Getinterfacedef: boolean;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010314);
        result:=implprocdefinfo^.interfacedef;
      end;


    procedure tprocdef.Setforwarddef(AValue: boolean);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010315);
        implprocdefinfo^.forwarddef:=AValue;
      end;


    procedure tprocdef.Sethasforward(AValue: boolean);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010316);
        implprocdefinfo^.hasforward:=AValue;
      end;


    function tprocdef.GetIsEmpty: boolean;
      begin
        result:=pio_empty in implprocoptions;
      end;


    procedure tprocdef.SetIsEmpty(AValue: boolean);
      begin
        if AValue then
          include(implprocoptions,pio_empty)
        else
          include(implprocoptions,pio_empty);
      end;


    function tprocdef.GetHasInliningInfo: boolean;
      begin
        result:=pio_has_inlininginfo in implprocoptions;
      end;


    procedure tprocdef.SetHasInliningInfo(AValue: boolean);
      begin
        if AValue then
          include(implprocoptions,pio_has_inlininginfo)
        else
          exclude(implprocoptions,pio_has_inlininginfo);
      end;


    procedure tprocdef.Setinterfacedef(AValue: boolean);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010317);
        implprocdefinfo^.interfacedef:=AValue;
      end;


    procedure tprocdef.Setprocstarttai(AValue: tai);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010318);
        implprocdefinfo^.procstarttai:=AValue;
      end;


    procedure tprocdef.SetParentFPStruct(AValue: tsym);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010319);
        implprocdefinfo^.parentfpstruct:=AValue;
      end;


    procedure tprocdef.SetParentFPStructPtrType(AValue: tdef);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010320);
        implprocdefinfo^.parentfpstructptrtype:=AValue;
      end;


    constructor tprocdef.create(level:byte;doregister:boolean);
      begin
         inherited create(procdef,level,doregister);
         implprocdefinfo:=allocmem(sizeof(implprocdefinfo^));
         localst:=tlocalsymtable.create(self,parast.symtablelevel);
{$ifdef symansistr}
         _mangledname:='';
{$else symansistr}
         _mangledname:=nil;
{$endif symansistr}
         fileinfo:=current_filepos;
         extnumber:=$ffff;
         aliasnames:=TCmdStrList.create;
         funcretsym:=nil;
         funcretsymderef.reset;
         procsymderef.reset;
         forwarddef:=true;
         interfacedef:=false;
         hasforward:=false;
         struct := nil;
         structderef.reset;
         import_dll:=nil;
         import_name:=nil;
         import_nr:=0;
         inlininginfo:=nil;
         deprecatedmsg:=nil;
      end;


    constructor tprocdef.ppuload(ppufile:tcompilerppufile);
      var
        i,aliasnamescount,sizeleft : longint;
        level : byte;
        buf : array[0..255] of byte;
      begin
         inherited ppuload(procdef,ppufile);
{$ifdef symansistr}
         if po_has_mangledname in procoptions then
           _mangledname:=ppufile.getansistring
         else
           _mangledname:='';
{$else symansistr}
         if po_has_mangledname in procoptions then
          _mangledname:=ppufile.getpshortstring
         else
          _mangledname:=nil;
{$endif symansistr}
         extnumber:=ppufile.getword;
         level:=ppufile.getbyte;
         ppufile.getderef(structderef);
         ppufile.getderef(procsymderef);
         ppufile.getposinfo(fileinfo);
         visibility:=tvisibility(ppufile.getbyte);
         ppufile.getset(tppuset2(symoptions));
         if sp_has_deprecated_msg in symoptions then
           deprecatedmsg:=ppufile.getpshortstring
         else
           deprecatedmsg:=nil;
         { import stuff }
         if po_has_importdll in procoptions then
           import_dll:=ppufile.getpshortstring
         else
           import_dll:=nil;
         if po_has_importname in procoptions then
           import_name:=ppufile.getpshortstring
         else
           import_name:=nil;
         import_nr:=ppufile.getword;
         if (po_msgint in procoptions) then
           messageinf.i:=ppufile.getlongint;
         if (po_msgstr in procoptions) then
           messageinf.str:=ppufile.getpshortstring;
         if (po_dispid in procoptions) then
           dispid:=ppufile.getlongint;
         { inline stuff }
         ppufile.getset(tppuset1(implprocoptions));
         if has_inlininginfo then
           begin
             ppufile.getderef(funcretsymderef);
             new(inlininginfo);
             ppufile.getset(tppuset4(inlininginfo^.flags));
           end
         else
           begin
             inlininginfo:=nil;
             funcretsym:=nil;
           end;

         aliasnames:=TCmdStrList.create;
         { count alias names }
         aliasnamescount:=ppufile.getbyte;
         for i:=1 to aliasnamescount do
           aliasnames.insert(ppufile.getstring);

         { load the token stream containing the declaration }
         sizeleft:=ppufile.getlongint;
         if sizeleft>0 then
           begin
             init_genericdecl;
             while sizeleft>0 do
               begin
                 if sizeleft>sizeof(buf) then
                   i:=sizeof(buf)
                 else
                   i:=sizeleft;
                 ppufile.getdata(buf,i);
                 genericdecltokenbuf.write(buf,i);
                 dec(sizeleft,i);
               end;
           end;

         ppuload_platform(ppufile);

         { load para symtable }
         parast:=tparasymtable.create(self,level);
         tparasymtable(parast).ppuload(ppufile);
         { load local symtable }
         if store_localst then
          begin
            localst:=tlocalsymtable.create(self,level);
            tlocalsymtable(localst).ppuload(ppufile);
          end
         else
          localst:=nil;
         { inline stuff }
         if has_inlininginfo then
           inlininginfo^.code:=ppuloadnodetree(ppufile);
         { default values for no persistent data }
         if (cs_link_deffile in current_settings.globalswitches) and
            (tf_need_export in target_info.flags) and
            (po_exports in procoptions) then
           deffile.AddExport(mangledname);
         { Disable po_has_inlining until the derefimpl is done }
         has_inlininginfo:=false;
      end;


    destructor tprocdef.destroy;
      begin
         aliasnames.free;
         aliasnames:=nil;
         if assigned(localst) and
           (localst.symtabletype<>staticsymtable) then
          begin
{$ifdef MEMDEBUG}
            memproclocalst.start;
{$endif MEMDEBUG}
            localst.free;
            localst:=nil;
{$ifdef MEMDEBUG}
            memproclocalst.start;
{$endif MEMDEBUG}
          end;
         if assigned(inlininginfo) then
          begin
{$ifdef MEMDEBUG}
            memprocnodetree.start;
{$endif MEMDEBUG}
            tnode(inlininginfo^.code).free;
{$ifdef MEMDEBUG}
            memprocnodetree.start;
{$endif MEMDEBUG}
            dispose(inlininginfo);
            inlininginfo:=nil;
          end;
         freeimplprocdefinfo;
         genericdecltokenbuf.free;
         genericdecltokenbuf:=nil;
         stringdispose(import_dll);
         stringdispose(import_name);
         stringdispose(deprecatedmsg);
         if (po_msgstr in procoptions) then
           stringdispose(messageinf.str);
{$ifndef symansistr}
         if assigned(_mangledname) then
          begin
{$ifdef MEMDEBUG}
            memmanglednames.start;
{$endif MEMDEBUG}
            stringdispose(_mangledname);
{$ifdef MEMDEBUG}
            memmanglednames.stop;
{$endif MEMDEBUG}
          end;
{$endif symansistr}
         inherited destroy;
      end;


    procedure tprocdef.freeimplprocdefinfo;
      begin
        if assigned(implprocdefinfo) then
          begin
            stringdispose(implprocdefinfo^.resultname);
            freemem(implprocdefinfo);
            implprocdefinfo:=nil;
          end;
      end;


    procedure tprocdef.ppuwrite(ppufile:tcompilerppufile);
      var
        oldintfcrc : boolean;
        aliasnamescount,i,sizeleft : longint;
        item : TCmdStrListItem;
        buf : array[0..255] of byte;
      begin
         { released procdef? }
         if not assigned(parast) then
           exit;

         inherited ppuwrite(ppufile);
{$ifdef symansistr}
         if po_has_mangledname in procoptions then
           ppufile.putansistring(_mangledname);
{$else symansistr}
         if po_has_mangledname in procoptions then
          ppufile.putstring(_mangledname^);
{$endif symansistr}

         ppufile.putword(extnumber);
         ppufile.putbyte(parast.symtablelevel);
         ppufile.putderef(structderef);
         ppufile.putderef(procsymderef);
         ppufile.putposinfo(fileinfo);
         ppufile.putbyte(byte(visibility));
         ppufile.putset(tppuset2(symoptions));
         if sp_has_deprecated_msg in symoptions then
           ppufile.putstring(deprecatedmsg^);
         { import }
         if po_has_importdll in procoptions then
           ppufile.putstring(import_dll^);
         if po_has_importname in procoptions then
           ppufile.putstring(import_name^);
         ppufile.putword(import_nr);
         if (po_msgint in procoptions) then
           ppufile.putlongint(messageinf.i);
         if (po_msgstr in procoptions) then
           ppufile.putstring(messageinf.str^);
         if (po_dispid in procoptions) then
           ppufile.putlongint(dispid);
         { inline stuff }
         oldintfcrc:=ppufile.do_crc;
         ppufile.do_crc:=false;
         ppufile.putset(tppuset1(implprocoptions));
         if has_inlininginfo then
           begin
             ppufile.putderef(funcretsymderef);
             ppufile.putset(tppuset4(inlininginfo^.flags));
           end;

         { count alias names }
         aliasnamescount:=0;
         item:=TCmdStrListItem(aliasnames.first);
         while assigned(item) do
           begin
             inc(aliasnamescount);
             item:=TCmdStrListItem(item.next);
           end;
         if aliasnamescount>255 then
           internalerror(200711021);
         ppufile.putbyte(aliasnamescount);
         item:=TCmdStrListItem(aliasnames.first);
         while assigned(item) do
          begin
            ppufile.putstring(item.str);
            item:=TCmdStrListItem(item.next);
          end;

         ppufile.do_crc:=oldintfcrc;

         { generic tokens for the declaration }
         if assigned(genericdecltokenbuf) and (genericdecltokenbuf.size>0) then
           begin
             sizeleft:=genericdecltokenbuf.size;
             genericdecltokenbuf.seek(0);
             ppufile.putlongint(sizeleft);
             while sizeleft>0 do
               begin
                 if sizeleft>sizeof(buf) then
                   i:=sizeof(buf)
                 else
                   i:=sizeleft;
                 genericdecltokenbuf.read(buf,i);
                 ppufile.putdata(buf,i);
                 dec(sizeleft,i);
               end;
           end
         else
           ppufile.putlongint(0);

         { write this entry }
         writeentry(ppufile,ibprocdef);

         { Save the para symtable, this is taken from the interface }
         tparasymtable(parast).ppuwrite(ppufile);

         { save localsymtable for inline procedures or when local
           browser info is requested, this has no influence on the crc }
         if store_localst and not ppufile.crc_only then
          begin
            oldintfcrc:=ppufile.do_crc;
            ppufile.do_crc:=false;
            tlocalsymtable(localst).ppuwrite(ppufile);
            ppufile.do_crc:=oldintfcrc;
          end;

         { node tree for inlining }
         oldintfcrc:=ppufile.do_crc;
         ppufile.do_crc:=false;
         if has_inlininginfo then
           ppuwritenodetree(ppufile,inlininginfo^.code);
         ppufile.do_crc:=oldintfcrc;
      end;


    function tprocdef.fullprocname(showhidden:boolean):string;
      var
        pno: tprocnameoptions;
      begin
        pno:=[];
        if showhidden then
          include(pno,pno_showhidden);
        result:=customprocname(pno);
      end;


    function tprocdef.customprocname(pno: tprocnameoptions):ansistring;
      var
        s, rn : ansistring;
        t : ttoken;
        syssym : tsyssym;
      begin
{$ifdef EXTDEBUG}
        include(pno,pno_showhidden);
{$endif EXTDEBUG}
        s:='';
        if proctypeoption=potype_operator then
          begin
            for t:=NOTOKEN to last_overloaded do
              if procsym.realname='$'+overloaded_names[t] then
                begin
                  s:='operator ';
                  if (pno_ownername in pno) and
                     assigned(struct) then
                    s:=s+struct.RttiName+'.';
                  s:=s+arraytokeninfo[t].str;
                  if not (pno_noparams in pno) then
                    s:=s+typename_paras(pno);
                  break;
                end;
          end
        else
          begin
            if (po_classmethod in procoptions) and
               not(pno_noclassmarker in pno) then
              s:='class ';
            case proctypeoption of
              potype_constructor,
              potype_class_constructor:
                s:=s+'constructor ';
              potype_class_destructor,
              potype_destructor:
                s:=s+'destructor ';
              else
                if (pno_proctypeoption in pno) then
                  begin
                   if assigned(returndef) and
                     not(is_void(returndef)) then
                     s:=s+'function '
                   else
                     s:=s+'procedure ';
                  end;
            end;
            if (pno_ownername in pno) and
               (owner.symtabletype in [recordsymtable,objectsymtable]) then
              s:=s+tabstractrecorddef(owner.defowner).RttiName+'.';
            if (po_compilerproc in procoptions) and (extnumber<>$ffff) then
              begin
                syssym:=tsyssym.find_by_number(extnumber);
                if not assigned(syssym) then
                  internalerror(2016060305);
                rn:=syssym.realname;
              end
            else
              rn:=procsym.realname;
            if (pno_noleadingdollar in pno) and
               (rn[1]='$') then
              delete(rn,1,1);
            s:=s+rn;
            if not (pno_noparams in pno) then
              s:=s+typename_paras(pno);
          end;
        if not(proctypeoption in [potype_constructor,potype_destructor,
             potype_class_constructor,potype_class_destructor]) and
           assigned(returndef) and
           not(is_void(returndef)) then
          s:=s+':'+returndef.GetTypeName;
        if assigned(owner) and (owner.symtabletype=localsymtable) then
          s:=s+' is nested'
        else if po_is_block in procoptions then
          s:=s+' is block';
        s:=s+';';
        if po_far in procoptions then
          s:=s+' far;';
        { forced calling convention? }
        if (po_hascallingconvention in procoptions) then
          s:=s+' '+ProcCallOptionStr[proccalloption]+';';
        if (po_staticmethod in procoptions) and
           not (proctypeoption in [potype_class_constructor,potype_class_destructor]) then
          s:=s+' Static;';
        if pno_mangledname in pno then
          s:=s+' -- mangled name: '+mangledname;
        customprocname:=s;
      end;


    function tprocdef.is_methodpointer:boolean;
      begin
        { don't check assigned(_class), that's also the case for nested
          procedures inside methods }
        result:=(owner.symtabletype=ObjectSymtable)and not no_self_node;
      end;


    function tprocdef.is_addressonly:boolean;
      begin
        result:=assigned(owner) and
                not is_methodpointer and
                (not(m_nested_procvars in current_settings.modeswitches) or
                 not is_nested_pd(self));
      end;


    procedure tprocdef.make_external;
      begin
        include(procoptions,po_external);
        forwarddef:=false;
      end;


    procedure tprocdef.init_genericdecl;
      begin
        if assigned(genericdecltokenbuf) then
          internalerror(2015061901);
        genericdecltokenbuf:=tdynamicarray.create(256);
      end;


    function tprocdef.get_funcretsym_info(out ressym: tsym; out resdef: tdef): boolean;
      begin
        result:=false;
        if proctypeoption=potype_constructor then
          begin
            result:=true;
            ressym:=tsym(parast.Find('self'));
            resdef:=tabstractnormalvarsym(ressym).vardef;
            { and TP-style constructors return a pointer to self }
            if is_object(resdef) then
              resdef:=cpointerdef.getreusable(resdef);
          end
        else if (proccalloption=pocall_safecall) and
           (tf_safecall_exceptions in target_info.flags) then
          begin
            result:=true;
            ressym:=tsym(localst.Find('safecallresult'));
            resdef:=tabstractnormalvarsym(ressym).vardef;
          end
        else if not is_void(returndef) then
          begin
            result:=true;
            ressym:=funcretsym;
            resdef:=tabstractnormalvarsym(ressym).vardef;
          end;
      end;


    function tprocdef.get_safecall_funcretsym_info(out ressym: tsym; out resdef: tdef): boolean;
      begin
        result:=false;
        if (proctypeoption<>potype_constructor) and
           (proccalloption=pocall_safecall) and
           (tf_safecall_exceptions in target_info.flags) then
          begin
            result:=true;
            ressym:=tsym(localst.Find('safecallresult'));
            resdef:=tabstractnormalvarsym(ressym).vardef;
          end
      end;


    function tprocdef.has_alias_name(const s: TSymStr): boolean;
      var
        item : TCmdStrListItem;
      begin
        result:=true;
        if mangledname=s then
          exit;
        item:=TCmdStrListItem(aliasnames.first);
        while assigned(item) do
          begin
            if item.str=s then
              exit;
            item:=TCmdStrListItem(item.next);
          end;
        result:=false;
      end;


    function tprocdef.GetSymtable(t:tGetSymtable):TSymtable;
      begin
        case t of
          gs_local :
            GetSymtable:=localst;
          gs_para :
            GetSymtable:=parast;
          else
            GetSymtable:=nil;
        end;
      end;


    function tprocdef.getcopyas(newtyp: tdeftyp; copytyp: tproccopytyp; const paraprefix: string): tstoreddef;
      var
        j : longint;
      begin
        result:=inherited;
        if newtyp=procvardef then
          begin
            { create new paralist }
            tprocvardef(result).calcparas;
            exit;
          end;
        { don't copy mangled name, can be different }
        tprocdef(result).messageinf:=messageinf;
        tprocdef(result).dispid:=dispid;
        if po_msgstr in procoptions then
          tprocdef(result).messageinf.str:=stringdup(messageinf.str^);
        tprocdef(result).symoptions:=symoptions;
        if assigned(deprecatedmsg) then
          tprocdef(result).deprecatedmsg:=stringdup(deprecatedmsg^);
        { will have to be associated with appropriate procsym }
        tprocdef(result).procsym:=nil;
        { don't create aliases for bare copies, nor copy the funcretsym as
          the function result parameter will be inserted again if necessary
          (e.g. if the calling convention is changed) }
        if not(copytyp in [pc_bareproc,pc_normal_no_hidden]) then
          begin
            tprocdef(result).aliasnames.concatListcopy(aliasnames);
            if assigned(funcretsym) then
              begin
                if funcretsym.owner=parast then
                  begin
                    j:=parast.symlist.indexof(funcretsym);
                    if j<0 then
                      internalerror(2011040606);
                    tprocdef(result).funcretsym:=tsym(tprocdef(result).parast.symlist[j]);
                  end
                else if funcretsym.owner=localst then
                  begin
                    { nothing to do, will be inserted for the new procdef while
                      parsing its body (by pdecsub.insert_funcret_local) }
                  end
                else
                  internalerror(2011040605);
              end;
          end;
        { will have to be associated with a new struct }
        tprocdef(result).struct:=nil;
        if assigned(implprocdefinfo) then
          begin
            if assigned(resultname) then
              tprocdef(result).resultname:=stringdup(resultname^);
            tprocdef(result).synthetickind:=synthetickind;
          end;
        if assigned(import_dll) then
          tprocdef(result).import_dll:=stringdup(import_dll^);
        if assigned(import_name) then
          tprocdef(result).import_name:=stringdup(import_name^);
        tprocdef(result).import_nr:=import_nr;
        tprocdef(result).extnumber:=$ffff;
        tprocdef(result).visibility:=visibility;
        { we need a separate implementation for the copied def }
        tprocdef(result).forwarddef:=true;
        tprocdef(result).interfacedef:=true;

        { create new paralist }
        tprocdef(result).calcparas;
      end;


    function tprocdef.getcopy: tstoreddef;
      begin
        result:=getcopyas(procdef,pc_normal,'');
      end;


    procedure tprocdef.buildderef;
      begin
         inherited buildderef;
         structderef.build(struct);
         { procsym that originaly defined this definition, should be in the
           same symtable }
         procsymderef.build(procsym);
      end;


    procedure tprocdef.buildderefimpl;
      begin
         inherited buildderefimpl;

         { Localst is not available for main/unit init }
         if store_localst and assigned(localst) then
           begin
             tlocalsymtable(localst).buildderef;
             tlocalsymtable(localst).buildderefimpl;
           end;

         { inline tree }
         if has_inlininginfo then
           begin
             funcretsymderef.build(funcretsym);
             inlininginfo^.code.buildderefimpl;
           end;
      end;


    procedure tprocdef.deref;
      begin
         inherited deref;
         struct:=tabstractrecorddef(structderef.resolve);
         { procsym that originaly defined this definition, should be in the
           same symtable }
         procsym:=tprocsym(procsymderef.resolve);
      end;


    procedure tprocdef.derefimpl;
      begin
         { Enable has_inlininginfo when the inlininginfo
           structure is available. The has_inlininginfo was disabled
           after the load, since the data was invalid }
         if assigned(inlininginfo) then
           has_inlininginfo:=true;

         { Locals }
         if store_localst and assigned(localst) then
           begin
             tlocalsymtable(localst).deref(false);
             tlocalsymtable(localst).derefimpl(false);
           end;

        { Inline }
        if has_inlininginfo then
          begin
            inlininginfo^.code.derefimpl;
            { funcretsym, this is always located in the localst }
            funcretsym:=tsym(funcretsymderef.resolve);
          end
        else
          begin
            { safety }
            { Not safe! A unit may be reresolved after its interface has been
              parsed but before its implementation has been parsed, and in that
              case the funcretsym is still required!
            funcretsym:=nil; }
          end;
      end;


    function tprocdef.GetTypeName : string;
      begin
         GetTypeName := FullProcName(false);
      end;


    function tprocdef.mangledname : TSymStr;
      begin
{$ifdef symansistr}
        if _mangledname='' then
          begin
            result:=defaultmangledname;
            _mangledname:=result;
          end
        else
          result:=_mangledname;
{$else symansistr}
        if not assigned(_mangledname) then
          begin
            result:=defaultmangledname;
            _mangledname:=stringdup(mangledname);
          end
        else
          result:=_mangledname^;
{$endif symansistr}
      end;


    function tprocdef.defaultmangledname: TSymStr;
      begin
        { we need to use the symtable where the procsym is inserted,
          because that is visible to the world }
        defaultmangledname:=make_mangledname('',procsym.owner,procsym.name);
        defaultmangledname:=defaultmangledname+mangledprocparanames(Length(defaultmangledname))
      end;


    function tprocdef.cplusplusmangledname : TSymStr;

      function getcppparaname(p : tdef) : TSymStr;

        const
{$ifdef NAMEMANGLING_GCC2}
           ordtype2str : array[tordtype] of string[2] = (
             '',
             'Uc','Us','Ui','Us','',
             'Sc','s','i','x','',
             'b','b','b','b','b','b',
             'c','w','x');
{$else NAMEMANGLING_GCC2}
           ordtype2str : array[tordtype] of string[1] = (
             'v',
             'h','t','j','y','',
             'a','s','i','x','',
             'b','b','b','b','b',
             'b','b','b','b',
             'c','w','x','C');

           floattype2str : array[tfloattype] of string[1] = (
             'f','d','e','e',
             'd','d','g');
{$endif NAMEMANGLING_GCC2}

        var
           s : TSymStr;

        begin
           case p.typ of
              orddef:
                begin
                  s:=ordtype2str[torddef(p).ordtype];
                  if s='C' then
                    s:=ordtype2str[range_to_basetype(torddef(p).low,torddef(p).high)];
                end;
              pointerdef:
                s:='P'+getcppparaname(tpointerdef(p).pointeddef);
{$ifndef NAMEMANGLING_GCC2}
              floatdef:
                s:=floattype2str[tfloatdef(p).floattype];
{$endif NAMEMANGLING_GCC2}
              else
                internalerror(2103001);
           end;
           getcppparaname:=s;
        end;

      var
         s,s2 : TSymStr;
         hp   : TParavarsym;
         i    : integer;

      begin
{$ifdef NAMEMANGLING_GCC2}

        { outdated gcc 2.x name mangling scheme }
         s := procsym.realname;
         if procsym.owner.symtabletype=ObjectSymtable then
           begin
              s2:=upper(tobjectdef(procsym.owner.defowner).objrealname^);
              case proctypeoption of
                 potype_destructor:
                   s:='_$_'+tostr(length(s2))+s2;
                 potype_constructor:
                   s:='___'+tostr(length(s2))+s2;
                 else
                   s:='_'+s+'__'+tostr(length(s2))+s2;
              end;

           end
         else s:=s+'__';

         s:=s+'F';

         { concat modifiers }
         { !!!!! }

         { now we handle the parameters }
         if maxparacount>0 then
           begin
             for i:=0 to paras.count-1 do
               begin
                 hp:=tparavarsym(paras[i]);
                 { no hidden parameters form part of a C++ mangled name:
                     a) self is not included
                     b) there are no "high" or other hidden parameters
                 }
                 if vo_is_hidden_para in hp.varoptions then
                   continue;
                 s2:=getcppparaname(hp.vardef);
                 if hp.varspez in [vs_var,vs_out] then
                   s2:='R'+s2;
                 s:=s+s2;
               end;
           end
         else
           s:=s+'v';
         cplusplusmangledname:=s;
{$else NAMEMANGLING_GCC2}

         { gcc 3.x and 4.x name mangling scheme }
         { see http://www.codesourcery.com/public/cxx-abi/abi.html#mangling }
         if procsym.owner.symtabletype=ObjectSymtable then
           begin
             s:='_ZN';

             s2:=tobjectdef(procsym.owner.defowner).objextname^;
             s:=s+tostr(length(s2))+s2;
             case proctypeoption of
                potype_constructor:
                  s:=s+'C1';
                potype_destructor:
                  s:=s+'D1';
                else
                  s:=s+tostr(length(procsym.realname))+procsym.realname;
             end;

             s:=s+'E';
           end
         else
           s:=procsym.realname;

         { now we handle the parameters }
         if maxparacount>0 then
           begin
             for i:=0 to paras.count-1 do
               begin
                 hp:=tparavarsym(paras[i]);
                 { no hidden parameters form part of a C++ mangled name:
                     a) self is not included
                     b) there are no "high" or other hidden parameters
                 }
                 if vo_is_hidden_para in hp.varoptions then
                   continue;
                 s2:=getcppparaname(hp.vardef);
                 if hp.varspez in [vs_var,vs_out] then
                   s2:='R'+s2;
                 s:=s+s2;
               end;
           end
         else
           s:=s+'v';
         cplusplusmangledname:=s;
{$endif NAMEMANGLING_GCC2}
      end;


    function  tprocdef.objcmangledname : TSymStr;
      var
        manglednamelen: longint;
        iscatmethod   : boolean;
      begin
        if not (po_msgstr in procoptions) then
          internalerror(2009030901);
        { we may very well need longer strings to handle these... }
        manglednamelen:=length(tobjectdef(procsym.owner.defowner).objextname^)+
          length('+"[ ]"')+length(messageinf.str^);
        iscatmethod:=oo_is_classhelper in tobjectdef(procsym.owner.defowner).objectoptions;
        if (iscatmethod) then
          inc(manglednamelen,length(tobjectdef(procsym.owner.defowner).childof.objextname^)+length('()'));
        if manglednamelen>255 then
          Message1(parser_e_objc_message_name_too_long,messageinf.str^);
        if not(po_classmethod in procoptions) then
          result:='"-['
        else
          result:='"+[';
        { quotes are necessary because the +/- otherwise confuse the assembler
          into expecting a number
        }
        if iscatmethod then
          result:=result+tobjectdef(procsym.owner.defowner).childof.objextname^+'(';
        result:=result+tobjectdef(procsym.owner.defowner).objextname^;
        if iscatmethod then
          result:=result+')';
        result:=result+' '+messageinf.str^+']"';
      end;


    procedure tprocdef.setmangledname(const s : TSymStr);
      begin
        { This is not allowed anymore, the forward declaration
          already needs to create the correct mangledname, no changes
          afterwards are allowed (PFV) }
        { Exception: interface definitions in mode macpas, since in that }
        {   case no reference to the old name can exist yet (JM)         }
{$ifdef symansistr}
        if _mangledname<>'' then
          if ((m_mac in current_settings.modeswitches) and
              (interfacedef)) then
            _mangledname:=''
          else
            internalerror(200411171);
{$else symansistr}
        if assigned(_mangledname) then
          if ((m_mac in current_settings.modeswitches) and
              (interfacedef)) then
            stringdispose(_mangledname)
          else
            internalerror(200411171);
{$endif symansistr}
{$ifdef jvm}
        { this routine can be called for compilerproces. can't set mangled
          name since it must be calculated, but it uses import_name when set
          -> set that one }
        import_name:=stringdup(s);
        include(procoptions,po_has_importname);
        include(procoptions,po_has_mangledname);
{$else}
  {$ifdef symansistr}
        _mangledname:=s;
  {$else symansistr}
        _mangledname:=stringdup(s);
  {$endif symansistr}
{$endif jvm}
        include(procoptions,po_has_mangledname);
      end;

    function tprocdef.needsglobalasmsym: boolean;
      begin
        result:=
          (cs_profile in current_settings.moduleswitches) or
          { smart linking using a library requires to promote
            all non-nested procedures to AB_GLOBAL
            otherwise you get undefined symbol error at linking
            for msdos  target with -CX option for instance }
          (create_smartlink_library and not is_nested_pd(self)) or
          (po_global in procoptions);
    end;


    procedure tprocdef.setcompilerprocname;
      begin
        procsym.realname:='$'+lower(procsym.name);
      end;


{***************************************************************************
                                 TPROCVARDEF
***************************************************************************}

    constructor tprocvardef.create(level:byte);
      begin
         inherited create(procvardef,level,true);
      end;


    class function tprocvardef.getreusableprocaddr(def: tabstractprocdef): tprocvardef;
      var
        res: PHashSetItem;
        oldsymtablestack: tsymtablestack;
      begin
        if not assigned(current_module) then
          internalerror(2011081301);
        res:=current_module.procaddrdefs.FindOrAdd(@def,sizeof(def));
        if not assigned(res^.Data) then
          begin
            { since these pointerdefs can be reused anywhere in the current
              unit, add them to the global/staticsymtable (or local symtable
              if they're a local def, because otherwise they'll be saved
              to the ppu referencing a local symtable entry that doesn't
              exist in the ppu) }
            oldsymtablestack:=symtablestack;
            { do not simply push/pop current_module.localsymtable, because
              that can have side-effects (e.g., it removes helpers) }
            symtablestack:=nil;
            result:=tprocvardef(def.getcopyas(procvardef,pc_address_only,''));
            setup_reusable_def(def,result,res,oldsymtablestack);
            { res^.Data may still be nil -> don't overwrite result }
            exit;
          end;
        result:=tprocvardef(res^.Data);
      end;


    class function tprocvardef.getreusableprocaddr_no_free(def: tabstractprocdef): tprocvardef;
      begin
        result:=getreusableprocaddr(def);
        if not result.is_registered then
          include(result.defoptions,df_not_registered_no_free);
      end;


    constructor tprocvardef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(procvardef,ppufile);
         { load para symtable }
         parast:=tparasymtable.create(self,ppufile.getbyte);
         ppuload_platform(ppufile);
         tparasymtable(parast).ppuload(ppufile);
      end;


    function tprocvardef.getcopy : tstoreddef;
      var
        i : tcallercallee;
        j : longint;
      begin
        result:=cprocvardef.create(parast.symtablelevel);
        tprocvardef(result).returndef:=returndef;
        tprocvardef(result).returndefderef:=returndefderef;
        tprocvardef(result).parast:=parast.getcopy;
        tprocvardef(result).savesize:=savesize;

        { create paralist copy }
        calcparas;
        tprocvardef(result).paras:=tparalist.create(false);
        tprocvardef(result).paras.count:=paras.count;
        for j:=0 to paras.count-1 do
          tprocvardef(result).paras[j]:=paras[j];

        tprocvardef(result).proctypeoption:=proctypeoption;
        tprocvardef(result).proccalloption:=proccalloption;
        tprocvardef(result).procoptions:=procoptions;
        tprocvardef(result).callerargareasize:=callerargareasize;
        tprocvardef(result).calleeargareasize:=calleeargareasize;
        tprocvardef(result).maxparacount:=maxparacount;
        tprocvardef(result).minparacount:=minparacount;
        for i:=low(funcretloc) to high(funcretloc) do
          tprocvardef(result).funcretloc[i]:=funcretloc[i].getcopy;
        tprocvardef(result).has_paraloc_info:=has_paraloc_info;
{$ifdef m68k}
        tprocvardef(result).exp_funcretloc:=exp_funcretloc;
{$endif}
      end;


    procedure tprocvardef.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);

        { Save the para symtable level (necessary to distinguish nested
          procvars) }
        ppufile.putbyte(parast.symtablelevel);

        { Write this entry }
        writeentry(ppufile,ibprocvardef);

        { Save the para symtable, this is taken from the interface }
        tparasymtable(parast).ppuwrite(ppufile);
      end;


    function tprocvardef.GetSymtable(t:tGetSymtable):TSymtable;
      begin
        case t of
          gs_para :
            GetSymtable:=parast;
          else
            GetSymtable:=nil;
        end;
      end;


    function tprocvardef.size : asizeint;
      begin
         { we return false for is_addressonly for a block (because it's not a
           simple pointer to a function), but they are handled as implicit
           pointers to a datastructure that contains everything ->
           voidpointertype.size instead of voidcodepointertype.size }
         if po_is_block in procoptions then
           size:=voidpointertype.size
         else if not is_addressonly then
           begin
             if is_nested_pd(self) then
               size:=voidcodepointertype.size+parentfpvoidpointertype.size
             else
               size:=voidcodepointertype.size+voidpointertype.size;
           end
         else
           size:=voidcodepointertype.size;
      end;


    function tprocvardef.is_methodpointer:boolean;
      begin
        result:=(po_methodpointer in procoptions);
      end;


    function tprocvardef.is_addressonly:boolean;
      begin
        result:=((not(po_methodpointer in procoptions) or (po_staticmethod in procoptions)) and
                 not(po_is_block in procoptions) and
                 not is_nested_pd(self)) or
                (po_addressonly in procoptions);
      end;


    function tprocvardef.getmangledparaname:TSymStr;
      begin
        if not(po_methodpointer in procoptions) then
          if not is_nested_pd(self) then
            result:='procvar'
          else
            { we need the manglednames here, because nestedprocvars can be anonymous, e.g.
              having not a type name or not an unique one, see webtbs/tw27515.pp

              Further, use $_ ... _$ delimiters to avoid ambiguous names, see webtbs/tw27515.pp }
            result:='$_nestedprovar'+mangledprocparanames(0)+'_$'
        else
          result:='procvarofobj'
      end;


    function tprocvardef.getcopyas(newtyp: tdeftyp; copytyp: tproccopytyp; const paraprefix: string): tstoreddef;
      begin
        result:=inherited;
        tabstractprocdef(result).calcparas;
      end;


    function tprocvardef.is_publishable : boolean;
      begin
         is_publishable:=(po_methodpointer in procoptions);
      end;


    function tprocvardef.GetTypeName : string;
      var
        s: string;
        pno : tprocnameoptions;
      begin
{$ifdef EXTDEBUG}
         pno:=[pno_showhidden];
{$else EXTDEBUG}
         pno:=[];
{$endif EXTDEBUG}
         s:='<';
         if po_is_block in procoptions then
           s := s+'reference to'
         else if po_classmethod in procoptions then
           s := s+'class method type of'
         else
           if po_addressonly in procoptions then
             s := s+'address of'
           else
             s := s+'procedure variable type of';
         if assigned(returndef) and
            (returndef<>voidtype) then
           s:=s+' function'+typename_paras(pno)+':'+returndef.GetTypeName
         else
           s:=s+' procedure'+typename_paras(pno);
         if po_methodpointer in procoptions then
           s := s+' of object';
         if is_nested_pd(self) then
           s := s+' is nested';
         if po_far in procoptions then
           s := s+';far';
         GetTypeName := s+';'+ProcCallOptionStr[proccalloption]+'>';
      end;


{***************************************************************************
                              TOBJECTDEF
***************************************************************************}

   constructor tobjectdef.create(ot:tobjecttyp;const n:string;c:tobjectdef;doregister:boolean);
     begin
        inherited create(n,objectdef,doregister);
        fcurrent_dispid:=0;
        objecttype:=ot;
        childof:=nil;
        childofderef.reset;
        vmt_fieldderef.reset;
        extendeddefderef.reset;
        cloneddefderef.reset;
        if objecttype=odt_helper then
          owner.includeoption(sto_has_helper);
        symtable:=tObjectSymtable.create(self,n,current_settings.packrecords,
          current_settings.alignment.recordalignmin);
        { create space for vmt !! }
        vmtentries:=TFPList.Create;
        set_parent(c);
        if objecttype in [odt_interfacecorba,odt_interfacecom,odt_dispinterface] then
          prepareguid;
        { setup implemented interfaces }
        if objecttype in [odt_class,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
          ImplementedInterfaces:=TFPObjectList.Create(true)
        else
          ImplementedInterfaces:=nil;
        writing_class_record_dbginfo:=false;
     end;


    constructor tobjectdef.ppuload(ppufile:tcompilerppufile);
      var
         i,
         implintfcount : longint;
         d, getterd : tderef;
         ImplIntf : TImplementedInterface;
         vmtentry : pvmtentry;
      begin
         inherited ppuload(objectdef,ppufile);
         objecttype:=tobjecttyp(ppufile.getbyte);
         helpertype:=thelpertype(ppufile.getbyte);
         objextname:=ppufile.getpshortstring;
         { only used for external Objective-C classes/protocols }
         if (objextname^='') then
           stringdispose(objextname);
         symtable:=tObjectSymtable.create(self,objrealname^,0,0);
         tObjectSymtable(symtable).datasize:=ppufile.getasizeint;
         tObjectSymtable(symtable).paddingsize:=ppufile.getword;
         tObjectSymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
         tObjectSymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
         tObjectSymtable(symtable).recordalignmin:=shortint(ppufile.getbyte);
         ppufile.getderef(vmt_fieldderef);
         ppufile.getderef(childofderef);

         { load guid }
         iidstr:=nil;
         if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface] then
           begin
              new(iidguid);
              ppufile.getguid(iidguid^);
              iidstr:=ppufile.getpshortstring;
           end;
         abstractcnt:=ppufile.getlongint;

         if objecttype=odt_helper then
           ppufile.getderef(extendeddefderef);

         vmtentries:=TFPList.Create;
         vmtentries.count:=ppufile.getlongint;
         for i:=0 to vmtentries.count-1 do
           begin
             ppufile.getderef(d);
             new(vmtentry);
             vmtentry^.procdef:=nil;
             vmtentry^.procdefderef:=d;
             vmtentry^.visibility:=tvisibility(ppufile.getbyte);
             vmtentries[i]:=vmtentry;
           end;

         { load implemented interfaces }
         if objecttype in [odt_class,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
           begin
             ImplementedInterfaces:=TFPObjectList.Create(true);
             implintfcount:=ppufile.getlongint;
             for i:=0 to implintfcount-1 do
               begin
                 ppufile.getderef(d);
                 ppufile.getderef(getterd);
                 ImplIntf:=TImplementedInterface.Create_deref(d,getterd);
                 ImplIntf.IOffset:=ppufile.getlongint;
                 byte(ImplIntf.IType):=ppufile.getbyte;
                 ImplementedInterfaces.Add(ImplIntf);
               end;
           end
         else
           ImplementedInterfaces:=nil;

         if df_copied_def in defoptions then
           begin
             ppufile.getderef(cloneddefderef);
             ppuload_platform(ppufile);
           end
         else
           begin
             ppuload_platform(ppufile);
             tObjectSymtable(symtable).ppuload(ppufile);
           end;

         { handles the predefined class tobject  }
         { the last TOBJECT which is loaded gets }
         { it !                                  }
         if (childof=nil) and
            (objecttype in [odt_class,odt_javaclass]) and
            (objname^='TOBJECT') then
           class_tobject:=self;
         if (childof=nil) and
            (objecttype=odt_interfacecom) then
            if (objname^='IUNKNOWN') then
              interface_iunknown:=self
            else
            if (objname^='IDISPATCH') then
              interface_idispatch:=self;
         if (childof=nil) and
            (objecttype=odt_objcclass) and
            (objname^='PROTOCOL') then
           objc_protocoltype:=self;
         if (objecttype=odt_javaclass) and
            not(oo_is_formal in objectoptions) then
           begin
             if (objname^='JLOBJECT') then
               java_jlobject:=self
             else if (objname^='JLTHROWABLE') then
               java_jlthrowable:=self
             else if (objname^='FPCBASERECORDTYPE') then
               java_fpcbaserecordtype:=self
             else if (objname^='JLSTRING') then
               java_jlstring:=self
             else if (objname^='ANSISTRINGCLASS') then
               java_ansistring:=self
             else if (objname^='SHORTSTRINGCLASS') then
               java_shortstring:=self
             else if (objname^='JLENUM') then
               java_jlenum:=self
             else if (objname^='JUENUMSET') then
               java_juenumset:=self
             else if (objname^='FPCBITSET') then
               java_jubitset:=self
             else if (objname^='FPCBASEPROCVARTYPE') then
               java_procvarbase:=self;
           end;
         writing_class_record_dbginfo:=false;
       end;


    destructor tobjectdef.destroy;
      var
        i: longint;
      begin
         if assigned(symtable) then
           begin
             symtable.free;
             symtable:=nil;
           end;
         stringdispose(objextname);
         stringdispose(iidstr);
         if assigned(ImplementedInterfaces) then
           begin
             ImplementedInterfaces.free;
             ImplementedInterfaces:=nil;
           end;
         if assigned(iidguid) then
           begin
             dispose(iidguid);
             iidguid:=nil;
           end;
         if assigned(vmtentries) then
           begin
             for i:=0 to vmtentries.count-1 do
               dispose(pvmtentry(vmtentries[i]));
             vmtentries.free;
             vmtentries:=nil;
           end;
         if assigned(vmcallstaticinfo) then
           begin
             freemem(vmcallstaticinfo);
             vmcallstaticinfo:=nil;
           end;
         inherited destroy;
      end;


    function tobjectdef.getcopy : tstoreddef;
      var
        i : longint;
      begin
        result:=cobjectdef.create(objecttype,objrealname^,childof,true);
        { the constructor allocates a symtable which we release to avoid memory leaks }
        tobjectdef(result).symtable.free;
        tobjectdef(result).symtable:=symtable.getcopy;
        if assigned(objextname) then
          tobjectdef(result).objextname:=stringdup(objextname^);
        if assigned(import_lib) then
          tobjectdef(result).import_lib:=stringdup(import_lib^);
        tobjectdef(result).objectoptions:=objectoptions;
        include(tobjectdef(result).defoptions,df_copied_def);
        tobjectdef(result).extendeddef:=extendeddef;
        if assigned(tcinitcode) then
          tobjectdef(result).tcinitcode:=tcinitcode.getcopy;
        tobjectdef(result).vmt_field:=vmt_field;
        if assigned(iidguid) then
          begin
            new(tobjectdef(result).iidguid);
            move(iidguid^,tobjectdef(result).iidguid^,sizeof(iidguid^));
          end;
        if assigned(iidstr) then
          tobjectdef(result).iidstr:=stringdup(iidstr^);
        tobjectdef(result).abstractcnt:=abstractcnt;
        if assigned(ImplementedInterfaces) then
          begin
            for i:=0 to ImplementedInterfaces.count-1 do
              tobjectdef(result).ImplementedInterfaces.Add(TImplementedInterface(ImplementedInterfaces[i]).Getcopy);
          end;
        tobjectdef(result).copyvmtentries(self);
      end;


    procedure tobjectdef.ppuwrite(ppufile:tcompilerppufile);
      var
         i : longint;
         vmtentry : pvmtentry;
         ImplIntf : TImplementedInterface;
         old_do_indirect_crc: boolean;
      begin
         { if class1 in unit A changes, and class2 in unit B inherits from it
           (so unit B uses unit A), then unit B with class2 will be recompiled.
           However, if there is also a class3 in unit C that only depends on
           unit B, then unit C will not be recompiled because nothing changed
           to the interface of unit B. Nevertheless, unit C can indirectly
           depend on unit A via derefs, and these must be updated -> the
           indirect crc keeps track of such changes. }
         old_do_indirect_crc:=ppufile.do_indirect_crc;
         ppufile.do_indirect_crc:=true;
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(objecttype));
         ppufile.putbyte(byte(helpertype));
         if assigned(objextname) then
           ppufile.putstring(objextname^)
         else
           ppufile.putstring('');
         ppufile.putasizeint(tObjectSymtable(symtable).datasize);
         ppufile.putword(tObjectSymtable(symtable).paddingsize);
         ppufile.putbyte(byte(tObjectSymtable(symtable).fieldalignment));
         ppufile.putbyte(byte(tObjectSymtable(symtable).recordalignment));
         ppufile.putbyte(byte(tObjectSymtable(symtable).recordalignmin));
         ppufile.putderef(vmt_fieldderef);
         ppufile.putderef(childofderef);
         if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface] then
           begin
              ppufile.putguid(iidguid^);
              ppufile.putstring(iidstr^);
           end;
         ppufile.putlongint(abstractcnt);
         if objecttype=odt_helper then
           ppufile.putderef(extendeddefderef);

         ppufile.putlongint(vmtentries.count);
         for i:=0 to vmtentries.count-1 do
           begin
             vmtentry:=pvmtentry(vmtentries[i]);
             ppufile.putderef(vmtentry^.procdefderef);
             ppufile.putbyte(byte(vmtentry^.visibility));
           end;


         if assigned(ImplementedInterfaces) then
           begin
             ppufile.putlongint(ImplementedInterfaces.Count);
             for i:=0 to ImplementedInterfaces.Count-1 do
               begin
                 ImplIntf:=TImplementedInterface(ImplementedInterfaces[i]);
                 ppufile.putderef(ImplIntf.intfdefderef);
                 ppufile.putderef(ImplIntf.ImplementsGetterDeref);
                 ppufile.putlongint(ImplIntf.Ioffset);
                 ppufile.putbyte(byte(ImplIntf.IType));
               end;
           end;

         if df_copied_def in defoptions then
           ppufile.putderef(cloneddefderef);

         writeentry(ppufile,ibobjectdef);

         if not(df_copied_def in defoptions) then
           tObjectSymtable(symtable).ppuwrite(ppufile);

         ppufile.do_indirect_crc:=old_do_indirect_crc;
      end;


    function tobjectdef.GetTypeName:string;
      begin
        { in this case we will go in endless recursion, because then  }
        { there is no tsym associated yet with the def. It can occur  }
        { (tests/webtbf/tw4757.pp), so for now give a generic name    }
        { instead of the actual type name                             }
        if not assigned(typesym) then
          result:='<Currently Parsed Class>'
        else
          result:=typesymbolprettyname;
      end;


    procedure tobjectdef.buildderef;
      var
         i : longint;
         vmtentry : pvmtentry;
      begin
         inherited buildderef;
         vmt_fieldderef.build(vmt_field);
         childofderef.build(childof);
         if df_copied_def in defoptions then
           cloneddefderef.build(symtable.defowner)
         else
           tstoredsymtable(symtable).buildderef;

         if objecttype=odt_helper then
           extendeddefderef.build(extendeddef);

         for i:=0 to vmtentries.count-1 do
           begin
             vmtentry:=pvmtentry(vmtentries[i]);
             vmtentry^.procdefderef.build(vmtentry^.procdef);
           end;

         if assigned(ImplementedInterfaces) then
           begin
             for i:=0 to ImplementedInterfaces.count-1 do
               TImplementedInterface(ImplementedInterfaces[i]).buildderef;
           end;
      end;


    procedure tobjectdef.deref;
      var
         i : longint;
         vmtentry : pvmtentry;
      begin
         inherited deref;
         vmt_field:=tsym(vmt_fieldderef.resolve);
         childof:=tobjectdef(childofderef.resolve);
         if df_copied_def in defoptions then
           begin
             cloneddef:=tobjectdef(cloneddefderef.resolve);
             symtable:=cloneddef.symtable.getcopy;
           end
         else
           tstoredsymtable(symtable).deref(false);
         if objecttype=odt_helper then
           extendeddef:=tdef(extendeddefderef.resolve);
         for i:=0 to vmtentries.count-1 do
           begin
             vmtentry:=pvmtentry(vmtentries[i]);
             vmtentry^.procdef:=tprocdef(vmtentry^.procdefderef.resolve);
           end;
         if assigned(ImplementedInterfaces) then
           begin
             for i:=0 to ImplementedInterfaces.count-1 do
               TImplementedInterface(ImplementedInterfaces[i]).deref;
           end;
      end;


    procedure create_class_helper_for_procdef(def: tobject; arg: pointer);
      var
        pd: tprocdef absolute def;
        st: tsymtable;
        psym: tsym;
        nname: TIDString;
      begin
        if (tdef(def).typ<>procdef) then
          exit;
        { pd.owner = objcclass symtable -> defowner = objcclassdef ->
          owner = symtable in which objcclassdef is defined
        }
        st:=pd.owner.defowner.owner;
        nname:=class_helper_prefix+tprocsym(pd.procsym).name;
        { check for an existing procsym with our special name }
        psym:=tsym(st.find(nname));
        if not assigned(psym) then
          begin
            psym:=cprocsym.create(nname);
            { avoid warning about this symbol being unused }
            psym.IncRefCount;
            { don't check for duplicates:
               a) we checked above
               b) in case we are in the implementation section of a unit, this
                  will also check for this symbol in the interface section
                  (since you normally cannot have symbols with the same name
                   both interface and implementation), and it's possible to
                   have class helpers for the same class in the interface and
                   in the implementation, and they cannot be merged since only
                   the once in the interface must be saved to the ppu/visible
                   from other units }
            st.insert(psym,false);
          end
        else if (psym.typ<>procsym) then
          internalerror(2009111501);
        { add ourselves to this special procsym }
        tprocsym(psym).procdeflist.add(def);
      end;


    procedure tobjectdef.derefimpl;
      begin
         inherited derefimpl;
         { the procdefs are not owned by the class helper procsyms, so they
           are not stored/restored either -> re-add them here }
         if (objecttype in [odt_objcclass,odt_objcprotocol]) or
            (oo_is_classhelper in objectoptions) then
           symtable.DefList.ForEachCall(@create_class_helper_for_procdef,nil);
      end;


    procedure tobjectdef.copyvmtentries(objdef:tobjectdef);
      var
        i : longint;
        vmtentry : pvmtentry;
      begin
        if vmtentries.count<>0 then
          internalerror(2019081401);
        vmtentries.count:=objdef.vmtentries.count;
        for i:=0 to objdef.vmtentries.count-1 do
          begin
            new(vmtentry);
            vmtentry^:=pvmtentry(objdef.vmtentries[i])^;
            vmtentries[i]:=vmtentry;
          end;
       end;


    function tobjectdef.getparentdef:tdef;
      begin
{ TODO: Remove getparentdef hack}
        { With 2 forward declared classes with the child class before the
          parent class the child class is written earlier to the ppu. Leaving it
          possible to have a reference to the parent class for property overriding,
          but the parent class still has the childof not resolved yet (PFV) }
        if childof=nil then
          childof:=tobjectdef(childofderef.resolve);
        result:=childof;
      end;


    procedure tobjectdef.prepareguid;
      begin
        { set up guid }
        if not assigned(iidguid) then
         begin
            new(iidguid);
            fillchar(iidguid^,sizeof(iidguid^),0); { default null guid }
         end;
        { setup iidstring }
        if not assigned(iidstr) then
          iidstr:=stringdup(''); { default is empty string }
      end;


    procedure tobjectdef.set_parent( c : tobjectdef);
      begin
        if assigned(childof) then
          exit;
        childof:=c;
        if not assigned(c) then
          exit;
        { inherit options and status }
        objectoptions:=objectoptions+(c.objectoptions*inherited_objectoptions);
        { initially has the same number of abstract methods as the parent }
        abstractcnt:=c.abstractcnt;
        { add the data of the anchestor class/object }
        if (objecttype in [odt_class,odt_object,odt_objcclass,odt_javaclass]) then
          begin
            tObjectSymtable(symtable).datasize:=tObjectSymtable(symtable).datasize+tObjectSymtable(c.symtable).datasize;
            { inherit recordalignment }
            tObjectSymtable(symtable).recordalignment:=tObjectSymtable(c.symtable).recordalignment;
            { if both the parent and this record use C-alignment, also inherit
              the current field alignment }
            if (tObjectSymtable(c.symtable).usefieldalignment=C_alignment) and
               (tObjectSymtable(symtable).usefieldalignment=C_alignment) then
              tObjectSymtable(symtable).fieldalignment:=tObjectSymtable(c.symtable).fieldalignment;
            { the padding is not inherited for Objective-C classes (maybe not
              for cppclass either?) }
            if objecttype=odt_objcclass then
              tObjectSymtable(symtable).datasize:=tObjectSymtable(symtable).datasize-tObjectSymtable(c.symtable).paddingsize;
            if (oo_has_vmt in objectoptions) and
               (oo_has_vmt in c.objectoptions) then
              tObjectSymtable(symtable).datasize:=tObjectSymtable(symtable).datasize-sizeof(pint);
            { if parent has a vmt field then the offset is the same for the child PM }
            if (oo_has_vmt in c.objectoptions) or is_class(self) then
              begin
                vmt_field:=c.vmt_field;
                include(objectoptions,oo_has_vmt);
              end;
          end;
      end;


   procedure tobjectdef.insertvmt;
     begin
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
          exit;
        if (oo_has_vmt in objectoptions) then
          internalerror(12345)
        else
          begin
             tObjectSymtable(symtable).datasize:=align(tObjectSymtable(symtable).datasize,
                 tObjectSymtable(symtable).fieldalignment);

             if (tf_requires_proper_alignment in target_info.flags) then
               begin
                 { Align VMT pointer and whole object instance if target CPU requires alignment. }
                 tObjectSymtable(symtable).datasize:=align(tObjectSymtable(symtable).datasize,sizeof(pint));
                 tObjectSymtable(symtable).alignrecord(tObjectSymtable(symtable).datasize,sizeof(pint));
               end;
             vmt_field:=cfieldvarsym.create('_vptr$'+objname^,vs_value,voidpointertype,[]);
             hidesym(vmt_field);
             tObjectSymtable(symtable).insert(vmt_field);
             tObjectSymtable(symtable).addfield(tfieldvarsym(vmt_field),vis_hidden);
             include(objectoptions,oo_has_vmt);
          end;
     end;


   function tobjectdef.vmt_offset: asizeint;
     begin
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
          result:=0
        else if (tObjectSymtable(symtable).usefieldalignment<>bit_alignment) then
          result:=tfieldvarsym(vmt_field).fieldoffset
        else
          result:=tfieldvarsym(vmt_field).fieldoffset div 8;
     end;


   procedure tobjectdef.check_forwards;
     begin
        if not(objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcprotocol,odt_interfacejava]) then
          inherited;
        if (oo_is_forward in objectoptions) then
          begin
             { ok, in future, the forward can be resolved }
             Message1(sym_e_class_forward_not_resolved,objrealname^);
             exclude(objectoptions,oo_is_forward);
          end;
     end;


   function tobjectdef.find_destructor: tprocdef;
     var
       objdef: tobjectdef;
     begin
        objdef:=self;
        while assigned(objdef) do
          begin
            result:=objdef.find_procdef_bytype(potype_destructor);
            if assigned(result) then
              exit;
            objdef:=objdef.childof;
          end;
        result:=nil;
     end;

    function tobjectdef.implements_any_interfaces: boolean;
      begin
        result := (ImplementedInterfaces.Count > 0) or
          (assigned(childof) and childof.implements_any_interfaces);
      end;

    function tobjectdef.size : asizeint;
      begin
        if objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_helper,odt_javaclass,odt_interfacejava] then
          result:=voidpointertype.size
        else
          result:=tObjectSymtable(symtable).datasize;
      end;


    function tobjectdef.alignment:shortint;
      begin
        if objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_helper,odt_javaclass,odt_interfacejava] then
          alignment:=voidpointertype.alignment
        else
          alignment:=tObjectSymtable(symtable).recordalignment;
      end;


    function tobjectdef.vmtmethodoffset(index:longint):longint;
      begin
        { for offset of methods for classes, see rtl/inc/objpash.inc }
        case objecttype of
        odt_class:
          { the +2*sizeof(pint) is size and -size }
          vmtmethodoffset:=index*voidcodepointertype.size+10*voidpointertype.size+2*sizeof(pint);
        odt_helper,
        odt_objcclass,
        odt_objcprotocol:
          vmtmethodoffset:=0;
        odt_interfacecom,odt_interfacecorba,odt_dispinterface:
          vmtmethodoffset:=index*voidcodepointertype.size;
        odt_javaclass,
        odt_interfacejava:
          { invalid }
          vmtmethodoffset:=-1;
        else
          { the +2*sizeof(pint) is size and -size }
{$ifdef WITHDMT}
          vmtmethodoffset:=index*voidcodepointertype.size+2*voidpointertype.size+2*sizeof(pint);
{$else WITHDMT}
          vmtmethodoffset:=index*voidcodepointertype.size+1*voidpointertype.size+2*sizeof(pint);
{$endif WITHDMT}
        end;
      end;


    function tobjectdef.vmt_mangledname : TSymStr;
      begin
        if not(oo_has_vmt in objectoptions) then
          Message1(parser_n_object_has_no_vmt,objrealname^);
        vmt_mangledname:=make_mangledname('VMT',owner,objname^);
      end;


    function tobjectdef.vmt_def: trecorddef;
      var
        vmttypesym: tsymentry;
      begin
        if not(typesym.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
          vmttypesym:=typesym.owner.Find('vmtdef$'+mangledparaname)
        else
          { Use common parent of trecordsymtable and tobjectsymtable
            to avoid invalid typecast error when compiled with -CR option }
          vmttypesym:=tabstractrecordsymtable(typesym.owner).get_unit_symtable.Find('vmtdef$'+mangledparaname);
        if not assigned(vmttypesym) or
           (vmttypesym.typ<>symconst.typesym) or
           (ttypesym(vmttypesym).typedef.typ<>recorddef) then
          internalerror(2015052501);
        result:=trecorddef(ttypesym(vmttypesym).typedef);
      end;


    function tobjectdef.needs_inittable : boolean;
      var
        hp : tobjectdef;
      begin
         case objecttype of
            odt_helper,
            odt_class :
              needs_inittable:=false;
            odt_dispinterface,
            odt_interfacecom:
              needs_inittable:=true;
            odt_interfacecorba:
              begin
                hp:=childof;
                while assigned(hp) do
                  begin
                    if hp=interface_iunknown then
                      begin
                        needs_inittable:=true;
                        exit;
                      end;
                    hp:=hp.childof;
                  end;
                needs_inittable:=false;
              end;
            odt_object:
              needs_inittable:=
                tObjectSymtable(symtable).needs_init_final or
                (assigned(childof) and
                 childof.needs_inittable);
            odt_cppclass,
            odt_objcclass,
            odt_objcprotocol,
            odt_javaclass,
            odt_interfacejava:
              needs_inittable:=false;
            else
              internalerror(200108267);
         end;
      end;

    function tobjectdef.needs_separate_initrtti : boolean;
      begin
        result:=not (objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface]);
      end;


    function tobjectdef.has_non_trivial_init_child(check_parent:boolean):boolean;
      begin
        if objecttype in [odt_class,odt_object] then
          begin
            result:=tobjectsymtable(symtable).has_non_trivial_init or
                      (check_parent and assigned(childof) and childof.has_non_trivial_init_child(true));
          end
        else
          result:=false;
      end;


    function tobjectdef.rtti_mangledname(rt: trttitype): TSymStr;
      begin
        if not(objecttype in [odt_objcclass,odt_objcprotocol]) then
          result:=inherited rtti_mangledname(rt)
        else
          begin
            { necessary in case of a dynamic array of nsobject, or
              if an nsobject field appears in a record that needs
              init/finalisation }
            if rt=initrtti then
              begin
                result:=voidpointertype.rtti_mangledname(rt);
                exit;
              end;

            if not(target_info.system in systems_objc_nfabi) then
              begin
                result:=target_asm.labelprefix;
                case objecttype of
                  odt_objcclass:
                    begin
                      case rt of
                        objcclassrtti:
                          if not(oo_is_classhelper in objectoptions) then
                            result:=result+'_OBJC_CLASS_'
                          else
                            result:=result+'_OBJC_CATEGORY_';
                        objcmetartti:
                          if not(oo_is_classhelper in objectoptions) then
                            result:=result+'_OBJC_METACLASS_'
                          else
                            internalerror(2009111511);
                        else
                         internalerror(2009092302);
                      end;
                    end;
                  odt_objcprotocol:
                    result:=result+'_OBJC_PROTOCOL_';
                  else
                   ;
                end;
              end
            else
              begin
                case objecttype of
                  odt_objcclass:
                    begin
                      if (oo_is_classhelper in objectoptions) and
                         (rt<>objcclassrtti) then
                        internalerror(2009111512);
                      case rt of
                        objcclassrtti:
                          if not(oo_is_classhelper in objectoptions) then
                            result:='_OBJC_CLASS_$_'
                          else
                            result:='_OBJC_$_CATEGORY_';
                        objcmetartti:
                          result:='_OBJC_METACLASS_$_';
                        objcclassrortti:
                          result:=lower(target_asm.labelprefix)+'_OBJC_CLASS_RO_$_';
                        objcmetarortti:
                          result:=lower(target_asm.labelprefix)+'_OBJC_METACLASS_RO_$_';
                        else
                         internalerror(2009092303);
                      end;
                    end;
                  odt_objcprotocol:
                    begin
                      result:=lower(target_asm.labelprefix);
                      case rt of
                        objcclassrtti:
                          result:=result+'_OBJC_PROTOCOL_$_';
                        objcmetartti:
                          result:=result+'_OBJC_LABEL_PROTOCOL_$_';
                        else
                          internalerror(2009092501);
                      end;
                    end;
                  else
                    internalerror(2013113005);
                end;
              end;
            result:=result+objextname^;
          end;
      end;


    function tobjectdef.members_need_inittable : boolean;
      begin
        members_need_inittable:=tObjectSymtable(symtable).needs_init_final;
      end;


    function tobjectdef.is_publishable : boolean;
      begin
         is_publishable:=objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface];
      end;


    function tobjectdef.get_next_dispid: longint;
      begin
        inc(fcurrent_dispid);
        result:=fcurrent_dispid;
      end;

    function tobjectdef.search_enumerator_get: tprocdef;
      begin
        result:=inherited;
        if not assigned(result) and assigned(childof) then
          result:=childof.search_enumerator_get;
      end;

    function tobjectdef.search_enumerator_move: tprocdef;
      begin
        result:=inherited;
        if not assigned(result) and assigned(childof) then
          result:=childof.search_enumerator_move;
      end;

    function tobjectdef.search_enumerator_current: tsym;
      begin
        result:=inherited;
        if not assigned(result) and assigned(childof) then
          result:=childof.search_enumerator_current;
      end;

    procedure tobjectdef.register_created_classref_type;
      begin
        if not classref_created_in_current_module then
          begin
            classref_created_in_current_module:=true;
            current_module.wpoinfo.addcreatedobjtypeforclassref(self);
          end;
      end;


    procedure tobjectdef.register_created_object_type;
      begin
        if not created_in_current_module then
          begin
            created_in_current_module:=true;
            current_module.wpoinfo.addcreatedobjtype(self);
          end;
      end;


    procedure tobjectdef.register_maybe_created_object_type;
      begin
        { if we know it has been created for sure, no need
          to also record that it maybe can be created in
          this module
        }
        if not (created_in_current_module) and
           not (maybe_created_in_current_module) then
          begin
            maybe_created_in_current_module:=true;
            current_module.wpoinfo.addmaybecreatedbyclassref(self);
          end;
      end;


    procedure tobjectdef.register_vmt_call(index: longint);
      begin
        if (is_object(self) or is_class(self)) then
          current_module.wpoinfo.addcalledvmtentry(self,index);
      end;


    procedure check_and_finish_msg(data: tobject; arg: pointer);
      var
        def: tdef absolute data;
        pd: tprocdef absolute data;
        i,
        paracount: longint;
      begin
        if (def.typ=procdef) then
          begin
            { add all messages also under a dummy name to the symtable in
              which the objcclass/protocol/category is declared, so they can
              be called via id.<name>
            }
            create_class_helper_for_procdef(pd,nil);

            { we have to wait until now to set the mangled name because it
              depends on the (possibly external) class name, which is defined
              at the very end.  }
            if not(po_msgstr in pd.procoptions) then
              begin
                CGMessagePos(pd.fileinfo,parser_e_objc_requires_msgstr);
                { recover to avoid internalerror later on }
                include(pd.procoptions,po_msgstr);
                pd.messageinf.str:=stringdup('MissingDeclaration');
              end;
            { Mangled name is already set in case this is a copy of
              another type.  }
            if not(po_has_mangledname in pd.procoptions) then
              begin
                { check whether the number of formal parameters is correct,
                  and whether they have valid Objective-C types }
                paracount:=0;
                for i:=1 to length(pd.messageinf.str^) do
                  if pd.messageinf.str^[i]=':' then
                    inc(paracount);
                for i:=0 to pd.paras.count-1 do
                  if not(vo_is_hidden_para in tparavarsym(pd.paras[i]).varoptions) and
                     not is_array_of_const(tparavarsym(pd.paras[i]).vardef) then
                    dec(paracount);
                if (paracount<>0) then
                  MessagePos(pd.fileinfo,sym_e_objc_para_mismatch);

                pd.setmangledname(pd.objcmangledname);
              end
            else
              { all checks already done }
              exit;
            if not(oo_is_external in pd.struct.objectoptions) then
              begin
                if (po_varargs in pd.procoptions) then
                  MessagePos(pd.fileinfo,parser_e_varargs_need_cdecl_and_external)
                else
                  begin
                    { check for "array of const" parameters }
                    for i:=0 to pd.parast.symlist.count-1 do
                      begin
                        if (tsym(pd.parast.symlist[i]).typ=paravarsym) and
                           is_array_of_const(tparavarsym(pd.parast.symlist[i]).vardef) then
                          MessagePos(pd.fileinfo,parser_e_varargs_need_cdecl_and_external);
                      end;
                  end;
              end;
          end;
      end;


    procedure mark_private_fields_used(data: tobject; arg: pointer);
      var
        sym: tsym absolute data;
      begin
        if (sym.typ=fieldvarsym) and
           (tfieldvarsym(sym).visibility in [vis_private,vis_strictprivate]) then
          sym.IncRefCount;
      end;


    procedure tobjectdef.finish_objc_data;
      begin
        self.symtable.DefList.foreachcall(@check_and_finish_msg,nil);
        if (oo_is_external in objectoptions) then
          self.symtable.SymList.ForEachCall(@mark_private_fields_used,nil);
      end;


    procedure verify_objc_vardef(data: tobject; arg: pointer);
      var
        sym: tabstractvarsym absolute data;
        res: pboolean absolute arg;
        founderrordef: tdef;
      begin
        if not(tsym(data).typ in [paravarsym,fieldvarsym]) then
          exit;
        if (sym.typ=paravarsym) and
           ((vo_is_hidden_para in tparavarsym(sym).varoptions) or
            is_array_of_const(tparavarsym(sym).vardef)) then
          exit;
        if not objcchecktype(sym.vardef,founderrordef) then
          begin
            MessagePos1(sym.fileinfo,type_e_objc_type_unsupported,founderrordef.typename);
            res^:=false;
          end;
      end;


    procedure verify_objc_procdef_paras(data: tobject; arg: pointer);
      var
        def: tdef absolute data;
        res: pboolean absolute arg;
        founderrordef: tdef;
      begin
        if (def.typ<>procdef) then
          exit;
        { check parameter types for validity }
        tprocdef(def).paras.foreachcall(@verify_objc_vardef,arg);
        { check the result type for validity }
        if not objcchecktype(tprocdef(def).returndef,founderrordef) then
          begin
            MessagePos1(tprocdef(def).funcretsym.fileinfo,type_e_objc_type_unsupported,founderrordef.typename);
            res^:=false;
          end;
      end;


    function tobjectdef.check_objc_types: boolean;
      begin
        { done in separate step from finish_objc_data, because when
          finish_objc_data is called, not all forwarddefs have been resolved
          yet and we need to know all types here }
        result:=true;
        self.symtable.symlist.foreachcall(@verify_objc_vardef,@result);
        self.symtable.deflist.foreachcall(@verify_objc_procdef_paras,@result);
      end;


    procedure do_cpp_import_info(data: tobject; arg: pointer);
      var
        def: tdef absolute data;
        pd: tprocdef absolute data;
      begin
        if (def.typ=procdef) then
          begin
            pd.setmangledname(target_info.Cprefix+pd.cplusplusmangledname);
            if (oo_is_external in pd.struct.objectoptions) then
              begin
                { copied from psub.read_proc }
                if assigned(tobjectdef(pd.struct).import_lib) then
                   current_module.AddExternalImport(tobjectdef(pd.struct).import_lib^,pd.mangledname,pd.mangledname,0,false,false)
                 else
                   begin
                     { add import name to external list for DLL scanning }
                     if tf_has_dllscanner in target_info.flags then
                       current_module.dllscannerinputlist.Add(pd.mangledname,pd);
                   end;

              end;
          end;
      end;


    procedure tobjectdef.finish_cpp_data;
      begin
        self.symtable.DefList.ForEachCall(@do_cpp_import_info,nil);
      end;


{****************************************************************************
                             TImplementedInterface
****************************************************************************}

    function TImplementedInterface.GetIOffset: longint;
      begin
        if (fIOffset=-1) and
           (IType in [etFieldValue,etFieldValueClass]) then
          result:=tfieldvarsym(ImplementsField).fieldoffset
        else
          result:=fIOffset;
      end;


    constructor TImplementedInterface.create(aintf: tobjectdef);
      begin
        inherited create;
        intfdef:=aintf;
        intfdefderef.reset;
        IOffset:=-1;
        IType:=etStandard;
        NameMappings:=nil;
        procdefs:=nil;
      end;


    constructor TImplementedInterface.create_deref(intfd,getterd:tderef);
      begin
        inherited create;
        intfdef:=nil;
        intfdefderef:=intfd;
        ImplementsGetterDeref:=getterd;
        IOffset:=-1;
        IType:=etStandard;
        NameMappings:=nil;
        procdefs:=nil;
      end;


    destructor  TImplementedInterface.destroy;
      var
        i : longint;
        mappedname : pshortstring;
      begin
        if assigned(NameMappings) then
          begin
            for i:=0 to NameMappings.Count-1 do
              begin
                mappedname:=pshortstring(NameMappings[i]);
                stringdispose(mappedname);
              end;
            NameMappings.free;
            NameMappings:=nil;
          end;
        if assigned(procdefs) then
          begin
            procdefs.free;
            procdefs:=nil;
          end;
        inherited destroy;
      end;


    procedure TImplementedInterface.buildderef;
      begin
        intfdefderef.build(intfdef);
        ImplementsGetterDeref.build(ImplementsGetter);
      end;


    procedure TImplementedInterface.deref;
      begin
        intfdef:=tobjectdef(intfdefderef.resolve);
        ImplementsGetter:=tsym(ImplementsGetterDeref.resolve);
      end;


    procedure TImplementedInterface.AddMapping(const origname,newname: string);
      begin
        if not assigned(NameMappings) then
          NameMappings:=TFPHashList.Create;
        NameMappings.Add(origname,stringdup(newname));
      end;


    function TImplementedInterface.GetMapping(const origname: string):string;
      var
        mappedname : pshortstring;
      begin
        result:='';
        if not assigned(NameMappings) then
          exit;
        mappedname:=PShortstring(NameMappings.Find(origname));
        if assigned(mappedname) then
          result:=mappedname^;
      end;


    procedure TImplementedInterface.AddImplProc(pd:tprocdef);
      begin
        if not assigned(procdefs) then
          procdefs:=TFPObjectList.Create(false);
        { duplicate entries must be stored, because multiple }
        { interfaces can declare methods with the same name  }
        { and all of these get their own VMT entry           }
        procdefs.Add(pd);
      end;


    function TImplementedInterface.IsImplMergePossible(MergingIntf:TImplementedInterface;out weight: longint): boolean;
      var
        i : longint;
      begin
        result:=false;
        { interfaces being implemented through delegation are not mergable (FK) }
        if (IType<>etStandard) or (MergingIntf.IType<>etStandard) or not(assigned(ProcDefs)) or not(assigned(MergingIntf.ProcDefs)) then
          exit;
        weight:=0;
        { empty interface is mergeable }
        if ProcDefs.Count=0 then
          begin
            result:=true;
            exit;
          end;
        { The interface to merge must at least the number of
          procedures of this interface }
        if MergingIntf.ProcDefs.Count<ProcDefs.Count then
          exit;
        for i:=0 to ProcDefs.Count-1 do
          begin
            if MergingIntf.ProcDefs[i]<>ProcDefs[i] then
              exit;
          end;
        weight:=ProcDefs.Count;
        result:=true;
      end;


    function TImplementedInterface.getcopy:TImplementedInterface;
      begin
        Result:=TImplementedInterface.Create(nil);
        { 1) the procdefs list will be freed once for each copy
          2) since the procdefs list owns its elements, those will also be freed for each copy
          3) idem for the name mappings
        }
        { warning: this is completely wrong on so many levels...
        Move(pointer(self)^,pointer(result)^,InstanceSize);
        We need to make clean copies of the different fields
        this is not implemented yet, and thus we generate an internal
        error instead PM 2011-06-14 }
        internalerror(2011061401);
      end;

{****************************************************************************
                                TFORWARDDEF
****************************************************************************}

   constructor tforwarddef.create(const s:string;const pos:tfileposinfo);
     begin
        inherited create(forwarddef,true);
        tosymname:=stringdup(s);
        forwardpos:=pos;
     end;


    function tforwarddef.GetTypeName:string;
      begin
        GetTypeName:='unresolved forward to '+tosymname^;
      end;


    destructor tforwarddef.destroy;
      begin
        stringdispose(tosymname);
        inherited destroy;
      end;

    function tforwarddef.getcopy:tstoreddef;
      begin
        result:=cforwarddef.create(tosymname^, forwardpos);
      end;

{****************************************************************************
                               TUNDEFINEDDEF
****************************************************************************}

   constructor tundefineddef.create(doregister:boolean);
     begin
        inherited create(undefineddef,doregister);
     end;


    constructor tundefineddef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(undefineddef,ppufile);
         ppuload_platform(ppufile);
      end;

    function tundefineddef.GetTypeName:string;
      begin
        GetTypeName:='<undefined type>';
      end;


    procedure tundefineddef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         writeentry(ppufile,ibundefineddef);
      end;


{****************************************************************************
                                  TERRORDEF
****************************************************************************}

    constructor terrordef.create;
      begin
        inherited create(errordef,true);
        { prevent consecutive faults }
        savesize:=1;
      end;


    procedure terrordef.ppuwrite(ppufile:tcompilerppufile);
      begin
        { Can't write errordefs to ppu }
        internalerror(200411063);
      end;


    function terrordef.GetTypeName:string;
      begin
        GetTypeName:='<erroneous type>';
      end;


    function terrordef.getmangledparaname:TSymStr;
      begin
        getmangledparaname:='error';
      end;


{****************************************************************************
                           Definition Helpers
****************************************************************************}

    function is_interfacecom(def: tdef): boolean;
      begin
        is_interfacecom:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_interfacecom);
      end;

    function is_interfacecom_or_dispinterface(def: tdef): boolean;
      begin
        is_interfacecom_or_dispinterface:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_interfacecom,odt_dispinterface]);
      end;

    function is_any_interface_kind(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          ((tobjectdef(def).objecttype in [odt_interfacecom,odt_dispinterface,odt_interfacecorba,odt_interfacejava,odt_objcprotocol]) or
           is_objccategory(def));
      end;

    function is_interfacecorba(def: tdef): boolean;
      begin
        is_interfacecorba:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_interfacecorba);
      end;

    function is_interface(def: tdef): boolean;
      begin
        is_interface:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_interfacecom,odt_interfacecorba]);
      end;


    function is_dispinterface(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_dispinterface);
      end;


    function is_class(def: tdef): boolean;
      begin
        is_class:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_class);
      end;


    function is_object(def: tdef): boolean;
      begin
        is_object:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_object);
      end;


    function is_cppclass(def: tdef): boolean;
      begin
        is_cppclass:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_cppclass);
      end;


    function is_objcclass(def: tdef): boolean;
      begin
        is_objcclass:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_objcclass);
      end;


    function is_objectpascal_helper(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_helper);
      end;


    function is_objcclassref(def: tdef): boolean;
      begin
        is_objcclassref:=
          assigned(def) and
          (def.typ=classrefdef) and
          is_objcclass(tclassrefdef(def).pointeddef);
      end;


    function is_objcprotocol(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_objcprotocol);
      end;


    function is_objccategory(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          { if used as a forward type }
          ((tobjectdef(def).objecttype=odt_objccategory) or
          { if used as after it has been resolved }
           ((tobjectdef(def).objecttype=odt_objcclass) and
            (oo_is_classhelper in tobjectdef(def).objectoptions)));
      end;

    function is_objc_class_or_protocol(def: tdef): boolean;
      begin
         result:=
           assigned(def) and
           (def.typ=objectdef) and
           (tobjectdef(def).objecttype in [odt_objcclass,odt_objcprotocol]);
      end;


    function is_objc_protocol_or_category(def: tdef): boolean;
      begin
         result:=
           assigned(def) and
           (def.typ=objectdef) and
           ((tobjectdef(def).objecttype = odt_objcprotocol) or
            ((tobjectdef(def).objecttype = odt_objcclass) and
             (oo_is_classhelper in tobjectdef(def).objectoptions)));
      end;

    function is_classhelper(def: tdef): boolean;
      begin
         result:=
           is_objectpascal_helper(def) or
           is_objccategory(def);
      end;

    function is_class_or_interface(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba]);
      end;


    function is_class_or_interface_or_objc(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_objcclass,odt_objcprotocol]);
      end;


    function is_class_or_interface_or_objc_or_java(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava]);
      end;


    function is_class_or_interface_or_dispinterface_or_objc_or_java(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava]);
      end;


    function is_class_or_interface_or_object(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_object]);
      end;


    function is_class_or_interface_or_dispinterface(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface]);
      end;


    function is_implicit_pointer_object_type(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (((def.typ=objectdef) and
            (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_helper,odt_javaclass,odt_interfacejava])) or
           ((target_info.system in systems_jvm) and
            (def.typ=recorddef)));
      end;

    function is_implicit_array_pointer(def: tdef): boolean;
      begin
        result:=is_dynamic_array(def) or is_dynamicstring(def);
      end;

    function is_class_or_object(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_object]);
      end;

    function is_record(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=recorddef);
      end;

    function is_javaclass(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_javaclass);
      end;

    function is_javaclassref(def: tdef): boolean;
      begin
        is_javaclassref:=
          assigned(def) and
          (def.typ=classrefdef) and
          is_javaclass(tclassrefdef(def).pointeddef);
      end;

    function is_javainterface(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_interfacejava);
      end;

    function is_java_class_or_interface(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_javaclass,odt_interfacejava]);
      end;

    procedure loadobjctypes;
      begin
        objc_metaclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_CLASS',true).typedef);
        objc_superclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_SUPER',true).typedef);
        objc_idtype:=tpointerdef(search_named_unit_globaltype('OBJC','ID',true).typedef);
        objc_seltype:=tpointerdef(search_named_unit_globaltype('OBJC','SEL',true).typedef);
        objc_objecttype:=trecorddef(search_named_unit_globaltype('OBJC','OBJC_OBJECT',true).typedef);
      end;


    procedure maybeloadcocoatypes;
      var
        tsym: ttypesym;
        cocoaunit: string[15];
      begin
        if assigned(objc_fastenumeration) then
          exit;
        if not(target_info.system in [system_arm_darwin,system_i386_iphonesim,system_aarch64_darwin,system_x86_64_iphonesim]) then
          cocoaunit:='COCOAALL'
        else
          cocoaunit:='IPHONEALL';
        tsym:=search_named_unit_globaltype(cocoaunit,'NSFASTENUMERATIONPROTOCOL',false);
        if assigned(tsym) then
          objc_fastenumeration:=tobjectdef(tsym.typedef)
        else
          objc_fastenumeration:=nil;
        tsym:=search_named_unit_globaltype(cocoaunit,'NSFASTENUMERATIONSTATE',false);
        if assigned(tsym) then
          objc_fastenumerationstate:=trecorddef(tsym.typedef)
        else
          objc_fastenumerationstate:=nil;
      end;


    function use_vectorfpu(def : tdef) : boolean;
      begin
{$ifdef x86}
{$define use_vectorfpuimplemented}
        use_vectorfpu:=(is_single(def) and (current_settings.fputype in sse_singlescalar)) or
          (is_double(def) and (current_settings.fputype in sse_doublescalar));
{$endif x86}
{$ifdef arm}
{$define use_vectorfpuimplemented}
        use_vectorfpu:=FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype];
{$endif arm}
{$ifdef aarch64}
{$define use_vectorfpuimplemented}
        use_vectorfpu:=true;
{$endif aarch64}

{$ifndef use_vectorfpuimplemented}
        use_vectorfpu:=false;
{$endif}
      end;


end.
