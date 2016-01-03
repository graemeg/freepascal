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
       cclasses,
       { global }
       globtype,globals,tokens,constexp,
       { symtable }
       symconst,symbase,symtype,
       { ppu }
       ppu,
       { node }
       node,
       { aasm }
       aasmbase,aasmtai,
       cpubase,cpuinfo,
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
          constructor create(dt:tdeftyp;doregister:boolean);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          destructor  destroy;override;
          function getcopy : tstoreddef;virtual;
          procedure ppuwrite(ppufile:tcompilerppufile);virtual;
          procedure buildderef;override;
          procedure buildderefimpl;override;
          procedure deref;override;
          procedure derefimpl;override;
          function  size:asizeint;override;
          function  getvardef:longint;override;
          function  alignment:shortint;override;
          function  is_publishable : boolean;override;
          function  needs_inittable : boolean;override;
<<<<<<< HEAD
          function  rtti_mangledname(rt:trttitype):TSymStr;override;
          function  OwnerHierarchyName: string; override;
          function  fullownerhierarchyname:TSymStr;override;
          function  needs_separate_initrtti:boolean;override;
=======
          function  rtti_mangledname(rt:trttitype):string;override;
          function  OwnerHierarchyName: string; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
          function is_generic:boolean;inline;
          { same as above for specializations }
          function is_specialization:boolean;inline;
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
       end;
       tfiledefclass = class of tfiledef;

       tvariantdef = class(tstoreddef)
          varianttype : tvarianttype;
          constructor create(v : tvarianttype);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          function GetTypeName:string;override;
<<<<<<< HEAD
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
=======
          procedure ppuwrite(ppufile:tcompilerppufile);override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          constructor create(const s:string;const pos:tfileposinfo);virtual;
=======
          constructor create(const s:string;const pos:tfileposinfo);
>>>>>>> graemeg/cpstrnew
=======
          constructor create(const s:string;const pos:tfileposinfo);
>>>>>>> graemeg/cpstrnew
=======
          constructor create(const s:string;const pos:tfileposinfo);
>>>>>>> graemeg/cpstrnew
=======
          constructor create(const s:string;const pos:tfileposinfo);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
=======
          is_far : boolean;
          has_pointer_math : boolean;
          constructor create(def:tdef);
          constructor createfar(def:tdef);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
          {# returns the int type produced when subtracting two pointers of the given type.
             Normally, this is sinttype, except on i8086, where it takes into account the
             special i8086 pointer types (near, far, huge). }
          function pointer_subtraction_result_type:tdef;virtual;
          function compatible_with_pointerdef_size(ptr: tpointerdef): boolean; virtual;
       end;
       tpointerdefclass = class of tpointerdef;

       tprocdef = class;

       tprocdef = class;
       { tabstractrecorddef }

       tprocdef = class;
       { tabstractrecorddef }

       tprocdef = class;
       { tabstractrecorddef }

       tprocdef = class;
       { tabstractrecorddef }

       tabstractrecorddef= class(tstoreddef)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       private
          rttistring     : string;
       public
          objname,
          objrealname    : PShortString;
          { for C++ classes: name of the library this class is imported from }
          { for Java classes/records: package name }
          import_lib     : PShortString;
=======
          objname,
          objrealname    : PShortString;
>>>>>>> graemeg/cpstrnew
=======
          objname,
          objrealname    : PShortString;
>>>>>>> graemeg/cpstrnew
=======
          objname,
          objrealname    : PShortString;
>>>>>>> graemeg/cpstrnew
=======
          objname,
          objrealname    : PShortString;
>>>>>>> origin/cpstrnew
          symtable       : TSymtable;
          cloneddef      : tabstractrecorddef;
          cloneddefderef : tderef;
          objectoptions  : tobjectoptions;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { for targets that initialise typed constants via explicit assignments
            instead of by generating an initialised data sectino }
          tcinitcode     : tnode;
          constructor create(const n:string; dt:tdeftyp;doregister:boolean);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          destructor destroy; override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          constructor create(const n:string; dt:tdeftyp);
          constructor ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          destructor destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          procedure check_forwards; virtual;
          function find_procdef_bytype(pt:tproctypeoption): tprocdef;
          function GetSymtable(t:tGetSymtable):TSymtable;override;
=======
          symtable : TSymtable;
          cloneddef      : tabstractrecorddef;
          cloneddefderef : tderef;
          procedure reset;override;
          function  GetSymtable(t:tGetSymtable):TSymtable;override;
>>>>>>> graemeg/fixes_2_2
          function is_packed:boolean;
          function RttiName: string;
          { enumerator support }
          function search_enumerator_get: tprocdef; virtual;
          function search_enumerator_move: tprocdef; virtual;
          function search_enumerator_current: tsym; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { JVM }
          function jvm_full_typename(with_package_name: boolean): string;
          { check if the symtable contains a float field }
          function contains_float_field : boolean;
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       end;

       trecorddef = class(tabstractrecorddef)
       public
          variantrecdesc : pvariantrecdesc;
          isunion       : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          constructor create(const n:string; p:TSymtable);virtual;
          constructor create_global_internal(n: string; packrecords, recordalignmin, maxCrecordalign: shortint); virtual;
          function add_field_by_def(const optionalname: TIDString; def: tdef): tsym;
          procedure add_fields_from_deflist(fieldtypes: tfplist);
=======
          constructor create(const n:string; p:TSymtable);
>>>>>>> graemeg/cpstrnew
=======
          constructor create(const n:string; p:TSymtable);
>>>>>>> graemeg/cpstrnew
=======
          constructor create(const n:string; p:TSymtable);
>>>>>>> graemeg/cpstrnew
=======
          constructor create(const n:string; p:TSymtable);
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { for Object Pascal helpers }
          extendeddef   : tdef;
          extendeddefderef: tderef;
=======
          { for C++ classes: name of the library this class is imported from }
          import_lib,
>>>>>>> graemeg/cpstrnew
=======
          { for C++ classes: name of the library this class is imported from }
          import_lib,
>>>>>>> graemeg/cpstrnew
=======
          { for C++ classes: name of the library this class is imported from }
          import_lib,
>>>>>>> graemeg/cpstrnew
=======
          { for C++ classes: name of the library this class is imported from }
          import_lib,
>>>>>>> origin/cpstrnew
          { for Objective-C: protocols and classes can have the same name there }
          objextname     : pshortstring;
=======
=======
>>>>>>> origin/fixes_2_2
          objname,
          objrealname    : pshortstring;
          objectoptions  : tobjectoptions;
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
          objecttype     : tobjecttyp;
          constructor create(ot:tobjecttyp;const n:string;c:tobjectdef;doregister:boolean);virtual;
=======
          { store implemented interfaces defs and name mappings }
          ImplementedInterfaces : TFPObjectList;
          constructor create(ot:tobjecttyp;const n:string;c:tobjectdef);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          constructor ppuload(ppufile:tcompilerppufile);
          destructor  destroy;override;
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function GetTypeName:string;override;
          procedure buildderef;override;
          procedure deref;override;
<<<<<<< HEAD
<<<<<<< HEAD
          procedure derefimpl;override;
          procedure resetvmtentries;
          procedure copyvmtentries(objdef:tobjectdef);
=======
          procedure buildderefimpl;override;
          procedure derefimpl;override;
>>>>>>> graemeg/fixes_2_2
=======
          procedure buildderefimpl;override;
          procedure derefimpl;override;
>>>>>>> origin/fixes_2_2
          function  getparentdef:tdef;override;
          function  size : asizeint;override;
          function  alignment:shortint;override;
          function  vmtmethodoffset(index:longint):longint;
          function  members_need_inittable : boolean;
          { this should be called when this class implements an interface }
          procedure prepareguid;
          function  is_publishable : boolean;override;
          function  is_related(d : tdef) : boolean;override;
          function  needs_inittable : boolean;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          function  needs_separate_initrtti : boolean;override;
          function  rtti_mangledname(rt:trttitype):TSymStr;override;
          function  vmt_mangledname : TSymStr;
          function  vmt_def: trecorddef;
=======
          function  rtti_mangledname(rt:trttitype):string;override;
          function  vmt_mangledname : string;
>>>>>>> graemeg/cpstrnew
=======
          function  rtti_mangledname(rt:trttitype):string;override;
          function  vmt_mangledname : string;
>>>>>>> graemeg/cpstrnew
=======
          function  rtti_mangledname(rt:trttitype):string;override;
          function  vmt_mangledname : string;
>>>>>>> graemeg/cpstrnew
=======
          function  rtti_mangledname(rt:trttitype):string;override;
          function  vmt_mangledname : string;
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function getcopy:tstoreddef;override;
          function GetTypeName:string;override;
          function is_publishable : boolean;override;
          function rtti_mangledname(rt:trttitype):TSymStr;override;
=======
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function getcopy:tstoreddef;override;
          function GetTypeName:string;override;
          function is_publishable : boolean;override;
          function rtti_mangledname(rt:trttitype):string;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
       public
<<<<<<< HEAD
          function elesize : asizeint;
          function elepackedbitsize : asizeint;
          function elecount : asizeuint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          constructor create_from_pointer(def:tpointerdef);virtual;
          constructor create(l,h:asizeint;def:tdef);virtual;
          class function getreusable(def: tdef; elems: asizeint): tarraydef; virtual;
          { same as above, but in case the def must never be freed after the
            current module has been compiled -- even if the def was not written
            to the ppu file (for defs in para locations, as we don't reset them
            so we don't have to recalculate them all the time) }
          class function getreusable_no_free(def: tdef; elems: asizeint): tarraydef;
=======
          constructor create_from_pointer(def:tdef);
          constructor create(l,h:asizeint;def:tdef);
>>>>>>> graemeg/cpstrnew
=======
          constructor create_from_pointer(def:tdef);
          constructor create(l,h:asizeint;def:tdef);
>>>>>>> graemeg/cpstrnew
=======
          constructor create_from_pointer(def:tdef);
          constructor create(l,h:asizeint;def:tdef);
>>>>>>> graemeg/cpstrnew
=======
=======
          function elesize : aint;
          function elepackedbitsize : aint;
          function elecount : aword;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          constructor create_from_pointer(def:tdef);
          constructor create(l,h:asizeint;def:tdef);
>>>>>>> origin/cpstrnew
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
         pno_mangledname);
       tprocnameoptions = set of tprocnameoption;
       tproccopytyp = (pc_normal,
                       { always creates a top-level function, removes all
                         special parameters (self, vmt, parentfp, ...) }
                       pc_bareproc,
                       { creates a procvardef describing only the code pointer
                         of a method/netsted function/... }
                       pc_address_only
                       );

       { tabstractprocdef }

       { tabstractprocdef }

       { tabstractprocdef }

       { tabstractprocdef }

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef m68k}
          exp_funcretloc : tregister;   { explicit funcretloc for AmigaOS }
{$endif}
          funcretloc : array[callerside..calleeside] of TCGPara;
          has_paraloc_info : tcallercallee; { paraloc info is available }
          { number of user visible parameters }
          maxparacount,
          minparacount    : byte;
          constructor create(dt:tdeftyp;level:byte;doregister:boolean);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          { number of user visibile parameters }
          maxparacount,
          minparacount    : byte;
{$ifdef m68k}
          exp_funcretloc : tregister;   { explicit funcretloc for AmigaOS }
{$endif}
          funcretloc : array[tcallercallee] of TCGPara;
          has_paraloc_info : tcallercallee; { paraloc info is available }
          constructor create(dt:tdeftyp;level:byte);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { get either a copy as a procdef or procvardef }
          function  getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp): tstoreddef; virtual;
          function  compatible_with_pointerdef_size(ptr: tpointerdef): boolean; virtual;
          procedure check_mark_as_nested;
          procedure init_paraloc_info(side: tcallercallee);
          procedure done_paraloc_info(side: tcallercallee);
          function stack_tainting_parameter(side: tcallercallee): boolean;
          function is_pushleftright: boolean;virtual;
          function address_type:tdef;virtual;
          procedure declared_far;virtual;
          procedure declared_near;virtual;
=======
          procedure check_mark_as_nested;
          procedure init_paraloc_info(side: tcallercallee);
          function stack_tainting_parameter(side: tcallercallee): boolean;
>>>>>>> graemeg/cpstrnew
=======
          procedure check_mark_as_nested;
          procedure init_paraloc_info(side: tcallercallee);
          function stack_tainting_parameter(side: tcallercallee): boolean;
>>>>>>> graemeg/cpstrnew
=======
          procedure check_mark_as_nested;
          procedure init_paraloc_info(side: tcallercallee);
          function stack_tainting_parameter(side: tcallercallee): boolean;
>>>>>>> graemeg/cpstrnew
=======
          procedure check_mark_as_nested;
          procedure init_paraloc_info(side: tcallercallee);
          function stack_tainting_parameter(side: tcallercallee): boolean;
>>>>>>> origin/cpstrnew
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
          function getcopyas(newtyp: tdeftyp; copytyp: tproccopytyp): tstoreddef; override;
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       timplprocdefinfo = record
          resultname : pshortstring;
          parentfpstruct: tsym;
          parentfpstructptrtype: tdef;
          parentfpinitblock: tnode;
          procstarttai,
          procendtai   : tai;
          skpara: pointer;
          forwarddef,
          interfacedef : boolean;
          hasforward  : boolean;
       end;
       pimplprocdefinfo = ^timplprocdefinfo;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
         function Getforwarddef: boolean;
         procedure Setforwarddef(AValue: boolean);
         function Getinterfacedef: boolean;
         procedure Setinterfacedef(AValue: boolean);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { generic support }
          genericdecltokenbuf : tdynamicarray;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          implprocoptions: timplprocoptions;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$if defined(powerpc) or defined(m68k)}
          { library symbol for AmigaOS/MorphOS }
          libsym : tsym;
          libsymderef : tderef;
{$endif powerpc or m68k}
          { name of the result variable to insert in the localsymtable }
          resultname : pshortstring;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          { import info }
          import_dll,
          import_name : pshortstring;
          { info for inlining the subroutine, if this pointer is nil,
            the procedure can't be inlined }
          inlininginfo : pinlininginfo;
{$ifdef oldregvars}
          regvarinfo: pregvarinfo;
{$endif oldregvars}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          import_nr    : word;
          extnumber    : word;
          visibility   : tvisibility;
          { set to a value different from tsk_none in case this procdef is for
            a routine that has to be internally generated by the compiler }
          synthetickind : tsynthetickind;
          constructor create(level:byte;doregister:boolean);virtual;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          { First/last assembler symbol/instruction in aasmoutput list.
            Note: initialised after compiling the code for the procdef, but
              not saved to/restored from ppu. Used when inserting debug info }
          procstarttai,
          procendtai   : tai;
          import_nr    : word;
          extnumber    : word;
{$ifdef i386}
          fpu_used     : byte;
{$endif i386}
          visibility   : tvisibility;
          { true, if the procedure is only declared
            (forward procedure) }
          forwarddef,
          { true if the procedure is declared in the interface }
          interfacedef : boolean;
          { true if the procedure has a forward declaration }
          hasforward  : boolean;
          { interrupt vector }
          interruptvector : longint;
          constructor create(level:byte);
>>>>>>> graemeg/cpstrnew
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
          function  getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp): tstoreddef; override;
          function  getcopy: tstoreddef; override;
          function  GetTypeName : string;override;
          function  mangledname : TSymStr; virtual;
          procedure setmangledname(const s : TSymStr);
          function  fullprocname(showhidden:boolean):string;
<<<<<<< HEAD
          function  customprocname(pno: tprocnameoptions):ansistring;
          function  defaultmangledname: TSymStr;
          function  cplusplusmangledname : TSymStr;
          function  objcmangledname : TSymStr;
          function  is_methodpointer:boolean;override;
          function  is_addressonly:boolean;override;
          procedure make_external;
          procedure init_genericdecl;

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
=======
          function  cplusplusmangledname : string;
          function  objcmangledname : string;
          function  is_methodpointer:boolean;override;
          function  is_addressonly:boolean;override;
          procedure make_external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          constructor createshort(l: byte; doregister: boolean);virtual;
          constructor loadshort(ppufile:tcompilerppufile);
          constructor createlong(l: asizeint; doregister: boolean);virtual;
=======
          constructor createshort(l : byte);
          constructor loadshort(ppufile:tcompilerppufile);
          constructor createlong(l : asizeint);
>>>>>>> graemeg/cpstrnew
=======
          constructor createshort(l : byte);
          constructor loadshort(ppufile:tcompilerppufile);
          constructor createlong(l : asizeint);
>>>>>>> graemeg/cpstrnew
=======
          constructor createshort(l : byte);
          constructor loadshort(ppufile:tcompilerppufile);
          constructor createlong(l : asizeint);
>>>>>>> graemeg/cpstrnew
=======
          constructor createshort(l : byte);
          constructor loadshort(ppufile:tcompilerppufile);
          constructor createlong(l : asizeint);
>>>>>>> origin/cpstrnew
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

       { tenumdef }

       { tenumdef }

       { tenumdef }

       { tenumdef }

       tenumdef = class(tstoreddef)
          minval,
          maxval    : asizeint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          basedef   : tenumdef;
          basedefderef : tderef;
          symtable  : TSymtable;
          has_jumps : boolean;
          constructor create;virtual;
          constructor create_subrange(_basedef:tenumdef;_min,_max:asizeint);virtual;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          has_jumps : boolean;
          basedef   : tenumdef;
          basedefderef : tderef;
          symtable  : TSymtable;
          constructor create;
          constructor create_subrange(_basedef:tenumdef;_min,_max:asizeint);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
          procedure calcsavesize;
          function  packedbitsize: asizeint; override;
          procedure setmax(_max:asizeint);
          procedure setmin(_min:asizeint);
          function  min:asizeint;
          function  max:asizeint;
          function  getfirstsym:tsym;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          function  int2enumsym(l: asizeint): tsym;
          { returns basedef if assigned, otherwise self }
          function getbasedef: tenumdef;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       end;
       tenumdefclass = class of tenumdef;

       tsetdef = class(tstoreddef)
          elementdef : tdef;
          elementdefderef : tderef;
          setbase,
<<<<<<< HEAD
<<<<<<< HEAD
          setmax   : asizeint;
          constructor create(def: tdef; low, high: asizeint; doregister: boolean);virtual;
=======
          setmax   : aword;
          constructor create(def:tdef;low, high : asizeint);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
          setmax   : aint;
          constructor create(def:tdef;low, high : aint);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          constructor ppuload(ppufile:tcompilerppufile);
          function getcopy : tstoreddef;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function  GetTypeName:string;override;
          function  is_publishable : boolean;override;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

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
       pasbool8type,              { boolean type }
       pasbool16type,
       pasbool32type,
       pasbool64type,
       bool8type,
       bool16type,
       bool32type,
       bool64type,                { implement me }
       u8inttype,                 { 8-Bit unsigned integer }
       s8inttype,                 { 8-Bit signed integer }
       u16inttype,                { 16-Bit unsigned integer }
       s16inttype,                { 16-Bit signed integer }
       u32inttype,                { 32-Bit unsigned integer }
       s32inttype,                { 32-Bit signed integer }
       u64inttype,                { 64-bit unsigned integer }
       s64inttype,                { 64-bit signed integer }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       u128inttype,               { 128-bit unsigned integer }
       s128inttype,               { 128-bit signed integer }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
       { we use only one variant def for every variant class }
       cvarianttype,
       colevarianttype,
       { default integer type, normally s32inttype on 32 bit systems and s64bittype on 64 bit systems }
       sinttype,
       uinttype,
       { integer types corresponding to OS_SINT/OS_INT }
       ossinttype,
       osuinttype,
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
       { pointer to the ancestor of all COM interfaces }
       interface_iunknown : tobjectdef;
       { pointer to the ancestor of all dispinterfaces }
       interface_idispatch : tobjectdef;
       { pointer to the TGUID type
         of all interfaces         }
       rec_tguid : trecorddef;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       { jump buffer type, used by setjmp }
       rec_jmp_buf : trecorddef;

       { system.texceptaddr type, used by fpc_pushexceptaddr }
       rec_exceptaddr: trecorddef;

=======
       { pointer to jump buffer }
       rec_jmp_buf : trecorddef;

>>>>>>> graemeg/cpstrnew
=======
       { pointer to jump buffer }
       rec_jmp_buf : trecorddef;

>>>>>>> graemeg/cpstrnew
=======
       { pointer to jump buffer }
       rec_jmp_buf : trecorddef;

>>>>>>> graemeg/cpstrnew
=======
       { pointer to jump buffer }
       rec_jmp_buf : trecorddef;

>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    const
{$ifdef i386}
       pbestrealtype : ^tdef = @s80floattype;
{$endif}
{$ifdef x86_64}
       pbestrealtype : ^tdef = @s80floattype;
{$endif}
{$ifdef m68k}
       pbestrealtype : ^tdef = @s64floattype;
{$endif}
{$ifdef alpha}
       pbestrealtype : ^tdef = @s64floattype;
{$endif}
{$ifdef powerpc}
       pbestrealtype : ^tdef = @s64floattype;
{$endif}
{$ifdef POWERPC64}
       pbestrealtype : ^tdef = @s64floattype;
{$endif}
{$ifdef ia64}
       pbestrealtype : ^tdef = @s64floattype;
{$endif}
{$ifdef SPARC}
       pbestrealtype : ^tdef = @s64floattype;
{$endif SPARC}
{$ifdef vis}
       pbestrealtype : ^tdef = @s64floattype;
{$endif vis}
{$ifdef ARM}
       pbestrealtype : ^tdef = @s64floattype;
{$endif ARM}
{$ifdef MIPS}
       pbestrealtype : ^tdef = @s64floattype;
{$endif MIPS}
{$ifdef AVR}
       pbestrealtype : ^tdef = @s64floattype;
{$endif AVR}

    function make_mangledname(const typeprefix:string;st:TSymtable;const suffix:string):string;
>>>>>>> graemeg/cpstrnew

    { should be in the types unit, but the types unit uses the node stuff :( }
    function is_interfacecom(def: tdef): boolean;
    function is_interfacecom_or_dispinterface(def: tdef): boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function is_any_interface_kind(def: tdef): boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function is_interfacecorba(def: tdef): boolean;
    function is_interface(def: tdef): boolean;
    function is_dispinterface(def: tdef): boolean;
    function is_object(def: tdef): boolean;
    function is_class(def: tdef): boolean;
    function is_cppclass(def: tdef): boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function is_objectpascal_helper(def: tdef): boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function is_objcclass(def: tdef): boolean;
    function is_objcclassref(def: tdef): boolean;
    function is_objcprotocol(def: tdef): boolean;
    function is_objccategory(def: tdef): boolean;
    function is_objc_class_or_protocol(def: tdef): boolean;
    function is_objc_protocol_or_category(def: tdef): boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function is_classhelper(def: tdef): boolean;
    function is_class_or_interface(def: tdef): boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    function is_class_or_interface_or_objc(def: tdef): boolean;
    function is_class_or_interface_or_objc_or_java(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface_or_objc_or_java(def: tdef): boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
=======
    function is_class_or_interface(def: tdef): boolean;
    function is_class_or_interface_or_objc(def: tdef): boolean;
    function is_class_or_interface_or_object(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface(def: tdef): boolean;
    function is_implicit_pointer_object_type(def: tdef): boolean;
    function is_class_or_object(def: tdef): boolean;
    function is_record(def: tdef): boolean;

    procedure loadobjctypes;
    procedure maybeloadcocoatypes;

    function use_vectorfpu(def : tdef) : boolean;
>>>>>>> graemeg/cpstrnew
=======
    function is_class_or_interface(def: tdef): boolean;
    function is_class_or_interface_or_objc(def: tdef): boolean;
    function is_class_or_interface_or_object(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface(def: tdef): boolean;
    function is_implicit_pointer_object_type(def: tdef): boolean;
    function is_class_or_object(def: tdef): boolean;
    function is_record(def: tdef): boolean;

    procedure loadobjctypes;
    procedure maybeloadcocoatypes;

    function use_vectorfpu(def : tdef) : boolean;
>>>>>>> graemeg/cpstrnew
=======
    function is_class_or_interface(def: tdef): boolean;
    function is_class_or_interface_or_objc(def: tdef): boolean;
    function is_class_or_interface_or_object(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface(def: tdef): boolean;
    function is_implicit_pointer_object_type(def: tdef): boolean;
    function is_class_or_object(def: tdef): boolean;
    function is_record(def: tdef): boolean;

    procedure loadobjctypes;
    procedure maybeloadcocoatypes;

    function use_vectorfpu(def : tdef) : boolean;
>>>>>>> graemeg/cpstrnew
=======
    function is_class_or_interface(def: tdef): boolean;
    function is_class_or_interface_or_objc(def: tdef): boolean;
    function is_class_or_interface_or_object(def: tdef): boolean;
    function is_class_or_interface_or_dispinterface(def: tdef): boolean;
    function is_implicit_pointer_object_type(def: tdef): boolean;
    function is_class_or_object(def: tdef): boolean;
    function is_record(def: tdef): boolean;

    procedure loadobjctypes;
    procedure maybeloadcocoatypes;

    function use_vectorfpu(def : tdef) : boolean;
>>>>>>> origin/cpstrnew

implementation

    uses
      SysUtils,
      cutils,
      { global }
      verbose,
      { target }
      systems,paramgr,
      { symtable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      symsym,symtable,defutil,objcdef,
      { parser }
      pgenutil,
=======
      symsym,symtable,symutil,defutil,objcdef,
>>>>>>> graemeg/cpstrnew
=======
      symsym,symtable,symutil,defutil,objcdef,
>>>>>>> graemeg/cpstrnew
=======
      symsym,symtable,symutil,defutil,objcdef,
>>>>>>> graemeg/cpstrnew
=======
      symsym,symtable,symutil,defutil,objcdef,
>>>>>>> origin/cpstrnew
      { module }
      fmodule,
      { other }
      gendef,
      fpccrc
      ;

{****************************************************************************
                                  Helpers
****************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

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
                symtablestack.push(symtable);
                current_module.ansistrdef:=cstringdef.createansi(current_settings.sourcecodepage,true);
                symtablestack.pop(symtable);
              end;
            result:=tstringdef(current_module.ansistrdef);
          end
        else
          result:=tstringdef(cansistringtype);
      end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

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
          name,sizeof(pint),sizeof(pint),
          init_settings.alignment.maxCrecordalign);
{$ifdef cpu16bitaddr}
        index_field:=result.add_field_by_def('',u16inttype);
{$else cpu16bitaddr}
        index_field:=result.add_field_by_def('',u32inttype);
{$endif cpu16bitaddr}
        non_mt_data_field:=result.add_field_by_def('',def);
        { no need to add alignment padding, we won't create arrays of these }
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
        while (st.symtabletype=localsymtable) do
         begin
           if st.defowner.typ<>procdef then
            internalerror(200204173);
           { Add the full mangledname of procedure to prevent
             conflicts with 2 overloads having both a nested procedure
             with the same name, see tb0314 (PFV) }
           s:=tprocdef(st.defowner).procsym.name;
<<<<<<< HEAD
           s:=s+tprocdef(st.defowner).mangledprocparanames(Length(s));
=======
           oldlen:=length(s);
           for i:=0 to tprocdef(st.defowner).paras.count-1 do
            begin
              hp:=tparavarsym(tprocdef(st.defowner).paras[i]);
              if not(vo_is_hidden_para in hp.varoptions) then
                s:=s+'$'+hp.vardef.mangledparaname;
            end;
           if not is_void(tprocdef(st.defowner).returndef) then
             s:=s+'$$'+tprocdef(st.defowner).returndef.mangledparaname;
           newlen:=length(s);
           { Replace with CRC if the parameter line is very long }
           if (newlen-oldlen>12) and
<<<<<<< HEAD
<<<<<<< HEAD
              ((newlen+length(prefix)>100) or (newlen-oldlen>32)) then
=======
              ((newlen+length(prefix)>128) or (newlen-oldlen>32)) then
>>>>>>> graemeg/fixes_2_2
=======
              ((newlen+length(prefix)>128) or (newlen-oldlen>32)) then
>>>>>>> origin/fixes_2_2
             begin
               crc:=0;
               for i:=0 to tprocdef(st.defowner).paras.count-1 do
                 begin
                   hp:=tparavarsym(tprocdef(st.defowner).paras[i]);
                   if not(vo_is_hidden_para in hp.varoptions) then
                     begin
                       hs:=hp.vardef.mangledparaname;
                       crc:=UpdateCrc32(crc,hs[1],length(hs));
                     end;
                 end;
               hs:=hp.vardef.mangledparaname;
               crc:=UpdateCrc32(crc,hs[1],length(hs));
               s:=Copy(s,1,oldlen)+'$crc'+hexstr(crc,8);
             end;
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
        sym,srsym : tsym;
        srsymtable : tsymtable;
        entry : tgenericdummyentry;
      begin
        { search the symtable from first to last; the helper to use will be the
          last one in the list }
        for i:=0 to st.symlist.count-1 do
          begin
            if not (st.symlist[i] is ttypesym) then
              continue;
            def:=ttypesym(st.SymList[i]).typedef;
            sym:=tsym(st.symlist[i]);
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
                if addgenerics and
                    (sp_generic_dummy in sym.symoptions)
                    then
                  begin
                    { did we already search for a generic with that name? }
                    list:=tfpobjectlist(current_module.genericdummysyms.find(sym.name));
                    if not assigned(list) then
                      begin
                        list:=tfpobjectlist.create(true);
                        current_module.genericdummysyms.add(sym.name,list);
                      end;
                    { is the dummy sym still "dummy"? }
                    if (sym.typ=typesym) and
                        (
                          { dummy sym defined in mode Delphi }
                          (ttypesym(sym).typedef.typ=undefineddef) or
                          { dummy sym defined in non-Delphi mode }
                          (tstoreddef(ttypesym(sym).typedef).is_generic)
                        ) then
                      begin
                        { do we have a non-generic type of the same name
                          available? }
                        if not searchsym_with_flags(sym.name,srsym,srsymtable,[ssf_no_addsymref]) then
                          srsym:=nil;
                      end
                    else
                      { dummy symbol is already not so dummy anymore }
                      srsym:=nil;
                    if assigned(srsym) then
                      begin
                        entry:=tgenericdummyentry.create;
                        entry.resolvedsym:=srsym;
                        entry.dummysym:=sym;
                        list.add(entry);
                      end;
                  end;
                { add nested helpers as well }
                if (def.typ in [recorddef,objectdef]) and
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
        ppufile.getsmallset(flags);
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
        ppufile.putsmallset(flags);
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
         ppufile.getsmallset(defoptions);
         ppufile.getsmallset(defstates);
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
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

    function tstoreddef.fullownerhierarchyname: TSymStr;
      var
        lastowner: tsymtable;
        tmp: tdef;
      begin
{$ifdef symansistr}
        if _fullownerhierarchyname<>'' then
          exit(_fullownerhierarchyname);
{$else symansistr}
        if assigned(_fullownerhierarchyname) then
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
              result:=tprocdef(tmp).customprocname([pno_paranames,pno_proctypeoption])+'.'+result;
        until tmp=nil;
        { add the unit name }
        if assigned(lastowner) and
           assigned(lastowner.realname) then
          result:=lastowner.realname^+'.'+result;
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
=======
      begin
        tmp:=self;
        result:='';
        repeat
          if tmp.owner.symtabletype in [ObjectSymtable,recordsymtable] then
            tmp:=tdef(tmp.owner.defowner)
          else
            break;
          result:=tabstractrecorddef(tmp).objrealname^+'.'+result;
        until tmp=nil;
      end;


<<<<<<< HEAD
    function tstoreddef.in_currentunit: boolean;
      var
        st: tsymtable;
      begin
>>>>>>> graemeg/cpstrnew
=======
    function tstoreddef.OwnerHierarchyName: string;
      var
        tmp: tdef;
      begin
        tmp:=self;
        result:='';
        repeat
          if tmp.owner.symtabletype in [ObjectSymtable,recordsymtable] then
            tmp:=tdef(tmp.owner.defowner)
          else
            break;
          result:=tabstractrecorddef(tmp).objrealname^+'.'+result;
        until tmp=nil;
      end;


<<<<<<< HEAD
    function tstoreddef.in_currentunit: boolean;
      var
        st: tsymtable;
      begin
>>>>>>> graemeg/cpstrnew
=======
    function tstoreddef.OwnerHierarchyName: string;
      var
        tmp: tdef;
      begin
        tmp:=self;
        result:='';
        repeat
          if tmp.owner.symtabletype in [ObjectSymtable,recordsymtable] then
            tmp:=tdef(tmp.owner.defowner)
          else
            break;
          result:=tabstractrecorddef(tmp).objrealname^+'.'+result;
        until tmp=nil;
      end;


<<<<<<< HEAD
    function tstoreddef.in_currentunit: boolean;
      var
        st: tsymtable;
      begin
>>>>>>> graemeg/cpstrnew
=======
    function tstoreddef.OwnerHierarchyName: string;
      var
        tmp: tdef;
      begin
        tmp:=self;
        result:='';
        repeat
          if tmp.owner.symtabletype in [ObjectSymtable,recordsymtable] then
            tmp:=tdef(tmp.owner.defowner)
          else
            break;
          result:=tabstractrecorddef(tmp).objrealname^+'.'+result;
        until tmp=nil;
      end;


    function tstoreddef.in_currentunit: boolean;
      var
        st: tsymtable;
      begin
>>>>>>> origin/cpstrnew
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
        ppufile.putsmallset(defoptions);
        oldintfcrc:=ppufile.do_crc;
        ppufile.do_crc:=false;
        ppufile.putsmallset(defstates);
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
                dispose(symderef);
              end;
            genericparaderefs.free;
            genericparaderefs:=nil;
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


   function tstoreddef.is_intregable : boolean;
     var
       recsize,temp: longint;
     begin
        is_intregable:=false;
        case typ of
          orddef,
          pointerdef,
          enumdef,
          classrefdef:
            is_intregable:=true;
          procvardef :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            is_intregable:=tprocvardef(self).is_addressonly or (po_methodpointer in tprocvardef(self).procoptions);
=======
            is_intregable:=tprocvardef(self).is_addressonly;
>>>>>>> graemeg/cpstrnew
=======
            is_intregable:=tprocvardef(self).is_addressonly;
>>>>>>> graemeg/cpstrnew
=======
            is_intregable:=tprocvardef(self).is_addressonly;
>>>>>>> graemeg/cpstrnew
=======
            is_intregable:=tprocvardef(self).is_addressonly;
>>>>>>> origin/cpstrnew
          objectdef:
            is_intregable:=(is_implicit_pointer_object_type(self)) and not needs_inittable;
          setdef:
            is_intregable:=is_smallset(self);
          recorddef:
            begin
{$ifdef llvm}
              is_intregable:=false;
{$else llvm}
              recsize:=size;
              is_intregable:=
                ispowerof2(recsize,temp) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                (((recsize <= sizeof(aint)*2) and
                 { records cannot go into registers on 16 bit targets for now }
                  (sizeof(aint)>2) and
                  not trecorddef(self).contains_float_field) or
                  (recsize <= sizeof(aint))) and
                not needs_inittable;
{$endif llvm}
=======
                (recsize <= sizeof(asizeint));
>>>>>>> graemeg/cpstrnew
=======
                (recsize <= sizeof(asizeint));
>>>>>>> graemeg/cpstrnew
=======
                (recsize <= sizeof(asizeint));
>>>>>>> graemeg/cpstrnew
=======
                (recsize <= sizeof(asizeint));
>>>>>>> origin/cpstrnew
            end;
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
     begin
       if registered then
         exit;
       { Register in current_module }
       if assigned(current_module) then
         begin
           exclude(defoptions,df_not_registered_no_free);
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor tstringdef.createlong(l: asizeint; doregister: boolean);
=======
    constructor tstringdef.createlong(l : asizeint);
>>>>>>> graemeg/cpstrnew
=======
    constructor tstringdef.createlong(l : asizeint);
>>>>>>> graemeg/cpstrnew
=======
    constructor tstringdef.createlong(l : asizeint);
>>>>>>> graemeg/cpstrnew
=======
    constructor tstringdef.createlong(l : asizeint);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         encoding:=0;
         len:=ppufile.getasizeint;
         ppuload_platform(ppufile);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         len:=ppufile.getasizeint;
         savesize:=sizeof(pint);
>>>>>>> graemeg/cpstrnew
      end;


    constructor tstringdef.createansi(aencoding: tstringencoding; doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_ansistring;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         encoding:=aencoding;
=======
         encoding:=65535;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=65535;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=65535;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=65535;
>>>>>>> origin/cpstrnew
         len:=-1;
      end;


    constructor tstringdef.loadansi(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_ansistring;
         len:=ppufile.getaint;
         encoding:=ppufile.getword;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         ppuload_platform(ppufile);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         savesize:=sizeof(pint);
>>>>>>> graemeg/cpstrnew
      end;


    constructor tstringdef.createwide(doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_widestring;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if target_info.endian=endian_little then
           encoding:=CP_UTF16LE
         else
           encoding:=CP_UTF16BE;
=======
         encoding:=CP_UTF16;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=CP_UTF16;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=CP_UTF16;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=CP_UTF16;
>>>>>>> origin/cpstrnew
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
         len:=ppufile.getaint;
         ppuload_platform(ppufile);
      end;


    constructor tstringdef.createunicode(doregister: boolean);
      begin
         inherited create(stringdef,doregister);
         stringtype:=st_unicodestring;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if target_info.endian=endian_little then
           encoding:=CP_UTF16LE
         else
           encoding:=CP_UTF16BE;
=======
         encoding:=CP_UTF16;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=CP_UTF16;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=CP_UTF16;
>>>>>>> graemeg/cpstrnew
=======
         encoding:=CP_UTF16;
>>>>>>> origin/cpstrnew
         len:=-1;
      end;


    constructor tstringdef.loadunicode(ppufile:tcompilerppufile);
      begin
         inherited ppuload(stringdef,ppufile);
         stringtype:=st_unicodestring;
         len:=ppufile.getaint;
         encoding:=ppufile.getword;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         ppuload_platform(ppufile);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         savesize:=sizeof(pint);
>>>>>>> graemeg/cpstrnew
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
           ppufile.putaint(len);
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
<<<<<<< HEAD
<<<<<<< HEAD
              { char to string accesses byte 0 and 1 with one word access }
            if (tf_requires_proper_alignment in target_info.flags) or
              { macpas needs an alignment of 2 (MetroWerks compatible) }
               (m_mac in current_settings.modeswitches) then
=======
            if (tf_requires_proper_alignment in target_info.flags) then
              { char to string accesses byte 0 and 1 with one word access }
>>>>>>> graemeg/fixes_2_2
=======
            if (tf_requires_proper_alignment in target_info.flags) then
              { char to string accesses byte 0 and 1 with one word access }
>>>>>>> origin/fixes_2_2
              alignment:=size_2_align(2)
            else
              alignment:=size_2_align(1);
          else
            internalerror(200412301);
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
          else
            internalerror(2014032301);
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
         calcsavesize;
         has_jumps:=false;
         basedef:=nil;
         symtable:=tenumsymtable.create(self);
      end;


    constructor tenumdef.create_subrange(_basedef:tenumdef;_min,_max:asizeint);
      begin
         inherited create(enumdef,true);
         minval:=_min;
         maxval:=_max;
         basedef:=_basedef;
         calcsavesize;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             ppuload_platform(ppufile);
           end
         else
           begin
             ppuload_platform(ppufile);
=======
           end
         else
           begin
>>>>>>> graemeg/cpstrnew
=======
           end
         else
           begin
>>>>>>> graemeg/cpstrnew
=======
           end
         else
           begin
>>>>>>> graemeg/cpstrnew
=======
           end
         else
           begin
>>>>>>> origin/cpstrnew
             // create with nil defowner first to prevent values changes on insert
             symtable:=tenumsymtable.create(nil);
             tenumsymtable(symtable).ppuload(ppufile);
             symtable.defowner:=self;
           end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
      end;

    destructor tenumdef.destroy;
      begin
        symtable.free;
        symtable:=nil;
        inherited destroy;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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


    procedure tenumdef.calcsavesize;
      begin
{$IFNDEF cpu64bitaddr} {$push}{$warnings off} {$ENDIF} //comparison always false warning
        if (current_settings.packenum=8) or (min<low(longint)) or (int64(max)>high(cardinal)) then
         savesize:=8
{$IFNDEF cpu64bitaddr} {$pop} {$ENDIF}
        else
{$IFDEF cpu16bitaddr} {$push}{$warnings off} {$ENDIF} //comparison always false warning
         if (current_settings.packenum=4) or (min<low(smallint)) or (max>high(word)) then
          savesize:=4
{$IFDEF cpu16bitaddr} {$pop} {$ENDIF}
        else
         if (current_settings.packenum=2) or (min<low(shortint)) or (max>high(byte)) then
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
              { don't count 0 twice }
              sizeval:=(cutils.max(-minval,maxval)*2)-1;
            { 256 must become 512 etc. }
            nextpowerof2(sizeval+1,power);
            result := power;
          end;
      end;


    procedure tenumdef.setmax(_max:asizeint);
      begin
        maxval:=_max;
        calcsavesize;
      end;


    procedure tenumdef.setmin(_min:asizeint);
      begin
        minval:=_min;
        calcsavesize;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      begin
        for i := 0 to symtable.SymList.Count - 1 do
          begin
            result:=tsym(symtable.SymList[i]);
            if tenumsym(result).value=minval then
              exit;
          end;
        result:=nil;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
        for i := 0 to symtable.SymList.Count - 1 do
          begin
            result:=tsym(symtable.SymList[i]);
            if tenumsym(result).value=minval then
              exit;
          end;
        result:=nil;
      end;


    procedure tenumdef.buildderef;
      begin
        inherited buildderef;
        if df_copied_def in defoptions then
          basedefderef.build(basedef)
        else
          tenumsymtable(symtable).buildderef;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;


    function tenumdef.int2enumsym(l: asizeint): tsym;
      var
        i: longint;
        sym: tsym;
        bdef: tenumdef;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
        inherited buildderef;
        if df_copied_def in defoptions then
          basedefderef.build(basedef)
        else
          tenumsymtable(symtable).buildderef;
>>>>>>> graemeg/cpstrnew
      end;


    function tenumdef.getbasedef: tenumdef;
      begin
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        inherited deref;
        if df_copied_def in defoptions then
          begin
            basedef:=tenumdef(basedefderef.resolve);
            symtable:=basedef.symtable.getcopy;
          end
        else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          tenumsymtable(symtable).deref(false);
=======
          tenumsymtable(symtable).deref;
>>>>>>> graemeg/cpstrnew
=======
          tenumsymtable(symtable).deref;
>>>>>>> graemeg/cpstrnew
=======
          tenumsymtable(symtable).deref;
>>>>>>> graemeg/cpstrnew
=======
          tenumsymtable(symtable).deref;
>>>>>>> origin/cpstrnew
      end;


    procedure tenumdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putaint(min);
         ppufile.putaint(max);
         ppufile.putaint(savesize);
         if df_copied_def in defoptions then
           ppufile.putderef(basedefderef);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         writeentry(ppufile,ibenumdef);
=======
         ppufile.writeentry(ibenumdef);
>>>>>>> graemeg/cpstrnew
=======
         ppufile.writeentry(ibenumdef);
>>>>>>> graemeg/cpstrnew
=======
         ppufile.writeentry(ibenumdef);
>>>>>>> graemeg/cpstrnew
=======
         ppufile.writeentry(ibenumdef);
>>>>>>> origin/cpstrnew
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
          1,2,4,8,
          1,2,4,8,
          1,2,8
        );
      begin
        savesize:=sizetbl[ordtype];
      end;


    function torddef.packedbitsize: asizeint;
      var
        sizeval: tconstexprint;
        power: longint;
      begin
        result := 0;
        if ordtype = uvoid then
          exit;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef cpu64bitalu}
        if (ordtype in [s64bit,u64bit]) then
{$else not cpu64bitalu}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        if (ordtype = u64bit) or
           ((ordtype = s64bit) and
            ((low <= (system.low(int64) div 2)) or
             (high > (system.high(int64) div 2)))) then
<<<<<<< HEAD
<<<<<<< HEAD
{$endif cpu64bitalu}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          result := 64
        else if (low >= 0) and
           (high <= 1) then
          result := 1
        else
          begin
            if (low>=0) then
              sizeval:=high
            else
              { don't count 0 twice }
              sizeval:=(cutils.max(-low,high)*2)-1;
            { 256 must become 512 etc. }
            nextpowerof2(sizeval+1,power);
            result := power;
         end;
      end;


    function torddef.getvardef : longint;
      const
        basetype2vardef : array[tordtype] of longint = (
          varUndefined,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          varbyte,varword,varlongword,varqword,varUndefined,
          varshortint,varsmallint,varinteger,varint64,varUndefined,
          varboolean,varboolean,varboolean,varboolean,
          varboolean,varboolean,varUndefined,varUndefined,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          varbyte,varword,varlongword,varqword,
          varshortint,varsmallint,varinteger,varint64,
<<<<<<< HEAD
<<<<<<< HEAD
          varboolean,varboolean,varboolean,varUndefined,varUndefined,
>>>>>>> graemeg/cpstrnew
=======
          varboolean,varboolean,varUndefined,varUndefined,
>>>>>>> graemeg/fixes_2_2
=======
          varboolean,varboolean,varUndefined,varUndefined,
>>>>>>> origin/fixes_2_2
          varUndefined,varUndefined,varCurrency);
      begin
        result:=basetype2vardef[ordtype];
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
          'Boolean','Boolean16','Boolean32','Boolean64',
          'ByteBool','WordBool','LongBool','QWordBool',
          'Char','WideChar','Currency');

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            sc80real,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             if target_info.system in [system_i386_darwin,
                  system_i386_iphonesim,system_x86_64_darwin,
                  system_x86_64_iphonesim,
                  system_x86_64_linux,system_x86_64_freebsd,
                  system_x86_64_openbsd,system_x86_64_netbsd,
                  system_x86_64_solaris,system_x86_64_embedded,
                  system_x86_64_dragonfly] then
=======
             if target_info.system in [system_i386_darwin,system_i386_iphonesim,system_x86_64_darwin,
                  system_x86_64_linux,system_x86_64_freebsd,
                  system_x86_64_solaris,system_x86_64_embedded] then
>>>>>>> graemeg/cpstrnew
=======
             if target_info.system in [system_i386_darwin,system_i386_iphonesim,system_x86_64_darwin,
                  system_x86_64_linux,system_x86_64_freebsd,
                  system_x86_64_solaris,system_x86_64_embedded] then
>>>>>>> graemeg/cpstrnew
=======
             if target_info.system in [system_i386_darwin,system_i386_iphonesim,system_x86_64_darwin,
                  system_x86_64_linux,system_x86_64_freebsd,
                  system_x86_64_solaris,system_x86_64_embedded] then
>>>>>>> graemeg/cpstrnew
=======
             if target_info.system in [system_i386_darwin,system_i386_iphonesim,system_x86_64_darwin,
                  system_x86_64_linux,system_x86_64_freebsd,
                  system_x86_64_solaris,system_x86_64_embedded] then
>>>>>>> origin/cpstrnew
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
          else
            internalerror(2004121201);
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
<<<<<<< HEAD
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
         else
           internalerror(2013113001);
         end;
=======
{$ifdef cpu64bitaddr}
        case filetyp of
          ft_text :
            if target_info.system in [system_x86_64_win64,system_ia64_win64] then
              savesize:=632{+8}
            else
              savesize:=628{+8};
          ft_typed,
          ft_untyped :
            if target_info.system in [system_x86_64_win64,system_ia64_win64] then
              savesize:=372
            else
              savesize:=368;
        end;
{$endif cpu64bitaddr}
{$ifdef cpu32bitaddr}
        case filetyp of
          ft_text :
            savesize:=592{+4};
          ft_typed,
          ft_untyped :
            savesize:=332;
        end;
{$endif cpu32bitaddr}
{$ifdef cpu8bitaddr}
        case filetyp of
          ft_text :
            savesize:=127;
          ft_typed,
          ft_untyped :
            savesize:=127;
        end;
{$endif cpu8bitaddr}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
           else
             internalerror(2013113002);
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
           else
             internalerror(2013113003);
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


    function tvariantdef.getvardef : longint;
      begin
        Result:=varVariant;
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
           else
             internalerror(2013113004);
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
<<<<<<< HEAD
=======
        is_far:=false;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
        has_pointer_math:=cs_pointermath in current_settings.localswitches;
      end;


    class function tpointerdef.getreusable(def: tdef): tpointerdef;
      var
        res: PHashSetItem;
        oldsymtablestack: tsymtablestack;
      begin
<<<<<<< HEAD
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
=======
        has_pointer_math:=cs_pointermath in current_settings.localswitches;
>>>>>>> graemeg/cpstrnew
=======
        has_pointer_math:=cs_pointermath in current_settings.localswitches;
>>>>>>> graemeg/cpstrnew
=======
        has_pointer_math:=cs_pointermath in current_settings.localswitches;
>>>>>>> origin/cpstrnew
      end;


    function tpointerdef.size: asizeint;
      begin
        result:=sizeof(pint);
=======
        inherited create(pointerdef,def);
        is_far:=true;
        has_pointer_math:=cs_pointermath in current_settings.localswitches;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;


    constructor tpointerdef.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(pointerdef,ppufile);
<<<<<<< HEAD
         has_pointer_math:=(ppufile.getbyte<>0);
         ppuload_platform(ppufile);
=======
         is_far:=(ppufile.getbyte<>0);
         has_pointer_math:=(ppufile.getbyte<>0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;


    function tpointerdef.getcopy : tstoreddef;
      begin
        { don't use direct pointeddef if it is a forwarddef because in other case
          one of them will be destroyed on forward type resolve and the second will
          point to garbage }
        if pointeddef.typ=forwarddef then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          result:=cpointerdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=cpointerdef.create(pointeddef);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          result:=tpointerdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=tpointerdef.create(pointeddef);
        tpointerdef(result).is_far:=is_far;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        tpointerdef(result).has_pointer_math:=has_pointer_math;
        tpointerdef(result).savesize:=savesize;
      end;


    procedure tpointerdef.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
<<<<<<< HEAD
         ppufile.putbyte(byte(has_pointer_math));
         writeentry(ppufile,ibpointerdef);
=======
         ppufile.putbyte(byte(is_far));
         ppufile.putbyte(byte(has_pointer_math));
         ppufile.writeentry(ibpointerdef);
>>>>>>> graemeg/cpstrnew
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


    function tpointerdef.pointer_subtraction_result_type:tdef;
      begin
        result:=ptrsinttype;
      end;


    function tpointerdef.compatible_with_pointerdef_size(ptr: tpointerdef): boolean;
      begin
        result:=true;
      end;


{****************************************************************************
                              TCLASSREFDEF
****************************************************************************}

    constructor tclassrefdef.create(def:tdef);
      begin
         inherited create(classrefdef,def);
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


    function tclassrefdef.getcopy:tstoreddef;
      begin
        if pointeddef.typ=forwarddef then
          result:=tclassrefdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=tclassrefdef.create(pointeddef);
        tclassrefdef(result).savesize:=savesize;
      end;


    function tclassrefdef.getcopy:tstoreddef;
      begin
        if pointeddef.typ=forwarddef then
          result:=tclassrefdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=tclassrefdef.create(pointeddef);
        tclassrefdef(result).savesize:=savesize;
      end;


    function tclassrefdef.getcopy:tstoreddef;
      begin
        if pointeddef.typ=forwarddef then
          result:=tclassrefdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=tclassrefdef.create(pointeddef);
        tclassrefdef(result).savesize:=savesize;
      end;


    function tclassrefdef.getcopy:tstoreddef;
      begin
        if pointeddef.typ=forwarddef then
          result:=tclassrefdef.create(tforwarddef(pointeddef).getcopy)
        else
          result:=tclassrefdef.create(pointeddef);
        tclassrefdef(result).savesize:=savesize;
      end;


    function tclassrefdef.GetTypeName : string;
      begin
         GetTypeName:='Class Of '+pointeddef.typename;
      end;


    function tclassrefdef.is_publishable : boolean;
      begin
         result:=true;
      end;
<<<<<<< HEAD
=======

>>>>>>> origin/fixes_2.4


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function tclassrefdef.rtti_mangledname(rt: trttitype): TSymStr;
=======
    function tclassrefdef.rtti_mangledname(rt: trttitype): string;
>>>>>>> graemeg/cpstrnew
=======
    function tclassrefdef.rtti_mangledname(rt: trttitype): string;
>>>>>>> graemeg/cpstrnew
=======
    function tclassrefdef.rtti_mangledname(rt: trttitype): string;
>>>>>>> graemeg/cpstrnew
=======
    function tclassrefdef.rtti_mangledname(rt: trttitype): string;
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor tsetdef.create(def: tdef; low, high: asizeint; doregister: boolean);
=======
    constructor tsetdef.create(def:tdef;low, high : asizeint);
>>>>>>> graemeg/cpstrnew
=======
    constructor tsetdef.create(def:tdef;low, high : asizeint);
>>>>>>> graemeg/cpstrnew
=======
    constructor tsetdef.create(def:tdef;low, high : asizeint);
>>>>>>> graemeg/cpstrnew
=======
    constructor tsetdef.create(def:tdef;low, high : asizeint);
>>>>>>> origin/cpstrnew
      var
        setallocbits: aint;
        packedsavesize: aint;
        actual_setalloc: ShortInt;
=======
=======
>>>>>>> origin/fixes_2_2
    constructor tsetdef.create(def:tdef;low, high : aint);
      var
        setallocbits: aint;
        packedsavesize: aint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
         inherited create(setdef,doregister);
         elementdef:=def;
         setmax:=high;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
         if (current_settings.setalloc=0) then
           begin
             setbase:=0;
             if (high<32) then
=======
         if (current_settings.setalloc=0) then
           begin
             setbase:=0;
             if (high<32) then
>>>>>>> origin/fixes_2_2
               begin
                 settype:=smallset;
                 savesize:=Sizeof(longint)
               end
             else if (high<256) then
               begin
                 settype:=normset;
                 savesize:=32
               end
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             else
               savesize:=(high+7) div 8
           end
         else
           begin
<<<<<<< HEAD
<<<<<<< HEAD
             setallocbits:=actual_setalloc*8;
             setbase:=low and not(setallocbits-1);
             packedsavesize:=actual_setalloc*((((high+setallocbits)-setbase)) DIV setallocbits);
             savesize:=packedsavesize;
{$if not defined(cpu8bitalu) and not defined(cpu16bitalu)}
             if savesize=3 then
               savesize:=4;
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
             setallocbits:=current_settings.setalloc*8;
             setbase:=low and not(setallocbits-1);
             packedsavesize:=current_settings.setalloc*((((high+setallocbits)-setbase)) DIV setallocbits);
             savesize:=packedsavesize;
             if (packedsavesize<=4) then
               begin
                 settype:=smallset;
                 if savesize=3 then
                   savesize:=4;
               end
             else if (packedsavesize<=32) then
               settype:=normset;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
        result:=csetdef.create(elementdef,setbase,setmax,true);
=======
        result:=tsetdef.create(elementdef,setbase,setmax);
>>>>>>> graemeg/fixes_2_2
=======
        result:=tsetdef.create(elementdef,setbase,setmax);
>>>>>>> origin/fixes_2_2
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

    constructor tarraydef.create(l,h:asizeint;def:tdef);
      begin
         inherited create(arraydef,true);
         lowrange:=l;
         highrange:=h;
         rangedef:=def;
         _elementdef:=nil;
         arrayoptions:=[];
         symtable:=tarraysymtable.create(self);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      end;


    class function tarraydef.getreusable(def: tdef; elems: asizeint): tarraydef;
      var
        res: PHashSetItem;
        oldsymtablestack: tsymtablestack;
        arrdesc: packed record
          def: tdef;
          elecount: asizeint;
        end;
      begin
<<<<<<< HEAD
        if not assigned(current_module) then
          internalerror(2011081301);
        arrdesc.def:=def;
        arrdesc.elecount:=elems;
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
            result:=carraydef.create(0,elems-1,ptrsinttype);
            result.elementdef:=def;
            setup_reusable_def(def,result,res,oldsymtablestack);
            { res^.Data may still be nil -> don't overwrite result }
            exit;
          end;
        result:=tarraydef(res^.Data);
=======
         { use -1 so that the elecount will not overflow }
         self.create(0,high(aint)-1,s32inttype);
         arrayoptions:=[ado_IsConvertedPointer];
         setelementdef(def);
>>>>>>> graemeg/fixes_2_2
      end;

    class function tarraydef.getreusable_no_free(def: tdef; elems: asizeint): tarraydef;
      begin
        result:=getreusable(def,elems);
        if not result.is_registered then
          include(result.defoptions,df_not_registered_no_free);
      end;


=======
      end;

>>>>>>> graemeg/cpstrnew
=======
      end;

>>>>>>> graemeg/cpstrnew
=======
      end;

>>>>>>> graemeg/cpstrnew
=======
      end;

>>>>>>> origin/cpstrnew
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
         self.create(0,(high(asizeint) div max(def.pointeddef.size,1))-1,ptrsinttype);
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
         ppufile.getsmallset(arrayoptions);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         ppuload_platform(ppufile);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        tarraysymtable(symtable).deref(false);
=======
        tarraysymtable(symtable).deref;
>>>>>>> graemeg/cpstrnew
=======
        tarraysymtable(symtable).deref;
>>>>>>> graemeg/cpstrnew
=======
        tarraysymtable(symtable).deref;
>>>>>>> graemeg/cpstrnew
=======
        tarraysymtable(symtable).deref;
>>>>>>> origin/cpstrnew
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
         ppufile.putsmallset(arrayoptions);
<<<<<<< HEAD
         writeentry(ppufile,ibarraydef);
=======
         ppufile.writeentry(ibarraydef);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
<<<<<<< HEAD
    function tarraydef.elecount : asizeuint;
=======
    function tarraydef.elecount : aword;
>>>>>>> graemeg/fixes_2_2
=======
    function tarraydef.elecount : aword;
>>>>>>> origin/fixes_2_2
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
            qlow:=qword(-lowrange);
            { prevent overflow, return 0 to indicate overflow }
<<<<<<< HEAD
<<<<<<< HEAD
            if qhigh+qlow>qword(high(asizeint)-1) then
=======
            if qhigh+qlow>qword(high(aint)-1) then
>>>>>>> graemeg/fixes_2_2
=======
            if qhigh+qlow>qword(high(aint)-1) then
>>>>>>> origin/fixes_2_2
              result:=0
            else
              result:=qhigh+qlow+1;
          end
        else
          result:=int64(highrange)-lowrange+1;
      end;


    function tarraydef.size : asizeint;
      var
<<<<<<< HEAD
<<<<<<< HEAD
        cachedelecount : asizeuint;
        cachedelesize : asizeint;
=======
=======
>>>>>>> origin/fixes_2_2
        cachedelecount : aword;
        cachedelesize : aint;
>>>>>>> graemeg/fixes_2_2
      begin
        if ado_IsDynamicArray in arrayoptions then
          begin
            size:=voidpointertype.size;
            exit;
          end;

        { Tarraydef.size may never be called for an open array! }
        if highrange<lowrange then
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
               (highrange<lowrange)
	      ) and
           (size=-1) then
          Message(sym_e_segment_too_large);
      end;


    function tarraydef.alignment : shortint;
      begin
         { alignment of dyn. arrays doesn't depend on the element size }
         if (ado_IsDynamicArray in arrayoptions) then
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

{***************************************************************************
                              tabstractrecorddef
***************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
        objrealname:=stringdup(ppufile.getstring);
        objname:=stringdup(upper(objrealname^));
        import_lib:=stringdup(ppufile.getstring);
        { only used for external C++ classes and Java classes/records }
        if (import_lib^='') then
          stringdispose(import_lib);
        ppufile.getsmallset(objectoptions);
=======
    constructor tabstractrecorddef.create(const n:string; dt:tdeftyp);
      begin
        inherited create(dt);
        objname:=stringdup(upper(n));
        objrealname:=stringdup(n);
        objectoptions:=[];
=======
    constructor tabstractrecorddef.create(const n:string; dt:tdeftyp);
      begin
        inherited create(dt);
        objname:=stringdup(upper(n));
        objrealname:=stringdup(n);
        objectoptions:=[];
=======
    constructor tabstractrecorddef.create(const n:string; dt:tdeftyp);
      begin
        inherited create(dt);
        objname:=stringdup(upper(n));
        objrealname:=stringdup(n);
        objectoptions:=[];
      end;

    constructor tabstractrecorddef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
<<<<<<< HEAD
        inherited ppuload(dt,ppufile);
        objrealname:=stringdup(ppufile.getstring);
        objname:=stringdup(upper(objrealname^));
        ppufile.getsmallset(objectoptions);
=======
         inherited create(recorddef);
         symtable:=p;
         { we can own the symtable only if nobody else owns a copy so far }
         if symtable.refcount=1 then
           symtable.defowner:=self;
         isunion:=false;
>>>>>>> graemeg/fixes_2_2
      end;

    procedure tabstractrecorddef.ppuwrite(ppufile: tcompilerppufile);
      begin
<<<<<<< HEAD
        inherited ppuwrite(ppufile);
        ppufile.putstring(objrealname^);
        ppufile.putsmallset(objectoptions);
=======
         inherited ppuload(recorddef,ppufile);
         if df_copied_def in defoptions then
           ppufile.getderef(cloneddefderef)
         else
           begin
             symtable:=trecordsymtable.create(0);
             trecordsymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).padalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).usefieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).datasize:=ppufile.getaint;
             trecordsymtable(symtable).ppuload(ppufile);
             { requires usefieldalignment to be set }
             symtable.defowner:=self;
           end;
         isunion:=false;
>>>>>>> graemeg/fixes_2_2
      end;

    destructor tabstractrecorddef.destroy;
      begin
        stringdispose(objname);
        stringdispose(objrealname);
        inherited destroy;
      end;

    procedure tabstractrecorddef.check_forwards;
      begin
<<<<<<< HEAD
        tstoredsymtable(symtable).check_forwards;
=======
        result:=trecorddef.create(symtable.getcopy);
        trecorddef(result).isunion:=isunion;
        include(trecorddef(result).defoptions,df_copied_def);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
        if t=gs_record then
          GetSymtable:=symtable
        else
          GetSymtable:=nil;
>>>>>>> origin/cpstrnew
=======
         inherited buildderef;
         if df_copied_def in defoptions then
           cloneddefderef.build(symtable.defowner)
         else
           tstoredsymtable(symtable).buildderef;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

    constructor tabstractrecorddef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
<<<<<<< HEAD
        inherited ppuload(dt,ppufile);
        objrealname:=stringdup(ppufile.getstring);
        objname:=stringdup(upper(objrealname^));
        ppufile.getsmallset(objectoptions);
=======
         inherited deref;
         { now dereference the definitions }
         if df_copied_def in defoptions then
           begin
             cloneddef:=trecorddef(cloneddefderef.resolve);
             symtable:=cloneddef.symtable.getcopy;
           end
         else
           tstoredsymtable(symtable).deref;
         { assign TGUID? load only from system unit }
         if not(assigned(rec_tguid)) and
            (upper(typename)='TGUID') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_tguid:=self;
>>>>>>> graemeg/fixes_2_2
      end;

<<<<<<< HEAD
    procedure tabstractrecorddef.ppuwrite(ppufile: tcompilerppufile);
      begin
<<<<<<< HEAD
        inherited ppuwrite(ppufile);
        ppufile.putstring(objrealname^);
        ppufile.putsmallset(objectoptions);
=======
         inherited ppuwrite(ppufile);
         if df_copied_def in defoptions then
           ppufile.putderef(cloneddefderef)
         else
           begin
             ppufile.putbyte(byte(trecordsymtable(symtable).fieldalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).recordalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).padalignment));
             ppufile.putbyte(byte(trecordsymtable(symtable).usefieldalignment));
             ppufile.putaint(trecordsymtable(symtable).datasize);
           end;
<<<<<<< HEAD
=======

         ppufile.writeentry(ibrecorddef);

         if not(df_copied_def in defoptions) then
           trecordsymtable(symtable).ppuwrite(ppufile);
      end;
>>>>>>> origin/fixes_2_2

         ppufile.writeentry(ibrecorddef);

         if not(df_copied_def in defoptions) then
           trecordsymtable(symtable).ppuwrite(ppufile);
>>>>>>> graemeg/fixes_2_2
      end;

    destructor tabstractrecorddef.destroy;
      begin
        stringdispose(objname);
        stringdispose(objrealname);
        inherited destroy;
=======
    function tabstractrecorddef.RttiName: string;
      var
        tmp: tabstractrecorddef;
      begin
        Result:=OwnerHierarchyName+objrealname^;
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

{***************************************************************************
                                  trecorddef
***************************************************************************}

    constructor trecorddef.create(const n:string; p:TSymtable);
      begin
         inherited create(n,recorddef);
         symtable:=p;
         { we can own the symtable only if nobody else owns a copy so far }
         if symtable.refcount=1 then
           symtable.defowner:=self;
         isunion:=false;
>>>>>>> origin/cpstrnew
      end;

    procedure tabstractrecorddef.check_forwards;
      begin
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
>>>>>>> graemeg/cpstrnew
      end;

    constructor tabstractrecorddef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
        inherited ppuload(dt,ppufile);
        objrealname:=stringdup(ppufile.getstring);
        objname:=stringdup(upper(objrealname^));
        ppufile.getsmallset(objectoptions);
      end;

<<<<<<< HEAD
    procedure tabstractrecorddef.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putstring(objrealname^);
        ppufile.putsmallset(objectoptions);
      end;

    destructor tabstractrecorddef.destroy;
      begin
        stringdispose(objname);
        stringdispose(objrealname);
        inherited destroy;
=======
    function tabstractrecorddef.RttiName: string;
      var
        tmp: tabstractrecorddef;
      begin
        Result:=OwnerHierarchyName+objrealname^;
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

{***************************************************************************
                                  trecorddef
***************************************************************************}

    constructor trecorddef.create(const n:string; p:TSymtable);
      begin
         inherited create(n,recorddef);
         symtable:=p;
         { we can own the symtable only if nobody else owns a copy so far }
         if symtable.refcount=1 then
           symtable.defowner:=self;
         isunion:=false;
>>>>>>> graemeg/cpstrnew
      end;

    procedure tabstractrecorddef.check_forwards;
      begin
        tstoredsymtable(symtable).check_forwards;
      end;

    function tabstractrecorddef.find_procdef_bytype(pt:tproctypeoption): tprocdef;
      var
        i: longint;
        sym: tsym;
      begin
<<<<<<< HEAD
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
=======
         inherited ppuload(recorddef,ppufile);
         if df_copied_def in defoptions then
           ppufile.getderef(cloneddefderef)
         else
           begin
             symtable:=trecordsymtable.create(objrealname^,0);
             trecordsymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).padalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).usefieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).datasize:=ppufile.getaint;
             trecordsymtable(symtable).ppuload(ppufile);
             { requires usefieldalignment to be set }
             symtable.defowner:=self;
           end;
         isunion:=false;
>>>>>>> graemeg/cpstrnew
      end;

    function tabstractrecorddef.GetSymtable(t:tGetSymtable):TSymtable;
      begin
        if t=gs_record then
          GetSymtable:=symtable
        else
          GetSymtable:=nil;
>>>>>>> graemeg/cpstrnew
      end;

    procedure tabstractrecorddef.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putstring(objrealname^);
        if assigned(import_lib) then
          ppufile.putstring(import_lib^)
        else
          ppufile.putstring('');
        ppufile.putsmallset(objectoptions);
      end;

<<<<<<< HEAD
    destructor tabstractrecorddef.destroy;
      begin
        stringdispose(objname);
        stringdispose(objrealname);
        stringdispose(import_lib);
        tcinitcode.free;
        inherited destroy;
=======
    function tabstractrecorddef.RttiName: string;
      var
        tmp: tabstractrecorddef;
      begin
<<<<<<< HEAD
        Result:=OwnerHierarchyName+objrealname^;
=======
    constructor tabstractrecorddef.create(const n:string; dt:tdeftyp);
      begin
        inherited create(dt);
        objname:=stringdup(upper(n));
        objrealname:=stringdup(n);
        objectoptions:=[];
=======
        s:='';
        first:=true;
        for i:=0 to paras.count-1 do
         begin
           hp:=tparavarsym(paras[i]);
           if not(vo_is_hidden_para in hp.varoptions) or
              (showhidden) then
            begin
               if first then
                begin
                  s:=s+'(';
                  first:=false;
                end
               else
                s:=s+',';
               if vo_is_hidden_para in hp.varoptions then
                 s:=s+'<';
               case hp.varspez of
                 vs_var :
                   s:=s+'var';
                 vs_const :
                   s:=s+'const';
                 vs_out :
                   s:=s+'out';
               end;
               if assigned(hp.vardef.typesym) then
                 begin
                   if s<>'(' then
                    s:=s+' ';
                   hs:=hp.vardef.typesym.realname;
                   if hs[1]<>'$' then
                     s:=s+hp.vardef.typesym.realname
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
                    conststring,
                    constresourcestring :
                      begin
                      If hpc.value.len>0 then
                        begin
                        setLength(hs,hpc.value.len);
                        move(hpc.value.valueptr^,hs[1],hpc.value.len);
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
                  end;
                  if hs<>'' then
                   s:=s+'="'+hs+'"';
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
>>>>>>> graemeg/fixes_2_2
      end;

    constructor tabstractrecorddef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
      begin
<<<<<<< HEAD
        inherited ppuload(dt,ppufile);
        objrealname:=stringdup(ppufile.getstring);
        objname:=stringdup(upper(objrealname^));
        ppufile.getsmallset(objectoptions);
=======
        result:=trecorddef.create(objrealname^,symtable.getcopy);
        trecorddef(result).isunion:=isunion;
        include(trecorddef(result).defoptions,df_copied_def);
>>>>>>> graemeg/cpstrnew
      end;

    procedure tabstractrecorddef.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putstring(objrealname^);
        ppufile.putsmallset(objectoptions);
      end;

    destructor tabstractrecorddef.destroy;
      begin
        stringdispose(objname);
        stringdispose(objrealname);
        inherited destroy;
      end;

    procedure tabstractrecorddef.check_forwards;
      begin
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
>>>>>>> graemeg/cpstrnew
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

<<<<<<< HEAD
=======
    function tabstractrecorddef.RttiName: string;
      var
        tmp: tabstractrecorddef;
      begin
        Result:=OwnerHierarchyName+objrealname^;
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

>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
=======
            for t:=NOTOKEN to last_overloaded do
              if procsym.realname='$'+overloaded_names[t] then
                begin
                  s:='operator '+arraytokeninfo[t].str+typename_paras(showhidden);
                  break;
                end;
          end
        else
          s:=s+procsym.realname+typename_paras(showhidden);
        case proctypeoption of
          potype_constructor:
            s:='constructor '+s;
          potype_destructor:
            s:='destructor '+s;
          else
            if assigned(returndef) and
              not(is_void(returndef)) then
              s:=s+':'+returndef.GetTypeName;
        end;
        s:=s+';';
        { forced calling convention? }
        if (po_hascallingconvention in procoptions) then
          s:=s+' '+ProcCallOptionStr[proccalloption]+';';
        if po_staticmethod in procoptions then
          s:=s+' Static;';
        fullprocname:=s;
>>>>>>> graemeg/fixes_2_2
      end;

{***************************************************************************
                                  trecorddef
***************************************************************************}

    constructor trecorddef.create(const n:string; p:TSymtable);
      begin
         inherited create(n,recorddef);
         symtable:=p;
         { we can own the symtable only if nobody else owns a copy so far }
         if symtable.refcount=1 then
           symtable.defowner:=self;
         isunion:=false;
>>>>>>> graemeg/cpstrnew
      end;


    procedure tabstractrecorddef.buildderefimpl;
      begin
<<<<<<< HEAD
         inherited buildderefimpl;
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).buildderefimpl;
=======
         inherited ppuload(recorddef,ppufile);
         if df_copied_def in defoptions then
           ppufile.getderef(cloneddefderef)
         else
           begin
             symtable:=trecordsymtable.create(objrealname^,0);
             trecordsymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).padalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).usefieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).datasize:=ppufile.getaint;
             trecordsymtable(symtable).ppuload(ppufile);
             { requires usefieldalignment to be set }
             symtable.defowner:=self;
           end;
         isunion:=false;
>>>>>>> graemeg/cpstrnew
      end;


    procedure tabstractrecorddef.derefimpl;
      begin
        inherited derefimpl;
        if not (df_copied_def in defoptions) then
          tstoredsymtable(symtable).derefimpl(false);
      end;


    procedure tabstractrecorddef.check_forwards;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        { the defs of a copied def are defined for the original type only }
        if not(df_copied_def in defoptions) then
          tstoredsymtable(symtable).check_forwards;
=======
=======
>>>>>>> graemeg/cpstrnew
        result:=trecorddef.create(objrealname^,symtable.getcopy);
        trecorddef(result).isunion:=isunion;
        include(trecorddef(result).defoptions,df_copied_def);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
        result:=tabstractrecordsymtable(symtable).is_packed;
=======
         inherited deref;
         { now dereference the definitions }
         if df_copied_def in defoptions then
           begin
             cloneddef:=trecorddef(cloneddefderef.resolve);
             symtable:=cloneddef.symtable.getcopy;
           end
         else
           tstoredsymtable(symtable).deref;

         { assign TGUID? load only from system unit }
         if not(assigned(rec_tguid)) and
            (upper(typename)='TGUID') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_tguid:=self;
<<<<<<< HEAD
=======

         { assign JMP_BUF? load only from system unit }
         if not(assigned(rec_jmp_buf)) and
            (upper(typename)='JMP_BUF') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_jmp_buf:=self;
      end;
>>>>>>> graemeg/cpstrnew

         { assign JMP_BUF? load only from system unit }
         if not(assigned(rec_jmp_buf)) and
            (upper(typename)='JMP_BUF') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_jmp_buf:=self;
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
=======
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
>>>>>>> origin/fixes_2.4
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

<<<<<<< HEAD
    function tabstractrecorddef.search_enumerator_get: tprocdef;
      var
        sym : tsym;
        i : integer;
        pd : tprocdef;
        hashedid : THashedIDString;
=======

    function trecorddef.size:asizeint;
>>>>>>> graemeg/cpstrnew
      begin
        result:=nil;
        hashedid.id:='GETENUMERATOR';
        sym:=tsym(symtable.FindWithHash(hashedid));
        if assigned(sym) and (sym.typ=procsym) then
          begin
<<<<<<< HEAD
            for i := 0 to Tprocsym(sym).ProcdefList.Count - 1 do
            begin
              pd := tprocdef(Tprocsym(sym).ProcdefList[i]);
              if (pd.proctypeoption = potype_function) and
                 (is_class_or_interface_or_object(pd.returndef) or is_record(pd.returndef)) and
                 (pd.visibility >= vis_public) then
=======
            crc:=0;
            for i:=0 to paras.count-1 do
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
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
=======
        { outdated gcc 2.x name mangling scheme }
{$ifdef NAMEMANGLING_GCC2}

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
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
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

<<<<<<< HEAD
        if with_package_name and
           assigned(import_lib) then
          result:=import_lib^+'/'+result;
=======
         { now we handle the parameters }
         if maxparacount>0 then
           begin
             for i:=0 to paras.count-1 do
               begin
                 hp:=tparavarsym(paras[i]);
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

         { gcc 3.x name mangling scheme }
         if procsym.owner.symtabletype=ObjectSymtable then
           begin
             s:='_ZN';

             s2:=tobjectdef(procsym.owner.defowner).objrealname^;
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
                 s2:=getcppparaname(hp.vardef);
                 if hp.varspez in [vs_var,vs_out] then
                   s2:='R'+s2;
                 s:=s+s2;
               end;
           end
         else
           s:=s+'v';
         cplusplusmangledname:=s;
>>>>>>> origin/fixes_2_2
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
      end;


    constructor trecorddef.create_global_internal(n: string; packrecords, recordalignmin, maxCrecordalign: shortint);
      var
        oldsymtablestack: tsymtablestack;
        ts: ttypesym;
        definedname: boolean;
      begin
        { construct name }
        definedname:=n<>'';
        if not definedname then
          n:='$InternalRec'+tostr(current_module.deflist.count);
        oldsymtablestack:=symtablestack;
        { do not simply push/pop current_module.localsymtable, because
          that can have side-effects (e.g., it removes helpers) }
        symtablestack:=nil;

        symtable:=trecordsymtable.create(n,packrecords,recordalignmin,maxCrecordalign);
        symtable.defowner:=self;
        isunion:=false;
        inherited create(n,recorddef,true);
        { if we specified a name, then we'll probably want to look up the
          type again by name too -> create typesym }
        ts:=nil;
        if definedname then
          begin
            ts:=ctypesym.create(n,self,true);
            { avoid hints about unused types (these may only be used for
              typed constant data) }
            ts.increfcount;
          end;
        if assigned(current_module.localsymtable) then
          begin
            current_module.localsymtable.insertdef(self);
            if definedname then
              current_module.localsymtable.insert(ts);
          end
        else
          begin
            current_module.globalsymtable.insertdef(self);
            if definedname then
              current_module.globalsymtable.insert(ts);
          end;
        symtablestack:=oldsymtablestack;
        { don't create RTTI for internal types, these are not exported }
        defstates:=defstates+[ds_rtti_table_written,ds_init_table_written];
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
        sym:=cfieldvarsym.create(pname^,vs_value,def,[],true);
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
             ppuload_platform(ppufile);
             symtable:=trecordsymtable.create(objrealname^,0,0,0);
             trecordsymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).padalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).usefieldalignment:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).recordalignmin:=shortint(ppufile.getbyte);
             trecordsymtable(symtable).datasize:=ppufile.getasizeint;
             trecordsymtable(symtable).paddingsize:=ppufile.getword;
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
      begin
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
        needs_inittable:=trecordsymtable(symtable).needs_init_final
      end;

    function trecorddef.needs_separate_initrtti : boolean;
      begin
        result:=true;
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
<<<<<<< HEAD
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
=======
           tstoredsymtable(symtable).deref;

         { assign TGUID? load only from system unit }
         if not(assigned(rec_tguid)) and
            (upper(typename)='TGUID') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_tguid:=self;

         { assign JMP_BUF? load only from system unit }
         if not(assigned(rec_jmp_buf)) and
            (upper(typename)='JMP_BUF') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_jmp_buf:=self;
>>>>>>> graemeg/cpstrnew
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


<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
         ppufile.getnormalset(procoptions);

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
         ppufile.putnormalset(procoptions);
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
        i     : integer;
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
               end;
<<<<<<< HEAD
               if (pno_paranames in pno) then
                 s:=s+hp.realname+':';
=======
>>>>>>> graemeg/cpstrnew
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
                    conststring,
                    constresourcestring :
                      begin
                      If hpc.value.len>0 then
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
                  end;
                  if hs<>'' then
                   s:=s+'="'+hs+'"';
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


    function tabstractprocdef.getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp): tstoreddef;
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
                  if (copytyp=pc_bareproc) and
                     (([vo_is_self,vo_is_vmt,vo_is_parentfp,vo_is_result,vo_is_funcret]*pvs.varoptions)<>[]) then
                    continue;
                  npvs:=cparavarsym.create(pvs.realname,pvs.paranr,pvs.varspez,
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
<<<<<<< HEAD
         { nested procvars require that nested functions use the Delphi-style
           nested procedure calling convention }
         if (parast.symtablelevel>normal_function_level) and
            (m_nested_procvars in current_settings.modeswitches) then
           include(procoptions,po_delphi_nested_cc);
=======
        result:=trecorddef.create(objrealname^,symtable.getcopy);
        trecorddef(result).isunion:=isunion;
        include(trecorddef(result).defoptions,df_copied_def);
>>>>>>> origin/cpstrnew
      end;


    procedure tabstractprocdef.init_paraloc_info(side: tcallercallee);
      begin
        if (side in [callerside,callbothsides]) and
           not(has_paraloc_info in [callerside,callbothsides]) then
          begin
            callerargareasize:=paramanager.create_paraloc_info(self,callerside);
            if has_paraloc_info in [calleeside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=callerside;
          end;
        if (side in [calleeside,callbothsides]) and
           not(has_paraloc_info in [calleeside,callbothsides]) then
          begin
            calleeargareasize:=paramanager.create_paraloc_info(self,calleeside);
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
<<<<<<< HEAD
        result:=false;
        init_paraloc_info(side);
        for i:=0 to parast.SymList.Count-1 do
          if tsym(parast.SymList[i]).typ=paravarsym then
            begin
              p:=tparavarsym(parast.SymList[i]);
              { check if no parameter is located on the stack }
              if is_open_array(p.vardef) or
                 is_array_of_const(p.vardef) then
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
=======
         inherited deref;
         { now dereference the definitions }
         if df_copied_def in defoptions then
           begin
             cloneddef:=trecorddef(cloneddefderef.resolve);
             symtable:=cloneddef.symtable.getcopy;
           end
         else
           tstoredsymtable(symtable).deref;

         { assign TGUID? load only from system unit }
         if not(assigned(rec_tguid)) and
            (upper(typename)='TGUID') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_tguid:=self;

         { assign JMP_BUF? load only from system unit }
         if not(assigned(rec_jmp_buf)) and
            (upper(typename)='JMP_BUF') and
            assigned(owner) and
            assigned(owner.name) and
            (owner.name^='SYSTEM') then
           rec_jmp_buf:=self;
>>>>>>> origin/cpstrnew
      end;

    function tabstractprocdef.is_pushleftright: boolean;
      begin
        result:=false;
      end;

    function tabstractprocdef.address_type: tdef;
      begin
        result:=voidcodepointertype;
      end;


    procedure tabstractprocdef.declared_far;
      begin
        Message1(parser_w_proc_directive_ignored,'FAR');
      end;


<<<<<<< HEAD
    procedure tabstractprocdef.declared_near;
=======
    function trecorddef.size:asizeint;
>>>>>>> origin/cpstrnew
      begin
        Message1(parser_w_proc_directive_ignored,'NEAR');
      end;


{***************************************************************************
                                  TPROCDEF
***************************************************************************}

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
<<<<<<< HEAD
        if not assigned(implprocdefinfo) then
          internalerror(2014010305);
        result:=implprocdefinfo^.parentfpstructptrtype;
=======
         inherited create(dt);
         parast:=tparasymtable.create(self,level);
         paras:=nil;
         minparacount:=0;
         maxparacount:=0;
         proctypeoption:=potype_none;
         proccalloption:=pocall_none;
         procoptions:=[];
         returndef:=voidtype;
         savesize:=sizeof(pint);
         callerargareasize:=0;
         calleeargareasize:=0;
         has_paraloc_info:=callnoside;
         funcretloc[callerside].init;
         funcretloc[calleeside].init;
         check_mark_as_nested;
>>>>>>> origin/cpstrnew
      end;


    procedure tprocdef.SetParentFPInitBlock(AValue: tnode);
      begin
<<<<<<< HEAD
        if not assigned(implprocdefinfo) then
          internalerror(2014010306);
        implprocdefinfo^.parentfpinitblock:=AValue;
=======
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
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
    function tprocdef.Getforwarddef: boolean;
=======
    constructor tabstractprocdef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
>>>>>>> origin/cpstrnew
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010312);
        result:=implprocdefinfo^.forwarddef;
      end;

<<<<<<< HEAD

    function tprocdef.Gethasforward: boolean;
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010313);
        result:=implprocdefinfo^.hasforward;
=======
         funcretloc[callerside].init;
         if po_explicitparaloc in procoptions then
           funcretloc[callerside].ppuload(ppufile);

         savesize:=sizeof(pint);
         if (po_explicitparaloc in procoptions) then
           has_paraloc_info:=callerside;
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
    procedure tprocdef.Setinterfacedef(AValue: boolean);
=======
    constructor tabstractprocdef.ppuload(dt:tdeftyp;ppufile:tcompilerppufile);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010317);
        implprocdefinfo^.interfacedef:=AValue;
      end;

<<<<<<< HEAD

    procedure tprocdef.Setprocstarttai(AValue: tai);
      begin
        if not assigned(implprocdefinfo) then
          internalerror(2014010318);
        implprocdefinfo^.procstarttai:=AValue;
      end;

<<<<<<< HEAD
    function tabstractprocdef.no_self_node: boolean;
      begin
        Result:=([po_staticmethod,po_classmethod]<=procoptions)or
                (proctypeoption in [potype_class_constructor,potype_class_destructor,potype_operator]);
=======
=======
>>>>>>> graemeg/cpstrnew
         funcretloc[callerside].init;
         if po_explicitparaloc in procoptions then
           funcretloc[callerside].ppuload(ppufile);

         savesize:=sizeof(pint);
         if (po_explicitparaloc in procoptions) then
           has_paraloc_info:=callerside;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
      end;


    procedure tabstractprocdef.check_mark_as_nested;
      begin
         { nested procvars require that nested functions use the Delphi-style
           nested procedure calling convention }
         if (parast.symtablelevel>normal_function_level) and
            (m_nested_procvars in current_settings.modeswitches) then
           include(procoptions,po_delphi_nested_cc);
      end;

<<<<<<< HEAD

    procedure tabstractprocdef.init_paraloc_info(side: tcallercallee);
      begin
        if (side in [callerside,callbothsides]) and
           not(has_paraloc_info in [callerside,callbothsides]) then
          begin
            callerargareasize:=paramanager.create_paraloc_info(self,callerside);
            if has_paraloc_info in [calleeside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=callerside;
          end;
        if (side in [calleeside,callbothsides]) and
           not(has_paraloc_info in [calleeside,callbothsides]) then
          begin
            calleeargareasize:=paramanager.create_paraloc_info(self,calleeside);
            if has_paraloc_info in [callerside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=calleeside;
          end;
=======
         if (po_explicitparaloc in procoptions) then
           funcretloc[callerside].ppuwrite(ppufile);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
              if is_open_array(p.vardef) or
                 is_array_of_const(p.vardef) then
                begin
<<<<<<< HEAD
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
=======
                  s:=s+'(';
                  first:=false;
                end
               else
                s:=s+',';
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
               end;
               if hp.univpara then
                 s:=s+'univ ';
               if assigned(hp.vardef.typesym) then
                 begin
                   hs:=hp.vardef.typesym.realname;
                   if hs[1]<>'$' then
                     s:=s+hs
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
                    conststring,
                    constresourcestring :
                      begin
                      If hpc.value.len>0 then
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
                  end;
                  if hs<>'' then
                   s:=s+'="'+hs+'"';
>>>>>>> graemeg/cpstrnew
                end;
            end;
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

    function tabstractprocdef.no_self_node: boolean;
      begin
        Result:=([po_staticmethod,po_classmethod]<=procoptions)or
                (proctypeoption in [potype_class_constructor,potype_class_destructor,potype_operator]);
      end;

    function tabstractprocdef.no_self_node: boolean;
      begin
        Result:=([po_staticmethod,po_classmethod]<=procoptions)or
                (proctypeoption in [potype_class_constructor,potype_class_destructor,potype_operator]);
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
            callerargareasize:=paramanager.create_paraloc_info(self,callerside);
            if has_paraloc_info in [calleeside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=callerside;
          end;
        if (side in [calleeside,callbothsides]) and
           not(has_paraloc_info in [calleeside,callbothsides]) then
          begin
            calleeargareasize:=paramanager.create_paraloc_info(self,calleeside);
            if has_paraloc_info in [callerside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=calleeside;
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
              if is_open_array(p.vardef) or
                 is_array_of_const(p.vardef) then
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




    procedure tabstractprocdef.check_mark_as_nested;
      begin
         { nested procvars require that nested functions use the Delphi-style
           nested procedure calling convention }
         if (parast.symtablelevel>normal_function_level) and
            (m_nested_procvars in current_settings.modeswitches) then
           include(procoptions,po_delphi_nested_cc);
      end;

    function tabstractprocdef.no_self_node: boolean;
      begin
        Result:=([po_staticmethod,po_classmethod]<=procoptions)or
                (proctypeoption in [potype_class_constructor,potype_class_destructor,potype_operator]);
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
            callerargareasize:=paramanager.create_paraloc_info(self,callerside);
            if has_paraloc_info in [calleeside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=callerside;
          end;
        if (side in [calleeside,callbothsides]) and
           not(has_paraloc_info in [calleeside,callbothsides]) then
          begin
            calleeargareasize:=paramanager.create_paraloc_info(self,calleeside);
            if has_paraloc_info in [callerside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=calleeside;
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
              if is_open_array(p.vardef) or
                 is_array_of_const(p.vardef) then
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




    procedure tabstractprocdef.init_paraloc_info(side: tcallercallee);
      begin
        if (side in [callerside,callbothsides]) and
           not(has_paraloc_info in [callerside,callbothsides]) then
          begin
            callerargareasize:=paramanager.create_paraloc_info(self,callerside);
            if has_paraloc_info in [calleeside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=callerside;
          end;
        if (side in [calleeside,callbothsides]) and
           not(has_paraloc_info in [calleeside,callbothsides]) then
          begin
            calleeargareasize:=paramanager.create_paraloc_info(self,calleeside);
            if has_paraloc_info in [callerside,callbothsides] then
              has_paraloc_info:=callbothsides
            else
              has_paraloc_info:=calleeside;
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
              if is_open_array(p.vardef) or
                 is_array_of_const(p.vardef) then
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
         forwarddef:=true;
         interfacedef:=false;
         hasforward:=false;
         struct := nil;
         import_dll:=nil;
         import_name:=nil;
         import_nr:=0;
         inlininginfo:=nil;
         deprecatedmsg:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef i386}
          fpu_used:=maxfpuregs;
{$endif i386}
         interruptvector:=-1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
          _mangledname:=stringdup(ppufile.getstring)
         else
          _mangledname:=nil;
{$endif symansistr}
         extnumber:=ppufile.getword;
         level:=ppufile.getbyte;
         ppufile.getderef(structderef);
         ppufile.getderef(procsymderef);
         ppufile.getposinfo(fileinfo);
         visibility:=tvisibility(ppufile.getbyte);
         ppufile.getsmallset(symoptions);
         if sp_has_deprecated_msg in symoptions then
           deprecatedmsg:=stringdup(ppufile.getstring)
         else
           deprecatedmsg:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         synthetickind:=tsynthetickind(ppufile.getbyte);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef powerpc}
         { library symbol for AmigaOS/MorphOS }
         ppufile.getderef(libsymderef);
{$endif powerpc}
>>>>>>> graemeg/cpstrnew
         { import stuff }
         if po_has_importdll in procoptions then
           import_dll:=stringdup(ppufile.getstring)
         else
           import_dll:=nil;
         if po_has_importname in procoptions then
           import_name:=stringdup(ppufile.getstring)
         else
           import_name:=nil;
         import_nr:=ppufile.getword;
         if target_info.system in systems_interrupt_table then
           interruptvector:=ppufile.getlongint;
         if (po_msgint in procoptions) then
           messageinf.i:=ppufile.getlongint;
         if (po_msgstr in procoptions) then
           messageinf.str:=stringdup(ppufile.getstring);
         if (po_dispid in procoptions) then
           dispid:=ppufile.getlongint;
         { inline stuff }
         ppufile.getsmallset(implprocoptions);
         if has_inlininginfo then
           begin
             ppufile.getderef(funcretsymderef);
             new(inlininginfo);
             ppufile.getsmallset(inlininginfo^.flags);
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
         if has_inlininginfo then
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
         ppufile.putsmallset(symoptions);
         if sp_has_deprecated_msg in symoptions then
           ppufile.putstring(deprecatedmsg^);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         ppufile.putbyte(byte(synthetickind));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef powerpc}
         { library symbol for AmigaOS/MorphOS }
         ppufile.putderef(libsymderef);
{$endif powerpc}
>>>>>>> graemeg/cpstrnew
         { import }
         if po_has_importdll in procoptions then
           ppufile.putstring(import_dll^);
         if po_has_importname in procoptions then
           ppufile.putstring(import_name^);
         ppufile.putword(import_nr);
         if target_info.system in systems_interrupt_table then
           ppufile.putlongint(interruptvector);
         if (po_msgint in procoptions) then
           ppufile.putlongint(messageinf.i);
         if (po_msgstr in procoptions) then
           ppufile.putstring(messageinf.str^);
         if (po_dispid in procoptions) then
           ppufile.putlongint(dispid);
         { inline stuff }
         oldintfcrc:=ppufile.do_crc;
         ppufile.do_crc:=false;
         ppufile.putsmallset(implprocoptions);
         if has_inlininginfo then
           begin
             ppufile.putderef(funcretsymderef);
             ppufile.putsmallset(inlininginfo^.flags);
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
         if has_inlininginfo then
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      var
        pno: tprocnameoptions;
      begin
        pno:=[];
        if showhidden then
          include(pno,pno_showhidden);
        result:=customprocname(pno);
      end;


    function tprocdef.customprocname(pno: tprocnameoptions):ansistring;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      var
        s, rn : ansistring;
        t : ttoken;
      begin
{$ifdef EXTDEBUG}
        include(pno,pno_showhidden);
{$endif EXTDEBUG}
        s:='';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if assigned(struct) then
         begin
           s:=struct.RttiName+'.';
           if (po_classmethod in procoptions) and
              not (proctypeoption in [potype_class_constructor,potype_class_destructor]) then
             s:='class ' + s;
         end;
>>>>>>> graemeg/cpstrnew
        if proctypeoption=potype_operator then
          begin
            for t:=NOTOKEN to last_overloaded do
              if procsym.realname='$'+overloaded_names[t] then
                begin
                  s:='operator ';
                  if (pno_ownername in pno) and
                     assigned(struct) then
                    s:=s+struct.RttiName+'.';
                  s:=s+arraytokeninfo[t].str+typename_paras(pno);
                  break;
                end;
          end
        else
<<<<<<< HEAD
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
            rn:=procsym.realname;
            if (pno_noleadingdollar in pno) and
               (rn[1]='$') then
              delete(rn,1,1);
            s:=s+rn+typename_paras(pno);
          end;
        if not(proctypeoption in [potype_constructor,potype_destructor,
             potype_class_constructor,potype_class_destructor]) and
           assigned(returndef) and
           not(is_void(returndef)) then
          s:=s+':'+returndef.GetTypeName;
        if owner.symtabletype=localsymtable then
          s:=s+' is nested'
        else if po_is_block in procoptions then
          s:=s+' is block';
=======
          s:=s+procsym.realname+typename_paras(showhidden);
        case proctypeoption of
          potype_constructor:
            s:='constructor '+s;
          potype_destructor:
            s:='destructor '+s;
          potype_class_constructor:
            s:='class constructor '+s;
          potype_class_destructor:
            s:='class destructor '+s;
          else
            if assigned(returndef) and
              not(is_void(returndef)) then
              s:=s+':'+returndef.GetTypeName;
        end;
        if owner.symtabletype=localsymtable then
          s:=s+' is nested';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        s:=s+';';
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        result:=(owner.symtabletype=ObjectSymtable)and not no_self_node;
=======
        result:=owner.symtabletype=ObjectSymtable;
>>>>>>> graemeg/cpstrnew
=======
        result:=owner.symtabletype=ObjectSymtable;
>>>>>>> graemeg/cpstrnew
=======
        result:=owner.symtabletype=ObjectSymtable;
>>>>>>> graemeg/cpstrnew
=======
        result:=owner.symtabletype=ObjectSymtable;
>>>>>>> origin/cpstrnew
      end;


    function tprocdef.is_addressonly:boolean;
      begin
        result:=assigned(owner) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                not is_methodpointer and
=======
                (owner.symtabletype<>ObjectSymtable) and
>>>>>>> graemeg/cpstrnew
=======
                (owner.symtabletype<>ObjectSymtable) and
>>>>>>> graemeg/cpstrnew
=======
                (owner.symtabletype<>ObjectSymtable) and
>>>>>>> graemeg/cpstrnew
=======
                (owner.symtabletype<>ObjectSymtable) and
>>>>>>> origin/cpstrnew
                (not(m_nested_procvars in current_settings.modeswitches) or
                 not is_nested_pd(self));
      end;


    procedure tprocdef.make_external;
      begin
        include(procoptions,po_external);
        forwarddef:=false;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      end;


    procedure tprocdef.init_genericdecl;
      begin
        if assigned(genericdecltokenbuf) then
          internalerror(2015061901);
        genericdecltokenbuf:=tdynamicarray.create(256);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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


    function tprocdef.getcopyas(newtyp: tdeftyp; copytyp: tproccopytyp): tstoreddef;
      var
        j : longint;
      begin
        result:=inherited getcopyas(newtyp,copytyp);
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
        if copytyp<>pc_bareproc then
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
          end;
        if assigned(import_dll) then
          tprocdef(result).import_dll:=stringdup(import_dll^);
        if assigned(import_name) then
          tprocdef(result).import_name:=stringdup(import_name^);
        tprocdef(result).import_nr:=import_nr;
        tprocdef(result).extnumber:=$ffff;
        tprocdef(result).visibility:=visibility;
        tprocdef(result).synthetickind:=synthetickind;
        { we need a separate implementation for the copied def }
        tprocdef(result).forwarddef:=true;
        tprocdef(result).interfacedef:=true;

        { create new paralist }
        tprocdef(result).calcparas;
      end;


    function tprocdef.getcopy: tstoreddef;
      begin
        result:=getcopyas(procdef,pc_normal);
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

         { inline tree }
         if has_inlininginfo then
           begin
             { Localst is not available for main/unit init }
             if assigned(localst) then
               begin
                 tlocalsymtable(localst).buildderef;
                 tlocalsymtable(localst).buildderefimpl;
               end;

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

        { Inline }
        if has_inlininginfo then
          begin
            { Locals }
            if assigned(localst) then
              begin
                tlocalsymtable(localst).deref(false);
                tlocalsymtable(localst).derefimpl(false);
              end;

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
<<<<<<< HEAD
        defaultmangledname:=make_mangledname('',procsym.owner,procsym.name);
        defaultmangledname:=defaultmangledname+mangledprocparanames(Length(defaultmangledname))
=======
        mangledname:=make_mangledname('',procsym.owner,procsym.name);
        oldlen:=length(mangledname);
        { add parameter types }
        for i:=0 to paras.count-1 do
         begin
           hp:=tparavarsym(paras[i]);
           if not(vo_is_hidden_para in hp.varoptions) then
             mangledname:=mangledname+'$'+hp.vardef.mangledparaname;
         end;
        { add resultdef, add $$ as separator to make it unique from a
          parameter separator }
        if not is_void(returndef) then
          mangledname:=mangledname+'$$'+returndef.mangledparaname;
        newlen:=length(mangledname);
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
            hs:=hp.vardef.mangledparaname;
            crc:=UpdateCrc32(crc,hs[1],length(hs));
            mangledname:=Copy(mangledname,1,oldlen)+'$crc'+hexstr(crc,8);
          end;
       {$ifdef compress}
        _mangledname:=stringdup(minilzw_encode(mangledname));
       {$else}
        _mangledname:=stringdup(mangledname);
       {$endif}
>>>>>>> graemeg/cpstrnew
      end;


    function tprocdef.cplusplusmangledname : TSymStr;

      function getcppparaname(p : tdef) : TSymStr;

        const
{$ifdef NAMEMANGLING_GCC2}
           ordtype2str : array[tordtype] of string[2] = (
             '',
             'Uc','Us','Ui','Us','',
             'Sc','s','i','x','',
             'b','b','b','b','b',
             'c','w','x');
{$else NAMEMANGLING_GCC2}
           ordtype2str : array[tordtype] of string[1] = (
             'v',
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             'h','t','j','y','',
             'a','s','i','x','',
             'b','b','b','b',
             'b','b','b','b',
=======
             'h','t','j','y',
             'a','s','i','x',
             'b','b','b','b','b',
>>>>>>> graemeg/cpstrnew
=======
             'h','t','j','y',
             'a','s','i','x',
             'b','b','b','b','b',
>>>>>>> graemeg/cpstrnew
             'c','w','x');
{$else NAMEMANGLING_GCC2}
           ordtype2str : array[tordtype] of string[1] = (
             'v',
             'h','t','j','y',
             'a','s','i','x',
             'b','b','b','b','b',
             'c','w','x');

           floattype2str : array[tfloattype] of string[1] = (
             'f','d','e','e',
             'd','d','g');
{$endif NAMEMANGLING_GCC2}
=======
             'h','t','j','y',
             'a','s','i','x',
             'b','b','b','b','b',
             'c','w','x');
>>>>>>> origin/cpstrnew

           floattype2str : array[tfloattype] of string[1] = (
             'f','d','e','e',
             'd','d','g');
{$endif NAMEMANGLING_GCC2}

        var
           s : TSymStr;

        begin
           case p.typ of
              orddef:
                s:=ordtype2str[torddef(p).ordtype];
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function  tprocdef.objcmangledname : TSymStr;
=======
    function  tprocdef.objcmangledname : string;
>>>>>>> graemeg/cpstrnew
=======
    function  tprocdef.objcmangledname : string;
>>>>>>> graemeg/cpstrnew
=======
    function  tprocdef.objcmangledname : string;
>>>>>>> graemeg/cpstrnew
=======
    function  tprocdef.objcmangledname : string;
>>>>>>> origin/cpstrnew
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
            result:=tprocvardef(def.getcopyas(procvardef,pc_address_only));
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         ppuload_platform(ppufile);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
        for i:=low(funcretloc) to high(funcretloc) do
=======
        for i:=low(tcallercallee) to high(tcallercallee) do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
         if ((po_methodpointer in procoptions) or
             is_nested_pd(self)) and
            not(po_addressonly in procoptions) then
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
        result:=(not(po_methodpointer in procoptions) and
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


    function tprocvardef.getcopyas(newtyp: tdeftyp; copytyp: tproccopytyp): tstoreddef;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        result:=inherited;
        tabstractprocdef(result).calcparas;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if not(po_methodpointer in procoptions) then
          if not is_nested_pd(self) then
            result:='procvar'
          else
            result:='nestedprovar'
        else
          result:='procvarofobj'
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
         if po_classmethod in procoptions then
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         { calling convention doesn't matter for blocks }
         if po_is_block in procoptions then
           GetTypeName := s+' is block;'
         else
           GetTypeName := s+';'+ProcCallOptionStr[proccalloption]+'>';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         GetTypeName := s+';'+ProcCallOptionStr[proccalloption]+'>';
>>>>>>> graemeg/cpstrnew
      end;


{***************************************************************************
                              TOBJECTDEF
***************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   constructor tobjectdef.create(ot:tobjecttyp;const n:string;c:tobjectdef;doregister:boolean);
     begin
        inherited create(n,objectdef,doregister);
=======
   constructor tobjectdef.create(ot:tobjecttyp;const n:string;c:tobjectdef);
     begin
        inherited create(n,objectdef);
>>>>>>> graemeg/cpstrnew
=======
   constructor tobjectdef.create(ot:tobjecttyp;const n:string;c:tobjectdef);
     begin
        inherited create(n,objectdef);
>>>>>>> graemeg/cpstrnew
=======
   constructor tobjectdef.create(ot:tobjecttyp;const n:string;c:tobjectdef);
     begin
        inherited create(n,objectdef);
>>>>>>> graemeg/cpstrnew
=======
   constructor tobjectdef.create(ot:tobjecttyp;const n:string;c:tobjectdef);
     begin
        inherited create(n,objectdef);
>>>>>>> origin/cpstrnew
        fcurrent_dispid:=0;
        objecttype:=ot;
        childof:=nil;
        if objecttype=odt_helper then
          owner.includeoption(sto_has_helper);
        symtable:=tObjectSymtable.create(self,n,current_settings.packrecords,
          current_settings.alignment.recordalignmin,current_settings.alignment.maxCrecordalign);
        { create space for vmt !! }
        vmtentries:=TFPList.Create;
        set_parent(c);
        if objecttype in [odt_interfacecorba,odt_interfacecom,odt_dispinterface] then
          prepareguid;
        { setup implemented interfaces }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if objecttype in [odt_class,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
=======
        if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
        if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
        if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
        if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> origin/cpstrnew
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
         objextname:=stringdup(ppufile.getstring);
         { only used for external Objective-C classes/protocols }
         if (objextname^='') then
           stringdispose(objextname);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         symtable:=tObjectSymtable.create(self,objrealname^,0,0,0);
         tObjectSymtable(symtable).datasize:=ppufile.getasizeint;
         tObjectSymtable(symtable).paddingsize:=ppufile.getword;
         tObjectSymtable(symtable).fieldalignment:=shortint(ppufile.getbyte);
         tObjectSymtable(symtable).recordalignment:=shortint(ppufile.getbyte);
         tObjectSymtable(symtable).recordalignmin:=shortint(ppufile.getbyte);
         ppufile.getderef(vmt_fieldderef);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         import_lib:=stringdup(ppufile.getstring);
         { only used for external C++ classes }
         if (import_lib^='') then
           stringdispose(import_lib);
         symtable:=tObjectSymtable.create(self,objrealname^,0);
         tObjectSymtable(symtable).datasize:=ppufile.getaint;
         tObjectSymtable(symtable).fieldalignment:=ppufile.getbyte;
         tObjectSymtable(symtable).recordalignment:=ppufile.getbyte;
         vmt_offset:=ppufile.getlongint;
>>>>>>> graemeg/cpstrnew
         ppufile.getderef(childofderef);

         { load guid }
         iidstr:=nil;
         if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface] then
           begin
              new(iidguid);
              ppufile.getguid(iidguid^);
              iidstr:=stringdup(ppufile.getstring);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if objecttype in [odt_class,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
=======
         if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
         if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
         if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
         if objecttype in [odt_class,odt_objcclass,odt_objcprotocol] then
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         writing_class_record_dbginfo:=false;
       end;


    destructor tobjectdef.destroy;
      begin
         if assigned(symtable) then
           begin
             symtable.free;
             symtable:=nil;
           end;
         stringdispose(objextname);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
         stringdispose(import_lib);
>>>>>>> graemeg/cpstrnew
=======
         stringdispose(import_lib);
>>>>>>> graemeg/cpstrnew
=======
         stringdispose(import_lib);
>>>>>>> graemeg/cpstrnew
=======
         stringdispose(import_lib);
>>>>>>> origin/cpstrnew
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
             resetvmtentries;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        result:=cobjectdef.create(objecttype,objrealname^,childof,true);
=======
        result:=tobjectdef.create(objecttype,objrealname^,childof);
>>>>>>> graemeg/cpstrnew
=======
        result:=tobjectdef.create(objecttype,objrealname^,childof);
>>>>>>> graemeg/cpstrnew
=======
        result:=tobjectdef.create(objecttype,objrealname^,childof);
>>>>>>> graemeg/cpstrnew
=======
        result:=tobjectdef.create(objecttype,objrealname^,childof);
>>>>>>> origin/cpstrnew
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
        if assigned(vmtentries) then
          begin
            tobjectdef(result).vmtentries:=TFPList.Create;
            tobjectdef(result).copyvmtentries(self);
          end;
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
         if assigned(objextname) then
           ppufile.putstring(objextname^)
         else
           ppufile.putstring('');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         ppufile.putasizeint(tObjectSymtable(symtable).datasize);
         ppufile.putword(tObjectSymtable(symtable).paddingsize);
         ppufile.putbyte(byte(tObjectSymtable(symtable).fieldalignment));
         ppufile.putbyte(byte(tObjectSymtable(symtable).recordalignment));
         ppufile.putbyte(byte(tObjectSymtable(symtable).recordalignmin));
         ppufile.putderef(vmt_fieldderef);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         if assigned(import_lib) then
           ppufile.putstring(import_lib^)
         else
           ppufile.putstring('');
         ppufile.putaint(tObjectSymtable(symtable).datasize);
         ppufile.putbyte(tObjectSymtable(symtable).fieldalignment);
         ppufile.putbyte(tObjectSymtable(symtable).recordalignment);
         ppufile.putlongint(vmt_offset);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
            psym:=tprocsym.create(nname);
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


    procedure tobjectdef.buildderefimpl;
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
         { the procdefs are not owned by the class helper procsyms, so they
           are not stored/restored either -> re-add them here }
         if (objecttype in [odt_objcclass,odt_objcprotocol]) or
=======
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).derefimpl;
         { the procdefs are not owned by the class helper procsyms, so they
           are not stored/restored either -> re-add them here }
         if (objecttype=odt_objcclass) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            (oo_is_classhelper in objectoptions) then
           symtable.DefList.ForEachCall(@create_class_helper_for_procdef,nil);
      end;


    procedure tobjectdef.resetvmtentries;
      var
        i : longint;
      begin
        for i:=0 to vmtentries.Count-1 do
          Dispose(pvmtentry(vmtentries[i]));
        vmtentries.clear;
      end;


    procedure tobjectdef.copyvmtentries(objdef:tobjectdef);
      var
        i : longint;
        vmtentry : pvmtentry;
      begin
        resetvmtentries;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (objecttype in [odt_class,odt_object,odt_objcclass,odt_javaclass]) then
=======
        if (objecttype in [odt_class,odt_object,odt_objcclass]) then
>>>>>>> graemeg/cpstrnew
=======
        if (objecttype in [odt_class,odt_object,odt_objcclass]) then
>>>>>>> graemeg/cpstrnew
=======
        if (objecttype in [odt_class,odt_object,odt_objcclass]) then
>>>>>>> graemeg/cpstrnew
=======
        if (objecttype in [odt_class,odt_object,odt_objcclass]) then
>>>>>>> origin/cpstrnew
          begin
<<<<<<< HEAD
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
=======
            crc:=0;
            for i:=0 to paras.count-1 do
>>>>>>> graemeg/fixes_2_2
              begin
                vmt_field:=c.vmt_field;
                include(objectoptions,oo_has_vmt);
              end;
          end;
      end;


   procedure tobjectdef.insertvmt;
     begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
=======
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol] then
>>>>>>> graemeg/cpstrnew
=======
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol] then
>>>>>>> origin/cpstrnew
          exit;
        if (oo_has_vmt in objectoptions) then
          internalerror(12345)
        else
          begin
             tObjectSymtable(symtable).datasize:=align(tObjectSymtable(symtable).datasize,
                 tObjectSymtable(symtable).fieldalignment);

<<<<<<< HEAD
             if (tf_requires_proper_alignment in target_info.flags) then
               begin
                 { Align VMT pointer and whole object instance if target CPU requires alignment. }
                 tObjectSymtable(symtable).datasize:=align(tObjectSymtable(symtable).datasize,sizeof(pint));
                 tObjectSymtable(symtable).alignrecord(tObjectSymtable(symtable).datasize,sizeof(pint));
               end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             vmt_field:=cfieldvarsym.create('_vptr$'+objname^,vs_value,voidpointertype,[],true);
             hidesym(vmt_field);
             tObjectSymtable(symtable).insert(vmt_field);
             tObjectSymtable(symtable).addfield(tfieldvarsym(vmt_field),vis_hidden);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             vs:=tfieldvarsym.create('_vptr$'+objname^,vs_value,voidpointertype,[]);
             hidesym(vs);
             tObjectSymtable(symtable).insert(vs);
             tObjectSymtable(symtable).addfield(vs,vis_hidden);
             if (tObjectSymtable(symtable).usefieldalignment<>bit_alignment) then
               vmt_offset:=vs.fieldoffset
             else
               vmt_offset:=vs.fieldoffset div 8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             include(objectoptions,oo_has_vmt);
          end;
     end;
=======
      begin
        { outdated gcc 2.x name mangling scheme }
{$ifdef NAMEMANGLING_GCC2}

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
>>>>>>> graemeg/fixes_2_2


   function tobjectdef.vmt_offset: asizeint;
     begin
        if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
          result:=0
        else if (tObjectSymtable(symtable).usefieldalignment<>bit_alignment) then
          result:=tfieldvarsym(vmt_field).fieldoffset
        else
          result:=tfieldvarsym(vmt_field).fieldoffset div 8;
     end;


<<<<<<< HEAD
   procedure tobjectdef.check_forwards;
     begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if not(objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcprotocol,odt_interfacejava]) then
=======
        if not(objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcprotocol]) then
>>>>>>> graemeg/cpstrnew
=======
        if not(objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcprotocol]) then
>>>>>>> graemeg/cpstrnew
=======
        if not(objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcprotocol]) then
>>>>>>> graemeg/cpstrnew
=======
        if not(objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcprotocol]) then
>>>>>>> origin/cpstrnew
          inherited;
        if (oo_is_forward in objectoptions) then
          begin
             { ok, in future, the forward can be resolved }
             Message1(sym_e_class_forward_not_resolved,objrealname^);
             exclude(objectoptions,oo_is_forward);
          end;
     end;
=======
         { now we handle the parameters }
         if maxparacount>0 then
           begin
             for i:=0 to paras.count-1 do
               begin
                 hp:=tparavarsym(paras[i]);
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

         { gcc 3.x name mangling scheme }
         if procsym.owner.symtabletype=ObjectSymtable then
           begin
             s:='_ZN';

             s2:=tobjectdef(procsym.owner.defowner).objrealname^;
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
                 s2:=getcppparaname(hp.vardef);
                 if hp.varspez in [vs_var,vs_out] then
                   s2:='R'+s2;
                 s:=s+s2;
               end;
           end
         else
           s:=s+'v';
         cplusplusmangledname:=s;
      end;
>>>>>>> graemeg/fixes_2_2


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   { true if prot implements d (or if they are equal) }
   function is_related_protocol(prot: tobjectdef; d : tdef) : boolean;
     var
       i  : longint;
     begin
       { objcprotocols have multiple inheritance, all protocols from which
         the current protocol inherits are stored in implementedinterfaces }
       result:=prot=d;
       if result then
         exit;

       for i:=0 to prot.ImplementedInterfaces.count-1 do
         begin
           result:=is_related_protocol(TImplementedInterface(prot.ImplementedInterfaces[i]).intfdef,d);
           if result then
             exit;
         end;
     end;


   { true, if self inherits from d (or if they are equal) }
   function tobjectdef.is_related(d : tdef) : boolean;
     var
        hp : tobjectdef;
     begin
        if self=d then
          begin
            is_related:=true;
            exit;
          end;

        if (d.typ<>objectdef) then
          begin
            is_related:=false;
            exit;
          end;

        { Objective-C protocols can use multiple inheritance }
        if (objecttype=odt_objcprotocol) then
          begin
            is_related:=is_related_protocol(self,d);
            exit
          end;

        { formally declared Objective-C classes match Objective-C classes with
          the same name }
        if (objecttype=odt_objcclass) and
           (tobjectdef(d).objecttype=odt_objcclass) and
           ((oo_is_formal in objectoptions) or
            (oo_is_formal in tobjectdef(d).objectoptions)) and
           (objrealname^=tobjectdef(d).objrealname^) then
          begin
            is_related:=true;
            exit;
          end;

        hp:=childof;
        while assigned(hp) do
          begin
             if hp=d then
               begin
                  is_related:=true;
                  exit;
               end;
             hp:=hp.childof;
          end;
        is_related:=false;
     end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_helper,odt_javaclass,odt_interfacejava] then
          result:=voidpointertype.size
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol] then
          result:=sizeof(pint)
>>>>>>> graemeg/cpstrnew
        else
          result:=tObjectSymtable(symtable).datasize;
      end;


    function tobjectdef.alignment:shortint;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_helper,odt_javaclass,odt_interfacejava] then
          alignment:=voidpointertype.alignment
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol] then
          alignment:=sizeof(pint)
>>>>>>> graemeg/cpstrnew
        else
          alignment:=tObjectSymtable(symtable).recordalignment;
      end;


    function tobjectdef.vmtmethodoffset(index:longint):longint;
      begin
        { for offset of methods for classes, see rtl/inc/objpash.inc }
        case objecttype of
        odt_class:
          { the +2*sizeof(pint) is size and -size }
<<<<<<< HEAD
          vmtmethodoffset:=index*voidcodepointertype.size+10*voidpointertype.size+2*sizeof(pint);
        odt_helper,
=======
          vmtmethodoffset:=(index+10)*sizeof(pint)+2*sizeof(pint);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        odt_objcclass,
        odt_objcprotocol:
          vmtmethodoffset:=0;
        odt_interfacecom,odt_interfacecorba,odt_dispinterface:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          vmtmethodoffset:=index*voidcodepointertype.size;
        odt_javaclass,
        odt_interfacejava:
          { invalid }
          vmtmethodoffset:=-1;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          vmtmethodoffset:=index*sizeof(pint);
>>>>>>> graemeg/cpstrnew
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
        vmttypesym: tsym;
      begin
        vmttypesym:=tsym(get_top_level_symtable.Find('vmtdef$'+mangledparaname));
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            odt_helper,
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
              needs_inittable:=
                tObjectSymtable(symtable).needs_init_final or
                (assigned(childof) and
                 childof.needs_inittable);
            odt_cppclass,
            odt_objcclass,
            odt_objcprotocol,
            odt_javaclass,
            odt_interfacejava:
=======
              needs_inittable:=tObjectSymtable(symtable).needs_init_final;
            odt_cppclass,
            odt_objcclass,
            odt_objcprotocol:
>>>>>>> graemeg/cpstrnew
              needs_inittable:=false;
            else
              internalerror(200108267);
         end;
      end;

<<<<<<< HEAD
    function tobjectdef.needs_separate_initrtti : boolean;
=======

    function tobjectdef.rtti_mangledname(rt: trttitype): string;
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
                end;
              end;
            result:=result+objextname^;
          end;
      end;


    function tobjectdef.members_need_inittable : boolean;
>>>>>>> graemeg/cpstrnew
      begin
        result:=not (objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface]);
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

<<<<<<< HEAD
=======
         if df_copied_def in defoptions then
           ppufile.getderef(cloneddefderef)
         else
           tObjectSymtable(symtable).ppuload(ppufile);
>>>>>>> graemeg/fixes_2_2

<<<<<<< HEAD
    function tobjectdef.members_need_inittable : boolean;
      begin
        members_need_inittable:=tObjectSymtable(symtable).needs_init_final;
      end;
=======
         if df_copied_def in defoptions then
           ppufile.getderef(cloneddefderef)
         else
           tObjectSymtable(symtable).ppuload(ppufile);
>>>>>>> origin/fixes_2_2


<<<<<<< HEAD
<<<<<<< HEAD
    function tobjectdef.is_publishable : boolean;
=======
=======

>>>>>>> origin/fixes_2_2
    destructor tobjectdef.destroy;
>>>>>>> graemeg/fixes_2_2
      begin
<<<<<<< HEAD
         is_publishable:=objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface];
      end;


    function tobjectdef.get_next_dispid: longint;
<<<<<<< HEAD
=======
         case objecttype of
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            odt_class :
              needs_inittable:=false;
            odt_dispinterface,
            odt_interfacecom:
              needs_inittable:=true;
            odt_interfacecorba:
              needs_inittable:=is_related(interface_iunknown);
            odt_object:
              needs_inittable:=tObjectSymtable(symtable).needs_init_final;
            odt_cppclass,
            odt_objcclass,
            odt_objcprotocol:
              needs_inittable:=false;
            else
              internalerror(200108267);
         end;
      end;


    function tobjectdef.rtti_mangledname(rt: trttitype): string;
      begin
<<<<<<< HEAD
        if not(objecttype in [odt_objcclass,odt_objcprotocol]) then
          result:=inherited rtti_mangledname(rt)
        else
=======
        result:=tobjectdef.create(objecttype,objname^,childof);
        { the constructor allocates a symtable which we release to avoid memory leaks }
        tobjectdef(result).symtable.free;
        tobjectdef(result).symtable:=symtable.getcopy;
        if assigned(objname) then
          tobjectdef(result).objname:=stringdup(objname^);
        if assigned(objrealname) then
          tobjectdef(result).objrealname:=stringdup(objrealname^);
        tobjectdef(result).objectoptions:=objectoptions;
        include(tobjectdef(result).defoptions,df_copied_def);
        tobjectdef(result).vmt_offset:=vmt_offset;
        if assigned(iidguid) then
          begin
            new(tobjectdef(result).iidguid);
            move(iidguid^,tobjectdef(result).iidguid^,sizeof(iidguid^));
          end;
        if assigned(iidstr) then
          tobjectdef(result).iidstr:=stringdup(iidstr^);
        if assigned(ImplementedInterfaces) then
          begin
            for i:=0 to ImplementedInterfaces.count-1 do
              tobjectdef(result).ImplementedInterfaces.Add(TImplementedInterface(ImplementedInterfaces[i]).Getcopy);
          end;
        if assigned(vmtentries) then
>>>>>>> graemeg/fixes_2_2
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
                end;
              end;
            result:=result+objextname^;
          end;
      end;


<<<<<<< HEAD
    function tobjectdef.rtti_mangledname(rt: trttitype): string;
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
                end;
              end;
            result:=result+objextname^;
          end;
      end;

<<<<<<< HEAD

<<<<<<< HEAD
    function tobjectdef.members_need_inittable : boolean;
>>>>>>> graemeg/cpstrnew
      begin
        inc(fcurrent_dispid);
        result:=fcurrent_dispid;
=======
         if df_copied_def in defoptions then
           ppufile.putderef(cloneddefderef);
=======
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
         ppufile.putstring(objrealname^);
         ppufile.putaint(tObjectSymtable(symtable).datasize);
         ppufile.putbyte(tObjectSymtable(symtable).fieldalignment);
         ppufile.putbyte(tObjectSymtable(symtable).recordalignment);
         ppufile.putlongint(vmt_offset);
         ppufile.putderef(childofderef);
         ppufile.putsmallset(objectoptions);
         if objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface] then
           begin
              ppufile.putguid(iidguid^);
              ppufile.putstring(iidstr^);
           end;

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
                 ppufile.putlongint(ImplIntf.Ioffset);
               end;
           end;
>>>>>>> origin/fixes_2.4

         if df_copied_def in defoptions then
           ppufile.putderef(cloneddefderef);

         ppufile.writeentry(ibobjectdef);

         if not(df_copied_def in defoptions) then
           tObjectSymtable(symtable).ppuwrite(ppufile);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

         ppufile.do_indirect_crc:=old_do_indirect_crc;
>>>>>>> origin/fixes_2.4
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

=======
=======
    function tobjectdef.members_need_inittable : boolean;
>>>>>>> origin/cpstrnew
      begin
<<<<<<< HEAD
        inc(fcurrent_dispid);
        result:=fcurrent_dispid;
=======
         inherited buildderef;
         childofderef.build(childof);
         if df_copied_def in defoptions then
           cloneddefderef.build(symtable.defowner)
         else
           tstoredsymtable(symtable).buildderef;

         if objecttype in [odt_class,odt_interfacecorba] then
           begin
             for i:=0 to ImplementedInterfaces.count-1 do
               TImplementedInterface(ImplementedInterfaces[i]).buildderef;
           end;
>>>>>>> graemeg/fixes_2_2
      end;

    function tobjectdef.search_enumerator_get: tprocdef;
      begin
        result:=inherited;
        if not assigned(result) and assigned(childof) then
          result:=childof.search_enumerator_get;
      end;

<<<<<<< HEAD
=======
    function tobjectdef.get_next_dispid: longint;
      begin
<<<<<<< HEAD
        inc(fcurrent_dispid);
        result:=fcurrent_dispid;
=======
         inherited deref;
         childof:=tobjectdef(childofderef.resolve);
         if df_copied_def in defoptions then
           begin
             cloneddef:=tobjectdef(cloneddefderef.resolve);
             symtable:=cloneddef.symtable.getcopy;
           end
         else
           tstoredsymtable(symtable).deref;
         if objecttype in [odt_class,odt_interfacecorba] then
           begin
             for i:=0 to ImplementedInterfaces.count-1 do
               TImplementedInterface(ImplementedInterfaces[i]).deref;
           end;
>>>>>>> graemeg/fixes_2_2
      end;

    function tobjectdef.search_enumerator_get: tprocdef;
      begin
        result:=inherited;
        if not assigned(result) and assigned(childof) then
          result:=childof.search_enumerator_get;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    function tobjectdef.get_next_dispid: longint;
=======
    procedure tobjectdef.buildderefimpl;
      var
         i : longint;
      begin
         inherited buildderefimpl;
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).buildderefimpl;
      end;


    procedure tobjectdef.derefimpl;
      begin
         inherited derefimpl;
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).derefimpl;
      end;


    procedure tobjectdef.buildderefimpl;
      var
         i : longint;
      begin
         inherited buildderefimpl;
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).buildderefimpl;
      end;


    procedure tobjectdef.derefimpl;
      begin
         inherited derefimpl;
         if not (df_copied_def in defoptions) then
           tstoredsymtable(symtable).derefimpl;
      end;


    function tobjectdef.getparentdef:tdef;
>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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

>>>>>>> origin/cpstrnew
    function tobjectdef.search_enumerator_move: tprocdef;
      begin
<<<<<<< HEAD
        result:=inherited;
        if not assigned(result) and assigned(childof) then
          result:=childof.search_enumerator_move;
=======
        { nothing to do if the parent was not forward !}
        if assigned(childof) then
          exit;
        childof:=c;
        { some options are inherited !! }
        if assigned(c) then
          begin
             { only important for classes }
             objectoptions:=objectoptions+(c.objectoptions*inherited_objectoptions);
             if not (objecttype in [odt_interfacecom,odt_interfacecorba,odt_dispinterface]) then
               begin
                  { add the data of the anchestor class }
                  tObjectSymtable(symtable).datasize:=
                    tObjectSymtable(symtable).datasize+
                    tObjectSymtable(c.symtable).datasize;
                  { inherit recordalignment }
                  tObjectSymtable(symtable).recordalignment:=tObjectSymtable(c.symtable).recordalignment;
                  if (oo_has_vmt in objectoptions) and
                     (oo_has_vmt in c.objectoptions) then
                    tObjectSymtable(symtable).datasize:=
                      tObjectSymtable(symtable).datasize-sizeof(aint);
                  { if parent has a vmt field then
                    the offset is the same for the child PM }
                  if (oo_has_vmt in c.objectoptions) or is_class(self) then
                    begin
                       vmt_offset:=c.vmt_offset;
                       include(objectoptions,oo_has_vmt);
                    end;
               end;
          end;
>>>>>>> graemeg/fixes_2_2
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
            classref_created_in_current_module:=true;
            current_module.wpoinfo.addcreatedobjtypeforclassref(self);
=======
             tObjectSymtable(symtable).datasize:=align(tObjectSymtable(symtable).datasize,
                 tObjectSymtable(symtable).fieldalignment);

             if (tf_requires_proper_alignment in target_info.flags) then
               tObjectSymtable(symtable).datasize:=align(tObjectSymtable(symtable).datasize,sizeof(aint));

             vmt_offset:=tObjectSymtable(symtable).datasize;
             tObjectSymtable(symtable).datasize:=
               tObjectSymtable(symtable).datasize+sizeof(aint);
             include(objectoptions,oo_has_vmt);
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
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
=======
         case objecttype of
            odt_dispinterface,
            odt_class :
              needs_inittable:=false;
            odt_interfacecom:
              needs_inittable:=true;
            odt_interfacecorba:
              needs_inittable:=is_related(interface_iunknown);
            odt_object:
              needs_inittable:=tObjectSymtable(symtable).needs_init_final;
            odt_cppclass:
              needs_inittable:=false;
            else
              internalerror(200108267);
         end;
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   current_module.AddExternalImport(tobjectdef(pd.struct).import_lib^,pd.mangledname,pd.mangledname,0,false,false)
=======
                   current_module.AddExternalImport(tobjectdef(pd.struct).import_lib^,pd.mangledname,0,false,false)
>>>>>>> graemeg/cpstrnew
=======
                   current_module.AddExternalImport(tobjectdef(pd.struct).import_lib^,pd.mangledname,0,false,false)
>>>>>>> graemeg/cpstrnew
=======
                   current_module.AddExternalImport(tobjectdef(pd.struct).import_lib^,pd.mangledname,0,false,false)
>>>>>>> graemeg/cpstrnew
=======
                   current_module.AddExternalImport(tobjectdef(pd.struct).import_lib^,pd.mangledname,0,false,false)
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
        if (IType<>etStandard) or (MergingIntf.IType<>etStandard) or not(assigned(ProcDefs)) or not(assigned(MergingIntf.ProcDefs)) then
=======
        if (IType<>etStandard) or (MergingIntf.IType<>etStandard) or not(assigned(ProcDefs)) then
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
        {$warning: this is completely wrong on so many levels...}
>>>>>>> graemeg/cpstrnew
=======
        {$warning: this is completely wrong on so many levels...}
>>>>>>> graemeg/cpstrnew
=======
        {$warning: this is completely wrong on so many levels...}
>>>>>>> graemeg/cpstrnew
=======
        {$warning: this is completely wrong on so many levels...}
>>>>>>> origin/cpstrnew
        { 1) the procdefs list will be freed once for each copy
          2) since the procdefs list owns its elements, those will also be freed for each copy
          3) idem for the name mappings
        }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        { warning: this is completely wrong on so many levels...
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        result:=cforwarddef.create(tosymname^, forwardpos);
=======
        result:=tforwarddef.create(tosymname^, forwardpos);
>>>>>>> graemeg/cpstrnew
=======
        result:=tforwarddef.create(tosymname^, forwardpos);
>>>>>>> graemeg/cpstrnew
=======
        result:=tforwarddef.create(tosymname^, forwardpos);
>>>>>>> graemeg/cpstrnew
=======
        result:=tforwarddef.create(tosymname^, forwardpos);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
        inherited create(errordef,true);
=======
        inherited create(errordef);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function is_any_interface_kind(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          ((tobjectdef(def).objecttype in [odt_interfacecom,odt_dispinterface,odt_interfacecorba,odt_interfacejava,odt_objcprotocol]) or
           is_objccategory(def));
      end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
<<<<<<< HEAD
    function is_objcclass(def: tdef): boolean;
      begin
        is_objcclass:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_objcclass);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function is_objectpascal_helper(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype=odt_helper);
      end;


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function is_classhelper(def: tdef): boolean;
      begin
         result:=
           is_objectpascal_helper(def) or
           is_objccategory(def);
      end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function is_class_or_interface(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba]);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    function is_class_or_interface_or_objc(def: tdef): boolean;
      begin
        result:=
          assigned(def) and
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_objcclass,odt_objcprotocol]);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          (((def.typ=objectdef) and
            (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol,odt_helper,odt_javaclass,odt_interfacejava])) or
           ((target_info.system in systems_jvm) and
            (def.typ=recorddef)));
      end;

    function is_implicit_array_pointer(def: tdef): boolean;
      begin
        result:=is_dynamic_array(def) or is_dynamicstring(def);
=======
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol]);
>>>>>>> graemeg/cpstrnew
=======
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol]);
>>>>>>> graemeg/cpstrnew
=======
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol]);
>>>>>>> graemeg/cpstrnew
=======
          (def.typ=objectdef) and
          (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_dispinterface,odt_objcclass,odt_objcprotocol]);
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/cpstrnew
    procedure loadobjctypes;
      begin
        objc_metaclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_CLASS',true).typedef);
        objc_superclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_SUPER',true).typedef);
        objc_idtype:=tpointerdef(search_named_unit_globaltype('OBJC','ID',true).typedef);
        objc_seltype:=tpointerdef(search_named_unit_globaltype('OBJC','SEL',true).typedef);
        objc_objecttype:=trecorddef(search_named_unit_globaltype('OBJC','OBJC_OBJECT',true).typedef);
<<<<<<< HEAD
      end;

=======
=======
>>>>>>> origin/cpstrnew
    procedure loadobjctypes;
      begin
        objc_metaclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_CLASS',true).typedef);
        objc_superclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_SUPER',true).typedef);
        objc_idtype:=tpointerdef(search_named_unit_globaltype('OBJC','ID',true).typedef);
        objc_seltype:=tpointerdef(search_named_unit_globaltype('OBJC','SEL',true).typedef);
        objc_objecttype:=trecorddef(search_named_unit_globaltype('OBJC','OBJC_OBJECT',true).typedef);
<<<<<<< HEAD
      end;

>>>>>>> graemeg/cpstrnew
=======
    procedure loadobjctypes;
      begin
        objc_metaclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_CLASS',true).typedef);
        objc_superclasstype:=tpointerdef(search_named_unit_globaltype('OBJC','POBJC_SUPER',true).typedef);
        objc_idtype:=tpointerdef(search_named_unit_globaltype('OBJC','ID',true).typedef);
        objc_seltype:=tpointerdef(search_named_unit_globaltype('OBJC','SEL',true).typedef);
        objc_objecttype:=trecorddef(search_named_unit_globaltype('OBJC','OBJC_OBJECT',true).typedef);
      end;

>>>>>>> graemeg/cpstrnew

    procedure maybeloadcocoatypes;
      var
        tsym: ttypesym;
<<<<<<< HEAD
<<<<<<< HEAD
        cocoaunit: string[15];
      begin
        if assigned(objc_fastenumeration) then
          exit;
        if not(target_info.system in [system_arm_darwin,system_i386_iphonesim,system_aarch64_darwin,system_x86_64_iphonesim]) then
          cocoaunit:='COCOAALL'
        else
          cocoaunit:='IPHONEALL';
        tsym:=search_named_unit_globaltype(cocoaunit,'NSFASTENUMERATIONPROTOCOL',false);
=======
=======
>>>>>>> graemeg/cpstrnew
      begin
        if assigned(objc_fastenumeration) then
          exit;
        tsym:=search_named_unit_globaltype('COCOAALL','NSFASTENUMERATIONPROTOCOL',false);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
        if assigned(tsym) then
          objc_fastenumeration:=tobjectdef(tsym.typedef)
        else
          objc_fastenumeration:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
        tsym:=search_named_unit_globaltype(cocoaunit,'NSFASTENUMERATIONSTATE',false);
        if assigned(tsym) then
          objc_fastenumerationstate:=trecorddef(tsym.typedef)
        else
          objc_fastenumerationstate:=nil;
=======
=======
>>>>>>> graemeg/cpstrnew
        tsym:=search_named_unit_globaltype('COCOAALL','NSFASTENUMERATIONSTATE',false);
        if assigned(tsym) then
          objc_fastenumerationstate:=trecorddef(tsym.typedef)
        else
        objc_fastenumerationstate:=nil;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
      end;


=======
=======
>>>>>>> origin/cpstrnew
      end;


    procedure maybeloadcocoatypes;
      var
        tsym: ttypesym;
      begin
        if assigned(objc_fastenumeration) then
          exit;
        tsym:=search_named_unit_globaltype('COCOAALL','NSFASTENUMERATIONPROTOCOL',false);
        if assigned(tsym) then
          objc_fastenumeration:=tobjectdef(tsym.typedef)
        else
          objc_fastenumeration:=nil;
        tsym:=search_named_unit_globaltype('COCOAALL','NSFASTENUMERATIONSTATE',false);
        if assigned(tsym) then
          objc_fastenumerationstate:=trecorddef(tsym.typedef)
        else
        objc_fastenumerationstate:=nil;
      end;


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function use_vectorfpu(def : tdef) : boolean;
      begin
{$ifdef x86}
{$define use_vectorfpuimplemented}
        use_vectorfpu:=(is_single(def) and (current_settings.fputype in sse_singlescalar)) or
          (is_double(def) and (current_settings.fputype in sse_doublescalar));
{$endif x86}
{$ifdef arm}
{$define use_vectorfpuimplemented}
        use_vectorfpu:=(current_settings.fputype in vfp_scalar);
{$endif arm}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef aarch64}
{$define use_vectorfpuimplemented}
        use_vectorfpu:=true;
{$endif aarch64}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef use_vectorfpuimplemented}
        use_vectorfpu:=false;
{$endif}
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

end.
