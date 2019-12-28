{
    Copyright (c) 2011 by Jonas Maebe

    This unit implements some JVM parser helper routines.

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

{$i fpcdefs.inc}

unit pjvm;

interface

    uses
      globtype,
      symconst,symtype,symbase,symdef,symsym;

    { records are emulated via Java classes. They require a default constructor
      to initialise temps, a deep copy helper for assignments, and clone()
      to initialse dynamic arrays }
    procedure add_java_default_record_methods_intf(def: trecorddef);

    procedure jvm_maybe_create_enum_class(const name: TIDString; def: tdef);
    procedure jvm_create_procvar_class(const name: TIDString; def: tdef);

    procedure jvm_wrap_virtual_class_methods(obj: tobjectdef);

    function jvm_add_typed_const_initializer(csym: tconstsym): tstaticvarsym;

    function jvm_wrap_method_with_vis(pd: tprocdef; vis: tvisibility): tprocdef;


implementation

  uses
    cutils,cclasses,
    verbose,globals,systems,
    fmodule,
    parabase,aasmdata,
    ngenutil,pparautl,
    symtable,symcreat,defcmp,jvmdef,symcpu,nobj,
    defutil,paramgr;


    procedure add_java_default_record_methods_intf(def: trecorddef);
      var
        sstate: tscannerstate;
        pd: tprocdef;
        sym: tsym;
        i: longint;
      begin
        maybe_add_public_default_java_constructor(def);
        replace_scanner('record_jvm_helpers',sstate);
        { no override, because not supported in records. Only required in case
          some of the fields require deep copies (otherwise the default
          shallow clone is fine) }
        for i:=0 to def.symtable.symlist.count-1 do
          begin
            sym:=tsym(def.symtable.symlist[i]);
            if (sym.typ=fieldvarsym) and
               jvmimplicitpointertype(tfieldvarsym(sym).vardef) then
              begin
                if str_parse_method_dec('function clone: JLObject;',potype_function,false,def,pd) then
                  pd.synthetickind:=tsk_jvm_clone
                else
                  internalerror(2011032806);
                break;
              end;
          end;
        { can't use def.typesym, not yet set at this point }
        if not assigned(def.symtable.realname) then
          internalerror(2011032803);
        if str_parse_method_dec('procedure fpcDeepCopy(result: FpcBaseRecordType);',potype_procedure,false,def,pd) then
          begin
            pd.synthetickind:=tsk_record_deepcopy;
            { can't add to the declaration since record methods can't override;
              it is in fact an overriding method, because all records inherit
              from a Java base class }
            include(pd.procoptions,po_overridingmethod);
          end
        else
          internalerror(2011032807);
        if def.needs_inittable then
          begin
            { 'var' instead of 'out' parameter, because 'out' would trigger
               calling the initialize method recursively }
            if str_parse_method_dec('procedure fpcInitializeRec;',potype_procedure,false,def,pd) then
              pd.synthetickind:=tsk_record_initialize
            else
              internalerror(2011071711);
          end;
        restore_scanner(sstate);
      end;


    procedure setup_for_new_class(const scannername: string; out sstate: tscannerstate; out islocal: boolean; out oldsymtablestack: TSymtablestack);
      begin
        replace_scanner(scannername,sstate);
        oldsymtablestack:=symtablestack;
        islocal:=symtablestack.top.symtablelevel>=normal_function_level;
        if islocal then
          begin
            { we cannot add a class local to a procedure -> insert it in the
              static symtable. This is not ideal because this means that it will
              be saved to the ppu file for no good reason, and loaded again
              even though it contains a reference to a type that was never
              saved to the ppu file (the locally defined enum type). Since this
              alias for the locally defined enumtype is only used while
              implementing the class' methods, this is however no problem. }
            symtablestack:=symtablestack.getcopyuntil(current_module.localsymtable);
          end;
      end;


    procedure restore_after_new_class(const sstate: tscannerstate; const islocal: boolean; const oldsymtablestack: TSymtablestack);
      begin
        if islocal then
          begin
            symtablestack.free;
            symtablestack:=oldsymtablestack;
          end;
        restore_scanner(sstate);
      end;


    procedure jvm_maybe_create_enum_class(const name: TIDString; def: tdef);
      var
        arrdef: tarraydef;
        arrsym: ttypesym;
        juhashmap: tdef;
        enumclass: tobjectdef;
        pd: tprocdef;
        old_current_structdef: tabstractrecorddef;
        i: longint;
        sym,
        aliassym: tstaticvarsym;
        fsym: tfieldvarsym;
        sstate: tscannerstate;
        sl: tpropaccesslist;
        temptypesym: ttypesym;
        oldsymtablestack: tsymtablestack;
        islocal: boolean;
      begin
        { if it's a subrange type, don't create a new class }
        if assigned(tenumdef(def).basedef) then
          exit;

        setup_for_new_class('jvm_enum_class',sstate,islocal,oldsymtablestack);

        { create new class (different internal name than enum to prevent name
          clash; at unit level because we don't want its methods to be nested
          inside a function in case its a local type) }
        enumclass:=cobjectdef.create(odt_javaclass,'$'+current_module.realmodulename^+'$'+name+'$InternEnum$'+def.unique_id_str,java_jlenum,true);
        tcpuenumdef(def).classdef:=enumclass;
        include(enumclass.objectoptions,oo_is_enum_class);
        include(enumclass.objectoptions,oo_is_sealed);
        { implement FpcEnumValueObtainable interface }
        enumclass.ImplementedInterfaces.add(TImplementedInterface.Create(tobjectdef(search_system_type('FPCENUMVALUEOBTAINABLE').typedef)));
        { create an alias for this type inside itself: this way we can choose a
          name that can be used in generated Pascal code without risking an
          identifier conflict (since it is local to this class; the global name
          is unique because it's an identifier that contains $-signs) }
        enumclass.symtable.insert(ctypesym.create('__FPC_TEnumClassAlias',enumclass));

        { also create an alias for the enum type so that we can iterate over
          all enum values when creating the body of the class constructor }
        temptypesym:=ctypesym.create('__FPC_TEnumAlias',nil);
        { don't pass def to the ttypesym constructor, because then it
          will replace the current (real) typesym of that def with the alias }
        temptypesym.typedef:=def;
        enumclass.symtable.insert(temptypesym);
        { but the name of the class as far as the JVM is concerned will match
          the enum's original name (the enum type itself won't be output in
          any class file, so no conflict there)

          name can be empty in case of declaration such as "set of (ea,eb)"  }
        if not islocal and
           (name <> '')  then
          enumclass.objextname:=stringdup(name)
        else
          { for local types, use a unique name to prevent conflicts (since such
            types are not visible outside the routine anyway, it doesn't matter
          }
          begin
            enumclass.objextname:=stringdup(enumclass.objrealname^);
            { also mark it as private (not strict private, because the class
              is not a subclass of the unit in which it is declared, so then
              the unit's procedures would not be able to use it) }
            enumclass.typesym.visibility:=vis_private;
          end;
        { now add a bunch of extra things to the enum class }
        old_current_structdef:=current_structdef;
        current_structdef:=enumclass;

        symtablestack.push(enumclass.symtable);
        { create static fields representing all enums }
        for i:=0 to tenumdef(def).symtable.symlist.count-1 do
          begin
            fsym:=cfieldvarsym.create(tenumsym(tenumdef(def).symtable.symlist[i]).realname,vs_final,enumclass,[]);
            enumclass.symtable.insert(fsym);
            sym:=make_field_static(enumclass.symtable,fsym);
            { add alias for the field representing ordinal(0), for use in
              initialization code }
            if tenumsym(tenumdef(def).symtable.symlist[i]).value=0 then
              begin
                aliassym:=cstaticvarsym.create('__FPC_Zero_Initializer',vs_final,enumclass,[vo_is_external]);
                enumclass.symtable.insert(aliassym);
                aliassym.set_raw_mangledname(sym.mangledname);
              end;
          end;
        { create local "array of enumtype" type for the "values" functionality
          (used internally by the JDK) }
        arrdef:=carraydef.create(0,tenumdef(def).symtable.symlist.count-1,s32inttype);
        arrdef.elementdef:=enumclass;
        arrsym:=ctypesym.create('__FPC_TEnumValues',arrdef);
        enumclass.symtable.insert(arrsym);
        { insert "public static values: array of enumclass" that returns $VALUES.clone()
          (rather than a dynamic array and using clone --which we don't support yet for arrays--
           simply use a fixed length array and copy it) }
        if not str_parse_method_dec('function values: __FPC_TEnumValues;static;',potype_function,true,enumclass,pd) then
          internalerror(2011062301);
        include(pd.procoptions,po_staticmethod);
        pd.synthetickind:=tsk_jvm_enum_values;
        { do we have to store the ordinal value separately? (if no jumps, we can
          just call the default ordinal() java.lang.Enum function) }
        if tenumdef(def).has_jumps then
          begin
            { add field for the value }
            fsym:=cfieldvarsym.create('__fpc_fenumval',vs_final,s32inttype,[]);
            enumclass.symtable.insert(fsym);
            tobjectsymtable(enumclass.symtable).addfield(fsym,vis_strictprivate);
            { add class field with hash table that maps from FPC-declared ordinal value -> enum instance }
            juhashmap:=search_system_type('JUHASHMAP').typedef;
            fsym:=cfieldvarsym.create('__fpc_ord2enum',vs_final,juhashmap,[]);
            enumclass.symtable.insert(fsym);
            make_field_static(enumclass.symtable,fsym);
            { add custom constructor }
            if not str_parse_method_dec('constructor Create(const __fpc_name: JLString; const __fpc_ord, __fpc_initenumval: longint);',potype_constructor,false,enumclass,pd) then
              internalerror(2011062401);
            pd.synthetickind:=tsk_jvm_enum_jumps_constr;
            pd.visibility:=vis_strictprivate;
          end
        else
          begin
            { insert "private constructor(string,int,int)" that calls inherited and
              initialises the FPC value field }
            add_missing_parent_constructors_intf(enumclass,false,vis_strictprivate);
          end;
        { add instance method to get the enum's value as declared in FPC }
        if not str_parse_method_dec('function FPCOrdinal: longint;',potype_function,false,enumclass,pd) then
          internalerror(2011062402);
        pd.synthetickind:=tsk_jvm_enum_fpcordinal;
        { add static class method to convert an ordinal to the corresponding enum }
        if not str_parse_method_dec('function FPCValueOf(__fpc_int: longint): __FPC_TEnumClassAlias; static;',potype_function,true,enumclass,pd) then
          internalerror(2011062402);
        pd.synthetickind:=tsk_jvm_enum_fpcvalueof;
        { similar (instance) function for use in set factories; implements FpcEnumValueObtainable interface }
        if not str_parse_method_dec('function fpcGenericValueOf(__fpc_int: longint): JLEnum;',potype_function,false,enumclass,pd) then
          internalerror(2011062402);
        pd.synthetickind:=tsk_jvm_enum_fpcvalueof;

        { insert "public static valueOf(string): tenumclass" that returns tenumclass(inherited valueOf(tenumclass,string)) }
        if not str_parse_method_dec('function valueOf(const __fpc_str: JLString): __FPC_TEnumClassAlias; static;',potype_function,true,enumclass,pd) then
          internalerror(2011062302);
        include(pd.procoptions,po_staticmethod);
        pd.synthetickind:=tsk_jvm_enum_valueof;

        { add instance method to convert an ordinal and an array into a set of
          (we always need/can use both in case of subrange types and/or array
           -> set type casts) }
        if not str_parse_method_dec('function fpcLongToEnumSet(__val: jlong; __setbase, __setsize: jint): JUEnumSet;',potype_function,true,enumclass,pd) then
          internalerror(2011070501);
        pd.synthetickind:=tsk_jvm_enum_long2set;

        if not str_parse_method_dec('function fpcBitSetToEnumSet(const __val: FpcBitSet; __fromsetbase, __tosetbase: jint): JUEnumSet; static;',potype_function,true,enumclass,pd) then
          internalerror(2011071004);
        pd.synthetickind:=tsk_jvm_enum_bitset2set;

        if not str_parse_method_dec('function fpcEnumSetToEnumSet(const __val: JUEnumSet; __fromsetbase, __tosetbase: jint): JUEnumSet; static;',potype_function,true,enumclass,pd) then
          internalerror(2011071005);
        pd.synthetickind:=tsk_jvm_enum_set2set;

        { create array called "$VALUES" that will contain a reference to all
          enum instances (JDK convention)
          Disable duplicate identifier checking when inserting, because it will
          check for a conflict with "VALUES" ($<id> normally means "check for
          <id> without uppercasing first"), which will conflict with the
          "Values" instance method -- that's also the reason why we insert the
          field only now, because we cannot disable duplicate identifier
          checking when creating the "Values" method }
        fsym:=cfieldvarsym.create('$VALUES',vs_final,arrdef,[]);
        fsym.visibility:=vis_strictprivate;
        enumclass.symtable.insert(fsym,false);
        sym:=make_field_static(enumclass.symtable,fsym);
        { alias for accessing the field in generated Pascal code }
        sl:=tpropaccesslist.create;
        sl.addsym(sl_load,sym);
        enumclass.symtable.insert(cabsolutevarsym.create_ref('__fpc_FVALUES',arrdef,sl));
        { add initialization of the static class fields created above }
        if not str_parse_method_dec('constructor fpc_enum_class_constructor;',potype_class_constructor,true,enumclass,pd) then
          internalerror(2011062303);
        pd.synthetickind:=tsk_jvm_enum_classconstr;

        symtablestack.pop(enumclass.symtable);

        build_vmt(enumclass);

        restore_after_new_class(sstate,islocal,oldsymtablestack);
        current_structdef:=old_current_structdef;
      end;


    procedure jvm_create_procvar_class_intern(const name: TIDString; def: tdef; force_no_callback_intf: boolean);
      var
        oldsymtablestack: tsymtablestack;
        pvclass,
        pvintf: tobjectdef;
        temptypesym: ttypesym;
        sstate: tscannerstate;
        methoddef: tprocdef;
        old_current_structdef: tabstractrecorddef;
        islocal: boolean;
      begin
        { inlined definition of procvar -> generate name, derive from
          FpcBaseNestedProcVarType, pass nestedfpstruct to constructor and
          copy it }
        if name='' then
          begin
            if is_nested_pd(tabstractprocdef(def)) then
              internalerror(2011071901);
          end;

        setup_for_new_class('jvm_pvar_class',sstate,islocal,oldsymtablestack);

        { create new class (different internal name than pvar to prevent name
          clash; at unit level because we don't want its methods to be nested
          inside a function in case its a local type) }
        pvclass:=cobjectdef.create(odt_javaclass,'$'+current_module.realmodulename^+'$'+name+'$InternProcvar$'+def.unique_id_str,java_procvarbase,true);
        tcpuprocvardef(def).classdef:=pvclass;
        include(pvclass.objectoptions,oo_is_sealed);
        if df_generic in def.defoptions then
          include(pvclass.defoptions,df_generic);
        { associate typesym }
        pvclass.symtable.insert(ctypesym.create('__FPC_TProcVarClassAlias',pvclass));
        { set external name to match procvar type name }
        if not islocal then
          pvclass.objextname:=stringdup(name)
        else
          pvclass.objextname:=stringdup(pvclass.objrealname^);

        symtablestack.push(pvclass.symtable);

        { inherit constructor and keep public }
        add_missing_parent_constructors_intf(pvclass,true,vis_public);

        { add a method to call the procvar using unwrapped arguments, which
          then wraps them and calls through to JLRMethod.invoke }
        methoddef:=tprocdef(tprocvardef(def).getcopyas(procdef,pc_bareproc,''));
        finish_copied_procdef(methoddef,'invoke',pvclass.symtable,pvclass);
        methoddef.synthetickind:=tsk_jvm_procvar_invoke;
        methoddef.calcparas;

        { add local alias for the procvartype that we can use when implementing
          the invoke method }
        temptypesym:=ctypesym.create('__FPC_ProcVarAlias',nil);
        { don't pass def to the ttypesym constructor, because then it
          will replace the current (real) typesym of that def with the alias }
        temptypesym.typedef:=def;
        pvclass.symtable.insert(temptypesym);

        { in case of a procedure of object, add a nested interface type that
          has one method that conforms to the procvartype (with name
          procvartypename+'Callback') and an extra constructor that takes
          an instance conforming to this interface and which sets up the
          procvar by taking the address of its Callback method (convenient to
          use from Java code) }
        if (po_methodpointer in tprocvardef(def).procoptions) and
           not islocal and
           not force_no_callback_intf then
          begin
            pvintf:=cobjectdef.create(odt_interfacejava,'Callback',nil,true);
            pvintf.objextname:=stringdup('Callback');
            if df_generic in def.defoptions then
              include(pvintf.defoptions,df_generic);
            { associate typesym }
            pvclass.symtable.insert(ctypesym.create('Callback',pvintf));

            { add a method prototype matching the procvar (like the invoke
              in the procvarclass itself) }
            symtablestack.push(pvintf.symtable);
            methoddef:=tprocdef(tprocvardef(def).getcopyas(procdef,pc_bareproc,''));
            finish_copied_procdef(methoddef,name+'Callback',pvintf.symtable,pvintf);
            { can't be final/static/private/protected, and must be virtual
              since it's an interface method }
            methoddef.procoptions:=methoddef.procoptions-[po_staticmethod,po_finalmethod];
            include(methoddef.procoptions,po_virtualmethod);
            methoddef.visibility:=vis_public;
            symtablestack.pop(pvintf.symtable);

            { add an extra constructor to the procvarclass that takes an
              instance of this interface as parameter }
            old_current_structdef:=current_structdef;
            current_structdef:=pvclass;
            if not str_parse_method_dec('constructor Create(__intf:'+pvintf.objextname^+');overload;',potype_constructor,false,pvclass,methoddef) then
              internalerror(2011092401);
            methoddef.synthetickind:=tsk_jvm_procvar_intconstr;
            methoddef.skpara:=def;
            current_structdef:=old_current_structdef;
          end;

        symtablestack.pop(pvclass.symtable);

        build_vmt(pvclass);

        restore_after_new_class(sstate,islocal,oldsymtablestack);
      end;


    procedure jvm_create_procvar_class(const name: TIDString; def: tdef);
      begin
        jvm_create_procvar_class_intern(name,def,false);
      end;


    procedure jvm_wrap_virtual_class_method(pd: tprocdef);
      var
        wrapperpd: tprocdef;
        wrapperpv: tcpuprocvardef;
        typ: ttypesym;
        wrappername: shortstring;
      begin
        if (po_external in pd.procoptions) or
           (oo_is_external in pd.struct.objectoptions) then
          exit;
        { the JVM does not support virtual class methods -> we generate
          wrappers with the original name so they can be called normally,
          and these wrappers will then perform a dynamic lookup. To enable
          calling the class method by its intended name from external Java code,
          we have to change its external name so that we give that original
          name to the wrapper function -> "switch" the external names around for
          the original and wrapper methods }

        { replace importname of original procdef }
        include(pd.procoptions,po_has_importname);
        if not assigned(pd.import_name) then
          wrappername:=pd.procsym.realname
        else
          wrappername:=pd.import_name^;
        stringdispose(pd.import_name);
        pd.import_name:=stringdup(wrappername+'__fpcvirtualclassmethod__');

        { wrapper is part of the same symtable as the original procdef }
        symtablestack.push(pd.owner);
        { get a copy of the virtual class method }
        wrapperpd:=tprocdef(pd.getcopyas(procdef,pc_normal_no_hidden,''));
        { this one is not virtual nor override }
        exclude(wrapperpd.procoptions,po_virtualmethod);
        exclude(wrapperpd.procoptions,po_overridingmethod);
        { import/external name = name of original class method }
        stringdispose(wrapperpd.import_name);
        wrapperpd.import_name:=stringdup(wrappername);
        include(wrapperpd.procoptions,po_has_importname);
        { associate with wrapper procsym (Pascal-level name = wrapper name ->
          in callnodes, we will have to replace the calls to virtual class
          methods with calls to the wrappers) }
        finish_copied_procdef(wrapperpd,pd.import_name^,pd.owner,tabstractrecorddef(pd.owner.defowner));

        { we only have to generate the dispatching routine for non-overriding
          methods; the overriding ones can use the original one, but generate
          a skeleton for that anyway because the overriding one may still
          change the visibility (but we can just call the inherited routine
          in that case) }
        if po_overridingmethod in pd.procoptions then
          begin
            { by default do not include this routine when looking for overloads }
            include(wrapperpd.procoptions,po_ignore_for_overload_resolution);
            wrapperpd.synthetickind:=tsk_anon_inherited;
            symtablestack.pop(pd.owner);
            exit;
          end;

        { implementation }
        wrapperpd.synthetickind:=tsk_jvm_virtual_clmethod;
        wrapperpd.skpara:=pd;
        { also create procvar type that we can use in the implementation }
        wrapperpv:=tcpuprocvardef(pd.getcopyas(procvardef,pc_normal_no_hidden,''));
        handle_calling_convention(wrapperpv,hcc_default_actions_intf);
        { no use in creating a callback wrapper here, this procvar type isn't
          for public consumption }
        jvm_create_procvar_class_intern('__fpc_virtualclassmethod_pv_t'+wrapperpd.unique_id_str,wrapperpv,true);
        { create alias for the procvar type so we can use it in generated
          Pascal code }
        typ:=ctypesym.create('__fpc_virtualclassmethod_pv_t'+wrapperpd.unique_id_str,wrapperpv);
        wrapperpv.classdef.typesym.visibility:=vis_strictprivate;
        symtablestack.top.insert(typ);
        symtablestack.pop(pd.owner);
      end;


    procedure jvm_wrap_virtual_constructor(pd: tprocdef);
      var
        wrapperpd: tprocdef;
      begin
        { to avoid having to implement procvar-like support for dynamically
          invoking constructors, call the constructors from virtual class
          methods and replace calls to the constructors with calls to the
          virtual class methods -> we can reuse lots of infrastructure }
        if (po_external in pd.procoptions) or
           (oo_is_external in pd.struct.objectoptions) then
          exit;
        { wrapper is part of the same symtable as the original procdef }
        symtablestack.push(pd.owner);
        { get a copy of the constructor }
        wrapperpd:=tprocdef(pd.getcopyas(procdef,pc_bareproc,''));
        { this one is a class method rather than a constructor }
        include(wrapperpd.procoptions,po_classmethod);
        wrapperpd.proctypeoption:=potype_function;
        wrapperpd.returndef:=tobjectdef(pd.owner.defowner);

        { import/external name = name of original constructor (since
          constructors don't have names in Java, this won't conflict with the
          original constructor definition) }
        stringdispose(wrapperpd.import_name);
        wrapperpd.import_name:=stringdup(pd.procsym.realname);
        { associate with wrapper procsym (Pascal-level name = wrapper name ->
          in callnodes, we will have to replace the calls to virtual
          constructors with calls to the wrappers) }
        finish_copied_procdef(wrapperpd,pd.procsym.realname+'__fpcvirtconstrwrapper__',pd.owner,tabstractrecorddef(pd.owner.defowner));
        { implementation: call through to the constructor
          Exception: if the current class is abstract, do not call the
            constructor, since abstract class cannot be constructed (and the
            Android verifier does not accept such code, even if it is
            unreachable) }
        wrapperpd.synthetickind:=tsk_callthrough_nonabstract;
        wrapperpd.skpara:=pd;
        symtablestack.pop(pd.owner);
        { and now wrap this generated virtual static method itself as well }
        jvm_wrap_virtual_class_method(wrapperpd);
      end;


    procedure jvm_wrap_virtual_class_methods(obj: tobjectdef);
      var
        i: longint;
        def: tdef;
      begin
        { new methods will be inserted while we do this, but since
          symtable.deflist.count is evaluated at the start of the loop that
          doesn't matter }
        for i:=0 to obj.symtable.deflist.count-1 do
          begin
            def:=tdef(obj.symtable.deflist[i]);
            if def.typ<>procdef then
              continue;
            if [po_classmethod,po_virtualmethod]<=tprocdef(def).procoptions then
              jvm_wrap_virtual_class_method(tprocdef(def))
            else if (tprocdef(def).proctypeoption=potype_constructor) and
               (po_virtualmethod in tprocdef(def).procoptions) then
              jvm_wrap_virtual_constructor(tprocdef(def));
          end;
      end;


    function jvm_add_typed_const_initializer(csym: tconstsym): tstaticvarsym;
      var
        ssym: tstaticvarsym;
        esym: tenumsym;
        i: longint;
        sstate: tscannerstate;
        elemdef: tdef;
        elemdefname,
        conststr: ansistring;
        first: boolean;
      begin
        result:=nil;
        esym:=nil;
        case csym.constdef.typ of
          enumdef:
            begin
              { make sure we don't emit a definition for this field (we'll do
                that for the constsym already) -> mark as external }
              ssym:=cstaticvarsym.create(internal_static_field_name(csym.realname),vs_final,csym.constdef,[vo_is_external]);
              csym.owner.insert(ssym);
              { alias storage to the constsym }
              ssym.set_mangledname(csym.realname);
              for i:=0 to tenumdef(csym.constdef).symtable.symlist.count-1 do
                begin
                  esym:=tenumsym(tenumdef(csym.constdef).symtable.symlist[i]);
                  if esym.value=csym.value.valueord.svalue then
                    break;
                  esym:=nil;
                end;
              { can happen in case of explicit typecast from integer constant
                to enum type }
              if not assigned(esym) then
                begin
                  MessagePos(csym.fileinfo,parser_e_range_check_error);
                  exit;
                end;
              replace_scanner('jvm_enum_const',sstate);
              str_parse_typedconst(current_asmdata.asmlists[al_typedconsts],esym.name+';',ssym);
              restore_scanner(sstate);
              result:=ssym;
            end;
          setdef:
            begin
              replace_scanner('jvm_set_const',sstate);
              { make sure we don't emit a definition for this field (we'll do
                that for the constsym already) -> mark as external;
                on the other hand, we don't create instances for constsyms in
                (or external syms) the program/unit initialization code -> add
                vo_has_local_copy to indicate that this should be done after all
                (in thlcgjvm.allocate_implicit_structs_for_st_with_base_ref) }

              { the constant can be defined in the body of a function and its
                def can also belong to that -> will be freed when the function
                has been compiler -> insert a copy in the unit's staticsymtable
              }
              symtablestack.push(current_module.localsymtable);
              ssym:=cstaticvarsym.create(internal_static_field_name(csym.realname),vs_final,tsetdef(csym.constdef).getcopy,[vo_is_external,vo_has_local_copy]);
              symtablestack.top.insert(ssym);
              symtablestack.pop(current_module.localsymtable);
              { alias storage to the constsym }
              ssym.set_mangledname(csym.realname);
              { ensure that we allocate space for global symbols (won't actually
                allocate space for this one, since it's external, but for the
                constsym) }
              cnodeutils.insertbssdata(ssym);
              elemdef:=tsetdef(csym.constdef).elementdef;
              if not assigned(elemdef) then
                begin
                  internalerror(2011070502);
                end
              else
                begin
                  elemdefname:=elemdef.typename;
                  conststr:='[';
                  first:=true;
                  for i:=0 to 255 do
                    if i in pnormalset(csym.value.valueptr)^ then
                      begin
                        if not first then
                          conststr:=conststr+',';
                        first:=false;
                        { instead of looking up all enum value names/boolean
                           names, type cast integers to the required type }
                        conststr:=conststr+elemdefname+'('+tostr(i)+')';
                      end;
                  conststr:=conststr+'];';
                end;
              str_parse_typedconst(current_asmdata.asmlists[al_typedconsts],conststr,ssym);
              restore_scanner(sstate);
              result:=ssym;
            end;
          else
            internalerror(2011062701);
        end;
      end;


    function jvm_wrap_method_with_vis(pd: tprocdef; vis: tvisibility): tprocdef;
      var
        obj: tabstractrecorddef;
        visname: string;
      begin
        obj:=current_structdef;
        { if someone gets the idea to add a property to an external class
          definition, don't try to wrap it since we cannot add methods to
          external classes }
        if oo_is_external in obj.objectoptions then
          begin
            result:=pd;
            exit
          end;
        symtablestack.push(obj.symtable);
        result:=tprocdef(pd.getcopy);
        result.visibility:=vis;
        visname:=visibilityName[vis];
        replace(visname,' ','_');
        { create a name that is unique amongst all units (start with '$unitname$$') and
          unique in this unit (result.unique_id_str) }
        finish_copied_procdef(result,'$'+current_module.realmodulename^+'$$'+result.unique_id_str+pd.procsym.realname+'$'+visname,obj.symtable,obj);
        { in case the referred method is from an external class }
        exclude(result.procoptions,po_external);
        { not virtual/override/abstract/... }
        result.procoptions:=result.procoptions*[po_classmethod,po_staticmethod,po_varargs,po_public];
        result.synthetickind:=tsk_callthrough;
        { so we know the name of the routine to call through to }
        result.skpara:=pd;
        symtablestack.pop(obj.symtable);
      end;


end.
