{
    Copyright (c) 1998-2011 by Florian Klaempfl

    Generic version of some node tree helper routines that can be overridden
    by cpu-specific versions

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
unit ngenutil;

{$i fpcdefs.inc}

interface

  uses
    cclasses,globtype,
    fmodule,
    aasmbase,aasmdata,
    node,nbas,symtype,symsym,symconst,symdef;


  type
    tinitfinalentry = record
      initfunc : TSymStr;
      finifunc : TSymStr;
      initpd : tprocdef;
      finipd : tprocdef;
      module : tmodule;
    end;
    pinitfinalentry = ^tinitfinalentry;

    tnodeutils = class
      class function call_fail_node:tnode; virtual;
      class function initialize_data_node(p:tnode; force: boolean):tnode; virtual;
      class function finalize_data_node(p:tnode):tnode; virtual;
     strict protected
      type
        tstructinifinipotype = potype_class_constructor..potype_class_destructor;
      class procedure sym_maybe_initialize(p: TObject; arg: pointer);
      { generates the code for finalisation of local variables }
      class procedure local_varsyms_finalize(p:TObject;arg:pointer);
      { generates the code for finalization of static symtable and
        all local (static) typed consts }
      class procedure static_syms_finalize(p: TObject; arg: pointer);
      class procedure sym_maybe_finalize(var stat: tstatementnode; sym: tsym);
      class procedure append_struct_initfinis(u: tmodule; initfini: tstructinifinipotype; var stat: tstatementnode); virtual;
     public
      class procedure procdef_block_add_implicit_initialize_nodes(pd: tprocdef; var stat: tstatementnode);
      class procedure procdef_block_add_implicit_finalize_nodes(pd: tprocdef; var stat: tstatementnode);
      { returns true if the unit requires an initialisation section (e.g.,
        to force class constructors for the JVM target to initialise global
        records/arrays) }
      class function force_init: boolean; virtual;
      { idem for finalization }
      class function force_final: boolean; virtual;

      { if the funcretsym was moved to the parentfpstruct, use this method to
        move its value back back into the funcretsym before the function exit, as
        the code generator is hardcoded to use to use the funcretsym when loading
        the value to be returned; replacing it with an absolutevarsym that
        redirects to the field in the parentfpstruct doesn't work, as the code
        generator cannot deal with such symbols }
       class procedure load_parentfpstruct_nested_funcret(ressym: tsym; var stat: tstatementnode);
      { called after parsing a routine with the code of the entire routine
        as argument; can be used to modify the node tree. By default handles
        insertion of code for systems that perform the typed constant
        initialisation via the node tree }
      class function wrap_proc_body(pd: tprocdef; n: tnode): tnode; virtual;

      { trashes a paravarsym or localvarsym if possible (not a managed type,
        "out" in case of parameter, ...) }
      class procedure maybe_trash_variable(var stat: tstatementnode; p: tabstractnormalvarsym; trashn: tnode); virtual;
     strict protected
      { called from wrap_proc_body to insert the trashing for the wrapped
        routine's local variables and parameters }
      class function  maybe_insert_trashing(pd: tprocdef; n: tnode): tnode;
      class function  check_insert_trashing(pd: tprocdef): boolean; virtual;
      { callback called for every local variable and parameter by
        maybe_insert_trashing(), calls through to maybe_trash_variable() }
      class procedure maybe_trash_variable_callback(p: TObject; statn: pointer);
      { returns whether a particular sym can be trashed. If not,
        maybe_trash_variable won't do anything }
      class function  trashable_sym(p: tsym): boolean; virtual;
      { trashing for 1/2/3/4/8-byte sized variables }
      class procedure trash_small(var stat: tstatementnode; trashn: tnode; trashvaln: tnode); virtual;
      { trashing for differently sized variables that those handled by
        trash_small() }
      class procedure trash_large(var stat: tstatementnode; trashn, sizen: tnode; trashintval: int64); virtual;
      { insert a single bss sym, called by insert bssdata (factored out
        non-common part for llvm) }
      class procedure insertbsssym(list: tasmlist; sym: tstaticvarsym; size: asizeint; varalign: shortint; _typ: Tasmsymtype); virtual;

      { initialization of iso styled program parameters }
      class procedure initialize_textrec(p : TObject; statn : pointer);
      { finalization of iso styled program parameters }
      class procedure finalize_textrec(p : TObject; statn : pointer);
     public
      class procedure insertbssdata(sym : tstaticvarsym); virtual;

      class function create_main_procdef(const name: string; potype:tproctypeoption; ps: tprocsym):tdef; virtual;
      class procedure InsertInitFinalTable;
     protected
      class procedure InsertRuntimeInits(const prefix:string;list:TLinkedList;unitflag:tmoduleflag); virtual;
      class procedure InsertRuntimeInitsTablesTable(const prefix,tablename:string;unitflag:tmoduleflag); virtual;

      class procedure insert_init_final_table(entries:tfplist); virtual;

      class function get_init_final_list: tfplist;
      class procedure release_init_final_list(list:tfplist);
     public
      class procedure InsertThreadvarTablesTable; virtual;
      class procedure InsertThreadvars; virtual;
      class procedure InsertWideInitsTablesTable; virtual;
      class procedure InsertWideInits; virtual;
      class procedure InsertResStrInits; virtual;
      class procedure InsertResStrTablesTable; virtual;
      class procedure InsertResourceTablesTable; virtual;
      class procedure InsertResourceInfo(ResourcesUsed : boolean); virtual;

      class procedure InsertMemorySizes; virtual;

      { called right before an object is assembled, can be used to insert
        global information into the assembler list (used by LLVM to insert type
        info) }
      class procedure InsertObjectInfo; virtual;

      { register that asm symbol sym with type def has to be considered as "used" even if not
        references to it can be found. If compileronly, this is only for the compiler, otherwise
        also for the linker }
      class procedure RegisterUsedAsmSym(sym: TAsmSymbol; def: tdef; compileronly: boolean); virtual;

      class procedure RegisterModuleInitFunction(pd: tprocdef); virtual;
      class procedure RegisterModuleFiniFunction(pd: tprocdef); virtual;

      class procedure GenerateObjCImageInfo; virtual;

     strict protected
      class procedure add_main_procdef_paras(pd: tdef); virtual;
    end;
    tnodeutilsclass = class of tnodeutils;

  const
    cnodeutils: tnodeutilsclass = tnodeutils;


implementation

    uses
      verbose,version,globals,cutils,constexp,compinnr,
      systems,procinfo,pparautl,
      aasmtai,aasmcnst,
      symbase,symtable,defutil,
      nadd,ncal,ncnv,ncon,nflw,ninl,nld,nmem,nutils,
      ppu,
      pass_1,
      export;

  class function tnodeutils.call_fail_node:tnode;
    var
      para : tcallparanode;
      newstatement : tstatementnode;
      srsym : tsym;
    begin
      result:=internalstatements(newstatement);

      { call fail helper and exit normal }
      if is_class(current_structdef) then
        begin
          srsym:=search_struct_member(current_structdef,'FREEINSTANCE');
          if assigned(srsym) and
             (srsym.typ=procsym) then
            begin
              { if self<>0 and vmt<>0 then freeinstance }
              addstatement(newstatement,cifnode.create(
                  caddnode.create(andn,
                      caddnode.create(unequaln,
                          load_self_pointer_node,
                          cnilnode.create),
                      caddnode.create(unequaln,
                          load_vmt_pointer_node,
                          cnilnode.create)),
                  ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[],nil),
                  nil));
            end
          else
            internalerror(200305108);
        end
      else
        if is_object(current_structdef) then
          begin
            { parameter 3 : vmt_offset }
            { parameter 2 : pointer to vmt }
            { parameter 1 : self pointer }
            para:=ccallparanode.create(
                      cordconstnode.create(tobjectdef(current_structdef).vmt_offset,s32inttype,false),
                  ccallparanode.create(
                      ctypeconvnode.create_internal(
                          load_vmt_pointer_node,
                          voidpointertype),
                  ccallparanode.create(
                      ctypeconvnode.create_internal(
                          load_self_pointer_node,
                          voidpointertype),
                  nil)));
            addstatement(newstatement,
                ccallnode.createintern('fpc_help_fail',para));
          end
      else
        internalerror(200305132);
      { self:=nil }
      addstatement(newstatement,cassignmentnode.create(
          load_self_pointer_node,
          cnilnode.create));
      { exit }
      addstatement(newstatement,cexitnode.create(nil));
    end;


  class function tnodeutils.initialize_data_node(p:tnode; force: boolean):tnode;
    begin
      { prevent initialisation of hidden syms that were moved to
        parentfpstructs: the original symbol isn't used anymore, the version
        in parentfpstruct will be initialised when that struct gets initialised,
        and references to it will actually be translated into references to the
        field in the parentfpstruct (so we'll initialise it twice) }
      if (target_info.system in systems_fpnestedstruct) and
         (p.nodetype=loadn) and
         (tloadnode(p).symtableentry.typ=localvarsym) and
         (tloadnode(p).symtableentry.visibility=vis_hidden) then
        begin
          p.free;
          result:=cnothingnode.create;
        end
      else
        begin
          if not assigned(p.resultdef) then
            typecheckpass(p);
          if is_ansistring(p.resultdef) or
             is_wide_or_unicode_string(p.resultdef) or
             is_interfacecom_or_dispinterface(p.resultdef) or
             is_dynamic_array(p.resultdef) then
            begin
              result:=cassignmentnode.create(
                 ctypeconvnode.create_internal(p,voidpointertype),
                 cnilnode.create
                 );
            end
          else if (p.resultdef.typ=variantdef) then
            begin
              result:=ccallnode.createintern('fpc_variant_init',
                ccallparanode.create(
                  ctypeconvnode.create_internal(p,search_system_type('TVARDATA').typedef),
                nil));
            end
          else
            begin
              result:=ccallnode.createintern('fpc_initialize',
                    ccallparanode.create(
                        caddrnode.create_internal(
                            crttinode.create(
                                tstoreddef(p.resultdef),initrtti,rdt_normal)),
                    ccallparanode.create(
                        caddrnode.create_internal(p),
                    nil)));
            end;
        end;
    end;


  class function tnodeutils.finalize_data_node(p:tnode):tnode;
    var
      hs : string;
    begin
      { see comment in initialize_data_node above }
      if (target_info.system in systems_fpnestedstruct) and
         (p.nodetype=loadn) and
         (tloadnode(p).symtableentry.typ=localvarsym) and
         (tloadnode(p).symtableentry.visibility=vis_hidden) then
        begin
          p.free;
          result:=cnothingnode.create;
        end
      else
        begin
          if not assigned(p.resultdef) then
            typecheckpass(p);
          { 'decr_ref' suffix is somewhat misleading, all these helpers
            set the passed pointer to nil now }
          if is_ansistring(p.resultdef) then
            hs:='fpc_ansistr_decr_ref'
          else if is_widestring(p.resultdef) then
            hs:='fpc_widestr_decr_ref'
          else if is_unicodestring(p.resultdef) then
            hs:='fpc_unicodestr_decr_ref'
          else if is_interfacecom_or_dispinterface(p.resultdef) then
            hs:='fpc_intf_decr_ref'
          else
            hs:='';
          if hs<>'' then
            result:=ccallnode.createintern(hs,
               ccallparanode.create(
                 ctypeconvnode.create_internal(p,voidpointertype),
                 nil))
          else if p.resultdef.typ=variantdef then
            begin
              result:=ccallnode.createintern('fpc_variant_clear',
                ccallparanode.create(
                  ctypeconvnode.create_internal(p,search_system_type('TVARDATA').typedef),
                nil));
            end
          else
            result:=ccallnode.createintern('fpc_finalize',
                  ccallparanode.create(
                      caddrnode.create_internal(
                          crttinode.create(
                              tstoreddef(p.resultdef),initrtti,rdt_normal)),
                  ccallparanode.create(
                      caddrnode.create_internal(p),
                  nil)));
        end;
    end;


  class procedure tnodeutils.sym_maybe_initialize(p: TObject; arg: pointer);
    begin
      if ((tsym(p).typ = localvarsym) or
          { check staticvarsym for record management opeators and for objects
            which might contain record with management operators }
          ((tsym(p).typ = staticvarsym) and
           (
             is_record(tabstractvarsym(p).vardef) or
             is_object(tabstractvarsym(p).vardef)
           )
          )
         ) and
         { local (procedure or unit) variables only need initialization if
           they are used }
         ((tabstractvarsym(p).refs>0) or
          { managed return symbols must be inited }
          ((tsym(p).typ=localvarsym) and (vo_is_funcret in tlocalvarsym(p).varoptions))
         ) and
         not(vo_is_typed_const in tabstractvarsym(p).varoptions) and
         not(vo_is_external in tabstractvarsym(p).varoptions) and
         not(vo_is_default_var in tabstractvarsym(p).varoptions) and
         (is_managed_type(tabstractvarsym(p).vardef) or
          ((m_iso in current_settings.modeswitches) and (tabstractvarsym(p).vardef.typ=filedef))
         ) then
        begin
          addstatement(tstatementnode(arg^),initialize_data_node(cloadnode.create(tsym(p),tsym(p).owner),false));
        end;
    end;


  class procedure tnodeutils.local_varsyms_finalize(p: TObject; arg: pointer);
    begin
      if (tsym(p).typ=localvarsym) and
         (tlocalvarsym(p).refs>0) and
         not(vo_is_external in tlocalvarsym(p).varoptions) and
         not(vo_is_funcret in tlocalvarsym(p).varoptions) and
         not(vo_is_default_var in tabstractvarsym(p).varoptions) and
         is_managed_type(tlocalvarsym(p).vardef) then
        sym_maybe_finalize(tstatementnode(arg^),tsym(p));
    end;


  class procedure tnodeutils.static_syms_finalize(p: TObject; arg: pointer);
    var
      i : longint;
      pd : tprocdef;
    begin
      case tsym(p).typ of
        staticvarsym :
          begin
            { local (procedure or unit) variables only need finalization
              if they are used
            }
            if ((tstaticvarsym(p).refs>0) or
                { global (unit) variables always need finalization, since
                  they may also be used in another unit
                }
                (tstaticvarsym(p).owner.symtabletype=globalsymtable)) and
                (
                  (tstaticvarsym(p).varspez<>vs_const) or
                  (vo_force_finalize in tstaticvarsym(p).varoptions)
                ) and
               not(vo_is_funcret in tstaticvarsym(p).varoptions) and
               not(vo_is_external in tstaticvarsym(p).varoptions) and
               is_managed_type(tstaticvarsym(p).vardef) and
               not (
                   assigned(tstaticvarsym(p).fieldvarsym) and
                   assigned(tstaticvarsym(p).fieldvarsym.owner.defowner) and
                   (df_generic in tdef(tstaticvarsym(p).fieldvarsym.owner.defowner).defoptions)
                 )
               then
              sym_maybe_finalize(tstatementnode(arg^),tsym(p));
          end;
        procsym :
          begin
            for i:=0 to tprocsym(p).ProcdefList.Count-1 do
              begin
                pd:=tprocdef(tprocsym(p).ProcdefList[i]);
                if assigned(pd.localst) and
                   (pd.procsym=tprocsym(p)) and
                   (pd.localst.symtabletype<>staticsymtable) then
                  pd.localst.SymList.ForEachCall(@static_syms_finalize,arg);
              end;
          end;
        else
          ;
      end;
    end;


  class procedure tnodeutils.sym_maybe_finalize(var stat: tstatementnode; sym: tsym);
    var
      hp: tnode;
    begin
      include(current_procinfo.flags,pi_needs_implicit_finally);
      hp:=cloadnode.create(sym,sym.owner);
      if (sym.typ=staticvarsym) and (vo_force_finalize in tstaticvarsym(sym).varoptions) then
        include(tloadnode(hp).loadnodeflags,loadnf_isinternal_ignoreconst);
      addstatement(stat,finalize_data_node(hp));
    end;


  procedure AddToStructInits(p:TObject;arg:pointer);
    var
      StructList: TFPList absolute arg;
    begin
      if (tdef(p).typ in [objectdef,recorddef]) and
         not (df_generic in tdef(p).defoptions) then
        begin
          { first add the class... }
          if ([oo_has_class_constructor,oo_has_class_destructor] * tabstractrecorddef(p).objectoptions <> []) then
            StructList.Add(p);
          { ... and then also add all subclasses }
          tabstractrecorddef(p).symtable.deflist.foreachcall(@AddToStructInits,arg);
        end;
    end;


  class procedure tnodeutils.append_struct_initfinis(u: tmodule; initfini: tstructinifinipotype; var stat: tstatementnode);
    var
      structlist: tfplist;
      i: integer;
      pd: tprocdef;
    begin
      structlist:=tfplist.Create;
      if assigned(u.globalsymtable) then
        u.globalsymtable.DefList.ForEachCall(@AddToStructInits,structlist);
      u.localsymtable.DefList.ForEachCall(@AddToStructInits,structlist);
      { write structures }
      for i:=0 to structlist.Count-1 do
        begin
          pd:=tabstractrecorddef(structlist[i]).find_procdef_bytype(initfini);
          if assigned(pd) then
            begin
              { class constructors are private -> ignore visibility checks }
              addstatement(stat,
                ccallnode.create(nil,tprocsym(pd.procsym),pd.owner,nil,[cnf_ignore_visibility],nil))
            end;
        end;
      structlist.free;
    end;


  class procedure tnodeutils.procdef_block_add_implicit_initialize_nodes(pd: tprocdef; var stat: tstatementnode);
    begin
      { initialize local data like ansistrings }
      case pd.proctypeoption of
         potype_unitinit:
           begin
             { this is also used for initialization of variables in a
               program which does not have a globalsymtable }
             if assigned(current_module.globalsymtable) then
               TSymtable(current_module.globalsymtable).SymList.ForEachCall(@sym_maybe_initialize,@stat);
             TSymtable(current_module.localsymtable).SymList.ForEachCall(@sym_maybe_initialize,@stat);
             { insert class constructors  }
             if mf_classinits in current_module.moduleflags then
               append_struct_initfinis(current_module, potype_class_constructor, stat);
           end;
         { units have seperate code for initilization and finalization }
         potype_unitfinalize: ;
         { program init/final is generated in separate procedure }
         potype_proginit: ;
         else
           current_procinfo.procdef.localst.SymList.ForEachCall(@sym_maybe_initialize,@stat);
      end;
    end;


  class procedure tnodeutils.procdef_block_add_implicit_finalize_nodes(pd: tprocdef; var stat: tstatementnode);
    begin
      { no finalization in exceptfilters, they /are/ the finalization code }
      if current_procinfo.procdef.proctypeoption=potype_exceptfilter then
          exit;

      { finalize local data like ansistrings}
      case current_procinfo.procdef.proctypeoption of
         potype_unitfinalize:
           begin
             { insert class destructors  }
             if mf_classinits in current_module.moduleflags then
               append_struct_initfinis(current_module, potype_class_destructor, stat);
             { this is also used for initialization of variables in a
               program which does not have a globalsymtable }
             if assigned(current_module.globalsymtable) then
               TSymtable(current_module.globalsymtable).SymList.ForEachCall(@static_syms_finalize,@stat);
             TSymtable(current_module.localsymtable).SymList.ForEachCall(@static_syms_finalize,@stat);
           end;
         { units/progs have separate code for initialization and finalization }
         potype_unitinit: ;
         { program init/final is generated in separate procedure }
         potype_proginit: ;
         else
           current_procinfo.procdef.localst.SymList.ForEachCall(@local_varsyms_finalize,@stat);
      end;
    end;


  class function tnodeutils.force_init: boolean;
    begin
      result:=
        (target_info.system in systems_typed_constants_node_init) and
        assigned(current_module.tcinitcode);
    end;


  class function tnodeutils.force_final: boolean;
    begin
      result:=false;
    end;


  class procedure tnodeutils.initialize_textrec(p:TObject;statn:pointer);
    var
      stat: ^tstatementnode absolute statn;
    begin
      if (tsym(p).typ=staticvarsym) and
       (tstaticvarsym(p).vardef.typ=filedef) and
       (tfiledef(tstaticvarsym(p).vardef).filetyp=ft_text) and
       (tstaticvarsym(p).isoindex<>0) then
       begin
         if cs_transparent_file_names in current_settings.globalswitches then
           addstatement(stat^,ccallnode.createintern('fpc_textinit_filename_iso',
             ccallparanode.create(
               cstringconstnode.createstr(tstaticvarsym(p).Name),
             ccallparanode.create(
               cordconstnode.create(tstaticvarsym(p).isoindex,uinttype,false),
             ccallparanode.create(
               cloadnode.create(tstaticvarsym(p),tstaticvarsym(p).Owner),
             nil)))))
         else
           addstatement(stat^,ccallnode.createintern('fpc_textinit_iso',
             ccallparanode.create(
               cordconstnode.create(tstaticvarsym(p).isoindex,uinttype,false),
             ccallparanode.create(
               cloadnode.create(tstaticvarsym(p),tstaticvarsym(p).Owner),
             nil))));
       end;
    end;


  class procedure tnodeutils.finalize_textrec(p:TObject;statn:pointer);
    var
      stat: ^tstatementnode absolute statn;
    begin
      if (tsym(p).typ=staticvarsym) and
       (tstaticvarsym(p).vardef.typ=filedef) and
       (tfiledef(tstaticvarsym(p).vardef).filetyp=ft_text) and
       (tstaticvarsym(p).isoindex<>0) then
       begin
         addstatement(stat^,ccallnode.createintern('fpc_textclose_iso',
           ccallparanode.create(
             cloadnode.create(tstaticvarsym(p),tstaticvarsym(p).Owner),
           nil)));
       end;
    end;


  class procedure tnodeutils.load_parentfpstruct_nested_funcret(ressym: tsym; var stat: tstatementnode);
    var
      target: tnode;
    begin
      target:=cloadnode.create(ressym, ressym.owner);
      { ensure the target of this assignment doesn't translate the
        funcretsym also to its alias in the parentfpstruct }
      include(target.flags, nf_internal);
      addstatement(stat,
        cassignmentnode.create(
          target, cloadnode.create(ressym, ressym.owner)
        )
      );
    end;


  class function tnodeutils.wrap_proc_body(pd: tprocdef; n: tnode): tnode;
    var
      stat: tstatementnode;
      block: tnode;
      ressym,
      psym: tsym;
      resdef: tdef;
    begin
      result:=maybe_insert_trashing(pd,n);

      { initialise safecall result variable }
      if pd.generate_safecall_wrapper then
        begin
          ressym:=tsym(pd.localst.Find('safecallresult'));
          block:=internalstatements(stat);
          addstatement(stat,
            cassignmentnode.create(
              cloadnode.create(ressym,ressym.owner),
              genintconstnode(0)
            )
          );
          addstatement(stat,result);
          result:=block;
        end;

      if (m_isolike_program_para in current_settings.modeswitches) and
        (pd.proctypeoption=potype_proginit) then
        begin
          block:=internalstatements(stat);
          pd.localst.SymList.ForEachCall(@initialize_textrec,@stat);
          addstatement(stat,result);
          pd.localst.SymList.ForEachCall(@finalize_textrec,@stat);
          result:=block;
        end;

      if target_info.system in systems_typed_constants_node_init then
        begin
          case pd.proctypeoption of
            potype_class_constructor:
              begin
                { even though the initialisation code for typed constants may
                  not yet be complete at this point (there may be more inside
                  method definitions coming after this class constructor), the
                  ones from inside the class definition have already been parsed.
                  in case of $j-, these are marked "final" in Java and such
                  static fields must be initialsed in the class constructor
                  itself -> add them here }
                block:=internalstatements(stat);
                if assigned(pd.struct.tcinitcode) then
                  begin
                    addstatement(stat,pd.struct.tcinitcode);
                    pd.struct.tcinitcode:=nil;
                  end;
                psym:=tsym(pd.struct.symtable.find('FPC_INIT_TYPED_CONSTS_HELPER'));
                if assigned(psym) then
                  begin
                    if (psym.typ<>procsym) or
                       (tprocsym(psym).procdeflist.count<>1) then
                      internalerror(2011040301);
                    addstatement(stat,ccallnode.create(nil,tprocsym(psym),
                      pd.struct.symtable,nil,[],nil));
                  end;
                addstatement(stat,result);
                result:=block
              end;
            potype_unitinit:
              begin
                if assigned(current_module.tcinitcode) then
                  begin
                    block:=internalstatements(stat);
                    addstatement(stat,tnode(current_module.tcinitcode));
                    current_module.tcinitcode:=nil;
                    addstatement(stat,result);
                    result:=block;
                  end;
              end;
            else case pd.synthetickind of
              tsk_tcinit:
                begin
                  if assigned(pd.struct.tcinitcode) then
                    begin
                      block:=internalstatements(stat);
                      addstatement(stat,pd.struct.tcinitcode);
                      pd.struct.tcinitcode:=nil;
                      addstatement(stat,result);
                      result:=block
                    end
                end;
              else
                ;
            end;
          end;
        end;
      if (target_info.system in systems_fpnestedstruct) and
         pd.get_funcretsym_info(ressym,resdef) and
         (tabstractnormalvarsym(ressym).inparentfpstruct) then
        begin
          block:=internalstatements(stat);
          addstatement(stat,result);
          load_parentfpstruct_nested_funcret(ressym,stat);
          result:=block;
        end;
    end;


  class function tnodeutils.maybe_insert_trashing(pd: tprocdef; n: tnode): tnode;
    var
      stat: tstatementnode;
    begin
      result:=n;
      if check_insert_trashing(pd) then
        begin
          result:=internalstatements(stat);
          pd.parast.SymList.ForEachCall(@maybe_trash_variable_callback,@stat);
          pd.localst.SymList.ForEachCall(@maybe_trash_variable_callback,@stat);
          addstatement(stat,n);
        end;
    end;

  class function tnodeutils.check_insert_trashing(pd: tprocdef): boolean;
    begin
      result:=
        (localvartrashing<>-1) and
        not(po_assembler in pd.procoptions);
    end;


  class function tnodeutils.trashable_sym(p: tsym): boolean;
    begin
      result:=
        ((p.typ=localvarsym) or
         ((p.typ=paravarsym) and
          ((vo_is_funcret in tabstractnormalvarsym(p).varoptions) or
           (tabstractnormalvarsym(p).varspez=vs_out)))) and
         not (vo_is_default_var in tabstractnormalvarsym(p).varoptions) and
         (not is_managed_type(tabstractnormalvarsym(p).vardef) or
          (is_string(tabstractnormalvarsym(p).vardef) and
           (vo_is_funcret in tabstractnormalvarsym(p).varoptions)
          )
         ) and
         not assigned(tabstractnormalvarsym(p).defaultconstsym);
    end;


  class procedure tnodeutils.maybe_trash_variable(var stat: tstatementnode; p: tabstractnormalvarsym; trashn: tnode);
    var
      size: asizeint;
      trashintval: int64;
      stringres: tstringconstnode;
    begin
      if trashable_sym(p) then
        begin
          trashintval:=trashintvalues[localvartrashing];
          if (p.vardef.typ=procvardef) and
             ([m_tp_procvar,m_mac_procvar]*current_settings.modeswitches<>[]) then
            begin
              if tprocvardef(p.vardef).is_addressonly then
                { in tp/delphi mode, you need @procvar to get at the contents of
                  a procvar ... }
                trashn:=caddrnode.create(trashn)
              else
                { ... but if it's a procedure of object, that will only return
                  the procedure address -> cast to tmethod instead }
                trashn:=ctypeconvnode.create_explicit(trashn,methodpointertype);
            end;
          if is_managed_type(p.vardef) then
            begin
              if is_string(p.vardef) then
                begin
                  stringres:=
                    cstringconstnode.createstr(
                      'uninitialized function result in '+
                      tprocdef(p.owner.defowner).customprocname([pno_proctypeoption, pno_paranames,pno_ownername, pno_noclassmarker])
                    );
                  { prevent attempts to convert the string to the specified
                    code page at compile time, as it may not be available (and
                    it does not matter) }
                  if is_ansistring(p.vardef) then
                    stringres.changestringtype(search_system_type('RAWBYTESTRING').typedef);
                  trash_small(stat,trashn,stringres);
                end
              else
                internalerror(2016030601);
            end
          else if ((p.typ=localvarsym) and
              (not(vo_is_funcret in p.varoptions) or
               not is_shortstring(p.vardef))) or
             ((p.typ=paravarsym) and
              not is_shortstring(p.vardef)) then
            begin
              size:=p.getsize;
              case size of
                0:
                  begin
                    { open array -> at least size 1. Can also be zero-sized
                      record, so check it's actually an array }
                    if p.vardef.typ=arraydef then
                      trash_large(stat,trashn,caddnode.create(addn,cinlinenode.create(in_high_x,false,trashn.getcopy),genintconstnode(1)),trashintval)
                    else
                      trashn.free;
                  end;
                1: trash_small(stat,
                  ctypeconvnode.create_internal(trashn,s8inttype),
                    genintconstnode(shortint(trashintval)));
                2: trash_small(stat,
                  ctypeconvnode.create_internal(trashn,s16inttype),
                    genintconstnode(smallint(trashintval)));
                4: trash_small(stat,
                  ctypeconvnode.create_internal(trashn,s32inttype),
                    genintconstnode(longint(trashintval)));
                8: trash_small(stat,
                  ctypeconvnode.create_internal(trashn,s64inttype),
                    genintconstnode(int64(trashintval)));
                else
                  trash_large(stat,trashn,genintconstnode(size),trashintval);
              end;
            end
          else
            begin
              { may be an open string, even if is_open_string() returns false
                (for some helpers in the system unit)             }
              { an open string has at least size 2                      }
              trash_small(stat,
                cvecnode.create(trashn.getcopy,genintconstnode(0)),
                cordconstnode.create(tconstexprint(byte(trashintval)),cansichartype,false));
              trash_small(stat,
                cvecnode.create(trashn,genintconstnode(1)),
                cordconstnode.create(tconstexprint(byte(trashintval)),cansichartype,false));
            end;
        end
      else
        trashn.free;
    end;


  class procedure tnodeutils.maybe_trash_variable_callback(p:TObject;statn:pointer);
    var
      stat: ^tstatementnode absolute statn;
    begin
      if not(tsym(p).typ in [localvarsym,paravarsym]) then
        exit;
      maybe_trash_variable(stat^,tabstractnormalvarsym(p),cloadnode.create(tsym(p),tsym(p).owner));
    end;


  class procedure tnodeutils.trash_small(var stat: tstatementnode; trashn: tnode; trashvaln: tnode);
    begin
      addstatement(stat,cassignmentnode.create(trashn,trashvaln));
    end;


  class procedure tnodeutils.trash_large(var stat: tstatementnode; trashn, sizen: tnode; trashintval: int64);
    begin
      addstatement(stat,ccallnode.createintern('fpc_fillmem',
        ccallparanode.Create(cordconstnode.create(tconstexprint(byte(trashintval)),u8inttype,false),
        ccallparanode.Create(sizen,
        ccallparanode.Create(trashn,nil)))
        ));
    end;


  class procedure tnodeutils.insertbsssym(list: tasmlist; sym: tstaticvarsym; size: asizeint; varalign: shortint; _typ:Tasmsymtype);
    begin
      if sym.globalasmsym then
        begin
          { on AIX/stabx, we cannot generate debug information that encodes
            the address of a global symbol, you need a symbol with the same
            name as the identifier -> create an extra *local* symbol.
            Moreover, such a local symbol will be removed if it's not
            referenced anywhere, so also create a reference }
          if (target_dbg.id=dbg_stabx) and
             (cs_debuginfo in current_settings.moduleswitches) and
             not assigned(current_asmdata.GetAsmSymbol(sym.name)) then
            begin
              list.concat(tai_symbol.Create(current_asmdata.DefineAsmSymbol(sym.name,AB_LOCAL,AT_DATA,sym.vardef),0));
              list.concat(tai_directive.Create(asd_reference,sym.name));
            end;
          list.concat(Tai_datablock.create_global(sym.mangledname,size,sym.vardef,_typ));
        end
      else
        list.concat(Tai_datablock.create_hidden(sym.mangledname,size,sym.vardef,_typ));
    end;


  class procedure tnodeutils.insertbssdata(sym: tstaticvarsym);
    var
      l : asizeint;
      varalign : shortint;
      storefilepos : tfileposinfo;
      list : TAsmList;
      sectype : TAsmSectiontype;
      asmtype: TAsmsymtype;
    begin
      storefilepos:=current_filepos;
      current_filepos:=sym.fileinfo;
      l:=sym.getsize;
      varalign:=sym.vardef.alignment;
      if (varalign=0) then
        varalign:=var_align_size(l)
      else
        varalign:=var_align(varalign);
      asmtype:=AT_DATA;
      if tf_section_threadvars in target_info.flags then
        begin
          if (vo_is_thread_var in sym.varoptions) then
            begin
              list:=current_asmdata.asmlists[al_threadvars];
              sectype:=sec_threadvar;
              asmtype:=AT_TLS;
            end
          else
            begin
              list:=current_asmdata.asmlists[al_globals];
              sectype:=sec_bss;
            end;
        end
      else
        begin
          if (vo_is_thread_var in sym.varoptions) then
            begin
              inc(l,sizeof(pint));
              { it doesn't help to set a higher alignment, as  }
              { the first sizeof(pint) bytes field will offset }
              { everything anyway                              }
              varalign:=sizeof(pint);
            end;
          list:=current_asmdata.asmlists[al_globals];
          sectype:=sec_bss;
        end;
      maybe_new_object_file(list);
      if vo_has_section in sym.varoptions then
        new_section(list,sec_user,sym.section,varalign)
      else
        new_section(list,sectype,lower(sym.mangledname),varalign);
      insertbsssym(list,sym,l,varalign,asmtype);
      current_filepos:=storefilepos;
    end;


  class function tnodeutils.create_main_procdef(const name: string; potype: tproctypeoption; ps: tprocsym): tdef;
    var
      pd: tprocdef;
    begin
      if potype<>potype_mainstub then
        pd:=cprocdef.create(main_program_level,true)
      else
        pd:=cprocdef.create(normal_function_level,true);
      { always register the def }
      pd.register_def;
      pd.procsym:=ps;
      ps.ProcdefList.Add(pd);
      include(pd.procoptions,po_global);
      { set procdef options }
      pd.proctypeoption:=potype;
      pd.proccalloption:=pocall_default;
      include(pd.procoptions,po_hascallingconvention);
      pd.forwarddef:=false;
      { may be required to calculate the mangled name }
      add_main_procdef_paras(pd);
      pd.setmangledname(name);
      { the mainstub is generated via a synthetic proc -> parsed via
        psub.read_proc_body() -> that one will insert the mangled name in the
        alias names already }
      if not(potype in [potype_mainstub,potype_libmainstub]) then
        pd.aliasnames.insert(pd.mangledname);
      result:=pd;
    end;


  class function tnodeutils.get_init_final_list:tfplist;
    var
      hp : tused_unit;
      entry : pinitfinalentry;
    begin
      result:=tfplist.create;
      { Insert initialization/finalization of the used units }
      hp:=tused_unit(usedunits.first);
      while assigned(hp) do
       begin
         if (hp.u.moduleflags * [mf_init,mf_finalize])<>[] then
           begin
             new(entry);
             entry^.module:=hp.u;
             entry^.initpd:=nil;
             entry^.finipd:=nil;
             if mf_init in hp.u.moduleflags then
               entry^.initfunc:=make_mangledname('INIT$',hp.u.globalsymtable,'')
             else
               entry^.initfunc:='';
             if mf_finalize in hp.u.moduleflags then
               entry^.finifunc:=make_mangledname('FINALIZE$',hp.u.globalsymtable,'')
             else
               entry^.finifunc:='';
             result.add(entry);
           end;
         hp:=tused_unit(hp.next);
       end;

      { Insert initialization/finalization of the program }
      if (current_module.moduleflags * [mf_init,mf_finalize])<>[] then
        begin
          new(entry);
          entry^.module:=current_module;
          entry^.initpd:=nil;
          entry^.finipd:=nil;
          if mf_init in current_module.moduleflags then
            entry^.initfunc:=make_mangledname('INIT$',current_module.localsymtable,'')
          else
            entry^.initfunc:='';
          if mf_finalize in current_module.moduleflags then
            entry^.finifunc:=make_mangledname('FINALIZE$',current_module.localsymtable,'')
          else
            entry^.finifunc:='';
          result.add(entry);
        end;
    end;


  class procedure tnodeutils.release_init_final_list(list:tfplist);
    var
      i : longint;
    begin
      if not assigned(list) then
        internalerror(2017051901);
      for i:=0 to list.count-1 do
        dispose(pinitfinalentry(list[i]));
      list.free;
    end;


  class procedure tnodeutils.InsertInitFinalTable;
    var
      entries : tfplist;
    begin
      entries := get_init_final_list;

      insert_init_final_table(entries);

      release_init_final_list(entries);
    end;


  class procedure tnodeutils.insert_init_final_table(entries:tfplist);
    var
      i : longint;
      unitinits : ttai_typedconstbuilder;
      nameinit,namefini : TSymStr;
      tabledef: tdef;
      entry : pinitfinalentry;

      procedure add_initfinal_import(symtable:tsymtable);
        var
          i,j : longint;
          foundinit,foundfini : boolean;
          sym : TSymEntry;
          pd : tprocdef;
        begin
          if (nameinit='') and (namefini='') then
            exit;
          foundinit:=nameinit='';
          foundfini:=namefini='';
          for i:=0 to symtable.SymList.Count-1 do
            begin
              sym:=tsymentry(symtable.SymList[i]);
              if sym.typ<>procsym then
                continue;
              for j:=0 to tprocsym(sym).procdeflist.count-1 do
                begin
                  pd:=tprocdef(tprocsym(sym).procdeflist[j]);
                  if (nameinit<>'') and not foundinit and pd.has_alias_name(nameinit) then
                    begin
                      current_module.addimportedsym(sym);
                      foundinit:=true;
                    end;
                  if (namefini<>'') and not foundfini and pd.has_alias_name(namefini) then
                    begin
                      current_module.addimportedsym(sym);
                      foundfini:=true;
                    end;
                  if foundinit and foundfini then
                    break;
                end;
              if foundinit and foundfini then
                break;
            end;
          if not foundinit or not foundfini then
            internalerror(2016041401);
        end;

    begin
      unitinits:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
      unitinits.begin_anonymous_record('',default_settings.packrecords,sizeof(pint),
        targetinfos[target_info.system]^.alignment.recordalignmin);

      { tablecount }
      unitinits.emit_ord_const(entries.count,aluuinttype);
      { initcount (initialised at run time }
      unitinits.emit_ord_const(0,aluuinttype);

      for i:=0 to entries.count-1 do
        begin
          entry:=pinitfinalentry(entries[i]);
          if assigned(entry^.initpd) or assigned(entry^.finipd) then
            begin
              if assigned(entry^.initpd) then
                begin
                  unitinits.emit_procdef_const(entry^.initpd);
                  if entry^.module<>current_module then
                    current_module.addimportedsym(entry^.initpd.procsym);
                end
              else
                unitinits.emit_tai(Tai_const.Create_nil_codeptr,voidcodepointertype);
              if assigned(entry^.finipd) then
                begin
                  unitinits.emit_procdef_const(entry^.finipd);
                  if entry^.module<>current_module then
                    current_module.addimportedsym(entry^.finipd.procsym);
                end
              else
                unitinits.emit_tai(Tai_const.Create_nil_codeptr,voidcodepointertype);
            end
          else
            begin
              nameinit:='';
              namefini:='';
              if entry^.initfunc<>'' then
                begin
                  nameinit:=entry^.initfunc;
                  unitinits.emit_tai(
                    Tai_const.Createname(nameinit,AT_FUNCTION,0),
                    voidcodepointertype);
                end
              else
                unitinits.emit_tai(Tai_const.Create_nil_codeptr,voidcodepointertype);
              if entry^.finifunc<>'' then
                begin
                  namefini:=entry^.finifunc;
                  unitinits.emit_tai(
                    Tai_const.Createname(namefini,AT_FUNCTION,0),
                    voidcodepointertype);
                end
              else
                unitinits.emit_tai(Tai_const.Create_nil_codeptr,voidcodepointertype);
              if entry^.module<>current_module then
                add_initfinal_import(entry^.module.localsymtable);
            end;
        end;

      { Add to data segment }
      tabledef:=unitinits.end_anonymous_record;
      current_asmdata.asmlists[al_globals].concatlist(
        unitinits.get_final_asmlist(
          current_asmdata.DefineAsmSymbol('INITFINAL',AB_GLOBAL,AT_DATA,tabledef),
          tabledef,
          sec_data,'INITFINAL',sizeof(pint)
        )
      );

      unitinits.free;
    end;


  class procedure tnodeutils.InsertThreadvarTablesTable;
    var
      hp : tused_unit;
      tcb: ttai_typedconstbuilder;
      count: longint;
      sym: tasmsymbol;
      placeholder: ttypedconstplaceholder;
      tabledef: tdef;
    begin
      if (tf_section_threadvars in target_info.flags) then
        exit;
      count:=0;
      tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
      tcb.begin_anonymous_record('',1,sizeof(pint),
        targetinfos[target_info.system]^.alignment.recordalignmin
      );
      placeholder:=tcb.emit_placeholder(u32inttype);

      hp:=tused_unit(usedunits.first);
      while assigned(hp) do
       begin
         if mf_threadvars in hp.u.moduleflags then
           begin
             sym:=current_asmdata.RefAsmSymbol(make_mangledname('THREADVARLIST',hp.u.globalsymtable,''),AT_DATA,true);
             tcb.emit_tai(
               tai_const.Create_sym(sym),
               voidpointertype);
             current_module.add_extern_asmsym(sym);
             inc(count);
           end;
         hp:=tused_unit(hp.next);
       end;
      { Add program threadvars, if any }
      if mf_threadvars in current_module.moduleflags then
        begin
          sym:=current_asmdata.RefAsmSymbol(make_mangledname('THREADVARLIST',current_module.localsymtable,''),AT_DATA,true);
          tcb.emit_tai(
            Tai_const.Create_sym(sym),
            voidpointertype);
          inc(count);
        end;
      { set the count at the start }
      placeholder.replace(tai_const.Create_32bit(count),u32inttype);
      placeholder.free;
      { insert in data segment }
      tabledef:=tcb.end_anonymous_record;
      sym:=current_asmdata.DefineAsmSymbol('FPC_THREADVARTABLES',AB_GLOBAL,AT_DATA,tabledef);
      current_asmdata.asmlists[al_globals].concatlist(
        tcb.get_final_asmlist(
          sym,tabledef,sec_data,'FPC_THREADVARTABLES',sizeof(pint)
        )
      );
      tcb.free;
    end;



  procedure AddToThreadvarList(p:TObject;arg:pointer);
    var
      tcb: ttai_typedconstbuilder;
      field1, field2: tsym;
    begin
      if (tsym(p).typ=staticvarsym) and
         (vo_is_thread_var in tstaticvarsym(p).varoptions) then
       begin
         tcb:=ttai_typedconstbuilder(arg);
         { address of threadvar }
         tcb.emit_tai(tai_const.Createname(tstaticvarsym(p).mangledname,0),
           cpointerdef.getreusable(
             get_threadvar_record(tstaticvarsym(p).vardef,field1,field2)
           )
         );
         { size of threadvar }
         tcb.emit_ord_const(tstaticvarsym(p).getsize,u32inttype);
       end;
    end;


  class procedure tnodeutils.InsertThreadvars;
    var
      s : string;
      tcb: ttai_typedconstbuilder;
      sym: tasmsymbol;
      tabledef: trecorddef;
      add : boolean;
    begin
       if (tf_section_threadvars in target_info.flags) then
         exit;
       tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
       tabledef:=tcb.begin_anonymous_record('',1,sizeof(pint),
         targetinfos[target_info.system]^.alignment.recordalignmin);
       if assigned(current_module.globalsymtable) then
         current_module.globalsymtable.SymList.ForEachCall(@AddToThreadvarList,tcb);
       current_module.localsymtable.SymList.ForEachCall(@AddToThreadvarList,tcb);
       if trecordsymtable(tabledef.symtable).datasize<>0 then
         { terminator }
         tcb.emit_tai(tai_const.Create_nil_dataptr,voidpointertype);
       tcb.end_anonymous_record;
       add:=trecordsymtable(tabledef.symtable).datasize<>0;
       if add then
         begin
           s:=make_mangledname('THREADVARLIST',current_module.localsymtable,'');
           sym:=current_asmdata.DefineAsmSymbol(s,AB_GLOBAL,AT_DATA_FORCEINDIRECT,tabledef);
           current_asmdata.asmlists[al_globals].concatlist(
             tcb.get_final_asmlist(sym,tabledef,sec_data,s,sizeof(pint)));
           include(current_module.moduleflags,mf_threadvars);
           current_module.add_public_asmsym(sym);
         end
       else
         s:='';
       tcb.Free;
    end;


  class procedure tnodeutils.InsertRuntimeInitsTablesTable(const prefix,tablename:string;unitflag:tmoduleflag);
    var
      hp: tused_unit;
      tcb: ttai_typedconstbuilder;
      countplaceholder: ttypedconstplaceholder;
      tabledef: tdef;
      count: longint;
    begin
      tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
      tcb.begin_anonymous_record('',default_settings.packrecords,sizeof(pint),
        targetinfos[target_info.system]^.alignment.recordalignmin
      );
      { placeholder for the count }
      countplaceholder:=tcb.emit_placeholder(sizesinttype);
      count:=0;
      hp:=tused_unit(usedunits.first);
      while assigned(hp) do
       begin
         if unitflag in hp.u.moduleflags then
          begin
            tcb.emit_tai(
              Tai_const.Createname(make_mangledname(prefix,hp.u.globalsymtable,''),0),
              voidcodepointertype);
            inc(count);
          end;
         hp:=tused_unit(hp.next);
       end;
      { Add items from program, if any }
      if unitflag in current_module.moduleflags then
       begin
         tcb.emit_tai(
           Tai_const.Createname(make_mangledname(prefix,current_module.localsymtable,''),0),
           voidcodepointertype);
         inc(count);
       end;
      { Insert TableCount at start }
      countplaceholder.replace(Tai_const.Create_sizeint(count),sizesinttype);
      countplaceholder.free;
      { insert in data segment }
      tabledef:=tcb.end_anonymous_record;
      current_asmdata.asmlists[al_globals].concatlist(
        tcb.get_final_asmlist(
          current_asmdata.DefineAsmSymbol(tablename,AB_GLOBAL,AT_DATA,tabledef),
          tabledef,
          sec_data,tablename,sizeof(pint)
        )
      );
      tcb.free;
    end;


  class procedure tnodeutils.InsertRuntimeInits(const prefix:string;list:TLinkedList;unitflag:tmoduleflag);
    var
      s: string;
      item: TTCInitItem;
      tcb: ttai_typedconstbuilder;
      rawdatadef: tdef;
    begin
      item:=TTCInitItem(list.First);
      if item=nil then
        exit;
      s:=make_mangledname(prefix,current_module.localsymtable,'');
      tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
      tcb.begin_anonymous_record('',default_settings.packrecords,sizeof(pint),
        targetinfos[target_info.system]^.alignment.recordalignmin);
      repeat
        { optimize away unused local/static symbols }
        if (item.sym.refs>0) or (item.sym.owner.symtabletype=globalsymtable) then
          begin
            { address to initialize }
            tcb.queue_init(voidpointertype);
            rawdatadef:=carraydef.getreusable(cansichartype,tstaticvarsym(item.sym).vardef.size);
            tcb.queue_vecn(rawdatadef,item.offset);
            tcb.queue_typeconvn(cpointerdef.getreusable(tstaticvarsym(item.sym).vardef),cpointerdef.getreusable(rawdatadef));
            tcb.queue_emit_staticvar(tstaticvarsym(item.sym));
            { value with which to initialize }
            tcb.emit_tai(Tai_const.Create_sym(item.datalabel),item.datadef)
          end;
        item:=TTCInitItem(item.Next);
      until item=nil;
      { end-of-list marker }
      tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
      rawdatadef:=tcb.end_anonymous_record;
      current_asmdata.asmlists[al_globals].concatList(
        tcb.get_final_asmlist(
          current_asmdata.DefineAsmSymbol(s,AB_GLOBAL,AT_DATA,rawdatadef),
          rawdatadef,sec_data,s,sizeof(pint)));
      tcb.free;
      include(current_module.moduleflags,unitflag);
    end;


  class procedure tnodeutils.InsertWideInits;
    begin
      InsertRuntimeInits('WIDEINITS',current_asmdata.WideInits,mf_wideinits);
    end;


  class procedure tnodeutils.InsertResStrInits;
    begin
      InsertRuntimeInits('RESSTRINITS',current_asmdata.ResStrInits,mf_resstrinits);
    end;


  class procedure tnodeutils.InsertWideInitsTablesTable;
    begin
      InsertRuntimeInitsTablesTable('WIDEINITS','FPC_WIDEINITTABLES',mf_wideinits);
    end;


  class procedure tnodeutils.InsertResStrTablesTable;
    begin
      InsertRuntimeInitsTablesTable('RESSTRINITS','FPC_RESSTRINITTABLES',mf_resstrinits);
    end;


  class procedure tnodeutils.InsertResourceTablesTable;
    var
      hp : tmodule;
      count : longint;
      tcb : ttai_typedconstbuilder;
      countplaceholder : ttypedconstplaceholder;
      tabledef: tdef;
    begin
      tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
      count:=0;
      hp:=tmodule(loaded_units.first);
      tcb.begin_anonymous_record('',default_settings.packrecords,sizeof(pint),
        targetinfos[target_info.system]^.alignment.recordalignmin);
      countplaceholder:=tcb.emit_placeholder(sizesinttype);
      while assigned(hp) do
        begin
          if mf_has_resourcestrings in hp.moduleflags then
            begin
              tcb.emit_tai(Tai_const.Create_sym(
                ctai_typedconstbuilder.get_vectorized_dead_strip_section_symbol_start('RESSTR',hp.localsymtable,[tcdssso_register_asmsym,tcdssso_use_indirect])),
                voidpointertype
              );
              tcb.emit_tai(Tai_const.Create_sym(
                ctai_typedconstbuilder.get_vectorized_dead_strip_section_symbol_end('RESSTR',hp.localsymtable,[tcdssso_register_asmsym,tcdssso_use_indirect])),
                voidpointertype
              );
              inc(count);
            end;
          hp:=tmodule(hp.next);
        end;
      { Insert TableCount at start }
      countplaceholder.replace(Tai_const.Create_sizeint(count),sizesinttype);
      countplaceholder.free;
      { Add to data segment }
      tabledef:=tcb.end_anonymous_record;
      current_asmdata.AsmLists[al_globals].concatList(
        tcb.get_final_asmlist(
          current_asmdata.DefineAsmSymbol('FPC_RESOURCESTRINGTABLES',AB_GLOBAL,AT_DATA,tabledef),
          tabledef,sec_rodata,'FPC_RESOURCESTRINGTABLES',sizeof(pint)
        )
      );
      tcb.free;
    end;


  class procedure tnodeutils.InsertResourceInfo(ResourcesUsed: boolean);
    var
      tcb: ttai_typedconstbuilder;
    begin
      if (target_res.id in [res_elf,res_macho,res_xcoff]) or
         { generate the FPC_RESLOCATION symbol even when using external resources,
           because in SysInit we can only reference it unconditionally }
         ((target_res.id=res_ext) and (target_info.system in systems_darwin)) then
        begin
          tcb:=ctai_typedconstbuilder.create([tcalo_new_section,tcalo_make_dead_strippable]);

          if ResourcesUsed and (target_res.id<>res_ext) then
            tcb.emit_tai(Tai_const.Createname('FPC_RESSYMBOL',0),voidpointertype)
          else
            { Nil pointer to resource information }
            tcb.emit_tai(tai_const.Create_nil_dataptr,voidpointertype);
          current_asmdata.asmlists[al_globals].concatList(
            tcb.get_final_asmlist(
              current_asmdata.DefineAsmSymbol('FPC_RESLOCATION',AB_GLOBAL,AT_DATA,voidpointertype),
              voidpointertype,
              sec_rodata,
              'FPC_RESLOCATION',
              sizeof(puint)
            )
          );

          tcb.free;
        end;
    end;


  class procedure tnodeutils.InsertMemorySizes;
    var
      tcb: ttai_typedconstbuilder;
      s: shortstring;
      sym: tasmsymbol;
      def: tdef;
    begin
      { Insert Ident of the compiler in the .fpc.version section }
      tcb:=ctai_typedconstbuilder.create([tcalo_no_dead_strip]);
      s:='FPC '+full_version_string+
        ' ['+date_string+'] for '+target_cpu_string+' - '+target_info.shortname;
{$ifdef m68k}
      { Ensure that the size of s is multiple of 2 to avoid problems
        like on m68k-amiga which has a .balignw just after,
        causes an assembler error }
      while (length(s) mod 2) <> 0 do
        s:=s+' ';
{$endif m68k}
      def:=carraydef.getreusable(cansichartype,length(s));
      tcb.maybe_begin_aggregate(def);
      tcb.emit_tai(Tai_string.Create(s),def);
      tcb.maybe_end_aggregate(def);
      sym:=current_asmdata.DefineAsmSymbol('__fpc_ident',AB_LOCAL,AT_DATA,def);
      current_asmdata.asmlists[al_globals].concatlist(
        tcb.get_final_asmlist(sym,def,sec_fpc,'version',const_align(32))
      );
      tcb.free;

      if (tf_emit_stklen in target_info.flags) or
          not(tf_no_generic_stackcheck in target_info.flags) then
        begin
          { stacksize can be specified and is now simulated }
          tcb:=ctai_typedconstbuilder.create([tcalo_new_section,tcalo_make_dead_strippable]);
          tcb.emit_tai(Tai_const.Create_int_dataptr(stacksize),ptruinttype);
          sym:=current_asmdata.DefineAsmSymbol('__stklen',AB_GLOBAL,AT_DATA,ptruinttype);
          current_asmdata.asmlists[al_globals].concatlist(
            tcb.get_final_asmlist(sym,ptruinttype,sec_data,'__stklen',sizeof(pint))
          );
          tcb.free;
        end;
{$IFDEF POWERPC}
      { AmigaOS4 "stack cookie" support }
      if ( target_info.system = system_powerpc_amiga ) then
       begin
         { this symbol is needed to ignite powerpc amigaos' }
         { stack allocation magic for us with the given stack size. }
         { note: won't work for m68k amigaos or morphos. (KB) }
         str(stacksize,s);
         s:='$STACK: '+s+#0;
         def:=carraydef.getreusable(cansichartype,length(s));
         tcb:=ctai_typedconstbuilder.create([tcalo_new_section]);
         tcb.maybe_begin_aggregate(def);
         tcb.emit_tai(Tai_string.Create(s),def);
         tcb.maybe_end_aggregate(def);
         sym:=current_asmdata.DefineAsmSymbol('__stack_cookie',AB_GLOBAL,AT_DATA,def);
         current_asmdata.asmlists[al_globals].concatlist(
           tcb.get_final_asmlist(sym,def,sec_data,'__stack_cookie',sizeof(pint))
         );
         tcb.free;
       end;
{$ENDIF POWERPC}
      { Initial heapsize }
      tcb:=ctai_typedconstbuilder.create([tcalo_new_section,tcalo_make_dead_strippable]);
      tcb.emit_tai(Tai_const.Create_int_dataptr(heapsize),ptruinttype);
      sym:=current_asmdata.DefineAsmSymbol('__heapsize',AB_GLOBAL,AT_DATA,ptruinttype);
      current_asmdata.asmlists[al_globals].concatlist(
        tcb.get_final_asmlist(sym,ptruinttype,sec_data,'__heapsize',sizeof(pint))
      );
      tcb.free;

      { allocate an initial heap on embedded systems }
      if target_info.system in systems_embedded then
        begin
          { tai_datablock cannot yet be handled via the high level typed const
            builder, because it implies the generation of a symbol, while this
            is separate in the builder }
          maybe_new_object_file(current_asmdata.asmlists[al_globals]);
          new_section(current_asmdata.asmlists[al_globals],sec_bss,'__fpc_initialheap',current_settings.alignment.varalignmax);
          current_asmdata.asmlists[al_globals].concat(tai_datablock.Create_global('__fpc_initialheap',heapsize,carraydef.getreusable(u8inttype,heapsize),AT_DATA));
        end;

      { Valgrind usage }
      tcb:=ctai_typedconstbuilder.create([tcalo_new_section,tcalo_make_dead_strippable]);
      tcb.emit_ord_const(byte(cs_gdb_valgrind in current_settings.globalswitches),u8inttype);
      sym:=current_asmdata.DefineAsmSymbol('__fpc_valgrind',AB_GLOBAL,AT_DATA,u8inttype);
      current_asmdata.asmlists[al_globals].concatlist(
        tcb.get_final_asmlist(sym,ptruinttype,sec_data,'__fpc_valgrind',sizeof(pint))
      );
      tcb.free;
    end;


  class procedure tnodeutils.InsertObjectInfo;
    begin
      { don't do anything by default }
    end;


  class procedure tnodeutils.RegisterUsedAsmSym(sym: TAsmSymbol; def: tdef; compileronly: boolean);
    begin
      { don't do anything by default }
    end;


  class procedure tnodeutils.RegisterModuleInitFunction(pd: tprocdef);
    begin
      { setinitname may generate a new section -> don't add to the
        current list, because we assume this remains a text section }
      exportlib.setinitname(current_asmdata.AsmLists[al_pure_assembler],pd.mangledname);
    end;


  class procedure tnodeutils.RegisterModuleFiniFunction(pd: tprocdef);
    begin
      exportlib.setfininame(current_asmdata.AsmLists[al_pure_assembler],pd.mangledname);
    end;


  class procedure tnodeutils.GenerateObjCImageInfo;
    var
      tcb: ttai_typedconstbuilder;
    begin
      { first 4 bytes contain version information about this section (currently version 0),
        next 4 bytes contain flags (currently only regarding whether the code in the object
        file supports or requires garbage collection)
      }
      tcb:=ctai_typedconstbuilder.create([tcalo_new_section,tcalo_no_dead_strip]);
      tcb.emit_ord_const(0,u64inttype);
      current_asmdata.asmlists[al_objc_data].concatList(
        tcb.get_final_asmlist(
          current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'_OBJC_IMAGE_INFO',AB_LOCAL,AT_DATA,u64inttype),
          u64inttype,sec_objc_image_info,'_OBJC_IMAGE_INFO',sizeof(pint)
        )
      );
      tcb.free;
    end;


   class procedure tnodeutils.add_main_procdef_paras(pd: tdef);
     var
       pvs: tparavarsym;
     begin
       { stub for calling FPC_SYSTEMMAIN from the C main -> add argc/argv/argp }
       if (tprocdef(pd).proctypeoption=potype_mainstub) and
          (target_info.system in (systems_darwin+[system_powerpc_macos]+systems_aix)) then
         begin
           pvs:=cparavarsym.create('ARGC',1,vs_const,s32inttype,[]);
           tprocdef(pd).parast.insert(pvs);
           pvs:=cparavarsym.create('ARGV',2,vs_const,cpointerdef.getreusable(charpointertype),[]);
           tprocdef(pd).parast.insert(pvs);
           pvs:=cparavarsym.create('ARGP',3,vs_const,cpointerdef.getreusable(charpointertype),[]);
           tprocdef(pd).parast.insert(pvs);
           tprocdef(pd).calcparas;
         end
       { package stub for Windows is a DLLMain }
       else if (tprocdef(pd).proctypeoption=potype_pkgstub) and
           (target_info.system in systems_all_windows+systems_nativent) then
         begin
           pvs:=cparavarsym.create('HINSTANCE',1,vs_const,uinttype,[]);
           tprocdef(pd).parast.insert(pvs);
           pvs:=cparavarsym.create('DLLREASON',2,vs_const,u32inttype,[]);
           tprocdef(pd).parast.insert(pvs);
           pvs:=cparavarsym.create('DLLPARAM',3,vs_const,voidpointertype,[]);
           tprocdef(pd).parast.insert(pvs);
           tprocdef(pd).returndef:=bool32type;
           insert_funcret_para(tprocdef(pd));
           tprocdef(pd).calcparas;
         end;
     end;


end.
