{
    Copyright (c) 2008,2015 by Peter Vreman, Florian Klaempfl and Jonas Maebe

    This units contains support for generating LLVM type info

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
  This units contains support for LLVM type info generation.

  It's based on the debug info system, since it's quite similar
}
unit llvmtype;

{$i fpcdefs.inc}
{$h+}

interface

    uses
      cclasses,globtype,
      aasmbase,aasmtai,aasmdata,
      symbase,symtype,symdef,symsym,
      aasmllvm,aasmcnst,
      finput,
      dbgbase;


    { TLLVMTypeInfo }
    type
      TLLVMTypeInfo = class(TDebugInfo)
      protected
        { using alias/external declarations it's possible to refer to the same
          assembler symbol using multiple types:
            function f(p: pointer): pointer; [public, alias: 'FPC_FUNC'];
            procedure test(p: pointer); external name 'FPC_FUNC';

          We have to insert the appropriate typecasts (per module) for LLVM in
          this case. That can only be done after all code for a module has been
          generated, as these alias declarations can appear anywhere }
        asmsymtypes: THashSet;

        function check_insert_bitcast(toplevellist: tasmlist; sym: tasmsymbol; const opdef: tdef): taillvm;
        procedure record_asmsym_def(sym: TAsmSymbol; def: tdef; redefine: boolean);
        function  get_asmsym_def(sym: TAsmSymbol): tdef;

        function record_def(def:tdef): tdef;

        procedure appenddef_array(list:TAsmList;def:tarraydef);override;
        procedure appenddef_abstractrecord(list:TAsmList;def:tabstractrecorddef);
        procedure appenddef_record(list:TAsmList;def:trecorddef);override;
        procedure appenddef_pointer(list:TAsmList;def:tpointerdef);override;
        procedure appenddef_procvar(list:TAsmList;def:tprocvardef);override;
        procedure appendprocdef(list:TAsmList;def:tprocdef);override;
        procedure appenddef_object(list:TAsmList;def: tobjectdef);override;
        procedure appenddef_classref(list: TAsmList; def: tclassrefdef);override;
        procedure appenddef_variant(list:TAsmList;def: tvariantdef);override;
        procedure appenddef_file(list:TasmList;def:tfiledef);override;

        procedure appendsym_var(list:TAsmList;sym:tabstractnormalvarsym);
        procedure appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);override;
        procedure appendsym_paravar(list:TAsmList;sym:tparavarsym);override;
        procedure appendsym_localvar(list:TAsmList;sym:tlocalvarsym);override;
        procedure appendsym_fieldvar(list:TAsmList;sym:tfieldvarsym);override;
        procedure appendsym_const(list:TAsmList;sym:tconstsym);override;
        procedure appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);override;

        procedure enum_membersyms_callback(p:TObject;arg:pointer);

        procedure collect_llvmins_info(deftypelist: tasmlist; p: taillvm);
        procedure collect_tai_info(deftypelist: tasmlist; p: tai);
        procedure collect_asmlist_info(deftypelist, asmlist: tasmlist);

        procedure insert_llvmins_typeconversions(toplevellist: tasmlist; p: taillvm);
        procedure insert_typedconst_typeconversion(toplevellist: tasmlist; p: tai_abstracttypedconst);
        procedure insert_tai_typeconversions(toplevellist: tasmlist; p: tai);
        procedure insert_asmlist_typeconversions(toplevellist, list: tasmlist);
        procedure maybe_insert_extern_sym_decl(toplevellist: tasmlist; sym: tasmsymbol; def: tdef);
        procedure update_asmlist_alias_types(list: tasmlist);

      public
        constructor Create;override;
        destructor Destroy;override;
        procedure inserttypeinfo;override;
      end;

implementation

    uses
      sysutils,cutils,cfileutl,constexp,
      version,globals,verbose,systems,
      cpubase,cgbase,paramgr,
      fmodule,nobj,
      defutil,defcmp,symconst,symtable,
      llvmbase,llvmdef
      ;

{****************************************************************************
                              TLLVMTypeInfo
****************************************************************************}

    procedure TLLVMTypeInfo.record_asmsym_def(sym: TAsmSymbol; def: tdef; redefine: boolean);
      var
        res: PHashSetItem;
      begin
        record_def(def);
        res:=asmsymtypes.FindOrAdd(@sym,sizeof(sym));
        { due to internal aliases with different signatures, we may end up with
          multiple defs for the same symbol -> use the one from the declaration,
          and insert typecasts as necessary elsewhere }
        if redefine or
           not assigned(res^.Data) then
          res^.Data:=def;
      end;


    function equal_llvm_defs(def1, def2: tdef): boolean;
      var
        def1str, def2str: TSymStr;
      begin
        if def1=def2 then
          exit(true);
        def1str:=llvmencodetypename(def1);
        def2str:=llvmencodetypename(def2);
        { normalise both type representations in case one is a procdef
          and the other is a procvardef}
        if def1.typ=procdef then
          def1str:=def1str+'*';
        if def2.typ=procdef then
          def2str:=def2str+'*';
        result:=def1str=def2str;
      end;


    function TLLVMTypeInfo.check_insert_bitcast(toplevellist: tasmlist; sym: tasmsymbol; const opdef: tdef): taillvm;
      var
        opcmpdef: tdef;
        symdef: tdef;
      begin
        result:=nil;
        case opdef.typ of
          pointerdef:
            opcmpdef:=tpointerdef(opdef).pointeddef;
          procvardef,
          procdef:
            opcmpdef:=opdef;
          else
            internalerror(2015073101);
        end;
        maybe_insert_extern_sym_decl(toplevellist, sym, opcmpdef);
        symdef:=get_asmsym_def(sym);
        if not equal_llvm_defs(symdef, opcmpdef) then
          begin
            if symdef.typ=procdef then
              symdef:=cpointerdef.getreusable(symdef);
            result:=taillvm.op_reg_size_sym_size(la_bitcast, NR_NO, cpointerdef.getreusable(symdef), sym, opdef);
          end;
      end;


    function TLLVMTypeInfo.get_asmsym_def(sym: TAsmSymbol): tdef;
      var
        res: PHashSetItem;
      begin
        res:=asmsymtypes.Find(@sym,sizeof(sym));
        { we must have a def for every used asmsym }
        if not assigned(res) or
           not assigned(res^.data) then
          internalerror(2015042701);
        result:=tdef(res^.Data);
      end;


    function TLLVMTypeInfo.record_def(def:tdef): tdef;
      begin
        result:=def;
        if def.stab_number<>0 then
          exit;
        def.stab_number:=1;
        { this is an internal llvm type }
        if def=llvm_metadatatype then
          exit;
        if def.dbg_state=dbg_state_unused then
          begin
            def.dbg_state:=dbg_state_used;
            deftowritelist.Add(def);
          end;
        defnumberlist.Add(def);
      end;


    constructor TLLVMTypeInfo.Create;
      begin
        inherited Create;
        asmsymtypes:=THashSet.Create(current_asmdata.AsmSymbolDict.Count,true,false);
      end;


    destructor TLLVMTypeInfo.Destroy;
      begin
        asmsymtypes.free;
        inherited destroy;
      end;


    procedure TLLVMTypeInfo.enum_membersyms_callback(p:TObject; arg: pointer);
      begin
        case tsym(p).typ of
          fieldvarsym:
            appendsym_fieldvar(TAsmList(arg),tfieldvarsym(p));
          else
            ;
        end;
      end;


    procedure TLLVMTypeInfo.collect_llvmins_info(deftypelist: tasmlist; p: taillvm);
      var
        opidx, paraidx: longint;
        callpara: pllvmcallpara;
      begin
        for opidx:=0 to p.ops-1 do
          case p.oper[opidx]^.typ of
            top_def:
              record_def(p.oper[opidx]^.def);
            top_tai:
              collect_tai_info(deftypelist,p.oper[opidx]^.ai);
            top_ref:
              begin
                if (p.llvmopcode<>la_br) and
                   assigned(p.oper[opidx]^.ref^.symbol) and
                   (p.oper[opidx]^.ref^.symbol.bind<>AB_TEMP) then
                  begin
                    if (opidx=4) and
                       (p.llvmopcode in [la_call,la_invoke]) then
                      record_asmsym_def(p.oper[opidx]^.ref^.symbol,tpointerdef(p.oper[3]^.def).pointeddef,false)
                    { not a named register }
                    else if (p.oper[opidx]^.ref^.refaddr<>addr_full) then
                      record_asmsym_def(p.oper[opidx]^.ref^.symbol,p.spilling_get_reg_type(opidx),false);
                  end;
              end;
            top_para:
              for paraidx:=0 to p.oper[opidx]^.paras.count-1 do
                begin
                  callpara:=pllvmcallpara(p.oper[opidx]^.paras[paraidx]);
                  record_def(callpara^.def);
                  if callpara^.typ=top_tai then
                    collect_tai_info(deftypelist,callpara^.ai);
                end;
            else
              ;
          end;
      end;


    procedure TLLVMTypeInfo.collect_tai_info(deftypelist: tasmlist; p: tai);
      var
        value: tai_abstracttypedconst;
      begin
        if not assigned(p) then
          exit;
        case p.typ of
          ait_llvmalias:
            begin
              record_asmsym_def(taillvmalias(p).newsym,taillvmalias(p).def,true);
            end;
          ait_llvmdecl:
            begin
              record_asmsym_def(taillvmdecl(p).namesym,taillvmdecl(p).def,true);
              collect_asmlist_info(deftypelist,taillvmdecl(p).initdata);
            end;
          ait_llvmins:
            collect_llvmins_info(deftypelist,taillvm(p));
          ait_typedconst:
            begin
              record_def(tai_abstracttypedconst(p).def);
              case tai_abstracttypedconst(p).adetyp of
                tck_simple:
                  collect_tai_info(deftypelist,tai_simpletypedconst(p).val);
                tck_array,tck_record:
                  for value in tai_aggregatetypedconst(p) do
                    collect_tai_info(deftypelist,value);
              end;
            end;
          else
            ;
        end;
      end;


    procedure TLLVMTypeInfo.collect_asmlist_info(deftypelist, asmlist: tasmlist);
      var
        hp: tai;
      begin
        if not assigned(asmlist) then
          exit;
        hp:=tai(asmlist.first);
        while assigned(hp) do
          begin
            collect_tai_info(deftypelist,hp);
            hp:=tai(hp.next);
          end;
      end;


    procedure TLLVMTypeInfo.insert_llvmins_typeconversions(toplevellist: tasmlist; p: taillvm);
      var
        symdef,
        opdef: tdef;
        callpara: pllvmcallpara;
        cnv: taillvm;
        i, paraidx: longint;
      begin
        case p.llvmopcode of
          la_call,
          la_invoke:
            begin
              if p.oper[4]^.typ=top_ref then
                begin
                  maybe_insert_extern_sym_decl(toplevellist,p.oper[4]^.ref^.symbol,tpointerdef(p.oper[3]^.def).pointeddef);
                  symdef:=get_asmsym_def(p.oper[4]^.ref^.symbol);
                  { the type used in the call is different from the type used to
                    declare the symbol -> insert a typecast }
                  if not equal_llvm_defs(symdef,p.oper[3]^.def) then
                    begin
                      if symdef.typ=procdef then
                        { ugly, but can't use getcopyas(procvardef) due to the
                          symtablestack not being available here (cpointerdef.getreusable
                          is hardcoded to put things in the current module's
                          symtable) and "pointer to procedure" results in the
                          correct llvm type }
                        symdef:=cpointerdef.getreusable(tprocdef(symdef));
                      cnv:=taillvm.op_reg_size_sym_size(la_bitcast,NR_NO,symdef,p.oper[4]^.ref^.symbol,p.oper[3]^.def);
                      p.loadtai(4,cnv);
                    end;
                end;
              for i:=0 to p.ops-1 do
                begin
                  if p.oper[i]^.typ=top_para then
                    begin
                      for paraidx:=0 to p.oper[i]^.paras.count-1 do
                        begin
                          callpara:=pllvmcallpara(p.oper[i]^.paras[paraidx]);
                          case callpara^.typ of
                            top_tai:
                              insert_tai_typeconversions(toplevellist,callpara^.ai);
                            top_ref:
                              begin
                                cnv:=check_insert_bitcast(toplevellist,callpara^.sym,callpara^.def);
                                if assigned(cnv) then
                                  begin
                                    callpara^.typ:=top_tai;
                                    callpara^.ai:=cnv;
                                  end;
                              end;
                            else
                              ;
                          end;
                        end;
                    end;
                end;
            end
          else if p.llvmopcode<>la_br then
            begin
              { check the types of all symbolic operands }
              for i:=0 to p.ops-1 do
                case p.oper[i]^.typ of
                  top_ref:
                    if (p.oper[i]^.ref^.refaddr<>addr_full) and
                       assigned(p.oper[i]^.ref^.symbol) and
                       (p.oper[i]^.ref^.symbol.bind<>AB_TEMP) then
                      begin
                        opdef:=p.spilling_get_reg_type(i);
                        cnv:=check_insert_bitcast(toplevellist,p.oper[i]^.ref^.symbol, opdef);
                        if assigned(cnv) then
                          p.loadtai(i, cnv);
                      end;
                  top_tai:
                    insert_tai_typeconversions(toplevellist,p.oper[i]^.ai);
                  else
                    ;
                end;
            end;
        end;
      end;


    procedure TLLVMTypeInfo.insert_typedconst_typeconversion(toplevellist: tasmlist; p: tai_abstracttypedconst);
      var
        symdef: tdef;
        cnv: taillvm;
        elementp: tai_abstracttypedconst;
      begin
        case p.adetyp of
          tck_simple:
            begin
              case tai_simpletypedconst(p).val.typ of
                ait_const:
                  if assigned(tai_const(tai_simpletypedconst(p).val).sym) and
                     not assigned(tai_const(tai_simpletypedconst(p).val).endsym) then
                    begin
                      maybe_insert_extern_sym_decl(toplevellist,tai_const(tai_simpletypedconst(p).val).sym,p.def);
                      symdef:=get_asmsym_def(tai_const(tai_simpletypedconst(p).val).sym);
                      { all references to symbols in typed constants are
                        references to the address of a global symbol (you can't
                        refer to the data itself, just like you can't initialise
                        a Pascal (typed) constant with the contents of another
                        typed constant) }
                      symdef:=cpointerdef.getreusable(symdef);
                      if not equal_llvm_defs(symdef,p.def) then
                        begin
                          cnv:=taillvm.op_reg_tai_size(la_bitcast,NR_NO,tai_simpletypedconst.create(symdef,tai_simpletypedconst(p).val),p.def);
                          tai_simpletypedconst(p).val:=cnv;
                        end;
                    end;
                else
                  insert_tai_typeconversions(toplevellist,tai_simpletypedconst(p).val);
              end;
            end;
          tck_array,
          tck_record:
            begin
              for elementp in tai_aggregatetypedconst(p) do
                insert_typedconst_typeconversion(toplevellist,elementp);
            end;
        end;
      end;


    procedure TLLVMTypeInfo.insert_tai_typeconversions(toplevellist: tasmlist; p: tai);
      begin
        if not assigned(p) then
          exit;
        case p.typ of
          ait_llvmins:
            insert_llvmins_typeconversions(toplevellist,taillvm(p));
          { can also be necessary in case someone initialises a typed const with
            the address of an external symbol aliasing one declared with a
            different type in the same mmodule. }
          ait_typedconst:
            insert_typedconst_typeconversion(toplevellist,tai_abstracttypedconst(p));
          ait_llvmdecl:
            begin
              if (ldf_definition in taillvmdecl(p).flags) and
                 (taillvmdecl(p).def.typ=procdef) and
                 assigned(tprocdef(taillvmdecl(p).def).personality) then
                maybe_insert_extern_sym_decl(toplevellist,
                  current_asmdata.RefAsmSymbol(tprocdef(taillvmdecl(p).def).personality.mangledname,AT_FUNCTION,false),
                  tprocdef(taillvmdecl(p).def).personality);
              insert_asmlist_typeconversions(toplevellist,taillvmdecl(p).initdata);
            end;
          else
            ;
        end;
      end;


    procedure TLLVMTypeInfo.insert_asmlist_typeconversions(toplevellist, list: tasmlist);
      var
        hp: tai;
      begin
        if not assigned(list) then
          exit;
        hp:=tai(list.first);
        while assigned(hp) do
          begin
            insert_tai_typeconversions(toplevellist,hp);
            hp:=tai(hp.next);
          end;
      end;


    procedure TLLVMTypeInfo.maybe_insert_extern_sym_decl(toplevellist: tasmlist; sym: tasmsymbol; def: tdef);
      var
        sec: tasmsectiontype;
        i: longint;
      begin
        { Necessery for "external" declarations for symbols not declared in the
          current unit. We can't create these declarations when the alias is
          initially generated, because the symbol may still be defined later at
          that point.

          We also do it for all other external symbol references (e.g.
          references to symbols declared in other units), because then this
          handling is centralised in one place. }
        if not(sym.declared) then
          begin
            if def.typ=procdef then
              sec:=sec_code
            else
              sec:=sec_data;
            toplevellist.Concat(taillvmdecl.createdecl(sym,def,nil,sec,def.alignment));
            record_asmsym_def(sym,def,true);
            { the external symbol may never be called, in which case the types
              of its parameters will never be process -> do it here }
            if (def.typ=procdef) then
              begin
                { can't use this condition to determine whether or not we need
                  to generate the argument defs, because this information does
                  not get reset when multiple units are compiled during a
                  single compiler invocation }
                if (tprocdef(def).has_paraloc_info=callnoside) then
                  tprocdef(def).init_paraloc_info(callerside);
                for i:=0 to tprocdef(def).paras.count-1 do
                  record_def(llvmgetcgparadef(tparavarsym(tprocdef(def).paras[i]).paraloc[callerside],true,calleeside));
                record_def(llvmgetcgparadef(tprocdef(def).funcretloc[callerside],true,calleeside));
              end;
          end;
      end;


    procedure TLLVMTypeInfo.update_asmlist_alias_types(list: tasmlist);
      var
        hp: tai;
        def: tdef;
      begin
        if not assigned(list) then
          exit;
        hp:=tai(list.first);
        while assigned(hp) do
          begin
            case hp.typ of
              ait_llvmalias:
                begin
                  { replace the def of the alias declaration with the def of
                    the aliased symbol -> we'll insert the appropriate type
                    conversions for all uses of this symbol in the code (since
                    every use also specifies the used type) }
                  record_asmsym_def(taillvmalias(hp).oldsym,taillvmalias(hp).def,false);
                  def:=get_asmsym_def(taillvmalias(hp).oldsym);
                  if taillvmalias(hp).def<>def then
                    begin
                      taillvmalias(hp).def:=def;
                      record_asmsym_def(taillvmalias(hp).newsym,def,true);
                    end;
                end;
              ait_llvmdecl:
                update_asmlist_alias_types(taillvmdecl(hp).initdata);
              else
                ;
            end;
            hp:=tai(hp.next);
          end;
      end;


    procedure TLLVMTypeInfo.appenddef_array(list:TAsmList;def:tarraydef);
      begin
        record_def(def);
        appenddef(list,def.elementdef);
      end;


    procedure TLLVMTypeInfo.appenddef_abstractrecord(list:TAsmList;def:tabstractrecorddef);
      var
        symdeflist: tfpobjectlist;
        i: longint;
      begin
        record_def(def);
        symdeflist:=tabstractrecordsymtable(def.symtable).llvmst.symdeflist;
        for i:=0 to symdeflist.Count-1 do
          record_def(tllvmshadowsymtableentry(symdeflist[i]).def);
        list.concat(taillvm.op_size(LA_TYPE,record_def(def)));
      end;


    procedure TLLVMTypeInfo.appenddef_record(list:TAsmList;def:trecorddef);
      begin
        appenddef_abstractrecord(list,def);
      end;


    procedure TLLVMTypeInfo.appenddef_pointer(list:TAsmList;def:tpointerdef);
      begin
        record_def(def);
        appenddef(list,def.pointeddef);
      end;


    procedure TLLVMTypeInfo.appenddef_procvar(list:TAsmList;def:tprocvardef);
      var
        i: longint;
      begin
        record_def(def);
        { todo: handle mantis #25551; there is no way to create a symbolic
          la_type for a procvardef (unless it's a procedure of object/record),
          which means that recursive references should become plain "procedure"
          types that are then casted to the real type when they are used }
        def.init_paraloc_info(callerside);
        for i:=0 to def.paras.count-1 do
          appenddef(list,llvmgetcgparadef(tparavarsym(def.paras[i]).paraloc[callerside],true,calleeside));
        appenddef(list,llvmgetcgparadef(def.funcretloc[callerside],true,calleeside));
        if not def.is_addressonly then
          list.concat(taillvm.op_size(LA_TYPE,record_def(def)));
      end;


    procedure TLLVMTypeInfo.appendprocdef(list:TAsmList;def:tprocdef);
      begin
        { the procdef itself is already written by appendprocdef_implicit }
      
        { last write the types from this procdef }
        if assigned(def.parast) then
          write_symtable_defs(current_asmdata.asmlists[al_start],def.parast);
        if assigned(def.localst) and
           (def.localst.symtabletype=localsymtable) then
          write_symtable_defs(current_asmdata.asmlists[al_start],def.localst);
      end;


    procedure TLLVMTypeInfo.appendsym_var(list:TAsmList;sym:tabstractnormalvarsym);
      begin
        appenddef(list,sym.vardef);
      end;


    procedure TLLVMTypeInfo.appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);
      begin
        appendsym_var(list,sym);
      end;


    procedure TLLVMTypeInfo.appendsym_localvar(list:TAsmList;sym:tlocalvarsym);
      begin
        appendsym_var(list,sym);
      end;


    procedure TLLVMTypeInfo.appendsym_paravar(list:TAsmList;sym:tparavarsym);
      begin
        appendsym_var(list,sym);
      end;


    procedure TLLVMTypeInfo.appendsym_fieldvar(list:TAsmList;sym: tfieldvarsym);
      begin
        appenddef(list,sym.vardef);
      end;


    procedure TLLVMTypeInfo.appendsym_const(list:TAsmList;sym:tconstsym);
      begin
        appenddef(list,sym.constdef);
      end;


    procedure TLLVMTypeInfo.appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);
      begin
        appenddef(list,sym.vardef);
      end;


    procedure TLLVMTypeInfo.inserttypeinfo;

      procedure write_defs_to_write;
        var
          n       : integer;
          looplist,
          templist: TFPObjectList;
          def     : tdef;
        begin
          templist := TFPObjectList.Create(False);
          looplist := deftowritelist;
          while looplist.count > 0 do
            begin
              deftowritelist := templist;
              for n := 0 to looplist.count - 1 do
                begin
                  def := tdef(looplist[n]);
                  case def.dbg_state of
                    dbg_state_written:
                      continue;
                    dbg_state_writing:
                      internalerror(200610052);
                    dbg_state_unused:
                      internalerror(200610053);
                    dbg_state_used:
                      appenddef(current_asmdata.asmlists[al_start],def)
                  else
                    internalerror(200610054);
                  end;
                end;
              looplist.clear;
              templist := looplist;
              looplist := deftowritelist;
            end;
          templist.free;
        end;


      var
        storefilepos: tfileposinfo;
        def: tdef;
        i: longint;
        hal: tasmlisttype;
      begin
        if cs_no_regalloc in current_settings.globalswitches then
          exit;
        storefilepos:=current_filepos;
        current_filepos:=current_module.mainfilepos;

        defnumberlist:=TFPObjectList.create(false);
        deftowritelist:=TFPObjectList.create(false);

        { write all global/static variables, part of flaggin all required tdefs  }
        if assigned(current_module.globalsymtable) then
          write_symtable_syms(current_asmdata.asmlists[al_start],current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_syms(current_asmdata.asmlists[al_start],current_module.localsymtable);

        { write all procedures and methods, part of flagging all required tdefs }
        if assigned(current_module.globalsymtable) then
          write_symtable_procdefs(current_asmdata.asmlists[al_start],current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_procdefs(current_asmdata.asmlists[al_start],current_module.localsymtable);

        { process all llvm instructions, part of flagging all required tdefs }
        for hal:=low(TasmlistType) to high(TasmlistType) do
          if hal<>al_start then
            collect_asmlist_info(current_asmdata.asmlists[al_start],current_asmdata.asmlists[hal]);

        { update the defs of all alias declarations so they match those of the
          declarations of the symbols they alias }
        for hal:=low(TasmlistType) to high(TasmlistType) do
          if hal<>al_start then
            update_asmlist_alias_types(current_asmdata.asmlists[hal]);

        { and insert the necessary type conversions }
        for hal:=low(TasmlistType) to high(TasmlistType) do
          if hal<>al_start then
            insert_asmlist_typeconversions(
              current_asmdata.asmlists[hal],
              current_asmdata.asmlists[hal]);

        { write all used defs }
        write_defs_to_write;

        { reset all def labels }
        for i:=0 to defnumberlist.count-1 do
          begin
            def:=tdef(defnumberlist[i]);
            def.dbg_state:=dbg_state_unused;
            def.stab_number:=0;
          end;

        defnumberlist.free;
        defnumberlist:=nil;
        deftowritelist.free;
        deftowritelist:=nil;

        current_filepos:=storefilepos;
      end;


    procedure TLLVMTypeInfo.appenddef_object(list:TAsmList;def: tobjectdef);
      begin
        if is_interface(def) then
          begin
            record_def(def);
            record_def(def.vmt_def);
          end
        else
          appenddef_abstractrecord(list,def);
      end;


    procedure TLLVMTypeInfo.appenddef_classref(list: TAsmList; def: tclassrefdef);
      begin
        record_def(def);
        { can also be an objcclass, which doesn't have a vmt }
        if is_class(tclassrefdef(def).pointeddef) then
          record_def(tobjectdef(tclassrefdef(def).pointeddef).vmt_def);
      end;


    procedure TLLVMTypeInfo.appenddef_variant(list:TAsmList;def: tvariantdef);
      begin
        record_def(def);
        appenddef(list,tabstractrecorddef(search_system_type('TVARDATA').typedef));
      end;


    procedure TLLVMTypeInfo.appenddef_file(list:TAsmList;def:tfiledef);
      begin
        record_def(def);
        case tfiledef(def).filetyp of
          ft_text    :
            appenddef(list,tabstractrecorddef(search_system_type('TEXTREC').typedef));
          ft_typed,
          ft_untyped :
            appenddef(list,tabstractrecorddef(search_system_type('FILEREC').typedef));
        end;
      end;

end.
