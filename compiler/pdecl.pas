{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Does declaration (but not type) parsing for Free Pascal

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
unit pdecl;

{$i fpcdefs.inc}

interface

    uses
      { common }
      cclasses,
      { global }
      globtype,
      { symtable }
      symsym,symdef,
      { pass_1 }
      node;

    function  readconstant(const orgname:string;const filepos:tfileposinfo; out nodetype: tnodetype):tconstsym;

    procedure const_dec(out had_generic:boolean);
    procedure consts_dec(in_structure, allow_typed_const: boolean;out had_generic:boolean);
    procedure label_dec;
    procedure type_dec(out had_generic:boolean);
    procedure types_dec(in_structure: boolean;out had_generic:boolean;var rtti_attrs_def: trtti_attribute_list);
    procedure var_dec(out had_generic:boolean);
    procedure threadvar_dec(out had_generic:boolean);
    procedure property_dec;
    procedure resourcestring_dec(out had_generic:boolean);
    procedure parse_rttiattributes(var rtti_attrs_def:trtti_attribute_list);

implementation

    uses
       SysUtils,
       { common }
       cutils,
       { global }
       globals,tokens,verbose,widestr,constexp,
       systems,aasmdata,fmodule,compinnr,
       { symtable }
       symconst,symbase,symtype,symcpu,symcreat,defutil,defcmp,symtable,
       { pass 1 }
       ninl,ncon,nobj,ngenutil,nld,nmem,ncal,pass_1,
       { parser }
       scanner,
       pbase,pexpr,ptype,ptconst,pdecsub,pdecvar,pdecobj,pgenutil,pparautl,
{$ifdef jvm}
       pjvm,
{$endif}
       { cpu-information }
       cpuinfo
       ;

    function is_system_custom_attribute_descendant(def:tdef):boolean;
    begin
      if not assigned(class_tcustomattribute) then
        class_tcustomattribute:=tobjectdef(search_system_type('TCUSTOMATTRIBUTE').typedef);
      Result:=def_is_related(def,class_tcustomattribute);
    end;

    function readconstant(const orgname:string;const filepos:tfileposinfo; out nodetype: tnodetype):tconstsym;
      var
        hp : tconstsym;
        p : tnode;
        ps : pconstset;
        pd : pbestreal;
        pg : pguid;
        sp : pchar;
        pw : pcompilerwidestring;
        storetokenpos : tfileposinfo;
      begin
        readconstant:=nil;
        if orgname='' then
         internalerror(9584582);
        hp:=nil;
        p:=comp_expr([ef_accept_equal]);
        nodetype:=p.nodetype;
        storetokenpos:=current_tokenpos;
        current_tokenpos:=filepos;
        case p.nodetype of
           ordconstn:
             begin
               if p.resultdef.typ=pointerdef then
                 hp:=cconstsym.create_ordptr(orgname,constpointer,tordconstnode(p).value.uvalue,p.resultdef)
               else
                 hp:=cconstsym.create_ord(orgname,constord,tordconstnode(p).value,p.resultdef);
             end;
           stringconstn:
             begin
               if is_wide_or_unicode_string(p.resultdef) then
                 begin
                   initwidestring(pw);
                   copywidestring(pcompilerwidestring(tstringconstnode(p).value_str),pw);
                   hp:=cconstsym.create_wstring(orgname,constwstring,pw);
                 end
               else
                 begin
                   getmem(sp,tstringconstnode(p).len+1);
                   move(tstringconstnode(p).value_str^,sp^,tstringconstnode(p).len+1);
                   { if a non-default ansistring code page has been specified,
                     keep it }
                   if is_ansistring(p.resultdef) and
                      (tstringdef(p.resultdef).encoding<>0) then
                     hp:=cconstsym.create_string(orgname,conststring,sp,tstringconstnode(p).len,p.resultdef)
                   else
                     hp:=cconstsym.create_string(orgname,conststring,sp,tstringconstnode(p).len,nil);
                 end;
             end;
           realconstn :
             begin
                new(pd);
                pd^:=trealconstnode(p).value_real;
                hp:=cconstsym.create_ptr(orgname,constreal,pd,p.resultdef);
             end;
           setconstn :
             begin
               new(ps);
               ps^:=tsetconstnode(p).value_set^;
               hp:=cconstsym.create_ptr(orgname,constset,ps,p.resultdef);
             end;
           pointerconstn :
             begin
               hp:=cconstsym.create_ordptr(orgname,constpointer,tpointerconstnode(p).value,p.resultdef);
             end;
           niln :
             begin
               hp:=cconstsym.create_ord(orgname,constnil,0,p.resultdef);
             end;
           typen :
             begin
               if is_interface(p.resultdef) then
                begin
                  if assigned(tobjectdef(p.resultdef).iidguid) then
                   begin
                     new(pg);
                     pg^:=tobjectdef(p.resultdef).iidguid^;
                     hp:=cconstsym.create_ptr(orgname,constguid,pg,p.resultdef);
                   end
                  else
                   Message1(parser_e_interface_has_no_guid,tobjectdef(p.resultdef).objrealname^);
                end
               else
                Message(parser_e_illegal_expression);
             end;
           inlinen:
             begin
               { this situation only happens if a intrinsic is parsed that has a
                 generic type as its argument. As we don't know certain
                 information about the final type yet, we need to use safe
                 values (mostly 0, except for (Bit)SizeOf()) }
               if not parse_generic then
                 Message(parser_e_illegal_expression);
               case tinlinenode(p).inlinenumber of
                 in_sizeof_x:
                   begin
                     hp:=cconstsym.create_ord(orgname,constord,1,p.resultdef);
                   end;
                 in_bitsizeof_x:
                   begin
                     hp:=cconstsym.create_ord(orgname,constord,8,p.resultdef);
                   end;
                 { add other cases here if necessary }
                 else
                   Message(parser_e_illegal_expression);
               end;
             end;
           else
             Message(parser_e_illegal_expression);
        end;
        current_tokenpos:=storetokenpos;
        p.free;
        readconstant:=hp;
      end;

    procedure const_dec(out had_generic:boolean);
      begin
        consume(_CONST);
        consts_dec(false,true,had_generic);
      end;

    procedure consts_dec(in_structure, allow_typed_const: boolean;out had_generic:boolean);
      var
         orgname : TIDString;
         hdef : tdef;
         sym : tsym;
         dummysymoptions : tsymoptions;
         deprecatedmsg : pshortstring;
         storetokenpos,filepos : tfileposinfo;
         nodetype : tnodetype;
         old_block_type : tblock_type;
         first,
         isgeneric,
         skipequal : boolean;
         tclist : tasmlist;
         varspez : tvarspez;
      begin
         old_block_type:=block_type;
         block_type:=bt_const;
         had_generic:=false;
         first:=true;
         repeat
           orgname:=orgpattern;
           filepos:=current_tokenpos;
           isgeneric:=not (m_delphi in current_settings.modeswitches) and (token=_ID) and (idtoken=_GENERIC);
           consume(_ID);
           case token of

             _EQ:
                begin
                   consume(_EQ);
                   sym:=readconstant(orgname,filepos,nodetype);
                   { Support hint directives }
                   dummysymoptions:=[];
                   deprecatedmsg:=nil;
                   try_consume_hintdirective(dummysymoptions,deprecatedmsg);
                   if assigned(sym) then
                     begin
                       sym.symoptions:=sym.symoptions+dummysymoptions;
                       sym.deprecatedmsg:=deprecatedmsg;
                       sym.visibility:=symtablestack.top.currentvisibility;
                       symtablestack.top.insert(sym);
                       sym.register_sym;
{$ifdef jvm}
                       { for the JVM target, some constants need to be
                         initialized at run time (enums, sets) -> create fake
                         typed const to do so (at least if they are visible
                         outside this routine, since we won't directly access
                         these symbols in the generated code) }
                       if (symtablestack.top.symtablelevel<normal_function_level) and
                          assigned(tconstsym(sym).constdef) and
                          (tconstsym(sym).constdef.typ in [enumdef,setdef]) then
                         jvm_add_typed_const_initializer(tconstsym(sym));
{$endif}
                     end
                   else
                     stringdispose(deprecatedmsg);
                   consume(_SEMICOLON);
                end;

             _COLON:
                begin
                   if not allow_typed_const then
                     begin
                       Message(parser_e_no_typed_const);
                       consume_all_until(_SEMICOLON);
                     end;
                   { set the blocktype first so a consume also supports a
                     caret, to support const s : ^string = nil }
                   block_type:=bt_const_type;
                   consume(_COLON);
                   read_anon_type(hdef,false);
                   block_type:=bt_const;
                   skipequal:=false;
                   { create symbol }
                   storetokenpos:=current_tokenpos;
                   current_tokenpos:=filepos;
                   if not (cs_typed_const_writable in current_settings.localswitches) then
                     varspez:=vs_const
                   else
                     varspez:=vs_value;
                   { if we are dealing with structure const then we need to handle it as a
                     structure static variable: create a symbol in unit symtable and a reference
                     to it from the structure or linking will fail }
                   if symtablestack.top.symtabletype in [recordsymtable,ObjectSymtable] then
                     begin
                       { note: we keep hdef so that we might at least read the
                               constant data correctly for error recovery }
                       check_allowed_for_var_or_const(hdef,false);
                       sym:=cfieldvarsym.create(orgname,varspez,hdef,[]);
                       symtablestack.top.insert(sym);
                       sym:=make_field_static(symtablestack.top,tfieldvarsym(sym));
                     end
                   else
                     begin
                       sym:=cstaticvarsym.create(orgname,varspez,hdef,[]);
                       sym.visibility:=symtablestack.top.currentvisibility;
                       symtablestack.top.insert(sym);
                     end;
                   sym.register_sym;
                   current_tokenpos:=storetokenpos;
                   { procvar can have proc directives, but not type references }
                   if (hdef.typ=procvardef) and
                      (hdef.typesym=nil) then
                    begin
                      { support p : procedure;stdcall=nil; }
                      if try_to_consume(_SEMICOLON) then
                       begin
                         if check_proc_directive(true) then
                          parse_var_proc_directives(sym)
                         else
                          begin
                            Message(parser_e_proc_directive_expected);
                            skipequal:=true;
                          end;
                       end
                      else
                      { support p : procedure stdcall=nil; }
                       begin
                         if check_proc_directive(true) then
                          parse_var_proc_directives(sym);
                       end;
                      { add default calling convention }
                      handle_calling_convention(tabstractprocdef(hdef),hcc_default_actions_intf);
                    end;
                   if not skipequal then
                    begin
                      { get init value }
                      consume(_EQ);
                      if (cs_typed_const_writable in current_settings.localswitches) then
                        tclist:=current_asmdata.asmlists[al_typedconsts]
                      else
                        tclist:=current_asmdata.asmlists[al_rotypedconsts];
                      read_typed_const(tclist,tstaticvarsym(sym),in_structure);
                    end;
                end;

              else
                if not first and isgeneric and (token in [_PROCEDURE,_FUNCTION,_CLASS]) then
                  begin
                    had_generic:=true;
                    break;
                  end
                else
                  { generate an error }
                  consume(_EQ);
           end;

           first:=false;
         until (token<>_ID) or
               (in_structure and
                ((idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]) or
                 ((m_final_fields in current_settings.modeswitches) and
                  (idtoken=_FINAL))));
         block_type:=old_block_type;
      end;


    procedure label_dec;
      var
        labelsym : tlabelsym;
      begin
         consume(_LABEL);
         if not(cs_support_goto in current_settings.moduleswitches) then
           Message(sym_e_goto_and_label_not_supported);
         repeat
           if not(token in [_ID,_INTCONST]) then
             consume(_ID)
           else
             begin
                if token=_ID then
                  labelsym:=clabelsym.create(orgpattern)
                else
                  labelsym:=clabelsym.create(pattern);
                symtablestack.top.insert(labelsym);
                if m_non_local_goto in current_settings.modeswitches then
                  begin
                    if symtablestack.top.symtabletype=localsymtable then
                      begin
                        labelsym.jumpbuf:=clocalvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
                        symtablestack.top.insert(labelsym.jumpbuf);
                      end
                    else
                      begin
                        labelsym.jumpbuf:=cstaticvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
                        symtablestack.top.insert(labelsym.jumpbuf);
                        cnodeutils.insertbssdata(tstaticvarsym(labelsym.jumpbuf));
                      end;
                    include(labelsym.jumpbuf.symoptions,sp_internal);
                    { the buffer will be setup later, but avoid a hint }
                    tabstractvarsym(labelsym.jumpbuf).varstate:=vs_written;
                  end;
                consume(token);
             end;
           if token<>_SEMICOLON then consume(_COMMA);
         until not(token in [_ID,_INTCONST]);
         consume(_SEMICOLON);
      end;

    function find_create_constructor(objdef:tobjectdef):tsymentry;
      begin
         while assigned(objdef) do
           begin
             result:=objdef.symtable.Find('CREATE');
             if assigned(result) then
               exit;
             objdef:=objdef.childof;
           end;
         // A class without a constructor called 'create'?!?
         internalerror(2012111101);
      end;

    procedure parse_rttiattributes(var rtti_attrs_def:trtti_attribute_list);

      function read_attr_paras:tnode;
        var
          old_block_type : tblock_type;
        begin
          if try_to_consume(_LKLAMMER) then
            begin
              { we only want constants here }
              old_block_type:=block_type;
              block_type:=bt_const;
              result:=parse_paras(false,false,_RKLAMMER);
              block_type:=old_block_type;
              consume(_RKLAMMER);
            end
          else
            result:=nil;
        end;

      var
        p,paran,pcalln,ptmp : tnode;
        i,pcount : sizeint;
        paras : array of tnode;
        od : tobjectdef;
        constrsym : tsymentry;
        typesym : ttypesym;
        parasok : boolean;
      begin
        consume(_LECKKLAMMER);

        repeat
          { Parse attribute type }
          p:=factor(false,[ef_type_only,ef_check_attr_suffix]);
          if p.nodetype=typen then
            begin
              typesym:=ttypesym(ttypenode(p).typesym);
              od:=tobjectdef(ttypenode(p).typedef);

              { Check if the attribute class is related to TCustomAttribute }
              if not is_system_custom_attribute_descendant(od) then
                begin
                  incompatibletypes(od,class_tcustomattribute);
                  read_attr_paras.free;
                  continue;
                end;

              paran:=read_attr_paras;

              { Search the tprocdef of the constructor which has to be called. }
              constrsym:=find_create_constructor(od);
              if constrsym.typ<>procsym then
                internalerror(2018102301);

              pcalln:=ccallnode.create(paran,tprocsym(constrsym),od.symtable,cloadvmtaddrnode.create(p),[],nil);
              p:=nil;
              typecheckpass(pcalln);

              if (pcalln.nodetype=calln) and assigned(tcallnode(pcalln).procdefinition) and not codegenerror then
                begin
                  { TODO: once extended RTTI for methods is supported, reject a
                          constructor if it doesn't have extended RTTI enabled }

                  { collect the parameters of the call node as there might be
                    compile time type conversions (e.g. a Byte parameter being
                    passed a value > 255) }
                  paran:=tcallnode(pcalln).left;

                  { only count visible parameters (thankfully open arrays are not
                    supported, otherwise we'd need to handle those as well) }
                  parasok:=true;
                  paras:=nil;
                  if assigned(paran) then
                    begin
                      ptmp:=paran;
                      pcount:=0;
                      while assigned(ptmp) do
                        begin
                          if not (vo_is_hidden_para in tcallparanode(ptmp).parasym.varoptions) then
                            inc(pcount);
                          ptmp:=tcallparanode(ptmp).right;
                        end;
                      setlength(paras,pcount);
                      ptmp:=paran;
                      pcount:=0;
                      while assigned(ptmp) do
                        begin
                          if not (vo_is_hidden_para in tcallparanode(ptmp).parasym.varoptions) then
                            begin
                              if not is_constnode(tcallparanode(ptmp).left) then
                                begin
                                  parasok:=false;
                                  messagepos(tcallparanode(ptmp).left.fileinfo,type_e_constant_expr_expected);
                                end;
                              paras[high(paras)-pcount]:=tcallparanode(ptmp).left.getcopy;
                              inc(pcount);
                            end;
                          ptmp:=tcallparanode(ptmp).right;
                        end;
                    end;

                  if parasok then
                    begin
                      { Add attribute to attribute list which will be added
                        to the property which is defined next. }
                      if not assigned(rtti_attrs_def) then
                        rtti_attrs_def:=trtti_attribute_list.create;
                      rtti_attrs_def.addattribute(typesym,tcallnode(pcalln).procdefinition,pcalln,paras);
                    end
                  else
                    begin
                      { cleanup }
                      pcalln.free;
                      for i:=0 to high(paras) do
                        paras[i].free;
                    end;
                end
              else
                pcalln.free;
            end
          else
            begin
              Message(type_e_type_id_expected);
              { try to recover by nevertheless reading the parameters (if any) }
              read_attr_paras.free;
            end;

          p.free;
        until not try_to_consume(_COMMA);

        consume(_RECKKLAMMER);
      end;


    { From http://clang.llvm.org/docs/LanguageExtensions.html#objective-c-features :
      To determine whether a method has an inferred related result type, the first word in the camel-case selector
      (e.g., “init” in “initWithObjects”) is considered, and the method will have a related result type if its return
      type is compatible with the type of its class and if:
        * the first word is "alloc" or "new", and the method is a class method, or
        * the first word is "autorelease", "init", "retain", or "self", and the method is an instance method.

      If a method with a related result type is overridden by a subclass method, the subclass method must also return
      a type that is compatible with the subclass type.
    }
    procedure pd_set_objc_related_result(def: tobject; para: pointer);
      var
        pd: tprocdef;
        i, firstcamelend: longint;
        inferresult: boolean;
      begin
        if tdef(def).typ<>procdef then
          exit;
        pd:=tprocdef(def);
        if not(po_msgstr in pd.procoptions) then
          internalerror(2019082401);
        firstcamelend:=length(pd.messageinf.str^);
        for i:=1 to length(pd.messageinf.str^) do
          if pd.messageinf.str^[i] in ['A'..'Z'] then
            begin
              firstcamelend:=pred(i);
              break;
            end;
        case copy(pd.messageinf.str^,1,firstcamelend) of
          'alloc',
          'new':
             inferresult:=po_classmethod in pd.procoptions;
          'autorelease',
          'init',
          'retain',
          'self':
             inferresult:=not(po_classmethod in pd.procoptions);
          else
            inferresult:=false;
        end;
        if inferresult and
           def_is_related(tdef(pd.procsym.owner.defowner),pd.returndef) then
          include(pd.procoptions,po_objc_related_result_type);
      end;

    procedure types_dec(in_structure: boolean;out had_generic:boolean;var rtti_attrs_def: trtti_attribute_list);

      function determine_generic_def(name:tidstring):tstoreddef;
        var
          hashedid : THashedIDString;
          pd : tprocdef;
          sym : tsym;
        begin
          result:=nil;
          { check whether this is a declaration of a type inside a
            specialization }
          if assigned(current_structdef) and
              (df_specialization in current_structdef.defoptions) then
            begin
              if not assigned(current_structdef.genericdef) or
                  not (current_structdef.genericdef.typ in [recorddef,objectdef]) then
                internalerror(2011052301);
              hashedid.id:=name;
              { we could be inside a method of the specialization
                instead of its declaration, so check that first (as
                local nested types aren't allowed we don't need to
                walk the symtablestack to find the localsymtable) }
              if symtablestack.top.symtabletype=localsymtable then
                begin
                  { we are in a method }
                  if not assigned(symtablestack.top.defowner) or
                      (symtablestack.top.defowner.typ<>procdef) then
                    internalerror(2011120701);
                  pd:=tprocdef(symtablestack.top.defowner);
                  if not assigned(pd.genericdef) or (pd.genericdef.typ<>procdef) then
                    internalerror(2011120702);
                  sym:=tsym(tprocdef(pd.genericdef).localst.findwithhash(hashedid));
                end
              else
                sym:=nil;
              if not assigned(sym) or not (sym.typ=typesym) then
                begin
                  { now search in the declaration of the generic }
                  sym:=tsym(tabstractrecorddef(current_structdef.genericdef).symtable.findwithhash(hashedid));
                  if not assigned(sym) or not (sym.typ=typesym) then
                    internalerror(2011052302);
                end;
              { use the corresponding type in the generic's symtable as
                genericdef for the specialized type }
              result:=tstoreddef(ttypesym(sym).typedef);
            end;
        end;

      procedure finalize_class_external_status(od: tobjectdef);
        begin
          if  [oo_is_external,oo_is_forward] <= od.objectoptions then
            begin
              { formal definition: x = objcclass external; }
              exclude(od.objectoptions,oo_is_forward);
              include(od.objectoptions,oo_is_formal);
            end;
        end;

      var
         typename,orgtypename,
         gentypename,genorgtypename : TIDString;
         newtype  : ttypesym;
         sym      : tsym;
         hdef     : tdef;
         defpos,storetokenpos : tfileposinfo;
         old_block_type : tblock_type;
         old_checkforwarddefs: TFPObjectList;
         objecttype : tobjecttyp;
         first,
         isgeneric,
         isunique,
         istyperenaming : boolean;
         generictypelist : tfphashobjectlist;
         generictokenbuf : tdynamicarray;
         p:tnode;
         gendef : tstoreddef;
         s : shortstring;
         i : longint;
{$ifdef x86}
         segment_register: string;
{$endif x86}
      begin
         old_block_type:=block_type;
         { save unit container of forward declarations -
           we can be inside nested class type block }
         old_checkforwarddefs:=current_module.checkforwarddefs;
         current_module.checkforwarddefs:=TFPObjectList.Create(false);
         block_type:=bt_type;
         hdef:=nil;
         first:=true;
         had_generic:=false;
         repeat
           defpos:=current_tokenpos;
           istyperenaming:=false;
           generictypelist:=nil;
           generictokenbuf:=nil;

           { class attribute definitions? }
           if m_prefixed_attributes in current_settings.modeswitches then
             while token=_LECKKLAMMER do
               parse_rttiattributes(rtti_attrs_def);

           { fpc generic declaration? }
           if first then
             had_generic:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_GENERIC);
           isgeneric:=had_generic;

           typename:=pattern;
           orgtypename:=orgpattern;
           consume(_ID);

           { delphi generic declaration? }
           if (m_delphi in current_settings.modeswitches) then
             isgeneric:=token=_LSHARPBRACKET;

           { Generic type declaration? }
           if isgeneric then
             begin
               if assigned(current_genericdef) then
                 Message(parser_f_no_generic_inside_generic);

               consume(_LSHARPBRACKET);
               generictypelist:=parse_generic_parameters(true);
               consume(_RSHARPBRACKET);

               { we are not freeing the type parameters, so register them }
               for i:=0 to generictypelist.count-1 do
                 begin
                    ttypesym(generictypelist[i]).register_sym;
                    tstoreddef(ttypesym(generictypelist[i]).typedef).register_def;
                 end;

               str(generictypelist.Count,s);
               gentypename:=typename+'$'+s;
               genorgtypename:=orgtypename+'$'+s;
             end
           else
             begin
               gentypename:=typename;
               genorgtypename:=orgtypename;
             end;


           consume(_EQ);

           { support 'ttype=type word' syntax }
           isunique:=try_to_consume(_TYPE);

           { MacPas object model is more like Delphi's than like TP's, but }
           { uses the object keyword instead of class                      }
           if (m_mac in current_settings.modeswitches) and
              (token = _OBJECT) then
             token := _CLASS;

           { Start recording a generic template }
           if assigned(generictypelist) then
             begin
               generictokenbuf:=tdynamicarray.create(256);
               current_scanner.startrecordtokens(generictokenbuf);
             end;

           { is the type already defined? -- must be in the current symtable,
             not in a nested symtable or one higher up the stack -> don't
             use searchsym & frinds! }
           sym:=tsym(symtablestack.top.find(gentypename));
           newtype:=nil;
           { found a symbol with this name? }
           if assigned(sym) then
            begin
              if (sym.typ=typesym) and
                 { this should not be a symbol that was created by a generic
                   that was declared earlier }
                 not (
                   (ttypesym(sym).typedef.typ=undefineddef) and
                   (sp_generic_dummy in sym.symoptions)
                 ) then
               begin
                 if ((token=_CLASS) or
                     (token=_INTERFACE) or
                     (token=_DISPINTERFACE) or
                     (token=_OBJCCLASS) or
                     (token=_OBJCPROTOCOL) or
                     (token=_OBJCCATEGORY)) and
                    (assigned(ttypesym(sym).typedef)) and
                    is_implicit_pointer_object_type(ttypesym(sym).typedef) and
                    (oo_is_forward in tobjectdef(ttypesym(sym).typedef).objectoptions) then
                  begin
                    case token of
                      _CLASS :
                        objecttype:=default_class_type;
                      _INTERFACE :
                        case current_settings.interfacetype of
                          it_interfacecom:
                            objecttype:=odt_interfacecom;
                          it_interfacecorba:
                            objecttype:=odt_interfacecorba;
                          it_interfacejava:
                            objecttype:=odt_interfacejava;
                        end;
                      _DISPINTERFACE :
                        objecttype:=odt_dispinterface;
                      _OBJCCLASS,
                      _OBJCCATEGORY :
                        objecttype:=odt_objcclass;
                      _OBJCPROTOCOL :
                        objecttype:=odt_objcprotocol;
                      else
                        internalerror(200811072);
                    end;
                    consume(token);
                    { determine the generic def in case we are in a nested type
                      of a specialization }
                    gendef:=determine_generic_def(gentypename);
                    { we can ignore the result, the definition is modified }
                    object_dec(objecttype,genorgtypename,newtype,gendef,generictypelist,tobjectdef(ttypesym(sym).typedef),ht_none);
                    newtype:=ttypesym(sym);
                    hdef:=newtype.typedef;
                  end
                 else
                  message1(parser_h_type_redef,genorgtypename);
               end;
            end;
           { no old type reused ? Then insert this new type }
           if not assigned(newtype) then
            begin
              { insert the new type first with an errordef, so that
                referencing the type before it's really set it
                will give an error (PFV) }
              hdef:=generrordef;
              gendef:=nil;
              storetokenpos:=current_tokenpos;
              if isgeneric then
                begin
                  { for generics we need to check whether a non-generic type
                    already exists and if not we need to insert a symbol with
                    the non-generic name (available in (org)typename) that is a
                    undefineddef, so that inline specializations can be used }
                  sym:=tsym(symtablestack.top.Find(typename));
                  if not assigned(sym) then
                    begin
                      sym:=ctypesym.create(orgtypename,cundefineddef.create(true));
                      Include(sym.symoptions,sp_generic_dummy);
                      ttypesym(sym).typedef.typesym:=sym;
                      sym.visibility:=symtablestack.top.currentvisibility;
                      symtablestack.top.insert(sym);
                      ttypesym(sym).typedef.owner:=sym.owner;
                    end
                  else
                    { this is not allowed in non-Delphi modes }
                    if not (m_delphi in current_settings.modeswitches) then
                      Message1(sym_e_duplicate_id,genorgtypename)
                    else
                      begin
                        { we need to find this symbol even if it's a variable or
                          something else when doing an inline specialization }
                        Include(sym.symoptions,sp_generic_dummy);
                        add_generic_dummysym(sym);
                      end;
                end
              else
                begin
                  if assigned(sym) and (sym.typ=typesym) and
                      (ttypesym(sym).typedef.typ=undefineddef) and
                      (sp_generic_dummy in sym.symoptions) then
                    begin
                      { this is a symbol that was added by an earlier generic
                        declaration, reuse it }
                      newtype:=ttypesym(sym);
                      newtype.typedef:=hdef;
                      { use the correct casing }
                      newtype.RealName:=genorgtypename;
                      sym:=nil;
                    end;

                  { determine the generic def in case we are in a nested type
                    of a specialization }
                  gendef:=determine_generic_def(gentypename);
                end;
              { insert a new type if we don't reuse an existing symbol }
              if not assigned(newtype) then
                begin
                  newtype:=ctypesym.create(genorgtypename,hdef);
                  newtype.visibility:=symtablestack.top.currentvisibility;
                  symtablestack.top.insert(newtype);
                end;
              current_tokenpos:=defpos;
              current_tokenpos:=storetokenpos;
              { read the type definition }
              read_named_type(hdef,newtype,gendef,generictypelist,false,isunique);
              { update the definition of the type }
              if assigned(hdef) then
                begin
                  if df_generic in hdef.defoptions then
                    { flag parent symtables that they now contain a generic }
                    hdef.owner.includeoption(sto_has_generic);
                  if assigned(hdef.typesym) then
                    begin
                      istyperenaming:=true;
                      include(newtype.symoptions,sp_explicitrename);
                    end;
                  if isunique then
                    begin
                      if is_objc_class_or_protocol(hdef) or
                         is_java_class_or_interface(hdef) then
                        Message(parser_e_unique_unsupported);

                      if is_object(hdef) or
                         is_class_or_interface_or_dispinterface(hdef) then
                        begin
                          { just create a child class type; this is
                            Delphi-compatible }
                          hdef:=cobjectdef.create(tobjectdef(hdef).objecttype,genorgtypename,tobjectdef(hdef),true);
                        end
                      else
                        begin
                          hdef:=tstoreddef(hdef).getcopy;
                          { check if it is an ansistirng(codepage) declaration }
                          if is_ansistring(hdef) and try_to_consume(_LKLAMMER) then
                            begin
                              p:=comp_expr([ef_accept_equal]);
                              consume(_RKLAMMER);
                              if not is_constintnode(p) then
                                begin
                                  Message(parser_e_illegal_expression);
                                  { error recovery }
                                end
                              else
                                begin
                                  if (tordconstnode(p).value<0) or (tordconstnode(p).value>65535) then
                                    begin
                                      Message(parser_e_invalid_codepage);
                                      tordconstnode(p).value:=0;
                                    end;
                                  tstringdef(hdef).encoding:=int64(tordconstnode(p).value);
                                end;
                              p.free;
                            end;
                          if (hdef.typ in [pointerdef,classrefdef]) and
                             (tabstractpointerdef(hdef).pointeddef.typ=forwarddef) then
                            current_module.checkforwarddefs.add(hdef);
                        end;
                      include(hdef.defoptions,df_unique);
                    end;
                  if not assigned(hdef.typesym) then
                    begin
                      hdef.typesym:=newtype;
                      if sp_generic_dummy in newtype.symoptions then
                        add_generic_dummysym(newtype);
                    end;
                end;
              { in non-Delphi modes we need a reference to the generic def
                without the generic suffix, so it can be found easily when
                parsing method implementations }
              if isgeneric and assigned(sym) and
                  not (m_delphi in current_settings.modeswitches) and
                  (ttypesym(sym).typedef.typ=undefineddef) then
                { don't free the undefineddef as the defids rely on the count
                  of the defs in the def list of the module}
                ttypesym(sym).typedef:=hdef;
              newtype.typedef:=hdef;
              { ensure that the type is registered when no specialization is
                currently done }
              if current_scanner.replay_stack_depth=0 then
                hdef.register_def;
              { KAZ: handle TGUID declaration in system unit }
              if (cs_compilesystem in current_settings.moduleswitches) and
                 assigned(hdef) and
                 (hdef.typ=recorddef) then
                begin
                  if not assigned(rec_tguid) and
                     (gentypename='TGUID') and
                     (hdef.size=16) then
                    rec_tguid:=trecorddef(hdef)
                  else if not assigned(rec_jmp_buf) and
                     (gentypename='JMP_BUF') then
                    rec_jmp_buf:=trecorddef(hdef)
                  else if not assigned(rec_exceptaddr) and
                     (gentypename='TEXCEPTADDR') then
                    rec_exceptaddr:=trecorddef(hdef);
                end;
            end;
           if assigned(hdef) then
            begin
              case hdef.typ of
                pointerdef :
                  begin
                    try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                    consume(_SEMICOLON);
{$ifdef x86}
  {$ifdef i8086}
                    if try_to_consume(_HUGE) then
                     begin
                       tcpupointerdef(hdef).x86pointertyp:=x86pt_huge;
                       consume(_SEMICOLON);
                     end
                    else
  {$endif i8086}
                    if try_to_consume(_FAR) then
                     begin
  {$if defined(i8086)}
                       tcpupointerdef(hdef).x86pointertyp:=x86pt_far;
  {$elseif defined(i386)}
                       tcpupointerdef(hdef).x86pointertyp:=x86pt_near_fs;
  {$elseif defined(x86_64)}
                       { for compatibility with previous versions of fpc,
                         far pointer = regular pointer on x86_64 }
                       Message1(parser_w_ptr_type_ignored,'FAR');
  {$endif}
                       consume(_SEMICOLON);
                     end
                    else
                      if try_to_consume(_NEAR) then
                       begin
                         if token <> _SEMICOLON then
                           begin
                             segment_register:=get_stringconst;
                             case UpCase(segment_register) of
                               'CS': tcpupointerdef(hdef).x86pointertyp:=x86pt_near_cs;
                               'DS': tcpupointerdef(hdef).x86pointertyp:=x86pt_near_ds;
                               'SS': tcpupointerdef(hdef).x86pointertyp:=x86pt_near_ss;
                               'ES': tcpupointerdef(hdef).x86pointertyp:=x86pt_near_es;
                               'FS': tcpupointerdef(hdef).x86pointertyp:=x86pt_near_fs;
                               'GS': tcpupointerdef(hdef).x86pointertyp:=x86pt_near_gs;
                               else
                                 Message(asmr_e_invalid_register);
                             end;
                           end
                         else
                           tcpupointerdef(hdef).x86pointertyp:=x86pt_near;
                         consume(_SEMICOLON);
                       end;
{$else x86}
                    { Previous versions of FPC support declaring a pointer as
                      far even on non-x86 platforms. }
                    if try_to_consume(_FAR) then
                     begin
                       Message1(parser_w_ptr_type_ignored,'FAR');
                       consume(_SEMICOLON);
                     end;
{$endif x86}
                  end;
                procvardef :
                  begin
                    { in case of type renaming, don't parse proc directives }
                    if istyperenaming then
                      begin
                        try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                        consume(_SEMICOLON);
                      end
                    else
                     begin
                       if not check_proc_directive(true) then
                         begin
                           try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                           consume(_SEMICOLON);
                         end;
                       parse_var_proc_directives(tsym(newtype));
                       if po_is_function_ref in tprocvardef(hdef).procoptions then
                         begin
                           { these always support everything, no "of object" or
                             "is_nested" is allowed }
                           if is_nested_pd(tprocvardef(hdef)) or
                              is_methodpointer(hdef) then
                             cgmessage(type_e_function_reference_kind)
                           else
                             begin
                               { this message is only temporary; once Delphi style anonymous functions
                                 are supported, this check is no longer required }
                               if not (po_is_block in tprocvardef(hdef).procoptions) then
                                 comment(v_error,'Function references are not yet supported, only C blocks (add "cblock;" at the end)');
                             end;
                         end;
                       handle_calling_convention(tprocvardef(hdef),hcc_default_actions_intf);
                       if po_is_function_ref in tprocvardef(hdef).procoptions then
                         begin
                           if (po_is_block in tprocvardef(hdef).procoptions) and
                              not (tprocvardef(hdef).proccalloption in [pocall_cdecl,pocall_mwpascal]) then
                             message(type_e_cblock_callconv);
                         end;
                       if try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg) then
                         consume(_SEMICOLON);
                     end;
                  end;
                objectdef :
                  begin
                    try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                    consume(_SEMICOLON);

                    { change a forward and external class declaration into
                      formal external definition, so the compiler does not
                      expect an real definition later }
                    if is_objc_class_or_protocol(hdef) or
                       is_java_class_or_interface(hdef) then
                      finalize_class_external_status(tobjectdef(hdef));

                    { Build VMT indexes, skip for type renaming and forward classes }
                    if (hdef.typesym=newtype) and
                       not(oo_is_forward in tobjectdef(hdef).objectoptions) then
                      build_vmt(tobjectdef(hdef));

                    { In case of an objcclass, verify that all methods have a message
                      name set. We only check this now, because message names can be set
                      during the protocol (interface) mapping. At the same time, set the
                      mangled names (these depend on the "external" name of the class),
                      and mark private fields of external classes as "used" (to avoid
                      bogus notes about them being unused)
                    }
                    { watch out for crashes in case of errors }
                    if is_objc_class_or_protocol(hdef) and
                       (not is_objccategory(hdef) or
                        assigned(tobjectdef(hdef).childof)) then
                      begin
                        tobjectdef(hdef).finish_objc_data;
                        tobjectdef(hdef).symtable.DefList.ForEachCall(@pd_set_objc_related_result,nil);
                      end;

                    if is_cppclass(hdef) then
                      tobjectdef(hdef).finish_cpp_data;
                  end;
                recorddef :
                  begin
                    try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                    consume(_SEMICOLON);
                  end;
                else
                  begin
                    try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                    consume(_SEMICOLON);
                  end;
              end;

              { if we have a real type definition or a unique type we may bind
                attributes to this def }
              if not istyperenaming or isunique then
                trtti_attribute_list.bind(rtti_attrs_def,tstoreddef(hdef).rtti_attribute_list);
            end;

           if isgeneric and (not(hdef.typ in [objectdef,recorddef,arraydef,procvardef])
               or is_objectpascal_helper(hdef)) then
             message(parser_e_cant_create_generics_of_this_type);

           { Stop recording a generic template }
           if assigned(generictypelist) then
             begin
               current_scanner.stoprecordtokens;
               tstoreddef(hdef).generictokenbuf:=generictokenbuf;
               { Generic is never a type renaming }
               hdef.typesym:=newtype;
               generictypelist.free;
             end;

           if not (m_delphi in current_settings.modeswitches) and
               (token=_ID) and (idtoken=_GENERIC) then
             begin
               had_generic:=true;
               consume(_ID);
               if token in [_PROCEDURE,_FUNCTION,_CLASS] then
                 break;
             end
           else
             had_generic:=false;
           first:=false;
           if assigned(rtti_attrs_def) and (rtti_attrs_def.get_attribute_count>0) then
             Message1(parser_e_unbound_attribute,trtti_attribute(rtti_attrs_def.rtti_attributes[0]).typesym.prettyname);

         until ((token<>_ID) and (token<>_LECKKLAMMER)) or
               (in_structure and
                ((idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]) or
                 ((m_final_fields in current_settings.modeswitches) and
                  (idtoken=_FINAL))));
         { resolve type block forward declarations and restore a unit
           container for them }
         resolve_forward_types;
         current_module.checkforwarddefs.free;
         current_module.checkforwarddefs:=old_checkforwarddefs;
         block_type:=old_block_type;
      end;


    { reads a type declaration to the symbol table }
    procedure type_dec(out had_generic:boolean);
      var
        rtti_attrs_def: trtti_attribute_list;
      begin
        consume(_TYPE);
        rtti_attrs_def := nil;
        types_dec(false,had_generic,rtti_attrs_def);
        rtti_attrs_def.free;
      end;


    procedure var_dec(out had_generic:boolean);
    { parses variable declarations and inserts them in }
    { the top symbol table of symtablestack         }
      begin
        consume(_VAR);
        read_var_decls([vd_check_generic],had_generic);
      end;


    procedure property_dec;
    { parses a global property (fpc mode feature) }
      var
         old_block_type: tblock_type;
      begin
         consume(_PROPERTY);
         if not(symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) then
           message(parser_e_property_only_sgr);
         old_block_type:=block_type;
         block_type:=bt_const;
         repeat
           read_property_dec(false, nil);
           consume(_SEMICOLON);
         until token<>_ID;
         block_type:=old_block_type;
      end;


    procedure threadvar_dec(out had_generic:boolean);
    { parses thread variable declarations and inserts them in }
    { the top symbol table of symtablestack                }
      begin
        consume(_THREADVAR);
        if not(symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) then
          message(parser_e_threadvars_only_sg);
        if f_threading in features then
          read_var_decls([vd_threadvar,vd_check_generic],had_generic)
        else
          begin
            Message1(parser_f_unsupported_feature,featurestr[f_threading]);
            read_var_decls([vd_check_generic],had_generic);
          end;
      end;


    procedure resourcestring_dec(out had_generic:boolean);
      var
         orgname : TIDString;
         p : tnode;
         dummysymoptions : tsymoptions;
         deprecatedmsg : pshortstring;
         storetokenpos,filepos : tfileposinfo;
         old_block_type : tblock_type;
         sp : pchar;
         sym : tsym;
         first,
         isgeneric : boolean;
      begin
         if target_info.system in systems_managed_vm then
           message(parser_e_feature_unsupported_for_vm);
         consume(_RESOURCESTRING);
         if not(symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) then
           message(parser_e_resourcestring_only_sg);
         first:=true;
         had_generic:=false;
         old_block_type:=block_type;
         block_type:=bt_const;
         repeat
           orgname:=orgpattern;
           filepos:=current_tokenpos;
           isgeneric:=not (m_delphi in current_settings.modeswitches) and (token=_ID) and (idtoken=_GENERIC);
           consume(_ID);
           case token of
             _EQ:
                begin
                   consume(_EQ);
                   p:=comp_expr([ef_accept_equal]);
                   storetokenpos:=current_tokenpos;
                   current_tokenpos:=filepos;
                   sym:=nil;
                   case p.nodetype of
                      ordconstn:
                        begin
                           if is_constcharnode(p) then
                             begin
                                getmem(sp,2);
                                sp[0]:=chr(tordconstnode(p).value.svalue);
                                sp[1]:=#0;
                                sym:=cconstsym.create_string(orgname,constresourcestring,sp,1,nil);
                             end
                           else
                             Message(parser_e_illegal_expression);
                        end;
                      stringconstn:
                        with Tstringconstnode(p) do
                          begin
                             { resourcestrings are currently always single byte }
                             if cst_type in [cst_widestring,cst_unicodestring] then
                               changestringtype(getansistringdef);
                             getmem(sp,len+1);
                             move(value_str^,sp^,len+1);
                             sym:=cconstsym.create_string(orgname,constresourcestring,sp,len,nil);
                          end;
                      else
                        Message(parser_e_illegal_expression);
                   end;
                   current_tokenpos:=storetokenpos;
                   { Support hint directives }
                   dummysymoptions:=[];
                   deprecatedmsg:=nil;
                   try_consume_hintdirective(dummysymoptions,deprecatedmsg);
                   if assigned(sym) then
                     begin
                       sym.symoptions:=sym.symoptions+dummysymoptions;
                       sym.deprecatedmsg:=deprecatedmsg;
                       symtablestack.top.insert(sym);
                     end
                   else
                     stringdispose(deprecatedmsg);
                   consume(_SEMICOLON);
                   p.free;
                end;
              else
                if not first and isgeneric and
                    (token in [_PROCEDURE, _FUNCTION, _CLASS]) then
                  begin
                    had_generic:=true;
                    break;
                  end
                else
                  consume(_EQ);
           end;
           first:=false;
         until token<>_ID;
         block_type:=old_block_type;
      end;

end.
