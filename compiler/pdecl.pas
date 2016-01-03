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

<<<<<<< HEAD
    procedure const_dec(out had_generic:boolean);
    procedure consts_dec(in_structure, allow_typed_const: boolean;out had_generic:boolean);
    procedure label_dec;
    procedure type_dec(out had_generic:boolean);
    procedure types_dec(in_structure: boolean;out had_generic:boolean);
    procedure var_dec(out had_generic:boolean);
    procedure threadvar_dec(out had_generic:boolean);
    procedure property_dec;
    procedure resourcestring_dec(out had_generic:boolean);
=======
    procedure const_dec;
    procedure consts_dec(in_structure: boolean);
    procedure label_dec;
    procedure type_dec;
    procedure types_dec(in_structure: boolean);
    procedure var_dec;
    procedure threadvar_dec;
    procedure property_dec(is_classpropery: boolean);
    procedure resourcestring_dec;
>>>>>>> graemeg/cpstrnew

    { generics support }
    function parse_generic_parameters:TFPObjectList;
    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);

    { generics support }
    function parse_generic_parameters:TFPObjectList;
    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);

    { generics support }
    function parse_generic_parameters:TFPObjectList;
    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);

    { generics support }
    function parse_generic_parameters:TFPObjectList;
    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);

implementation

    uses
       SysUtils,
       { common }
       cutils,
       { global }
       globals,tokens,verbose,widestr,constexp,
       systems,aasmdata,fmodule,
       { symtable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       symconst,symbase,symtype,symcpu,symtable,symcreat,defutil,
=======
       symconst,symbase,symtype,symtable,paramgr,defutil,
>>>>>>> graemeg/cpstrnew
=======
       symconst,symbase,symtype,symtable,paramgr,defutil,
>>>>>>> graemeg/cpstrnew
=======
       symconst,symbase,symtype,symtable,paramgr,defutil,
>>>>>>> graemeg/cpstrnew
=======
       symconst,symbase,symtype,symtable,paramgr,defutil,
>>>>>>> origin/cpstrnew
       { pass 1 }
       htypechk,ninl,ncon,nobj,ngenutil,
       { parser }
       scanner,
       pbase,pexpr,ptype,ptconst,pdecsub,pdecvar,pdecobj,pgenutil,
{$ifdef jvm}
       pjvm,
{$endif}
       { cpu-information }
       cpuinfo
       ;


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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        p:=comp_expr([ef_accept_equal]);
        nodetype:=p.nodetype;
=======
        p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
        p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
        p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
        p:=comp_expr(true,false);
>>>>>>> origin/cpstrnew
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
                 values (mostly 0) }
               if not parse_generic then
                 Message(parser_e_illegal_expression);
               case tinlinenode(p).inlinenumber of
                 in_sizeof_x,
                 in_bitsizeof_x:
                   begin
                     hp:=cconstsym.create_ord(orgname,constord,0,p.resultdef);
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure const_dec(out had_generic:boolean);
      begin
        consume(_CONST);
        consts_dec(false,true,had_generic);
      end;

    procedure consts_dec(in_structure, allow_typed_const: boolean;out had_generic:boolean);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure const_dec;
      begin
        consume(_CONST);
        consts_dec(false);
      end;

    procedure consts_dec(in_structure: boolean);
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
      var
         orgname : TIDString;
         hdef : tdef;
         sym, tmp : tsym;
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
         static_name : string;
         sl : tpropaccesslist;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   sym:=readconstant(orgname,filepos,nodetype);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                   sym:=readconstant(orgname,filepos);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       sym:=cfieldvarsym.create(orgname,varspez,hdef,[],true);
                       symtablestack.top.insert(sym);
                       sym:=make_field_static(symtablestack.top,tfieldvarsym(sym));
                     end
                   else
                     begin
                       sym:=cstaticvarsym.create(orgname,varspez,hdef,[],true);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                       { generate the symbol which reserves the space }
                       static_name:=lower(generate_nested_name(symtablestack.top,'_'))+'_'+orgname;
                       sym:=tstaticvarsym.create('$_static_'+static_name,varspez,hdef,[]);
                       include(sym.symoptions,sp_internal);
                       tabstractrecordsymtable(symtablestack.top).get_unit_symtable.insert(sym);
                       { generate the symbol for the access }
                       sl:=tpropaccesslist.create;
                       sl.addsym(sl_load,sym);
                       tmp:=tabsolutevarsym.create_ref(orgname,hdef,sl);
                       tmp.visibility:=symtablestack.top.currentvisibility;
                       symtablestack.top.insert(tmp);
                     end
                   else
                     begin
                       sym:=tstaticvarsym.create(orgname,varspez,hdef,[]);
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
                       sym.visibility:=symtablestack.top.currentvisibility;
                       symtablestack.top.insert(sym);
                     end;
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
                      handle_calling_convention(tabstractprocdef(hdef));
                    end;
                   if not skipequal then
                    begin
                      { get init value }
                      consume(_EQ);
                      if (cs_typed_const_writable in current_settings.localswitches) then
                        tclist:=current_asmdata.asmlists[al_typedconsts]
                      else
<<<<<<< HEAD
                        tclist:=current_asmdata.asmlists[al_rotypedconsts];
=======
                        tclist:=current_asmdata.asmlists[al_typedconsts];
<<<<<<< HEAD
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
                      read_typed_const(tclist,tstaticvarsym(sym),in_structure);
=======
                      read_typed_const(tclist,tstaticvarsym(sym));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                    end;
                end;

              else
<<<<<<< HEAD
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
=======
                { generate an error }
                consume(_EQ);
           end;
         until (token<>_ID)or(in_structure and (idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]));
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  labelsym:=clabelsym.create(orgpattern)
                else
                  labelsym:=clabelsym.create(pattern);
=======
                  labelsym:=tlabelsym.create(orgpattern)
                else
                  labelsym:=tlabelsym.create(pattern);
>>>>>>> graemeg/cpstrnew
=======
                  labelsym:=tlabelsym.create(orgpattern)
                else
                  labelsym:=tlabelsym.create(pattern);
>>>>>>> graemeg/cpstrnew
=======
                  labelsym:=tlabelsym.create(orgpattern)
                else
                  labelsym:=tlabelsym.create(pattern);
>>>>>>> graemeg/cpstrnew
=======
                  labelsym:=tlabelsym.create(orgpattern)
                else
                  labelsym:=tlabelsym.create(pattern);
>>>>>>> origin/cpstrnew
                symtablestack.top.insert(labelsym);
                if m_non_local_goto in current_settings.modeswitches then
                  begin
                    if symtablestack.top.symtabletype=localsymtable then
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        labelsym.jumpbuf:=clocalvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[],true);
=======
                        labelsym.jumpbuf:=tlocalvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
>>>>>>> graemeg/cpstrnew
=======
                        labelsym.jumpbuf:=tlocalvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
>>>>>>> graemeg/cpstrnew
=======
                        labelsym.jumpbuf:=tlocalvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
>>>>>>> graemeg/cpstrnew
=======
                        labelsym.jumpbuf:=tlocalvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
>>>>>>> origin/cpstrnew
                        symtablestack.top.insert(labelsym.jumpbuf);
                      end
                    else
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        labelsym.jumpbuf:=cstaticvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[],true);
                        symtablestack.top.insert(labelsym.jumpbuf);
                        cnodeutils.insertbssdata(tstaticvarsym(labelsym.jumpbuf));
=======
                        labelsym.jumpbuf:=tstaticvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
                        symtablestack.top.insert(labelsym.jumpbuf);
                        insertbssdata(tstaticvarsym(labelsym.jumpbuf));
>>>>>>> graemeg/cpstrnew
=======
                        labelsym.jumpbuf:=tstaticvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
                        symtablestack.top.insert(labelsym.jumpbuf);
                        insertbssdata(tstaticvarsym(labelsym.jumpbuf));
>>>>>>> graemeg/cpstrnew
=======
                        labelsym.jumpbuf:=tstaticvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
                        symtablestack.top.insert(labelsym.jumpbuf);
                        insertbssdata(tstaticvarsym(labelsym.jumpbuf));
>>>>>>> graemeg/cpstrnew
=======
                        labelsym.jumpbuf:=tstaticvarsym.create('LABEL$_'+labelsym.name,vs_value,rec_jmp_buf,[]);
                        symtablestack.top.insert(labelsym.jumpbuf);
                        insertbssdata(tstaticvarsym(labelsym.jumpbuf));
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure types_dec(in_structure: boolean;out had_generic:boolean);

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
=======
    function parse_generic_parameters:TFPObjectList;
    var
      generictype : ttypesym;
    begin
      result:=TFPObjectList.Create(false);
      repeat
        if token=_ID then
          begin
            generictype:=ttypesym.create(orgpattern,cundefinedtype);
            include(generictype.symoptions,sp_generic_para);
            result.add(generictype);
          end;
        consume(_ID);
      until not try_to_consume(_COMMA) ;
    end;

    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);
      var
        i: longint;
        generictype: ttypesym;
        st: tsymtable;
      begin
        def.genericdef:=genericdef;
        if not assigned(genericlist) then
          exit;

        case def.typ of
          recorddef,objectdef: st:=tabstractrecorddef(def).symtable;
          arraydef: st:=tarraydef(def).symtable;
          procvardef,procdef: st:=tabstractprocdef(def).parast;
          else
            internalerror(201101020);
=======
    function parse_generic_parameters:TFPObjectList;
    var
      generictype : ttypesym;
    begin
      result:=TFPObjectList.Create(false);
      repeat
        if token=_ID then
          begin
            generictype:=ttypesym.create(orgpattern,cundefinedtype);
            include(generictype.symoptions,sp_generic_para);
            result.add(generictype);
          end;
        consume(_ID);
      until not try_to_consume(_COMMA) ;
    end;

    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);
      var
        i: longint;
        generictype: ttypesym;
        st: tsymtable;
      begin
        def.genericdef:=genericdef;
        if not assigned(genericlist) then
          exit;

        case def.typ of
          recorddef,objectdef: st:=tabstractrecorddef(def).symtable;
          arraydef: st:=tarraydef(def).symtable;
          procvardef,procdef: st:=tabstractprocdef(def).parast;
          else
            internalerror(201101020);
        end;

        for i:=0 to genericlist.count-1 do
          begin
            generictype:=ttypesym(genericlist[i]);
            if generictype.typedef.typ=undefineddef then
              include(def.defoptions,df_generic)
            else
              include(def.defoptions,df_specialization);
            st.insert(generictype);
          end;
       end;

    procedure types_dec(in_structure: boolean);

      procedure finalize_objc_class_or_protocol_external_status(od: tobjectdef);
        begin
          if  [oo_is_external,oo_is_forward] <= od.objectoptions then
            begin
              { formal definition: x = objcclass external; }
              exclude(od.objectoptions,oo_is_forward);
              include(od.objectoptions,oo_is_formal);
            end;
>>>>>>> origin/cpstrnew
        end;

        for i:=0 to genericlist.count-1 do
          begin
            generictype:=ttypesym(genericlist[i]);
            if generictype.typedef.typ=undefineddef then
              include(def.defoptions,df_generic)
            else
              include(def.defoptions,df_specialization);
            st.insert(generictype);
          end;
       end;

    procedure types_dec(in_structure: boolean);

      procedure finalize_objc_class_or_protocol_external_status(od: tobjectdef);
        begin
<<<<<<< HEAD
          if  [oo_is_external,oo_is_forward] <= od.objectoptions then
            begin
              { formal definition: x = objcclass external; }
              exclude(od.objectoptions,oo_is_forward);
              include(od.objectoptions,oo_is_formal);
>>>>>>> graemeg/cpstrnew
            end;
        end;

      procedure finalize_class_external_status(od: tobjectdef);
        begin
=======
    function parse_generic_parameters:TFPObjectList;
    var
      generictype : ttypesym;
    begin
      result:=TFPObjectList.Create(false);
      repeat
        if token=_ID then
          begin
            generictype:=ttypesym.create(orgpattern,cundefinedtype);
            include(generictype.symoptions,sp_generic_para);
            result.add(generictype);
          end;
        consume(_ID);
      until not try_to_consume(_COMMA) ;
    end;

    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);
      var
        i: longint;
        generictype: ttypesym;
        st: tsymtable;
      begin
        def.genericdef:=genericdef;
        if not assigned(genericlist) then
          exit;

        case def.typ of
          recorddef,objectdef: st:=tabstractrecorddef(def).symtable;
          arraydef: st:=tarraydef(def).symtable;
          procvardef,procdef: st:=tabstractprocdef(def).parast;
          else
            internalerror(201101020);
        end;

        for i:=0 to genericlist.count-1 do
          begin
            generictype:=ttypesym(genericlist[i]);
            if generictype.typedef.typ=undefineddef then
              include(def.defoptions,df_generic)
            else
              include(def.defoptions,df_specialization);
            st.insert(generictype);
          end;
       end;

    procedure types_dec(in_structure: boolean);

      procedure finalize_objc_class_or_protocol_external_status(od: tobjectdef);
        begin
>>>>>>> graemeg/cpstrnew
=======
    function parse_generic_parameters:TFPObjectList;
    var
      generictype : ttypesym;
    begin
      result:=TFPObjectList.Create(false);
      repeat
        if token=_ID then
          begin
            generictype:=ttypesym.create(orgpattern,cundefinedtype);
            include(generictype.symoptions,sp_generic_para);
            result.add(generictype);
          end;
        consume(_ID);
      until not try_to_consume(_COMMA) ;
    end;

    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:TFPObjectList);
      var
        i: longint;
        generictype: ttypesym;
        st: tsymtable;
      begin
        def.genericdef:=genericdef;
        if not assigned(genericlist) then
          exit;

        case def.typ of
          recorddef,objectdef: st:=tabstractrecorddef(def).symtable;
          arraydef: st:=tarraydef(def).symtable;
          procvardef,procdef: st:=tabstractprocdef(def).parast;
          else
            internalerror(201101020);
        end;

        for i:=0 to genericlist.count-1 do
          begin
            generictype:=ttypesym(genericlist[i]);
            if generictype.typedef.typ=undefineddef then
              include(def.defoptions,df_generic)
            else
              include(def.defoptions,df_specialization);
            st.insert(generictype);
          end;
       end;

    procedure types_dec(in_structure: boolean);

      procedure finalize_objc_class_or_protocol_external_status(od: tobjectdef);
        begin
>>>>>>> graemeg/cpstrnew
          if  [oo_is_external,oo_is_forward] <= od.objectoptions then
            begin
              { formal definition: x = objcclass external; }
              exclude(od.objectoptions,oo_is_forward);
              include(od.objectoptions,oo_is_formal);
            end;
=======
          result:=TFPObjectList.Create(false);
          repeat
            if token=_ID then
              begin
                generictype:=ttypesym.create(orgpattern,cundefinedtype);
                include(generictype.symoptions,sp_generic_para);
                result.add(generictype);
              end;
            consume(_ID);
          until not try_to_consume(_COMMA) ;
>>>>>>> graemeg/fixes_2_2
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
         vmtbuilder : TVMTBuilder;
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

           { fpc generic declaration? }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           if first then
             had_generic:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_GENERIC);
           isgeneric:=had_generic;
=======
           isgeneric:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_GENERIC);
>>>>>>> graemeg/cpstrnew
=======
           isgeneric:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_GENERIC);
>>>>>>> graemeg/cpstrnew
=======
           isgeneric:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_GENERIC);
>>>>>>> graemeg/cpstrnew
=======
           isgeneric:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_GENERIC);
>>>>>>> origin/cpstrnew

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           sym:=tsym(symtablestack.top.find(gentypename));
=======
           sym:=tsym(symtablestack.top.find(typename));
>>>>>>> graemeg/cpstrnew
=======
           sym:=tsym(symtablestack.top.find(typename));
>>>>>>> graemeg/cpstrnew
=======
           sym:=tsym(symtablestack.top.find(typename));
>>>>>>> graemeg/cpstrnew
=======
           sym:=tsym(symtablestack.top.find(typename));
>>>>>>> origin/cpstrnew
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
                          else
                            internalerror(2010122611);
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
<<<<<<< HEAD
              if isgeneric then
                begin
                  { for generics we need to check whether a non-generic type
                    already exists and if not we need to insert a symbol with
                    the non-generic name (available in (org)typename) that is a
                    undefineddef, so that inline specializations can be used }
                  sym:=tsym(symtablestack.top.Find(typename));
                  if not assigned(sym) then
                    begin
                      sym:=ctypesym.create(orgtypename,cundefineddef.create(true),true);
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
                      { we need to find this symbol even if it's a variable or
                        something else when doing an inline specialization }
                      Include(sym.symoptions,sp_generic_dummy);
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
                      sym:=nil;
                    end;

                  { determine the generic def in case we are in a nested type
                    of a specialization }
                  gendef:=determine_generic_def(gentypename);
                end;
              { insert a new type if we don't reuse an existing symbol }
              if not assigned(newtype) then
                begin
                  newtype:=ctypesym.create(genorgtypename,hdef,true);
                  newtype.visibility:=symtablestack.top.currentvisibility;
                  symtablestack.top.insert(newtype);
                end;
=======
              newtype:=ttypesym.create(orgtypename,hdef);
              newtype.visibility:=symtablestack.top.currentvisibility;
              symtablestack.top.insert(newtype);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      if is_objc_class_or_protocol(hdef) or
                         is_java_class_or_interface(hdef) then
                        Message(parser_e_unique_unsupported);
=======
                      if is_objc_class_or_protocol(hdef) then
                        Message(parser_e_no_objc_unique);
>>>>>>> graemeg/cpstrnew
=======
                      if is_objc_class_or_protocol(hdef) then
                        Message(parser_e_no_objc_unique);
>>>>>>> graemeg/cpstrnew
=======
                      if is_objc_class_or_protocol(hdef) then
                        Message(parser_e_no_objc_unique);
>>>>>>> graemeg/cpstrnew
=======
                      if is_objc_class_or_protocol(hdef) then
                        Message(parser_e_no_objc_unique);
>>>>>>> origin/cpstrnew

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

                      { fix name, it is used e.g. for tables }
                      if is_class_or_interface_or_dispinterface(hdef) then
                        with tobjectdef(hdef) do
                          begin
                            stringdispose(objname);
                            stringdispose(objrealname);
                            objrealname:=stringdup(genorgtypename);
                            objname:=stringdup(upper(genorgtypename));
                          end;

                      include(hdef.defoptions,df_unique);
                      if (hdef.typ in [pointerdef,classrefdef]) and
                         (tabstractpointerdef(hdef).pointeddef.typ=forwarddef) then
                        current_module.checkforwarddefs.add(hdef);
                    end;
                  if not assigned(hdef.typesym) then
                    hdef.typesym:=newtype;
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
                               if (po_hascallingconvention in tprocvardef(hdef).procoptions) and
                                  (tprocvardef(hdef).proccalloption=pocall_cdecl) then
                                 begin
                                   include(tprocvardef(hdef).procoptions,po_is_block);
                                   { can't check yet whether the parameter types
                                     are valid for a block, since some of them
                                     may still be forwarddefs }
                                 end
                               else
                                 { a regular anonymous function type: not yet supported }
                                 { the }
                                 Comment(V_Error,'Function references are not yet supported, only C blocks (add "cdecl;" at the end)');
                             end
                         end;
                       handle_calling_convention(tprocvardef(hdef));
                       if try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg) then
                         consume(_SEMICOLON);
                     end;
                  end;
                objectdef :
                  begin
                    try_consume_hintdirective(newtype.symoptions,newtype.deprecatedmsg);
                    consume(_SEMICOLON);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    { change a forward and external class declaration into
                      formal external definition, so the compiler does not
                      expect an real definition later }
                    if is_objc_class_or_protocol(hdef) or
                       is_java_class_or_interface(hdef) then
                      finalize_class_external_status(tobjectdef(hdef));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    { change a forward and external objcclass declaration into
                      formal external definition, so the compiler does not
                      expect an real definition later }
                    if is_objc_class_or_protocol(hdef) then
                      finalize_objc_class_or_protocol_external_status(tobjectdef(hdef));
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

                    { Build VMT indexes, skip for type renaming and forward classes }
                    if (hdef.typesym=newtype) and
                       not(oo_is_forward in tobjectdef(hdef).objectoptions) then
                      begin
                        vmtbuilder:=TVMTBuilder.Create(tobjectdef(hdef));
                        vmtbuilder.generate_vmt;
                        vmtbuilder.free;
                      end;

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
                      tobjectdef(hdef).finish_objc_data;

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
            end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           if isgeneric and (not(hdef.typ in [objectdef,recorddef,arraydef,procvardef])
               or is_objectpascal_helper(hdef)) then
=======
           if isgeneric and not(hdef.typ in [objectdef,recorddef,arraydef,procvardef]) then
>>>>>>> graemeg/cpstrnew
=======
           if isgeneric and not(hdef.typ in [objectdef,recorddef,arraydef,procvardef]) then
>>>>>>> graemeg/cpstrnew
=======
           if isgeneric and not(hdef.typ in [objectdef,recorddef,arraydef,procvardef]) then
>>>>>>> graemeg/cpstrnew
=======
           if isgeneric and not(hdef.typ in [objectdef,recorddef,arraydef,procvardef]) then
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

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
         until (token<>_ID) or
               (in_structure and
                ((idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]) or
                 ((m_final_fields in current_settings.modeswitches) and
                  (idtoken=_FINAL))));
=======
         until (token<>_ID)or(in_structure and (idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]));
>>>>>>> graemeg/cpstrnew
=======
         until (token<>_ID)or(in_structure and (idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]));
>>>>>>> graemeg/cpstrnew
=======
         until (token<>_ID)or(in_structure and (idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]));
>>>>>>> graemeg/cpstrnew
=======
         until (token<>_ID)or(in_structure and (idtoken in [_PRIVATE,_PROTECTED,_PUBLIC,_PUBLISHED,_STRICT]));
>>>>>>> origin/cpstrnew
         { resolve type block forward declarations and restore a unit
           container for them }
         resolve_forward_types;
         current_module.checkforwarddefs.free;
         current_module.checkforwarddefs:=old_checkforwarddefs;
=======
=======
>>>>>>> origin/fixes_2_2
           if assigned(generictypelist) then
             generictypelist.free;
         until token<>_ID;
         typecanbeforward:=false;
         symtablestack.top.SymList.ForEachCall(@resolve_type_forward,nil);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         block_type:=old_block_type;
      end;


    { reads a type declaration to the symbol table }
    procedure type_dec(out had_generic:boolean);
      begin
        consume(_TYPE);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        types_dec(false,had_generic);
=======
        types_dec(false);
>>>>>>> graemeg/cpstrnew
=======
        types_dec(false);
>>>>>>> graemeg/cpstrnew
=======
        types_dec(false);
>>>>>>> graemeg/cpstrnew
=======
        types_dec(false);
>>>>>>> origin/cpstrnew
      end;


    procedure var_dec(out had_generic:boolean);
    { parses variable declarations and inserts them in }
    { the top symbol table of symtablestack         }
      begin
        consume(_VAR);
        read_var_decls([vd_check_generic],had_generic);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure property_dec;
    { parses a global property (fpc mode feature) }
=======
    procedure property_dec(is_classpropery: boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure property_dec(is_classpropery: boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure property_dec(is_classpropery: boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure property_dec(is_classpropery: boolean);
>>>>>>> origin/cpstrnew
      var
         old_block_type: tblock_type;
      begin
         consume(_PROPERTY);
         if not(symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) then
           message(parser_e_property_only_sgr);
         old_block_type:=block_type;
         block_type:=bt_const;
         repeat
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           read_property_dec(false, nil);
=======
           read_property_dec(is_classpropery, nil);
>>>>>>> graemeg/cpstrnew
=======
           read_property_dec(is_classpropery, nil);
>>>>>>> graemeg/cpstrnew
=======
           read_property_dec(is_classpropery, nil);
>>>>>>> graemeg/cpstrnew
=======
           read_property_dec(is_classpropery, nil);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   p:=comp_expr([ef_accept_equal]);
=======
                   p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                   p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                   p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                   p:=comp_expr(true,false);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              else
                if not first and isgeneric and
                    (token in [_PROCEDURE, _FUNCTION, _CLASS]) then
                  begin
                    had_generic:=true;
                    break;
                  end
                else
                  consume(_EQ);
=======
              else consume(_EQ);
>>>>>>> graemeg/cpstrnew
=======
              else consume(_EQ);
>>>>>>> graemeg/cpstrnew
=======
              else consume(_EQ);
>>>>>>> graemeg/cpstrnew
=======
              else consume(_EQ);
>>>>>>> origin/cpstrnew
           end;
           first:=false;
         until token<>_ID;
         block_type:=old_block_type;
      end;

end.
