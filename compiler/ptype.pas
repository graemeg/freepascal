{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Does parsing types for Free Pascal

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
unit ptype;

{$i fpcdefs.inc}

interface

    uses
       globtype,cclasses,
       symtype,symdef,symbase;

    type
      TSingleTypeOption=(
        stoIsForwardDef,          { foward declaration         }
        stoAllowTypeDef,          { allow type definitions     }
        stoAllowSpecialization,   { allow type specialization  }
        stoParseClassParent       { parse of parent class type }
      );
      TSingleTypeOptions=set of TSingleTypeOption;

    procedure resolve_forward_types;

    { reads a string, file type or a type identifier }
    procedure single_type(var def:tdef;options:TSingleTypeOptions);

    { reads any type declaration, where the resulting type will get name as type identifier }
    procedure read_named_type(var def:tdef;const newsym:tsym;genericdef:tstoreddef;genericlist:tfphashobjectlist;parseprocvardir:boolean;var hadtypetoken:boolean);

    { reads any type declaration }
    procedure read_anon_type(var def : tdef;parseprocvardir:boolean);

<<<<<<< HEAD
    { parse nested type declaration of the def (typedef) }
    procedure parse_nested_types(var def: tdef; isforwarddef,allowspecialization: boolean; currentstructstack: tfpobjectlist);


<<<<<<< HEAD
    { add a definition for a method to a record/objectdef that will contain
      all code for initialising typed constants (only for targets in
      systems.systems_typed_constants_node_init) }
    procedure add_typedconst_init_routine(def: tabstractrecorddef);

<<<<<<< HEAD
<<<<<<< HEAD
    { parse hint directives (platform, deprecated, ...) for a procdef }
    procedure maybe_parse_hint_directives(pd:tprocdef);
=======
    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean);
>>>>>>> graemeg/cpstrnew
=======
    { generate persistent type information like VMT, RTTI and inittables }
    procedure write_persistent_type_info(st:tsymtable);

    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean);
>>>>>>> origin/cpstrnew

implementation

    uses
       { common }
       cutils,
       { global }
       globals,tokens,verbose,constexp,
       systems,
       { target }
       paramgr,procinfo,
       { symtable }
<<<<<<< HEAD
       symconst,symsym,symtable,symcreat,
       defutil,defcmp,objcdef,
{$ifdef jvm}
       jvmdef,
{$endif}
=======
       symconst,symsym,symtable,
       defutil,defcmp,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       { modules }
       fmodule,
       { pass 1 }
       node,
       nmat,nadd,ncal,nset,ncnv,ninl,ncon,nld,nflw,
       { parser }
       scanner,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       pbase,pexpr,pdecsub,pdecvar,pdecobj,pdecl,pgenutil
{$ifdef jvm}
       ,pjvm
{$endif}
       ;


    procedure maybe_parse_hint_directives(pd:tprocdef);
      var
        dummysymoptions : tsymoptions;
        deprecatedmsg : pshortstring;
      begin
        if assigned(pd) then
          begin
            dummysymoptions:=pd.symoptions;
            deprecatedmsg:=pd.deprecatedmsg;
          end
        else
          begin
            dummysymoptions:=[];
            deprecatedmsg:=nil;
          end;
        while try_consume_hintdirective(dummysymoptions,deprecatedmsg) do
          consume(_SEMICOLON);
        if assigned(pd) then
          begin
            pd.symoptions:=pd.symoptions+dummysymoptions;
            if sp_has_deprecated_msg in dummysymoptions then
              pd.deprecatedmsg:=deprecatedmsg;
          end
        else
          stringdispose(deprecatedmsg);
      end;
=======
       pbase,pexpr,pdecsub,pdecvar,pdecobj,pdecl;
>>>>>>> graemeg/cpstrnew
=======
       pbase,pexpr,pdecsub,pdecvar,pdecobj,pdecl;
>>>>>>> graemeg/cpstrnew
=======
       pbase,pexpr,pdecsub,pdecvar,pdecobj,pdecl;
>>>>>>> graemeg/cpstrnew
=======
       pbase,pexpr,pdecsub,pdecvar,pdecobj,pdecl;
>>>>>>> origin/cpstrnew


    procedure resolve_forward_types;
      var
        i: longint;
        tmp,
        hpd,
        def : tdef;
        srsym  : tsym;
        srsymtable : TSymtable;
        hs : string;
        fileinfo : tfileposinfo;
      begin
        for i:=0 to current_module.checkforwarddefs.Count-1 do
          begin
            def:=tdef(current_module.checkforwarddefs[i]);
            case def.typ of
              pointerdef,
              classrefdef :
                begin
                  { classrefdef inherits from pointerdef }
                  hpd:=tabstractpointerdef(def).pointeddef;
                  { still a forward def ? }
                  if hpd.typ=forwarddef then
                   begin
                     { try to resolve the forward }
                     if not assigned(tforwarddef(hpd).tosymname) then
                       internalerror(200211201);
                     hs:=tforwarddef(hpd).tosymname^;
                     searchsym(upper(hs),srsym,srsymtable);
                     { we don't need the forwarddef anymore, dispose it }
                     hpd.free;
                     tabstractpointerdef(def).pointeddef:=nil; { if error occurs }
                     { was a type sym found ? }
                     if assigned(srsym) and
                        (srsym.typ=typesym) then
                      begin
                        if (sp_generic_dummy in srsym.symoptions) and
                            not (ttypesym(srsym).typedef.typ=undefineddef) and
                            assigned(def.owner.defowner) then
                          begin
                            { is the forward def part of a specialization? }
                            tmp:=tdef(def.owner.defowner);
                            while not tstoreddef(tmp).is_specialization and assigned(tmp.owner.defowner) do
                              tmp:=tdef(tmp.owner.defowner);
                            { if the genericdef of the specialization is the same as the
                              def the dummy points to, then update the found symbol }
                            if tstoreddef(tmp).is_specialization and
                                (tstoreddef(tmp).genericdef=ttypesym(srsym).typedef) then
                              srsym:=tstoreddef(tmp).typesym;
                          end;
                        tabstractpointerdef(def).pointeddef:=ttypesym(srsym).typedef;
                        { avoid wrong unused warnings web bug 801 PM }
                        inc(ttypesym(srsym).refs);
                        { we need a class type for classrefdef }
                        if (def.typ=classrefdef) and
<<<<<<< HEAD
                           not(is_class(ttypesym(srsym).typedef)) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           not(is_objcclass(ttypesym(srsym).typedef)) and
                           not(is_javaclass(ttypesym(srsym).typedef)) then
                          MessagePos1(def.typesym.fileinfo,type_e_class_type_expected,ttypesym(srsym).typedef.typename);
                        { this could also be a generic dummy that was not
                          overridden with a specific type }
                        if (sp_generic_dummy in srsym.symoptions) and
                            (
                              (ttypesym(srsym).typedef.typ=undefineddef) or
                              (
                                { or an unspecialized generic symbol, which is
                                  the case for generics defined in non-Delphi
                                  modes }
                                tstoreddef(ttypesym(srsym).typedef).is_generic and
                                not defs_belong_to_same_generic(def,ttypesym(srsym).typedef)
                              )
                            ) then
                          begin
                            if assigned(def.typesym) then
                              fileinfo:=def.typesym.fileinfo
                            else
                              { this is the case for inline pointer declarations }
                              fileinfo:=srsym.fileinfo;
                            MessagePos(fileinfo,parser_e_no_generics_as_types);
                          end;
=======
                           not(is_objcclass(ttypesym(srsym).typedef)) then
                          MessagePos1(def.typesym.fileinfo,type_e_class_type_expected,ttypesym(srsym).typedef.typename);
>>>>>>> graemeg/cpstrnew
=======
                           not(is_objcclass(ttypesym(srsym).typedef)) then
                          MessagePos1(def.typesym.fileinfo,type_e_class_type_expected,ttypesym(srsym).typedef.typename);
>>>>>>> graemeg/cpstrnew
=======
                           not(is_objcclass(ttypesym(srsym).typedef)) then
                          MessagePos1(def.typesym.fileinfo,type_e_class_type_expected,ttypesym(srsym).typedef.typename);
>>>>>>> graemeg/cpstrnew
=======
                           not(is_objcclass(ttypesym(srsym).typedef)) then
                          MessagePos1(def.typesym.fileinfo,type_e_class_type_expected,ttypesym(srsym).typedef.typename);
>>>>>>> origin/cpstrnew
=======
                           not(is_class(ttypesym(srsym).typedef)) then
                          MessagePos1(def.typesym.fileinfo,type_e_class_type_expected,ttypesym(srsym).typedef.typename);
>>>>>>> origin/fixes_2.4
                      end
                     else
                      begin
                        Message1(sym_e_forward_type_not_resolved,hs);
                        { try to recover }
                        tabstractpointerdef(def).pointeddef:=generrordef;
                      end;
                   end;
                end;
              objectdef :
                begin
                  { give an error as the implementation may follow in an
                    other type block which is allowed by FPC modes }
                  if not(m_fpc in current_settings.modeswitches) and
                     (oo_is_forward in tobjectdef(def).objectoptions) then
                    MessagePos1(def.typesym.fileinfo,type_e_type_is_not_completly_defined,def.typename);
                 end;
              else
                internalerror(200811071);
            end;
          end;
        current_module.checkforwarddefs.clear;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure id_type(var def : tdef;isforwarddef,checkcurrentrecdef,allowgenericsyms,allowunitsym:boolean;out srsym:tsym;out srsymtable:tsymtable;out is_specialize:boolean); forward;

<<<<<<< HEAD

    { def is the outermost type in which other types have to be searched

<<<<<<< HEAD
      isforward indicates whether the current definition can be a forward definition

      if assigned, currentstructstack is a list of tabstractrecorddefs that, from
      last to first, are child types of def that are not yet visible via the
      normal symtable searching routines because they are types that are currently
      being parsed (so using id_type on them after pushing def on the
      symtablestack would result in errors because they'd come back as errordef)
    }
    procedure parse_nested_types(var def: tdef; isforwarddef,allowspecialization: boolean; currentstructstack: tfpobjectlist);
      var
        t2: tdef;
        structstackindex: longint;
        srsym: tsym;
        srsymtable: tsymtable;
        oldsymtablestack: TSymtablestack;
        isspecialize : boolean;
      begin
        if assigned(currentstructstack) then
          structstackindex:=currentstructstack.count-1
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean);
=======
    procedure generate_specialization(var tt:tdef);
>>>>>>> graemeg/fixes_2_2
=======
    procedure generate_specialization(var tt:tdef);
>>>>>>> origin/fixes_2_2
      var
        st  : TSymtable;
        srsym : tsym;
        pt2 : tnode;
        first,
        err : boolean;
        i   : longint;
        sym : tsym;
        old_block_type : tblock_type;
        genericdef : tstoreddef;
        generictype : ttypesym;
        generictypelist : TFPObjectList;
        oldsymtablestack   : tsymtablestack;
        hmodule : tmodule;
        pu : tused_unit;
        uspecializename,
        specializename : string;
        vmtbuilder : TVMTBuilder;
        onlyparsepara : boolean;
        specializest : tsymtable;
<<<<<<< HEAD
<<<<<<< HEAD
        item: psymtablestackitem;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
        { retrieve generic def that we are going to replace }
        genericdef:=tstoreddef(tt);
        tt:=nil;
        onlyparsepara:=false;

        if not(df_generic in genericdef.defoptions) then
          begin
            Message(parser_e_special_onlygenerics);
            tt:=generrordef;
            onlyparsepara:=true;
          end;

<<<<<<< HEAD
<<<<<<< HEAD
        { only need to record the tokens, then we don't know the type yet  ... }
        if parse_generic then
          begin
            { ... but we have to insert a def into the symtable else the deflist
              of generic and specialization might not be equally sized which
              is later assumed }
            tt:=tundefineddef.create;
            if parse_class_parent then
              tt:=genericdef;
=======
=======
>>>>>>> origin/fixes_2_2
        { Only need to record the tokens, then we don't know the type yet }
        if parse_generic then
          begin
            tt:=cundefinedtype;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            onlyparsepara:=true;
          end;

        { Only parse the parameters for recovery or
          for recording in genericbuf }
        if onlyparsepara then
          begin
            consume(_LSHARPBRACKET);
            repeat
              pt2:=factor(false,true);
              pt2.free;
            until not try_to_consume(_COMMA);
            consume(_RSHARPBRACKET);
            exit;
          end;

<<<<<<< HEAD
<<<<<<< HEAD
        if not try_to_consume(_LT) then
          consume(_LSHARPBRACKET);
=======
=======
>>>>>>> origin/fixes_2_2
        consume(_LSHARPBRACKET);
        old_block_type:=block_type;
        block_type:=bt_specialize;
>>>>>>> graemeg/fixes_2_2
        { Parse generic parameters, for each undefineddef in the symtable of
          the genericdef we need to have a new def }
        err:=false;
        first:=true;
        generictypelist:=TFPObjectList.create(false);
        case genericdef.typ of
          procdef:
            st:=genericdef.GetSymtable(gs_para);
          objectdef,
          recorddef:
            st:=genericdef.GetSymtable(gs_record);
          arraydef:
            st:=tarraydef(genericdef).symtable;
          procvardef:
            st:=genericdef.GetSymtable(gs_para);
          else
            internalerror(200511182);
        end;
<<<<<<< HEAD
=======
        if not assigned(st) then
          internalerror(200511182);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

        { Parse type parameters }
        if not assigned(genericdef.typesym) then
          internalerror(200710173);
        specializename:=genericdef.typesym.realname;
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (sp_generic_para in sym.symoptions) then
              begin
                if not first then
                  consume(_COMMA)
                else
                  first:=false;
                pt2:=factor(false,true);
                if pt2.nodetype=typen then
                  begin
                    if df_generic in pt2.resultdef.defoptions then
                      Message(parser_e_no_generics_as_params);
                    generictype:=ttypesym.create(sym.realname,pt2.resultdef);
                    generictypelist.add(generictype);
                    if not assigned(pt2.resultdef.typesym) then
<<<<<<< HEAD
<<<<<<< HEAD
                      message(type_e_generics_cannot_reference_itself)
                    else
                      specializename:=specializename+'$'+pt2.resultdef.typesym.realname;
=======
                      internalerror(200710172);
                    specializename:=specializename+'$'+pt2.resultdef.typesym.realname;
>>>>>>> graemeg/fixes_2_2
=======
                      internalerror(200710172);
                    specializename:=specializename+'$'+pt2.resultdef.typesym.realname;
>>>>>>> origin/fixes_2_2
                  end
                else
                  begin
                    Message(type_e_type_id_expected);
                    err:=true;
                  end;
                pt2.free;
              end;
          end;
        uspecializename:=upper(specializename);
        { force correct error location if too much type parameters are passed }
<<<<<<< HEAD
<<<<<<< HEAD
        if not (token in [_RSHARPBRACKET,_GT]) then
          consume(_RSHARPBRACKET);

        { Special case if we are referencing the current defined object }
        if assigned(current_structdef) and
           (current_structdef.objname^=uspecializename) then
          tt:=current_structdef;
=======
=======
>>>>>>> origin/fixes_2_2
        if token<>_RSHARPBRACKET then
          consume(_RSHARPBRACKET);

        { Special case if we are referencing the current defined object }
        if assigned(aktobjectdef) and
           (aktobjectdef.objname^=uspecializename) then
          tt:=aktobjectdef;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

        { for units specializations can already be needed in the interface, therefor we
          will use the global symtable. Programs don't have a globalsymtable and there we
          use the localsymtable }
        if current_module.is_unit then
          specializest:=current_module.globalsymtable
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
        else
          structstackindex:=-1;
        { handle types inside classes, e.g. TNode.TLongint }
        while (token=_POINT) do
          begin
             if is_class_or_object(def) or is_record(def) or is_java_class_or_interface(def) then
               begin
                 if (def.typ=objectdef) then
                   def:=find_real_class_definition(tobjectdef(def),false);
                 consume(_POINT);
                 if (structstackindex>=0) and
                    (tabstractrecorddef(currentstructstack[structstackindex]).objname^=pattern) then
                   begin
                     def:=tdef(currentstructstack[structstackindex]);
                     dec(structstackindex);
                     consume(_ID);
                   end
                 else
                   begin
                     structstackindex:=-1;
                     oldsymtablestack:=symtablestack;
                     symtablestack:=TSymtablestack.create;
                     symtablestack.push(tabstractrecorddef(def).symtable);
                     t2:=generrordef;
                     id_type(t2,isforwarddef,false,false,false,srsym,srsymtable,isspecialize);
                     symtablestack.pop(tabstractrecorddef(def).symtable);
                     symtablestack.free;
                     symtablestack:=oldsymtablestack;
                     if isspecialize then
                       begin
                         if not allowspecialization then
                           Message(parser_e_no_local_para_def);
                         generate_specialization(t2,false,'');
                       end;
                     def:=t2;
                   end;
               end
             else
               break;
          end;
      end;

<<<<<<< HEAD

    function try_parse_structdef_nested_type(out def: tdef; basedef: tabstractrecorddef; isfowarddef: boolean): boolean;
      var
        structdef : tdef;
        structdefstack : tfpobjectlist;
      begin
         def:=nil;
         { use of current parsed object:
           classes, objects, records can be used also in themself }
         structdef:=basedef;
         structdefstack:=nil;
         while assigned(structdef) and (structdef.typ in [objectdef,recorddef]) do
           begin
             if (tabstractrecorddef(structdef).objname^=pattern) then
               begin
                 consume(_ID);
                 def:=structdef;
                 { we found the top-most match, now check how far down we can
                   follow }
                 structdefstack:=tfpobjectlist.create(false);
                 structdef:=basedef;
                 while (structdef<>def) do
                   begin
                     structdefstack.add(structdef);
                     structdef:=tabstractrecorddef(structdef.owner.defowner);
                   end;
                 parse_nested_types(def,isfowarddef,false,structdefstack);
                 structdefstack.free;
                 result:=true;
                 exit;
               end;
             structdef:=tdef(tabstractrecorddef(structdef).owner.defowner);
           end;
         result:=false;
      end;

    procedure id_type(var def : tdef;isforwarddef,checkcurrentrecdef,allowgenericsyms,allowunitsym:boolean;out srsym:tsym;out srsymtable:tsymtable;out is_specialize:boolean);
=======
        if not assigned(tt) then
          begin
            { Setup symtablestack at definition time
              to get types right, however this is not perfect, we should probably record
              the resolved symbols }
            oldsymtablestack:=symtablestack;
            symtablestack:=tsymtablestack.create;
            if not assigned(genericdef) then
              internalerror(200705151);
            hmodule:=find_module_from_symtable(genericdef.owner);
            if hmodule=nil then
              internalerror(200705152);
            pu:=tused_unit(hmodule.used_units.first);
            while assigned(pu) do
              begin
                if not assigned(pu.u.globalsymtable) then
                  internalerror(200705153);
                symtablestack.push(pu.u.globalsymtable);
                pu:=tused_unit(pu.next);
              end;

            if assigned(hmodule.globalsymtable) then
              symtablestack.push(hmodule.globalsymtable);

            { hacky, but necessary to insert the newly generated class properly }
            item:=oldsymtablestack.stack;
            while assigned(item) and (item^.symtable.symtablelevel>main_program_level) do
              item:=item^.next;
            if assigned(item) and (item^.symtable<>symtablestack.top) then
              symtablestack.push(item^.symtable);

            { Reparse the original type definition }
            if not err then
              begin
                { First a new typesym so we can reuse this specialization and
                  references to this specialization can be handled }
                srsym:=ttypesym.create(specializename,generrordef);
                specializest.insert(srsym);

                if not assigned(genericdef.generictokenbuf) then
                  internalerror(200511171);
                current_scanner.startreplaytokens(genericdef.generictokenbuf);
                read_named_type(tt,specializename,genericdef,generictypelist,false);
                ttypesym(srsym).typedef:=tt;
                tt.typesym:=srsym;

                case tt.typ of
                  { Build VMT indexes for classes }
                  objectdef:
                    begin
                      vmtbuilder:=TVMTBuilder.Create(tobjectdef(tt));
                      vmtbuilder.generate_vmt;
                      vmtbuilder.free;
                    end;
                  { handle params, calling convention, etc }
                  procvardef:
                    begin
                      if not check_proc_directive(true) then
                        begin
                          try_consume_hintdirective(ttypesym(srsym).symoptions,ttypesym(srsym).deprecatedmsg);
                          consume(_SEMICOLON);
                        end;
                      parse_var_proc_directives(ttypesym(srsym));
                      handle_calling_convention(tprocvardef(tt));
                      if try_consume_hintdirective(ttypesym(srsym).symoptions,ttypesym(srsym).deprecatedmsg) then
                        consume(_SEMICOLON);
                    end;
                end;
                { Consume the semicolon if it is also recorded }
                try_to_consume(_SEMICOLON);
              end;

            { Restore symtablestack }
            symtablestack.free;
            symtablestack:=oldsymtablestack;
          end
        else
          begin
            { There is comment few lines before ie 200512115
              saying "We are parsing the same objectdef, the def index numbers
              are the same". This is wrong (index numbers are not same)
              in case there is specialization (S2 in this case) inside
              specialized generic (G2 in this case) which is equal to
              some previous specialization (S1 in this case). In that case,
              new symbol is not added to currently specialized type
              (S in this case) for that specializations (S2 in this case),
              and this results in that specialization and generic definition
              don't have same number of elements in their object symbol tables.
              This patch adds undefined def to ensure that those
              two symbol tables will have same number of elements.
            }
            tundefineddef.create;
          end;

        generictypelist.free;
        if not try_to_consume(_GT) then
          consume(_RSHARPBRACKET);
<<<<<<< HEAD
      end;


    procedure id_type(var def : tdef;isforwarddef,checkcurrentrecdef:boolean); forward;

    { def is the outermost type in which other types have to be searched

      isforward indicates whether the current definition can be a forward definition

      if assigned, currentstructstack is a list of tabstractrecorddefs that, from
      last to first, are child types of def that are not yet visible via the
      normal symtable searching routines because they are types that are currently
      being parsed (so using id_type on them after pushing def on the
      symtablestack would result in errors because they'd come back as errordef)
    }
    procedure parse_nested_types(var def: tdef; isforwarddef: boolean; currentstructstack: tfpobjectlist);
      var
        t2: tdef;
        structstackindex: longint;
      begin
        if assigned(currentstructstack) then
          structstackindex:=currentstructstack.count-1
        else
          structstackindex:=-1;
        { handle types inside classes, e.g. TNode.TLongint }
        while (token=_POINT) do
          begin
            if parse_generic then
              begin
                 consume(_POINT);
                 consume(_ID);
              end
             else if is_class_or_object(def) or is_record(def) then
               begin
                 consume(_POINT);
                 if (structstackindex>=0) and
                    (tabstractrecorddef(currentstructstack[structstackindex]).objname^=pattern) then
                   begin
                     def:=tdef(currentstructstack[structstackindex]);
                     dec(structstackindex);
                     consume(_ID);
                   end
                 else
                   begin
                     structstackindex:=-1;
                     symtablestack.push(tabstractrecorddef(def).symtable);
                     t2:=generrordef;
                     id_type(t2,isforwarddef,false);
                     symtablestack.pop(tabstractrecorddef(def).symtable);
                     def:=t2;
                   end;
               end
             else
               break;
          end;
      end;


=======
      end;


    procedure id_type(var def : tdef;isforwarddef,checkcurrentrecdef:boolean); forward;

    { def is the outermost type in which other types have to be searched

      isforward indicates whether the current definition can be a forward definition

      if assigned, currentstructstack is a list of tabstractrecorddefs that, from
      last to first, are child types of def that are not yet visible via the
      normal symtable searching routines because they are types that are currently
      being parsed (so using id_type on them after pushing def on the
      symtablestack would result in errors because they'd come back as errordef)
    }
    procedure parse_nested_types(var def: tdef; isforwarddef: boolean; currentstructstack: tfpobjectlist);
      var
        t2: tdef;
        structstackindex: longint;
      begin
        if assigned(currentstructstack) then
          structstackindex:=currentstructstack.count-1
        else
          structstackindex:=-1;
        { handle types inside classes, e.g. TNode.TLongint }
        while (token=_POINT) do
          begin
            if parse_generic then
              begin
                 consume(_POINT);
                 consume(_ID);
              end
             else if is_class_or_object(def) or is_record(def) then
               begin
                 consume(_POINT);
                 if (structstackindex>=0) and
                    (tabstractrecorddef(currentstructstack[structstackindex]).objname^=pattern) then
                   begin
                     def:=tdef(currentstructstack[structstackindex]);
                     dec(structstackindex);
                     consume(_ID);
                   end
                 else
                   begin
                     structstackindex:=-1;
                     symtablestack.push(tabstractrecorddef(def).symtable);
                     t2:=generrordef;
                     id_type(t2,isforwarddef,false);
                     symtablestack.pop(tabstractrecorddef(def).symtable);
                     def:=t2;
                   end;
               end
             else
               break;
          end;
=======
=======
>>>>>>> origin/fixes_2_2
        else
          specializest:=current_module.localsymtable;

        { Can we reuse an already specialized type? }
        if not assigned(tt) then
<<<<<<< HEAD
          begin
            srsym:=tsym(specializest.find(uspecializename));
            if assigned(srsym) then
              begin
                if srsym.typ<>typesym then
                  internalerror(200710171);
                tt:=ttypesym(srsym).typedef;
              end;
          end;

        if not assigned(tt) then
          begin
            { Setup symtablestack at definition time
              to get types right, however this is not perfect, we should probably record
              the resolved symbols }
            oldsymtablestack:=symtablestack;
            symtablestack:=tsymtablestack.create;
            if not assigned(genericdef) then
              internalerror(200705151);
            hmodule:=find_module_from_symtable(genericdef.owner);
            if hmodule=nil then
              internalerror(200705152);
            pu:=tused_unit(hmodule.used_units.first);
            while assigned(pu) do
              begin
                if not assigned(pu.u.globalsymtable) then
                  internalerror(200705153);
                symtablestack.push(pu.u.globalsymtable);
                pu:=tused_unit(pu.next);
              end;

            if assigned(hmodule.globalsymtable) then
              symtablestack.push(hmodule.globalsymtable);

            { hacky, but necessary to insert the newly generated class properly }
            symtablestack.push(oldsymtablestack.top);

            { Reparse the original type definition }
            if not err then
              begin
                { Firsta new typesym so we can reuse this specialization and
                  references to this specialization can be handled }
                srsym:=ttypesym.create(specializename,generrordef);
                specializest.insert(srsym);

                if not assigned(genericdef.generictokenbuf) then
                  internalerror(200511171);
                current_scanner.startreplaytokens(genericdef.generictokenbuf);
                read_named_type(tt,specializename,genericdef,generictypelist,false);
                ttypesym(srsym).typedef:=tt;
                tt.typesym:=srsym;
                { Consume the semicolon if it is also recorded }
                try_to_consume(_SEMICOLON);

                { Build VMT indexes for classes }
                if (tt.typ=objectdef) then
                  begin
                    vmtbuilder:=TVMTBuilder.Create(tobjectdef(tt));
                    vmtbuilder.generate_vmt;
                    vmtbuilder.free;
                  end;
              end;

            { Restore symtablestack }
            symtablestack.free;
            symtablestack:=oldsymtablestack;
          end;

        generictypelist.free;
        consume(_RSHARPBRACKET);
        block_type:=old_block_type;
>>>>>>> graemeg/fixes_2_2
=======
          begin
            srsym:=tsym(specializest.find(uspecializename));
            if assigned(srsym) then
              begin
                if srsym.typ<>typesym then
                  internalerror(200710171);
                tt:=ttypesym(srsym).typedef;
              end;
          end;

        if not assigned(tt) then
          begin
            { Setup symtablestack at definition time
              to get types right, however this is not perfect, we should probably record
              the resolved symbols }
            oldsymtablestack:=symtablestack;
            symtablestack:=tsymtablestack.create;
            if not assigned(genericdef) then
              internalerror(200705151);
            hmodule:=find_module_from_symtable(genericdef.owner);
            if hmodule=nil then
              internalerror(200705152);
            pu:=tused_unit(hmodule.used_units.first);
            while assigned(pu) do
              begin
                if not assigned(pu.u.globalsymtable) then
                  internalerror(200705153);
                symtablestack.push(pu.u.globalsymtable);
                pu:=tused_unit(pu.next);
              end;

            if assigned(hmodule.globalsymtable) then
              symtablestack.push(hmodule.globalsymtable);

            { hacky, but necessary to insert the newly generated class properly }
            symtablestack.push(oldsymtablestack.top);

            { Reparse the original type definition }
            if not err then
              begin
                { Firsta new typesym so we can reuse this specialization and
                  references to this specialization can be handled }
                srsym:=ttypesym.create(specializename,generrordef);
                specializest.insert(srsym);

                if not assigned(genericdef.generictokenbuf) then
                  internalerror(200511171);
                current_scanner.startreplaytokens(genericdef.generictokenbuf);
                read_named_type(tt,specializename,genericdef,generictypelist,false);
                ttypesym(srsym).typedef:=tt;
                tt.typesym:=srsym;
                { Consume the semicolon if it is also recorded }
                try_to_consume(_SEMICOLON);

                { Build VMT indexes for classes }
                if (tt.typ=objectdef) then
                  begin
                    vmtbuilder:=TVMTBuilder.Create(tobjectdef(tt));
                    vmtbuilder.generate_vmt;
                    vmtbuilder.free;
                  end;
              end;

            { Restore symtablestack }
            symtablestack.free;
            symtablestack:=oldsymtablestack;
          end;

        generictypelist.free;
        consume(_RSHARPBRACKET);
        block_type:=old_block_type;
>>>>>>> origin/fixes_2_2
      end;


>>>>>>> origin/cpstrnew
    function try_parse_structdef_nested_type(out def: tdef; basedef: tabstractrecorddef; isfowarddef: boolean): boolean;
      var
        structdef : tdef;
        structdefstack : tfpobjectlist;
      begin
         { use of current parsed object:
           classes, objects, records can be used also in themself }
         structdef:=basedef;
         structdefstack:=nil;
         while assigned(structdef) and (structdef.typ in [objectdef,recorddef]) do
           begin
             if (tabstractrecorddef(structdef).objname^=pattern) then
               begin
                 consume(_ID);
                 def:=structdef;
                 { we found the top-most match, now check how far down we can
                   follow }
                 structdefstack:=tfpobjectlist.create(false);
                 structdef:=basedef;
                 while (structdef<>def) do
                   begin
                     structdefstack.add(structdef);
                     structdef:=tabstractrecorddef(structdef.owner.defowner);
                   end;
                 parse_nested_types(def,isfowarddef,structdefstack);
                 structdefstack.free;
                 result:=true;
                 exit;
               end;
             structdef:=tdef(tabstractrecorddef(structdef).owner.defowner);
           end;
         result:=false;
      end;

    procedure id_type(var def : tdef;isforwarddef,checkcurrentrecdef:boolean);
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
    { reads a type definition }
    { to a appropriating tdef, s gets the name of   }
    { the type to allow name mangling          }
      var
        is_unit_specific,not_a_type : boolean;
        pos : tfileposinfo;
        s,sorg : TIDString;
        t : ttoken;
<<<<<<< HEAD
<<<<<<< HEAD
        structdef : tabstractrecorddef;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
         srsym:=nil;
         srsymtable:=nil;
         is_specialize:=false;
         s:=pattern;
         sorg:=orgpattern;
         pos:=current_tokenpos;
         { use of current parsed object:
           classes, objects, records can be used also in themself }
         if checkcurrentrecdef and
            try_parse_structdef_nested_type(def,current_structdef,isforwarddef) then
           exit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if not allowunitsym and (idtoken=_SPECIALIZE) then
           begin
             consume(_ID);
             is_specialize:=true;
             s:=pattern;
             sorg:=orgpattern;
             pos:=current_tokenpos;
           end;
         { Use the special searchsym_type that search only types }
         if not searchsym_type(s,srsym,srsymtable) then
           { for a good error message we need to know whether the symbol really did not exist or
             whether we found a non-type one }
           not_a_type:=searchsym(s,srsym,srsymtable)
         else
           not_a_type:=false;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         { Use the special searchsym_type that search only types }
         searchsym_type(s,srsym,srsymtable);
>>>>>>> graemeg/cpstrnew
         { handle unit specification like System.Writeln }
<<<<<<< HEAD
<<<<<<< HEAD
         if allowunitsym then
           is_unit_specific:=try_consume_unitsym(srsym,srsymtable,t,true,true,is_specialize)
         else
           begin
             t:=_ID;
             is_unit_specific:=false;
           end;
         consume(t);
         if not_a_type then
           begin
             { reset the symbol and symtable to not leak any unexpected values }
             srsym:=nil;
             srsymtable:=nil;
           end;
=======
         is_unit_specific:=try_consume_unitsym(srsym,srsymtable,t);
         consume(t);
>>>>>>> graemeg/fixes_2_2
=======
         is_unit_specific:=try_consume_unitsym(srsym,srsymtable,t);
         consume(t);
>>>>>>> origin/fixes_2_2
         { Types are first defined with an error def before assigning
           the real type so check if it's an errordef. if so then
           give an error. Only check for typesyms in the current symbol
           table as forwarddef are not resolved directly }
         if assigned(srsym) and
            (srsym.typ=typesym) and
            ((ttypesym(srsym).typedef.typ=errordef) or
            (not allowgenericsyms and
            (ttypesym(srsym).typedef.typ=undefineddef) and
            not (sp_generic_para in srsym.symoptions) and
            not (sp_explicitrename in srsym.symoptions) and
            not assigned(srsym.owner.defowner) and
            { use df_generic instead of is_generic to allow aliases in nested types as well }
            not (df_generic in tstoreddef(srsym.owner.defowner).defoptions))) then
          begin
            Message1(type_e_type_is_not_completly_defined,ttypesym(srsym).realname);
            def:=generrordef;
            exit;
          end;
         { are we parsing a possible forward def ? }
         if isforwarddef and
            not(is_unit_specific) then
           begin
             def:=cforwarddef.create(sorg,pos);
             exit;
           end;
         { unknown sym ? }
         if not assigned(srsym) and not not_a_type then
          begin
            Message1(sym_e_id_not_found,sorg);
            def:=generrordef;
            exit;
          end;
         { type sym ? }
         if not_a_type or (srsym.typ<>typesym) then
          begin
            Message(type_e_type_id_expected);
            def:=generrordef;
            exit;
          end;
         { Give an error when referring to an errordef }
         if (ttypesym(srsym).typedef.typ=errordef) then
          begin
            Message(sym_e_error_in_type_def);
            def:=generrordef;
            exit;
          end;
         { In non-Delphi modes the class/record name of a generic might be used
           in the declaration of sub types without type parameters; in that case
           we need to check by name as the link from the dummy symbol to the
           current type is not yet established }
         if (sp_generic_dummy in srsym.symoptions) and
             assigned(current_structdef) and
             (df_generic in current_structdef.defoptions) and
             (ttypesym(srsym).typedef.typ=undefineddef) and
             not (m_delphi in current_settings.modeswitches) then
           begin
             def:=get_generic_in_hierarchy_by_name(srsym,current_structdef);
             if assigned(def) then
               exit;
           end;
        def:=ttypesym(srsym).typedef;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure single_type(var def:tdef;options:TSingleTypeOptions);
       var
         t2 : tdef;
         isspecialize,
         dospecialize,
         again : boolean;
         srsym : tsym;
         srsymtable : tsymtable;
       begin
         dospecialize:=false;
         srsym:=nil;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure single_type(var def:tdef;isforwarddef:boolean);
       var
         t2 : tdef;
         dospecialize,
         again : boolean;
       begin
         dospecialize:=false;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         repeat
           again:=false;
             case token of
               _STRING:
<<<<<<< HEAD
<<<<<<< HEAD
                 string_dec(def,stoAllowTypeDef in options);

               _FILE:
                 begin
                    consume(_FILE);
                    if (token=_OF) then
                      begin
                         if not(stoAllowTypeDef in options) then
                           Message(parser_e_no_local_para_def);
                         consume(_OF);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                         single_type(t2,[stoAllowTypeDef]);
                         if is_managed_type(t2) then
                           Message(parser_e_no_refcounted_typed_file);
                         def:=cfiledef.createtyped(t2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                         single_type(t2,[]);
                         if is_managed_type(t2) then
                           Message(parser_e_no_refcounted_typed_file);
                         def:=tfiledef.createtyped(t2);
>>>>>>> graemeg/cpstrnew
                      end
                    else
                      def:=cfiletype;
                 end;

               _ID:
                 begin
                   if try_to_consume(_SPECIALIZE) then
                     begin
                       if ([stoAllowSpecialization,stoAllowTypeDef] * options = []) then
                         begin
                           Message(parser_e_no_local_para_def);

                           { try to recover }
                           while token<>_SEMICOLON do
                             consume(token);
                           def:=generrordef;
                         end
                       else
                         begin
                           dospecialize:=true;
                           again:=true;
                         end;
                     end
                   else
                     begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       id_type(def,stoIsForwardDef in options,true,true,not dospecialize or ([stoAllowSpecialization,stoAllowTypeDef]*options=[]),srsym,srsymtable,isspecialize);
                       if isspecialize and dospecialize then
                         internalerror(2015021301);
                       if isspecialize then
                         dospecialize:=true;
                       parse_nested_types(def,stoIsForwardDef in options,[stoAllowSpecialization,stoAllowTypeDef]*options<>[],nil);
=======
                       id_type(def,stoIsForwardDef in options,true);
                       parse_nested_types(def,stoIsForwardDef in options,nil);
>>>>>>> graemeg/cpstrnew
=======
                       id_type(def,stoIsForwardDef in options,true);
                       parse_nested_types(def,stoIsForwardDef in options,nil);
>>>>>>> graemeg/cpstrnew
=======
                       id_type(def,stoIsForwardDef in options,true);
                       parse_nested_types(def,stoIsForwardDef in options,nil);
>>>>>>> graemeg/cpstrnew
=======
                       id_type(def,stoIsForwardDef in options,true);
                       parse_nested_types(def,stoIsForwardDef in options,nil);
>>>>>>> origin/cpstrnew
                     end;
                 end;

               else
                 begin
                   message(type_e_type_id_expected);
                   def:=generrordef;
                 end;
            end;
        until not again;
        if ([stoAllowSpecialization,stoAllowTypeDef] * options <> []) and
           (m_delphi in current_settings.modeswitches) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          dospecialize:=token in [_LSHARPBRACKET,_LT];
        if dospecialize and
            (def.typ=forwarddef) then
          begin
            if not assigned(srsym) or not (srsym.typ=typesym) then
              begin
                Message1(type_e_type_is_not_completly_defined,def.typename);
                def:=generrordef;
                dospecialize:=false;
              end;
          end;
        if dospecialize then
          begin
            if def.typ=forwarddef then
              def:=ttypesym(srsym).typedef;
            generate_specialization(def,stoParseClassParent in options,'');
            parse_nested_types(def,stoIsForwardDef in options,[stoAllowSpecialization,stoAllowTypeDef]*options<>[],nil);
          end
=======
          dospecialize:=token=_LSHARPBRACKET;
        if dospecialize then
          generate_specialization(def,stoParseClassParent in options)
>>>>>>> graemeg/cpstrnew
=======
          dospecialize:=token=_LSHARPBRACKET;
        if dospecialize then
          generate_specialization(def,stoParseClassParent in options)
>>>>>>> graemeg/cpstrnew
        else
          begin
            if assigned(current_specializedef) and (def=current_specializedef.genericdef) then
              begin
                def:=current_specializedef
              end
            else if (def=current_genericdef) then
              begin
                def:=current_genericdef
              end
<<<<<<< HEAD
<<<<<<< HEAD
            { when parsing a nested specialization in non-Delphi mode it might
              use the name of the topmost generic without type paramaters, thus
              def will contain the generic definition, but we need a reference
              to the specialization of that generic }
            { TODO : only in non-Delphi modes? }
            else if assigned(current_structdef) and
                (df_specialization in current_structdef.defoptions) and
                return_specialization_of_generic(current_structdef,def,t2) then
              begin
                def:=t2
              end
            else if tstoreddef(def).is_generic and
                not
                  (
                    parse_generic and
                    (
                      { if this is a generic parameter than it has already been checked that this is
                        a valid usage of a generic }
                      (sp_generic_para in srsym.symoptions) or
                      (
                        (current_genericdef.typ in [recorddef,objectdef]) and
                        (
                          { if both defs belong to the same generic (e.g. both are
                            subtypes) then we must allow the usage }
                          defs_belong_to_same_generic(def,current_genericdef) or
                          { this is needed to correctly resolve "type Foo=SomeGeneric<T>"
                            declarations inside a generic }
                          sym_is_owned_by(srsym,tabstractrecorddef(current_genericdef).symtable)
                        )
                      )
                    )
                  )
                then
              begin
                srsym:=resolve_generic_dummysym(srsym.name);
                if assigned(srsym) and
                    not (sp_generic_dummy in srsym.symoptions) and
                    (srsym.typ=typesym) then
                  def:=ttypesym(srsym).typedef
                else
                  begin
                    Message(parser_e_no_generics_as_types);
                    def:=generrordef;
                  end;
              end
            else if (def.typ=undefineddef) and
                (sp_generic_dummy in srsym.symoptions) and
                parse_generic and
                (current_genericdef.typ in [recorddef,objectdef]) and
                (Pos(upper(srsym.realname),tabstractrecorddef(current_genericdef).objname^)=1) then
              begin
                if m_delphi in current_settings.modeswitches then
                  begin
                    srsym:=resolve_generic_dummysym(srsym.name);
                    if assigned(srsym) and
                        not (sp_generic_dummy in srsym.symoptions) and
                        (srsym.typ=typesym) then
                      def:=ttypesym(srsym).typedef
                    else
                      begin
                        Message(parser_e_no_generics_as_types);
                        def:=generrordef;
                      end;
                  end
                else
                  def:=current_genericdef;
              end
            else if is_classhelper(def) and
                not (stoParseClassParent in options) then
=======
=======
>>>>>>> origin/cpstrnew
          dospecialize:=token=_LSHARPBRACKET;
        if dospecialize then
          generate_specialization(def,stoParseClassParent in options)
        else
          begin
            if assigned(current_specializedef) and (def=current_specializedef.genericdef) then
              begin
                def:=current_specializedef
              end
            else if (def=current_genericdef) then
              begin
                def:=current_genericdef
              end
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            else if (df_generic in def.defoptions) then
              begin
                Message(parser_e_no_generics_as_types);
                def:=generrordef;
              end
            else if is_objccategory(def) then
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
              begin
                Message(parser_e_no_category_as_types);
                def:=generrordef
              end
          end;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    procedure parse_record_members(recsym:tsym);

      function IsAnonOrLocal: Boolean;
        begin
          result:=(current_structdef.objname^='') or
                  not(symtablestack.stack^.next^.symtable.symtabletype in [globalsymtable,staticsymtable,objectsymtable,recordsymtable]);
        end;

      var
        olddef : tdef;

      procedure set_typesym;
        begin
          if not assigned(recsym) then
            exit;
          if ttypesym(recsym).typedef=current_structdef then
            exit;
          ttypesym(recsym).typedef:=current_structdef;
          current_structdef.typesym:=recsym;
        end;

      procedure reset_typesym;
        begin
          if not assigned(recsym) then
            exit;
          if ttypesym(recsym).typedef<>current_structdef then
            exit;
          ttypesym(recsym).typedef:=olddef;
          current_structdef.typesym:=nil;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure parse_record_members;

        procedure maybe_parse_hint_directives(pd:tprocdef);
        var
          dummysymoptions : tsymoptions;
          deprecatedmsg : pshortstring;
        begin
          dummysymoptions:=[];
          deprecatedmsg:=nil;
          while try_consume_hintdirective(dummysymoptions,deprecatedmsg) do
            Consume(_SEMICOLON);
          if assigned(pd) then
            begin
              pd.symoptions:=pd.symoptions+dummysymoptions;
              pd.deprecatedmsg:=deprecatedmsg;
            end
          else
            stringdispose(deprecatedmsg);
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

      var
        pd : tprocdef;
        oldparse_only: boolean;
        member_blocktype : tblock_type;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        hadgeneric,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        fields_allowed, is_classdef, classfields: boolean;
        vdoptions: tvar_dec_options;
      begin
        { empty record declaration ? }
        if (token=_SEMICOLON) then
          Exit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        { the correct typesym<->def relationship is needed for example when
          parsing parameters that are specializations of the record or when
          using nested constants and such }
        if assigned(recsym) then
          olddef:=ttypesym(recsym).typedef
        else
          olddef:=nil;
        set_typesym;
        current_structdef.symtable.currentvisibility:=vis_public;
        fields_allowed:=true;
        is_classdef:=false;
        hadgeneric:=false;
=======
        current_structdef.symtable.currentvisibility:=vis_public;
        fields_allowed:=true;
        is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
        current_structdef.symtable.currentvisibility:=vis_public;
        fields_allowed:=true;
        is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
        current_structdef.symtable.currentvisibility:=vis_public;
        fields_allowed:=true;
        is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
        current_structdef.symtable.currentvisibility:=vis_public;
        fields_allowed:=true;
        is_classdef:=false;
>>>>>>> origin/cpstrnew
        classfields:=false;
        member_blocktype:=bt_general;
        repeat
          case token of
            _TYPE :
              begin
                consume(_TYPE);
                member_blocktype:=bt_type;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

                { local and anonymous records can not have inner types. skip top record symtable }
                if IsAnonOrLocal then
                  Message(parser_e_no_types_in_local_anonymous_records);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              end;
            _VAR :
              begin
                consume(_VAR);
                fields_allowed:=true;
                member_blocktype:=bt_general;
                classfields:=is_classdef;
                is_classdef:=false;
<<<<<<< HEAD
<<<<<<< HEAD
              end;
            _CONST:
              begin
                consume(_CONST);
                member_blocktype:=bt_const;
<<<<<<< HEAD
<<<<<<< HEAD

                { local and anonymous records can not have constants. skip top record symtable }
                if IsAnonOrLocal then
                  Message(parser_e_no_consts_in_local_anonymous_records);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
              end;
            _ID, _CASE, _OPERATOR :
              begin
                case idtoken of
                  _PRIVATE :
                    begin
                       consume(_PRIVATE);
                       current_structdef.symtable.currentvisibility:=vis_private;
                       include(current_structdef.objectoptions,oo_has_private);
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PROTECTED :
                     begin
<<<<<<< HEAD
<<<<<<< HEAD
                       Message1(parser_e_not_allowed_in_record,tokeninfo^[_PROTECTED].str);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                       consume(_PROTECTED);
                       current_structdef.symtable.currentvisibility:=vis_protected;
                       include(current_structdef.objectoptions,oo_has_protected);
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PUBLIC :
                     begin
                       consume(_PUBLIC);
                       current_structdef.symtable.currentvisibility:=vis_public;
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PUBLISHED :
                     begin
                       Message(parser_e_no_record_published);
                       consume(_PUBLISHED);
                       current_structdef.symtable.currentvisibility:=vis_published;
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _STRICT :
                     begin
                        consume(_STRICT);
                        if token=_ID then
                          begin
                            case idtoken of
                              _PRIVATE:
                                begin
                                  consume(_PRIVATE);
                                  current_structdef.symtable.currentvisibility:=vis_strictprivate;
                                  include(current_structdef.objectoptions,oo_has_strictprivate);
                                end;
                              _PROTECTED:
                                begin
<<<<<<< HEAD
<<<<<<< HEAD
                                  { "strict protected" is not allowed for records }
                                  Message1(parser_e_not_allowed_in_record,tokeninfo^[_STRICT].str+' '+tokeninfo^[_PROTECTED].str);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                                  consume(_PROTECTED);
                                  current_structdef.symtable.currentvisibility:=vis_strictprotected;
                                  include(current_structdef.objectoptions,oo_has_strictprotected);
                                end;
                              else
                                message(parser_e_protected_or_private_expected);
                            end;
                          end
                        else
                          message(parser_e_protected_or_private_expected);
                        fields_allowed:=true;
                        is_classdef:=false;
                        classfields:=false;
                        member_blocktype:=bt_general;
                     end
                    else
                    if is_classdef and (idtoken=_OPERATOR) then
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
                        pd:=parse_record_method_dec(current_structdef,is_classdef,false);
                        fields_allowed:=false;
                        is_classdef:=false;
                      end
                    else
                      begin
                        if member_blocktype=bt_general then
                          begin
                            if (idtoken=_GENERIC) and
                                not (m_delphi in current_settings.modeswitches) and
                                not fields_allowed then
                              begin
                                if hadgeneric then
                                  Message(parser_e_procedure_or_function_expected);
                                consume(_ID);
                                hadgeneric:=true;
                                if not (token in [_PROCEDURE,_FUNCTION,_CLASS]) then
                                  Message(parser_e_procedure_or_function_expected);
                              end
                            else
                              begin
                                if (not fields_allowed)and(idtoken<>_CASE) then
                                  Message(parser_e_field_not_allowed_here);
                                vdoptions:=[vd_record];
                                if classfields then
                                  include(vdoptions,vd_class);
                                if not (m_delphi in current_settings.modeswitches) then
                                  include(vdoptions,vd_check_generic);
                                read_record_fields(vdoptions,nil,nil,hadgeneric);
                              end;
                          end
                        else if member_blocktype=bt_type then
                          types_dec(true,hadgeneric)
                        else if member_blocktype=bt_const then
                          consts_dec(true,true,hadgeneric)
=======
=======
>>>>>>> graemeg/cpstrnew
                        oldparse_only:=parse_only;
                        parse_only:=true;
                        pd:=parse_proc_dec(is_classdef,current_structdef);

                        { this is for error recovery as well as forward }
                        { interface mappings, i.e. mapping to a method  }
                        { which isn't declared yet                      }
                        if assigned(pd) then
                          begin
                            parse_record_proc_directives(pd);

                            handle_calling_convention(pd);

                            { add definition to procsym }
                            proc_add_definition(pd);
                          end;

                        maybe_parse_hint_directives(pd);

                        parse_only:=oldparse_only;
                        fields_allowed:=false;
                        is_classdef:=false;
                      end
                      else
                      begin
                        if member_blocktype=bt_general then
                          begin
                            if (not fields_allowed) then
                              Message(parser_e_field_not_allowed_here);
                            vdoptions:=[vd_record];
                            if classfields then
                              include(vdoptions,vd_class);
                            read_record_fields(vdoptions);
                          end
                        else if member_blocktype=bt_type then
                          types_dec(true)
                        else if member_blocktype=bt_const then
                          consts_dec(true)
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                        else
                          internalerror(201001110);
                      end;
                end;
              end;
            _PROPERTY :
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                if IsAnonOrLocal then
                  Message(parser_e_no_properties_in_local_anonymous_records);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                struct_property_dec(is_classdef);
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _CLASS:
              begin
                is_classdef:=false;
                { read class method/field/property }
                consume(_CLASS);
                { class modifier is only allowed for procedures, functions, }
                { constructors, destructors, fields and properties          }
<<<<<<< HEAD
<<<<<<< HEAD
                if (hadgeneric and not (token in [_FUNCTION,_PROCEDURE])) or
                    (not hadgeneric and (not (token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
                   not((token=_ID) and (idtoken=_OPERATOR)))) then
                  Message(parser_e_procedure_or_function_expected);

                if IsAnonOrLocal then
                  Message(parser_e_no_class_in_local_anonymous_records);

=======
=======
>>>>>>> graemeg/cpstrnew
                if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
                   not((token=_ID) and (idtoken=_OPERATOR)) then
                  Message(parser_e_procedure_or_function_expected);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                is_classdef:=true;
              end;
            _PROCEDURE,
            _FUNCTION:
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                if IsAnonOrLocal then
                  Message(parser_e_no_methods_in_local_anonymous_records);
                pd:=parse_record_method_dec(current_structdef,is_classdef,hadgeneric);
                hadgeneric:=false;
=======
=======
>>>>>>> graemeg/cpstrnew
                oldparse_only:=parse_only;
                parse_only:=true;
                pd:=parse_proc_dec(is_classdef,current_structdef);

                { this is for error recovery as well as forward }
                { interface mappings, i.e. mapping to a method  }
                { which isn't declared yet                      }
                if assigned(pd) then
                  begin
                    parse_record_proc_directives(pd);

                    { since records have no inheritance don't allow non static
                      class methods. delphi do so. }
                    if is_classdef and not (po_staticmethod in pd.procoptions) then
                      MessagePos(pd.fileinfo, parser_e_class_methods_only_static_in_records);

                    handle_calling_convention(pd);

                    { add definition to procsym }
                    proc_add_definition(pd);
                  end;

                maybe_parse_hint_directives(pd);

                parse_only:=oldparse_only;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _CONSTRUCTOR :
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                if IsAnonOrLocal then
                  Message(parser_e_no_methods_in_local_anonymous_records);
=======
                if not is_classdef then
                  Message(parser_e_no_constructor_in_records);
>>>>>>> graemeg/cpstrnew
=======
                if not is_classdef then
                  Message(parser_e_no_constructor_in_records);
>>>>>>> graemeg/cpstrnew
                if not is_classdef and (current_structdef.symtable.currentvisibility <> vis_public) then
                  Message(parser_w_constructor_should_be_public);

                { only 1 class constructor is allowed }
                if is_classdef and (oo_has_class_constructor in current_structdef.objectoptions) then
                  Message1(parser_e_only_one_class_constructor_allowed, current_structdef.objrealname^);

                oldparse_only:=parse_only;
                parse_only:=true;
                if is_classdef then
<<<<<<< HEAD
<<<<<<< HEAD
                  pd:=class_constructor_head(current_structdef)
                else
                  begin
                    pd:=constructor_head;
                    if pd.minparacount = 0 then
                      MessagePos(pd.procsym.fileinfo,parser_e_no_parameterless_constructor_in_records);
                  end;

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _DESTRUCTOR :
              begin
                if IsAnonOrLocal then
                  Message(parser_e_no_methods_in_local_anonymous_records);
                if not is_classdef then
                  Message(parser_e_no_destructor_in_records);

                { only 1 class destructor is allowed }
                if is_classdef and (oo_has_class_destructor in current_structdef.objectoptions) then
                  Message1(parser_e_only_one_class_destructor_allowed, current_structdef.objrealname^);

                oldparse_only:=parse_only;
                parse_only:=true;
                if is_classdef then
                  pd:=class_destructor_head(current_structdef)
                else
                  pd:=destructor_head;
=======
=======
>>>>>>> graemeg/cpstrnew
                  pd:=class_constructor_head
                else
                  pd:=constructor_head;
                parse_record_proc_directives(pd);
                handle_calling_convention(pd);

                { add definition to procsym }
                proc_add_definition(pd);

                maybe_parse_hint_directives(pd);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
<<<<<<< HEAD
            _END :
              begin
{$ifdef jvm}
                add_java_default_record_methods_intf(trecorddef(current_structdef));
{$endif}
                if target_info.system in systems_typed_constants_node_init then
                  add_typedconst_init_routine(current_structdef);
=======
            _DESTRUCTOR :
              begin
                if not is_classdef then
                  Message(parser_e_no_destructor_in_records);

                { only 1 class destructor is allowed }
                if is_classdef and (oo_has_class_destructor in current_structdef.objectoptions) then
                  Message1(parser_e_only_one_class_destructor_allowed, current_structdef.objrealname^);

                oldparse_only:=parse_only;
                parse_only:=true;
                if is_classdef then
                  pd:=class_destructor_head
                else
                  pd:=destructor_head;
                parse_record_proc_directives(pd);
                handle_calling_convention(pd);

                { add definition to procsym }
                proc_add_definition(pd);

                maybe_parse_hint_directives(pd);
=======
>>>>>>> graemeg/cpstrnew

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
<<<<<<< HEAD
              end;
            _END :
              begin
>>>>>>> graemeg/cpstrnew
                consume(_END);
                break;
              end;
=======
              end;
=======
              end;
            _CONST:
              begin
                consume(_CONST);
                member_blocktype:=bt_const;
              end;
            _ID, _CASE, _OPERATOR :
              begin
                case idtoken of
                  _PRIVATE :
                    begin
                       consume(_PRIVATE);
                       current_structdef.symtable.currentvisibility:=vis_private;
                       include(current_structdef.objectoptions,oo_has_private);
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PROTECTED :
                     begin
                       consume(_PROTECTED);
                       current_structdef.symtable.currentvisibility:=vis_protected;
                       include(current_structdef.objectoptions,oo_has_protected);
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PUBLIC :
                     begin
                       consume(_PUBLIC);
                       current_structdef.symtable.currentvisibility:=vis_public;
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PUBLISHED :
                     begin
                       Message(parser_e_no_record_published);
                       consume(_PUBLISHED);
                       current_structdef.symtable.currentvisibility:=vis_published;
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _STRICT :
                     begin
                        consume(_STRICT);
                        if token=_ID then
                          begin
                            case idtoken of
                              _PRIVATE:
                                begin
                                  consume(_PRIVATE);
                                  current_structdef.symtable.currentvisibility:=vis_strictprivate;
                                  include(current_structdef.objectoptions,oo_has_strictprivate);
                                end;
                              _PROTECTED:
                                begin
                                  consume(_PROTECTED);
                                  current_structdef.symtable.currentvisibility:=vis_strictprotected;
                                  include(current_structdef.objectoptions,oo_has_strictprotected);
                                end;
                              else
                                message(parser_e_protected_or_private_expected);
                            end;
                          end
                        else
                          message(parser_e_protected_or_private_expected);
                        fields_allowed:=true;
                        is_classdef:=false;
                        classfields:=false;
                        member_blocktype:=bt_general;
                     end
                    else
                    if is_classdef and (idtoken=_OPERATOR) then
                      begin
                        oldparse_only:=parse_only;
                        parse_only:=true;
                        pd:=parse_proc_dec(is_classdef,current_structdef);

                        { this is for error recovery as well as forward }
                        { interface mappings, i.e. mapping to a method  }
                        { which isn't declared yet                      }
                        if assigned(pd) then
                          begin
                            parse_record_proc_directives(pd);

                            handle_calling_convention(pd);

                            { add definition to procsym }
                            proc_add_definition(pd);
                          end;

                        maybe_parse_hint_directives(pd);

                        parse_only:=oldparse_only;
                        fields_allowed:=false;
                        is_classdef:=false;
                      end
                      else
                      begin
                        if member_blocktype=bt_general then
                          begin
                            if (not fields_allowed) then
                              Message(parser_e_field_not_allowed_here);
                            vdoptions:=[vd_record];
                            if classfields then
                              include(vdoptions,vd_class);
                            read_record_fields(vdoptions);
                          end
                        else if member_blocktype=bt_type then
                          types_dec(true)
                        else if member_blocktype=bt_const then
                          consts_dec(true)
                        else
                          internalerror(201001110);
                      end;
                end;
              end;
            _PROPERTY :
              begin
                struct_property_dec(is_classdef);
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _CLASS:
              begin
                is_classdef:=false;
                { read class method/field/property }
                consume(_CLASS);
                { class modifier is only allowed for procedures, functions, }
                { constructors, destructors, fields and properties          }
                if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
                   not((token=_ID) and (idtoken=_OPERATOR)) then
                  Message(parser_e_procedure_or_function_expected);

                is_classdef:=true;
              end;
            _PROCEDURE,
            _FUNCTION:
              begin
                oldparse_only:=parse_only;
                parse_only:=true;
                pd:=parse_proc_dec(is_classdef,current_structdef);

                { this is for error recovery as well as forward }
                { interface mappings, i.e. mapping to a method  }
                { which isn't declared yet                      }
                if assigned(pd) then
                  begin
                    parse_record_proc_directives(pd);

                    { since records have no inheritance don't allow non static
                      class methods. delphi do so. }
                    if is_classdef and not (po_staticmethod in pd.procoptions) then
                      MessagePos(pd.fileinfo, parser_e_class_methods_only_static_in_records);

                    handle_calling_convention(pd);

                    { add definition to procsym }
                    proc_add_definition(pd);
                  end;

                maybe_parse_hint_directives(pd);

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _CONSTRUCTOR :
              begin
                if not is_classdef then
                  Message(parser_e_no_constructor_in_records);
                if not is_classdef and (current_structdef.symtable.currentvisibility <> vis_public) then
                  Message(parser_w_constructor_should_be_public);

                { only 1 class constructor is allowed }
                if is_classdef and (oo_has_class_constructor in current_structdef.objectoptions) then
                  Message1(parser_e_only_one_class_constructor_allowed, current_structdef.objrealname^);

                oldparse_only:=parse_only;
                parse_only:=true;
                if is_classdef then
                  pd:=class_constructor_head
                else
                  pd:=constructor_head;
                parse_record_proc_directives(pd);
                handle_calling_convention(pd);

                { add definition to procsym }
                proc_add_definition(pd);

                maybe_parse_hint_directives(pd);

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
>>>>>>> graemeg/cpstrnew
=======
              end;
            _CONST:
              begin
                consume(_CONST);
                member_blocktype:=bt_const;
              end;
            _ID, _CASE, _OPERATOR :
              begin
                case idtoken of
                  _PRIVATE :
                    begin
                       consume(_PRIVATE);
                       current_structdef.symtable.currentvisibility:=vis_private;
                       include(current_structdef.objectoptions,oo_has_private);
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PROTECTED :
                     begin
                       consume(_PROTECTED);
                       current_structdef.symtable.currentvisibility:=vis_protected;
                       include(current_structdef.objectoptions,oo_has_protected);
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PUBLIC :
                     begin
                       consume(_PUBLIC);
                       current_structdef.symtable.currentvisibility:=vis_public;
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _PUBLISHED :
                     begin
                       Message(parser_e_no_record_published);
                       consume(_PUBLISHED);
                       current_structdef.symtable.currentvisibility:=vis_published;
                       fields_allowed:=true;
                       is_classdef:=false;
                       classfields:=false;
                       member_blocktype:=bt_general;
                     end;
                   _STRICT :
                     begin
                        consume(_STRICT);
                        if token=_ID then
                          begin
                            case idtoken of
                              _PRIVATE:
                                begin
                                  consume(_PRIVATE);
                                  current_structdef.symtable.currentvisibility:=vis_strictprivate;
                                  include(current_structdef.objectoptions,oo_has_strictprivate);
                                end;
                              _PROTECTED:
                                begin
                                  consume(_PROTECTED);
                                  current_structdef.symtable.currentvisibility:=vis_strictprotected;
                                  include(current_structdef.objectoptions,oo_has_strictprotected);
                                end;
                              else
                                message(parser_e_protected_or_private_expected);
                            end;
                          end
                        else
                          message(parser_e_protected_or_private_expected);
                        fields_allowed:=true;
                        is_classdef:=false;
                        classfields:=false;
                        member_blocktype:=bt_general;
                     end
                    else
                    if is_classdef and (idtoken=_OPERATOR) then
                      begin
                        oldparse_only:=parse_only;
                        parse_only:=true;
                        pd:=parse_proc_dec(is_classdef,current_structdef);

                        { this is for error recovery as well as forward }
                        { interface mappings, i.e. mapping to a method  }
                        { which isn't declared yet                      }
                        if assigned(pd) then
                          begin
                            parse_record_proc_directives(pd);

                            handle_calling_convention(pd);
=======
=======
>>>>>>> origin/fixes_2_2
                 string_dec(def);
>>>>>>> graemeg/fixes_2_2

                            { add definition to procsym }
                            proc_add_definition(pd);
                          end;

                        maybe_parse_hint_directives(pd);

                        parse_only:=oldparse_only;
                        fields_allowed:=false;
                        is_classdef:=false;
                      end
                      else
                      begin
                        if member_blocktype=bt_general then
                          begin
                            if (not fields_allowed) then
                              Message(parser_e_field_not_allowed_here);
                            vdoptions:=[vd_record];
                            if classfields then
                              include(vdoptions,vd_class);
                            read_record_fields(vdoptions);
                          end
                        else if member_blocktype=bt_type then
                          types_dec(true)
                        else if member_blocktype=bt_const then
                          consts_dec(true)
                        else
                          internalerror(201001110);
                      end;
                end;
              end;
            _PROPERTY :
              begin
                struct_property_dec(is_classdef);
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _CLASS:
              begin
                is_classdef:=false;
                { read class method/field/property }
                consume(_CLASS);
                { class modifier is only allowed for procedures, functions, }
                { constructors, destructors, fields and properties          }
                if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
                   not((token=_ID) and (idtoken=_OPERATOR)) then
                  Message(parser_e_procedure_or_function_expected);

<<<<<<< HEAD
                is_classdef:=true;
              end;
            _PROCEDURE,
            _FUNCTION:
              begin
                oldparse_only:=parse_only;
                parse_only:=true;
                pd:=parse_proc_dec(is_classdef,current_structdef);

                { this is for error recovery as well as forward }
                { interface mappings, i.e. mapping to a method  }
                { which isn't declared yet                      }
                if assigned(pd) then
                  begin
                    parse_record_proc_directives(pd);

                    { since records have no inheritance don't allow non static
                      class methods. delphi do so. }
                    if is_classdef and not (po_staticmethod in pd.procoptions) then
                      MessagePos(pd.fileinfo, parser_e_class_methods_only_static_in_records);

                    handle_calling_convention(pd);

                    { add definition to procsym }
                    proc_add_definition(pd);
                  end;

                maybe_parse_hint_directives(pd);

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _CONSTRUCTOR :
              begin
                if not is_classdef then
                  Message(parser_e_no_constructor_in_records);
                if not is_classdef and (current_structdef.symtable.currentvisibility <> vis_public) then
                  Message(parser_w_constructor_should_be_public);

                { only 1 class constructor is allowed }
                if is_classdef and (oo_has_class_constructor in current_structdef.objectoptions) then
                  Message1(parser_e_only_one_class_constructor_allowed, current_structdef.objrealname^);

                oldparse_only:=parse_only;
                parse_only:=true;
                if is_classdef then
                  pd:=class_constructor_head
                else
                  pd:=constructor_head;
                parse_record_proc_directives(pd);
                handle_calling_convention(pd);

                { add definition to procsym }
                proc_add_definition(pd);

                maybe_parse_hint_directives(pd);

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
>>>>>>> origin/cpstrnew
            _DESTRUCTOR :
              begin
                if not is_classdef then
                  Message(parser_e_no_destructor_in_records);

                { only 1 class destructor is allowed }
                if is_classdef and (oo_has_class_destructor in current_structdef.objectoptions) then
                  Message1(parser_e_only_one_class_destructor_allowed, current_structdef.objrealname^);

                oldparse_only:=parse_only;
                parse_only:=true;
                if is_classdef then
                  pd:=class_destructor_head
                else
                  pd:=destructor_head;
                parse_record_proc_directives(pd);
                handle_calling_convention(pd);

                { add definition to procsym }
                proc_add_definition(pd);

                maybe_parse_hint_directives(pd);

                parse_only:=oldparse_only;
                fields_allowed:=false;
                is_classdef:=false;
              end;
            _END :
              begin
                consume(_END);
                break;
              end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            else
              consume(_ID); { Give a ident expected message, like tp7 }
          end;
        until false;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        reset_typesym;
      end;

    { reads a record declaration }
    function record_dec(const n:tidstring;recsym:tsym;genericdef:tstoreddef;genericlist:tfphashobjectlist):tdef;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
               _ID:
                 begin
                   if try_to_consume(_SPECIALIZE) then
                     begin
                       dospecialize:=true;
                       again:=true;
                     end
                   else
                     begin
                       id_type(def,isforwarddef);
                       { handle types inside classes for generics, e.g. TNode.TLongint }
                       while (token=_POINT) do
                         begin
                           if parse_generic then
                             begin
                                consume(_POINT);
                                consume(_ID);
                             end
                            else if ((def.typ=objectdef) and (df_specialization in def.defoptions)) then
                              begin
                                symtablestack.push(tobjectdef(def).symtable);
                                consume(_POINT);
                                id_type(t2,isforwarddef);
                                symtablestack.pop(tobjectdef(def).symtable);
                                def:=t2;
                              end
                            else
                              break;
                         end;
                     end;
                 end;

               else
                 begin
                   message(type_e_type_id_expected);
                   def:=generrordef;
                 end;
            end;
        until not again;
        if dospecialize then
          generate_specialization(def)
        else
          begin
            if (df_generic in def.defoptions)  then
              begin
                Message(parser_e_no_generics_as_types);
                def:=generrordef;
              end;
          end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

    { reads a record declaration }
    function record_dec(const n:tidstring;genericdef:tstoreddef;genericlist:TFPObjectList):tdef;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
      end;

    { reads a record declaration }
    function record_dec(const n:tidstring;genericdef:tstoreddef;genericlist:TFPObjectList):tdef;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      var
         old_current_structdef: tabstractrecorddef;
         old_current_genericdef,
         old_current_specializedef: tstoreddef;
         old_parse_generic: boolean;
         recst: trecordsymtable;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hadgendummy : boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
         old_current_structdef:=current_structdef;
         old_current_genericdef:=current_genericdef;
         old_current_specializedef:=current_specializedef;
         old_parse_generic:=parse_generic;

         current_genericdef:=nil;
         current_specializedef:=nil;
         { create recdef }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if (n<>'') or
            not(target_info.system in systems_jvm) then
           begin
             recst:=trecordsymtable.create(n,current_settings.packrecords,current_settings.alignment.recordalignmin,current_settings.alignment.maxCrecordalign);
             { can't use recst.realname^ instead of n, because recst.realname is
               nil in case of an empty name }
             current_structdef:=crecorddef.create(n,recst);
           end
         else
           begin
             { for the JVM target records always need a name, because they are
               represented by a class }
             recst:=trecordsymtable.create(current_module.realmodulename^+'__fpc_intern_recname_'+tostr(current_module.deflist.count),
               current_settings.packrecords,current_settings.alignment.recordalignmin,current_settings.alignment.maxCrecordalign);
             current_structdef:=crecorddef.create(recst.name^,recst);
           end;
         result:=current_structdef;
         { insert in symtablestack }
         symtablestack.push(recst);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         recst:=trecordsymtable.create(n,current_settings.packrecords);
         current_structdef:=trecorddef.create(n,recst);
         result:=current_structdef;
         { insert in symtablestack }
         symtablestack.push(recst);
         { parse record }
         consume(_RECORD);
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

         { usage of specialized type inside its generic template }
         if assigned(genericdef) then
           current_specializedef:=current_structdef
         { reject declaration of generic class inside generic class }
         else if assigned(genericlist) then
           current_genericdef:=current_structdef;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         { nested types of specializations are specializations as well }
         if assigned(old_current_structdef) and
             (df_specialization in old_current_structdef.defoptions) then
           include(current_structdef.defoptions,df_specialization);
         if assigned(old_current_structdef) and
             (df_generic in old_current_structdef.defoptions) then
           include(current_structdef.defoptions,df_generic);

         insert_generic_parameter_types(current_structdef,genericdef,genericlist);
         { when we are parsing a generic already then this is a generic as
           well }
         if old_parse_generic then
           include(current_structdef.defoptions, df_generic);
         parse_generic:=(df_generic in current_structdef.defoptions);
         if parse_generic and not assigned(current_genericdef) then
           current_genericdef:=current_structdef;
         { in non-Delphi modes we need a strict private symbol without type
           count and type parameters in the name to simply resolving }
         maybe_insert_generic_rename_symbol(n,genericlist);

         if m_advanced_records in current_settings.modeswitches then
           begin
             parse_record_members(recsym);
           end
         else
           begin
             read_record_fields([vd_record],nil,nil,hadgendummy);
{$ifdef jvm}
             { we need a constructor to create temps, a deep copy helper, ... }
             add_java_default_record_methods_intf(trecorddef(current_structdef));
{$endif}
             if target_info.system in systems_typed_constants_node_init then
               add_typedconst_init_routine(current_structdef);
             consume(_END);
            end;
         { make the record size aligned (has to be done before inserting the
           parameters, because that may depend on the record's size) }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         insert_generic_parameter_types(current_structdef,genericdef,genericlist);
         parse_generic:=(df_generic in current_structdef.defoptions);
         if m_advanced_records in current_settings.modeswitches then
           parse_record_members
         else
           begin
             read_record_fields([vd_record]);
             consume(_END);
            end;
         { make the record size aligned }
>>>>>>> graemeg/cpstrnew
         recst.addalignmentpadding;
         { don't keep track of procdefs in a separate list, because the
           compiler may add additional procdefs (e.g. property wrappers for
           the jvm backend) }
         insert_record_hidden_paras(trecorddef(current_structdef));
         { restore symtable stack }
         symtablestack.pop(recst);
         if trecorddef(current_structdef).is_packed and is_managed_type(current_structdef) then
           Message(type_e_no_packed_inittable);
         { restore old state }
         parse_generic:=old_parse_generic;
         current_structdef:=old_current_structdef;
         current_genericdef:=old_current_genericdef;
         current_specializedef:=old_current_specializedef;
      end;


    { reads a type definition and returns a pointer to it }
    procedure read_named_type(var def:tdef;const newsym:tsym;genericdef:tstoreddef;genericlist:tfphashobjectlist;parseprocvardir:boolean;var hadtypetoken:boolean);
      var
        pt : tnode;
        tt2 : tdef;
        aktenumdef : tenumdef;
        s : TIDString;
        l,v : TConstExprInt;
        oldpackrecords : longint;
        defpos,storepos : tfileposinfo;
        name: TIDString;

        procedure expr_type;
        var
           pt1,pt2 : tnode;
           lv,hv   : TConstExprInt;
           old_block_type : tblock_type;
           dospecialize : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           newdef  : tdef;
           sym     : tsym;
           genstr  : string;
           gencount : longint;
=======
           structdef: tdef;
>>>>>>> graemeg/cpstrnew
=======
           structdef: tdef;
>>>>>>> graemeg/cpstrnew
=======
           structdef: tdef;
>>>>>>> graemeg/cpstrnew
=======
           structdef: tdef;
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        begin
           old_block_type:=block_type;
           dospecialize:=false;
           { use of current parsed object:
             classes, objects, records can be used also in themself }
           if (token=_ID) then
             if try_parse_structdef_nested_type(def,current_structdef,false) then
               exit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           { we can't accept a equal in type }
           pt1:=comp_expr([ef_type_only]);
           if try_to_consume(_POINTPOINT) then
             begin
               { get high value of range }
               pt2:=comp_expr([]);
=======
           { Generate a specialization in FPC mode? }
           dospecialize:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_SPECIALIZE);
           { we can't accept a equal in type }
=======
           { Generate a specialization in FPC mode? }
           dospecialize:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_SPECIALIZE);
           { we can't accept a equal in type }
>>>>>>> graemeg/cpstrnew
=======
           { Generate a specialization in FPC mode? }
           dospecialize:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_SPECIALIZE);
           { we can't accept a equal in type }
>>>>>>> graemeg/cpstrnew
=======
           { Generate a specialization in FPC mode? }
           dospecialize:=not(m_delphi in current_settings.modeswitches) and try_to_consume(_SPECIALIZE);
           { we can't accept a equal in type }
>>>>>>> origin/cpstrnew
           pt1:=comp_expr(false,true);
=======
             end;
           { Generate a specialization? }
           if try_to_consume(_SPECIALIZE) then
             dospecialize:=true;
           { we can't accept a equal in type }
           pt1:=comp_expr(false);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           if not dospecialize and
              try_to_consume(_POINTPOINT) then
             begin
               { get high value of range }
               pt2:=comp_expr(false,false);
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
               { make both the same type or give an error. This is not
                 done when both are integer values, because typecasting
                 between -3200..3200 will result in a signed-unsigned
                 conflict and give a range check error (PFV) }
               if not(is_integer(pt1.resultdef) and is_integer(pt2.resultdef)) then
                 inserttypeconv(pt1,pt2.resultdef);
               { both must be evaluated to constants now }
               if (pt1.nodetype=ordconstn) and
                  (pt2.nodetype=ordconstn) then
                 begin
                   lv:=tordconstnode(pt1).value;
                   hv:=tordconstnode(pt2).value;
                   { Check bounds }
                   if hv<lv then
                     message(parser_e_upper_lower_than_lower)
                   else if (lv.signed and (lv.svalue<0)) and (not hv.signed and (hv.uvalue>qword(high(int64)))) then
                     message(type_e_cant_eval_constant_expr)
                   else
                     begin
                       { All checks passed, create the new def }
                       case pt1.resultdef.typ of
                         enumdef :
                           def:=cenumdef.create_subrange(tenumdef(pt1.resultdef),lv.svalue,hv.svalue);
                         orddef :
                           begin
                             if is_char(pt1.resultdef) then
                               def:=corddef.create(uchar,lv,hv,true)
                             else
                               if is_boolean(pt1.resultdef) then
                                 def:=corddef.create(pasbool8,lv,hv,true)
                               else if is_signed(pt1.resultdef) then
                                 def:=corddef.create(range_to_basetype(lv,hv),lv,hv,true)
                               else
                                 def:=corddef.create(range_to_basetype(lv,hv),lv,hv,true);
                           end;
                       end;
                     end;
                 end
               else
                 Message(sym_e_error_in_type_def);
               pt2.free;
             end
           else
             begin
               { a simple type renaming or generic specialization }
               if (pt1.nodetype=typen) then
                 begin
                   def:=ttypenode(pt1).resultdef;
<<<<<<< HEAD
<<<<<<< HEAD
                   { Delphi mode specialization? }
                   if (m_delphi in current_settings.modeswitches) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     dospecialize:=token=_LSHARPBRACKET
                   else
                     begin
                       dospecialize:=false;
                       { in non-Delphi modes we might get a inline specialization
                         without "specialize" or "<T>" of the same type we're
                         currently parsing, so we need to handle that special }
                       newdef:=nil;
                     end;
                   if not dospecialize and
                       assigned(ttypenode(pt1).typesym) and
                       (ttypenode(pt1).typesym.typ=typesym) and
                       (sp_generic_dummy in ttypenode(pt1).typesym.symoptions) and
                       assigned(current_structdef) and
                       (
                         (
                           not (m_delphi in current_settings.modeswitches) and
                           (ttypesym(ttypenode(pt1).typesym).typedef.typ=undefineddef) and
                           (df_generic in current_structdef.defoptions) and
                           (ttypesym(ttypenode(pt1).typesym).typedef.owner=current_structdef.owner) and
                           (upper(ttypenode(pt1).typesym.realname)=copy(current_structdef.objname^,1,pos('$',current_structdef.objname^)-1))
                         ) or (
                           { this could be a nested specialization which uses
                             the type name of a surrounding generic to
                             reference the specialization of said surrounding
                             class }
                           (df_specialization in current_structdef.defoptions) and
                           return_specialization_of_generic(current_structdef,ttypesym(ttypenode(pt1).typesym).typedef,newdef)
                         )
                       )
                       then
                     begin
                       if assigned(newdef) then
                         def:=newdef
                       else
                         def:=current_structdef;
                       if assigned(def) then
                         { handle nested types }
                         post_comp_expr_gendef(def)
                       else
                         def:=generrordef;
                     end;
                   if dospecialize then
                     begin
                       generate_specialization(def,false,name);
                       { handle nested types }
                       if assigned(def) then
                         post_comp_expr_gendef(def);
                     end
=======
                     dospecialize:=token=_LSHARPBRACKET;
                   if dospecialize then
                     generate_specialization(def,false)
>>>>>>> graemeg/cpstrnew
=======
                     dospecialize:=token=_LSHARPBRACKET;
                   if dospecialize then
                     generate_specialization(def,false)
>>>>>>> graemeg/cpstrnew
                   else
                     begin
                       if assigned(current_specializedef) and (def=current_specializedef.genericdef) then
                         begin
                           def:=current_specializedef
                         end
                       else if (def=current_genericdef) then
                         begin
                           def:=current_genericdef
                         end
<<<<<<< HEAD
<<<<<<< HEAD
                       else if tstoreddef(def).is_generic and
                           { TODO : check once nested generics are allowed }
                           not
                             (
                               parse_generic and
                               (current_genericdef.typ in [recorddef,objectdef]) and
                               (def.typ in [recorddef,objectdef]) and
                               (
                                 { if both defs belong to the same generic (e.g. both are
                                   subtypes) then we must allow the usage }
                                 defs_belong_to_same_generic(def,current_genericdef) or
                                 { this is needed to correctly resolve "type Foo=SomeGeneric<T>"
                                   declarations inside a generic }
                                 (
                                   (ttypenode(pt1).typesym<>nil) and
                                   sym_is_owned_by(ttypenode(pt1).typesym,tabstractrecorddef(current_genericdef).symtable)
                                 )
                               )
                             )
                           then
                         begin
                           if assigned(def.typesym) then
                             begin
                               if ttypesym(def.typesym).typedef.typ<>undefineddef then
                                 { non-Delphi modes... }
                                 split_generic_name(def.typesym.name,genstr,gencount)
                               else
                                 genstr:=def.typesym.name;
                               sym:=resolve_generic_dummysym(genstr);
                             end
                           else
                             sym:=nil;
                           if assigned(sym) and
                               not (sp_generic_dummy in sym.symoptions) and
                               (sym.typ=typesym) then
                             def:=ttypesym(sym).typedef
                           else
                             begin
                               Message(parser_e_no_generics_as_types);
                               def:=generrordef;
                             end;
                         end
                       else if is_classhelper(def) then
=======
=======
>>>>>>> origin/cpstrnew
                     dospecialize:=token=_LSHARPBRACKET;
                   if dospecialize then
                     generate_specialization(def,false)
                   else
                     begin
                       if assigned(current_specializedef) and (def=current_specializedef.genericdef) then
                         begin
                           def:=current_specializedef
                         end
                       else if (def=current_genericdef) then
                         begin
                           def:=current_genericdef
                         end
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                       else if (df_generic in def.defoptions) then
                         begin
                           Message(parser_e_no_generics_as_types);
                           def:=generrordef;
                         end
                       else if is_objccategory(def) then
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
                         begin
                           Message(parser_e_no_category_as_types);
                           def:=generrordef
                         end
=======
=======
>>>>>>> origin/fixes_2_2
                   if dospecialize then
                     generate_specialization(def)
                   else
                     begin
                       if (df_generic in def.defoptions)  then
                         begin
                           Message(parser_e_no_generics_as_types);
                           def:=generrordef;
                         end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                     end;
                 end
               else
                 Message(sym_e_error_in_type_def);
             end;
           pt1.free;
           block_type:=old_block_type;
        end;


      procedure set_dec;
        begin
          consume(_SET);
          consume(_OF);
          read_anon_type(tt2,true);
          if assigned(tt2) then
           begin
             case tt2.typ of
               { don't forget that min can be negativ  PM }
               enumdef :
                 if (tenumdef(tt2).min>=0) and
                    (tenumdef(tt2).max<=255) then
<<<<<<< HEAD
<<<<<<< HEAD
                  // !! def:=csetdef.create(tt2,tenumdef(tt2.def).min,tenumdef(tt2.def).max),true)
                  def:=csetdef.create(tt2,tenumdef(tt2).min,tenumdef(tt2).max,true)
=======
                  // !! def:=tsetdef.create(tt2,tenumdef(tt2.def).min,tenumdef(tt2.def).max))
                  def:=tsetdef.create(tt2,tenumdef(tt2).min,tenumdef(tt2).max)
>>>>>>> graemeg/fixes_2_2
=======
                  // !! def:=tsetdef.create(tt2,tenumdef(tt2.def).min,tenumdef(tt2.def).max))
                  def:=tsetdef.create(tt2,tenumdef(tt2).min,tenumdef(tt2).max)
>>>>>>> origin/fixes_2_2
                 else
                  Message(sym_e_ill_type_decl_set);
               orddef :
                 begin
                   if (torddef(tt2).ordtype<>uvoid) and
                      (torddef(tt2).ordtype<>uwidechar) and
                      (torddef(tt2).low>=0) then
<<<<<<< HEAD
                     // !! def:=csetdef.create(tt2,torddef(tt2.def).low,torddef(tt2.def).high),true)
                     if Torddef(tt2).high>int64(high(byte)) then
                       message(sym_e_ill_type_decl_set)
                     else
                       def:=csetdef.create(tt2,torddef(tt2).low.svalue,torddef(tt2).high.svalue,true)
=======
                     // !! def:=tsetdef.create(tt2,torddef(tt2.def).low,torddef(tt2.def).high))
                     if Torddef(tt2).high>int64(high(byte)) then
                       message(sym_e_ill_type_decl_set)
                     else
                       def:=tsetdef.create(tt2,torddef(tt2).low,torddef(tt2).high)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                   else
                     Message(sym_e_ill_type_decl_set);
                 end;
               else
                 Message(sym_e_ill_type_decl_set);
             end;
           end
          else
           def:=generrordef;
        end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      procedure array_dec(is_packed:boolean;genericdef:tstoreddef;genericlist:tfphashobjectlist);
=======
      procedure array_dec(is_packed:boolean;genericdef:tstoreddef;genericlist:TFPObjectList);
>>>>>>> graemeg/cpstrnew
=======
      procedure array_dec(is_packed:boolean;genericdef:tstoreddef;genericlist:TFPObjectList);
>>>>>>> graemeg/cpstrnew
=======
      procedure array_dec(is_packed:boolean;genericdef:tstoreddef;genericlist:TFPObjectList);
>>>>>>> graemeg/cpstrnew
=======
      procedure array_dec(is_packed:boolean;genericdef:tstoreddef;genericlist:TFPObjectList);
>>>>>>> origin/cpstrnew
        var
          lowval,
          highval   : TConstExprInt;
          indexdef  : tdef;
          hdef      : tdef;
          arrdef    : tarraydef;

        procedure setdefdecl(def:tdef);
          begin
            case def.typ of
              enumdef :
                begin
                  lowval:=tenumdef(def).min;
                  highval:=tenumdef(def).max;
                  if (m_fpc in current_settings.modeswitches) and
                     (tenumdef(def).has_jumps) then
                   Message(type_e_array_index_enums_with_assign_not_possible);
                  indexdef:=def;
                end;
              orddef :
                begin
                  if torddef(def).ordtype in [uchar,
                    u8bit,
                    s8bit,s16bit,
{$if defined(cpu32bitaddr) or defined(cpu64bitaddr)}
                    u16bit,s32bit,
{$endif defined(cpu32bitaddr) or defined(cpu64bitaddr)}
{$ifdef cpu64bitaddr}
                    u32bit,s64bit,
{$endif cpu64bitaddr}
                    pasbool8,pasbool16,pasbool32,pasbool64,
                    bool8bit,bool16bit,bool32bit,bool64bit,
                    uwidechar] then
                    begin
                       lowval:=torddef(def).low;
                       highval:=torddef(def).high;
                       indexdef:=def;
                    end
                  else
                    Message1(parser_e_type_cant_be_used_in_array_index,def.typename);
                end;
              else
                Message(sym_e_error_in_type_def);
            end;
          end;

        var
          old_current_genericdef,
          old_current_specializedef: tstoreddef;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          first,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          old_parse_generic: boolean;
        begin
           old_current_genericdef:=current_genericdef;
           old_current_specializedef:=current_specializedef;
           old_parse_generic:=parse_generic;

           current_genericdef:=nil;
           current_specializedef:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           first:=true;
           arrdef:=carraydef.create(0,0,s32inttype);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
           arrdef:=nil;
>>>>>>> graemeg/cpstrnew
           consume(_ARRAY);

           { usage of specialized type inside its generic template }
           if assigned(genericdef) then
             current_specializedef:=arrdef
           { reject declaration of generic class inside generic class }
           else if assigned(genericlist) then
             current_genericdef:=arrdef;
           symtablestack.push(arrdef.symtable);
           insert_generic_parameter_types(arrdef,genericdef,genericlist);
           { there are two possibilties for the following to be true:
             * the array declaration itself is generic
             * the array is declared inside a generic
             in both cases we need "parse_generic" and "current_genericdef"
             so that e.g. specializations of another generic inside the
             current generic can be used (either inline ones or "type" ones) }
           if old_parse_generic then
             include(arrdef.defoptions,df_generic);
           parse_generic:=(df_generic in arrdef.defoptions);
           if parse_generic and not assigned(current_genericdef) then
             current_genericdef:=old_current_genericdef;

           { open array? }
           if try_to_consume(_LECKKLAMMER) then
             begin
                { defaults }
                indexdef:=generrordef;
                { use defaults which don't overflow the compiler }
                lowval:=0;
                highval:=0;
                repeat
                  { read the expression and check it, check apart if the
                    declaration is an enum declaration because that needs to
                    be parsed by readtype (PFV) }
                  if token=_LKLAMMER then
                   begin
                     read_anon_type(hdef,true);
                     setdefdecl(hdef);
                   end
                  else
                   begin
                     pt:=expr(true);
                     if pt.nodetype=typen then
                       setdefdecl(pt.resultdef)
                     else
                       begin
<<<<<<< HEAD
<<<<<<< HEAD
                         if pt.nodetype=rangen then
=======
                         if (pt.nodetype=rangen) then
>>>>>>> graemeg/fixes_2_2
=======
                         if (pt.nodetype=rangen) then
>>>>>>> origin/fixes_2_2
                           begin
                             { pure ordconstn expressions can be checked for
                               generics as well, but don't give an error in case
                               of parsing a generic if that isn't yet the case }
                             if (trangenode(pt).left.nodetype=ordconstn) and
                                (trangenode(pt).right.nodetype=ordconstn) then
                               begin
                                 { make both the same type or give an error. This is not
                                   done when both are integer values, because typecasting
                                   between -3200..3200 will result in a signed-unsigned
                                   conflict and give a range check error (PFV) }
                                 if not(is_integer(trangenode(pt).left.resultdef) and is_integer(trangenode(pt).left.resultdef)) then
                                   inserttypeconv(trangenode(pt).left,trangenode(pt).right.resultdef);
                                 lowval:=tordconstnode(trangenode(pt).left).value;
                                 highval:=tordconstnode(trangenode(pt).right).value;
                                 if highval<lowval then
                                  begin
                                    Message(parser_e_array_lower_less_than_upper_bound);
                                    highval:=lowval;
                                  end
<<<<<<< HEAD
<<<<<<< HEAD
                                 else if (lowval<int64(low(asizeint))) or
                                         (highval>high(asizeint)) then
=======
                                 else if (lowval<int64(low(aint))) or
                                         (highval > high(aint)) then
>>>>>>> graemeg/fixes_2_2
=======
                                 else if (lowval<int64(low(aint))) or
                                         (highval > high(aint)) then
>>>>>>> origin/fixes_2_2
                                   begin
                                     Message(parser_e_array_range_out_of_bounds);
                                     lowval :=0;
                                     highval:=0;
                                   end;
                                 if is_integer(trangenode(pt).left.resultdef) then
                                   range_to_type(lowval,highval,indexdef)
                                 else
                                   indexdef:=trangenode(pt).left.resultdef;
                               end
                             else
<<<<<<< HEAD
<<<<<<< HEAD
                               if not parse_generic then
                                 Message(type_e_cant_eval_constant_expr)
                               else
                                 { we need a valid range for debug information }
                                 range_to_type(lowval,highval,indexdef);
=======
                               Message(type_e_cant_eval_constant_expr);
>>>>>>> graemeg/fixes_2_2
=======
                               Message(type_e_cant_eval_constant_expr);
>>>>>>> origin/fixes_2_2
                           end
                         else
                           Message(sym_e_error_in_type_def)
                       end;
                     pt.free;
                   end;

                  { if we are not at the first dimension, add the new arrray
                    as element of the existing array, otherwise modify the existing array }
                  if not(first) then
                    begin
                      arrdef.elementdef:=carraydef.create(lowval.svalue,highval.svalue,indexdef);
                      { push new symtable }
                      symtablestack.pop(arrdef.symtable);
                      arrdef:=tarraydef(arrdef.elementdef);
                      symtablestack.push(arrdef.symtable);
                    end
                  else
                    begin
                      arrdef.lowrange:=lowval.svalue;
                      arrdef.highrange:=highval.svalue;
                      arrdef.rangedef:=indexdef;
                      def:=arrdef;
                      first:=false;
                    end;
                  if is_packed then
                    include(arrdef.arrayoptions,ado_IsBitPacked);

                  if token=_COMMA then
                    consume(_COMMA)
                  else
                    break;
                until false;
                consume(_RECKKLAMMER);
             end
           else
             begin
                if is_packed then
                  Message(parser_e_packed_dynamic_open_array);
                arrdef.lowrange:=0;
                arrdef.highrange:=-1;
                arrdef.rangedef:=s32inttype;
                include(arrdef.arrayoptions,ado_IsDynamicArray);
                def:=arrdef;
             end;
           if assigned(arrdef) then
             begin
               { usage of specialized type inside its generic template }
               if assigned(genericdef) then
                 current_specializedef:=arrdef
               { reject declaration of generic class inside generic class }
               else if assigned(genericlist) then
                 current_genericdef:=arrdef;
               symtablestack.push(arrdef.symtable);
               insert_generic_parameter_types(arrdef,genericdef,genericlist);
               parse_generic:=(df_generic in arrdef.defoptions);
             end;
           consume(_OF);
           read_anon_type(tt2,true);
           { set element type of the last array definition }
           if assigned(arrdef) then
             begin
               symtablestack.pop(arrdef.symtable);
               arrdef.elementdef:=tt2;
               if is_packed and
                  is_managed_type(tt2) then
                 Message(type_e_no_packed_inittable);
             end;
           { restore old state }
           parse_generic:=old_parse_generic;
           current_genericdef:=old_current_genericdef;
           current_specializedef:=old_current_specializedef;
        end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

        function procvar_dec(genericdef:tstoreddef;genericlist:tfphashobjectlist):tdef;
=======
        function procvar_dec(genericdef:tstoreddef;genericlist:TFPObjectList):tdef;
>>>>>>> graemeg/cpstrnew
=======
        function procvar_dec(genericdef:tstoreddef;genericlist:TFPObjectList):tdef;
>>>>>>> graemeg/cpstrnew
=======
        function procvar_dec(genericdef:tstoreddef;genericlist:TFPObjectList):tdef;
>>>>>>> graemeg/cpstrnew
=======
        function procvar_dec(genericdef:tstoreddef;genericlist:TFPObjectList):tdef;
>>>>>>> origin/cpstrnew
          var
            is_func:boolean;
            pd:tabstractprocdef;
            newtype:ttypesym;
            old_current_genericdef,
            old_current_specializedef: tstoreddef;
            old_parse_generic: boolean;
          begin
            old_current_genericdef:=current_genericdef;
            old_current_specializedef:=current_specializedef;
            old_parse_generic:=parse_generic;

            current_genericdef:=nil;
            current_specializedef:=nil;

            is_func:=(token=_FUNCTION);
            consume(token);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            pd:=cprocvardef.create(normal_function_level);
=======
            pd:=tprocvardef.create(normal_function_level);
>>>>>>> graemeg/cpstrnew
=======
            pd:=tprocvardef.create(normal_function_level);
>>>>>>> graemeg/cpstrnew
=======
            pd:=tprocvardef.create(normal_function_level);
>>>>>>> graemeg/cpstrnew
=======
            pd:=tprocvardef.create(normal_function_level);
>>>>>>> origin/cpstrnew

            { usage of specialized type inside its generic template }
            if assigned(genericdef) then
              current_specializedef:=pd
            { reject declaration of generic class inside generic class }
            else if assigned(genericlist) then
              current_genericdef:=pd;
            symtablestack.push(pd.parast);
            insert_generic_parameter_types(pd,genericdef,genericlist);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            { there are two possibilties for the following to be true:
              * the procvar declaration itself is generic
              * the procvar is declared inside a generic
              in both cases we need "parse_generic" and "current_genericdef"
              so that e.g. specializations of another generic inside the
              current generic can be used (either inline ones or "type" ones) }
            if old_parse_generic then
              include(pd.defoptions,df_generic);
            parse_generic:=(df_generic in pd.defoptions);
            if parse_generic and not assigned(current_genericdef) then
              current_genericdef:=old_current_genericdef;
=======
            parse_generic:=(df_generic in pd.defoptions);
>>>>>>> graemeg/cpstrnew
=======
            parse_generic:=(df_generic in pd.defoptions);
>>>>>>> graemeg/cpstrnew
=======
            parse_generic:=(df_generic in pd.defoptions);
>>>>>>> graemeg/cpstrnew
=======
            parse_generic:=(df_generic in pd.defoptions);
>>>>>>> origin/cpstrnew
            { don't allow to add defs to the symtable - use it for type param search only }
            tparasymtable(pd.parast).readonly:=true;

            if token=_LKLAMMER then
              parse_parameter_dec(pd);
            if is_func then
              begin
                consume(_COLON);
                single_type(pd.returndef,[]);
              end;
            if try_to_consume(_OF) then
              begin
                consume(_OBJECT);
                include(pd.procoptions,po_methodpointer);
              end
            else if (m_nested_procvars in current_settings.modeswitches) and
                    try_to_consume(_IS) then
              begin
                consume(_NESTED);
                pd.parast.symtablelevel:=normal_function_level+1;
                pd.check_mark_as_nested;
              end;
            symtablestack.pop(pd.parast);
            tparasymtable(pd.parast).readonly:=false;
            result:=pd;
            { possible proc directives }
            if parseprocvardir then
              begin
                if check_proc_directive(true) then
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    newtype:=ctypesym.create('unnamed',result,true);
=======
                    newtype:=ttypesym.create('unnamed',result);
>>>>>>> graemeg/cpstrnew
=======
                    newtype:=ttypesym.create('unnamed',result);
>>>>>>> graemeg/cpstrnew
=======
                    newtype:=ttypesym.create('unnamed',result);
>>>>>>> graemeg/cpstrnew
=======
                    newtype:=ttypesym.create('unnamed',result);
>>>>>>> origin/cpstrnew
                    parse_var_proc_directives(tsym(newtype));
                    newtype.typedef:=nil;
                    result.typesym:=nil;
                    newtype.free;
                  end;
                { Add implicit hidden parameters and function result }
                handle_calling_convention(pd);
              end;
            { restore old state }
            parse_generic:=old_parse_generic;
            current_genericdef:=old_current_genericdef;
            current_specializedef:=old_current_specializedef;
          end;

      const
        SingleTypeOptionsInTypeBlock:array[Boolean] of TSingleTypeOptions = ([],[stoIsForwardDef]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        SingleTypeOptionsIsDelphi:array[Boolean] of TSingleTypeOptions = ([],[stoAllowSpecialization]);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      var
        p  : tnode;
        hdef : tdef;
        enumdupmsg, first, is_specialize : boolean;
        oldlocalswitches : tlocalswitches;
        bitpacking: boolean;
        stitem: psymtablestackitem;
        sym: tsym;
        st: tsymtable;
      begin
         def:=nil;
         v:=0;
         l:=0;
         if assigned(newsym) then
           name:=newsym.RealName
         else
           name:='';
         case token of
            _STRING,_FILE:
              begin
                single_type(def,[stoAllowTypeDef]);
              end;
           _LKLAMMER:
              begin
                consume(_LKLAMMER);
                first:=true;
                { allow negativ value_str }
                l:=int64(-1);
                enumdupmsg:=false;
                { check that we are not adding an enum from specialization
                  we can't just use current_specializedef because of inner types
                  like specialize array of record }
                is_specialize:=false;
                stitem:=symtablestack.stack;
                while assigned(stitem) do
                  begin
                    { check records, classes and arrays because they can be specialized }
                    if stitem^.symtable.symtabletype in [recordsymtable,ObjectSymtable,arraysymtable] then
                      begin
                        is_specialize:=is_specialize or (df_specialization in tstoreddef(stitem^.symtable.defowner).defoptions);
                        stitem:=stitem^.next;
                      end
                    else
                      break;
                  end;
                if not is_specialize then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  aktenumdef:=cenumdef.create
=======
                  aktenumdef:=tenumdef.create
>>>>>>> graemeg/cpstrnew
=======
                  aktenumdef:=tenumdef.create
>>>>>>> graemeg/cpstrnew
=======
                  aktenumdef:=tenumdef.create
>>>>>>> graemeg/cpstrnew
=======
                  aktenumdef:=tenumdef.create
>>>>>>> origin/cpstrnew
                else
                  aktenumdef:=nil;
                repeat
                  { if it is a specialization then search the first enum member
                    and get the member owner instead of just created enumdef }
                  if not assigned(aktenumdef) then
                    begin
                      searchsym(pattern,sym,st);
                      if sym.typ=enumsym then
                        aktenumdef:=tenumsym(sym).definition
                      else
                        internalerror(201101021);
                    end;
                  s:=orgpattern;
                  defpos:=current_tokenpos;
                  consume(_ID);
                  { only allow assigning of specific numbers under fpc mode }
                  if not(m_tp7 in current_settings.modeswitches) and
                     (
                      { in fpc mode also allow := to be compatible
                        with previous 1.0.x versions }
                      ((m_fpc in current_settings.modeswitches) and
                       try_to_consume(_ASSIGNMENT)) or
                      try_to_consume(_EQ)
                     ) then
                    begin
                       oldlocalswitches:=current_settings.localswitches;
                       include(current_settings.localswitches,cs_allow_enum_calc);
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
                       current_settings.localswitches:=oldlocalswitches;
                       if (p.nodetype=ordconstn) then
                        begin
                          { we expect an integer or an enum of the
                            same type }
                          if is_integer(p.resultdef) or
                             is_char(p.resultdef) or
                             equal_defs(p.resultdef,aktenumdef) then
                           v:=tordconstnode(p).value
                          else
                           IncompatibleTypes(p.resultdef,s32inttype);
                        end
                       else
                        Message(parser_e_illegal_expression);
                       p.free;
                       { please leave that a note, allows type save }
                       { declarations in the win32 units ! }
                       if (not first) and (v<=l) and (not enumdupmsg) then
                        begin
                          Message(parser_n_duplicate_enum);
                          enumdupmsg:=true;
                        end;
                       l:=v;
                    end
                  else
                    inc(l.svalue);
                  first:=false;
                  { don't generate enum members is this is a specialization because aktenumdef is copied from the generic type }
                  if not is_specialize then
                    begin
                      storepos:=current_tokenpos;
                      current_tokenpos:=defpos;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      tenumsymtable(aktenumdef.symtable).insert(cenumsym.create(s,aktenumdef,longint(l.svalue)));
                      if not (cs_scopedenums in current_settings.localswitches) then
                        tstoredsymtable(aktenumdef.owner).insert(cenumsym.create(s,aktenumdef,longint(l.svalue)));
=======
                      tenumsymtable(aktenumdef.symtable).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
                      if not (cs_scopedenums in current_settings.localswitches) then
                        tstoredsymtable(aktenumdef.owner).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
>>>>>>> graemeg/cpstrnew
=======
                      tenumsymtable(aktenumdef.symtable).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
                      if not (cs_scopedenums in current_settings.localswitches) then
                        tstoredsymtable(aktenumdef.owner).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
>>>>>>> graemeg/cpstrnew
=======
                      tenumsymtable(aktenumdef.symtable).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
                      if not (cs_scopedenums in current_settings.localswitches) then
                        tstoredsymtable(aktenumdef.owner).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
>>>>>>> graemeg/cpstrnew
=======
                      tenumsymtable(aktenumdef.symtable).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
                      if not (cs_scopedenums in current_settings.localswitches) then
                        tstoredsymtable(aktenumdef.owner).insert(tenumsym.create(s,aktenumdef,longint(l.svalue)));
>>>>>>> origin/cpstrnew
                      current_tokenpos:=storepos;
                    end;
                until not try_to_consume(_COMMA);
                def:=aktenumdef;
                consume(_RKLAMMER);
{$ifdef jvm}
                jvm_maybe_create_enum_class(name,def);
{$endif}
              end;
            _ARRAY:
              begin
                array_dec(false,genericdef,genericlist);
              end;
            _SET:
              begin
                set_dec;
              end;
           _CARET:
              begin
                consume(_CARET);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                single_type(tt2,
                    SingleTypeOptionsInTypeBlock[block_type=bt_type]+
                    SingleTypeOptionsIsDelphi[m_delphi in current_settings.modeswitches]
                  );
                { in case of e.g. var or const sections we need to especially
                  check that we don't use a generic dummy symbol }
                if (block_type<>bt_type) and
                    (tt2.typ=undefineddef) and
                    assigned(tt2.typesym) and
                    (sp_generic_dummy in tt2.typesym.symoptions) then
                  begin
                    sym:=resolve_generic_dummysym(tt2.typesym.name);
                    if assigned(sym) and
                        not (sp_generic_dummy in sym.symoptions) and
                        (sym.typ=typesym) then
                      tt2:=ttypesym(sym).typedef
                    else
                      Message(parser_e_no_generics_as_types);
                  end;
                { don't use cpointerdef.getreusable() here, since this is a type
                  declaration (-> must create new typedef) }
                def:=cpointerdef.create(tt2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                single_type(tt2,SingleTypeOptionsInTypeBlock[block_type=bt_type]);
                def:=tpointerdef.create(tt2);
>>>>>>> graemeg/cpstrnew
                if tt2.typ=forwarddef then
                  current_module.checkforwarddefs.add(def);
              end;
            _RECORD:
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                consume(token);
                if (idtoken=_HELPER) and (m_advanced_records in current_settings.modeswitches) then
                  begin
                    consume(_HELPER);
                    def:=object_dec(odt_helper,name,newsym,genericdef,genericlist,nil,ht_record);
                  end
                else
                  def:=record_dec(name,newsym,genericdef,genericlist);
=======
                def:=record_dec(name,genericdef,genericlist);
>>>>>>> graemeg/cpstrnew
=======
                def:=record_dec(name,genericdef,genericlist);
>>>>>>> graemeg/cpstrnew
=======
                def:=record_dec(name,genericdef,genericlist);
>>>>>>> graemeg/cpstrnew
=======
                def:=record_dec(name,genericdef,genericlist);
>>>>>>> origin/cpstrnew
              end;
            _PACKED,
            _BITPACKED:
              begin
                bitpacking :=
                  (cs_bitpacking in current_settings.localswitches) or
                  (token = _BITPACKED);
                consume(token);
                if token=_ARRAY then
                  array_dec(bitpacking,genericdef,genericlist)
                else if token=_SET then
                  set_dec
                else if token=_FILE then
                  single_type(def,[stoAllowTypeDef])
                else
                  begin
                    oldpackrecords:=current_settings.packrecords;
                    if (not bitpacking) or
                       (token in [_CLASS,_OBJECT]) then
                      current_settings.packrecords:=1
                    else
                      current_settings.packrecords:=bit_alignment;
                    case token of
                      _CLASS :
                        begin
                          consume(_CLASS);
                          def:=object_dec(odt_class,name,newsym,genericdef,genericlist,nil,ht_none);
                        end;
                      _OBJECT :
                        begin
                          consume(_OBJECT);
                          def:=object_dec(odt_object,name,newsym,genericdef,genericlist,nil,ht_none);
                        end;
<<<<<<< HEAD
                      else begin
                        consume(_RECORD);
                        def:=record_dec(name,newsym,genericdef,genericlist);
                      end;
=======
                      else
                        def:=record_dec(name,genericdef,genericlist);
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
                    current_settings.packrecords:=oldpackrecords;
                  end;
              end;
            _DISPINTERFACE :
              begin
                { need extra check here since interface is a keyword
                  in all pascal modes }
                if not(m_class in current_settings.modeswitches) then
                  Message(parser_f_need_objfpc_or_delphi_mode);
                consume(token);
                def:=object_dec(odt_dispinterface,name,newsym,genericdef,genericlist,nil,ht_none);
              end;
            _CLASS :
              begin
                consume(token);
                { Delphi only allows class of in type blocks }
                if (token=_OF) and
                   (
                    not(m_delphi in current_settings.modeswitches) or
                    (block_type=bt_type)
                   ) then
                  begin
                    consume(_OF);
                    single_type(hdef,SingleTypeOptionsInTypeBlock[block_type=bt_type]);
                    if is_class(hdef) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       is_objcclass(hdef) or
                       is_javaclass(hdef) then
                      def:=cclassrefdef.create(hdef)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                       is_objcclass(hdef) then
                      def:=tclassrefdef.create(hdef)
>>>>>>> graemeg/cpstrnew
                    else
                      if hdef.typ=forwarddef then
                        begin
                          def:=cclassrefdef.create(hdef);
                          current_module.checkforwarddefs.add(def);
                        end
                    else
                      Message1(type_e_class_or_objcclass_type_expected,hdef.typename);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  end
                else
                if (idtoken=_HELPER) then
                  begin
                    consume(_HELPER);
                    def:=object_dec(odt_helper,name,newsym,genericdef,genericlist,nil,ht_class);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                  end
                else
                  def:=object_dec(default_class_type,name,newsym,genericdef,genericlist,nil,ht_none);
              end;
            _CPPCLASS :
              begin
                consume(token);
                def:=object_dec(odt_cppclass,name,newsym,genericdef,genericlist,nil,ht_none);
              end;
            _OBJCCLASS :
              begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
                def:=object_dec(odt_objcclass,name,newsym,genericdef,genericlist,nil,ht_none);
              end;
            _OBJCCLASS :
              begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
                def:=object_dec(odt_objcclass,name,genericdef,genericlist,nil);
              end;
            _OBJCCLASS :
              begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
                def:=object_dec(odt_objcclass,name,genericdef,genericlist,nil);
              end;
            _OBJCCLASS :
              begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
                def:=object_dec(odt_objcclass,name,genericdef,genericlist,nil);
              end;
            _OBJCCLASS :
              begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
                def:=object_dec(odt_objcclass,name,genericdef,genericlist,nil);
              end;
            _INTERFACE :
              begin
                { need extra check here since interface is a keyword
                  in all pascal modes }
                if not(m_class in current_settings.modeswitches) then
                  Message(parser_f_need_objfpc_or_delphi_mode);
                consume(token);
                case current_settings.interfacetype of
                  it_interfacecom:
                    def:=object_dec(odt_interfacecom,name,newsym,genericdef,genericlist,nil,ht_none);
                  it_interfacecorba:
                    def:=object_dec(odt_interfacecorba,name,newsym,genericdef,genericlist,nil,ht_none);
                  it_interfacejava:
                    def:=object_dec(odt_interfacejava,name,newsym,genericdef,genericlist,nil,ht_none);
                  else
                    internalerror(2010122612);
                end;
              end;
            _OBJCPROTOCOL :
               begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                def:=object_dec(odt_objcprotocol,name,newsym,genericdef,genericlist,nil,ht_none);
=======
                def:=object_dec(odt_objcprotocol,name,genericdef,genericlist,nil);
>>>>>>> graemeg/cpstrnew
=======
                def:=object_dec(odt_objcprotocol,name,genericdef,genericlist,nil);
>>>>>>> graemeg/cpstrnew
=======
                def:=object_dec(odt_objcprotocol,name,genericdef,genericlist,nil);
>>>>>>> graemeg/cpstrnew
=======
                def:=object_dec(odt_objcprotocol,name,genericdef,genericlist,nil);
>>>>>>> origin/cpstrnew
               end;
            _OBJCCATEGORY :
               begin
                if not(m_objectivec1 in current_settings.modeswitches) then
                  Message(parser_f_need_objc);

                consume(token);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                def:=object_dec(odt_objccategory,name,newsym,genericdef,genericlist,nil,ht_none);
=======
                def:=object_dec(odt_objccategory,name,genericdef,genericlist,nil);
>>>>>>> graemeg/cpstrnew
=======
                def:=object_dec(odt_objccategory,name,genericdef,genericlist,nil);
>>>>>>> graemeg/cpstrnew
=======
                def:=object_dec(odt_objccategory,name,genericdef,genericlist,nil);
>>>>>>> graemeg/cpstrnew
=======
                def:=object_dec(odt_objccategory,name,genericdef,genericlist,nil);
>>>>>>> origin/cpstrnew
               end;
            _OBJECT :
              begin
                consume(token);
                def:=object_dec(odt_object,name,newsym,genericdef,genericlist,nil,ht_none);
              end;
            _PROCEDURE,
            _FUNCTION:
              begin
                def:=procvar_dec(genericdef,genericlist);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef jvm}
                jvm_create_procvar_class(name,def);
{$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              end;
            _ID:
              begin
                case idtoken of
                  _HELPER:
                    begin
                      if hadtypetoken and
                         { don't allow "type helper" in mode delphi and require modeswitch typehelpers }
                         ([m_delphi,m_type_helpers]*current_settings.modeswitches=[m_type_helpers]) then
                        begin
                          { reset hadtypetoken, so that calling code knows that it should not be handled
                            as a "unique" type }
                          hadtypetoken:=false;
                          consume(_HELPER);
                          def:=object_dec(odt_helper,name,newsym,genericdef,genericlist,nil,ht_type);
                        end
                      else
                        expr_type
                    end;
                  _REFERENCE:
                    begin
                      if m_blocks in current_settings.modeswitches then
                        begin
                          consume(_REFERENCE);
                          consume(_TO);
                          def:=procvar_dec(genericdef,genericlist);
                          { could be errordef in case of a syntax error }
                          if assigned(def) and
                             (def.typ=procvardef) then
                            include(tprocvardef(def).procoptions,po_is_function_ref);
                        end
                      else
                        expr_type;
                    end;
                  else
                    expr_type;
                end;
              end
            else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              if (token=_KLAMMERAFFE) and (([m_iso,m_extpas]*current_settings.modeswitches)<>[]) then
                begin
                  consume(_KLAMMERAFFE);
                  single_type(tt2,SingleTypeOptionsInTypeBlock[block_type=bt_type]);
                  def:=cpointerdef.create(tt2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              if (token=_KLAMMERAFFE) and (m_iso in current_settings.modeswitches) then
                begin
                  consume(_KLAMMERAFFE);
                  single_type(tt2,SingleTypeOptionsInTypeBlock[block_type=bt_type]);
                  def:=tpointerdef.create(tt2);
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
                  if tt2.typ=forwarddef then
                    current_module.checkforwarddefs.add(def);
                end
              else
                expr_type;
         end;

         if def=nil then
          def:=generrordef;
      end;


    procedure read_anon_type(var def : tdef;parseprocvardir:boolean);
      var
        hadtypetoken : boolean;
      begin
        hadtypetoken:=false;
        read_named_type(def,nil,nil,nil,parseprocvardir,hadtypetoken);
      end;




    procedure add_typedconst_init_routine(def: tabstractrecorddef);
      var
        sstate: tscannerstate;
        pd: tprocdef;
      begin
        replace_scanner('tcinit_routine',sstate);
        { the typed constant initialization code is called from the class
          constructor by tnodeutils.wrap_proc_body; at this point, we don't
          know yet whether that will be necessary, because there may be
          typed constants inside method bodies -> always force the addition
          of a class constructor.

          We cannot directly add the typed constant initialisations to the
          class constructor, because when it's parsed not all method bodies
          are necessarily already parsed }
        pd:=def.find_procdef_bytype(potype_class_constructor);
        { the class constructor }
        if not assigned(pd) then
          begin
<<<<<<< HEAD
            if str_parse_method_dec('constructor fpc_init_typed_consts_class_constructor;',potype_class_constructor,true,def,pd) then
              pd.synthetickind:=tsk_empty
            else
              internalerror(2011040206);
=======
            def:=tdef(st.DefList[i]);
            case def.typ of
              recorddef :
                write_persistent_type_info(trecorddef(def).symtable);
              objectdef :
                begin
                  { Skip generics and forward defs }
                  if (df_generic in def.defoptions) or
                     (oo_is_forward in tobjectdef(def).objectoptions) then
                    continue;
                  write_persistent_type_info(tobjectdef(def).symtable);
                  { Write also VMT if not done yet }
                  if not(ds_vmt_written in def.defstates) then
                    begin
                      vmtwriter:=TVMTWriter.create(tobjectdef(def));
                      if is_interface(tobjectdef(def)) then
                        vmtwriter.writeinterfaceids;
                      if (oo_has_vmt in tobjectdef(def).objectoptions) then
                        vmtwriter.writevmt;
                      vmtwriter.free;
                      include(def.defstates,ds_vmt_written);
                    end;
                end;
              procdef :
                begin
                  if assigned(tprocdef(def).localst) and
                     (tprocdef(def).localst.symtabletype=localsymtable) then
                    write_persistent_type_info(tprocdef(def).localst);
                  if assigned(tprocdef(def).parast) then
                    write_persistent_type_info(tprocdef(def).parast);
                end;
            end;
            { generate always persistent tables for types in the interface so it can
              be reused in other units and give always the same pointer location. }
            { Init }
            if (
                assigned(def.typesym) and
                (st.symtabletype=globalsymtable) and
                not is_objc_class_or_protocol(def)
               ) or
               is_managed_type(def) or
               (ds_init_table_used in def.defstates) then
              RTTIWriter.write_rtti(def,initrtti);
            { RTTI }
            if (
                assigned(def.typesym) and
                (st.symtabletype=globalsymtable) and
                not is_objc_class_or_protocol(def)
               ) or
               (ds_rtti_table_used in def.defstates) then
              RTTIWriter.write_rtti(def,fullrtti);
>>>>>>> graemeg/cpstrnew
          end;
        { the initialisation helper }
        if str_parse_method_dec('procedure fpc_init_typed_consts_helper; static;',potype_procedure,true,def,pd) then
          pd.synthetickind:=tsk_tcinit
        else
          internalerror(2011040207);
        restore_scanner(sstate);
      end;


end.
