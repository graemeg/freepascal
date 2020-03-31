{
    Copyright (c) 2011

    Contains different functions that are used in the context of
    parsing generics.

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
unit pgenutil;

{$i fpcdefs.inc}

interface

uses
  { common }
  cclasses,
  { global }
  globtype,
  { parser }
  pgentype,
  { symtable }
  symtype,symdef,symbase;

    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean;_prettyname:string;parsedtype:tdef;symname:string;parsedpos:tfileposinfo);inline;
    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean;_prettyname:string);inline;
    function generate_specialization_phase1(out context:tspecializationcontext;genericdef:tdef):tdef;inline;
    function generate_specialization_phase1(out context:tspecializationcontext;genericdef:tdef;symname:string):tdef;inline;
    function generate_specialization_phase1(out context:tspecializationcontext;genericdef:tdef;parsedtype:tdef;symname:string;parsedpos:tfileposinfo):tdef;
    function generate_specialization_phase2(context:tspecializationcontext;genericdef:tstoreddef;parse_class_parent:boolean;_prettyname:ansistring):tdef;
    function check_generic_constraints(genericdef:tstoreddef;paradeflist:tfpobjectlist;poslist:tfplist):boolean;
    function parse_generic_parameters(allowconstraints:boolean):tfphashobjectlist;
    function parse_generic_specialization_types(genericdeflist:tfpobjectlist;poslist:tfplist;out prettyname,specializename:ansistring):boolean;
    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:tfphashobjectlist);
    procedure maybe_insert_generic_rename_symbol(const name:tidstring;genericlist:tfphashobjectlist);
    function generate_generic_name(const name:tidstring;specializename:ansistring;owner_hierarchy:string):tidstring;
    procedure split_generic_name(const name:tidstring;out nongeneric:string;out count:longint);
    procedure add_generic_dummysym(sym:tsym);
    function resolve_generic_dummysym(const name:tidstring):tsym;
    function could_be_generic(const name:tidstring):boolean;inline;

    procedure generate_specialization_procs;
    procedure maybe_add_pending_specialization(def:tdef);

    procedure specialization_init(genericdef:tdef;var state:tspecializationstate);
    procedure specialization_done(var state:tspecializationstate);

implementation

uses
  { common }
  cutils,fpccrc,
  { global }
  globals,tokens,verbose,finput,
  { symtable }
  symconst,symsym,symtable,defcmp,procinfo,
  { modules }
  fmodule,
  node,nobj,
  { parser }
  scanner,
  pbase,pexpr,pdecsub,ptype,psub,pparautl;


    procedure maybe_add_waiting_unit(tt:tdef);
      var
        hmodule : tmodule;
      begin
        if not assigned(tt) or
            not (df_generic in tt.defoptions) then
          exit;

        hmodule:=find_module_from_symtable(tt.owner);
        if not assigned(hmodule) then
          internalerror(2012092401);

        if hmodule=current_module then
          exit;

        if hmodule.state<>ms_compiled then
          begin
{$ifdef DEBUG_UNITWAITING}
            Writeln('Unit ', current_module.modulename^,
              ' waiting for ', hmodule.modulename^);
{$endif DEBUG_UNITWAITING}
            if current_module.waitingforunit.indexof(hmodule)<0 then
              current_module.waitingforunit.add(hmodule);
            if hmodule.waitingunits.indexof(current_module)<0 then
              hmodule.waitingunits.add(current_module);
          end;
      end;

    function check_generic_constraints(genericdef:tstoreddef;paradeflist:tfpobjectlist;poslist:tfplist):boolean;
      var
        i,j,
        intfcount : longint;
        formaldef,
        paradef : tstoreddef;
        objdef,
        paraobjdef,
        formalobjdef : tobjectdef;
        intffound : boolean;
        filepos : tfileposinfo;
      begin
        { check whether the given specialization parameters fit to the eventual
          constraints of the generic }
        if not assigned(genericdef.genericparas) or (genericdef.genericparas.count=0) then
          internalerror(2012101001);
        if genericdef.genericparas.count<>paradeflist.count then
          internalerror(2012101002);
        if paradeflist.count<>poslist.count then
          internalerror(2012120801);
        result:=true;
        for i:=0 to genericdef.genericparas.count-1 do
          begin
            filepos:=pfileposinfo(poslist[i])^;
            formaldef:=tstoreddef(ttypesym(genericdef.genericparas[i]).typedef);
            if formaldef.typ=undefineddef then
              { the parameter is of unspecified type, so no need to check }
              continue;
            if not (df_genconstraint in formaldef.defoptions) or
                not assigned(formaldef.genconstraintdata) then
              internalerror(2013021602);
            paradef:=tstoreddef(paradeflist[i]);
            { undefineddef is compatible with anything }
            if formaldef.typ=undefineddef then
              continue;
            if paradef.typ<>formaldef.typ then
              begin
                case formaldef.typ of
                  recorddef:
                    { delphi has own fantasy about record constraint
                      (almost non-nullable/non-nilable value type) }
                    if m_delphi in current_settings.modeswitches then
                      case paradef.typ of
                        floatdef,enumdef,orddef:
                          continue;
                        objectdef:
                          if tobjectdef(paradef).objecttype=odt_object then
                            continue
                          else
                            MessagePos(filepos,type_e_record_type_expected);
                        else
                          MessagePos(filepos,type_e_record_type_expected);
                      end
                    else
                      MessagePos(filepos,type_e_record_type_expected);
                  objectdef:
                    case tobjectdef(formaldef).objecttype of
                      odt_class,
                      odt_javaclass:
                        MessagePos1(filepos,type_e_class_type_expected,paradef.typename);
                      odt_interfacecom,
                      odt_interfacecorba,
                      odt_dispinterface,
                      odt_interfacejava:
                        MessagePos1(filepos,type_e_interface_type_expected,paradef.typename);
                      else
                        internalerror(2012101003);
                    end;
                  errordef:
                    { ignore }
                    ;
                  else
                    internalerror(2012101004);
                end;
                result:=false;
              end
            else
              begin
                { the paradef types are the same, so do special checks for the
                  cases in which they are needed }
                if formaldef.typ=objectdef then
                  begin
                    paraobjdef:=tobjectdef(paradef);
                    formalobjdef:=tobjectdef(formaldef);
                    if not (formalobjdef.objecttype in [odt_class,odt_javaclass,odt_interfacecom,odt_interfacecorba,odt_interfacejava,odt_dispinterface]) then
                      internalerror(2012101102);
                    if formalobjdef.objecttype in [odt_interfacecom,odt_interfacecorba,odt_interfacejava,odt_dispinterface] then
                      begin
                        { this is either a concerete interface or class type (the
                          latter without specific implemented interfaces) }
                        case paraobjdef.objecttype of
                          odt_interfacecom,
                          odt_interfacecorba,
                          odt_interfacejava,
                          odt_dispinterface:
                            begin
                              if (oo_is_forward in paraobjdef.objectoptions) and
                                  (paraobjdef.objecttype=formalobjdef.objecttype) and
                                  (df_genconstraint in formalobjdef.defoptions) and
                                  (
                                    (formalobjdef.objecttype=odt_interfacecom) and
                                    (formalobjdef.childof=interface_iunknown)
                                  )
                                  or
                                  (
                                    (formalobjdef.objecttype=odt_interfacecorba) and
                                    (formalobjdef.childof=nil)
                                  ) then
                                continue;
                              if not def_is_related(paraobjdef,formalobjdef.childof) then
                                begin
                                  MessagePos2(filepos,type_e_incompatible_types,paraobjdef.typename,formalobjdef.childof.typename);
                                  result:=false;
                                end;
                            end;
                          odt_class,
                          odt_javaclass:
                            begin
                              objdef:=paraobjdef;
                              intffound:=false;
                              while assigned(objdef) do
                                begin
                                  for j:=0 to objdef.implementedinterfaces.count-1 do
                                    if timplementedinterface(objdef.implementedinterfaces[j]).intfdef=formalobjdef.childof then
                                      begin
                                        intffound:=true;
                                        break;
                                      end;
                                  if intffound then
                                    break;
                                  objdef:=objdef.childof;
                                end;
                              result:=intffound;
                              if not result then
                                MessagePos2(filepos,parser_e_class_doesnt_implement_interface,paraobjdef.typename,formalobjdef.childof.typename);
                            end;
                          else
                            begin
                              MessagePos1(filepos,type_e_class_or_interface_type_expected,paraobjdef.typename);
                              result:=false;
                            end;
                        end;
                      end
                    else
                      begin
                        { this is either a "class" or a concrete instance with
                          or without implemented interfaces }
                        if not (paraobjdef.objecttype in [odt_class,odt_javaclass]) then
                          begin
                            MessagePos1(filepos,type_e_class_type_expected,paraobjdef.typename);
                            result:=false;
                            continue;
                          end;
                        { for forward declared classes we allow pure TObject/class declarations }
                        if (oo_is_forward in paraobjdef.objectoptions) and
                            (df_genconstraint in formaldef.defoptions) then
                          begin
                            if (formalobjdef.childof=class_tobject) and
                                not formalobjdef.implements_any_interfaces then
                              continue;
                          end;
                        if assigned(formalobjdef.childof) and
                            not def_is_related(paradef,formalobjdef.childof) then
                          begin
                            MessagePos2(filepos,type_e_incompatible_types,paraobjdef.typename,formalobjdef.childof.typename);
                            result:=false;
                          end;
                        intfcount:=0;
                        for j:=0 to formalobjdef.implementedinterfaces.count-1 do
                          begin
                            objdef:=paraobjdef;
                            while assigned(objdef) do
                              begin
                                intffound:=assigned(
                                             find_implemented_interface(objdef,
                                               timplementedinterface(formalobjdef.implementedinterfaces[j]).intfdef
                                             )
                                           );
                                if intffound then
                                  break;
                                objdef:=objdef.childof;
                              end;
                            if intffound then
                              inc(intfcount)
                            else
                              MessagePos2(filepos,parser_e_class_doesnt_implement_interface,paraobjdef.typename,timplementedinterface(formalobjdef.implementedinterfaces[j]).intfdef.typename);
                          end;
                        if intfcount<>formalobjdef.implementedinterfaces.count then
                          result:=false;
                      end;
                  end;
              end;
          end;
      end;


    function parse_generic_specialization_types_internal(genericdeflist:tfpobjectlist;poslist:tfplist;out prettyname,specializename:ansistring;parsedtype:tdef;parsedpos:tfileposinfo):boolean;
      var
        old_block_type : tblock_type;
        first : boolean;
        typeparam : tnode;
        parampos : pfileposinfo;
        tmpparampos : tfileposinfo;
        namepart : string;
        prettynamepart : ansistring;
        module : tmodule;
      begin
        result:=true;
        if genericdeflist=nil then
          internalerror(2012061401);
        { set the block type to type, so that the parsed type are returned as
          ttypenode (e.g. classes are in non type-compatible blocks returned as
          tloadvmtaddrnode) }
        old_block_type:=block_type;
        { if parsedtype is set, then the first type identifer was already parsed
          (happens in inline specializations) and thus we only need to parse
          the remaining types and do as if the first one was already given }
        first:=not assigned(parsedtype);
        if assigned(parsedtype) then
          begin
            genericdeflist.Add(parsedtype);
            module:=find_module_from_symtable(parsedtype.owner);
            if not assigned(module) then
              internalerror(2016112801);
            namepart:='_$'+hexstr(module.moduleid,8)+'$$'+parsedtype.unique_id_str;
            specializename:='$'+namepart;
            prettyname:=parsedtype.fullownerhierarchyname(true)+parsedtype.typesym.prettyname;
            if assigned(poslist) then
              begin
                New(parampos);
                parampos^:=parsedpos;
                poslist.add(parampos);
              end;
          end
        else
          begin
            specializename:='$';
            prettyname:='';
          end;
        while not (token in [_GT,_RSHARPBRACKET]) do
          begin
            { "first" is set to false at the end of the loop! }
            if not first then
              consume(_COMMA);
            block_type:=bt_type;
            tmpparampos:=current_filepos;
            typeparam:=factor(false,[ef_type_only]);
            if typeparam.nodetype=typen then
              begin
                if tstoreddef(typeparam.resultdef).is_generic and
                    (
                      not parse_generic or
                      not defs_belong_to_same_generic(typeparam.resultdef,current_genericdef)
                    ) then
                  Message(parser_e_no_generics_as_params);
                if assigned(poslist) then
                  begin
                    New(parampos);
                    parampos^:=tmpparampos;
                    poslist.add(parampos);
                  end;
                if typeparam.resultdef.typ<>errordef then
                  begin
                    if not assigned(typeparam.resultdef.typesym) then
                      message(type_e_generics_cannot_reference_itself)
                    else if (typeparam.resultdef.typ<>errordef) then
                      begin
                        genericdeflist.Add(typeparam.resultdef);
                        module:=find_module_from_symtable(typeparam.resultdef.owner);
                        if not assigned(module) then
                          internalerror(2016112802);
                        namepart:='_$'+hexstr(module.moduleid,8)+'$$'+typeparam.resultdef.unique_id_str;
                        { we use the full name of the type to uniquely identify it }
                        if (symtablestack.top.symtabletype=parasymtable) and
                            (symtablestack.top.defowner.typ=procdef) and
                            (typeparam.resultdef.owner=symtablestack.top) then
                          begin
                            { special handling for specializations inside generic function declarations }
                            prettynamepart:=tdef(symtablestack.top.defowner).fullownerhierarchyname(true)+tprocdef(symtablestack.top.defowner).procsym.prettyname;
                          end
                        else
                          begin
                            prettynamepart:=typeparam.resultdef.fullownerhierarchyname(true);
                          end;
                        specializename:=specializename+namepart;
                        if not first then
                          prettyname:=prettyname+',';
                        prettyname:=prettyname+prettynamepart+typeparam.resultdef.typesym.prettyname;
                      end;
                  end
                else
                  begin
                    result:=false;
                  end;
              end
            else
              begin
                Message(type_e_type_id_expected);
                result:=false;
              end;
            typeparam.free;
            first:=false;
          end;
        block_type:=old_block_type;
      end;


    function parse_generic_specialization_types(genericdeflist:tfpobjectlist;poslist:tfplist;out prettyname,specializename:ansistring):boolean;
      var
        dummypos : tfileposinfo;
      begin
        FillChar(dummypos, SizeOf(tfileposinfo), 0);
        result:=parse_generic_specialization_types_internal(genericdeflist,poslist,prettyname,specializename,nil,dummypos);
      end;


    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean;_prettyname:string);
      var
        dummypos : tfileposinfo;
      begin
        FillChar(dummypos, SizeOf(tfileposinfo), 0);
        generate_specialization(tt,parse_class_parent,_prettyname,nil,'',dummypos);
      end;


    function generate_specialization_phase1(out context:tspecializationcontext;genericdef:tdef):tdef;
      var
        dummypos : tfileposinfo;
{$push}
{$warn 5036 off}
      begin
        result:=generate_specialization_phase1(context,genericdef,nil,'',dummypos);
      end;
{$pop}


    function generate_specialization_phase1(out context:tspecializationcontext;genericdef:tdef;symname:string):tdef;
      var
        dummypos : tfileposinfo;
{$push}
{$warn 5036 off}
      begin
        result:=generate_specialization_phase1(context,genericdef,nil,symname,dummypos);
      end;
{$pop}


    function generate_specialization_phase1(out context:tspecializationcontext;genericdef:tdef;parsedtype:tdef;symname:string;parsedpos:tfileposinfo):tdef;
      var
        found,
        err : boolean;
        i,
        gencount : longint;
        countstr,genname,ugenname : string;
        tmpstack : tfpobjectlist;
      begin
        context:=nil;
        result:=nil;

        { either symname must be given or genericdef needs to be valid }
        if (symname='') and
            (not assigned(genericdef) or
              (
                (genericdef.typ<>procdef) and
                (
                  not assigned(genericdef.typesym) or
                  (genericdef.typesym.typ<>typesym)
                )
              ) or
              (
                (genericdef.typ=procdef) and
                (
                  not assigned(tprocdef(genericdef).procsym) or
                  (tprocdef(genericdef).procsym.typ<>procsym)
                )
              )
            ) then
          begin
            internalerror(2019112401);
          end;

        if not assigned(parsedtype) and not try_to_consume(_LT) then
          begin
            consume(_LSHARPBRACKET);
            { handle "<>" }
            if (token=_GT) or (token=_RSHARPBRACKET) then
              begin
                Message(type_e_type_id_expected);
                if not try_to_consume(_GT) then
                  try_to_consume(_RSHARPBRACKET);
                result:=generrordef;
                exit;
              end;
          end;

        context:=tspecializationcontext.create;

        { Parse type parameters }
        err:=not parse_generic_specialization_types_internal(context.genericdeflist,context.poslist,context.prettyname,context.specializename,parsedtype,parsedpos);
        if err then
          begin
            if not try_to_consume(_GT) then
              try_to_consume(_RSHARPBRACKET);
            context.free;
            context:=nil;
            result:=generrordef;
            exit;
          end;

        { use the name of the symbol as procvars return a user friendly version
          of the name }
        if symname='' then
          begin
            if genericdef.typ=procdef then
              genname:=tprocdef(genericdef).procsym.realname
            else
              genname:=ttypesym(genericdef.typesym).realname;
          end
        else
          genname:=symname;

        { in case of non-Delphi mode the type name could already be a generic
          def (but maybe the wrong one) }
        if assigned(genericdef) and
            ([df_generic,df_specialization]*genericdef.defoptions<>[]) then
          begin
            { remove the type count suffix from the generic's name }
            for i:=Length(genname) downto 1 do
              if genname[i]='$' then
                begin
                  genname:=copy(genname,1,i-1);
                  break;
                end;
            { in case of a specialization we've only reached the specialization
              checksum yet }
            if df_specialization in genericdef.defoptions then
              for i:=length(genname) downto 1 do
                if genname[i]='$' then
                  begin
                    genname:=copy(genname,1,i-1);
                    break;
                  end;
          end
        else
          begin
            split_generic_name(genname,ugenname,gencount);
            if genname<>ugenname then
              genname:=ugenname;
          end;

        { search a generic with the given count of params }
        countstr:='';
        str(context.genericdeflist.Count,countstr);

        genname:=genname+'$'+countstr;
        ugenname:=upper(genname);

        context.genname:=genname;

        if assigned(genericdef) and (genericdef.owner.symtabletype in [objectsymtable,recordsymtable]) then
          begin
            if genericdef.owner.symtabletype = objectsymtable then
              found:=searchsym_in_class(tobjectdef(genericdef.owner.defowner),tobjectdef(genericdef.owner.defowner),ugenname,context.sym,context.symtable,[])
            else
              found:=searchsym_in_record(tabstractrecorddef(genericdef.owner.defowner),ugenname,context.sym,context.symtable);
            if not found then
              found:=searchsym(ugenname,context.sym,context.symtable);
          end
        else
          found:=searchsym(ugenname,context.sym,context.symtable);

        if found and (context.sym.typ=absolutevarsym) and
            (vo_is_funcret in tabstractvarsym(context.sym).varoptions) then
          begin
            { we found the function result alias of a generic function; go up the
              symbol stack *before* this alias was inserted, so that we can
              (hopefully) find the correct generic symbol }
            tmpstack:=tfpobjectlist.create(false);
            while assigned(symtablestack.top) do
              begin
                tmpstack.Add(symtablestack.top);
                symtablestack.pop(symtablestack.top);
                if tmpstack.Last=context.symtable then
                  break;
              end;
            if not assigned(symtablestack.top) then
              internalerror(2019123001);
            found:=searchsym(ugenname,context.sym,context.symtable);
            for i:=tmpstack.count-1 downto 0 do
              symtablestack.push(tsymtable(tmpstack[i]));
            tmpstack.free;
          end;

        if not found or not (context.sym.typ in [typesym,procsym]) then
          begin
            identifier_not_found(genname);
            if not try_to_consume(_GT) then
              try_to_consume(_RSHARPBRACKET);
            context.free;
            context:=nil;
            result:=generrordef;
            exit;
          end;

        { we've found the correct def }
        if context.sym.typ=typesym then
          result:=tstoreddef(ttypesym(context.sym).typedef)
        else
          begin
            if tprocsym(context.sym).procdeflist.count=0 then
              internalerror(2015061203);
            result:=tstoreddef(tprocsym(context.sym).procdefList[0]);
          end;

        if not try_to_consume(_GT) then
          consume(_RSHARPBRACKET);
      end;

    function generate_specialization_phase2(context:tspecializationcontext;genericdef:tstoreddef;parse_class_parent:boolean;_prettyname:ansistring):tdef;

        procedure unset_forwarddef(def: tdef);
          var
            st : TSymtable;
            i : longint;
          begin
            case def.typ of
              procdef:
                tprocdef(def).forwarddef:=false;
              objectdef,
              recorddef:
                begin
                  st:=def.getsymtable(gs_record);
                  for i:=0 to st.deflist.count-1 do
                    unset_forwarddef(tdef(st.deflist[i]));
                end;
              else
                ;
            end;
          end;

        procedure retrieve_genericdef_or_procsym(sym:tsym;out gendef:tdef;out psym:tsym);
          var
            i : longint;
          begin
            gendef:=nil;
            psym:=nil;
            case sym.typ of
              typesym:
                begin
                  gendef:=ttypesym(sym).typedef
                end;
              procsym:
                begin
                  for i:=0 to tprocsym(sym).procdeflist.count-1 do
                    if tstoreddef(tprocsym(sym).procdeflist[i]).genericdef=genericdef then
                      begin
                        gendef:=tdef(tprocsym(sym).procdeflist[i]);
                        break;
                      end;
                  psym:=sym;
                end
              else
                internalerror(200710171);
            end;
          end;

      var
        finalspecializename,
        ufinalspecializename : tidstring;
        prettyname : ansistring;
        generictypelist : tfphashobjectlist;
        srsymtable,
        specializest : tsymtable;
        hashedid : thashedidstring;
        tempst : tglobalsymtable;
        psym,
        srsym : tsym;
        def : tdef;
        old_block_type : tblock_type;
        state : tspecializationstate;
        old_current_structdef : tabstractrecorddef;
        old_current_specializedef,
        old_current_genericdef : tstoreddef;
        old_current_procinfo : tprocinfo;
        old_module_procinfo : tobject;
        hmodule : tmodule;
        oldcurrent_filepos : tfileposinfo;
        recordbuf : tdynamicarray;
        hadtypetoken : boolean;
        i,
        replaydepth : longint;
        item : tobject;
        allequal,
        hintsprocessed : boolean;
        pd : tprocdef;
        pdflags : tpdflags;
      begin
        if not assigned(context) then
          internalerror(2015052203);

        result:=nil;

        pd:=nil;

        if not check_generic_constraints(genericdef,context.genericdeflist,context.poslist) then
          begin
            { the parameters didn't fit the constraints, so don't continue with the
              specialization }
            result:=generrordef;
            exit;
          end;

        { build the new type's name }
        finalspecializename:=generate_generic_name(context.genname,context.specializename,genericdef.ownerhierarchyname);
        ufinalspecializename:=upper(finalspecializename);
        if genericdef.typ=procdef then
          prettyname:=tprocdef(genericdef).procsym.prettyname
        else
          prettyname:=genericdef.typesym.prettyname;
        prettyname:=prettyname+'<'+context.prettyname+'>';

        generictypelist:=tfphashobjectlist.create(false);

        { build the list containing the types for the generic params }
        if not assigned(genericdef.genericparas) then
          internalerror(2013092601);
        if context.genericdeflist.count<>genericdef.genericparas.count then
          internalerror(2013092603);
        for i:=0 to genericdef.genericparas.Count-1 do
          begin
            srsym:=tsym(genericdef.genericparas[i]);
            if not (sp_generic_para in srsym.symoptions) then
              internalerror(2013092602);
            generictypelist.add(srsym.realname,tdef(context.genericdeflist[i]).typesym);
          end;

        { Special case if we are referencing the current defined object }
        if assigned(current_structdef) and
           (current_structdef.objname^=ufinalspecializename) then
          result:=current_structdef;

        { Can we reuse an already specialized type? }

        { for this first check whether we are currently specializing a nested
          type of the current (main) specialization (this is necessary, because
          during that time the symbol of the main specialization will still
          contain a reference to an errordef) }
        if not assigned(result) and assigned(current_specializedef) then
          begin
            def:=current_specializedef;
            repeat
              if def.typ in [objectdef,recorddef] then
                if tabstractrecorddef(def).objname^=ufinalspecializename then begin
                  result:=def;
                  break;
                end;
              if assigned(def.owner) then
                def:=tstoreddef(def.owner.defowner)
              else
                { this can happen when specializing a generic function }
                def:=nil;
            until not assigned(def) or not (df_specialization in def.defoptions);
          end;

        { if the genericdef is the def we are currently parsing (or one of its parents) then we can
          not use it for specializing as the tokenbuffer is not yet set (and we aren't done with
          parsing anyway), so for now we treat those still as generic defs without doing a partial
          specialization }
        if not assigned(result) then
          begin
            def:=current_genericdef;
            while assigned(def) and (def.typ in [recorddef,objectdef]) do
              begin
                if (df_generic in def.defoptions) and (def=genericdef) then
                  begin
                    result:=def;
                    break;
                  end;
                { the following happens when a routine with its parent struct
                  as parameter is specialized as a parameter or result of a
                  generic function }
                if (df_specialization in def.defoptions) and (tstoreddef(def).genericdef=genericdef) then
                  begin
                    if tstoreddef(def).genericparas.count=generictypelist.count then
                      begin
                        allequal:=true;
                        for i:=0 to generictypelist.count-1 do
                          begin
                            if not equal_defs(ttypesym(generictypelist[i]).typedef,ttypesym(tstoreddef(def).genericparas[i]).typedef) then
                              begin
                                allequal:=false;
                                break;
                              end;
                          end;
                        if allequal then
                          begin
                            result:=def;
                            break;
                          end;
                      end;
                  end;
                def:=tstoreddef(def.owner.defowner);
              end;
          end;

        { decide in which symtable to put the specialization }
        if parse_generic and not assigned(result) then
          begin
            srsymtable:=symtablestack.top;
            if (srsymtable.symtabletype in [localsymtable,parasymtable]) and tstoreddef(srsymtable.defowner).is_specialization then
              { if we are currently specializing a routine we need to specialize into
                the routine's local- or parasymtable so that they are correctly
                registered should the specialization be finalized }
              specializest:=srsymtable
            else if assigned(current_procinfo) and (df_generic in current_procinfo.procdef.defoptions) then
              { if we are parsing the definition of a method we specialize into
                the local symtable of it }
              specializest:=current_procinfo.procdef.getsymtable(gs_local)
            else
              begin
                if not assigned(current_genericdef) then
                  internalerror(2014050901);
                { we specialize the partial specialization into the symtable of the currently parsed
                  generic }
                case current_genericdef.typ of
                  procvardef:
                    specializest:=current_genericdef.getsymtable(gs_para);
                  procdef:
                    specializest:=current_genericdef.getsymtable(gs_local);
                  objectdef,
                  recorddef:
                    specializest:=current_genericdef.getsymtable(gs_record);
                  arraydef:
                    specializest:=tarraydef(current_genericdef).symtable;
                  else
                    internalerror(2014050902);
                end;
              end;
          end
        else
          if current_module.is_unit and current_module.in_interface then
            specializest:=current_module.globalsymtable
          else
            specializest:=current_module.localsymtable;
        if not assigned(specializest) then
          internalerror(2014050910);

        { now check whether there is a specialization somewhere else }
        psym:=nil;
        if not assigned(result) then
          begin
            hashedid.id:=ufinalspecializename;

            if specializest.symtabletype=objectsymtable then
              begin
                { search also in parent classes }
                if not assigned(current_genericdef) or (current_genericdef.typ<>objectdef) then
                  internalerror(2016112901);
                if not searchsym_in_class(tobjectdef(current_genericdef),tobjectdef(current_genericdef),ufinalspecializename,srsym,srsymtable,[]) then
                  srsym:=nil;
              end
            else
              srsym:=tsym(specializest.findwithhash(hashedid));

            if assigned(srsym) then
              begin
                retrieve_genericdef_or_procsym(srsym,result,psym);
              end
            else
              { the generic could have been specialized in the globalsymtable
                already, so search there as well }
              if (specializest<>current_module.globalsymtable) and assigned(current_module.globalsymtable) then
                begin
                  srsym:=tsym(current_module.globalsymtable.findwithhash(hashedid));
                  if assigned(srsym) then
                    begin
                      retrieve_genericdef_or_procsym(srsym,result,psym);
                    end;
                end;
          end;

        if not assigned(result) then
          begin
            specialization_init(genericdef,state);

            { push a temporary global symtable so that the specialization is
              added to the correct symtable; this symtable does not contain
              any other symbols, so that the type resolution can not be
              influenced by symbols in the current unit }
            tempst:=tspecializesymtable.create(current_module.modulename^,current_module.moduleid);
            symtablestack.push(tempst);

            { Reparse the original type definition }
              begin
                old_current_specializedef:=nil;
                old_current_genericdef:=nil;
                old_current_structdef:=nil;
                old_current_procinfo:=current_procinfo;
                old_module_procinfo:=current_module.procinfo;

                current_procinfo:=nil;
                current_module.procinfo:=nil;

                if parse_class_parent then
                  begin
                    old_current_structdef:=current_structdef;
                    old_current_genericdef:=current_genericdef;
                    old_current_specializedef:=current_specializedef;

                    if genericdef.owner.symtabletype in [recordsymtable,objectsymtable] then
                      current_structdef:=tabstractrecorddef(genericdef.owner.defowner)
                    else
                      current_structdef:=nil;
                    current_genericdef:=nil;
                    current_specializedef:=nil;
                  end;

                maybe_add_waiting_unit(genericdef);

                { First a new sym so we can reuse this specialization and
                  references to this specialization can be handled }
                if genericdef.typ=procdef then
                  if assigned(psym) then
                    srsym:=psym
                  else
                    srsym:=cprocsym.create(finalspecializename)
                else
                  srsym:=ctypesym.create(finalspecializename,generrordef);
                { insert the symbol only if we don't know already that we have
                  a procsym to add it to }
                if not assigned(psym) then
                  specializest.insert(srsym);

                { specializations are declarations as such it is the wisest to
                  declare set the blocktype to "type"; otherwise we'll
                  experience unexpected side effects like the addition of
                  classrefdefs if we have a generic that's derived from another
                  generic }
                old_block_type:=block_type;
                block_type:=bt_type;

                if (
                     (genericdef.typ=procdef) and
                     not assigned(tprocdef(genericdef).genericdecltokenbuf)
                   ) or (
                     (genericdef.typ<>procdef) and
                     not assigned(genericdef.generictokenbuf)
                   ) then
                  internalerror(200511171);
                hmodule:=find_module_from_symtable(genericdef.owner);
                if hmodule=nil then
                  internalerror(2012051202);
                oldcurrent_filepos:=current_filepos;
                { use the index the module got from the current compilation process }
                current_filepos.moduleindex:=hmodule.unit_index;
                current_tokenpos:=current_filepos;
                if parse_generic then
                  begin
                    recordbuf:=current_scanner.recordtokenbuf;
                    current_scanner.recordtokenbuf:=nil;
                  end
                else
                  recordbuf:=nil;
                replaydepth:=current_scanner.replay_stack_depth;
                if genericdef.typ=procdef then
                  begin
                    current_scanner.startreplaytokens(tprocdef(genericdef).genericdecltokenbuf,hmodule.change_endian);
                    parse_proc_head(tprocdef(genericdef).struct,tprocdef(genericdef).proctypeoption,false,genericdef,generictypelist,pd);
                    if assigned(pd) then
                      begin
                        if assigned(psym) then
                          pd.procsym:=psym
                        else
                          pd.procsym:=srsym;
                        parse_proc_dec_finish(pd,po_classmethod in tprocdef(genericdef).procoptions,tprocdef(genericdef).struct);
                      end;
                    result:=pd;
                  end
                else
                  begin
                    current_scanner.startreplaytokens(genericdef.generictokenbuf,hmodule.change_endian);
                    hadtypetoken:=false;
                    read_named_type(result,srsym,genericdef,generictypelist,false,hadtypetoken);
                    ttypesym(srsym).typedef:=result;
                    result.typesym:=srsym;

                    if _prettyname<>'' then
                      ttypesym(result.typesym).fprettyname:=_prettyname
                    else
                      ttypesym(result.typesym).fprettyname:=prettyname;
                  end;
                current_filepos:=oldcurrent_filepos;

                { Note regarding hint directives:
                  There is no need to remove the flags for them from the
                  specialized generic symbol, because hint directives that
                  follow the specialization are handled by the code in
                  pdecl.types_dec and added to the type symbol.
                  E.g.: TFoo = TBar<Blubb> deprecated;
                  Here the symbol TBar$1$Blubb will contain the
                  "sp_hint_deprecated" flag while the TFoo symbol won't.}

                case result.typ of
                  { Build VMT indexes for classes and read hint directives }
                  objectdef:
                    begin
                      if replaydepth>current_scanner.replay_stack_depth then
                        begin
                          try_consume_hintdirective(srsym.symoptions,srsym.deprecatedmsg);
                          if replaydepth>current_scanner.replay_stack_depth then
                            consume(_SEMICOLON);
                        end;

                      build_vmt(tobjectdef(result));
                    end;
                  { handle params, calling convention, etc }
                  procvardef:
                    begin
                      hintsprocessed:=false;
                      if replaydepth>current_scanner.replay_stack_depth then
                        begin
                          if not check_proc_directive(true) then
                            begin
                              hintsprocessed:=try_consume_hintdirective(ttypesym(srsym).symoptions,ttypesym(srsym).deprecatedmsg);
                              if replaydepth>current_scanner.replay_stack_depth then
                                consume(_SEMICOLON);
                            end
                          else
                            hintsprocessed:=true;
                        end;
                      if replaydepth>current_scanner.replay_stack_depth then
                        parse_var_proc_directives(ttypesym(srsym));
                      handle_calling_convention(tprocvardef(result),hcc_default_actions_intf);
                      if not hintsprocessed and (replaydepth>current_scanner.replay_stack_depth) then
                        begin
                          try_consume_hintdirective(ttypesym(srsym).symoptions,ttypesym(srsym).deprecatedmsg);
                          if replaydepth>current_scanner.replay_stack_depth then
                            consume(_SEMICOLON);
                        end;
                    end;
                  procdef:
                    begin
                      pdflags:=[pd_body,pd_implemen];
                      if genericdef.owner.symtabletype=objectsymtable then
                        include(pdflags,pd_object)
                      else if genericdef.owner.symtabletype=recordsymtable then
                        include(pdflags,pd_record);
                      parse_proc_directives(pd,pdflags);
                      while try_consume_hintdirective(pd.symoptions,pd.deprecatedmsg) do
                        consume(_SEMICOLON);
                      if parse_generic then
                        handle_calling_convention(tprocdef(result),hcc_default_actions_intf)
                      else
                        handle_calling_convention(tprocdef(result),hcc_default_actions_impl);
                      proc_add_definition(tprocdef(result));
                      { for partial specializations we implicitely declare the routine as
                        having its implementation although we'll not specialize it in reality }
                      if parse_generic then
                        unset_forwarddef(result);
                    end;
                  else
                    { parse hint directives for records and arrays }
                    if replaydepth>current_scanner.replay_stack_depth then begin
                      try_consume_hintdirective(srsym.symoptions,srsym.deprecatedmsg);
                      if replaydepth>current_scanner.replay_stack_depth then
                        consume(_SEMICOLON);
                    end;
                end;
                { Consume the remainder of the buffer }
                while current_scanner.replay_stack_depth>replaydepth do
                  consume(token);

                if assigned(recordbuf) then
                  begin
                    if assigned(current_scanner.recordtokenbuf) then
                      internalerror(2014050909);
                    current_scanner.recordtokenbuf:=recordbuf;
                  end;

                block_type:=old_block_type;
                current_procinfo:=old_current_procinfo;
                current_module.procinfo:=old_module_procinfo;
                if parse_class_parent then
                  begin
                    current_structdef:=old_current_structdef;
                    current_genericdef:=old_current_genericdef;
                    current_specializedef:=old_current_specializedef;
                  end;
              end;

            { extract all created symbols and defs from the temporary symtable
              and add them to the specializest }
            for i:=tempst.SymList.Count-1 downto 0 do
              begin
                item:=tempst.SymList.Items[i];
                { using changeowner the symbol is automatically added to the
                  new symtable }
                tsym(item).ChangeOwner(specializest);
              end;

            for i:=tempst.DefList.Count-1 downto 0 do
              begin
                item:=tempst.DefList.Items[i];
                { using changeowner the def is automatically added to the new
                  symtable }
                tdef(item).ChangeOwner(specializest);
                { for partial specializations we implicitely declare any methods as having their
                  implementations although we'll not specialize them in reality }
                if parse_generic then
                  unset_forwarddef(tdef(item));
              end;

            { if a generic was declared during the specialization we need to
              flag the specialize symtable accordingly }
            if sto_has_generic in tempst.tableoptions then
              specializest.includeoption(sto_has_generic);

            tempst.free;

            specialization_done(state);

            { procdefs are only added once we know which overload we use }
            if not parse_generic and (result.typ<>procdef) then
              current_module.pendingspecializations.add(result.typename,result);
          end;

        generictypelist.free;
        if assigned(genericdef) then
          begin
            { check the hints of the found generic symbol }
            if genericdef.typ=procdef then
              srsym:=tprocdef(genericdef).procsym
            else
              srsym:=genericdef.typesym;
            check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
          end;
      end;


    procedure generate_specialization(var tt:tdef;parse_class_parent:boolean;_prettyname:string;parsedtype:tdef;symname:string;parsedpos:tfileposinfo);
      var
        context : tspecializationcontext;
        genericdef : tstoreddef;
      begin
        genericdef:=tstoreddef(generate_specialization_phase1(context,tt,parsedtype,symname,parsedpos));
        if genericdef<>generrordef then
          genericdef:=tstoreddef(generate_specialization_phase2(context,genericdef,parse_class_parent,_prettyname));
        tt:=genericdef;
        if assigned(context) then
          context.free;
      end;


    function parse_generic_parameters(allowconstraints:boolean):tfphashobjectlist;
      var
        generictype : ttypesym;
        i,firstidx : longint;
        srsymtable : tsymtable;
        basedef,def : tdef;
        defname : tidstring;
        allowconstructor,
        doconsume : boolean;
        constraintdata : tgenericconstraintdata;
        old_block_type : tblock_type;
        fileinfo : tfileposinfo;
      begin
        result:=tfphashobjectlist.create(false);
        firstidx:=0;
        old_block_type:=block_type;
        block_type:=bt_type;
        repeat
          if token=_ID then
            begin
              generictype:=ctypesym.create(orgpattern,cundefinedtype);
              { type parameters need to be added as strict private }
              generictype.visibility:=vis_strictprivate;
              include(generictype.symoptions,sp_generic_para);
              result.add(orgpattern,generictype);
            end;
          consume(_ID);
          fileinfo:=current_tokenpos;
          if try_to_consume(_COLON) then
            begin
              if not allowconstraints then
                Message(parser_e_generic_constraints_not_allowed_here);
              { construct a name which can be used for a type specification }
              constraintdata:=tgenericconstraintdata.create;
              constraintdata.fileinfo:=fileinfo;
              defname:='';
              str(current_module.deflist.count,defname);
              defname:='$gendef'+defname;

              allowconstructor:=m_delphi in current_settings.modeswitches;

              basedef:=generrordef;
              repeat
                doconsume:=true;

                case token of
                  _CONSTRUCTOR:
                    begin
                      if not allowconstructor or (gcf_constructor in constraintdata.flags) then
                        Message(parser_e_illegal_expression);
                      include(constraintdata.flags,gcf_constructor);
                      allowconstructor:=false;
                    end;
                  _CLASS:
                    begin
                      if gcf_class in constraintdata.flags then
                        Message(parser_e_illegal_expression);
                      if basedef=generrordef then
                        include(constraintdata.flags,gcf_class)
                      else
                        Message(parser_e_illegal_expression);
                    end;
                  _RECORD:
                    begin
                      if ([gcf_constructor,gcf_class]*constraintdata.flags<>[])
                          or (constraintdata.interfaces.count>0) then
                        Message(parser_e_illegal_expression)
                      else
                        begin
                          srsymtable:=trecordsymtable.create(defname,0,1);
                          basedef:=crecorddef.create(defname,srsymtable);
                          include(constraintdata.flags,gcf_record);
                          allowconstructor:=false;
                        end;
                    end;
                  else
                    begin
                      { after single_type "token" is the trailing ",", ";" or
                        ">"! }
                      doconsume:=false;
                      { def is already set to a class or record }
                      if gcf_record in constraintdata.flags then
                        Message(parser_e_illegal_expression);
                      single_type(def, [stoAllowSpecialization]);
                      { only types that are inheritable are allowed }
                      if (def.typ<>objectdef) or
                          not (tobjectdef(def).objecttype in [odt_class,odt_interfacecom,odt_interfacecorba,odt_interfacejava,odt_javaclass]) then
                        Message1(type_e_class_or_interface_type_expected,def.typename)
                      else
                        case tobjectdef(def).objecttype of
                          odt_class,
                          odt_javaclass:
                            begin
                              if gcf_class in constraintdata.flags then
                                { "class" + concrete class is not allowed }
                                Message(parser_e_illegal_expression)
                              else
                                { do we already have a concrete class? }
                                if basedef<>generrordef then
                                  Message(parser_e_illegal_expression)
                                else
                                  basedef:=def;
                            end;
                          odt_interfacecom,
                          odt_interfacecorba,
                          odt_interfacejava,
                          odt_dispinterface:
                            constraintdata.interfaces.add(def);
                          else
                            ;
                        end;
                    end;
                end;
                if doconsume then
                  consume(token);
              until not try_to_consume(_COMMA);

              if ([gcf_class,gcf_constructor]*constraintdata.flags<>[]) or
                  (constraintdata.interfaces.count>1) or
                  (
                    (basedef.typ=objectdef) and
                    (tobjectdef(basedef).objecttype in [odt_javaclass,odt_class])
                  ) then
                begin
                  if basedef.typ=errordef then
                    { don't pass an errordef as a parent to a tobjectdef }
                    basedef:=class_tobject
                  else
                    if (basedef.typ<>objectdef) or
                        not (tobjectdef(basedef).objecttype in [odt_javaclass,odt_class]) then
                      internalerror(2012101101);
                  basedef:=cobjectdef.create(tobjectdef(basedef).objecttype,defname,tobjectdef(basedef),false);
                  for i:=0 to constraintdata.interfaces.count-1 do
                    tobjectdef(basedef).implementedinterfaces.add(
                      timplementedinterface.create(tobjectdef(constraintdata.interfaces[i])));
                end
              else
                if constraintdata.interfaces.count=1 then
                  begin
                    if basedef.typ<>errordef then
                      internalerror(2013021601);
                    def:=tdef(constraintdata.interfaces[0]);
                    basedef:=cobjectdef.create(tobjectdef(def).objecttype,defname,tobjectdef(def),false);
                    constraintdata.interfaces.delete(0);
                  end;
              if basedef.typ<>errordef then
                with tstoreddef(basedef) do
                  begin
                    genconstraintdata:=tgenericconstraintdata.create;
                    genconstraintdata.flags:=constraintdata.flags;
                    genconstraintdata.interfaces.assign(constraintdata.interfaces);
                    genconstraintdata.fileinfo:=constraintdata.fileinfo;
                    include(defoptions,df_genconstraint);
                  end;

              for i:=firstidx to result.count-1 do
                ttypesym(result[i]).typedef:=basedef;
              { we need a typesym in case we do a Delphi-mode inline
                specialization with this parameter; so just use the first sym }
              if not assigned(basedef.typesym) then
                basedef.typesym:=ttypesym(result[firstidx]);
              firstidx:=result.count;

              constraintdata.free;
            end
          else
            begin
              if token=_SEMICOLON then
                begin
                  { two different typeless parameters are considered as incompatible }
                  for i:=firstidx to result.count-1 do
                    begin
                      ttypesym(result[i]).typedef:=cundefineddef.create(false);
                      ttypesym(result[i]).typedef.typesym:=ttypesym(result[i]);
                    end;
                  { a semicolon terminates a type parameter group }
                  firstidx:=result.count;
                end;
            end;
        until not (try_to_consume(_COMMA) or try_to_consume(_SEMICOLON));
        { two different typeless parameters are considered as incompatible }
        for i:=firstidx to result.count-1 do
          begin
            ttypesym(result[i]).typedef:=cundefineddef.create(false);
            ttypesym(result[i]).typedef.typesym:=ttypesym(result[i]);
          end;
        block_type:=old_block_type;
      end;


    procedure insert_generic_parameter_types(def:tstoreddef;genericdef:tstoreddef;genericlist:tfphashobjectlist);
      var
        i : longint;
        generictype,sym : ttypesym;
        st : tsymtable;
      begin
        def.genericdef:=genericdef;
        if not assigned(genericlist) then
          exit;

        if assigned(genericdef) then
          include(def.defoptions,df_specialization)
        else
          if genericlist.count>0 then
            include(def.defoptions,df_generic);

        case def.typ of
          recorddef,objectdef: st:=tabstractrecorddef(def).symtable;
          arraydef: st:=tarraydef(def).symtable;
          procvardef,procdef: st:=tabstractprocdef(def).parast;
          else
            internalerror(201101020);
        end;

        if (genericlist.count>0) and not assigned(def.genericparas) then
          def.genericparas:=tfphashobjectlist.create(false);
        for i:=0 to genericlist.count-1 do
          begin
            generictype:=ttypesym(genericlist[i]);
            if assigned(generictype.owner) then
              begin
                sym:=ctypesym.create(genericlist.nameofindex(i),generictype.typedef);
                { type parameters need to be added as strict private }
                sym.visibility:=vis_strictprivate;
                st.insert(sym);
                include(sym.symoptions,sp_generic_para);
              end
            else
              begin
                if (generictype.typedef.typ=undefineddef) and (generictype.typedef<>cundefinedtype) then
                  begin
                    { the generic parameters were parsed before the genericdef existed thus the
                      undefineddefs were added as part of the parent symtable }
                    if assigned(generictype.typedef.owner) then
                      generictype.typedef.owner.DefList.Extract(generictype.typedef);
                    generictype.typedef.changeowner(st);
                  end;
                st.insert(generictype);
                include(generictype.symoptions,sp_generic_para);
              end;
            def.genericparas.add(genericlist.nameofindex(i),generictype);
          end;
       end;

    procedure maybe_insert_generic_rename_symbol(const name:tidstring;genericlist:tfphashobjectlist);
      var
        gensym : ttypesym;
      begin
        { for generics in non-Delphi modes we insert a private type symbol
          that has the same base name as the currently parsed generic and
          that references this defs }
        if not (m_delphi in current_settings.modeswitches) and
            (
              (
                parse_generic and
                assigned(genericlist) and
                (genericlist.count>0)
              ) or
              (
                assigned(current_specializedef) and
                assigned(current_structdef.genericdef) and
                (current_structdef.genericdef.typ in [objectdef,recorddef]) and
                (pos('$',name)>0)
              )
            ) then
          begin
            { we need to pass nil as def here, because the constructor wants
              to set the typesym of the def which is not what we want }
            gensym:=ctypesym.create(copy(name,1,pos('$',name)-1),nil);
            gensym.typedef:=current_structdef;
            include(gensym.symoptions,sp_internal);
            { the symbol should be only visible to the generic class
              itself }
            gensym.visibility:=vis_strictprivate;
            symtablestack.top.insert(gensym);
          end;
      end;

    function generate_generic_name(const name:tidstring;specializename:ansistring;owner_hierarchy:string):tidstring;
    var
      crc : cardinal;
    begin
      if specializename='' then
        internalerror(2012061901);
      { build the new type's name }
      crc:=UpdateCrc32(0,specializename[1],length(specializename));
      result:=name+'$crc'+hexstr(crc,8);
      if owner_hierarchy<>'' then
        begin
          crc:=UpdateCrc32(0,owner_hierarchy[1],length(owner_hierarchy));
          result:=result+'$crc'+hexstr(crc,8);
        end;
    end;

    procedure split_generic_name(const name:tidstring;out nongeneric:string;out count:longint);
      var
        i,code : longint;
        countstr : string;
      begin
        for i:=length(name) downto 1 do
          if name[i]='$' then
            begin
              nongeneric:=copy(name,1,i-1);
              countstr:=copy(name,i+1,length(name)-i);
              val(countstr,count,code);
              if code<>0 then
                break;
              exit;
            end;
        nongeneric:=name;
        count:=0;
      end;


    procedure add_generic_dummysym(sym:tsym);
      var
        list: TFPObjectList;
        srsym : tsym;
        srsymtable : tsymtable;
        entry : tgenericdummyentry;
      begin
        if sp_generic_dummy in sym.symoptions then
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
            else if (sym.typ=procsym) and
                (tprocsym(sym).procdeflist.count>0) then
              srsym:=sym
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
      end;


    function resolve_generic_dummysym(const name:tidstring):tsym;
      var
        list : tfpobjectlist;
      begin
        list:=tfpobjectlist(current_module.genericdummysyms.find(name));
        if assigned(list) and (list.count>0) then
          result:=tgenericdummyentry(list.last).resolvedsym
        else
          result:=nil;
      end;


    function could_be_generic(const name:tidstring):boolean;
      begin
        result:=(name<>'') and
                  (current_module.genericdummysyms.findindexof(name)>=0);
      end;

    procedure specialization_init(genericdef:tdef;var state: tspecializationstate);
    var
      pu : tused_unit;
      hmodule : tmodule;
      unitsyms : TFPHashObjectList;
      sym : tsym;
      i : Integer;
    begin
      if not assigned(genericdef) then
        internalerror(200705151);
      { Setup symtablestack at definition time
        to get types right, however this is not perfect, we should probably record
        the resolved symbols }
      state.oldsymtablestack:=symtablestack;
      state.oldextendeddefs:=current_module.extendeddefs;
      state.oldgenericdummysyms:=current_module.genericdummysyms;
      current_module.extendeddefs:=TFPHashObjectList.create(true);
      current_module.genericdummysyms:=tfphashobjectlist.create(true);
      symtablestack:=tdefawaresymtablestack.create;
      hmodule:=find_module_from_symtable(genericdef.owner);
      if hmodule=nil then
        internalerror(200705152);
      { collect all unit syms in the generic's unit as we need to establish
        their unitsym.module link again so that unit identifiers can be used }
      unitsyms:=tfphashobjectlist.create(false);
      if (hmodule<>current_module) and assigned(hmodule.globalsymtable) then
        for i:=0 to hmodule.globalsymtable.symlist.count-1 do
          begin
            sym:=tsym(hmodule.globalsymtable.symlist[i]);
            if sym.typ=unitsym then
              unitsyms.add(upper(sym.realname),sym);
          end;
      { add all units if we are specializing inside the current unit (as the
        generic could have been declared in the implementation part), but load
        only interface units, if we are in a different unit as then the generic
        needs to be in the interface section }
      pu:=tused_unit(hmodule.used_units.first);
      while assigned(pu) do
        begin
          if not assigned(pu.u.globalsymtable) then
            { in certain circular, but valid unit constellations it can happen
              that we specialize a generic in a different unit that was used
              in the implementation section of the generic's unit and were the
              interface is still being parsed and thus the localsymtable is in
              reality the global symtable }
            if pu.u.in_interface then
              symtablestack.push(pu.u.localsymtable)
            else
              internalerror(200705153)
          else
            symtablestack.push(pu.u.globalsymtable);
          sym:=tsym(unitsyms.find(pu.u.modulename^));
          if assigned(sym) and not assigned(tunitsym(sym).module) then
            tunitsym(sym).module:=pu.u;
          pu:=tused_unit(pu.next);
        end;
      unitsyms.free;
      if assigned(hmodule.globalsymtable) then
        symtablestack.push(hmodule.globalsymtable);
      { push the localsymtable if needed }
      if ((hmodule<>current_module) or not current_module.in_interface)
          and assigned(hmodule.localsymtable) then
        symtablestack.push(hmodule.localsymtable);
    end;

    procedure specialization_done(var state: tspecializationstate);
    begin
      { Restore symtablestack }
      current_module.extendeddefs.free;
      current_module.extendeddefs:=state.oldextendeddefs;
      current_module.genericdummysyms.free;
      current_module.genericdummysyms:=state.oldgenericdummysyms;
      symtablestack.free;
      symtablestack:=state.oldsymtablestack;
      { clear the state record to be on the safe side }
      fillchar(state, sizeof(state), 0);
    end;


{****************************************************************************
                      SPECIALIZATION BODY GENERATION
****************************************************************************}


    procedure process_procdef(def:tprocdef;hmodule:tmodule);
      var
        oldcurrent_filepos : tfileposinfo;
      begin
        if assigned(def.genericdef) and
            (def.genericdef.typ=procdef) and
            assigned(tprocdef(def.genericdef).generictokenbuf) then
          begin
            if not assigned(tprocdef(def.genericdef).generictokenbuf) then
              internalerror(2015061902);
            oldcurrent_filepos:=current_filepos;
            current_filepos:=tprocdef(def.genericdef).fileinfo;
            { use the index the module got from the current compilation process }
            current_filepos.moduleindex:=hmodule.unit_index;
            current_tokenpos:=current_filepos;
            current_scanner.startreplaytokens(tprocdef(def.genericdef).generictokenbuf,hmodule.change_endian);
            read_proc_body(def);
            current_filepos:=oldcurrent_filepos;
          end
        { synthetic routines will be implemented afterwards }
        else if def.synthetickind=tsk_none then
          MessagePos1(def.fileinfo,sym_e_forward_not_resolved,def.fullprocname(false));
      end;


    function process_abstractrecorddef(def:tabstractrecorddef):boolean;
      var
        i  : longint;
        hp : tdef;
        hmodule : tmodule;
      begin
        result:=true;
        hmodule:=find_module_from_symtable(def.genericdef.owner);
        if hmodule=nil then
          internalerror(201202041);
        for i:=0 to def.symtable.DefList.Count-1 do
          begin
            hp:=tdef(def.symtable.DefList[i]);
            if hp.typ=procdef then
             begin
               { only generate the code if we need a body }
               if assigned(tprocdef(hp).struct) and not tprocdef(hp).forwarddef then
                 continue;
               { and the body is available already (which is implicitely the
                 case if the generic routine is part of another unit) }
               if ((hmodule=current_module) or (hmodule.state=ms_compile)) and
                  { may not be assigned in case it's a synthetic procdef that
                    still needs to be generated }
                  assigned(tprocdef(hp).genericdef) and
                  tprocdef(tprocdef(hp).genericdef).forwarddef then
                 begin
                   result:=false;
                   continue;
                 end;
               process_procdef(tprocdef(hp),hmodule);
             end
           else
             if hp.typ in [objectdef,recorddef] then
               { generate code for subtypes as well }
               result:=process_abstractrecorddef(tabstractrecorddef(hp)) and result;
         end;
      end;


    procedure generate_specialization_procs;
      var
        i : longint;
        list,
        readdlist : tfpobjectlist;
        def : tstoreddef;
        state : tspecializationstate;
        hmodule : tmodule;
      begin
        { first copy all entries and then work with that list to ensure that
          we don't get an infinite recursion }
        list:=tfpobjectlist.create(false);
        readdlist:=tfpobjectlist.create(false);

        for i:=0 to current_module.pendingspecializations.Count-1 do
          list.add(current_module.pendingspecializations.Items[i]);

        current_module.pendingspecializations.clear;

        for i:=0 to list.count-1 do
          begin
            def:=tstoreddef(list[i]);
            if not tstoreddef(def).is_specialization then
              continue;
            case def.typ of
              procdef:
                begin
                  { the use of forwarddef should not backfire as the
                    specialization always belongs to the current module }
                  if not tprocdef(def).forwarddef then
                    continue;
                  if not assigned(def.genericdef) then
                    internalerror(2015061903);
                  hmodule:=find_module_from_symtable(def.genericdef.owner);
                  if hmodule=nil then
                    internalerror(2015061904);
                  { we need to check for a forward declaration only if the
                    generic was declared in the same unit (otherwise there
                    should be one) }
                  if ((hmodule=current_module) or (hmodule.state=ms_compile)) and tprocdef(def.genericdef).forwarddef then
                    begin
                      readdlist.add(def);
                      continue;
                    end;

                  specialization_init(tstoreddef(def).genericdef,state);

                  process_procdef(tprocdef(def),hmodule);

                  specialization_done(state);
                end;
              recorddef,
              objectdef:
                begin
                  specialization_init(tstoreddef(def).genericdef,state);

                  if not process_abstractrecorddef(tabstractrecorddef(def)) then
                    readdlist.add(def);

                  specialization_done(state);
                end;
              else
                ;
            end;
          end;

        { add those defs back to the pending list for which we don't yet have
          all method bodies }
        for i:=0 to readdlist.count-1 do
          current_module.pendingspecializations.add(tstoreddef(readdlist[i]).typename,readdlist[i]);

        readdlist.free;
        list.free;
      end;


    procedure maybe_add_pending_specialization(def:tdef);
      var
        hmodule : tmodule;
        st : tsymtable;
      begin
        if parse_generic then
          exit;
        st:=def.owner;
        while st.symtabletype in [localsymtable] do
          st:=st.defowner.owner;
        hmodule:=find_module_from_symtable(st);
        if tstoreddef(def).is_specialization and (hmodule=current_module) then
          current_module.pendingspecializations.add(def.typename,def);
      end;

end.
