{
    Copyright (c) 1998-2002 by Florian Klaempfl, Daniel Mantione

    Does the parsing of the procedures/functions

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
unit pdecsub;

{$i fpcdefs.inc}

interface

    uses
      tokens,symconst,symtype,symdef,symsym;

    type
      tpdflag=(
        pd_body,         { directive needs a body }
        pd_implemen,     { directive can be used in implementation section }
        pd_interface,    { directive can be used in interface section }
        pd_object,       { directive can be used with object declaration }
        pd_record,       { directive can be used with record declaration }
        pd_procvar,      { directive can be used with procvar declaration }
        pd_notobject,    { directive can not be used with object declaration }
        pd_notrecord,    { directive can not be used with record declaration }
        pd_notobjintf,   { directive can not be used with interface declaration }
        pd_notprocvar,   { directive can not be used with procvar declaration }
        pd_dispinterface,{ directive can be used with dispinterface methods }
        pd_cppobject,    { directive can be used with cppclass }
        pd_objcclass,    { directive can be used with objcclass }
        pd_objcprot,     { directive can be used with objcprotocol }
        pd_nothelper,    { directive can not be used with record/class helper declaration }
        pd_javaclass,    { directive can be used with Java class }
        pd_intfjava      { directive can be used with Java interface }
      );
      tpdflags=set of tpdflag;

      // flags of handle_calling_convention routine
      thccflag=(
        hcc_check,                // perform checks and outup errors if found
        hcc_insert_hidden_paras   // insert hidden parameters
      );
      thccflags=set of thccflag;
    const
      hcc_all=[hcc_check,hcc_insert_hidden_paras];

    function  check_proc_directive(isprocvar:boolean):boolean;

    function  proc_add_definition(var currpd:tprocdef):boolean;
    function  proc_get_importname(pd:tprocdef):string;
    procedure proc_set_mangledname(pd:tprocdef);

    procedure handle_calling_convention(pd:tabstractprocdef;flags:thccflags=hcc_all);

    procedure parse_parameter_dec(pd:tabstractprocdef);
    procedure parse_proc_directives(pd:tabstractprocdef;var pdflags:tpdflags);
    procedure parse_var_proc_directives(sym:tsym);
    procedure parse_object_proc_directives(pd:tabstractprocdef);
    procedure parse_record_proc_directives(pd:tabstractprocdef);
    function  parse_proc_head(astruct:tabstractrecorddef;potype:tproctypeoption;out pd:tprocdef):boolean;
    function  parse_proc_dec(isclassmethod:boolean;astruct:tabstractrecorddef):tprocdef;

    { parse a record method declaration (not a (class) constructor/destructor) }
    function parse_record_method_dec(astruct: tabstractrecorddef; is_classdef: boolean): tprocdef;

    procedure insert_record_hidden_paras(astruct: trecorddef);

    { helper functions - they insert nested objects hierarcy to the symtablestack
      with object hierarchy
    }
    function push_child_hierarcy(obj:tabstractrecorddef):integer;
    function pop_child_hierarchy(obj:tabstractrecorddef):integer;
    function push_nested_hierarchy(obj:tabstractrecorddef):integer;
    function pop_nested_hierarchy(obj:tabstractrecorddef):integer;

implementation

    uses
       SysUtils,
       { common }
       cutils,cclasses,
       { global }
       globtype,globals,verbose,constexp,
       systems,fpccrc,
       cpuinfo,
       { symtable }
       symbase,symtable,symcreat,defutil,defcmp,paramgr,cpupara,
       { pass 1 }
       fmodule,node,htypechk,
       nmat,nadd,ncal,nset,ncnv,ninl,ncon,nld,nflw,
       objcutil,
       { parser }
       scanner,
       pbase,pexpr,ptype,pdecl,pparautl
{$ifdef jvm}
       ,pjvm
{$endif}
       ;

    const
      { Please leave this here, this module should NOT use
        these variables.
        Declaring it as string here results in an error when compiling (PFV) }
      current_procinfo = 'error';

    function push_child_hierarcy(obj:tabstractrecorddef):integer;
      var
        _class,hp : tobjectdef;
      begin
        if obj.typ=recorddef then
          begin
            symtablestack.push(obj.symtable);
            result:=1;
            exit;
          end;
        result:=0;
        { insert class hierarchy in the reverse order }
        hp:=nil;
        repeat
          _class:=tobjectdef(obj);
          while _class.childof<>hp do
            _class:=_class.childof;
          hp:=_class;
          symtablestack.push(_class.symtable);
          inc(result);
        until hp=obj;
      end;

    function push_nested_hierarchy(obj:tabstractrecorddef):integer;
      begin
        result:=0;
        if obj.owner.symtabletype in [ObjectSymtable,recordsymtable] then
          inc(result,push_nested_hierarchy(tabstractrecorddef(obj.owner.defowner)));
        inc(result,push_child_hierarcy(obj));
      end;

    function pop_child_hierarchy(obj:tabstractrecorddef):integer;
      var
        _class : tobjectdef;
      begin
        if obj.typ=recorddef then
          begin
            symtablestack.pop(obj.symtable);
            result:=1;
            exit;
          end;
        result:=0;
        _class:=tobjectdef(obj);
        while assigned(_class) do
          begin
            symtablestack.pop(_class.symtable);
            _class:=_class.childof;
            inc(result);
          end;
      end;

    function pop_nested_hierarchy(obj:tabstractrecorddef):integer;
      begin
        result:=pop_child_hierarchy(obj);
        if obj.owner.symtabletype in [ObjectSymtable,recordsymtable] then
          inc(result,pop_nested_hierarchy(tabstractrecorddef(obj.owner.defowner)));
      end;


    procedure check_msg_para(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ<>paravarsym) then
         exit;
        with tparavarsym(p) do
          begin
            { Count parameters }
            if (paranr>=10) then
              inc(plongint(arg)^);
            { First parameter must be var }
            if (paranr=10) and
               (varspez<>vs_var) then
              MessagePos(fileinfo,parser_e_ill_msg_param);
          end;
      end;


    procedure set_addr_param_regable(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ<>paravarsym) then
         exit;
        with tparavarsym(p) do
         begin
           if (not needs_finalization) and
              paramanager.push_addr_param(varspez,vardef,tprocdef(arg).proccalloption) then
             varregable:=vr_intreg;
         end;
      end;


    procedure parse_parameter_dec(pd:tabstractprocdef);
      {
        handle_procvar needs the same changes
      }
      type
        tppv = (pv_none,pv_proc,pv_func);
      var
        sc      : TFPObjectList;
        hdef    : tdef;
        arrayelementdef : tdef;
        vs      : tparavarsym;
        i       : longint;
        srsym   : tsym;
        pv      : tprocvardef;
        varspez : Tvarspez;
        defaultvalue : tconstsym;
        defaultrequired : boolean;
        old_block_type : tblock_type;
        currparast : tparasymtable;
        parseprocvar : tppv;
        locationstr : string;
        paranr : integer;
        dummytype : ttypesym;
        explicit_paraloc,
        need_array,
        is_univ: boolean;
      begin
        old_block_type:=block_type;
        explicit_paraloc:=false;
        consume(_LKLAMMER);
        { Delphi/Kylix supports nonsense like }
        { procedure p();                      }
        if try_to_consume(_RKLAMMER) and
          not(m_tp7 in current_settings.modeswitches) then
          exit;
        { parsing a proc or procvar ? }
        currparast:=tparasymtable(pd.parast);
        { reset }
        sc:=TFPObjectList.create(false);
        defaultrequired:=false;
        paranr:=0;
        block_type:=bt_var;
        is_univ:=false;
        repeat
          parseprocvar:=pv_none;
          if try_to_consume(_VAR) then
            varspez:=vs_var
          else
            if try_to_consume(_CONST) then
              varspez:=vs_const
          else
            if (m_out in current_settings.modeswitches) and
               try_to_consume(_OUT) then
              varspez:=vs_out
          else
           if try_to_consume(_CONSTREF) then
             varspez:=vs_constref
          else
            if (m_mac in current_settings.modeswitches) and
               try_to_consume(_POINTPOINTPOINT) then
              begin
                include(pd.procoptions,po_varargs);
                break;
              end
          else
            if (m_nested_procvars in current_settings.modeswitches) and
               try_to_consume(_PROCEDURE) then
              begin
                parseprocvar:=pv_proc;
                varspez:=vs_const;
              end
          else
            if (m_nested_procvars in current_settings.modeswitches) and
               try_to_consume(_FUNCTION) then
              begin
                parseprocvar:=pv_func;
                varspez:=vs_const;
              end
          else
              varspez:=vs_value;
          defaultvalue:=nil;
          hdef:=nil;
          { read identifiers and insert with error type }
          sc.clear;
          repeat
            inc(paranr);
            vs:=tparavarsym.create(orgpattern,paranr*10,varspez,generrordef,[]);
            currparast.insert(vs);
            if assigned(vs.owner) then
             sc.add(vs)
            else
             vs.free;
            consume(_ID);
          until not try_to_consume(_COMMA);
          locationstr:='';
          { macpas anonymous procvar }
          if parseprocvar<>pv_none then
           begin
             { inline procvar definitions are always nested procvars }
             pv:=tprocvardef.create(normal_function_level+1);
             if token=_LKLAMMER then
               parse_parameter_dec(pv);
             if parseprocvar=pv_func then
              begin
                block_type:=bt_var_type;
                consume(_COLON);
                single_type(pv.returndef,[]);
                block_type:=bt_var;
              end;
             hdef:=pv;
             { possible proc directives }
             if check_proc_directive(true) then
               begin
                  dummytype:=ttypesym.create('unnamed',hdef);
                  parse_var_proc_directives(tsym(dummytype));
                  dummytype.typedef:=nil;
                  hdef.typesym:=nil;
                  dummytype.free;
               end;
             { Add implicit hidden parameters and function result }
             handle_calling_convention(pv);
{$ifdef jvm}
             { anonymous -> no name }
             jvm_create_procvar_class('',pv);
{$endif}
           end
          else
          { read type declaration, force reading for value paras }
           if (token=_COLON) or (varspez=vs_value) then
           begin
             consume(_COLON);
             { check for an open array }
             need_array:=false;
             { bitpacked open array are not yet supported }
             if (token=_PACKED) and
                not(cs_bitpacking in current_settings.localswitches) then
               begin
                 consume(_PACKED);
                 need_array:=true;
               end;
             if (token=_ARRAY) or
                need_array then
              begin
                consume(_ARRAY);
                consume(_OF);
                { define range and type of range }
                hdef:=tarraydef.create(0,-1,s32inttype);
                { array of const ? }
                if (token=_CONST) and (m_objpas in current_settings.modeswitches) then
                 begin
                   consume(_CONST);
                   srsym:=search_system_type('TVARREC');
                   tarraydef(hdef).elementdef:=ttypesym(srsym).typedef;
                   include(tarraydef(hdef).arrayoptions,ado_IsArrayOfConst);
                 end
                else
                 begin
                   { define field type }
                   single_type(arrayelementdef,[]);
                   tarraydef(hdef).elementdef:=arrayelementdef;
                 end;
              end
             else
              begin
                if (m_mac in current_settings.modeswitches) then
                  is_univ:=try_to_consume(_UNIV);

                if try_to_consume(_TYPE) then
                  hdef:=ctypedformaltype
                else
                  begin
                    block_type:=bt_var_type;
                    single_type(hdef,[stoAllowSpecialization]);
                    block_type:=bt_var;
                  end;

                { open string ? }
                if is_shortstring(hdef) then
                  begin
                    case varspez of
                      vs_var,vs_out,vs_constref:
                        begin
                          { not 100% Delphi-compatible: type xstr=string[255] cannot
                            become an openstring there, while here it can }
                          if (cs_openstring in current_settings.moduleswitches) and
                             (tstringdef(hdef).len=255) then
                            hdef:=openshortstringtype
                        end;
                      vs_value:
                       begin
                         { value "openstring" parameters don't make sense (the
                            original string can never be modified, so there's no
                            use in passing its original length), so change these
                            into regular shortstring parameters (seems to be what
                            Delphi also does) }
                        if is_open_string(hdef) then
                          hdef:=cshortstringtype;
                       end;
                    end;
                  end;
                if (target_info.system in [system_powerpc_morphos,system_m68k_amiga]) then
                  begin
                    if (idtoken=_LOCATION) then
                      begin
                        consume(_LOCATION);
                        locationstr:=cstringpattern;
                        consume(_CSTRING);
                      end
                    else
                      begin
                        if explicit_paraloc then
                          Message(parser_e_paraloc_all_paras);
                        locationstr:='';
                      end;
                  end
                else
                  locationstr:='';

                { default parameter }
                if (m_default_para in current_settings.modeswitches) then
                 begin
                   if try_to_consume(_EQ) then
                    begin
                      vs:=tparavarsym(sc[0]);
                      if sc.count>1 then
                        Message(parser_e_default_value_only_one_para);
                      { prefix 'def' to the parameter name }
                      defaultvalue:=ReadConstant('$def'+vs.name,vs.fileinfo);
                      if assigned(defaultvalue) then
                        begin
                          include(defaultvalue.symoptions,sp_internal);
                          pd.parast.insert(defaultvalue);
                        end;
                      defaultrequired:=true;
                    end
                   else
                    begin
                      if defaultrequired then
                        Message1(parser_e_default_value_expected_for_para,vs.name);
                    end;
                 end;
              end;
           end
          else
           hdef:=cformaltype;

          { File types are only allowed for var and out parameters }
          if (hdef.typ=filedef) and
             not(varspez in [vs_out,vs_var]) then
            CGMessage(cg_e_file_must_call_by_reference);

          { Dispinterfaces are restricted to using only automatable types }
          if (pd.typ=procdef) and is_dispinterface(tprocdef(pd).struct) and
             not is_automatable(hdef) then
            Message1(type_e_not_automatable,hdef.typename);

          { univ cannot be used with types whose size is not known at compile
            time }
          if is_univ and
             not is_valid_univ_para_type(hdef) then
            Message1(parser_e_invalid_univ_para,hdef.typename);

          for i:=0 to sc.count-1 do
            begin
              vs:=tparavarsym(sc[i]);
              vs.univpara:=is_univ;
              { update varsym }
              vs.vardef:=hdef;
              vs.defaultconstsym:=defaultvalue;

              if (target_info.system in [system_powerpc_morphos,system_m68k_amiga]) then
                begin
                  if locationstr<>'' then
                    begin
                      if sc.count>1 then
                        Message(parser_e_paraloc_only_one_para);
                      if (paranr>1) and not(explicit_paraloc) then
                        Message(parser_e_paraloc_all_paras);
                      explicit_paraloc:=true;
                      include(vs.varoptions,vo_has_explicit_paraloc);
                      if not(paramanager.parseparaloc(vs,upper(locationstr))) then
                        message(parser_e_illegal_explicit_paraloc);
                    end
                  else
                    if explicit_paraloc then
                      Message(parser_e_paraloc_all_paras);
                end;
            end;
        until not try_to_consume(_SEMICOLON);

        if explicit_paraloc then
          begin
            pd.has_paraloc_info:=callerside;
            include(pd.procoptions,po_explicitparaloc);
          end;
        { remove parasymtable from stack }
        sc.free;
        { reset object options }
        block_type:=old_block_type;
        consume(_RKLAMMER);
      end;


    function parse_proc_head(astruct:tabstractrecorddef;potype:tproctypeoption;out pd:tprocdef):boolean;
      var
        hs       : string;
        orgsp,sp : TIDString;
        srsym : tsym;
        checkstack : psymtablestackitem;
        procstartfilepos : tfileposinfo;
        searchagain : boolean;
        st,
        genericst: TSymtable;
        aprocsym : tprocsym;
        popclass : integer;
        ImplIntf : TImplementedInterface;
        old_parse_generic : boolean;
        old_current_structdef: tabstractrecorddef;
        old_current_genericdef,
        old_current_specializedef: tstoreddef;
        lasttoken,lastidtoken: ttoken;

        procedure parse_operator_name;
         begin
           if (lasttoken in [first_overloaded..last_overloaded]) then
            begin
              optoken:=token;
            end
           else
            begin
              case lasttoken of
                _CARET:
                  Message1(parser_e_overload_operator_failed,'**');
                _ID:
                  case lastidtoken of
                    _ENUMERATOR:optoken:=_OP_ENUMERATOR;
                    _EXPLICIT:optoken:=_OP_EXPLICIT;
                    _INC:optoken:=_OP_INC;
                    _DEC:optoken:=_OP_DEC;
                    else
                    if (m_delphi in current_settings.modeswitches) then
                      case lastidtoken of
                        _IMPLICIT:optoken:=_ASSIGNMENT;
                        _NEGATIVE:optoken:=_MINUS;
                        _POSITIVE:optoken:=_PLUS;
                        _LOGICALNOT:optoken:=_OP_NOT;
                        _IN:optoken:=_OP_IN;
                        _EQUAL:optoken:=_EQ;
                        _NOTEQUAL:optoken:=_NE;
                        _GREATERTHAN:optoken:=_GT;
                        _GREATERTHANOREQUAL:optoken:=_GTE;
                        _LESSTHAN:optoken:=_LT;
                        _LESSTHANOREQUAL:optoken:=_LTE;
                        _ADD:optoken:=_PLUS;
                        _SUBTRACT:optoken:=_MINUS;
                        _MULTIPLY:optoken:=_STAR;
                        _DIVIDE:optoken:=_SLASH;
                        _INTDIVIDE:optoken:=_OP_DIV;
                        _MODULUS:optoken:=_OP_MOD;
                        _LEFTSHIFT:optoken:=_OP_SHL;
                        _RIGHTSHIFT:optoken:=_OP_SHR;
                        _LOGICALAND:optoken:=_OP_AND;
                        _LOGICALOR:optoken:=_OP_OR;
                        _LOGICALXOR:optoken:=_OP_XOR;
                        _BITWISEAND:optoken:=_OP_AND;
                        _BITWISEOR:optoken:=_OP_OR;
                        _BITWISEXOR:optoken:=_OP_XOR;
                        else
                          Message1(parser_e_overload_operator_failed,'');
                      end
                    else
                      Message1(parser_e_overload_operator_failed,'');
                  end
                else
                  Message1(parser_e_overload_operator_failed,'');
              end;
            end;
           sp:=overloaded_names[optoken];
           orgsp:=sp;
         end;

        procedure consume_proc_name;
          begin
            lasttoken:=token;
            lastidtoken:=idtoken;
            if potype=potype_operator then
              optoken:=NOTOKEN;
            if (potype=potype_operator) and (token<>_ID) then
              begin
                parse_operator_name;
                consume(token);
              end
            else
              begin
                sp:=pattern;
                orgsp:=orgpattern;
                consume(_ID);
              end;
          end;

        function search_object_name(sp:TIDString;gen_error:boolean):tsym;
          var
            storepos:tfileposinfo;
            srsymtable:TSymtable;
          begin
            storepos:=current_tokenpos;
            current_tokenpos:=procstartfilepos;
            searchsym(sp,result,srsymtable);
            if not assigned(result) then
              begin
                if gen_error then
                  identifier_not_found(orgsp);
                result:=generrorsym;
              end;
            current_tokenpos:=storepos;
          end;

        function consume_generic_type_parameter:boolean;
          var
            idx : integer;
            genparalistdecl : TFPHashList;
            genname : tidstring;
            s : shortstring;
          begin
            result:=not assigned(astruct)and(m_delphi in current_settings.modeswitches);
            if result then
              begin
                { parse all parameters first so we can check whether we have
                  the correct generic def available }
                genparalistdecl:=TFPHashList.Create;
                if try_to_consume(_LT) then
                  begin
                    { start with 1, so Find can return Nil (= 0) }
                    idx:=1;
                    repeat
                      if token=_ID then
                        begin
                          genparalistdecl.Add(pattern, Pointer(PtrInt(idx)));
                          consume(_ID);
                          inc(idx);
                        end
                      else
                        begin
                          message2(scan_f_syn_expected,arraytokeninfo[_ID].str,arraytokeninfo[token].str);
                          if token<>_COMMA then
                            consume(token);
                        end;
                    until not try_to_consume(_COMMA);
                    if not try_to_consume(_GT) then
                      consume(_RSHARPBRACKET);
                  end
                else
                  begin
                    { no generic }
                    srsym:=nil;
                    exit;
                  end;

                s:='';
                str(genparalistdecl.count,s);
                genname:=sp+'$'+s;

                genparalistdecl.free;

                srsym:=search_object_name(genname,false);

                if not assigned(srsym) then
                  begin
                    { TODO : print a nicer typename that contains the parsed
                             generic types }
                    Message1(type_e_generic_declaration_does_not_match,genname);
                    srsym:=nil;
                    exit;
                  end;
              end;
          end;

      begin
        { Save the position where this procedure really starts }
        procstartfilepos:=current_tokenpos;
        old_parse_generic:=parse_generic;

        result:=false;
        pd:=nil;
        aprocsym:=nil;

        consume_proc_name;

        { examine interface map: function/procedure iname.functionname=locfuncname }
        if assigned(astruct) and
           (astruct.typ=objectdef) and
           assigned(tobjectdef(astruct).ImplementedInterfaces) and
           (tobjectdef(astruct).ImplementedInterfaces.count>0) and
           try_to_consume(_POINT) then
         begin
           srsym:=search_object_name(sp,true);
           { qualifier is interface? }
           ImplIntf:=nil;
           if (srsym.typ=typesym) and
              (ttypesym(srsym).typedef.typ=objectdef) then
             ImplIntf:=tobjectdef(astruct).find_implemented_interface(tobjectdef(ttypesym(srsym).typedef));
           if ImplIntf=nil then
             Message(parser_e_interface_id_expected);
           { must be a directly implemented interface }
           if Assigned(ImplIntf.ImplementsGetter) then
             Message2(parser_e_implements_no_mapping,ImplIntf.IntfDef.typename,astruct.objrealname^);
           consume(_ID);
           { Create unique name <interface>.<method> }
           hs:=sp+'.'+pattern;
           consume(_EQ);
           if assigned(ImplIntf) and
              (token=_ID) then
             ImplIntf.AddMapping(hs,pattern);
           consume(_ID);
           result:=true;
           exit;
         end;

        { method  ? }
        srsym:=nil;
        if (consume_generic_type_parameter or not assigned(astruct)) and
           (symtablestack.top.symtablelevel=main_program_level) and
           try_to_consume(_POINT) then
         begin
           repeat
             searchagain:=false;
             if not assigned(astruct) and not assigned(srsym) then
               srsym:=search_object_name(sp,true);
             { consume proc name }
             procstartfilepos:=current_tokenpos;
             consume_proc_name;
             { qualifier is class name ? }
             if (srsym.typ=typesym) and
                (ttypesym(srsym).typedef.typ in [objectdef,recorddef]) then
              begin
                astruct:=tabstractrecorddef(ttypesym(srsym).typedef);
                if (token<>_POINT) then
                  if (potype in [potype_class_constructor,potype_class_destructor]) then
                    sp:=lower(sp)
                  else
                  if (potype=potype_operator)and(optoken=NOTOKEN) then
                    parse_operator_name;
                srsym:=tsym(astruct.symtable.Find(sp));
                if assigned(srsym) then
                 begin
                   if srsym.typ=procsym then
                     aprocsym:=tprocsym(srsym)
                   else
                   if (srsym.typ=typesym) and
                      (ttypesym(srsym).typedef.typ in [objectdef,recorddef]) then
                     begin
                       searchagain:=true;
                       consume(_POINT);
                     end
                   else
                     begin
                       {  we use a different error message for tp7 so it looks more compatible }
                       if (m_fpc in current_settings.modeswitches) then
                         Message1(parser_e_overloaded_no_procedure,srsym.realname)
                       else
                         Message(parser_e_methode_id_expected);
                       { rename the name to an unique name to avoid an
                         error when inserting the symbol in the symtable }
                       orgsp:=orgsp+'$'+tostr(current_filepos.line);
                     end;
                 end
                else
                 begin
                   Message(parser_e_methode_id_expected);
                   { recover by making it a normal procedure instead of method }
                   astruct:=nil;
                 end;
              end
             else
              Message(parser_e_class_id_expected);
           until not searchagain;
         end
        else
         begin
           { check for constructor/destructor/class operators which are not allowed here }
           if (not parse_only) and
              ((potype in [potype_constructor,potype_destructor,
                           potype_class_constructor,potype_class_destructor]) or
               ((potype=potype_operator) and (m_delphi in current_settings.modeswitches))) then
             Message(parser_e_only_methods_allowed);

           repeat
             searchagain:=false;
             current_tokenpos:=procstartfilepos;

             if (potype=potype_operator)and(optoken=NOTOKEN) then
               parse_operator_name;

             srsym:=tsym(symtablestack.top.Find(sp));

             { Also look in the globalsymtable if we didn't found
               the symbol in the localsymtable }
             if not assigned(srsym) and
                not(parse_only) and
                (symtablestack.top=current_module.localsymtable) and
                assigned(current_module.globalsymtable) then
               srsym:=tsym(current_module.globalsymtable.Find(sp));

             { Check if overloaded is a procsym }
             if assigned(srsym) then
               begin
                 if srsym.typ=procsym then
                   aprocsym:=tprocsym(srsym)
                 else
                   begin
                     { when the other symbol is a unit symbol then hide the unit
                       symbol, this is not supported in tp7 }
                     if not(m_tp7 in current_settings.modeswitches) and
                        (srsym.typ=unitsym) then
                      begin
                        HideSym(srsym);
                        searchagain:=true;
                      end
                     else
                      begin
                        {  we use a different error message for tp7 so it looks more compatible }
                        if (m_fpc in current_settings.modeswitches) then
                          Message1(parser_e_overloaded_no_procedure,srsym.realname)
                        else
                          Message1(sym_e_duplicate_id,srsym.realname);
                        { rename the name to an unique name to avoid an
                          error when inserting the symbol in the symtable }
                        orgsp:=orgsp+'$'+tostr(current_filepos.line);
                      end;
                   end;
              end;
           until not searchagain;
         end;

        { test again if assigned, it can be reset to recover }
        if not assigned(aprocsym) then
          begin
            { create a new procsym and set the real filepos }
            current_tokenpos:=procstartfilepos;
            { for operator we have only one procsym for each overloaded
              operation }
            if (potype=potype_operator) then
              begin
                aprocsym:=Tprocsym(symtablestack.top.Find(sp));
                if aprocsym=nil then
                  aprocsym:=tprocsym.create('$'+sp);
              end
            else
            if (potype in [potype_class_constructor,potype_class_destructor]) then
              aprocsym:=tprocsym.create('$'+lower(sp))
            else
              aprocsym:=tprocsym.create(orgsp);
            symtablestack.top.insert(aprocsym);
          end;

        { to get the correct symtablelevel we must ignore ObjectSymtables }
        st:=nil;
        checkstack:=symtablestack.stack;
        while assigned(checkstack) do
          begin
            st:=checkstack^.symtable;
            if st.symtabletype in [staticsymtable,globalsymtable,localsymtable] then
              break;
            checkstack:=checkstack^.next;
          end;
        pd:=tprocdef.create(st.symtablelevel+1);
        pd.struct:=astruct;
        pd.procsym:=aprocsym;
        pd.proctypeoption:=potype;

        { methods inherit df_generic or df_specialization from the objectdef }
        if assigned(pd.struct) and
           (pd.parast.symtablelevel=normal_function_level) then
          begin
            if (df_generic in pd.struct.defoptions) then
              begin
                include(pd.defoptions,df_generic);
                parse_generic:=true;
              end;
            if (df_specialization in pd.struct.defoptions) then
              begin
                include(pd.defoptions,df_specialization);
                { Find corresponding genericdef, we need it later to
                  replay the tokens to generate the body }
                if not assigned(pd.struct.genericdef) then
                  internalerror(200512113);
                genericst:=pd.struct.genericdef.GetSymtable(gs_record);
                if not assigned(genericst) then
                  internalerror(200512114);
                { We are parsing the same objectdef, the def index numbers
                  are the same }
                pd.genericdef:=tstoreddef(genericst.DefList[pd.owner.DefList.IndexOf(pd)]);
                if not assigned(pd.genericdef) or
                   (pd.genericdef.typ<>procdef) then
                  internalerror(200512115);
              end;
          end;

        { methods need to be exported }
        if assigned(astruct) and
           (
            (symtablestack.top.symtabletype in [ObjectSymtable,recordsymtable]) or
            (symtablestack.top.symtablelevel=main_program_level)
           ) then
          include(pd.procoptions,po_global);

        { symbol options that need to be kept per procdef }
        pd.fileinfo:=procstartfilepos;
        pd.visibility:=symtablestack.top.currentvisibility;
        if symtablestack.top.currentlyoptional then
          include(pd.procoptions,po_optional);

        { parse parameters }
        if token=_LKLAMMER then
          begin
            { Add ObjectSymtable to be able to find nested type definitions }
            popclass:=0;
            if assigned(pd.struct) and
               (pd.parast.symtablelevel>=normal_function_level) and
               not(symtablestack.top.symtabletype in [ObjectSymtable,recordsymtable]) then
              begin
                popclass:=push_nested_hierarchy(pd.struct);
                old_current_structdef:=current_structdef;
                old_current_genericdef:=current_genericdef;
                old_current_specializedef:=current_specializedef;
                current_structdef:=pd.struct;
                if assigned(current_structdef) and (df_generic in current_structdef.defoptions) then
                  current_genericdef:=current_structdef;
                if assigned(current_structdef) and (df_specialization in current_structdef.defoptions) then
                  current_specializedef:=current_structdef;
              end;
            { Add parameter symtable }
            if pd.parast.symtabletype<>staticsymtable then
              symtablestack.push(pd.parast);
            parse_parameter_dec(pd);
            if pd.parast.symtabletype<>staticsymtable then
              symtablestack.pop(pd.parast);
            if popclass>0 then
              begin
                current_structdef:=old_current_structdef;
                current_genericdef:=old_current_genericdef;
                current_specializedef:=old_current_specializedef;
                dec(popclass,pop_nested_hierarchy(pd.struct));
                if popclass<>0 then
                  internalerror(201011260); // 11 nov 2010 index 0
              end;
          end;

        parse_generic:=old_parse_generic;
        result:=true;
      end;


    function parse_proc_dec(isclassmethod:boolean;astruct:tabstractrecorddef):tprocdef;
      var
        pd: tprocdef;
        locationstr: string;
        i: integer;
        found: boolean;
        old_block_type: tblock_type;

        procedure read_returndef(pd: tprocdef);
          var
            popclass: integer;
            old_parse_generic: boolean;
            old_current_structdef: tabstractrecorddef;
            old_current_genericdef,
            old_current_specializedef: tstoreddef;
          begin
            old_parse_generic:=parse_generic;
            { Add ObjectSymtable to be able to find generic type definitions }
            popclass:=0;
            if assigned(pd.struct) and
               (pd.parast.symtablelevel>=normal_function_level) and
               not (symtablestack.top.symtabletype in [ObjectSymtable,recordsymtable]) then
              begin
                popclass:=push_nested_hierarchy(pd.struct);
                parse_generic:=(df_generic in pd.struct.defoptions);
                old_current_structdef:=current_structdef;
                old_current_genericdef:=current_genericdef;
                old_current_specializedef:=current_specializedef;
                current_structdef:=pd.struct;
                if assigned(current_structdef) and (df_generic in current_structdef.defoptions) then
                  current_genericdef:=current_structdef;
                if assigned(current_structdef) and (df_specialization in current_structdef.defoptions) then
                  current_specializedef:=current_structdef;
              end;
            single_type(pd.returndef,[stoAllowSpecialization]);

            if is_dispinterface(pd.struct) and not is_automatable(pd.returndef) then
              Message1(type_e_not_automatable,pd.returndef.typename);

            if popclass>0 then
              begin
                current_structdef:=old_current_structdef;
                current_genericdef:=old_current_genericdef;
                current_specializedef:=old_current_specializedef;
                dec(popclass,pop_nested_hierarchy(pd.struct));
                if popclass<>0 then
                  internalerror(201012020);
              end;
            parse_generic:=old_parse_generic;
          end;

      begin
        locationstr:='';
        pd:=nil;
        case token of
          _FUNCTION :
            begin
              consume(_FUNCTION);
              if parse_proc_head(astruct,potype_function,pd) then
                begin
                  { pd=nil when it is a interface mapping }
                  if assigned(pd) then
                    begin
                      if try_to_consume(_COLON) then
                       begin
                         read_returndef(pd);
                         if (target_info.system in [system_m68k_amiga]) then
                          begin
                           if (idtoken=_LOCATION) then
                            begin
                             if po_explicitparaloc in pd.procoptions then
                              begin
                               consume(_LOCATION);
                               locationstr:=cstringpattern;
                               consume(_CSTRING);
                              end
                             else
                              { I guess this needs a new message... (KB) }
                              Message(parser_e_paraloc_all_paras);
                            end
                           else
                            begin
                             if po_explicitparaloc in pd.procoptions then
                              { assign default locationstr, if none specified }
                              { and we've arguments with explicit paraloc }
                              locationstr:='D0';
                            end;
                          end;

                       end
                      else
                       begin
                          if (
                              parse_only and
                              not(is_interface(pd.struct))
                             ) or
                             (m_repeat_forward in current_settings.modeswitches) then
                          begin
                            consume(_COLON);
                            consume_all_until(_SEMICOLON);
                          end;
                       end;
                      if isclassmethod then
                       include(pd.procoptions,po_classmethod);
                    end;
                end
              else
                begin
                  { recover }
                  consume(_COLON);
                  consume_all_until(_SEMICOLON);
                end;
            end;

          _PROCEDURE :
            begin
              consume(_PROCEDURE);
              if parse_proc_head(astruct,potype_procedure,pd) then
                begin
                  { pd=nil when it is an interface mapping }
                  if assigned(pd) then
                    begin
                      pd.returndef:=voidtype;
                      if isclassmethod then
                        include(pd.procoptions,po_classmethod);
                    end;
                end;
            end;

          _CONSTRUCTOR :
            begin
              consume(_CONSTRUCTOR);
              if isclassmethod then
                parse_proc_head(astruct,potype_class_constructor,pd)
              else
                parse_proc_head(astruct,potype_constructor,pd);
              if not isclassmethod and
                 assigned(pd) and
                 assigned(pd.struct) then
                begin
                  { Set return type, class constructors return the
                    created instance, object constructors return boolean }
                  if is_class(pd.struct) or
                     is_record(pd.struct) or
                     is_javaclass(pd.struct) then
                    pd.returndef:=pd.struct
                  else
{$ifdef CPU64bitaddr}
                    pd.returndef:=bool64type;
{$else CPU64bitaddr}
                    pd.returndef:=bool32type;
{$endif CPU64bitaddr}
                end
              else
                pd.returndef:=voidtype;
            end;

          _DESTRUCTOR :
            begin
              consume(_DESTRUCTOR);
              if isclassmethod then
                parse_proc_head(astruct,potype_class_destructor,pd)
              else
                parse_proc_head(astruct,potype_destructor,pd);
              if assigned(pd) then
                pd.returndef:=voidtype;
            end;
        else
          if (token=_OPERATOR) or
             (isclassmethod and (idtoken=_OPERATOR)) then
            begin
              { we need to set the block type to bt_body, so that operator names
                like ">", "=>" or "<>" are parsed correctly instead of e.g.
                _LSHARPBRACKET and _RSHARPBRACKET for "<>" }
              old_block_type:=block_type;
              block_type:=bt_body;
              consume(_OPERATOR);
              parse_proc_head(astruct,potype_operator,pd);
              block_type:=old_block_type;
              if assigned(pd) then
                begin
                  { operators always need to be searched in all units }
                  include(pd.procoptions,po_overload);
                  if pd.parast.symtablelevel>normal_function_level then
                    Message(parser_e_no_local_operator);
                  if isclassmethod then
                    include(pd.procoptions,po_classmethod);
                  if token<>_ID then
                    begin
                       if not(m_result in current_settings.modeswitches) then
                         consume(_ID);
                    end
                  else
                    begin
                      pd.resultname:=stringdup(orgpattern);
                      consume(_ID);
                    end;
                  if not try_to_consume(_COLON) then
                    begin
                      consume(_COLON);
                      pd.returndef:=generrordef;
                      consume_all_until(_SEMICOLON);
                    end
                  else
                   begin
                     read_returndef(pd);
                     { check that class operators have either return type of structure or }
                     { at least one argument of that type                                 }
                     if (po_classmethod in pd.procoptions) and
                        (pd.returndef <> pd.struct) then
                       begin
                         found:=false;
                         for i := 0 to pd.parast.SymList.Count - 1 do
                           if tparavarsym(pd.parast.SymList[i]).vardef=pd.struct then
                             begin
                               found:=true;
                               break;
                             end;
                         if not found then
                           if assigned(pd.struct) then
                             Message1(parser_e_at_least_one_argument_must_be_of_type,pd.struct.RttiName)
                           else
                             MessagePos(pd.fileinfo,type_e_type_id_expected);
                       end;
                     if (optoken in [_EQ,_NE,_GT,_LT,_GTE,_LTE,_OP_IN]) and
                        ((pd.returndef.typ<>orddef) or
                         (torddef(pd.returndef).ordtype<>pasbool8)) then
                        Message(parser_e_comparative_operator_return_boolean);
                     if (optoken in [_ASSIGNMENT,_OP_EXPLICIT]) and
                        equal_defs(pd.returndef,tparavarsym(pd.parast.SymList[0]).vardef) and
                        (pd.returndef.typ<>undefineddef) and (tparavarsym(pd.parast.SymList[0]).vardef.typ<>undefineddef) then
                       message(parser_e_no_such_assignment)
                     else if not isoperatoracceptable(pd,optoken) then
                       Message(parser_e_overload_impossible);
                   end;
                end
              else
                begin
                  { recover }
                  try_to_consume(_ID);
                  consume(_COLON);
                  consume_all_until(_SEMICOLON);
                end;
            end;
        end;
        { file types can't be function results }
        if assigned(pd) and
           (pd.returndef.typ=filedef) then
          message(parser_e_illegal_function_result);
        { support procedure proc stdcall export; }
        if not(check_proc_directive(false)) then
          begin
            if (token=_COLON) then
              begin
                message(parser_e_field_not_allowed_here);
                consume_all_until(_SEMICOLON);
              end;
            consume(_SEMICOLON);
          end;
        result:=pd;

        if locationstr<>'' then
         begin
           if not(paramanager.parsefuncretloc(pd,upper(locationstr))) then
             { I guess this needs a new message... (KB) }
             message(parser_e_illegal_explicit_paraloc);
         end;
      end;


    function parse_record_method_dec(astruct: tabstractrecorddef; is_classdef: boolean): tprocdef;
      var
        oldparse_only: boolean;
      begin
        oldparse_only:=parse_only;
        parse_only:=true;
        result:=parse_proc_dec(is_classdef,astruct);

        { this is for error recovery as well as forward }
        { interface mappings, i.e. mapping to a method  }
        { which isn't declared yet                      }
        if assigned(result) then
          begin
            parse_record_proc_directives(result);

            { since records have no inheritance, don't allow non-static
              class methods. Selphi does the same. }
            if (result.proctypeoption<>potype_operator) and
               is_classdef and
               not (po_staticmethod in result.procoptions) then
              MessagePos(result.fileinfo, parser_e_class_methods_only_static_in_records);

            // we can't add hidden params here because record is not yet defined
            // and therefore record size which has influence on paramter passing rules may change too
            // look at record_dec to see where calling conventions are applied (issue #0021044)
            handle_calling_convention(result,[hcc_check]);

            { add definition to procsym }
            proc_add_definition(result);
          end;

        maybe_parse_hint_directives(result);

        parse_only:=oldparse_only;
      end;


    procedure insert_record_hidden_paras(astruct: trecorddef);
      var
        pd: tdef;
        i: longint;
        oldpos : tfileposinfo;
        oldparse_only: boolean;
      begin
        // handle calling conventions of record methods
        oldpos:=current_filepos;
        oldparse_only:=parse_only;
        parse_only:=true;
        { don't keep track of procdefs in a separate list, because the
          compiler may add additional procdefs (e.g. property wrappers for
          the jvm backend) }
        for i := 0 to astruct.symtable.deflist.count - 1 do
          begin
            pd:=tdef(astruct.symtable.deflist[i]);
            if pd.typ<>procdef then
              continue;
            current_filepos:=tprocdef(pd).fileinfo;
            handle_calling_convention(tprocdef(pd),[hcc_insert_hidden_paras]);
          end;
        parse_only:=oldparse_only;
        current_filepos:=oldpos;
      end;


{****************************************************************************
                        Procedure directive handlers
****************************************************************************}

procedure pd_far(pd:tabstractprocdef);
begin
  Message1(parser_w_proc_directive_ignored,'FAR');
end;

procedure pd_near(pd:tabstractprocdef);
begin
  Message1(parser_w_proc_directive_ignored,'NEAR');
end;

procedure pd_export(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200304264);
  if assigned(tprocdef(pd).struct) then
    Message(parser_e_methods_dont_be_export);
  if pd.parast.symtablelevel>normal_function_level then
    Message(parser_e_dont_nest_export);
end;

procedure pd_forward(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200304265);
  tprocdef(pd).forwarddef:=true;
end;


procedure pd_alias(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200304266);
  consume(_COLON);
  tprocdef(pd).aliasnames.insert(get_stringconst);
  include(pd.procoptions,po_has_public_name);
end;


procedure pd_public(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200304266);
  if try_to_consume(_NAME) then
    begin
      tprocdef(pd).aliasnames.insert(get_stringconst);
      include(pd.procoptions,po_has_public_name);
    end;
end;


procedure pd_asmname(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200304267);
  if token=_CCHAR then
    begin
      tprocdef(pd).aliasnames.insert(target_info.Cprefix+pattern);
      consume(_CCHAR)
    end
  else
    begin
      tprocdef(pd).aliasnames.insert(target_info.Cprefix+cstringpattern);
      consume(_CSTRING);
    end;
  { we don't need anything else }
  tprocdef(pd).forwarddef:=false;
end;


procedure pd_internconst(pd:tabstractprocdef);

var v:Tconstexprint;

begin
  if pd.typ<>procdef then
    internalerror(200304268);
  consume(_COLON);
  v:=get_intconst;
  if (v<int64(low(longint))) or (v>int64(high(longint))) then
    message3(type_e_range_check_error_bounds,tostr(v),tostr(low(longint)),tostr(high(longint)))
  else
    Tprocdef(pd).extnumber:=longint(v.svalue);
end;


procedure pd_internproc(pd:tabstractprocdef);

var v:Tconstexprint;

begin
  if pd.typ<>procdef then
    internalerror(200304268);
  consume(_COLON);
  v:=get_intconst;
  if (v<int64(low(longint))) or (v>int64(high(longint))) then
    message3(type_e_range_check_error_bounds,tostr(v),tostr(low(longint)),tostr(high(longint)))
  else
    Tprocdef(pd).extnumber:=longint(v.svalue);
  { the proc is defined }
  tprocdef(pd).forwarddef:=false;
end;

procedure pd_interrupt(pd:tabstractprocdef);

{$ifdef FPC_HAS_SYSTEMS_INTERRUPT_TABLE}
var v: Tconstexprint;
{$endif FPC_HAS_SYSTEMS_INTERRUPT_TABLE}

begin
  if pd.parast.symtablelevel>normal_function_level then
    Message(parser_e_dont_nest_interrupt);

{$ifdef FPC_HAS_SYSTEMS_INTERRUPT_TABLE}
  if target_info.system in systems_interrupt_table then
    begin
      if token<>_SEMICOLON then
        begin
          pd.proccalloption:=pocall_interrupt;
          v:=get_intconst;
          Tprocdef(pd).interruptvector:=v.uvalue;
        end;
    end;
{$endif FPC_HAS_SYSTEMS_INTERRUPT_TABLE}
end;

procedure pd_abstract(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200304269);
  if is_objectpascal_helper(tprocdef(pd).struct) then
    Message1(parser_e_not_allowed_in_helper, arraytokeninfo[_ABSTRACT].str);
  if assigned(tprocdef(pd).struct) and
    (oo_is_sealed in tprocdef(pd).struct.objectoptions) then
    Message(parser_e_sealed_class_cannot_have_abstract_methods)
  else if (po_virtualmethod in pd.procoptions) then
    begin
      include(pd.procoptions,po_abstractmethod);
      { one more abstract method }
      inc(tobjectdef(pd.owner.defowner).abstractcnt);
    end
  else
    Message(parser_e_only_virtual_methods_abstract);
  { the method is defined }
  tprocdef(pd).forwarddef:=false;
end;

procedure pd_final(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200910170);
  if is_objectpascal_helper(tprocdef(pd).struct) and
      (m_objfpc in current_settings.modeswitches) then
    Message1(parser_e_not_allowed_in_helper, arraytokeninfo[_FINAL].str);
  if (po_virtualmethod in pd.procoptions) or
     (is_javaclass(tprocdef(pd).struct) and
      (po_classmethod in pd.procoptions)) then
    include(pd.procoptions,po_finalmethod)
  else
    Message(parser_e_only_virtual_methods_final);
end;

procedure pd_enumerator(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200910250);
  if (token = _ID) then
  begin
    if pattern='MOVENEXT' then
    begin
      if oo_has_enumerator_movenext in tprocdef(pd).struct.objectoptions then
        message(parser_e_only_one_enumerator_movenext);
      pd.calcparas;
      if (pd.proctypeoption = potype_function) and is_boolean(pd.returndef) and
         (pd.minparacount = 0) then
      begin
        include(tprocdef(pd).struct.objectoptions, oo_has_enumerator_movenext);
        include(pd.procoptions,po_enumerator_movenext);
      end
      else
        Message(parser_e_enumerator_movenext_is_not_valid)
    end
    else
      Message1(parser_e_invalid_enumerator_identifier, pattern);
    consume(token);
  end
  else
    Message(parser_e_enumerator_identifier_required);
end;

procedure pd_virtual(pd:tabstractprocdef);
{$ifdef WITHDMT}
var
  pt : tnode;
{$endif WITHDMT}
begin
  if (not assigned(pd.owner.defowner) or
      not is_java_class_or_interface(tdef(pd.owner.defowner))) and
     (po_external in pd.procoptions) then
    Message1(parser_e_proc_dir_conflict,'EXTERNAL');

  if pd.typ<>procdef then
    internalerror(2003042610);
  if (pd.proctypeoption=potype_constructor) and
     is_object(tprocdef(pd).struct) then
    Message(parser_e_constructor_cannot_be_not_virtual);
  if is_objectpascal_helper(tprocdef(pd).struct) and
      (m_objfpc in current_settings.modeswitches) then
    Message1(parser_e_not_allowed_in_helper, arraytokeninfo[_VIRTUAL].str);
{$ifdef WITHDMT}
  if is_object(tprocdef(pd).struct) and
     (token<>_SEMICOLON) then
    begin
       { any type of parameter is allowed here! }
       pt:=comp_expr(true);
       if is_constintnode(pt) then
         begin
           include(pd.procoptions,po_msgint);
           pd.messageinf.i:=pt.value;
         end
       else
         Message(parser_e_ill_msg_expr);
       disposetree(pt);
    end;
{$endif WITHDMT}
end;


procedure pd_dispid(pd:tabstractprocdef);

var pt:Tnode;

begin
  if pd.typ<>procdef then
    internalerror(200604301);
  pt:=comp_expr(true,false);
  if is_constintnode(pt) then
    if (Tordconstnode(pt).value<int64(low(longint))) or (Tordconstnode(pt).value>int64(high(longint))) then
      message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(longint)),tostr(high(longint)))
    else
      Tprocdef(pd).dispid:=Tordconstnode(pt).value.svalue
  else
    message(parser_e_dispid_must_be_ord_const);
  pt.free;
end;


procedure pd_static(pd:tabstractprocdef);
begin
  if pd.typ=procdef then
    include(tprocdef(pd).procsym.symoptions,sp_static);
  include(pd.procoptions,po_staticmethod);
end;

procedure pd_override(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(2003042611);
  if is_objectpascal_helper(tprocdef(pd).struct) then
    begin
      if m_objfpc in current_settings.modeswitches then
        Message1(parser_e_not_allowed_in_helper, arraytokeninfo[_OVERRIDE].str)
    end
  else if not(is_class_or_interface_or_objc_or_java(tprocdef(pd).struct)) then
    Message(parser_e_no_object_override)
  else if is_objccategory(tprocdef(pd).struct) then
    Message(parser_e_no_category_override)
  else if (po_external in pd.procoptions) and
          not is_objc_class_or_protocol(tprocdef(pd).struct) and
          not is_cppclass(tprocdef(pd).struct) and
          not is_java_class_or_interface(tprocdef(pd).struct) then
    Message1(parser_e_proc_dir_conflict,'OVERRIDE');
end;

procedure pd_overload(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(2003042612);
  include(tprocdef(pd).procsym.symoptions,sp_has_overloaded);
end;

procedure pd_message(pd:tabstractprocdef);
var
  pt : tnode;
  paracnt : longint;
begin
  if pd.typ<>procdef then
    internalerror(2003042613);
  if is_objectpascal_helper(tprocdef(pd).struct) then
    begin
      if m_objfpc in current_settings.modeswitches then
        Message1(parser_e_not_allowed_in_helper, arraytokeninfo[_MESSAGE].str);
    end
  else
    if not is_class(tprocdef(pd).struct) and
       not is_objc_class_or_protocol(tprocdef(pd).struct) then
      Message(parser_e_msg_only_for_classes);
  if ([po_msgstr,po_msgint]*pd.procoptions)<>[] then
    Message(parser_e_multiple_messages);
  { check parameter type }
  if not is_objc_class_or_protocol(tprocdef(pd).struct) then
    begin
      if po_external in pd.procoptions then
        Message1(parser_e_proc_dir_conflict,'MESSAGE');
      paracnt:=0;
      pd.parast.SymList.ForEachCall(@check_msg_para,@paracnt);
      if paracnt<>1 then
        Message(parser_e_ill_msg_param);
    end;
  pt:=comp_expr(true,false);
  { message is 1-character long }
  if is_constcharnode(pt) then
    begin
      include(pd.procoptions,po_msgstr);
      tprocdef(pd).messageinf.str:=stringdup(chr(byte(tordconstnode(pt).value.uvalue and $FF)));
    end
  else if pt.nodetype=stringconstn then
    begin
      include(pd.procoptions,po_msgstr);
      if (tstringconstnode(pt).len>255) then
        Message(parser_e_message_string_too_long);
      tprocdef(pd).messageinf.str:=stringdup(tstringconstnode(pt).value_str);
    end
  else
   if is_constintnode(pt) and
      (is_class(tprocdef(pd).struct) or
      is_objectpascal_helper(tprocdef(pd).struct)) then
    begin
      include(pd.procoptions,po_msgint);
      if (Tordconstnode(pt).value<int64(low(Tprocdef(pd).messageinf.i))) or
         (Tordconstnode(pt).value>int64(high(Tprocdef(pd).messageinf.i))) then
        message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(Tprocdef(pd).messageinf.i)),tostr(high(Tprocdef(pd).messageinf.i)))
      else
        Tprocdef(pd).messageinf.i:=tordconstnode(pt).value.svalue;
    end
  else
    Message(parser_e_ill_msg_expr);
  { check whether the selector name is valid in case of Objective-C }
  if (po_msgstr in pd.procoptions) and
     is_objc_class_or_protocol(tprocdef(pd).struct) and
     not objcvalidselectorname(@tprocdef(pd).messageinf.str^[1],length(tprocdef(pd).messageinf.str^)) then
    Message1(type_e_invalid_objc_selector_name,tprocdef(pd).messageinf.str^);
  pt.free;
end;


procedure pd_reintroduce(pd:tabstractprocdef);
begin
  if pd.typ<>procdef then
    internalerror(200401211);
  if is_objectpascal_helper(tprocdef(pd).struct) then
    begin
      if m_objfpc in current_settings.modeswitches then
        Message1(parser_e_not_allowed_in_helper, arraytokeninfo[_REINTRODUCE].str);
    end
  else
    if not(is_class_or_interface_or_object(tprocdef(pd).struct)) and
       not(is_objccategory(tprocdef(pd).struct)) and
       not(is_javaclass(tprocdef(pd).struct)) then
      Message(parser_e_no_object_reintroduce);
end;


procedure pd_syscall(pd:tabstractprocdef);
{$if defined(powerpc) or defined(m68k)}
var
  vs  : tparavarsym;
  sym : tsym;
  symtable : TSymtable;
  v: Tconstexprint;
{$endif defined(powerpc) or defined(m68k)}
begin
  if (pd.typ<>procdef) and (target_info.system <> system_powerpc_amiga) then
    internalerror(2003042614);
  tprocdef(pd).forwarddef:=false;
{$ifdef m68k}
   if target_info.system in [system_m68k_amiga] then
    begin
      include(pd.procoptions,po_syscall_legacy);

      if consume_sym(sym,symtable) then
        begin
          if (sym.typ=staticvarsym) and
             (
              (tabstractvarsym(sym).vardef.typ=pointerdef) or
              is_32bitint(tabstractvarsym(sym).vardef)
             ) then
            begin
              tprocdef(pd).libsym:=sym;
              if po_syscall_legacy in tprocdef(pd).procoptions then
                begin
                  vs:=tparavarsym.create('$syscalllib',paranr_syscall_legacy,vs_value,tabstractvarsym(sym).vardef,[vo_is_syscall_lib,vo_is_hidden_para,vo_has_explicit_paraloc]);
                  paramanager.parseparaloc(vs,'A6');
                  pd.parast.insert(vs);
                end
            end
          else
            Message(parser_e_32bitint_or_pointer_variable_expected);
        end;
      (paramanager as tm68kparamanager).create_funcretloc_info(pd,calleeside);
      (paramanager as tm68kparamanager).create_funcretloc_info(pd,callerside);

      v:=get_intconst;
      if (v<low(Tprocdef(pd).extnumber)) or (v>high(Tprocdef(pd).extnumber)) then
        message3(type_e_range_check_error_bounds,tostr(v),tostr(low(Tprocdef(pd).extnumber)),tostr(high(Tprocdef(pd).extnumber)))
      else
        Tprocdef(pd).extnumber:=v.uvalue;
    end;
{$endif m68k}
{$ifdef powerpc}
   if target_info.system = system_powerpc_amiga then
    begin
      include(pd.procoptions,po_syscall_basesysv);

      if consume_sym(sym,symtable) then
        begin
          if (sym.typ=staticvarsym) and
             (
              (tabstractvarsym(sym).vardef.typ=pointerdef) or
              is_32bitint(tabstractvarsym(sym).vardef)
             ) then
            begin
              tprocdef(pd).libsym:=sym;
              vs:=tparavarsym.create('$syscalllib',paranr_syscall_basesysv,vs_value,tabstractvarsym(sym).vardef,[vo_is_syscall_lib,vo_is_hidden_para]);
              pd.parast.insert(vs);
            end
          else
            Message(parser_e_32bitint_or_pointer_variable_expected);
        end;

      (paramanager as tppcparamanager).create_funcretloc_info(pd,calleeside);
      (paramanager as tppcparamanager).create_funcretloc_info(pd,callerside);

      v:=get_intconst;
      if (v<low(Tprocdef(pd).extnumber)) or (v>high(Tprocdef(pd).extnumber)) then
        message(parser_e_range_check_error)
      else
        Tprocdef(pd).extnumber:=v.uvalue;
    end else

   if target_info.system = system_powerpc_morphos then
    begin
      if idtoken=_LEGACY then
        begin
          consume(_LEGACY);
          include(pd.procoptions,po_syscall_legacy);
        end
      else if idtoken=_SYSV then
        begin
          consume(_SYSV);
          include(pd.procoptions,po_syscall_sysv);
        end
      else if idtoken=_BASESYSV then
        begin
          consume(_BASESYSV);
          include(pd.procoptions,po_syscall_basesysv);
        end
      else if idtoken=_SYSVBASE then
        begin
          consume(_SYSVBASE);
          include(pd.procoptions,po_syscall_sysvbase);
        end
      else if idtoken=_R12BASE then
        begin
          consume(_R12BASE);
          include(pd.procoptions,po_syscall_r12base);
        end
      else
        if syscall_convention='LEGACY' then
          include(pd.procoptions,po_syscall_legacy)
        else if syscall_convention='SYSV' then
          include(pd.procoptions,po_syscall_sysv)
        else if syscall_convention='BASESYSV' then
          include(pd.procoptions,po_syscall_basesysv)
        else if syscall_convention='SYSVBASE' then
          include(pd.procoptions,po_syscall_sysvbase)
        else if syscall_convention='R12BASE' then
          include(pd.procoptions,po_syscall_r12base)
        else
          internalerror(2005010404);

      if consume_sym(sym,symtable) then
        begin
          if (sym.typ=staticvarsym) and
             (
              (tabstractvarsym(sym).vardef.typ=pointerdef) or
              is_32bitint(tabstractvarsym(sym).vardef)
             ) then
            begin
              tprocdef(pd).libsym:=sym;
              if po_syscall_legacy in tprocdef(pd).procoptions then
                begin
                  vs:=tparavarsym.create('$syscalllib',paranr_syscall_legacy,vs_value,tabstractvarsym(sym).vardef,[vo_is_syscall_lib,vo_is_hidden_para,vo_has_explicit_paraloc]);
                  paramanager.parseparaloc(vs,'A6');
                  pd.parast.insert(vs);
                end
              else if po_syscall_sysv in tprocdef(pd).procoptions then
                begin
                  { Nothing to be done for sysv here for now, but this might change }
                end
              else if po_syscall_basesysv in tprocdef(pd).procoptions then
                begin
                  vs:=tparavarsym.create('$syscalllib',paranr_syscall_basesysv,vs_value,tabstractvarsym(sym).vardef,[vo_is_syscall_lib,vo_is_hidden_para]);
                  pd.parast.insert(vs);
                end
              else if po_syscall_sysvbase in tprocdef(pd).procoptions then
                begin
                  vs:=tparavarsym.create('$syscalllib',paranr_syscall_sysvbase,vs_value,tabstractvarsym(sym).vardef,[vo_is_syscall_lib,vo_is_hidden_para]);
                  pd.parast.insert(vs);
                end
              else if po_syscall_r12base in tprocdef(pd).procoptions then
                begin
                  vs:=tparavarsym.create('$syscalllib',paranr_syscall_r12base,vs_value,tabstractvarsym(sym).vardef,[vo_is_syscall_lib,vo_is_hidden_para,vo_has_explicit_paraloc]);
                  paramanager.parseparaloc(vs,'R12');
                  pd.parast.insert(vs);
                end
              else
                internalerror(2005010501);
            end
          else
            Message(parser_e_32bitint_or_pointer_variable_expected);
        end;
      (paramanager as tppcparamanager).create_funcretloc_info(pd,calleeside);
      (paramanager as tppcparamanager).create_funcretloc_info(pd,callerside);

      v:=get_intconst;
      if (v<low(Tprocdef(pd).extnumber)) or (v>high(Tprocdef(pd).extnumber)) then
        message(parser_e_range_check_error)
      else
        Tprocdef(pd).extnumber:=v.uvalue;
    end;
{$endif powerpc}
end;


procedure pd_external(pd:tabstractprocdef);
{
  If import_dll=nil the procedure is assumed to be in another
  object file. In that object file it should have the name to
  which import_name is pointing to. Otherwise, the procedure is
  assumed to be in the DLL to which import_dll is pointing to. In
  that case either import_nr<>0 or import_name<>nil is true, so
  the procedure is either imported by number or by name. (DM)
}
var
  hs : string;
  v:Tconstexprint;
  is_java_external: boolean;
begin
  if pd.typ<>procdef then
    internalerror(2003042615);
  { Allow specifying a separate external name for methods in external Java
    because its identifier naming constraints are laxer than FPC's
    (e.g., case sensitive).
    Limitation: only allows specifying the symbol name and not the package name,
    and only for external classes/interfaces }
  is_java_external:=
    (pd.typ=procdef) and
    is_java_class_or_interface(tdef(pd.owner.defowner)) and
    (oo_is_external in tobjectdef(pd.owner.defowner).objectoptions);
  with tprocdef(pd) do
    begin
      forwarddef:=false;
      { forbid local external procedures }
      if parast.symtablelevel>normal_function_level then
        Message(parser_e_no_local_proc_external);
      { If the procedure should be imported from a DLL, a constant string follows.
        This isn't really correct, an contant string expression follows
        so we check if an semicolon follows, else a string constant have to
        follow (FK) }
      if not is_java_external and
         not(token=_SEMICOLON) and not(idtoken=_NAME) then
        begin
          { Always add library prefix and suffix to create an uniform name }
          hs:=get_stringconst;
          if ExtractFileExt(hs)='' then
            hs:=ChangeFileExt(hs,target_info.sharedlibext);
          if Copy(hs,1,length(target_info.sharedlibprefix))<>target_info.sharedlibprefix then
            hs:=target_info.sharedlibprefix+hs;
          { the JVM expects java/lang/Object rather than java.lang.Object }
          if target_info.system in systems_jvm then
            Replace(hs,'.','/');
          import_dll:=stringdup(hs);
          include(procoptions,po_has_importdll);
          if (idtoken=_NAME) then
           begin
             consume(_NAME);
             import_name:=stringdup(get_stringconst);
             include(procoptions,po_has_importname);
             if import_name^='' then
               message(parser_e_empty_import_name);
           end;
          if (idtoken=_INDEX) then
           begin
             {After the word index follows the index number in the DLL.}
             consume(_INDEX);
             v:=get_intconst;
             if (v<int64(low(import_nr))) or (v>int64(high(import_nr))) then
               message(parser_e_range_check_error)
             else
               import_nr:=longint(v.svalue);
           end;
          { default is to used the realname of the procedure }
          if (import_nr=0) and not assigned(import_name) then
            begin
              import_name:=stringdup(procsym.realname);
              include(procoptions,po_has_importname);
            end;
        end
      else
        begin
          if (idtoken=_NAME) or
             is_java_external then
           begin
             consume(_NAME);
             import_name:=stringdup(get_stringconst);
             include(procoptions,po_has_importname);
             if import_name^='' then
               message(parser_e_empty_import_name);
           end;
        end;
    end;
end;


procedure pd_weakexternal(pd:tabstractprocdef);
begin
  if not(target_info.system in systems_weak_linking) then
    message(parser_e_weak_external_not_supported)
  else
    pd_external(pd);
end;


type
   pd_handler=procedure(pd:tabstractprocdef);
   proc_dir_rec=record
     idtok     : ttoken;
     pd_flags  : tpdflags;
     handler   : pd_handler;
     pocall    : tproccalloption;
     pooption  : tprocoptions;
     mutexclpocall : tproccalloptions;
     mutexclpotype : tproctypeoptions;
     mutexclpo     : tprocoptions;
   end;
const
  {Should contain the number of procedure directives we support.}
  num_proc_directives=43;
  proc_direcdata:array[1..num_proc_directives] of proc_dir_rec=
   (
    (
      idtok:_ABSTRACT;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_notrecord,pd_javaclass];
      handler  : @pd_abstract;
      pocall   : pocall_none;
      pooption : [po_abstractmethod];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_exports,po_interrupt,po_inline]
    ),(
      idtok:_ALIAS;
      pd_flags : [pd_implemen,pd_body,pd_notobjintf];
      handler  : @pd_alias;
      pocall   : pocall_none;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [];
      mutexclpo     : [po_external,po_inline]
    ),(
      idtok:_ASMNAME;
      pd_flags : [pd_interface,pd_implemen,pd_notobjintf];
      handler  : @pd_asmname;
      pocall   : pocall_cdecl;
      pooption : [po_external];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_external,po_inline]
    ),(
      idtok:_ASSEMBLER;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_notobjintf];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_assembler];
      mutexclpocall : [];
      mutexclpotype : [];
      mutexclpo     : [po_external]
    ),(
      idtok:_C; {same as cdecl for mode mac}
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_cdecl;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_assembler,po_external]
    ),(
      idtok:_CDECL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_cdecl;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_assembler,po_external]
    ),(
      idtok:_DISPID;
      pd_flags : [pd_dispinterface];
      handler  : @pd_dispid;
      pocall   : pocall_none;
      pooption : [po_dispid];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [potype_constructor,potype_destructor,potype_operator,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_interrupt,po_external,po_inline]
    ),(
      idtok:_DYNAMIC;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_notrecord];
      handler  : @pd_virtual;
      pocall   : pocall_none;
      pooption : [po_virtualmethod];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_exports,po_interrupt,po_external,po_overridingmethod,po_inline]
    ),(
      idtok:_EXPORT;
      pd_flags : [pd_body,pd_interface,pd_implemen,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_export;
      pocall   : pocall_none;
      pooption : [po_exports,po_global];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external,po_interrupt,po_inline]
    ),(
      idtok:_EXTERNAL;
      pd_flags : [pd_implemen,pd_interface,pd_notobject,pd_notobjintf,pd_cppobject,pd_notrecord,pd_nothelper,pd_javaclass,pd_intfjava];
      handler  : @pd_external;
      pocall   : pocall_none;
      pooption : [po_external];
      mutexclpocall : [pocall_syscall];
      { allowed for external cpp classes }
      mutexclpotype : [{potype_constructor,potype_destructor}potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_public,po_exports,po_interrupt,po_assembler,po_inline]
    ),(
      idtok:_FAR;
      pd_flags : [pd_implemen,pd_body,pd_interface,pd_procvar,pd_notobject,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_far;
      pocall   : pocall_none;
      pooption : [];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_inline]
    ),(
      idtok:_FAR16;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar,pd_notobject,pd_notrecord,pd_nothelper];
      handler  : nil;
      pocall   : pocall_far16;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [];
      mutexclpo     : [po_external]
    ),(
      idtok:_FINAL;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_notrecord,pd_javaclass];
      handler  : @pd_final;
      pocall   : pocall_none;
      pooption : [po_finalmethod];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_exports,po_interrupt,po_inline]
    ),(
      idtok:_FORWARD;
      pd_flags : [pd_implemen,pd_notobject,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_forward;
      pocall   : pocall_none;
      pooption : [];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_external,po_inline]
    ),(
      idtok:_OLDFPCCALL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_oldfpccall;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [];
      mutexclpo     : []
    ),(
      idtok:_INLINE;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_notobjintf];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_inline];
      mutexclpocall : [pocall_safecall];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_exports,po_external,po_interrupt,po_virtualmethod,po_iocheck]
    ),(
      idtok:_INTERNCONST;
      pd_flags : [pd_interface,pd_body,pd_notobject,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_internconst;
      pocall   : pocall_none;
      pooption : [po_internconst];
      mutexclpocall : [];
      mutexclpotype : [potype_operator];
      mutexclpo     : []
    ),(
      idtok:_INTERNPROC;
      pd_flags : [pd_interface,pd_notobject,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_internproc;
      pocall   : pocall_internproc;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_operator,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_exports,po_external,po_interrupt,po_assembler,po_iocheck,po_virtualmethod]
    ),(
      idtok:_INTERRUPT;
      pd_flags : [pd_implemen,pd_body,pd_notobject,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_interrupt;
{$ifdef FPC_HAS_SYSTEMS_INTERRUPT_TABLE}
      pocall   : pocall_interrupt;
{$else FPC_HAS_SYSTEMS_INTERRUPT_TABLE}
      pocall   : pocall_oldfpccall;
{$endif FPC_HAS_SYSTEMS_INTERRUPT_TABLE}      
      pooption : [po_interrupt];
      mutexclpocall : [pocall_internproc,pocall_cdecl,pocall_cppdecl,pocall_stdcall,
                       pocall_pascal,pocall_far16,pocall_oldfpccall];
      mutexclpotype : [potype_constructor,potype_destructor,potype_operator,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external,po_inline]
    ),(
      idtok:_IOCHECK;
      pd_flags : [pd_implemen,pd_body,pd_notobjintf];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_iocheck];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_external]
    ),(
      idtok:_LOCAL;
      pd_flags : [pd_implemen,pd_body];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_kylixlocal];
      mutexclpocall : [pocall_internproc,pocall_far16];
      mutexclpotype : [];
      mutexclpo     : [po_external,po_exports]
    ),(
      idtok:_MESSAGE;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_objcclass,pd_objcprot,pd_notrecord];
      handler  : @pd_message;
      pocall   : pocall_none;
      pooption : []; { can be po_msgstr or po_msgint }
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [potype_constructor,potype_destructor,potype_operator,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_interrupt,po_inline]
    ),(
      idtok:_MWPASCAL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_mwpascal;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [];
      mutexclpo     : []
    ),(
      idtok:_NEAR;
      pd_flags : [pd_implemen,pd_body,pd_procvar,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_near;
      pocall   : pocall_none;
      pooption : [];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : []
    ),(
      idtok:_NOSTACKFRAME;
      pd_flags : [pd_implemen,pd_body,pd_procvar,pd_notobjintf];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_nostackframe];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : []
    ),(
      idtok:_OVERLOAD;
      pd_flags : [pd_implemen,pd_interface,pd_body,pd_javaclass,pd_intfjava];
      handler  : @pd_overload;
      pocall   : pocall_none;
      pooption : [po_overload];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : []
    ),(
      idtok:_OVERRIDE;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_objcclass,pd_javaclass,pd_intfjava,pd_notrecord];
      handler  : @pd_override;
      pocall   : pocall_none;
      pooption : [po_overridingmethod,po_virtualmethod];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_exports,po_interrupt,po_virtualmethod,po_inline]
    ),(
      idtok:_PASCAL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_pascal;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external]
    ),(
      idtok:_PUBLIC;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_notobject,pd_notobjintf,pd_notrecord,pd_nothelper];
      handler  : @pd_public;
      pocall   : pocall_none;
      pooption : [po_public,po_global];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_external,po_inline]
    ),(
      idtok:_REGISTER;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_register;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external]
    ),(
      idtok:_REINTRODUCE;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_objcclass,pd_notrecord,pd_javaclass];
      handler  : @pd_reintroduce;
      pocall   : pocall_none;
      pooption : [po_reintroduce];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_external,po_interrupt,po_exports,po_overridingmethod,po_inline]
    ),(
      idtok:_SAFECALL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_safecall;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external]
    ),(
      idtok:_SOFTFLOAT;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_softfloat;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      { it's available with po_external because the libgcc floating point routines on the arm
        uses this calling convention }
      mutexclpo     : []
    ),(
      idtok:_STATIC;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_object,pd_record,pd_javaclass,pd_notobjintf];
      handler  : @pd_static;
      pocall   : pocall_none;
      pooption : [po_staticmethod];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_interrupt,po_exports]
    ),(
      idtok:_STDCALL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_stdcall;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external]
    ),(
      idtok:_SYSCALL;
      { Different kind of syscalls are valid for AOS68k, AOSPPC and MOS. }
      { FIX ME!!! MorphOS/AOS68k pd_flags should be:
        pd_interface, pd_implemen, pd_notobject, pd_notobjintf (KB) }
      pd_flags : [pd_interface,pd_implemen,pd_procvar];
      handler  : @pd_syscall;
      pocall   : pocall_syscall;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_external,po_assembler,po_interrupt,po_exports]
    ),(
      idtok:_VIRTUAL;
      pd_flags : [pd_interface,pd_object,pd_notobjintf,pd_notrecord,pd_javaclass];
      handler  : @pd_virtual;
      pocall   : pocall_none;
      pooption : [po_virtualmethod];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_interrupt,po_exports,po_overridingmethod,po_inline]
    ),(
      idtok:_CPPDECL;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_procvar];
      handler  : nil;
      pocall   : pocall_cppdecl;
      pooption : [];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_assembler,po_external,po_virtualmethod]
    ),(
      idtok:_VARARGS;
      pd_flags : [pd_interface,pd_implemen,pd_procvar,pd_objcclass,pd_objcprot];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_varargs];
      mutexclpocall : [pocall_internproc,pocall_stdcall,pocall_register,
                       pocall_far16,pocall_oldfpccall,pocall_mwpascal];
      mutexclpotype : [];
      mutexclpo     : [po_assembler,po_interrupt,po_inline]
    ),(
      idtok:_COMPILERPROC;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_notobjintf];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_compilerproc];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_interrupt]
    ),(
      idtok:_WEAKEXTERNAL;
      pd_flags : [pd_implemen,pd_interface,pd_notobject,pd_notobjintf,pd_cppobject,pd_notrecord,pd_nothelper];
      handler  : @pd_weakexternal;
      pocall   : pocall_none;
      { mark it both external and weak external, so we don't have to
        adapt all code for external symbols to also check for weak external
      }
      pooption : [po_external,po_weakexternal];
      mutexclpocall : [pocall_internproc,pocall_syscall];
      { allowed for external cpp classes }
      mutexclpotype : [{potype_constructor,potype_destructor}potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_public,po_exports,po_interrupt,po_assembler,po_inline]
    ),(
      idtok:_ENUMERATOR;
      pd_flags : [pd_interface,pd_object,pd_record];
      handler  : @pd_enumerator;
      pocall   : pocall_none;
      pooption : [];
      mutexclpocall : [pocall_internproc];
      mutexclpotype : [];
      mutexclpo     : [po_exports,po_interrupt,po_external,po_inline]
    ),(
      idtok:_RTLPROC;
      pd_flags : [pd_interface,pd_implemen,pd_body,pd_notobjintf];
      handler  : nil;
      pocall   : pocall_none;
      pooption : [po_rtlproc];
      mutexclpocall : [];
      mutexclpotype : [potype_constructor,potype_destructor,potype_class_constructor,potype_class_destructor];
      mutexclpo     : [po_interrupt]
    )
   );


    function check_proc_directive(isprocvar:boolean):boolean;
      var
        i : longint;
      begin
        result:=false;
        for i:=1 to num_proc_directives do
         if proc_direcdata[i].idtok=idtoken then
          begin
            if ((not isprocvar) or
               (pd_procvar in proc_direcdata[i].pd_flags)) and
               { don't eat a public directive in classes }
               not((idtoken=_PUBLIC) and (symtablestack.top.symtabletype=ObjectSymtable)) then
              result:=true;
            exit;
          end;
      end;


    function parse_proc_direc(pd:tabstractprocdef;var pdflags:tpdflags):boolean;
      {
        Parse the procedure directive, returns true if a correct directive is found
      }
      var
        p     : longint;
        found : boolean;
        name  : TIDString;
      begin
        parse_proc_direc:=false;
        name:=tokeninfo^[idtoken].str;
        found:=false;

      { Hint directive? Then exit immediatly }
        if (m_hintdirective in current_settings.modeswitches) then
         begin
           case idtoken of
             _LIBRARY,
             _PLATFORM,
             _UNIMPLEMENTED,
             _EXPERIMENTAL,
             _DEPRECATED :
               exit;
           end;
         end;

        { C directive is MacPas only, because it breaks too much existing code
          on other platforms (PFV) }
        if (idtoken=_C) and
           not(m_mac in current_settings.modeswitches) then
          exit;

      { retrieve data for directive if found }
        for p:=1 to num_proc_directives do
         if proc_direcdata[p].idtok=idtoken then
          begin
            found:=true;
            break;
          end;

      { Check if the procedure directive is known }
        if not found then
         begin
            { parsing a procvar type the name can be any
              next variable !! }
            if ((pdflags * [pd_procvar,pd_object,pd_record,pd_objcclass,pd_objcprot])=[]) and
               not(idtoken=_PROPERTY) then
              Message1(parser_w_unknown_proc_directive_ignored,name);
            exit;
         end;

        { check if method and directive not for object, like public.
          This needs to be checked also for procvars }
        if (pd_notobject in proc_direcdata[p].pd_flags) and
           (symtablestack.top.symtabletype=ObjectSymtable) and
           { directive allowed for cpp classes? }
           not((pd_cppobject in proc_direcdata[p].pd_flags) and is_cppclass(tdef(symtablestack.top.defowner))) and
           not((pd_javaclass in proc_direcdata[p].pd_flags) and is_javaclass(tdef(symtablestack.top.defowner))) and
           not((pd_intfjava in proc_direcdata[p].pd_flags) and is_javainterface(tdef(symtablestack.top.defowner))) then
           exit;

        if (pd_notrecord in proc_direcdata[p].pd_flags) and
           (symtablestack.top.symtabletype=recordsymtable) then
           exit;

        { check if method and directive not for java class }
        if not(pd_javaclass in proc_direcdata[p].pd_flags) and
           is_javaclass(tdef(symtablestack.top.defowner)) then
          exit;

        { check if method and directive not for java interface }
        if not(pd_intfjava in proc_direcdata[p].pd_flags) and
           is_javainterface(tdef(symtablestack.top.defowner)) then
          exit;

        { Conflicts between directives ? }
        if (pd.proctypeoption in proc_direcdata[p].mutexclpotype) or
           (pd.proccalloption in proc_direcdata[p].mutexclpocall) or
           ((pd.procoptions*proc_direcdata[p].mutexclpo)<>[]) then
         begin
           Message1(parser_e_proc_dir_conflict,name);
           exit;
         end;

        { set calling convention }
        if proc_direcdata[p].pocall<>pocall_none then
         begin
           if (po_hascallingconvention in pd.procoptions) then
            begin
              Message2(parser_w_proc_overriding_calling,
                proccalloptionStr[pd.proccalloption],
                proccalloptionStr[proc_direcdata[p].pocall]);
            end;
           { check if the target processor supports this calling convention }
           if not(proc_direcdata[p].pocall in supported_calling_conventions) then
             begin
               Message1(parser_e_illegal_calling_convention,proccalloptionStr[proc_direcdata[p].pocall]);
               { recover }
               proc_direcdata[p].pocall:=pocall_stdcall;
             end;
           pd.proccalloption:=proc_direcdata[p].pocall;
           include(pd.procoptions,po_hascallingconvention);
         end;

        if pd.typ=procdef then
         begin
           { Check if the directive is only for objects }
           if (pd_object in proc_direcdata[p].pd_flags) and
              not assigned(tprocdef(pd).struct) then
            exit;

           { Check if the directive is only for records }
           if (pd_record in proc_direcdata[p].pd_flags) and
              not assigned(tprocdef(pd).struct) then
            exit;

           { check if method and directive not for interface }
           if (pd_notobjintf in proc_direcdata[p].pd_flags) and
              is_interface(tprocdef(pd).struct) then
            exit;

           { check if method and directive not for interface }
           if is_dispinterface(tprocdef(pd).struct) and
             not(pd_dispinterface in proc_direcdata[p].pd_flags) then
            exit;

           { check if method and directive not for objcclass }
           if is_objcclass(tprocdef(pd).struct) and
             not(pd_objcclass in proc_direcdata[p].pd_flags) then
            exit;

           { check if method and directive not for objcprotocol }
           if is_objcprotocol(tprocdef(pd).struct) and
             not(pd_objcprot in proc_direcdata[p].pd_flags) then
            exit;

           { check if method and directive not for record/class helper }
           if is_objectpascal_helper(tprocdef(pd).struct) and
             (pd_nothelper in proc_direcdata[p].pd_flags) then
         end;

        { consume directive, and turn flag on }
        consume(token);
        parse_proc_direc:=true;

        { Check the pd_flags if the directive should be allowed }
        if (pd_interface in pdflags) and
           not(pd_interface in proc_direcdata[p].pd_flags) then
          begin
            Message1(parser_e_proc_dir_not_allowed_in_interface,name);
            exit;
          end;
        if (pd_implemen in pdflags) and
           not(pd_implemen in proc_direcdata[p].pd_flags) then
          begin
            Message1(parser_e_proc_dir_not_allowed_in_implementation,name);
            exit;
          end;
        if (pd_procvar in pdflags) and
           not(pd_procvar in proc_direcdata[p].pd_flags) then
          begin
            Message1(parser_e_proc_dir_not_allowed_in_procvar,name);
            exit;
          end;

        { Return the new pd_flags }
        if not(pd_body in proc_direcdata[p].pd_flags) then
          exclude(pdflags,pd_body);

        { Add the correct flag }
        pd.procoptions:=pd.procoptions+proc_direcdata[p].pooption;

        { Call the handler }
        if pointer(proc_direcdata[p].handler)<>nil then
          proc_direcdata[p].handler(pd);
      end;



    function proc_get_importname(pd:tprocdef):string;
      var
        dllname, importname : string;

      begin
        result:='';
        if not(po_external in pd.procoptions) then
          internalerror(200412151);
        { external name or number is specified }
        if assigned(pd.import_name) or (pd.import_nr<>0) then
          begin
            if assigned(pd.import_dll) then
              dllname:=pd.import_dll^
            else
              dllname:='';
            if assigned(pd.import_name) then
              importname:=pd.import_name^
            else
              importname:='';
            proc_get_importname:=make_dllmangledname(dllname,
              importname,pd.import_nr,pd.proccalloption);
          end
        else
          begin
            { Default names when importing variables }
            case pd.proccalloption of
              pocall_cdecl :
                begin
                  if assigned(pd.struct) then
                    result:=target_info.Cprefix+pd.struct.objrealname^+'_'+pd.procsym.realname
                  else
                    result:=target_info.Cprefix+pd.procsym.realname;
                end;
              pocall_cppdecl :
                begin
                  result:=target_info.Cprefix+pd.cplusplusmangledname;
                end;
              else
                begin
                  {In MacPas a single "external" has the same effect as "external name 'xxx'" }
                  { but according to MacPas mode description
                    Cprefix should still be used PM }
                  if (m_mac in current_settings.modeswitches) then
                    result:=target_info.Cprefix+tprocdef(pd).procsym.realname;
                end;
            end;
          end;
      end;


    procedure proc_set_mangledname(pd:tprocdef);
      var
        s : string;
      begin
        { When the mangledname is already set we aren't allowed to change
          it because it can already be used somewhere (PFV) }
        if not(po_has_mangledname in pd.procoptions) then
          begin
            if (po_external in pd.procoptions) then
              begin
                { External Procedures are only allowed to change the mangledname
                  in their first declaration }
                if (pd.forwarddef or (not pd.hasforward)) then
                  begin
                    s:=proc_get_importname(pd);
                    if s<>'' then
                      begin
                        pd.setmangledname(s);
                      end;
                  end;
              end
            else
            { Normal procedures }
              begin
                if (po_compilerproc in pd.procoptions) then
                  begin
                    pd.setmangledname(lower(pd.procsym.name));
                  end;
              end;
          end;

        { Public/exported alias names }
        if (([po_public,po_exports]*pd.procoptions)<>[]) and
           not(po_has_public_name in pd.procoptions) then
          begin
            case pd.proccalloption of
              pocall_cdecl :
                begin
                  if assigned(pd.struct) then
                   pd.aliasnames.insert(target_info.Cprefix+pd.struct.objrealname^+'_'+pd.procsym.realname)
                  else
                    begin
                      { Export names are not mangled on Windows and OS/2, see also pexports.pas }
                      if (target_info.system in (systems_all_windows+[system_i386_emx, system_i386_os2])) and
                        (po_exports in pd.procoptions) then
                        pd.aliasnames.insert(pd.procsym.realname)
                      else
                        pd.aliasnames.insert(target_info.Cprefix+pd.procsym.realname);
                    end;
                end;
              pocall_cppdecl :
                begin
                  pd.aliasnames.insert(target_info.Cprefix+pd.cplusplusmangledname);
                end;
            end;
            { prevent adding the alias a second time }
            include(pd.procoptions,po_has_public_name);
          end;
      end;


    procedure handle_calling_convention(pd:tabstractprocdef;flags:thccflags=hcc_all);
      begin
        if hcc_check in flags then
          begin
            { set the default calling convention if none provided }
            if (pd.typ=procdef) and
               (is_objc_class_or_protocol(tprocdef(pd).struct) or
                is_cppclass(tprocdef(pd).struct)) then
              begin
                { none of the explicit calling conventions should be allowed }
                if (po_hascallingconvention in pd.procoptions) then
                  internalerror(2009032501);
                if is_cppclass(tprocdef(pd).struct) then
                  pd.proccalloption:=pocall_cppdecl
                else
                  pd.proccalloption:=pocall_cdecl;
              end
            else if not(po_hascallingconvention in pd.procoptions) then
              pd.proccalloption:=current_settings.defproccall
            else
              begin
                if pd.proccalloption=pocall_none then
                  internalerror(200309081);
              end;

            { handle proccall specific settings }
            case pd.proccalloption of
              pocall_cdecl,
              pocall_cppdecl :
                begin
                  { check C cdecl para types }
                  check_c_para(pd);
                end;
              pocall_far16 :
                begin
                  { Temporary stub, must be rewritten to support OS/2 far16 }
                  Message1(parser_w_proc_directive_ignored,'FAR16');
                end;
            end;

            { Inlining is enabled and supported? }
            if (po_inline in pd.procoptions) and
               not(cs_do_inline in current_settings.localswitches) then
              begin
                { Give an error if inline is not supported by the compiler mode,
                  otherwise only give a hint that this procedure will not be inlined }
                if not(m_default_inline in current_settings.modeswitches) then
                  Message(parser_e_proc_inline_not_supported)
                else
                  Message(parser_h_inlining_disabled);
                exclude(pd.procoptions,po_inline);
              end;

            { For varargs directive also cdecl and external must be defined }
            if (po_varargs in pd.procoptions) then
             begin
               { check first for external in the interface, if available there
                 then the cdecl must also be there since there is no implementation
                 available to contain it }
               if parse_only then
                begin
                  { if external is available, then cdecl must also be available,
                    procvars don't need external }
                  if not((po_external in pd.procoptions) or
                         (pd.typ=procvardef) or
                         { for objcclasses this is checked later, because the entire
                           class may be external.  }
                         is_objc_class_or_protocol(tprocdef(pd).struct)) and
                     not(pd.proccalloption in (cdecl_pocalls + [pocall_mwpascal])) then
                    Message(parser_e_varargs_need_cdecl_and_external);
                end
               else
                begin
                  { both must be defined now }
                  if not((po_external in pd.procoptions) or
                         (pd.typ=procvardef)) or
                     not(pd.proccalloption in (cdecl_pocalls + [pocall_mwpascal])) then
                    Message(parser_e_varargs_need_cdecl_and_external);
                end;
             end;
          end;

        if hcc_insert_hidden_paras in flags then
          begin
            { insert hidden high parameters }
            pd.parast.SymList.ForEachCall(@insert_hidden_para,pd);

            { insert hidden self parameter }
            insert_self_and_vmt_para(pd);

            { insert funcret parameter if required }
            insert_funcret_para(pd);

            { Make var parameters regable, this must be done after the calling
              convention is set. }
            { this must be done before parentfp is insert, because getting all cases
              where parentfp must be in a memory location isn't catched properly so
              we put parentfp never in a register }
            pd.parast.SymList.ForEachCall(@set_addr_param_regable,pd);

            { insert parentfp parameter if required }
            insert_parentfp_para(pd);
          end;

        { Calculate parameter tlist }
        pd.calcparas;
      end;


    procedure parse_proc_directives(pd:tabstractprocdef;var pdflags:tpdflags);
      {
        Parse the procedure directives. It does not matter if procedure directives
        are written using ;procdir; or ['procdir'] syntax.
      }
      var
        res : boolean;
      begin
        if (m_mac in current_settings.modeswitches) and (cs_externally_visible in current_settings.localswitches) then
          begin
            tprocdef(pd).aliasnames.insert(target_info.Cprefix+tprocdef(pd).procsym.realname);
            include(pd.procoptions,po_public);
            include(pd.procoptions,po_has_public_name);
            include(pd.procoptions,po_global);
          end;

        { methods from external class definitions are all external themselves }
        if (pd.typ=procdef) and
           assigned(tprocdef(pd).struct) and
           (tprocdef(pd).struct.typ=objectdef) and
           (oo_is_external in tobjectdef(tprocdef(pd).struct).objectoptions) then
          tprocdef(pd).make_external;

        { Class constructors and destructor are static class methods in real. }
        { There are many places in the compiler where either class or static  }
        { method flag changes the behavior. It is simplier to add them to     }
        { the class constructors/destructors options than to fix all the      }
        { occurencies. (Paul)                                                 }
        if pd.proctypeoption in [potype_class_constructor,potype_class_destructor] then
          begin
            include(pd.procoptions,po_classmethod);
            include(pd.procoptions,po_staticmethod);
          end;

        while token in [_ID,_LECKKLAMMER] do
         begin
           if try_to_consume(_LECKKLAMMER) then
            begin
              repeat
                parse_proc_direc(pd,pdflags);
              until not try_to_consume(_COMMA);
              consume(_RECKKLAMMER);
              { we always expect at least '[];' }
              res:=true;
            end
           else
            begin
              res:=parse_proc_direc(pd,pdflags);
            end;
           { A procedure directive normally followed by a semicolon, but in
             a const section or reading a type we should stop when _EQ is found,
             because a constant/default value follows }
           if res then
            begin
              if (block_type=bt_const_type) and
                 (token=_EQ) then
               break;
              { support procedure proc;stdcall export; }
              if not(check_proc_directive((pd.typ=procvardef))) then
                begin
                  { support "record p : procedure stdcall end;" and
                    "var p : procedure stdcall = nil;" }
                  if (pd_procvar in pdflags) and
                     (token in [_END,_RKLAMMER,_EQ]) then
                    break
                  else
                    begin
                      if (token=_COLON) then
                        begin
                          Message(parser_e_field_not_allowed_here);
                          consume_all_until(_SEMICOLON);
                        end;
                      consume(_SEMICOLON)
                    end;
                end;
            end
           else
            break;
         end;
      end;


    procedure parse_var_proc_directives(sym:tsym);
      var
        pdflags : tpdflags;
        pd      : tabstractprocdef;
      begin
        pdflags:=[pd_procvar];
        pd:=nil;
        case sym.typ of
          fieldvarsym,
          staticvarsym,
          localvarsym,
          paravarsym :
            pd:=tabstractprocdef(tabstractvarsym(sym).vardef);
          typesym :
            pd:=tabstractprocdef(ttypesym(sym).typedef);
          else
            internalerror(2003042617);
        end;
        if pd.typ<>procvardef then
          internalerror(2003042618);
        { names should never be used anyway }
        parse_proc_directives(pd,pdflags);
      end;


    procedure parse_object_proc_directives(pd:tabstractprocdef);
      var
        pdflags : tpdflags;
      begin
        pdflags:=[pd_object];
        parse_proc_directives(pd,pdflags);
      end;

    procedure parse_record_proc_directives(pd:tabstractprocdef);
      var
        pdflags : tpdflags;
      begin
        pdflags:=[pd_record];
        parse_proc_directives(pd,pdflags);
      end;

    function proc_add_definition(var currpd:tprocdef):boolean;
      {
        Add definition aprocdef to the overloaded definitions of aprocsym. If a
        forwarddef is found and reused it returns true
      }
      var
        fwpd    : tprocdef;
        currparasym,
        fwparasym : tsym;
        currparacnt,
        fwparacnt,
        curridx,
        fwidx,
        i       : longint;
        po_comp : tprocoptions;
        paracompopt: tcompare_paras_options;
        forwardfound : boolean;
        symentry: TSymEntry;
      begin
        forwardfound:=false;

        { check overloaded functions if the same function already exists }
        for i:=0 to tprocsym(currpd.procsym).ProcdefList.Count-1 do
         begin
           fwpd:=tprocdef(tprocsym(currpd.procsym).ProcdefList[i]);

           { can happen for internally generated routines }
           if (fwpd=currpd) then
             begin
               result:=true;
               exit;
             end;

           { Skip overloaded definitions that are declared in other units }
           if fwpd.procsym<>currpd.procsym then
             continue;

           { check the parameters, for delphi/tp it is possible to
             leave the parameters away in the implementation (forwarddef=false).
             But for an overload declared function this is not allowed }
           if { check if empty implementation arguments match is allowed }
              (
               not(m_repeat_forward in current_settings.modeswitches) and
               not(currpd.forwarddef) and
               is_bareprocdef(currpd) and
               not(po_overload in fwpd.procoptions)
              ) or
              { check arguments, we need to check only the user visible parameters. The hidden parameters
                can be in a different location because of the calling convention, eg. L-R vs. R-L order (PFV)

                don't check default values here, because routines that are the same except for their default
                values should be reported as mismatches (since you can't overload based on different default
                parameter values) }
              (
               (compare_paras(fwpd.paras,currpd.paras,cp_none,[cpo_ignorehidden,cpo_openequalisexact,cpo_ignoreuniv])=te_exact) and
               (compare_defs(fwpd.returndef,currpd.returndef,nothingn)=te_exact)
              ) then
             begin
               { Check if we've found the forwarddef, if found then
                 we need to update the forward def with the current
                 implementation settings }
               if fwpd.forwarddef then
                 begin
                   forwardfound:=true;

                   if not(m_repeat_forward in current_settings.modeswitches) and
                      (fwpd.proccalloption<>currpd.proccalloption) then
                     paracompopt:=[cpo_ignorehidden,cpo_comparedefaultvalue,cpo_openequalisexact,cpo_ignoreuniv]
                   else
                     paracompopt:=[cpo_comparedefaultvalue,cpo_openequalisexact,cpo_ignoreuniv];

                   { Check calling convention }
                   if (fwpd.proccalloption<>currpd.proccalloption) then
                    begin
                      { In delphi it is possible to specify the calling
                        convention in the interface or implementation if
                        there was no convention specified in the other
                        part }
                      if (m_delphi in current_settings.modeswitches) then
                        begin
                          if not(po_hascallingconvention in currpd.procoptions) then
                            currpd.proccalloption:=fwpd.proccalloption
                          else
                            if not(po_hascallingconvention in fwpd.procoptions) then
                              fwpd.proccalloption:=currpd.proccalloption
                          else
                            begin
                              MessagePos(currpd.fileinfo,parser_e_call_convention_dont_match_forward);
                              tprocsym(currpd.procsym).write_parameter_lists(currpd);
                              { restore interface settings }
                              currpd.proccalloption:=fwpd.proccalloption;
                            end;
                        end
                      else
                        begin
                          MessagePos(currpd.fileinfo,parser_e_call_convention_dont_match_forward);
                          tprocsym(currpd.procsym).write_parameter_lists(currpd);
                          { restore interface settings }
                          currpd.proccalloption:=fwpd.proccalloption;
                        end;
                    end;

                   { Check static }
                   if (po_staticmethod in fwpd.procoptions) then
                    begin
                      if not (po_staticmethod in currpd.procoptions) then
                       begin
                         include(currpd.procoptions, po_staticmethod);
                         if (po_classmethod in currpd.procoptions) then
                          begin
                           { remove self from the hidden paras }
                           symentry:=currpd.parast.Find('self');
                           if symentry<>nil then
                            begin
                              currpd.parast.Delete(symentry);
                              currpd.calcparas;
                            end;
                          end;
                       end;
                    end;

                   { Check if the procedure type and return type are correct,
                     also the parameters must match also with the type and that
                     if the implementation has default parameters, the interface
                     also has them and that if they both have them, that they
                     have the same value }
                   if ((m_repeat_forward in current_settings.modeswitches) or
                       not is_bareprocdef(currpd)) and
                      ((compare_paras(fwpd.paras,currpd.paras,cp_all,paracompopt)<>te_exact) or
                       (compare_defs(fwpd.returndef,currpd.returndef,nothingn)<>te_exact)) then
                     begin
                       MessagePos1(currpd.fileinfo,parser_e_header_dont_match_forward,
                                   fwpd.fullprocname(false));
                       tprocsym(currpd.procsym).write_parameter_lists(currpd);
                       break;
                     end;

                   { Check if both are declared forward }
                   if fwpd.forwarddef and currpd.forwarddef then
                    begin
                      MessagePos1(currpd.fileinfo,parser_e_function_already_declared_public_forward,
                                  currpd.fullprocname(false));
                    end;

                   { internconst or internproc only need to be defined once }
                   if (fwpd.proccalloption=pocall_internproc) then
                    currpd.proccalloption:=fwpd.proccalloption
                   else
                    if (currpd.proccalloption=pocall_internproc) then
                     fwpd.proccalloption:=currpd.proccalloption;

                   { Check procedure options, Delphi requires that class is
                     repeated in the implementation for class methods }
                   if (m_fpc in current_settings.modeswitches) then
                     po_comp:=[po_classmethod,po_varargs,po_methodpointer,po_interrupt]
                   else
                     po_comp:=[po_classmethod,po_methodpointer];

                   if ((po_comp * fwpd.procoptions)<>(po_comp * currpd.procoptions)) or
                      (fwpd.proctypeoption <> currpd.proctypeoption) or
                      { if the implementation version has an "overload" modifier,
                        the interface version must also have it (otherwise we can
                        get annoying crashes due to interface crc changes) }
                      (not(po_overload in fwpd.procoptions) and
                       (po_overload in currpd.procoptions)) then
                     begin
                       MessagePos1(currpd.fileinfo,parser_e_header_dont_match_forward,
                                   fwpd.fullprocname(false));
                       tprocsym(fwpd.procsym).write_parameter_lists(fwpd);
                       { This error is non-fatal, we can recover }
                     end;

                   { Forward declaration is external? }
                   if (po_external in fwpd.procoptions) then
                     MessagePos(currpd.fileinfo,parser_e_proc_already_external);

                   { Check parameters }
                   if (m_repeat_forward in current_settings.modeswitches) or
                      (currpd.minparacount>0) then
                    begin
                      { If mangled names are equal then they have the same amount of arguments }
                      { We can check the names of the arguments }
                      { both symtables are in the same order from left to right }
                      curridx:=0;
                      fwidx:=0;
                      currparacnt:=currpd.parast.SymList.Count;
                      fwparacnt:=fwpd.parast.SymList.Count;
                      repeat
                        { skip default parameter constsyms }
                        while (curridx<currparacnt) and
                              (tsym(currpd.parast.SymList[curridx]).typ<>paravarsym) do
                          inc(curridx);
                        while (fwidx<fwparacnt) and
                              (tsym(fwpd.parast.SymList[fwidx]).typ<>paravarsym) do
                          inc(fwidx);
                        { stop when one of the two lists is at the end }
                        if (fwidx>=fwparacnt) or (curridx>=currparacnt) then
                          break;
                        { compare names of parameters, ignore implictly
                          renamed parameters }
                        currparasym:=tsym(currpd.parast.SymList[curridx]);
                        fwparasym:=tsym(fwpd.parast.SymList[fwidx]);
                        if not(sp_implicitrename in currparasym.symoptions) and
                           not(sp_implicitrename in fwparasym.symoptions) then
                          begin
                            if (currparasym.name<>fwparasym.name) then
                              begin
                                MessagePos3(currpd.fileinfo,parser_e_header_different_var_names,
                                            tprocsym(currpd.procsym).realname,fwparasym.realname,currparasym.realname);
                                break;
                              end;
                          end;
                        { next parameter }
                        inc(curridx);
                        inc(fwidx);
                      until false;
                    end;
                   { Everything is checked, now we can update the forward declaration
                     with the new data from the implementation }
                   fwpd.forwarddef:=currpd.forwarddef;
                   fwpd.hasforward:=true;
                   fwpd.procoptions:=fwpd.procoptions+currpd.procoptions;

                   { marked as local but exported from unit? }
                   if (po_kylixlocal in fwpd.procoptions) and (fwpd.owner.symtabletype=globalsymtable) then
                     MessagePos(fwpd.fileinfo,type_e_cant_export_local);

                   if fwpd.extnumber=$ffff then
                     fwpd.extnumber:=currpd.extnumber;
                   while not currpd.aliasnames.empty do
                     fwpd.aliasnames.insert(currpd.aliasnames.getfirst);
                   { update fileinfo so position references the implementation,
                     also update funcretsym if it is already generated }
                   fwpd.fileinfo:=currpd.fileinfo;
                   if assigned(fwpd.funcretsym) then
                     fwpd.funcretsym.fileinfo:=currpd.fileinfo;
                   if assigned(currpd.deprecatedmsg) then
                     begin
                       stringdispose(fwpd.deprecatedmsg);
                       fwpd.deprecatedmsg:=stringdup(currpd.deprecatedmsg^);
                     end;
                   { import names }
                   if assigned(currpd.import_dll) then
                     begin
                       stringdispose(fwpd.import_dll);
                       fwpd.import_dll:=stringdup(currpd.import_dll^);
                     end;
                   if assigned(currpd.import_name) then
                     begin
                       stringdispose(fwpd.import_name);
                       fwpd.import_name:=stringdup(currpd.import_name^);
                     end;
                   fwpd.import_nr:=currpd.import_nr;
                   { for compilerproc defines we need to rename and update the
                     symbolname to lowercase }
                   if (po_compilerproc in fwpd.procoptions) then
                    begin
                      { rename to lowercase so users can't access it }
                      fwpd.procsym.realname:='$'+lower(fwpd.procsym.name);
                      { the mangeled name is already changed by the pd_compilerproc }
                      { handler. It must be done immediately because if we have a   }
                      { call to a compilerproc before it's implementation is        }
                      { encountered, it must already use the new mangled name (JM)  }
                    end;

                   { Release current procdef }
                   currpd.owner.deletedef(currpd);
                   currpd:=fwpd;
                 end
               else
                begin
                  { abstract methods aren't forward defined, but this }
                  { needs another error message                   }
                  if (po_abstractmethod in fwpd.procoptions) then
                    MessagePos(currpd.fileinfo,parser_e_abstract_no_definition)
                  else
                    begin
                      MessagePos(currpd.fileinfo,parser_e_overloaded_have_same_parameters);
                      tprocsym(currpd.procsym).write_parameter_lists(currpd);
                    end;
                 end;

               { we found one proc with the same arguments, there are no others
                 so we can stop }
               break;
             end;

           { check for allowing overload directive }
           if not(m_fpc in current_settings.modeswitches) then
            begin
              { overload directive turns on overloading }
              if ((po_overload in currpd.procoptions) or
                  (po_overload in fwpd.procoptions)) then
               begin
                 { check if all procs have overloading, but not if the proc is a method or
                   already declared forward, then the check is already done }
                 if not(fwpd.hasforward or
                        assigned(currpd.struct) or
                        (currpd.forwarddef<>fwpd.forwarddef) or
                        ((po_overload in currpd.procoptions) and
                         (po_overload in fwpd.procoptions))) then
                  begin
                    MessagePos1(currpd.fileinfo,parser_e_no_overload_for_all_procs,currpd.procsym.realname);
                    break;
                  end
               end
              else
               begin
                 if not(fwpd.forwarddef) then
                  begin
                    if (m_tp7 in current_settings.modeswitches) then
                      MessagePos(currpd.fileinfo,parser_e_procedure_overloading_is_off)
                    else
                      MessagePos1(currpd.fileinfo,parser_e_no_overload_for_all_procs,currpd.procsym.realname);
                    break;
                  end;
               end;
            end; { equal arguments }
         end;

        { if we didn't reuse a forwarddef then we add the procdef to the overloaded
          list }
        if not forwardfound then
          begin
            { can happen in Delphi mode }
            if (currpd.proctypeoption = potype_function) and
               is_void(currpd.returndef) then
              MessagePos1(currpd.fileinfo,parser_e_no_funcret_specified,currpd.procsym.realname);
            tprocsym(currpd.procsym).ProcdefList.Add(currpd);
          end;

        proc_add_definition:=forwardfound;
      end;

end.
