{
    Copyright (c) 1998-2002 by Florian Klaempfl, Daniel Mantione

    Helpers for dealing with subroutine parameters during parsing

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
unit pparautl;

{$i fpcdefs.inc}

interface

    uses
      symconst,symdef;

    procedure insert_funcret_para(pd:tabstractprocdef);
    procedure insert_parentfp_para(pd:tabstractprocdef);
    procedure insert_self_and_vmt_para(pd:tabstractprocdef);
    procedure insert_funcret_local(pd:tprocdef);
    procedure insert_hidden_para(p:TObject;arg:pointer);
    procedure check_c_para(pd:Tabstractprocdef);
    procedure insert_struct_hidden_paras(astruct: tabstractrecorddef);

    type
      // flags of the *handle_calling_convention routines
      thccflag=(
        hcc_declaration,          // declaration (as opposed to definition, i.e. interface rather than implementation)
        hcc_check,                // perform checks and outup errors if found
        hcc_insert_hidden_paras   // insert hidden parameters
      );
      thccflags=set of thccflag;

    const
      hcc_default_actions_intf=[hcc_declaration,hcc_check,hcc_insert_hidden_paras];
      hcc_default_actions_intf_struct=hcc_default_actions_intf-[hcc_insert_hidden_paras];
      hcc_default_actions_impl=[hcc_check,hcc_insert_hidden_paras];
      hcc_default_actions_parse=[hcc_check,hcc_insert_hidden_paras];
      PD_VIRTUAL_MUTEXCLPO = [po_interrupt,po_exports,po_overridingmethod,po_inline,po_staticmethod];

    procedure handle_calling_convention(pd:tabstractprocdef;flags:thccflags);
    function proc_add_definition(var currpd:tprocdef):boolean;

    { create "parent frame pointer" record skeleton for procdef, in which local
      variables and parameters from pd accessed from nested routines can be
      stored }
    procedure build_parentfpstruct(pd: tprocdef);

implementation

    uses
      globals,globtype,cclasses,cutils,verbose,systems,fmodule,
      tokens,
      symtype,symbase,symsym,symtable,symutil,defutil,defcmp,blockutl,
{$ifdef jvm}
      jvmdef,
{$endif jvm}
      node,nbas,
      aasmbase,
      paramgr;


    procedure insert_funcret_para(pd:tabstractprocdef);
      var
        storepos : tfileposinfo;
        vs       : tparavarsym;
        paranr   : word;
      begin
        if not(pd.proctypeoption in [potype_constructor,potype_destructor]) and
           not is_void(pd.returndef) and
           not (df_generic in pd.defoptions) and
           paramanager.ret_in_param(pd.returndef,pd) then
         begin
           storepos:=current_tokenpos;
           if pd.typ=procdef then
            current_tokenpos:=tprocdef(pd).fileinfo;

{$if defined(i386)}
           { For left to right add it at the end to be delphi compatible.
             In the case of safecalls with safecal-exceptions support the
             funcret-para is (from the 'c'-point of view) a normal parameter
             which has to be added to the end of the parameter-list }
           if (pd.proccalloption in (pushleftright_pocalls)) or
              ((tf_safecall_exceptions in target_info.flags) and
               (pd.proccalloption=pocall_safecall)) then
             paranr:=paranr_result_leftright
           else
{$elseif defined(SUPPORT_SAFECALL)}
           if (tf_safecall_exceptions in target_info.flags) and
              (pd.proccalloption = pocall_safecall)  then
             paranr:=paranr_result_leftright
           else
{$endif}
             paranr:=paranr_result;
           { Generate result variable accessing function result }
           vs:=cparavarsym.create('$result',paranr,vs_var,pd.returndef,[vo_is_funcret,vo_is_hidden_para]);
           pd.parast.insert(vs);
           { Store this symbol as funcretsym for procedures }
           if pd.typ=procdef then
            tprocdef(pd).funcretsym:=vs;

           current_tokenpos:=storepos;
         end;
      end;


    procedure insert_parentfp_para(pd:tabstractprocdef);
      var
        storepos : tfileposinfo;
        vs       : tparavarsym;
        paranr   : longint;
      begin
        if pd.parast.symtablelevel>normal_function_level then
          begin
            storepos:=current_tokenpos;
            if pd.typ=procdef then
             current_tokenpos:=tprocdef(pd).fileinfo;

            { if no support for nested procvars is activated, use the old
              calling convention to pass the parent frame pointer for backwards
              compatibility }
            if not(m_nested_procvars in current_settings.modeswitches) then
              paranr:=paranr_parentfp
            { nested procvars require Delphi-style parentfp passing, see
              po_delphi_nested_cc declaration for more info }
{$if defined(i386) or defined(i8086)}
            else if (pd.proccalloption in pushleftright_pocalls) then
              paranr:=paranr_parentfp_delphi_cc_leftright
{$endif i386 or i8086}
            else
              paranr:=paranr_parentfp_delphi_cc;
            { Generate frame pointer. It can't be put in a register since it
              must be accessable from nested routines }
            if not(target_info.system in systems_fpnestedstruct) or
               { in case of errors or declared procvardef types, prevent invalid
                 type cast and possible nil pointer dereference }
               not assigned(pd.owner.defowner) or
               (pd.owner.defowner.typ<>procdef) then
              begin
                vs:=cparavarsym.create('$parentfp',paranr,vs_value
                      ,parentfpvoidpointertype,[vo_is_parentfp,vo_is_hidden_para]);
                vs.varregable:=vr_none;
              end
            else
              begin
                if not assigned(tprocdef(pd.owner.defowner).parentfpstruct) then
                  build_parentfpstruct(tprocdef(pd.owner.defowner));
                vs:=cparavarsym.create('$parentfp',paranr,vs_value,
                      tprocdef(pd.owner.defowner).parentfpstructptrtype,[vo_is_parentfp,vo_is_hidden_para]);
              end;
            pd.parast.insert(vs);

            current_tokenpos:=storepos;
          end;
      end;


    procedure insert_self_and_vmt_para(pd:tabstractprocdef);
      var
        storepos : tfileposinfo;
        vs       : tparavarsym;
        hdef     : tdef;
        selfdef  : tdef;
        vsp      : tvarspez;
        aliasvs  : tabsolutevarsym;
        sl       : tpropaccesslist;
      begin
        if (pd.typ=procdef) and
           is_objc_class_or_protocol(tprocdef(pd).struct) and
           (pd.parast.symtablelevel=normal_function_level) then
          begin
            { insert Objective-C self and selector parameters }
            vs:=cparavarsym.create('$_cmd',paranr_objc_cmd,vs_value,objc_seltype,[vo_is_msgsel,vo_is_hidden_para]);
            pd.parast.insert(vs);
            { make accessible to code }
            sl:=tpropaccesslist.create;
            sl.addsym(sl_load,vs);
            aliasvs:=cabsolutevarsym.create_ref('_CMD',objc_seltype,sl);
            include(aliasvs.varoptions,vo_is_msgsel);
            tlocalsymtable(tprocdef(pd).localst).insert(aliasvs);

            if (po_classmethod in pd.procoptions) then
              { compatible with what gcc does }
              hdef:=objc_idtype
            else
              hdef:=tprocdef(pd).struct;

            vs:=cparavarsym.create('$self',paranr_objc_self,vs_value,hdef,[vo_is_self,vo_is_hidden_para]);
            pd.parast.insert(vs);
          end
        else if (pd.typ=procvardef) and
           pd.is_methodpointer then
          begin
            { Generate self variable }
            vs:=cparavarsym.create('$self',paranr_self,vs_value,voidpointertype,[vo_is_self,vo_is_hidden_para]);
            pd.parast.insert(vs);
          end
        { while only procvardefs of this type can be declared in Pascal code,
          internally we also generate procdefs of this type when creating
          block wrappers }
        else if (po_is_block in pd.procoptions) then
          begin
            { generate the first hidden parameter, which is a so-called "block
              literal" describing the block and containing its invocation
              procedure  }
            hdef:=cpointerdef.getreusable(get_block_literal_type_for_proc(pd));
            { mark as vo_is_parentfp so that proc2procvar comparisons will
              succeed when assigning arbitrary routines to the block }
            vs:=cparavarsym.create('$_block_literal',paranr_blockselfpara,vs_value,
              hdef,[vo_is_hidden_para,vo_is_parentfp]
            );
            pd.parast.insert(vs);
            if pd.typ=procdef then
              begin
                { make accessible to code }
                sl:=tpropaccesslist.create;
                sl.addsym(sl_load,vs);
                aliasvs:=cabsolutevarsym.create_ref('FPC_BLOCK_SELF',hdef,sl);
                include(aliasvs.varoptions,vo_is_parentfp);
                tlocalsymtable(tprocdef(pd).localst).insert(aliasvs);
              end;
          end
        else
          begin
             if (pd.typ=procdef) and
                assigned(tprocdef(pd).struct) and
                (pd.parast.symtablelevel=normal_function_level) then
              begin
                { static class methods have no hidden self/vmt pointer }
                if pd.no_self_node then
                   exit;

                storepos:=current_tokenpos;
                current_tokenpos:=tprocdef(pd).fileinfo;

                { Generate VMT variable for constructor/destructor }
                if (pd.proctypeoption in [potype_constructor,potype_destructor]) and
                   not(is_cppclass(tprocdef(pd).struct) or
                       is_record(tprocdef(pd).struct) or
                       is_javaclass(tprocdef(pd).struct) or
                       (
                         { no vmt for record/type helper constructors }
                         is_objectpascal_helper(tprocdef(pd).struct) and
                         (tobjectdef(tprocdef(pd).struct).extendeddef.typ<>objectdef)
                       )) then
                 begin
                   vs:=cparavarsym.create('$vmt',paranr_vmt,vs_value,cclassrefdef.create(tprocdef(pd).struct),[vo_is_vmt,vo_is_hidden_para]);
                   pd.parast.insert(vs);
                 end;

                { for helpers the type of Self is equivalent to the extended
                  type or equal to an instance of it }
                if is_objectpascal_helper(tprocdef(pd).struct) then
                  selfdef:=tobjectdef(tprocdef(pd).struct).extendeddef
                else if is_objccategory(tprocdef(pd).struct) then
                  selfdef:=tobjectdef(tprocdef(pd).struct).childof
                else
                  selfdef:=tprocdef(pd).struct;
                { Generate self variable, for classes we need
                  to use the generic voidpointer to be compatible with
                  methodpointers }
                vsp:=vs_value;
                if (po_staticmethod in pd.procoptions) or
                   (po_classmethod in pd.procoptions) then
                  hdef:=cclassrefdef.create(selfdef)
                else
                  begin
                    if is_object(selfdef) or (selfdef.typ<>objectdef) then
                      vsp:=vs_var;
                    hdef:=selfdef;
                  end;
                vs:=cparavarsym.create('$self',paranr_self,vsp,hdef,[vo_is_self,vo_is_hidden_para]);
                pd.parast.insert(vs);

                current_tokenpos:=storepos;
              end;
          end;
      end;


    procedure insert_funcret_local(pd:tprocdef);
      var
        storepos : tfileposinfo;
        vs       : tlocalvarsym;
        aliasvs  : tabsolutevarsym;
        sl       : tpropaccesslist;
        hs       : string;
      begin
        storepos:=current_tokenpos;
        current_tokenpos:=pd.fileinfo;

        { The result from constructors and destructors can't be accessed directly }
        if not(pd.proctypeoption in [potype_constructor,potype_destructor]) and
           not is_void(pd.returndef) and
           (not(po_assembler in pd.procoptions) or paramanager.asm_result_var(pd.returndef,pd)) then
         begin
           { We need to insert a varsym for the result in the localst
             when it is returning in a register }
           { we also need to do this for a generic procdef as we didn't allow
             the creation of a result symbol in insert_funcret_para, but we need
             a valid funcretsym }
           if (df_generic in pd.defoptions) or
               not paramanager.ret_in_param(pd.returndef,pd) then
            begin
              vs:=clocalvarsym.create('$result',vs_value,pd.returndef,[vo_is_funcret]);
              pd.localst.insert(vs);
              pd.funcretsym:=vs;
            end;

           { insert the name of the procedure as alias for the function result,
             we can't use realname because that will not work for compilerprocs
             as the name is lowercase and unreachable from the code }
           if (pd.proctypeoption<>potype_operator) or assigned(pd.resultname) then
             begin
               if assigned(pd.resultname) then
                 hs:=pd.resultname^
               else
                 hs:=pd.procsym.name;
               sl:=tpropaccesslist.create;
               sl.addsym(sl_load,pd.funcretsym);
               aliasvs:=cabsolutevarsym.create_ref(hs,pd.returndef,sl);
               include(aliasvs.varoptions,vo_is_funcret);
               tlocalsymtable(pd.localst).insert(aliasvs);
             end;

           { insert result also if support is on }
           if (m_result in current_settings.modeswitches) then
            begin
              sl:=tpropaccesslist.create;
              sl.addsym(sl_load,pd.funcretsym);
              aliasvs:=cabsolutevarsym.create_ref('RESULT',pd.returndef,sl);
              include(aliasvs.varoptions,vo_is_funcret);
              include(aliasvs.varoptions,vo_is_result);
              tlocalsymtable(pd.localst).insert(aliasvs);
            end;

         end;

        if pd.generate_safecall_wrapper then
          begin
            { vo_is_funcret is necessary so the local only gets freed after we loaded its
              value into the return register }
            vs:=clocalvarsym.create('$safecallresult',vs_value,search_system_type('HRESULT').typedef,[vo_is_funcret]);
            { do not put this variable in a register. The register which will be bound
              to this symbol will not be allocated automatically. Which means it will
              be re-used wich breaks the code. Besides this it is questionable if it is
              an optimization if one of the registers is kept allocated during the complete
              function, without ever using it.
              (It would be better to re-write the safecall-support in such a way that this
              variable it not needed at all, but that the HRESULT is set when the method
              is finalized) }
            vs.varregable:=vr_none;
            pd.localst.insert(vs);
          end;

        current_tokenpos:=storepos;
      end;


    procedure insert_hidden_para(p:TObject;arg:pointer);
      var
        hvs : tparavarsym;
        pd  : tabstractprocdef absolute arg;
      begin
        if (tsym(p).typ<>paravarsym) then
         exit;
        with tparavarsym(p) do
         begin
           { We need a local copy for a value parameter when only the
             address is pushed. Open arrays and Array of Const are
             an exception because they are allocated at runtime and the
             address that is pushed is patched.

             Arrays passed to cdecl routines are special: they are pointers in
             C and hence must be passed as such. Due to historical reasons, if
             a cdecl routine is implemented in Pascal, we still make a copy on
             the callee side. Do this the same on platforms that normally must
             make a copy on the caller side, as otherwise the behaviour will
             be different (and less perfomant) for routines implemented in C }
           if (varspez=vs_value) and
              paramanager.push_addr_param(varspez,vardef,pd.proccalloption) and
              not(is_open_array(vardef) or
                  is_array_of_const(vardef)) and
              (not(target_info.system in systems_caller_copy_addr_value_para) or
               ((pd.proccalloption in cdecl_pocalls) and
                (vardef.typ=arraydef))) then
             include(varoptions,vo_has_local_copy);

           { needs high parameter ? }
           if paramanager.push_high_param(varspez,vardef,pd.proccalloption) then
             begin
               hvs:=cparavarsym.create('$high'+name,paranr+1,vs_const,sizesinttype,[vo_is_high_para,vo_is_hidden_para]);
               hvs.symoptions:=[];
               owner.insert(hvs);
               { don't place to register if it will be accessed from implicit finally block }
               if (varspez=vs_value) and
                  is_open_array(vardef) and
                  is_managed_type(vardef) then
                 hvs.varregable:=vr_none;
             end
           else
            begin
              { Give a warning that cdecl routines does not include high()
                support }
              if (pd.proccalloption in cdecl_pocalls) and
                 paramanager.push_high_param(varspez,vardef,pocall_default) then
               begin
                 if is_open_string(vardef) then
                    MessagePos(fileinfo,parser_w_cdecl_no_openstring);
                 if not(po_external in pd.procoptions) and
                    (pd.typ<>procvardef) and
                    not is_objc_class_or_protocol(tprocdef(pd).struct) then
                   if is_array_of_const(vardef) then
                     MessagePos(fileinfo,parser_e_varargs_need_cdecl_and_external)
                   else
                     MessagePos(fileinfo,parser_w_cdecl_has_no_high);
               end;
              if (vardef.typ=formaldef) and (Tformaldef(vardef).typed) then
                begin
                  hvs:=cparavarsym.create('$typinfo'+name,paranr+1,vs_const,voidpointertype,
                                          [vo_is_typinfo_para,vo_is_hidden_para]);
                  owner.insert(hvs);
                end;
            end;
         end;
      end;


    procedure check_c_para(pd:Tabstractprocdef);
      var
        i,
        lastparaidx : longint;
        sym : TSym;
      begin
        lastparaidx:=pd.parast.SymList.Count-1;
        for i:=0 to pd.parast.SymList.Count-1 do
          begin
            sym:=tsym(pd.parast.SymList[i]);
            if (sym.typ=paravarsym) and
               (tparavarsym(sym).vardef.typ=arraydef) then
              begin
                if not is_variant_array(tparavarsym(sym).vardef) and
                   not is_array_of_const(tparavarsym(sym).vardef) and
                   (tparavarsym(sym).varspez<>vs_var) then
                  MessagePos(tparavarsym(sym).fileinfo,parser_h_c_arrays_are_references);
                if is_array_of_const(tparavarsym(sym).vardef) and
                   (i<lastparaidx) and
                   (tsym(pd.parast.SymList[i+1]).typ=paravarsym) and
                   not(vo_is_high_para in tparavarsym(pd.parast.SymList[i+1]).varoptions) then
                  MessagePos(tparavarsym(sym).fileinfo,parser_e_C_array_of_const_must_be_last);
              end;
          end;
      end;


    procedure insert_struct_hidden_paras(astruct: tabstractrecorddef);
      var
        pd: tdef;
        i: longint;
        oldpos: tfileposinfo;
      begin
        // handle calling conventions of record methods
        oldpos:=current_filepos;
        { don't keep track of procdefs in a separate list, because the
          compiler may add additional procdefs (e.g. property wrappers for
          the jvm backend) }
        for i := 0 to astruct.symtable.deflist.count - 1 do
          begin
            pd:=tdef(astruct.symtable.deflist[i]);
            if pd.typ<>procdef then
              continue;
            current_filepos:=tprocdef(pd).fileinfo;
            handle_calling_convention(tprocdef(pd),[hcc_declaration,hcc_insert_hidden_paras]);
          end;
        current_filepos:=oldpos;
      end;


    procedure set_addr_param_regable(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ<>paravarsym) then
         exit;
        with tparavarsym(p) do
         begin
           if (not needs_finalization) and
              paramanager.push_addr_param(varspez,vardef,tprocdef(arg).proccalloption) then
             varregable:=vr_addr;
         end;
      end;


    procedure handle_calling_convention(pd:tabstractprocdef;flags:thccflags);
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
              pocall_cppdecl,
              pocall_sysv_abi_cdecl,
              pocall_ms_abi_cdecl:
                begin
                  { check C cdecl para types }
                  check_c_para(pd);
                end;
              pocall_far16 :
                begin
                  { Temporary stub, must be rewritten to support OS/2 far16 }
                  Message1(parser_w_proc_directive_ignored,'FAR16');
                end;
              else
                ;
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
               if hcc_declaration in flags then
                begin
                  { if external is available, then cdecl must also be available,
                    procvars don't need external }
                  if not((po_external in pd.procoptions) or
                         (pd.typ=procvardef) or
                         { for objcclasses this is checked later, because the entire
                           class may be external.  }
                         is_objc_class_or_protocol(tprocdef(pd).struct)) and
                     not(pd.proccalloption in (cdecl_pocalls + [pocall_stdcall])) then
                    Message(parser_e_varargs_need_cdecl_and_external);
                end
               else
                begin
                  { both must be defined now }
                  if not((po_external in pd.procoptions) or
                         (pd.typ=procvardef)) or
                     not(pd.proccalloption in (cdecl_pocalls + [pocall_stdcall])) then
                    Message(parser_e_varargs_need_cdecl_and_external);
                end;
             end;
          end;

        if hcc_insert_hidden_paras in flags then
          begin
            { If the paraloc info has been calculated already, it will be missing for
              the new parameters we add below. This should never be necessary before
              we add them, as users of this information would not process these extra
              parameters in that case }
            if pd.has_paraloc_info<>callnoside then
              internalerror(2019031610);

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


    function proc_add_definition(var currpd:tprocdef):boolean;

      function check_generic_parameters(fwpd,currpd:tprocdef):boolean;
        var
          i : longint;
          fwtype,
          currtype : ttypesym;
        begin
          result:=true;
          if fwpd.genericparas.count<>currpd.genericparas.count then
            internalerror(2018090101);
          for i:=0 to fwpd.genericparas.count-1 do
            begin
              fwtype:=ttypesym(fwpd.genericparas[i]);
              currtype:=ttypesym(currpd.genericparas[i]);
              if fwtype.name<>currtype.name then
                begin
                  messagepos1(currtype.fileinfo,sym_e_generic_type_param_mismatch,currtype.realname);
                  messagepos1(fwtype.fileinfo,sym_e_generic_type_param_decl,fwtype.realname);
                  result:=false;
                end;
            end;
        end;


      function equal_generic_procdefs(fwpd,currpd:tprocdef):boolean;
        var
          i : longint;
          fwtype,
          currtype : ttypesym;
          foundretdef : boolean;
        begin
          result:=false;
          if fwpd.genericparas.count<>currpd.genericparas.count then
            exit;
          { comparing generic declarations is a bit more cumbersome as the
            defs of the generic parameter types are not equal, especially if the
            declaration contains constraints; essentially we have two cases:
            - proc declared in interface of unit (or in class/record/object)
              and defined in implementation; here the fwpd might contain
              constraints while currpd must only contain undefineddefs
            - forward declaration in implementation }
          foundretdef:=false;
          for i:=0 to fwpd.genericparas.count-1 do
            begin
              fwtype:=ttypesym(fwpd.genericparas[i]);
              currtype:=ttypesym(currpd.genericparas[i]);
              { if the type in the currpd isn't a pure undefineddef, then we can
                stop right there }
              if (currtype.typedef.typ<>undefineddef) or (df_genconstraint in currtype.typedef.defoptions) then
                exit;
              if not foundretdef then
                begin
                  { if the returndef is the same as this parameter's def then this
                    needs to be the case for both procdefs }
                  foundretdef:=fwpd.returndef=fwtype.typedef;
                  if foundretdef xor (currpd.returndef=currtype.typedef) then
                    exit;
                end;
            end;
          if compare_paras(fwpd.paras,currpd.paras,cp_none,[cpo_ignorehidden,cpo_openequalisexact,cpo_ignoreuniv,cpo_generic])<>te_exact then
            exit;
          if not foundretdef then
            begin
              if (df_specialization in tstoreddef(fwpd.returndef).defoptions) and (df_specialization in tstoreddef(currpd.returndef).defoptions) then
                { for specializations we're happy with equal defs instead of exactly the same defs }
                result:=equal_defs(fwpd.returndef,currpd.returndef)
              else
                { the returndef isn't a type parameter, so compare as usual }
                result:=compare_defs(fwpd.returndef,currpd.returndef,nothingn)=te_exact;
            end
          else
            result:=true;
        end;

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
        item : tlinkedlistitem;
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
              (
                fwpd.is_generic and
                currpd.is_generic and
                equal_generic_procdefs(fwpd,currpd)
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
                       (
                         (
                           fwpd.is_generic and
                           currpd.is_generic and
                           not equal_generic_procdefs(fwpd,currpd)
                         ) or
                         (
                           (
                             not fwpd.is_generic or
                             not currpd.is_generic
                           ) and
                           (
                             (compare_paras(fwpd.paras,currpd.paras,cp_all,paracompopt)<>te_exact) or
                             (compare_defs(fwpd.returndef,currpd.returndef,nothingn)<>te_exact)
                           )
                         )
                       ) then
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
                       (po_overload in currpd.procoptions)) or
                      { same with noreturn }
                      (not(po_noreturn in fwpd.procoptions) and
                       (po_noreturn in currpd.procoptions)) then
                     begin
                       MessagePos1(currpd.fileinfo,parser_e_header_dont_match_forward,
                                   fwpd.fullprocname(false));
                       tprocsym(fwpd.procsym).write_parameter_lists(fwpd);
                       { This error is non-fatal, we can recover }
                     end;

                   { Forward declaration is external? }
                   if (po_external in fwpd.procoptions) then
                     MessagePos(currpd.fileinfo,parser_e_proc_already_external);

                   { check for conflicts with "virtual" if this is a virtual
                     method, as "virtual" cannot be repeated in the
                     implementation and hence does not get checked against }
                   if (po_virtualmethod in fwpd.procoptions) then
                     begin
                       po_comp:=currpd.procoptions*PD_VIRTUAL_MUTEXCLPO;
                       if po_comp<>[] then
                         MessagePos2(currpd.fileinfo,parser_e_proc_dir_conflict,tokeninfo^[_VIRTUAL].str,get_first_proc_str(po_comp));
                     end;
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
                   { check that the type parameter names for generic methods match;
                     we check this here and not in equal_generic_procdefs as the defs
                     might still be different due to their parameters, so we'd generate
                     errors without any need }
                   if currpd.is_generic and fwpd.is_generic then
                     { an error here is recoverable, so we simply continue }
                     check_generic_parameters(fwpd,currpd);
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
                     symbolname to lowercase so users can' access it (can't do
                     it immediately, because then the implementation symbol
                     won't be matched) }
                   if po_compilerproc in fwpd.procoptions then
                     begin
                       fwpd.setcompilerprocname;
                       current_module.add_public_asmsym(fwpd.procsym.realname,AB_GLOBAL,AT_FUNCTION);
                     end;
                   if po_public in fwpd.procoptions then
                     begin
                       item:=fwpd.aliasnames.first;
                       while assigned(item) do
                         begin
                           current_module.add_public_asmsym(TCmdStrListItem(item).str,AB_GLOBAL,AT_FUNCTION);
                           item:=item.next;
                         end;
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
            if not currpd.forwarddef and (po_public in currpd.procoptions) then
              begin
                item:=currpd.aliasnames.first;
                while assigned(item) do
                  begin
                    current_module.add_public_asmsym(TCmdStrListItem(item).str,AB_GLOBAL,AT_FUNCTION);
                    item:=item.next;
                  end;
              end;
          end;

        proc_add_definition:=forwardfound;
      end;


    procedure build_parentfpstruct(pd: tprocdef);
      var
        nestedvars: tsym;
        nestedvarsst: tsymtable;
        pnestedvarsdef,
        nestedvarsdef: tdef;
        old_symtablestack: tsymtablestack;
      begin
        { make sure the defs are not registered in the current symtablestack,
          because they may be for a parent procdef (changeowner does remove a def
          from the symtable in which it was originally created, so that by itself
          is not enough) }
        old_symtablestack:=symtablestack;
        symtablestack:=old_symtablestack.getcopyuntil(current_module.localsymtable);
        { create struct to hold local variables and parameters that are
          accessed from within nested routines (start with extra dollar to prevent
          the JVM from thinking this is a nested class in the unit) }
        nestedvarsst:=trecordsymtable.create('$'+current_module.realmodulename^+'$$_fpc_nestedvars$'+pd.unique_id_str,
          current_settings.alignment.localalignmax,current_settings.alignment.localalignmin);
        nestedvarsdef:=crecorddef.create(nestedvarsst.name^,nestedvarsst);
  {$ifdef jvm}
        maybe_guarantee_record_typesym(nestedvarsdef,nestedvarsdef.owner);
        { don't add clone/FpcDeepCopy, because the field names are not all
          representable in source form and we don't need them anyway }
        symtablestack.push(trecorddef(nestedvarsdef).symtable);
        maybe_add_public_default_java_constructor(trecorddef(nestedvarsdef));
        insert_struct_hidden_paras(trecorddef(nestedvarsdef));
        symtablestack.pop(trecorddef(nestedvarsdef).symtable);
  {$endif}
        symtablestack.free;
        symtablestack:=old_symtablestack.getcopyuntil(pd.localst);
        pnestedvarsdef:=cpointerdef.getreusable(nestedvarsdef);
        if not(po_assembler in pd.procoptions) then
          begin
            nestedvars:=clocalvarsym.create('$nestedvars',vs_var,nestedvarsdef,[]);
            include(nestedvars.symoptions,sp_internal);
            pd.localst.insert(nestedvars);
            pd.parentfpstruct:=nestedvars;
            pd.parentfpinitblock:=cblocknode.create(nil);
          end;
        symtablestack.free;
        pd.parentfpstructptrtype:=pnestedvarsdef;

        symtablestack:=old_symtablestack;
      end;

end.
