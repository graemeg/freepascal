{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Routines for the code generation of data structures
    like VMT, Messages, VTables, Interfaces descs

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
unit nobj;

{$i fpcdefs.inc}

interface

    uses
       cutils,cclasses,
       globtype,
       symdef,symsym
       ;

    type
      TVMTBuilder=class
      private
        _Class : tobjectdef;
        handledprotocols: tfpobjectlist;
        function  is_new_vmt_entry(pd:tprocdef; out overridesclasshelper: boolean):boolean;
        procedure add_new_vmt_entry(pd:tprocdef; allowoverridingmethod: boolean);
        function  check_msg_str(vmtpd, pd: tprocdef):boolean;
        function  intf_search_procdef_by_name(proc: tprocdef;const name: string): tprocdef;
        procedure intf_get_procdefs(ImplIntf:TImplementedInterface;IntfDef:TObjectDef);
        procedure intf_get_procdefs_recursive(ImplIntf:TImplementedInterface;IntfDef:TObjectDef);
        procedure prot_get_procdefs_recursive(ImplProt:TImplementedInterface;ProtDef:TObjectDef);
        procedure intf_optimize_vtbls;
        procedure intf_allocate_vtbls;
        procedure generate_vmt_def;
      public
        constructor create(c:tobjectdef);
        procedure  generate_vmt;
        procedure  build_interface_mappings;
      end;


implementation

    uses
       globals,verbose,systems,
       node,
       symbase,symtable,symconst,symtype,symcpu,
       defcmp,
       pparautl;


{*****************************************************************************
                              TVMTBuilder
*****************************************************************************}

    constructor TVMTBuilder.create(c:tobjectdef);
      begin
        inherited Create;
        _Class:=c;
      end;


    procedure TVMTBuilder.add_new_vmt_entry(pd:tprocdef; allowoverridingmethod: boolean);
      var
        i : longint;
        vmtentry : pvmtentry;
        vmtpd : tprocdef;
      begin
        { new entry is needed, override was not possible }
        { Allowed when overriding a category method for a parent class in a
          descendent Objective-C class }
        if not allowoverridingmethod and
           (po_overridingmethod in pd.procoptions) then
          MessagePos1(pd.fileinfo,parser_e_nothing_to_be_overridden,pd.fullprocname(false));

        { check that all methods have overload directive }
        if not(m_fpc in current_settings.modeswitches) then
          begin
            for i:=0 to _class.vmtentries.count-1 do
              begin
                vmtentry:=pvmtentry(_class.vmtentries[i]);
                vmtpd:=tprocdef(vmtentry^.procdef);
                if (vmtpd.procsym=pd.procsym) and
                   (not(po_overload in pd.procoptions) or
                    not(po_overload in vmtpd.procoptions)) then
                  begin
                    MessagePos1(pd.fileinfo,parser_e_no_overload_for_all_procs,pd.procsym.realname);
                    { recover }
                    include(vmtpd.procoptions,po_overload);
                    include(pd.procoptions,po_overload);
                  end;
              end;
          end;

        { Register virtual method and give it a number }
        if (po_virtualmethod in pd.procoptions) then
          begin
             { store vmt entry number in procdef }
             if (pd.extnumber<>$ffff) and
                (pd.extnumber<>_class.VMTEntries.Count) then
               internalerror(200810283);
             pd.extnumber:=_class.VMTEntries.Count;
             new(vmtentry);
             vmtentry^.procdef:=pd;
             vmtentry^.procdefderef.reset;
             vmtentry^.visibility:=pd.visibility;
             _class.VMTEntries.Add(vmtentry);
          end;
      end;


      function TVMTBuilder.check_msg_str(vmtpd, pd: tprocdef): boolean;
        begin
          result:=true;
          if not(is_objc_class_or_protocol(_class)) then
            begin
              { the only requirement for normal methods is that both either
                have a message string or not (the value is irrelevant) }
              if ((pd.procoptions * [po_msgstr]) <> (vmtpd.procoptions * [po_msgstr])) then
                begin
                  MessagePos1(pd.fileinfo,parser_e_header_dont_match_forward,pd.fullprocname(false));
                  tprocsym(vmtpd.procsym).write_parameter_lists(pd);
                  result:=false;
                end
            end
          else
            begin
              { the compiler should have ensured that the protocol or parent
                class method has a message name specified }
              if not(po_msgstr in vmtpd.procoptions) then
                internalerror(2009070601);
              if not(po_msgstr in pd.procoptions) then
                begin
                  { copy the protocol's/parent class' message name to the one in
                    the class if none has been specified there }
                  include(pd.procoptions,po_msgstr);
                  pd.messageinf.str:=stringdup(vmtpd.messageinf.str^);
                end
              else
                begin
                  { if both have a message name, make sure they are equal }
                  if (vmtpd.messageinf.str^<>pd.messageinf.str^) then
                    begin
                      MessagePos2(pd.fileinfo,parser_e_objc_message_name_changed,vmtpd.messageinf.str^,pd.messageinf.str^);
                      result:=false;
                    end;
                end;
            end;
        end;


    function TVMTBuilder.is_new_vmt_entry(pd:tprocdef; out overridesclasshelper: boolean):boolean;
      const
        po_comp = [po_classmethod,po_virtualmethod,po_staticmethod,po_interrupt,po_iocheck,po_msgint,
                   po_exports,po_varargs,po_explicitparaloc,po_nostackframe];
      var
        i : longint;
        hasequalpara,
        hasoverloads,
        pdoverload : boolean;
        srsym : tsym;
        st : tsymtable;

      // returns true if we can stop checking, false if we have to continue
      function found_entry(var vmtpd: tprocdef; var vmtentryvis: tvisibility; updatevalues: boolean): boolean;
{$ifdef jvm}
        var
          javanewtreeok: boolean;
{$endif jvm}
        begin
          result:=false;

          { ignore hidden entries (e.g. virtual overridden by a static) that are not visible anymore }
          if vmtentryvis=vis_hidden then
            exit;

          { ignore different names }
          if vmtpd.procsym.name<>pd.procsym.name then
            exit;

          { hide private methods that are not visible anymore. For this check we
            must override the visibility with the highest value in the override chain.
            This is required for case (see tw3292) with protected-private-protected where the
            same vmtentry is used (PFV) }
          if not is_visible_for_object(vmtpd.owner,vmtentryvis,_class) then
            exit;

          { inherit overload }
          if (po_overload in vmtpd.procoptions) then
            begin
              include(pd.procoptions,po_overload);
              pdoverload:=true;
            end;

          { compare parameter types only, no specifiers yet }
          hasequalpara:=(compare_paras(vmtpd.paras,pd.paras,cp_none,[cpo_ignoreuniv,cpo_ignorehidden])>=te_equal);

          { check that we are not trying to override a final method }
          { in Java, new virtual inheritance trees can never be started ->
            treat all methods as "overriding" in the context of this check
            (Java does check whether the mangled names are identical, so if they
             are not we can stil get away with it) }
          if (po_finalmethod in vmtpd.procoptions) and
             hasequalpara and
             ((po_overridingmethod in pd.procoptions) or
              (is_javaclass(_class) and
               (pd.mangledname=vmtpd.mangledname))) and
             (is_class(_class) or is_objectpascal_helper(_class) or is_javaclass(_class)) then
            MessagePos1(pd.fileinfo,parser_e_final_can_no_be_overridden,pd.fullprocname(false))
          else
          { old definition has virtual
            new definition has no virtual or override }
          if (po_virtualmethod in vmtpd.procoptions) and
             (
              not(po_virtualmethod in pd.procoptions) or
              (
               { new one does not have reintroduce in case of an objccategory }
               (is_objccategory(_class) and
                 not(po_reintroduce in pd.procoptions)) or
               { new one does not have override in case of objpas/objc/java class/intf/proto }
               ((is_class_or_interface_or_objc_or_java(_class) or is_objectpascal_helper(_class)) and
                not is_objccategory(_class) and
                not(po_overridingmethod in pd.procoptions)
               )
              )
             ) then
            begin
              if (
                  not(pdoverload or hasoverloads) or
                  hasequalpara
                 ) then
                begin
{$ifdef jvm}
                  { if the mangled names are different, the inheritance trees
                    are different too in Java; exception: when the parent method
                    is a virtual class method or virtual constructor, because
                    those are looked up dynamicall by name }
                  javanewtreeok:=
                    is_java_class_or_interface(_class) and
                    (tcpuprocdef(pd).jvmmangledbasename(false)<>tcpuprocdef(vmtpd).jvmmangledbasename(false)) and
                    ((vmtpd.proctypeoption<>potype_constructor) and
                     not(po_staticmethod in vmtpd.procoptions));
{$endif}
                  if not(po_reintroduce in pd.procoptions) and
                     not(po_java_nonvirtual in vmtpd.procoptions) then
                    if not(is_objc_class_or_protocol(_class))
{$ifdef jvm}
                       and (not is_java_class_or_interface(_class) or
                        javanewtreeok)
{$endif jvm}
                       then
                      MessagePos1(pd.fileinfo,parser_w_should_use_override,pd.fullprocname(false))
                    else
                      begin
                        { In Objective-C, you cannot create a new VMT entry to
                          start a new inheritance tree. We therefore give an
                          error when the class is implemented in Pascal, to
                          avoid confusion due to things working differently
                          with Object Pascal classes.

                          In case of external classes, we only give a hint,
                          because requiring override everywhere may make
                          automated header translation tools too complex.

                          The same goes for Java. }
{$ifndef jvm}
                        if hasequalpara then
{$endif}
                          begin
                            if not(oo_is_external in _class.objectoptions) then
                              if not is_objccategory(_class) then
                                MessagePos1(pd.fileinfo,parser_e_must_use_override,FullTypeName(tdef(vmtpd.owner.defowner),nil))
                              else
                                MessagePos1(pd.fileinfo,parser_e_must_use_reintroduce_objc,FullTypeName(tdef(vmtpd.owner.defowner),nil))
                            { there may be a lot of these in auto-translated
                              headers, so only calculate the fulltypename if
                              the hint will be shown  }
                            else if CheckVerbosity(V_Hint) then
                              if not is_objccategory(_class) then
                                MessagePos1(pd.fileinfo,parser_h_should_use_override,FullTypeName(tdef(vmtpd.owner.defowner),nil))
                              else
                                MessagePos1(pd.fileinfo,parser_h_should_use_reintroduce_objc,FullTypeName(tdef(vmtpd.owner.defowner),nil));
                          end;
                        { no new entry, but copy the message name if any from
                          the procdef in the parent class }
                        if not is_objc_class_or_protocol(_class) or
                           hasequalpara then
                          check_msg_str(vmtpd,pd);
                        if updatevalues then
                          begin
                            { in case of Java, copy the real name from the parent,
                              since overriding "Destroy" with "destroy" is not
                              going to work very well }
                            if is_java_class_or_interface(_class) and
                               (pd.procsym.realname<>vmtpd.procsym.realname) then
                              pd.procsym.realname:=vmtpd.procsym.realname;
                            { in case we are overriding an abstract method,
                              decrease the number of abstract methods in this class }
                            if (po_abstractmethod in vmtpd.procoptions) then
                              dec(tobjectdef(pd.owner.defowner).abstractcnt);
                            if (vmtpd.extnumber<>i) then
                              internalerror(2011083101);
                            pd.extnumber:=vmtpd.extnumber;
                            vmtpd:=pd;
                          end;
                        result:=true;
                        exit;
{$ifdef jvm}
                      end
                  else
                    if not javanewtreeok and
                       is_java_class_or_interface(_class) then
                      begin
                        { mangled names are the same -> can only override }
                        MessagePos1(pd.fileinfo,parser_e_must_use_override,FullTypeName(tdef(vmtpd.owner.defowner),nil))
{$endif jvm}
                      end;
                  { disable/hide old VMT entry }
                  if updatevalues then
                    vmtentryvis:=vis_hidden;
                end;
            end
          { both are virtual? }
          else if (po_virtualmethod in pd.procoptions) and
                  (po_virtualmethod in vmtpd.procoptions) then
            begin
              { same parameter and return types (parameter specifiers will be checked below) }
              if hasequalpara and
                 compatible_childmethod_resultdef(vmtpd.returndef,pd.returndef) then
                begin
                  { inherite calling convention when it was explicit and the
                    current definition has none explicit set }
                  if (po_hascallingconvention in vmtpd.procoptions) and
                     not(po_hascallingconvention in pd.procoptions) then
                    begin
                      pd.proccalloption:=vmtpd.proccalloption;
                      include(pd.procoptions,po_hascallingconvention);
                    end;

                  { All parameter specifiers and some procedure the flags have to match
                    except abstract and override }
                  if (compare_paras(vmtpd.paras,pd.paras,cp_all,[cpo_ignoreuniv,cpo_ignorehidden])<te_equal) or
                     (vmtpd.proccalloption<>pd.proccalloption) or
                     (vmtpd.proctypeoption<>pd.proctypeoption) or
                     ((vmtpd.procoptions*po_comp)<>(pd.procoptions*po_comp)) then
                     begin
                       MessagePos1(pd.fileinfo,parser_e_header_dont_match_forward,pd.fullprocname(false));
                       tprocsym(vmtpd.procsym).write_parameter_lists(pd);
                     end;

                  check_msg_str(vmtpd,pd);

                  { Give a note if the new visibility is lower. For a higher
                    visibility update the vmt info }
                  if vmtentryvis>pd.visibility then
                    begin
                      if po_auto_raised_visibility in vmtpd.procoptions then
                        begin
                          if updatevalues then
                            begin
                              pd.visibility:=vmtentryvis;
                              { this one's visibility is now also auto-raised }
                              include(pd.procoptions,po_auto_raised_visibility);
                            end
                        end
                      else
{$ifdef jvm}
                        MessagePos4(pd.fileinfo,parser_e_method_lower_visibility,
{$else jvm}
                        MessagePos4(pd.fileinfo,parser_n_ignore_lower_visibility,
{$endif jvm}
                          pd.fullprocname(false),
                          visibilityname[pd.visibility],tobjectdef(vmtpd.owner.defowner).objrealname^,visibilityname[vmtentryvis])
                      end
                  else if pd.visibility>vmtentryvis then
                    begin
                      if updatevalues then
                        vmtentryvis:=pd.visibility;
                    end;

                  { override old virtual method in VMT }
                  if updatevalues then
                    begin
                      { in case we are overriding an abstract method,
                        decrease the number of abstract methods in this class }
                      if (po_overridingmethod in pd.procoptions) and
                         (po_abstractmethod in vmtpd.procoptions) then
                        dec(tobjectdef(pd.owner.defowner).abstractcnt);

                      if (vmtpd.extnumber<>i) then
                        internalerror(200611084);
                      pd.extnumber:=vmtpd.extnumber;
                      { in case of Java, copy the real name from the parent,
                        since overriding "Destroy" with "destroy" is not
                        going to work very well }
                      if is_java_class_or_interface(_class) and
                         (pd.procsym.realname<>vmtpd.procsym.realname) then
                        pd.procsym.realname:=vmtpd.procsym.realname;
                      vmtpd:=pd;
                    end;
                  result:=true;
                  exit;
                end
              { different parameters }
              else
               begin
                 { when we got an override directive then can search futher for
                   the procedure to override.
                   If we are starting a new virtual tree then hide the old tree }
                 if not(po_overridingmethod in pd.procoptions) and
                    not(pdoverload or hasoverloads) then
                   begin
                     if not(po_reintroduce in pd.procoptions) then
                       begin
                         if not is_object(_class) and
                            not is_objc_class_or_protocol(_class) and
                            not is_java_class_or_interface(_class) then
                           MessagePos1(pd.fileinfo,parser_w_should_use_override,pd.fullprocname(false))
                         else
                           { objects don't allow starting a new virtual tree
                             and neither do Objective-C or Java }
                           MessagePos1(pd.fileinfo,parser_e_header_dont_match_forward,vmtpd.fullprocname(false));
                       end;
                     { disable/hide old VMT entry }
                     if updatevalues then
                       vmtentryvis:=vis_hidden;
                   end;
               end;
            end;
        end;

        function found_category_method(st: tsymtable): boolean;
          var
            entrycount: longint;
            cat: tobjectdef;
            vmtpd: tprocdef;
            vmtvis: tvisibility;
          begin
            result:=false;
            if is_objccategory(tdef(st.defowner)) then
              begin
                cat:=tobjectdef(st.defowner);
                { go through all of the category's methods to find the
                  vmtentry corresponding to the procdef we are handling }
                for entrycount:=0 to cat.vmtentries.Count-1 do
                  begin
                    vmtpd:=pvmtentry(cat.vmtentries[entrycount])^.procdef;
                    vmtvis:=pvmtentry(cat.vmtentries[entrycount])^.visibility;
                    { don't change the vmtentry of the category }
                    if found_entry(vmtpd,vmtvis,false) then
                      begin
                        result:=true;
                        exit;
                      end;
                  end;
              end;
          end;

      begin
        result:=false;
        overridesclasshelper:=false;
        { Load other values for easier readability }
        hasoverloads:=(tprocsym(pd.procsym).ProcdefList.Count>1);
        pdoverload:=(po_overload in pd.procoptions);

        { compare with all stored definitions }
        for i:=0 to _class.vmtentries.Count-1 do
          begin
            if found_entry(pvmtentry(_class.vmtentries[i])^.procdef, pvmtentry(_class.vmtentries[i])^.visibility,true) then
              exit;
          end;

        { in case of Objective-C, also check the categories that apply to this
          class' *parent* for methods to override (don't allow class X to
          "override" a method added by a category to class X itself, since in
          that case the category method will in fact replace class X'
          "overriding" method }
        if is_objcclass(_class) and
           assigned(_class.childof) and
           search_objc_helper(_class.childof,pd.procsym.name,srsym,st) then
          begin
            overridesclasshelper:=found_category_method(st);
          end;

        { No entry found, we need to create a new entry }
        result:=true;
      end;


    function TVMTBuilder.intf_search_procdef_by_name(proc: tprocdef;const name: string): tprocdef;
      const
        po_comp = [po_classmethod,po_staticmethod,po_interrupt,po_iocheck,po_msgint,
                   po_exports,po_varargs,po_explicitparaloc,po_nostackframe];
      var
        implprocdef : Tprocdef;
        i: cardinal;
        hclass : tobjectdef;
        hashedid : THashedIDString;
        srsym      : tsym;
        overload: boolean;
      begin
        result:=nil;
        hashedid.id:=name;
        hclass:=_class;
        while assigned(hclass) do
          begin
            srsym:=tsym(hclass.symtable.FindWithHash(hashedid));
            if assigned(srsym) and
               (srsym.typ=procsym) and
               ((hclass=_class) or
                is_visible_for_object(srsym,_class)) then
              begin
                overload:=false;
                for i:=0 to Tprocsym(srsym).ProcdefList.Count-1 do
                  begin
                    implprocdef:=tprocdef(tprocsym(srsym).ProcdefList[i]);
                    if po_overload in implprocdef.procoptions then
                      overload:=true;
                    if (implprocdef.procsym=tprocsym(srsym)) and
                       (compare_paras(proc.paras,implprocdef.paras,cp_all,[cpo_ignorehidden,cpo_ignoreuniv])>=te_equal) and
                       (compare_defs(proc.returndef,implprocdef.returndef,nothingn)>=te_equal) and
                       (proc.proccalloption=implprocdef.proccalloption) and
                       (proc.proctypeoption=implprocdef.proctypeoption) and
                       ((proc.procoptions*po_comp)=((implprocdef.procoptions+[po_virtualmethod])*po_comp)) and
                       check_msg_str(proc,implprocdef) then
                      begin
                        { does the interface increase the visibility of the
                          implementing method? }
                        if implprocdef.visibility<proc.visibility then
{$ifdef jvm}
                          MessagePos2(implprocdef.fileinfo,type_e_interface_lower_visibility,proc.fullprocname(false),implprocdef.fullprocname(false));
{$else}
                          MessagePos2(implprocdef.fileinfo,type_w_interface_lower_visibility,proc.fullprocname(false),implprocdef.fullprocname(false));
{$endif}
                        result:=implprocdef;
                        addsymref(result.procsym,result);
                        exit;
                      end;
                  end;
                { like with normal procdef resolution (in htypechk), stop if
                  we encounter a proc without the overload directive }
                if not overload then
                  exit;
              end;
            hclass:=hclass.childof;
          end;
      end;


    procedure TVMTBuilder.intf_get_procdefs(ImplIntf:TImplementedInterface;IntfDef:TObjectDef);
      var
        i   : longint;
        def : tdef;
        hs,
        prefix,
        mappedname: string;
        implprocdef: tprocdef;
      begin
        prefix:=ImplIntf.IntfDef.symtable.name^+'.';
        for i:=0 to IntfDef.symtable.DefList.Count-1 do
          begin
            def:=tdef(IntfDef.symtable.DefList[i]);
            if assigned(def) and
               (def.typ=procdef) then
              begin
                { Find implementing procdef
                   1. Check for mapped name
                   2. Use symbol name, but only if there's no mapping,
                      or we're processing ancestor of interface.
                  When modifying this code, ensure that webtbs/tw11862, webtbs/tw4950
                  and webtbf/tw19591 stay correct. }
                implprocdef:=nil;
                hs:=prefix+tprocdef(def).procsym.name;
                mappedname:=ImplIntf.GetMapping(hs);
                if mappedname<>'' then
                  implprocdef:=intf_search_procdef_by_name(tprocdef(def),mappedname);
                if not assigned(implprocdef) then
                  if (mappedname='') or (ImplIntf.IntfDef<>IntfDef) then
                    implprocdef:=intf_search_procdef_by_name(tprocdef(def),tprocdef(def).procsym.name);

                { Add procdef to the implemented interface }
                if assigned(implprocdef) then
                  begin
                    if (tobjectdef(implprocdef.struct).objecttype<>odt_objcclass) then
                      begin
                        { in case of Java, copy the real name from the parent,
                          since overriding "Destroy" with "destroy" is not
                          going to work very well }
                        if is_javaclass(implprocdef.struct) and
                           (implprocdef.procsym.realname<>tprocdef(def).procsym.realname) then
                          implprocdef.procsym.realname:=tprocdef(def).procsym.realname;
                        ImplIntf.AddImplProc(implprocdef);
                      end
                    else
                      begin
                        { If no message name has been specified for the method
                          in the objcclass, copy it from the protocol
                          definition.  }
                        if not(po_msgstr in tprocdef(def).procoptions) then
                          begin
                            include(tprocdef(def).procoptions,po_msgstr);
                            implprocdef.messageinf.str:=stringdup(tprocdef(def).messageinf.str^);
                          end
                        else
                          begin
                            { If a message name has been specified in the
                              objcclass, it has to match the message name in the
                              protocol definition.  }
                            if (implprocdef.messageinf.str^<>tprocdef(def).messageinf.str^) then
                              MessagePos2(implprocdef.fileinfo,parser_e_objc_message_name_changed,tprocdef(def).messageinf.str^,implprocdef.messageinf.str^);
                          end;
                      end;
                  end
                else
                  if (ImplIntf.IType=etStandard) and
                     not(po_optional in tprocdef(def).procoptions) then
                    MessagePos1(_Class.typesym.fileinfo,sym_e_no_matching_implementation_found,tprocdef(def).fullprocname(false));
              end;
          end;
      end;


    procedure TVMTBuilder.intf_get_procdefs_recursive(ImplIntf:TImplementedInterface;IntfDef:TObjectDef);
      begin
        if assigned(IntfDef.childof) then
          intf_get_procdefs_recursive(ImplIntf,IntfDef.childof);
        intf_get_procdefs(ImplIntf,IntfDef);
      end;


    procedure TVMTBuilder.prot_get_procdefs_recursive(ImplProt:TImplementedInterface;ProtDef:TObjectDef);
      var
        i: longint;
      begin
        { don't check the same protocol twice }
        if handledprotocols.IndexOf(ProtDef)<>-1 then
          exit;
        handledprotocols.add(ProtDef);
        for i:=0 to ProtDef.ImplementedInterfaces.count-1 do
          prot_get_procdefs_recursive(ImplProt,TImplementedInterface(ProtDef.ImplementedInterfaces[i]).intfdef);
        intf_get_procdefs(ImplProt,ProtDef);
      end;


    procedure TVMTBuilder.intf_optimize_vtbls;
      type
        tcompintfentry = record
          weight: longint;
          compintf: longint;
        end;
        { Max 1000 interface in the class header interfaces it's enough imho }
        tcompintfs = array[0..1000] of tcompintfentry;
        pcompintfs = ^tcompintfs;
        tequals    = array[0..1000] of longint;
        pequals    = ^tequals;
        timpls    = array[0..1000] of longint;
        pimpls    = ^timpls;
      var
        aequals: pequals;
        compats: pcompintfs;
        impls: pimpls;
        ImplIntfCount,
        w,i,j,k: longint;
        ImplIntfI,
        ImplIntfJ  : TImplementedInterface;
        cij: boolean;
        cji: boolean;
      begin
        ImplIntfCount:=_class.ImplementedInterfaces.count;
        if ImplIntfCount>=High(tequals) then
          Internalerror(200006135);
        getmem(compats,sizeof(tcompintfentry)*ImplIntfCount);
        getmem(aequals,sizeof(longint)*ImplIntfCount);
        getmem(impls,sizeof(longint)*ImplIntfCount);
        filldword(compats^,(sizeof(tcompintfentry) div sizeof(dword))*ImplIntfCount,dword(-1));
        filldword(aequals^,ImplIntfCount,dword(-1));
        filldword(impls^,ImplIntfCount,dword(-1));
        { ismergepossible is a containing relation
          meaning of ismergepossible(a,b,w) =
          if implementorfunction map of a is contained implementorfunction map of b
          imp(a,b) and imp(b,c) => imp(a,c) ; imp(a,b) and imp(b,a) => a == b
        }
        { the order is very important for correct allocation }
        for i:=0 to ImplIntfCount-1 do
          begin
            for j:=i+1 to ImplIntfCount-1 do
              begin
                ImplIntfI:=TImplementedInterface(_class.ImplementedInterfaces[i]);
                ImplIntfJ:=TImplementedInterface(_class.ImplementedInterfaces[j]);
                cij:=ImplIntfI.IsImplMergePossible(ImplIntfJ,w);
                cji:=ImplIntfJ.IsImplMergePossible(ImplIntfI,w);
                if cij and cji then { i equal j }
                  begin
                    { get minimum index of equal }
                    if aequals^[j]=-1 then
                      aequals^[j]:=i;
                  end
                else if cij then
                  begin
                    { get minimum index of maximum weight  }
                    if compats^[i].weight<w then
                      begin
                        compats^[i].weight:=w;
                        compats^[i].compintf:=j;
                      end;
                  end
                else if cji then
                  begin
                    { get minimum index of maximum weight  }
                    if (compats^[j].weight<w) then
                      begin
                        compats^[j].weight:=w;
                        compats^[j].compintf:=i;
                      end;
                  end;
              end;
          end;
        { Reset, no replacements by default }
        for i:=0 to ImplIntfCount-1 do
          impls^[i]:=i;
        { Replace vtbls when equal or compat, repeat
          until there are no replacements possible anymore. This is
          needed for the cases like:
            First loop: 2->3, 3->1
            Second loop: 2->1 (because 3 was replaced with 1)
        }
        repeat
          k:=0;
          for i:=0 to ImplIntfCount-1 do
            begin
              if compats^[impls^[i]].compintf<>-1 then
                impls^[i]:=compats^[impls^[i]].compintf
              else if aequals^[impls^[i]]<>-1 then
                impls^[i]:=aequals^[impls^[i]]
              else
                inc(k);
            end;
        until k=ImplIntfCount;
        { Update the VtblImplIntf }
        for i:=0 to ImplIntfCount-1 do
          begin
            ImplIntfI:=TImplementedInterface(_class.ImplementedInterfaces[i]);
            ImplIntfI.VtblImplIntf:=TImplementedInterface(_class.ImplementedInterfaces[impls^[i]]);
          end;
        freemem(compats);
        freemem(aequals);
        freemem(impls);
      end;


    procedure TVMTBuilder.intf_allocate_vtbls;
      var
        i : longint;
        ImplIntf : TImplementedInterface;
      begin
        { Allocation vtbl space }
        for i:=0 to _class.ImplementedInterfaces.count-1 do
          begin
            ImplIntf:=TImplementedInterface(_class.ImplementedInterfaces[i]);
            { if it implements itself and if it's not implemented by delegation }
            if (ImplIntf.VtblImplIntf=ImplIntf) and (ImplIntf.IType=etStandard) then
              begin
                { allocate a pointer in the object memory }
                with tObjectSymtable(_class.symtable) do
                  begin
                    datasize:=align(datasize,voidpointertype.alignment);
                    ImplIntf.Ioffset:=datasize;
                    datasize:=datasize+voidpointertype.size;
                  end;
              end;
          end;
        { Update ioffset of current interface with the ioffset from
          the interface that is reused to implements this interface }
        for i:=0 to _class.ImplementedInterfaces.count-1 do
          begin
            ImplIntf:=TImplementedInterface(_class.ImplementedInterfaces[i]);
            if ImplIntf.VtblImplIntf<>ImplIntf then
              ImplIntf.IOffset:=ImplIntf.VtblImplIntf.IOffset;
          end;
      end;


    procedure TVMTBuilder.generate_vmt_def;
      var
        i: longint;
        vmtdef: trecorddef;
        systemvmt: tdef;
        sym: tsym;
        vmtdefname: TIDString;
      begin
        { these types don't have an actual VMT, we only use the other methods
          in TVMTBuilder to determine duplicates/overrides }
        if _class.objecttype in [
           odt_helper,
           odt_objcclass,
           odt_objccategory,
           odt_objcprotocol,
           odt_javaclass,
           odt_interfacecom_property,
           odt_interfacecom_function,
           odt_interfacejava] then
         exit;

        { don't generate VMT for generics (only use duplicates/overrides detection) }
        { Note: don't use is_generic here as we also need to check nested non-
                generic classes }
        if df_generic in _class.defoptions then
          exit;

        { todo in the future }
        if _class.objecttype = odt_cppclass then
          exit;

        { the VMT definition may already exist in case of generics }
        vmtdefname:=internaltypeprefixName[itp_vmtdef]+_class.mangledparaname;
        if assigned(try_search_current_module_type(vmtdefname)) then
          exit;
        { create VMT type definition }
        vmtdef:=crecorddef.create_global_internal(
          vmtdefname,
          0,
          target_info.alignment.recordalignmin);
{$ifdef llvm}
        { in case of a class declared in the implementation section of unit
          whose method is called from an inline routine -- LLVM needs to be able
          to access the vmt def to create signatures }
        vmtdef.register_def;
{$endif}
        { standard VMT fields }
        case _Class.objecttype of
          odt_class:
            begin
              systemvmt:=search_system_type('TVMT').typedef;
              if not assigned(systemvmt) then
                Message1(cg_f_internal_type_not_found,'TVMT');

              { does the TVMT type look like we expect? (so that this code is
                easily triggered in case the definition of the VMT would
                change) }
              if (systemvmt.typ<>recorddef) or
                 (trecorddef(systemvmt).symtable.SymList.count<>27) then
                Message1(cg_f_internal_type_does_not_match,'TVMT');
              { system.tvmt is a record that represents the VMT of TObject,
                including its virtual methods. We only want the non-method
                fields, as the methods will be added automatically based on
                the VMT we generated here only add the 12 first fields }
              for i:=0 to 11 do
                begin
                  sym:=tsym(trecorddef(systemvmt).symtable.SymList[i]);
                  if sym.typ in [procsym,propertysym] then
                    continue;
                  if sym.typ<>fieldvarsym then
                    internalerror(2015052602);
                  vmtdef.add_field_by_def('',tfieldvarsym(sym).vardef);
                end;
            end;
           odt_interfacecom,odt_interfacecorba,odt_dispinterface:
             { nothing }
             ;
          odt_object:
            begin
              { size, -size, parent vmt [, dmt ] (same names as for class) }
              vmtdef.add_field_by_def('vInstanceSize',sizesinttype);
              vmtdef.add_field_by_def('vInstanceSize2',sizesinttype);
              vmtdef.add_field_by_def('vParent',voidpointertype);
{$ifdef WITHDMT}
              vmtdef.add_field_by_def('',voidpointertype);
{$endif WITHDMT}
            end;
          else
            internalerror(2015052605);
        end;

        { now add the methods }
        for i:=0 to _class.vmtentries.count-1 do
          vmtdef.add_field_by_def('',
            cprocvardef.getreusableprocaddr(pvmtentry(_class.vmtentries[i])^.procdef)
          );
        { the VMT ends with a nil pointer }
        vmtdef.add_field_by_def('',voidcodepointertype);
      end;


    procedure TVMTBuilder.generate_vmt;
      var
        i : longint;
        def : tdef;
        old_current_structdef : tabstractrecorddef;
        overridesclasshelper : boolean;
      begin
        old_current_structdef:=current_structdef;
        current_structdef:=_class;

        _class.resetvmtentries;

        { inherit (copy) VMT from parent object }
        if assigned(_class.childof) then
          begin
            if not assigned(_class.childof.vmtentries) then
              internalerror(200810281);
            _class.copyvmtentries(_class.childof);
          end;

        { process all procdefs, we must process the defs to
          keep the same order as that is written in the source
          to be compatible with the indexes in the interface vtable (PFV) }
        for i:=0 to _class.symtable.DefList.Count-1 do
          begin
            def:=tdef(_class.symtable.DefList[i]);
            if def.typ=procdef then
              begin
                { VMT entry }
                if is_new_vmt_entry(tprocdef(def),overridesclasshelper) then
                  add_new_vmt_entry(tprocdef(def),overridesclasshelper);
              end;
          end;
        insert_struct_hidden_paras(_class);
        build_interface_mappings;
        if assigned(_class.ImplementedInterfaces) and
           not(is_objc_class_or_protocol(_class)) and
           not(is_java_class_or_interface(_class)) then
          begin
            { Optimize interface tables to reuse wrappers }
            intf_optimize_vtbls;
            { Allocate interface tables }
            intf_allocate_vtbls;
          end;
        generate_vmt_def;
        current_structdef:=old_current_structdef;
      end;


    procedure TVMTBuilder.build_interface_mappings;
      var
        ImplIntf : TImplementedInterface;
        i: longint;
      begin
        { Find Procdefs implementing the interfaces (both Objective-C protocols
          and Java interfaces can have multiple parent interfaces, but in that
          case obviously no implementations are required) }
        if assigned(_class.ImplementedInterfaces) and
           not(_class.objecttype in [odt_objcprotocol,odt_interfacejava]) and
           // abstract java classes do not have to implement all interface
           // methods. todo: check that non-abstract descendents do!
           not((_class.objecttype=odt_javaclass) and (oo_is_abstract in _class.objectoptions)) then
          begin
            { Collect implementor functions into the tImplementedInterface.procdefs }
            case _class.objecttype of
              odt_class:
                begin
                  for i:=0 to _class.ImplementedInterfaces.count-1 do
                    begin
                      ImplIntf:=TImplementedInterface(_class.ImplementedInterfaces[i]);
                      intf_get_procdefs_recursive(ImplIntf,ImplIntf.IntfDef)
                    end;
                end;
              odt_objcclass,
              odt_javaclass:
                begin
                  { Object Pascal interfaces are afterwards optimized via the
                    intf_optimize_vtbls() method, but we can't do this for
                    protocols/Java interfaces -> check for duplicates here
                    already. }
                  handledprotocols:=tfpobjectlist.create(false);
                  for i:=0 to _class.ImplementedInterfaces.count-1 do
                    begin
                      ImplIntf:=TImplementedInterface(_class.ImplementedInterfaces[i]);
                      prot_get_procdefs_recursive(ImplIntf,ImplIntf.IntfDef);
                    end;
                  handledprotocols.free;
                end
              else
                internalerror(2009091801);
            end
          end;
      end;

end.
