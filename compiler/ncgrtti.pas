{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Routines for the code generation of RTTI data structures

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
unit ncgrtti;

{$i fpcdefs.inc}

interface

    uses
      cclasses,constexp,globtype,
      aasmbase,aasmcnst,
      symbase,symconst,symtype,symdef,symsym,
      parabase;

    type

      { TRTTIWriter }

      TRTTIWriter=class
      private
        { required internal alignment of the rtti data }
        reqalign: shortint;
        { required packing of all structures except for ttypeinfo and tpropinfo,
          which always use packrecords 1 }
        defaultpacking: shortint;

        procedure fields_write_rtti(st:tsymtable;rt:trttitype);
        procedure params_write_rtti(def:tabstractprocdef;rt:trttitype;allow_hidden:boolean);
        procedure fields_write_rtti_data(tcb: ttai_typedconstbuilder; def: tabstractrecorddef; rt: trttitype);
        procedure methods_write_rtti(st:tsymtable;rt:trttitype;visibilities:tvisibilities;allow_hidden:boolean);
        procedure write_rtti_extrasyms(def:Tdef;rt:Trttitype;mainrtti:Tasmsymbol);
        procedure published_write_rtti(st:tsymtable;rt:trttitype);
        function  published_properties_count(st:tsymtable):longint;
        procedure published_properties_write_rtti_data(tcb: ttai_typedconstbuilder; propnamelist: TFPHashObjectList; st: tsymtable);
        procedure collect_propnamelist(propnamelist:TFPHashObjectList;objdef:tobjectdef);
        { only use a direct reference if the referenced type can *only* reside
          in the same unit as the current one }
        function ref_rtti(def:tdef;rt:trttitype;indirect:boolean;suffix:tsymstr):tasmsymbol;
        procedure write_rtti_name(tcb: ttai_typedconstbuilder; def: tdef);
        procedure write_common_rtti_data(tcb:ttai_typedconstbuilder;def:tdef;rt:trttitype);
        procedure write_rtti_data(tcb: ttai_typedconstbuilder; def:tdef; rt: trttitype);
        procedure write_attribute_data(tcb: ttai_typedconstbuilder;attr_list:trtti_attribute_list);
        procedure write_child_rtti_data(def:tdef;rt:trttitype);
        procedure write_rtti_reference(tcb: ttai_typedconstbuilder; def: tdef; rt: trttitype);
        procedure write_methods(tcb:ttai_typedconstbuilder;st:tsymtable;visibilities:tvisibilities);
        procedure write_header(tcb: ttai_typedconstbuilder; def: tdef; typekind: byte);
        function write_methodkind(tcb:ttai_typedconstbuilder;def:tabstractprocdef):byte;
        procedure write_callconv(tcb:ttai_typedconstbuilder;def:tabstractprocdef);
        procedure write_paralocs(tcb:ttai_typedconstbuilder;para:pcgpara);
        procedure write_param_flag(tcb:ttai_typedconstbuilder;parasym:tparavarsym);
        procedure write_mop_offset_table(tcb:ttai_typedconstbuilder;def:tabstractrecorddef;mop:tmanagementoperator);
      public
        constructor create;
        procedure write_rtti(def:tdef;rt:trttitype);
        function  get_rtti_label(def:tdef;rt:trttitype;indirect:boolean):tasmsymbol; inline;
        function  get_rtti_label_ord2str(def:tdef;rt:trttitype;indirect:boolean):tasmsymbol; inline;
        function  get_rtti_label_str2ord(def:tdef;rt:trttitype;indirect:boolean):tasmsymbol; inline;
      end;

    { generate RTTI and init tables }
    procedure write_persistent_type_info(st:tsymtable;is_global:boolean);

    var
      RTTIWriter : TRTTIWriter;


implementation

    uses
       cutils,
       globals,verbose,systems,
       node,ncal,ncon,
       fmodule, procinfo,
       symtable,symutil,
       aasmtai,aasmdata,
       defutil,
       paramgr
       ;


    const
       rttidefstate : array[trttitype] of tdefstate =
         (ds_rtti_table_written,ds_init_table_written,
         { Objective-C related, does not pass here }
         symconst.ds_none,symconst.ds_none,
         symconst.ds_none,symconst.ds_none);

    type
       TPropNameListItem = class(TFPHashObject)
         propindex : longint;
         propowner : TSymtable;
       end;


    procedure write_persistent_type_info(st: tsymtable; is_global: boolean);
      var
        i : longint;
        def : tdef;
      begin
        { no Delphi-style RTTI for managed platforms }
        if target_info.system in systems_managed_vm then
          exit;
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            { skip generics }
            if [df_generic,df_genconstraint]*def.defoptions<>[] then
              continue;
            case def.typ of
              recorddef:
                write_persistent_type_info(trecorddef(def).symtable,is_global);
              objectdef :
                begin
                  { Skip forward defs }
                  if (oo_is_forward in tobjectdef(def).objectoptions) then
                    continue;
                  write_persistent_type_info(tobjectdef(def).symtable,is_global);
                end;
              procdef :
                begin
                  if assigned(tprocdef(def).localst) and
                     (tprocdef(def).localst.symtabletype=localsymtable) then
                    write_persistent_type_info(tprocdef(def).localst,false);
                  if assigned(tprocdef(def).parast) then
                    write_persistent_type_info(tprocdef(def).parast,false);
                end;
              errordef:
                { we shouldn't have come this far if we have an errordef somewhere }
                internalerror(2017010701);
              undefineddef:
                { don't write any RTTI for these }
                continue;
              else
                ;
            end;
            { always generate persistent tables for types in the interface so
              they can be reused in other units and give always the same pointer
              location. }
            { Init }
            if (
                assigned(def.typesym) and
                is_global and
                not is_objc_class_or_protocol(def)
               ) or
               is_managed_type(def) or
               (ds_init_table_used in def.defstates) then
              RTTIWriter.write_rtti(def,initrtti);
            { RTTI }
            if (
                assigned(def.typesym) and
                is_global and
                not is_objc_class_or_protocol(def)
               ) or
               (ds_rtti_table_used in def.defstates) then
              RTTIWriter.write_rtti(def,fullrtti);
          end;
      end;


{***************************************************************************
                              TRTTIWriter
***************************************************************************}

    function TRTTIWriter.get_rtti_label(def:tdef;rt:trttitype;indirect:boolean):tasmsymbol;
      begin
        result:=ref_rtti(def,rt,indirect,'');
      end;

    function TRTTIWriter.get_rtti_label_ord2str(def:tdef;rt:trttitype;indirect:boolean):tasmsymbol;
      begin
        result:=ref_rtti(def,rt,indirect,'_o2s');
      end;

    function TRTTIWriter.get_rtti_label_str2ord(def:tdef;rt:trttitype;indirect:boolean):tasmsymbol;
      begin
        result:=ref_rtti(def,rt,indirect,'_s2o');
      end;

    procedure TRTTIWriter.write_methods(tcb:ttai_typedconstbuilder;st:tsymtable;visibilities:tvisibilities);
      var
        rtticount,
        totalcount,
        i,j,k : longint;
        sym : tprocsym;
        def : tprocdef;
        para : tparavarsym;
      begin
        tcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
          targetinfos[target_info.system]^.alignment.recordalignmin);

        totalcount:=0;
        rtticount:=0;
        for i:=0 to st.symlist.count-1 do
          if tsym(st.symlist[i]).typ=procsym then
            begin
              sym:=tprocsym(st.symlist[i]);
              inc(totalcount,sym.procdeflist.count);
              for j:=0 to sym.procdeflist.count-1 do
                if tprocdef(sym.procdeflist[j]).visibility in visibilities then
                  inc(rtticount);
            end;

        tcb.emit_ord_const(totalcount,u16inttype);
        if rtticount = 0 then
          tcb.emit_ord_const($FFFF,u16inttype)
        else
          begin
            tcb.emit_ord_const(rtticount,u16inttype);

            for i:=0 to st.symlist.count-1 do
              if tsym(st.symlist[i]).typ=procsym then
                begin
                  sym:=tprocsym(st.symlist[i]);
                  for j:=0 to sym.procdeflist.count-1 do
                    begin
                      def:=tprocdef(sym.procdeflist[j]);

                      if not (def.visibility in visibilities) then
                        continue;

                      def.init_paraloc_info(callerside);

                      tcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
                        targetinfos[target_info.system]^.alignment.recordalignmin);

                      write_rtti_reference(tcb,def.returndef,fullrtti);
                      write_callconv(tcb,def);
                      write_methodkind(tcb,def);
                      tcb.emit_ord_const(def.paras.count,u16inttype);
                      tcb.emit_ord_const(def.callerargareasize,ptrsinttype);
                      tcb.emit_pooled_shortstring_const_ref(sym.realname);

                      for k:=0 to def.paras.count-1 do
                        begin
                          para:=tparavarsym(def.paras[k]);

                          tcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
                            targetinfos[target_info.system]^.alignment.recordalignmin);

                          if is_open_array(para.vardef) or is_array_of_const(para.vardef) then
                            write_rtti_reference(tcb,tarraydef(para.vardef).elementdef,fullrtti)
                          else if para.vardef=cformaltype then
                            write_rtti_reference(tcb,nil,fullrtti)
                          else
                            write_rtti_reference(tcb,para.vardef,fullrtti);
                          write_param_flag(tcb,para);

                          tcb.emit_pooled_shortstring_const_ref(para.realname);

                          write_paralocs(tcb,@para.paraloc[callerside]);

                          tcb.end_anonymous_record;
                        end;

                      if not is_void(def.returndef) then
                        write_paralocs(tcb,@def.funcretloc[callerside]);

                      tcb.end_anonymous_record;
                    end;
                end;
          end;

        tcb.end_anonymous_record;
      end;


    procedure TRTTIWriter.write_header(tcb: ttai_typedconstbuilder; def: tdef; typekind: byte);
      var
        name: shortstring;
      begin
        if assigned(def.typesym) then
          name:=ttypesym(def.typesym).realname
        else
          name:='';
        { TTypeInfo, always packed and doesn't need alignment }
        tcb.begin_anonymous_record(
          internaltypeprefixName[itp_rtti_header]+tostr(length(name)),1,1,
          targetinfos[target_info.system]^.alignment.recordalignmin);
        if def.typ=arraydef then
          InternalError(201012211);
        tcb.emit_tai(Tai_const.Create_8bit(typekind),u8inttype);
        tcb.emit_shortstring_const(name);

        tcb.end_anonymous_record;
      end;


    function TRTTIWriter.write_methodkind(tcb:ttai_typedconstbuilder;def:tabstractprocdef):byte;
      begin
        case def.proctypeoption of
          potype_constructor: result:=mkConstructor;
          potype_destructor: result:=mkDestructor;
          potype_class_constructor: result:=mkClassConstructor;
          potype_class_destructor: result:=mkClassDestructor;
          potype_operator: result:=mkOperatorOverload;
          potype_procedure:
            if po_classmethod in def.procoptions then
              result:=mkClassProcedure
            else
              result:=mkProcedure;
          potype_function:
            if po_classmethod in def.procoptions then
              result:=mkClassFunction
            else
              result:=mkFunction;
        else
          begin
            if def.returndef = voidtype then
              result:=mkProcedure
            else
              result:=mkFunction;
          end;
        end;
        tcb.emit_ord_const(result,u8inttype);
      end;


    procedure TRTTIWriter.write_callconv(tcb:ttai_typedconstbuilder;def:tabstractprocdef);
      const
        ProcCallOptionToCallConv: array[tproccalloption] of byte = (
         { pocall_none       } 0,
         { pocall_cdecl      } 1,
         { pocall_cppdecl    } 5,
         { pocall_far16      } 6,
         { pocall_oldfpccall } 7,
         { pocall_internproc } 8,
         { pocall_syscall    } 9,
         { pocall_pascal     } 2,
         { pocall_register   } 0,
         { pocall_safecall   } 4,
         { pocall_stdcall    } 3,
         { pocall_softfloat  } 10,
         { pocall_mwpascal   } 11,
         { pocall_interrupt  } 12,
         { pocall_hardfloat  } 13,
         { pocall_sysv_abi_default } 14,
         { pocall_sysv_abi_cdecl }   15,
         { pocall_ms_abi_default }   16,
         { pocall_ms_abi_cdecl }     17,
         { pocall_vectorcall }       18
        );
      begin
        tcb.emit_ord_const(ProcCallOptionToCallConv[def.proccalloption],u8inttype);
      end;


    procedure TRTTIWriter.write_paralocs(tcb:ttai_typedconstbuilder;para:pcgpara);
      var
        locs : trttiparalocs;
        i : longint;
        pool : THashSet;
        entry : PHashSetItem;
        loclab : TAsmLabel;
        loctcb : ttai_typedconstbuilder;
        datadef : tdef;
      begin
        locs:=paramanager.cgparalocs_to_rttiparalocs(para^.location);
        if length(locs)>high(byte) then
          internalerror(2017010601);

        if length(locs)=0 then
          begin
            { *shrugs* }
            tcb.emit_tai(Tai_const.Create_nil_codeptr,voidpointertype);
            exit;
          end;

        { do we have such a paraloc already in the pool? }
        pool:=current_asmdata.ConstPools[sp_paraloc];

        entry:=pool.FindOrAdd(@locs[0],length(locs)*sizeof(trttiparaloc));

        if not assigned(entry^.Data) then
          begin
            current_asmdata.getglobaldatalabel(loclab);

            loctcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable]);

            loctcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
              targetinfos[target_info.system]^.alignment.recordalignmin);
            loctcb.emit_ord_const(length(locs),u8inttype);
            for i:=low(locs) to high(locs) do
              begin
                loctcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
                  targetinfos[target_info.system]^.alignment.recordalignmin);
                loctcb.emit_ord_const(locs[i].loctype,u8inttype);
                loctcb.emit_ord_const(locs[i].regsub,u8inttype);
                loctcb.emit_ord_const(locs[i].regindex,u16inttype);
                { the corresponding type for aint is alusinttype }
                loctcb.emit_ord_const(locs[i].offset,alusinttype);
                loctcb.end_anonymous_record;
              end;
            datadef:=loctcb.end_anonymous_record;

            current_asmdata.asmlists[al_typedconsts].concatList(
              loctcb.get_final_asmlist(loclab,datadef,sec_rodata_norel,loclab.name,const_align(sizeof(pint)))
            );

            loctcb.free;

            entry^.data:=loclab;
          end
        else
          loclab:=TAsmLabel(entry^.Data);

        tcb.emit_tai(Tai_const.Create_sym(loclab),voidpointertype);
      end;


    procedure TRTTIWriter.write_param_flag(tcb:ttai_typedconstbuilder;parasym:tparavarsym);
      var
        paraspec : word;
      begin
        case parasym.varspez of
          vs_value   : paraspec := 0;
          vs_const   : paraspec := pfConst;
          vs_var     : paraspec := pfVar;
          vs_out     : paraspec := pfOut;
          vs_constref: paraspec := pfConstRef;
          else
            internalerror(2013112904);
        end;
        { Kylix also seems to always add both pfArray and pfReference
          in this case
        }
        if is_open_array(parasym.vardef) or is_array_of_const(parasym.vardef) then
          paraspec:=paraspec or pfArray or pfReference;
        { and these for classes and interfaces (maybe because they
          are themselves addresses?)
        }
        if is_class_or_interface(parasym.vardef) then
          paraspec:=paraspec or pfAddress;
        { flags for the hidden parameters }
        if vo_is_hidden_para in parasym.varoptions then
          paraspec:=paraspec or pfHidden;
        if vo_is_high_para in parasym.varoptions then
          paraspec:=paraspec or pfHigh;
        if vo_is_self in parasym.varoptions then
          paraspec:=paraspec or pfSelf;
        if vo_is_vmt in parasym.varoptions then
          paraspec:=paraspec or pfVmt;
        if vo_is_funcret in parasym.varoptions then
          paraspec:=paraspec or pfResult;
        { set bits run from the highest to the lowest bit on
          big endian systems
        }
        if (target_info.endian = endian_big) then
          paraspec:=reverse_word(paraspec);
        { write flags for current parameter }
        tcb.emit_ord_const(paraspec,u16inttype);
      end;


    function compare_mop_offset_entry(item1,item2:pointer):longint;
      var
        entry1: pmanagementoperator_offset_entry absolute item1;
        entry2: pmanagementoperator_offset_entry absolute item2;
      begin
        if entry1^.offset<entry2^.offset then
          result:=-1
        else if entry1^.offset>entry2^.offset then
          result:=1
        else
          result:=0;
      end;


    procedure TRTTIWriter.write_mop_offset_table(tcb:ttai_typedconstbuilder;def:tabstractrecorddef;mop:tmanagementoperator);
      var
        list : tfplist;
        datatcb : ttai_typedconstbuilder;
        tbllbl : TAsmLabel;
        entry : pmanagementoperator_offset_entry;
        datadef,entrydef : tdef;
        i : longint;
        pdef : tobjectdef;
      begin
        list:=tfplist.create;
        tabstractrecordsymtable(def.symtable).get_managementoperator_offset_list(mop,list);
        if (def.typ=objectdef) then
          begin
            pdef:=tobjectdef(def).childof;
            while assigned(pdef) do
              begin
                tabstractrecordsymtable(pdef.symtable).get_managementoperator_offset_list(mop,list);
                pdef:=pdef.childof;
              end;
            list.sort(@compare_mop_offset_entry);
          end;
        if list.count=0 then
          tcb.emit_tai(tai_const.create_nil_dataptr,voidpointertype)
        else
          begin
            tcb.start_internal_data_builder(current_asmdata.AsmLists[al_rtti],sec_rodata,'',datatcb,tbllbl);

            datatcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
              targetinfos[target_info.system]^.alignment.recordalignmin);
            datatcb.emit_ord_const(list.count,u32inttype);

            entrydef:=get_recorddef(itp_init_mop_offset_entry,[voidcodepointertype,sizeuinttype],defaultpacking);

            for i:=0 to list.count-1 do
              begin
                entry:=pmanagementoperator_offset_entry(list[i]);

                datatcb.maybe_begin_aggregate(entrydef);

                datatcb.queue_init(voidcodepointertype);
                datatcb.queue_emit_proc(entry^.pd);

                datatcb.queue_init(sizeuinttype);
                datatcb.queue_emit_ordconst(entry^.offset,sizeuinttype);

                datatcb.maybe_end_aggregate(entrydef);

                dispose(entry);
              end;

            datadef:=datatcb.end_anonymous_record;

            tcb.finish_internal_data_builder(datatcb,tbllbl,datadef,sizeof(pint));

            tcb.emit_tai(tai_const.Create_sym(tbllbl),voidpointertype);
          end;
        list.free;
      end;


    procedure TRTTIWriter.write_rtti_name(tcb: ttai_typedconstbuilder; def: tdef);
      begin
         if is_open_array(def) then
           { open arrays never have a typesym with a name, since you cannot
             define an "open array type". Kylix prints the type of the
             elements in the array in this case (so together with the pfArray
             flag, you can reconstruct the full typename, I assume (JM))
           }
           def:=tarraydef(def).elementdef;
         { name }
         if assigned(def.typesym) then
           tcb.emit_shortstring_const(ttypesym(def.typesym).realname)
         else
           tcb.emit_shortstring_const('');
      end;


    procedure TRTTIWriter.write_common_rtti_data(tcb:ttai_typedconstbuilder;def:tdef;rt:trttitype);
      begin
        if rt<>fullrtti then
          write_attribute_data(tcb,nil)
        else
          write_attribute_data(tcb,tstoreddef(def).rtti_attribute_list);
      end;


    { writes a 32-bit count followed by array of field infos for given symtable }
    procedure TRTTIWriter.fields_write_rtti_data(tcb: ttai_typedconstbuilder; def: tabstractrecorddef; rt: trttitype);
      var
        i   : longint;
        sym : tsym;
        fieldcnt: longint;
        st: tsymtable;
        fields: tfplist;
        parentrtti: boolean;
      begin
        fieldcnt:=0;
        parentrtti:=false;
        st:=def.symtable;
        fields:=tfplist.create;
        fields.capacity:=st.symlist.count+1;
        { For objects, treat parent (if any) as a field with offset 0. This
          provides correct handling of entire instance with RTL rtti routines. }
        if (def.typ=objectdef) and (tobjectdef(def).objecttype=odt_object) and
            Assigned(tobjectdef(def).childof) and
            ((rt=fullrtti) or (tobjectdef(def).childof.needs_inittable)) then
           begin
             parentrtti:=true;
             inc(fieldcnt);
           end;

        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if is_normal_fieldvarsym(sym) and
               (
                (rt=fullrtti) or
                tfieldvarsym(sym).vardef.needs_inittable
               ) and
               not is_objc_class_or_protocol(tfieldvarsym(sym).vardef) then
              begin
                fields.add(tfieldvarsym(sym));
                inc(fieldcnt);
              end;
          end;
        { insert field count before data }
        tcb.emit_ord_const(fieldcnt,u32inttype);
        { parent object? }
        if parentrtti then
          begin
            write_rtti_reference(tcb,tobjectdef(def).childof,rt);
            tcb.emit_ord_const(0,ptruinttype);
          end;
        { fields }
        for i:=0 to fields.count-1 do
          begin
            sym:=tsym(fields[i]);
            write_rtti_reference(tcb,tfieldvarsym(sym).vardef,rt);
            tcb.emit_ord_const(tfieldvarsym(sym).fieldoffset,ptruinttype);
          end;
        fields.free;
      end;


    procedure TRTTIWriter.fields_write_rtti(st:tsymtable;rt:trttitype);
      var
        i   : longint;
        sym : tsym;
      begin
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if is_normal_fieldvarsym(sym) and
               (
                (rt=fullrtti) or
                tfieldvarsym(sym).vardef.needs_inittable
               ) then
              write_rtti(tfieldvarsym(sym).vardef,rt);
          end;
      end;


    procedure TRTTIWriter.params_write_rtti(def:tabstractprocdef;rt:trttitype;allow_hidden:boolean);
      var
        i   : longint;
        sym : tparavarsym;
      begin
        for i:=0 to def.paras.count-1 do
          begin
            sym:=tparavarsym(def.paras[i]);
            if not (vo_is_hidden_para in sym.varoptions) or allow_hidden then
              begin
                if is_open_array(sym.vardef) or is_array_of_const(sym.vardef) then
                  write_rtti(tarraydef(sym.vardef).elementdef,rt)
                else
                  write_rtti(sym.vardef,rt);
              end;
          end;
      end;


    procedure TRTTIWriter.methods_write_rtti(st:tsymtable;rt:trttitype;visibilities:tvisibilities;allow_hidden:boolean);
      var
        i,j : longint;
        sym : tprocsym;
        def : tabstractprocdef;
      begin
        for i:=0 to st.symlist.count-1 do
          if tsym(st.symlist[i]).typ=procsym then
            begin
              sym:=tprocsym(st.symlist[i]);
              for j:=0 to sym.procdeflist.count-1 do
                begin
                  def:=tabstractprocdef(sym.procdeflist[j]);
                  write_rtti(def.returndef,rt);
                  params_write_rtti(def,rt,allow_hidden);
                end;
            end;
      end;


    procedure TRTTIWriter.published_write_rtti(st:tsymtable;rt:trttitype);
      var
        i   : longint;
        sym : tsym;
      begin
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (sym.visibility=vis_published) then
              begin
                case tsym(sym).typ of
                  propertysym:
                    write_rtti(tpropertysym(sym).propdef,rt);
                  fieldvarsym:
                    write_rtti(tfieldvarsym(sym).vardef,rt);
                  else
                    ;
                end;
              end;
          end;
      end;


    function TRTTIWriter.published_properties_count(st:tsymtable):longint;
      var
        i   : longint;
        sym : tsym;
      begin
        result:=0;
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (tsym(sym).typ=propertysym) and
               (sym.visibility=vis_published) then
              inc(result);
          end;
      end;


    procedure TRTTIWriter.collect_propnamelist(propnamelist:TFPHashObjectList;objdef:tobjectdef);
      var
        i   : longint;
        sym : tsym;
        pn  : tpropnamelistitem;
      begin
        if assigned(objdef.childof) then
          collect_propnamelist(propnamelist,objdef.childof);
        for i:=0 to objdef.symtable.SymList.Count-1 do
          begin
            sym:=tsym(objdef.symtable.SymList[i]);
            if (tsym(sym).typ=propertysym) and
               (sym.visibility=vis_published) then
              begin
                pn:=TPropNameListItem(propnamelist.Find(tsym(sym).name));
                if not assigned(pn) then
                  begin
                     pn:=tpropnamelistitem.create(propnamelist,tsym(sym).name);
                     pn.propindex:=propnamelist.count-1;
                     pn.propowner:=tsym(sym).owner;
                  end;
             end;
          end;
      end;


    procedure TRTTIWriter.published_properties_write_rtti_data(tcb: ttai_typedconstbuilder; propnamelist:TFPHashObjectList;st:tsymtable);
      var
        i : longint;
        sym : tsym;
        proctypesinfo : byte;
        propnameitem  : tpropnamelistitem;
        propdefname : string;

        procedure writeaccessproc(pap:tpropaccesslisttypes; shiftvalue : byte; unsetvalue: byte);
        var
           typvalue : byte;
           hp : ppropaccesslistitem;
           extnumber: longint;
           address,space : longint;
           def : tdef;
           hpropsym : tpropertysym;
           propaccesslist : tpropaccesslist;
        begin
           hpropsym:=tpropertysym(sym);
           repeat
             propaccesslist:=hpropsym.propaccesslist[pap];
             if not propaccesslist.empty then
               break;
             hpropsym:=hpropsym.overriddenpropsym;
           until not assigned(hpropsym);
           if not(assigned(propaccesslist) and assigned(propaccesslist.firstsym))  then
             begin
               tcb.emit_tai(Tai_const.Create_int_codeptr(unsetvalue),codeptruinttype);
               typvalue:=3;
             end
           { also for accessing class fields }
           else if propaccesslist.firstsym^.sym.typ=fieldvarsym then
             begin
                address:=0;
                hp:=propaccesslist.firstsym;
                def:=nil;
                while assigned(hp) do
                  begin
                     case hp^.sltype of
                       sl_load :
                         begin
                           def:=tfieldvarsym(hp^.sym).vardef;
                           inc(address,tfieldvarsym(hp^.sym).fieldoffset);
                         end;
                       sl_subscript :
                         begin
                           if not(assigned(def) and
                                  ((def.typ=recorddef) or
                                   is_object(def))) then
                             internalerror(200402171);
                           inc(address,tfieldvarsym(hp^.sym).fieldoffset);
                           def:=tfieldvarsym(hp^.sym).vardef;
                         end;
                       sl_vec :
                         begin
                           if not(assigned(def) and (def.typ=arraydef)) then
                             internalerror(200402172);
                           def:=tarraydef(def).elementdef;
                           {Hp.value is a Tconstexprint, which can be rather large,
                            sanity check for longint overflow.}
                           space:=(high(address)-address) div def.size;
                           if int64(space)<hp^.value then
                             internalerror(200706101);
                           inc(address,int64(def.size*hp^.value));
                         end;
                       else
                         internalerror(2019050523);
                     end;
                     hp:=hp^.next;
                  end;
                tcb.emit_tai(Tai_const.Create_int_codeptr(address),codeptruinttype);
                typvalue:=0;
             end
           else
             begin
                { When there was an error then procdef is not assigned }
                if not assigned(propaccesslist.procdef) then
                  exit;
                if not(po_virtualmethod in tprocdef(propaccesslist.procdef).procoptions) or
                   is_objectpascal_helper(tprocdef(propaccesslist.procdef).struct) then
                  begin
                    tcb.queue_init(codeptruinttype);
                    tcb.queue_emit_proc(tprocdef(propaccesslist.procdef));
                    typvalue:=1;
                  end
                else
                  begin
                    { virtual method, write vmt offset }
                    extnumber:=tprocdef(propaccesslist.procdef).extnumber;
                    tcb.emit_tai(Tai_const.Create_int_codeptr(
                      tobjectdef(tprocdef(propaccesslist.procdef).struct).vmtmethodoffset(extnumber)),
                      codeptruinttype);
                    { register for wpo }
                    tobjectdef(tprocdef(propaccesslist.procdef).struct).register_vmt_call(extnumber);
                    {$ifdef vtentry}
                    { not sure if we can insert those vtentry symbols safely here }
                    {$error register methods used for published properties}
                    {$endif vtentry}
                    typvalue:=2;
                  end;
             end;
           proctypesinfo:=proctypesinfo or (typvalue shl shiftvalue);
        end;

      begin
        tcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
          targetinfos[target_info.system]^.alignment.recordalignmin);
        tcb.emit_ord_const(published_properties_count(st),u16inttype);
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (sym.typ=propertysym) and
               (sym.visibility=vis_published) then
              begin
                { we can only easily reuse defs if the property is not stored,
                  because otherwise the rtti layout depends on how the "stored"
                  is defined (field, indexed expression, virtual method, ...) }
                if not(ppo_stored in tpropertysym(sym).propoptions) then
                  propdefname:=internaltypeprefixName[itp_rtti_prop]+tostr(length(tpropertysym(sym).realname))
                else
                  propdefname:='';
                { TPropInfo is a packed record (even on targets that require
                  alignment), but it starts aligned }
                tcb.begin_anonymous_record(
                  propdefname,
                  1,min(reqalign,SizeOf(PInt)),
                  targetinfos[target_info.system]^.alignment.recordalignmin);
                if ppo_indexed in tpropertysym(sym).propoptions then
                  proctypesinfo:=$40
                else
                  proctypesinfo:=0;
                write_rtti_reference(tcb,tpropertysym(sym).propdef,fullrtti);
                writeaccessproc(palt_read,0,0);
                writeaccessproc(palt_write,2,0);
                { is it stored ? }
                if not(ppo_stored in tpropertysym(sym).propoptions) then
                  begin
                    { no, so put a constant zero }
                    tcb.emit_tai(Tai_const.Create_nil_codeptr,codeptruinttype);
                    proctypesinfo:=proctypesinfo or (3 shl 4);
                  end
                else
                  writeaccessproc(palt_stored,4,1); { maybe; if no procedure put a constant 1 (=true) }
                tcb.emit_ord_const(tpropertysym(sym).index,u32inttype);
                tcb.emit_ord_const(tpropertysym(sym).default,u32inttype);
                propnameitem:=TPropNameListItem(propnamelist.Find(tpropertysym(sym).name));
                if not assigned(propnameitem) then
                  internalerror(200512201);
                tcb.emit_ord_const(propnameitem.propindex,u16inttype);
                tcb.emit_ord_const(proctypesinfo,u8inttype);

                { write reference to attribute table }
                write_attribute_data(tcb,tpropertysym(sym).rtti_attribute_list);

                { Write property name }
                tcb.emit_shortstring_const(tpropertysym(sym).realname);

                tcb.end_anonymous_record;
             end;
          end;
        tcb.end_anonymous_record;
      end;


    procedure TRTTIWriter.write_rtti_data(tcb: ttai_typedconstbuilder; def: tdef; rt: trttitype);

        procedure unknown_rtti(def:tstoreddef);
        begin
          tcb.emit_ord_const(tkUnknown,u8inttype);
          write_rtti_name(tcb,def);
          write_common_rtti_data(tcb,def,rt);
        end;

        procedure variantdef_rtti(def:tvariantdef);
        begin
          write_header(tcb,def,tkVariant);
          write_common_rtti_data(tcb,def,rt);
        end;

        procedure stringdef_rtti(def:tstringdef);
        const
          string_typekinds: array[tstringtype] of byte = (
            tkSString,tkLString,tkAString,tkWString,tkUString
          );
        begin
          write_header(tcb,def,string_typekinds[def.stringtype]);
          tcb.begin_anonymous_record(
            internaltypeprefixName[itp_rtti_outer]+tostr(string_typekinds[def.stringtype]),
            defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          write_common_rtti_data(tcb,def,rt);
          case def.stringtype of
            st_ansistring:
              begin
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_rtti_case]+tostr(string_typekinds[def.stringtype]),
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin);
                tcb.emit_ord_const(def.encoding,u16inttype);
                tcb.end_anonymous_record;
              end;

            st_shortstring:
              begin
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_rtti_case]+tostr(string_typekinds[def.stringtype]),
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin);
                tcb.emit_ord_const(def.len,u8inttype);
                tcb.end_anonymous_record;
              end;

            st_widestring,
            st_unicodestring,
            st_longstring:
              { nothing }
              ;

          end;
          tcb.end_anonymous_record;
        end;

        procedure enumdef_rtti(def: tenumdef);
        var
           i  : integer;
           hp : tenumsym;
        begin
          write_header(tcb,def,tkEnumeration);
          { align; the named fields are so that we can let the compiler
            calculate the string offsets later on }
          tcb.next_field_name:='size_start_rec';
          { add a typename so that it can be reused when writing the the s2o
            and o2s arrays for llvm (otherwise we have to write out the entire
            type definition every time we access an element from this record) }
          tcb.begin_anonymous_record(internaltypeprefixName[itp_rtti_enum_size_start_rec]+def.unique_id_str,defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          write_common_rtti_data(tcb,def,rt);
          tcb.next_field_name:='typ_union_rec';
          tcb.begin_anonymous_record(internaltypeprefixName[itp_rtti_enum_size_start_rec2]+def.unique_id_str,defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          case longint(def.size) of
            1 :
              tcb.emit_ord_const(otUByte,u8inttype);
            2 :
              tcb.emit_ord_const(otUWord,u8inttype);
            4 :
              tcb.emit_ord_const(otULong,u8inttype);
          end;
          { we need to align by Tconstptruint here to satisfy the alignment
            rules set by records: in the typinfo unit we overlay a TTypeData
            record on this data, which at the innermost variant record needs an
            alignment of TConstPtrUint due to e.g. the "CompType" member for
            tkSet (also the "BaseType" member for tkEnumeration).

            We need to adhere to this, otherwise things will break. }
          tcb.next_field_name:='min_max_rec';
          tcb.begin_anonymous_record(internaltypeprefixName[itp_rtti_enum_min_max_rec]+def.unique_id_str,defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          tcb.emit_ord_const(def.min,s32inttype);
          tcb.emit_ord_const(def.max,s32inttype);
          tcb.next_field_name:='basetype_array_rec';
          { all strings must appear right after each other -> from now on
            packrecords 1 (but the start must still be aligned) }
          tcb.begin_anonymous_record(internaltypeprefixName[itp_rtti_enum_basetype_array_rec]+def.unique_id_str,1,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          { write base type }
          write_rtti_reference(tcb,def.basedef,rt);
          for i:=0 to def.symtable.SymList.Count-1 do
            begin
              hp:=tenumsym(def.symtable.SymList[i]);
              if hp.value<def.minval then
                continue
              else
              if hp.value>def.maxval then
                break;
              tcb.next_field_name:=hp.name;
              tcb.emit_shortstring_const(hp.realname);
            end;
          { write unit name }
          tcb.emit_shortstring_const(current_module.realmodulename^);
          { write zero which is required by RTL }
          tcb.emit_ord_const(0,u8inttype);
          { terminate all records }
          tcb.end_anonymous_record;
          tcb.end_anonymous_record;
          tcb.end_anonymous_record;
          tcb.end_anonymous_record;
        end;

        procedure orddef_rtti(def:torddef);

          procedure doint32_64(typekind: byte;min,max:int64);
            const
              trans : array[tordtype] of byte =
                (otUByte{otNone},
                 otUByte,otUWord,otULong,otUQWord,otUByte{otNone},
                 otSByte,otSWord,otSLong,otSQWord,otUByte{otNone},
                 otUByte,otUByte,otUWord,otULong,otUQWord,
                 otSByte,otSWord,otSLong,otSQWord,
                 otUByte,otUWord,otUByte,255);
            var
              elesize: string[1];
              deftrans: byte;
          begin
            write_header(tcb,def,typekind);
            deftrans:=trans[def.ordtype];
            case deftrans of
              otUQWord,
              otSQWord:
                elesize:='8';
              255:
                begin
                  if def.packedbitsize<=32 then
                    begin
                      elesize:='4';
                      if def.low<0 then
                        deftrans:=otSLong
                      else
                        deftrans:=otULong;
                    end
                  else
                    begin
                      elesize:='8';
                      if def.low<0 then
                        deftrans:=otSQWord
                      else
                        deftrans:=otUQWord;
                    end;
                end
              else
                elesize:='4'
            end;
            tcb.begin_anonymous_record(
              internaltypeprefixName[itp_rtti_ord_outer]+elesize,
              defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);
            write_common_rtti_data(tcb,def,rt);
            tcb.begin_anonymous_record(
              internaltypeprefixName[itp_rtti_ord_middle]+elesize,
              defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);
            tcb.emit_ord_const(byte(trans[def.ordtype]),u8inttype);
            tcb.begin_anonymous_record(
              internaltypeprefixName[itp_rtti_ord_inner]+elesize,
              defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);
            {Convert to longint to smuggle values in high(longint)+1..high(cardinal) into asmlist.}
            case deftrans of
              otUQWord:
                begin
                  tcb.emit_ord_const(min,u64inttype);
                  tcb.emit_ord_const(max,u64inttype);
                end;
              otSQWord:
                begin
                  tcb.emit_ord_const(min,s64inttype);
                  tcb.emit_ord_const(max,s64inttype);
                end;
              else
                begin
                  tcb.emit_ord_const(longint(min),s32inttype);
                  tcb.emit_ord_const(longint(max),s32inttype);
                end;
            end;
            tcb.end_anonymous_record;
            tcb.end_anonymous_record;
            tcb.end_anonymous_record;
          end;

        procedure dointeger(typekind:byte);inline;
          begin
            doint32_64(typekind,int64(def.low.svalue),int64(def.high.svalue));
          end;

        begin
          case def.ordtype of
            s64bit :
                dointeger(tkInt64);
            u64bit :
                dointeger(tkQWord);
            pasbool1,
            pasbool8,
            pasbool16,
            pasbool32,
            pasbool64:
                dointeger(tkBool);
            { use different low/high values to be Delphi compatible }
            bool8bit,
            bool16bit,
            bool32bit:
                doint32_64(tkBool,longint(low(longint)),longint(high(longint)));
            bool64bit:
                doint32_64(tkBool,low(int64),high(int64));
            uchar:
                dointeger(tkChar);
            uwidechar:
                dointeger(tkWChar);
            scurrency:
              begin
                write_header(tcb,def,tkFloat);
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_1byte],
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin);
                write_common_rtti_data(tcb,def,rt);
                tcb.emit_ord_const(ftCurr,u8inttype);
                tcb.end_anonymous_record;
              end;
            else
              dointeger(tkInteger);
          end;
        end;


        procedure floatdef_rtti(def:tfloatdef);
        const
          {tfloattype = (s32real,s64real,s80real,sc80real,s64bit,s128bit);}
          translate : array[tfloattype] of byte =
             (ftSingle,ftDouble,ftExtended,ftExtended,ftComp,ftCurr,ftFloat128);
        begin
           write_header(tcb,def,tkFloat);
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_1byte],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);
           write_common_rtti_data(tcb,def,rt);
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_rtti_float],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);
           tcb.emit_ord_const(translate[def.floattype],u8inttype);
           tcb.end_anonymous_record;
           tcb.end_anonymous_record;
        end;


        procedure setdef_rtti(def:tsetdef);
        begin
           write_header(tcb,def,tkSet);
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_rtti_set_outer],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);
           write_common_rtti_data(tcb,def,rt);
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_rtti_set_middle],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);
           case def.size of
             1:
               tcb.emit_ord_const(otUByte,u8inttype);
             2:
               tcb.emit_ord_const(otUWord,u8inttype);
             4:
               tcb.emit_ord_const(otULong,u8inttype);
             else
               tcb.emit_ord_const(otUByte,u8inttype);
           end;
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_rtti_set_inner],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);
           tcb.emit_ord_const(def.size,sizesinttype);
           write_rtti_reference(tcb,def.elementdef,rt);
           tcb.end_anonymous_record;
           tcb.end_anonymous_record;
           tcb.end_anonymous_record;
        end;


        procedure arraydef_rtti(def:tarraydef);
          var
            i,dimcount: byte;
            totalcount: asizeuint;
            finaldef: tdef;
            curdef:tarraydef;
        begin
           if ado_IsDynamicArray in def.arrayoptions then
             tcb.emit_ord_const(tkDynArray,u8inttype)
           else
             tcb.emit_ord_const(tkArray,u8inttype);
           write_rtti_name(tcb,def);

           if not(ado_IsDynamicArray in def.arrayoptions) then
             begin
               { remember tha last instruction. we will need to insert some
                 calculated values after it }
               finaldef:=def;
               totalcount:=1;
               dimcount:=0;
               repeat
                 curdef:=tarraydef(finaldef);
                 finaldef:=curdef.elementdef;
                 { Dims[i] PTypeInfo }
                 inc(dimcount);
                 totalcount:=totalcount*curdef.elecount;
               until (finaldef.typ<>arraydef) or
                     (ado_IsDynamicArray in tarraydef(finaldef).arrayoptions);
               tcb.begin_anonymous_record(
                 internaltypeprefixName[itp_rtti_normal_array]+tostr(dimcount),
                 defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin);

               write_common_rtti_data(tcb,def,rt);

               tcb.begin_anonymous_record(internaltypeprefixName[itp_rtti_normal_array_inner]+tostr(dimcount),
                 defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin);

               { total size = elecount * elesize of the first arraydef }
               tcb.emit_tai(Tai_const.Create_sizeint(def.elecount*def.elesize),sizeuinttype);
               { total element count }
               tcb.emit_tai(Tai_const.Create_sizeint(asizeint(totalcount)),sizeuinttype);
               { last dimension element type }
               tcb.emit_tai(Tai_const.Create_sym(get_rtti_label(curdef.elementdef,rt,true)),voidpointertype);
               { dimension count }
               tcb.emit_ord_const(dimcount,u8inttype);
               finaldef:=def;
               { ranges of the dimensions }
               for i:=1 to dimcount do
                 begin
                   curdef:=tarraydef(finaldef);
                   finaldef:=curdef.elementdef;
                   { Dims[i] PPTypeInfo }
                   write_rtti_reference(tcb,curdef.rangedef,rt);
                 end;
             end
           else
             { write a delphi almost compatible dyn. array entry:
               there are two types, eltype and eltype2, the latter is nil if the element type needs
               no finalization, the former is always valid, delphi has this swapped, but for
               compatibility with older fpc versions we do it different, to be delphi compatible,
               the names are swapped in typinfo.pp
             }
             begin
               tcb.begin_anonymous_record(
                 internaltypeprefixName[itp_rtti_dyn_array],
                 defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin);

               write_common_rtti_data(tcb,def,rt);

               { record in TypInfo is aligned differently from init rtti }
               tcb.begin_anonymous_record(
                 internaltypeprefixName[itp_rtti_dyn_array_inner],
                 defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin);

               { size of elements }
               tcb.emit_tai(Tai_const.Create_sizeint(def.elesize),sizeuinttype);
               { element type }
               write_rtti_reference(tcb,def.elementdef,rt);
               { variant type }
               tcb.emit_ord_const(tstoreddef(def.elementdef).getvardef,s32inttype);
               { element type }
               if def.elementdef.needs_inittable then
                 write_rtti_reference(tcb,def.elementdef,rt)
               else
                 tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
               { write unit name }
               tcb.emit_shortstring_const(current_module.realmodulename^);
             end;

          tcb.end_anonymous_record;
          tcb.end_anonymous_record;
        end;

        procedure classrefdef_rtti(def:tclassrefdef);
        begin
          write_header(tcb,def,tkClassRef);
          tcb.begin_anonymous_record(
            internaltypeprefixName[itp_rtti_ref],
            defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          write_common_rtti_data(tcb,def,rt);
          tcb.begin_anonymous_record(
            internaltypeprefixName[itp_rtti_classref],
            defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          write_rtti_reference(tcb,def.pointeddef,rt);
          tcb.end_anonymous_record;
          tcb.end_anonymous_record;
        end;

        procedure pointerdef_rtti(def:tpointerdef);
        begin
          write_header(tcb,def,tkPointer);
          tcb.begin_anonymous_record(
            internaltypeprefixName[itp_rtti_ref],
            defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          write_common_rtti_data(tcb,def,rt);
          tcb.begin_anonymous_record(
            internaltypeprefixName[itp_rtti_pointer],
            defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          write_rtti_reference(tcb,def.pointeddef,rt);
          tcb.end_anonymous_record;
          tcb.end_anonymous_record;
        end;

        procedure recorddef_rtti(def:trecorddef);

          procedure write_record_operators;
          var
            rttilab: Tasmsymbol;
            rttidef: tdef;
            tcb: ttai_typedconstbuilder;
            mop: tmanagementoperator;
            procdef: tprocdef;
          begin
            rttilab := current_asmdata.DefineAsmSymbol(
                internaltypeprefixName[itp_init_record_operators]+def.rtti_mangledname(rt),
                AB_GLOBAL,AT_DATA,def);
            tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable]);

            tcb.begin_anonymous_record(
              rttilab.Name,
              defaultpacking,min(reqalign,SizeOf(PInt)),
              targetinfos[target_info.system]^.alignment.recordalignmin
            );

            { use "succ" to omit first enum item "mop_none" }
            for mop := succ(low(tmanagementoperator)) to high(tmanagementoperator) do
            begin
              if not (mop in trecordsymtable(def.symtable).managementoperators) then
                tcb.emit_tai(Tai_const.Create_nil_codeptr,voidcodepointertype)
              else
                begin
                  procdef := search_management_operator(mop, def);
                  if procdef = nil then
                    internalerror(201603021)
                  else
                    tcb.emit_tai(Tai_const.Createname(procdef.mangledname,AT_FUNCTION,0),
                      cprocvardef.getreusableprocaddr(procdef));
                end;
            end;

            rttidef := tcb.end_anonymous_record;

            current_asmdata.AsmLists[al_rtti].concatList(
              tcb.get_final_asmlist(rttilab,rttidef,sec_rodata,rttilab.name,
              sizeof(PInt)));
            tcb.free;
          end;

        begin
           write_header(tcb,def,tkRecord);
           { need extra reqalign record, because otherwise the u32 int will
             only be aligned to 4 even on 64 bit target (while the rtti code
             in typinfo expects alignments to sizeof(pointer)) }
           tcb.begin_anonymous_record('',
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);

           write_common_rtti_data(tcb,def,rt);

           tcb.begin_anonymous_record('',
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);
           { store special terminator for init table for more optimal rtl operations
             strictly related to RecordRTTI procedure in rtti.inc (directly 
             related to RTTIRecordRttiInfoToInitInfo function) }
           if rt=initrtti then
             tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype)
           else
             { we use a direct reference as the init RTTI is always in the same
               unit as the full RTTI }
             tcb.emit_tai(Tai_const.Create_sym(get_rtti_label(def,initrtti,false)),voidpointertype);

           tcb.emit_ord_const(def.size,u32inttype);

           { store rtti management operators only for init table }
           if rt=initrtti then
             begin
               { for now records don't have the initializer table }
               tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
               if (trecordsymtable(def.symtable).managementoperators=[]) then
                 tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype)
               else
                 tcb.emit_tai(Tai_const.Createname(
                   internaltypeprefixName[itp_init_record_operators]+def.rtti_mangledname(rt),
                   AT_DATA_FORCEINDIRECT,0),voidpointertype);
             end;

           fields_write_rtti_data(tcb,def,rt);
           tcb.end_anonymous_record;
           tcb.end_anonymous_record;

           { write pointers to operators if needed }
           if (rt=initrtti) and (trecordsymtable(def.symtable).managementoperators<>[]) then
             write_record_operators;
        end;


        procedure procvardef_rtti(def:tprocvardef);

           procedure write_para(parasym:tparavarsym);
             begin
               { write flags for current parameter }
               write_param_flag(tcb,parasym);
               { write name of current parameter }
               tcb.emit_shortstring_const(parasym.realname);
               { write name of type of current parameter }
               write_rtti_name(tcb,parasym.vardef);
             end;

           procedure write_procedure_param(parasym:tparavarsym);
             begin
               { every parameter is expected to start aligned }
               tcb.begin_anonymous_record(
                 internaltypeprefixName[itp_rtti_proc_param]+tostr(length(parasym.realname)),
                 defaultpacking,min(reqalign,SizeOf(PInt)),
                 targetinfos[target_info.system]^.alignment.recordalignmin);
               { write flags for current parameter }
               write_param_flag(tcb,parasym);
               { write param type }
               if is_open_array(parasym.vardef) or is_array_of_const(parasym.vardef) then
                 write_rtti_reference(tcb,tarraydef(parasym.vardef).elementdef,fullrtti)
               else if parasym.vardef=cformaltype then
                 write_rtti_reference(tcb,nil,fullrtti)
               else
                 write_rtti_reference(tcb,parasym.vardef,fullrtti);
               { write name of current parameter }
               tcb.emit_shortstring_const(parasym.realname);
               tcb.end_anonymous_record;
             end;

        var
          methodkind : byte;
          i : integer;
        begin
          if po_methodpointer in def.procoptions then
            begin
               { write method id and name }
               write_header(tcb,def,tkMethod);
               tcb.begin_anonymous_record('',defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin);

               write_common_rtti_data(tcb,def,rt);

               tcb.begin_anonymous_record('',
                 defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin);
               { write kind of method }
               methodkind:=write_methodkind(tcb,def);

               { write parameter info. The parameters must be written in reverse order
                 if this method uses right to left parameter pushing! }
               tcb.emit_ord_const(def.paras.count,u8inttype);

               for i:=0 to def.paras.count-1 do
                 write_para(tparavarsym(def.paras[i]));

               if (methodkind=mkFunction) or (methodkind=mkClassFunction) then
               begin
                 { write name of result type }
                 write_rtti_name(tcb,def.returndef);
                 { enclosing record takes care of alignment }
                 { write result typeinfo }
                 write_rtti_reference(tcb,def.returndef,fullrtti);
               end;

               { write calling convention }
               write_callconv(tcb,def);

               { enclosing record takes care of alignment }
               { write params typeinfo }
               for i:=0 to def.paras.count-1 do
                 begin
                   if is_open_array(tparavarsym(def.paras[i]).vardef) or is_array_of_const(tparavarsym(def.paras[i]).vardef) then
                     write_rtti_reference(tcb,tarraydef(tparavarsym(def.paras[i]).vardef).elementdef,fullrtti)
                   else if tparavarsym(def.paras[i]).vardef=cformaltype then
                     write_rtti_reference(tcb,nil,fullrtti)
                   else
                     write_rtti_reference(tcb,tparavarsym(def.paras[i]).vardef,fullrtti);
                 end;
               tcb.end_anonymous_record;
               tcb.end_anonymous_record;
            end
          else
            begin
              write_header(tcb,def,tkProcvar);
              tcb.begin_anonymous_record('',defaultpacking,reqalign,
                targetinfos[target_info.system]^.alignment.recordalignmin);

              write_common_rtti_data(tcb,def,rt);

              tcb.begin_anonymous_record('',
                defaultpacking,reqalign,
                targetinfos[target_info.system]^.alignment.recordalignmin);
              { flags }
              tcb.emit_ord_const(0,u8inttype);
              { write calling convention }
              write_callconv(tcb,def);
              { enclosing record takes care of alignment }
              { write result typeinfo }
              write_rtti_reference(tcb,def.returndef,fullrtti);
              { write parameter count }
              tcb.emit_ord_const(def.paras.count,u8inttype);
              for i:=0 to def.paras.count-1 do
                write_procedure_param(tparavarsym(def.paras[i]));
              tcb.end_anonymous_record;
              tcb.end_anonymous_record;
            end;
        end;


        procedure objectdef_rtti(def: tobjectdef);

          procedure objectdef_rtti_fields(def:tobjectdef);
          begin
            tcb.begin_anonymous_record('',defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);

            { - for compatiblity with record RTTI we need to write a terminator-
                Nil pointer for initrtti as well for objects
              - for RTTI consistency for objects we need point from fullrtti
                to initrtti
              - classes are assumed to have the same INIT RTTI as records
                (see TObject.CleanupInstance)
              - neither helper nor class type have fullrtti for fields
            }
            if (rt=initrtti) then
              tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype)
            else
              if (def.objecttype=odt_object) then
                tcb.emit_tai(Tai_const.Create_sym(get_rtti_label(def,initrtti,false)),voidpointertype)
              else
                internalerror(2017011801);

            tcb.emit_ord_const(def.size, u32inttype);
            { pointer to management operators available only for initrtti }
            if (rt=initrtti) then
              begin
                { initializer table only available for classes currently }
                if def.objecttype=odt_class then
                  write_mop_offset_table(tcb,def,mop_initialize)
                else
                  tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
                tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
              end;
            { enclosing record takes care of alignment }
            fields_write_rtti_data(tcb,def,rt);

            tcb.end_anonymous_record;
          end;

          procedure objectdef_rtti_interface_init(def:tobjectdef);
          begin
            tcb.emit_ord_const(def.size, u32inttype);
          end;

          procedure objectdef_rtti_class_full(def:tobjectdef);
          var
            propnamelist : TFPHashObjectList;
          begin
            { Collect unique property names with nameindex }
            propnamelist:=TFPHashObjectList.Create;
            collect_propnamelist(propnamelist,def);

            tcb.begin_anonymous_record('',defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);

            if not is_objectpascal_helper(def) then
              if (oo_has_vmt in def.objectoptions) then
                tcb.emit_tai(
                  Tai_const.Createname(def.vmt_mangledname,AT_DATA_FORCEINDIRECT,0),
                  cpointerdef.getreusable(def.vmt_def))
              else
                tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);

            { write parent typeinfo }
            write_rtti_reference(tcb,def.childof,fullrtti);

            { write typeinfo of extended type }
            if is_objectpascal_helper(def) then
              if assigned(def.extendeddef) then
                write_rtti_reference(tcb,def.extendeddef,fullrtti)
              else
                InternalError(2011033001);

            { total number of unique properties }
            tcb.emit_ord_const(propnamelist.count,u16inttype);

            { write unit name }
            tcb.emit_shortstring_const(current_module.realmodulename^);

            { write published properties for this object }
            published_properties_write_rtti_data(tcb,propnamelist,def.symtable);

            tcb.end_anonymous_record;

            propnamelist.free;
          end;

          procedure objectdef_rtti_interface_full(def:tobjectdef);
          var
            propnamelist : TFPHashObjectList;
            { if changed to a set, make sure it's still a byte large, and
              swap appropriately when cross-compiling
            }
            IntfFlags: byte;
          begin
            { Collect unique property names with nameindex }
            propnamelist:=TFPHashObjectList.Create;
            collect_propnamelist(propnamelist,def);

            tcb.begin_anonymous_record('',defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);

            { write parent typeinfo }
            write_rtti_reference(tcb,def.childof,fullrtti);

            { interface: write flags, iid and iidstr }
            IntfFlags:=0;
            if assigned(def.iidguid) then
              IntfFlags:=IntfFlags or (1 shl ord(ifHasGuid));
            if (def.objecttype=odt_interfacecorba) and (def.iidstr^<>'') then
              IntfFlags:=IntfFlags or (1 shl ord(ifHasStrGUID));
            if (def.objecttype=odt_dispinterface) then
              IntfFlags:=IntfFlags or (1 shl ord(ifDispInterface));
            if (target_info.endian=endian_big) then
              IntfFlags:=reverse_byte(IntfFlags);
              {
              ifDispatch, }
            tcb.emit_ord_const(IntfFlags,u8inttype);

            { write GUID }
            tcb.emit_guid_const(def.iidguid^);

            { write unit name }
            tcb.emit_shortstring_const(current_module.realmodulename^);

            tcb.begin_anonymous_record('',defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin);

            { write iidstr }
            if def.objecttype=odt_interfacecorba then
              begin
                { prepareguid always allocates an empty string }
                if not assigned(def.iidstr) then
                  internalerror(2016021901);
                tcb.emit_shortstring_const(def.iidstr^)
              end;

            { write published properties for this object }
            published_properties_write_rtti_data(tcb,propnamelist,def.symtable);

            { write published methods for this interface }
            write_methods(tcb,def.symtable,[vis_published]);

            tcb.end_anonymous_record;
            tcb.end_anonymous_record;

            propnamelist.free;
          end;

        begin
           case def.objecttype of
             odt_class:
               tcb.emit_ord_const(tkclass,u8inttype);
             odt_object:
               tcb.emit_ord_const(tkobject,u8inttype);
             odt_dispinterface,
             odt_interfacecom:
               tcb.emit_ord_const(tkInterface,u8inttype);
             odt_interfacecorba:
               tcb.emit_ord_const(tkinterfaceCorba,u8inttype);
             odt_helper:
               tcb.emit_ord_const(tkhelper,u8inttype);
             else
               internalerror(200611034);
           end;

           { generate the name }
           tcb.emit_shortstring_const(def.objrealname^);

           tcb.begin_anonymous_record('',defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin);

           write_common_rtti_data(tcb,def,rt);

           case rt of
             initrtti :
               begin
                 if def.objecttype in [odt_class,odt_object,odt_helper] then
                   objectdef_rtti_fields(def)
                 else
                   objectdef_rtti_interface_init(def);
               end;
             fullrtti :
               begin
                 tcb.begin_anonymous_record('',
                   defaultpacking,reqalign,
                   targetinfos[target_info.system]^.alignment.recordalignmin);
                 case def.objecttype of
                   odt_helper,
                   odt_class:
                     objectdef_rtti_class_full(def);
                   odt_object:
                     objectdef_rtti_fields(def);
                 else
                   objectdef_rtti_interface_full(def);
                 end;
                 tcb.end_anonymous_record;
               end;
             else
               ;
           end;
           tcb.end_anonymous_record;
        end;

      begin
        case def.typ of
          variantdef :
            variantdef_rtti(tvariantdef(def));
          stringdef :
            stringdef_rtti(tstringdef(def));
          enumdef :
            enumdef_rtti(tenumdef(def));
          orddef :
            orddef_rtti(torddef(def));
          floatdef :
            floatdef_rtti(tfloatdef(def));
          setdef :
            setdef_rtti(tsetdef(def));
          procvardef :
            procvardef_rtti(tprocvardef(def));
          arraydef :
            begin
              if ado_IsBitPacked in tarraydef(def).arrayoptions then
                unknown_rtti(tstoreddef(def))
              else
                arraydef_rtti(tarraydef(def));
            end;
          recorddef :
            begin
              if trecorddef(def).is_packed then
                unknown_rtti(tstoreddef(def))
              else
                recorddef_rtti(trecorddef(def));
            end;
          objectdef :
            objectdef_rtti(tobjectdef(def));
          classrefdef :
            classrefdef_rtti(tclassrefdef(def));
          pointerdef :
            pointerdef_rtti(tpointerdef(def));
          else
            unknown_rtti(tstoreddef(def));
        end;
      end;

  procedure TRTTIWriter.write_attribute_data(tcb:ttai_typedconstbuilder;attr_list:trtti_attribute_list);

    procedure write_args(tbltcb:ttai_typedconstbuilder;attr:trtti_attribute);
      var
        argtcb : ttai_typedconstbuilder;
        arglab : tasmlabel;
        argdef : tdef;
        i : sizeint;
      begin
        if length(attr.paras)=0 then
          begin
            tbltcb.emit_tai(tai_const.Create_16bit(0),u16inttype);
            tbltcb.emit_tai(tai_const.Create_nil_dataptr,voidpointertype);
          end
        else
          begin
            current_asmdata.getglobaldatalabel(arglab);

            argtcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable]);

            argtcb.begin_anonymous_record('',defaultpacking,min(reqalign,SizeOf(PInt)),
              targetinfos[target_info.system]^.alignment.recordalignmin);

            for i:=0 to High(attr.paras) do
              begin
                case attr.paras[i].nodetype of
                  niln,
                  ordconstn,
                  realconstn,
                  stringconstn,
                  pointerconstn,
                  guidconstn:
                    tconstnode(attr.paras[i]).emit_data(argtcb);
                  setconstn:
                    tsetconstnode(attr.paras[i]).emit_data(argtcb);
                  else
                    internalerror(2019070803);
                end;
              end;

            argdef:=argtcb.end_anonymous_record;

            current_asmdata.asmlists[al_rtti].concatlist(
              argtcb.get_final_asmlist(arglab,argdef,sec_rodata,arglab.name,const_align(sizeof(pint)))
            );

            argtcb.free;

            { write argument size and the reference to the argument entry }
            tbltcb.emit_ord_const(argdef.size,u16inttype);
            tbltcb.emit_tai(Tai_const.Create_sym(arglab),voidpointertype);
          end;
      end;

    var
      count,i: word;
      attr : trtti_attribute;
      tbltcb : ttai_typedconstbuilder;
      tbllab : tasmlabel;
      tbldef : tdef;
    begin
      if assigned(attr_list) then
        count:=attr_list.get_attribute_count
      else
        count:=0;

      if count=0 then
        begin
          { write a Nil reference }
          tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
          exit;
        end;

      { first write the attribute list as a separate table }
      current_asmdata.getglobaldatalabel(tbllab);

      tbltcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable]);

      tbltcb.begin_anonymous_record(
        internaltypeprefixName[itp_rtti_attr_list]+tostr(count),
        defaultpacking,min(reqalign,SizeOf(PInt)),
        targetinfos[target_info.system]^.alignment.recordalignmin);
      tbltcb.emit_ord_const(count,u16inttype);
      for i:=0 to count-1 do
        begin
          tbltcb.begin_anonymous_record(internaltypeprefixName[itp_rtti_attr_entry],defaultpacking,min(reqalign,SizeOf(PInt)),
            targetinfos[target_info.system]^.alignment.recordalignmin);
          attr:=trtti_attribute(attr_list.rtti_attributes[i]);

          write_rtti_reference(tbltcb,ttypesym(attr.typesym).typedef,fullrtti);

          tbltcb.emit_procdef_const(tprocdef(tcallnode(attr.constructorcall).procdefinition));

          if not assigned(attr.constructorpd) then
            internalerror(2019071004);
          tbltcb.emit_procdef_const(tprocdef(attr.constructorpd));

          write_args(tbltcb,attr);

          tbltcb.end_anonymous_record;
        end;
      tbldef:=tbltcb.end_anonymous_record;

      current_asmdata.asmlists[al_rtti].concatlist(
        tbltcb.get_final_asmlist(tbllab,tbldef,sec_rodata,tbllab.name,const_align(sizeof(pint)))
      );

      tbltcb.free;

      { write the reference to the attribute table }
      tcb.emit_tai(Tai_const.Create_sym(tbllab),voidpointertype);
    end;


    function enumsym_compare_name(item1, item2: pointer): Integer;
      var
        enum1: tenumsym absolute item1;
        enum2: tenumsym absolute item2;
      begin
        if enum1=enum2 then
          result:=0
        else if enum1.name>enum2.name then
          result:=1
        else
          { there can't be equal names, identifiers are unique }
          result:=-1;
      end;


    function enumsym_compare_value(item1, item2: pointer): Integer;
      var
        enum1: tenumsym absolute item1;
        enum2: tenumsym absolute item2;
      begin
        if enum1.value>enum2.value then
          result:=1
        else if enum1.value<enum2.value then
          result:=-1
        else
          result:=0;
      end;


    procedure TRTTIWriter.write_rtti_extrasyms(def:Tdef;rt:Trttitype;mainrtti:Tasmsymbol);

        type Penumsym = ^Tenumsym;

        { Writes a helper table for accelerated conversion of ordinal enum values to strings.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_ord2stringindex(rttidef: trecorddef; const syms: tfplist);

        var rttilab:Tasmsymbol;
            h,i,o,prev_value:longint;
            mode:(lookup,search); {Modify with care, ordinal value of enum is written.}
            r:single;             {Must be real type because of integer overflow risk.}
            tcb: ttai_typedconstbuilder;
            sym_count: integer;
            tabledef: tdef;
        begin

          {Decide wether a lookup array is size efficient.}
          mode:=lookup;
          sym_count:=syms.count;
          if sym_count>0 then
            begin
              i:=1;
              r:=0;
              h:=tenumsym(syms[0]).value; {Next expected enum value is min.}
              { set prev_value for the first iteration to a value that is
                different from the first one without risking overflow (it's used
                to detect whether two enum values are the same) }
              if h=0 then
                prev_value:=1
              else
                prev_value:=0;
              while i<sym_count do
                begin
                  { if two enum values are the same, we have to create a table }
                  if (prev_value=h) then
                    begin
                      mode:=search;
                      break;
                    end;
                  {Calculate size of hole between values. Avoid integer overflows.}
                  r:=r+(single(tenumsym(syms[i]).value)-single(h))-1;
                  prev_value:=h;
                  h:=tenumsym(syms[i]).value;
                  inc(i);
                end;
              if r>sym_count then
                mode:=search; {Don't waste more than 50% space.}
            end;
          { write rtti data; make sure that the alignment matches the corresponding data structure
            in the code that uses it (if alignment is required). }
          tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_data_force_indirect]);
          { use TConstPtrUInt packrecords to ensure good alignment }
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          { now emit the data: first the mode }
          tcb.emit_tai(Tai_const.create_32bit(longint(mode)),u32inttype);
          { align }
          tcb.begin_anonymous_record('',defaultpacking,min(reqalign,sizeof(PInt)),
            targetinfos[target_info.system]^.alignment.recordalignmin);
          if mode=lookup then
            begin
              o:=tenumsym(syms[0]).value;  {Start with min value.}
              for i:=0 to sym_count-1 do
                begin
                  while o<tenumsym(syms[i]).value do
                    begin
                      tcb.emit_tai(Tai_const.create_nil_dataptr,ptruinttype);
                      inc(o);
                    end;
                  inc(o);
                  tcb.queue_init(voidpointertype);
                  tcb.queue_subscriptn_multiple_by_name(rttidef,
                    ['size_start_rec',
                      'typ_union_rec',
                      'min_max_rec',
                      'basetype_array_rec',
                      tsym(syms[i]).Name]
                  );
                  tcb.queue_emit_asmsym(mainrtti,rttidef);
                end;
            end
          else
            begin
              tcb.emit_ord_const(sym_count,u32inttype);
              tcb.begin_anonymous_record('',defaultpacking,min(reqalign,sizeof(PInt)),
                targetinfos[target_info.system]^.alignment.recordalignmin);
              for i:=0 to sym_count-1 do
                begin
                  tcb.emit_ord_const(tenumsym(syms[i]).value,s32inttype);
                  tcb.queue_init(voidpointertype);
                  tcb.queue_subscriptn_multiple_by_name(rttidef,
                    ['size_start_rec',
                      'typ_union_rec',
                      'min_max_rec',
                      'basetype_array_rec',
                      tsym(syms[i]).Name]
                  );
                  tcb.queue_emit_asmsym(mainrtti,rttidef);
                end;
              tcb.end_anonymous_record;
            end;
            tcb.end_anonymous_record;

            tabledef:=tcb.end_anonymous_record;
            rttilab:=current_asmdata.DefineAsmSymbol(Tstoreddef(def).rtti_mangledname(rt)+'_o2s',AB_GLOBAL,AT_DATA_NOINDIRECT,tabledef);
            current_asmdata.asmlists[al_rtti].concatlist(tcb.get_final_asmlist(
              rttilab,tabledef,sec_rodata,
              rttilab.name,sizeof(PInt)));
            tcb.free;

            current_module.add_public_asmsym(rttilab);
        end;


        { Writes a helper table for accelerated conversion of string to ordinal enum values.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_string2ordindex(rttidef: trecorddef; const syms: tfplist);

        var
          tcb: ttai_typedconstbuilder;
          rttilab: Tasmsymbol;
          i:longint;
          tabledef: tdef;
        begin
          { write rtti data }
          tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_data_force_indirect]);
          { begin of Tstring_to_ord }
          tcb.begin_anonymous_record('',defaultpacking,min(reqalign,sizeof(PInt)),
            targetinfos[target_info.system]^.alignment.recordalignmin);
          tcb.emit_ord_const(syms.count,s32inttype);
          { begin of "data" array in Tstring_to_ord }
          tcb.begin_anonymous_record('',defaultpacking,min(reqalign,sizeof(PInt)),
            targetinfos[target_info.system]^.alignment.recordalignmin);
          for i:=0 to syms.count-1 do
            begin
              tcb.emit_ord_const(tenumsym(syms[i]).value,s32inttype);
              { alignment of pointer value handled by enclosing record already }
              tcb.queue_init(voidpointertype);
              tcb.queue_subscriptn_multiple_by_name(rttidef,
                ['size_start_rec',
                  'typ_union_rec',
                  'min_max_rec',
                  'basetype_array_rec',
                  tsym(syms[i]).Name]
              );
              tcb.queue_emit_asmsym(mainrtti,rttidef);
            end;
          tcb.end_anonymous_record;
          tabledef:=tcb.end_anonymous_record;
          rttilab:=current_asmdata.DefineAsmSymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA_NOINDIRECT,tabledef);
          current_asmdata.asmlists[al_rtti].concatlist(tcb.get_final_asmlist(
            rttilab,tabledef,sec_rodata,
            rttilab.name,sizeof(PInt)));
          tcb.free;

          current_module.add_public_asmsym(rttilab);
        end;

        procedure enumdef_rtti_extrasyms(def:Tenumdef);
        var
          t:Tenumsym;
          syms:tfplist;
          i:longint;
          rttitypesym: ttypesym;
          rttidef: trecorddef;
        begin
          { collect enumsyms belonging to this enum type (could be a subsection
            in case of a subrange type) }
          syms:=tfplist.create;
          for i := 0 to def.symtable.SymList.Count - 1 do
            begin
              t:=tenumsym(def.symtable.SymList[i]);
              if t.value<def.minval then
                continue
              else
              if t.value>def.maxval then
                break;
              syms.add(t);
            end;
          { sort the syms by enum name }
          syms.sort(@enumsym_compare_name);
          rttitypesym:=try_search_current_module_type(internaltypeprefixName[itp_rttidef]+def.rtti_mangledname(fullrtti));
          if not assigned(rttitypesym) or
             (ttypesym(rttitypesym).typedef.typ<>recorddef) then
            internalerror(2015071402);
          rttidef:=trecorddef(ttypesym(rttitypesym).typedef);
          enumdef_rtti_string2ordindex(rttidef,syms);
          { sort the syms by enum value }
          syms.sort(@enumsym_compare_value);
          enumdef_rtti_ord2stringindex(rttidef,syms);
          syms.free;
        end;


    begin
      case def.typ of
        enumdef:
          if rt=fullrtti then
            begin
              enumdef_rtti_extrasyms(Tenumdef(def));
            end;
        else
          ;
      end;
    end;

    procedure TRTTIWriter.write_child_rtti_data(def:tdef;rt:trttitype);
      begin
        case def.typ of
          enumdef :
            if assigned(tenumdef(def).basedef) then
              write_rtti(tenumdef(def).basedef,rt);
          setdef :
            write_rtti(tsetdef(def).elementdef,rt);
          arraydef :
            begin
              write_rtti(tarraydef(def).rangedef,rt);
              write_rtti(tarraydef(def).elementdef,rt);
            end;
          recorddef :
            begin
              { guarantee initrtti for any record for RTTI purposes 
                also for fpc_initialize, fpc_finalize }
              if (rt=fullrtti) then
                begin
                  include(def.defstates,ds_init_table_used);
                  write_rtti(def, initrtti);
                end;
              fields_write_rtti(trecorddef(def).symtable,rt);
            end;
          objectdef :
            begin
              if assigned(tobjectdef(def).childof) then
                write_rtti(tobjectdef(def).childof,rt);
              if (rt=initrtti) or (tobjectdef(def).objecttype=odt_object) then
                fields_write_rtti(tobjectdef(def).symtable,rt)
              else
                published_write_rtti(tobjectdef(def).symtable,rt);

              if (rt=fullrtti) then
                begin
                  { guarantee initrtti for any object for RTTI purposes 
                    also for fpc_initialize, fpc_finalize }
                  if (tobjectdef(def).objecttype=odt_object) then
                    begin
                      include(def.defstates,ds_init_table_used);
                      write_rtti(def,initrtti);
                    end;
                  if (is_interface(def) or is_dispinterface(def))
                      and (oo_can_have_published in tobjectdef(def).objectoptions) then
                    methods_write_rtti(tobjectdef(def).symtable,rt,[vis_published],true);
                end;
            end;
          classrefdef,
          pointerdef:
            if not is_objc_class_or_protocol(tabstractpointerdef(def).pointeddef) then
              write_rtti(tabstractpointerdef(def).pointeddef,rt);
          procvardef:
            params_write_rtti(tabstractprocdef(def),rt,false);
          else
            ;
        end;
      end;

    procedure TRTTIWriter.write_rtti_reference(tcb: ttai_typedconstbuilder; def: tdef; rt: trttitype);
      begin
        { we don't care about the real type here, because
           a) we don't index into these elements
           b) we may not have the rtti type available at the point that we
              are emitting this data, because of forward definitions etc
           c) if the rtti is emitted in another unit, we won't have the type
              available at all
          For the cases where the type is emitted in the current unit and hence
          the underlying system will detect and complain about symbol def
          mismatches, type conversions will have to be inserted afterwards (like
          in llvm/llvmtype)
        }
        if not assigned(def) or is_void(def) or ((rt<>initrtti) and is_objc_class_or_protocol(def)) then
          tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype)
        else
          tcb.emit_tai(Tai_const.Create_sym(get_rtti_label(def,rt,true)),voidpointertype);
      end;


    function TRTTIWriter.ref_rtti(def:tdef;rt:trttitype;indirect:boolean;suffix:tsymstr):tasmsymbol;
      var
        s : tsymstr;
      begin
        s:=def.rtti_mangledname(rt)+suffix;
        result:=current_asmdata.RefAsmSymbol(s,AT_DATA,indirect);
        if def.owner.moduleid<>current_module.moduleid then
          current_module.add_extern_asmsym(s,AB_EXTERNAL,AT_DATA);
      end;

    procedure TRTTIWriter.write_rtti(def:tdef;rt:trttitype);
      var
        tcb: ttai_typedconstbuilder;
        rttilab: tasmsymbol;
        rttidef: tdef;
      begin
        { only write rtti of definitions from the current module }
        if not findunitsymtable(def.owner).iscurrentunit then
          exit;
        { check if separate initrtti is actually needed }
        if (rt=initrtti) and (not def.needs_separate_initrtti) then
          rt:=fullrtti;
        { prevent recursion }
        if rttidefstate[rt] in def.defstates then
          exit;
        include(def.defstates,rttidefstate[rt]);
        { write first all dependencies }
        write_child_rtti_data(def,rt);
        { write rtti data }
        tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_data_force_indirect]);
        tcb.begin_anonymous_record(
          internaltypeprefixName[itp_rttidef]+tstoreddef(def).rtti_mangledname(rt),
          defaultpacking,reqalign,
          targetinfos[target_info.system]^.alignment.recordalignmin
        );
        write_rtti_data(tcb,def,rt);
        rttidef:=tcb.end_anonymous_record;
        rttilab:=current_asmdata.DefineAsmSymbol(tstoreddef(def).rtti_mangledname(rt),AB_GLOBAL,AT_DATA_NOINDIRECT,rttidef);
        current_asmdata.AsmLists[al_rtti].concatList(
          tcb.get_final_asmlist(rttilab,rttidef,sec_rodata,rttilab.name,min(target_info.alignment.maxCrecordalign,SizeOf(QWord))));
        tcb.free;

        current_module.add_public_asmsym(rttilab);

        { write additional data }
        write_rtti_extrasyms(def,rt,rttilab);
      end;


    constructor TRTTIWriter.create;
      begin
        if tf_requires_proper_alignment in target_info.flags then
          begin
            reqalign:=min(sizeof(QWord),target_info.alignment.maxCrecordalign);
            defaultpacking:=C_alignment;
          end
        else
          begin
            reqalign:=1;
            defaultpacking:=1;
          end;
      end;


end.

