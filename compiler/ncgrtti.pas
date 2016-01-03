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
      cclasses,constexp,
      aasmbase,aasmcnst,
      symbase,symconst,symtype,symdef;

    type

      { TRTTIWriter }

      TRTTIWriter=class
      private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        { required internal alignment of the rtti data }
        reqalign: shortint;
        { required packing of all structures except for ttypeinfo and tpropinfo,
          which always use packrecords 1 }
        defaultpacking: shortint;

        procedure fields_write_rtti(st:tsymtable;rt:trttitype);
        procedure params_write_rtti(def:tabstractprocdef;rt:trttitype);
        procedure fields_write_rtti_data(tcb: ttai_typedconstbuilder; def: tabstractrecorddef; rt: trttitype);
=======
        procedure fields_write_rtti(st:tsymtable;rt:trttitype);
        procedure fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> graemeg/cpstrnew
=======
        procedure fields_write_rtti(st:tsymtable;rt:trttitype);
<<<<<<< HEAD
        procedure fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> graemeg/cpstrnew
=======
        procedure fields_write_rtti(st:tsymtable;rt:trttitype);
        procedure fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> graemeg/cpstrnew
=======
        procedure fields_write_rtti(st:tsymtable;rt:trttitype);
        procedure fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> origin/cpstrnew
=======
        procedure fields_write_rtti_data(st:tsymtable;rt:trttitype);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        procedure write_rtti_extrasyms(def:Tdef;rt:Trttitype;mainrtti:Tasmsymbol);
        procedure published_write_rtti(st:tsymtable;rt:trttitype);
        function  published_properties_count(st:tsymtable):longint;
        procedure published_properties_write_rtti_data(tcb: ttai_typedconstbuilder; propnamelist: TFPHashObjectList; st: tsymtable);
        procedure collect_propnamelist(propnamelist:TFPHashObjectList;objdef:tobjectdef);
        function  ref_rtti(def:tdef;rt:trttitype):tasmsymbol;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        procedure write_rtti_name(tcb: ttai_typedconstbuilder; def: tdef);
        procedure write_rtti_data(tcb: ttai_typedconstbuilder; def:tdef; rt: trttitype);
        procedure write_child_rtti_data(def:tdef;rt:trttitype);
        procedure write_rtti_reference(tcb: ttai_typedconstbuilder; def: tdef; rt: trttitype);
        procedure write_header(tcb: ttai_typedconstbuilder; def: tdef; typekind: byte);
=======
        procedure write_header(def: tdef; typekind: byte);
        procedure write_string(const s: string);
        procedure maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
        procedure write_header(def: tdef; typekind: byte);
        procedure write_string(const s: string);
        procedure maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
        procedure write_header(def: tdef; typekind: byte);
        procedure write_string(const s: string);
        procedure maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
        procedure write_header(def: tdef; typekind: byte);
        procedure write_string(const s: string);
        procedure maybe_write_align;
>>>>>>> origin/cpstrnew
      public
        constructor create;
        procedure write_rtti(def:tdef;rt:trttitype);
        function  get_rtti_label(def:tdef;rt:trttitype):tasmsymbol;
        function  get_rtti_label_ord2str(def:tdef;rt:trttitype):tasmsymbol;
        function  get_rtti_label_str2ord(def:tdef;rt:trttitype):tasmsymbol;
      end;

    { generate RTTI and init tables }
    procedure write_persistent_type_info(st:tsymtable;is_global:boolean);

    var
      RTTIWriter : TRTTIWriter;


implementation

    uses
       cutils,
       globals,globtype,verbose,systems,
<<<<<<< HEAD
<<<<<<< HEAD
       fmodule, procinfo,
       symtable,symsym,
       aasmtai,aasmdata,
       defutil,
       wpobase
=======
=======
>>>>>>> origin/fixes_2_2
       fmodule,
       symsym,
       aasmtai,aasmdata
>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TRTTIWriter.write_header(tcb: ttai_typedconstbuilder; def: tdef; typekind: byte);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure TRTTIWriter.maybe_write_align;
      begin
        if (tf_requires_proper_alignment in target_info.flags) then
          current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
      end;

    procedure TRTTIWriter.write_string(const s: string);
      begin
        current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(length(s)));
        current_asmdata.asmlists[al_rtti].concat(Tai_string.Create(s));
      end;

    procedure TRTTIWriter.write_header(def: tdef; typekind: byte);
      begin
        if def.typ=arraydef then
          InternalError(201012211);
        current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(typekind));
        if assigned(def.typesym) then
          write_string(ttypesym(def.typesym).realname)
        else
          current_asmdata.asmlists[al_rtti].concat(Tai_string.Create(#0));
      end;

    procedure TRTTIWriter.write_rtti_name(def:tdef);
>>>>>>> graemeg/cpstrnew
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
          targetinfos[target_info.system]^.alignment.recordalignmin,
          targetinfos[target_info.system]^.alignment.maxCrecordalign);
        if def.typ=arraydef then
          InternalError(201012211);
        tcb.emit_tai(Tai_const.Create_8bit(typekind),u8inttype);
        tcb.emit_shortstring_const(name);
        tcb.end_anonymous_record;
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

    { writes a 32-bit count followed by array of field infos for given symtable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TRTTIWriter.fields_write_rtti_data(tcb: ttai_typedconstbuilder; def: tabstractrecorddef; rt: trttitype);
=======
    procedure TRTTIWriter.fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> graemeg/cpstrnew
=======
    procedure TRTTIWriter.fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> graemeg/cpstrnew
=======
    procedure TRTTIWriter.fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> graemeg/cpstrnew
=======
    procedure TRTTIWriter.fields_write_rtti_data(def:tabstractrecorddef;rt:trttitype);
>>>>>>> origin/cpstrnew
      var
        i   : longint;
        sym : tsym;
        fieldcnt: longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        st: tsymtable;
        fields: tfplist;
        parentrtti: boolean;
      begin
        fieldcnt:=0;
        parentrtti:=false;
        st:=def.symtable;
        fields:=tfplist.create;
        fields.capacity:=st.symlist.count+1;
=======
        lastai: TLinkedListItem;
        st: tsymtable;
      begin
        fieldcnt:=0;
=======
        lastai: TLinkedListItem;
        st: tsymtable;
      begin
        fieldcnt:=0;
>>>>>>> graemeg/cpstrnew
=======
        lastai: TLinkedListItem;
        st: tsymtable;
      begin
        fieldcnt:=0;
>>>>>>> graemeg/cpstrnew
=======
        lastai: TLinkedListItem;
        st: tsymtable;
      begin
        fieldcnt:=0;
>>>>>>> origin/cpstrnew
        { Count will be inserted at this location. It cannot be nil as we've just
          written header for this symtable owner. But stay safe. }
        lastai:=current_asmdata.asmlists[al_rtti].last;
        if lastai=nil then
          InternalError(201012212);

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
        { For objects, treat parent (if any) as a field with offset 0. This
          provides correct handling of entire instance with RTL rtti routines. }
        if (def.typ=objectdef) and (tobjectdef(def).objecttype=odt_object) and
            Assigned(tobjectdef(def).childof) and
            ((rt=fullrtti) or (tobjectdef(def).childof.needs_inittable)) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           begin
             parentrtti:=true;
             inc(fieldcnt);
           end;

        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (tsym(sym).typ=fieldvarsym) and
               not(sp_static in tsym(sym).symoptions) and
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
=======
          begin
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tobjectdef(def).childof,rt)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(0));
            inc(fieldcnt);
          end;
        st:=def.symtable;
>>>>>>> graemeg/cpstrnew
=======
          begin
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tobjectdef(def).childof,rt)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(0));
            inc(fieldcnt);
          end;
        st:=def.symtable;
>>>>>>> graemeg/cpstrnew
=======
          begin
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tobjectdef(def).childof,rt)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(0));
            inc(fieldcnt);
          end;
        st:=def.symtable;
>>>>>>> graemeg/cpstrnew
=======
          begin
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tobjectdef(def).childof,rt)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(0));
            inc(fieldcnt);
          end;
        st:=def.symtable;
>>>>>>> origin/cpstrnew
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (tsym(sym).typ=fieldvarsym) and
               not(sp_static in tsym(sym).symoptions) and
               (
                (rt=fullrtti) or
                tfieldvarsym(sym).vardef.needs_inittable
               ) then
<<<<<<< HEAD
              write_rtti(tfieldvarsym(sym).vardef,rt);
=======
              begin
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tfieldvarsym(sym).vardef,rt)));
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(tfieldvarsym(sym).fieldoffset));
                inc(fieldcnt);
              end;
>>>>>>> graemeg/cpstrnew
          end;
        { insert field count before data }
        current_asmdata.asmlists[al_rtti].InsertAfter(Tai_const.Create_32bit(fieldcnt),lastai)
      end;


    procedure TRTTIWriter.params_write_rtti(def:tabstractprocdef;rt:trttitype);
      var
        i   : longint;
        sym : tparavarsym;
      begin
        for i:=0 to def.paras.count-1 do
          begin
<<<<<<< HEAD
            sym:=tparavarsym(def.paras[i]);
            if not (vo_is_hidden_para in sym.varoptions) then
              write_rtti(sym.vardef,rt);
=======
            sym:=tsym(st.SymList[i]);
            if (tsym(sym).typ=fieldvarsym) and
               not(sp_static in tsym(sym).symoptions) and
               (
                (rt=fullrtti) or
                tfieldvarsym(sym).vardef.needs_inittable
               ) then
              write_rtti(tfieldvarsym(sym).vardef,rt);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
           extnumber: longint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
                    tcb.emit_procdef_const(tprocdef(propaccesslist.procdef));
                    typvalue:=1;
                  end
                else
                  begin
<<<<<<< HEAD
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
=======
                     { virtual method, write vmt offset }
                     current_asmdata.asmlists[al_rtti].concat(Tai_const.create(aitconst_ptr,
                       tobjectdef(tprocdef(propaccesslist.procdef).struct).vmtmethodoffset(tprocdef(propaccesslist.procdef).extnumber)));
                     { register for wpo }
                     tobjectdef(tprocdef(propaccesslist.procdef).struct).register_vmt_call(tprocdef(propaccesslist.procdef).extnumber);
                     {$ifdef vtentry}
                     { not sure if we can insert those vtentry symbols safely here }
                     {$error register methods used for published properties}
                     {$endif vtentry}
                     typvalue:=2;
>>>>>>> graemeg/cpstrnew
                  end;
             end;
           proctypesinfo:=proctypesinfo or (typvalue shl shiftvalue);
        end;

      begin
        tcb.begin_anonymous_record('',defaultpacking,reqalign,
          targetinfos[target_info.system]^.alignment.recordalignmin,
          targetinfos[target_info.system]^.alignment.maxCrecordalign);
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
                  1,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin,
                  targetinfos[target_info.system]^.alignment.maxCrecordalign);
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
<<<<<<< HEAD
                tcb.emit_ord_const(propnameitem.propindex,u16inttype);
                tcb.emit_ord_const(proctypesinfo,u8inttype);
                tcb.emit_shortstring_const(tpropertysym(sym).realname);
                tcb.end_anonymous_record;
=======
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_16bit(propnameitem.propindex));
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(proctypesinfo));
<<<<<<< HEAD
                write_string(tpropertysym(sym).realname);
                maybe_write_align;
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
=======
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(length(tpropertysym(sym).realname)));
                current_asmdata.asmlists[al_rtti].concat(Tai_string.Create(tpropertysym(sym).realname));
                if (tf_requires_proper_alignment in target_info.flags) then
                  current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             end;
          end;
        tcb.end_anonymous_record;
      end;


    procedure TRTTIWriter.write_rtti_data(tcb: ttai_typedconstbuilder; def: tdef; rt: trttitype);

        procedure unknown_rtti(def:tstoreddef);
        begin
          tcb.emit_ord_const(tkUnknown,u8inttype);
          write_rtti_name(tcb,def);
        end;

        procedure variantdef_rtti(def:tvariantdef);
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          write_header(tcb,def,tkVariant);
=======
          write_header(def,tkVariant);
>>>>>>> graemeg/cpstrnew
=======
          write_header(def,tkVariant);
>>>>>>> graemeg/cpstrnew
=======
          write_header(def,tkVariant);
>>>>>>> graemeg/cpstrnew
=======
          write_header(def,tkVariant);
>>>>>>> origin/cpstrnew
        end;

        procedure stringdef_rtti(def:tstringdef);
        begin
          case def.stringtype of
            st_ansistring:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              begin
                write_header(tcb,def,tkAString);
                { align }
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_rtti_ansistr],
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin,
                  targetinfos[target_info.system]^.alignment.maxCrecordalign);
                tcb.emit_ord_const(def.encoding,u16inttype);
                tcb.end_anonymous_record;
              end;

            st_widestring:
              write_header(tcb,def,tkWString);

            st_unicodestring:
              write_header(tcb,def,tkUString);

            st_longstring:
              write_header(tcb,def,tkLString);

            st_shortstring:
              begin
                 write_header(tcb,def,tkSString);
                 tcb.emit_ord_const(def.len,u8inttype);
=======
              write_header(def,tkAString);

            st_widestring:
              write_header(def,tkWString);

            st_unicodestring:
              write_header(def,tkUString);

            st_longstring:
=======
              write_header(def,tkAString);

            st_widestring:
              write_header(def,tkWString);

            st_unicodestring:
              write_header(def,tkUString);

            st_longstring:
>>>>>>> graemeg/cpstrnew
=======
              write_header(def,tkAString);

            st_widestring:
              write_header(def,tkWString);

            st_unicodestring:
              write_header(def,tkUString);

            st_longstring:
>>>>>>> graemeg/cpstrnew
=======
              write_header(def,tkAString);

            st_widestring:
              write_header(def,tkWString);

            st_unicodestring:
              write_header(def,tkUString);

            st_longstring:
>>>>>>> origin/cpstrnew
              write_header(def,tkLString);

            st_shortstring:
              begin
                 write_header(def,tkSString);
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(def.len));
<<<<<<< HEAD
<<<<<<< HEAD
                 maybe_write_align;  // is align necessary here?
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
=======
                 if (tf_requires_proper_alignment in target_info.flags) then
                   current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
>>>>>>> graemeg/fixes_2_2
=======
                 if (tf_requires_proper_alignment in target_info.flags) then
                   current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
>>>>>>> origin/fixes_2_2
              end;
          end;
        end;

        procedure enumdef_rtti(def: tenumdef);
        var
           i  : integer;
           hp : tenumsym;
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          write_header(tcb,def,tkEnumeration);
          { align; the named fields are so that we can let the compiler
            calculate the string offsets later on }
          tcb.next_field_name:='size_start_rec';
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
=======
          write_header(def,tkEnumeration);
          maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
          write_header(def,tkEnumeration);
          maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
          write_header(def,tkEnumeration);
          maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
          write_header(def,tkEnumeration);
          maybe_write_align;
>>>>>>> origin/cpstrnew
=======
          current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkEnumeration));
          write_rtti_name(def);
          if (tf_requires_proper_alignment in target_info.flags) then
            current_asmdata.asmlists[al_rtti].concat(Cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          case longint(def.size) of
            1 :
              tcb.emit_ord_const(otUByte,u8inttype);
            2 :
              tcb.emit_ord_const(otUWord,u8inttype);
            4 :
              tcb.emit_ord_const(otULong,u8inttype);
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { we need to align by Tconstptruint here to satisfy the alignment
            rules set by records: in the typinfo unit we overlay a TTypeData
            record on this data, which at the innermost variant record needs an
            alignment of TConstPtrUint due to e.g. the "CompType" member for
            tkSet (also the "BaseType" member for tkEnumeration).

            We need to adhere to this, otherwise things will break. }
          tcb.next_field_name:='min_max_rec';
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
          tcb.emit_ord_const(def.min,s32inttype);
          tcb.emit_ord_const(def.max,s32inttype);
          tcb.next_field_name:='basetype_array_rec';
          { all strings must appear right after each other -> from now on
            packrecords 1 (but the start must still be aligned) }
          tcb.begin_anonymous_record('',1,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
          { write base type }
          write_rtti_reference(tcb,def.basedef,rt);
          for i:=0 to def.symtable.SymList.Count-1 do
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          { we need to align by Tconstptruint here to satisfy the alignment rules set by
            records: in the typinfo unit we overlay a TTypeData record on this data, which at
            the innermost variant record needs an alignment of TConstPtrUint due to e.g. 
            the "CompType" member for tkSet (also the "BaseType" member for tkEnumeration).
            We need to adhere to this, otherwise things will break.
            Note that other code (e.g. enumdef_rtti_calcstringtablestart()) relies on the
            exact sequence too. }
          maybe_write_align;
=======
          if (tf_requires_proper_alignment in target_info.flags) then
            current_asmdata.asmlists[al_rtti].concat(Cai_align.Create(sizeof(TConstPtrUInt)));
>>>>>>> graemeg/fixes_2_2
=======
          if (tf_requires_proper_alignment in target_info.flags) then
            current_asmdata.asmlists[al_rtti].concat(Cai_align.Create(sizeof(TConstPtrUInt)));
>>>>>>> origin/fixes_2_2
          current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(def.min));
          current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(def.max));
          maybe_write_align;  // is align necessary here?
          { write base type }
          if assigned(def.basedef) then
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.basedef,rt)))
          else
            current_asmdata.asmlists[al_rtti].concat(Tai_const.create_sym(nil));
          for i := 0 to def.symtable.SymList.Count - 1 do
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
              hp:=tenumsym(def.symtable.SymList[i]);
              if hp.value<def.minval then
                continue
              else
              if hp.value>def.maxval then
                break;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
              write_string(hp.realname);
            end;
          { write unit name }
          write_string(current_module.realmodulename^);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
              write_string(hp.realname);
            end;
          { write unit name }
          write_string(current_module.realmodulename^);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        end;

        procedure orddef_rtti(def:torddef);

          procedure dointeger(typekind: byte);
          const
            trans : array[tordtype] of byte =
              (otUByte{otNone},
               otUByte,otUWord,otULong,otUByte{otNone},otUByte{otNone},
               otSByte,otSWord,otSLong,otUByte{otNone},otUByte{otNone},
               otUByte,otUWord,otULong,otUByte,
               otSByte,otSWord,otSLong,otSByte,
               otUByte,otUWord,otUByte);
          begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            write_header(tcb,def,typekind);
            tcb.begin_anonymous_record(
              internaltypeprefixName[itp_rtti_ord_outer],
              defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin,
              targetinfos[target_info.system]^.alignment.maxCrecordalign);
            tcb.emit_ord_const(byte(trans[def.ordtype]),u8inttype);
            tcb.begin_anonymous_record(
              internaltypeprefixName[itp_rtti_ord_inner],
              defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin,
              targetinfos[target_info.system]^.alignment.maxCrecordalign);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            write_header(def,typekind);
            maybe_write_align;
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(byte(trans[def.ordtype])));
            maybe_write_align;
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
            {Convert to longint to smuggle values in high(longint)+1..high(cardinal) into asmlist.}
            tcb.emit_ord_const(longint(def.low.svalue),s32inttype);
            tcb.emit_ord_const(longint(def.high.svalue),s32inttype);
            tcb.end_anonymous_record;
            tcb.end_anonymous_record;
=======
            write_rtti_name(def);
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(byte(trans[def.ordtype])));
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(longint(def.low)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(longint(def.high)));
>>>>>>> graemeg/fixes_2_2
          end;

        begin
          case def.ordtype of
            s64bit :
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                write_header(tcb,def,tkInt64);
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_rtti_ord_64bit],
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin,
                  targetinfos[target_info.system]^.alignment.maxCrecordalign);
=======
                write_header(def,tkInt64);
                maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
                write_header(def,tkInt64);
                maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
                write_header(def,tkInt64);
                maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
                write_header(def,tkInt64);
                maybe_write_align;
>>>>>>> origin/cpstrnew
=======
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkInt64));
                write_rtti_name(def);
                if (tf_requires_proper_alignment in target_info.flags) then
                  current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                { low }
                tcb.emit_ord_const(def.low.svalue,s64inttype);
                { high }
                tcb.emit_ord_const(def.high.svalue,s64inttype);
                tcb.end_anonymous_record;
              end;
            u64bit :
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                write_header(tcb,def,tkQWord);
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_rtti_ord_64bit],
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin,
                  targetinfos[target_info.system]^.alignment.maxCrecordalign);
                { use svalue because emit_ord_const accepts int64, prevents
                  range check errors }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                write_header(def,tkQWord);
                maybe_write_align;
                {use svalue because Create_64bit accepts int64, prevents range checks}
>>>>>>> graemeg/cpstrnew
=======
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkQWord));
                write_rtti_name(def);
                if (tf_requires_proper_alignment in target_info.flags) then
                  current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                { low }
                tcb.emit_ord_const(def.low.svalue,s64inttype);
                { high }
<<<<<<< HEAD
                tcb.emit_ord_const(def.high.svalue,s64inttype);
                tcb.end_anonymous_record;
              end;
            pasbool8:
=======
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_64bit(def.high.svalue));
              end;
            pasbool:
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
                dointeger(tkBool);
            uchar:
                dointeger(tkChar);
            uwidechar:
                dointeger(tkWChar);
            scurrency:
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                write_header(tcb,def,tkFloat);
                tcb.begin_anonymous_record(
                  internaltypeprefixName[itp_1byte],
                  defaultpacking,reqalign,
                  targetinfos[target_info.system]^.alignment.recordalignmin,
                  targetinfos[target_info.system]^.alignment.maxCrecordalign);
                tcb.emit_ord_const(ftCurr,u8inttype);
                tcb.end_anonymous_record;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                write_header(def,tkFloat);
                maybe_write_align;
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ftCurr));
>>>>>>> graemeg/cpstrnew
              end;
            scurrency:
<<<<<<< HEAD
=======
              begin
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkFloat));
                write_rtti_name(def);
                if (tf_requires_proper_alignment in target_info.flags) then
                  current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ftCurr));
              end;
            else
>>>>>>> origin/fixes_2_2
              begin
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkFloat));
                write_rtti_name(def);
                if (tf_requires_proper_alignment in target_info.flags) then
                  current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
                current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ftCurr));
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           write_header(tcb,def,tkFloat);
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_1byte],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin,
             targetinfos[target_info.system]^.alignment.maxCrecordalign);
           tcb.emit_ord_const(translate[def.floattype],u8inttype);
           tcb.end_anonymous_record;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
           write_header(def,tkFloat);
           maybe_write_align;
=======
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkFloat));
           write_rtti_name(def);
           if (tf_requires_proper_alignment in target_info.flags) then
             current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(translate[def.floattype]));
>>>>>>> graemeg/cpstrnew
        end;


        procedure setdef_rtti(def:tsetdef);
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           write_header(tcb,def,tkSet);
           tcb.begin_anonymous_record(
             internaltypeprefixName[itp_1byte],
             defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin,
             targetinfos[target_info.system]^.alignment.maxCrecordalign);
=======
           write_header(def,tkSet);
           maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
           write_header(def,tkSet);
           maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
           write_header(def,tkSet);
           maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
           write_header(def,tkSet);
           maybe_write_align;
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           { since this record has an alignment of reqalign, its size will also
             be rounded up to a multiple of reqalign -> the following value will
             also be properly aligned without having to start an extra record }
           tcb.end_anonymous_record;
           write_rtti_reference(tcb,def.elementdef,rt);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
           maybe_write_align;
=======
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkSet));
           write_rtti_name(def);
           if (tf_requires_proper_alignment in target_info.flags) then
             current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
           case def.size of
             1:
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(otUByte));
             2:
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(otUWord));
             4:
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(otULong));
           end;
           if (tf_requires_proper_alignment in target_info.flags) then
             current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.elementdef,rt)));
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
             tcb.emit_ord_const(tkArray,u8inttype);
           write_rtti_name(tcb,def);
=======
             current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkarray));
           write_rtti_name(def);
<<<<<<< HEAD
<<<<<<< HEAD
           maybe_write_align;
           { size of elements }
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_pint(def.elesize));
>>>>>>> graemeg/cpstrnew

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
                 targetinfos[target_info.system]^.alignment.recordalignmin,
                 targetinfos[target_info.system]^.alignment.maxCrecordalign);
               { total size = elecount * elesize of the first arraydef }
               tcb.emit_tai(Tai_const.Create_pint(def.elecount*def.elesize),ptruinttype);
               { total element count }
               tcb.emit_tai(Tai_const.Create_pint(pint(totalcount)),ptruinttype);
               { last dimension element type }
               tcb.emit_tai(Tai_const.Create_sym(ref_rtti(curdef.elementdef,rt)),voidpointertype);
               { dimension count }
               tcb.emit_ord_const(dimcount,u8inttype);
               finaldef:=def;
               { ranges of the dimensions }
               for i:=1 to dimcount do
                 begin
                   curdef:=tarraydef(finaldef);
                   finaldef:=curdef.elementdef;
                   { Dims[i] PTypeInfo }
                   write_rtti_reference(tcb,curdef.rangedef,rt);
                 end;
=======
=======
>>>>>>> origin/fixes_2_2
           if (tf_requires_proper_alignment in target_info.flags) then
             current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
           { size of elements }
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_aint(def.elesize));

           if not(ado_IsDynamicArray in def.arrayoptions) then
             begin
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_aint(aint(def.elecount)));
               { element type }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.elementdef,rt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             end
           else
             { write a delphi almost compatible dyn. array entry:
               there are two types, eltype and eltype2, the latter is nil if the element type needs
               no finalization, the former is always valid, delphi has this swapped, but for
               compatibility with older fpc versions we do it different, to be delphi compatible,
               the names are swapped in typinfo.pp
             }
             begin
<<<<<<< HEAD
<<<<<<< HEAD
               tcb.begin_anonymous_record(
                 internaltypeprefixName[itp_rtti_dyn_array],
                 defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin,
                 targetinfos[target_info.system]^.alignment.maxCrecordalign);
               { size of elements }
               tcb.emit_tai(Tai_const.Create_pint(def.elesize),ptruinttype);
               { element type }
               write_rtti_reference(tcb,def.elementdef,rt);
               { variant type }
               tcb.emit_ord_const(tstoreddef(def.elementdef).getvardef,s32inttype);
               { element type }
               if def.elementdef.needs_inittable then
                 write_rtti_reference(tcb,def.elementdef,rt)
               else
<<<<<<< HEAD
                 tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);
               { write unit name }
               tcb.emit_shortstring_const(current_module.realmodulename^);
=======
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_pint(0));
               { write unit name }
               write_string(current_module.realmodulename^);
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
          tcb.end_anonymous_record;
        end;

        procedure classrefdef_rtti(def:tclassrefdef);
        begin
          write_header(tcb,def,tkClassRef);
          { will be aligned thanks to encompassing record }
          write_rtti_reference(tcb,def.pointeddef,rt);
        end;

        procedure pointerdef_rtti(def:tpointerdef);
        begin
          write_header(tcb,def,tkPointer);
          { will be aligned thanks to encompassing record }
          write_rtti_reference(tcb,def.pointeddef,rt);
=======
=======
>>>>>>> origin/fixes_2_2
               { element type }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.elementdef,rt)));
             end;
           { variant type }
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(tstoreddef(def.elementdef).getvardef));
           if ado_IsDynamicArray in def.arrayoptions then
             begin
               { element type }
               if def.elementdef.needs_inittable then
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.elementdef,rt)))
               else
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_aint(0));
               { dummy DynUnitName }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(0));
             end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;

        procedure recorddef_rtti(def:trecorddef);
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           write_header(tcb,def,tkRecord);
           { need extra reqalign record, because otherwise the u32 int will
             only be aligned to 4 even on 64 bit target (while the rtti code
             in typinfo expects alignments to sizeof(pointer)) }
           tcb.begin_anonymous_record('',defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin,
             targetinfos[target_info.system]^.alignment.maxCrecordalign);
           tcb.emit_ord_const(def.size,u32inttype);
           fields_write_rtti_data(tcb,def,rt);
           tcb.end_anonymous_record;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
           write_header(def,tkRecord);
           maybe_write_align;
=======
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkrecord));
           write_rtti_name(def);
           if (tf_requires_proper_alignment in target_info.flags) then
             current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(def.size));
           fields_write_rtti_data(def,rt);
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


        procedure procvardef_rtti(def:tprocvardef);
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
              { pocall_interrupt  } 12
             );

           procedure write_param_flag(parasym:tparavarsym);
             var
               paraspec : byte;
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
               if is_open_array(parasym.vardef) then
                 paraspec:=paraspec or pfArray or pfReference;
               { and these for classes and interfaces (maybe because they
                 are themselves addresses?)
               }
               if is_class_or_interface(parasym.vardef) then
                 paraspec:=paraspec or pfAddress;
               { set bits run from the highest to the lowest bit on
                 big endian systems
               }
               if (target_info.endian = endian_big) then
                 paraspec:=reverse_byte(paraspec);
               { write flags for current parameter }
               tcb.emit_ord_const(paraspec,u8inttype);
             end;

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
              { pocall_interrupt  } 12
             );

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
              { pocall_interrupt  } 12
             );

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
              { pocall_interrupt  } 12
             );

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
              { pocall_interrupt  } 12
             );

           procedure write_para(parasym:tparavarsym);
<<<<<<< HEAD
             begin
               { only store user visible parameters }
               if not(vo_is_hidden_para in parasym.varoptions) then
                 begin
                   { write flags for current parameter }
                   write_param_flag(parasym);
                   { write name of current parameter }
                   tcb.emit_shortstring_const(parasym.realname);
                   { write name of type of current parameter }
                   write_rtti_name(tcb,parasym.vardef);
                 end;
             end;

           procedure write_procedure_param(parasym:tparavarsym);
             begin
               { only store user visible parameters }
               if not(vo_is_hidden_para in parasym.varoptions) then
                 begin
                   { every parameter is expected to start aligned }
                   tcb.begin_anonymous_record(
                     internaltypeprefixName[itp_rtti_proc_param]+tostr(length(parasym.realname)),
                     defaultpacking,reqalign,
                     targetinfos[target_info.system]^.alignment.recordalignmin,
                     targetinfos[target_info.system]^.alignment.maxCrecordalign);
                   { write flags for current parameter }
                   write_param_flag(parasym);
                   { write param type }
                   write_rtti_reference(tcb,parasym.vardef,fullrtti);
                   { write name of current parameter }
                   tcb.emit_shortstring_const(parasym.realname);
                   tcb.end_anonymous_record;
                 end;
             end;
=======
           var
             paraspec : byte;
           begin
             { only store user visible parameters }
             if not(vo_is_hidden_para in parasym.varoptions) then
               begin
                 case parasym.varspez of
                   vs_value   : paraspec := 0;
                   vs_const   : paraspec := pfConst;
                   vs_var     : paraspec := pfVar;
                   vs_out     : paraspec := pfOut;
                   vs_constref: paraspec := pfConstRef;
                 end;
                 { Kylix also seems to always add both pfArray and pfReference
                   in this case
                 }
                 if is_open_array(parasym.vardef) then
                   paraspec:=paraspec or pfArray or pfReference;
                 { and these for classes and interfaces (maybe because they
                   are themselves addresses?)
                 }
                 if is_class_or_interface(parasym.vardef) then
                   paraspec:=paraspec or pfAddress;
                 { set bits run from the highest to the lowest bit on
                   big endian systems
                 }
                 if (target_info.endian = endian_big) then
                   paraspec:=reverse_byte(paraspec);
                 { write flags for current parameter }
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(paraspec));
                 { write name of current parameter }
                 write_string(parasym.realname);
                 { write name of type of current parameter }
                 write_rtti_name(parasym.vardef);
               end;
           end;
>>>>>>> graemeg/cpstrnew

        var
          methodkind : byte;
          i : integer;
        begin
          if po_methodpointer in def.procoptions then
            begin
               { write method id and name }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               write_header(tcb,def,tkMethod);
               tcb.begin_anonymous_record('',defaultpacking,reqalign,
                 targetinfos[target_info.system]^.alignment.recordalignmin,
                 targetinfos[target_info.system]^.alignment.maxCrecordalign);
=======
               write_header(def,tkMethod);
               maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
               write_header(def,tkMethod);
               maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
               write_header(def,tkMethod);
               maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
               write_header(def,tkMethod);
               maybe_write_align;
>>>>>>> origin/cpstrnew

               { write kind of method }
               case def.proctypeoption of
                 potype_constructor: methodkind:=mkConstructor;
                 potype_destructor: methodkind:=mkDestructor;
                 potype_class_constructor: methodkind:=mkClassConstructor;
                 potype_class_destructor: methodkind:=mkClassDestructor;
                 potype_operator: methodkind:=mkOperatorOverload;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 potype_procedure:
                   if po_classmethod in def.procoptions then
=======
                 potype_procedure: 
                   if po_classmethod in def.procoptions then 
>>>>>>> graemeg/cpstrnew
=======
                 potype_procedure: 
                   if po_classmethod in def.procoptions then 
>>>>>>> graemeg/cpstrnew
=======
                 potype_procedure: 
                   if po_classmethod in def.procoptions then 
>>>>>>> graemeg/cpstrnew
=======
                 potype_procedure: 
                   if po_classmethod in def.procoptions then 
>>>>>>> origin/cpstrnew
                     methodkind:=mkClassProcedure
                   else
                     methodkind:=mkProcedure;
                 potype_function:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   if po_classmethod in def.procoptions then
=======
                   if po_classmethod in def.procoptions then 
>>>>>>> graemeg/cpstrnew
=======
                   if po_classmethod in def.procoptions then 
>>>>>>> graemeg/cpstrnew
=======
                   if po_classmethod in def.procoptions then 
>>>>>>> graemeg/cpstrnew
=======
                   if po_classmethod in def.procoptions then 
>>>>>>> origin/cpstrnew
                     methodkind:=mkClassFunction
                   else
                     methodkind:=mkFunction;
=======
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkmethod));
               write_rtti_name(def);
               if (tf_requires_proper_alignment in target_info.flags) then
                 current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));

               { write kind of method (can only be function or procedure)}
               if def.returndef = voidtype then
                 methodkind := mkProcedure
>>>>>>> graemeg/fixes_2_2
               else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 begin
=======
                 begin                   
>>>>>>> graemeg/cpstrnew
=======
                 begin                   
>>>>>>> graemeg/cpstrnew
=======
                 begin                   
>>>>>>> graemeg/cpstrnew
=======
                 begin                   
>>>>>>> origin/cpstrnew
                   if def.returndef = voidtype then
                     methodkind:=mkProcedure
                   else
                     methodkind:=mkFunction;
                 end;
               end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               tcb.emit_ord_const(methodkind,u8inttype);

               { write parameter info. The parameters must be written in reverse order
                 if this method uses right to left parameter pushing! }
               tcb.emit_ord_const(def.maxparacount,u8inttype);

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
               tcb.emit_ord_const(ProcCallOptionToCallConv[def.proccalloption],u8inttype);

               { enclosing record takes care of alignment }
               { write params typeinfo }
               for i:=0 to def.paras.count-1 do
                 if not(vo_is_hidden_para in tparavarsym(def.paras[i]).varoptions) then
                   write_rtti_reference(tcb,tparavarsym(def.paras[i]).vardef,fullrtti);
               tcb.end_anonymous_record;
            end
          else
            begin
              write_header(tcb,def,tkProcvar);
              tcb.begin_anonymous_record('',defaultpacking,reqalign,
                targetinfos[target_info.system]^.alignment.recordalignmin,
                targetinfos[target_info.system]^.alignment.maxCrecordalign);

              { flags }
              tcb.emit_ord_const(0,u8inttype);
              { write calling convention }
              tcb.emit_ord_const(ProcCallOptionToCallConv[def.proccalloption],u8inttype);
              { enclosing record takes care of alignment }
              { write result typeinfo }
              write_rtti_reference(tcb,def.returndef,fullrtti);
              { write parameter count }
              tcb.emit_ord_const(def.maxparacount,u8inttype);
              for i:=0 to def.paras.count-1 do
                write_procedure_param(tparavarsym(def.paras[i]));
              tcb.end_anonymous_record;
            end;
=======
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(methodkind));

               { write parameter info. The parameters must be written in reverse order
                 if this method uses right to left parameter pushing! }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(def.maxparacount));

               for i:=0 to def.paras.count-1 do
                 write_para(tparavarsym(def.paras[i]));

               if (methodkind=mkFunction) or (methodkind=mkClassFunction) then
               begin
                 { write name of result type }
                 write_rtti_name(def.returndef);
                 maybe_write_align;

                 { write result typeinfo }
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.returndef,fullrtti)))
               end;

               { write calling convention }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ProcCallOptionToCallConv[def.proccalloption]));
               maybe_write_align;

               { write params typeinfo }
               for i:=0 to def.paras.count-1 do
                 if not(vo_is_hidden_para in tparavarsym(def.paras[i]).varoptions) then
                   current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tparavarsym(def.paras[i]).vardef,fullrtti)));
            end
          else
            write_header(def,tkProcvar);
>>>>>>> graemeg/cpstrnew
=======
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(methodkind));

               { write parameter info. The parameters must be written in reverse order
                 if this method uses right to left parameter pushing! }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(def.maxparacount));

               for i:=0 to def.paras.count-1 do
                 write_para(tparavarsym(def.paras[i]));

               if (methodkind=mkFunction) or (methodkind=mkClassFunction) then
               begin
                 { write name of result type }
                 write_rtti_name(def.returndef);
                 maybe_write_align;

                 { write result typeinfo }
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.returndef,fullrtti)))
               end;

               { write calling convention }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ProcCallOptionToCallConv[def.proccalloption]));
               maybe_write_align;

               { write params typeinfo }
               for i:=0 to def.paras.count-1 do
                 if not(vo_is_hidden_para in tparavarsym(def.paras[i]).varoptions) then
                   current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tparavarsym(def.paras[i]).vardef,fullrtti)));
            end
          else
            write_header(def,tkProcvar);
>>>>>>> graemeg/cpstrnew
=======
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(methodkind));

               { write parameter info. The parameters must be written in reverse order
                 if this method uses right to left parameter pushing! }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(def.maxparacount));

               for i:=0 to def.paras.count-1 do
                 write_para(tparavarsym(def.paras[i]));

               if (methodkind=mkFunction) or (methodkind=mkClassFunction) then
               begin
                 { write name of result type }
                 write_rtti_name(def.returndef);
                 maybe_write_align;

                 { write result typeinfo }
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.returndef,fullrtti)))
               end;

               { write calling convention }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ProcCallOptionToCallConv[def.proccalloption]));
               maybe_write_align;

               { write params typeinfo }
               for i:=0 to def.paras.count-1 do
                 if not(vo_is_hidden_para in tparavarsym(def.paras[i]).varoptions) then
                   current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tparavarsym(def.paras[i]).vardef,fullrtti)));
            end
          else
            write_header(def,tkProcvar);
>>>>>>> graemeg/cpstrnew
=======
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(methodkind));

               { write parameter info. The parameters must be written in reverse order
                 if this method uses right to left parameter pushing! }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(def.maxparacount));

               for i:=0 to def.paras.count-1 do
                 write_para(tparavarsym(def.paras[i]));

               if (methodkind=mkFunction) or (methodkind=mkClassFunction) then
               begin
                 { write name of result type }
                 write_rtti_name(def.returndef);
                 maybe_write_align;

                 { write result typeinfo }
                 current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.returndef,fullrtti)))
               end;

               { write calling convention }
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(ProcCallOptionToCallConv[def.proccalloption]));
               maybe_write_align;

               { write params typeinfo }
               for i:=0 to def.paras.count-1 do
                 if not(vo_is_hidden_para in tparavarsym(def.paras[i]).varoptions) then
                   current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(tparavarsym(def.paras[i]).vardef,fullrtti)));
            end
          else
            write_header(def,tkProcvar);
>>>>>>> origin/cpstrnew
        end;


        procedure objectdef_rtti(def: tobjectdef);

          procedure objectdef_rtti_fields(def:tobjectdef);
          begin
<<<<<<< HEAD
            tcb.emit_ord_const(def.size, u32inttype);
            { enclosing record takes care of alignment }
            fields_write_rtti_data(tcb,def,rt);
=======
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(def.size));
            fields_write_rtti_data(def,rt);
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

            if not is_objectpascal_helper(def) then
              if (oo_has_vmt in def.objectoptions) then
                tcb.emit_tai(
                  Tai_const.Createname(def.vmt_mangledname,AT_DATA,0),
                  cpointerdef.getreusable(def.vmt_def))
              else
                tcb.emit_tai(Tai_const.Create_nil_dataptr,voidpointertype);

            { write parent typeinfo }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            write_rtti_reference(tcb,def.childof,fullrtti);

            { write typeinfo of extended type }
            if is_objectpascal_helper(def) then
              if assigned(def.extendeddef) then
                write_rtti_reference(tcb,def.extendeddef,fullrtti)
              else
                InternalError(2011033001);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            if assigned(def.childof) then
              current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_sym(ref_rtti(def.childof,fullrtti)))
            else
              current_asmdata.asmlists[al_rtti].concat(Tai_const.create_sym(nil));
>>>>>>> graemeg/cpstrnew

            { total number of unique properties }
            tcb.emit_ord_const(propnamelist.count,u16inttype);

            { write unit name }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            tcb.emit_shortstring_const(current_module.realmodulename^);

            { write published properties for this object }
            published_properties_write_rtti_data(tcb,propnamelist,def.symtable);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            write_string(current_module.realmodulename^);
            maybe_write_align;

            { write published properties for this object }
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_16bit(published_properties_count(def.symtable)));
            maybe_write_align;
=======
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(length(current_module.realmodulename^)));
            current_asmdata.asmlists[al_rtti].concat(Tai_string.Create(current_module.realmodulename^));
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));

            { write published properties for this object }
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_16bit(published_properties_count(def.symtable)));
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            published_properties_write_rtti_data(propnamelist,def.symtable);
>>>>>>> graemeg/cpstrnew

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

            { write parent typeinfo }
            write_rtti_reference(tcb,def.childof,fullrtti);

            { interface: write flags, iid and iidstr }
<<<<<<< HEAD
            IntfFlags:=0;
            if assigned(def.iidguid) then
              IntfFlags:=IntfFlags or (1 shl ord(ifHasGuid));
            if assigned(def.iidstr) then
              IntfFlags:=IntfFlags or (1 shl ord(ifHasStrGUID));
            if (def.objecttype=odt_dispinterface) then
              IntfFlags:=IntfFlags or (1 shl ord(ifDispInterface));
            if (target_info.endian=endian_big) then
              IntfFlags:=reverse_byte(IntfFlags);
              {
              ifDispatch, }
<<<<<<< HEAD
            tcb.emit_ord_const(IntfFlags,u8inttype);
            tcb.begin_anonymous_record('',defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin,
              targetinfos[target_info.system]^.alignment.maxCrecordalign);

            tcb.emit_guid_const(def.iidguid^);

            { write unit name }
            tcb.emit_shortstring_const(current_module.realmodulename^);

            tcb.begin_anonymous_record('',defaultpacking,reqalign,
              targetinfos[target_info.system]^.alignment.recordalignmin,
              targetinfos[target_info.system]^.alignment.maxCrecordalign);

            { write iidstr }
            if assigned(def.iidstr) then
              tcb.emit_shortstring_const(def.iidstr^)
            else
              tcb.emit_shortstring_const('');
=======
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(IntfFlags));
            maybe_write_align;
=======
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(
              { ugly, but working }
{$ifdef USE_PACKSET1}
              byte([
{$else USE_PACKSET1}
              longint([
{$endif USE_PACKSET1}
                TCompilerIntfFlag(ord(ifHasGuid)*ord(assigned(def.iidguid))),
                TCompilerIntfFlag(ord(ifHasStrGUID)*ord(assigned(def.iidstr))),
                TCompilerIntfFlag(ord(ifDispInterface)*ord(def.objecttype=odt_dispinterface))
              ])
              {
              ifDispatch, }
              ));
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_32bit(longint(def.iidguid^.D1)));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_16bit(def.iidguid^.D2));
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_16bit(def.iidguid^.D3));
            for i:=Low(def.iidguid^.D4) to High(def.iidguid^.D4) do
              current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(def.iidguid^.D4[i]));

            { write unit name }
<<<<<<< HEAD
            write_string(current_module.realmodulename^);
            maybe_write_align;
=======
            current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(length(current_module.realmodulename^)));
            current_asmdata.asmlists[al_rtti].concat(Tai_string.Create(current_module.realmodulename^));
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

            { write iidstr }
            if assigned(def.iidstr) then
              write_string(def.iidstr^)
            else
              current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(0));
<<<<<<< HEAD
<<<<<<< HEAD
            maybe_write_align;
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
=======
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
>>>>>>> graemeg/fixes_2_2
=======
            if (tf_requires_proper_alignment in target_info.flags) then
              current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
>>>>>>> origin/fixes_2_2

            { write published properties for this object }
            published_properties_write_rtti_data(tcb,propnamelist,def.symtable);

            tcb.end_anonymous_record;
            tcb.end_anonymous_record;

            propnamelist.free;
          end;

        begin
           case def.objecttype of
             odt_class:
               tcb.emit_ord_const(tkclass,u8inttype);
             odt_object:
<<<<<<< HEAD
               tcb.emit_ord_const(tkobject,u8inttype);
=======
               current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(tkobject));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           tcb.emit_shortstring_const(def.objrealname^);

           tcb.begin_anonymous_record('',defaultpacking,reqalign,
             targetinfos[target_info.system]^.alignment.recordalignmin,
             targetinfos[target_info.system]^.alignment.maxCrecordalign);
=======
           write_string(def.objrealname^);
           maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
           write_string(def.objrealname^);
           maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
           write_string(def.objrealname^);
           maybe_write_align;
>>>>>>> graemeg/cpstrnew
=======
           write_string(def.objrealname^);
           maybe_write_align;
>>>>>>> origin/cpstrnew
=======
           current_asmdata.asmlists[al_rtti].concat(Tai_const.Create_8bit(length(def.objrealname^)));
           current_asmdata.asmlists[al_rtti].concat(Tai_string.Create(def.objrealname^));
           if (tf_requires_proper_alignment in target_info.flags) then
             current_asmdata.asmlists[al_rtti].concat(cai_align.Create(sizeof(TConstPtrUInt)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

           case rt of
             initrtti :
               begin
<<<<<<< HEAD
                 if def.objecttype in [odt_class,odt_object,odt_helper] then
=======
                 if def.objecttype in [odt_class,odt_object] then
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
                   objectdef_rtti_fields(def)
                 else
                   objectdef_rtti_interface_init(def);
               end;
             fullrtti :
               begin
                 case def.objecttype of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   odt_helper,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                   odt_class:
                     objectdef_rtti_class_full(def);
                   odt_object:
                     objectdef_rtti_fields(def);
                 else
                   objectdef_rtti_interface_full(def);
                 end;
               end;
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

    procedure TRTTIWriter.write_rtti_extrasyms(def:Tdef;rt:Trttitype;mainrtti:Tasmsymbol);

        procedure enumdef_rtti_ord2stringindex(def:Tenumdef);

        var rttilab:Tasmsymbol;
            t:Tenumsym;
            syms:^Tenumsym;
            offsets:^longint;
            sym_count,sym_alloc:longint;
            h,i,p,o,st:longint;
            mode:(lookup,search); {Modify with care, ordinal value of enum is written.}
            r:single;             {Must be real type because of integer overflow risk.}

        begin
          {Random access needed, put in array.}
          getmem(syms,64*sizeof(Tenumsym));
          getmem(offsets,64*sizeof(longint));
          sym_count:=0;
          sym_alloc:=64;
          st:=0;
          t:=Tenumsym(def.firstenum);
          while assigned(t) do
            begin
              if sym_count>=sym_alloc then
                begin
                  reallocmem(syms,2*sym_alloc*sizeof(Tenumsym));
                  reallocmem(offsets,2*sym_alloc*sizeof(longint));
                  sym_alloc:=sym_alloc*2;
                end;
              syms[sym_count]:=t;
              offsets[sym_count]:=st;
              inc(sym_count);
              st:=st+length(t.realname)+1;
              t:=t.nextenum;
            end;
          {Sort the syms by enum value}
          if sym_count>=2 then
            begin
              p:=1;
              while 2*p<sym_count do
                p:=2*p;
              while p<>0 do
                begin
                  for h:=p to sym_count-1 do
                    begin
                      i:=h;
                      t:=syms[i];
                      o:=offsets[i];
                      repeat
                        if syms[i-p].value<=t.value then
                          break;
                        syms[i]:=syms[i-p];
                        offsets[i]:=offsets[i-p];
                        dec(i,p);
                      until i<p;
                      syms[i]:=t;
                      offsets[i]:=o;
                    end;
                  p:=p shr 1;
                end;
            end;
          {Decide wether a lookup array is size efficient.}
          mode:=lookup;
          if sym_count>0 then
            begin
              i:=1;
              r:=0;
              h:=syms[0].value; {Next expected enum value is min.}
              while i<sym_count do
                begin
                  {Calculate size of hole between values. Avoid integer overflows.}
                  r:=r+(single(syms[i].value)-single(h))-1;
                  h:=syms[i].value;
                  inc(i);
                end;
              if r>sym_count then
                mode:=search; {Don't waste more than 50% space.}
            end;
          {Calculate start of string table.}
          st:=1;
          if assigned(def.typesym) then
            inc(st,length(def.typesym.realname)+1)
          else
            inc(st);
          if (tf_requires_proper_alignment in target_info.flags) then
            align(st,sizeof(Tconstptruint));
          inc(st);
          if (tf_requires_proper_alignment in target_info.flags) then
            align(st,sizeof(Tconstptruint));
          inc(st,8+sizeof(aint));
          { write rtti data }
          with current_asmdata do
            begin
              rttilab:=defineasmsymbol(Tstoreddef(def).rtti_mangledname(rt)+'_o2s',AB_GLOBAL,AT_DATA);
              maybe_new_object_file(asmlists[al_rtti]);
              new_section(asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(aint)));
              asmlists[al_rtti].concat(Tai_symbol.create_global(rttilab,0));
              asmlists[al_rtti].concat(Tai_const.create_32bit(longint(mode)));
              if mode=lookup then
                begin
                  o:=syms[0].value;  {Start with min value.}
                  for i:=0 to sym_count-1 do
                    begin
                      while o<syms[i].value do
                        begin
                          asmlists[al_rtti].concat(Tai_const.create_aint(0));
                          inc(o);
                        end;
                      inc(o);
                      asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                    end;
                end
              else
                begin
                  asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
                  for i:=0 to sym_count-1 do
                    begin
                      asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                      asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                    end;
                end;
              asmlists[al_rtti].concat(Tai_symbol_end.create(rttilab));
            end;
          dispose(syms);
          dispose(offsets);
        end;

        procedure enumdef_rtti_string2ordindex(def:Tenumdef);

        var rttilab:Tasmsymbol;
            t:Tenumsym;
            syms:^Tenumsym;
            offsets:^longint;
            sym_count,sym_alloc:longint;
            h,i,p,o,st:longint;

        begin
          {Random access needed, put in array.}
          getmem(syms,64*sizeof(Tenumsym));
          getmem(offsets,64*sizeof(longint));
          sym_count:=0;
          sym_alloc:=64;
          st:=0;
          t:=Tenumsym(def.firstenum);
          while assigned(t) do
            begin
              if sym_count>=sym_alloc then
                begin
                  reallocmem(syms,2*sym_alloc*sizeof(Tenumsym));
                  reallocmem(offsets,2*sym_alloc*sizeof(longint));
                  sym_alloc:=sym_alloc*2;
                end;
              syms[sym_count]:=t;
              offsets[sym_count]:=st;
              inc(sym_count);
              st:=st+length(t.realname)+1;
              t:=t.nextenum;
            end;
          {Sort the syms by enum name}
          if sym_count>=2 then
            begin
              p:=1;
              while 2*p<sym_count do
                p:=2*p;
              while p<>0 do
                begin
                  for h:=p to sym_count-1 do
                    begin
                      i:=h;
                      t:=syms[i];
                      o:=offsets[i];
                      repeat
                        if syms[i-p].name<=t.name then
                          break;
                        syms[i]:=syms[i-p];
                        offsets[i]:=offsets[i-p];
                        dec(i,p);
                      until i<p;
                      syms[i]:=t;
                      offsets[i]:=o;
                    end;
                  p:=p shr 1;
                end;
            end;
          {Calculate start of string table.}
          st:=1;
          if assigned(def.typesym) then
            inc(st,length(def.typesym.realname)+1)
          else
            inc(st);
          if (tf_requires_proper_alignment in target_info.flags) then
            align(st,sizeof(Tconstptruint));
          inc(st);
          if (tf_requires_proper_alignment in target_info.flags) then
            align(st,sizeof(Tconstptruint));
          inc(st,8+sizeof(aint));
          { write rtti data }
          with current_asmdata do
            begin
              rttilab:=defineasmsymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA);
              maybe_new_object_file(asmlists[al_rtti]);
              new_section(asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(aint)));
              asmlists[al_rtti].concat(Tai_symbol.create_global(rttilab,0));
              asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
              for i:=0 to sym_count-1 do
                begin
                  asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                  asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                end;
              asmlists[al_rtti].concat(Tai_symbol_end.create(rttilab));
            end;
          dispose(syms);
          dispose(offsets);
        end;

    begin
      case def.typ of
        enumdef:
          if rt=fullrtti then
            begin
              enumdef_rtti_ord2stringindex(Tenumdef(def));
              enumdef_rtti_string2ordindex(Tenumdef(def));
            end;
      end;
    end;
<<<<<<< HEAD

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        { Writes a helper table for accelerated conversion of ordinal enum values to strings.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_ord2stringindex(rttidef: trecorddef; const syms: tfplist);

        var rttilab:Tasmsymbol;
            h,i,o,prev_value:longint;
=======
=======
>>>>>>> graemeg/cpstrnew
        function enumdef_rtti_calcstringtablestart(const def : Tenumdef) : integer;
        begin
          { the alignment calls must correspond to the ones used during generating the
            actual data structure created elsewhere in this file }
          result:=1;
          if assigned(def.typesym) then
            inc(result,length(def.typesym.realname)+1)
          else
            inc(result);
          if (tf_requires_proper_alignment in target_info.flags) then
            result:=align(result,sizeof(Tconstptruint));
          inc(result);
          if (tf_requires_proper_alignment in target_info.flags) then
            result:=align(result,sizeof(Tconstptruint));
          inc(result, sizeof(longint) * 2);
          if (tf_requires_proper_alignment in target_info.flags) then
            result:=align(result,sizeof(Tconstptruint));
          inc(result, sizeof(pint));
        end;

        { Writes a helper table for accelerated conversion of ordinal enum values to strings.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_ord2stringindex(const sym_count:longint; const offsets:plongint; const syms:Penumsym; const st:longint);

        var rttilab:Tasmsymbol;
            h,i,o:longint;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
        function enumdef_rtti_calcstringtablestart(const def : Tenumdef) : integer;
        begin
          { the alignment calls must correspond to the ones used during generating the
            actual data structure created elsewhere in this file }
          result:=1;
          if assigned(def.typesym) then
            inc(result,length(def.typesym.realname)+1)
          else
            inc(result);
          if (tf_requires_proper_alignment in target_info.flags) then
            result:=align(result,sizeof(Tconstptruint));
          inc(result);
          if (tf_requires_proper_alignment in target_info.flags) then
            result:=align(result,sizeof(Tconstptruint));
          inc(result, sizeof(longint) * 2);
          if (tf_requires_proper_alignment in target_info.flags) then
            result:=align(result,sizeof(Tconstptruint));
          inc(result, sizeof(pint));
        end;

        { Writes a helper table for accelerated conversion of ordinal enum values to strings.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_ord2stringindex(const sym_count:longint; const offsets:plongint; const syms:Penumsym; const st:longint);

        var rttilab:Tasmsymbol;
            h,i,o:longint;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            mode:(lookup,search); {Modify with care, ordinal value of enum is written.}
            r:single;             {Must be real type because of integer overflow risk.}
            tcb: ttai_typedconstbuilder;
            sym_count: integer;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          rttilab:=current_asmdata.DefineAsmSymbol(Tstoreddef(def).rtti_mangledname(rt)+'_o2s',AB_GLOBAL,AT_DATA);
          tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable]);
          { use TConstPtrUInt packrecords to ensure good alignment }
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
          { now emit the data: first the mode }
          tcb.emit_tai(Tai_const.create_32bit(longint(mode)),u32inttype);
          { align }
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
          if mode=lookup then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          with current_asmdata do
>>>>>>> graemeg/cpstrnew
            begin
              o:=tenumsym(syms[0]).value;  {Start with min value.}
              for i:=0 to sym_count-1 do
                begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  while o<tenumsym(syms[i]).value do
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  maybe_write_align;
                  o:=syms[0].value;  {Start with min value.}
                  for i:=0 to sym_count-1 do
>>>>>>> graemeg/cpstrnew
                    begin
                      tcb.emit_tai(Tai_const.create_pint(0),ptruinttype);
                      inc(o);
                    end;
                  inc(o);
                  tcb.queue_init(voidpointertype);
                  tcb.queue_subscriptn_multiple_by_name(rttidef,
                    ['size_start_rec',
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
              tcb.begin_anonymous_record('',defaultpacking,reqalign,
                targetinfos[target_info.system]^.alignment.recordalignmin,
                targetinfos[target_info.system]^.alignment.maxCrecordalign);
              for i:=0 to sym_count-1 do
                begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  tcb.emit_ord_const(tenumsym(syms[i]).value,s32inttype);
                  tcb.queue_init(voidpointertype);
                  tcb.queue_subscriptn_multiple_by_name(rttidef,
                    ['size_start_rec',
                      'min_max_rec',
                      'basetype_array_rec',
                      tsym(syms[i]).Name]
                  );
                  tcb.queue_emit_asmsym(mainrtti,rttidef);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  maybe_write_align;
                  asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
                  for i:=0 to sym_count-1 do
                    begin
                      maybe_write_align;
                      asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                      maybe_write_align;
                      asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                    end;
>>>>>>> graemeg/cpstrnew
                end;
              tcb.end_anonymous_record;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            tcb.end_anonymous_record;

            current_asmdata.asmlists[al_rtti].concatlist(tcb.get_final_asmlist(
              rttilab,tcb.end_anonymous_record,sec_rodata,
              rttilab.name,const_align(sizeof(pint))));
            tcb.free;
        end;


        { Writes a helper table for accelerated conversion of string to ordinal enum values.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_string2ordindex(rttidef: trecorddef; const syms: tfplist);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
        end;

=======
        end;

>>>>>>> origin/cpstrnew
        { Writes a helper table for accelerated conversion of string to ordinal enum values.
          If you change something in this method, make sure to adapt the corresponding code
          in sstrings.inc. }
        procedure enumdef_rtti_string2ordindex(const sym_count:longint; const offsets:plongint; const syms:Penumsym; const st:longint);

        var rttilab:Tasmsymbol;
            i:longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

        var
          tcb: ttai_typedconstbuilder;
          rttilab:Tasmsymbol;
          i:longint;
        begin
          { write rtti data }
<<<<<<< HEAD
          rttilab:=current_asmdata.DefineAsmSymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA);
          tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable]);
          { begin of Tstring_to_ord }
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
          tcb.emit_ord_const(syms.count,s32inttype);
          { begin of "data" array in Tstring_to_ord }
          tcb.begin_anonymous_record('',defaultpacking,reqalign,
            targetinfos[target_info.system]^.alignment.recordalignmin,
            targetinfos[target_info.system]^.alignment.maxCrecordalign);
          for i:=0 to syms.count-1 do
            begin
              tcb.emit_ord_const(tenumsym(syms[i]).value,s32inttype);
              { alignment of pointer value handled by enclosing record already }
              tcb.queue_init(voidpointertype);
              tcb.queue_subscriptn_multiple_by_name(rttidef,
                ['size_start_rec',
                  'min_max_rec',
                  'basetype_array_rec',
                  tsym(syms[i]).Name]
              );
              tcb.queue_emit_asmsym(mainrtti,rttidef);
            end;
          tcb.end_anonymous_record;
          current_asmdata.asmlists[al_rtti].concatlist(tcb.get_final_asmlist(
            rttilab,tcb.end_anonymous_record,sec_rodata,
            rttilab.name,const_align(sizeof(pint))));
          tcb.free;
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
=======
          with current_asmdata do
            begin
              rttilab:=defineasmsymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA);
              maybe_new_object_file(asmlists[al_rtti]);
              new_section(asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(pint)));
              asmlists[al_rtti].concat(Tai_symbol.create_global(rttilab,0));
              asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
              { need to align the entry record according to the largest member }
              maybe_write_align;
              for i:=0 to sym_count-1 do
                begin
                  if (tf_requires_proper_alignment in target_info.flags) then
                    current_asmdata.asmlists[al_rtti].concat(cai_align.Create(4));  // necessary?
                  asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                  maybe_write_align;
                  asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                end;
              asmlists[al_rtti].concat(Tai_symbol_end.create(rttilab));
            end;
        end;
=======
>>>>>>> graemeg/cpstrnew

        procedure enumdef_rtti_extrasyms(def:Tenumdef);
        var
          t:Tenumsym;
          syms:Penumsym;
          sym_count,sym_alloc:sizeuint;
          offsets:^longint;
          h,i,p,o,st:longint;
        begin
          { write rtti data }
          with current_asmdata do
            begin
              rttilab:=defineasmsymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA);
              maybe_new_object_file(asmlists[al_rtti]);
              new_section(asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(pint)));
              asmlists[al_rtti].concat(Tai_symbol.create_global(rttilab,0));
              asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
              { need to align the entry record according to the largest member }
              maybe_write_align;
              for i:=0 to sym_count-1 do
                begin
                  if (tf_requires_proper_alignment in target_info.flags) then
                    current_asmdata.asmlists[al_rtti].concat(cai_align.Create(4));  // necessary?
                  asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                  maybe_write_align;
                  asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                end;
              asmlists[al_rtti].concat(Tai_symbol_end.create(rttilab));
            end;
        end;
=======
>>>>>>> origin/cpstrnew

        procedure enumdef_rtti_extrasyms(def:Tenumdef);
        var
          t:Tenumsym;
          syms:Penumsym;
          sym_count,sym_alloc:sizeuint;
          offsets:^longint;
          h,i,p,o,st:longint;
        begin
          { write rtti data }
          with current_asmdata do
            begin
              rttilab:=defineasmsymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA);
              maybe_new_object_file(asmlists[al_rtti]);
              new_section(asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(pint)));
              asmlists[al_rtti].concat(Tai_symbol.create_global(rttilab,0));
              asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
              { need to align the entry record according to the largest member }
              maybe_write_align;
              for i:=0 to sym_count-1 do
                begin
                  if (tf_requires_proper_alignment in target_info.flags) then
                    current_asmdata.asmlists[al_rtti].concat(cai_align.Create(4));  // necessary?
                  asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                  maybe_write_align;
                  asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                end;
              asmlists[al_rtti].concat(Tai_symbol_end.create(rttilab));
            end;
        end;

        procedure enumdef_rtti_extrasyms(def:Tenumdef);
        var
          t:Tenumsym;
          syms:Penumsym;
          sym_count,sym_alloc:sizeuint;
          offsets:^longint;
          h,i,p,o,st:longint;
        begin
          { write rtti data }
          with current_asmdata do
            begin
              rttilab:=defineasmsymbol(Tstoreddef(def).rtti_mangledname(rt)+'_s2o',AB_GLOBAL,AT_DATA);
              maybe_new_object_file(asmlists[al_rtti]);
              new_section(asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(pint)));
              asmlists[al_rtti].concat(Tai_symbol.create_global(rttilab,0));
              asmlists[al_rtti].concat(Tai_const.create_32bit(sym_count));
              { need to align the entry record according to the largest member }
              maybe_write_align;
              for i:=0 to sym_count-1 do
                begin
                  if (tf_requires_proper_alignment in target_info.flags) then
                    current_asmdata.asmlists[al_rtti].concat(cai_align.Create(4));  // necessary?
                  asmlists[al_rtti].concat(Tai_const.create_32bit(syms[i].value));
                  maybe_write_align;
                  asmlists[al_rtti].concat(Tai_const.create_sym_offset(mainrtti,st+offsets[i]));
                end;
              asmlists[al_rtti].concat(Tai_symbol_end.create(rttilab));
            end;
        end;

        procedure enumdef_rtti_extrasyms(def:Tenumdef);
        var
          t:Tenumsym;
          syms:Penumsym;
          sym_count,sym_alloc:sizeuint;
          offsets:^longint;
          h,i,p,o,st:longint;
        begin
          {Random access needed, put in array.}
          getmem(syms,64*sizeof(Tenumsym));
          getmem(offsets,64*sizeof(longint));
          sym_count:=0;
          sym_alloc:=64;
          st:=0;
          for i := 0 to def.symtable.SymList.Count - 1 do
            begin
              t:=tenumsym(def.symtable.SymList[i]);
              if t.value<def.minval then
                continue
              else
              if t.value>def.maxval then
                break;
              if sym_count>=sym_alloc then
                begin
                  reallocmem(syms,2*sym_alloc*sizeof(Tenumsym));
                  reallocmem(offsets,2*sym_alloc*sizeof(longint));
                  sym_alloc:=sym_alloc*2;
                end;
              syms[sym_count]:=t;
              offsets[sym_count]:=st;
              inc(sym_count);
              st:=st+length(t.realname)+1;
            end;
          {Sort the syms by enum name}
          if sym_count>=2 then
            begin
              p:=1;
              while 2*p<sym_count do
                p:=2*p;
              while p<>0 do
                begin
                  for h:=p to sym_count-1 do
                    begin
                      i:=h;
                      t:=syms[i];
                      o:=offsets[i];
                      repeat
                        if syms[i-p].name<=t.name then
                          break;
                        syms[i]:=syms[i-p];
                        offsets[i]:=offsets[i-p];
                        dec(i,p);
                      until i<p;
                      syms[i]:=t;
                      offsets[i]:=o;
                    end;
                  p:=p shr 1;
                end;
            end;
          st:=enumdef_rtti_calcstringtablestart(def);
          enumdef_rtti_string2ordindex(sym_count,offsets,syms,st);
          { Sort the syms by enum value }
          if sym_count>=2 then
            begin
              p:=1;
              while 2*p<sym_count do
                p:=2*p;
              while p<>0 do
                begin
                  for h:=p to sym_count-1 do
                    begin
                      i:=h;
                      t:=syms[i];
                      o:=offsets[i];
                      repeat
                        if syms[i-p].value<=t.value then
                          break;
                        syms[i]:=syms[i-p];
                        offsets[i]:=offsets[i-p];
                        dec(i,p);
                      until i<p;
                      syms[i]:=t;
                      offsets[i]:=o;
                    end;
                  p:=p shr 1;
                end;
            end;
          enumdef_rtti_ord2stringindex(sym_count,offsets,syms,st);
          freemem(syms);
          freemem(offsets);
>>>>>>> graemeg/cpstrnew
        end;


    begin
      case def.typ of
        enumdef:
          if rt=fullrtti then
            begin
              enumdef_rtti_extrasyms(Tenumdef(def));
            end;
      end;
    end;
=======
>>>>>>> origin/fixes_2_2

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
            fields_write_rtti(trecorddef(def).symtable,rt);
          objectdef :
            begin
              if assigned(tobjectdef(def).childof) then
                write_rtti(tobjectdef(def).childof,rt);
              if (rt=initrtti) or (tobjectdef(def).objecttype=odt_object) then
                fields_write_rtti(tobjectdef(def).symtable,rt)
              else
                published_write_rtti(tobjectdef(def).symtable,rt);
            end;
          classrefdef,
          pointerdef:
            if not is_objc_class_or_protocol(tabstractpointerdef(def).pointeddef) then
              write_rtti(tabstractpointerdef(def).pointeddef,rt);
          procvardef:
            params_write_rtti(tabstractprocdef(def),rt);
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
          tcb.emit_tai(Tai_const.Create_sym(ref_rtti(def,rt)),voidpointertype);
      end;


    function TRTTIWriter.ref_rtti(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt),AT_DATA);
        if (cs_create_pic in current_settings.moduleswitches) and
           assigned(current_procinfo) then
          include(current_procinfo.flags,pi_needs_got);
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
        tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable]);
        rttilab:=current_asmdata.DefineAsmSymbol(tstoreddef(def).rtti_mangledname(rt),AB_GLOBAL,AT_DATA);
<<<<<<< HEAD
        tcb.begin_anonymous_record(
          internaltypeprefixName[itp_rttidef]+rttilab.Name,
          defaultpacking,reqalign,
          targetinfos[target_info.system]^.alignment.recordalignmin,
          targetinfos[target_info.system]^.alignment.maxCrecordalign
        );
        write_rtti_data(tcb,def,rt);
        rttidef:=tcb.end_anonymous_record;
        current_asmdata.AsmLists[al_rtti].concatList(
          tcb.get_final_asmlist(rttilab,rttidef,sec_rodata,rttilab.name,const_align(sizeof(pint))));
        write_rtti_extrasyms(def,rt,rttilab);
        tcb.free;
      end;


    constructor TRTTIWriter.create;
      begin
        if tf_requires_proper_alignment in target_info.flags then
          begin
            reqalign:=sizeof(TConstPtrUInt);
            defaultpacking:=C_alignment;
          end
        else
          begin
            reqalign:=1;
            defaultpacking:=1;
          end;
=======
        maybe_new_object_file(current_asmdata.asmlists[al_rtti]);
        new_section(current_asmdata.asmlists[al_rtti],sec_rodata,rttilab.name,const_align(sizeof(aint)));
        current_asmdata.asmlists[al_rtti].concat(Tai_symbol.Create_global(rttilab,0));
        write_rtti_data(def,rt);
        current_asmdata.asmlists[al_rtti].concat(Tai_symbol_end.Create(rttilab));
        write_rtti_extrasyms(def,rt,rttilab);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    function TRTTIWriter.get_rtti_label(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt),AT_DATA);
        if (cs_create_pic in current_settings.moduleswitches) and
           assigned(current_procinfo) then
          include(current_procinfo.flags,pi_needs_got);
      end;

    function TRTTIWriter.get_rtti_label_ord2str(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt)+'_o2s',AT_DATA);
        if (cs_create_pic in current_settings.moduleswitches) and
           assigned(current_procinfo) then
          include(current_procinfo.flags,pi_needs_got);
      end;

    function TRTTIWriter.get_rtti_label_str2ord(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt)+'_s2o',AT_DATA);
        if (cs_create_pic in current_settings.moduleswitches) and
           assigned(current_procinfo) then
          include(current_procinfo.flags,pi_needs_got);
      end;

    function TRTTIWriter.get_rtti_label_ord2str(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt)+'_o2s');
      end;

    function TRTTIWriter.get_rtti_label_str2ord(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt)+'_s2o');
      end;

    function TRTTIWriter.get_rtti_label_ord2str(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt)+'_o2s');
      end;

    function TRTTIWriter.get_rtti_label_str2ord(def:tdef;rt:trttitype):tasmsymbol;
      begin
        result:=current_asmdata.RefAsmSymbol(def.rtti_mangledname(rt)+'_s2o');
      end;

end.

