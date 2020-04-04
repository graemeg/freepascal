{
    Copyright (c) 2014 by Jonas Maebe

    Generates code for typed constant declarations for the LLVM target

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
unit nllvmtcon;

{$i fpcdefs.inc}

interface

  uses
    cclasses,constexp,globtype,
    aasmbase,aasmtai,aasmcnst,aasmllvm,
    symconst,symbase,symtype,symdef,symsym,
    ngtcon;

  type
    tllvmaggregateinformation = class(taggregateinformation)
     private
      faggai: tai_aggregatetypedconst;
      fanonrecalignpos: longint;
      { if this is a non-anonymous record, keep track of the current field at
        the llvm level that gets emitted, so we know when the data types of the
        Pascal and llvm representation don't match up (because of variant
        records, or because not all fields are defined at the Pascal level and
        the rest is zeroed) }
      fllvmnextfieldindex: longint;
      fdoesnotmatchllvmdef: boolean;
     public
      constructor create(_def: tdef; _typ: ttypedconstkind); override;

      function prepare_next_field(nextfielddef: tdef): asizeint; override;

      property aggai: tai_aggregatetypedconst read faggai write faggai;
      property anonrecalignpos: longint read fanonrecalignpos write fanonrecalignpos;
      property llvmnextfieldindex: longint read fllvmnextfieldindex write fllvmnextfieldindex;
      property doesnotmatchllvmdef: boolean read fdoesnotmatchllvmdef write fdoesnotmatchllvmdef;
    end;

    tllvmtypedconstplaceholder = class(ttypedconstplaceholder)
      agginfo: tllvmaggregateinformation;
      pos: longint;
      constructor create(info: tllvmaggregateinformation; p: longint; d: tdef);
      procedure replace(ai: tai; d: tdef); override;
    end;

    tllvmtai_typedconstbuilder = class(ttai_typedconstbuilder)
     public
       { set the default value for caggregateinformation (= tllvmaggregateinformation) }
       class constructor classcreate;
     protected
      foverriding_def: tdef;
      fappendingdef: boolean;

      fqueued_tai,
      flast_added_tai: tai;
      fqueued_tai_opidx: longint;

      procedure finalize_asmlist(sym: tasmsymbol; def: tdef; section: TAsmSectiontype; const secname: TSymStr; alignment: shortint; const options: ttcasmlistoptions); override;
      { outerai: the ai that should become fqueued_tai in case it's still nil,
          or that should be filled in the fqueued_tai_opidx of the current
          fqueued_tai if it's not nil
        innerai: the innermost ai (possibly an operand of outerai) in which
          newindex indicates which operand is empty and can be filled with the
          next queued tai }
      procedure update_queued_tai(resdef: tdef; outerai, innerai: tai; newindex: longint);
      function wrap_with_type(p: tai; def: tdef): tai;
      procedure do_emit_tai(p: tai; def: tdef); override;
      procedure mark_anon_aggregate_alignment; override;
      procedure insert_marked_aggregate_alignment(def: tdef); override;
      procedure maybe_emit_tail_padding(def: tdef); override;
      procedure begin_aggregate_internal(def: tdef; anonymous: boolean); override;
      procedure end_aggregate_internal(def: tdef; anonymous: boolean); override;

      function get_internal_data_section_start_label: tasmlabel; override;
      function get_internal_data_section_internal_label: tasmlabel; override;

      procedure do_emit_extended_in_aggregate(p: tai);

      { mark the current agginfo, and hence also all the ones higher up in ther
        aggregate hierarchy, as not matching our canonical llvm definition for
        their def }
      procedure mark_aggregate_hierarchy_llvmdef_mismatch(new_current_level_def: trecorddef);
     public
      destructor destroy; override;
      procedure emit_tai(p: tai; def: tdef); override;
      procedure emit_tai_procvar2procdef(p: tai; pvdef: tprocvardef); override;
      procedure emit_string_offset(const ll: tasmlabofs; const strlength: longint; const st: tstringtype; const winlikewidestring: boolean; const charptrdef: tdef); override;
      procedure emit_dynarray_offset(const ll: tasmlabofs; const arrlength: asizeint; const arrdef: tarraydef; const arrconstdatadef: trecorddef); override;
      procedure queue_init(todef: tdef); override;
      procedure queue_vecn(def: tdef; const index: tconstexprint); override;
      procedure queue_subscriptn(def: tabstractrecorddef; vs: tfieldvarsym); override;
      procedure queue_typeconvn(fromdef, todef: tdef); override;
      procedure queue_emit_staticvar(vs: tstaticvarsym); override;
      procedure queue_emit_label(l: tlabelsym); override;
      procedure queue_emit_asmsym(sym: tasmsymbol; def: tdef); override;
      procedure queue_emit_ordconst(value: int64; def: tdef); override;

      class function get_vectorized_dead_strip_custom_section_name(const basename: TSymStr; st: tsymtable; out secname: TSymStr): boolean; override;

      function emit_placeholder(def: tdef): ttypedconstplaceholder; override;

      class function get_string_symofs(typ: tstringtype; winlikewidestring: boolean): pint; override;
      class function get_dynarray_symofs: pint; override;

      property appendingdef: boolean write fappendingdef;
    end;


implementation

  uses
    verbose,systems,fmodule,
    aasmdata,
    procinfo,
    cpubase,cpuinfo,llvmbase,
    symtable,llvmdef,defutil,defcmp,
    ngenutil;

  { tllvmaggregateinformation }

   constructor tllvmaggregateinformation.create(_def: tdef; _typ: ttypedconstkind);
     begin
       inherited;
       fanonrecalignpos:=-1;
       fllvmnextfieldindex:=0;
     end;


   function tllvmaggregateinformation.prepare_next_field(nextfielddef: tdef): asizeint;
     begin
       result:=inherited;
       { in case we let LLVM align, don't add padding ourselves }
       if df_llvm_no_struct_packing in def.defoptions then
         result:=0;
     end;


   { tllvmtypedconstplaceholder }

  constructor tllvmtypedconstplaceholder.create(info: tllvmaggregateinformation; p: longint; d: tdef);
    begin
      inherited create(d);
      agginfo:=info;
      pos:=p;
    end;


  procedure tllvmtypedconstplaceholder.replace(ai: tai; d: tdef);
    begin
      if d<>def then
        internalerror(2015091002);
      agginfo.aggai.replacevalueatpos(
        tai_simpletypedconst.create(d,ai),pos
      );
    end;


  { tllvmtai_typedconstbuilder }

  class constructor tllvmtai_typedconstbuilder.classcreate;
    begin
      caggregateinformation:=tllvmaggregateinformation;
    end;


  procedure tllvmtai_typedconstbuilder.finalize_asmlist(sym: tasmsymbol; def: tdef; section: TAsmSectiontype; const secname: TSymStr; alignment: shortint; const options: ttcasmlistoptions);
    var
      newasmlist: tasmlist;
      decl: taillvmdecl;
    begin
      finalize_asmlist_prepare(options,alignment);
      newasmlist:=tasmlist.create;
      if assigned(foverriding_def) then
        def:=foverriding_def;
      { llvm declaration with as initialisation data all the elements from the
        original asmlist }
      decl:=taillvmdecl.createdef(sym,def,fasmlist,section,alignment);
      if fappendingdef then
        include(decl.flags,ldf_appending);
      if section=sec_user then
        decl.setsecname(secname);
      if tcalo_is_lab in options then
        include(decl.flags,ldf_unnamed_addr);
      if ([tcalo_vectorized_dead_strip_start,
           tcalo_vectorized_dead_strip_item,
           tcalo_vectorized_dead_strip_end]*options)<>[] then
        include(decl.flags,ldf_vectorized);
      if tcalo_weak in options then
        include(decl.flags,ldf_weak);
      if tcalo_no_dead_strip in options then
        { Objective-C section declarations already contain "no_dead_strip"
          attributes if none of their symbols need to be stripped -> only
          add the symbols to llvm.compiler.used (only affects compiler
          optimisations) and not to llvm.used (also affects linker -- which in
          this case is already taken care of by the section attribute; not sure
          why it's done like this, but this is how Clang does it) }
        if (target_info.system in systems_darwin) and
           (section in [low(TObjCAsmSectionType)..high(TObjCAsmSectionType)]) then
          cnodeutils.RegisterUsedAsmSym(sym,def,false)
        else
          cnodeutils.RegisterUsedAsmSym(sym,def,true);
      newasmlist.concat(decl);
      fasmlist:=newasmlist;
    end;


  procedure tllvmtai_typedconstbuilder.update_queued_tai(resdef: tdef; outerai, innerai: tai; newindex: longint);
    begin
      { the outer tai must always be a typed constant (possibly a wrapper
        around a taillvm or so), in order for result type information to be
        available }
      if outerai.typ<>ait_typedconst then
        internalerror(2014060401);
      { is the result of the outermost expression different from the type of
        this typed const? -> insert type conversion }
      if not assigned(fqueued_tai) and
         (resdef<>fqueued_def) and
         (llvmencodetypename(resdef)<>llvmencodetypename(fqueued_def)) then
        queue_typeconvn(resdef,fqueued_def);
      if assigned(fqueued_tai) then
        begin
          taillvm(flast_added_tai).loadtai(fqueued_tai_opidx,outerai);
          { already flushed? }
          if fqueued_tai_opidx=-1 then
            internalerror(2014062201);
        end
      else
        begin
          fqueued_tai:=outerai;
          fqueued_def:=resdef;
        end;
      fqueued_tai_opidx:=newindex;
      flast_added_tai:=innerai;
    end;


  function tllvmtai_typedconstbuilder.wrap_with_type(p: tai; def: tdef): tai;
    begin
      result:=tai_simpletypedconst.create(def,p);
    end;


  destructor tllvmtai_typedconstbuilder.destroy;
    begin
      inherited destroy;
    end;


  procedure tllvmtai_typedconstbuilder.emit_tai(p: tai; def: tdef);
    var
      arrdef: tdef;
    begin
      { inside an aggregate, an 80 bit floating point number must be
        emitted as an array of 10 bytes to prevent ABI alignment and
        padding to 16 bytes }
      if (def.typ=floatdef) and
         (tfloatdef(def).floattype=s80real) and
         assigned(curagginfo) then
        do_emit_extended_in_aggregate(p)
      else
        inherited;
    end;


  procedure tllvmtai_typedconstbuilder.do_emit_tai(p: tai; def: tdef);
    var
      ai: tai;
      stc: tai_abstracttypedconst;
      kind: ttypedconstkind;
      info: tllvmaggregateinformation;
    begin
      if queue_is_active then
        begin
          kind:=tck_simple;
          { finalise the queued expression }
          ai:=tai_simpletypedconst.create(def,p);
          { set the new index to -1, so we internalerror should we try to
            add anything further }
          update_queued_tai(def,ai,ai,-1);
          { and emit it }
          stc:=tai_abstracttypedconst(fqueued_tai);
          def:=fqueued_def;
          { ensure we don't try to emit this one again }
          fqueued_tai:=nil;
        end
      else
        stc:=tai_simpletypedconst.create(def,p);
      info:=tllvmaggregateinformation(curagginfo);
      { these elements can be aggregates themselves, e.g. a shortstring can
        be emitted as a series of bytes and string data arrays }
      kind:=aggregate_kind(def);
      if (kind<>tck_simple) then
        begin
          if not assigned(info) or
             (info.aggai.adetyp<>kind) then
           internalerror(2014052906);
        end;
      if assigned(info) then
        begin
          { are we emitting data that does not match the equivalent data in
            the llvm structure? If so, record this so that we know we have to
            use a custom recorddef to emit this data }
          if not(info.anonrecord) and
             (info.def.typ<>procvardef) and
             (aggregate_kind(info.def)=tck_record) and
             not info.doesnotmatchllvmdef then
            begin
              if (info.llvmnextfieldindex>=tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).llvmst.symdeflist.count) or
                 not equal_defs(def,tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).llvmst.entries_by_llvm_index[info.llvmnextfieldindex].def) then
                info.doesnotmatchllvmdef:=true
              else
                info.llvmnextfieldindex:=info.llvmnextfieldindex+1;
            end;
          info.aggai.addvalue(stc);
        end
      else
        inherited do_emit_tai(stc,def);
    end;


  procedure tllvmtai_typedconstbuilder.mark_anon_aggregate_alignment;
    var
      info: tllvmaggregateinformation;
    begin
      info:=tllvmaggregateinformation(curagginfo);
      info.anonrecalignpos:=info.aggai.valuecount;
    end;


  procedure tllvmtai_typedconstbuilder.insert_marked_aggregate_alignment(def: tdef);
    var
      info: tllvmaggregateinformation;
      fillbytes: asizeint;
    begin
      info:=tllvmaggregateinformation(curagginfo);
      if info.anonrecalignpos=-1 then
        internalerror(2014091501);
      fillbytes:=info.prepare_next_field(def);
      while fillbytes>0 do
        begin
          info.aggai.insertvaluebeforepos(tai_simpletypedconst.create(u8inttype,tai_const.create_8bit(0)),info.anonrecalignpos);
          dec(fillbytes);
        end;
    end;

  procedure tllvmtai_typedconstbuilder.maybe_emit_tail_padding(def: tdef);
    var
      info: tllvmaggregateinformation;
      constdata: tai_abstracttypedconst;
      newdef: trecorddef;
    begin
      { in case we let LLVM align, don't add padding ourselves }
      if df_llvm_no_struct_packing in def.defoptions then
        exit;
      inherited;
      { we can only check here whether the aggregate does not match our
        cononical llvm definition, as the tail padding may cause a mismatch
        (in case not all fields have been defined), and we can't do it inside
        end_aggregate_internal as its inherited method (which calls this
        method) frees curagginfo before it returns }
      info:=tllvmaggregateinformation(curagginfo);
      if info.doesnotmatchllvmdef then
        begin
          { create a new recorddef representing this mismatched def; this can
            even replace an array in case it contains e.g. variant records }
          case info.def.typ of
            arraydef:
              { in an array, all elements come right after each other ->
                replace with a packed record }
              newdef:=crecorddef.create_global_internal('',1,1);
            recorddef,
            objectdef:
              begin
                newdef:=crecorddef.create_global_internal('',
                  tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).usefieldalignment,
                  tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).recordalignmin);
                tabstractrecordsymtable(newdef.symtable).recordalignment:=tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).recordalignment;
              end
            else
              internalerror(2015122401);
          end;
          for constdata in tai_aggregatetypedconst(info.aggai) do
            newdef.add_field_by_def('',constdata.def);
          tai_aggregatetypedconst(info.aggai).changetorecord(newdef);
          mark_aggregate_hierarchy_llvmdef_mismatch(newdef);
        end;
    end;


  procedure tllvmtai_typedconstbuilder.emit_tai_procvar2procdef(p: tai; pvdef: tprocvardef);
    begin
      if not pvdef.is_addressonly then
        pvdef:=cprocvardef.getreusableprocaddr(pvdef,pc_address_only);
      emit_tai(p,pvdef);
    end;


  procedure tllvmtai_typedconstbuilder.emit_string_offset(const ll: tasmlabofs; const strlength: longint; const st: tstringtype; const winlikewidestring: boolean; const charptrdef: tdef);
    var
      strrecdef : trecorddef;
      strdef: tdef;
      offset: pint;
      field: tfieldvarsym;
      dataptrdef: tdef;
      typesym: ttypesym;
    begin
      { nil pointer? }
      if not assigned(ll.lab) then
        begin
          if ll.ofs<>0 then
            internalerror(2015030701);
          inherited;
          exit;
        end;
      { if the returned offset is <> 0, then the string data
        starts at that offset -> translate to a field for the
        high level code generator }
      if ll.ofs<>0 then
        begin
          { get the recorddef for this string constant }
          typesym:=try_search_current_module_type(ctai_typedconstbuilder.get_dynstring_rec_name(st,winlikewidestring,strlength));
          if not assigned(typesym) then
            internalerror(2014080406);
          strrecdef:=trecorddef(typesym.typedef);
          { offset in the record of the the string data }
          offset:=ctai_typedconstbuilder.get_string_symofs(st,winlikewidestring);
          { field corresponding to this offset }
          field:=trecordsymtable(strrecdef.symtable).findfieldbyoffset(offset);
          { pointerdef to the string data array }
          dataptrdef:=cpointerdef.getreusable(field.vardef);
          { the fields of the resourcestring record are declared as ansistring }
          strdef:=get_dynstring_def_for_type(st,winlikewidestring);
          queue_init(strdef);
          queue_typeconvn(charptrdef,strdef);
          queue_subscriptn(strrecdef,field);
          queue_emit_asmsym(ll.lab,strrecdef);
        end
      else
       { since llvm doesn't support labels in the middle of structs, this
         offset should never be 0  }
       internalerror(2014080506);
    end;


  procedure tllvmtai_typedconstbuilder.emit_dynarray_offset(const ll: tasmlabofs; const arrlength: asizeint; const arrdef: tarraydef; const arrconstdatadef: trecorddef);
    var
      field: tfieldvarsym;
      dataptrdef: tdef;
    begin
      { nil pointer? }
      if not assigned(ll.lab) then
        begin
          if ll.ofs<>0 then
            internalerror(2015030701);
          inherited;
          exit;
        end;
      { if the returned offset is <> 0, then the string data
        starts at that offset -> translate to a field for the
        high level code generator }
      if ll.ofs<>0 then
        begin
          { field corresponding to this offset }
          field:=trecordsymtable(arrconstdatadef.symtable).findfieldbyoffset(ll.ofs);
          { pointerdef to the string data array }
          dataptrdef:=cpointerdef.getreusable(field.vardef);
          { the fields of the resourcestring record are declared as ansistring }
          queue_init(dataptrdef);
          queue_subscriptn(arrconstdatadef,field);
          queue_emit_asmsym(ll.lab,arrconstdatadef);
        end
      else
       { since llvm doesn't support labels in the middle of structs, this
         offset should never be 0  }
       internalerror(2018112401);
    end;


  procedure tllvmtai_typedconstbuilder.begin_aggregate_internal(def: tdef; anonymous: boolean);
    var
      agg: tai_aggregatetypedconst;
      tck: ttypedconstkind;
      curagg: tllvmaggregateinformation;
    begin
      tck:=aggregate_kind(def);
      if tck<>tck_simple then
        begin
          { create new typed const aggregate }
          agg:=tai_aggregatetypedconst.create(tck,def);
          { either add to the current typed const aggregate (if nested), or
            emit to the asmlist (if top level) }
          curagg:=tllvmaggregateinformation(curagginfo);
          { create aggregate information for this new aggregate }
          inherited;
          { only add the new aggregate to the previous aggregate now, because
            the inherited call may have had to add padding bytes first }
          if assigned(curagg) then
            curagg.aggai.addvalue(agg)
          else
            fasmlist.concat(agg);
          { set new current typed const aggregate }
          tllvmaggregateinformation(curagginfo).aggai:=agg
        end
      else
       inherited;
    end;


  procedure tllvmtai_typedconstbuilder.end_aggregate_internal(def: tdef; anonymous: boolean);
    var
      info: tllvmaggregateinformation;
      was_aggregate: boolean;
    begin
      was_aggregate:=false;
      if aggregate_kind(def)<>tck_simple then
        begin
          was_aggregate:=true;
          info:=tllvmaggregateinformation(curagginfo);
          if not assigned(info) then
            internalerror(2014060101);
          info.aggai.finish;
        end;
      inherited;
      info:=tllvmaggregateinformation(curagginfo);
      if assigned(info) and
         was_aggregate and
         not info.doesnotmatchllvmdef then
        begin
          { are we emitting data that does not match the equivalent data in
            the llvm structure? If so, record this so that we know we have to
            use a custom recorddef to emit this data }
          if not info.anonrecord and
             (aggregate_kind(info.def)=tck_record) and
             ((info.llvmnextfieldindex>=tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).llvmst.symdeflist.count) or
              not equal_defs(def,tabstractrecordsymtable(tabstractrecorddef(info.def).symtable).llvmst.entries_by_llvm_index[info.llvmnextfieldindex].def)) then
            info.doesnotmatchllvmdef:=true
          else
            info.llvmnextfieldindex:=info.llvmnextfieldindex+1;
        end;
    end;


  function tllvmtai_typedconstbuilder.get_internal_data_section_start_label: tasmlabel;
    begin
      { let llvm take care of everything by creating internal nameless
        constants }
      current_asmdata.getlocaldatalabel(result);
    end;


  function tllvmtai_typedconstbuilder.get_internal_data_section_internal_label: tasmlabel;
    begin
      current_asmdata.getlocaldatalabel(result);
    end;


  procedure tllvmtai_typedconstbuilder.do_emit_extended_in_aggregate(p: tai);
    type
      p80realval =^t80realval;
      t80realval = packed record
        case byte of
          0: (v: ts80real);
          1: (a: array[0..9] of byte);
      end;

    var
      arrdef: tdef;
      i: longint;
      realval: p80realval;
    begin
      { emit as an array of 10 bytes }
      arrdef:=carraydef.getreusable(u8inttype,10);
      maybe_begin_aggregate(arrdef);
      if (p.typ<>ait_realconst) then
        internalerror(2015062401);
      realval:=p80realval(@tai_realconst(p).value.s80val);
      if target_info.endian=source_info.endian then
        for i:=0 to 9 do
          emit_tai(tai_const.Create_8bit(realval^.a[i]),u8inttype)
      else
        for i:=9 downto 0 do
          emit_tai(tai_const.Create_8bit(realval^.a[i]),u8inttype);
      maybe_end_aggregate(arrdef);
      { free the original constant, since we didn't emit it }
      p.free;
    end;


  procedure tllvmtai_typedconstbuilder.mark_aggregate_hierarchy_llvmdef_mismatch(new_current_level_def: trecorddef);
    var
      aggregate_level,
      i: longint;
      info: tllvmaggregateinformation;
    begin
      if assigned(faggregateinformation) then
        begin
          aggregate_level:=faggregateinformation.count;
          { the top element, at aggregate_level-1, is already marked, since
            that's why we are marking the rest }
          for i:=aggregate_level-2 downto 0 do
            begin
              info:=tllvmaggregateinformation(faggregateinformation[i]);
              if info.doesnotmatchllvmdef then
                break;
              info.doesnotmatchllvmdef:=true;
            end;
          if aggregate_level=1 then
            foverriding_def:=new_current_level_def;
        end;
    end;


  procedure tllvmtai_typedconstbuilder.queue_init(todef: tdef);
    begin
      inherited;
      fqueued_tai:=nil;
      flast_added_tai:=nil;
      fqueued_tai_opidx:=-1;
    end;


  procedure tllvmtai_typedconstbuilder.queue_vecn(def: tdef; const index: tconstexprint);
    var
      ai: taillvm;
      aityped: tai;
      eledef: tdef;
      vecindex: asizeint;
    begin
      { update range checking info }
      inherited;
      vecindex:=index.svalue;
      if def.typ=arraydef then
        dec(vecindex,tarraydef(def).lowrange);
      ai:=taillvm.getelementptr_reg_tai_size_const(NR_NO,nil,ptrsinttype,vecindex,true);
      case def.typ of
        arraydef:
          eledef:=tarraydef(def).elementdef;
        stringdef:
          case tstringdef(def).stringtype of
            st_shortstring,
            st_longstring,
            st_ansistring:
              eledef:=cansichartype;
            st_widestring,
            st_unicodestring:
              eledef:=cwidechartype;
          end;
        else
          internalerror(2014062203);
      end;
      aityped:=wrap_with_type(ai,cpointerdef.getreusable(eledef));
      update_queued_tai(cpointerdef.getreusable(eledef),aityped,ai,1);
    end;


  procedure tllvmtai_typedconstbuilder.queue_subscriptn(def: tabstractrecorddef; vs: tfieldvarsym);
    var
      getllvmfieldaddr,
      getpascalfieldaddr,
      getllvmfieldaddrtyped: tai;
      llvmfielddef: tdef;
    begin
      { update range checking info }
      inherited;
      llvmfielddef:=tabstractrecordsymtable(def.symtable).llvmst[vs].def;
      { get the address of the llvm-struct field that corresponds to this
        Pascal field }
      getllvmfieldaddr:=taillvm.getelementptr_reg_tai_size_const(NR_NO,nil,s32inttype,vs.llvmfieldnr,true);
      { getelementptr doesn't contain its own resultdef, so encode it via a
        tai_simpletypedconst tai }
      getllvmfieldaddrtyped:=wrap_with_type(getllvmfieldaddr,cpointerdef.getreusable(llvmfielddef));
      { if it doesn't match the requested field exactly (variant record),
        fixup the result }
      getpascalfieldaddr:=getllvmfieldaddrtyped;
      if (vs.offsetfromllvmfield<>0) or
         (llvmfielddef<>vs.vardef) then
        begin
          { offset of real field relative to llvm-struct field <> 0? }
          if vs.offsetfromllvmfield<>0 then
            begin
              { convert to a pointer to a 1-sized element }
              if llvmfielddef.size<>1 then
                begin
                  getpascalfieldaddr:=taillvm.op_reg_tai_size(la_bitcast,NR_NO,getpascalfieldaddr,u8inttype);
                  { update the current fielddef of the expression }
                  llvmfielddef:=u8inttype;
                end;
              { add the offset }
              getpascalfieldaddr:=taillvm.getelementptr_reg_tai_size_const(NR_NO,getpascalfieldaddr,ptrsinttype,vs.offsetfromllvmfield,true);
              { ... and set the result type of the getelementptr }
              getpascalfieldaddr:=wrap_with_type(getpascalfieldaddr,cpointerdef.getreusable(u8inttype));
              llvmfielddef:=u8inttype;
            end;
          { bitcast the data at the final offset to the right type }
          if llvmfielddef<>vs.vardef then
            getpascalfieldaddr:=wrap_with_type(taillvm.op_reg_tai_size(la_bitcast,NR_NO,getpascalfieldaddr,cpointerdef.getreusable(vs.vardef)),cpointerdef.getreusable(vs.vardef));
        end;
      update_queued_tai(cpointerdef.getreusable(vs.vardef),getpascalfieldaddr,getllvmfieldaddr,1);
    end;


  procedure tllvmtai_typedconstbuilder.queue_typeconvn(fromdef, todef: tdef);
    var
      ai: taillvm;
      typedai: tai;
      tmpintdef: tdef;
      op,
      firstop,
      secondop: tllvmop;
    begin
      inherited;
      { special case: procdef -> procvardef/pointerdef: must take address of
        the procdef }
      if (fromdef.typ=procdef) and
         (todef.typ<>procdef) then
        fromdef:=cprocvardef.getreusableprocaddr(tprocdef(fromdef),pc_address_only);
      { typecasting a pointer-sized entity to a complex procvardef -> convert
        to the pointer-component of the complex procvardef (not always, because
        e.g. a tmethod to complex procvar initialises the entire complex
        procvar) }
      if (todef.typ=procvardef) and
         not tprocvardef(todef).is_addressonly and
         (fromdef.size<todef.size) then
        todef:=cprocvardef.getreusableprocaddr(tprocvardef(todef),pc_address_only);
      op:=llvmconvop(fromdef,todef,false);
      case op of
        la_ptrtoint_to_x,
        la_x_to_inttoptr:
          begin
            { convert via an integer with the same size as "x" }
            if op=la_ptrtoint_to_x then
              begin
                tmpintdef:=cgsize_orddef(def_cgsize(todef));
                firstop:=la_ptrtoint;
                secondop:=la_bitcast
              end
            else
              begin
                tmpintdef:=cgsize_orddef(def_cgsize(fromdef));
                firstop:=la_bitcast;
                secondop:=la_inttoptr;
              end;
            { since we have to queue operations from outer to inner, first queue
              the conversion from the tempintdef to the todef }
            ai:=taillvm.op_reg_tai_size(secondop,NR_NO,nil,todef);
            typedai:=wrap_with_type(ai,todef);
            update_queued_tai(todef,typedai,ai,1);
            todef:=tmpintdef;
            op:=firstop
          end;
        else
          ;
      end;
      ai:=taillvm.op_reg_tai_size(op,NR_NO,nil,todef);
      typedai:=wrap_with_type(ai,todef);
      update_queued_tai(todef,typedai,ai,1);
    end;


  procedure tllvmtai_typedconstbuilder.queue_emit_staticvar(vs: tstaticvarsym);
    begin
      { we've already incorporated the offset via the inserted operations above,
        make sure it doesn't get emitted again as part of the tai_const for
        the tasmsymbol }
      fqueue_offset:=0;
      inherited;
    end;


  procedure tllvmtai_typedconstbuilder.queue_emit_label(l: tlabelsym);
    var
      ai: taillvm;
      typedai: tai;
      tmpintdef: tdef;
      op,
      firstop,
      secondop: tllvmop;
    begin
      if not assigned(l.asmblocklabel) or
         not l.asmblocklabel.defined_in_asmstatement then
        begin
          ai:=taillvm.blockaddress(voidcodepointertype,
              current_asmdata.RefAsmSymbol(current_procinfo.procdef.mangledname,AT_FUNCTION),
              current_asmdata.RefAsmSymbol(l.mangledname,AT_LABEL)
            );
          emit_tai(ai,voidcodepointertype);
          fqueue_offset:=low(fqueue_offset);
        end
      else
        begin
          { we've already incorporated the offset via the inserted operations above,
            make sure it doesn't get emitted again as part of the tai_const for
            the tasmsymbol }
          fqueue_offset:=0;
          inherited;
        end;
    end;


  procedure tllvmtai_typedconstbuilder.queue_emit_asmsym(sym: tasmsymbol; def: tdef);
    begin
      { we've already incorporated the offset via the inserted operations above,
        make sure it doesn't get emitted again as part of the tai_const for
        the tasmsymbol }
      fqueue_offset:=0;
      inherited;
    end;


  procedure tllvmtai_typedconstbuilder.queue_emit_ordconst(value: int64; def: tdef);
    var
      valuedef: tdef;
    begin
      { no offset into an ordinal constant }
      if fqueue_offset<>0 then
        internalerror(2015030702);
      if not is_ordinal(def) then
        begin
          { insert an ordinal -> non-ordinal (e.g. pointer) conversion, as you
            cannot have integer constants as pointer values in LLVM }
          int_to_type(value,valuedef);
          queue_typeconvn(valuedef,def);
          { and now emit the constant as an ordinal }
          def:=valuedef;
        end;
      inherited;
    end;


  class function tllvmtai_typedconstbuilder.get_vectorized_dead_strip_custom_section_name(const basename: TSymStr; st: tsymtable; out secname: TSymStr): boolean;
    begin
      result:=inherited;
      if result then
        exit;
      { put all of the resource strings in a single section: it doesn't hurt,
        and this avoids problems with Darwin/mach-o's limitation of 255
        sections }
      secname:=basename;
      { Darwin requires specifying a segment name too }
      if target_info.system in systems_darwin then
        secname:='__DATA,'+secname;
      result:=true;
    end;


  function tllvmtai_typedconstbuilder.emit_placeholder(def: tdef): ttypedconstplaceholder;
    var
      pos: longint;
    begin
      check_add_placeholder(def);
      { we can't support extended constants, because those are transformed into
        an array of bytes, so we can't easily replace them afterwards }
      if (def.typ=floatdef) and
         (tfloatdef(def).floattype=s80real) then
        internalerror(2015091003);
      pos:=tllvmaggregateinformation(curagginfo).aggai.valuecount;
      emit_tai(tai_marker.Create(mark_position),def);
      result:=tllvmtypedconstplaceholder.create(tllvmaggregateinformation(curagginfo),pos,def);
    end;


  class function tllvmtai_typedconstbuilder.get_string_symofs(typ: tstringtype; winlikewidestring: boolean): pint;
    begin
      { LLVM does not support labels in the middle of a declaration }
      result:=get_string_header_size(typ,winlikewidestring);
    end;


  class function tllvmtai_typedconstbuilder.get_dynarray_symofs: pint;
    begin
      { LLVM does not support labels in the middle of a declaration }
      result:=get_dynarray_header_size;
    end;


begin
  ctai_typedconstbuilder:=tllvmtai_typedconstbuilder;
end.

