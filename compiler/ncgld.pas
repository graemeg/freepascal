 {
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate assembler for nodes that handle loads and assignments which
    are the same for all (most) processors

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
unit ncgld;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      symtype,symsym,
      aasmdata,
      node,nld,cgutils;

    type
       tcgloadnode = class(tloadnode)
         protected
          procedure generate_nested_access(vs: tsym);virtual;
          procedure generate_absaddr_access(vs: tabsolutevarsym); virtual;
          procedure generate_threadvar_access(gvs: tstaticvarsym); virtual;
         public
          procedure pass_generate_code;override;
          procedure changereflocation(const ref: treference);
       end;

       tcgassignmentnode = class(tassignmentnode)
        protected
          function maybechangetemp(list: TAsmList; var n: tnode; const newref: treference): boolean;virtual;
        public
          procedure pass_generate_code;override;
       end;

       tcgarrayconstructornode = class(tarrayconstructornode)
         protected
          procedure makearrayref(var ref: treference; eledef: tdef);virtual;
          procedure advancearrayoffset(var ref: treference; elesize: asizeint);virtual;
         public
          procedure pass_generate_code;override;
       end;

       tcgrttinode = class(trttinode)
          procedure pass_generate_code;override;
       end;


implementation

    uses
      cutils,
      systems,
      verbose,globals,constexp,
      nutils,
      symtable,symconst,symdef,defutil,paramgr,ncon,nbas,ncgrtti,
      aasmbase,
      cgbase,pass_2,
      procinfo,
      cpubase,parabase,cpuinfo,
      tgobj,ncgutil,
      cgobj,hlcgobj,
      ncgbas,ncgflw,
      wpobase;

{*****************************************************************************
                   SSA (for memory temps) support
*****************************************************************************}

    type
      preplacerefrec = ^treplacerefrec;
      treplacerefrec = record
        old, new: preference;
        ressym: tsym;
      end;

    function doreplaceref(var n: tnode; para: pointer): foreachnoderesult;
      var
        rr: preplacerefrec absolute para;
      begin
        result := fen_false;
        case n.nodetype of
          loadn:
            begin
                 { regular variable }
              if (tabstractvarsym(tloadnode(n).symtableentry).varoptions * [vo_is_dll_var, vo_is_thread_var] = []) and
                 not assigned(tloadnode(n).left) and
                 { not function result, or no exit in function }
                 (((tloadnode(n).symtableentry <> rr^.ressym) and
                   not(vo_is_funcret in tabstractvarsym(tloadnode(n).symtableentry).varoptions)) or
                  not(fc_exit in flowcontrol)) and
                 { stored in memory... }
                 (tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.loc in [LOC_REFERENCE]) and
                 { ... at the place we are looking for }
                 references_equal(tabstractnormalvarsym(tloadnode(n).symtableentry).localloc.reference,rr^.old^) and
                 { its address cannot have escaped the current routine }
                 not(tabstractvarsym(tloadnode(n).symtableentry).addr_taken) then
                begin
                  { relocate variable }
                  tcgloadnode(n).changereflocation(rr^.new^);
                  result := fen_norecurse_true;
                end;
            end;
          temprefn:
            begin
              if (ti_valid in ttemprefnode(n).tempinfo^.flags) and
                 { memory temp... }
                 (ttemprefnode(n).tempinfo^.location.loc in [LOC_REFERENCE]) and
                 { ... at the place we are looking for }
                 references_equal(ttemprefnode(n).tempinfo^.location.reference,rr^.old^) and
                 { its address cannot have escaped the current routine }
                 not(ti_addr_taken in ttemprefnode(n).tempinfo^.flags) then
                begin
                  { relocate the temp }
                  tcgtemprefnode(n).changelocation(rr^.new^);
                  result := fen_norecurse_true;
                end;
            end;
          { Subscriptn must be rejected, otherwise we may replace an
            an entire record with a temp for its first field, mantis #13948)
            Exception: the field's size is the same as the entire record

            The same goes for array indexing
          }
          subscriptn,
          vecn:
            if not(tunarynode(n).left.resultdef.typ in [recorddef,objectdef,arraydef,stringdef]) or
               { make sure we don't try to call resultdef.size for types that
                 don't have a compile-time size such as open arrays }
               is_special_array(tunarynode(n).left.resultdef) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               (tunarynode(n).left.resultdef.size<>tunarynode(n).resultdef.size) then
=======
               (tsubscriptnode(n).left.resultdef.size <> tunarynode(n).resultdef.size) then
>>>>>>> graemeg/cpstrnew
=======
               (tsubscriptnode(n).left.resultdef.size <> tunarynode(n).resultdef.size) then
>>>>>>> graemeg/cpstrnew
=======
               (tsubscriptnode(n).left.resultdef.size <> tunarynode(n).resultdef.size) then
>>>>>>> graemeg/cpstrnew
=======
               (tsubscriptnode(n).left.resultdef.size <> tunarynode(n).resultdef.size) then
>>>>>>> origin/cpstrnew
              result := fen_norecurse_false;

          { optimize the searching a bit }
          derefn,addrn,
          calln,inlinen,casen,
          addn,subn,muln,
          andn,orn,xorn,
          ltn,lten,gtn,gten,equaln,unequaln,
          slashn,divn,shrn,shln,notn,
          inn,
          asn,isn:
            result := fen_norecurse_false;
        end;
      end;


    function tcgassignmentnode.maybechangetemp(list: TAsmList; var n: tnode; const newref: treference): boolean;
      var
        rr: treplacerefrec;
      begin
        result := false;

        { only do for -O2 or higher (breaks debugging since }
        { variables move to different memory locations)     }
        if not(cs_opt_level2 in current_settings.optimizerswitches) or
           { must be a copy to a memory location ... }
           (n.location.loc <> LOC_REFERENCE) or
           { not inside a control flow statement and no goto's in sight }
           ([fc_inflowcontrol,fc_gotolabel] * flowcontrol <> []) or
           { not for refcounted types, because those locations are   }
           { still used later on in initialisation/finalisation code }
           is_managed_type(n.resultdef) or
           { source and destination are temps (= not global variables) }
           not tg.istemp(n.location.reference) or
           not tg.istemp(newref) or
           { and both point to the start of a temp, and the source is a }
           { non-persistent temp (otherwise we need some kind of copy-  }
           { on-write support in case later on both are still used)     }
           (tg.gettypeoftemp(newref) <> tt_normal) or
           not (tg.gettypeoftemp(n.location.reference) in [tt_normal,tt_persistent]) or
           { and both have the same size }
           (tg.sizeoftemp(current_asmdata.CurrAsmList,newref) <> tg.sizeoftemp(current_asmdata.CurrAsmList,n.location.reference)) then
          exit;

        { find the source of the old reference (loadnode or tempnode) }
        { and replace it with the new reference                       }
        rr.old := @n.location.reference;
        rr.new := @newref;
        rr.ressym := nil;

        if assigned(current_procinfo.procdef.funcretsym) and
           (tabstractvarsym(current_procinfo.procdef.funcretsym).refs <> 0) then
          if (current_procinfo.procdef.proctypeoption=potype_constructor) then
            rr.ressym:=tsym(current_procinfo.procdef.parast.Find('self'))
         else
            rr.ressym:=current_procinfo.procdef.funcretsym;

        { if source not found, don't do anything }
        if not foreachnodestatic(n,@doreplaceref,@rr) then
          exit;

        n.location.reference := newref;
        result:=true;
      end;

{*****************************************************************************
                             SecondLoad
*****************************************************************************}

    procedure tcgloadnode.changereflocation(const ref: treference);
      var
        oldtemptype: ttemptype;
      begin
        if (location.loc<>LOC_REFERENCE) then
          internalerror(2007020812);
        if not tg.istemp(location.reference) then
          internalerror(2007020813);
        oldtemptype:=tg.gettypeoftemp(location.reference);
        if (oldtemptype = tt_persistent) then
          tg.ChangeTempType(current_asmdata.CurrAsmList,location.reference,tt_normal);
        tg.ungettemp(current_asmdata.CurrAsmList,location.reference);
        location.reference:=ref;
        tg.ChangeTempType(current_asmdata.CurrAsmList,location.reference,oldtemptype);
        tabstractnormalvarsym(symtableentry).localloc:=location;
      end;


    procedure tcgloadnode.generate_nested_access(vs: tsym);
      var
        { paramter declared as tsym to reduce interface unit dependencies }
        lvs: tabstractnormalvarsym absolute vs;
      begin
        secondpass(left);
        if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
          internalerror(200309286);
        if lvs.localloc.loc<>LOC_REFERENCE then
          internalerror(200409241);
        hlcg.reference_reset_base(location.reference,left.resultdef,left.location.register,lvs.localloc.reference.offset,lvs.localloc.reference.alignment);
      end;


    procedure tcgloadnode.generate_absaddr_access(vs: tabsolutevarsym);
      begin
        location.reference.offset:=asizeint(vs.addroffset);
      end;


    procedure tcgloadnode.generate_threadvar_access(gvs: tstaticvarsym);
      var
        respara,
        paraloc1 : tcgpara;
        fieldptrdef,
        pvd : tdef;
        endrelocatelab,
        norelocatelab : tasmlabel;
        tvref,
        href : treference;
        hregister : tregister;
        tv_rec : trecorddef;
        tv_index_field,
        tv_non_mt_data_field: tsym;
        tmpresloc: tlocation;
      begin
         if (tf_section_threadvars in target_info.flags) then
           begin
             if gvs.localloc.loc=LOC_INVALID then
               if not(vo_is_weak_external in gvs.varoptions) then
                 reference_reset_symbol(location.reference,current_asmdata.RefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
               else
                 reference_reset_symbol(location.reference,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
             else
               location:=gvs.localloc;
           end
         else
           begin
             {
               Thread var loading is optimized to first check if
               a relocate function is available. When the function
               is available it is called to retrieve the address.
               Otherwise the address is loaded with the symbol
             }
             tv_rec:=get_threadvar_record(resultdef,tv_index_field,tv_non_mt_data_field);
             fieldptrdef:=cpointerdef.getreusable(resultdef);
             current_asmdata.getjumplabel(norelocatelab);
             current_asmdata.getjumplabel(endrelocatelab);
             { make sure hregister can't allocate the register necessary for the parameter }
             pvd:=search_system_type('TRELOCATETHREADVARHANDLER').typedef;
             if pvd.typ<>procvardef then
               internalerror(2012120901);

             { FPC_THREADVAR_RELOCATE is nil? }
             paraloc1.init;
             paramanager.getintparaloc(current_asmdata.CurrAsmList,tprocvardef(pvd),1,paraloc1);
             hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,pvd);
             reference_reset_symbol(href,current_asmdata.RefAsmSymbol('FPC_THREADVAR_RELOCATE'),0,pvd.size);
             hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,pvd,pvd,href,hregister);
             hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,pvd,OC_EQ,0,hregister,norelocatelab);
             { no, call it with the index of the threadvar as parameter }
             if not(vo_is_weak_external in gvs.varoptions) then
               reference_reset_symbol(tvref,current_asmdata.RefAsmSymbol(gvs.mangledname),0,sizeof(pint))
             else
               reference_reset_symbol(tvref,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),0,sizeof(pint));
             href:=tvref;
             hlcg.g_set_addr_nonbitpacked_field_ref(current_asmdata.CurrAsmList,
               tv_rec,
               tfieldvarsym(tv_index_field),href);
             hlcg.a_load_ref_cgpara(current_asmdata.CurrAsmList,tfieldvarsym(tv_index_field).vardef,href,paraloc1);
             paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
             cg.allocallcpuregisters(current_asmdata.CurrAsmList);
             { result is the address of the threadvar }
             respara:=hlcg.a_call_reg(current_asmdata.CurrAsmList,tprocvardef(pvd),hregister,[@paraloc1]);
             paraloc1.done;
             cg.deallocallcpuregisters(current_asmdata.CurrAsmList);

             { load the address of the result in hregister }
             hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,fieldptrdef);
             location_reset(tmpresloc,LOC_REGISTER,def_cgsize(fieldptrdef));
             tmpresloc.register:=hregister;
             hlcg.gen_load_cgpara_loc(current_asmdata.CurrAsmList,fieldptrdef,respara,tmpresloc,true);
             respara.resetiftemp;
             hlcg.a_jmp_always(current_asmdata.CurrAsmList,endrelocatelab);

             { no relocation needed, load the address of the variable only, the
               layout of a threadvar is:
                 0            - Threadvar index
                 sizeof(pint) - Threadvar value in single threading }
             hlcg.a_label(current_asmdata.CurrAsmList,norelocatelab);
             href:=tvref;
             hlcg.g_set_addr_nonbitpacked_field_ref(current_asmdata.CurrAsmList,
               tv_rec,
               tfieldvarsym(tv_non_mt_data_field),href);
             { load in the same "hregister" as above, so after this sequence
               the address of the threadvar is always in hregister }
             hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,resultdef,fieldptrdef,href,hregister);
             hlcg.a_label(current_asmdata.CurrAsmList,endrelocatelab);

             hlcg.reference_reset_base(location.reference,fieldptrdef,hregister,0,location.reference.alignment);
           end;
       end;


    procedure tcgloadnode.pass_generate_code;
      var
        hregister : tregister;
        vs   : tabstractnormalvarsym;
        gvs  : tstaticvarsym;
        vmtdef : tpointerdef;
        vmtentry: tfieldvarsym;
        pd   : tprocdef;
        href : treference;
        newsize : tcgsize;
        vd : tdef;
      begin
        { we don't know the size of all arrays }
        newsize:=def_cgsize(resultdef);
        { alignment is overridden per case below }
        location_reset_ref(location,LOC_REFERENCE,newsize,resultdef.alignment);
        case symtableentry.typ of
           absolutevarsym :
              begin
                 { this is only for toasm and toaddr }
                 case tabsolutevarsym(symtableentry).abstyp of
                   toaddr :
<<<<<<< HEAD
                     generate_absaddr_access(tabsolutevarsym(symtableentry));
=======
                     begin
{$ifdef i386}
                       if tabsolutevarsym(symtableentry).absseg then
                         location.reference.segment:=NR_FS;
{$endif i386}
                       location.reference.offset:=aint(tabsolutevarsym(symtableentry).addroffset);
                     end;
>>>>>>> graemeg/cpstrnew
                   toasm :
                     location.reference.symbol:=current_asmdata.RefAsmSymbol(tabsolutevarsym(symtableentry).mangledname);
                   else
                     internalerror(200310283);
                 end;
              end;
           constsym:
             begin
                if tconstsym(symtableentry).consttyp=constresourcestring then
                  begin
                     location_reset_ref(location,LOC_CREFERENCE,def_cgsize(cansistringtype),cansistringtype.size);
                     location.reference.symbol:=current_asmdata.RefAsmSymbol(make_mangledname('RESSTR',symtableentry.owner,symtableentry.name),AT_DATA);
                     vd:=search_system_type('TRESOURCESTRINGRECORD').typedef;
                     hlcg.g_set_addr_nonbitpacked_field_ref(
                       current_asmdata.CurrAsmList,
                       trecorddef(vd),
                       tfieldvarsym(search_struct_member(trecorddef(vd),'CURRENTVALUE')),
                       location.reference);
                  end
                else
                  internalerror(22798);
             end;
           staticvarsym :
             begin
               gvs:=tstaticvarsym(symtableentry);

               if (vo_is_dll_var in gvs.varoptions) then
               { DLL variable }
                 begin
                   hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                   if not(vo_is_weak_external in gvs.varoptions) then
                     location.reference.symbol:=current_asmdata.RefAsmSymbol(tstaticvarsym(symtableentry).mangledname)
                   else
                     location.reference.symbol:=current_asmdata.WeakRefAsmSymbol(tstaticvarsym(symtableentry).mangledname);
                   cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,location.reference,hregister);
                   reference_reset_base(location.reference,hregister,0,location.reference.alignment);
                 end
               { Thread variable }
               else if (vo_is_thread_var in gvs.varoptions) then
                 generate_threadvar_access(gvs)
               { Normal (or external) variable }
               else
                 begin
<<<<<<< HEAD
                   if gvs.localloc.loc=LOC_INVALID then
                     if not(vo_is_weak_external in gvs.varoptions) then
                       reference_reset_symbol(location.reference,current_asmdata.RefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
                     else
                       reference_reset_symbol(location.reference,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
                   else
                     location:=gvs.localloc;
                 end;
=======
                    if (tf_section_threadvars in target_info.flags) then
                      begin
                        if gvs.localloc.loc=LOC_INVALID then
                          if not(vo_is_weak_external in gvs.varoptions) then
                            reference_reset_symbol(location.reference,current_asmdata.RefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
                          else
                            reference_reset_symbol(location.reference,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
                        else
                          location:=gvs.localloc;
{$ifdef i386}
                        case target_info.system of
                          system_i386_linux:
                            location.reference.segment:=NR_GS;
                          system_i386_win32:
                            location.reference.segment:=NR_FS;
                        end;
{$endif i386}
                      end
                    else
                      begin
                        {
                          Thread var loading is optimized to first check if
                          a relocate function is available. When the function
                          is available it is called to retrieve the address.
                          Otherwise the address is loaded with the symbol

                          The code needs to be in the order to first handle the
                          call and then the address load to be sure that the
                          register that is used for returning is the same (PFV)
                        }
                        current_asmdata.getjumplabel(norelocatelab);
                        current_asmdata.getjumplabel(endrelocatelab);
                        { make sure hregister can't allocate the register necessary for the parameter }
                        paraloc1.init;
                        paramanager.getintparaloc(pocall_default,1,paraloc1);
                        hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                        reference_reset_symbol(href,current_asmdata.RefAsmSymbol('FPC_THREADVAR_RELOCATE'),0,sizeof(pint));
                        cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,hregister);
                        cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_ADDR,OC_EQ,0,hregister,norelocatelab);
                        { don't save the allocated register else the result will be destroyed later }
                        if not(vo_is_weak_external in gvs.varoptions) then
                          reference_reset_symbol(href,current_asmdata.RefAsmSymbol(gvs.mangledname),0,sizeof(pint))
                        else
                          reference_reset_symbol(href,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),0,sizeof(pint));
                        cg.a_load_ref_cgpara(current_asmdata.CurrAsmList,OS_32,href,paraloc1);
                        paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
                        paraloc1.done;
                        cg.allocallcpuregisters(current_asmdata.CurrAsmList);
                        cg.a_call_reg(current_asmdata.CurrAsmList,hregister);
                        cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
                        cg.getcpuregister(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
                        cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
                        hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                        cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_ADDR,NR_FUNCTION_RESULT_REG,hregister);
                        cg.a_jmp_always(current_asmdata.CurrAsmList,endrelocatelab);
                        cg.a_label(current_asmdata.CurrAsmList,norelocatelab);
                        { no relocation needed, load the address of the variable only, the
                          layout of a threadvar is (4 bytes pointer):
                            0 - Threadvar index
                            4 - Threadvar value in single threading }
                        if not(vo_is_weak_external in gvs.varoptions) then
                          reference_reset_symbol(href,current_asmdata.RefAsmSymbol(gvs.mangledname),sizeof(pint),sizeof(pint))
                        else
                          reference_reset_symbol(href,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),sizeof(pint),sizeof(pint));
                        cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,href,hregister);
                        cg.a_label(current_asmdata.CurrAsmList,endrelocatelab);
                        location.reference.base:=hregister;
                      end;
                  end
                { Normal (or external) variable }
                else
                  begin
                    if gvs.localloc.loc=LOC_INVALID then
                      if not(vo_is_weak_external in gvs.varoptions) then
                        reference_reset_symbol(location.reference,current_asmdata.RefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
                      else
                        reference_reset_symbol(location.reference,current_asmdata.WeakRefAsmSymbol(gvs.mangledname),0,location.reference.alignment)
                    else
                      location:=gvs.localloc;
                  end;
>>>>>>> graemeg/cpstrnew

                { make const a LOC_CREFERENCE }
                if (gvs.varspez=vs_const) and
                   (location.loc=LOC_REFERENCE) then
                  location.loc:=LOC_CREFERENCE;
              end;
            paravarsym,
            localvarsym :
              begin
                vs:=tabstractnormalvarsym(symtableentry);
                { Nested variable }
                if assigned(left) then
<<<<<<< HEAD
                  generate_nested_access(vs)
=======
                  begin
                    secondpass(left);
                    if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
                      internalerror(200309286);
                    if vs.localloc.loc<>LOC_REFERENCE then
                      internalerror(200409241);
                    reference_reset_base(location.reference,left.location.register,vs.localloc.reference.offset,vs.localloc.reference.alignment);
                  end
>>>>>>> graemeg/cpstrnew
                else
                  location:=vs.localloc;

                { handle call by reference variables when they are not
                  already copied to local copies. Also ignore the reference
                  when we need to load the self pointer for objects }
                if is_addr_param_load then
                  begin
                    if (location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                      hregister:=location.register
                    else
                      begin
                        vd:=cpointerdef.getreusable(resultdef);
                        hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,vd);
                        { we need to load only an address }
                        location.size:=int_cgsize(vd.size);
                        hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,vd,vd,location,hregister);
                      end;
                    { assume packed records may always be unaligned }
                    if not(resultdef.typ in [recorddef,objectdef]) or
                       (tabstractrecordsymtable(tabstractrecorddef(resultdef).symtable).usefieldalignment<>1) then
                      location_reset_ref(location,LOC_REFERENCE,newsize,resultdef.alignment)
                    else
                      location_reset_ref(location,LOC_REFERENCE,newsize,1);
                    hlcg.reference_reset_base(location.reference,voidpointertype,hregister,0,location.reference.alignment);
                  end;

                { make const a LOC_CREFERENCE }
                if (vs.varspez=vs_const) and
                   (location.loc=LOC_REFERENCE) then
                  location.loc:=LOC_CREFERENCE;
             end;
           procsym:
              begin
                 if not assigned(procdef) then
                   internalerror(200312011);
                 if assigned(left) then
                   begin
                     location_reset(location,LOC_CREGISTER,int_cgsize(voidpointertype.size*2));
                     secondpass(left);

                     { load class instance/classrefdef address }
                     if left.location.loc=LOC_CONSTANT then
                       hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
                     { vd will contain the type of the self pointer (self in
                       case of a class/classref, address of self in case of
                       an object }
                     vd:=nil;
                     case left.location.loc of
                        LOC_CREGISTER,
                        LOC_REGISTER:
                          begin
                             { this is not possible for objects }
                             if is_object(left.resultdef) then
                               internalerror(200304234);
                             location.registerhi:=left.location.register;
                             vd:=left.resultdef;
                          end;
                        LOC_CREFERENCE,
                        LOC_REFERENCE:
                          begin
                             if is_implicit_pointer_object_type(left.resultdef) or 
                                 (left.resultdef.typ=classrefdef) then
                               begin
                                 vd:=left.resultdef;
                                 location.registerhi:=hlcg.getaddressregister(current_asmdata.CurrAsmList,left.resultdef);
                                 hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,left.resultdef,left.resultdef,left.location.reference,location.registerhi)
                               end
                             else
                               begin
                                 vd:=cpointerdef.getreusable(left.resultdef);
                                 location.registerhi:=hlcg.getaddressregister(current_asmdata.CurrAsmList,vd);
                                 hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.resultdef,vd,left.location.reference,location.registerhi);
                               end;
                             location_freetemp(current_asmdata.CurrAsmList,left.location);
                          end;
                        else
                          internalerror(200610311);
                     end;

<<<<<<< HEAD
<<<<<<< HEAD
                     { virtual method ? }
                     if (po_virtualmethod in procdef.procoptions) and
                        not(loadnf_inherited in loadnodeflags) and
                        not is_objectpascal_helper(procdef.struct) then
                       begin
                         if (not assigned(current_procinfo) or
                             wpoinfomanager.symbol_live(current_procinfo.procdef.mangledname)) then
                           tobjectdef(procdef.struct).register_vmt_call(procdef.extnumber);
            {$ifdef vtentry}
                         if not is_interface(procdef.struct) then
=======
=======
>>>>>>> origin/fixes_2_2
                      { load class instance/classrefdef address }
                      if left.location.loc=LOC_CONSTANT then
                        location_force_reg(current_asmdata.CurrAsmList,left.location,OS_ADDR,false);
                      case left.location.loc of
                         LOC_CREGISTER,
                         LOC_REGISTER:
>>>>>>> graemeg/fixes_2_2
                           begin
                             inc(current_asmdata.NextVTEntryNr);
                             current_asmdata.CurrAsmList.Concat(tai_symbol.CreateName('VTREF'+tostr(current_asmdata.NextVTEntryNr)+'_'+procdef._class.vmt_mangledname+'$$'+tostr(vmtoffset div sizeof(pint)),AT_FUNCTION,0));
                           end;
            {$endif vtentry}
                         if (left.resultdef.typ=objectdef) and
                            assigned(tobjectdef(left.resultdef).vmt_field) then
                           begin
<<<<<<< HEAD
<<<<<<< HEAD
                             { vmt pointer is a pointer to the vmt record }
                             hlcg.reference_reset_base(href,vd,location.registerhi,0,vd.alignment);
                             vmtdef:=cpointerdef.getreusable(tobjectdef(left.resultdef).vmt_def);
                             hlcg.g_set_addr_nonbitpacked_field_ref(current_asmdata.CurrAsmList,tobjectdef(left.resultdef),tfieldvarsym(tobjectdef(left.resultdef).vmt_field),href);
                             hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,vmtdef);
                             hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,tfieldvarsym(tobjectdef(left.resultdef).vmt_field).vardef,vmtdef,href,hregister);
                           end
                         else if left.resultdef.typ=classrefdef then
                           begin
                             { classrefdef is a pointer to the vmt already }
                             hregister:=location.registerhi;
                             vmtdef:=cpointerdef.getreusable(tobjectdef(tclassrefdef(left.resultdef).pointeddef).vmt_def);
                             hlcg.g_ptrtypecast_reg(current_asmdata.CurrAsmList,left.resultdef,vmtdef,hregister);
                           end
                         else if is_any_interface_kind(left.resultdef) then
                           begin
                             { an interface is a pointer to a pointer to a vmt }
                             hlcg.reference_reset_base(href,vd,location.registerhi,0,vd.alignment);
                             vmtdef:=cpointerdef.getreusable(tobjectdef(left.resultdef).vmt_def);
                             hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,vmtdef);
                             hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,vmtdef,vmtdef,href,hregister);
                           end
                         else
                           internalerror(2015112501);
                         { load method address }
                         vmtentry:=tabstractrecordsymtable(trecorddef(vmtdef.pointeddef).symtable).findfieldbyoffset(
                           tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber));
                         hlcg.reference_reset_base(href,vmtdef,hregister,0,vmtdef.alignment);
                         location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,vmtentry.vardef);
                         hlcg.g_set_addr_nonbitpacked_field_ref(current_asmdata.CurrAsmList,tabstractrecorddef(vmtdef.pointeddef),vmtentry,href);
                         hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,vmtentry.vardef,vmtentry.vardef,href,location.register);
=======
                             { load vmt pointer }
                             reference_reset_base(href,hregister,tobjectdef(left.resultdef).vmt_offset,sizeof(pint));
                             hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                             cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,hregister);
                           end;
                         { load method address }
                         reference_reset_base(href,hregister,tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber),sizeof(pint));
                         hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                         cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,hregister);
                         { ... and store it }
                         cg.a_load_reg_ref(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,hregister,location.reference);
>>>>>>> graemeg/cpstrnew
                       end
                     else
                       begin
                         { load address of the function }
                         reference_reset_symbol(href,current_asmdata.RefAsmSymbol(procdef.mangledname),0,procdef.address_type.alignment);
                         location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,cprocvardef.getreusableprocaddr(procdef));
                         hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,procdef,cprocvardef.getreusableprocaddr(procdef),href,location.register);
                       end;

                     { to get methodpointers stored correctly, code and self register must be swapped on
                       big endian targets }
                     if target_info.endian=endian_big then
                       begin
                         hregister:=location.register;
                         location.register:=location.registerhi;
                         location.registerhi:=hregister;
                       end;
                   end
                 else
                   begin
                      pd:=tprocdef(tprocsym(symtableentry).ProcdefList[0]);
                      { def_cgsize does not work for tprocdef, so we use pd.address_type }
                      location.size:=def_cgsize(pd.address_type);
                      if not(po_weakexternal in pd.procoptions) then
                        location.reference.symbol:=current_asmdata.RefAsmSymbol(procdef.mangledname)
                      else
                        location.reference.symbol:=current_asmdata.WeakRefAsmSymbol(procdef.mangledname);
                   end;
              end;
           labelsym :
             if assigned(tlabelsym(symtableentry).asmblocklabel) then
               location.reference.symbol:=tlabelsym(symtableentry).asmblocklabel
             else
               location.reference.symbol:=tcglabelnode((tlabelsym(symtableentry).code)).getasmlabel;
           else internalerror(200510032);
        end;
=======
                              hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                              if not is_object(left.resultdef) then
                                cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,left.location.reference,hregister)
                              else
                                cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.location.reference,hregister);
                              location_freetemp(current_asmdata.CurrAsmList,left.location);
                           end;
                         else
                           internalerror(200610311);
                      end;

                      { store the class instance or classredef address }
                      href:=location.reference;
                      inc(href.offset,sizeof(aint));
                      cg.a_load_reg_ref(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,hregister,href);

                      { virtual method ? }
                      if (po_virtualmethod in procdef.procoptions) and
                         not(nf_inherited in flags) then
                        begin
                          { a classrefdef already points to the VMT }
                          if (left.resultdef.typ<>classrefdef) then
                            begin
                              { load vmt pointer }
                              reference_reset_base(href,hregister,0);
                              hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                              cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,hregister);
                            end;
                          { load method address }
                          reference_reset_base(href,hregister,procdef._class.vmtmethodoffset(procdef.extnumber));
                          hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                          cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,href,hregister);
                          { ... and store it }
                          cg.a_load_reg_ref(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,hregister,location.reference);
                        end
                      else
                        begin
                          { load address of the function }
                          reference_reset_symbol(href,current_asmdata.RefAsmSymbol(procdef.mangledname),0);
                          hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                          cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,href,hregister);
                          cg.a_load_reg_ref(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,hregister,location.reference);
                        end;
                    end
                  else
                    begin
                       pd:=tprocdef(tprocsym(symtableentry).ProcdefList[0]);
                       if (po_external in pd.procoptions) then
                         location.reference.base := cg.g_indirect_sym_load(current_asmdata.CurrAsmList,pd.mangledname);
                       {!!!!! Be aware, work on virtual methods too }
                       if (location.reference.base = NR_NO) then
                         location.reference.symbol:=current_asmdata.RefAsmSymbol(procdef.mangledname);
                    end;
               end;
            labelsym :
              if assigned(tlabelsym(symtableentry).asmblocklabel) then
                location.reference.symbol:=tlabelsym(symtableentry).asmblocklabel
              else
                location.reference.symbol:=tcglabelnode((tlabelsym(symtableentry).code)).getasmlabel;
            else internalerror(200510032);
         end;
>>>>>>> graemeg/fixes_2_2
      end;


{*****************************************************************************
                             SecondAssignment
*****************************************************************************}

    procedure tcgassignmentnode.pass_generate_code;
      var
         hlabel : tasmlabel;
         href : treference;
         releaseright : boolean;
         alignmentrequirement,
         len : aint;
         r : tregister;
<<<<<<< HEAD
<<<<<<< HEAD
         {$if not defined(cpu64bitalu)}
         r64 : tregister64;
         {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         oldflowcontrol : tflowcontrol;
      begin
        { previously, managed types were handled in firstpass
          newer FPCs however can identify situations when
          assignments of managed types require no special code and the
          value could be just copied so this could should be able also to handle
          managed types without any special "managing code"}

        location_reset(location,LOC_VOID,OS_NO);

        {
          in most cases we can process first the right node which contains
          the most complex code. Exceptions for this are:
            - result is in flags, loading left will then destroy the flags
            - result is a jump, loading left must be already done before the jump is made
            - result need reference count, when left points to a value used in
              right then decreasing the refcnt on left can possibly release
              the memory before right increased the refcnt, result is that an
              empty value is assigned

           But not when the result is in the flags, then
          loading the left node afterwards can destroy the flags.
        }
        if not(right.expectloc in [LOC_FLAGS,LOC_JUMP]) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            (node_complexity(right)>node_complexity(left)) then
         begin
           secondpass(right);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
           (is_managed_type(right.resultdef) or
            (node_complexity(right)>node_complexity(left))) then
         begin
           secondpass(right);
           { increment source reference counter, this is
             useless for constants }
<<<<<<< HEAD
<<<<<<< HEAD
           if is_managed_type(right.resultdef) and
=======
           if (right.resultdef.needs_inittable) and
>>>>>>> graemeg/fixes_2_2
=======
           if (right.resultdef.needs_inittable) and
>>>>>>> origin/fixes_2_2
              not is_constnode(right) then
            begin
              location_force_mem(current_asmdata.CurrAsmList,right.location);
              location_get_data_ref(current_asmdata.CurrAsmList,right.location,href,false,sizeof(pint));
              cg.g_incrrefcount(current_asmdata.CurrAsmList,right.resultdef,href);
            end;
>>>>>>> graemeg/cpstrnew
           if codegenerror then
             exit;

           secondpass(left);
<<<<<<< HEAD
=======
           { decrement destination reference counter }
           if is_managed_type(left.resultdef) then
             begin
               location_get_data_ref(current_asmdata.CurrAsmList,left.location,href,false,sizeof(pint));
               cg.g_decrrefcount(current_asmdata.CurrAsmList,left.resultdef,href);
             end;
>>>>>>> graemeg/cpstrnew
           if codegenerror then
             exit;
         end
        else
         begin
           { calculate left sides }
           secondpass(left);
<<<<<<< HEAD
=======
           { decrement destination reference counter }
           if is_managed_type(left.resultdef) then
             begin
               location_get_data_ref(current_asmdata.CurrAsmList,left.location,href,false,sizeof(pint));
               cg.g_decrrefcount(current_asmdata.CurrAsmList,left.resultdef,href);
             end;
>>>>>>> graemeg/cpstrnew
           if codegenerror then
             exit;

           { tell the SSA/SSL code that the left side was handled first so
             ni SSL is done
           }
           oldflowcontrol:=flowcontrol;
           include(flowcontrol,fc_lefthandled);

<<<<<<< HEAD
<<<<<<< HEAD
           secondpass(right);
           flowcontrol:=oldflowcontrol;
<<<<<<< HEAD
=======
=======
=======
>>>>>>> origin/fixes_2_2
           { left can't be never a 64 bit LOC_REGISTER, so the 3. arg }
           { can be false                                             }
           secondpass(right);
           flowcontrol:=oldflowcontrol;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           { increment source reference counter, this is
             useless for string constants}
           if is_managed_type(right.resultdef) and
              (right.nodetype<>stringconstn) then
             begin
               location_force_mem(current_asmdata.CurrAsmList,right.location);
               location_get_data_ref(current_asmdata.CurrAsmList,right.location,href,false,sizeof(pint));
               cg.g_incrrefcount(current_asmdata.CurrAsmList,right.resultdef,href);
             end;
>>>>>>> graemeg/cpstrnew

           if codegenerror then
             exit;
         end;

        releaseright:=true;

        { shortstring assignments are handled separately }
        if is_shortstring(left.resultdef) then
          begin
            {
              we can get here only in the following situations
              for the right node:
               - empty constant string
               - char
            }

            { The addn is replaced by a blockn or calln that already returns
              a shortstring }
            if is_shortstring(right.resultdef) and
               (right.nodetype in [blockn,calln]) then
              begin
                { verify that we indeed have nothing to do }
                if not(nf_assign_done_in_right in flags) then
                  internalerror(2015042201);
              end
            { empty constant string }
            else if (right.nodetype=stringconstn) and
               (tstringconstnode(right).len=0) then
              begin
                hlcg.g_ptrtypecast_ref(current_asmdata.CurrAsmList,cpointerdef.getreusable(left.resultdef),tpointerdef(charpointertype),left.location.reference);
                hlcg.a_load_const_ref(current_asmdata.CurrAsmList,cansichartype,0,left.location.reference);
              end
            { char loading }
            else if is_char(right.resultdef) then
              begin
                if right.nodetype=ordconstn then
                  begin
                    hlcg.g_ptrtypecast_ref(current_asmdata.CurrAsmList,cpointerdef.getreusable(left.resultdef),cpointerdef.getreusable(u16inttype),left.location.reference);
                    if (target_info.endian = endian_little) then
<<<<<<< HEAD
                      hlcg.a_load_const_ref(current_asmdata.CurrAsmList,u16inttype,(tordconstnode(right).value.svalue shl 8) or 1,
                          setalignment(left.location.reference,1))
                    else
                      hlcg.a_load_const_ref(current_asmdata.CurrAsmList,u16inttype,tordconstnode(right).value.svalue or (1 shl 8),
=======
                      cg.a_load_const_ref(current_asmdata.CurrAsmList,OS_16,(tordconstnode(right).value shl 8) or 1,
                          setalignment(left.location.reference,1))
                    else
                      cg.a_load_const_ref(current_asmdata.CurrAsmList,OS_16,tordconstnode(right).value or (1 shl 8),
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                          setalignment(left.location.reference,1));
                  end
                else
                  begin
                    href:=left.location.reference;
                    hlcg.g_ptrtypecast_ref(current_asmdata.CurrAsmList,cpointerdef.getreusable(left.resultdef),tpointerdef(charpointertype),href);
                    hlcg.a_load_const_ref(current_asmdata.CurrAsmList,cansichartype,1,href);
                    inc(href.offset,1);
                    case right.location.loc of
                      LOC_REGISTER,
                      LOC_CREGISTER :
                        begin
{$ifndef cpuhighleveltarget}
                          r:=cg.makeregsize(current_asmdata.CurrAsmList,right.location.register,OS_8);
{$else not cpuhighleveltarget}
                          r:=hlcg.getintregister(current_asmdata.CurrAsmList,u8inttype);
                          hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,cansichartype,u8inttype,right.location.register,r);
{$endif cpuhighleveltarget}
                          hlcg.a_load_reg_ref(current_asmdata.CurrAsmList,u8inttype,u8inttype,r,href);
                        end;
                      LOC_REFERENCE,
                      LOC_CREFERENCE :
                        hlcg.a_load_ref_ref(current_asmdata.CurrAsmList,cansichartype,cansichartype,right.location.reference,href);
                      else
                        internalerror(200205111);
                    end;
                  end;
              end
            else
              internalerror(2002042410);
          end
       { try to reuse memory locations instead of copying }
       { copy to a memory location ... }
        else if (right.location.loc = LOC_REFERENCE) and
           maybechangetemp(current_asmdata.CurrAsmList,left,right.location.reference) then
          begin
            { if it worked, we're done }
          end
        else
          begin
            { SSA support }
            hlcg.maybe_change_load_node_reg(current_asmdata.CurrAsmList,left,false);
            hlcg.maybe_change_load_node_reg(current_asmdata.CurrAsmList,right,true);
            case right.location.loc of
              LOC_CONSTANT :
                begin
<<<<<<< HEAD
{$ifndef cpu64bitalu}
                  if (left.location.size in [OS_64,OS_S64]) or (right.location.size in [OS_64,OS_S64]) then
                    cg64.a_load64_const_loc(current_asmdata.CurrAsmList,right.location.value64,left.location)
                  else
{$endif not cpu64bitalu}
                    hlcg.a_load_const_loc(current_asmdata.CurrAsmList,left.resultdef,right.location.value,left.location);
=======
{$ifndef cpu64bit}
                  if (left.location.size in [OS_64,OS_S64]) or (right.location.size in [OS_64,OS_S64]) then
                    cg64.a_load64_const_loc(current_asmdata.CurrAsmList,right.location.value64,left.location)
                  else
{$endif cpu64bit}
                    cg.a_load_const_loc(current_asmdata.CurrAsmList,right.location.value,left.location);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                end;
              LOC_REFERENCE,
              LOC_CREFERENCE :
                begin
                  case left.location.loc of
                    LOC_REGISTER,
                    LOC_CREGISTER :
                      begin
{$ifndef cpuhighleveltarget}
{$ifdef cpu64bitalu}
                        if left.location.size in [OS_128,OS_S128] then
                          cg128.a_load128_ref_reg(current_asmdata.CurrAsmList,right.location.reference,left.location.register128)
                        else
{$else cpu64bitalu}
                        if left.location.size in [OS_64,OS_S64] then
                          cg64.a_load64_ref_reg(current_asmdata.CurrAsmList,right.location.reference,left.location.register64)
                        else
{$endif cpu64bitalu}
{$endif not cpuhighleveltarget}
                          hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location.reference,left.location.register);
                      end;
                    LOC_FPUREGISTER,
                    LOC_CFPUREGISTER :
                      begin
                        hlcg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList,
                            right.resultdef,left.resultdef,
                            right.location.reference,
                            left.location.register);
                      end;
                    LOC_REFERENCE,
                    LOC_CREFERENCE :
                      begin
                        if (left.resultdef.typ=floatdef) and
                           (right.resultdef.typ=floatdef) and
                           (left.location.size<>right.location.size) then
                          begin
<<<<<<< HEAD
<<<<<<< HEAD
                            { assume that all float types can be handed by the
                              fpu if one can be handled by the fpu }
                            if not use_vectorfpu(left.resultdef) or
                               not use_vectorfpu(right.resultdef) then
                              hlcg.a_loadfpu_ref_ref(current_asmdata.CurrAsmList,
                                right.resultdef,left.resultdef,
                                right.location.reference,left.location.reference)
                            else
                              hlcg.a_loadmm_ref_ref(current_asmdata.CurrAsmList,
                                right.resultdef,left.resultdef,
                                right.location.reference,left.location.reference,mms_movescalar)
                          end
                        else
                          begin
{ TODO: HACK: unaligned test, maybe remove all unaligned locations (array of char) from the compiler}
                            { Use unaligned copy when the offset is not aligned }
                            len:=left.resultdef.size;
                            { can be 0 in case of formaldef on JVM target }
                            if len=0 then
                              len:=sizeof(pint);

                            { data smaller than an aint has less alignment requirements }
                            { max(1,...) avoids div by zero in case of an empty record  }
                            alignmentrequirement:=min(max(1,len),sizeof(aint));

                            if (right.location.reference.offset mod alignmentrequirement<>0) or
                              (left.location.reference.offset mod alignmentrequirement<>0) or
                              (right.resultdef.alignment<alignmentrequirement) or
                              ((right.location.reference.alignment<>0) and
                               (right.location.reference.alignment<alignmentrequirement)) or
                              ((left.location.reference.alignment<>0) and
                               (left.location.reference.alignment<alignmentrequirement)) then
                              hlcg.g_concatcopy_unaligned(current_asmdata.CurrAsmList,left.resultdef,right.location.reference,left.location.reference)
                            else
                              hlcg.g_concatcopy(current_asmdata.CurrAsmList,left.resultdef,right.location.reference,left.location.reference);
=======
=======
>>>>>>> origin/fixes_2_2
                            cg.a_loadfpu_ref_ref(current_asmdata.CurrAsmList,
                              right.location.size,left.location.size,
                              right.location.reference,left.location.reference)
                          end
                        else
                          begin
{$warning HACK: unaligned test, maybe remove all unaligned locations (array of char) from the compiler}
                            { Use unaligned copy when the offset is not aligned }
                            len:=left.resultdef.size;
                            if (right.location.reference.offset mod sizeof(aint)<>0) or
                              (left.location.reference.offset mod sizeof(aint)<>0) or
                              (right.resultdef.alignment<sizeof(aint)) or
                              ((right.location.reference.alignment<>0) and
                               (right.location.reference.alignment<sizeof(aint))) or
                              ((left.location.reference.alignment<>0) and
                               (left.location.reference.alignment<sizeof(aint))) then
                              cg.g_concatcopy_unaligned(current_asmdata.CurrAsmList,right.location.reference,left.location.reference,len)
                            else
                              cg.g_concatcopy(current_asmdata.CurrAsmList,right.location.reference,left.location.reference,len);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                          end;
                      end;
                    LOC_MMREGISTER,
                    LOC_CMMREGISTER:
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(x86) and not defined(llvm)}
=======
{$ifdef x86}
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
                        if (right.resultdef.typ=floatdef) and
                           not use_vectorfpu(right.resultdef) then
=======
{$ifdef x86}
                        if not use_sse(right.resultdef) then
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef x86}
                        if not use_sse(right.resultdef) then
>>>>>>> origin/fixes_2_2
                          begin
                            { perform size conversion if needed (the mm-code cannot }
                            { convert an extended into a double/single, since sse   }
                            { doesn't support extended)                             }
                            r:=cg.getfpuregister(current_asmdata.CurrAsmList,right.location.size);
<<<<<<< HEAD
<<<<<<< HEAD
                            tg.gethltemp(current_asmdata.CurrAsmList,left.resultdef,left.resultdef.size,tt_normal,href);
=======
                            tg.gettemp(current_asmdata.CurrAsmList,left.resultdef.size,tt_normal,href);
>>>>>>> graemeg/fixes_2_2
=======
                            tg.gettemp(current_asmdata.CurrAsmList,left.resultdef.size,tt_normal,href);
>>>>>>> origin/fixes_2_2
                            cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList,right.location.size,right.location.size,right.location.reference,r);
                            cg.a_loadfpu_reg_ref(current_asmdata.CurrAsmList,right.location.size,left.location.size,r,href);
                            if releaseright then
                              location_freetemp(current_asmdata.CurrAsmList,right.location);
                            releaseright:=true;
<<<<<<< HEAD
<<<<<<< HEAD
                            location_reset_ref(right.location,LOC_REFERENCE,left.location.size,0);
                            right.location.reference:=href;
                            right.resultdef:=left.resultdef;
                          end;
{$endif}
                        hlcg.a_loadmm_ref_reg(current_asmdata.CurrAsmList,
                          right.resultdef,
                          left.resultdef,
=======
=======
>>>>>>> origin/fixes_2_2
                            location_reset(right.location,LOC_REFERENCE,left.location.size);
                            right.location.reference:=href;
                          end;
{$endif}
                        cg.a_loadmm_ref_reg(current_asmdata.CurrAsmList,
                          right.location.size,
                          left.location.size,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                          right.location.reference,
                          left.location.register,mms_movescalar);
                      end;
                    LOC_SUBSETREG,
                    LOC_CSUBSETREG:
                      hlcg.a_load_ref_subsetreg(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location.reference,left.location.sreg);
                    LOC_SUBSETREF,
                    LOC_CSUBSETREF:
{$ifndef cpu64bitalu}
                      if right.location.size in [OS_64,OS_S64] then
                       cg64.a_load64_ref_subsetref(current_asmdata.CurrAsmList,right.location.reference,left.location.sref)
                      else
{$endif not cpu64bitalu}
                       hlcg.a_load_ref_subsetref(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location.reference,left.location.sref);
                    else
                      internalerror(200203284);
                  end;
                end;
{$ifdef SUPPORT_MMX}
              LOC_CMMXREGISTER,
              LOC_MMXREGISTER:
                begin
                  if left.location.loc=LOC_CMMXREGISTER then
                    cg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,OS_M64,OS_M64,right.location.register,left.location.register,nil)
                  else
                    cg.a_loadmm_reg_ref(current_asmdata.CurrAsmList,OS_M64,OS_M64,right.location.register,left.location.reference,nil);
                end;
{$endif SUPPORT_MMX}
              LOC_MMREGISTER,
              LOC_CMMREGISTER:
                begin
                  if left.resultdef.typ=arraydef then
                    begin
                    end
                  else
                    begin
                      case left.location.loc of
                        LOC_CMMREGISTER,
                        LOC_MMREGISTER:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                          hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location.register,left.location.register,mms_movescalar);
                        LOC_REFERENCE,
                        LOC_CREFERENCE:
                          hlcg.a_loadmm_reg_ref(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location.register,left.location.reference,mms_movescalar);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                          cg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,right.location.size,left.location.size,right.location.register,left.location.register,mms_movescalar);
                        LOC_REFERENCE,
                        LOC_CREFERENCE:
                          cg.a_loadmm_reg_ref(current_asmdata.CurrAsmList,right.location.size,left.location.size,right.location.register,left.location.reference,mms_movescalar);
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
                        else
                          internalerror(2009112601);
                      end;
                    end;
                end;
              LOC_REGISTER,
              LOC_CREGISTER :
                begin
<<<<<<< HEAD
{$ifndef cpuhighleveltarget}
{$ifdef cpu64bitalu}
                  if left.location.size in [OS_128,OS_S128] then
                    cg128.a_load128_reg_loc(current_asmdata.CurrAsmList,
                      right.location.register128,left.location)
                  else
{$else cpu64bitalu}
=======
{$ifndef cpu64bitalu}
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
                  { also OS_F64 in case of mmreg -> intreg }
                  if left.location.size in [OS_64,OS_S64,OS_F64] then
                    cg64.a_load64_reg_loc(current_asmdata.CurrAsmList,
                      right.location.register64,left.location)
                  else
{$endif cpu64bitalu}
{$endif not cpuhighleveltarget}
{$ifdef i8086}
                  { prefer a_load_loc_ref, because it supports i8086-specific types
                    that use registerhi (like 6-byte method pointers)
                    (todo: maybe we should add a_load_loc_loc?) }
                  if left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
                    hlcg.a_load_loc_ref(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location,left.location.reference)
                  else
{$endif i8086}
                    hlcg.a_load_reg_loc(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location.register,left.location);
                end;
              LOC_FPUREGISTER,
              LOC_CFPUREGISTER :
                begin
                  { we can't do direct moves between fpu and mm registers }
                  if left.location.loc in [LOC_MMREGISTER,LOC_CMMREGISTER] then
                    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(x86) and not defined(llvm)}
=======
{$ifdef x86}
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
                      if not use_vectorfpu(right.resultdef) then
                        begin
                          { perform size conversion if needed (the mm-code cannot convert an   }
                          { extended into a double/single, since sse doesn't support extended) }
                          tg.gethltemp(current_asmdata.CurrAsmList,left.resultdef,left.resultdef.size,tt_normal,href);
                          cg.a_loadfpu_reg_ref(current_asmdata.CurrAsmList,right.location.size,left.location.size,right.location.register,href);
                          location_reset_ref(right.location,LOC_REFERENCE,left.location.size,0);
                          right.location.reference:=href;
                          right.resultdef:=left.resultdef;
                        end;
{$endif}
                      hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,false);
                      hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,
                          right.resultdef,left.resultdef,
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef x86}
                      if not use_sse(right.resultdef) then
                        begin
                          { perform size conversion if needed (the mm-code cannot convert an   }
                          { extended into a double/single, since sse doesn't support extended) }
                          tg.gettemp(current_asmdata.CurrAsmList,left.resultdef.size,tt_normal,href);
                          cg.a_loadfpu_reg_ref(current_asmdata.CurrAsmList,right.location.size,left.location.size,right.location.register,href);
                          location_reset(right.location,LOC_REFERENCE,left.location.size);
                          right.location.reference:=href;
                        end;
{$endif}
                      location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,false);
                      cg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,
                          right.location.size,left.location.size,
>>>>>>> graemeg/fixes_2_2
                          right.location.register,left.location.register,mms_movescalar);
                    end
                  else
                    hlcg.a_loadfpu_reg_loc(current_asmdata.CurrAsmList,
                        right.resultdef,left.resultdef,
                        right.location.register,left.location);
                end;
              LOC_SUBSETREG,
              LOC_CSUBSETREG:
                begin
                  hlcg.a_load_subsetreg_loc(current_asmdata.CurrAsmList,
                      right.resultdef,left.resultdef,right.location.sreg,left.location);
                end;
              LOC_SUBSETREF,
              LOC_CSUBSETREF:
                begin
{$ifndef cpu64bitalu}
                  if right.location.size in [OS_64,OS_S64] then
                   cg64.a_load64_subsetref_loc(current_asmdata.CurrAsmList,right.location.sref,left.location)
                  else
{$endif not cpu64bitalu}
                  hlcg.a_load_subsetref_loc(current_asmdata.CurrAsmList,
                      right.resultdef,left.resultdef,right.location.sref,left.location);
                end;
              LOC_JUMP :
                begin
                  current_asmdata.getjumplabel(hlabel);
                  hlcg.a_label(current_asmdata.CurrAsmList,right.location.truelabel);
                  if is_pasbool(left.resultdef) then
                    begin
{$ifndef cpu64bitalu}
                      if left.location.size in [OS_64,OS_S64] then
                        cg64.a_load64_const_loc(current_asmdata.CurrAsmList,1,left.location)
                      else
{$endif not cpu64bitalu}
                        hlcg.a_load_const_loc(current_asmdata.CurrAsmList,left.resultdef,1,left.location)
                    end
                  else
                    begin
{$ifndef cpu64bitalu}
                      if left.location.size in [OS_64,OS_S64] then
                        cg64.a_load64_const_loc(current_asmdata.CurrAsmList,-1,left.location)
                      else
{$endif not cpu64bitalu}
                        hlcg.a_load_const_loc(current_asmdata.CurrAsmList,left.resultdef,-1,left.location);
                    end;

                  hlcg.a_jmp_always(current_asmdata.CurrAsmList,hlabel);
                  hlcg.a_label(current_asmdata.CurrAsmList,right.location.falselabel);
{$ifndef cpu64bitalu}
                  if left.location.size in [OS_64,OS_S64] then
                    cg64.a_load64_const_loc(current_asmdata.CurrAsmList,0,left.location)
                  else
{$endif not cpu64bitalu}
                    hlcg.a_load_const_loc(current_asmdata.CurrAsmList,left.resultdef,0,left.location);
                  hlcg.a_label(current_asmdata.CurrAsmList,hlabel);
                end;
{$ifdef cpuflags}
              LOC_FLAGS :
                begin
                  if is_pasbool(left.resultdef) then
                    begin
                      case left.location.loc of
                        LOC_REGISTER,LOC_CREGISTER:
{$ifndef cpu64bitalu}
                          if left.location.size in [OS_S64,OS_64] then
                            begin
                              cg.g_flags2reg(current_asmdata.CurrAsmList,OS_32,right.location.resflags,left.location.register64.reglo);
                              cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                              cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,left.location.register64.reghi);
                            end
                          else
{$endif not cpu64bitalu}
                            begin
                              cg.g_flags2reg(current_asmdata.CurrAsmList,left.location.size,right.location.resflags,left.location.register);
                              cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                            end;
                        LOC_REFERENCE:
                        { i8086 and i386 have hacks in their code generators so that they can
                          deal with 64 bit locations in this parcticular case }
{$if not defined(cpu64bitalu) and not defined(x86)}
                          if left.location.size in [OS_S64,OS_64] then
                            begin
                              r64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                              r64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                              cg.g_flags2reg(current_asmdata.CurrAsmList,OS_32,right.location.resflags,r64.reglo);
                              cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                              cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,r64.reghi);
                              cg64.a_load64_reg_ref(current_asmdata.CurrAsmList,r64,left.location.reference);
                            end
                          else
{$endif not cpu64bitalu}
                            begin
                              cg.g_flags2ref(current_asmdata.CurrAsmList,left.location.size,right.location.resflags,left.location.reference);
                              cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                            end;
                        LOC_SUBSETREG,LOC_SUBSETREF:
                          begin
                            r:=cg.getintregister(current_asmdata.CurrAsmList,left.location.size);
                            cg.g_flags2reg(current_asmdata.CurrAsmList,left.location.size,right.location.resflags,r);
                            cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                            hlcg.a_load_reg_loc(current_asmdata.CurrAsmList,left.resultdef,left.resultdef,r,left.location);
                          end;
                        else
                          internalerror(200203273);
                      end;
                    end
                  else
                    begin
{$ifndef cpu64bitalu}
                      if left.location.size in [OS_S64,OS_64] then
                        begin
                          r64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                          r64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                          cg.g_flags2reg(current_asmdata.CurrAsmList,OS_32,right.location.resflags,r64.reglo);
                          cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                          cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,r64.reghi);
                          cg64.a_op64_reg_reg(current_asmdata.CurrAsmList,OP_NEG,OS_S64,
                            r64,r64);
                          cg64.a_load64_reg_loc(current_asmdata.CurrAsmList,r64,left.location);
                        end
                      else
{$endif not cpu64bitalu}
                        begin
                          r:=cg.getintregister(current_asmdata.CurrAsmList,left.location.size);
                          cg.g_flags2reg(current_asmdata.CurrAsmList,left.location.size,right.location.resflags,r);
                          cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                          cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,left.location.size,r,r);
                          hlcg.a_load_reg_loc(current_asmdata.CurrAsmList,left.resultdef,left.resultdef,r,left.location);
                        end
                    end;
                end;
{$endif cpuflags}
            end;
         end;

        if releaseright then
          location_freetemp(current_asmdata.CurrAsmList,right.location);
      end;


{*****************************************************************************
                           SecondArrayConstruct
*****************************************************************************}

      const
        vtInteger       = 0;
        vtBoolean       = 1;
        vtChar          = 2;
        vtExtended      = 3;
        vtString        = 4;
        vtPointer       = 5;
        vtPChar         = 6;
        vtObject        = 7;
        vtClass         = 8;
        vtWideChar      = 9;
        vtPWideChar     = 10;
        vtAnsiString32  = 11;
        vtCurrency      = 12;
        vtVariant       = 13;
        vtInterface     = 14;
        vtWideString    = 15;
        vtInt64         = 16;
        vtQWord         = 17;
        vtUnicodeString = 18;
        vtAnsiString16  = 19;
        vtAnsiString64  = 20;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


    procedure tcgarrayconstructornode.makearrayref(var ref: treference; eledef: tdef);
      begin
        { do nothing by default }
      end;


    procedure tcgarrayconstructornode.advancearrayoffset(var ref: treference; elesize: asizeint);
      begin
        inc(ref.offset,elesize);
      end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    procedure tcgarrayconstructornode.pass_generate_code;
      var
        hp    : tarrayconstructornode;
        href,
        fref  : treference;
        lt    : tdef;
        paraloc : tcgparalocation;
        varvtypefield,
        varfield : tfieldvarsym;
        vtype : longint;
        eledef: tdef;
        elesize : longint;
        tmpreg  : tregister;
<<<<<<< HEAD
        vaddr : boolean;
        freetemp,
        dovariant: boolean;
=======
        paraloc : tcgparalocation;
        otlabel,
        oflabel : tasmlabel;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
        if is_packed_array(resultdef) then
          internalerror(200608042);
        dovariant:=
          ((nf_forcevaria in flags) or is_variant_array(resultdef)) and
          not(target_info.system in systems_managed_vm);
        eledef:=tarraydef(resultdef).elementdef;
        elesize:=eledef.size;
        if dovariant then
          varvtypefield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VTYPE'))
        else
          varvtypefield:=nil;
        { alignment is filled in by tg.gethltemp below }
        location_reset_ref(location,LOC_CREFERENCE,OS_NO,0);
        fillchar(paraloc,sizeof(paraloc),0);
        { Allocate always a temp, also if no elements are required, to
          be sure that location is valid (PFV) }
        { on the JVM platform, an array can have 0 elements; since the length
          of the array is part of the array itself, make sure we allocate one
          of the proper length to avoid getting unexpected results later --
          allocating a temp of size 0 also forces it to be size 4 on regular
          targets }
        tg.gethltemp(current_asmdata.CurrAsmList,resultdef,(tarraydef(resultdef).highrange+1)*elesize,tt_normal,location.reference);
        href:=location.reference;
        makearrayref(href,eledef);
        { Process nodes in array constructor }
        hp:=self;
        while assigned(hp) do
         begin
           if assigned(hp.left) then
            begin
              freetemp:=true;
              if (hp.left.expectloc=LOC_JUMP) then
                begin
                  otlabel:=current_procinfo.CurrTrueLabel;
                  oflabel:=current_procinfo.CurrFalseLabel;
                  current_asmdata.getjumplabel(current_procinfo.CurrTrueLabel);
                  current_asmdata.getjumplabel(current_procinfo.CurrFalseLabel);
                end;
              secondpass(hp.left);
<<<<<<< HEAD
<<<<<<< HEAD
              if (hp.left.location.loc=LOC_JUMP)<>
                 (hp.left.expectloc=LOC_JUMP) then
                internalerror(2007103101);
              { Move flags and jump in register }
              if hp.left.location.loc in [LOC_FLAGS,LOC_JUMP] then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,hp.left.location,hp.left.resultdef,hp.left.resultdef,false);
=======
=======
>>>>>>> origin/fixes_2_2
              { Move flags and jump in register }
              if hp.left.location.loc in [LOC_FLAGS,LOC_JUMP] then
                location_force_reg(current_asmdata.CurrAsmList,hp.left.location,def_cgsize(hp.left.resultdef),false);

              if (hp.left.location.loc=LOC_JUMP) then
                begin
                  if (hp.left.expectloc<>LOC_JUMP) then
                    internalerror(2007103101);
                  current_procinfo.CurrTrueLabel:=otlabel;
                  current_procinfo.CurrFalseLabel:=oflabel;
                end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

              if dovariant then
               begin
                 { find the correct vtype value }
                 vtype:=$ff;
                 varfield:=nil;
                 vaddr:=false;
                 lt:=hp.left.resultdef;
                 case lt.typ of
                   enumdef,
                   orddef :
                     begin
                       if is_64bit(lt) then
                         begin
                            case torddef(lt).ordtype of
                              scurrency:
                                begin
                                  vtype:=vtCurrency;
                                  varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VCURRENCY'));
                                end;
                              s64bit:
                                begin
                                  vtype:=vtInt64;
                                  varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VINT64'));
                                end;
                              u64bit:
                                begin
                                  vtype:=vtQWord;
                                  varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VQWORD'));
                                end;
                            end;
                            freetemp:=false;
                            vaddr:=true;
                         end
                       else if (lt.typ=enumdef) or
                           is_integer(lt) then
                         begin
                           vtype:=vtInteger;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VINTEGER'));
                         end
                       else
                         if is_boolean(lt) then
                           begin
                             vtype:=vtBoolean;
                             varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VINTEGER'));
                           end
                         else
                           if (lt.typ=orddef) then
                             begin
                               case torddef(lt).ordtype of
                                 uchar:
                                   begin
                                     vtype:=vtChar;
                                     varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VINTEGER'));
                                   end;
                                 uwidechar:
                                   begin
                                     vtype:=vtWideChar;
                                     varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VINTEGER'));
                                   end;
                               end;
                             end;
                     end;
                   floatdef :
                     begin
                       if is_currency(lt) then
                         begin
                           vtype:=vtCurrency;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VCURRENCY'));
                         end
                       else
                         begin
                           vtype:=vtExtended;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VEXTENDED'));
                         end;
                       freetemp:=false;
                       vaddr:=true;
                     end;
                   procvardef,
                   pointerdef :
                     begin
                       if is_pchar(lt) then
                         begin
                           vtype:=vtPChar;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VPCHAR'));
                         end
                       else if is_pwidechar(lt) then
                         begin
                           vtype:=vtPWideChar;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VPWIDECHAR'));
                         end
                       else
                         begin
                           vtype:=vtPointer;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VPOINTER'));
                         end;
                     end;
                   variantdef :
                     begin
                        vtype:=vtVariant;
                        varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VVARIANT'));
                        vaddr:=true;
                        freetemp:=false;
                     end;
                   classrefdef :
                     begin
                       vtype:=vtClass;
                       varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VCLASS'));
                     end;
                   objectdef :
                     if is_interface(lt) then
                       begin
                         vtype:=vtInterface;
                         varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VINTERFACE'));
                       end
                     { vtObject really means a class based on TObject }
                     else if is_class(lt) then
                       begin
                         vtype:=vtObject;
                         varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VOBJECT'));
                       end
                     else
                       internalerror(200505171);
                   stringdef :
                     begin
                       if is_shortstring(lt) then
                        begin
                          vtype:=vtString;
                          varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VSTRING'));
                          vaddr:=true;
                          freetemp:=false;
                        end
                       else
                        if is_ansistring(lt) then
                         begin
                           vtype:=vtAnsiString;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VANSISTRING'));
                           freetemp:=false;
                         end
                       else
                        if is_widestring(lt) then
                         begin
                           vtype:=vtWideString;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VWIDESTRING'));
                           freetemp:=false;
                         end
                       else
                        if is_unicodestring(lt) then
                         begin
                           vtype:=vtUnicodeString;
                           varfield:=tfieldvarsym(search_struct_member_no_helper(trecorddef(eledef),'VUNICODESTRING'));
                           freetemp:=false;
                         end
                       else
                        if is_unicodestring(lt) then
                         begin
                           vtype:=vtUnicodeString;
                           freetemp:=false;
                         end
                       else
                        if is_unicodestring(lt) then
                         begin
                           vtype:=vtUnicodeString;
                           freetemp:=false;
                         end
                       else
                        if is_unicodestring(lt) then
                         begin
                           vtype:=vtUnicodeString;
                           freetemp:=false;
                         end
                       else
                        if is_unicodestring(lt) then
                         begin
                           vtype:=vtUnicodeString;
                           freetemp:=false;
                         end;
                     end;
                 end;
                 if vtype=$ff then
                   internalerror(14357);
                 if not assigned(varfield) then
                   internalerror(2015102901);
                 { write changing field update href to the next element }
                 fref:=href;
                 hlcg.g_set_addr_nonbitpacked_field_ref(current_asmdata.CurrAsmList,trecorddef(eledef),varfield,fref);
                 if vaddr then
                   begin
                     hlcg.location_force_mem(current_asmdata.CurrAsmList,hp.left.location,lt);
                     tmpreg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,cpointerdef.getreusable(lt));
                     hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,hp.left.resultdef,cpointerdef.getreusable(lt),hp.left.location.reference,tmpreg);
                     hlcg.a_load_reg_ref(current_asmdata.CurrAsmList,cpointerdef.getreusable(lt),varfield.vardef,tmpreg,fref);
                   end
                 else
                   hlcg.a_load_loc_ref(current_asmdata.CurrAsmList,hp.left.resultdef,varfield.vardef,hp.left.location,fref);
                 { update href to the vtype field and write it }
                 fref:=href;
                 hlcg.g_set_addr_nonbitpacked_field_ref(current_asmdata.CurrAsmList,trecorddef(eledef),varvtypefield,fref);
                 hlcg.a_load_const_ref(current_asmdata.CurrAsmList,varvtypefield.vardef,vtype,fref);
                 { goto next array element }
                 advancearrayoffset(href,elesize);
               end
              else
              { normal array constructor of the same type }
               begin
                 if is_managed_type(resultdef) then
                   freetemp:=false;
                 case hp.left.location.loc of
                   LOC_MMREGISTER,
                   LOC_CMMREGISTER:
                     hlcg.a_loadmm_reg_ref(current_asmdata.CurrAsmList,hp.left.resultdef,hp.left.resultdef,
                       hp.left.location.register,href,mms_movescalar);
                   LOC_FPUREGISTER,
                   LOC_CFPUREGISTER :
                     hlcg.a_loadfpu_reg_ref(current_asmdata.CurrAsmList,hp.left.resultdef,hp.left.resultdef,hp.left.location.register,href);
                   LOC_REFERENCE,
                   LOC_CREFERENCE :
                     begin
                       if is_shortstring(hp.left.resultdef) then
                         hlcg.g_copyshortstring(current_asmdata.CurrAsmList,hp.left.location.reference,href,
                             Tstringdef(hp.left.resultdef))
                       else
                         hlcg.g_concatcopy(current_asmdata.CurrAsmList,eledef,hp.left.location.reference,href);
                     end;
                   else
                     begin
{$ifndef cpuhighleveltarget}
{$ifdef cpu64bitalu}
                       if hp.left.location.size in [OS_128,OS_S128] then
                         cg128.a_load128_loc_ref(current_asmdata.CurrAsmList,hp.left.location,href)
                       else
{$else cpu64bitalu}
                       if hp.left.location.size in [OS_64,OS_S64] then
                         cg64.a_load64_loc_ref(current_asmdata.CurrAsmList,hp.left.location,href)
                       else
{$endif cpu64bitalu}
{$endif not cpuhighleveltarget}
                         hlcg.a_load_loc_ref(current_asmdata.CurrAsmList,eledef,eledef,hp.left.location,href);
                     end;
                 end;
                 advancearrayoffset(href,elesize);
               end;
              if freetemp then
                location_freetemp(current_asmdata.CurrAsmList,hp.left.location);
            end;
           { load next entry }
           hp:=tarrayconstructornode(hp.right);
         end;
      end;


{*****************************************************************************
                           SecondRTTI
*****************************************************************************}

    procedure tcgrttinode.pass_generate_code;
      begin
        location_reset_ref(location,LOC_CREFERENCE,OS_NO,sizeof(pint));
        case rttidatatype of
          rdt_normal:
            location.reference.symbol:=RTTIWriter.get_rtti_label(rttidef,rttitype);
          rdt_ord2str:
            location.reference.symbol:=RTTIWriter.get_rtti_label_ord2str(rttidef,rttitype);
          rdt_str2ord:
            location.reference.symbol:=RTTIWriter.get_rtti_label_str2ord(rttidef,rttitype);
        end;
      end;



begin
   cloadnode:=tcgloadnode;
   cassignmentnode:=tcgassignmentnode;
   carrayconstructornode:=tcgarrayconstructornode;
   crttinode:=tcgrttinode;
end.
