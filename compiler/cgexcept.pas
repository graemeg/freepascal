{
    Copyright (c) 2017-2019 by Jonas Maebe, member of the
    Free Pascal Compiler development team

    Base class for exception handling support (setjump/longjump-based)

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
unit cgexcept;

{$i fpcdefs.inc}

  interface

    uses
      globtype,
      aasmbase, aasmdata,
      symtype,symdef,
      cgbase,cgutils,pass_2;

    type
      { Utility class for exception handling state management that is used
        by tryexcept/tryfinally/on nodes (in a separate class so it can both
        be shared and overridden)

        Never instantiated. }
      tcgexceptionstatehandler = class
       type
        texceptiontemps=record
          jmpbuf,
          envbuf,
          reasonbuf  : treference;
          { when using dwarf based eh handling, the landing pads get the unwind info passed, it is
            stored in the given register so it can be passed to unwind_resume }
          unwind_info : TRegister;
        end;

        texceptionstate = record
          exceptionlabel: TAsmLabel;
          oldflowcontrol,
          newflowcontrol: tflowcontrol;
          finallycodelabel  : TAsmLabel;
        end;

        texceptframekind = (tek_except, tek_implicitfinally, tek_normalfinally);

        class procedure get_exception_temps(list:TAsmList;var t:texceptiontemps); virtual;
        class procedure unget_exception_temps(list:TAsmList;const t:texceptiontemps); virtual;
        class procedure new_exception(list:TAsmList;const t:texceptiontemps; const exceptframekind: texceptframekind; out exceptstate: texceptionstate); virtual;
        { start of "except/finally" block }
        class procedure emit_except_label(list: TAsmList; exceptframekind: texceptframekind; var exceptstate: texceptionstate;var exceptiontemps:texceptiontemps); virtual;
        { end of a try-block, label comes after the end of try/except or
          try/finally }
        class procedure end_try_block(list: TAsmList; exceptframekind: texceptframekind; const t: texceptiontemps; var exceptionstate: texceptionstate; endlabel: TAsmLabel); virtual;
        class procedure free_exception(list: TAsmList; const t: texceptiontemps; const s: texceptionstate; a: aint; endexceptlabel: tasmlabel; onlyfree:boolean); virtual;
        class procedure handle_nested_exception(list:TAsmList;var t:texceptiontemps;var entrystate: texceptionstate); virtual;
        class procedure handle_reraise(list:TAsmList;const t:texceptiontemps;const entrystate: texceptionstate; const exceptframekind: texceptframekind); virtual;
        { start of an "on" (catch) block }
        class procedure begin_catch(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; out exceptlocdef: tdef; out exceptlocreg: tregister); virtual;
        { end of an "on" (catch) block }
        class procedure end_catch(list: TAsmList); virtual;
        { called for a catch all exception }
        class procedure catch_all_start(list: TAsmList); virtual;
        { called after the catch all exception has been started with new_exception }
        class procedure catch_all_add(list: TAsmList); virtual;
        class procedure catch_all_end(list: TAsmList); virtual;
        class procedure cleanupobjectstack(list: TAsmList); virtual;
        class procedure popaddrstack(list: TAsmList); virtual;
        class function use_cleanup(const exceptframekind: texceptframekind): boolean;
      end;
      tcgexceptionstatehandlerclass = class of tcgexceptionstatehandler;

    var
      cexceptionstatehandler: tcgexceptionstatehandlerclass = tcgexceptionstatehandler;

  implementation

    uses
      globals,
      systems,
      fmodule,
      aasmtai,
      symconst,symtable,defutil,
      parabase,paramgr,
      procinfo,
      tgobj,
      hlcgobj;

{*****************************************************************************
                     tcgexceptionstatehandler
*****************************************************************************}

    class function tcgexceptionstatehandler.use_cleanup(const exceptframekind: texceptframekind): boolean;
      begin
        { in case of an exception caught by the implicit exception frame of
          a safecall routine, this is not a cleanup frame but one that
          catches the exception and returns a value from the function }
        result:=
          (exceptframekind=tek_implicitfinally) and
          not((tf_safecall_exceptions in target_info.flags) and
             (current_procinfo.procdef.proccalloption=pocall_safecall));
      end;

    {  Allocate the buffers for exception management and setjmp environment.
       Return a pointer to these buffers, send them to the utility routine
       so they are registered, and then call setjmp.

       Then compare the result of setjmp with 0, and if not equal
       to zero, then jump to exceptlabel.

       Also store the result of setjmp to a temporary space by calling g_save_exception_reason

       It is to note that this routine may be called *after* the stackframe of a
       routine has been called, therefore on machines where the stack cannot
       be modified, all temps should be allocated on the heap instead of the
       stack. }


    class procedure tcgexceptionstatehandler.get_exception_temps(list:TAsmList;var t:texceptiontemps);
     begin
        tg.gethltemp(list,rec_exceptaddr,rec_exceptaddr.size,tt_persistent,t.envbuf);
        tg.gethltemp(list,rec_jmp_buf,rec_jmp_buf.size,tt_persistent,t.jmpbuf);
        tg.gethltemp(list,ossinttype,ossinttype.size,tt_persistent,t.reasonbuf);
      end;


    class procedure tcgexceptionstatehandler.unget_exception_temps(list:TAsmList;const t:texceptiontemps);
      begin
        tg.Ungettemp(list,t.jmpbuf);
        tg.ungettemp(list,t.envbuf);
        tg.ungettemp(list,t.reasonbuf);
      end;


    class procedure tcgexceptionstatehandler.new_exception(list:TAsmList;const t:texceptiontemps; const exceptframekind: texceptframekind; out exceptstate: texceptionstate);
      var
        paraloc1, paraloc2, paraloc3, pushexceptres, setjmpres: tcgpara;
        pd: tprocdef;
        tmpresloc: tlocation;
      begin
        current_asmdata.getjumplabel(exceptstate.exceptionlabel);
        exceptstate.oldflowcontrol:=flowcontrol;
        exceptstate.finallycodelabel:=nil;;

        paraloc1.init;
        paraloc2.init;
        paraloc3.init;

        { fpc_pushexceptaddr(exceptionframetype, setjmp_buffer, exception_address_chain_entry) }
        pd:=search_system_proc('fpc_pushexceptaddr');
        paramanager.getcgtempparaloc(list,pd,1,paraloc1);
        paramanager.getcgtempparaloc(list,pd,2,paraloc2);
        paramanager.getcgtempparaloc(list,pd,3,paraloc3);
        if pd.is_pushleftright then
          begin
            { type of exceptionframe }
            hlcg.a_load_const_cgpara(list,paraloc1.def,1,paraloc1);
            { setjmp buffer }
            hlcg.a_loadaddr_ref_cgpara(list,rec_jmp_buf,t.jmpbuf,paraloc2);
            { exception address chain entry }
            hlcg.a_loadaddr_ref_cgpara(list,rec_exceptaddr,t.envbuf,paraloc3);
          end
        else
          begin
            hlcg.a_loadaddr_ref_cgpara(list,rec_exceptaddr,t.envbuf,paraloc3);
            hlcg.a_loadaddr_ref_cgpara(list,rec_jmp_buf,t.jmpbuf,paraloc2);
            hlcg.a_load_const_cgpara(list,paraloc1.def,1,paraloc1);
          end;
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        { perform the fpc_pushexceptaddr call }
        pushexceptres:=hlcg.g_call_system_proc(list,pd,[@paraloc1,@paraloc2,@paraloc3],nil);
        paraloc1.done;
        paraloc2.done;
        paraloc3.done;

        { get the result }
        location_reset(tmpresloc,LOC_REGISTER,def_cgsize(pushexceptres.def));
        tmpresloc.register:=hlcg.getaddressregister(list,pushexceptres.def);
        hlcg.gen_load_cgpara_loc(list,pushexceptres.def,pushexceptres,tmpresloc,true);
        pushexceptres.resetiftemp;

        { fpc_setjmp(result_of_pushexceptaddr_call) }
        pd:=search_system_proc('fpc_setjmp');
        paramanager.getcgtempparaloc(list,pd,1,paraloc1);

        hlcg.a_load_reg_cgpara(list,pushexceptres.def,tmpresloc.register,paraloc1);
        paramanager.freecgpara(list,paraloc1);
        { perform the fpc_setjmp call }
        setjmpres:=hlcg.g_call_system_proc(list,pd,[@paraloc1],nil);
        paraloc1.done;
        location_reset(tmpresloc,LOC_REGISTER,def_cgsize(setjmpres.def));
        tmpresloc.register:=hlcg.getintregister(list,setjmpres.def);
        hlcg.gen_load_cgpara_loc(list,setjmpres.def,setjmpres,tmpresloc,true);
        hlcg.g_exception_reason_save(list,setjmpres.def,ossinttype,tmpresloc.register,t.reasonbuf);
        { if we get 1 here in the function result register, it means that we
          longjmp'd back here }
        hlcg.a_cmp_const_reg_label(list,setjmpres.def,OC_NE,0,tmpresloc.register,exceptstate.exceptionlabel);
        setjmpres.resetiftemp;

        flowcontrol:=[fc_inflowcontrol,fc_catching_exceptions];
     end;


    class procedure tcgexceptionstatehandler.emit_except_label(list: TAsmList; exceptframekind: texceptframekind; var exceptstate: texceptionstate;var exceptiontemps:texceptiontemps);
      begin
        hlcg.a_label(list,exceptstate.exceptionlabel);
      end;


    class procedure tcgexceptionstatehandler.end_try_block(list: TAsmList; exceptframekind: texceptframekind; const t: texceptiontemps; var exceptionstate: texceptionstate; endlabel: TAsmLabel);
      begin
         exceptionstate.newflowcontrol:=flowcontrol;
         flowcontrol:=exceptionstate.oldflowcontrol;
      end;


    class procedure tcgexceptionstatehandler.free_exception(list: TAsmList; const t: texceptiontemps; const s: texceptionstate; a: aint; endexceptlabel: tasmlabel; onlyfree: boolean);
      var
        reasonreg: tregister;
      begin
         popaddrstack(list);
         if not onlyfree then
          begin
            reasonreg:=hlcg.getintregister(list,osuinttype);
            hlcg.g_exception_reason_load(list,osuinttype,osuinttype,t.reasonbuf,reasonreg);
            hlcg.a_cmp_const_reg_label(list,osuinttype,OC_EQ,a,reasonreg,endexceptlabel);
          end;
      end;


    { does the necessary things to clean up the object stack }
    { in the except block                                    }
    class procedure tcgexceptionstatehandler.cleanupobjectstack(list: TAsmList);
      begin
         hlcg.g_call_system_proc(list,'fpc_doneexception',[],nil).resetiftemp;
      end;


    { generates code to be executed when another exeception is raised while
      control is inside except block }
    class procedure tcgexceptionstatehandler.handle_nested_exception(list:TAsmList;var t:texceptiontemps;var entrystate: texceptionstate);
      var
         exitlabel: tasmlabel;
      begin
         current_asmdata.getjumplabel(exitlabel);
         { add an catch all action clause, at least psabieh needs this }
         catch_all_add(list);
         end_try_block(list,tek_except,t,entrystate,exitlabel);
         emit_except_label(list,tek_except,entrystate,t);
         { don't generate line info for internal cleanup }
         list.concat(tai_marker.create(mark_NoLineInfoStart));
         free_exception(list,t,entrystate,0,exitlabel,false);
         { we don't need to save/restore registers here because reraise never }
         { returns                                                            }
         hlcg.g_call_system_proc(list,'fpc_raise_nested',[],nil).resetiftemp;
         hlcg.a_label(list,exitlabel);
         cleanupobjectstack(list);
      end;


    class procedure tcgexceptionstatehandler.handle_reraise(list: TAsmList; const t: texceptiontemps; const entrystate: texceptionstate; const exceptframekind: texceptframekind);
      begin
        hlcg.g_call_system_proc(list,'fpc_reraise',[],nil).resetiftemp;
      end;


    class procedure tcgexceptionstatehandler.begin_catch(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; out exceptlocdef: tdef; out exceptlocreg: tregister);
      var
        pd: tprocdef;
        href2: treference;
        fpc_catches_res,
        paraloc1: tcgpara;
        exceptloc: tlocation;
        indirect: boolean;
        otherunit: boolean;
      begin
        paraloc1.init;
        otherunit:=findunitsymtable(excepttype.owner).moduleid<>findunitsymtable(current_procinfo.procdef.owner).moduleid;
        indirect:=(tf_supports_packages in target_info.flags) and
                    (target_info.system in systems_indirect_var_imports) and
                    (cs_imported_data in current_settings.localswitches) and
                    otherunit;

        { send the vmt parameter }
        pd:=search_system_proc('fpc_catches');
        reference_reset_symbol(href2, current_asmdata.RefAsmSymbol(excepttype.vmt_mangledname, AT_DATA, indirect), 0, sizeof(pint), []);
        if otherunit then
          current_module.add_extern_asmsym(excepttype.vmt_mangledname, AB_EXTERNAL, AT_DATA);
        paramanager.getcgtempparaloc(list, pd, 1, paraloc1);
        hlcg.a_loadaddr_ref_cgpara(list, excepttype.vmt_def, href2, paraloc1);
        paramanager.freecgpara(list, paraloc1);
        fpc_catches_res:=hlcg.g_call_system_proc(list, pd, [@paraloc1], nil);
        location_reset(exceptloc, LOC_REGISTER, def_cgsize(fpc_catches_res.def));
        exceptloc.register:=hlcg.getaddressregister(list, fpc_catches_res.def);
        hlcg.gen_load_cgpara_loc(list, fpc_catches_res.def, fpc_catches_res, exceptloc, true);

        { is it this catch? No. go to next onlabel }
        hlcg.a_cmp_const_reg_label(list, fpc_catches_res.def, OC_EQ, 0, exceptloc.register, nextonlabel);

        paraloc1.done;

        exceptlocdef:=fpc_catches_res.def;
        exceptlocreg:=exceptloc.register;
      end;


    class procedure tcgexceptionstatehandler.end_catch(list: TAsmList);
      begin
        { nothing to do by default }
      end;


    class procedure tcgexceptionstatehandler.catch_all_start(list: TAsmList);
      begin
        { nothing to do by default }
      end;


    class procedure tcgexceptionstatehandler.catch_all_add(list: TAsmList);
      begin
        { nothing to do by default }
      end;


    class procedure tcgexceptionstatehandler.catch_all_end(list: TAsmList);
      begin
        { nothing to do by default }
      end;

    class procedure tcgexceptionstatehandler.popaddrstack(list: TAsmList);
      begin
        hlcg.g_call_system_proc(list,'fpc_popaddrstack',[],nil).resetiftemp;
      end;


end.

