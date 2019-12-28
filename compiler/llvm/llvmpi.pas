{
    Copyright (c) 2016 by Jonas Maebe

    Information about the current procedure that is being compiled

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
unit llvmpi;

{$i fpcdefs.inc}

interface

    uses
      cclasses,
      aasmbase,
      procinfo,
      cpupi,
      aasmdata,aasmllvm;

    type
      tllvmprocinfo = class(tcpuprocinfo)
       private
        fexceptlabelstack: tfplist;
        flandingpadstack: tfplist;
       public
        constructor create(aparent: tprocinfo); override;
        destructor destroy; override;
        procedure pushexceptlabel(lab: TAsmLabel);
        { returns true if there no more landing pads on the stack }
        function popexceptlabel(lab: TAsmLabel): boolean;
        function CurrExceptLabel: TAsmLabel;
        procedure pushlandingpad(pad: taillvm);
        procedure poppad;
        function currlandingpad: taillvm;
        procedure setup_eh; override;
        procedure finish_eh; override;
        procedure start_eh(list: TAsmList); override;
        procedure end_eh(list: TAsmList); override;
      end;

implementation

    uses
      globtype,globals,verbose,systems,
      symconst,symtype,symdef,symsym,symtable,defutil,llvmdef,
      pass_2,
      parabase,paramgr,
      cgbase,cgutils,cgexcept,tgobj,hlcgobj,llvmbase;

    {*****************************************************************************
                         tllvmexceptionstatehandler
    *****************************************************************************}

    type
      tllvmexceptionstatehandler = class(tcgexceptionstatehandler)
        class procedure get_exception_temps(list: TAsmList; var t: texceptiontemps); override;
        class procedure unget_exception_temps(list: TAsmList; const t: texceptiontemps); override;
        class procedure new_exception(list: TAsmList; const t: texceptiontemps; const exceptframekind: texceptframekind; out exceptstate: texceptionstate); override;
        class procedure emit_except_label(list: TAsmList; exceptframekind: texceptframekind; var exceptionstate: texceptionstate;var exceptiontemps:texceptiontemps); override;
        class procedure end_try_block(list: TAsmList; exceptframekind: texceptframekind; const t: texceptiontemps; var exceptionstate: texceptionstate; endlabel: TAsmLabel); override;
        class procedure cleanupobjectstack(list: TAsmList); override;
        class procedure popaddrstack(list: TAsmList); override;
        class procedure handle_reraise(list: TAsmList; const t: texceptiontemps; const entrystate: texceptionstate; const exceptframekind: texceptframekind); override;
        class procedure begin_catch(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; out exceptlocdef: tdef; out exceptlocreg: tregister); override;
        class procedure end_catch(list: TAsmList); override;
        class procedure catch_all_start(list: TAsmList); override;
        class procedure catch_all_end(list: TAsmList); override;
       protected
        class procedure begin_catch_internal(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; add_catch: boolean; out exceptlocdef: tdef; out exceptlocreg: tregister);
        class procedure catch_all_start_internal(list: TAsmList; add_catch: boolean);
      end;


      class procedure tllvmexceptionstatehandler.get_exception_temps(list: TAsmList; var t: texceptiontemps);
        begin
          tg.gethltemp(list,ossinttype,ossinttype.size,tt_persistent,t.reasonbuf);
        end;


      class procedure tllvmexceptionstatehandler.unget_exception_temps(list: TAsmList; const t: texceptiontemps);
        begin
          tg.ungettemp(list,t.reasonbuf);
          tllvmprocinfo(current_procinfo).poppad;
        end;


      class procedure tllvmexceptionstatehandler.new_exception(list: TAsmList; const t: texceptiontemps; const exceptframekind: texceptframekind; out exceptstate: texceptionstate);
        var
          reg: tregister;
        begin
          exceptstate.oldflowcontrol:=flowcontrol;
          if exceptframekind<>tek_except then
            current_asmdata.getjumplabel(exceptstate.finallycodelabel)
          else
            exceptstate.finallycodelabel:=nil;
          { all calls inside the exception block have to be invokes instead,
            which refer to the exception label:
              exceptionlabel:
                %reg = landingpad ..
                <exception handling code>
          }
          current_asmdata.getjumplabel(exceptstate.exceptionlabel);
          { for consistency checking when popping }
          tllvmprocinfo(current_procinfo).pushexceptlabel(exceptstate.exceptionlabel);
          flowcontrol:=[fc_inflowcontrol,fc_catching_exceptions];
          { the reasonbuf is set to 1 by the generic code if we got in
            the exception block by catching an exception -> do the same here, so
            we can share that generic code; llvm will optimise it away. The
            reasonbuf is later also used for break/continue/... }
          reg:=hlcg.getintregister(list,ossinttype);
          hlcg.a_load_const_reg(list,ossinttype,1,reg);
          hlcg.g_exception_reason_save(list,ossinttype,ossinttype,reg,t.reasonbuf);
          { There can only be a landingpad if there were any invokes in the try-block,
            as otherwise we get an error; we can also generate exceptions from
            invalid memory accesses and the like, but LLVM cannot model that
            --
            We cheat for now by adding an invoke to a dummy routine at the start and at
            the end of the try-block. That will not magically fix the state
            of all variables when the exception gets caught though. }
          hlcg.g_call_system_proc(list,'FPC_DUMMYPOTENTIALRAISE',[],nil).resetiftemp;
        end;


      class procedure tllvmexceptionstatehandler.emit_except_label(list: TAsmList; exceptframekind: texceptframekind; var exceptionstate: texceptionstate;var exceptiontemps:texceptiontemps);
        var
          reg: tregister;
          landingpad: taillvm;
          landingpaddef: trecorddef;
        begin
          hlcg.g_unreachable(list);
          hlcg.a_label(list,exceptionstate.exceptionlabel);
          { use packrecords 1 because we don't want padding (LLVM 4.0+ requires
            exactly two fields in this struct) }
          landingpaddef:=llvmgettemprecorddef([voidpointertype,u32inttype],
            1,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          reg:=hlcg.getregisterfordef(list,landingpaddef);
          landingpad:=taillvm.landingpad(reg,landingpaddef,{clause}nil);
          list.concat(landingpad);
          if exceptframekind<>tek_except then
            begin
              if not assigned(exceptionstate.finallycodelabel) then
                internalerror(2018111102);
              if use_cleanup(exceptframekind) then
                landingpad.landingpad_add_clause(la_cleanup, nil, nil)
              else
                landingpad.landingpad_add_clause(la_catch, voidpointertype, nil);
              hlcg.a_label(list,exceptionstate.finallycodelabel);
              exceptionstate.finallycodelabel:=nil;
            end;
          { consistency check }
          tllvmprocinfo(current_procinfo).popexceptlabel(exceptionstate.exceptionlabel);
          tllvmprocinfo(current_procinfo).pushlandingpad(landingpad);
        end;


      class procedure tllvmexceptionstatehandler.end_try_block(list: TAsmList; exceptframekind: texceptframekind; const t: texceptiontemps; var exceptionstate: texceptionstate; endlabel: TAsmLabel);
        var
          reg: tregister;
        begin
          { llvm does not allow creating a landing pad if there are no invokes in
            the try block -> create a call to a dummy routine that cannot be
            analysed by llvm and that supposedly may raise an exception. Has to
            be combined with marking stores inside try blocks as volatile and the
            loads afterwards as well in order to guarantee correct optimizations
            in case an exception gets triggered inside a try-block though }
          hlcg.g_call_system_proc(list,'FPC_DUMMYPOTENTIALRAISE',[],nil).resetiftemp;

          { record that no exception happened in the reason buf }
          reg:=hlcg.getintregister(list,ossinttype);
          hlcg.a_load_const_reg(list,ossinttype,0,reg);
          hlcg.g_exception_reason_save(list,ossinttype,ossinttype,reg,t.reasonbuf);
          inherited;
          if exceptframekind=tek_except then
            hlcg.a_jmp_always(list,endlabel);
        end;

      class procedure tllvmexceptionstatehandler.cleanupobjectstack(list: TAsmList);
        var
          landingpad: taillvm;
        begin
          { if not a single catch block added -> catch all }
          landingpad:=tllvmprocinfo(current_procinfo).currlandingpad;
          if assigned(landingpad) and
             not assigned(landingpad.oper[2]^.ai) then
            begin
              landingpad.landingpad_add_clause(la_catch,voidpointertype,nil);
            end;
        end;

      class procedure tllvmexceptionstatehandler.popaddrstack(list: TAsmList);
        begin
          // nothing
        end;


      class procedure tllvmexceptionstatehandler.handle_reraise(list: TAsmList; const t: texceptiontemps; const entrystate: texceptionstate; const exceptframekind: texceptframekind);
        var
          landingpad: taillvm;
          landingpadres: tregister;
          landingpadresdef: tdef;
        begin
          { We use resume to propagate the exception to an outer function frame, and call
            reraise in case we are nested in another exception frame in the current function
            (because then we will emit an invoke which will tie this re-raise to that other
             exception frame; that is impossible to do with a resume instruction).

            Furthermore, the resume opcode only works for landingpads with a cleanup clause,
            which we only generate for outer implicitfinally frames }
          if not(fc_catching_exceptions in flowcontrol) and
             use_cleanup(exceptframekind) then
            begin
              { resume <result from catchpad> }
              landingpad:=tllvmprocinfo(current_procinfo).currlandingpad;
              landingpadres:=landingpad.oper[0]^.reg;
              landingpadresdef:=landingpad.oper[1]^.def;
              list.concat(taillvm.op_size_reg(la_resume,landingpadresdef,landingpadres));
            end
          else
            begin
              { Need a begin_catch so that the reraise will know what exception to throw.
                Don't need to add a "catch all" to the landing pad, as it contains one.
                We want to rethrow whatever exception was caught rather than guarantee
                that all possible kinds of exceptions get caught. }
              catch_all_start_internal(list,false);
              hlcg.g_call_system_proc(list,'fpc_reraise',[],nil).resetiftemp;
            end;
        end;


      class procedure tllvmexceptionstatehandler.begin_catch(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; out exceptlocdef: tdef; out exceptlocreg: tregister);
        begin
          begin_catch_internal(list,excepttype,nextonlabel,true,exceptlocdef,exceptlocreg);
        end;


      class procedure tllvmexceptionstatehandler.end_catch(list: TAsmList);
        begin
          hlcg.g_call_system_proc(list,'fpc_psabi_end_catch',[],nil).resetiftemp;
          inherited;
        end;


      class procedure tllvmexceptionstatehandler.catch_all_start(list: TAsmList);
        begin
          catch_all_start_internal(list,true);
        end;


      class procedure tllvmexceptionstatehandler.catch_all_end(list: TAsmList);
        begin
          hlcg.g_call_system_proc(list,'fpc_psabi_end_catch',[],nil).resetiftemp;
        end;


      class procedure tllvmexceptionstatehandler.begin_catch_internal(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; add_catch: boolean; out exceptlocdef: tdef; out exceptlocreg: tregister);
        var
          catchstartlab: tasmlabel;
          landingpad: taillvm;
          begincatchres,
          typeidres,
          paraloc1: tcgpara;
          pd: tprocdef;
          landingpadstructdef,
          landingpadtypeiddef: tdef;
          rttisym: TAsmSymbol;
          rttidef: tdef;
          rttiref: treference;
          wrappedexception,
          exceptiontypeidreg,
          landingpadres: tregister;
          exceptloc: tlocation;
          indirect: boolean;
          otherunit: boolean;
        begin
          paraloc1.init;
          landingpad:=tllvmprocinfo(current_procinfo).currlandingpad;
          rttidef:=nil;
          rttisym:=nil;
          if add_catch then
            begin
              if assigned(excepttype) then
                begin
                  otherunit:=findunitsymtable(excepttype.owner).moduleid<>findunitsymtable(current_procinfo.procdef.owner).moduleid;
                  indirect:=(tf_supports_packages in target_info.flags) and
                          (target_info.system in systems_indirect_var_imports) and
                          (cs_imported_data in current_settings.localswitches) and
                          otherunit;
                  { add "catch exceptiontype" clause to the landing pad }
                  rttidef:=cpointerdef.getreusable(excepttype.vmt_def);
                  rttisym:=current_asmdata.RefAsmSymbol(excepttype.vmt_mangledname, AT_DATA, indirect);
                  landingpad.landingpad_add_clause(la_catch,rttidef,rttisym);
                end
              else
                begin
                  landingpad.landingpad_add_clause(la_catch,voidpointertype,nil);
                end;
            end;
          { pascal_exception := FPC_psabi_begin_catch(wrappedExceptionObject) where
            wrappedExceptionObject is the exception returned by the landingpad }
          landingpadres:=landingpad.oper[0]^.reg;
          landingpadstructdef:=landingpad.oper[1]^.def;
          { check if the exception is handled by this node }
          if assigned(excepttype) then
            begin
              landingpadtypeiddef:=tfieldvarsym(trecorddef(landingpadstructdef).symtable.symlist[1]).vardef;
              exceptiontypeidreg:=hlcg.getaddressregister(list,landingpadtypeiddef);
              pd:=search_system_proc('llvm_eh_typeid_for');
              paramanager.getcgtempparaloc(list,pd,1,paraloc1);
              reference_reset_symbol(rttiref,rttisym,0,rttidef.alignment,[]);
              rttiref.refaddr:=addr_full;
              hlcg.a_load_ref_cgpara(list,cpointerdef.getreusable(rttidef),rttiref,paraloc1);
              typeidres:=hlcg.g_call_system_proc(list,pd,[@paraloc1],nil);
              location_reset(exceptloc, LOC_REGISTER, def_cgsize(landingpadtypeiddef));
              exceptloc.register:=hlcg.getintregister(list,landingpadtypeiddef);
              hlcg.gen_load_cgpara_loc(list, landingpadtypeiddef, typeidres, exceptloc, true);
              list.concat(taillvm.extract(la_extractvalue,exceptiontypeidreg,landingpadstructdef,landingpadres,1));
              current_asmdata.getjumplabel(catchstartlab);
              hlcg.a_cmp_reg_loc_label(list,typeidres.Def,OC_EQ,exceptiontypeidreg,exceptloc,catchstartlab);
              hlcg.a_jmp_always(list,nextonlabel);
              hlcg.a_label(list,catchstartlab);
              typeidres.resetiftemp;
            end;

          wrappedexception:=hlcg.getaddressregister(list,voidpointertype);
          list.concat(taillvm.extract(la_extractvalue,wrappedexception,landingpadstructdef,landingpadres,0));

          pd:=search_system_proc('fpc_psabi_begin_catch');
          paramanager.getcgtempparaloc(list, pd, 1, paraloc1);
          hlcg.a_load_reg_cgpara(list,voidpointertype,wrappedexception,paraloc1);
          begincatchres:=hlcg.g_call_system_proc(list,pd,[@paraloc1],nil);
          location_reset(exceptloc, LOC_REGISTER, def_cgsize(begincatchres.def));
          exceptloc.register:=hlcg.getaddressregister(list, begincatchres.def);
          hlcg.gen_load_cgpara_loc(list, begincatchres.def, begincatchres, exceptloc, true);

          begincatchres.resetiftemp;
          paraloc1.done;

          exceptlocdef:=begincatchres.def;
          exceptlocreg:=exceptloc.register;
        end;


      class procedure tllvmexceptionstatehandler.catch_all_start_internal(list: TAsmList; add_catch: boolean);
        var
          exceptlocdef: tdef;
          exceptlocreg: tregister;
        begin
          begin_catch_internal(list,nil,nil,add_catch,exceptlocdef,exceptlocreg);
        end;



{*****************************************************************************
                     tllvmprocinfo
*****************************************************************************}

    constructor tllvmprocinfo.create(aparent: tprocinfo);
      begin
        inherited;
        fexceptlabelstack:=tfplist.create;
        flandingpadstack:=tfplist.create;
      end;

    destructor tllvmprocinfo.destroy;
      begin
        if fexceptlabelstack.Count<>0 then
          Internalerror(2016121301);
        fexceptlabelstack.free;
        if flandingpadstack.Count<>0 then
          internalerror(2018051901);
        flandingpadstack.free;
        inherited;
      end;


    procedure tllvmprocinfo.pushexceptlabel(lab: TAsmLabel);
      begin
        fexceptlabelstack.add(lab);
      end;


    function tllvmprocinfo.popexceptlabel(lab: TAsmLabel): boolean;
      begin
        if CurrExceptLabel<>lab then
          internalerror(2016121302);
        fexceptlabelstack.count:=fexceptlabelstack.count-1;
        result:=fexceptlabelstack.count=0;
      end;


    function tllvmprocinfo.CurrExceptLabel: TAsmLabel; inline;
      begin
        result:=TAsmLabel(fexceptlabelstack.last);
        if not assigned(result) then
          internalerror(2016121703);
      end;


    procedure tllvmprocinfo.pushlandingpad(pad: taillvm);
      begin
        flandingpadstack.add(pad);
      end;

    procedure tllvmprocinfo.poppad;
      begin
        if flandingpadstack.Count=0 then
          internalerror(2018051902);
        flandingpadstack.Count:=flandingpadstack.Count-1;
      end;


    function tllvmprocinfo.currlandingpad: taillvm;
      begin
        if flandingpadstack.Count=0 then
          internalerror(2018051903);
        result:=taillvm(flandingpadstack.last);
      end;


    procedure tllvmprocinfo.setup_eh;
      begin
        if po_assembler in procdef.procoptions then
          inherited
        else
          begin
            cexceptionstatehandler:=tllvmexceptionstatehandler;
          end;
      end;


    procedure tllvmprocinfo.finish_eh;
      begin
        if po_assembler in procdef.procoptions then
          inherited;
      end;


    procedure tllvmprocinfo.start_eh(list: TAsmList);
      begin
        if po_assembler in procdef.procoptions then
          inherited;
      end;


    procedure tllvmprocinfo.end_eh(list: TAsmList);
      begin
        if po_assembler in procdef.procoptions then
          inherited;
      end;


begin
  if not assigned(cprocinfo) then
    begin
      writeln('Internalerror 2018052005');
      halt(1);
    end;
  cprocinfo:=tllvmprocinfo;
end.

