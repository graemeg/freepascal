{
    Copyright (c) 1998-2002 by Florian Klaempfl

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
unit psabiehpi;

{ $define debug_eh}

{$i fpcdefs.inc}

  interface

    uses
      { common }
      cclasses,
      { global }
      globtype,
      { symtable }
      symconst,symtype,symdef,symsym,
      node,nutils,
      { aasm }
      cpubase,cgbase,cgutils,
      aasmbase,aasmdata,aasmtai,
      psub;

    type
       TPSABIEHAction = class
         landingpad : TAsmLabel;
         actiontablelabel : TAsmLabel;
         actionlist : TAsmList;
         first : boolean;
         constructor Create(pad : TAsmLabel);
         destructor Destroy; override;
         function AddAction(p: tobjectdef): LongInt;
       end;

       { This object gives information on the current routine being
         compiled.
       }
       tpsabiehprocinfo = class(tcgprocinfo)
         { set if the procedure needs exception tables because it
           has exception generating nodes }
         CreateExceptionTable: Boolean;

         { if a procedure needs exception tables, this is the outmost landing pad
           with "no action", covering everything not covered by other landing pads
           since a procedure which has one landing pad need to be covered completely by landing pads }
         OutmostLandingPad: TPSABIEHAction;

         { This is a "no action" action for re-use, normally equal to OutmostLandingPad }
         NoAction: TPSABIEHAction;

         { label to language specific data }
         LSDALabel : TAsmLabel;
         callsite_table_data,
         action_table_data,
         gcc_except_table_data : TAsmList;
         typefilterlistlabel,typefilterlistlabelref,
         callsitetablestart,callsitetableend,
         { first label which must be inserted into the entry code }
         entrycallsitestart,
         callsitelaststart : TAsmLabel;
         typefilterlist,
         landingpadstack,
         actionstack : tfplist;
         CurrentCallSiteNumber : Longint;

         destructor destroy; override;

         { PSABIEH stuff }
         procedure PushAction(action: TPSABIEHAction);
         function CurrentAction: TPSABIEHAction;inline;
         function PopAction(action: TPSABIEHAction): boolean;
         function FinalizeAndPopAction(action: TPSABIEHAction): boolean;
         { a landing pad is also an action, however, when the landing pad is popped from the stack
           the area covered by this landing pad ends, i.e. it is popped at the beginning of the finally/except clause,
           the action above is popped at the end of the finally/except clause, so if on clauses add new types, they
           are added to CurrentAction }
         procedure PushLandingPad(action: TPSABIEHAction);
         function CurrentLandingPad: TPSABIEHAction;inline;
         function PopLandingPad(action: TPSABIEHAction): boolean;
         procedure CreateNewPSABIEHCallsite(list: TAsmList);
         { adds a new type to the type filter list and returns its index
           be aware, that this method can also handle catch all filters so it
           is valid to pass nil }
         function AddTypeFilter(p: tobjectdef): Longint;
         procedure set_eh_info; override;
         procedure setup_eh; override;
         procedure finish_eh; override;
         procedure start_eh(list : TAsmList); override;
         procedure end_eh(list : TAsmList); override;

         function find_exception_handling(var n: tnode; para: pointer): foreachnoderesult; virtual;
       end;

implementation

    uses
      cutils,
      verbose,
      systems,
      dwarfbase,
      cfidwarf,
      globals,
      procinfo,
      symtable,
      defutil,
      tgobj,
      cgobj,cgexcept,
      parabase,paramgr,
      hlcgobj,
      pass_2
{$ifdef i386}
      ,aasmcpu
{$endif i386}
      ;


    type
       { Utility class for exception handling state management that is used
         by tryexcept/tryfinally/on nodes (in a separate class so it can both
         be shared and overridden)

         Never instantiated. }
       tpsabiehexceptionstatehandler = class(tcgexceptionstatehandler)
       protected
         class procedure begin_catch_internal(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; add_catch: boolean; out exceptlocdef: tdef; out
           exceptlocreg: tregister);
         class procedure catch_all_start_internal(list: TAsmList; add_catch: boolean);
       public
         class procedure get_exception_temps(list:TAsmList;var t:texceptiontemps); override;
         class procedure unget_exception_temps(list:TAsmList;const t:texceptiontemps); override;
         class procedure new_exception(list:TAsmList;const t:texceptiontemps; const exceptframekind: texceptframekind; out exceptstate: texceptionstate); override;
         { start of "except/finally" block }
         class procedure emit_except_label(list: TAsmList; exceptframekind: texceptframekind; var exceptionstate: texceptionstate;var exceptiontemps:texceptiontemps); override;
         { end of a try-block, label comes after the end of try/except or
           try/finally }
         class procedure end_try_block(list: TAsmList; exceptframekind: texceptframekind; const t: texceptiontemps; var exceptionstate: texceptionstate; endlabel: TAsmLabel); override;
         class procedure free_exception(list: TAsmList; const t: texceptiontemps; const s: texceptionstate; a: aint; endexceptlabel: tasmlabel; onlyfree:boolean); override;
         class procedure handle_reraise(list:TAsmList;const t:texceptiontemps;const entrystate: texceptionstate; const exceptframekind: texceptframekind); override;
         { start of an "on" (catch) block }
         class procedure begin_catch(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; out exceptlocdef: tdef; out exceptlocreg: tregister); override;
         { end of an "on" (catch) block }
         class procedure end_catch(list: TAsmList); override;
         { called for a catch all exception }
         class procedure catch_all_start(list: TAsmList); override;
         class procedure catch_all_end(list: TAsmList); override;
         class procedure catch_all_add(list: TAsmList); override;
         class procedure cleanupobjectstack(list: TAsmList); override;
         class procedure popaddrstack(list: TAsmList); override;
       end;


    constructor TPSABIEHAction.Create(pad: TAsmLabel);
      begin
        landingpad:=pad;
        actionlist:=TAsmList.create;
        current_asmdata.getlabel(actiontablelabel,alt_data);
        actionlist.concat(tai_label.create(actiontablelabel));
        first:=true;
      end;


    destructor TPSABIEHAction.Destroy;
      begin
        if not(actionlist.Empty) then
          Internalerror(2019020501);
        actionlist.Free;
        inherited Destroy;
      end;


    function TPSABIEHAction.AddAction(p: tobjectdef) : LongInt;
      var
        index: LongInt;
      begin
        { if not first entry, signal that another action follows }
        if not(first) then
          actionlist.concat(tai_const.Create_sleb128bit(1));
        first:=false;

        { catch all? }
        if p=tobjectdef(-1) then
          index:=(current_procinfo as tpsabiehprocinfo).AddTypeFilter(nil)
        else if assigned(p) then
          index:=(current_procinfo as tpsabiehprocinfo).AddTypeFilter(p)
        else
          index:=-1;
{$ifdef debug_eh}
        if p=tobjectdef(-1) then
          actionlist.concat(tai_comment.Create(strpnew('Catch all')))
        else if assigned(p) then
          actionlist.concat(tai_comment.Create(strpnew('Action for '+p.GetTypeName)))
        else
          actionlist.concat(tai_comment.Create(strpnew('Cleanup')));
{$endif debug_eh}
        if assigned(p) then
          actionlist.concat(tai_const.Create_sleb128bit(index+1))
        else
          actionlist.concat(tai_const.Create_sleb128bit(0));
        Result:=index;
      end;

{****************************************************************************
                                 tpsabiehprocinfo
****************************************************************************}


    destructor tpsabiehprocinfo.destroy;
      begin
         gcc_except_table_data.free;
         actionstack.free;
         landingpadstack.free;
         typefilterlist.free;
         callsite_table_data.Free;
         action_table_data.Free;
         inherited;
      end;


    procedure tpsabiehprocinfo.PushAction(action: TPSABIEHAction);
      begin
        actionstack.add(action);
      end;


    function tpsabiehprocinfo.PopAction(action: TPSABIEHAction): boolean;
      begin
        if CurrentAction<>action then
          internalerror(2019022501);
        actionstack.count:=actionstack.count-1;
        result:=actionstack.count=0;
      end;


    function tpsabiehprocinfo.FinalizeAndPopAction(action: TPSABIEHAction): boolean;
      var
        curpos: tasmlabel;
      begin
        include(flags,pi_has_except_table_data);
        if CurrentAction<>action then
          internalerror(2019021006);
        { no further actions follow, finalize table
          we check for >1 as the outmost landing pad has no action, so
          we can ignore it }
        if landingpadstack.count>1 then
          begin
            current_asmdata.getlabel(curpos,alt_data);
            action.actionlist.concat(tai_label.create(curpos));
            action.actionlist.concat(tai_const.Create_rel_sym(aitconst_sleb128bit,curpos,TPSABIEHAction(landingpadstack[landingpadstack.count-1]).actiontablelabel));
          end
        else
          action.actionlist.concat(tai_const.Create_sleb128bit(0));
        action_table_data.concatList(action.actionlist);
        actionstack.count:=actionstack.count-1;
        result:=actionstack.count=0;
      end;


    procedure tpsabiehprocinfo.PushLandingPad(action: TPSABIEHAction);
      begin
        landingpadstack.add(action);
      end;


    function tpsabiehprocinfo.CurrentLandingPad: TPSABIEHAction;
      begin
        result:=TPSABIEHAction(landingpadstack.last);
      end;


    function tpsabiehprocinfo.PopLandingPad(action: TPSABIEHAction): boolean;
      begin
        if CurrentLandingPad<>action then
          internalerror(2019021007);
        landingpadstack.count:=landingpadstack.count-1;
        result:=landingpadstack.count=0;
      end;


    procedure tpsabiehprocinfo.CreateNewPSABIEHCallsite(list : TAsmList);
      var
        callsiteend : TAsmLabel;
      begin
        include(flags,pi_has_except_table_data);
        { first, finish last entry }
        if assigned(callsitelaststart) and assigned(CurrentLandingPad) then
          begin
{$ifdef debug_eh}
            if assigned(CurrentLandingPad.actiontablelabel) then
              callsite_table_data.concat(tai_comment.Create(strpnew('Call site '+tostr(CurrentCallSiteNumber)+', action table index = '+tostr(landingpadstack.count-1))))
            else
              callsite_table_data.concat(tai_comment.Create(strpnew('Call site '+tostr(CurrentCallSiteNumber)+', no action')));
{$endif debug_eh}
            callsite_table_data.concat(tai_const.create_rel_sym(aitconst_uleb128bit,current_asmdata.AsmCFI.get_frame_start,callsitelaststart));
            current_asmdata.getlabel(callsiteend,alt_eh_end);
            list.concat(tai_label.create(callsiteend));
            callsite_table_data.concat(tai_const.create_rel_sym(aitconst_uleb128bit,callsitelaststart,callsiteend));
            { landing pad? }
            if assigned(CurrentLandingPad.landingpad) then
              callsite_table_data.concat(tai_const.create_rel_sym(aitconst_uleb128bit,current_asmdata.AsmCFI.get_frame_start,CurrentLandingPad.landingpad))
            else
              callsite_table_data.concat(tai_const.Create_uleb128bit(0));
            { action number set? if yes, concat }
            if assigned(CurrentLandingPad.actiontablelabel) then
              begin
                callsite_table_data.concat(tai_const.Create_rel_sym_offset(aitconst_uleb128bit,callsitetableend,CurrentLandingPad.actiontablelabel,1));
{$ifdef debug_eh}
                list.concat(tai_comment.Create(strpnew('New call site '+tostr(CurrentCallSiteNumber)+', action table index = '+tostr(landingpadstack.count-1))));
{$endif debug_eh}
              end
            else
              begin
                callsite_table_data.concat(tai_const.Create_uleb128bit(0));
{$ifdef debug_eh}
                list.concat(tai_comment.Create(strpnew('New call site '+tostr(CurrentCallSiteNumber)+', no action')));
{$endif debug_eh}
              end;
            current_asmdata.getlabel(callsitelaststart,alt_eh_begin);
            list.concat(tai_label.create(callsitelaststart));
          end
        else
          begin
            current_asmdata.getlabel(entrycallsitestart,alt_eh_begin);
            callsitelaststart:=entrycallsitestart
          end;

        Inc(CurrentCallSiteNumber);
      end;


    function tpsabiehprocinfo.AddTypeFilter(p: tobjectdef) : Longint;
      var
        i: Integer;
      begin
        for i:=0 to typefilterlist.count-1 do
          begin
            if tobjectdef(typefilterlist[i])=p then
              begin
                result:=i;
                exit;
              end;
          end;
        result:=typefilterlist.add(p);
      end;


    procedure tpsabiehprocinfo.set_eh_info;
      begin
        inherited set_eh_info;
        if (tf_use_psabieh in target_info.flags) and not(pi_has_except_table_data in flags) then
          LSDALabel:=nil
        else
          current_asmdata.AsmCFI.get_cfa_list.concat(tdwarfitem.create_sym(DW_Set_LSDALabel,doe_32bit,LSDALabel));
      end;


    function tpsabiehprocinfo.CurrentAction: TPSABIEHAction; inline;
      begin
        result:=TPSABIEHAction(actionstack.last);
      end;


    function tpsabiehprocinfo.find_exception_handling(var n: tnode; para: pointer): foreachnoderesult;
      begin
        if n.nodetype in [tryfinallyn,tryexceptn,raisen,onn] then
          Result:=fen_norecurse_true
        else
          Result:=fen_false;
        end;


    procedure tpsabiehprocinfo.setup_eh;
      var
        gcc_except_table: tai_section;
      begin
        if tf_use_psabieh in target_info.flags then
          begin
            CreateExceptionTable:=foreachnode(code,@find_exception_handling,nil);

            gcc_except_table_data:=TAsmList.Create;
            callsite_table_data:=TAsmList.Create;
            action_table_data:=TAsmList.Create;
            actionstack:=TFPList.Create;
            landingpadstack:=TFPList.Create;
            typefilterlist:=TFPList.Create;
            gcc_except_table:=new_section(gcc_except_table_data,sec_gcc_except_table,'',0);
            gcc_except_table.secflags:=[SF_A];
            gcc_except_table.secprogbits:=SPB_PROGBITS;
{$ifdef debug_eh}
            gcc_except_table_data.concat(tai_comment.Create(strpnew('gcc_except_table for '+procdef.fullprocname(true))));
 {$endif debug_eh}
            current_asmdata.getlabel(LSDALabel,alt_data);

            current_asmdata.getlabel(callsitetablestart,alt_data);
            current_asmdata.getlabel(callsitetableend,alt_data);

            callsite_table_data.concat(tai_label.create(callsitetablestart));
            cexceptionstatehandler:=tpsabiehexceptionstatehandler;

            if CreateExceptionTable then
              begin
                CreateNewPSABIEHCallsite(current_asmdata.CurrAsmList);

                OutmostLandingPad:=TPSABIEHAction.Create(nil);
                NoAction:=OutmostLandingPad;
                PushAction(OutmostLandingPad);
                PushLandingPad(OutmostLandingPad);
                OutmostLandingPad.AddAction(nil);
              end;
          end;
      end;


    procedure tpsabiehprocinfo.finish_eh;
      var
        i: Integer;
      begin
        if tf_use_psabieh in target_info.flags then
          begin
            if pi_has_except_table_data in flags then
              begin
                gcc_except_table_data.concat(tai_label.create(LSDALabel));
                { landing pad base is relative to procedure start, so write an omit }
                gcc_except_table_data.concat(tai_const.create_8bit(DW_EH_PE_omit));

                if typefilterlist.count>0 then
                  begin
{$if defined(CPU64BITADDR)}
                    gcc_except_table_data.concat(tai_const.create_8bit(DW_EH_PE_udata8));
{$elseif defined(CPU32BITADDR)}
                    gcc_except_table_data.concat(tai_const.create_8bit(DW_EH_PE_udata4));
{$elseif defined(CPU16BITADDR)}
                    gcc_except_table_data.concat(tai_const.create_8bit(DW_EH_PE_udata2));
{$endif}
                    current_asmdata.getlabel(typefilterlistlabel,alt_data);
                    current_asmdata.getlabel(typefilterlistlabelref,alt_data);
                    gcc_except_table_data.concat(tai_const.create_rel_sym(aitconst_uleb128bit,typefilterlistlabel,typefilterlistlabelref));
                    gcc_except_table_data.concat(tai_label.create(typefilterlistlabel));
                  end
                else
                  { default types table encoding }
                  gcc_except_table_data.concat(tai_const.create_8bit(DW_EH_PE_omit));

                { call-site table encoded using uleb128 }
                gcc_except_table_data.concat(tai_const.create_8bit(DW_EH_PE_uleb128));
                gcc_except_table_data.concat(tai_const.create_rel_sym(aitconst_uleb128bit,callsitetablestart,callsitetableend));

                callsite_table_data.concat(tai_label.create(callsitetableend));
{$ifdef debug_eh}
                gcc_except_table_data.concat(tai_comment.Create(strpnew('Call site table for '+procdef.fullprocname(true))));
{$endif debug_eh}
                gcc_except_table_data.concatList(callsite_table_data);
                { action table must follow immediatly after callsite table }
{$ifdef debug_eh}
                if not(action_table_data.Empty) then
                  gcc_except_table_data.concat(tai_comment.Create(strpnew('Action table for '+procdef.fullprocname(true))));
{$endif debug_eh}
                gcc_except_table_data.concatlist(action_table_data);
                if typefilterlist.count>0 then
                  begin
{$ifdef debug_eh}
                    gcc_except_table_data.concat(tai_comment.Create(strpnew('Type filter list for '+procdef.fullprocname(true))));
{$endif debug_eh}
                    for i:=typefilterlist.count-1 downto 0 do
                      begin
{$ifdef debug_eh}
                        gcc_except_table_data.concat(tai_comment.Create(strpnew('Type filter '+tostr(i))));
{$endif debug_eh}
                        if assigned(typefilterlist[i]) then
                          gcc_except_table_data.concat(tai_const.Create_sym(current_asmdata.RefAsmSymbol(tobjectdef(typefilterlist[i]).vmt_mangledname, AT_DATA)))
                        else
                          gcc_except_table_data.concat(tai_const.Create_sym(nil));
                      end;
                    { the types are resolved by the negative offset, so the label must be written after all types }
                    gcc_except_table_data.concat(tai_label.create(typefilterlistlabelref));
                  end;

                new_section(gcc_except_table_data,sec_code,'',0);
                aktproccode.concatlist(gcc_except_table_data);
              end;
          end;
      end;


    procedure tpsabiehprocinfo.start_eh(list: TAsmList);
      begin
        inherited start_eh(list);
        if CreateExceptionTable then
          list.insert(tai_label.create(entrycallsitestart));
      end;


    procedure tpsabiehprocinfo.end_eh(list: TAsmList);
      begin
       inherited end_eh(list);
       if CreateExceptionTable then
         begin
           CreateNewPSABIEHCallsite(list);
           PopLandingPad(CurrentLandingPad);
           FinalizeAndPopAction(OutmostLandingPad);
         end;
      end;


    class procedure tpsabiehexceptionstatehandler.get_exception_temps(list: TAsmList; var t: texceptiontemps);
      begin
        tg.gethltemp(list,ossinttype,ossinttype.size,tt_persistent,t.reasonbuf);
      end;


    class procedure tpsabiehexceptionstatehandler.unget_exception_temps(list: TAsmList; const t: texceptiontemps);
      begin
        tg.ungettemp(list,t.reasonbuf);
        (current_procinfo as tpsabiehprocinfo).FinalizeAndPopAction((current_procinfo as tpsabiehprocinfo).CurrentAction);
      end;


    class procedure tpsabiehexceptionstatehandler.new_exception(list: TAsmList; const t: texceptiontemps;
      const exceptframekind: texceptframekind; out exceptstate: texceptionstate);
      var
        reg: tregister;
        action: TPSABIEHAction;
      begin
        exceptstate.oldflowcontrol:=flowcontrol;
        current_asmdata.getjumplabel(exceptstate.exceptionlabel);
        if exceptframekind<>tek_except then
          begin
            current_asmdata.getjumplabel(exceptstate.finallycodelabel);
            action:=TPSABIEHAction.Create(exceptstate.finallycodelabel);
          end
        else
          begin
            exceptstate.finallycodelabel:=nil;
            action:=TPSABIEHAction.Create(exceptstate.exceptionlabel);
          end;
        (current_procinfo as tpsabiehprocinfo).CreateNewPSABIEHCallsite(list);
        (current_procinfo as tpsabiehprocinfo).PushAction(action);
        (current_procinfo as tpsabiehprocinfo).PushLandingPad(action);
        if exceptframekind<>tek_except then
          { no safecall? }
          if use_cleanup(exceptframekind) then
            (current_procinfo as tpsabiehprocinfo).CurrentAction.AddAction(nil)
          else
            { if safecall, catch all }
            (current_procinfo as tpsabiehprocinfo).CurrentAction.AddAction(tobjectdef(-1));

        flowcontrol:=[fc_inflowcontrol,fc_catching_exceptions];
        if exceptframekind<>tek_except then
          begin
            reg:=hlcg.getintregister(list,ossinttype);
            hlcg.a_load_const_reg(list,ossinttype,1,reg);
            hlcg.g_exception_reason_save(list,ossinttype,ossinttype,reg,t.reasonbuf);
          end;
      end;


    class procedure tpsabiehexceptionstatehandler.emit_except_label(list: TAsmList; exceptframekind: texceptframekind;
      var exceptionstate: texceptionstate;var exceptiontemps:texceptiontemps);
      begin
        hlcg.g_unreachable(list);
        hlcg.a_label(list,exceptionstate.exceptionlabel);
        if exceptframekind<>tek_except then
          begin
            if not assigned(exceptionstate.finallycodelabel) then
              internalerror(2019021002);

            hlcg.a_label(list,exceptionstate.finallycodelabel);
            exceptionstate.finallycodelabel:=nil;
            exceptiontemps.unwind_info:=cg.getaddressregister(list);
            hlcg.a_load_reg_reg(list,voidpointertype,voidpointertype,NR_FUNCTION_RESULT_REG,exceptiontemps.unwind_info);
          end;
      end;


    class procedure tpsabiehexceptionstatehandler.end_try_block(list: TAsmList; exceptframekind: texceptframekind; const t: texceptiontemps;
      var exceptionstate: texceptionstate; endlabel: TAsmLabel);
      var
        reg: TRegister;
      begin
        if exceptframekind<>tek_except then
          begin
            { record that no exception happened in the reason buf, in case we are in a try block of a finally statement }
            reg:=hlcg.getintregister(list,ossinttype);
            hlcg.a_load_const_reg(list,ossinttype,0,reg);
            hlcg.g_exception_reason_save(list,ossinttype,ossinttype,reg,t.reasonbuf);
          end;
        inherited;
        if exceptframekind=tek_except then
          hlcg.a_jmp_always(list,endlabel);
        (current_procinfo as tpsabiehprocinfo).CreateNewPSABIEHCallsite(list);
        (current_procinfo as tpsabiehprocinfo).PopLandingPad((current_procinfo as tpsabiehprocinfo).CurrentLandingPad);
      end;


    class procedure tpsabiehexceptionstatehandler.free_exception(list: TAsmList; const t: texceptiontemps; const s: texceptionstate; a: aint;
      endexceptlabel: tasmlabel; onlyfree: boolean);
      begin
        { nothing to do }
      end;


    class procedure tpsabiehexceptionstatehandler.handle_reraise(list: TAsmList; const t: texceptiontemps; const entrystate: texceptionstate;
      const exceptframekind: texceptframekind);
      var
        cgpara1: tcgpara;
        pd: tprocdef;
        action, ReRaiseLandingPad: TPSABIEHAction;
        psabiehprocinfo: tpsabiehprocinfo;
      begin
        if not(fc_catching_exceptions in flowcontrol) and
           use_cleanup(exceptframekind) then
          begin
            { Resume might not be called outside of an landing pad else
              the unwind is immediatly terminated, so create an empty landing pad }
            psabiehprocinfo:=current_procinfo as tpsabiehprocinfo;

            if psabiehprocinfo.landingpadstack.count>1 then
              begin
                psabiehprocinfo.CreateNewPSABIEHCallsite(list);

                psabiehprocinfo.PushAction(psabiehprocinfo.NoAction);
                psabiehprocinfo.PushLandingPad(psabiehprocinfo.NoAction);
              end;

            pd:=search_system_proc('_unwind_resume');
            cgpara1.init;
            paramanager.getcgtempparaloc(list,pd,1,cgpara1);
            hlcg.a_load_reg_cgpara(list,voidpointertype,t.unwind_info,cgpara1);
            paramanager.freecgpara(list,cgpara1);
            hlcg.g_call_system_proc(list,'_unwind_resume',[@cgpara1],nil).resetiftemp;
            { we do not have to clean up the stack, we never return }
            cgpara1.done;

            if psabiehprocinfo.landingpadstack.count>1 then
              begin
                psabiehprocinfo.CreateNewPSABIEHCallsite(list);
                psabiehprocinfo.PopLandingPad(psabiehprocinfo.NoAction);
                psabiehprocinfo.PopAction(psabiehprocinfo.NoAction);
              end;
          end
        else
          begin
            psabiehprocinfo:=current_procinfo as tpsabiehprocinfo;
            { empty landing pad needed to avoid immediate termination? }
            if psabiehprocinfo.landingpadstack.Count=0 then
              begin
                psabiehprocinfo.CreateNewPSABIEHCallsite(list);

                ReRaiseLandingPad:=psabiehprocinfo.NoAction;
                psabiehprocinfo.PushAction(ReRaiseLandingPad);
                psabiehprocinfo.PushLandingPad(ReRaiseLandingPad);
              end
            else
              ReRaiseLandingPad:=nil;
            hlcg.g_call_system_proc(list,'fpc_reraise',[],nil).resetiftemp;
            if assigned(ReRaiseLandingPad) then
              begin
                psabiehprocinfo.CreateNewPSABIEHCallsite(list);
                psabiehprocinfo.PopLandingPad(psabiehprocinfo.CurrentLandingPad);
                psabiehprocinfo.PopAction(ReRaiseLandingPad);
             end;
          end;
      end;


    class procedure tpsabiehexceptionstatehandler.begin_catch_internal(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel;
      add_catch: boolean; out exceptlocdef: tdef; out exceptlocreg: tregister);
      var
        catchstartlab : tasmlabel;
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
        typeindex : aint;
      begin
        paraloc1.init;
        rttidef:=nil;
        rttisym:=nil;
        wrappedexception:=hlcg.getaddressregister(list,voidpointertype);
        hlcg.a_load_reg_reg(list,voidpointertype,voidpointertype,NR_FUNCTION_RESULT_REG,wrappedexception);
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
              end;
          end;
        { check if the exception is handled by this node }
        if assigned(excepttype) then
          begin
            typeindex:=(current_procinfo as tpsabiehprocinfo).CurrentAction.AddAction(excepttype);
            current_asmdata.getjumplabel(catchstartlab);
{$if defined(i386)}
            hlcg.a_cmp_const_reg_label (list,osuinttype,OC_EQ,typeindex+1,NR_FUNCTION_RESULT64_HIGH_REG,catchstartlab);
{$elseif defined(x86_64)}
            hlcg.a_cmp_const_reg_label (list,s32inttype,OC_EQ,typeindex+1,NR_EDX,catchstartlab);
{$else}
            { we need to find a way to fix this in a generic way }
            Internalerror(2019021008);
{$endif}
            hlcg.a_jmp_always(list,nextonlabel);
            hlcg.a_label(list,catchstartlab);
          end
        else
          (current_procinfo as tpsabiehprocinfo).CurrentAction.AddAction(tobjectdef(-1));

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


    class procedure tpsabiehexceptionstatehandler.catch_all_start_internal(list: TAsmList; add_catch: boolean);
      var
        exceptlocdef: tdef;
        exceptlocreg: tregister;
      begin
        begin_catch_internal(list,nil,nil,add_catch,exceptlocdef,exceptlocreg);
      end;


    class procedure tpsabiehexceptionstatehandler.begin_catch(list: TAsmList; excepttype: tobjectdef; nextonlabel: tasmlabel; out exceptlocdef: tdef; out
      exceptlocreg: tregister);
      begin
        begin_catch_internal(list,excepttype,nextonlabel,true,exceptlocdef,exceptlocreg);
      end;


    class procedure tpsabiehexceptionstatehandler.end_catch(list: TAsmList);
      begin
        hlcg.g_call_system_proc(list,'fpc_psabi_end_catch',[],nil).resetiftemp;
        inherited;
      end;


    class procedure tpsabiehexceptionstatehandler.catch_all_start(list: TAsmList);
      begin
        catch_all_start_internal(list,true);
      end;


    class procedure tpsabiehexceptionstatehandler.catch_all_add(list: TAsmList);
      begin
        (current_procinfo as tpsabiehprocinfo).CurrentAction.AddAction(nil);
      end;


    class procedure tpsabiehexceptionstatehandler.catch_all_end(list: TAsmList);
      begin
        hlcg.g_call_system_proc(list,'fpc_psabi_end_catch',[],nil).resetiftemp;
      end;


    class procedure tpsabiehexceptionstatehandler.cleanupobjectstack(list: TAsmList);
      begin
        { there is nothing to do }
      end;


    class procedure tpsabiehexceptionstatehandler.popaddrstack(list: TAsmList);
      begin
        { there is no addr stack, so do nothing }
      end;

end.
