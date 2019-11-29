{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate assembler for nodes that influence the flow which are
    the same for all (most?) processors

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
unit ncgflw;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      symtype,symdef,
      aasmbase,aasmdata,
      node,nflw,
      pass_2,cgbase,cgutils,ncgutil,cgexcept;

    type
       tcgwhilerepeatnode = class(twhilerepeatnode)
          usedregvars: tusedregvars;

          procedure pass_generate_code;override;
          procedure sync_regvars(checkusedregvars: boolean);
       end;

       tcgifnode = class(tifnode)
          procedure pass_generate_code;override;
       end;

       tcgfornode = class(tfornode)
          procedure pass_generate_code;override;
       end;

       tcgexitnode = class(texitnode)
          procedure pass_generate_code;override;
       end;

       tcgbreaknode = class(tbreaknode)
          procedure pass_generate_code;override;
       end;

       tcgcontinuenode = class(tcontinuenode)
          procedure pass_generate_code;override;
       end;

       tcggotonode = class(tgotonode)
          procedure pass_generate_code;override;
       end;

       tcglabelnode = class(tlabelnode)
       protected
          asmlabel : tasmlabel;
       public
          function getasmlabel : tasmlabel; virtual;
          procedure pass_generate_code;override;
       end;

       tcgraisenode = class(traisenode)
         function pass_1: tnode;override;
{$ifndef jvm}
         procedure pass_generate_code;override;
{$endif jvm}
       end;

       tcgtryexceptnode = class(ttryexceptnode)
        protected
          type
            tframetype = (ft_try,ft_except);

          procedure emit_jump_out_of_try_except_frame(list: TasmList; frametype: tframetype; const exceptiontate: tcgexceptionstatehandler.texceptionstate; var excepttemps: tcgexceptionstatehandler.texceptiontemps; framelabel, outerlabel: tasmlabel); virtual;
        public
          procedure pass_generate_code;override;
       end;

       tcgtryfinallynode = class(ttryfinallynode)
        protected
          procedure emit_jump_out_of_try_finally_frame(list: TasmList; const reason: byte; const finallycodelabel: tasmlabel; var excepttemps: tcgexceptionstatehandler.texceptiontemps; framelabel: tasmlabel);
          function get_jump_out_of_try_finally_frame_label(const finallyexceptionstate: tcgexceptionstatehandler.texceptionstate): tasmlabel;
        public
          procedure handle_safecall_exception;
          procedure pass_generate_code;override;
       end;

       tcgonnode = class(tonnode)
          procedure pass_generate_code;override;
       end;


implementation

    uses
      cutils,
      verbose,globals,systems,
      symconst,symsym,symtable,aasmtai,aasmcpu,defutil,
      procinfo,parabase,
      fmodule,
      cpubase,
      tgobj,paramgr,
      cgobj,hlcgobj,nutils
{$ifndef jvm}
      ,psabiehpi
{$endif jvm}
      ;
{*****************************************************************************
                         Second_While_RepeatN
*****************************************************************************}

    procedure tcgwhilerepeatnode.sync_regvars(checkusedregvars: boolean);
      begin
         if (cs_opt_regvar in current_settings.optimizerswitches) and
            not(pi_has_label in current_procinfo.flags) then
           begin
             if checkusedregvars then
               begin
                 usedregvars.intregvars.init;
                 usedregvars.addrregvars.init;
                 usedregvars.fpuregvars.init;
                 usedregvars.mmregvars.init;

                 { we have to synchronise both the regvars used in the loop }
                 { and the ones in the while/until condition                }
                 get_used_regvars(self,usedregvars);
                 gen_sync_regvars(current_asmdata.CurrAsmList,usedregvars);
               end
             else
               begin
                 gen_sync_regvars(current_asmdata.CurrAsmList,usedregvars);
                 usedregvars.intregvars.done;
                 usedregvars.addrregvars.done;
                 usedregvars.fpuregvars.done;
                 usedregvars.mmregvars.done;
               end;
           end;
      end;


    procedure tcgwhilerepeatnode.pass_generate_code;
      var
         lcont,lbreak,lloop,
         oldclabel,oldblabel : tasmlabel;
         truelabel,falselabel : tasmlabel;
         oldflowcontrol : tflowcontrol;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         current_asmdata.getjumplabel(lloop);
         current_asmdata.getjumplabel(lcont);
         current_asmdata.getjumplabel(lbreak);
         { arrange continue and breaklabels: }
         oldflowcontrol:=flowcontrol;
         oldclabel:=current_procinfo.CurrContinueLabel;
         oldblabel:=current_procinfo.CurrBreakLabel;
         include(flowcontrol,fc_inflowcontrol);
         exclude(flowcontrol,fc_unwind_loop);

         sync_regvars(true);
{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
         { handling code at the end as it is much more efficient, and makes
           while equal to repeat loop, only the end true/false is swapped (PFV) }
         if lnf_testatbegin in loopflags then
           hlcg.a_jmp_always(current_asmdata.CurrAsmList,lcont);

         if not(cs_opt_size in current_settings.optimizerswitches) then
            { align loop target, as an unconditional jump is done before,
              use jump align which assume that the instructions inserted as alignment are never executed }
            current_asmdata.CurrAsmList.concat(cai_align.create_max(current_settings.alignment.jumpalign,current_settings.alignment.jumpalignskipmax));

         hlcg.a_label(current_asmdata.CurrAsmList,lloop);

         current_procinfo.CurrContinueLabel:=lcont;
         current_procinfo.CurrBreakLabel:=lbreak;

         if assigned(right) then
           secondpass(right);

{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}

         hlcg.a_label(current_asmdata.CurrAsmList,lcont);
         if lnf_checknegate in loopflags then
          begin
             truelabel:=lbreak;
             falselabel:=lloop;
          end
         else
          begin
             truelabel:=lloop;
             falselabel:=lbreak;
          end;
         secondpass(left);

         hlcg.maketojumpboollabels(current_asmdata.CurrAsmList,left,truelabel,falselabel);
         hlcg.a_label(current_asmdata.CurrAsmList,lbreak);

         sync_regvars(false);

         current_procinfo.CurrContinueLabel:=oldclabel;
         current_procinfo.CurrBreakLabel:=oldblabel;
         { a break/continue in a while/repeat block can't be seen outside }
         flowcontrol:=oldflowcontrol+(flowcontrol-[fc_break,fc_continue,fc_inflowcontrol]);
      end;


{*****************************************************************************
                               tcgIFNODE
*****************************************************************************}

    procedure tcgifnode.pass_generate_code;

      var
         hl : tasmlabel;
         oldflowcontrol: tflowcontrol;
(*
         org_regvar_loaded_other,
         then_regvar_loaded_other,
         else_regvar_loaded_other : regvarother_booleanarray;
         org_regvar_loaded_int,
         then_regvar_loaded_int,
         else_regvar_loaded_int : Tsuperregisterset;
         org_list,
         then_list,
         else_list : TAsmList;
*)

      begin
         location_reset(location,LOC_VOID,OS_NO);
         hl:=nil;

         oldflowcontrol := flowcontrol;
         include(flowcontrol,fc_inflowcontrol);
         secondpass(left);

(*
         { save regvars loaded in the beginning so that we can restore them }
         { when processing the else-block                                   }
         if cs_opt_regvar in current_settings.optimizerswitches then
           begin
             org_list := current_asmdata.CurrAsmList;
             current_asmdata.CurrAsmList := TAsmList.create;
           end;
*)
         hlcg.maketojumpbool(current_asmdata.CurrAsmList,left);

(*
         if cs_opt_regvar in current_settings.optimizerswitches then
           begin
             org_regvar_loaded_int := rg.regvar_loaded_int;
             org_regvar_loaded_other := rg.regvar_loaded_other;
           end;
*)
         if assigned(right) then
           begin
              hlcg.a_label(current_asmdata.CurrAsmList,left.location.truelabel);
              secondpass(right);
           end;

         { save current asmlist (previous instructions + then-block) and }
         { loaded regvar state and create new clean ones                 }
{
         if cs_opt_regvar in current_settings.optimizerswitches then
           begin
             then_regvar_loaded_int := rg.regvar_loaded_int;
             then_regvar_loaded_other := rg.regvar_loaded_other;
             rg.regvar_loaded_int := org_regvar_loaded_int;
             rg.regvar_loaded_other := org_regvar_loaded_other;
             then_list := current_asmdata.CurrAsmList;
             current_asmdata.CurrAsmList := TAsmList.create;
           end;
}

         if assigned(t1) then
           begin
              if assigned(right) then
                begin
                   current_asmdata.getjumplabel(hl);
                   { do go back to if line !! }
(*
                   if not(cs_opt_regvar in current_settings.optimizerswitches) then
*)
                     current_filepos:=current_asmdata.CurrAsmList.getlasttaifilepos^
(*
                   else
                     current_filepos:=then_list.getlasttaifilepos^
*)
                   ;
                   hlcg.a_jmp_always(current_asmdata.CurrAsmList,hl);
                   if not(cs_opt_size in current_settings.optimizerswitches) then
                     current_asmdata.CurrAsmList.concat(cai_align.create_max(current_settings.alignment.jumpalign,current_settings.alignment.jumpalignskipmax));
                end;
              hlcg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
              secondpass(t1);
(*
              { save current asmlist (previous instructions + else-block) }
              { and loaded regvar state and create a new clean list       }
              if cs_opt_regvar in current_settings.optimizerswitches then
                begin
{                  else_regvar_loaded_int := rg.regvar_loaded_int;
                  else_regvar_loaded_other := rg.regvar_loaded_other;}
                  else_list := current_asmdata.CurrAsmList;
                  current_asmdata.CurrAsmList := TAsmList.create;
                end;
*)
              if assigned(right) then
                hlcg.a_label(current_asmdata.CurrAsmList,hl);
           end
         else
           begin
(*
              if cs_opt_regvar in current_settings.optimizerswitches then
                begin
{                  else_regvar_loaded_int := rg.regvar_loaded_int;
                  else_regvar_loaded_other := rg.regvar_loaded_other;}
                  else_list := current_asmdata.CurrAsmList;
                  current_asmdata.CurrAsmList := TAsmList.create;
                end;
*)
              if not(cs_opt_size in current_settings.optimizerswitches) then
                current_asmdata.CurrAsmList.concat(cai_align.create_max(current_settings.alignment.coalescealign,current_settings.alignment.coalescealignskipmax));
              hlcg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
           end;
         if not(assigned(right)) then
           begin
             if not(cs_opt_size in current_settings.optimizerswitches) then
               current_asmdata.CurrAsmList.concat(cai_align.create_max(current_settings.alignment.coalescealign,current_settings.alignment.coalescealignskipmax));
             hlcg.a_label(current_asmdata.CurrAsmList,left.location.truelabel);
           end;

(*
         if cs_opt_regvar in current_settings.optimizerswitches then
           begin
             { add loads of regvars at the end of the then- and else-blocks  }
             { so that at the end of both blocks the same regvars are loaded }

             { no else block? }
             if not assigned(t1) then
               begin
                 sync_regvars_int(org_list,then_list,org_regvar_loaded_int,then_regvar_loaded_int);
                 sync_regvars_other(org_list,then_list,org_regvar_loaded_other,then_regvar_loaded_other);
               end
             { no then block? }
             else if not assigned(right) then
               begin
                 sync_regvars_int(org_list,else_list,org_regvar_loaded_int,else_regvar_loaded_int);
                 sync_regvars_other(org_list,else_list,org_regvar_loaded_other,else_regvar_loaded_other);
               end
             { both else and then blocks }
             else
               begin
                 sync_regvars_int(then_list,else_list,then_regvar_loaded_int,else_regvar_loaded_int);
                 sync_regvars_other(then_list,else_list,then_regvar_loaded_other,else_regvar_loaded_other);
               end;
             { add all lists together }
             org_list.concatlist(then_list);
             then_list.free;
             org_list.concatlist(else_list);
             else_list.free;
             org_list.concatlist(current_asmdata.CurrAsmList);
             current_asmdata.CurrAsmList.free;
             current_asmdata.CurrAsmList := org_list;
           end;
*)

         flowcontrol := oldflowcontrol + (flowcontrol - [fc_inflowcontrol]);
      end;


{*****************************************************************************
                              SecondFor
*****************************************************************************}

    procedure tcgfornode.pass_generate_code;
      begin
        { for nodes are converted in pass_1 in a while loop }
        internalerror(2015082501);
      end;


{*****************************************************************************
                              SecondExitN
*****************************************************************************}

    procedure tcgexitnode.pass_generate_code;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         include(flowcontrol,fc_exit);
         if assigned(left) then
           secondpass(left);

         if (fc_unwind_exit in flowcontrol) then
           hlcg.g_local_unwind(current_asmdata.CurrAsmList,current_procinfo.CurrExitLabel)
         else
           hlcg.a_jmp_always(current_asmdata.CurrAsmList,current_procinfo.CurrExitLabel);
       end;


{*****************************************************************************
                              SecondBreakN
*****************************************************************************}

    procedure tcgbreaknode.pass_generate_code;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         include(flowcontrol,fc_break);
         if current_procinfo.CurrBreakLabel<>nil then
           begin
{$ifdef OLDREGVARS}
             load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
             if (fc_unwind_loop in flowcontrol) then
               hlcg.g_local_unwind(current_asmdata.CurrAsmList,current_procinfo.CurrBreakLabel)
             else
               hlcg.a_jmp_always(current_asmdata.CurrAsmList,current_procinfo.CurrBreakLabel)
           end
         else
           CGMessage(cg_e_break_not_allowed);
      end;


{*****************************************************************************
                              SecondContinueN
*****************************************************************************}

    procedure tcgcontinuenode.pass_generate_code;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         include(flowcontrol,fc_continue);
         if current_procinfo.CurrContinueLabel<>nil then
           begin
{$ifdef OLDREGVARS}
             load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
             if (fc_unwind_loop in flowcontrol) then
               hlcg.g_local_unwind(current_asmdata.CurrAsmList,current_procinfo.CurrContinueLabel)
             else
               hlcg.a_jmp_always(current_asmdata.CurrAsmList,current_procinfo.CurrContinueLabel)
           end
         else
           CGMessage(cg_e_continue_not_allowed);
      end;


{*****************************************************************************
                             SecondGoto
*****************************************************************************}

    procedure tcggotonode.pass_generate_code;

       begin
         location_reset(location,LOC_VOID,OS_NO);

         include(flowcontrol,fc_gotolabel);
{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
         hlcg.a_jmp_always(current_asmdata.CurrAsmList,tcglabelnode(labelnode).getasmlabel)
       end;


{*****************************************************************************
                             SecondLabel
*****************************************************************************}

    function tcglabelnode.getasmlabel : tasmlabel;
      begin
        if not(assigned(asmlabel)) then
          { labsym is not set in inlined procedures, but since assembler }
          { routines can't be inlined, that shouldn't matter             }
          if assigned(labsym) and
             labsym.nonlocal then
            current_asmdata.getglobaljumplabel(asmlabel)
          else
            current_asmdata.getjumplabel(asmlabel);
        result:=asmlabel
      end;


    procedure tcglabelnode.pass_generate_code;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         include(flowcontrol,fc_gotolabel);
{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
         hlcg.a_label(current_asmdata.CurrAsmList,getasmlabel);

         { Write also extra label if this label was referenced from
           assembler block }
         if assigned(labsym) and
            assigned(labsym.asmblocklabel) then
           hlcg.a_label(current_asmdata.CurrAsmList,labsym.asmblocklabel);

         secondpass(left);
      end;


{*****************************************************************************
                             SecondTryExcept
*****************************************************************************}

    var
       endexceptlabel : tasmlabel;

     { jump out of an try/except block }
     procedure tcgtryexceptnode.emit_jump_out_of_try_except_frame(list: TasmList; frametype: tframetype; const exceptiontate: tcgexceptionstatehandler.texceptionstate; var excepttemps: tcgexceptionstatehandler.texceptiontemps; framelabel, outerlabel: tasmlabel);
       begin
          hlcg.a_label(list,framelabel);
          { we must also destroy the address frame which guards
            the exception object }
          cexceptionstatehandler.popaddrstack(list);
          hlcg.g_exception_reason_discard(list,osuinttype,excepttemps.reasonbuf);
          if frametype=ft_except then
            begin
              cexceptionstatehandler.cleanupobjectstack(list);
              cexceptionstatehandler.end_catch(list);
            end;
          hlcg.a_jmp_always(list,outerlabel);
       end;


    procedure tcgtryexceptnode.pass_generate_code;

      var
         oldendexceptlabel,
         lastonlabel,
         exitexceptlabel,
         continueexceptlabel,
         breakexceptlabel,
         exittrylabel,
         continuetrylabel,
         breaktrylabel,
         oldCurrExitLabel,
         oldContinueLabel,
         oldBreakLabel : tasmlabel;
         destroytemps,
         excepttemps : tcgexceptionstatehandler.texceptiontemps;
         trystate,doobjectdestroyandreraisestate: tcgexceptionstatehandler.texceptionstate;
         afteronflowcontrol: tflowcontrol;
      label
         errorexit;
      begin
         location_reset(location,LOC_VOID,OS_NO);
         continuetrylabel:=nil;
         breaktrylabel:=nil;
         continueexceptlabel:=nil;
         breakexceptlabel:=nil;
         doobjectdestroyandreraisestate:=Default(tcgexceptionstatehandler.texceptionstate);

         { this can be called recursivly }
         oldBreakLabel:=nil;
         oldContinueLabel:=nil;
         oldendexceptlabel:=endexceptlabel;

         { save the old labels for control flow statements }
         oldCurrExitLabel:=current_procinfo.CurrExitLabel;
         if assigned(current_procinfo.CurrBreakLabel) then
           begin
              oldContinueLabel:=current_procinfo.CurrContinueLabel;
              oldBreakLabel:=current_procinfo.CurrBreakLabel;
           end;

         { get new labels for the control flow statements }
         current_asmdata.getjumplabel(exittrylabel);
         current_asmdata.getjumplabel(exitexceptlabel);
         if assigned(current_procinfo.CurrBreakLabel) then
           begin
              current_asmdata.getjumplabel(breaktrylabel);
              current_asmdata.getjumplabel(continuetrylabel);
              current_asmdata.getjumplabel(breakexceptlabel);
              current_asmdata.getjumplabel(continueexceptlabel);
           end;

         current_asmdata.getjumplabel(endexceptlabel);
         current_asmdata.getjumplabel(lastonlabel);

         cexceptionstatehandler.get_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         cexceptionstatehandler.new_exception(current_asmdata.CurrAsmList,excepttemps,tek_except,trystate);

         { try block }
         { set control flow labels for the try block }
         current_procinfo.CurrExitLabel:=exittrylabel;
         if assigned(oldBreakLabel) then
          begin
            current_procinfo.CurrContinueLabel:=continuetrylabel;
            current_procinfo.CurrBreakLabel:=breaktrylabel;
          end;

         secondpass(left);
         if codegenerror then
           goto errorexit;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

         cexceptionstatehandler.end_try_block(current_asmdata.CurrAsmList,tek_except,excepttemps,trystate,endexceptlabel);

         cexceptionstatehandler.emit_except_label(current_asmdata.CurrAsmList,tek_except,trystate,excepttemps);
         cexceptionstatehandler.free_exception(current_asmdata.CurrAsmList, excepttemps, trystate, 0, endexceptlabel, false);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { set control flow labels for the except block }
         { and the on statements                        }
         current_procinfo.CurrExitLabel:=exitexceptlabel;
         if assigned(oldBreakLabel) then
          begin
            current_procinfo.CurrContinueLabel:=continueexceptlabel;
            current_procinfo.CurrBreakLabel:=breakexceptlabel;
          end;

         flowcontrol:=[fc_inflowcontrol]+trystate.oldflowcontrol*[fc_catching_exceptions];
         { on statements }
         if assigned(right) then
           secondpass(right);

         afteronflowcontrol:=flowcontrol;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

         hlcg.a_label(current_asmdata.CurrAsmList,lastonlabel);
         { default handling except handling }
         if assigned(t1) then
           begin
              { FPC_CATCHES with 'default handler' flag (=-1) need no longer be called,
                it doesn't change any state and its return value is ignored (Sergei)
              }

              { the destruction of the exception object must be also }
              { guarded by an exception frame, but it can be omitted }
              { if there's no user code in 'except' block            }

              cexceptionstatehandler.catch_all_start(current_asmdata.CurrAsmList);
              if not (has_no_code(t1)) then
               begin
                 { if there is an outer frame that catches exceptions, remember this for the "except"
                   part of this try/except }
                 flowcontrol:=trystate.oldflowcontrol*[fc_inflowcontrol,fc_catching_exceptions];
                 cexceptionstatehandler.get_exception_temps(current_asmdata.CurrAsmList,destroytemps);
                 cexceptionstatehandler.new_exception(current_asmdata.CurrAsmList,destroytemps,tek_except,doobjectdestroyandreraisestate);
                 cexceptionstatehandler.catch_all_add(current_asmdata.CurrAsmList);
                 { the flowcontrol from the default except-block must be merged
                   with the flowcontrol flags potentially set by the
                   on-statements handled above (secondpass(right)), as they are
                   at the same program level }
                 flowcontrol:=
                   flowcontrol+
                   afteronflowcontrol;

                 { except block needs line info }
                 current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

                 secondpass(t1);

                 cexceptionstatehandler.handle_nested_exception(current_asmdata.CurrAsmList,destroytemps,doobjectdestroyandreraisestate);

                 cexceptionstatehandler.unget_exception_temps(current_asmdata.CurrAsmList,destroytemps);
                 cexceptionstatehandler.catch_all_end(current_asmdata.CurrAsmList);
                 hlcg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);
               end
             else
               begin
                 doobjectdestroyandreraisestate.newflowcontrol:=afteronflowcontrol;
                 cexceptionstatehandler.cleanupobjectstack(current_asmdata.CurrAsmList);
                 cexceptionstatehandler.catch_all_end(current_asmdata.CurrAsmList);
                 hlcg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);
               end;
           end
         else
           begin
             cexceptionstatehandler.handle_reraise(current_asmdata.CurrAsmList,excepttemps,trystate,tek_except);
             doobjectdestroyandreraisestate.newflowcontrol:=afteronflowcontrol;
           end;

         if fc_exit in doobjectdestroyandreraisestate.newflowcontrol then
           emit_jump_out_of_try_except_frame(current_asmdata.CurrAsmList,ft_except,doobjectdestroyandreraisestate,excepttemps,exitexceptlabel,oldCurrExitLabel);

         if fc_break in doobjectdestroyandreraisestate.newflowcontrol then
           emit_jump_out_of_try_except_frame(current_asmdata.CurrAsmList,ft_except,doobjectdestroyandreraisestate,excepttemps,breakexceptlabel,oldBreakLabel);

         if fc_continue in doobjectdestroyandreraisestate.newflowcontrol then
           emit_jump_out_of_try_except_frame(current_asmdata.CurrAsmList,ft_except,doobjectdestroyandreraisestate,excepttemps,continueexceptlabel,oldContinueLabel);

         if fc_exit in trystate.newflowcontrol then
           emit_jump_out_of_try_except_frame(current_asmdata.CurrAsmList,ft_try,trystate,excepttemps,exittrylabel,oldCurrExitLabel);

         if fc_break in trystate.newflowcontrol then
          emit_jump_out_of_try_except_frame(current_asmdata.CurrAsmList,ft_try,trystate,excepttemps,breaktrylabel,oldBreakLabel);

         if fc_continue in trystate.newflowcontrol then
           emit_jump_out_of_try_except_frame(current_asmdata.CurrAsmList,ft_try,trystate,excepttemps,continuetrylabel,oldContinueLabel);

         cexceptionstatehandler.unget_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         hlcg.a_label(current_asmdata.CurrAsmList,endexceptlabel);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

       errorexit:
         { restore all saved labels }
         endexceptlabel:=oldendexceptlabel;

         { restore the control flow labels }
         current_procinfo.CurrExitLabel:=oldCurrExitLabel;
         if assigned(oldBreakLabel) then
          begin
            current_procinfo.CurrContinueLabel:=oldContinueLabel;
            current_procinfo.CurrBreakLabel:=oldBreakLabel;
          end;

         { return all used control flow statements }
         flowcontrol:=trystate.oldflowcontrol+(doobjectdestroyandreraisestate.newflowcontrol +
           trystate.newflowcontrol - [fc_inflowcontrol,fc_catching_exceptions]);
      end;


    procedure tcgonnode.pass_generate_code;
      var
         nextonlabel,
         exitonlabel,
         continueonlabel,
         breakonlabel,
         oldCurrExitLabel,
         oldContinueLabel,
         oldBreakLabel : tasmlabel;
         doobjectdestroyandreraisestate: tcgexceptionstatehandler.texceptionstate;
         excepttemps : tcgexceptionstatehandler.texceptiontemps;
         exceptvarsym : tlocalvarsym;
         exceptlocdef: tdef;
         exceptlocreg: tregister;
      begin
         location_reset(location,LOC_VOID,OS_NO);
         oldCurrExitLabel:=nil;
         continueonlabel:=nil;
         breakonlabel:=nil;
         exitonlabel:=nil;

         current_asmdata.getjumplabel(nextonlabel);

         cexceptionstatehandler.begin_catch(current_asmdata.CurrAsmList,excepttype,nextonlabel,exceptlocdef,exceptlocreg);

         { Retrieve exception variable }
         if assigned(excepTSymtable) then
           exceptvarsym:=tlocalvarsym(excepTSymtable.SymList[0])
         else
           internalerror(2011020401);

         if assigned(exceptvarsym) then
           begin
             location_reset_ref(exceptvarsym.localloc, LOC_REFERENCE, def_cgsize(voidpointertype), voidpointertype.alignment, []);
             tg.GetLocal(current_asmdata.CurrAsmList, exceptvarsym.vardef.size, exceptvarsym.vardef, exceptvarsym.localloc.reference);
             hlcg.a_load_reg_ref(current_asmdata.CurrAsmList, exceptlocdef, exceptvarsym.vardef, exceptlocreg, exceptvarsym.localloc.reference);
           end;
         { in the case that another exception is risen
           we've to destroy the old one, so create a new
           exception frame for the catch-handler }
         cexceptionstatehandler.get_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         cexceptionstatehandler.new_exception(current_asmdata.CurrAsmList,excepttemps,tek_except,doobjectdestroyandreraisestate);

         oldBreakLabel:=nil;
         oldContinueLabel:=nil;
         if assigned(right) then
           begin
              oldCurrExitLabel:=current_procinfo.CurrExitLabel;
              current_asmdata.getjumplabel(exitonlabel);
              current_procinfo.CurrExitLabel:=exitonlabel;
              if assigned(current_procinfo.CurrBreakLabel) then
               begin
                 oldContinueLabel:=current_procinfo.CurrContinueLabel;
                 oldBreakLabel:=current_procinfo.CurrBreakLabel;
                 current_asmdata.getjumplabel(breakonlabel);
                 current_asmdata.getjumplabel(continueonlabel);
                 current_procinfo.CurrContinueLabel:=continueonlabel;
                 current_procinfo.CurrBreakLabel:=breakonlabel;
               end;

              secondpass(right);
           end;

         cexceptionstatehandler.handle_nested_exception(current_asmdata.CurrAsmList,excepttemps,doobjectdestroyandreraisestate);

         { clear some stuff }
         if assigned(exceptvarsym) then
           begin
             tg.UngetLocal(current_asmdata.CurrAsmList,exceptvarsym.localloc.reference);
             exceptvarsym.localloc.loc:=LOC_INVALID;
           end;
         cexceptionstatehandler.end_catch(current_asmdata.CurrAsmList);
         hlcg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);

         if assigned(right) then
           begin
              { special handling for control flow instructions }
              if fc_exit in doobjectdestroyandreraisestate.newflowcontrol then
                begin
                   { the address and object pop does secondtryexcept }
                   hlcg.a_label(current_asmdata.CurrAsmList,exitonlabel);
                   hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
                end;

              if fc_break in doobjectdestroyandreraisestate.newflowcontrol then
                begin
                   { the address and object pop does secondtryexcept }
                   hlcg.a_label(current_asmdata.CurrAsmList,breakonlabel);
                   hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
                end;

              if fc_continue in doobjectdestroyandreraisestate.newflowcontrol then
                begin
                   { the address and object pop does secondtryexcept }
                   hlcg.a_label(current_asmdata.CurrAsmList,continueonlabel);
                   hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
                end;

              current_procinfo.CurrExitLabel:=oldCurrExitLabel;
              if assigned(oldBreakLabel) then
               begin
                 current_procinfo.CurrContinueLabel:=oldContinueLabel;
                 current_procinfo.CurrBreakLabel:=oldBreakLabel;
               end;
           end;

         cexceptionstatehandler.unget_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         hlcg.a_label(current_asmdata.CurrAsmList,nextonlabel);
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { propagate exit/break/continue }
         flowcontrol:=doobjectdestroyandreraisestate.oldflowcontrol+(doobjectdestroyandreraisestate.newflowcontrol-[fc_inflowcontrol,fc_catching_exceptions]);

         { next on node }
         if assigned(left) then
           secondpass(left);
      end;

{*****************************************************************************
                             SecondTryFinally
*****************************************************************************}

    { jump out of a finally block }
    procedure tcgtryfinallynode.emit_jump_out_of_try_finally_frame(list: TasmList; const reason: byte; const finallycodelabel: tasmlabel; var excepttemps: tcgexceptionstatehandler.texceptiontemps; framelabel: tasmlabel);
      begin
         hlcg.a_label(list,framelabel);
         hlcg.g_exception_reason_discard(list,osuinttype,excepttemps.reasonbuf);
         hlcg.g_exception_reason_save_const(list,osuinttype,reason,excepttemps.reasonbuf);
         hlcg.a_jmp_always(list,finallycodelabel);
      end;


    function tcgtryfinallynode.get_jump_out_of_try_finally_frame_label(const finallyexceptionstate: tcgexceptionstatehandler.texceptionstate): tasmlabel;
      begin
        current_asmdata.getjumplabel(result);
      end;


    procedure tcgtryfinallynode.handle_safecall_exception;
      var
        cgpara, resultpara: tcgpara;
        selfsym: tparavarsym;
        pd: tprocdef;
        safecallresult: tlocalvarsym;
      begin
        { call fpc_safecallhandler, passing self for methods of classes,
          nil otherwise. }
        pd:=search_system_proc('fpc_safecallhandler');
        cgpara.init;
        paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,1,cgpara);
        if is_class(current_procinfo.procdef.struct) then
          begin
            selfsym:=tparavarsym(current_procinfo.procdef.parast.Find('self'));
            if (selfsym=nil) or (selfsym.typ<>paravarsym) then
              InternalError(2011123101);
            hlcg.a_load_loc_cgpara(current_asmdata.CurrAsmList,selfsym.vardef,selfsym.localloc,cgpara);
          end
        else
          hlcg.a_load_const_cgpara(current_asmdata.CurrAsmList,voidpointertype,0,cgpara);
        paramanager.freecgpara(current_asmdata.CurrAsmList,cgpara);
        resultpara:=hlcg.g_call_system_proc(current_asmdata.CurrAsmList,pd,[@cgpara],nil);
        cgpara.done;
        safecallresult:=tlocalvarsym(current_procinfo.procdef.localst.Find('safecallresult'));
        hlcg.gen_load_cgpara_loc(current_asmdata.CurrAsmList,resultpara.def,resultpara,safecallresult.localloc,false);
        resultpara.resetiftemp;
      end;


    procedure tcgtryfinallynode.pass_generate_code;
      var
         endfinallylabel,
         exitfinallylabel,
         continuefinallylabel,
         breakfinallylabel,
         oldCurrExitLabel,
         oldContinueLabel,
         oldBreakLabel,
         finallyNoExceptionLabel: tasmlabel;
         finallyexceptionstate: tcgexceptionstatehandler.texceptionstate;
         excepttemps : tcgexceptionstatehandler.texceptiontemps;
         reasonreg : tregister;
         exceptframekind: tcgexceptionstatehandler.texceptframekind;
         tmplist: TAsmList;

        procedure handle_breakcontinueexit(const finallycode: tasmlabel; doreraise: boolean);
          begin
            { no exception happened, but maybe break/continue/exit }
            hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,0,reasonreg,endfinallylabel);
            if fc_exit in finallyexceptionstate.newflowcontrol then
              hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,2,reasonreg,oldCurrExitLabel);
            if fc_break in finallyexceptionstate.newflowcontrol then
              hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,3,reasonreg,oldBreakLabel);
            if fc_continue in finallyexceptionstate.newflowcontrol then
              hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,4,reasonreg,oldContinueLabel);
            if doreraise then
              cexceptionstatehandler.handle_reraise(current_asmdata.CurrAsmList,excepttemps,finallyexceptionstate,tek_normalfinally)
            else
              hlcg.g_unreachable(current_asmdata.CurrAsmList);
            { redirect break/continue/exit to the label above, with the reasonbuf set appropriately }
            if fc_exit in finallyexceptionstate.newflowcontrol then
              emit_jump_out_of_try_finally_frame(current_asmdata.CurrAsmList,2,finallycode,excepttemps,exitfinallylabel);
            if fc_break in finallyexceptionstate.newflowcontrol then
              emit_jump_out_of_try_finally_frame(current_asmdata.CurrAsmList,3,finallycode,excepttemps,breakfinallylabel);
            if fc_continue in finallyexceptionstate.newflowcontrol then
              emit_jump_out_of_try_finally_frame(current_asmdata.CurrAsmList,4,finallycode,excepttemps,continuefinallylabel);
          end;

      begin
         location_reset(location,LOC_VOID,OS_NO);
         oldBreakLabel:=nil;
         oldContinueLabel:=nil;
         continuefinallylabel:=nil;
         breakfinallylabel:=nil;

         if not implicitframe then
           exceptframekind:=tek_normalfinally
         else
           exceptframekind:=tek_implicitfinally;

         current_asmdata.getjumplabel(endfinallylabel);

         { call setjmp, and jump to finally label on non-zero result }
         cexceptionstatehandler.get_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         cexceptionstatehandler.new_exception(current_asmdata.CurrAsmList,excepttemps,exceptframekind,finallyexceptionstate);

         { the finally block must catch break, continue and exit }
         { statements                                            }
         oldCurrExitLabel:=current_procinfo.CurrExitLabel;
         exitfinallylabel:=get_jump_out_of_try_finally_frame_label(finallyexceptionstate);
         current_procinfo.CurrExitLabel:=exitfinallylabel;
         if assigned(current_procinfo.CurrBreakLabel) then
          begin
            oldContinueLabel:=current_procinfo.CurrContinueLabel;
            oldBreakLabel:=current_procinfo.CurrBreakLabel;
            breakfinallylabel:=get_jump_out_of_try_finally_frame_label(finallyexceptionstate);
            continuefinallylabel:=get_jump_out_of_try_finally_frame_label(finallyexceptionstate);
            current_procinfo.CurrContinueLabel:=continuefinallylabel;
            current_procinfo.CurrBreakLabel:=breakfinallylabel;
          end;

         { try code }
         if assigned(left) then
           begin
              secondpass(left);
              if codegenerror then
                exit;
           end;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

         cexceptionstatehandler.end_try_block(current_asmdata.CurrAsmList,exceptframekind,excepttemps,finallyexceptionstate,finallyexceptionstate.finallycodelabel);
         if assigned(third) then
           begin
             tmplist:=TAsmList.create;
             { emit the except label already (to a temporary list) to ensure that any calls in the
               finally block refer to the outer exception frame rather than to the exception frame
               that emits this same finally code in case an exception does happen }
             cexceptionstatehandler.emit_except_label(tmplist,exceptframekind,finallyexceptionstate,excepttemps);

             flowcontrol:=finallyexceptionstate.oldflowcontrol*[fc_inflowcontrol,fc_catching_exceptions];
             current_asmdata.getjumplabel(finallyNoExceptionLabel);
             hlcg.a_label(current_asmdata.CurrAsmList,finallyNoExceptionLabel);
             if not implicitframe then
               current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));
             secondpass(third);
             if codegenerror then
               exit;
             if not implicitframe then
               current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));
             reasonreg:=hlcg.getintregister(current_asmdata.CurrAsmList,osuinttype);
             hlcg.g_exception_reason_load(current_asmdata.CurrAsmList,osuinttype,osuinttype,excepttemps.reasonbuf,reasonreg);
             handle_breakcontinueexit(finallyNoExceptionLabel,false);

             current_asmdata.CurrAsmList.concatList(tmplist);
             tmplist.free;
           end
         else
           cexceptionstatehandler.emit_except_label(current_asmdata.CurrAsmList,exceptframekind,finallyexceptionstate,excepttemps);

         { just free the frame information }
         cexceptionstatehandler.free_exception(current_asmdata.CurrAsmList,excepttemps,finallyexceptionstate,1,finallyexceptionstate.exceptionlabel,true);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { finally code (don't unconditionally set fc_inflowcontrol, since the
           finally code is unconditionally executed; we do have to filter out
           flags regarding break/contrinue/etc. because we have to give an
           error in case one of those is used in the finally-code }
         flowcontrol:=finallyexceptionstate.oldflowcontrol*[fc_inflowcontrol,fc_catching_exceptions];
         secondpass(right);
         { goto is allowed if it stays inside the finally block,
           this is checked using the exception block number }
         if (flowcontrol-[fc_gotolabel])<>(finallyexceptionstate.oldflowcontrol*[fc_inflowcontrol,fc_catching_exceptions]) then
           CGMessage(cg_e_control_flow_outside_finally);
         if codegenerror then
           exit;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

         { same level as before try, but this part is only executed if an exception occcurred
           -> always fc_in_flowcontrol }
         flowcontrol:=finallyexceptionstate.oldflowcontrol*[fc_catching_exceptions];
         include(flowcontrol,fc_inflowcontrol);
         if not assigned(third) then
           begin
             { the value should now be in the exception handler }
             reasonreg:=hlcg.getintregister(current_asmdata.CurrAsmList,osuinttype);
             hlcg.g_exception_reason_load(current_asmdata.CurrAsmList,osuinttype,osuinttype,excepttemps.reasonbuf,reasonreg);
             if implicitframe then
               begin
                 hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,0,reasonreg,endfinallylabel);
                 { finally code only needed to be executed on exception (-> in
                   if-branch -> fc_inflowcontrol) }
                 if current_procinfo.procdef.generate_safecall_wrapper then
                   begin
                     handle_safecall_exception;
                     { we have to jump immediatly as we have to return the value of FPC_SAFECALL }
                     hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
                   end
                 else
                   cexceptionstatehandler.handle_reraise(current_asmdata.CurrAsmList,excepttemps,finallyexceptionstate,exceptframekind);
                 { we have to load 0 into the execepttemp, else the program thinks an exception happended }
                 emit_jump_out_of_try_finally_frame(current_asmdata.CurrAsmList,0,finallyexceptionstate.exceptionlabel,excepttemps,exitfinallylabel);
               end
             else
               begin
                 handle_breakcontinueexit(finallyexceptionstate.exceptionlabel,true);
               end;
           end
         else
           begin
             if implicitframe then
               begin
                 if current_procinfo.procdef.generate_safecall_wrapper then
                   handle_safecall_exception
                 else
                   cexceptionstatehandler.handle_reraise(current_asmdata.CurrAsmList,excepttemps,finallyexceptionstate,exceptframekind);
               end
             else
               begin
                 cexceptionstatehandler.handle_reraise(current_asmdata.CurrAsmList,excepttemps,finallyexceptionstate,exceptframekind);
               end;

           end;
         cexceptionstatehandler.unget_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         hlcg.a_label(current_asmdata.CurrAsmList,endfinallylabel);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         current_procinfo.CurrExitLabel:=oldCurrExitLabel;
         if assigned(current_procinfo.CurrBreakLabel) then
          begin
            current_procinfo.CurrContinueLabel:=oldContinueLabel;
            current_procinfo.CurrBreakLabel:=oldBreakLabel;
          end;
         flowcontrol:=finallyexceptionstate.oldflowcontrol+(finallyexceptionstate.newflowcontrol-[fc_inflowcontrol,fc_catching_exceptions]);
      end;


    function tcgraisenode.pass_1: tnode;
      begin
        if not(tf_use_psabieh in target_info.flags) or assigned(left) then
          result:=inherited
        else
          begin
            expectloc:=LOC_VOID;
            result:=nil;
          end;
      end;

{$ifndef jvm}
    { has to be factored out as well }
    procedure tcgraisenode.pass_generate_code;
      var
        CurrentLandingPad, CurrentAction, ReRaiseLandingPad: TPSABIEHAction;
        psabiehprocinfo: tpsabiehprocinfo;
      begin
        if not(tf_use_psabieh in target_info.flags) then
          Internalerror(2019021701);

        location_reset(location,LOC_VOID,OS_NO);
        CurrentLandingPad:=nil;
        CurrentAction:=nil;
        ReRaiseLandingPad:=nil;
        psabiehprocinfo:=current_procinfo as tpsabiehprocinfo;
        { a reraise must raise the exception to the parent exception frame }
        if fc_catching_exceptions in flowcontrol then
          begin
            psabiehprocinfo.CreateNewPSABIEHCallsite(current_asmdata.CurrAsmList);
            CurrentLandingPad:=psabiehprocinfo.CurrentLandingPad;
            if psabiehprocinfo.PopLandingPad(CurrentLandingPad) then
              exclude(flowcontrol,fc_catching_exceptions);
            CurrentAction:=psabiehprocinfo.CurrentAction;
            psabiehprocinfo.FinalizeAndPopAction(CurrentAction);

            if not(fc_catching_exceptions in flowcontrol) then
              begin
                ReRaiseLandingPad:=psabiehprocinfo.NoAction;
                psabiehprocinfo.PushAction(ReRaiseLandingPad);
                psabiehprocinfo.PushLandingPad(ReRaiseLandingPad);
              end;
          end;
        hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_reraise',[],nil).resetiftemp;
        if assigned(CurrentLandingPad) then
          begin
            psabiehprocinfo.CreateNewPSABIEHCallsite(current_asmdata.CurrAsmList);
            if not(fc_catching_exceptions in flowcontrol) then
              begin
                psabiehprocinfo.PopLandingPad(psabiehprocinfo.CurrentLandingPad);
                psabiehprocinfo.PopAction(ReRaiseLandingPad);
              end;

            psabiehprocinfo.PushAction(CurrentAction);
            psabiehprocinfo.PushLandingPad(CurrentLandingPad);
            include(flowcontrol,fc_catching_exceptions);
          end;
      end;
{$endif jvm}


begin
   cwhilerepeatnode:=tcgwhilerepeatnode;
   cifnode:=tcgifnode;
   cfornode:=tcgfornode;
   cexitnode:=tcgexitnode;
   cbreaknode:=tcgbreaknode;
   ccontinuenode:=tcgcontinuenode;
   cgotonode:=tcggotonode;
   clabelnode:=tcglabelnode;
   craisenode:=tcgraisenode;
   ctryexceptnode:=tcgtryexceptnode;
   ctryfinallynode:=tcgtryfinallynode;
   connode:=tcgonnode;
   cexceptionstatehandler:=tcgexceptionstatehandler;
end.

