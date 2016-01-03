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
      aasmbase,node,nflw,ncgutil;

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
          usedregvars: tusedregvars;

          procedure pass_generate_code;override;
          procedure sync_regvars(checkusedregvars: boolean);
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
       private
          asmlabel : tasmlabel;
       public
          function getasmlabel : tasmlabel;
          procedure pass_generate_code;override;
       end;

       tcgraisenode = class(traisenode)
       end;

       tcgtryexceptnode = class(ttryexceptnode)
          procedure pass_generate_code;override;
       end;

       tcgtryfinallynode = class(ttryfinallynode)
          procedure handle_safecall_exception;
          procedure pass_generate_code;override;
       end;

       tcgonnode = class(tonnode)
          procedure pass_generate_code;override;
       end;

implementation

    uses
      verbose,globals,systems,globtype,constexp,
      symconst,symdef,symsym,symtable,aasmtai,aasmdata,aasmcpu,defutil,
      procinfo,cgbase,pass_2,parabase,
      cpubase,ncon,
      tgobj,paramgr,
      cgutils,cgobj,hlcgobj,nutils
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
         oldexecutionweight : longint;
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
            { align loop target }
            current_asmdata.CurrAsmList.concat(Tai_align.Create(current_settings.alignment.loopalign));

         hlcg.a_label(current_asmdata.CurrAsmList,lloop);

         current_procinfo.CurrContinueLabel:=lcont;
         current_procinfo.CurrBreakLabel:=lbreak;

         if assigned(right) then
           begin
             { calc register weight }
             oldexecutionweight:=cg.executionweight;
             cg.executionweight:=cg.executionweight*8;
             secondpass(right);
             cg.executionweight:=oldexecutionweight;
           end;

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
         oldexecutionweight : longint;
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
         { determines registers weigths }
         oldexecutionweight:=cg.executionweight;
         cg.executionweight:=cg.executionweight div 2;
         if cg.executionweight<1 then
           cg.executionweight:=1;

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
              hlcg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
           end;
         if not(assigned(right)) then
           begin
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

         cg.executionweight:=oldexecutionweight;

         flowcontrol := oldflowcontrol + (flowcontrol - [fc_inflowcontrol]);
      end;


{*****************************************************************************
                              SecondFor
*****************************************************************************}

    procedure tcgfornode.sync_regvars(checkusedregvars: boolean);
      begin
         if (cs_opt_regvar in current_settings.optimizerswitches) and
            not(pi_has_label in current_procinfo.flags) then
           begin
             if checkusedregvars then
               begin
                 usedregvars.intregvars.init;
                 usedregvars.fpuregvars.init;
                 usedregvars.mmregvars.init;

                 { We have to synchronise the loop variable and loop body. }
                 { The loop end is not necessary, unless it's a register   }
                 { variable. The start value also doesn't matter.          }

                 { loop var }
                 get_used_regvars(left,usedregvars);
                 { loop body }
                 get_used_regvars(t2,usedregvars);
<<<<<<< HEAD
<<<<<<< HEAD
                 { end value can't be a regvar, but may be a temp in register }
                 get_used_regvars(t1,usedregvars);
=======
                 { end value (t1) is not necessary (it cannot be a regvar, }
                 { see webtbs/tw8883)                                      }
>>>>>>> graemeg/fixes_2_2
=======
                 { end value (t1) is not necessary (it cannot be a regvar, }
                 { see webtbs/tw8883)                                      }
>>>>>>> origin/fixes_2_2

                 gen_sync_regvars(current_asmdata.CurrAsmList,usedregvars);
               end
             else
               begin
                 gen_sync_regvars(current_asmdata.CurrAsmList,usedregvars);
                 usedregvars.intregvars.done;
                 usedregvars.fpuregvars.done;
                 usedregvars.mmregvars.done;
               end;
           end;
      end;


    procedure tcgfornode.pass_generate_code;
      var
         l3,oldclabel,oldblabel,
         otl, ofl : tasmlabel;
         temptovalue : boolean;
         hop : topcg;
         hcond : topcmp;
         opsize : tcgsize;
         count_var_is_signed,do_loopvar_at_end : boolean;
         cmp_const:Tconstexprint;
         oldflowcontrol : tflowcontrol;
         oldexecutionweight : longint;
         isjump: boolean;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         oldclabel:=current_procinfo.CurrContinueLabel;
         oldblabel:=current_procinfo.CurrBreakLabel;
         current_asmdata.getjumplabel(current_procinfo.CurrContinueLabel);
         current_asmdata.getjumplabel(current_procinfo.CurrBreakLabel);
         current_asmdata.getjumplabel(l3);

         { only calculate reference }
         opsize := def_cgsize(left.resultdef);
         count_var_is_signed:=is_signed(left.resultdef);

         { first set the to value
           because the count var can be in the expression ! }
         do_loopvar_at_end:=(lnf_dont_mind_loopvar_on_exit in loopflags)
         { if the loop is unrolled and there is a jump into the loop,
           then we can't do the trick with incrementing the loop var only at the
           end
         }
           and not(assigned(entrylabel));

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        secondpass(t1);
        if t1.location.loc in [LOC_FLAGS,LOC_JUMP] then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,t1.location,t1.resultdef,t1.resultdef,false);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        isjump:=(t1.expectloc=LOC_JUMP);
        if isjump then
          begin
             otl:=current_procinfo.CurrTrueLabel;
             current_asmdata.getjumplabel(current_procinfo.CurrTrueLabel);
             ofl:=current_procinfo.CurrFalseLabel;
             current_asmdata.getjumplabel(current_procinfo.CurrFalseLabel);
          end;
        secondpass(t1);
        if t1.location.loc in [LOC_FLAGS,LOC_JUMP] then
          location_force_reg(current_asmdata.CurrAsmList,t1.location,def_cgsize(t1.resultdef),false);
        if isjump then
          begin
            current_procinfo.CurrTrueLabel:=otl;
            current_procinfo.CurrFalseLabel:=ofl;
          end;
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
         { calculate pointer value and check if changeable and if so }
         { load into temporary variable                       }
         if t1.nodetype<>ordconstn then
           begin
              do_loopvar_at_end:=false;
<<<<<<< HEAD
<<<<<<< HEAD
=======
              location_force_reg(current_asmdata.CurrAsmList,t1.location,t1.location.size,false);
>>>>>>> graemeg/fixes_2_2
=======
              location_force_reg(current_asmdata.CurrAsmList,t1.location,t1.location.size,false);
>>>>>>> origin/fixes_2_2
              temptovalue:=true;
           end
         else
           temptovalue:=false;

         { load loopvar, prefer loopvar being a register variable }
         oldexecutionweight:=cg.executionweight;
         inc(cg.executionweight,8);
         secondpass(left);
         cg.executionweight:=oldexecutionweight;

         { load from value }
         isjump:=(right.expectloc=LOC_JUMP);
         if isjump then
           begin
              otl:=current_procinfo.CurrTrueLabel;
              current_asmdata.getjumplabel(current_procinfo.CurrTrueLabel);
              ofl:=current_procinfo.CurrFalseLabel;
              current_asmdata.getjumplabel(current_procinfo.CurrFalseLabel);
           end;
         secondpass(right);
         if right.location.loc in [LOC_FLAGS,LOC_JUMP] then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
           location_force_reg(current_asmdata.CurrAsmList,right.location,def_cgsize(right.resultdef),false);
         if isjump then
           begin
             current_procinfo.CurrTrueLabel:=otl;
             current_procinfo.CurrFalseLabel:=ofl;
           end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

         hlcg.maybe_change_load_node_reg(current_asmdata.CurrAsmList,left,false);
=======

         maybechangeloadnodereg(current_asmdata.CurrAsmList,left,false);
>>>>>>> origin/cpstrnew
         oldflowcontrol:=flowcontrol;
         include(flowcontrol,fc_inflowcontrol);
         exclude(flowcontrol,fc_unwind_loop);
         { produce start assignment }
         case left.location.loc of
           LOC_REFERENCE,
           LOC_CREFERENCE :
             hlcg.a_load_loc_ref(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location,left.location.reference);
           LOC_REGISTER,
           LOC_CREGISTER:
             hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location,left.location.register);
           LOC_SUBSETREG,
           LOC_CSUBSETREG :
             hlcg.a_load_loc_subsetreg(current_asmdata.CurrAsmList,right.resultdef,left.resultdef,right.location,left.location.sreg);
           else
             internalerror(200501311);
         end;

         if lnf_backward in loopflags then
           if count_var_is_signed then
             hcond:=OC_LT
           else
             hcond:=OC_B
         else
           if count_var_is_signed then
             hcond:=OC_GT
           else
             hcond:=OC_A;

         sync_regvars(true);
{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}

         if temptovalue then
           begin
             case t1.location.loc of
               LOC_REGISTER,LOC_CREGISTER:
                 hlcg.a_cmp_reg_loc_label(current_asmdata.CurrAsmList,left.resultdef,hcond,
                   t1.location.register,left.location,current_procinfo.CurrBreakLabel);
               LOC_REFERENCE,LOC_CREFERENCE:
                 hlcg.a_cmp_ref_loc_label(current_asmdata.CurrAsmList,left.resultdef,hcond,
                   t1.location.reference,left.location,current_procinfo.CurrBreakLabel);
             else
               InternalError(2013051601);
             end;
           end
         else
           begin
             if lnf_testatbegin in loopflags then
               begin
                 hlcg.a_cmp_const_loc_label(current_asmdata.CurrAsmList,left.resultdef,hcond,
                   tordconstnode(t1).value.svalue,
                   left.location,current_procinfo.CurrBreakLabel);
               end;
           end;

         {If the loopvar doesn't mind on exit, we avoid this ugly
          dec instruction and do the loopvar inc/dec after the loop
          body.}
         if not do_loopvar_at_end then
            begin
              if lnf_backward in loopflags then
                hop:=OP_ADD
              else
                hop:=OP_SUB;
              hlcg.a_op_const_loc(current_asmdata.CurrAsmList,hop,left.resultdef,1,left.location);
            end;

         if assigned(entrylabel) then
           hlcg.a_jmp_always(current_asmdata.CurrAsmList,tcglabelnode(entrylabel).getasmlabel);

         { align loop target }
         if not(cs_opt_size in current_settings.optimizerswitches) then
            current_asmdata.CurrAsmList.concat(Tai_align.Create(current_settings.alignment.loopalign));
         hlcg.a_label(current_asmdata.CurrAsmList,l3);

         {If the loopvar doesn't mind on exit, we avoid the loopvar inc/dec
          after the loop body instead of here.}
         if not do_loopvar_at_end then
            begin
              { according to count direction DEC or INC... }
              if lnf_backward in loopflags then
                hop:=OP_SUB
              else
                hop:=OP_ADD;
              hlcg.a_op_const_loc(current_asmdata.CurrAsmList,hop,left.resultdef,1,left.location);
            end;

         if assigned(t2) then
           begin
             { Calc register weight }
             oldexecutionweight:=cg.executionweight;
             cg.executionweight:=cg.executionweight*8;
             secondpass(t2);
             cg.executionweight:=oldexecutionweight;
{$ifdef OLDREGVARS}
             load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
           end;

         {If the loopvar doesn't mind on exit, we do the loopvar inc/dec
          after the loop body instead of here.}
         if do_loopvar_at_end then
            begin
              { according to count direction DEC or INC... }
              if lnf_backward in loopflags then
                hop:=OP_SUB
              else
                hop:=OP_ADD;
              hlcg.a_op_const_loc(current_asmdata.CurrAsmList,hop,left.resultdef,1,left.location);
            end;

         hlcg.a_label(current_asmdata.CurrAsmList,current_procinfo.CurrContinueLabel);

         if do_loopvar_at_end then
           if lnf_backward in loopflags then
             if count_var_is_signed then
               hcond:=OC_GTE
             else
               hcond:=OC_AE
            else
              if count_var_is_signed then
                hcond:=OC_LTE
              else
                hcond:=OC_BE
         else
           if lnf_backward in loopflags then
             if count_var_is_signed then
               hcond:=OC_GT
             else
               hcond:=OC_A
            else
              if count_var_is_signed then
                hcond:=OC_LT
              else
                hcond:=OC_B;
{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}

         { produce comparison and the corresponding }
         { jump                                     }
         if temptovalue then
           begin
             case t1.location.loc of
               LOC_REGISTER,LOC_CREGISTER:
                 hlcg.a_cmp_reg_loc_label(current_asmdata.CurrAsmList,left.resultdef,hcond,t1.location.register,
                   left.location,l3);
               LOC_REFERENCE,LOC_CREFERENCE:
                 hlcg.a_cmp_ref_loc_label(current_asmdata.CurrAsmList,left.resultdef,hcond,t1.location.reference,
                   left.location,l3);
             else
               InternalError(2013051602);
             end;
           end
         else
           begin
             cmp_const:=Tordconstnode(t1).value;
             if do_loopvar_at_end then
               begin
                 {Watch out for wrap around 255 -> 0.}
                 {Ugly: This code is way to long... Use tables?}
                 case opsize of
                   OS_8:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if byte(cmp_const.svalue)=low(byte) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=high(byte);
                             end
                         end
                       else
                         begin
                           if byte(cmp_const.svalue)=high(byte) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=low(byte);
                             end
                         end
                     end;
                   OS_16:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if word(cmp_const.svalue)=high(word) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=low(word);
                             end
                         end
                       else
                         begin
                           if word(cmp_const.svalue)=low(word) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=high(word);
                             end
                         end
                     end;
                   OS_32:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if cardinal(cmp_const.svalue)=high(cardinal) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=low(cardinal);
                             end
                         end
                       else
                         begin
                           if cardinal(cmp_const.svalue)=low(cardinal) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=high(cardinal);
                             end
                         end
                     end;
                   OS_64:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if qword(cmp_const.uvalue)=high(qword) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=low(qword);
                             end
                         end
                       else
                         begin
                           if qword(cmp_const.uvalue)=low(qword) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=high(qword);
                             end
                         end
                     end;
                   OS_S8:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if shortint(cmp_const.svalue)=low(shortint) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=high(shortint);
                             end
                         end
                       else
                         begin
                           if shortint(cmp_const.svalue)=high(shortint) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=int64(low(shortint));
                             end
                         end
                     end;
                   OS_S16:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if integer(cmp_const.svalue)=high(smallint) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=int64(low(smallint));
                             end
                         end
                       else
                         begin
                           if integer(cmp_const.svalue)=low(smallint) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=int64(high(smallint));
                             end
                         end
                     end;
                   OS_S32:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if longint(cmp_const.svalue)=high(longint) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=int64(low(longint));
                             end
                         end
                       else
                         begin
                           if longint(cmp_const.svalue)=low(longint) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=int64(high(longint));
                             end
                         end
                     end;
                   OS_S64:
                     begin
                       if lnf_backward in loopflags then
                         begin
                           if int64(cmp_const.svalue)=high(int64) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=low(int64);
                             end
                         end
                       else
                         begin
                           if int64(cmp_const.svalue)=low(int64) then
                             begin
                               hcond:=OC_NE;
                               cmp_const:=high(int64);
                             end
                         end
                     end;
                   else
                     internalerror(200201021);
                 end;
               end;

             hlcg.a_cmp_const_loc_label(current_asmdata.CurrAsmList,left.resultdef,hcond,
               tcgint(cmp_const.svalue),left.location,l3);
           end;

         { this is the break label: }
         hlcg.a_label(current_asmdata.CurrAsmList,current_procinfo.CurrBreakLabel);

         sync_regvars(false);

         current_procinfo.CurrContinueLabel:=oldclabel;
         current_procinfo.CurrBreakLabel:=oldblabel;
         { a break/continue in a while/repeat block can't be seen outside }
         flowcontrol:=oldflowcontrol+(flowcontrol-[fc_break,fc_continue,fc_inflowcontrol]);
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
<<<<<<< HEAD
         hlcg.a_label(current_asmdata.CurrAsmList,getasmlabel);
=======
         cg.a_label(current_asmdata.CurrAsmList,getasmlabel);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

         { Write also extra label if this label was referenced from
           assembler block }
         if assigned(labsym) and
            assigned(labsym.asmblocklabel) then
<<<<<<< HEAD
<<<<<<< HEAD
           hlcg.a_label(current_asmdata.CurrAsmList,labsym.asmblocklabel);
=======
           cg.a_label(current_asmdata.CurrAsmList,labsym.asmblocklabel);
>>>>>>> graemeg/fixes_2_2
=======
           cg.a_label(current_asmdata.CurrAsmList,labsym.asmblocklabel);
>>>>>>> origin/fixes_2_2

         secondpass(left);
      end;


{*****************************************************************************
<<<<<<< HEAD
=======
                             SecondRaise
*****************************************************************************}

    procedure tcgraisenode.pass_generate_code;

      var
         a : tasmlabel;
         href2: treference;
         paraloc1,paraloc2,paraloc3 : tcgpara;
      begin
         paraloc1.init;
         paraloc2.init;
         paraloc3.init;
         paramanager.getintparaloc(pocall_default,1,paraloc1);
         paramanager.getintparaloc(pocall_default,2,paraloc2);
         paramanager.getintparaloc(pocall_default,3,paraloc3);
         location_reset(location,LOC_VOID,OS_NO);

         if assigned(left) then
           begin
              { multiple parameters? }
              if assigned(right) then
                begin
                  { frame tree }
                  if assigned(third) then
                    secondpass(third);
                  secondpass(right);
                end;
              secondpass(left);
              if codegenerror then
                exit;

              { Push parameters }
              if assigned(right) then
                begin
                  { frame tree }
                  if assigned(third) then
                    cg.a_load_loc_cgpara(current_asmdata.CurrAsmList,third.location,paraloc3)
                  else
                    cg.a_load_const_cgpara(current_asmdata.CurrAsmList,OS_INT,0,paraloc3);
                  { push address }
                  cg.a_load_loc_cgpara(current_asmdata.CurrAsmList,right.location,paraloc2);
                end
              else
                begin
                   { get current address }
                   current_asmdata.getaddrlabel(a);
                   cg.a_label(current_asmdata.CurrAsmList,a);
                   reference_reset_symbol(href2,a,0,1);
                   { push current frame }
                   cg.a_load_reg_cgpara(current_asmdata.CurrAsmList,OS_ADDR,NR_FRAME_POINTER_REG,paraloc3);
                   { push current address }
                   if target_info.system <> system_powerpc_macos then
                     cg.a_loadaddr_ref_cgpara(current_asmdata.CurrAsmList,href2,paraloc2)
                   else
                     cg.a_load_const_cgpara(current_asmdata.CurrAsmList,OS_INT,0,paraloc2);
                end;
              cg.a_load_loc_cgpara(current_asmdata.CurrAsmList,left.location,paraloc1);
              paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
              paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc2);
              paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc3);
              cg.allocallcpuregisters(current_asmdata.CurrAsmList);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_RAISEEXCEPTION',false);
              cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
           end
         else
           begin
              cg.allocallcpuregisters(current_asmdata.CurrAsmList);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_POPADDRSTACK',false);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_RERAISE',false);
              cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
           end;
         paraloc1.done;
         paraloc2.done;
         paraloc3.done;
       end;


{*****************************************************************************
>>>>>>> graemeg/cpstrnew
                             SecondTryExcept
*****************************************************************************}

    var
       endexceptlabel : tasmlabel;


    { does the necessary things to clean up the object stack }
    { in the except block                                    }
    procedure cleanupobjectstack;
      begin
         hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_doneexception',[],nil);
      end;

    { generates code to be executed when another exeception is raised while
      control is inside except block }
    procedure handle_nested_exception(list:TAsmList;const t:texceptiontemps;entrylabel:TAsmLabel);
      var
         exitlabel: tasmlabel;
      begin
<<<<<<< HEAD
         { don't generate line info for internal cleanup }
         list.concat(tai_marker.create(mark_NoLineInfoStart));
         current_asmdata.getjumplabel(exitlabel);
         hlcg.a_label(list,entrylabel);
         free_exception(list,t,0,exitlabel,false);
         { we don't need to save/restore registers here because reraise never }
         { returns                                                            }
         hlcg.g_call_system_proc(list,'fpc_raise_nested',[],nil);
         hlcg.a_label(list,exitlabel);
         cleanupobjectstack;
=======
         cg.allocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_POPOBJECTSTACK',false);
         cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
         paraloc1.init;
         paramanager.getintparaloc(pocall_default,1,paraloc1);
         cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
         cg.a_load_reg_cgpara(current_asmdata.CurrAsmList,OS_ADDR,NR_FUNCTION_RESULT_REG,paraloc1);
         paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
         cg.allocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DESTROYEXCEPTION',false);
         cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
         paraloc1.done;
>>>>>>> graemeg/cpstrnew
      end;


    procedure tcgtryexceptnode.pass_generate_code;

      var
         exceptlabel,doexceptlabel,oldendexceptlabel,
         lastonlabel,
         exitexceptlabel,
         continueexceptlabel,
         breakexceptlabel,
         exittrylabel,
         continuetrylabel,
         breaktrylabel,
         doobjectdestroyandreraise,
         oldCurrExitLabel,
         oldContinueLabel,
         oldBreakLabel : tasmlabel;
         oldflowcontrol,tryflowcontrol,
         exceptflowcontrol : tflowcontrol;
         destroytemps,
         excepttemps : texceptiontemps;
      label
         errorexit;
      begin
         location_reset(location,LOC_VOID,OS_NO);
         exceptflowcontrol:=[];
         continuetrylabel:=nil;
         breaktrylabel:=nil;
         continueexceptlabel:=nil;
         breakexceptlabel:=nil;

         oldflowcontrol:=flowcontrol;
         flowcontrol:=[fc_inflowcontrol];
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

         current_asmdata.getjumplabel(exceptlabel);
         current_asmdata.getjumplabel(doexceptlabel);
         current_asmdata.getjumplabel(endexceptlabel);
         current_asmdata.getjumplabel(lastonlabel);

         get_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         new_exception(current_asmdata.CurrAsmList,excepttemps,exceptlabel);

         { try block }
         { set control flow labels for the try block }
         current_procinfo.CurrExitLabel:=exittrylabel;
         if assigned(oldBreakLabel) then
          begin
            current_procinfo.CurrContinueLabel:=continuetrylabel;
            current_procinfo.CurrBreakLabel:=breaktrylabel;
          end;

         flowcontrol:=[fc_inflowcontrol];
         secondpass(left);
         tryflowcontrol:=flowcontrol;
         if codegenerror then
           goto errorexit;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hlcg.a_label(current_asmdata.CurrAsmList,exceptlabel);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         cg.a_label(current_asmdata.CurrAsmList,exceptlabel);
>>>>>>> graemeg/cpstrnew

         free_exception(current_asmdata.CurrAsmList, excepttemps, 0, endexceptlabel, false);

         hlcg.a_label(current_asmdata.CurrAsmList,doexceptlabel);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

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

         flowcontrol:=[fc_inflowcontrol];
         { on statements }
         if assigned(right) then
           secondpass(right);

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hlcg.a_label(current_asmdata.CurrAsmList,lastonlabel);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         cg.a_label(current_asmdata.CurrAsmList,lastonlabel);
>>>>>>> graemeg/cpstrnew
         { default handling except handling }
         if assigned(t1) then
           begin
              { FPC_CATCHES with 'default handler' flag (=-1) need no longer be called,
                it doesn't change any state and its return value is ignored (Sergei)
              }
<<<<<<< HEAD
=======
              paraloc1.init;
              paramanager.getintparaloc(pocall_default,1,paraloc1);
              cg.a_load_const_cgpara(current_asmdata.CurrAsmList,OS_ADDR,-1,paraloc1);
              paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
              cg.allocallcpuregisters(current_asmdata.CurrAsmList);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_CATCHES',false);
              cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
              paraloc1.done;
>>>>>>> graemeg/cpstrnew

              { the destruction of the exception object must be also }
              { guarded by an exception frame, but it can be omitted }
              { if there's no user code in 'except' block            }

              if not (has_no_code(t1)) then
               begin
                 current_asmdata.getjumplabel(doobjectdestroyandreraise);

                 get_exception_temps(current_asmdata.CurrAsmList,destroytemps);
                 new_exception(current_asmdata.CurrAsmList,destroytemps,doobjectdestroyandreraise);

<<<<<<< HEAD
                 { except block needs line info }
                 current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

                 { here we don't have to reset flowcontrol           }
                 { the default and on flowcontrols are handled equal }
                 secondpass(t1);
                 exceptflowcontrol:=flowcontrol;

                 handle_nested_exception(current_asmdata.CurrAsmList,destroytemps,doobjectdestroyandreraise);

                 unget_exception_temps(current_asmdata.CurrAsmList,destroytemps);
                 hlcg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);
               end
               else
                 begin
                   exceptflowcontrol:=flowcontrol;
                   cleanupobjectstack;
                   hlcg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);
                 end;
=======
              cg.a_label(current_asmdata.CurrAsmList,doobjectdestroyandreraise);

              free_exception(current_asmdata.CurrAsmList,destroytemps,0,doobjectdestroy,false);

              cg.allocallcpuregisters(current_asmdata.CurrAsmList);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_POPSECONDOBJECTSTACK',false);
              cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);

              paraloc1.init;
              paramanager.getintparaloc(pocall_default,1,paraloc1);
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
              cg.a_load_reg_cgpara(current_asmdata.CurrAsmList, OS_ADDR, NR_FUNCTION_RESULT_REG, paraloc1);
              paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
              cg.allocallcpuregisters(current_asmdata.CurrAsmList);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DESTROYEXCEPTION',false);
              cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
              paraloc1.done;
              { we don't need to restore esi here because reraise never }
              { returns                                                 }
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_RERAISE',false);

              cg.a_label(current_asmdata.CurrAsmList,doobjectdestroy);
              cleanupobjectstack;
              unget_exception_temps(current_asmdata.CurrAsmList,destroytemps);
              cg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);
>>>>>>> graemeg/cpstrnew
           end
         else
           begin
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_reraise',[],nil);
              exceptflowcontrol:=flowcontrol;
           end;

         if fc_exit in exceptflowcontrol then
           begin
              { do some magic for exit in the try block }
              hlcg.a_label(current_asmdata.CurrAsmList,exitexceptlabel);
              { we must also destroy the address frame which guards }
              { exception object                                    }
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_popaddrstack',[],nil);
              hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
              cleanupobjectstack;
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
           end;

         if fc_break in exceptflowcontrol then
           begin
              hlcg.a_label(current_asmdata.CurrAsmList,breakexceptlabel);
              { we must also destroy the address frame which guards }
              { exception object                                    }
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_popaddrstack',[],nil);
              hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
              cleanupobjectstack;
              cg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
           end;

         if fc_continue in exceptflowcontrol then
           begin
              hlcg.a_label(current_asmdata.CurrAsmList,continueexceptlabel);
              { we must also destroy the address frame which guards }
              { exception object                                    }
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_popaddrstack',[],nil);
              hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
              cleanupobjectstack;
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
           end;

         if fc_exit in tryflowcontrol then
           begin
              { do some magic for exit in the try block }
              hlcg.a_label(current_asmdata.CurrAsmList,exittrylabel);
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_popaddrstack',[],nil);
              hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
           end;

         if fc_break in tryflowcontrol then
           begin
              hlcg.a_label(current_asmdata.CurrAsmList,breaktrylabel);
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_popaddrstack',[],nil);
              hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
              cg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
           end;

         if fc_continue in tryflowcontrol then
           begin
              hlcg.a_label(current_asmdata.CurrAsmList,continuetrylabel);
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_popaddrstack',[],nil);
              hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
              cg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
           end;
         unget_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         hlcg.a_label(current_asmdata.CurrAsmList,endexceptlabel);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

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
         flowcontrol:=oldflowcontrol+(exceptflowcontrol +
           tryflowcontrol - [fc_inflowcontrol]);
      end;


    procedure tcgonnode.pass_generate_code;
      var
         nextonlabel,
         exitonlabel,
         continueonlabel,
         breakonlabel,
         oldCurrExitLabel,
         oldContinueLabel,
         doobjectdestroyandreraise,
         oldBreakLabel : tasmlabel;
         oldflowcontrol : tflowcontrol;
         excepttemps : texceptiontemps;
         href2: treference;
         paraloc1 : tcgpara;
         exceptvarsym : tlocalvarsym;
         pd : tprocdef;
         fpc_catches_res: TCGPara;
         fpc_catches_resloc: tlocation;
      begin
         paraloc1.init;
         location_reset(location,LOC_VOID,OS_NO);
         oldCurrExitLabel:=nil;
         continueonlabel:=nil;
         breakonlabel:=nil;
         exitonlabel:=nil;

         oldflowcontrol:=flowcontrol;
         flowcontrol:=[fc_inflowcontrol];
         current_asmdata.getjumplabel(nextonlabel);

         { send the vmt parameter }
<<<<<<< HEAD
         pd:=search_system_proc('fpc_catches');
         reference_reset_symbol(href2,current_asmdata.RefAsmSymbol(excepttype.vmt_mangledname,AT_DATA),0,sizeof(pint));
         paramanager.getintparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);
         hlcg.a_loadaddr_ref_cgpara(current_asmdata.CurrAsmList,excepttype.vmt_def,href2,paraloc1);
         paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
         fpc_catches_res:=hlcg.g_call_system_proc(current_asmdata.CurrAsmList,pd,[@paraloc1],nil);
         location_reset(fpc_catches_resloc,LOC_REGISTER,def_cgsize(fpc_catches_res.def));
         fpc_catches_resloc.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,fpc_catches_res.def);
         hlcg.gen_load_cgpara_loc(current_asmdata.CurrAsmList,fpc_catches_res.def,fpc_catches_res,fpc_catches_resloc,true);

=======
         reference_reset_symbol(href2,current_asmdata.RefAsmSymbol(excepttype.vmt_mangledname),0,sizeof(pint));
         paramanager.getintparaloc(pocall_default,1,paraloc1);
         cg.a_loadaddr_ref_cgpara(current_asmdata.CurrAsmList,href2,paraloc1);
         paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
         cg.allocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_CATCHES',false);
         cg.deallocallcpuregisters(current_asmdata.CurrAsmList);

         cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
>>>>>>> graemeg/cpstrnew
         { is it this catch? No. go to next onlabel }
         hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,fpc_catches_res.def,OC_EQ,0,fpc_catches_resloc.register,nextonlabel);

         { Retrieve exception variable }
         if assigned(excepTSymtable) then
           exceptvarsym:=tlocalvarsym(excepTSymtable.SymList[0])
         else
           internalerror(2011020401);

         if assigned(exceptvarsym) then
           begin
             location_reset_ref(exceptvarsym.localloc,LOC_REFERENCE,def_cgsize(voidpointertype),voidpointertype.alignment);
             tg.GetLocal(current_asmdata.CurrAsmList,exceptvarsym.vardef.size,exceptvarsym.vardef,exceptvarsym.localloc.reference);
             hlcg.a_load_reg_ref(current_asmdata.CurrAsmList,fpc_catches_res.def,exceptvarsym.vardef,fpc_catches_resloc.register,exceptvarsym.localloc.reference);
           end;

         { in the case that another exception is risen
           we've to destroy the old one                }
         current_asmdata.getjumplabel(doobjectdestroyandreraise);

         { call setjmp, and jump to finally label on non-zero result }
         get_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         new_exception(current_asmdata.CurrAsmList,excepttemps,doobjectdestroyandreraise);

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         handle_nested_exception(current_asmdata.CurrAsmList,excepttemps,doobjectdestroyandreraise);

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         { don't generate lineinfo for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

         current_asmdata.getjumplabel(doobjectdestroy);
         cg.a_label(current_asmdata.CurrAsmList,doobjectdestroyandreraise);

         free_exception(current_asmdata.CurrAsmList,excepttemps,0,doobjectdestroy,false);

         cg.allocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_POPSECONDOBJECTSTACK',false);
         cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
         paramanager.getintparaloc(pocall_default,1,paraloc1);
         cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
         cg.a_load_reg_cgpara(current_asmdata.CurrAsmList, OS_ADDR, NR_FUNCTION_RESULT_REG, paraloc1);
         paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
         cg.allocallcpuregisters(current_asmdata.CurrAsmList);
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DESTROYEXCEPTION',false);
         cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
         { we don't need to store/restore registers here because reraise never
           returns                                                             }
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_RERAISE',false);

         cg.a_label(current_asmdata.CurrAsmList,doobjectdestroy);
         cleanupobjectstack;
>>>>>>> graemeg/cpstrnew
         { clear some stuff }
         if assigned(exceptvarsym) then
           begin
             tg.UngetLocal(current_asmdata.CurrAsmList,exceptvarsym.localloc.reference);
             exceptvarsym.localloc.loc:=LOC_INVALID;
           end;
         cg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);

         if assigned(right) then
           begin
              { special handling for control flow instructions }
              if fc_exit in flowcontrol then
                begin
                   { the address and object pop does secondtryexcept }
                   cg.a_label(current_asmdata.CurrAsmList,exitonlabel);
                   cg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
                end;

              if fc_break in flowcontrol then
                begin
                   { the address and object pop does secondtryexcept }
                   cg.a_label(current_asmdata.CurrAsmList,breakonlabel);
                   cg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
                end;

              if fc_continue in flowcontrol then
                begin
                   { the address and object pop does secondtryexcept }
                   cg.a_label(current_asmdata.CurrAsmList,continueonlabel);
                   cg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
                end;

              current_procinfo.CurrExitLabel:=oldCurrExitLabel;
              if assigned(oldBreakLabel) then
               begin
                 current_procinfo.CurrContinueLabel:=oldContinueLabel;
                 current_procinfo.CurrBreakLabel:=oldBreakLabel;
               end;
           end;

         unget_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         cg.a_label(current_asmdata.CurrAsmList,nextonlabel);
         flowcontrol:=oldflowcontrol+(flowcontrol-[fc_inflowcontrol]);
         paraloc1.done;
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { next on node }
         if assigned(left) then
           secondpass(left);
      end;

{*****************************************************************************
                             SecondTryFinally
*****************************************************************************}

    procedure tcgtryfinallynode.handle_safecall_exception;
      var
        cgpara: tcgpara;
        selfsym: tparavarsym;
        pd: tprocdef;
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
            cg.a_load_loc_cgpara(current_asmdata.CurrAsmList,selfsym.localloc,cgpara);
          end
        else
          cg.a_load_const_cgpara(current_asmdata.CurrAsmList,OS_ADDR,0,cgpara);
        paramanager.freecgpara(current_asmdata.CurrAsmList,cgpara);
        cgpara.done;
        cg.g_call(current_asmdata.CurrAsmList,'FPC_SAFECALLHANDLER');
        cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,NR_FUNCTION_RESULT_REG, NR_FUNCTION_RETURN_REG);
      end;

    procedure tcgtryfinallynode.pass_generate_code;
      var
         finallylabel,
         endfinallylabel,
         exitfinallylabel,
         continuefinallylabel,
         breakfinallylabel,
         oldCurrExitLabel,
         oldContinueLabel,
         oldBreakLabel : tasmlabel;
         oldflowcontrol,tryflowcontrol : tflowcontrol;
         excepttemps : texceptiontemps;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         reasonreg : tregister;
=======
         retsym: tlocalvarsym;
>>>>>>> graemeg/cpstrnew
=======
         retsym: tlocalvarsym;
>>>>>>> graemeg/cpstrnew
=======
         retsym: tlocalvarsym;
>>>>>>> graemeg/cpstrnew
=======
         retsym: tlocalvarsym;
>>>>>>> origin/cpstrnew
      begin
         location_reset(location,LOC_VOID,OS_NO);
         tryflowcontrol:=[];
         oldBreakLabel:=nil;
         oldContinueLabel:=nil;
         continuefinallylabel:=nil;
         breakfinallylabel:=nil;

         { check if child nodes do a break/continue/exit }
         oldflowcontrol:=flowcontrol;
         flowcontrol:=[fc_inflowcontrol];
         current_asmdata.getjumplabel(finallylabel);
         current_asmdata.getjumplabel(endfinallylabel);

         { the finally block must catch break, continue and exit }
         { statements                                            }
         oldCurrExitLabel:=current_procinfo.CurrExitLabel;
         if implicitframe then
           exitfinallylabel:=finallylabel
         else
           current_asmdata.getjumplabel(exitfinallylabel);
         current_procinfo.CurrExitLabel:=exitfinallylabel;
         if assigned(current_procinfo.CurrBreakLabel) then
          begin
            oldContinueLabel:=current_procinfo.CurrContinueLabel;
            oldBreakLabel:=current_procinfo.CurrBreakLabel;
            if implicitframe then
              begin
                breakfinallylabel:=finallylabel;
                continuefinallylabel:=finallylabel;
              end
            else
              begin
                current_asmdata.getjumplabel(breakfinallylabel);
                current_asmdata.getjumplabel(continuefinallylabel);
              end;
            current_procinfo.CurrContinueLabel:=continuefinallylabel;
            current_procinfo.CurrBreakLabel:=breakfinallylabel;
          end;

         { call setjmp, and jump to finally label on non-zero result }
         get_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         new_exception(current_asmdata.CurrAsmList,excepttemps,finallylabel);

         { try code }
         if assigned(left) then
           begin
              secondpass(left);
              tryflowcontrol:=flowcontrol;
              if codegenerror then
                exit;
           end;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hlcg.a_label(current_asmdata.CurrAsmList,finallylabel);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         cg.a_label(current_asmdata.CurrAsmList,finallylabel);
>>>>>>> graemeg/cpstrnew
         { just free the frame information }
         free_exception(current_asmdata.CurrAsmList,excepttemps,1,finallylabel,true);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { finally code }
         flowcontrol:=[fc_inflowcontrol];
         secondpass(right);
         { goto is allowed if it stays inside the finally block,
           this is checked using the exception block number }
         if (flowcontrol-[fc_gotolabel])<>[fc_inflowcontrol] then
           CGMessage(cg_e_control_flow_outside_finally);
         if codegenerror then
           exit;

         { don't generate line info for internal cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

         { the value should now be in the exception handler }
         reasonreg:=hlcg.getintregister(current_asmdata.CurrAsmList,osuinttype);
         hlcg.g_exception_reason_load(current_asmdata.CurrAsmList,osuinttype,osuinttype,excepttemps.reasonbuf,reasonreg);
         if implicitframe then
           begin
             hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,0,reasonreg,endfinallylabel);
             { finally code only needed to be executed on exception }
             flowcontrol:=[fc_inflowcontrol];
             secondpass(t1);
             if flowcontrol<>[fc_inflowcontrol] then
               CGMessage(cg_e_control_flow_outside_finally);
             if codegenerror then
               exit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             if (tf_safecall_exceptions in target_info.flags) and
                (current_procinfo.procdef.proccalloption=pocall_safecall) then
               handle_safecall_exception
=======
{$if defined(x86) or defined(arm)}
             if (tf_safecall_exceptions in target_info.flags) and
                (current_procinfo.procdef.proccalloption=pocall_safecall) then
=======
{$if defined(x86) or defined(arm)}
             if current_procinfo.procdef.proccalloption=pocall_safecall then
               begin
                 { Remove and destroy the last exception object }
                 cg.a_call_name(current_asmdata.CurrAsmList,'FPC_POPOBJECTSTACK');
                 cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DESTROYEXCEPTION');
                 { Set return value of safecall procedure to indicate exception.       }
                 { Exception will be raised after procedure exit based on return value }
                 cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_ADDR,aint($8000FFFF),NR_FUNCTION_RETURN_REG);
               end
             else
{$endif}
               cg.a_call_name(current_asmdata.CurrAsmList,'FPC_RERAISE');
           end
         else
           begin
             cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_EQ,0,NR_FUNCTION_RESULT_REG,endfinallylabel);
             cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,OS_INT,1,NR_FUNCTION_RESULT_REG);
             cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_EQ,0,NR_FUNCTION_RESULT_REG,reraiselabel);
             if fc_exit in tryflowcontrol then
>>>>>>> origin/fixes_2_2
               begin
                 { find safe_result variable we created in the generate_except_block }
                 retsym:=tlocalvarsym(current_procinfo.procdef.localst.Find('safe_result'));
                 { Set return value of safecall procedure to indicate exception.       }
                 { Exception will be raised after procedure exit based on return value }
                 cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
                 cg.a_load_loc_reg(current_asmdata.CurrAsmList,OS_INT,retsym.localloc,NR_FUNCTION_RESULT_REG);
                 cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
               end
>>>>>>> graemeg/cpstrnew
             else
                hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_reraise',[],nil);
=======
{$if defined(x86) or defined(arm)}
             if current_procinfo.procdef.proccalloption=pocall_safecall then
               begin
                 { Remove and destroy the last exception object }
                 cg.a_call_name(current_asmdata.CurrAsmList,'FPC_POPOBJECTSTACK');
                 cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DESTROYEXCEPTION');
                 { Set return value of safecall procedure to indicate exception.       }
                 { Exception will be raised after procedure exit based on return value }
                 cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_ADDR,aint($8000FFFF),NR_FUNCTION_RETURN_REG);
               end
             else
{$endif}
               cg.a_call_name(current_asmdata.CurrAsmList,'FPC_RERAISE');
>>>>>>> graemeg/fixes_2_2
           end
         else
           begin
             hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,0,reasonreg,endfinallylabel);
             if fc_exit in tryflowcontrol then
               hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,2,reasonreg,oldCurrExitLabel);
             if fc_break in tryflowcontrol then
               hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,3,reasonreg,oldBreakLabel);
             if fc_continue in tryflowcontrol then
               hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,osuinttype,OC_EQ,4,reasonreg,oldContinueLabel);
             hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_reraise',[],nil);
             { do some magic for exit,break,continue in the try block }
             if fc_exit in tryflowcontrol then
               begin
                  hlcg.a_label(current_asmdata.CurrAsmList,exitfinallylabel);
                  hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
                  hlcg.g_exception_reason_save_const(current_asmdata.CurrAsmList,osuinttype,2,excepttemps.reasonbuf);
                  hlcg.a_jmp_always(current_asmdata.CurrAsmList,finallylabel);
               end;
             if fc_break in tryflowcontrol then
              begin
                 hlcg.a_label(current_asmdata.CurrAsmList,breakfinallylabel);
                 hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
                 hlcg.g_exception_reason_save_const(current_asmdata.CurrAsmList,osuinttype,3,excepttemps.reasonbuf);
                 hlcg.a_jmp_always(current_asmdata.CurrAsmList,finallylabel);
               end;
             if fc_continue in tryflowcontrol then
               begin
                  hlcg.a_label(current_asmdata.CurrAsmList,continuefinallylabel);
                  hlcg.g_exception_reason_discard(current_asmdata.CurrAsmList,osuinttype,excepttemps.reasonbuf);
                  hlcg.g_exception_reason_save_const(current_asmdata.CurrAsmList,osuinttype,4,excepttemps.reasonbuf);
                  hlcg.a_jmp_always(current_asmdata.CurrAsmList,finallylabel);
               end;
           end;
         unget_exception_temps(current_asmdata.CurrAsmList,excepttemps);
         hlcg.a_label(current_asmdata.CurrAsmList,endfinallylabel);

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         { end cleanup }
         current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

         current_procinfo.CurrExitLabel:=oldCurrExitLabel;
         if assigned(current_procinfo.CurrBreakLabel) then
          begin
            current_procinfo.CurrContinueLabel:=oldContinueLabel;
            current_procinfo.CurrBreakLabel:=oldBreakLabel;
          end;
         flowcontrol:=oldflowcontrol+(tryflowcontrol-[fc_inflowcontrol]);
      end;


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
end.

