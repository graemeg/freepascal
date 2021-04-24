{
    Copyright (c) 2019 by Dmitry Boyarintsev

    Generate assembler for nodes that influence the flow for the JVM

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
unit nwasmflw;

{$i fpcdefs.inc}

interface

    uses
      aasmbase,node,nflw,ncgflw, cutils;

    type

      { twasmifnode }

      { Wasm doesn't have any jump(+offset) operations
        It only provide structured blockes to handle jumps
        (It's possible to jump-out-of-block at any time)
        "If" is also implemented as a block, identical to high-level language. }
      twasmifnode = class(tcgifnode)
      public
        procedure pass_generate_code;override;
      end;

      { twasmwhilerepeatnode }

      twasmwhilerepeatnode = class(tcgwhilerepeatnode)
      public
        procedure pass_generate_code_condition;
        procedure pass_generate_code;override;
      end;

      { twasmraisenode }

      twasmraisenode = class(tcgraisenode)
      public
        function pass_1 : tnode;override;
      end;

      { twasmtryexceptnode }

      twasmtryexceptnode = class(tcgtryexceptnode)
      public
        procedure pass_generate_code;override;
      end;

      { twasmtryfinallynode }

      twasmtryfinallynode = class(tcgtryfinallynode)
      public
        procedure pass_generate_code;override;
      end;

implementation

    uses
      verbose,globals,systems,globtype,constexp,
      symconst,symdef,symsym,aasmtai,aasmdata,aasmcpu,defutil,defcmp,
      procinfo,cgbase,pass_1,pass_2,parabase,compinnr,
      cpubase,cpuinfo,
      nbas,nld,ncon,ncnv,ncal,ninl,nmem,nadd,
      tgobj,paramgr,
      cgutils,hlcgobj,hlcgcpu;

{*****************************************************************************
                           twasmwhilerepeatnode
*****************************************************************************}

    procedure twasmwhilerepeatnode.pass_generate_code_condition;
      begin
        secondpass(left);
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);

        // reversing the condition
        if not (lnf_checknegate in loopflags) then
          current_asmdata.CurrAsmList.concat(taicpu.op_none(a_i32_eqz));

        current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,1) );
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
      end;


    procedure twasmwhilerepeatnode.pass_generate_code;
      var
         lcont,lbreak,lloop,
         oldclabel,oldblabel : tasmlabel;
         truelabel,falselabel : tasmlabel;
         oldflowcontrol : tflowcontrol;
         oldloopcontbroffset: Integer;
         oldloopbreakbroffset: Integer;
      begin
        location_reset(location,LOC_VOID,OS_NO);

        current_asmdata.getjumplabel(lloop);
        current_asmdata.getjumplabel(lcont);
        current_asmdata.getjumplabel(lbreak);

        oldflowcontrol:=flowcontrol;

        oldloopcontbroffset:=thlcgwasm(hlcg).loopContBr;
        oldloopbreakbroffset:=thlcgwasm(hlcg).loopBreakBr;
        oldclabel:=current_procinfo.CurrContinueLabel;
        oldblabel:=current_procinfo.CurrBreakLabel;

        include(flowcontrol,fc_inflowcontrol);
        exclude(flowcontrol,fc_unwind_loop);

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_block));
        thlcgwasm(hlcg).incblock;
        thlcgwasm(hlcg).loopBreakBr:=thlcgwasm(hlcg).br_blocks;
        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_loop));
        thlcgwasm(hlcg).incblock;

        if lnf_testatbegin in loopflags then
        begin
          pass_generate_code_condition;
          thlcgwasm(hlcg).loopContBr:=thlcgwasm(hlcg).br_blocks;
        end else
          thlcgwasm(hlcg).loopContBr:=thlcgwasm(hlcg).br_blocks+1;

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_block));
        thlcgwasm(hlcg).incblock;

        current_procinfo.CurrContinueLabel:=lcont;
        current_procinfo.CurrBreakLabel:=lbreak;

        secondpass(right);

        if (lnf_testatbegin in loopflags) then
          current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br,1) ); // jump back to the external loop

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_block));
        thlcgwasm(hlcg).decblock;
        if not (lnf_testatbegin in loopflags) then begin
          pass_generate_code_condition;
        end;
        current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br,0) ); // jump back to loop

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_loop));
        thlcgwasm(hlcg).decblock;
        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_block));
        thlcgwasm(hlcg).decblock;

        current_procinfo.CurrContinueLabel:=oldclabel;
        current_procinfo.CurrBreakLabel:=oldblabel;
        thlcgwasm(hlcg).loopContBr:=oldloopcontbroffset;
        thlcgwasm(hlcg).loopBreakBr:=oldloopbreakbroffset;

        { a break/continue in a while/repeat block can't be seen outside }
        flowcontrol:=oldflowcontrol+(flowcontrol-[fc_break,fc_continue,fc_inflowcontrol]);
      end;

{*****************************************************************************
                               twasmifnode
*****************************************************************************}

    procedure twasmifnode.pass_generate_code;
      var
        oldflowcontrol: tflowcontrol;
      begin
        // left  - condition
        // right - then
        // t1    - else (optional)

        location_reset(location,LOC_VOID,OS_NO);

        oldflowcontrol := flowcontrol;
        include(flowcontrol,fc_inflowcontrol);

        //todo: MOVE all current_asm_data actions to Wasm HL CodeGen

        secondpass(left); // condition exprssions
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_if));
        thlcgwasm(hlcg).incblock;
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

        if Assigned(right) then
          secondpass(right); // then branchs

        if Assigned(t1) then // else branch
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_none(a_else));
            secondpass(t1);
          end;

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_if));
        thlcgwasm(hlcg).decblock;

        flowcontrol := oldflowcontrol + (flowcontrol - [fc_inflowcontrol]);
      end;

{*****************************************************************************
                             twasmraisenode
*****************************************************************************}

    function twasmraisenode.pass_1 : tnode;
      var
        statements : tstatementnode;
        //current_addr : tlabelnode;
        raisenode : tcallnode;
      begin
        result:=internalstatements(statements);

        if assigned(left) then
          begin
            { first para must be a class }
            firstpass(left);
            { insert needed typeconvs for addr,frame }
            if assigned(right) then
              begin
                { addr }
                firstpass(right);
                { frame }
                if assigned(third) then
                  firstpass(third)
                else
                  third:=cpointerconstnode.Create(0,voidpointertype);
              end
            else
              begin
                third:=cinlinenode.create(in_get_frame,false,nil);
                //current_addr:=clabelnode.create(cnothingnode.create,clabelsym.create('$raiseaddr'));
                //addstatement(statements,current_addr);
                //right:=caddrnode.create(cloadnode.create(current_addr.labsym,current_addr.labsym.owner));
                right:=cnilnode.create;

                { raise address off by one so we are for sure inside the action area for the raise }
                if tf_use_psabieh in target_info.flags then
                  right:=caddnode.create_internal(addn,right,cordconstnode.create(1,sizesinttype,false));
              end;

            raisenode:=ccallnode.createintern('fpc_raiseexception',
              ccallparanode.create(third,
              ccallparanode.create(right,
              ccallparanode.create(left,nil)))
              );
            include(raisenode.callnodeflags,cnf_call_never_returns);
            addstatement(statements,raisenode);
          end
        else
          begin
            addstatement(statements,ccallnode.createintern('fpc_popaddrstack',nil));
            raisenode:=ccallnode.createintern('fpc_reraise',nil);
            include(raisenode.callnodeflags,cnf_call_never_returns);
            addstatement(statements,raisenode);
          end;
        left:=nil;
        right:=nil;
        third:=nil;
      end;

{*****************************************************************************
                             twasmtryexceptnode
*****************************************************************************}

    procedure twasmtryexceptnode.pass_generate_code;
      begin
        location_reset(location,LOC_VOID,OS_NO);

        current_asmdata.CurrAsmList.concat(tai_comment.Create(strpnew('TODO: try..except, try')));

        secondpass(left);
        //if codegenerror then
        //  goto errorexit;

        current_asmdata.CurrAsmList.concat(tai_comment.Create(strpnew('TODO: try..except, end')));
      end;

{*****************************************************************************
                             twasmtryfinallynode
*****************************************************************************}

    procedure twasmtryfinallynode.pass_generate_code;
      begin
        location_reset(location,LOC_VOID,OS_NO);

        current_asmdata.CurrAsmList.concat(tai_comment.Create(strpnew('TODO: try..finally, try')));

        { try code }
        if assigned(left) then
          begin
            secondpass(left);
            if codegenerror then
              exit;
          end;

        current_asmdata.CurrAsmList.concat(tai_comment.Create(strpnew('TODO: try..finally, finally')));

        { finally code (don't unconditionally set fc_inflowcontrol, since the
          finally code is unconditionally executed; we do have to filter out
          flags regarding break/contrinue/etc. because we have to give an
          error in case one of those is used in the finally-code }
        //flowcontrol:=finallyexceptionstate.oldflowcontrol*[fc_inflowcontrol,fc_catching_exceptions];
        secondpass(right);
        { goto is allowed if it stays inside the finally block,
          this is checked using the exception block number }
        //if (flowcontrol-[fc_gotolabel])<>(finallyexceptionstate.oldflowcontrol*[fc_inflowcontrol,fc_catching_exceptions]) then
        //  CGMessage(cg_e_control_flow_outside_finally);
        if codegenerror then
          exit;
        current_asmdata.CurrAsmList.concat(tai_comment.Create(strpnew('TODO: try..finally, end')));
      end;

initialization
  cifnode:=twasmifnode;
  cwhilerepeatnode:=twasmwhilerepeatnode;
  craisenode:=twasmraisenode;
  ctryexceptnode:=twasmtryexceptnode;
  ctryfinallynode:=twasmtryfinallynode;
end.
