{
    Copyright (c) 1998-2002, 2021 by Florian Klaempfl and Nikolay Nikolov

    Generate WebAssembly code for in/case nodes

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
unit nwasmset;

{$i fpcdefs.inc}

interface

    uses
      node,nset,ncgset;

    type

      { twasminnode }

      twasminnode = class(tcginnode)
      protected
        function checkgenjumps(out setparts: Tsetparts; out numparts: byte; out use_small: boolean): boolean;override;
      end;

      { twasmcasenode }

      twasmcasenode = class(tcgcasenode)
      private
        ElseBr: Integer;
        EndBr: Integer;

        function GetBranchBr(Block: TNode; out _Br: Integer): Boolean;
      protected
        function BlockBr(id:longint):Integer;
        procedure genlinearlist(hp : pcaselabel);override;
        procedure genlinearcmplist(hp : pcaselabel);override;
      public
        procedure pass_generate_code;override;
      end;

implementation

    uses
      globtype,globals,
      cpubase,
      cgbase,cgutils,
      aasmdata,aasmcpu,
      hlcgobj,hlcgcpu,
      nbas,
      symtype,
      pass_2,defutil,verbose,constexp;

{*****************************************************************************
                                 TWASMINNODE
*****************************************************************************}

    function twasminnode.checkgenjumps(out setparts: Tsetparts; out numparts: byte; out use_small: boolean): boolean;
      begin
        { call inherited to initialize use_small }
        inherited;
        result:=false;
      end;

{*****************************************************************************
                                TWASMCASENODE
*****************************************************************************}

    function twasmcasenode.GetBranchBr(Block: TNode; out _Br: Integer): Boolean;
      begin
        Result := True;

        if not Assigned(Block) then
          begin
            { Block doesn't exist / is empty }
            _Br := EndBr;
            Exit;
          end;

        { These optimisations aren't particularly debugger friendly }
        if not (cs_opt_level2 in current_settings.optimizerswitches) then
          begin
            Result := False;
            current_asmdata.CurrAsmList.concat(taicpu.op_none(a_block));
            thlcgwasm(hlcg).incblock;
            _Br:=thlcgwasm(hlcg).br_blocks;
            Exit;
          end;

        while Assigned(Block) do
          begin
            case Block.nodetype of
              nothingn:
                begin
                  _Br := EndBr;
                  Exit;
                end;
              goton:
                InternalError(2021011801);
              blockn:
                begin
                  Block := TBlockNode(Block).Left;
                  Continue;
                end;
              statementn:
                begin
                  { If the right node is assigned, then it's a compound block
                    that can't be simplified, so fall through, set Result to
                    False and make a new label }

                  if Assigned(TStatementNode(Block).right) then
                    Break;

                  Block := TStatementNode(Block).Left;
                  Continue;
                end;
              else
                ;
            end;

            Break;
          end;

        { Create unique label }
        Result := False;
        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_block));
        thlcgwasm(hlcg).incblock;
        _Br:=thlcgwasm(hlcg).br_blocks;
      end;


    function twasmcasenode.BlockBr(id: longint): Integer;
      begin
        if not assigned(blocks[id]) then
          internalerror(200411301);
        result:=pcaseblock(blocks[id])^.BlockBr;
      end;


    procedure twasmcasenode.genlinearlist(hp: pcaselabel);

      var
        first : boolean;
        last : TConstExprInt;
        scratch_reg: tregister;
        newsize: tcgsize;
        newdef: tdef;

      procedure gensub(value:tcgint);
        begin
          { here, since the sub and cmp are separate we need
            to move the result before subtract to help
            the register allocator
          }
          hlcg.a_load_reg_reg(current_asmdata.CurrAsmList, opsize, opsize, hregister, scratch_reg);
          hlcg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opsize, value, hregister);
        end;


      procedure genitem(t : pcaselabel);
        begin
          if assigned(t^.less) then
            genitem(t^.less);
          { do we need to test the first value? }
          if first and (t^._low>get_min_value(left.resultdef)) then
            thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList,opsize,jmp_lt,tcgint(t^._low.svalue),hregister,thlcgwasm(hlcg).br_blocks-ElseBr);
          if t^._low=t^._high then
            begin
              if t^._low-last=0 then
                thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList,opsize,OC_EQ,0,hregister,thlcgwasm(hlcg).br_blocks-BlockBr(t^.blockid))
              else
                begin
                  gensub(tcgint(t^._low.svalue-last.svalue));
                  thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList,opsize,
                                           OC_EQ,tcgint(t^._low.svalue-last.svalue),scratch_reg,thlcgwasm(hlcg).br_blocks-BlockBr(t^.blockid));
                end;
              last:=t^._low;
            end
          else
            begin
               { it begins with the smallest label, if the value }
               { is even smaller then jump immediately to the    }
               { ELSE-label                                }
               if first then
                 begin
                   { have we to ajust the first value ? }
                   if (t^._low>get_min_value(left.resultdef)) or (get_min_value(left.resultdef)<>0) then
                     gensub(tcgint(t^._low.svalue));
                 end
               else
                 begin
                   { if there is no unused label between the last and the }
                   { present label then the lower limit can be checked    }
                   { immediately. else check the range in between:       }
                   gensub(tcgint(t^._low.svalue-last.svalue));
                   thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList, opsize,jmp_lt,tcgint(t^._low.svalue-last.svalue),scratch_reg,thlcgwasm(hlcg).br_blocks-ElseBr);
                 end;
               gensub(tcgint(t^._high.svalue-t^._low.svalue));
               thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList,opsize,jmp_le,tcgint(t^._high.svalue-t^._low.svalue),scratch_reg,thlcgwasm(hlcg).br_blocks-BlockBr(t^.blockid));
               last:=t^._high;
            end;
          first:=false;
          if assigned(t^.greater) then
            genitem(t^.greater);
        end;

      begin
        { do we need to generate cmps? }
        if (with_sign and (min_label<0)) then
          genlinearcmplist(hp)
        else
          begin
            { sign/zero extend the value to a full register before starting to
              subtract values, so that on platforms that don't have
              subregisters of the same size as the value we don't generate
              sign/zero-extensions after every subtraction

              make newsize always signed, since we only do this if the size in
              bytes of the register is larger than the original opsize, so
              the value can always be represented by a larger signed type }
            newsize:=tcgsize2signed[reg_cgsize(hregister)];
            if tcgsize2size[newsize]>opsize.size then
              begin
                newdef:=cgsize_orddef(newsize);
                scratch_reg:=hlcg.getintregister(current_asmdata.CurrAsmList,newdef);
                hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,newdef,hregister,scratch_reg);
                hregister:=scratch_reg;
                opsize:=newdef;
              end;
            if (labelcnt>1) or not(cs_opt_level1 in current_settings.optimizerswitches) then
              begin
                last:=0;
                first:=true;
                scratch_reg:=hlcg.getintregister(current_asmdata.CurrAsmList,opsize);
                genitem(hp);
              end
            else
              begin
                { If only one label exists, we can greatly simplify the checks to a simple comparison }
                if hp^._low=hp^._high then
                  thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList, opsize, OC_EQ, tcgint(hp^._low.svalue), hregister, thlcgwasm(hlcg).br_blocks-BlockBr(hp^.blockid))
                else
                  begin
                    scratch_reg:=hlcg.getintregister(current_asmdata.CurrAsmList,opsize);
                    gensub(tcgint(hp^._low.svalue));
                    thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList, opsize, OC_BE, tcgint(hp^._high.svalue-hp^._low.svalue), hregister, thlcgwasm(hlcg).br_blocks-BlockBr(hp^.blockid))
                  end;
              end;
            current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br,thlcgwasm(hlcg).br_blocks-ElseBr));
          end;
      end;


    procedure twasmcasenode.genlinearcmplist(hp : pcaselabel);

      var
        last : TConstExprInt;
        lastwasrange: boolean;

      procedure genitem(t : pcaselabel);
        begin
          if assigned(t^.less) then
            genitem(t^.less);
          if t^._low=t^._high then
            begin
              thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList, opsize, OC_EQ, tcgint(t^._low.svalue),hregister, thlcgwasm(hlcg).br_blocks-BlockBr(t^.blockid));
              { Reset last here, because we've only checked for one value and need to compare
                for the next range both the lower and upper bound }
              lastwasrange := false;
            end
          else
            begin
              { it begins with the smallest label, if the value }
              { is even smaller then jump immediately to the    }
              { ELSE-label                                }
              if not lastwasrange or (t^._low-last>1) then
                thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList, opsize, jmp_lt, tcgint(t^._low.svalue), hregister, thlcgwasm(hlcg).br_blocks-ElseBr);
              thlcgwasm(hlcg).a_cmp_const_reg_br(current_asmdata.CurrAsmList, opsize, jmp_le, tcgint(t^._high.svalue), hregister, thlcgwasm(hlcg).br_blocks-BlockBr(t^.blockid));

              last:=t^._high;
              lastwasrange := true;
            end;
          if assigned(t^.greater) then
            genitem(t^.greater);
        end;

      begin
        last:=0;
        lastwasrange:=false;
        genitem(hp);
        current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br,thlcgwasm(hlcg).br_blocks-ElseBr));
      end;


    procedure twasmcasenode.pass_generate_code;
      var
        oldflowcontrol: tflowcontrol;
        ShortcutElse: Boolean;
        i: Integer;
      begin
        location_reset(location,LOC_VOID,OS_NO);

        oldflowcontrol := flowcontrol;
        include(flowcontrol,fc_inflowcontrol);

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_block));
        thlcgwasm(hlcg).incblock;
        EndBr:=thlcgwasm(hlcg).br_blocks;

        { Do some optimisation to deal with empty else blocks }
        ShortcutElse := GetBranchBr(elseblock, ElseBr);

        for i:=blocks.count-1 downto 0 do
          with pcaseblock(blocks[i])^ do
            shortcut := GetBranchBr(statement, BlockBr);

        with_sign:=is_signed(left.resultdef);
        if with_sign then
          begin
            jmp_gt:=OC_GT;
            jmp_lt:=OC_LT;
            jmp_le:=OC_LTE;
          end
        else
          begin
            jmp_gt:=OC_A;
            jmp_lt:=OC_B;
            jmp_le:=OC_BE;
          end;

        secondpass(left);
        if (left.expectloc=LOC_JUMP)<>
           (left.location.loc=LOC_JUMP) then
          internalerror(2006050501);
        { determines the size of the operand }
        opsize:=left.resultdef;
        { copy the case expression to a register }
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opsize,false);
{$if not defined(cpu64bitalu)}
        if def_cgsize(opsize) in [OS_S64,OS_64] then
          begin
            hregister:=left.location.register64.reglo;
            hregister2:=left.location.register64.reghi;
          end
        else
{$endif not cpu64bitalu and not cpuhighleveltarget}
          hregister:=left.location.register;

        { we need the min_label always to choose between }
        { cmps and subs/decs                             }
        min_label:=case_get_min(labels);

        { Generate the jumps }
{$ifdef OLDREGVARS}
        load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
{$if not defined(cpu64bitalu)}
        if def_cgsize(opsize) in [OS_64,OS_S64] then
          genlinearcmplist(labels)
        else
{$endif not cpu64bitalu and not cpuhighleveltarget}
          begin
            //if cs_opt_level1 in current_settings.optimizerswitches then
            //  begin
            //    { procedures are empirically passed on }
            //    { consumption can also be calculated   }
            //    { but does it pay on the different     }
            //    { processors?                       }
            //    { moreover can the size only be appro- }
            //    { ximated as it is not known if rel8,  }
            //    { rel16 or rel32 jumps are used   }
            //
            //    max_label := case_get_max(labels);
            //
            //    { can we omit the range check of the jump table ? }
            //    getrange(left.resultdef,lv,hv);
            //    jumptable_no_range:=(lv=min_label) and (hv=max_label);
            //
            //    distv:=max_label-min_label;
            //    if distv>=0 then
            //      dist:=distv.uvalue
            //    else
            //      dist:=asizeuint(-distv.svalue);
            //
            //    { optimize for size ? }
            //    if cs_opt_size in current_settings.optimizerswitches  then
            //      begin
            //        if has_jumptable and
            //           (min_label>=int64(low(aint))) and
            //           (max_label<=high(aint)) and
            //           not((labelcnt<=2) or
            //               (distv.svalue<0) or
            //               (dist>3*labelcnt)) then
            //          begin
            //            { if the labels less or more a continuum then }
            //            genjumptable(labels,min_label.svalue,max_label.svalue);
            //          end
            //        else
            //          begin
            //            { a linear list is always smaller than a jump tree }
            //            genlinearlist(labels);
            //          end;
            //      end
            //    else
            //      begin
            //        max_dist:=4*labelcoverage;
            //
            //        { Don't allow jump tables to get too large }
            //        if max_dist>4*labelcnt then
            //          max_dist:=min(max_dist,2048);
            //
            //        if jumptable_no_range then
            //          max_linear_list:=4
            //        else
            //          max_linear_list:=2;
            //
            //        { allow processor specific values }
            //        optimizevalues(max_linear_list,max_dist);
            //
            //        if (labelcnt<=max_linear_list) then
            //          genlinearlist(labels)
            //        else
            //          begin
            //            if (has_jumptable) and
            //               (dist<max_dist) and
            //               (min_label>=int64(low(aint))) and
            //               (max_label<=high(aint)) then
            //              genjumptable(labels,min_label.svalue,max_label.svalue)
            //            { value has been determined on an i7-4770 using a random case with random values
            //              if more values are known, this can be handled depending on the target CPU
            //
            //              Testing on a Core 2 Duo E6850 as well as on a Raspi3 showed also, that 64 is
            //              a good value }
            //            else if labelcnt>=64 then
            //              genjmptree(labels)
            //            else
            //              genlinearlist(labels);
            //          end;
            //      end;
            //  end
            //else
              { it's always not bad }
              genlinearlist(labels);
          end;

        { generate the instruction blocks }
        for i:=0 to blocks.count-1 do with pcaseblock(blocks[i])^ do
          begin
            { If the labels are not equal, then the block label has been shortcut to point elsewhere,
              so there's no need to implement it }
            if not shortcut then
              begin
                current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_block));
                thlcgwasm(hlcg).decblock;
                secondpass(statement);
                { don't come back to case line }
                current_filepos:=current_asmdata.CurrAsmList.getlasttaifilepos^;
{$ifdef OLDREGVARS}
                load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
                current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br,thlcgwasm(hlcg).br_blocks-EndBr));
              end;
          end;

        { ...and the else block }
        if not ShortcutElse then
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_block));
            thlcgwasm(hlcg).decblock;
          end;

        if Assigned(elseblock) then
          begin

            secondpass(elseblock);
{$ifdef OLDREGVARS}
            load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
          end;

        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_block));
        thlcgwasm(hlcg).decblock;

        flowcontrol := oldflowcontrol + (flowcontrol - [fc_inflowcontrol]);
      end;

begin
  cinnode:=twasminnode;
  ccasenode:=twasmcasenode;
end.
