{
    Copyright (c) 1998-2002 by Florian Klaempfl and Carl Eric Codere

    Generate generic assembler for in set/case labels

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
unit ncgset;

{$i fpcdefs.inc}

interface

    uses
       globtype,globals,constexp,symtype,
       node,nset,cpubase,cgbase,cgutils,cgobj,aasmbase,aasmtai,aasmdata;

    type
       tcgsetelementnode = class(tsetelementnode)
          procedure pass_generate_code;override;
       end;


       Tsetpart=record
         range : boolean;      {Part is a range.}
         start,stop : byte;    {Start/stop when range; Stop=element when an element.}
       end;
       Tsetparts=array[1..8] of Tsetpart;

       { tcginnode }

       tcginnode = class(tinnode)
         procedure in_smallset(opdef: tdef; setbase: aint); virtual;

         function pass_1: tnode;override;
         procedure pass_generate_code;override;
       protected
         function checkgenjumps(out setparts: Tsetparts; out numparts: byte; out use_small: boolean): boolean; virtual;
         function analizeset(const Aset:Tconstset;out setparts: Tsetparts; out numparts: byte;is_small:boolean):boolean;virtual;
       end;

       tcgcasenode = class(tcasenode)
          {
            Emits the case node statement. Contrary to the intel
            80x86 version, this version does not emit jump tables,
            because of portability problems.
          }
          procedure pass_generate_code;override;

        protected
          with_sign : boolean;
          opsize : tdef;
          jmp_gt,jmp_lt,jmp_le : topcmp;
          { register with case expression }
          hregister,hregister2 : tregister;
          endlabel,elselabel : tasmlabel;

          { true, if we can omit the range check of the jump table }
          jumptable_no_range : boolean;
          { has the implementation jumptable support }
          min_label : tconstexprint;

          function GetBranchLabel(Block: TNode; out _Label: TAsmLabel): Boolean;

          function  blocklabel(id:longint):tasmlabel;
          procedure optimizevalues(var max_linear_list:int64;var max_dist:qword);virtual;
          function  has_jumptable : boolean;virtual;
          procedure genjumptable(hp : pcaselabel;min_,max_ : int64); virtual;
          procedure genlinearlist(hp : pcaselabel); virtual;
          procedure genlinearcmplist(hp : pcaselabel); virtual;

         procedure genjmptreeentry(p : pcaselabel;parentvalue : TConstExprInt); virtual;
         procedure genjmptree(root : pcaselabel); virtual;
       end;


implementation

    uses
      verbose,
      cutils,
      symconst,symdef,symsym,defutil,
      pass_2,tgobj,
      nbas,ncon,ncgflw,
{$ifdef WASM}
      hlcgcpu,aasmcpu,
{$endif WASM}
      ncgutil,hlcgobj;


{*****************************************************************************
                          TCGSETELEMENTNODE
*****************************************************************************}

    procedure tcgsetelementnode.pass_generate_code;
      begin
        { load the set element's value }
        secondpass(left);

        { also a second value ? }
        if assigned(right) then
          internalerror(2015111106);

        { we don't modify the left side, we only check the location type; our
          parent node (an add-node) will use the resulting location to perform
          the set operation without creating an intermediate set }
        location_copy(location,left.location);
      end;


{*****************************************************************************
*****************************************************************************}

  function tcginnode.analizeset(const Aset:Tconstset; out setparts:tsetparts; out numparts: byte; is_small:boolean):boolean;
    var
      compares,maxcompares:word;
      i:byte;
    begin
      analizeset:=false;
      fillchar(setparts,sizeof(setparts),0);
      numparts:=0;
      compares:=0;
      { Lots of comparisions take a lot of time, so do not allow
        too much comparisions. 8 comparisions are, however, still
        smalller than emitting the set }
      if cs_opt_size in current_settings.optimizerswitches then
        maxcompares:=8
      else
        maxcompares:=5;
      { when smallset is possible allow only 3 compares the smallset
        code is for littlesize also smaller when more compares are used }
      if is_small then
        maxcompares:=3;
      for i:=0 to 255 do
        if i in Aset then
          begin
            if (numparts=0) or (i<>setparts[numparts].stop+1) then
              begin
                {Set element is a separate element.}
                inc(compares);
                if compares>maxcompares then
                  exit;
                inc(numparts);
                setparts[numparts].range:=false;
                setparts[numparts].stop:=i;
              end
            else
              {Set element is part of a range.}
              if not setparts[numparts].range then
                begin
                  {Transform an element into a range.}
                  setparts[numparts].range:=true;
                  setparts[numparts].start:=setparts[numparts].stop;
                  setparts[numparts].stop:=i;
                  { there's only one compare per range anymore. Only a }
                  { sub is added, but that's much faster than a        }
                  { cmp/jcc combo so neglect its effect                }
{                  inc(compares);
                  if compares>maxcompares then
                   exit; }
                end
              else
                begin
                  {Extend a range.}
                  setparts[numparts].stop:=i;
                end;
          end;
      analizeset:=true;
    end;


    procedure tcginnode.in_smallset(opdef: tdef; setbase: aint);
      begin
        { location is always LOC_REGISTER }
        location_reset(location, LOC_REGISTER, def_cgsize(resultdef));
        { allocate a register for the result }
        location.register := hlcg.getintregister(current_asmdata.CurrAsmList, resultdef);
        {****************************  SMALL SET **********************}
        if left.location.loc=LOC_CONSTANT then
          begin
            hlcg.a_bit_test_const_loc_reg(current_asmdata.CurrAsmList,
             right.resultdef, resultdef,
              left.location.value-setbase, right.location,
              location.register);
          end
        else
          begin
            hlcg.location_force_reg(current_asmdata.CurrAsmList, left.location,
             left.resultdef, opdef, true);
            register_maybe_adjust_setbase(current_asmdata.CurrAsmList, opdef, left.location,
             setbase);
            hlcg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList, opdef,
              right.resultdef, resultdef, left.location.register, right.location,
               location.register);
          end;
      end;


    function tcginnode.checkgenjumps(out setparts: Tsetparts; out numparts: byte;out use_small: boolean): boolean;
      begin
         { check if we can use smallset operation using btl which is limited
           to 32 bits, the left side may also not contain higher values !! }
         use_small:=is_smallset(right.resultdef) and
                    not is_signed(left.resultdef) and
                    ((left.resultdef.typ=orddef) and (torddef(left.resultdef).high<32) or
                     (left.resultdef.typ=enumdef) and (tenumdef(left.resultdef).max<32));

         { Can we generate jumps? Possible for all types of sets }
         checkgenjumps:=(right.nodetype=setconstn) and
                   analizeset(Tsetconstnode(right).value_set^,setparts,numparts,use_small);
      end;


    function tcginnode.pass_1: tnode;
      var
        setparts: Tsetparts;
        numparts: byte;
        use_small: boolean;
      begin
        result := inherited pass_1;
        if not(assigned(result)) and
          checkgenjumps(setparts,numparts,use_small) then
          expectloc := LOC_JUMP;
      end;

    procedure tcginnode.pass_generate_code;
       var
         adjustment,
         setbase    : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
         l, l2      : tasmlabel;
         hr,
         pleftreg   : tregister;
         setparts   : Tsetparts;
         opsize     : tcgsize;
         opdef      : tdef;
         uopsize    : tcgsize;
         uopdef     : tdef;
         orgopsize  : tcgsize;
         genjumps,
         use_small  : boolean;
         i,numparts : byte;
         needslabel : Boolean;
       begin
         l2:=nil;

         { We check first if we can generate jumps, this can be done
           because the resultdef is already set in firstpass }

         genjumps := checkgenjumps(setparts,numparts,use_small);

         orgopsize := def_cgsize(left.resultdef);
{$if defined(cpu8bitalu)}
         if (tsetdef(right.resultdef).setbase>=-128) and
           (tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase+1<=256) then
           begin
             uopsize := OS_8;
             uopdef := u8inttype;
             if is_signed(left.resultdef) then
               begin
                 opsize := OS_S8;
                 opdef := s8inttype;
               end
             else
               begin
                 opsize := uopsize;
                 opdef := uopdef;
               end;
           end
{$endif defined(cpu8bitalu)}
{$if defined(cpu8bitalu)}
{ this should be also enabled for 16 bit CPUs, however, I have no proper testing facility for 16 bit, my
  testing results using Dosbox are no reliable }
{ $if defined(cpu8bitalu) or defined(cpu16bitalu)}
         else if (tsetdef(right.resultdef).setbase>=-32768) and
           (tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase+1<=65536) then
           begin
             uopsize := OS_16;
             uopdef := u16inttype;
             if is_signed(left.resultdef) then
               begin
                 opsize := OS_S16;
                 opdef := s16inttype;
               end
             else
               begin
                 opsize := uopsize;
                 opdef := uopdef;
               end;
           end
         else
{$endif defined(cpu8bitalu)}
           begin
             uopsize := OS_32;
             uopdef := u32inttype;
             if is_signed(left.resultdef) then
               begin
                 opsize := OS_S32;
                 opdef := s32inttype;
               end
             else
               begin
                 opsize := uopsize;
                 opdef := uopdef;
               end;
           end;
         needslabel := false;

         if not genjumps then
           { calculate both operators }
           { the complex one first }
           { not in case of genjumps, because then we don't secondpass      }
           { right at all (so we have to make sure that "right" really is   }
           { "right" and not "swapped left" in that case)                   }
           firstcomplex(self);

         secondpass(left);
         if (left.expectloc=LOC_JUMP)<>
            (left.location.loc=LOC_JUMP) then
           internalerror(2007070101);

         { Only process the right if we are not generating jumps }
         if not genjumps then
           secondpass(right);
         if codegenerror then
           exit;

         { ofcourse not commutative }
         if nf_swapped in flags then
          swapleftright;

         setbase:=tsetdef(right.resultdef).setbase;
         if genjumps then
           begin
             { location is always LOC_JUMP }
             current_asmdata.getjumplabel(l);
             current_asmdata.getjumplabel(l2);
             location_reset_jump(location,l,l2);

             { If register is used, use only lower 8 bits }
             hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
             pleftreg := left.location.register;

             { how much have we already substracted from the x in the }
             { "x in [y..z]" expression                               }
             adjustment := 0;
             hr:=NR_NO;

             for i:=1 to numparts do
              if setparts[i].range then
               { use fact that a <= x <= b <=> aword(x-a) <= aword(b-a) }
               begin
                 { is the range different from all legal values? }
                 if (setparts[i].stop-setparts[i].start <> 255) or not (orgopsize = OS_8) then
                   begin
                     { yes, is the lower bound <> 0? }
                     if (setparts[i].start <> 0) then
                       { we're going to substract from the left register,   }
                       { so in case of a LOC_CREGISTER first move the value }
                       { to edi (not done before because now we can do the  }
                       { move and substract in one instruction with LEA)    }
                       if (left.location.loc = LOC_CREGISTER) and
                          (hr<>pleftreg) then
                         begin
                           { don't change this back to a_op_const_reg/a_load_reg_reg, since pleftreg must not be modified }
                           hr:=hlcg.getintregister(current_asmdata.CurrAsmList,opdef);
                           hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SUB,opdef,setparts[i].start,pleftreg,hr);
                           pleftreg:=hr;
                         end
                       else
                         begin
                           { otherwise, the value is already in a register   }
                           { that can be modified                            }
                           hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,opdef,
                              setparts[i].start-adjustment,pleftreg)
                         end;
                     { new total value substracted from x:           }
                     { adjustment + (setparts[i].start - adjustment) }
                     adjustment := setparts[i].start;

                     { check if result < b-a+1 (not "result <= b-a", since }
                     { we need a carry in case the element is in the range }
                     { (this will never overflow since we check at the     }
                     { beginning whether stop-start <> 255)                }
                     hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, uopdef, OC_B,
                       setparts[i].stop-setparts[i].start+1,pleftreg,location.truelabel);
                   end
                 else
                   { if setparts[i].start = 0 and setparts[i].stop = 255,  }
                   { it's always true since "in" is only allowed for bytes }
                   begin
                     hlcg.a_jmp_always(current_asmdata.CurrAsmList,location.truelabel);
                   end;
               end
              else
               begin
                 { Emit code to check if left is an element }
                 hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opdef, OC_EQ,
                       setparts[i].stop-adjustment,pleftreg,location.truelabel);
               end;
              { To compensate for not doing a second pass }
              right.location.reference.symbol:=nil;
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
           end
         else
         {*****************************************************************}
         {                     NO JUMP TABLE GENERATION                    }
         {*****************************************************************}
           begin
             { We will now generated code to check the set itself, no jmps,
               handle smallsets separate, because it allows faster checks }
             if use_small then
               begin
                 in_smallset(opdef, setbase);
               end
             else
               {************************** NOT SMALL SET ********************}
               begin
                 { location is always LOC_REGISTER }
                 location_reset(location, LOC_REGISTER, uopsize{def_cgsize(resultdef)});
                 { allocate a register for the result }
                 location.register := hlcg.getintregister(current_asmdata.CurrAsmList, uopdef);

                 if right.location.loc=LOC_CONSTANT then
                   begin
                     { can it actually occur currently? CEC }
                     { yes: "if bytevar in [1,3,5,7,9,11,13,15]" (JM) }

                     { note: this code assumes that left in [0..255], which is a valid }
                     { assumption (other cases will be caught by range checking) (JM)  }

                     { load left in register }
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,uopdef,true);
                     register_maybe_adjust_setbase(current_asmdata.CurrAsmList,uopdef,left.location,setbase);
                     { emit bit test operation -- warning: do not use
                       location_force_reg() to force a set into a register, except
                       to a register of the same size as the set. The reason is
                       that on big endian systems, this would require moving the
                       set to the most significant part of the new register,
                       and location_force_register can't do that (it does not
                       know the type).

                      a_bit_test_reg_loc_reg() properly takes into account the
                      size of the set to adjust the register index to test }
                     hlcg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList,
                       uopdef,right.resultdef,uopdef,
                       left.location.register,right.location,location.register);

                     { now zero the result if left > nr_of_bits_in_right_register }
                     hr := hlcg.getintregister(current_asmdata.CurrAsmList,uopdef);
                     { if left > tcgsize2size[opsize]*8 then hr := 0 else hr := $ffffffff }
                     { (left.location.size = location.size at this point) }
                     hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_SUB, uopdef, tcgsize2size[opsize]*8, left.location.register, hr);
                     hlcg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SAR, uopdef, (tcgsize2size[opsize]*8)-1, hr);

                     { if left > tcgsize2size[opsize]*8-1, then result := 0 else result := result of bit test }
                     hlcg.a_op_reg_reg(current_asmdata.CurrAsmList, OP_AND, uopdef, hr, location.register);
                   end { of right.location.loc=LOC_CONSTANT }
                 { do search in a normal set which could have >32 elements
                   but also used if the left side contains higher values > 32 }
                 else if (left.location.loc=LOC_CONSTANT) then
                   begin
                     if (left.location.value < setbase) or (((left.location.value-setbase) shr 3) >= right.resultdef.size) then
                       {should be caught earlier }
                       internalerror(2007020402);

                     hlcg.a_bit_test_const_loc_reg(current_asmdata.CurrAsmList,right.resultdef,uopdef,left.location.value-setbase,
                       right.location,location.register);
                   end
                 else
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,true);
                     register_maybe_adjust_setbase(current_asmdata.CurrAsmList,opdef,left.location,setbase);
                     pleftreg := left.location.register;

                     if (opsize >= OS_S8) or { = if signed }
                        ((left.resultdef.typ=orddef) and
                         ((torddef(left.resultdef).low < int64(tsetdef(right.resultdef).setbase)) or
                          (torddef(left.resultdef).high > int64(tsetdef(right.resultdef).setmax)))) or
                        ((left.resultdef.typ=enumdef) and
                         ((tenumdef(left.resultdef).min < aint(tsetdef(right.resultdef).setbase)) or
                          (tenumdef(left.resultdef).max > aint(tsetdef(right.resultdef).setmax)))) then
                       begin
{$ifdef WASM}
                         needslabel := True;

                         thlcgwasm(hlcg).a_cmp_const_reg_stack(current_asmdata.CurrAsmList, opdef, OC_A, tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase, pleftreg);

                         current_asmdata.CurrAsmList.concat(taicpu.op_none(a_if));
                         thlcgwasm(hlcg).incblock;
                         thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

                         hlcg.a_load_const_reg(current_asmdata.CurrAsmList, uopdef, 0, location.register);
                         current_asmdata.CurrAsmList.concat(taicpu.op_none(a_else));
{$else WASM}
                         current_asmdata.getjumplabel(l);
                         current_asmdata.getjumplabel(l2);
                         needslabel := True;

                         hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opdef, OC_BE, tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase, pleftreg, l);

                         hlcg.a_load_const_reg(current_asmdata.CurrAsmList, uopdef, 0, location.register);
                         hlcg.a_jmp_always(current_asmdata.CurrAsmList, l2);

                         hlcg.a_label(current_asmdata.CurrAsmList, l);
{$endif WASM}
                       end;

                     hlcg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList,opdef,right.resultdef,uopdef,
                       pleftreg,right.location,location.register);

                     if needslabel then
                       begin
{$ifdef WASM}
                         current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_if));
                         thlcgwasm(hlcg).decblock;
{$else WASM}
                         hlcg.a_label(current_asmdata.CurrAsmList, l2);
{$endif WASM}
                       end
                   end;
{$ifndef cpuhighleveltarget}
                 location.size := def_cgsize(resultdef);
                 location.register := cg.makeregsize(current_asmdata.CurrAsmList, location.register, location.size);
{$else not cpuhighleveltarget}
                 hr:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
                 hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,uopdef,resultdef,location.register,hr);
                 location.register:=hr;
                 location.size := def_cgsize(resultdef);
{$endif not cpuhighleveltarget}
               end;
           end;
         location_freetemp(current_asmdata.CurrAsmList, right.location);
       end;

{*****************************************************************************
                            TCGCASENODE
*****************************************************************************}


    { Analyse the nodes following the else label - if empty, change to end label }
    function tcgcasenode.GetBranchLabel(Block: TNode; out _Label: TAsmLabel): Boolean;
      var
        LabelSym: TLabelSym;
      begin
        Result := True;

        if not Assigned(Block) then
          begin
            { Block doesn't exist / is empty }
            _Label := endlabel;
            Exit;
          end;

        { These optimisations aren't particularly debugger friendly }
        if not (cs_opt_level2 in current_settings.optimizerswitches) then
          begin
            Result := False;
            current_asmdata.getjumplabel(_Label);
            Exit;
          end;

        while Assigned(Block) do
          begin
            case Block.nodetype of
              nothingn:
                begin
                  _Label := endlabel;
                  Exit;
                end;
              goton:
                begin
                  LabelSym := TCGGotoNode(Block).labelsym;
                  if not Assigned(LabelSym) then
                    InternalError(2018121131);

                  _Label := TCGLabelNode(TCGGotoNode(Block).labelnode).getasmlabel;
                  if Assigned(_Label) then
                    { Keep tabs on the fact that an actual 'goto' was used }
                    Include(flowcontrol,fc_gotolabel)
                  else
                    Break;
                  Exit;
                end;
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
        current_asmdata.getjumplabel(_Label);
      end;


    function tcgcasenode.blocklabel(id:longint):tasmlabel;
      begin
        if not assigned(blocks[id]) then
          internalerror(200411301);
        result:=pcaseblock(blocks[id])^.blocklabel;
      end;


    procedure tcgcasenode.optimizevalues(var max_linear_list:int64;var max_dist:qword);
      begin
        { no changes by default }
      end;


    function tcgcasenode.has_jumptable : boolean;
      begin
        { No jumptable support in the default implementation }
        has_jumptable:=false;
      end;


    procedure tcgcasenode.genjumptable(hp : pcaselabel;min_,max_ : int64);
      begin
        internalerror(200209161);
      end;


    procedure tcgcasenode.genlinearlist(hp : pcaselabel);

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
             hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_lt,tcgint(t^._low.svalue),hregister,elselabel);
           if t^._low=t^._high then
             begin
               if t^._low-last=0 then
                 hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,OC_EQ,0,hregister,blocklabel(t^.blockid))
               else
                 begin
                   gensub(tcgint(t^._low.svalue-last.svalue));
                   hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,
                                            OC_EQ,tcgint(t^._low.svalue-last.svalue),scratch_reg,blocklabel(t^.blockid));
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
                    hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize,jmp_lt,tcgint(t^._low.svalue-last.svalue),scratch_reg,elselabel);
                  end;
                gensub(tcgint(t^._high.svalue-t^._low.svalue));
                hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_le,tcgint(t^._high.svalue-t^._low.svalue),scratch_reg,blocklabel(t^.blockid));
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
                    hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, OC_EQ, tcgint(hp^._low.svalue), hregister, blocklabel(hp^.blockid))
                  else
                    begin
                      scratch_reg:=hlcg.getintregister(current_asmdata.CurrAsmList,opsize);
                      gensub(tcgint(hp^._low.svalue));
                      hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, OC_BE, tcgint(hp^._high.svalue-hp^._low.svalue), hregister, blocklabel(hp^.blockid))
                    end;
                end;
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,elselabel);
           end;
      end;


    procedure tcgcasenode.genlinearcmplist(hp : pcaselabel);

      var
         last : TConstExprInt;
         lastwasrange: boolean;

      procedure genitem(t : pcaselabel);

{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
        var
           l1 : tasmlabel;
{$endif not cpu64bitalu and not cpuhighleveltarget}

        begin
           if assigned(t^.less) then
             genitem(t^.less);
           if t^._low=t^._high then
             begin
{$ifndef cpuhighleveltarget}
{$if defined(cpu32bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, OC_NE, aint(hi(int64(t^._low.svalue))),hregister2,l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, OC_EQ, aint(lo(int64(t^._low.svalue))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$elseif defined(cpu16bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(hi(hi(int64(t^._low.svalue)))),cg.GetNextReg(hregister2),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(lo(hi(int64(t^._low.svalue)))),hregister2,l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(hi(lo(int64(t^._low.svalue)))),cg.GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_EQ, aint(lo(lo(int64(t^._low.svalue)))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(hi(int32(t^._low.svalue))),cg.GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_EQ, aint(lo(int32(t^._low.svalue))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$elseif defined(cpu8bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(hi(hi(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister2))),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(hi(hi(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister2)),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(lo(hi(int64(t^._low.svalue))))),cg.GetNextReg(hregister2),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(lo(hi(int64(t^._low.svalue))))),hregister2,l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(hi(lo(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(hi(lo(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister)),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(lo(lo(int64(t^._low.svalue))))),cg.GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_EQ, aint(lo(lo(lo(int64(t^._low.svalue))))),hregister,blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(hi(int32(t^._low.svalue)))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(hi(int32(t^._low.svalue)))),cg.GetNextReg(cg.GetNextReg(hregister)),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(lo(int32(t^._low.svalue)))),cg.GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_EQ, aint(lo(lo(int32(t^._low.svalue)))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S16,OS_16] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(int16(t^._low.svalue))),cg.GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_EQ, aint(lo(int16(t^._low.svalue))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$endif}
{$endif cpuhighleveltarget}
                  begin
                     hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, OC_EQ, tcgint(t^._low.svalue),hregister, blocklabel(t^.blockid));
                  end;
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
                  begin
{$ifndef cpuhighleveltarget}
{$if defined(cpu32bitalu)}
                     if def_cgsize(opsize) in [OS_64,OS_S64] then
                       begin
                          current_asmdata.getjumplabel(l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, jmp_lt, aint(hi(int64(t^._low.svalue))),
                               hregister2, elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, jmp_gt, aint(hi(int64(t^._low.svalue))),
                               hregister2, l1);
                          { the comparisation of the low dword must be always unsigned! }
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, OC_B, aint(lo(int64(t^._low.svalue))), hregister, elselabel);
                          cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else
{$elseif defined(cpu16bitalu)}
                     if def_cgsize(opsize) in [OS_64,OS_S64] then
                       begin
                          current_asmdata.getjumplabel(l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(hi(int64(t^._low.svalue)))),
                               cg.GetNextReg(hregister2), elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(hi(int64(t^._low.svalue)))),
                               cg.GetNextReg(hregister2), l1);
                          { the comparison of the low words must be always unsigned! }
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(hi(int64(t^._low.svalue)))),
                               hregister2, elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(lo(hi(int64(t^._low.svalue)))),
                               hregister2, l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(hi(lo(int64(t^._low.svalue)))),
                               cg.GetNextReg(hregister), elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(hi(lo(int64(t^._low.svalue)))),
                               cg.GetNextReg(hregister), l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(lo(int64(t^._low.svalue)))), hregister, elselabel);
                          cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else if def_cgsize(opsize) in [OS_32,OS_S32] then
                       begin
                          current_asmdata.getjumplabel(l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(int32(t^._low.svalue))),
                               cg.GetNextReg(hregister), elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(int32(t^._low.svalue))),
                               cg.GetNextReg(hregister), l1);
                          { the comparisation of the low dword must be always unsigned! }
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(int32(t^._low.svalue))), hregister, elselabel);
                          cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else
{$elseif defined(cpu8bitalu)}
                     if def_cgsize(opsize) in [OS_64,OS_S64] then
                       begin
                         current_asmdata.getjumplabel(l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(hi(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister2))),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(hi(hi(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister2))),l1);
                         { the comparison of the low words must be always unsigned! }
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(hi(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister2)),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(hi(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister2)),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(hi(int64(t^._low.svalue))))),cg.GetNextReg(hregister2),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(hi(int64(t^._low.svalue))))),cg.GetNextReg(hregister2),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(hi(int64(t^._low.svalue))))),hregister2,elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(lo(hi(int64(t^._low.svalue))))),hregister2,l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(hi(lo(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(hi(lo(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(lo(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister)),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(lo(int64(t^._low.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister)),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(lo(int64(t^._low.svalue))))),cg.GetNextReg(hregister),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(lo(int64(t^._low.svalue))))),cg.GetNextReg(hregister),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(lo(int64(t^._low.svalue))))),hregister,elselabel);
                         cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else if def_cgsize(opsize) in [OS_32,OS_S32] then
                       begin
                         current_asmdata.getjumplabel(l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(int32(t^._low.svalue)))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8,jmp_gt,aint(hi(hi(int32(t^._low.svalue)))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),l1);
                         { the comparison of the low words must be always unsigned! }
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(int32(t^._low.svalue)))),cg.GetNextReg(cg.GetNextReg(hregister)),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(int32(t^._low.svalue)))),cg.GetNextReg(cg.GetNextReg(hregister)),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(int32(t^._low.svalue)))),cg.GetNextReg(hregister),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(int32(t^._low.svalue)))),cg.GetNextReg(hregister),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(int32(t^._low.svalue)))),hregister,elselabel);
                         cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else if def_cgsize(opsize) in [OS_16,OS_S16] then
                       begin
                         current_asmdata.getjumplabel(l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(int16(t^._low.svalue))),cg.GetNextReg(hregister),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(int16(t^._low.svalue))),cg.GetNextReg(hregister),l1);
                         { the comparisation of the low dword must be always unsigned! }
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(int16(t^._low.svalue))),hregister,elselabel);
                         cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else
{$endif}
{$endif cpuhighleveltarget}
                       begin
                        hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, jmp_lt, tcgint(t^._low.svalue), hregister,
                           elselabel);
                       end;
                  end;
{$ifndef cpuhighleveltarget}
{$if defined(cpu32bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, jmp_lt, aint(hi(int64(t^._high.svalue))), hregister2,
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, jmp_gt, aint(hi(int64(t^._high.svalue))), hregister2,
                           l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_32, OC_BE, aint(lo(int64(t^._high.svalue))), hregister, blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$elseif defined(cpu16bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(hi(int64(t^._high.svalue)))), cg.GetNextReg(hregister2),
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(hi(int64(t^._high.svalue)))), cg.GetNextReg(hregister2),
                           l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(hi(int64(t^._high.svalue)))), hregister2,
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(lo(hi(int64(t^._high.svalue)))), hregister2,
                           l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(hi(lo(int64(t^._high.svalue)))), cg.GetNextReg(hregister),
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(hi(lo(int64(t^._high.svalue)))), cg.GetNextReg(hregister),
                           l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_BE, aint(lo(lo(int64(t^._high.svalue)))), hregister, blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(int32(t^._high.svalue))), cg.GetNextReg(hregister),
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(int32(t^._high.svalue))), cg.GetNextReg(hregister),
                           l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_BE, aint(lo(int32(t^._high.svalue))), hregister, blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$elseif defined(cpu8bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                    current_asmdata.getjumplabel(l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(hi(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister2))),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(hi(hi(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister2))),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(hi(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister2)),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(hi(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister2)),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(hi(int64(t^._high.svalue))))),cg.GetNextReg(hregister2),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(hi(int64(t^._high.svalue))))),cg.GetNextReg(hregister2),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(hi(int64(t^._high.svalue))))),hregister2,blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(lo(hi(int64(t^._high.svalue))))),hregister2,l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(hi(lo(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(hi(lo(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(lo(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister)),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(lo(int64(t^._high.svalue))))),cg.GetNextReg(cg.GetNextReg(hregister)),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(lo(int64(t^._high.svalue))))),cg.GetNextReg(hregister),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(lo(int64(t^._high.svalue))))),cg.GetNextReg(hregister),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_BE,aint(lo(lo(lo(int64(t^._high.svalue))))),hregister,blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                    current_asmdata.getjumplabel(l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(int32(t^._high.svalue)))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(hi(int32(t^._high.svalue)))),cg.GetNextReg(cg.GetNextReg(cg.GetNextReg(hregister))),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(int32(t^._high.svalue)))),cg.GetNextReg(cg.GetNextReg(hregister)),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(int32(t^._high.svalue)))),cg.GetNextReg(cg.GetNextReg(hregister)),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(int32(t^._high.svalue)))),cg.GetNextReg(hregister),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(int32(t^._high.svalue)))),cg.GetNextReg(hregister),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_BE,aint(lo(lo(int32(t^._high.svalue)))),hregister,blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S16,OS_16] then
                  begin
                    current_asmdata.getjumplabel(l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(int16(t^._high.svalue))),cg.GetNextReg(hregister),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(int16(t^._high.svalue))),cg.GetNextReg(hregister),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_BE,aint(lo(int16(t^._high.svalue))),hregister,blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$endif}
{$endif cpuhighleveltarget}
                  begin
                     hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, jmp_le, tcgint(t^._high.svalue), hregister, blocklabel(t^.blockid));
                  end;

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
         hlcg.a_jmp_always(current_asmdata.CurrAsmList,elselabel);
      end;


      procedure tcgcasenode.genjmptreeentry(p : pcaselabel;parentvalue : TConstExprInt);
        var
          lesslabel,greaterlabel : tasmlabel;
        begin
          current_asmdata.CurrAsmList.concat(cai_align.Create(current_settings.alignment.jumpalign));
          cg.a_label(current_asmdata.CurrAsmList,p^.labellabel);

          { calculate labels for left and right }
          if p^.less=nil then
            lesslabel:=elselabel
          else
            lesslabel:=p^.less^.labellabel;
          if p^.greater=nil then
            greaterlabel:=elselabel
          else
            greaterlabel:=p^.greater^.labellabel;

          { calculate labels for left and right }
          { no range label: }
          if p^._low=p^._high then
            begin
              if greaterlabel=lesslabel then
                hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, OC_NE,p^._low,hregister, lesslabel)
              else
                begin
                  hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize, jmp_lt,p^._low,hregister, lesslabel);
                  hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize, jmp_gt,p^._low,hregister, greaterlabel);
                end;
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,blocklabel(p^.blockid));
            end
          else
            begin
              hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_lt,p^._low, hregister, lesslabel);
              hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opsize,jmp_gt,p^._high,hregister, greaterlabel);
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,blocklabel(p^.blockid));
            end;
           if assigned(p^.less) then
             genjmptreeentry(p^.less,p^._low);
           if assigned(p^.greater) then
             genjmptreeentry(p^.greater,p^._high);
        end;


    procedure tcgcasenode.genjmptree(root : pcaselabel);
      type
        tlabelarrayentry = record
          caselabel : pcaselabel;
          asmlabel : TAsmLabel;
        end;
        tlabelarray = array of tlabelarrayentry;
      var
        labelarray : tlabelarray;

      var
        nextarrayentry : int64;
        i : longint;

      procedure addarrayentry(entry : pcaselabel);
        begin
          if assigned(entry^.less) then
            addarrayentry(entry^.less);
          with labelarray[nextarrayentry] do
            begin
              caselabel:=entry;
              current_asmdata.getjumplabel(asmlabel);
            end;
          inc(nextarrayentry);
          if assigned(entry^.greater) then
            addarrayentry(entry^.greater);
        end;

      { rebuild the label tree balanced }
      procedure rebuild(first,last : int64;var p : pcaselabel);
        var
          current : int64;
        begin
          current:=(first+last) div 2;

          p:=labelarray[current].caselabel;
          if first<current then
            rebuild(first,current-1,p^.less)
          else
            p^.less:=nil;

          if last>current then
            rebuild(current+1,last,p^.greater)
          else
            p^.greater:=nil;
        end;

      begin
        labelarray:=nil;
        SetLength(labelarray,labelcnt);
        nextarrayentry:=0;
        addarrayentry(root);
        rebuild(0,high(labelarray),root);
        for i:=0 to high(labelarray) do
          current_asmdata.getjumplabel(labelarray[i].caselabel^.labellabel);
        genjmptreeentry(root,root^._high+10);
      end;

    procedure tcgcasenode.pass_generate_code;
      var
         oldflowcontrol: tflowcontrol;
         i : longint;
         dist : asizeuint;
         distv,
         lv,hv,
         max_label: tconstexprint;
         max_linear_list : int64;
         max_dist : qword;
         ShortcutElse: Boolean;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         oldflowcontrol := flowcontrol;
         include(flowcontrol,fc_inflowcontrol);
         { Allocate labels }

         current_asmdata.getjumplabel(endlabel);

         { Do some optimisation to deal with empty else blocks }
         ShortcutElse := GetBranchLabel(elseblock, elselabel);

         for i:=0 to blocks.count-1 do
           with pcaseblock(blocks[i])^ do
             shortcut := GetBranchLabel(statement, blocklabel);

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
              if cs_opt_level1 in current_settings.optimizerswitches then
                begin
                   { procedures are empirically passed on }
                   { consumption can also be calculated   }
                   { but does it pay on the different     }
                   { processors?                       }
                   { moreover can the size only be appro- }
                   { ximated as it is not known if rel8,  }
                   { rel16 or rel32 jumps are used   }

                   max_label := case_get_max(labels);

                   { can we omit the range check of the jump table ? }
                   getrange(left.resultdef,lv,hv);
                   jumptable_no_range:=(lv=min_label) and (hv=max_label);

                   distv:=max_label-min_label;
                   if distv>=0 then
                     dist:=distv.uvalue
                   else
                     dist:=asizeuint(-distv.svalue);

                   { optimize for size ? }
                   if cs_opt_size in current_settings.optimizerswitches  then
                     begin
                       if has_jumptable and
                          (min_label>=int64(low(aint))) and
                          (max_label<=high(aint)) and
                          not((labelcnt<=2) or
                              (distv.svalue<0) or
                              (dist>3*labelcnt)) then
                         begin
                           { if the labels less or more a continuum then }
                           genjumptable(labels,min_label.svalue,max_label.svalue);
                         end
                       else
                         begin
                           { a linear list is always smaller than a jump tree }
                           genlinearlist(labels);
                         end;
                     end
                   else
                     begin
                        max_dist:=4*labelcoverage;

                        { Don't allow jump tables to get too large }
                        if max_dist>4*labelcnt then
                          max_dist:=min(max_dist,2048);

                        if jumptable_no_range then
                          max_linear_list:=4
                        else
                          max_linear_list:=2;

                        { allow processor specific values }
                        optimizevalues(max_linear_list,max_dist);

                        if (labelcnt<=max_linear_list) then
                          genlinearlist(labels)
                        else
                          begin
                            if (has_jumptable) and
                               (dist<max_dist) and
                               (min_label>=int64(low(aint))) and
                               (max_label<=high(aint)) then
                              genjumptable(labels,min_label.svalue,max_label.svalue)
                            { value has been determined on an i7-4770 using a random case with random values
                              if more values are known, this can be handled depending on the target CPU
                              
                              Testing on a Core 2 Duo E6850 as well as on a Raspi3 showed also, that 64 is
                              a good value }
                            else if labelcnt>=64 then
                              genjmptree(labels)
                            else
                              genlinearlist(labels);
                          end;
                     end;
                end
              else
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
                 current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
                 cg.a_label(current_asmdata.CurrAsmList,blocklabel);
                 secondpass(statement);
                 { don't come back to case line }
                 current_filepos:=current_asmdata.CurrAsmList.getlasttaifilepos^;
{$ifdef OLDREGVARS}
                 load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
                 hlcg.a_jmp_always(current_asmdata.CurrAsmList,endlabel);
               end;
           end;

         { ...and the else block }
         if not ShortcutElse then
           begin
             current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
             hlcg.a_label(current_asmdata.CurrAsmList,elselabel);
           end;

         if Assigned(elseblock) then
           begin

             secondpass(elseblock);
{$ifdef OLDREGVARS}
             load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
           end;

         current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
         hlcg.a_label(current_asmdata.CurrAsmList,endlabel);

         { Reset labels }
         for i:=0 to blocks.count-1 do
           pcaseblock(blocks[i])^.blocklabel:=nil;
         flowcontrol := oldflowcontrol + (flowcontrol - [fc_inflowcontrol]);
      end;


begin
   csetelementnode:=tcgsetelementnode;
   cinnode:=tcginnode;
   ccasenode:=tcgcasenode;
end.
