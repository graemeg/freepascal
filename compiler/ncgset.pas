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
<<<<<<< HEAD
       globtype,globals,constexp,symtype,
=======
       globtype,globals,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
       { tcginnode }

       tcginnode = class(tinnode)
         procedure in_smallset(opdef: tdef; setbase: aint); virtual;

         function pass_1: tnode;override;
         procedure pass_generate_code;override;
       protected
         function checkgenjumps(out setparts: Tsetparts; out numparts: byte; out use_small: boolean): boolean; virtual;
         function analizeset(const Aset:Tconstset;out setparts: Tsetparts; out numparts: byte;is_small:boolean):boolean;virtual;
=======
       tcginnode = class(tinnode)
          function pass_1: tnode;override;
          procedure pass_generate_code;override;
       protected
          function checkgenjumps(out setparts: Tsetparts; out numparts: byte; out use_small: boolean): boolean; virtual;
          function analizeset(const Aset:Tconstset;out setparts: Tsetparts; out numparts: byte;is_small:boolean):boolean;virtual;
>>>>>>> graemeg/fixes_2_2
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

          function  blocklabel(id:longint):tasmlabel;
          procedure optimizevalues(var max_linear_list:aint;var max_dist:aword);virtual;
          function  has_jumptable : boolean;virtual;
          procedure genjumptable(hp : pcaselabel;min_,max_ : aint); virtual;
          procedure genlinearlist(hp : pcaselabel); virtual;
          procedure genlinearcmplist(hp : pcaselabel); virtual;
       end;


implementation

    uses
      systems,
      verbose,
      symconst,symdef,defutil,
      paramgr,
      procinfo,pass_2,tgobj,
      nbas,ncon,nflw,
<<<<<<< HEAD
<<<<<<< HEAD
      ncgutil,hlcgobj;
=======
      ncgutil;
>>>>>>> graemeg/fixes_2_2
=======
      ncgutil;
>>>>>>> origin/fixes_2_2


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
<<<<<<< HEAD
         use_small:=is_smallset(right.resultdef) and
                    not is_signed(left.resultdef) and
=======
         use_small:=(tsetdef(right.resultdef).settype=smallset) and not is_signed(left.resultdef) and
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
         setbase    : aint;
<<<<<<< HEAD
<<<<<<< HEAD
         l, l2      : tasmlabel;
         hr,
=======
=======
>>>>>>> origin/fixes_2_2
				 l,l2,
         otl, ofl   : tasmlabel;
         hr,hr2,
>>>>>>> graemeg/fixes_2_2
         pleftreg   : tregister;
         href       : treference;
         setparts   : Tsetparts;
         opsize     : tcgsize;
         opdef      : tdef;
         uopsize    : tcgsize;
         uopdef     : tdef;
         orgopsize  : tcgsize;
         genjumps,
         use_small,
         isjump     : boolean;
         i,numparts : byte;
         needslabel : Boolean;
       begin
         l2:=nil;

         { We check first if we can generate jumps, this can be done
           because the resultdef is already set in firstpass }

         genjumps := checkgenjumps(setparts,numparts,use_small);

         orgopsize := def_cgsize(left.resultdef);
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
         needslabel := false;

<<<<<<< HEAD
<<<<<<< HEAD
         if not genjumps then
           { calculate both operators }
           { the complex one first }
           { not in case of genjumps, because then we don't secondpass      }
=======
=======
>>>>>>> origin/fixes_2_2
         isjump:=false;
         if (left.expectloc=LOC_JUMP) then
           begin
             otl:=current_procinfo.CurrTrueLabel;
             current_asmdata.getjumplabel(current_procinfo.CurrTrueLabel);
             ofl:=current_procinfo.CurrFalseLabel;
             current_asmdata.getjumplabel(current_procinfo.CurrFalseLabel);
             isjump:=true;
           end
         else if not genjumps then
           { calculate both operators }
           { the complex one first }
           { only if left will not be a LOC_JUMP, to keep complexity in the }
           { code generator down. This almost never happens anyway, only in }
           { case like "if ((a in someset) in someboolset) then" etc        }
           { also not in case of genjumps, because then we don't secondpass }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           { right at all (so we have to make sure that "right" really is   }
           { "right" and not "swapped left" in that case)                   }
           firstcomplex(self);

         secondpass(left);
<<<<<<< HEAD
<<<<<<< HEAD
         if (left.expectloc=LOC_JUMP)<>
            (left.location.loc=LOC_JUMP) then
=======
=======
>>>>>>> origin/fixes_2_2
         if isjump then
           begin
             location_force_reg(current_asmdata.CurrAsmList,left.location,opsize,true);
             current_procinfo.CurrTrueLabel:=otl;
             current_procinfo.CurrFalseLabel:=ofl;
           end
         else if (left.location.loc=LOC_JUMP) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
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
                       left.resultdef,right.resultdef,uopdef,
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
                         current_asmdata.getjumplabel(l);
                         current_asmdata.getjumplabel(l2);
                         needslabel := True;
=======
          begin
            { location is always LOC_REGISTER }
            location_reset(location, LOC_REGISTER, uopsize{def_cgsize(resultdef)});
            { allocate a register for the result }
            location.register := cg.getintregister(current_asmdata.CurrAsmList, uopsize);

            { We will now generated code to check the set itself, no jmps,
              handle smallsets separate, because it allows faster checks }
            if use_small then
             begin
               {****************************  SMALL SET **********************}
               if left.location.loc=LOC_CONSTANT then
                begin
                  cg.a_bit_test_const_loc_reg(current_asmdata.CurrAsmList,location.size,
                    left.location.value-setbase,right.location,
                    location.register);
                end
               else
                begin
                  location_force_reg(current_asmdata.CurrAsmList,left.location,opsize,true);
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,left.location,setbase);
                  cg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList,left.location.size,
                    location.size,left.location.register,right.location,location.register);
                end;
             end
            else
             {************************** NOT SMALL SET ********************}
             begin
               if right.location.loc=LOC_CONSTANT then
                begin
                  { can it actually occur currently? CEC }
                  { yes: "if bytevar in [1,3,5,7,9,11,13,15]" (JM) }

                  { note: this code assumes that left in [0..255], which is a valid }
                  { assumption (other cases will be caught by range checking) (JM)  }

                  { load left in register }
                  location_force_reg(current_asmdata.CurrAsmList,left.location,location.size,true);
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,left.location,setbase);
<<<<<<< HEAD
<<<<<<< HEAD
                  { emit bit test operation -- warning: do not use
                    location_force_reg() to force a set into a register, except
                    to a register of the same size as the set. The reason is
                    that on big endian systems, this would require moving the
                    set to the most significant part of the new register,
                    and location_force_register can't do that (it does not
                    know the type).

                   a_bit_test_reg_loc_reg() properly takes into account the
                   size of the set to adjust the register index to test }
                  cg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList,
                    left.location.size,location.size,
                    left.location.register,right.location,location.register);
=======
=======
>>>>>>> origin/fixes_2_2
                  location_force_reg(current_asmdata.CurrAsmList,right.location,opsize,true);
                  { emit bit test operation }
                  cg.a_bit_test_reg_reg_reg(current_asmdata.CurrAsmList,
                    left.location.size,right.location.size,location.size,
                    left.location.register,right.location.register,location.register);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

                  { now zero the result if left > nr_of_bits_in_right_register }
                  hr := cg.getintregister(current_asmdata.CurrAsmList,location.size);
                  { if left > tcgsize2size[opsize]*8 then hr := 0 else hr := $ffffffff }
                  { (left.location.size = location.size at this point) }
                  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_SUB, location.size, tcgsize2size[opsize]*8, left.location.register, hr);
                  cg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SAR, location.size, (tcgsize2size[opsize]*8)-1, hr);

                  { if left > tcgsize2size[opsize]*8-1, then result := 0 else result := result of bit test }
                  cg.a_op_reg_reg(current_asmdata.CurrAsmList, OP_AND, location.size, hr, location.register);
                end { of right.location.loc=LOC_CONSTANT }
               { do search in a normal set which could have >32 elements
                 but also used if the left side contains higher values > 32 }
               else if (left.location.loc=LOC_CONSTANT) then
                begin
                  if (left.location.value < setbase) or (((left.location.value-setbase) shr 3) >= right.resultdef.size) then
                    {should be caught earlier }
                    internalerror(2007020402);
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
<<<<<<< HEAD
                         hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opdef, OC_BE, tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase, pleftreg, l);

                         hlcg.a_load_const_reg(current_asmdata.CurrAsmList, uopdef, 0, location.register);
                         hlcg.a_jmp_always(current_asmdata.CurrAsmList, l2);
=======
=======
>>>>>>> origin/fixes_2_2
                  cg.a_bit_test_const_loc_reg(current_asmdata.CurrAsmList,location.size,left.location.value-setbase,
                    right.location,location.register);
                end
               else
                begin
                  location_force_reg(current_asmdata.CurrAsmList, left.location, opsize, true);
                  register_maybe_adjust_setbase(current_asmdata.CurrAsmList,left.location,setbase);
                  pleftreg := left.location.register;

                  if (opsize >= OS_S8) or { = if signed }
                     ((left.resultdef.typ=orddef) and 
                      ((torddef(left.resultdef).low < int64(tsetdef(right.resultdef).setbase)) or
                       (torddef(left.resultdef).high > int64(tsetdef(right.resultdef).setmax)))) or
                     ((left.resultdef.typ=enumdef) and
                      ((tenumdef(left.resultdef).min < tsetdef(right.resultdef).setbase) or
                       (tenumdef(left.resultdef).max > tsetdef(right.resultdef).setmax))) then
                    begin
                      current_asmdata.getjumplabel(l);
                      current_asmdata.getjumplabel(l2);
                      needslabel := True;

                      cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, left.location.size, OC_BE, tsetdef(right.resultdef).setmax-tsetdef(right.resultdef).setbase, pleftreg, l);

                      cg.a_load_const_reg(current_asmdata.CurrAsmList, location.size, 0, location.register);
                      cg.a_jmp_always(current_asmdata.CurrAsmList, l2);

                      cg.a_label(current_asmdata.CurrAsmList, l);
                    end;

                  cg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList,left.location.size,location.size,
                    pleftreg,right.location,location.register);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

                         hlcg.a_label(current_asmdata.CurrAsmList, l);
                       end;

                     hlcg.a_bit_test_reg_loc_reg(current_asmdata.CurrAsmList,opdef,right.resultdef,uopdef,
                       pleftreg,right.location,location.register);

                     if needslabel then
                       hlcg.a_label(current_asmdata.CurrAsmList, l2);
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

    function tcgcasenode.blocklabel(id:longint):tasmlabel;
      begin
        if not assigned(blocks[id]) then
          internalerror(200411301);
        result:=pcaseblock(blocks[id])^.blocklabel;
      end;


    procedure tcgcasenode.optimizevalues(var max_linear_list:aint;var max_dist:aword);
      begin
        { no changes by default }
      end;


    function tcgcasenode.has_jumptable : boolean;
      begin
        { No jumptable support in the default implementation }
        has_jumptable:=false;
      end;


    procedure tcgcasenode.genjumptable(hp : pcaselabel;min_,max_ : aint);
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

      procedure genitem(t : pcaselabel);

          procedure gensub(value:tcgint);
            begin
              { here, since the sub and cmp are separate we need
                to move the result before subtract to help
                the register allocator
              }
              hlcg.a_load_reg_reg(current_asmdata.CurrAsmList, opsize, opsize, hregister, scratch_reg);
              hlcg.a_op_const_reg(current_asmdata.CurrAsmList, OP_SUB, opsize, value, hregister);
            end;

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
<<<<<<< HEAD
<<<<<<< HEAD
                       gensub(tcgint(t^._low.svalue));
=======
=======
>>>>>>> origin/fixes_2_2
                       gensub(aint(t^._low));
>>>>>>> graemeg/fixes_2_2
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
              last:=0;
              first:=true;
              scratch_reg:=hlcg.getintregister(current_asmdata.CurrAsmList,opsize);
              genitem(hp);
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,elselabel);
           end;
      end;


    procedure tcgcasenode.genlinearcmplist(hp : pcaselabel);

      var
         last : TConstExprInt;
         lastwasrange: boolean;

      procedure genitem(t : pcaselabel);

{$ifndef cpu64bitalu}
        var
           l1 : tasmlabel;
{$endif not cpu64bitalu}

        begin
           if assigned(t^.less) then
             genitem(t^.less);
           if t^._low=t^._high then
             begin
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
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(hi(hi(int64(t^._low.svalue)))),GetNextReg(hregister2),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(lo(hi(int64(t^._low.svalue)))),hregister2,l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(hi(lo(int64(t^._low.svalue)))),GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_EQ, aint(lo(lo(int64(t^._low.svalue)))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_NE, aint(hi(int32(t^._low.svalue))),GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_EQ, aint(lo(int32(t^._low.svalue))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$elseif defined(cpu8bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(hi(hi(int64(t^._low.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister2))),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(hi(hi(int64(t^._low.svalue))))),GetNextReg(GetNextReg(hregister2)),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(lo(hi(int64(t^._low.svalue))))),GetNextReg(hregister2),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(lo(hi(int64(t^._low.svalue))))),hregister2,l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(hi(lo(int64(t^._low.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister))),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(hi(lo(int64(t^._low.svalue))))),GetNextReg(GetNextReg(hregister)),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(lo(lo(int64(t^._low.svalue))))),GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_EQ, aint(lo(lo(lo(int64(t^._low.svalue))))),hregister,blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(hi(int32(t^._low.svalue)))),GetNextReg(GetNextReg(GetNextReg(hregister))),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(lo(hi(int32(t^._low.svalue)))),GetNextReg(GetNextReg(hregister)),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(lo(int32(t^._low.svalue)))),GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_EQ, aint(lo(lo(int32(t^._low.svalue)))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S16,OS_16] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_NE, aint(hi(int16(t^._low.svalue))),GetNextReg(hregister),l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8, OC_EQ, aint(lo(int16(t^._low.svalue))),hregister, blocklabel(t^.blockid));
                     cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$endif}
                  begin
                     hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, OC_EQ, aint(t^._low.svalue),hregister, blocklabel(t^.blockid));
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
                               GetNextReg(hregister2), elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(hi(int64(t^._low.svalue)))),
                               GetNextReg(hregister2), l1);
                          { the comparison of the low words must be always unsigned! }
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(hi(int64(t^._low.svalue)))),
                               hregister2, elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(lo(hi(int64(t^._low.svalue)))),
                               hregister2, l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(hi(lo(int64(t^._low.svalue)))),
                               GetNextReg(hregister), elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(hi(lo(int64(t^._low.svalue)))),
                               GetNextReg(hregister), l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(lo(int64(t^._low.svalue)))), hregister, elselabel);
                          cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else if def_cgsize(opsize) in [OS_32,OS_S32] then
                       begin
                          current_asmdata.getjumplabel(l1);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(int32(t^._low.svalue))),
                               GetNextReg(hregister), elselabel);
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(int32(t^._low.svalue))),
                               GetNextReg(hregister), l1);
                          { the comparisation of the low dword must be always unsigned! }
                          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(int32(t^._low.svalue))), hregister, elselabel);
                          cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else
{$elseif defined(cpu8bitalu)}
                     if def_cgsize(opsize) in [OS_64,OS_S64] then
                       begin
                         current_asmdata.getjumplabel(l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(hi(int64(t^._low.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister2))),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(hi(hi(int64(t^._low.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister2))),l1);
                         { the comparison of the low words must be always unsigned! }
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(hi(int64(t^._low.svalue))))),GetNextReg(GetNextReg(hregister2)),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(hi(int64(t^._low.svalue))))),GetNextReg(GetNextReg(hregister2)),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(hi(int64(t^._low.svalue))))),GetNextReg(hregister2),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(hi(int64(t^._low.svalue))))),GetNextReg(hregister2),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(hi(int64(t^._low.svalue))))),hregister2,elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(lo(hi(int64(t^._low.svalue))))),hregister2,l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(hi(lo(int64(t^._low.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister))),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(hi(lo(int64(t^._low.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister))),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(lo(int64(t^._low.svalue))))),GetNextReg(GetNextReg(hregister)),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(lo(int64(t^._low.svalue))))),GetNextReg(GetNextReg(hregister)),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(lo(int64(t^._low.svalue))))),GetNextReg(hregister),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(lo(int64(t^._low.svalue))))),GetNextReg(hregister),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(lo(int64(t^._low.svalue))))),hregister,elselabel);
                         cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else if def_cgsize(opsize) in [OS_32,OS_S32] then
                       begin
                         current_asmdata.getjumplabel(l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(int32(t^._low.svalue)))),GetNextReg(GetNextReg(GetNextReg(hregister))),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_8,jmp_gt,aint(hi(hi(int32(t^._low.svalue)))),GetNextReg(GetNextReg(GetNextReg(hregister))),l1);
                         { the comparison of the low words must be always unsigned! }
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(int32(t^._low.svalue)))),GetNextReg(GetNextReg(hregister)),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(int32(t^._low.svalue)))),GetNextReg(GetNextReg(hregister)),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(int32(t^._low.svalue)))),GetNextReg(hregister),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(int32(t^._low.svalue)))),GetNextReg(hregister),l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(int32(t^._low.svalue)))),hregister,elselabel);
                         cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else if def_cgsize(opsize) in [OS_16,OS_S16] then
                       begin
                         current_asmdata.getjumplabel(l1);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(int16(t^._low.svalue))),GetNextReg(hregister),elselabel);
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(int16(t^._low.svalue))),GetNextReg(hregister),l1);
                         { the comparisation of the low dword must be always unsigned! }
                         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(int16(t^._low.svalue))),hregister,elselabel);
                         cg.a_label(current_asmdata.CurrAsmList,l1);
                       end
                     else
{$endif}
                       begin
                        hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, jmp_lt, aint(t^._low.svalue), hregister,
                           elselabel);
                       end;
                  end;
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
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(hi(int64(t^._high.svalue)))), GetNextReg(hregister2),
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(hi(int64(t^._high.svalue)))), GetNextReg(hregister2),
                           l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(lo(hi(int64(t^._high.svalue)))), hregister2,
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(lo(hi(int64(t^._high.svalue)))), hregister2,
                           l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_B, aint(hi(lo(int64(t^._high.svalue)))), GetNextReg(hregister),
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_A, aint(hi(lo(int64(t^._high.svalue)))), GetNextReg(hregister),
                           l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_BE, aint(lo(lo(int64(t^._high.svalue)))), hregister, blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                     current_asmdata.getjumplabel(l1);
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_lt, aint(hi(int32(t^._high.svalue))), GetNextReg(hregister),
                           blocklabel(t^.blockid));
                     cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, jmp_gt, aint(hi(int32(t^._high.svalue))), GetNextReg(hregister),
                           l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_16, OC_BE, aint(lo(int32(t^._high.svalue))), hregister, blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$elseif defined(cpu8bitalu)}
                if def_cgsize(opsize) in [OS_S64,OS_64] then
                  begin
                    current_asmdata.getjumplabel(l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(hi(int64(t^._high.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister2))),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(hi(hi(int64(t^._high.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister2))),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(hi(int64(t^._high.svalue))))),GetNextReg(GetNextReg(hregister2)),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(hi(int64(t^._high.svalue))))),GetNextReg(GetNextReg(hregister2)),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(hi(int64(t^._high.svalue))))),GetNextReg(hregister2),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(hi(int64(t^._high.svalue))))),GetNextReg(hregister2),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(lo(hi(int64(t^._high.svalue))))),hregister2,blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(lo(hi(int64(t^._high.svalue))))),hregister2,l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(hi(lo(int64(t^._high.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister))),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(hi(lo(int64(t^._high.svalue))))),GetNextReg(GetNextReg(GetNextReg(hregister))),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(lo(int64(t^._high.svalue))))),GetNextReg(GetNextReg(hregister)),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(lo(int64(t^._high.svalue))))),GetNextReg(GetNextReg(hregister)),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(lo(int64(t^._high.svalue))))),GetNextReg(hregister),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(lo(int64(t^._high.svalue))))),GetNextReg(hregister),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_BE,aint(lo(lo(lo(int64(t^._high.svalue))))),hregister,blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S32,OS_32] then
                  begin
                    current_asmdata.getjumplabel(l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(hi(int32(t^._high.svalue)))),GetNextReg(GetNextReg(GetNextReg(hregister))),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(hi(int32(t^._high.svalue)))),GetNextReg(GetNextReg(GetNextReg(hregister))),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(lo(hi(int32(t^._high.svalue)))),GetNextReg(GetNextReg(hregister)),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(lo(hi(int32(t^._high.svalue)))),GetNextReg(GetNextReg(hregister)),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_B,aint(hi(lo(int32(t^._high.svalue)))),GetNextReg(hregister),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_A,aint(hi(lo(int32(t^._high.svalue)))),GetNextReg(hregister),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_BE,aint(lo(lo(int32(t^._high.svalue)))),hregister,blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else if def_cgsize(opsize) in [OS_S16,OS_16] then
                  begin
                    current_asmdata.getjumplabel(l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_lt,aint(hi(int16(t^._high.svalue))),GetNextReg(hregister),blocklabel(t^.blockid));
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,jmp_gt,aint(hi(int16(t^._high.svalue))),GetNextReg(hregister),l1);
                    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_8,OC_BE,aint(lo(int16(t^._high.svalue))),hregister,blocklabel(t^.blockid));
                    cg.a_label(current_asmdata.CurrAsmList,l1);
                  end
                else
{$endif}
                  begin
                     hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, jmp_le, aint(t^._high.svalue), hregister, blocklabel(t^.blockid));
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


    procedure tcgcasenode.pass_generate_code;
      var
         oldflowcontrol: tflowcontrol;
         i : longint;
         distv,
         lv,hv,
         max_label: tconstexprint;
         labelcnt : tcgint;
         max_linear_list : aint;
         max_dist,
         dist : aword;
         oldexecutionweight : longint;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         oldflowcontrol := flowcontrol;
         include(flowcontrol,fc_inflowcontrol);
         { Allocate labels }
         current_asmdata.getjumplabel(endlabel);
         current_asmdata.getjumplabel(elselabel);
         for i:=0 to blocks.count-1 do
           current_asmdata.getjumplabel(pcaseblock(blocks[i])^.blocklabel);

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
{$ifndef cpu64bitalu}
         if def_cgsize(opsize) in [OS_S64,OS_64] then
           begin
             hregister:=left.location.register64.reglo;
             hregister2:=left.location.register64.reghi;
           end
         else
{$endif not cpu64bitalu}
           hregister:=left.location.register;

         { we need the min_label always to choose between }
         { cmps and subs/decs                             }
         min_label:=case_get_min(labels);

         { Generate the jumps }
{$ifdef OLDREGVARS}
         load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
{$ifndef cpu64bitalu}
         if def_cgsize(opsize) in [OS_64,OS_S64] then
           genlinearcmplist(labels)
         else
{$endif not cpu64bitalu}
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
                   max_label:=case_get_max(labels);
                   labelcnt:=case_count_labels(labels);
                   { can we omit the range check of the jump table ? }
                   getrange(left.resultdef,lv,hv);
                   jumptable_no_range:=(lv=min_label) and (hv=max_label);
                   { hack a little bit, because the range can be greater }
                   { than the positive range of a aint            }

                   if (min_label<0) and (max_label>0) then
                     distv:=max_label+min_label
                   else
                     distv:=max_label-min_label;
                   if (distv>=0) then
                     dist:=distv.uvalue
                   else
                     dist:=-distv.svalue;

                   { optimize for size ? }
                   if cs_opt_size in current_settings.optimizerswitches  then
                     begin
                       if has_jumptable and
                          (min_label>=int64(low(aint))) and
                          (max_label<=high(aint)) and
                          not((labelcnt<=2) or
                              ((max_label-min_label)<0) or
                              ((max_label-min_label)>3*labelcnt)) then
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
                        max_dist:=4*labelcnt;
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
                            else
                              genlinearlist(labels);
                          end;
                     end;
                end
              else
                { it's always not bad }
                genlinearlist(labels);
           end;

         { estimates the repeat of each instruction }
         oldexecutionweight:=cg.executionweight;
         cg.executionweight:=cg.executionweight div case_count_labels(labels);
         if cg.executionweight<1 then
           cg.executionweight:=1;

         { generate the instruction blocks }
         for i:=0 to blocks.count-1 do
           begin
              current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
              cg.a_label(current_asmdata.CurrAsmList,pcaseblock(blocks[i])^.blocklabel);
              secondpass(pcaseblock(blocks[i])^.statement);
              { don't come back to case line }
              current_filepos:=current_asmdata.CurrAsmList.getlasttaifilepos^;
{$ifdef OLDREGVARS}
              load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
              hlcg.a_jmp_always(current_asmdata.CurrAsmList,endlabel);
           end;
         current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
         { ...and the else block }
         hlcg.a_label(current_asmdata.CurrAsmList,elselabel);
         if assigned(elseblock) then
           begin
              secondpass(elseblock);
{$ifdef OLDREGVARS}
              load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
           end;
<<<<<<< HEAD
<<<<<<< HEAD

         cg.executionweight:=oldexecutionweight;

         current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
         hlcg.a_label(current_asmdata.CurrAsmList,endlabel);
=======
=======
>>>>>>> origin/fixes_2_2
         current_asmdata.CurrAsmList.concat(cai_align.create(current_settings.alignment.jumpalign));
         cg.a_label(current_asmdata.CurrAsmList,endlabel);
>>>>>>> graemeg/fixes_2_2

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
