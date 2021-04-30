{
    Copyright (c) 1998-2002, 2014 by Florian Klaempfl and Jonas Maebe

    Generate AArch64 assembler for math nodes

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
unit ncpumat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat;

    type
      taarch64moddivnode = class(tmoddivnode)
         function pass_1: tnode; override;
         procedure pass_generate_code;override;
      end;

      taarch64notnode = class(tcgnotnode)
         procedure second_boolean;override;
      end;

      taarch64unaryminusnode = class(tcgunaryminusnode)
         procedure second_float; override;
      end;

implementation

    uses
      globtype,systems,constexp,
      cutils,verbose,globals,
      symconst,symdef,
      aasmbase,aasmcpu,aasmtai,aasmdata,
      defutil,
      cgbase,cgobj,hlcgobj,pass_2,procinfo,
      ncon,
      cpubase,
      ncgutil,cgcpu,cgutils;

{*****************************************************************************
                             taarch64moddivnode
*****************************************************************************}

    function taarch64moddivnode.pass_1: tnode;
      begin
        result:=inherited pass_1;
        if not assigned(result) then
          include(current_procinfo.flags,pi_do_call);
      end;


    procedure taarch64moddivnode.pass_generate_code;
      var
         op         : tasmop;
         tmpreg,
         zeroreg,
         numerator,
         divider,
         largernumreg,
         largerresreg,
         resultreg  : tregister;
         hl         : tasmlabel;
         overflowloc: tlocation;
         power      : longint;
         opsize     : tcgsize;

         dividend   : Int64;
         high_bit,
         reciprocal : QWord;
         { Just to save on stack space and the like }
         reciprocal_signed : Int64 absolute reciprocal;

         expandword,
         magic_add  : Boolean;
         shift      : byte;

         shifterop  : tshifterop;
         hp         : taicpu;

       procedure genOrdConstNodeDiv;
         var
           helper1, helper2: TRegister;
           so: tshifterop;
         begin
           if tordconstnode(right).value=0 then
             internalerror(2020021601)
           else if tordconstnode(right).value=1 then
             cg.a_load_reg_reg(current_asmdata.CurrAsmList, opsize, opsize, numerator, resultreg)
           else if (tordconstnode(right).value = int64(-1)) then
             begin
               // note: only in the signed case possible..., may overflow
               if cs_check_overflow in current_settings.localswitches then
                 cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);

               current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_NEG,
                 resultreg,numerator),toppostfix(ord(cs_check_overflow in current_settings.localswitches)*ord(PF_S))));
             end
           else if isabspowerof2(tordconstnode(right).value,power) then
             begin
               if (is_signed(right.resultdef)) then
                 begin
                    helper2:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                    if power = 1 then
                      helper1:=numerator
                    else
                      begin
                        helper1:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,opsize,resultdef.size*8-1,numerator,helper1);
                      end;
                    shifterop_reset(so);
                    so.shiftmode:=SM_LSR;
                    so.shiftimm:=resultdef.size*8-power;
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,helper2,numerator,helper1,so));
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,def_cgsize(resultdef),power,helper2,resultreg);

                    if (tordconstnode(right).value < 0) then
                      { Invert the result }
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_NEG,resultreg,resultreg));
                  end
                else
                  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,opsize,power,numerator,resultreg)
             end
           else
             { Generic division }
             begin
               if is_signed(left.resultdef) then
                 op:=A_SDIV
               else
                 op:=A_UDIV;

               { If we didn't acquire the original divisor earlier, grab it now }
               if divider = NR_NO then
                 begin
                   divider:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                   cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,tordconstnode(right).value.svalue,divider);
                 end;

               current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,resultreg,numerator,divider));
             end;
         end;

       procedure genOverflowCheck;
         begin
           { in case of overflow checking, also check for low(int64) div (-1)
             (no hardware support for this either) }
           if (cs_check_overflow in current_settings.localswitches) and
              is_signed(left.resultdef) and
              ((right.nodetype<>ordconstn) or
               (tordconstnode(right).value=-1)) then
             begin
               { num=ffff... and div=8000... <=>
                 num xor not(div xor 8000...) = 0
                 (and we have the "eon" operation, which performs "xor not(...)" }
               tmpreg:=hlcg.getintregister(current_asmdata.CurrAsmList,left.resultdef);
               hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_XOR,left.resultdef,low(int64),numerator,tmpreg);
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_EON,
                 tmpreg,numerator,tmpreg));
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,tmpreg,0));
               { now the zero/equal flag is set in case we divided low(int64) by
                 (-1) }
               location_reset(overflowloc,LOC_FLAGS,OS_NO);
               overflowloc.resflags:=F_EQ;
               cg.g_overflowcheck_loc(current_asmdata.CurrAsmList,location,resultdef,overflowloc);
             end;
         end;

      begin
        secondpass(left);
        secondpass(right);
        { avoid warning }
        divider := NR_NO;
        largernumreg := NR_NO;
        expandword := False;

        opsize := def_cgsize(resultdef);

        { set result location }
        location_reset(location,LOC_REGISTER,opsize);
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
        resultreg:=location.register;

        { put numerator in register }
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
        numerator:=left.location.register;

        if (right.nodetype=ordconstn) then
          begin
            { If optimising for size, just use regular division operations }
            if (cs_opt_size in current_settings.optimizerswitches) or
              ((tordconstnode(right).value=1) or
              (tordconstnode(right).value=int64(-1)) or
              isabspowerof2(tordconstnode(right).value,power)) then
              begin

                { Store divisor for later (and executed at the same time as the multiplication) }
                if (nodetype=modn) then
                  begin
                    if (tordconstnode(right).value = 1) or (tordconstnode(right).value = int64(-1)) then
                      begin
                        { Just evaluates to zero }
                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_MOVZ,resultreg, 0));
                        Exit;
                      end
                    { "not cs_opt_size" saves from checking the value of the divisor again
                      (if cs_opt_size is not set, then the divisor is a power of 2) }
                    else if not (cs_opt_size in current_settings.optimizerswitches) then
                      begin
                        divider:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                        cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,tordconstnode(right).value.svalue,divider);
                      end
                  end;

                genOrdConstNodeDiv;
                genOverflowCheck;

                { in case of modulo, multiply result again by the divider and subtract
                  from the numerator }
                if (nodetype=modn) then
                  begin
                    if ispowerof2(tordconstnode(right).value,power) then
                      begin
                        shifterop.shiftmode := SM_LSL;
                        shifterop.shiftimm := power;

                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_SUB,resultreg,numerator,resultreg,shifterop));
                      end
                    else
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_reg(A_MSUB,resultreg,
                        resultreg,divider,numerator));
                  end;

                Exit;
              end
            else
              begin
                if is_signed(left.resultdef) then
                  begin
                    if (nodetype=modn) then { Signed mod doesn't work properly }
                      begin
                        divider:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                        cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,tordconstnode(right).value.svalue,divider);
                        genOrdConstNodeDiv;
                      end
                    else
                      begin
                        { Read signed value to avoid Internal Error 200706094 }
                        dividend := tordconstnode(right).value.svalue;

                        calc_divconst_magic_signed(resultdef.size * 8, dividend, reciprocal_signed, shift);
                        cg.a_load_const_reg(current_asmdata.CurrAsmList, opsize, reciprocal_signed, resultreg);

                        { SMULH is only available for the full 64-bit registers }
                        if opsize in [OS_64, OS_S64] then
                          begin
                            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SMULH,resultreg,resultreg,numerator));
                            largerresreg := resultreg;
                          end
                        else
                          begin
                            largerresreg := newreg(getregtype(resultreg), getsupreg(resultreg), R_SUBWHOLE);
                            largernumreg := newreg(getregtype(numerator), getsupreg(numerator), R_SUBWHOLE);
                            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MUL,largerresreg,largerresreg,largernumreg));
                            expandword := True; { Merge the shift operation with something below }
                          end;

                        { Store divisor for later (and executed at the same time as the multiplication) }
                        if nodetype=modn then
                          begin
                            divider:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                            cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,dividend,divider);
                          end;

                        { add or subtract dividend }
                        if (dividend > 0) and (reciprocal_signed < 0) then
                          begin
                            if expandword then
                              begin
                                shifterop.shiftmode := SM_ASR;
                                shifterop.shiftimm := 32;
                                expandword := False;
                                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,largerresreg,largernumreg,largerresreg,shifterop));
                              end
                            else
                              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ADD,resultreg,resultreg,numerator));
                          end
                        else if (dividend < 0) and (reciprocal_signed > 0) then
                          begin
                            if expandword then
                              begin
                                { We can't append LSR to the SUB below because it's on the wrong operand }
                                expandword := False;
                                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ASR,largerresreg,largerresreg,32));
                              end;

                            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SUB,resultreg,resultreg,numerator));
                          end
                        else if expandword then
                          Inc(shift,32);

                        { shift if necessary }
                        if (shift <> 0) then
                          begin
                            if expandword then
                              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ASR,largerresreg,largerresreg,shift))
                            else
                              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ASR,resultreg,resultreg,shift));
                          end;

                        { extract and add the sign bit }
                        shifterop.shiftmode := SM_LSR;
                        shifterop.shiftimm := left.resultdef.size*8 - 1;

                        if (dividend < 0) then
                          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,resultreg,resultreg,resultreg,shifterop))
                        else
                          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,resultreg,resultreg,numerator,shifterop));
                      end;
                  end
                else
                  begin
                    calc_divconst_magic_unsigned(resultdef.size * 8, tordconstnode(right).value, reciprocal, magic_add, shift);
                    { Add explicit typecast to tcgint type, to avoid range or overflow check }
                    cg.a_load_const_reg(current_asmdata.CurrAsmList, opsize, tcgint(reciprocal), resultreg);
                    { UMULH is only available for the full 64-bit registers }
                    if opsize in [OS_64, OS_S64] then
                      begin
                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_UMULH,resultreg,resultreg,numerator));
                        largerresreg := resultreg;
                      end
                    else
                      begin
                        largerresreg := newreg(getregtype(resultreg), getsupreg(resultreg), R_SUBWHOLE);
                        largernumreg := newreg(getregtype(numerator), getsupreg(numerator), R_SUBWHOLE);
                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MUL,largerresreg,largerresreg,largernumreg));
                        expandword := True; { Try to merge the shift operation with something below }
                      end;

                    { Store divisor for later (and executed at the same time as the multiplication) }
                    if (nodetype=modn) then
                      begin
                        divider:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                        cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,tordconstnode(right).value.svalue,divider);
                      end;

                    if magic_add then
                      begin
                        { We can't append LSR to the ADD below because it would require extending the registers
                          and interfere with the carry bit }
                        if expandword then
                          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_LSR,largerresreg,largerresreg,32));

                        { Add the reciprocal to the high-order word, tracking the carry bit, shift, then
                          insert the carry bit via CSEL and ORR }

                        if opsize in [OS_64,OS_S64] then
                          zeroreg := NR_XZR
                        else
                          zeroreg := NR_WZR;

                        high_bit := QWord(1) shl ((resultdef.size * 8) - shift);

                        tmpreg := cg.getintregister(current_asmdata.CurrAsmList, opsize);
                        cg.a_load_const_reg(current_asmdata.CurrAsmList, opsize, high_bit, tmpreg);

                        { Generate ADDS instruction }
                        hp := taicpu.op_reg_reg_reg(A_ADD,resultreg,resultreg,numerator);
                        hp.oppostfix := PF_S;
                        current_asmdata.CurrAsmList.concat(hp);

                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_cond(A_CSEL,tmpreg,tmpreg,zeroreg, C_CS));

                        shifterop.shiftmode := SM_LSR;
                        shifterop.shiftimm := shift;

                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,resultreg,tmpreg,resultreg,shifterop));
                      end
                    else if expandword then
                      { Include the right-shift by 32 to get the high-order DWord }
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_LSR,largerresreg,largerresreg,shift + 32))
                    else
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_LSR,resultreg,resultreg,shift));
                  end;

              end;

          end
        { no divide-by-zero detection available in hardware, emulate (if it's a
          constant, this will have been detected earlier already) }
        else
          begin
            { load divider in a register }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
            divider:=right.location.register;

            { ARM-64 developer guides recommend checking for division by zero conditions
              AFTER the division, since the check and the division can be done in tandem }
            if is_signed(left.resultdef) then
              op:=A_SDIV
            else
              op:=A_UDIV;

            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,resultreg,numerator,divider));

            current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,divider,0));
            current_asmdata.getjumplabel(hl);
            current_asmdata.CurrAsmList.concat(taicpu.op_cond_sym(A_B,C_NE,hl));
            cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DIVBYZERO',false);
            cg.a_label(current_asmdata.CurrAsmList,hl);
          end;

        genOverflowCheck;

        { in case of modulo, multiply result again by the divider and subtract
          from the numerator }
        if (nodetype=modn) then
          begin
            { If we didn't acquire the original divisor earlier, grab it now }
            if divider = NR_NO then
              begin
                divider:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,tordconstnode(right).value.svalue,divider);
              end;

            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_reg(A_MSUB,resultreg,
              resultreg,divider,numerator));
          end;
    end;


{*****************************************************************************
                               taarch64notnode
*****************************************************************************}

    procedure taarch64notnode.second_boolean;
      begin
        secondpass(left);
        if not handle_locjump then
          begin
            case left.location.loc of
              LOC_FLAGS :
                begin
                  location_copy(location,left.location);
                  inverse_flags(location.resflags);
                end;
              LOC_REGISTER, LOC_CREGISTER,
              LOC_REFERENCE, LOC_CREFERENCE,
              LOC_SUBSETREG, LOC_CSUBSETREG,
              LOC_SUBSETREF, LOC_CSUBSETREF:
                begin
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,
                    left.location.register,0));
                  location_reset(location,LOC_FLAGS,OS_NO);
                  location.resflags:=F_EQ;
               end;
              else
                internalerror(2003042401);
            end;
          end;
      end;


{*****************************************************************************
                                   taarch64unaryminusnode
*****************************************************************************}

    procedure taarch64unaryminusnode.second_float;
      begin
        secondpass(left);
        hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
        location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FNEG,location.register,left.location.register));
        cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
      end;

begin
   cmoddivnode:=taarch64moddivnode;
   cnotnode:=taarch64notnode;
   cunaryminusnode:=taarch64unaryminusnode;
end.
