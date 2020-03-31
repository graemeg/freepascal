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
         numerator,
         divider,
         resultreg  : tregister;
         hl : tasmlabel;
         overflowloc: tlocation;
         power: longint;

       procedure genOrdConstNodeDiv;
         var
           helper1, helper2: TRegister;
           so: tshifterop;
         begin
           if tordconstnode(right).value=0 then
             internalerror(2020021601)
           else if tordconstnode(right).value=1 then
             cg.a_load_reg_reg(current_asmdata.CurrAsmList, OS_INT, OS_INT, numerator, resultreg)
           else if (tordconstnode(right).value = int64(-1)) then
             begin
               // note: only in the signed case possible..., may overflow
               if cs_check_overflow in current_settings.localswitches then
                 cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);

               current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_NEG,
                 resultreg,numerator),toppostfix(ord(cs_check_overflow in current_settings.localswitches)*ord(PF_S))));
             end
           else if ispowerof2(tordconstnode(right).value,power) then
             begin
               if (is_signed(right.resultdef)) then
                 begin
                    helper2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                    if power = 1 then
                      helper1:=numerator
                    else
                      begin
                        helper1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,OS_INT,63,numerator,helper1);
                      end;
                    shifterop_reset(so);
                    so.shiftmode:=SM_LSR;
                    so.shiftimm:=64-power;
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,helper2,numerator,helper1,so));
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,OS_INT,power,helper2,resultreg);
                  end
               else
                 cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_INT,power,numerator,resultreg)
             end
           else
             { Everything else is handled in the generic code }
             cg.g_div_const_reg_reg(current_asmdata.CurrAsmList,def_cgsize(resultdef),
               tordconstnode(right).value.svalue,numerator,resultreg);
         end;

      begin
       secondpass(left);
       secondpass(right);
       { avoid warning }
       divider:=NR_NO;

       { set result location }
       location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
       location.register:=cg.getintregister(current_asmdata.CurrAsmList,def_cgsize(resultdef));
       resultreg:=location.register;

       { put numerator in register }
       hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
       numerator:=left.location.register;

       if (right.nodetype=ordconstn) and
          ((tordconstnode(right).value=1) or
           (tordconstnode(right).value=int64(-1)) or
           (tordconstnode(right).value=0) or
           ispowerof2(tordconstnode(right).value,power)) then
         begin
           genOrdConstNodeDiv;
           if nodetype=modn then
             begin
               divider:=cg.getintregister(current_asmdata.CurrAsmList,def_cgsize(resultdef));
               cg.a_load_const_reg(current_asmdata.CurrAsmList,def_cgsize(resultdef),int64(tordconstnode(right).value),divider);
             end;
         end
       else
         begin
           { load divider in a register }
           hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
           divider:=right.location.register;

           { start division }
           if is_signed(left.resultdef) then
             op:=A_SDIV
           else
             op:=A_UDIV;
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,location.register,numerator,divider));
         end;

       { no divide-by-zero detection available in hardware, emulate (if it's a
         constant, this will have been detected earlier already) }
       if (right.nodetype<>ordconstn) then
         begin
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,
             right.location.register,0));

           current_asmdata.getjumplabel(hl);
           current_asmdata.CurrAsmList.concat(taicpu.op_cond_sym(A_B,C_NE,hl));
           cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DIVBYZERO',false);
           cg.a_label(current_asmdata.CurrAsmList,hl);
         end;

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
           hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_XOR,left.resultdef,low(int64),left.location.register,tmpreg);
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_EON,
             tmpreg,left.location.register,tmpreg));
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,tmpreg,0));
           { now the zero/equal flag is set in case we divided low(int64) by
             (-1) }
           location_reset(overflowloc,LOC_FLAGS,OS_NO);
           overflowloc.resflags:=F_EQ;
           cg.g_overflowcheck_loc(current_asmdata.CurrAsmList,location,resultdef,overflowloc);
         end;

       { in case of modulo, multiply result again by the divider and subtract
         from the numerator }
       if nodetype=modn then
         current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_reg(A_MSUB,resultreg,
           resultreg,divider,numerator));
    end;


{*****************************************************************************
                               taarch64notnode
*****************************************************************************}

    procedure taarch64notnode.second_boolean;
      begin
        if not handle_locjump then
          begin
            secondpass(left);
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
