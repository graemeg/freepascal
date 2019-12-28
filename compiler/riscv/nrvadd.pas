{
    Copyright (c) 2000-2006 by Florian Klaempfl and Jonas Maebe

    Code generation for add nodes on the Risc-V (32 and 64 bit generic)

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

unit nrvadd;


{$i fpcdefs.inc}

  interface

    uses
       node,nadd,ncgadd,cpubase;

    type
      trvaddnode = class(tcgaddnode)
        function pass_1: tnode; override;
      protected                            
        procedure Cmp(signed: boolean);

        function use_mul_helper: boolean; override;

        procedure second_cmpsmallset;override;
        procedure second_cmpordinal;override;
        procedure second_cmp64bit; override;

        procedure second_addordinal; override;

        procedure pass_left_and_right;  

        function use_fma: boolean; override;

        procedure second_addfloat;override;
        procedure second_cmpfloat;override;
      end;


implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,htypechk,
      cgbase,cpuinfo,pass_1,pass_2,
      cpupara,cgcpu,cgutils,procinfo,
      ncon,nset,
      ncgutil,tgobj,rgobj,rgcpu,cgobj,hlcgobj;

{$undef AVOID_OVERFLOW}
{$ifopt Q+}
  {$define AVOID_OVERFLOW}
  const
     low_value = {$ifdef CPU64BITALU} low(int64) {$else} low(longint) {$endif};
{$endif}

    procedure trvaddnode.Cmp(signed: boolean);
      var
        flabel,tlabel: tasmlabel;
        op, opi: TAsmOp;
      begin
        pass_left_right;

        force_reg_left_right(true,true);

        if nf_swapped in flags then
          swapleftright;

        location_reset(location,LOC_REGISTER,OS_INT);
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);

        if signed then op:=A_SLT else op:=A_SLTU;
        if signed then opi:=A_SLTI else opi:=A_SLTIU;

        case nodetype of
          equaln:
            begin
              if not (left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

              if (right.location.loc=LOC_CONSTANT) and
                 { right.location.value might be $8000000000000000, 
                   and its minus value generates an overflow here }
                 {$ifdef AVOID_OVERFLOW} ((right.location.value = low_value) or {$endif}
                 (not is_imm12(-right.location.value)) {$ifdef AVOID_OVERFLOW}){$endif} then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

              if right.location.loc=LOC_CONSTANT then
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(A_ADDI,location.register,left.location.register,-right.location.value))
              else
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(A_SUB,location.register,left.location.register,right.location.register));
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(A_SLTIU,location.register,location.register,1));
            end;
          unequaln:
            begin
              if not (left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

              if (right.location.loc=LOC_CONSTANT) and
                 { right.location.value might be $8000000000000000, 
                   and its minus value generates an overflow here }
                 {$ifdef AVOID_OVERFLOW} ((right.location.value = low_value) or {$endif}
                 (not is_imm12(-right.location.value)) {$ifdef AVOID_OVERFLOW}){$endif} then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

              if right.location.loc=LOC_CONSTANT then
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(A_ADDI,location.register,left.location.register,-right.location.value))
              else
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(A_SUB,location.register,left.location.register,right.location.register));
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(A_SLTU,location.register,NR_X0,location.register));
            end;
          ltn:
            begin
              if not (left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

              if (right.location.loc=LOC_CONSTANT) and
                 (not is_imm12(right.location.value)) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

              if right.location.loc=LOC_CONSTANT then
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(opi,location.register,left.location.register,right.location.value))
              else
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register));
            end;
          gtn:
            begin
              if not (right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

              if (left.location.loc=LOC_CONSTANT) and
                 (not is_imm12(left.location.value)) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

              if left.location.loc=LOC_CONSTANT then
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(opi,location.register,right.location.register,left.location.value))
              else
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(op,location.register,right.location.register,left.location.register));
            end;

          lten:
            begin
              if not (right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

              if (left.location.loc=LOC_CONSTANT) and
                 (not is_imm12(left.location.value)) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

              if left.location.loc=LOC_CONSTANT then
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(opi,location.register,right.location.register,left.location.value))
              else
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(op,location.register,right.location.register,left.location.register));
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(A_SLTIU,location.register,location.register,1));
            end;
          gten:
            begin
              if not (left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

              if (right.location.loc=LOC_CONSTANT) and
                 (not is_imm12(right.location.value)) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

              if right.location.loc=LOC_CONSTANT then
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(opi,location.register,left.location.register,right.location.value))
              else
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register));
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(A_SLTIU,location.register,location.register,1));
            end;
        else
          Internalerror(2016061101);
        end;
      end;


    function trvaddnode.use_mul_helper: boolean;
      begin
        if not (CPURV_HAS_MUL in cpu_capabilities[current_settings.cputype]) and
           (nodetype=muln) and
           not(torddef(resultdef).ordtype in [u8bit,s8bit]) then
          result:=true
        else
          Result:=inherited use_mul_helper;
      end;


    procedure trvaddnode.second_cmpsmallset;
      begin
        Cmp(true);
      end;


    procedure trvaddnode.second_cmpordinal;
      var
        unsigned: Boolean;
      begin
        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        Cmp(not unsigned);
      end;


    procedure trvaddnode.second_cmp64bit;
      var
        unsigned: Boolean;
      begin
        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        Cmp(not unsigned);
      end;                  


    procedure trvaddnode.second_addordinal;
      var
        unsigned: boolean;
      begin
        { 32x32->64 multiplication }
        if (nodetype=muln) and
           is_32bit(left.resultdef) and
           is_32bit(right.resultdef) and
           is_64bit(resultdef) then
          begin
            unsigned:=not(is_signed(left.resultdef)) or
                      not(is_signed(right.resultdef));
            pass_left_right;
            force_reg_left_right(true,true);
            { force_reg_left_right can leave right as a LOC_CONSTANT (we can't
              say "a constant register is okay, but an ordinal constant isn't) }
            if right.location.loc=LOC_CONSTANT then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
            location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,def_cgsize(resultdef));
            current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(A_MUL,location.register,left.location.register,right.location.register));
          end
        else
          inherited second_addordinal;
      end;


    function trvaddnode.pass_1: tnode;
      begin
        if (nodetype=muln) and
           (left.resultdef.typ=orddef) and (left.resultdef.typ=orddef) and
           (CPURV_HAS_MUL in cpu_capabilities[current_settings.cputype])
{$ifdef cpu32bitalu}
           and (not (is_64bit(left.resultdef) or
                     is_64bit(right.resultdef)))
{$endif cpu32bitalu}
           then
          begin
            result:=nil;

            firstpass(left);
            firstpass(right);

            expectloc:=LOC_REGISTER;
          end
        else if (nodetype=muln) and
           (not (CPURV_HAS_MUL in cpu_capabilities[current_settings.cputype])) and
           (is_64bit(left.resultdef) or
            is_64bit(right.resultdef)) then
          begin
            result:=first_add64bitint;
          end
        else
          Result:=inherited pass_1;

        if expectloc=LOC_FLAGS then
          expectloc:=LOC_REGISTER;
        if (expectloc=LOC_JUMP) and (nodetype in [equaln, unequaln, ltn, lten, gtn, gten]) then
          expectloc:=LOC_REGISTER;
      end;


    procedure trvaddnode.pass_left_and_right;
      begin
        { calculate the operator which is more difficult }
        firstcomplex(self);

        { in case of constant put it to the left }
        if (left.nodetype=ordconstn) then
         swapleftright;

        secondpass(left);
        secondpass(right);
      end;


    function trvaddnode.use_fma: boolean;
      begin
        Result:=current_settings.fputype in [fpu_fd];
      end;


    procedure trvaddnode.second_addfloat;
      var
        op    : TAsmOp;
        cmpop,
        singleprec , inv: boolean;
      begin
        pass_left_and_right;
        if (nf_swapped in flags) then
          swapleftright;

        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

        cmpop:=false;
        singleprec:=tfloatdef(left.resultdef).floattype=s32real;
        inv:=false;
        case nodetype of
          addn :
            if singleprec then
              op:=A_FADD_S
            else
              op:=A_FADD_D;
          muln :
            if singleprec then
              op:=A_FMUL_S
            else
            op:=A_FMUL_D;
          subn :
            if singleprec then
              op:=A_FSUB_S
            else
              op:=A_FSUB_D;
          slashn :
            if singleprec then
              op:=A_FDIV_S
            else
             op:=A_FDIV_D;
          equaln:
            begin
              if singleprec then
                op:=A_FEQ_S
              else
                op:=A_FEQ_D;
              cmpop:=true;
            end;
          unequaln:
            begin
              if singleprec then
                op:=A_FEQ_S
              else
                op:=A_FEQ_D;
              inv:=true;
              cmpop:=true;
            end;
          ltn:
            begin
              if singleprec then
                op:=A_FLT_S
              else
                op:=A_FLT_D;
              cmpop:=true;
            end;
          lten:
            begin
              if singleprec then
                op:=A_FLE_S
              else
                op:=A_FLE_D;
              cmpop:=true;
            end;
          gtn:
            begin
              if singleprec then
                op:=A_FLT_S
              else
                op:=A_FLT_D;
              swapleftright;
              cmpop:=true;
            end;
          gten:
            begin
              if singleprec then
                op:=A_FLE_S
              else
                op:=A_FLE_D;
              swapleftright;
              cmpop:=true;
            end;
          else
            internalerror(200403182);
        end;

        // put both operands in a register
        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,true);
        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);

        // initialize de result
        if not cmpop then
          begin
            location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
            location.register := cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
          end
        else
         begin
           location_reset(location,LOC_REGISTER,OS_8);
           location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
         end;

        // emit the actual operation
        if not cmpop then
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register));
            cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
          end
        else
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register));
            cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);

            if inv then
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_XORI,location.register,location.register,1));
          end;
      end;

    procedure trvaddnode.second_cmpfloat;
      begin
        second_addfloat;
      end;

end.
