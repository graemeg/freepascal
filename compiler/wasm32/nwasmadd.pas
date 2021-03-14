{
    Copyright (c) 2019 by Dmitry Boyarintsev based on JVM by Jonas Maebe

    Code generation for add nodes on the WebAssembly

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
unit nwasmadd;

{$i fpcdefs.inc}

interface

    uses
       cgbase,
       node,ncgadd,cpubase, globals, pass_2;

    type

       { twasmaddnode }

       twasmaddnode = class(tcgaddnode)
       protected
          procedure second_generic_compare(unsigned: boolean);

          procedure pass_left_right;override;
          procedure second_addfloat;override;
          procedure second_cmpfloat;override;
          procedure second_cmpboolean;override;
          procedure second_cmp64bit;override;
          procedure second_add64bit; override;
          procedure second_cmpordinal;override;

          // special treatement for short-boolean expressions
          // using IF block, instead of direct labels
          procedure second_addboolean; override;
       public
          function pass_1: tnode;override;
       end;

  implementation

    uses
      systems,
      cutils,verbose,constexp,globtype,compinnr,
      symconst,symtable,symdef,symcpu,
      paramgr,procinfo,pass_1,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,
      hlcgobj,hlcgcpu,cgutils,
      cpupara,
      nbas,ncon,nset,nadd,ncal,ncnv,ninl,nld,nmat,nmem,
      //njvmcon,
      cgobj, symtype, tgobj;

{*****************************************************************************
                               twasmaddnode
*****************************************************************************}

    function twasmaddnode.pass_1: tnode;
      begin
        result:=inherited;
        if (result=nil) and (expectloc in [LOC_JUMP,LOC_FLAGS]) then
          expectloc:=LOC_REGISTER;
      end;

    procedure twasmaddnode.pass_left_right;
      begin
        //if not((nodetype in [orn,andn]) and
        //       is_boolean(left.resultdef)) then
        //  swapleftright;
        inherited pass_left_right;
      end;

    procedure twasmaddnode.second_addfloat;
      var
        op : TAsmOp;
        commutative : boolean;
      begin
        pass_left_right;

        location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getfpuregister(current_asmdata.CurrAsmList,resultdef);

        commutative:=false;
        case nodetype of
          addn :
            begin
              if location.size=OS_F64 then
                op:=a_f64_add
              else
                op:=a_f32_add;
              commutative:=true;
            end;
          muln :
            begin
              if location.size=OS_F64 then
                op:=a_f64_mul
              else
                op:=a_f32_mul;
              commutative:=true;
            end;
          subn :
            begin
              if location.size=OS_F64 then
                op:=a_f64_sub
              else
                op:=a_f32_sub;
            end;
          slashn :
            begin
              if location.size=OS_F64 then
                op:=a_f64_div
              else
                op:=a_f32_div;
            end;
          else
            internalerror(2011010402);
        end;

        { swap the operands to make it easier for the optimizer to optimize
          the operand stack slot reloading (non-commutative operations must
          always be in the correct order though) }
        if (commutative and
            (left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER]) and
            (right.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER])) or
           (not commutative and
            (nf_swapped in flags)) then
          swapleftright;

        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,right.resultdef,right.location);

        current_asmdata.CurrAsmList.concat(taicpu.op_none(op));
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
        { could be optimized in the future by keeping the results on the stack,
          if we add code to swap the operands when necessary (a_swap for
          singles, store/load/load for doubles since there is no swap for
          2-slot elements -- also adjust expectloc in that case! }
        thlcgwasm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
      end;

    procedure twasmaddnode.second_cmpfloat;
      var
        op : TAsmOp;
        commutative : boolean;
        cmpResultType : tdef;
      begin
        cmpResultType := s32inttype;
        pass_left_right;

        commutative:=false;
        case nodetype of
          ltn :
            begin
              if left.location.size=OS_F64 then
                op:=a_f64_lt
              else
                op:=a_f32_lt;
            end;
          lten :
            begin
              if left.location.size=OS_F64 then
                op:=a_f64_le
              else
                op:=a_f32_le;
            end;
          gtn :
            begin
              if left.location.size=OS_F64 then
                op:=a_f64_gt
              else
                op:=a_f32_gt;
            end;
          gten :
            begin
              if left.location.size=OS_F64 then
                op:=a_f64_ge
              else
                op:=a_f32_ge;
            end;
          equaln :
            begin
              if left.location.size=OS_F64 then
                op:=a_f64_eq
              else
                op:=a_f32_eq;
              commutative:=true;
            end;
          unequaln :
            begin
              if left.location.size=OS_F64 then
                op:=a_f64_ne
              else
                op:=a_f32_ne;
              commutative:=true;
            end;

          else
            internalerror(2011010402);
        end;

        { swap the operands to make it easier for the optimizer to optimize
          the operand stack slot reloading (non-commutative operations must
          always be in the correct order though) }
        if (commutative and
            (left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER]) and
            (right.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER])) or
           (not commutative and
            (nf_swapped in flags)) then
          swapleftright;

        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,right.resultdef,right.location);

        current_asmdata.CurrAsmList.concat(taicpu.op_none(op));
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
        { could be optimized in the future by keeping the results on the stack,
          if we add code to swap the operands when necessary (a_swap for
          singles, store/load/load for doubles since there is no swap for
          2-slot elements -- also adjust expectloc in that case! }
          set_result_location_reg;
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;


    procedure twasmaddnode.second_cmpboolean;
      begin
        second_generic_compare(true);
      end;


    procedure twasmaddnode.second_cmp64bit;
      begin
        second_generic_compare(not is_signed(left.resultdef));
      end;


    procedure twasmaddnode.second_add64bit;
      begin
        second_opordinal;
      end;


    procedure twasmaddnode.second_cmpordinal;
      begin
        second_generic_compare(not is_signed(left.resultdef));
      end;

    procedure twasmaddnode.second_addboolean;
      begin
        if (nodetype in [orn,andn]) and
           (not(cs_full_boolean_eval in current_settings.localswitches) or
            (nf_short_bool in flags)) then
        begin
          secondpass(left);
          thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);

          current_asmdata.CurrAsmList.Concat(taicpu.op_functype(a_if,TWasmFuncType.Create([],[wbt_i32])));
          thlcgwasm(hlcg).incblock;
          thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

          case nodetype of
              andn :
                begin
                   // inside of IF (the condition evaluated as true)
                   // for "and" must evaluate the right section
                   secondpass(right);
                   thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,right.resultdef,right.location);

                   current_asmdata.CurrAsmList.Concat( taicpu.op_none(a_else) );
                   thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

                   // inside of ELSE (the condition evaluated as false)
                   // for "and" must end evaluation immediately
                   current_asmdata.CurrAsmList.Concat( taicpu.op_const(a_i32_const, 0) );
                   thlcgwasm(hlcg).incstack(current_asmdata.CurrAsmList,1);
                end;
              orn :
                begin
                   // inside of IF (the condition evaluated as true)
                   // for "or" must end evalaution immediately - satified!
                   current_asmdata.CurrAsmList.Concat( taicpu.op_const(a_i32_const, 1) );
                   thlcgwasm(hlcg).incstack(current_asmdata.CurrAsmList,1);

                   current_asmdata.CurrAsmList.Concat( taicpu.op_none(a_else) );
                   thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
                   // inside of ELSE (the condition evaluated as false)
                   // for "or" must evaluate the right part
                   secondpass(right);
                   // inside of ELSE (the condition evaluated as false)
                   // for "and" must end evaluation immediately
                   thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,right.resultdef,right.location);
                end;
              else
                Internalerror(2019091902);
              end;
          current_asmdata.CurrAsmList.Concat( taicpu.op_none(a_end_if) );
          thlcgwasm(hlcg).decblock;
          set_result_location_reg;
          thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
        end else
          inherited;
      end;

    procedure twasmaddnode.second_generic_compare(unsigned: boolean);
      var
        truelabel,
        falselabel: tasmlabel;
        cmpop: TOpCmp;
      begin
        truelabel:=nil;
        falselabel:=nil;
        pass_left_right;
        { swap the operands to make it easier for the optimizer to optimize
          the operand stack slot reloading in case both are in a register }
        if (left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) and
           (right.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
          swapleftright;
        cmpop:=cmpnode2topcmp(unsigned);
        if (nf_swapped in flags) then
          cmpop:=swap_opcmp(cmpop);

        if left.location.loc in [LOC_REGISTER,LOC_CREGISTER] then
          thlcgwasm(hlcg).a_cmp_loc_reg_stack(current_asmdata.CurrAsmList,left.resultdef,cmpop,right.location,left.location.register)
        else case right.location.loc of
          LOC_REGISTER,LOC_CREGISTER:
            thlcgwasm(hlcg).a_cmp_reg_loc_stack(current_asmdata.CurrAsmList,left.resultdef,cmpop,right.location.register,left.location);
          LOC_REFERENCE,LOC_CREFERENCE:
            thlcgwasm(hlcg).a_cmp_ref_loc_stack(current_asmdata.CurrAsmList,left.resultdef,cmpop,right.location.reference,left.location);
          LOC_CONSTANT:
            thlcgwasm(hlcg).a_cmp_const_loc_stack(current_asmdata.CurrAsmList,left.resultdef,cmpop,right.location.value,left.location);
          else
            internalerror(2011010413);
        end;
        set_result_location_reg;
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;

begin
  caddnode:=twasmaddnode;
end.
