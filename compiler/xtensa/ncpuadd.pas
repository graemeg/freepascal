{
    Copyright (c) 2008 by Florian Klaempfl

    Code generation for add nodes on the Xtensa

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
unit ncpuadd;

{$i fpcdefs.inc}

interface

    uses
       node,ncgadd,cpubase;

    type
       TCPUAddNode = class(tcgaddnode)
       private
         procedure pass_left_and_right;
       protected
         function pass_1 : tnode;override;
         function first_addfloat: tnode;override;
         procedure second_cmpordinal;override;
         procedure second_cmpsmallset;override;
         procedure second_cmp64bit;override;
         procedure second_cmpfloat;override;
         procedure second_addfloat;override;
         procedure second_cmp;
       end;

  implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,htypechk,
      cgbase,cgutils,cgcpu,
      cpuinfo,pass_1,pass_2,procinfo,
      cpupara,
      ncon,nset,nadd,
      ncgutil,tgobj,rgobj,rgcpu,cgobj,cg64f32,
      hlcgobj;

{*****************************************************************************
                               TCPUAddNode
*****************************************************************************}

   procedure TCPUAddNode.second_cmpsmallset;
      var
        tmpreg : tregister;
        cond: TOpCmp;
        instr: taicpu;
        truelab, falselab: TAsmLabel;
      begin
        pass_left_right;
        current_asmdata.getjumplabel(truelab);
        current_asmdata.getjumplabel(falselab);

        location_reset_jump(location,truelab,falselab);
        force_reg_left_right(false,false);

        case nodetype of
          equaln:   cond:=OC_EQ;
          unequaln: cond:=OC_NE;
          ltn:      cond:=OC_LT;
          lten:     cond:=OC_LT;
          gtn:      cond:=OC_GT;
          gten:     cond:=OC_GTE;
        else
          internalerror(2020030801);
        end;

        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,left.location.register,right.location.register,location.truelabel);
        current_asmdata.CurrAsmList.concat(taicpu.op_sym(A_J,location.falselabel));
      end;


    procedure TCPUAddNode.second_cmp;
      var
        cond: TOpCmp;
        instr: taicpu;
        truelab, falselab: TAsmLabel;
      begin
        pass_left_right;

        current_asmdata.getjumplabel(truelab);
        current_asmdata.getjumplabel(falselab);

        location_reset_jump(location,truelab,falselab);

        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                                            
        if is_signed(left.resultdef) then
          case nodetype of
            equaln:   cond:=OC_EQ;
            unequaln: cond:=OC_NE;
            ltn:      cond:=OC_LT;
            lten:     cond:=OC_LTE;
            gtn:      cond:=OC_GT;
            gten:     cond:=OC_GTE;
          else
            internalerror(2020030801);
          end
        else
          case nodetype of
            equaln:   cond:=OC_EQ;
            unequaln: cond:=OC_NE;
            ltn:      cond:=OC_B;
            lten:     cond:=OC_BE;
            gtn:      cond:=OC_A;
            gten:     cond:=OC_AE;
          else
            internalerror(2020030801);
          end;

        if (right.nodetype=ordconstn) and not(nf_swapped in flags) then
          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,right.location.value,left.location.register,location.truelabel)
        else
          begin
            force_reg_left_right(false,false);
            if nf_swapped in flags then
               cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,left.location.register,right.location.register,location.truelabel)
             else
               cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,right.location.register,left.location.register,location.truelabel);
          end;
        current_asmdata.CurrAsmList.concat(taicpu.op_sym(A_J,location.falselabel));
      end;


    procedure TCPUAddNode.second_cmp64bit;
      begin
        second_cmp;
      end;


    function TCPUAddNode.pass_1 : tnode;
      begin
        result:=inherited pass_1;
{$ifdef dummy}
        if not(assigned(result)) then
          begin
            unsigned:=not(is_signed(left.resultdef)) or
              not(is_signed(right.resultdef));

            if is_64bit(left.resultdef) and
              ((nodetype in [equaln,unequaln]) or
               (unsigned and (nodetype in [ltn,lten,gtn,gten]))
              ) then
              expectloc:=LOC_FLAGS;
          end;
        { handling boolean expressions }
        if not(assigned(result)) and
           (
             not(is_boolean(left.resultdef)) or
             not(is_boolean(right.resultdef)) or
             is_dynamic_array(left.resultdef)
           ) then
          expectloc:=LOC_FLAGS;
{$endif dummy}
      end;


    procedure TCPUAddNode.second_cmpordinal;
      begin
        second_cmp;
      end;


    procedure TCPUAddNode.pass_left_and_right;
      begin
        { calculate the operator which is more difficult }
        firstcomplex(self);

        { in case of constant put it to the left }
        if (left.nodetype=ordconstn) then
         swapleftright;

        secondpass(left);
        secondpass(right);
      end;


    function TCPUAddNode.first_addfloat: tnode;
      begin
        result := nil;

        if (FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) and
          (tfloatdef(left.resultdef).floattype=s32real) and (nodetype<>slashn) then
          begin
            if nodetype in [equaln,unequaln,lten,ltn,gten,gtn] then
              expectloc:=LOC_FLAGS
            else
              expectloc:=LOC_FPUREGISTER;
          end
        else
          result:=first_addfloat_soft;
      end;


    procedure TCPUAddNode.second_addfloat;
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
        inv:=false;
        case nodetype of
          addn :
            op:=A_ADD_S;
          muln :
            op:=A_MUL_S;
          subn :
            op:=A_SUB_S;
          unequaln,
          equaln:
            begin
              op:=A_OEQ_S;
              cmpop:=true;
            end;
          ltn:
            begin
              op:=A_OLT_S;
              cmpop:=true;
            end;
          lten:
            begin
              op:=A_OLE_S;
              cmpop:=true;
            end;
          gtn:
            begin
              op:=A_OLT_S;
              swapleftright;
              cmpop:=true;
            end;
          gten:
            begin
              op:=A_OLE_S;
              swapleftright;
              cmpop:=true;
            end;
          else
            internalerror(2020032601);
        end;

        { initialize de result }
        if cmpop then
          begin
           location_reset(location,LOC_FLAGS,OS_NO);
           location.resflags.register:=NR_B0;
           location.resflags.flag:=F_NZ;
          end
        else
         begin
           location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
           location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
         end;

        { emit the actual operation }
        if cmpop then
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register));
            cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);

            if inv then
              location.resflags.flag:=F_Z;
          end
        else
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register));
            cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
          end;
      end;


    procedure TCPUAddNode.second_cmpfloat;
      begin
        second_addfloat;
      end;

begin
  caddnode:=tcpuaddnode;
end.

