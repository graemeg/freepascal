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
       protected
         function pass_1 : tnode;override;
         procedure second_cmpordinal;override;
         procedure second_cmpsmallset;override;
         procedure second_cmp64bit;override;
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

begin
  caddnode:=tcpuaddnode;
end.

