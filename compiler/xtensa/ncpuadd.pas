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
       cgbase,node,ncgadd,cpubase;

    type
       TCPUAddNode = class(tcgaddnode)
       private
         procedure pass_left_and_right;
         procedure cmp64_le(left_reg, right_reg: TRegister64; unsigned: boolean);
         procedure cmp64_lt(left_reg, right_reg: TRegister64; unsigned: boolean);
       protected
         function pass_1 : tnode;override;
         function first_addfloat: tnode;override;
         function use_generic_mul32to64: boolean;override;
         function use_generic_mul64bit: boolean;override;
         procedure second_addordinal;override;
         procedure second_cmpordinal;override;
         procedure second_cmpsmallset;override;
         procedure second_cmp64bit;override;
         procedure second_add64bit;override;
         procedure second_cmpfloat;override;
         procedure second_addfloat;override;
         procedure second_cmp;
         function use_fma: boolean;override;
       end;

  implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,htypechk,
      cgutils,cgcpu,
      cpuinfo,pass_1,pass_2,procinfo,
      cpupara,
      ncon,nset,nadd,
      ncgutil,tgobj,rgobj,rgcpu,cgobj,cg64f32,
      hlcgobj;

{*****************************************************************************
                               TCPUAddNode
*****************************************************************************}

    function TCPUAddNode.use_fma : boolean;
      begin
        Result:=is_single(left.resultdef) and is_single(right.resultdef) and
          (FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]);
      end;


    procedure TCPUAddNode.second_addordinal;
      var
        ophigh: tasmop;
      begin
        { this is only true, if the CPU supports 32x32 -> 64 bit MUL, see the relevant method }
        if (nodetype=muln) and is_64bit(resultdef) then
          begin
            if not(is_signed(left.resultdef)) or
               not(is_signed(right.resultdef)) then
              ophigh:=A_MULUH
            else
              ophigh:=A_MULSH;

            pass_left_right;

            if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            if not(right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

            { initialize the result }
            location_reset(location,LOC_REGISTER,def_cgsize(resultdef));

            location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);

            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULL,location.register64.reglo,left.location.register,right.location.register));
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(ophigh,location.register64.reghi,left.location.register,right.location.register));
          end
        else
          Inherited;
      end;


    procedure TCPUAddNode.second_cmpsmallset;
      var
        tmpreg : tregister;
        truelab, falselab: TAsmLabel;
      begin
        pass_left_right;

        if (not(nf_swapped in flags) and
            (nodetype = lten)) or
           ((nf_swapped in flags) and
            (nodetype = gten)) then
          swapleftright;

        current_asmdata.getjumplabel(truelab);
        current_asmdata.getjumplabel(falselab);

        location_reset_jump(location,truelab,falselab);
        force_reg_left_right(false,false);

        case nodetype of
          equaln:
            begin
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_EQ,left.location.register,right.location.register,location.truelabel);
              cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
            end;
          unequaln:
            begin
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left.location.register,right.location.register,location.truelabel);
              cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
            end;
          lten,
          gten:
            begin
              tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
              cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,left.location.register,right.location.register,tmpreg);
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_EQ,tmpreg,right.location.register,location.truelabel);
              cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
            end;
          else
            internalerror(2020082401);
        end;
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

        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(OS_INT),true);
                                            
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
            internalerror(2020030803);
          end;

        if (right.nodetype=ordconstn) and not(nf_swapped in flags) then
          cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,right.location.value,left.location.register,location.truelabel)
        else
          begin
            if not(right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cgsize_orddef(OS_INT),true);

            if nf_swapped in flags then
               cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,left.location.register,right.location.register,location.truelabel)
             else
               cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cond,right.location.register,left.location.register,location.truelabel);
          end;
        cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
      end;


    const
      cmpops: array[boolean] of TOpCmp = (OC_LT,OC_B);

    procedure TCPUAddNode.cmp64_lt(left_reg, right_reg: TRegister64;unsigned: boolean);
      begin
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cmpops[unsigned],right_reg.reghi,left_reg.reghi,location.truelabel);
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.falselabel);
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_B,right_reg.reglo,left_reg.reglo,location.truelabel);
        cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
      end;


    procedure TCPUAddNode.cmp64_le(left_reg, right_reg: TRegister64;unsigned: boolean);
      begin
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cmpops[unsigned],left_reg.reghi,right_reg.reghi,location.falselabel);
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.truelabel);
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_B,left_reg.reglo,right_reg.reglo,location.falselabel);
        cg.a_jmp_always(current_asmdata.CurrAsmList,location.truelabel);
      end;


    procedure TCPUAddNode.second_cmp64bit;
      var
        truelabel,
        falselabel: tasmlabel;
        unsigned: boolean;
        left_reg,right_reg: TRegister64;
      begin
        current_asmdata.getjumplabel(truelabel);
        current_asmdata.getjumplabel(falselabel);
        location_reset_jump(location,truelabel,falselabel);

        pass_left_right;
        force_reg_left_right(true,true);

        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        left_reg:=left.location.register64;
        { force_reg_left_right might leave right as LOC_CONSTANT, however, we cannot take advantage of this yet }
        if right.location.loc=LOC_CONSTANT then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);
        right_reg:=right.location.register64;

        case NodeType of
          equaln:
            begin
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.falselabel);
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reglo,right_reg.reglo,location.falselabel);
              cg.a_jmp_always(current_asmdata.CurrAsmList,location.truelabel);
            end;
          unequaln:
            begin
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.truelabel);
              cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reglo,right_reg.reglo,location.truelabel);
              cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
            end;
        else
          if nf_swapped in flags then
            case NodeType of
              ltn:
                cmp64_lt(right_reg, left_reg,unsigned);
              lten:
                cmp64_le(right_reg, left_reg,unsigned);
              gtn:
                cmp64_lt(left_reg, right_reg,unsigned);
              gten:
                cmp64_le(left_reg, right_reg,unsigned);
              else
                internalerror(2020082202);
            end
          else
            case NodeType of
              ltn:
                cmp64_lt(left_reg, right_reg,unsigned);
              lten:
                cmp64_le(left_reg, right_reg,unsigned);
              gtn:
                cmp64_lt(right_reg, left_reg,unsigned);
              gten:
                cmp64_le(right_reg, left_reg,unsigned);
              else
                internalerror(2020082203);
            end;
        end;
      end;


    function TCPUAddNode.pass_1 : tnode;
      begin
        result:=inherited pass_1;
        if not(assigned(result)) and (nodetype in [equaln,unequaln,ltn,lten,gtn,gten]) and
          not((FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) and
            is_single(left.resultdef) and (nodetype<>slashn)) then
          expectloc:=LOC_JUMP;
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


    function TCPUAddNode.use_generic_mul32to64: boolean;
      begin
        result:=not(CPUXTENSA_HAS_MUL32HIGH in cpu_capabilities[current_settings.cputype]) or needoverflowcheck;
      end;


    function TCPUAddNode.use_generic_mul64bit: boolean;
      begin
        result:=needoverflowcheck or
          (cs_opt_size in current_settings.optimizerswitches) or
          not(CPUXTENSA_HAS_MUL32HIGH in cpu_capabilities[current_settings.cputype]);
      end;


    procedure TCPUAddNode.second_addfloat;
      var
        op    : TAsmOp;
        cmpop,
        singleprec , inv: boolean;
        ai : taicpu;
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
            op:=A_ADD;
          muln :
            op:=A_MUL;
          subn :
            op:=A_SUB;
          unequaln:
            begin
              op:=A_OEQ;
              cmpop:=true;
              inv:=true;
            end;
          equaln:
            begin
              op:=A_OEQ;
              cmpop:=true;
            end;
          ltn:
            begin
              op:=A_OLT;
              cmpop:=true;
            end;
          lten:
            begin
              op:=A_OLE;
              cmpop:=true;
            end;
          gtn:
            begin
              op:=A_OLT;
              swapleftright;
              cmpop:=true;
            end;
          gten:
            begin
              op:=A_OLE;
              swapleftright;
              cmpop:=true;
            end;
          else
            internalerror(2020032601);
        end;

        { initialize de result }
        if cmpop then
          begin
            if CPUXTENSA_HAS_BOOLEAN_OPTION in cpu_capabilities[current_settings.cputype] then
              begin
                location_reset(location,LOC_FLAGS,OS_NO);
                location.resflags.register:=NR_B0;
                location.resflags.flag:=F_NZ;
              end
            else
              Internalerror(2020070402);
          end
        else
         begin
           location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
           location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
         end;

        { emit the actual operation }
        if cmpop then
          begin
            cg.getcpuregister(current_asmdata.CurrAsmList,location.resflags.register);
            ai:=taicpu.op_reg_reg_reg(op,location.resflags.register,left.location.register,right.location.register);
            ai.oppostfix:=PF_S;
            current_asmdata.CurrAsmList.concat(ai);
            cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);

            if inv then
              location.resflags.flag:=F_Z;
          end
        else
          begin
            ai:=taicpu.op_reg_reg_reg(op,location.register,left.location.register,right.location.register);
            ai.oppostfix := PF_S;
            current_asmdata.CurrAsmList.concat(ai);
            cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
          end;
      end;


    procedure TCPUAddNode.second_cmpfloat;
      begin
        second_addfloat;
      end;


    procedure TCPUAddNode.second_add64bit;
      var
        unsigned: Boolean;
        tmpreg: tregister;
      begin
        if nodetype=muln then
          begin
            pass_left_right;
            unsigned:=((left.resultdef.typ=orddef) and
                       (torddef(left.resultdef).ordtype=u64bit)) or
                      ((right.resultdef.typ=orddef) and
                       (torddef(right.resultdef).ordtype=u64bit));
            force_reg_left_right(true,true);

            { force_reg_left_right might leave right as LOC_CONSTANT, however, we cannot take advantage of this yet }
            if right.location.loc=LOC_CONSTANT then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

            location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
            location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULL,location.register64.reglo,left.location.register64.reglo,right.location.register64.reglo));
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULUH,location.register64.reghi,left.location.register64.reglo,right.location.register64.reglo));
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULL,tmpreg,left.location.register64.reglo,right.location.register64.reghi));
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ADD,location.register64.reghi,location.register64.reghi,tmpreg));
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULL,tmpreg,left.location.register64.reghi,right.location.register64.reglo));
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ADD,location.register64.reghi,location.register64.reghi,tmpreg));
          end
        else
          Inherited;
      end;


begin
  caddnode:=tcpuaddnode;
end.

