{
    Copyright (c) 2000-2002 by the FPC development team

    Code generation for add nodes (generic version)

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
unit ncgadd;

{$i fpcdefs.inc}

interface

    uses
       node,nadd,cpubase,cgbase;

    type
       tcgaddnode = class(taddnode)
{          function pass_1: tnode; override;}
          procedure pass_generate_code;override;
         protected
          { call secondpass for both left and right }
          procedure pass_left_right; virtual;
          { set the register of the result location }
          procedure set_result_location_reg;
          { load left and right nodes into registers }
          procedure force_reg_left_right(allow_swap,allow_constant:boolean); virtual;

          function cmpnode2topcmp(unsigned: boolean): TOpCmp;

          procedure second_opfloat;
          procedure second_opboolean;
          procedure second_opsmallset;
          procedure second_op64bit;
          procedure second_opordinal;

          procedure second_addstring;virtual;
          procedure second_addfloat;virtual;abstract;
          procedure second_addboolean;virtual;
          procedure second_addsmallset;virtual;
          procedure second_addsmallsetelement;virtual;
{$ifdef x86}
{$ifdef SUPPORT_MMX}
          procedure second_opmmx;virtual;abstract;
{$endif SUPPORT_MMX}
{$endif x86}
          procedure second_opvector;virtual;abstract;
          procedure second_add64bit;virtual;
          procedure second_addordinal;virtual;
          procedure second_cmpfloat;virtual;abstract;
          procedure second_cmpboolean;virtual;
          procedure second_cmpsmallset;virtual;abstract;
          procedure second_cmp64bit;virtual;abstract;
          procedure second_cmpordinal;virtual;abstract;
       end;

  implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,
      aasmbase,aasmtai,aasmdata,defutil,
      procinfo,pass_2,tgobj,
      nutils,ncon,nset,ncgutil,cgobj,cgutils,
      hlcgobj
      ;


{*****************************************************************************
                                  Helpers
*****************************************************************************}

    procedure tcgaddnode.pass_left_right;
{$if defined(x86) and not defined(llvm)}
      var
        tmpreg     : tregister;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pushedfpu  : boolean;
{$endif x86 and not llvm}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef x86}
        pushedfpu,
{$endif x86}
        isjump     : boolean;
        otl,ofl    : tasmlabel;
>>>>>>> graemeg/cpstrnew
      begin
        { calculate the operator which is more difficult }
        firstcomplex(self);

        { in case of constant put it to the left }
        if (left.nodetype=ordconstn) then
          swapleftright;

        secondpass(left);
        if left.location.loc in [LOC_FLAGS,LOC_JUMP] then
<<<<<<< HEAD
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
{$if defined(x86) and not defined(llvm)}
=======
          location_force_reg(current_asmdata.CurrAsmList,left.location,def_cgsize(resultdef),false);
        if isjump then
          begin
            current_procinfo.CurrTrueLabel:=otl;
            current_procinfo.CurrFalseLabel:=ofl;
          end;

{$ifdef x86}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        { are too few registers free? }
        pushedfpu:=false;
        if (left.location.loc=LOC_FPUREGISTER) and
           (node_resources_fpu(right)>=maxfpuregs) then
          begin
            hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
            pushedfpu:=true;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif x86 and not llvm}
=======
{$endif x86}
>>>>>>> graemeg/cpstrnew
=======
{$endif x86}
>>>>>>> graemeg/cpstrnew
=======
{$endif x86}
>>>>>>> graemeg/cpstrnew
=======
{$endif x86}
>>>>>>> origin/cpstrnew

        secondpass(right);
        if right.location.loc in [LOC_FLAGS,LOC_JUMP] then
<<<<<<< HEAD
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,resultdef,false);
{$if defined(x86) and not defined(llvm)}
=======
          location_force_reg(current_asmdata.CurrAsmList,right.location,def_cgsize(resultdef),false);
        if isjump then
          begin
            current_procinfo.CurrTrueLabel:=otl;
            current_procinfo.CurrFalseLabel:=ofl;
          end;
{$ifdef x86}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if pushedfpu then
          begin
<<<<<<< HEAD
<<<<<<< HEAD
            if use_vectorfpu(left.resultdef) then
              begin
                tmpreg := cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
                hlcg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,left.resultdef,left.resultdef,left.location,tmpreg,mms_movescalar);
                location_freetemp(current_asmdata.CurrAsmList,left.location);
                location_reset(left.location,LOC_MMREGISTER,left.location.size);
                left.location.register:=tmpreg;
              end
            else
              begin
                tmpreg := cg.getfpuregister(current_asmdata.CurrAsmList,left.location.size);
                cg.a_loadfpu_loc_reg(current_asmdata.CurrAsmList,left.location.size,left.location,tmpreg);
                location_freetemp(current_asmdata.CurrAsmList,left.location);
                location_reset(left.location,LOC_FPUREGISTER,left.location.size);
                left.location.register := tmpreg;
                { left operand is now on top of the stack, instead of the right one! }
                if (right.location.loc=LOC_FPUREGISTER) then
                  toggleflag(nf_swapped);
=======
{$ifdef x86}
            if use_sse(left.resultdef) then
              begin
                tmpreg := cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
                cg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,left.location.size,left.location,tmpreg,mms_movescalar);
                location_reset(left.location,LOC_MMREGISTER,left.location.size);
                left.location.register := tmpreg;
              end
            else
{$endif x86}
              begin
                tmpreg := cg.getfpuregister(current_asmdata.CurrAsmList,left.location.size);
                cg.a_loadfpu_loc_reg(current_asmdata.CurrAsmList,left.location.size,left.location,tmpreg);
                location_reset(left.location,LOC_FPUREGISTER,left.location.size);
                left.location.register := tmpreg;
{$ifdef x86}
                { left operand is now on top of the stack, instead of the right one! }
                if (right.location.loc=LOC_FPUREGISTER) then
                  toggleflag(nf_swapped);
{$endif x86}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef x86}
            if use_sse(left.resultdef) then
              begin
                tmpreg := cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
                cg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,left.location.size,left.location,tmpreg,mms_movescalar);
                location_reset(left.location,LOC_MMREGISTER,left.location.size);
                left.location.register := tmpreg;
              end
            else
{$endif x86}
              begin
                tmpreg := cg.getfpuregister(current_asmdata.CurrAsmList,left.location.size);
                cg.a_loadfpu_loc_reg(current_asmdata.CurrAsmList,left.location.size,left.location,tmpreg);
                location_reset(left.location,LOC_FPUREGISTER,left.location.size);
                left.location.register := tmpreg;
{$ifdef x86}
                { left operand is now on top of the stack, instead of the right one! }
                if (right.location.loc=LOC_FPUREGISTER) then
                  toggleflag(nf_swapped);
{$endif x86}
>>>>>>> origin/fixes_2_2
              end;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif x86 and not llvm}
=======
{$endif x86}
>>>>>>> graemeg/cpstrnew
=======
{$endif x86}
>>>>>>> graemeg/cpstrnew
=======
{$endif x86}
>>>>>>> graemeg/cpstrnew
=======
{$endif x86}
>>>>>>> origin/cpstrnew
      end;


    procedure tcgaddnode.set_result_location_reg;
      begin
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
{$ifndef cpu64bitalu}
        if location.size in [OS_64,OS_S64] then
          begin
            location.register64.reglo := cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            location.register64.reghi := cg.getintregister(current_asmdata.CurrAsmList,OS_32);
          end
        else
{$endif}
          location.register := hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
      end;


    procedure tcgaddnode.force_reg_left_right(allow_swap,allow_constant:boolean);
      begin
        if (left.location.loc<>LOC_REGISTER) and
           not(
               allow_constant and
               (left.location.loc in [LOC_CONSTANT,LOC_CREGISTER])
              ) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
        if (right.location.loc<>LOC_REGISTER) and
           not(
               allow_constant and
               (right.location.loc in [LOC_CONSTANT,LOC_CREGISTER]) and
               (left.location.loc<>LOC_CONSTANT)
              ) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,false);

        { Left is always a register, right can be register or constant }
        if left.location.loc=LOC_CONSTANT then
          begin
            { when it is not allowed to swap we have a constant on
              left, that will give problems }
            if not allow_swap then
              internalerror(200307043);
            swapleftright;
          end;
      end;


    function tcgaddnode.cmpnode2topcmp(unsigned: boolean): TOpCmp;
      begin
        if unsigned then
          case nodetype of
            gtn:      result:=OC_A;
            gten:     result:=OC_AE;
            ltn:      result:=OC_B;
            lten:     result:=OC_BE;
            equaln:   result:=OC_EQ;
            unequaln: result:=OC_NE;
          else
            internalerror(2011010412);
          end
        else
          case nodetype of
            gtn:      result:=OC_GT;
            gten:     result:=OC_GTE;
            ltn:      result:=OC_LT;
            lten:     result:=OC_LTE;
            equaln:   result:=OC_EQ;
            unequaln: result:=OC_NE;
          else
            internalerror(2011010412);
          end
      end;

{*****************************************************************************
                                Smallsets
*****************************************************************************}

    procedure tcgaddnode.second_opsmallset;
      begin
        { when a setdef is passed, it has to be a smallset }
<<<<<<< HEAD
<<<<<<< HEAD
        if not(
               ((left.nodetype=setelementn) or is_smallset(left.resultdef)) and
               ((right.nodetype=setelementn) or is_smallset(right.resultdef))
              ) then
          internalerror(200203302);
        if (left.nodetype=setelementn) or (right.nodetype=setelementn) then
          second_addsmallsetelement
        else if nodetype in [equaln,unequaln,gtn,gten,lten,ltn] then
=======
=======
>>>>>>> origin/fixes_2_2
        if is_varset(left.resultdef) or
          is_varset(right.resultdef) then
          internalerror(200203302);

        if nodetype in [equaln,unequaln,gtn,gten,lten,ltn] then
>>>>>>> graemeg/fixes_2_2
          second_cmpsmallset
        else
          second_addsmallset;
      end;


    procedure tcgaddnode.second_addsmallset;
      var
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
        tmpreg : tregister;
        mask,
        setbase : aint;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cgop    : TOpCg;
        opdone  : boolean;
      begin
        opdone := false;
        pass_left_right;
        force_reg_left_right(true,true);
        set_result_location_reg;
<<<<<<< HEAD
<<<<<<< HEAD
        case nodetype of
          addn :
            cgop:=OP_OR;
=======
=======
>>>>>>> origin/fixes_2_2
        if (left.resultdef.typ=setdef) then
          setbase:=tsetdef(left.resultdef).setbase
        else
          setbase:=tsetdef(right.resultdef).setbase;

        case nodetype of
          addn :
            begin
              { are we adding set elements ? }
              if right.nodetype=setelementn then
                begin
                  { no range support for smallsets! }
                  if assigned(tsetelementnode(right).right) then
                   internalerror(43244);
                  if (right.location.loc = LOC_CONSTANT) then
                    begin
                      if (target_info.endian=endian_big) then
                        mask:=aint((aword(1) shl (resultdef.size*8-1)) shr aword(right.location.value-setbase))
                      else
                        mask:=aint(1 shl (right.location.value-setbase));
                      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_OR,location.size,
                        mask,left.location.register,location.register);
                    end
                  else
                    begin
                      if (target_info.endian=endian_big) then
                        begin
                          mask:=aint((aword(1) shl (resultdef.size*8-1)));
                          cgop:=OP_SHR
                        end
                      else
                        begin
                          mask:=1;
                          cgop:=OP_SHL
                        end;
                      tmpreg := cg.getintregister(current_asmdata.CurrAsmList,location.size);
                      cg.a_load_const_reg(current_asmdata.CurrAsmList,location.size,mask,tmpreg);
                      location_force_reg(current_asmdata.CurrAsmList,right.location,location.size,true);
                      register_maybe_adjust_setbase(current_asmdata.CurrAsmList,right.location,setbase);
                      cg.a_op_reg_reg(current_asmdata.CurrAsmList,cgop,location.size,
                        right.location.register,tmpreg);
                      if left.location.loc <> LOC_CONSTANT then
                        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,location.size,tmpreg,
                            left.location.register,location.register)
                      else
                        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_OR,location.size,
                            left.location.value,tmpreg,location.register);
                    end;
                  opdone := true;
                end
              else
                cgop := OP_OR;
            end;
>>>>>>> graemeg/fixes_2_2
          symdifn :
            cgop:=OP_XOR;
          muln :
            cgop:=OP_AND;
          subn :
            begin
              cgop:=OP_AND;
              if (not(nf_swapped in flags)) then
                if (right.location.loc=LOC_CONSTANT) then
                  right.location.value := not(right.location.value)
                else
                  opdone := true
              else if (left.location.loc=LOC_CONSTANT) then
                left.location.value := not(left.location.value)
              else
                 begin
                   swapleftright;
                   opdone := true;
                 end;
              if opdone then
                begin
                  if (right.location.size<>left.location.size) or
                     (location.size<>left.location.size) then
                    internalerror(2010123001);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  { make sure that location.register is different from
                    left.location.register, since right will overwrite it
                    and we'll use left afterwards }
                  if (right.location.loc=LOC_REGISTER) then
                    location.register:=right.location.register
                  else
                    location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
                  { make sure we don't modify left/right.location, because we told
                    force_reg_left_right above that they can be constant }
                  hlcg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NOT,resultdef,right.location.register,location.register);
                  if left.location.loc = LOC_CONSTANT then
                    hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,resultdef,left.location.value,location.register)
                  else
                    hlcg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_AND,resultdef,left.location.register,location.register);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  { make sure we don't modify left/right.location, because we told
                    force_reg_left_right above that they can be constant }
                  cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NOT,location.size,right.location.register,location.register);
                  if left.location.loc = LOC_CONSTANT then
                    cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,location.size,left.location.value,location.register)
                  else
                    cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_AND,location.size,left.location.register,location.register);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                end;
            end;
          else
            internalerror(2002072701);
        end;

        if not opdone then
          begin
            // these are all commutative operations
            if (left.location.loc = LOC_CONSTANT) then
              swapleftright;
            if (right.location.loc = LOC_CONSTANT) then
              hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,cgop,resultdef,
                right.location.value,left.location.register,
                location.register)
            else
              hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,cgop,resultdef,
                right.location.register,left.location.register,
                location.register);
          end;
      end;


    procedure tcgaddnode.second_addsmallsetelement;
      var
        tmpreg : tregister;
        mask,
        setbase : aint;
        cgop    : TOpCg;
      begin
        if nodetype<>addn then
          internalerror(20080302);
        { no range support for smallsets }
        if assigned(tsetelementnode(right).right) then
          internalerror(20080303);
        pass_left_right;
        { setelementn is a special case, it must be on right }
        if (nf_swapped in flags) and
           (left.nodetype=setelementn) then
          swapleftright;
        force_reg_left_right(false,false);
        set_result_location_reg;
        setbase:=tsetdef(left.resultdef).setbase;
        if (right.location.loc = LOC_CONSTANT) then
          begin
            if (target_info.endian=endian_big) then
              mask:=aint((aword(1) shl (resultdef.size*8-1)) shr aword(right.location.value-setbase))
            else
              mask:=aint(1 shl (right.location.value-setbase));
            hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_OR,resultdef,
              mask,left.location.register,location.register);
          end
        else
          begin
            if (target_info.endian=endian_big) then
              begin
                mask:=aint((aword(1) shl (resultdef.size*8-1)));
                cgop:=OP_SHR
              end
            else
              begin
                mask:=1;
                cgop:=OP_SHL
              end;
            tmpreg:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
            hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,mask,tmpreg);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,resultdef,true);
            register_maybe_adjust_setbase(current_asmdata.CurrAsmList,resultdef,right.location,setbase);
            hlcg.a_op_reg_reg(current_asmdata.CurrAsmList,cgop,resultdef,
              right.location.register,tmpreg);
            if left.location.loc <> LOC_CONSTANT then
              hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,resultdef,tmpreg,
                  left.location.register,location.register)
            else
              hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_OR,resultdef,
                  left.location.value,tmpreg,location.register);
          end;
      end;


{*****************************************************************************
                                Boolean
*****************************************************************************}

    procedure tcgaddnode.second_opboolean;
      begin
        if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
          second_cmpboolean
        else
          second_addboolean;
      end;


    procedure tcgaddnode.second_addboolean;
      var
        cgop    : TOpCg;
        truelabel, falselabel : tasmlabel;
        oldflowcontrol : tflowcontrol;
      begin
        { And,Or will only evaluate from left to right only the
          needed nodes unless full boolean evaluation is enabled }
        if (nodetype in [orn,andn]) and
           (not(cs_full_boolean_eval in current_settings.localswitches) or
            (nf_short_bool in flags)) then
          begin
            case nodetype of
              andn :
                begin
                   secondpass(left);
                   hlcg.maketojumpbool(current_asmdata.CurrAsmList,left);
                   hlcg.a_label(current_asmdata.CurrAsmList,left.location.truelabel);
                   current_asmdata.getjumplabel(truelabel);
                   location_reset_jump(location,truelabel,left.location.falselabel);
                end;
              orn :
                begin
                   secondpass(left);
                   hlcg.maketojumpbool(current_asmdata.CurrAsmList,left);
                   hlcg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
                   current_asmdata.getjumplabel(falselabel);
                   location_reset_jump(location,left.location.truelabel,falselabel);
                end;
              else
                internalerror(200307044);
            end;
            { these jumps mean we're now in a flow control construct }
            oldflowcontrol:=flowcontrol;
            include(flowcontrol,fc_inflowcontrol);

            secondpass(right);
            { jump to the same labels as the left side, since the andn/orn
              merges the results of left and right }
            hlcg.maketojumpboollabels(current_asmdata.CurrAsmList,right,location.truelabel,location.falselabel);

            flowcontrol:=oldflowcontrol+(flowcontrol-[fc_inflowcontrol]);
          end
        else
          begin
            pass_left_right;
            force_reg_left_right(false,true);
            set_result_location_reg;

            case nodetype of
              xorn :
                cgop:=OP_XOR;
              orn :
                cgop:=OP_OR;
              andn :
                cgop:=OP_AND;
              else
                 internalerror(200203247);
            end;
{$ifndef cpu64bitalu}
            if right.location.size in [OS_64,OS_S64] then
              begin
                if right.location.loc <> LOC_CONSTANT then
                  cg64.a_op64_reg_reg_reg(current_asmdata.CurrAsmList,cgop,location.size,
                     left.location.register64,right.location.register64,
                     location.register64)
                else
                  cg64.a_op64_const_reg_reg(current_asmdata.CurrAsmList,cgop,location.size,
                     right.location.value,left.location.register64,
                     location.register64);
              end
            else
{$endif cpu64bitalu}
              begin
                if right.location.loc <> LOC_CONSTANT then
                  hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,cgop,resultdef,
                     left.location.register,right.location.register,
                     location.register)
                else
                  hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,cgop,resultdef,
                     right.location.value,left.location.register,
                     location.register);
              end;
         end;
      end;


{*****************************************************************************
                                64-bit
*****************************************************************************}

    procedure tcgaddnode.second_op64bit;
      begin
        if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
          second_cmp64bit
        else
          second_add64bit;
      end;


    procedure tcgaddnode.second_add64bit;
      var
        op         : TOpCG;
        checkoverflow : boolean;
        ovloc : tlocation;
      begin
        ovloc.loc:=LOC_VOID;

        pass_left_right;
        force_reg_left_right(false,true);
        set_result_location_reg;

        { assume no overflow checking is required }
        checkoverflow := false;
        case nodetype of
          addn :
             begin
                op:=OP_ADD;
                checkoverflow:=true;
             end;
          subn :
             begin
                op:=OP_SUB;
                checkoverflow:=true;
             end;
          xorn:
            op:=OP_XOR;
          orn:
            op:=OP_OR;
          andn:
            op:=OP_AND;
          muln:
            begin
              { should be handled in pass_1 (JM) }
              internalerror(200109051);
            end;
          else
            internalerror(2002072705);
        end;

        checkoverflow:=
          checkoverflow and
          (left.resultdef.typ<>pointerdef) and
          (right.resultdef.typ<>pointerdef) and
          (cs_check_overflow in current_settings.localswitches);

{$ifdef cpu64bitalu}
        case nodetype of
          xorn,orn,andn,addn:
            begin
              if (right.location.loc = LOC_CONSTANT) then
                hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,op,resultdef,right.location.value,
                  left.location.register,location.register)
              else
                hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,op,resultdef,right.location.register,
                  left.location.register,location.register);
            end;
          subn:
            begin
              if (nf_swapped in flags) then
                swapleftright;

              if left.location.loc <> LOC_CONSTANT then
                begin
                  if right.location.loc <> LOC_CONSTANT then
                    // reg64 - reg64
                    hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                      right.location.register,left.location.register,location.register,
                      checkoverflow,ovloc)
                  else
                    // reg64 - const64
                    hlcg.a_op_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                      right.location.value,left.location.register,location.register,
                      checkoverflow,ovloc);
                end
              else
                begin
                  // const64 - reg64
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                  hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                    right.location.register,left.location.register,location.register,
                    checkoverflow,ovloc);
                end;
            end;
          else
            internalerror(2002072803);
        end;
{$else cpu64bitalu}
        case nodetype of
          xorn,orn,andn,addn:
            begin
              if (right.location.loc = LOC_CONSTANT) then
                cg64.a_op64_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,op,location.size,right.location.value64,
                  left.location.register64,location.register64,
                  checkoverflow,ovloc)
              else
                cg64.a_op64_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,op,location.size,right.location.register64,
                  left.location.register64,location.register64,
                  checkoverflow,ovloc);
            end;
          subn:
            begin
              if (nf_swapped in flags) then
                swapleftright;

              if left.location.loc <> LOC_CONSTANT then
                begin
                  if right.location.loc <> LOC_CONSTANT then
                    // reg64 - reg64
                    cg64.a_op64_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,location.size,
                      right.location.register64,left.location.register64,
                      location.register64,
                      checkoverflow,ovloc)
                  else
                    // reg64 - const64
                    cg64.a_op64_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,location.size,
                      right.location.value64,left.location.register64,
                      location.register64,
                      checkoverflow,ovloc)
                end
              else
                begin
                  // const64 - reg64
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                  cg64.a_op64_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,location.size,
                    right.location.register64,left.location.register64,
                    location.register64,
                    checkoverflow,ovloc);
                end;
            end;
          else
            internalerror(2002072803);
        end;
{$endif cpu64bitalu}

        { emit overflow check if enabled }
        if checkoverflow then
           hlcg.g_overflowcheck_loc(current_asmdata.CurrAsmList,Location,resultdef,ovloc);
      end;


{*****************************************************************************
                                Strings
*****************************************************************************}

    procedure tcgaddnode.second_addstring;
      begin
        { this should already be handled in pass1 }
        internalerror(2002072402);
      end;


{*****************************************************************************
                                Floats
*****************************************************************************}

    procedure tcgaddnode.second_opfloat;
      begin
        if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
          second_cmpfloat
        else
          second_addfloat;
      end;


{*****************************************************************************
                                Ordinals
*****************************************************************************}

    procedure tcgaddnode.second_opordinal;
      begin
         if (nodetype in [ltn,lten,gtn,gten,equaln,unequaln]) then
           second_cmpordinal
         else
           second_addordinal;
      end;


    procedure tcgaddnode.second_addordinal;
      var
        unsigned,
        checkoverflow : boolean;
        cgop   : topcg;
        tmpreg : tregister;
        ovloc : tlocation;
      begin
        ovloc.loc:=LOC_VOID;

        pass_left_right;
        force_reg_left_right(false,true);
        set_result_location_reg;

        { determine if the comparison will be unsigned }
        unsigned:=not(is_signed(left.resultdef)) or
                    not(is_signed(right.resultdef));

        { assume no overflow checking is require }
        checkoverflow := false;

        case nodetype of
          addn:
            begin
              cgop:=OP_ADD;
              checkoverflow:=true;
            end;
          xorn :
            begin
              cgop:=OP_XOR;
            end;
          orn :
            begin
              cgop:=OP_OR;
            end;
          andn:
            begin
              cgop:=OP_AND;
            end;
          muln:
            begin
              checkoverflow:=true;
              if unsigned then
                cgop:=OP_MUL
              else
                cgop:=OP_IMUL;
            end;
          subn :
            begin
              checkoverflow:=true;
              cgop:=OP_SUB;
            end;
          else
            internalerror(2013120104);
        end;

       checkoverflow:=
         checkoverflow and
          (left.resultdef.typ<>pointerdef) and
          (right.resultdef.typ<>pointerdef) and
          (cs_check_overflow in current_settings.localswitches);

       if nodetype<>subn then
        begin
          if (right.location.loc<>LOC_CONSTANT) then
            hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,cgop,resultdef,
               left.location.register,right.location.register,
               location.register,checkoverflow,ovloc)
          else
            hlcg.a_op_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,cgop,resultdef,
               right.location.value,left.location.register,
               location.register,checkoverflow,ovloc);
        end
      else  { subtract is a special case since its not commutative }
        begin
          if (nf_swapped in flags) then
            swapleftright;
          if left.location.loc<>LOC_CONSTANT then
            begin
              if right.location.loc<>LOC_CONSTANT then
                hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                    right.location.register,left.location.register,
                    location.register,checkoverflow,ovloc)
              else
                hlcg.a_op_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                  right.location.value,left.location.register,
                  location.register,checkoverflow,ovloc);
            end
          else
            begin
              tmpreg:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
              hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,
                left.location.value,tmpreg);
              hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                right.location.register,tmpreg,location.register,checkoverflow,ovloc);
            end;
        end;

        { emit overflow check if required }
        if checkoverflow then
          hlcg.g_overflowcheck_loc(current_asmdata.CurrAsmList,Location,resultdef,ovloc);
      end;


    procedure tcgaddnode.second_cmpboolean;
      begin
        second_cmpordinal;
      end;


{*****************************************************************************
                                pass_generate_code;
*****************************************************************************}

    procedure tcgaddnode.pass_generate_code;
      begin
        case left.resultdef.typ of
          orddef :
            begin
              { handling boolean expressions }
              if is_boolean(left.resultdef) and
                 is_boolean(right.resultdef) then
                second_opboolean
              { 64bit operations }
              else if is_64bit(left.resultdef) then
                second_op64bit
              else
                second_opordinal;
            end;
          stringdef :
            begin
              second_addstring;
            end;
          setdef :
            begin
<<<<<<< HEAD
              if is_smallset(tsetdef(left.resultdef)) then
                second_opsmallset
=======
              {Normalsets are already handled in pass1 if mmx
               should not be used.}
              if is_varset(tsetdef(left.resultdef)) then
                begin
{$ifdef SUPPORT_MMX}
                {$ifdef i386}
                  if cs_mmx in current_settings.localswitches then
                    second_opmmxset
                  else
                {$endif}
{$endif SUPPORT_MMX}
                    internalerror(200109041);
                end
>>>>>>> graemeg/fixes_2_2
              else
                internalerror(200109041);
            end;
          arraydef :
            begin
              { support dynarr=nil }
              if is_dynamic_array(left.resultdef) then
                second_opordinal
              else
                if (cs_support_vectors in current_settings.globalswitches) and
                   is_vector(left.resultdef) then
                  second_opvector
{$ifdef SUPPORT_MMX}
              else
                if is_mmx_able_array(left.resultdef) then
                  second_opmmx
{$endif SUPPORT_MMX}
              else
                internalerror(200306016);
            end;
          floatdef :
            second_opfloat;
          else
            second_opordinal;
        end;
      end;

begin
   caddnode:=tcgaddnode;
end.
