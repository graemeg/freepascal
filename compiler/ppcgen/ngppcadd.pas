{
    Copyright (c) 2000-2006 by Florian Klaempfl and Jonas Maebe

    Code generation for add nodes on the PowerPC (32 and 64 bit generic)

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

unit ngppcadd;


{$i fpcdefs.inc}

interface

    uses
       node,nadd,ncgadd,cpubase;

    type
       tgenppcaddnode = class(tcgaddnode)
          function pass_1: tnode; override;
         protected
          procedure pass_left_and_right;
          procedure load_left_right(cmpop, load_constants: boolean);
          function  getresflags : tresflags;
          procedure emit_compare(unsigned: boolean); virtual; abstract;
          procedure second_addfloat;override;
          procedure second_addboolean;override;
          procedure second_addsmallset;override;
       end;


implementation

{*****************************************************************************
                                  Pass 1
*****************************************************************************}

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,htypechk,
      cgbase,cpuinfo,pass_1,pass_2,regvars,
      cpupara,cgcpu,cgutils,procinfo,
      ncon,nset,
      ncgutil,tgobj,rgobj,rgcpu,cgobj,hlcgobj;


{*****************************************************************************
                                  Pass 1
*****************************************************************************}

    function tgenppcaddnode.pass_1: tnode;
      begin
        typecheckpass(left);
        if (nodetype in [equaln,unequaln]) and
           (left.resultdef.typ = orddef) and
           is_64bit(left.resultdef) then
          begin
            result := nil;
            firstpass(left);
            firstpass(right);
            expectloc := LOC_FLAGS;
            exit;
          end;
        result := inherited pass_1;
      end;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    procedure tgenppcaddnode.pass_left_and_right;
      begin
        { calculate the operator which is more difficult }
        firstcomplex(self);

        { in case of constant put it to the left }
        if (left.nodetype=ordconstn) then
         swapleftright;

        secondpass(left);
        secondpass(right);
      end;


    procedure tgenppcaddnode.load_left_right(cmpop, load_constants: boolean);

      procedure load_node(var n: tnode);
        begin
          case n.location.loc of
            LOC_REGISTER,
            LOC_CREGISTER:
              ;
            LOC_CONSTANT:
              begin
                if load_constants then
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,n.location,n.resultdef,n.resultdef,false);
              end;
            else
              hlcg.location_force_reg(current_asmdata.CurrAsmList,n.location,n.resultdef,n.resultdef,false);
          end;
        end;

      begin
        load_node(left);
        load_node(right);
        if not(cmpop) then
          begin
            location.register := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
{$ifndef cpu64bitalu}
            if is_64bit(resultdef) then
              location.register64.reghi := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
{$endif not cpu64bitalu}
         end;
      end;


    function tgenppcaddnode.getresflags : tresflags;
      begin
        if (left.resultdef.typ <> floatdef) then
          begin
            result.cr := RS_CR0;
            case nodetype of
              equaln : result.flag:=F_EQ;
              unequaln : result.flag:=F_NE;
            else
              if nf_swapped in flags then
                case nodetype of
                  ltn : result.flag:=F_GT;
                  lten : result.flag:=F_GE;
                  gtn : result.flag:=F_LT;
                  gten : result.flag:=F_LE;
                end
              else
                case nodetype of
                  ltn : result.flag:=F_LT;
                  lten : result.flag:=F_LE;
                  gtn : result.flag:=F_GT;
                  gten : result.flag:=F_GE;
                end;
            end
          end
        else
          begin
            result.cr := RS_CR1;
            if (nodetype=equaln) then
              result.flag:=F_EQ
            else if (nodetype=unequaln) then
              result.flag:=F_NE
            else if (nf_swapped in flags) then
              case nodetype of
                ltn : result.flag:=F_FA;
                lten : result.flag:=F_FAE;
                gtn : result.flag:=F_FB;
                gten : result.flag:=F_FBE;
              else
                internalerror(2014031902);
              end
            else
              case nodetype of
                ltn : result.flag:=F_FB;
                lten : result.flag:=F_FBE;
                gtn : result.flag:=F_FA;
                gten : result.flag:=F_FAE;
              else
                internalerror(2014031903);
              end;
          end;
      end;


{*****************************************************************************
                                AddBoolean
*****************************************************************************}

    procedure tgenppcaddnode.second_addboolean;
      var
        cgop      : TOpCg;
        cgsize  : TCgSize;
        cmpop   : boolean;
      begin
        { calculate the operator which is more difficult }
        firstcomplex(self);

        cmpop:=false;
        if (torddef(left.resultdef).ordtype in [pasbool8,bool8bit]) or
           (torddef(right.resultdef).ordtype in [pasbool8,bool8bit]) then
          cgsize:=OS_8
        else if (torddef(left.resultdef).ordtype in [pasbool16,bool16bit]) or
           (torddef(right.resultdef).ordtype in [pasbool16,bool16bit]) then
          cgsize:=OS_16
        else if (torddef(left.resultdef).ordtype in [pasbool32,bool32bit]) or
           (torddef(right.resultdef).ordtype in [pasbool32,bool32bit]) then
          cgsize:=OS_32
        else
          cgsize:=OS_64;

        if {$ifndef cpu64bitalu}(cgsize<>OS_64) and{$endif}
           (((cs_full_boolean_eval in current_settings.localswitches) and
             not(nf_short_bool in flags)) or
            (nodetype in [unequaln,ltn,lten,gtn,gten,equaln,xorn])) then
          begin
            if left.nodetype in [ordconstn,realconstn] then
             swapleftright;

            secondpass(left);
            if (left.expectloc=LOC_JUMP)<>
               (left.location.loc=LOC_JUMP) then
              internalerror(2003122901);
            if left.location.loc in [LOC_FLAGS,LOC_JUMP] then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(cgsize),false);

            secondpass(right);
            if (right.expectloc=LOC_JUMP)<>
               (right.location.loc=LOC_JUMP) then
              internalerror(200312292);
            if right.location.loc in [LOC_FLAGS,LOC_JUMP] then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cgsize_orddef(cgsize),false);

            cmpop := nodetype in [ltn,lten,gtn,gten,equaln,unequaln];

            { set result location }
            if not cmpop then
              location_reset(location,LOC_REGISTER,def_cgsize(resultdef))
             else
              location_reset(location,LOC_FLAGS,OS_NO);

            load_left_right(cmpop,false);

            if (left.location.loc = LOC_CONSTANT) then
              swapleftright;

            { compare the }
            case nodetype of
              ltn,lten,gtn,gten,
              equaln,unequaln :
                begin
                  if (right.location.loc <> LOC_CONSTANT) then
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMPLW,
                      left.location.register,right.location.register))
                  else
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMPLWI,
                      left.location.register,longint(right.location.value)));
                  location.resflags := getresflags;
                end;
              else
                begin
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

                  if right.location.loc <> LOC_CONSTANT then
                    cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,cgop,OS_INT,
                      left.location.register,right.location.register,
                      location.register)
                  else
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,cgop,OS_INT,
                      right.location.value,left.location.register,
                      location.register);
                end;
            end;
         end
        else
          inherited second_addboolean;
      end;


{*****************************************************************************
                                AddFloat
*****************************************************************************}

    procedure tgenppcaddnode.second_addfloat;
      var
        op    : TAsmOp;
        cmpop,
        singleprec : boolean;
      begin
        pass_left_and_right;

        cmpop:=false;
        singleprec:=tfloatdef(left.resultdef).floattype=s32real;
        case nodetype of
          addn :
            if singleprec then
              op:=A_FADDS
            else
              op:=A_FADD;
          muln :
            if singleprec then
              op:=A_FMULS
            else
            op:=A_FMUL;
          subn :
            if singleprec then
              op:=A_FSUBS
            else
              op:=A_FSUB;
          slashn :
            if singleprec then
              op:=A_FDIVS
            else
             op:=A_FDIV;
          ltn,lten,gtn,gten,
          equaln,unequaln :
            begin
              op:=A_FCMPO;
              cmpop:=true;
            end;
          else
            internalerror(200403182);
        end;

        // get the operands in the correct order, there are no special cases
        // here, everything is register-based
        if nf_swapped in flags then
          swapleftright;

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
           location_reset(location,LOC_FLAGS,OS_NO);
           location.resflags := getresflags;
         end;

        // emit the actual operation
        if not cmpop then
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,
              location.register,left.location.register,
              right.location.register))
          end
        else
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,
              newreg(R_SPECIALREGISTER,location.resflags.cr,R_SUBNONE),left.location.register,right.location.register))
          end;
      end;


{*****************************************************************************
                                AddSmallSet
*****************************************************************************}

    procedure tgenppcaddnode.second_addsmallset;
      var
        cgop   : TOpCg;
        setbase: aint;
        tmpreg : tregister;
        opdone,
        cmpop  : boolean;
      begin
        if target_info.endian=endian_little then
          begin
            { this code currently assumes big endian }
            if (left.nodetype=setelementn) or (right.nodetype=setelementn) then
              begin
                inherited second_addsmallsetelement;
                exit;
              end
          end;

        cgop:=OP_None;

        pass_left_and_right;

        { when a setdef is passed, it has to be a smallset }
        if (not(nf_swapped in flags) and
            not is_smallset(left.resultdef) or
            (not is_smallset(right.resultdef) and
             (right.nodetype<>setelementn))) or
           ((nf_swapped in flags) and
            not is_smallset(right.resultdef) or
            (not is_smallset(left.resultdef) and
             (left.nodetype<>setelementn))) then
         internalerror(200203359);

        opdone := false;
        cmpop:=nodetype in [equaln,unequaln,lten,gten];

        { set result location }
        if not cmpop then
          location_reset(location,LOC_REGISTER,def_cgsize(resultdef))
         else
          location_reset(location,LOC_FLAGS,OS_NO);

        load_left_right(cmpop,false);

        if not(cmpop) then
          location.register := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);

        if (left.resultdef.typ=setdef) then
          setbase:=tsetdef(left.resultdef).setbase
        else
          setbase:=tsetdef(right.resultdef).setbase;
<<<<<<< HEAD
<<<<<<< HEAD
        if (nf_swapped in flags) and
           ((nodetype=subn) or
            (left.nodetype=setelementn)) then
          swapleftright;
        { we don't support two constant locations (should ideally be handled
          in simplify }
        if (left.location.loc=LOC_CONSTANT) and
           (right.location.loc=LOC_CONSTANT) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_INT,
                      aint((aword(1) shl (resultdef.size*8-1)) shr aword(right.location.value-setbase)),
                      left.location.register,location.register)
                  else
                    begin
                      hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,u32inttype,true);
                      tmpreg := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                      cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,aint((aword(1) shl (resultdef.size*8-1))),tmpreg);
<<<<<<< HEAD
<<<<<<< HEAD
                      register_maybe_adjust_setbase(current_asmdata.CurrAsmList,u32inttype,right.location,setbase);
=======
                      register_maybe_adjust_setbase(current_asmdata.CurrAsmList,right.location,setbase);
>>>>>>> graemeg/fixes_2_2
=======
                      register_maybe_adjust_setbase(current_asmdata.CurrAsmList,right.location,setbase);
>>>>>>> origin/fixes_2_2
                      cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_INT,
                        right.location.register,tmpreg);
                      if left.location.loc <> LOC_CONSTANT then
                        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_INT,tmpreg,
                          left.location.register,location.register)
                      else
                        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_INT,
                          left.location.value,tmpreg,location.register);
                    end;
                  opdone := true;
                end
              else
                cgop := OP_OR;
            end;
          symdifn :
            cgop:=OP_XOR;
          muln :
            cgop:=OP_AND;
          subn :
            begin
              cgop:=OP_AND;
              if (right.location.loc=LOC_CONSTANT) then
                right.location.value := not(right.location.value)
              else
                opdone := true;
              if opdone then
                begin
                  if left.location.loc = LOC_CONSTANT then
                    begin
                      tmpreg := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                      cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,
                        left.location.value,tmpreg);
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ANDC,
                        location.register,tmpreg,right.location.register));
                    end
                  else
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ANDC,
                      location.register,left.location.register,
                      right.location.register));
                end;
            end;
          equaln,
          unequaln :
            begin
              emit_compare(true);
              opdone := true;
            end;
          lten,gten:
            begin
              If (not(nf_swapped in flags) and
                  (nodetype = lten)) or
                 ((nf_swapped in flags) and
                  (nodetype = gten)) then
                swapleftright;
              // now we have to check whether left >= right
              tmpreg := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
              if left.location.loc = LOC_CONSTANT then
                begin
                  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_AND,OS_INT,
                    not(left.location.value),right.location.register,tmpreg);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMPWI,tmpreg,0));
                  // the two instructions above should be folded together by
                  // the peepholeoptimizer
                end
              else
                begin
                  if right.location.loc = LOC_CONSTANT then
                    begin
                      cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,
                        right.location.value,tmpreg);
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ANDC_,tmpreg,
                        tmpreg,left.location.register));
                    end
                  else
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ANDC_,tmpreg,
                      right.location.register,left.location.register));
                end;
              location.resflags.cr := RS_CR0;
              location.resflags.flag := F_EQ;
              opdone := true;
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
              cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,cgop,OS_INT,
                right.location.value,left.location.register,
                location.register)
            else
              cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,cgop,OS_INT,
                right.location.register,left.location.register,
                location.register);
          end;
      end;

end.
