{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Code generation for add nodes on the ARM

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
unit narmadd;

{$i fpcdefs.inc}

interface

    uses
       node,ncgadd,cpubase;

    type
       tarmaddnode = class(tcgaddnode)
       private
          function  GetResFlags(unsigned:Boolean):TResFlags;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          function  GetFpuResFlags:TResFlags;
       public
          function pass_1 : tnode;override;
          function use_generic_mul32to64: boolean; override;
          function use_generic_mul64bit: boolean; override;
       protected
          function first_addfloat: tnode; override;
          procedure second_addordinal;override;
=======
       public
          function pass_1 : tnode;override;
       protected
>>>>>>> graemeg/cpstrnew
=======
       public
          function pass_1 : tnode;override;
       protected
>>>>>>> graemeg/cpstrnew
=======
       public
          function pass_1 : tnode;override;
       protected
>>>>>>> graemeg/cpstrnew
=======
       public
          function pass_1 : tnode;override;
       protected
>>>>>>> origin/cpstrnew
          procedure second_addfloat;override;
          procedure second_cmpfloat;override;
          procedure second_cmpordinal;override;
          procedure second_cmpsmallset;override;
          procedure second_cmp64bit;override;
          procedure second_add64bit;override;
       end;

  implementation

    uses
      globtype,verbose,globals,systems,
      constexp,symdef,symtable,symtype,symconst,
      aasmbase,aasmdata,aasmcpu,
      defutil,htypechk,cgbase,cgutils,
      cpuinfo,pass_1,pass_2,procinfo,
      ncon,nadd,ncnv,ncal,nmat,
      ncgutil,cgobj,cgcpu,
      hlcgobj
      ;

{*****************************************************************************
                               TSparcAddNode
*****************************************************************************}

    function tarmaddnode.GetResFlags(unsigned:Boolean):TResFlags;
      begin
        case NodeType of
          equaln:
            GetResFlags:=F_EQ;
          unequaln:
            GetResFlags:=F_NE;
          else
            if not(unsigned) then
              begin
                if nf_swapped in flags then
                  case NodeType of
                    ltn:
                      GetResFlags:=F_GT;
                    lten:
                      GetResFlags:=F_GE;
                    gtn:
                      GetResFlags:=F_LT;
                    gten:
                      GetResFlags:=F_LE;
                    else
                      internalerror(201408203);
                  end
                else
                  case NodeType of
                    ltn:
                      GetResFlags:=F_LT;
                    lten:
                      GetResFlags:=F_LE;
                    gtn:
                      GetResFlags:=F_GT;
                    gten:
                      GetResFlags:=F_GE;
                    else
                      internalerror(201408204);
                  end;
              end
            else
              begin
                if nf_swapped in Flags then
                  case NodeType of
                    ltn:
                      GetResFlags:=F_HI;
                    lten:
                      GetResFlags:=F_CS;
                    gtn:
                      GetResFlags:=F_CC;
                    gten:
                      GetResFlags:=F_LS;
                    else
                      internalerror(201408205);
                  end
                else
                  case NodeType of
                    ltn:
                      GetResFlags:=F_CC;
                    lten:
                      GetResFlags:=F_LS;
                    gtn:
                      GetResFlags:=F_HI;
                    gten:
                      GetResFlags:=F_CS;
                    else
                      internalerror(201408206);
                  end;
              end;
        end;
      end;


    function tarmaddnode.GetFpuResFlags:TResFlags;
      begin
        if nf_swapped in Flags then
          internalerror(2014042001);
        case NodeType of
          equaln:
            result:=F_EQ;
          unequaln:
            result:=F_NE;
          ltn:
            result:=F_MI;
          lten:
            result:=F_LS;
          gtn:
            result:=F_GT;
          gten:
            result:=F_GE;
          else
            internalerror(201408207);
        end;
      end;


    procedure tarmaddnode.second_addfloat;
      var
        op : TAsmOp;
        singleprec: boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pf: TOpPostfix;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
        pass_left_right;
        if (nf_swapped in flags) then
          swapleftright;

        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            begin
              { force fpureg as location, left right doesn't matter
                as both will be in a fpureg }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
              location_force_fpureg(current_asmdata.CurrAsmList,right.location,(left.location.loc<>LOC_CFPUREGISTER));

              location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
              if left.location.loc<>LOC_CFPUREGISTER then
                location.register:=left.location.register
              else
                location.register:=right.location.register;
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

              case nodetype of
                addn :
                  op:=A_ADF;
                muln :
                  op:=A_MUF;
                subn :
                  op:=A_SUF;
                slashn :
                  op:=A_DVF;
                else
                  internalerror(200308313);
              end;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register),
                 cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fpu_vfpv3,
          fpu_vfpv3_d16:
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              singleprec:=tfloatdef(left.resultdef).floattype=s32real;
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              case nodetype of
                addn :
                  op:=A_VADD;
                muln :
                  op:=A_VMUL;
                subn :
                  op:=A_VSUB;
                slashn :
                  op:=A_VDIV;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register),pf));
            end;
          fpu_fpv4_s16:
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              case nodetype of
                addn :
                  op:=A_VADD;
                muln :
                  op:=A_VMUL;
                subn :
                  op:=A_VSUB;
                slashn :
                  op:=A_VDIV;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op, location.register,left.location.register,right.location.register), PF_F32));
=======
          fpu_vfpv3:
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              if left.location.loc<>LOC_CMMREGISTER then
                location.register:=left.location.register
              else if right.location.loc<>LOC_CMMREGISTER then
                location.register:=right.location.register
              else
                location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

=======
          fpu_vfpv3:
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              if left.location.loc<>LOC_CMMREGISTER then
                location.register:=left.location.register
              else if right.location.loc<>LOC_CMMREGISTER then
                location.register:=right.location.register
              else
                location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

>>>>>>> graemeg/cpstrnew
              singleprec:=tfloatdef(left.resultdef).floattype=s32real;
              case nodetype of
                addn :
                  if singleprec then
                    op:=A_FADDS
                  else
                    op:=A_FADDD;
                muln :
                  if singleprec then
                    op:=A_FMULS
                  else
                    op:=A_FMULD;
                subn :
                  if singleprec then
                    op:=A_FSUBS
                  else
                    op:=A_FSUBD;
                slashn :
                  if singleprec then
                    op:=A_FDIVS
                  else
                    op:=A_FDIVD;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register));
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
          fpu_vfpv3:
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              if left.location.loc<>LOC_CMMREGISTER then
                location.register:=left.location.register
              else if right.location.loc<>LOC_CMMREGISTER then
                location.register:=right.location.register
              else
                location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              singleprec:=tfloatdef(left.resultdef).floattype=s32real;
              case nodetype of
                addn :
                  if singleprec then
                    op:=A_FADDS
                  else
                    op:=A_FADDD;
                muln :
                  if singleprec then
                    op:=A_FMULS
                  else
                    op:=A_FMULD;
                subn :
                  if singleprec then
                    op:=A_FSUBS
                  else
                    op:=A_FSUBD;
                slashn :
                  if singleprec then
                    op:=A_FDIVS
                  else
                    op:=A_FDIVD;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register));
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
          fpu_vfpv3:
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              if left.location.loc<>LOC_CMMREGISTER then
                location.register:=left.location.register
              else if right.location.loc<>LOC_CMMREGISTER then
                location.register:=right.location.register
              else
                location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              singleprec:=tfloatdef(left.resultdef).floattype=s32real;
              case nodetype of
                addn :
                  if singleprec then
                    op:=A_FADDS
                  else
                    op:=A_FADDD;
                muln :
                  if singleprec then
                    op:=A_FMULS
                  else
                    op:=A_FMULD;
                subn :
                  if singleprec then
                    op:=A_FSUBS
                  else
                    op:=A_FSUBD;
                slashn :
                  if singleprec then
                    op:=A_FDIVS
                  else
                    op:=A_FDIVD;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register));
>>>>>>> origin/cpstrnew
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(200308252);
          else
            internalerror(200308251);
        end;
      end;


    procedure tarmaddnode.second_cmpfloat;
      var
        op: TAsmOp;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pf: TOpPostfix;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
        pass_left_right;
        if (nf_swapped in flags) then
          swapleftright;

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getresflags(false);

        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            begin
              { force fpureg as location, left right doesn't matter
                as both will be in a fpureg }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,true);
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
              location_force_fpureg(current_asmdata.CurrAsmList,right.location,true);
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
              location_force_fpureg(current_asmdata.CurrAsmList,right.location,true);
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
              location_force_fpureg(current_asmdata.CurrAsmList,right.location,true);

              if nodetype in [equaln,unequaln] then
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMF,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]))
              else
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMFE,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_vfpv2,
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              if (tfloatdef(left.resultdef).floattype=s32real) then
                if nodetype in [equaln,unequaln] then
                  op:=A_FCMPS
                 else
                   op:=A_FCMPES
              else if nodetype in [equaln,unequaln] then
                op:=A_FCMPD
              else
                op:=A_FCMPED;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,
                left.location.register,right.location.register));
              current_asmdata.CurrAsmList.concat(taicpu.op_none(A_FMSTAT));
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(2009112404);
        end;
>>>>>>> origin/cpstrnew

              if nodetype in [equaln,unequaln] then
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMF,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]))
              else
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMFE,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_vfpv2,
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              if (tfloatdef(left.resultdef).floattype=s32real) then
                if nodetype in [equaln,unequaln] then
                  op:=A_FCMPS
                 else
                   op:=A_FCMPES
              else if nodetype in [equaln,unequaln] then
                op:=A_FCMPD
              else
                op:=A_FCMPED;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,
                left.location.register,right.location.register));
              current_asmdata.CurrAsmList.concat(taicpu.op_none(A_FMSTAT));
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(2009112404);
        end;
>>>>>>> graemeg/cpstrnew

              if nodetype in [equaln,unequaln] then
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMF,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]))
              else
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMFE,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_vfpv2,
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              if (tfloatdef(left.resultdef).floattype=s32real) then
                if nodetype in [equaln,unequaln] then
                  op:=A_FCMPS
                 else
                   op:=A_FCMPES
              else if nodetype in [equaln,unequaln] then
                op:=A_FCMPD
              else
                op:=A_FCMPED;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,
                left.location.register,right.location.register));
              current_asmdata.CurrAsmList.concat(taicpu.op_none(A_FMSTAT));
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(2009112404);
        end;
>>>>>>> graemeg/cpstrnew

              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
              location_force_fpureg(current_asmdata.CurrAsmList,right.location,true);

>>>>>>> graemeg/cpstrnew
              if nodetype in [equaln,unequaln] then
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMF,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]))
              else
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMFE,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_vfpv2,
<<<<<<< HEAD
          fpu_vfpv3,
          fpu_vfpv3_d16:
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);
=======
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
              location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,true);

              if (tfloatdef(left.resultdef).floattype=s32real) then
                if nodetype in [equaln,unequaln] then
                  op:=A_FCMPS
                 else
                   op:=A_FCMPES
              else if nodetype in [equaln,unequaln] then
                op:=A_FCMPD
              else
                op:=A_FCMPED;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,
                left.location.register,right.location.register));
              current_asmdata.CurrAsmList.concat(taicpu.op_none(A_FMSTAT));
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(2009112404);
        end;
>>>>>>> graemeg/cpstrnew

              if nodetype in [equaln,unequaln] then
                op:=A_VCMP
              else
                op:=A_VCMPE;

              if (tfloatdef(left.resultdef).floattype=s32real) then
                pf:=PF_F32
              else
                pf:=PF_F64;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(op,
                left.location.register,right.location.register), pf));
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VMRS,NR_APSR_nzcv,NR_FPSCR));
              location.resflags:=GetFpuResFlags;
            end;
          fpu_fpv4_s16:
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              if nodetype in [equaln,unequaln] then
                op:=A_VCMP
              else
                op:=A_VCMPE;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(op,
                left.location.register,right.location.register),PF_F32));
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg(A_VMRS, NR_APSR_nzcv, NR_FPSCR));
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(2009112404);
        end;
      end;


    procedure tarmaddnode.second_cmpsmallset;
      var
        tmpreg : tregister;
        b: byte;
      begin
        pass_left_right;

        location_reset(location,LOC_FLAGS,OS_NO);

        if (not(nf_swapped in flags) and
            (nodetype = lten)) or
           ((nf_swapped in flags) and
            (nodetype = gten)) then
          swapleftright;

        (* Try to keep right as a constant *)
        if (right.location.loc <> LOC_CONSTANT) or
          not(is_shifter_const(right.location.value, b)) or
          ((GenerateThumbCode) and not(is_thumb_imm(right.location.value))) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

        case nodetype of
          equaln,
          unequaln:
            begin
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              if right.location.loc = LOC_CONSTANT then
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,left.location.register,right.location.value))
              else
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register,right.location.register));
              if nodetype = equaln then
                location.resflags:=F_EQ
              else
                location.resflags:=F_NE;
            end;
          lten,
          gten:
            begin
              tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
              if right.location.loc = LOC_CONSTANT then
                begin
                  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,right.location.value,left.location.register,tmpreg);
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,tmpreg,right.location.value));
                end
              else
                begin
                  cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,left.location.register,right.location.register,tmpreg);
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,tmpreg,right.location.register));
                end;
              location.resflags:=F_EQ;
            end;
          else
            internalerror(2004012401);
        end;
      end;


    procedure tarmaddnode.second_cmp64bit;
      var
        unsigned : boolean;
        oldnodetype : tnodetype;
        dummyreg : tregister;
        truelabel, falselabel: tasmlabel;
        l: tasmlabel;
      const
        lt_zero_swapped: array[boolean] of tnodetype = (ltn, gtn);
      begin
        truelabel:=nil;
        falselabel:=nil;
        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        pass_left_right;

        { pass_left_right moves possible consts to the right, the only
          remaining case with left consts (currency) can take this path too (KB) }
        if (right.nodetype=ordconstn) and
           (tordconstnode(right).value=0) and
           ((nodetype in [equaln,unequaln]) or
            (not(GenerateThumbCode) and is_signed(left.resultdef) and (nodetype = lt_zero_swapped[nf_swapped in Flags]))
           ) then
          begin
            location_reset(location,LOC_FLAGS,OS_NO);
<<<<<<< HEAD
            if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

            cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
            { Optimize for the common case of int64 < 0 }
            if nodetype in [ltn, gtn] then
              begin
                {Just check for the MSB in reghi to be set or not, this is independed from nf_swapped}
                location.resflags:=F_NE;
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_TST,left.location.register64.reghi, aint($80000000)));
              end
            else
              begin
                location.resflags:=getresflags(unsigned);
                dummyreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);

                if GenerateThumbCode then
                  cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.register64.reglo,left.location.register64.reghi,dummyreg)
                else
                  current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ORR,dummyreg,left.location.register64.reglo,left.location.register64.reghi),PF_S));
              end;
=======
            location.resflags:=getresflags(unsigned);
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reghi,right.location.register64.reghi));
            if current_settings.cputype in cpu_thumb2 then
              current_asmdata.CurrAsmList.concat(taicpu.op_cond(A_IT, C_EQ));
            current_asmdata.CurrAsmList.concat(setcondition(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo),C_EQ));
>>>>>>> graemeg/cpstrnew
          end
        else
          begin
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

            { operation requiring proper N, Z and C flags ? }
            if unsigned or (nodetype in [equaln,unequaln]) then
              begin
                location_reset(location,LOC_FLAGS,OS_NO);
                location.resflags:=getresflags(unsigned);
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reghi,right.location.register64.reghi));
                if GenerateThumbCode or GenerateThumb2Code then
                  begin
                    current_asmdata.getjumplabel(l);
                    cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NE,l);
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo));
                    cg.a_label(current_asmdata.CurrAsmList,l);
                  end
                else
                  current_asmdata.CurrAsmList.concat(setcondition(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo),C_EQ));
              end
            else
            { operation requiring proper N, Z and V flags ? }
              begin
                current_asmdata.getjumplabel(truelabel);
                current_asmdata.getjumplabel(falselabel);
                location_reset_jump(location,truelabel,falselabel);
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reghi,right.location.register64.reghi));
                { the jump the sequence is a little bit hairy }
                case nodetype of
                   ltn,gtn:
                     begin
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(false),location.truelabel);
                        { cheat a little bit for the negative test }
                        toggleflag(nf_swapped);
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(false),location.falselabel);
                        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        toggleflag(nf_swapped);
                     end;
                   lten,gten:
                     begin
                        oldnodetype:=nodetype;
                        if nodetype=lten then
                          nodetype:=ltn
                        else
                          nodetype:=gtn;
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.truelabel);
                        { cheat for the negative test }
                        if nodetype=ltn then
                          nodetype:=gtn
                        else
                          nodetype:=ltn;
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.falselabel);
                        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        nodetype:=oldnodetype;
                     end;
                end;
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo));
                { the comparisaion of the low dword have to be
                   always unsigned!                            }
                cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(true),location.truelabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
                cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              end;
          end;
      end;

    procedure tarmaddnode.second_add64bit;
      var
        asmList : TAsmList;
        ll,rl,res : TRegister64;
        tmpreg: TRegister;
      begin
        if (nodetype in [muln]) then
          begin
            asmList := current_asmdata.CurrAsmList;
            pass_left_right;
            force_reg_left_right(true, (left.location.loc<>LOC_CONSTANT) and (right.location.loc<>LOC_CONSTANT));
            set_result_location_reg;

            { shortcuts to register64s }
            ll:=left.location.register64;
            rl:=right.location.register64;
            res:=location.register64;

            tmpreg := cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            asmList.concat(taicpu.op_reg_reg_reg(A_MUL,tmpreg,ll.reglo,rl.reghi));
            asmList.concat(taicpu.op_reg_reg_reg_reg(A_UMULL,res.reglo,res.reghi,rl.reglo,ll.reglo));
            tbasecgarm(cg).safe_mla(asmList,tmpreg,rl.reglo,ll.reghi,tmpreg);
            asmList.concat(taicpu.op_reg_reg_reg(A_ADD,res.reghi,tmpreg,res.reghi));
          end
        else
          inherited second_add64bit;
      end;

    function tarmaddnode.pass_1 : tnode;
      var
        unsigned : boolean;
      begin
        result:=inherited pass_1;

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
<<<<<<< HEAD
<<<<<<< HEAD
      end;

    function tarmaddnode.first_addfloat: tnode;
      var
        procname: string[31];
        { do we need to reverse the result ? }
        notnode : boolean;
        fdef : tdef;
      begin
        result := nil;
        notnode := false;

        if current_settings.fputype = fpu_fpv4_s16 then
          begin
            case tfloatdef(left.resultdef).floattype of
              s32real:
                begin
                  result:=nil;
                  notnode:=false;
                end;
              s64real:
                begin
                  fdef:=search_system_type('FLOAT64').typedef;
                  procname:='float64';

                  case nodetype of
                    addn:
                      procname:=procname+'_add';
                    muln:
                      procname:=procname+'_mul';
                    subn:
                      procname:=procname+'_sub';
                    slashn:
                      procname:=procname+'_div';
                    ltn:
                      procname:=procname+'_lt';
                    lten:
                      procname:=procname+'_le';
                    gtn:
                      begin
                        procname:=procname+'_lt';
                        swapleftright;
                      end;
                    gten:
                      begin
                        procname:=procname+'_le';
                        swapleftright;
                      end;
                    equaln:
                      procname:=procname+'_eq';
                    unequaln:
                      begin
                        procname:=procname+'_eq';
                        notnode:=true;
                      end;
                    else
                      CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),left.resultdef.typename,right.resultdef.typename);
                  end;

                  if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
                    resultdef:=pasbool8type;
                  result:=ctypeconvnode.create_internal(ccallnode.createintern(procname,ccallparanode.create(
                      ctypeconvnode.create_internal(right,fdef),
                      ccallparanode.create(
                        ctypeconvnode.create_internal(left,fdef),nil))),resultdef);

                  left:=nil;
                  right:=nil;

                  { do we need to reverse the result }
                  if notnode then
                    result:=cnotnode.create(result);
                end;
            end;
          end
        else
          result:=inherited first_addfloat;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure tarmaddnode.second_cmpordinal;
      var
        unsigned : boolean;
        tmpreg : tregister;
        b : byte;
      begin
        pass_left_right;
        force_reg_left_right(true,true);

        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));
        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        if right.location.loc = LOC_CONSTANT then
          begin
             if (not(GenerateThumbCode) and is_shifter_const(right.location.value,b)) or
                ((GenerateThumbCode) and is_thumb_imm(right.location.value)) then
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,left.location.register,right.location.value))
             else
               begin
                 tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
                 cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,
                   right.location.value,tmpreg);
                 current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register,tmpreg));
               end;
          end
        else
          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register,right.location.register));

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getresflags(unsigned);
      end;

    const
      multops: array[boolean] of TAsmOp = (A_SMULL, A_UMULL);

    procedure tarmaddnode.second_addordinal;
      var
        unsigned: boolean;
      begin
        if (nodetype=muln) and
           is_64bit(resultdef) and
           not(GenerateThumbCode) and
           (CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]) then
          begin
            pass_left_right;
            force_reg_left_right(true, false);
            set_result_location_reg;
            unsigned:=not(is_signed(left.resultdef)) or
                      not(is_signed(right.resultdef));
            current_asmdata.CurrAsmList.Concat(
              taicpu.op_reg_reg_reg_reg(multops[unsigned], location.register64.reglo, location.register64.reghi,
                                        left.location.register,right.location.register));
          end
        else
          inherited second_addordinal;
      end;

    function tarmaddnode.use_generic_mul32to64: boolean;
      begin
        result:=GenerateThumbCode or not(CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]);
      end;

    function tarmaddnode.use_generic_mul64bit: boolean;
      begin
        result:=GenerateThumbCode or
          not(CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]) or
          (cs_check_overflow in current_settings.localswitches);
      end;

begin
  caddnode:=tarmaddnode;
end.
