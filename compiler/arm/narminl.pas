{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generates ARM inline nodes

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
unit narminl;

{$i fpcdefs.inc}

interface

    uses
      node,ninl,ncginl;

    type
      tarminlinenode = class(tcgInlineNode)
        function first_abs_real: tnode; override;
        function first_sqr_real: tnode; override;
        function first_sqrt_real: tnode; override;
        { atn,sin,cos,lgn isn't supported by the linux fpe
        function first_arctan_real: tnode; override;
        function first_ln_real: tnode; override;
        function first_cos_real: tnode; override;
        function first_sin_real: tnode; override;
        }
        procedure second_abs_real; override;
        procedure second_sqr_real; override;
        procedure second_sqrt_real; override;
        { atn,sin,cos,lgn isn't supported by the linux fpe
        procedure second_arctan_real; override;
        procedure second_ln_real; override;
        procedure second_cos_real; override;
        procedure second_sin_real; override;
        }
        procedure second_prefetch; override;
        procedure second_abs_long; override;
      private
        procedure load_fpu_location(out singleprec: boolean);
      end;


implementation

    uses
      globtype,verbose,globals,
      cpuinfo, defutil,symdef,aasmdata,aasmcpu,
      cgbase,cgutils,pass_1,pass_2,
      cpubase,ncgutil,cgobj,cgcpu, hlcgobj;

{*****************************************************************************
                              tarminlinenode
*****************************************************************************}

    procedure tarminlinenode.load_fpu_location(out singleprec: boolean);
      begin
        secondpass(left);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> graemeg/cpstrnew
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> graemeg/cpstrnew
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> graemeg/cpstrnew
=======
              location_force_fpureg(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> origin/cpstrnew
              location_copy(location,left.location);
              if left.location.loc=LOC_CFPUREGISTER then
                begin
                 location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
                 location.loc := LOC_FPUREGISTER;
               end;
            end;
          fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fpu_vfpv3,
          fpu_vfpv3_d16,
          fpu_fpv4_s16:
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
=======
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> graemeg/cpstrnew
=======
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> graemeg/cpstrnew
=======
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> graemeg/cpstrnew
=======
          fpu_vfpv3:
            begin
              location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,true);
>>>>>>> origin/cpstrnew
              location_copy(location,left.location);
              if left.location.loc=LOC_CMMREGISTER then
                begin
                 location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
                 location.loc := LOC_MMREGISTER;
               end;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fpu_soft:
            begin
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
              location_copy(location,left.location);
            end
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          else
            internalerror(2009111801);
        end;
        singleprec:=tfloatdef(left.resultdef).floattype=s32real;
      end;


    function tarminlinenode.first_abs_real : tnode;
      begin
        if (cs_fp_emulation in current_settings.moduleswitches) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          begin
            firstpass(left);
            expectloc:=LOC_REGISTER;
            first_abs_real:=nil;
          end
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          result:=inherited first_abs_real
>>>>>>> graemeg/cpstrnew
        else
          begin
            case current_settings.fputype of
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                expectloc:=LOC_FPUREGISTER;
              fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              fpu_vfpv3,
              fpu_vfpv3_d16:
                expectloc:=LOC_MMREGISTER;
              fpu_fpv4_s16:
                begin
                  if tfloatdef(left.resultdef).floattype=s32real then
                    expectloc:=LOC_MMREGISTER
                  else
                    exit(inherited first_abs_real);
                end;
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> origin/cpstrnew
              else
                internalerror(2009112401);
            end;
            first_abs_real:=nil;
          end;
      end;


    function tarminlinenode.first_sqr_real : tnode;
      begin
        if (cs_fp_emulation in current_settings.moduleswitches) then
          result:=inherited first_sqr_real
        else
          begin
            case current_settings.fputype of
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                expectloc:=LOC_FPUREGISTER;
              fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              fpu_vfpv3,
              fpu_vfpv3_d16:
                expectloc:=LOC_MMREGISTER;
              fpu_fpv4_s16:
                begin
                  if tfloatdef(left.resultdef).floattype=s32real then
                    expectloc:=LOC_MMREGISTER
                  else
                    exit(inherited first_sqr_real);
                end;
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> origin/cpstrnew
              else
                internalerror(2009112402);
            end;
            first_sqr_real:=nil;
          end;
      end;


    function tarminlinenode.first_sqrt_real : tnode;
      begin
        if cs_fp_emulation in current_settings.moduleswitches then
          result:=inherited first_sqrt_real
        else
          begin
            case current_settings.fputype of
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                expectloc:=LOC_FPUREGISTER;
              fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              fpu_vfpv3,
              fpu_vfpv3_d16:
                expectloc:=LOC_MMREGISTER;
              fpu_fpv4_s16:
                begin
                  if tfloatdef(left.resultdef).floattype=s32real then
                    expectloc:=LOC_MMREGISTER
                  else
                    exit(inherited first_sqrt_real);
                end;
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> graemeg/cpstrnew
=======
              fpu_vfpv3:
                expectloc:=LOC_MMREGISTER;
>>>>>>> origin/cpstrnew
              else
                internalerror(2009112403);
            end;
            first_sqrt_real := nil;
          end;
      end;


    { atn,sin,cos,lgn isn't supported by the linux fpe
    function tarminlinenode.first_arctan_real: tnode;
      begin
        expectloc:=LOC_FPUREGISTER;
        result:=nil;
      end;


    function tarminlinenode.first_ln_real: tnode;
      begin
        expectloc:=LOC_FPUREGISTER;
        result:=nil;
      end;

    function tarminlinenode.first_cos_real: tnode;
      begin
        expectloc:=LOC_FPUREGISTER;
        result:=nil;
      end;


    function tarminlinenode.first_sin_real: tnode;
      begin
        expectloc:=LOC_FPUREGISTER;
        result:=nil;
      end;
    }


    procedure tarminlinenode.second_abs_real;
      var
        singleprec: boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pf: TOpPostfix;
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> origin/cpstrnew
      begin
<<<<<<< HEAD
        load_fpu_location(singleprec);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_ABS,location.register,left.location.register),get_fpu_postfix(resultdef)));
          fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fpu_vfpv3,
          fpu_vfpv3_d16:
            begin
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VABS,location.register,left.location.register),pf));
            end;
          fpu_fpv4_s16:
            current_asmdata.CurrAsmList.Concat(setoppostfix(taicpu.op_reg_reg(A_VABS,location.register,left.location.register), PF_F32));
          fpu_soft:
            begin
              if singleprec then
                cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,tcgint($7fffffff),location.register)
              else
                cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,tcgint($7fffffff),location.registerhi);
            end
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          fpu_vfpv3:
            begin
              if singleprec then
                op:=A_FABSS
              else
                op:=A_FABSD;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,location.register,left.location.register));
            end;
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
        else
          internalerror(2009111402);
        end;
=======
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_ABS,location.register,left.location.register),get_fpu_postfix(resultdef)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure tarminlinenode.second_sqr_real;
      var
        singleprec: boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pf: TOpPostfix;
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> origin/cpstrnew
      begin
        load_fpu_location(singleprec);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_MUF,location.register,left.location.register,left.location.register),get_fpu_postfix(resultdef)));
          fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fpu_vfpv3,
          fpu_vfpv3_d16:
            begin
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_VMUL,location.register,left.location.register,left.location.register),pf));
            end;
          fpu_fpv4_s16:
            current_asmdata.CurrAsmList.Concat(setoppostfix(taicpu.op_reg_reg_reg(A_VMUL,location.register,left.location.register,left.location.register), PF_F32));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          fpu_vfpv3:
            begin
              if singleprec then
                op:=A_FMULS
              else
                op:=A_FMULD;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,location.register,left.location.register,left.location.register));
            end;
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
        else
          internalerror(2009111403);
        end;
      end;


    procedure tarminlinenode.second_sqrt_real;
      var
        singleprec: boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pf: TOpPostfix;
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> graemeg/cpstrnew
=======
        op: TAsmOp;
>>>>>>> origin/cpstrnew
      begin
<<<<<<< HEAD
        load_fpu_location(singleprec);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_SQT,location.register,left.location.register),get_fpu_postfix(resultdef)));
          fpu_vfpv2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fpu_vfpv3,
          fpu_vfpv3_d16:
            begin
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VSQRT,location.register,left.location.register),pf));
            end;
          fpu_fpv4_s16:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VSQRT,location.register,left.location.register), PF_F32));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          fpu_vfpv3:
            begin
              if singleprec then
                op:=A_FSQRTS
              else
                op:=A_FSQRTD;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,location.register,left.location.register));
            end;
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
        else
          internalerror(2009111402);
        end;
=======
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_SQT,location.register,left.location.register),get_fpu_postfix(resultdef)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    { atn, sin, cos, lgn isn't supported by the linux fpe
    procedure tarminlinenode.second_arctan_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_ATN,location.register,left.location.register),get_fpu_postfix(resultdef)));
      end;


    procedure tarminlinenode.second_ln_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_LGN,location.register,left.location.register),get_fpu_postfix(resultdef)));
      end;

    procedure tarminlinenode.second_cos_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_COS,location.register,left.location.register),get_fpu_postfix(resultdef)));
      end;


    procedure tarminlinenode.second_sin_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_SIN,location.register,left.location.register),get_fpu_postfix(resultdef)));
      end;
    }

    procedure tarminlinenode.second_prefetch;
      var
        ref : treference;
        r : tregister;
      begin
        if not(GenerateThumbCode) and (CPUARM_HAS_EDSP in cpu_capabilities[current_settings.cputype]) then
          begin
            secondpass(left);
            case left.location.loc of
              LOC_CREFERENCE,
              LOC_REFERENCE:
                begin
                  r:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
                  cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.location.reference,r);
                  reference_reset_base(ref,r,0,left.location.reference.alignment);
                  { since the address might be nil we can't use ldr for older cpus }
                  current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_PLD,ref));
                end;
              else
                { nothing to prefetch };
            end;
          end;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
    procedure tarminlinenode.second_abs_long;
      var
        opsize : tcgsize;
      begin
        if GenerateThumbCode then
          begin
            inherited second_abs_long;
            exit;
          end;

        secondpass(left);
        opsize:=def_cgsize(left.resultdef);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
        location:=left.location;
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);

        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_MOV,location.register,left.location.register), PF_S));

        if GenerateThumb2Code then
          current_asmdata.CurrAsmList.concat(taicpu.op_cond(A_IT,C_MI));

        current_asmdata.CurrAsmList.concat(setcondition(taicpu.op_reg_reg_const(A_RSB,location.register,location.register, 0), C_MI));

        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
      end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  cinlinenode:=tarminlinenode;
end.
