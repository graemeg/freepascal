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
        function first_fma : tnode; override;
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
        procedure second_fma; override;
      private
        procedure load_fpu_location(out singleprec: boolean);
      end;


implementation

    uses
      globtype,verbose,globals,
      cpuinfo, defutil,symdef,aasmdata,aasmcpu,
      cgbase,cgutils,pass_1,pass_2,
      cpubase,ncgutil,cgobj,cgcpu, hlcgobj,
      nutils,ncal;

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
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              location_copy(location,left.location);
              if left.location.loc=LOC_CFPUREGISTER then
                begin
                 location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
                 location.loc := LOC_FPUREGISTER;
               end;
            end;
          fpu_soft:
            begin
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
              location_copy(location,left.location);
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              location_copy(location,left.location);
              if left.location.loc=LOC_CMMREGISTER then
                begin
                 location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
                 location.loc := LOC_MMREGISTER;
               end;
            end
          else
            internalerror(2009111801);
        end;
        singleprec:=tfloatdef(left.resultdef).floattype=s32real;
      end;


    function tarminlinenode.first_abs_real : tnode;
      begin
        if (cs_fp_emulation in current_settings.moduleswitches) then
          begin
            firstpass(left);
            expectloc:=LOC_REGISTER;
            first_abs_real:=nil;
          end
        else
          begin
            case current_settings.fputype of
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                expectloc:=LOC_FPUREGISTER;
              else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
                expectloc:=LOC_MMREGISTER
              else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
                begin
                  if tfloatdef(left.resultdef).floattype=s32real then
                    expectloc:=LOC_MMREGISTER
                  else
                    exit(inherited first_abs_real);
                end
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
              else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
                expectloc:=LOC_MMREGISTER
              else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
                begin
                  if tfloatdef(left.resultdef).floattype=s32real then
                    expectloc:=LOC_MMREGISTER
                  else
                    exit(inherited first_sqr_real);
                end
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
              else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
                expectloc:=LOC_MMREGISTER
              else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
                begin
                  if tfloatdef(left.resultdef).floattype=s32real then
                    expectloc:=LOC_MMREGISTER
                  else
                    exit(inherited first_sqrt_real);
                end
              else
                internalerror(2009112403);
            end;
            first_sqrt_real := nil;
          end;
      end;


     function tarminlinenode.first_fma : tnode;
       begin
         if (true) and
           ((is_double(resultdef)) or (is_single(resultdef))) then
           begin
             expectloc:=LOC_MMREGISTER;
             Result:=nil;
           end
         else
           Result:=inherited first_fma;
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
        pf: TOpPostfix;
      begin
        load_fpu_location(singleprec);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_ABS,location.register,left.location.register),get_fpu_postfix(resultdef)));
          fpu_soft:
            begin
              if singleprec then
                cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,tcgint($7fffffff),location.register)
              else
                cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,tcgint($7fffffff),location.registerhi);
            end
          else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
            begin
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VABS,location.register,left.location.register),pf));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
            begin
              current_asmdata.CurrAsmList.Concat(setoppostfix(taicpu.op_reg_reg(A_VABS,location.register,left.location.register), PF_F32));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else
            internalerror(2009111402);
        end;
      end;


    procedure tarminlinenode.second_sqr_real;
      var
        singleprec: boolean;
        pf: TOpPostfix;
      begin
        load_fpu_location(singleprec);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_MUF,location.register,left.location.register,left.location.register),get_fpu_postfix(resultdef)));
          else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
            begin
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_VMUL,location.register,left.location.register,left.location.register),pf));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
            begin
              current_asmdata.CurrAsmList.Concat(setoppostfix(taicpu.op_reg_reg_reg(A_VMUL,location.register,left.location.register,left.location.register), PF_F32));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else
            internalerror(2009111403);
        end;
      end;


    procedure tarminlinenode.second_sqrt_real;
      var
        singleprec: boolean;
        pf: TOpPostfix;
      begin
        load_fpu_location(singleprec);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_SQT,location.register,left.location.register),get_fpu_postfix(resultdef)));
          else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
            begin
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VSQRT,location.register,left.location.register),pf));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
            begin
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VSQRT,location.register,left.location.register), PF_F32));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else
            internalerror(2009111402);
        end;
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
        checkpointer_used : boolean;
      begin
        if not(GenerateThumbCode) and (CPUARM_HAS_EDSP in cpu_capabilities[current_settings.cputype]) then
          begin
             { do not call Checkpointer for left node }
             checkpointer_used:=(cs_checkpointer in current_settings.localswitches);
             if checkpointer_used then
               node_change_local_switch(left,cs_checkpointer,false);
             secondpass(left);
             if checkpointer_used then
               node_change_local_switch(left,cs_checkpointer,false);
            case left.location.loc of
              LOC_CREFERENCE,
              LOC_REFERENCE:
                begin
                  r:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
                  cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.location.reference,r);
                  reference_reset_base(ref,r,0,location.reference.temppos,left.location.reference.alignment,location.reference.volatility);
                  { since the address might be nil we can't use ldr for older cpus }
                  current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_PLD,ref));
                end;
              else
                { nothing to prefetch };
            end;
          end;
      end;

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


    procedure tarminlinenode.second_fma;
      const
        op : array[false..true,false..true] of TAsmOp =
          { positive product }
          (
           { positive third operand }
           (A_VFMA,
           { negative third operand }
            A_VFNMS),
           { negative product }
            { positive third operand }
            (A_VFMS,
             A_VFNMA)
           );

      var
        paraarray : array[1..3] of tnode;
        i : integer;
        negop3,
        negproduct : boolean;
        oppostfix : TOpPostfix;
      begin
         if FPUARM_HAS_FMA in fpu_capabilities[current_settings.fputype] then
           begin
             negop3:=false;
             negproduct:=false;
             paraarray[1]:=tcallparanode(tcallparanode(tcallparanode(parameters).nextpara).nextpara).paravalue;
             paraarray[2]:=tcallparanode(tcallparanode(parameters).nextpara).paravalue;
             paraarray[3]:=tcallparanode(parameters).paravalue;

             { check if a neg. node can be removed
               this is possible because changing the sign of
               a floating point number does not affect its absolute
               value in any way
             }
             if paraarray[1].nodetype=unaryminusn then
               begin
                 paraarray[1]:=tunarynode(paraarray[1]).left;
                 { do not release the unused unary minus node, it is kept and release together with the other nodes,
                   only no code is generated for it }
                 negproduct:=not(negproduct);
               end;

             if paraarray[2].nodetype=unaryminusn then
               begin
                 paraarray[2]:=tunarynode(paraarray[2]).left;
                 { do not release the unused unary minus node, it is kept and release together with the other nodes,
                   only no code is generated for it }
                 negproduct:=not(negproduct);
               end;

             if paraarray[3].nodetype=unaryminusn then
               begin
                 paraarray[3]:=tunarynode(paraarray[3]).left;
                 { do not release the unused unary minus node, it is kept and release together with the other nodes,
                   only no code is generated for it }
                 negop3:=true;
               end;

              for i:=1 to 3 do
               secondpass(paraarray[i]);

             { no memory operand is allowed }
             for i:=1 to 3 do
               begin
                 if not(paraarray[i].location.loc in [LOC_MMREGISTER,LOC_CMMREGISTER]) then
                   hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,paraarray[i].location,paraarray[i].resultdef,true);
               end;

             location_reset(location,LOC_MMREGISTER,paraarray[1].location.size);
             location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

             hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,resultdef,
               paraarray[3].location.register,location.register,mms_movescalar);
             if is_double(resultdef) then
               oppostfix:=PF_F64
             else
               oppostfix:=PF_F32;
             current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op[negproduct,negop3],
               location.register,paraarray[1].location.register,paraarray[2].location.register),oppostfix));
             cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
           end
         else
           internalerror(2014032301);
      end;


begin
  cinlinenode:=tarminlinenode;
end.
