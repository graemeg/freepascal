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
unit ncpuinl;

{$i fpcdefs.inc}

interface

    uses
      node,ninl,ncginl;

    type
      taarch64inlinenode = class(tcgInlineNode)
        function first_abs_real: tnode; override;
        function first_sqr_real: tnode; override;
        function first_sqrt_real: tnode; override;
        function first_round_real: tnode; override;
        function first_trunc_real: tnode; override;
        function first_fma : tnode; override;
        procedure second_abs_real; override;
        procedure second_sqr_real; override;
        procedure second_sqrt_real; override;
        procedure second_abs_long; override;
        procedure second_round_real; override;
        procedure second_trunc_real; override;
        procedure second_get_frame; override;
        procedure second_fma; override;
        procedure second_prefetch; override;
      private
        procedure load_fpu_location;
      end;


implementation

    uses
      globtype,verbose,globals,
      cpuinfo, defutil,symdef,aasmdata,aasmcpu,
      cgbase,cgutils,pass_1,pass_2,
      ncal,nutils,
      cpubase,ncgutil,cgobj,cgcpu, hlcgobj;

{*****************************************************************************
                              taarch64inlinenode
*****************************************************************************}

    procedure taarch64inlinenode.load_fpu_location;
      begin
        secondpass(left);
        hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_copy(location,left.location);
        location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
        location.loc:=LOC_MMREGISTER;
      end;


    function taarch64inlinenode.first_abs_real : tnode;
      begin
        expectloc:=LOC_MMREGISTER;
        result:=nil;
      end;


    function taarch64inlinenode.first_sqr_real : tnode;
      begin
        expectloc:=LOC_MMREGISTER;
        result:=nil;
      end;


    function taarch64inlinenode.first_sqrt_real : tnode;
      begin
        expectloc:=LOC_MMREGISTER;
        result:=nil;
      end;


    function taarch64inlinenode.first_round_real: tnode;
      begin
        expectloc:=LOC_MMREGISTER;
        result:=nil;
      end;


    function taarch64inlinenode.first_trunc_real: tnode;
      begin
        expectloc:=LOC_MMREGISTER;
        result:=nil;
      end;


     function taarch64inlinenode.first_fma : tnode;
       begin
         if ((is_double(resultdef)) or (is_single(resultdef))) then
           begin
             expectloc:=LOC_MMREGISTER;
             Result:=nil;
           end
         else
           Result:=inherited first_fma;
       end;

    procedure taarch64inlinenode.second_abs_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FABS,location.register,left.location.register));
        cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
      end;


    procedure taarch64inlinenode.second_sqr_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_FMUL,location.register,left.location.register,left.location.register));
        cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
      end;


    procedure taarch64inlinenode.second_sqrt_real;
      begin
        load_fpu_location;
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FSQRT,location.register,left.location.register));
        cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
      end;


    procedure taarch64inlinenode.second_abs_long;
      var
        opsize : tcgsize;
      begin
        secondpass(left);
        opsize:=def_cgsize(left.resultdef);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
        location:=left.location;
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);

        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_NEG,location.register,left.location.register),PF_S));
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_cond(A_CSEL,location.register,location.register,left.location.register,C_GE));
      end;


    procedure taarch64inlinenode.second_round_real;
      var
        hreg: tregister;
      begin
        secondpass(left);
        hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
        hreg:=cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
        { round as floating point using current rounding mode }
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FRINTX,hreg,left.location.register));
        { convert to signed integer rounding towards zero (there's no "round to
          integer using current rounding mode") }
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FCVTZS,location.register,hreg));
        cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
      end;


    procedure taarch64inlinenode.second_trunc_real;
      begin
        secondpass(left);
        hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
        { convert to signed integer rounding towards zero }
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FCVTZS,location.register,left.location.register));
      end;


    procedure taarch64inlinenode.second_get_frame;
      begin
        location_reset(location,LOC_CREGISTER,OS_ADDR);
        { this routine is used to get the frame pointer for backtracing
          purposes. current_procinfo.framepointer is set to SP because that one
          is used to access temps. On most platforms these two frame pointers
          are the same, but not on AArch64. }
        location.register:=NR_FRAME_POINTER_REG;
      end;


    procedure taarch64inlinenode.second_fma;
      const
        op : array[false..true,false..true] of TAsmOp =
          { positive product }
          (
           { positive third operand }
           (A_FMADD,
           { negative third operand }
            A_FNMSUB),
           { negative product }
            { positive third operand }
            (A_FMSUB,
             A_FNMADD)
           );

      var
        paraarray : array[1..3] of tnode;
        i : integer;
        negop3,
        negproduct : boolean;
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

        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_reg(op[negproduct,negop3],
          location.register,paraarray[1].location.register,paraarray[2].location.register,paraarray[3].location.register));
        cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
      end;


    procedure taarch64inlinenode.second_prefetch;
      var
        ref : treference;
        r : tregister;
        checkpointer_used : boolean;
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
             current_asmdata.CurrAsmList.concat(taicpu.op_const_ref(A_PRFM,0,ref));
           end;
         else
           { nothing to prefetch };
       end;
      end;


begin
  cinlinenode:=taarch64inlinenode;
end.
