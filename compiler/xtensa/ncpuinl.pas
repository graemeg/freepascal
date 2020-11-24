{
    Copyright (c) 1998-2017 by Florian Klaempfl

    Generates Xtensa inline nodes

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
      node,ninl,ncginl, aasmbase;

    type
      tcpuinlineNode = class(tcginlinenode)
        function first_abs_real: tnode; override;
        procedure second_abs_long; override;
        procedure second_abs_real; override;
        function first_fma: tnode; override;
        procedure second_fma; override;
      end;

  implementation

    uses
      cpuinfo,
      verbose,globals,
      compinnr,
      aasmdata,
      aasmcpu,
      symdef,
      defutil,
      hlcgobj,
      pass_2,
      cgbase, cgobj, cgutils,
      ncal,
      cpubase;

    procedure tcpuinlinenode.second_abs_long;
      begin
        secondpass(left);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

        location:=left.location;
        location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,left.resultdef);

        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_ABS,location.register,left.location.register));
      end;


    function tcpuinlinenode.first_abs_real : tnode;
      begin
        result:=nil;
        if is_single(left.resultdef) and (FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) then
          expectloc:=LOC_FPUREGISTER
        else
          result:=inherited first_abs_real;
      end;


    procedure tcpuinlinenode.second_abs_real;
      begin
        if not(is_single(resultdef)) then
          InternalError(2020091101);
        secondpass(left);
        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_reset(location,LOC_FPUREGISTER,OS_F32);
        location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
        current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_ABS,location.register,left.location.register),PF_S));
      end;


    function tcpuinlinenode.first_fma : tnode;
      begin
        if is_single(resultdef) then
          begin
            expectloc:=LOC_FPUREGISTER;
            Result:=nil;
          end
        else
          Result:=inherited first_fma;
      end;


    procedure tcpuinlinenode.second_fma;
      const
        op : array[false..true] of TAsmOp =
          (A_MADD,
           A_MSUB);

      var
        paraarray : array[1..3] of tnode;
        i : integer;
        negproduct : boolean;
        oppostfix : TOpPostfix;
        ai: taicpu;
      begin
         if is_single(resultdef)and
           (FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) then
           begin
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
              for i:=1 to 3 do
               secondpass(paraarray[i]);

             { no memory operand is allowed }
             for i:=1 to 3 do
               begin
                 if not(paraarray[i].location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER]) then
                   hlcg.location_force_fpureg(current_asmdata.CurrAsmList,paraarray[i].location,paraarray[i].resultdef,true);
               end;

             location_reset(location,LOC_FPUREGISTER,paraarray[1].location.size);
             location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);

             hlcg.a_loadfpu_reg_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,resultdef,
               paraarray[3].location.register,location.register);

             ai:=taicpu.op_reg_reg_reg(op[negproduct],
               location.register,paraarray[1].location.register,paraarray[2].location.register);
             ai.oppostfix:=PF_S;
             current_asmdata.CurrAsmList.concat(ai);

             cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
           end
         else
           internalerror(2020112401);
      end;


begin
  cinlinenode:=tcpuinlinenode;
end.
