{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate x86-64 assembler for math nodes

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
unit nx64mat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat,nx86mat;

    type
      tx8664shlshrnode = class(tshlshrnode)
         procedure pass_generate_code;override;
      end;

      tx8664unaryminusnode = class(tx86unaryminusnode)
      end;

      tx8664notnode = class(tx86notnode)
      end;

implementation

    uses
      globtype,systems,constexp,
      cutils,verbose,globals,
      symconst,symdef,aasmbase,aasmtai,aasmdata,defutil,
      pass_1,pass_2,
      ncon,
      cpubase,cpuinfo,
      cgbase,cgutils,cga,cgobj,hlcgobj,cgx86,
      ncgutil;

<<<<<<< HEAD
=======
{*****************************************************************************
                             TX8664MODDIVNODE
*****************************************************************************}

    procedure tx8664moddivnode.pass_generate_code;
      var
        hreg1,hreg2:Tregister;
        power:longint;
        op:Tasmop;
      begin
        secondpass(left);
        if codegenerror then
          exit;
        secondpass(right);
        if codegenerror then
          exit;

        { put numerator in register }
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location_force_reg(current_asmdata.CurrAsmList,left.location,location.size,false);
        hreg1:=left.location.register;

        if (nodetype=divn) and (right.nodetype=ordconstn) and
           ispowerof2(int64(tordconstnode(right).value),power) then
          begin
            { for signed numbers, the numerator must be adjusted before the
              shift instruction, but not wih unsigned numbers! Otherwise,
              "Cardinal($ffffffff) div 16" overflows! (JM) }
            if is_signed(left.resultdef) Then
              begin
                  { use a sequence without jumps, saw this in
                    comp.compilers (JM) }
                  { no jumps, but more operations }
                  hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                  emit_reg_reg(A_MOV,S_Q,hreg1,hreg2);
                  {If the left value is signed, hreg2=$ffffffff, otherwise 0.}
                  emit_const_reg(A_SAR,S_Q,63,hreg2);
                  {If signed, hreg2=right value-1, otherwise 0.}
                  { (don't use emit_const_reg, because if value>high(longint)
                     then it must first be loaded into a register) }
                  cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,OS_S64,tordconstnode(right).value-1,hreg2);
                  { add to the left value }
                  emit_reg_reg(A_ADD,S_Q,hreg2,hreg1);
                  { do the shift }
                  emit_const_reg(A_SAR,S_Q,power,hreg1);
              end
            else
              emit_const_reg(A_SHR,S_Q,power,hreg1);
            location.register:=hreg1;
          end
        else
          begin
            {Bring denominator to a register.}
            cg.getcpuregister(current_asmdata.CurrAsmList,NR_RAX);
            emit_reg_reg(A_MOV,S_Q,hreg1,NR_RAX);
            cg.getcpuregister(current_asmdata.CurrAsmList,NR_RDX);
            {Sign extension depends on the left type.}
            if torddef(left.resultdef).ordtype=u64bit then
              emit_reg_reg(A_XOR,S_Q,NR_RDX,NR_RDX)
            else
              emit_none(A_CQO,S_NO);

            {Division depends on the right type.}
            if Torddef(right.resultdef).ordtype=u64bit then
              op:=A_DIV
            else
              op:=A_IDIV;

            if right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
              emit_ref(op,S_Q,right.location.reference)
            else if right.location.loc in [LOC_REGISTER,LOC_CREGISTER] then
              emit_reg(op,S_Q,right.location.register)
            else
              begin
                hreg1:=cg.getintregister(current_asmdata.CurrAsmList,right.location.size);
                cg.a_load_loc_reg(current_asmdata.CurrAsmList,OS_64,right.location,hreg1);
                emit_reg(op,S_Q,hreg1);
              end;

            { Copy the result into a new register. Release RAX & RDX.}
            cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_RDX);
            cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_RAX);
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            if nodetype=divn then
              cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,NR_RAX,location.register)
            else
              cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,NR_RDX,location.register);
          end;
      end;

>>>>>>> graemeg/cpstrnew

{*****************************************************************************
                             TX8664SHLRSHRNODE
*****************************************************************************}


    procedure tx8664shlshrnode.pass_generate_code;
      var
        op : topcg;
        opsize : tcgsize;
        mask : aint;
      begin
        secondpass(left);
        secondpass(right);

        { determine operator }
        if nodetype=shln then
          op:=OP_SHL
        else
          op:=OP_SHR;

        { special treatment of 32bit values for backwards compatibility }
        { mul optimizations require to keep the sign (FK) }
        if left.resultdef.size<=4 then
          begin
            if is_signed(left.resultdef) then
              opsize:=OS_S32
            else
              opsize:=OS_32;
            mask:=31;
          end
        else
          begin
            if is_signed(left.resultdef) then
              opsize:=OS_S64
            else
              opsize:=OS_64;
            mask:=63;
          end;

        { load left operators in a register }
        if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
          { location_force_reg can be also used to change the size of a register }
          (left.location.size<>opsize) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(opsize),true);
        location_reset(location,LOC_REGISTER,opsize);
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);

        { shifting by a constant directly coded: }
        if (right.nodetype=ordconstn) then
          cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,op,location.size,
            tordconstnode(right).value.uvalue and mask,left.location.register,location.register)
        else
          begin
            { load right operators in a register - this
              is done since most target cpu which will use this
              node do not support a shift count in a mem. location (cec)
            }
            if not(right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
               { location_force_reg can be also used to change the size of a register }
              (right.location.size<>opsize) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cgsize_orddef(opsize),true);

            cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,op,opsize,right.location.register,left.location.register,location.register);
          end;
      end;


begin
   cunaryminusnode:=tx8664unaryminusnode;
   cmoddivnode:=tx86moddivnode;
   cshlshrnode:=tx8664shlshrnode;
   cnotnode:=tx8664notnode;
end.
