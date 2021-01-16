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
      node,nmat,nx86mat;

    type
      tx8664shlshrnode = class(tx86shlshrnode)
         procedure pass_generate_code;override;
      end;

      tx8664unaryminusnode = class(tx86unaryminusnode)
      end;

      tx8664notnode = class(tx86notnode)
      end;

implementation

    uses
      globtype,constexp,
      cutils,
      aasmdata,defutil,
      pass_2,
      ncon,
      cgbase,cgutils,cgobj,hlcgobj;


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

        opsize:=def_cgsize(resultdef);
        mask:=max(resultdef.size,4)*8-1;

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
