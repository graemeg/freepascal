{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate Risc-V32 assembler for math nodes

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
unit nrv32mat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat, ncgmat,
      cgbase;

    type
      trv32moddivnode = class(tcgmoddivnode)
         procedure emit_div_reg_reg(signed: boolean; denum, num: tregister); override;
         procedure emit_mod_reg_reg(signed: boolean; denum, num: tregister); override;
         function first_moddivint: tnode; override;
      end;

      trv32shlshrnode = class(tcgshlshrnode)
      end;

      trv32unaryminusnode = class(tcgunaryminusnode)
      end;

      trv32notnode = class(tcgnotnode)
        procedure second_boolean; override;
      end;

implementation

    uses
      globtype,systems,constexp,
      cutils,verbose,globals,
      symconst,symdef,
      aasmbase,aasmcpu,aasmtai,aasmdata,
      defutil,
      cgutils,cgobj,hlcgobj,pass_2,
      cpubase,cpuinfo,
      ncon,procinfo,
      ncgutil,cgcpu;

    procedure trv32notnode.second_boolean;
      var
        tlabel, flabel: tasmlabel;
      begin
        if not handle_locjump then
          begin
            secondpass(left);
            case left.location.loc of
              LOC_FLAGS :
                begin
                  Internalerror(2016060601);
                  //location_copy(location,left.location);
                  //inverse_flags(location.resflags);
                end;
              LOC_REGISTER, LOC_CREGISTER,
              LOC_REFERENCE, LOC_CREFERENCE,
              LOC_SUBSETREG, LOC_CSUBSETREG,
              LOC_SUBSETREF, LOC_CSUBSETREF:
                begin
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

                  location_reset(location,LOC_REGISTER,OS_INT);
                  location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,s32inttype);

                  current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_const(A_SLTIU,location.register,left.location.register,1));
               end;
              else
                internalerror(2003042401);
            end;
          end;
      end;

    procedure trv32moddivnode.emit_div_reg_reg(signed: boolean; denum, num: tregister);
      var
        op: TAsmOp;
      begin
        if signed then
          op:=A_DIV
        else
          op:=A_DIVU;

        current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(op,denum,num,denum));
      end;

    procedure trv32moddivnode.emit_mod_reg_reg(signed: boolean; denum, num: tregister);
      var
        op: TAsmOp;
      begin
        if signed then
          op:=A_REM
        else
          op:=A_REMU;

        current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(op,denum,num,denum));
      end;


    function trv32moddivnode.first_moddivint: tnode;
      begin
        if (not is_64bitint(resultdef)) and
           (CPURV_HAS_MUL in cpu_capabilities[current_settings.cputype]) then
          Result:=nil
        else
          result:=inherited;
      end;

begin
   cmoddivnode:=trv32moddivnode;
   cshlshrnode:=trv32shlshrnode;
   cunaryminusnode:=trv32unaryminusnode;
   cnotnode:=trv32notnode;
end.
