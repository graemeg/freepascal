{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate i386 assembler for math nodes

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
unit n386mat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat,nx86mat;

    type
      ti386moddivnode = class(tx86moddivnode)
         procedure pass_generate_code;override;
      end;

      ti386shlshrnode = class(tcgshlshrnode)
         procedure second_64bit;override;
         function first_shlshr64bitint: tnode; override;
      end;

      ti386unaryminusnode = class(tx86unaryminusnode)
      end;

      ti386notnode = class(tx86notnode)
      end;


implementation

    uses
      globtype,systems,constexp,
      cutils,verbose,globals,
      symconst,symdef,aasmbase,aasmtai,aasmdata,defutil,
      cgbase,pass_2,
      ncon,
      cpubase,cpuinfo,
      cga,ncgutil,cgobj,cgutils,
      hlcgobj;

{*****************************************************************************
                             TI386MODDIVNODE
*****************************************************************************}



   procedure ti386moddivnode.pass_generate_code;
      var
        hreg1:Tregister;
        power:longint;
        hl:Tasmlabel;
      begin
        if is_64bitint(resultdef) then
          { should be handled in pass_1 (JM) }
          internalerror(200109052);
<<<<<<< HEAD
=======
        { put numerator in register }
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location_force_reg(current_asmdata.CurrAsmList,left.location,location.size,false);
        hreg1:=left.location.register;
>>>>>>> graemeg/fixes_2_2

        if (nodetype=divn) and (right.nodetype=ordconstn) and
          is_signed(left.resultdef) and
          ispowerof2(tordconstnode(right).value.svalue,power) and
          ((current_settings.optimizecputype = cpu_386) or
           (cs_opt_size in current_settings.optimizerswitches)) then
          begin
            { signed divide-by-power-of-two optimized for size }
            secondpass(left);
            if codegenerror then
              exit;
            secondpass(right);
            if codegenerror then
              exit;
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
            hreg1:=left.location.register;
            emit_reg_reg(A_TEST,S_L,hreg1,hreg1);
            current_asmdata.getjumplabel(hl);
            cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NS,hl);
            if power=1 then
              emit_reg(A_INC,S_L,hreg1)
            else
              emit_const_reg(A_ADD,S_L,tordconstnode(right).value.svalue-1,hreg1);
            cg.a_label(current_asmdata.CurrAsmList,hl);
            emit_const_reg(A_SAR,S_L,power,hreg1);
            location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
            location.register:=hreg1;
          end
        else
          inherited pass_generate_code;
      end;


{*****************************************************************************
                             TI386SHLRSHRNODE
*****************************************************************************}


    function ti386shlshrnode.first_shlshr64bitint: tnode;
      begin
        result := nil;
      end;

    procedure ti386shlshrnode.second_64bit;
      var
        hreg64hi,hreg64lo:Tregister;
        v : TConstExprInt;
        l1,l2,l3:Tasmlabel;
      begin
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));

        { load left operator in a register }
<<<<<<< HEAD
<<<<<<< HEAD
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
=======
        location_force_reg(current_asmdata.CurrAsmList,left.location,location.size,false);
>>>>>>> graemeg/fixes_2_2
=======
        location_force_reg(current_asmdata.CurrAsmList,left.location,location.size,false);
>>>>>>> origin/fixes_2_2
        hreg64hi:=left.location.register64.reghi;
        hreg64lo:=left.location.register64.reglo;

        { shifting by a constant directly coded: }
        if (right.nodetype=ordconstn) then
          begin
            v:=Tordconstnode(right).value and 63;
            if v>31 then
              begin
                if nodetype=shln then
                  begin
                    emit_reg_reg(A_XOR,S_L,hreg64hi,hreg64hi);
                    if ((v and 31) <> 0) then
                      emit_const_reg(A_SHL,S_L,v.svalue and 31,hreg64lo);
                  end
                else
                  begin
                    emit_reg_reg(A_XOR,S_L,hreg64lo,hreg64lo);
                    if ((v and 31) <> 0) then
                      emit_const_reg(A_SHR,S_L,v.svalue and 31,hreg64hi);
                  end;
                location.register64.reghi:=hreg64lo;
                location.register64.reglo:=hreg64hi;
              end
            else
              begin
                if nodetype=shln then
                  begin
                    emit_const_reg_reg(A_SHLD,S_L,v.svalue and 31,hreg64lo,hreg64hi);
                    emit_const_reg(A_SHL,S_L,v.svalue and 31,hreg64lo);
                  end
                else
                  begin
                    emit_const_reg_reg(A_SHRD,S_L,v.svalue and 31,hreg64hi,hreg64lo);
                    emit_const_reg(A_SHR,S_L,v.svalue and 31,hreg64hi);
                  end;
                location.register64.reglo:=hreg64lo;
                location.register64.reghi:=hreg64hi;
              end;
          end
        else
          begin
            { load right operators in a register }
            cg.getcpuregister(current_asmdata.CurrAsmList,NR_ECX);
            hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,u32inttype,right.location,NR_ECX);

            { left operator is already in a register }
            { hence are both in a register }
            { is it in the case ECX ? }

            { the damned shift instructions work only til a count of 32 }
            { so we've to do some tricks here                           }
            current_asmdata.getjumplabel(l1);
            current_asmdata.getjumplabel(l2);
            current_asmdata.getjumplabel(l3);
            emit_const_reg(A_CMP,S_L,64,NR_ECX);
            cg.a_jmp_flags(current_asmdata.CurrAsmList,F_L,l1);
            emit_reg_reg(A_XOR,S_L,hreg64lo,hreg64lo);
            emit_reg_reg(A_XOR,S_L,hreg64hi,hreg64hi);
            cg.a_jmp_always(current_asmdata.CurrAsmList,l3);
            cg.a_label(current_asmdata.CurrAsmList,l1);
            emit_const_reg(A_CMP,S_L,32,NR_ECX);
            cg.a_jmp_flags(current_asmdata.CurrAsmList,F_L,l2);
            emit_const_reg(A_SUB,S_L,32,NR_ECX);
            if nodetype=shln then
              begin
                emit_reg_reg(A_SHL,S_L,NR_CL,hreg64lo);
                emit_reg_reg(A_MOV,S_L,hreg64lo,hreg64hi);
                emit_reg_reg(A_XOR,S_L,hreg64lo,hreg64lo);
                cg.a_jmp_always(current_asmdata.CurrAsmList,l3);
                cg.a_label(current_asmdata.CurrAsmList,l2);
                emit_reg_reg_reg(A_SHLD,S_L,NR_CL,hreg64lo,hreg64hi);
                emit_reg_reg(A_SHL,S_L,NR_CL,hreg64lo);
              end
            else
              begin
                emit_reg_reg(A_SHR,S_L,NR_CL,hreg64hi);
                emit_reg_reg(A_MOV,S_L,hreg64hi,hreg64lo);
                emit_reg_reg(A_XOR,S_L,hreg64hi,hreg64hi);
                cg.a_jmp_always(current_asmdata.CurrAsmList,l3);
                cg.a_label(current_asmdata.CurrAsmList,l2);
                emit_reg_reg_reg(A_SHRD,S_L,NR_CL,hreg64hi,hreg64lo);
                emit_reg_reg(A_SHR,S_L,NR_CL,hreg64hi);
              end;
            cg.a_label(current_asmdata.CurrAsmList,l3);

            cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_ECX);
            location.register64.reglo:=hreg64lo;
            location.register64.reghi:=hreg64hi;
          end;
      end;


begin
   cunaryminusnode:=ti386unaryminusnode;
   cmoddivnode:=ti386moddivnode;
   cshlshrnode:=ti386shlshrnode;
   cnotnode:=ti386notnode;
end.
