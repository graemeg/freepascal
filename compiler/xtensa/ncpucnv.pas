{
    Copyright (c) 1998-2019 by Florian Klaempfl

    Generate Xtensa assembler for type converting nodes

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
unit ncpucnv;

{$i fpcdefs.inc}

interface

    uses
      node,ncnv,ncgcnv;

    type
       tcputypeconvnode = class(tcgtypeconvnode)
       protected
         function first_real_to_real: tnode;override;
         procedure second_int_to_bool;override;
       end;

implementation

   uses
      verbose,globtype,globals,symdef,aasmbase,aasmtai,aasmdata,symtable,
      defutil,
      cgbase,cgutils,
      pass_1,pass_2,procinfo,ncal,
      ncgutil,
      cpubase,cpuinfo,aasmcpu,cgobj,hlcgobj,cgcpu;


{*****************************************************************************
                             tcputypeconvnode
*****************************************************************************}

    function tcputypeconvnode.first_real_to_real: tnode;
      begin
        if (FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) and
          not(FPUXTENSA_DOUBLE in fpu_capabilities[current_settings.fputype]) then
          begin
            case tfloatdef(left.resultdef).floattype of
              s32real:
                case tfloatdef(resultdef).floattype of
                  s64real:
                    result:=ctypeconvnode.create_explicit(ccallnode.createintern('float32_to_float64',ccallparanode.create(
                      ctypeconvnode.create_internal(left,search_system_type('FLOAT32REC').typedef),nil)),resultdef);
                  s32real:
                    begin
                      result:=left;
                      left:=nil;
                    end;
                  else
                    internalerror(200610151);
                end;
              s64real:
                case tfloatdef(resultdef).floattype of
                  s32real:
                    result:=ctypeconvnode.create_explicit(ccallnode.createintern('float64_to_float32',ccallparanode.create(
                      ctypeconvnode.create_internal(left,search_system_type('FLOAT64').typedef),nil)),resultdef);
                  s64real:
                    begin
                      result:=left;
                      left:=nil;
                    end;
                  else
                    internalerror(200610152);
                end;
              else
                internalerror(200610153);
            end;
            left:=nil;
            firstpass(result);
            exit;
          end
        else
          Result := inherited first_real_to_real;
      end;



    procedure tcputypeconvnode.second_int_to_bool;
      var
        hreg1, onereg: tregister;
        href      : treference;
        hlabel    : tasmlabel;
        newsize   : tcgsize;
      begin
        secondpass(left);
        if codegenerror then
          exit;

        { Explicit typecasts from any ordinal type to a boolean type
          must not change the ordinal value                          }
        if (nf_explicit in flags) and
           not(left.location.loc in [LOC_JUMP]) then
          begin
             location_copy(location,left.location);
             newsize:=def_cgsize(resultdef);
             { change of size? change sign only if location is LOC_(C)REGISTER? Then we have to sign/zero-extend }
             if (tcgsize2size[newsize]<>tcgsize2size[left.location.size]) or
                ((newsize<>left.location.size) and (location.loc in [LOC_REGISTER,LOC_CREGISTER])) then
               hlcg.location_force_reg(current_asmdata.CurrAsmList,location,left.resultdef,resultdef,true)
             else
               location.size:=newsize;
             exit;
          end;

        if (left.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));

        onereg:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
        cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,1,onereg);
        hreg1:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
        case left.location.loc of
          LOC_CREFERENCE,
          LOC_REFERENCE :
            begin
              if left.location.size in [OS_64,OS_S64] then
                begin
                  cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_32,OS_32,left.location.reference,hreg1);
                  href:=left.location.reference;
                  inc(href.offset,4);
                  cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,href,hreg1);
                end
              else
                cg.a_load_ref_reg(current_asmdata.CurrAsmList,left.location.size,OS_32,left.location.reference,hreg1);
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(A_MOVNEZ,hreg1,onereg,hreg1));
            end;
          LOC_REGISTER,LOC_CREGISTER :
            begin
              if left.location.size in [OS_64,OS_S64] then
                cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.register64.reglo,left.location.register64.reghi,hreg1)
              else
                cg.a_load_reg_reg(current_asmdata.CurrAsmList,left.location.size,OS_32,left.location.register,hreg1);
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg_reg(A_MOVNEZ,hreg1,onereg,hreg1));
            end;
          LOC_JUMP :
            begin
              current_asmdata.getjumplabel(hlabel);
              cg.a_label(current_asmdata.CurrAsmList,left.location.truelabel);
              cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,1,hreg1);
              cg.a_jmp_always(current_asmdata.CurrAsmList,hlabel);
              cg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
              cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,0,hreg1);
              cg.a_label(current_asmdata.CurrAsmList,hlabel);
            end;
          else
            internalerror(2020031504);
        end;
        if (is_cbool(resultdef)) then
          cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,location.size,hreg1,hreg1);

{$ifndef cpu64bitalu}
        if (location.size in [OS_64,OS_S64]) then
          begin
            location.register64.reglo:=hreg1;
            location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            if (is_cbool(resultdef)) then
              { reglo is either 0 or -1 -> reghi has to become the same }
              cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_32,OS_32,location.register64.reglo,location.register64.reghi)
            else
              { unsigned }
              cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,location.register64.reghi);
          end
        else
{$endif cpu64bitalu}
          location.register:=hreg1;
      end;


begin
  ctypeconvnode:=tcputypeconvnode;
end.
