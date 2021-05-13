{
    Copyright (c) 1998-2017 by Florian Klaempfl

    Generates AVR inline nodes

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
unit navrinl;

{$i fpcdefs.inc}

  interface

    uses
      node,ninl,ncginl, aasmbase;

    type
      tavrinlinenode = class(tcginlinenode)
        procedure second_abs_long; override;

        function pass_typecheck_cpu:tnode;override;
        function first_cpu : tnode;override;
        procedure pass_generate_code_cpu;override;
      end;

  implementation

    uses
      verbose,
      constexp,
      compinnr,
      aasmdata,
      aasmcpu,
      symdef,
      defutil,
      hlcgobj,
      pass_2,
      cgbase, cgobj, cgutils,
      ncon,ncal,
      cpubase;

    procedure tavrinlinenode.second_abs_long;
      var
        hl: TAsmLabel;
        size: TCgSize;
      begin
        secondpass(left);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

        location:=left.location;
        location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,left.resultdef);

        size:=def_cgsize(left.resultdef);

        current_asmdata.getjumplabel(hl);
        cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,size,OC_GTE,0,left.location.register,hl);
        cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,size,left.location.register,location.register);

        cg.a_label(current_asmdata.CurrAsmList,hl);
      end;


    function tavrinlinenode.pass_typecheck_cpu : tnode;
      var
        para1,para2,para3,para4: tcallparanode;
      begin
        Result:=nil;
        case inlinenumber of
          in_avr_nop,
          in_avr_sleep,
          in_avr_sei,
          in_avr_wdr,
          in_avr_cli:
            begin
              CheckParameters(0);
              resultdef:=voidtype;
            end;
          in_avr_save:
            begin
              CheckParameters(0);
              resultdef:=u8inttype;
            end;
          in_avr_restore:
            begin
              CheckParameters(1);
              resultdef:=voidtype;
            end;
          in_avr_des:
            begin
              CheckParameters(4);
              resultdef:=voidtype;
              para4:=tcallparanode(left);
              para3:=tcallparanode(tcallparanode(para4).nextpara);
              para2:=tcallparanode(tcallparanode(para3).nextpara);
              para1:=tcallparanode(tcallparanode(para2).nextpara);
              if not(is_constintnode(para4.paravalue)) then
                MessagePos(para4.paravalue.fileinfo,type_e_constant_expr_expected);
              if not(is_constboolnode(para3.paravalue)) then
                MessagePos(para3.paravalue.fileinfo,type_e_constant_expr_expected);
              if (tordconstnode(para4.paravalue).value<0) or
                (tordconstnode(para4.paravalue).value>15) then
                MessagePos(para4.paravalue.fileinfo,parser_e_range_check_error);
            end;
          else
            Result:=inherited pass_typecheck_cpu;
        end;
      end;


    function tavrinlinenode.first_cpu : tnode;
      begin
        Result:=nil;
        case inlinenumber of
          in_avr_nop,
          in_avr_sleep,
          in_avr_sei,
          in_avr_wdr,
          in_avr_cli,
          in_avr_restore,
          in_avr_des:
            begin
              expectloc:=LOC_VOID;
              resultdef:=voidtype;
            end;
          in_avr_save:
            begin
              expectloc:=LOC_REGISTER;
              resultdef:=u8inttype;
            end;
          else
            Result:=inherited first_cpu;
        end;
      end;


    procedure tavrinlinenode.pass_generate_code_cpu;
      var
        para1,para2,para3,para4: tcallparanode;
        ref: treference;
        r: TRegister;
      begin
        case inlinenumber of
          in_avr_nop:
            current_asmdata.CurrAsmList.concat(taicpu.op_none(A_NOP));
          in_avr_sleep:
            current_asmdata.CurrAsmList.concat(taicpu.op_none(A_SLEEP));
          in_avr_sei:
            current_asmdata.CurrAsmList.concat(taicpu.op_none(A_SEI));
          in_avr_wdr:
            current_asmdata.CurrAsmList.concat(taicpu.op_none(A_WDR));
          in_avr_cli:
            current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLI));
          in_avr_save:
            begin
              location_reset(location,LOC_CREGISTER,OS_8);
              location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_8);

              current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_IN, location.register, NIO_SREG));
              current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLI));
            end;
          in_avr_restore:
            begin
              secondpass(left);
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
              current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_OUT, NIO_SREG, left.location.register));
            end;
          in_avr_des:
            begin
              para4:=tcallparanode(left);
              para3:=tcallparanode(tcallparanode(para4).nextpara);
              para2:=tcallparanode(tcallparanode(para3).nextpara);
              para1:=tcallparanode(tcallparanode(para2).nextpara);
              secondpass(tcallparanode(para1).paravalue);
              secondpass(tcallparanode(para2).paravalue);

              cg.getcpuregister(current_asmdata.CurrAsmList,NR_R30);
              cg.getcpuregister(current_asmdata.CurrAsmList,NR_R31);

              cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,tcallparanode(para2).paravalue.location.reference,NR_R30);
              reference_reset(ref,0,[]);
              ref.base:=NR_R30;
              for r:=NR_R8 to NR_R15 do
                begin
                  cg.getcpuregister(current_asmdata.CurrAsmList,r);
                  cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_8,OS_8,ref,r);
                  inc(ref.offset);
                end;
              cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,tcallparanode(para1).paravalue.location.reference,NR_R30);
              reference_reset(ref,0,[]);
              ref.base:=NR_R30;
              for r:=NR_R0 to NR_R7 do
                begin
                  cg.getcpuregister(current_asmdata.CurrAsmList,r);
                  cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_8,OS_8,ref,r);
                  inc(ref.offset);
                end;
              if tordconstnode(para3.paravalue).value=0 then
                current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLH))
              else
                current_asmdata.CurrAsmList.concat(taicpu.op_none(A_SEH));
              current_asmdata.CurrAsmList.concat(taicpu.op_const(A_DES,int64(tordconstnode(para4.paravalue).value)));

              for r:=NR_R8 to NR_R15 do
                cg.ungetcpuregister(current_asmdata.CurrAsmList,r);

              { save data }
              ref.offset:=0;
              for r:=NR_R0 to NR_R7 do
                begin
                  cg.a_load_reg_ref(current_asmdata.CurrAsmList,OS_8,OS_8,r,ref);
                  cg.ungetcpuregister(current_asmdata.CurrAsmList,r);
                  inc(ref.offset);
                end;
            end
          else
            inherited pass_generate_code_cpu;
        end;
      end;

begin
  cinlinenode:=tavrinlinenode;
end.
