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
              CheckParameters(2);
              resultdef:=voidtype;
              if not(is_constintnode(tcallparanode(left).paravalue)) then
                MessagePos(tcallparanode(left).paravalue.fileinfo,type_e_constant_expr_expected);
              if not(is_constboolnode(tcallparanode(tcallparanode(left).nextpara).paravalue)) then
                MessagePos(tcallparanode(tcallparanode(left).nextpara).paravalue.fileinfo,type_e_constant_expr_expected);
              if (tordconstnode(tcallparanode(left).paravalue).value<0) or
                (tordconstnode(tcallparanode(left).paravalue).value>15) then
                MessagePos(tcallparanode(left).paravalue.fileinfo,parser_e_range_check_error);
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
              if tordconstnode(tcallparanode(tcallparanode(left).nextpara).paravalue).value=0 then
                current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLH))
              else
                current_asmdata.CurrAsmList.concat(taicpu.op_none(A_SEH));
              current_asmdata.CurrAsmList.concat(taicpu.op_const(A_DES,int64(tordconstnode(tcallparanode(left).paravalue).value)));
            end;
          else
            inherited pass_generate_code_cpu;
        end;
      end;

begin
  cinlinenode:=tavrinlinenode;
end.
