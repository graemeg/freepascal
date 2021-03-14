{
    Copyright (c) 1998-2020 by Florian Klaempfl and Nikolay Nikolov

    Generate WebAssembly code for type converting nodes

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

 ****************************************************************************}
unit nwasmcnv;

{$i fpcdefs.inc}

interface

    uses
      node,ncnv,ncgcnv;

    type

       { twasmtypeconvnode }

       twasmtypeconvnode = class(tcgtypeconvnode)
       protected
         function first_int_to_real: tnode; override;
         procedure second_int_to_real;override;
         procedure second_int_to_bool;override;
         procedure second_ansistring_to_pchar;override;
       end;

implementation

   uses
      verbose,globals,globtype,aasmdata,
      defutil,fmodule,cpubase,
      cgbase,cgutils,pass_1,pass_2,
      aasmbase,aasmcpu,
      symdef,
      hlcgobj,hlcgcpu;


{ twasmtypeconvnode }

    function twasmtypeconvnode.first_int_to_real: tnode;
      begin
        first_int_to_real:=nil;
        if left.resultdef.size<4 then
          begin
            inserttypeconv(left,s32inttype);
            firstpass(left);
          end;
        expectloc:=LOC_FPUREGISTER;
      end;


    procedure twasmtypeconvnode.second_int_to_real;
      var
        op: TAsmOp;
      begin
        secondpass(left);
        if codegenerror then
          exit;

        case tfloatdef(resultdef).floattype of
          s32real:
            begin
              if is_64bitint(left.resultdef) or
                is_currency(left.resultdef) then
                begin
                  if is_signed(left.resultdef) then
                    op:=a_f32_convert_i64_s
                  else
                    op:=a_f32_convert_i64_u;
                end
              else
                { other integers are supposed to be 32 bit }
                begin
                  if is_signed(left.resultdef) then
                    op:=a_f32_convert_i32_s
                  else
                    op:=a_f32_convert_i32_u;
                end;
            end;
          s64real:
            begin
              if is_64bitint(left.resultdef) or
                is_currency(left.resultdef) then
                begin
                  if is_signed(left.resultdef) then
                    op:=a_f64_convert_i64_s
                  else
                    op:=a_f64_convert_i64_u;
                end
              else
                { other integers are supposed to be 32 bit }
                begin
                  if is_signed(left.resultdef) then
                    op:=a_f64_convert_i32_s
                  else
                    op:=a_f64_convert_i32_u;
                end;
            end;
          else
            internalerror(2021010501);
        end;

        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
        current_asmdata.CurrAsmList.concat(taicpu.op_none(op));
        location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
        location.register := hlcg.getfpuregister(current_asmdata.CurrAsmList,resultdef);
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;


    procedure twasmtypeconvnode.second_int_to_bool;
      begin
        secondpass(left);
        if codegenerror then
          exit;
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
        thlcgwasm(hlcg).a_load_const_stack(current_asmdata.CurrAsmList,left.resultdef,0,R_INTREGISTER);
        thlcgwasm(hlcg).a_cmp_stack_stack(current_asmdata.CurrAsmList,left.resultdef,OC_NE);
        thlcgwasm(hlcg).resize_stack_int_val(current_asmdata.CurrAsmList,left.resultdef,resultdef,false);
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register := hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;


    procedure twasmtypeconvnode.second_ansistring_to_pchar;
      var
        hr : treference;
      begin
        thlcgwasm(hlcg).a_cmp_const_loc_stack(current_asmdata.CurrAsmList,left.resultdef,OC_NE,0,left.location);

        current_asmdata.CurrAsmList.Concat(taicpu.op_functype(a_if,TWasmFuncType.Create([],[wbt_i32])));
        thlcgwasm(hlcg).incblock;
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);

        current_asmdata.CurrAsmList.Concat(taicpu.op_none(a_else));
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

        { FPC_EMPTYCHAR is a widechar -> 2 bytes }
        reference_reset(hr,2,[]);
        hr.symbol:=current_asmdata.RefAsmSymbol('FPC_EMPTYCHAR',AT_DATA);
        current_module.add_extern_asmsym('FPC_EMPTYCHAR',AB_EXTERNAL,AT_DATA);
        thlcgwasm(hlcg).a_loadaddr_ref_stack(current_asmdata.CurrAsmList,cwidechartype,resultdef,hr);

        current_asmdata.CurrAsmList.Concat( taicpu.op_none(a_end_if) );
        thlcgwasm(hlcg).decblock;

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,resultdef);
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;

begin
  ctypeconvnode:=twasmtypeconvnode;
end.
