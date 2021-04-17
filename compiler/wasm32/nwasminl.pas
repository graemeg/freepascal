{
    Copyright (c) 1998-2002, 2021 by Florian Klaempfl and Nikolay Nikolov

    Generate WebAssembly inline nodes

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
unit nwasminl;

{$i fpcdefs.inc}

interface

    uses
      node,ncginl;

    type

      { twasminlinenode }

      twasminlinenode = class(tcginlinenode)
      private
        procedure second_high; override;
        procedure second_memory_size;
        procedure second_memory_grow;
        procedure second_unreachable;
      public
        function pass_typecheck_cpu: tnode; override;
        function first_cpu: tnode; override;
        procedure pass_generate_code_cpu; override;
        procedure second_length;override;
      end;

implementation

    uses
      ninl,compinnr,
      cpubase,
      aasmbase,aasmdata,aasmcpu,
      cgbase,cgutils,
      hlcgobj,hlcgcpu,
      defutil,pass_2,verbose,
      symtype,symdef;

{*****************************************************************************
                               twasminlinenode
*****************************************************************************}

    procedure twasminlinenode.second_high;
      var
        hightype: TWasmBasicType;
      begin
        secondpass(left);
        if not(is_dynamic_array(left.resultdef)) then
          Internalerror(2019122801);
        { determine the WasmBasicType of the result }
        if is_64bit(resultdef) then
          hightype:=wbt_i64
        else
          hightype:=wbt_i32;
        { length in dynamic arrays is at offset -sizeof(pint) }
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
        { 64-bit pointer values need a <>0 comparison to produce a 32-bit int on the stack (0 or 1) for the 'if' instruction.
          32-bit pointer values don't need it, because 'if' already expects and pops a 32-bit int and checks for <>0. }
        if is_64bit(left.resultdef) then
          begin
            thlcgwasm(hlcg).a_load_const_stack(current_asmdata.CurrAsmList,left.resultdef,0,R_INTREGISTER);
            thlcgwasm(hlcg).a_cmp_stack_stack(current_asmdata.CurrAsmList,left.resultdef,OC_NE);
          end;
        { if not nil }
        current_asmdata.CurrAsmList.Concat(taicpu.op_functype(a_if,TWasmFuncType.Create([],[hightype])));
        thlcgwasm(hlcg).incblock;
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
        { volatility of the dyn. array refers to the volatility of the
          string pointer, not of the string data }
        thlcgwasm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
        { length in dynamic arrays is at offset -ossinttype.size }
        thlcgwasm(hlcg).a_op_const_stack(current_asmdata.CurrAsmList,OP_SUB,left.resultdef,ossinttype.size);
        { load length }
        if ossinttype.size=8 then
          current_asmdata.CurrAsmList.Concat(taicpu.op_const(a_i64_load,0))
        else
          current_asmdata.CurrAsmList.Concat(taicpu.op_const(a_i32_load,0));
        { else }
        current_asmdata.CurrAsmList.Concat(taicpu.op_none(a_else));
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
        { high=-1 }
        thlcgwasm(hlcg).a_load_const_stack(current_asmdata.CurrAsmList,resultdef,-1,R_INTREGISTER);
        { endif }
        current_asmdata.CurrAsmList.Concat(taicpu.op_none(a_end_if));
        thlcgwasm(hlcg).decblock;

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
        if location.size in [OS_64,OS_S64] then
          begin
            location.register64.reglo := cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            location.register64.reghi := cg.getintregister(current_asmdata.CurrAsmList,OS_32);
          end
        else
{$endif}
          location.register := hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);

        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;


    procedure twasminlinenode.second_memory_size;
      begin
        current_asmdata.CurrAsmList.Concat(taicpu.op_none(a_memory_size));
        thlcgwasm(hlcg).incstack(current_asmdata.CurrAsmList,1);

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,resultdef);
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;


    procedure twasminlinenode.second_memory_grow;
      begin
        secondpass(left);

        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
        thlcgwasm(hlcg).a_load_reg_stack(current_asmdata.CurrAsmList,left.resultdef,left.location.register);

        current_asmdata.CurrAsmList.Concat(taicpu.op_none(a_memory_grow));

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,resultdef);
        thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
      end;


    procedure twasminlinenode.second_unreachable;
      begin
        location_reset(location,LOC_VOID,OS_NO);
        current_asmdata.CurrAsmList.Concat(taicpu.op_none(a_unreachable));
      end;


    function twasminlinenode.pass_typecheck_cpu: tnode;
      begin
        Result:=nil;
        case inlinenumber of
          in_wasm32_memory_size:
            begin
              CheckParameters(0);
              resultdef:=u32inttype;
            end;
          in_wasm32_memory_grow:
            begin
              CheckParameters(1);
              resultdef:=u32inttype;
            end;
          in_wasm32_unreachable:
            begin
              CheckParameters(0);
              resultdef:=voidtype;
            end;
          else
            Result:=inherited pass_typecheck_cpu;
        end;
      end;


    function twasminlinenode.first_cpu: tnode;
      begin
        Result:=nil;
        case inlinenumber of
          in_wasm32_memory_size,
          in_wasm32_memory_grow:
            expectloc:=LOC_REGISTER;
          in_wasm32_unreachable:
            expectloc:=LOC_VOID;
          else
            Result:=inherited first_cpu;
        end;
      end;


    procedure twasminlinenode.pass_generate_code_cpu;
      begin
        case inlinenumber of
          in_wasm32_memory_size:
            second_memory_size;
          in_wasm32_memory_grow:
            second_memory_grow;
          in_wasm32_unreachable:
            second_unreachable;
          else
            inherited pass_generate_code_cpu;
        end;
      end;


    procedure twasminlinenode.second_length;
      var
        lendef : tdef;
        href : treference;
        extra_slots: LongInt;
      begin
        secondpass(left);
        if is_shortstring(left.resultdef) then
          begin
            location_copy(location,left.location);
            location.size:=OS_8;
          end
        else
          begin
            { length in ansi/wide strings and high in dynamic arrays is at offset -sizeof(pint) }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

            thlcgwasm(hlcg).a_cmp_const_reg_stack(current_asmdata.CurrAsmList,left.resultdef,OC_EQ,0,left.location.register);

            current_asmdata.CurrAsmList.Concat(taicpu.op_functype(a_if,TWasmFuncType.Create([],[wbt_i32])));
            thlcgwasm(hlcg).incblock;
            thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

            current_asmdata.CurrAsmList.Concat(taicpu.op_const(a_i32_const,0));
            thlcgwasm(hlcg).incstack(current_asmdata.CurrAsmList,1);

            current_asmdata.CurrAsmList.Concat( taicpu.op_none(a_else) );
            thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

            { the length of a widestring is a 32 bit unsigned int. Since every
              character occupies 2 bytes, on a 32 bit platform you can express
              the maximum length using 31 bits. On a 64 bit platform, it may be
              32 bits. This means that regardless of the platform, a location
              with size OS_SINT/ossinttype can hold the length without
              overflowing (this code returns an ossinttype value) }
            if is_widestring(left.resultdef) then
              lendef:=u32inttype
            else
              lendef:=ossinttype;
            { volatility of the ansistring/widestring refers to the volatility of the
              string pointer, not of the string data }
            hlcg.reference_reset_base(href,left.resultdef,left.location.register,-lendef.size,ctempposinvalid,lendef.alignment,[]);

            extra_slots:=thlcgwasm(hlcg).prepare_stack_for_ref(current_asmdata.CurrAsmList,href,false);
            thlcgwasm(hlcg).a_load_ref_stack(current_asmdata.CurrAsmList,lendef,href,extra_slots);
            if is_widestring(left.resultdef) then
              thlcgwasm(hlcg).a_op_const_stack(current_asmdata.CurrAsmList,OP_SHR,resultdef,1);

            { Dynamic arrays do not have their length attached but their maximum index }
            if is_dynamic_array(left.resultdef) then
              thlcgwasm(hlcg).a_op_const_stack(current_asmdata.CurrAsmList,OP_ADD,resultdef,1);

            current_asmdata.CurrAsmList.Concat( taicpu.op_none(a_end_if) );
            thlcgwasm(hlcg).decblock;

            location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
            location.register:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,resultdef);
            thlcgwasm(hlcg).a_load_stack_loc(current_asmdata.CurrAsmList,resultdef,location);
          end;
      end;

begin
  cinlinenode:=twasminlinenode;
end.
