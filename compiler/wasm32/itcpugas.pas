{
    Copyright (c) 1998-2012 by Florian Klaempfl and others

    This unit contains the WebAssembly GAS instruction tables

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
unit itcpugas;

{$i fpcdefs.inc}

interface

  uses
    cpubase,cgbase,
    itcpuwasm;


  const
    { Standard opcode string table (for each tasmop enumeration). The
      opcode strings should conform to the names as defined by the
      processor manufacturer.
    }
    gas_op2str : op2strtable = (
      '<none>',
      // control flow
      'block', 'loop', 'br', 'br_if', 'br_table', 'if', 'else', 'end_block',
      'end_loop', 'end_if', 'end_function', 'return', 'unreachable',
      // basic
      'nop', 'drop', 'i32.const', 'i64.const', 'f32.const', 'f64.const',
      'local.get', 'local.set', 'local.tee', 'global.get', 'global.set',
      'select', 'call', 'call_indirect',
      // integer
      'i32.add', 'i64.add', 'i32.sub', 'i64.sub', 'i32.mul', 'i64.mul',
      'i32.div_s', 'i64.div_s', 'i32.div_u', 'i64.div_u', 'i32.rem_s', 'i64.rem_s',
      'i32.rem_u', 'i64.rem_u', 'i32.and', 'i64.and', 'i32.or', 'i64.or',
      'i32.xor', 'i64.xor', 'i32.shl', 'i64.shl', 'i32.shr_s', 'i64.shr_s',
      'i32.shr_u', 'i64.shr_u', 'i32.rotl', 'i64.rotl', 'i32.rotr', 'i64.rotr',
      'i32.clz', 'i64.clz', 'i32.ctz', 'i64.ctz', 'i32.popcnt', 'i64.popcnt',
      'i32.eqz', 'i64.eqz',
      // floating point
      'f32.add', 'f64.add', 'f32.sub', 'f64.sub', 'f32.mul', 'f64.mul',
      'f32.div', 'f64.div', 'f32.sqrt', 'f64.sqrt', 'f32.min', 'f64.min',
      'f32.max', 'f64.max', 'f32.ceil', 'f64.ceil', 'f32.floor', 'f64.floor',
      'f32.trunc', 'f64.trunc', 'f32.nearest', 'f64.nearest', 'f32.abs', 'f64.abs',
      'f32.neg', 'f64.neg', 'f32.copysign', 'f64.copysign',
      // integer compare
      'i32.eq', 'i64.eq', 'i32.ne', 'i64.ne', 'i32.lt_s', 'i64.lt_s',
      'i32.lt_u', 'i64.lt_u', 'i32.le_s', 'i64.le_s', 'i32.le_u', 'i64.le_u',
      'i32.gt_s', 'i64.gt_s', 'i32.gt_u', 'i64.gt_u', 'i32.ge_s', 'i64.ge_s',
      'i32.ge_u', 'i64.ge_u',
      // floating point compare
      'f32.eq', 'f64.eq', 'f32.ne', 'f64.ne', 'f32.lt', 'f64.lt',
      'f32.le', 'f64.le', 'f32.gt', 'f64.gt', 'f32.ge', 'f64.gt',
      // conversion
      'i32.wrap_i64', 'i64.extend_i32_s', 'i64.extend_i32_u',
      'i32.extend8_s','i32.extend16_s','i64.extend8_s','i64.extend16_s','i64.extend32_s',
      'i32.trunc_f32_s', 'i32.trunc_f64_s', 'i64.trunc_f32_s', 'i64.trunc_f64_s',
      'i32.trunc_f32_u', 'i32.trunc_f64_u', 'i64.trunc_f32_u', 'i64.trunc_f64_u',
      'f32.demote_f64', 'f64.promote_f32',
      'f32.convert_i32_s', 'f32.convert_i64_s', 'f64.convert_i32_s', 'f64.convert_i64_s',
      'f32.convert_i32_u', 'f32.convert_i64_u', 'f64.convert_i32_u', 'f64.convert_i64_u',
      'i32.reinterpret_f32', 'i64.reinterpret_f64', 'f32.reinterpret_i32', 'f64.reinterpret_i64',
      // load/store
      'i32.load', 'i64.load', 'f32.load', 'f64.load',
      'i32.store', 'i64.store', 'f32.store', 'f64.store',
      'i32.load8_s', 'i32.load16_s', 'i64.load8_s', 'i64.load16_s', 'i64.load32_s',
      'i32.load8_u', 'i32.load16_u', 'i64.load8_u', 'i64.load16_u', 'i64.load32_u',
      'i32.store8', 'i32.store16', 'i64.store8', 'i64.store16', 'i64.store32',
      // additional memory
      'memory.grow 0', 'memory.size 0'
    );

    gas_wasm_basic_type_str : array [TWasmBasicType] of string = ('i32','i64','f32','f64');

    function gas_regname(r:Tregister):string;


implementation


    function gas_regname(r:Tregister):string;
      begin
        result:=generic_regname(r);
      end;

end.
