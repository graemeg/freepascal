{ This file is part of wasmbin - a collection of WebAssembly binary utils.

  Copyright (C) 2019, 2020 Dmitry Boyarintsev <skalogryz.lists@gmail.com>
  Copyright (C) 2020 by the Free Pascal development team

  This source is free software; you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  A copy of the GNU General Public License is available on the World Wide Web
  at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
  to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
  Boston, MA 02110-1335, USA.
}

unit wasmbincode;
// WebAssembly instructions information
// http://webassembly.github.io/spec/core/binary/instructions.html

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, lebutils;

const
  VALTYPE_NONE = $40;
  VALTYPE_I32  = $7F;
  VALTYPE_I64  = $7E;
  VALTYPE_F32  = $7D;
  VALTYPE_F64  = $7C;

  INST_TRAP = $00;
  // ..
  INST_IF   = $04;
  INST_ELSE = $05;
  // ...
  INST_END  = $0b;

  INST_block       = $02;
  INST_loop        = $03;
  INST_unreachable = $00;
  INST_nop         = $01;
  INST_br          = $0C;
  INST_br_if          = $0D;
  INST_br_table       = $0E;
  INST_return         = $0F;
  INST_call           = $10;
  INST_call_indirect  = $11;

  //INST_if             = $04;
  //INST_else           = $05;
  //INST_end            = $0B;
  INST_drop           = $1A;
  INST_select         = $1B;
  INST_local_get      = $20;
  INST_local_set      = $21;
  INST_local_tee      = $22;
  INST_global_get     = $23;
  INST_global_set     = $24;

  INST_i32_load       = $28;
  INST_i64_load       = $29;
  INST_f32_load       = $2a;
  INST_f64_load       = $2b;
  INST_i32_load8_s    = $2c;
  INST_i32_load8_u    = $2d;
  INST_i32_load16_s   = $2e;
  INST_i32_load16_u   = $2f;
  INST_i64_load8_s    = $30;
  INST_i64_load8_u    = $31;
  INST_i64_load16_s   = $32;
  INST_i64_load16_u   = $33;
  INST_i64_load32_s   = $34;
  INST_i64_load32_u   = $35;
  INST_i32_store      = $36;
  INST_i64_store      = $37;
  INST_f32_store      = $38;
  INST_f64_store      = $39;
  INST_i32_store8     = $3a;
  INST_i32_store16    = $3b;
  INST_i64_store8     = $3c;
  INST_i64_store16    = $3d;
  INST_i64_store32    = $3e;
  INST_memory_size    = $3f;
  INST_memory_grow    = $40;
  INST_i32_const      = $41;
  INST_i64_const      = $42;
  INST_f32_const      = $43;
  INST_f64_const      = $44;

  INST_i32_clz        = $67;
  INST_i32_ctz        = $68;
  INST_i32_popcnt     = $69;
  INST_i32_add        = $6a;
  INST_i32_sub        = $6b;
  INST_i32_mul        = $6c;
  INST_i32_div_s      = $6d;
  INST_i32_div_u      = $6e;
  INST_i32_rem_s      = $6f;
  INST_i32_rem_u      = $70;
  INST_i32_and        = $71;
  INST_i32_or         = $72;
  INST_i32_xor        = $73;
  INST_i32_shl        = $74;
  INST_i32_shr_s      = $75;
  INST_i32_shr_u      = $76;
  INST_i32_rotl       = $77;
  INST_i32_rotr       = $78;
  INST_i64_clz        = $79;
  INST_i64_ctz        = $7a;
  INST_i64_popcnt     = $7b;
  INST_i64_add        = $7c;
  INST_i64_sub        = $7d;
  INST_i64_mul        = $7e;
  INST_i64_div_s      = $7f;
  INST_i64_div_u      = $80;
  INST_i64_rem_s      = $81;
  INST_i64_rem_u      = $82;
  INST_i64_and        = $83;
  INST_i64_or         = $84;
  INST_i64_xor        = $85;
  INST_i64_shl        = $86;
  INST_i64_shr_s      = $87;
  INST_i64_shr_u      = $88;
  INST_i64_rotl       = $89;
  INST_i64_rotr       = $8a;
  INST_f32_abs        = $8b;
  INST_f32_neg        = $8c;
  INST_f32_ceil       = $8d;
  INST_f32_floor      = $8e;
  INST_f32_trunc      = $8f;
  INST_f32_nearest    = $90;
  INST_f32_sqrt       = $91;
  INST_f32_add        = $92;
  INST_f32_sub        = $93;
  INST_f32_mul        = $94;
  INST_f32_div        = $95;
  INST_f32_min        = $96;
  INST_f32_max        = $97;
  INST_f32_copysign   = $98;
  INST_f64_abs        = $99;
  INST_f64_neg        = $9a;
  INST_f64_ceil       = $9b;
  INST_f64_floor      = $9c;
  INST_f64_trunc      = $9d;
  INST_f64_nearest    = $9e;
  INST_f64_sqrt       = $9f;
  INST_f64_add        = $a0;
  INST_f64_sub        = $a1;
  INST_f64_mul        = $a2;
  INST_f64_div        = $a3;
  INST_f64_min        = $a4;
  INST_f64_max        = $a5;
  INST_f64_copysign   = $a6;

  INST_i32_eqz        = $45;
  INST_i32_eq         = $46;
  INST_i32_ne         = $47;
  INST_i32_lt_s       = $48;
  INST_i32_lt_u       = $49;
  INST_i32_gt_s       = $4a;
  INST_i32_gt_u       = $4b;
  INST_i32_le_s       = $4c;
  INST_i32_le_u       = $4d;
  INST_i32_ge_s       = $4e;
  INST_i32_ge_u       = $4f;
  INST_i64_eqz        = $50;
  INST_i64_eq         = $51;
  INST_i64_ne         = $52;
  INST_i64_lt_s       = $53;
  INST_i64_lt_u       = $54;
  INST_i64_gt_s       = $55;
  INST_i64_gt_u       = $56;
  INST_i64_le_s       = $57;
  INST_i64_le_u       = $58;
  INST_i64_ge_s       = $59;
  INST_i64_ge_u       = $5a;
  INST_f32_eq         = $5b;
  INST_f32_ne         = $5c;
  INST_f32_lt         = $5d;
  INST_f32_gt         = $5e;
  INST_f32_le         = $5f;
  INST_f32_ge         = $60;
  INST_f64_eq         = $61;
  INST_f64_ne         = $62;
  INST_f64_lt         = $63;
  INST_f64_gt         = $64;
  INST_f64_le         = $65;
  INST_f64_ge         = $66;

  INST_i32_wrap_i64        = $a7;
  INST_i32_trunc_f32_s     = $a8;
  INST_i32_trunc_f32_u     = $a9;
  INST_i32_trunc_f64_s     = $aa;
  INST_i32_trunc_f64_u     = $ab;
  INST_i64_extend_i32_s    = $ac;
  INST_i64_extend_i32_u    = $ad;
  INST_i64_trunc_f32_s     = $ae;
  INST_i64_trunc_f32_u     = $af;
  INST_i64_trunc_f64_s     = $b0;
  INST_i64_trunc_f64_u     = $b1;
  INST_f32_convert_i32_s   = $b2;
  INST_f32_convert_i32_u   = $b3;
  INST_f32_convert_i64_s   = $b4;
  INST_f32_convert_i64_u   = $b5;
  INST_f32_demote_f64      = $b6;
  INST_f64_convert_i32_s   = $b7;
  INST_f64_convert_i32_u   = $b8;
  INST_f64_convert_i64_s   = $b9;
  INST_f64_convert_i64_u   = $ba;
  INST_f64_promote_f32     = $bb;
  INST_i32_reinterpret_f32 = $bc;
  INST_i64_reinterpret_f64 = $bd;
  INST_f32_reinterpret_i32 = $be;
  INST_f64_reinterpret_i64 = $bf;
  // ...
  INST_REINTERPRET_I64 = $BF;

  MIN_INST = INST_TRAP;
  MAX_INST = INST_REINTERPRET_I64;

type
  TInstParamType = (ipNone,
    ipLeb,       // label index or function index
    ipOfsAlign,  // memory arguments, ask for offset + align
    ipi32,       // signed Leb of maximum 4 bytes
    ipi64,       // signed Leb of maximum 8 bytes
    ipu32,       // unsigned Leb of maximum 4 bytes
    ipu64,       // unsigned Leb of maximum 8 bytes
    ipf32,       // float point single
    ipf64,       // float point double
    ipJumpVec,   // an array of jump labels used for br_table only
    ipResType,   // result type used for blocks, such as If, block or loop
    ipCallType,  // used for call_indirect
    ipi32OrFunc, // use for i32.const. Either a numeric OR function id is accepted.
                 // numeric is used as an actually value.
                 // function Id will be replaced with a reference number to the function
                 // and relocation information would be generated
    ipZero       // followed by a single byte zero
  );
  TInstFlag = record
    valid  : Boolean;
    Param  : TInstParamType;
    align  : Byte; // used for memory operations only
  end;

const
  INST_FLAGS : array [MIN_INST..MAX_INST] of TInstFlag = (
    (valid: true;  Param: ipNone;      align: 0)  // 00 trap (unreachable)
   ,(valid: true;  Param: ipNone;      align: 0)  // 01 nop
   ,(valid: true;  Param: ipResType;   align: 0)  // 02 block
   ,(valid: true;  Param: ipResType;   align: 0)  // 03 lock
   ,(valid: true;  Param: ipResType;   align: 0)  // 04 if
   ,(valid: true;  Param: ipNone;      align: 0)  // 05 else
   ,(valid: false; Param: ipNone;      align: 0)  // 06
   ,(valid: false; Param: ipNone;      align: 0)  // 07
   ,(valid: false; Param: ipNone;      align: 0)  // 08
   ,(valid: false; Param: ipNone;      align: 0)  // 09
   ,(valid: false; Param: ipNone;      align: 0)  // 0A
   ,(valid: true;  Param: ipNone;      align: 0)  // 0B  end
   ,(valid: true;  Param: ipLeb;       align: 0)  // 0C  br
   ,(valid: true;  Param: ipLeb;       align: 0)  // 0D  br_if
   ,(valid: true;  Param: ipJumpVec;   align: 0)  // 0E  br_table
   ,(valid: true;  Param: ipNone;      align: 0)  // 0F  return
   ,(valid: true;  Param: ipLeb;       align: 0)  // 10  call
   ,(valid: true;  Param: ipCallType;  align: 0)  // 11  call_indirect
   ,(valid: false; Param: ipNone;      align: 0)  // 12
   ,(valid: false; Param: ipNone;      align: 0)  // 13
   ,(valid: false; Param: ipNone;      align: 0)  // 14
   ,(valid: false; Param: ipNone;      align: 0)  // 15
   ,(valid: false; Param: ipNone;      align: 0)  // 16
   ,(valid: false; Param: ipNone;      align: 0)  // 17
   ,(valid: false; Param: ipNone;      align: 0)  // 18
   ,(valid: false; Param: ipNone;      align: 0)  // 19
   ,(valid: true;  Param: ipNone;      align: 0)  // 1A drop
   ,(valid: true;  Param: ipNone;      align: 0)  // 1B select
   ,(valid: false; Param: ipNone;      align: 0)  // 1C
   ,(valid: false; Param: ipNone;      align: 0)  // 1D
   ,(valid: false; Param: ipNone;      align: 0)  // 1E
   ,(valid: false; Param: ipNone;      align: 0)  // 1F
   ,(valid: true;  Param: ipLeb;       align: 0)  // 20  local.get
   ,(valid: true;  Param: ipLeb;       align: 0)  // 21  local.set
   ,(valid: true;  Param: ipLeb;       align: 0)  // 22  local.tee
   ,(valid: true;  Param: ipLeb;       align: 0)  // 23  global.get
   ,(valid: true;  Param: ipLeb;       align: 0)  // 24  global.set
   ,(valid: false; Param: ipNone;      align: 0)  // 25
   ,(valid: false; Param: ipNone;      align: 0)  // 26
   ,(valid: false; Param: ipNone;      align: 0)  // 27
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 28  i32.load
   ,(valid: true;  Param: ipOfsAlign;  align: 3)  // 29  i64_load
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 2A  f32_load
   ,(valid: true;  Param: ipOfsAlign;  align: 3)  // 2B  f64_load
   ,(valid: true;  Param: ipOfsAlign;  align: 0)  // 2C  i32_load8_s
   ,(valid: true;  Param: ipOfsAlign;  align: 0)  // 2D  i32_load8_u
   ,(valid: true;  Param: ipOfsAlign;  align: 1)  // 2E  i32_load16_s
   ,(valid: true;  Param: ipOfsAlign;  align: 1)  // 2F  i32_load16_u
   ,(valid: true;  Param: ipOfsAlign;  align: 0)  // 30  i64_load8_s
   ,(valid: true;  Param: ipOfsAlign;  align: 0)  // 31  i64_load8_u
   ,(valid: true;  Param: ipOfsAlign;  align: 1)  // 32  i64_load16_s
   ,(valid: true;  Param: ipOfsAlign;  align: 1)  // 33  i64_load16_u
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 34  i64.load32_s
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 35  i64.load32_u
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 36  i32_store
   ,(valid: true;  Param: ipOfsAlign;  align: 3)  // 37  i64_store
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 38  f32_store
   ,(valid: true;  Param: ipOfsAlign;  align: 3)  // 39  f64_store
   ,(valid: true;  Param: ipOfsAlign;  align: 0)  // 3A  i32_store8
   ,(valid: true;  Param: ipOfsAlign;  align: 1)  // 3B  i32_store16
   ,(valid: true;  Param: ipOfsAlign;  align: 0)  // 3C  i64_store8
   ,(valid: true;  Param: ipOfsAlign;  align: 1)  // 3D  i64_store16
   ,(valid: true;  Param: ipOfsAlign;  align: 2)  // 3E  i64_store32
   ,(valid: true;  Param: ipZero;      align: 0)  // 3F  memory_size
   ,(valid: true;  Param: ipZero;      align: 0)  // 40  memory_grow
   ,(valid: true;  Param: ipi32OrFunc; align: 0)  // 41  i32_const
   ,(valid: true;  Param: ipi64;       align: 0)  // 42  i64_const
   ,(valid: true;  Param: ipf32;       align: 0)  // 43  f32_const
   ,(valid: true;  Param: ipf64;       align: 0)  // 44  f64_const
   ,(valid: true;  Param: ipNone;      align: 0)  // 45  i32_eqz
   ,(valid: true;  Param: ipNone;      align: 0)  // 46  i32_eq
   ,(valid: true;  Param: ipNone;      align: 0)  // 47  i32_ne
   ,(valid: true;  Param: ipNone;      align: 0)  // 48  i32_lt_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 49  i32_lt_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 4A  i32_gt_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 4B  i32_gt_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 4C  i32_le_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 4D  i32_le_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 4E  i32_ge_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 4F  i32_ge_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 50  i64_eqz
   ,(valid: true;  Param: ipNone;      align: 0)  // 51  i64_eq
   ,(valid: true;  Param: ipNone;      align: 0)  // 52  i64_ne
   ,(valid: true;  Param: ipNone;      align: 0)  // 53  i64_lt_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 54  i64_lt_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 55  i64_gt_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 56  i64_gt_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 57  i64_le_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 58  i64_le_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 59  i64_ge_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 5A  i64_ge_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 5B  f32_eq
   ,(valid: true;  Param: ipNone;      align: 0)  // 5C  f32_ne
   ,(valid: true;  Param: ipNone;      align: 0)  // 5D  f32_lt
   ,(valid: true;  Param: ipNone;      align: 0)  // 5E  f32_gt
   ,(valid: true;  Param: ipNone;      align: 0)  // 5F  f32_le
   ,(valid: true;  Param: ipNone;      align: 0)  // 60  f32_ge
   ,(valid: true;  Param: ipNone;      align: 0)  // 61  f64_eq
   ,(valid: true;  Param: ipNone;      align: 0)  // 62  f64_ne
   ,(valid: true;  Param: ipNone;      align: 0)  // 63  f64_lt
   ,(valid: true;  Param: ipNone;      align: 0)  // 64  f64_gt
   ,(valid: true;  Param: ipNone;      align: 0)  // 65  f64_le
   ,(valid: true;  Param: ipNone;      align: 0)  // 66  f64_ge
   ,(valid: true;  Param: ipNone;      align: 0)  // 67  i32_clz
   ,(valid: true;  Param: ipNone;      align: 0)  // 68  i32_ctz
   ,(valid: true;  Param: ipNone;      align: 0)  // 69  i32_popcnt
   ,(valid: true;  Param: ipNone;      align: 0)  // 6A  i32_add
   ,(valid: true;  Param: ipNone;      align: 0)  // 6B  i32_sub
   ,(valid: true;  Param: ipNone;      align: 0)  // 6C  i32_mul
   ,(valid: true;  Param: ipNone;      align: 0)  // 6D  i32_div_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 6E  i32_div_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 6F  i32_rem_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 70  i32_rem_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 71  i32_and
   ,(valid: true;  Param: ipNone;      align: 0)  // 72  i32_or
   ,(valid: true;  Param: ipNone;      align: 0)  // 73  i32_xor
   ,(valid: true;  Param: ipNone;      align: 0)  // 74  i32_shl
   ,(valid: true;  Param: ipNone;      align: 0)  // 75  i32_shr_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 76  i32_shr_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 77  i32_rotl
   ,(valid: true;  Param: ipNone;      align: 0)  // 78  i32_rotr
   ,(valid: true;  Param: ipNone;      align: 0)  // 79  i64_clz
   ,(valid: true;  Param: ipNone;      align: 0)  // 7A  i64_ctz
   ,(valid: true;  Param: ipNone;      align: 0)  // 7B  i64_popcnt
   ,(valid: true;  Param: ipNone;      align: 0)  // 7C  i64_add
   ,(valid: true;  Param: ipNone;      align: 0)  // 7D  i64_sub
   ,(valid: true;  Param: ipNone;      align: 0)  // 7E  i64_mul
   ,(valid: true;  Param: ipNone;      align: 0)  // 7F  i64_div_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 80  i64_div_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 81  i64_rem_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 82  i64_rem_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 83  i64_and
   ,(valid: true;  Param: ipNone;      align: 0)  // 84  i64_or
   ,(valid: true;  Param: ipNone;      align: 0)  // 85  i64_xor
   ,(valid: true;  Param: ipNone;      align: 0)  // 86  i64_shl
   ,(valid: true;  Param: ipNone;      align: 0)  // 87  i64_shr_s
   ,(valid: true;  Param: ipNone;      align: 0)  // 88  i64_shr_u
   ,(valid: true;  Param: ipNone;      align: 0)  // 89  i64_rotl
   ,(valid: true;  Param: ipNone;      align: 0)  // 8A  i64_rotr
   ,(valid: true;  Param: ipNone;      align: 0)  // 8B  f32_abs
   ,(valid: true;  Param: ipNone;      align: 0)  // 8C  f32_neg
   ,(valid: true;  Param: ipNone;      align: 0)  // 8D  f32_ceil
   ,(valid: true;  Param: ipNone;      align: 0)  // 8E  f32_floor
   ,(valid: true;  Param: ipNone;      align: 0)  // 8F  f32_trunc
   ,(valid: true;  Param: ipNone;      align: 0)  // 90  f32_nearest
   ,(valid: true;  Param: ipNone;      align: 0)  // 91  f32_sqrt
   ,(valid: true;  Param: ipNone;      align: 0)  // 92  f32_add
   ,(valid: true;  Param: ipNone;      align: 0)  // 93  f32_sub
   ,(valid: true;  Param: ipNone;      align: 0)  // 94  f32_mul
   ,(valid: true;  Param: ipNone;      align: 0)  // 95  f32_div
   ,(valid: true;  Param: ipNone;      align: 0)  // 96  f32_min
   ,(valid: true;  Param: ipNone;      align: 0)  // 97  f32_max
   ,(valid: true;  Param: ipNone;      align: 0)  // 98  f32_copysign
   ,(valid: true;  Param: ipNone;      align: 0)  // 99  f64_abs
   ,(valid: true;  Param: ipNone;      align: 0)  // 9A  f64_neg
   ,(valid: true;  Param: ipNone;      align: 0)  // 9B  f64_ceil
   ,(valid: true;  Param: ipNone;      align: 0)  // 9C  f64_floor
   ,(valid: true;  Param: ipNone;      align: 0)  // 9D  f64_trunc
   ,(valid: true;  Param: ipNone;      align: 0)  // 9E  f64_nearest
   ,(valid: true;  Param: ipNone;      align: 0)  // 9F  f64_sqrt
   ,(valid: true;  Param: ipNone;      align: 0)  // A0  f64_add
   ,(valid: true;  Param: ipNone;      align: 0)  // A1  f64_sub
   ,(valid: true;  Param: ipNone;      align: 0)  // A2  f64_mul
   ,(valid: true;  Param: ipNone;      align: 0)  // A3  f64_div
   ,(valid: true;  Param: ipNone;      align: 0)  // A4  f64_min
   ,(valid: true;  Param: ipNone;      align: 0)  // A5  f64_max
   ,(valid: true;  Param: ipNone;      align: 0)  // A6  f64_copysign
   ,(valid: true;  Param: ipNone;      align: 0)  // A7  i32_wrap_i64
   ,(valid: true;  Param: ipNone;      align: 0)  // A8  i32_trunc_f32_s
   ,(valid: true;  Param: ipNone;      align: 0)  // A9  i32_trunc_f32_u
   ,(valid: true;  Param: ipNone;      align: 0)  // AA  i32_trunc_f64_s
   ,(valid: true;  Param: ipNone;      align: 0)  // AB  i32_trunc_f64_u
   ,(valid: true;  Param: ipNone;      align: 0)  // AC  i64_extend_i32_s
   ,(valid: true;  Param: ipNone;      align: 0)  // AD  i64_extend_i32_u
   ,(valid: true;  Param: ipNone;      align: 0)  // AE  i64_trunc_f32_s
   ,(valid: true;  Param: ipNone;      align: 0)  // AF  i64_trunc_f32_u
   ,(valid: true;  Param: ipNone;      align: 0)  // B0  i64_trunc_f64_s
   ,(valid: true;  Param: ipNone;      align: 0)  // B1  i64_trunc_f64_u
   ,(valid: true;  Param: ipNone;      align: 0)  // B2  f32_convert_i32_s
   ,(valid: true;  Param: ipNone;      align: 0)  // B3  f32_convert_i32_u
   ,(valid: true;  Param: ipNone;      align: 0)  // B4  f32_convert_i64_s
   ,(valid: true;  Param: ipNone;      align: 0)  // B5  f32_convert_i64_u
   ,(valid: true;  Param: ipNone;      align: 0)  // B6  f32_demote_f64
   ,(valid: true;  Param: ipNone;      align: 0)  // B7  f64_convert_i32_s
   ,(valid: true;  Param: ipNone;      align: 0)  // B8  f64_convert_i32_u
   ,(valid: true;  Param: ipNone;      align: 0)  // B9  f64_convert_i64_s
   ,(valid: true;  Param: ipNone;      align: 0)  // BA  f64_convert_i64_u
   ,(valid: true;  Param: ipNone;      align: 0)  // BB  f64_promote_f32
   ,(valid: true;  Param: ipNone;      align: 0)  // BC  i32_reinterpret_f32
   ,(valid: true;  Param: ipNone;      align: 0)  // BD  i64_reinterpret_f64
   ,(valid: true;  Param: ipNone;      align: 0)  // BE  f32_reinterpret_i32
   ,(valid: true;  Param: ipNone;      align: 0)  // BF  f64_reinterpret_i64
  );

function InstLen(st: TStream; endOfInst: Byte = INST_END): Integer;

implementation

function InstLen(st: TStream; endOfInst: Byte = INST_END): Integer;
var
  cd  : byte;
  ofs : int64;
  b   : byte;
  sz  : int64;
begin
  ofs := st.Position;
  try
    sz:=st.Size;
    while ofs < sz do begin
      cd := st.ReadByte;

      if (cd > MAX_INST) then begin
        Result:=-1; // invalid code
        Exit;
      end;
      if cd = endOfInst then break;

      case INST_FLAGS[cd].Param of
        ipLeb:
          ReadU(st);
        ipOfsAlign: begin
          ReadU(st);
          ReadU(st);
        end;
        ipJumpVec: begin  // not implemented :(
          Result:=-2;
          Exit;
        end;
        ipi32: ReadS(st, 32);
        ipi64: ReadS(st, 64);
        ipf32: st.Position:=st.Position+4;
        ipf64: st.Position:=st.Position+8;
        ipResType: begin
          // it's a block. must go into recursion
          b := st.ReadByte; // reading type

          if (cd=INST_IF) and (b <> VALTYPE_NONE) then begin
            InstLen(st, INST_ELSE);
            InstLen(st, INST_END);
          end else
            InstLen(st, INST_END)
        end;
      end;
    end;
  finally
    Result := st.Position - ofs;
    st.Position:=ofs;
  end;
end;

end.
