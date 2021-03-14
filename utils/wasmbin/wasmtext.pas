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

unit wasmtext;

interface

// WebAssembly Wabt text utilities

{$mode objfpc}{$H+}

uses
 wasmbincode;

type
  TInstText = record
    text : string;
    inst : byte;
  end;

const
  WasmTextToInstr : array [MIN_INST..MAX_INST] of TInstText = (
   (text: 'unreachable'         ; inst: inst_unreachable )
  ,(text: 'nop'                 ; inst: inst_nop         )
  ,(text: 'block'               ; inst: inst_block       )
  ,(text: 'loop'                ; inst: inst_loop        )
  ,(text: 'if'                  ; inst: inst_if          )
  ,(text: 'else'                ; inst: inst_else        )
  ,(text: ''                    ; inst: $06)
  ,(text: ''                    ; inst: $07)
  ,(text: ''                    ; inst: $08)
  ,(text: ''                    ; inst: $09)
  ,(text: ''                    ; inst: $0a)
  ,(text: 'end'                 ; inst: inst_end      )
  ,(text: 'br'                  ; inst: inst_br)
  ,(text: 'br_if'               ; inst: inst_br_if)
  ,(text: 'br_table'            ; inst: inst_br_table)
  ,(text: 'return'              ; inst: inst_return)
  ,(text: 'call'                ; inst: inst_call)
  ,(text: 'call_indirect'       ; inst: inst_call_indirect)
  ,(text: ''                    ; inst: $12)
  ,(text: ''                    ; inst: $13)
  ,(text: ''                    ; inst: $14)
  ,(text: ''                    ; inst: $15)
  ,(text: ''                    ; inst: $16)
  ,(text: ''                    ; inst: $17)
  ,(text: ''                    ; inst: $18)
  ,(text: ''                    ; inst: $19)
  ,(text: 'drop'                ; inst: inst_drop     )
  ,(text: 'select'              ; inst: inst_select   )
  ,(text: ''                    ; inst: $1c)
  ,(text: ''                    ; inst: $1d)
  ,(text: ''                    ; inst: $1e)
  ,(text: ''                    ; inst: $1f)
  ,(text: 'local.get'           ; inst: inst_local_get)
  ,(text: 'local.set'           ; inst: inst_local_set)
  ,(text: 'local.tee'           ; inst: inst_local_tee)
  ,(text: 'global.get'          ; inst: inst_global_get)
  ,(text: 'global.set'          ; inst: inst_global_set)
  ,(text: ''                    ; inst: $25)
  ,(text: ''                    ; inst: $26)
  ,(text: ''                    ; inst: $27)
  ,(text: 'i32.load'            ; inst: inst_i32_load)
  ,(text: 'i64.load'            ; inst: inst_i64_load)
  ,(text: 'f32.load'            ; inst: inst_f32_load)
  ,(text: 'f64.load'            ; inst: inst_f64_load)
  ,(text: 'i32.load8_s'         ; inst: inst_i32_load8_s)
  ,(text: 'i32.load8_u'         ; inst: inst_i32_load8_u)
  ,(text: 'i32.load16_s'        ; inst: inst_i32_load16_s)
  ,(text: 'i32.load16_u'        ; inst: inst_i32_load16_u)
  ,(text: 'i64.load8_s'         ; inst: inst_i64_load8_s)
  ,(text: 'i64.load8_u'         ; inst: inst_i64_load8_u)
  ,(text: 'i64.load16_s'        ; inst: inst_i64_load16_s)
  ,(text: 'i64.load16_u'        ; inst: inst_i64_load16_u)
  ,(text: 'i64.load32_s'        ; inst: inst_i64_load32_s)
  ,(text: 'i64.load32_u'        ; inst: inst_i64_load32_u)
  ,(text: 'i32.store'           ; inst: inst_i32_store)
  ,(text: 'i64.store'           ; inst: inst_i64_store)
  ,(text: 'f32.store'           ; inst: inst_f32_store)
  ,(text: 'f64.store'           ; inst: inst_f64_store)
  ,(text: 'i32.store8'          ; inst: inst_i32_store8 )
  ,(text: 'i32.store16'         ; inst: inst_i32_store16)
  ,(text: 'i64.store8'          ; inst: inst_i64_store8 )
  ,(text: 'i64.store16'         ; inst: inst_i64_store16)
  ,(text: 'i64.store32'         ; inst: inst_i64_store32)
  ,(text: 'memory.size'         ; inst: inst_memory_size)
  ,(text: 'memory.grow'         ; inst: inst_memory_grow)
  ,(text: 'i32.const'           ; inst: inst_i32_const)
  ,(text: 'i64.const'           ; inst: inst_i64_const)
  ,(text: 'f32.const'           ; inst: inst_f32_const)
  ,(text: 'f64.const'           ; inst: inst_f64_const)

  ,(text: 'i32.eqz'             ; inst: inst_i32_eqz)
  ,(text: 'i32.eq'              ; inst: inst_i32_eq )
  ,(text: 'i32.ne'              ; inst: inst_i32_ne )
  ,(text: 'i32.lt_s'            ; inst: inst_i32_lt_s)
  ,(text: 'i32.lt_u'            ; inst: inst_i32_lt_u)
  ,(text: 'i32.gt_s'            ; inst: inst_i32_gt_s)
  ,(text: 'i32.gt_u'            ; inst: inst_i32_gt_u)
  ,(text: 'i32.le_s'            ; inst: inst_i32_le_s)
  ,(text: 'i32.le_u'            ; inst: inst_i32_le_u)
  ,(text: 'i32.ge_s'            ; inst: inst_i32_ge_s)
  ,(text: 'i32.ge_u'            ; inst: inst_i32_ge_u)
  ,(text: 'i64.eqz'             ; inst: inst_i64_eqz)
  ,(text: 'i64.eq'              ; inst: inst_i64_eq)
  ,(text: 'i64.ne'              ; inst: inst_i64_ne)
  ,(text: 'i64.lt_s'            ; inst: inst_i64_lt_s)
  ,(text: 'i64.lt_u'            ; inst: inst_i64_lt_u)
  ,(text: 'i64.gt_s'            ; inst: inst_i64_gt_s)
  ,(text: 'i64.gt_u'            ; inst: inst_i64_gt_u)
  ,(text: 'i64.le_s'            ; inst: inst_i64_le_s)
  ,(text: 'i64.le_u'            ; inst: inst_i64_le_u)
  ,(text: 'i64.ge_s'            ; inst: inst_i64_ge_s)
  ,(text: 'i64.ge_u'            ; inst: inst_i64_ge_u)
  ,(text: 'f32.eq'              ; inst: inst_f32_eq)
  ,(text: 'f32.ne'              ; inst: inst_f32_ne)
  ,(text: 'f32.lt'              ; inst: inst_f32_lt)
  ,(text: 'f32.gt'              ; inst: inst_f32_gt)
  ,(text: 'f32.le'              ; inst: inst_f32_le)
  ,(text: 'f32.ge'              ; inst: inst_f32_ge)
  ,(text: 'f64.eq'              ; inst: inst_f64_eq)
  ,(text: 'f64.ne'              ; inst: inst_f64_ne)
  ,(text: 'f64.lt'              ; inst: inst_f64_lt)
  ,(text: 'f64.gt'              ; inst: inst_f64_gt)
  ,(text: 'f64.le'              ; inst: inst_f64_le)
  ,(text: 'f64.ge'              ; inst: inst_f64_ge)

  ,(text: 'i32.clz'             ; inst: inst_i32_clz)
  ,(text: 'i32.ctz'             ; inst: inst_i32_ctz)
  ,(text: 'i32.popcnt'          ; inst: inst_i32_popcnt)
  ,(text: 'i32.add'             ; inst: inst_i32_add)
  ,(text: 'i32.sub'             ; inst: inst_i32_sub)
  ,(text: 'i32.mul'             ; inst: inst_i32_mul)
  ,(text: 'i32.div_s'           ; inst: inst_i32_div_s)
  ,(text: 'i32.div_u'           ; inst: inst_i32_div_u)
  ,(text: 'i32.rem_s'           ; inst: inst_i32_rem_s)
  ,(text: 'i32.rem_u'           ; inst: inst_i32_rem_u)
  ,(text: 'i32.and'             ; inst: inst_i32_and)
  ,(text: 'i32.or'              ; inst: inst_i32_or )
  ,(text: 'i32.xor'             ; inst: inst_i32_xor)
  ,(text: 'i32.shl'             ; inst: inst_i32_shl)
  ,(text: 'i32.shr_s'           ; inst: inst_i32_shr_s)
  ,(text: 'i32.shr_u'           ; inst: inst_i32_shr_u)
  ,(text: 'i32.rotl'            ; inst: inst_i32_rotl)
  ,(text: 'i32.rotr'            ; inst: inst_i32_rotr)
  ,(text: 'i64.clz'             ; inst: inst_i64_clz)
  ,(text: 'i64.ctz'             ; inst: inst_i64_ctz)
  ,(text: 'i64.popcnt'          ; inst: inst_i64_popcnt)
  ,(text: 'i64.add'             ; inst: inst_i64_add)
  ,(text: 'i64.sub'             ; inst: inst_i64_sub)
  ,(text: 'i64.mul'             ; inst: inst_i64_mul)
  ,(text: 'i64.div_s'           ; inst: inst_i64_div_s)
  ,(text: 'i64.div_u'           ; inst: inst_i64_div_u)
  ,(text: 'i64.rem_s'           ; inst: inst_i64_rem_s)
  ,(text: 'i64.rem_u'           ; inst: inst_i64_rem_u)
  ,(text: 'i64.and'             ; inst: inst_i64_and)
  ,(text: 'i64.or'              ; inst: inst_i64_or )
  ,(text: 'i64.xor'             ; inst: inst_i64_xor)
  ,(text: 'i64.shl'             ; inst: inst_i64_shl)
  ,(text: 'i64.shr_s'           ; inst: inst_i64_shr_s)
  ,(text: 'i64.shr_u'           ; inst: inst_i64_shr_u)
  ,(text: 'i64.rotl'            ; inst: inst_i64_rotl)
  ,(text: 'i64.rotr'            ; inst: inst_i64_rotr)
  ,(text: 'f32.abs'             ; inst: inst_f32_abs )
  ,(text: 'f32.neg'             ; inst: inst_f32_neg )
  ,(text: 'f32.ceil'            ; inst: inst_f32_ceil)
  ,(text: 'f32.floor'           ; inst: inst_f32_floor)
  ,(text: 'f32.trunc'           ; inst: inst_f32_trunc)
  ,(text: 'f32.nearest'         ; inst: inst_f32_nearest)
  ,(text: 'f32.sqrt'            ; inst: inst_f32_sqrt)
  ,(text: 'f32.add'             ; inst: inst_f32_add)
  ,(text: 'f32.sub'             ; inst: inst_f32_sub)
  ,(text: 'f32.mul'             ; inst: inst_f32_mul)
  ,(text: 'f32.div'             ; inst: inst_f32_div)
  ,(text: 'f32.min'             ; inst: inst_f32_min)
  ,(text: 'f32.max'             ; inst: inst_f32_max)
  ,(text: 'f32.copysign'        ; inst: inst_f32_copysign)
  ,(text: 'f64.abs'             ; inst: inst_f64_abs )
  ,(text: 'f64.neg'             ; inst: inst_f64_neg )
  ,(text: 'f64.ceil'            ; inst: inst_f64_ceil)
  ,(text: 'f64.floor'           ; inst: inst_f64_floor)
  ,(text: 'f64.trunc'           ; inst: inst_f64_trunc)
  ,(text: 'f64.nearest'         ; inst: inst_f64_nearest)
  ,(text: 'f64.sqrt'            ; inst: inst_f64_sqrt)
  ,(text: 'f64.add'             ; inst: inst_f64_add)
  ,(text: 'f64.sub'             ; inst: inst_f64_sub)
  ,(text: 'f64.mul'             ; inst: inst_f64_mul)
  ,(text: 'f64.div'             ; inst: inst_f64_div)
  ,(text: 'f64.min'             ; inst: inst_f64_min)
  ,(text: 'f64.max'             ; inst: inst_f64_max)
  ,(text: 'f64.copysign'        ; inst: inst_f64_copysign)

  ,(text: 'i32.wrap_i64'        ; inst: inst_i32_wrap_i64)
  ,(text: 'i32.trunc_f32_s'     ; inst: inst_i32_trunc_f32_s)
  ,(text: 'i32.trunc_f32_u'     ; inst: inst_i32_trunc_f32_u)
  ,(text: 'i32.trunc_f64_s'     ; inst: inst_i32_trunc_f64_s)
  ,(text: 'i32.trunc_f64_u'     ; inst: inst_i32_trunc_f64_u)
  ,(text: 'i64.extend_i32_s'    ; inst: inst_i64_extend_i32_s)
  ,(text: 'i64.extend_i32_u'    ; inst: inst_i64_extend_i32_u)
  ,(text: 'i64.trunc_f32_s'     ; inst: inst_i64_trunc_f32_s)
  ,(text: 'i64.trunc_f32_u'     ; inst: inst_i64_trunc_f32_u)
  ,(text: 'i64.trunc_f64_s'     ; inst: inst_i64_trunc_f64_s)
  ,(text: 'i64.trunc_f64_u'     ; inst: inst_i64_trunc_f64_u)
  ,(text: 'f32.convert_i32_s'   ; inst: inst_f32_convert_i32_s)
  ,(text: 'f32.convert_i32_u'   ; inst: inst_f32_convert_i32_u)
  ,(text: 'f32.convert_i64_s'   ; inst: inst_f32_convert_i64_s)
  ,(text: 'f32.convert_i64_u'   ; inst: inst_f32_convert_i64_u)
  ,(text: 'f32.demote_f64'      ; inst: inst_f32_demote_f64)
  ,(text: 'f64.convert_i32_s'   ; inst: inst_f64_convert_i32_s)
  ,(text: 'f64.convert_i32_u'   ; inst: inst_f64_convert_i32_u)
  ,(text: 'f64.convert_i64_s'   ; inst: inst_f64_convert_i64_s)
  ,(text: 'f64.convert_i64_u'   ; inst: inst_f64_convert_i64_u)
  ,(text: 'f64.promote_f32'     ; inst: inst_f64_promote_f32)
  ,(text: 'i32.reinterpret_f32' ; inst: inst_i32_reinterpret_f32)
  ,(text: 'i64.reinterpret_f64' ; inst: inst_i64_reinterpret_f64)
  ,(text: 'f32.reinterpret_i32' ; inst: inst_f32_reinterpret_i32)
  ,(text: 'f64.reinterpret_i64' ; inst: inst_f64_reinterpret_i64)
  );

function TextToInst(const t: string; out inst: byte): Boolean;

implementation

function floatTextToInst(const t: string; var inst: byte): Boolean;
begin
  Result := length(t)>4;
  if not Result then Exit;

  if (t[2]='3') and (t[3]='2') then begin
         if t = 'f32.load'            then inst := inst_f32_load
    else if t = 'f32.store'           then inst := inst_f32_store
    else if t = 'f32.const'           then inst := inst_f32_const
    else if t = 'f32.eq'              then inst := inst_f32_eq
    else if t = 'f32.ne'              then inst := inst_f32_ne
    else if t = 'f32.lt'              then inst := inst_f32_lt
    else if t = 'f32.gt'              then inst := inst_f32_gt
    else if t = 'f32.le'              then inst := inst_f32_le
    else if t = 'f32.ge'              then inst := inst_f32_ge
    else if t = 'f32.abs'             then inst := inst_f32_abs
    else if t = 'f32.neg'             then inst := inst_f32_neg
    else if t = 'f32.ceil'            then inst := inst_f32_ceil
    else if t = 'f32.floor'           then inst := inst_f32_floor
    else if t = 'f32.trunc'           then inst := inst_f32_trunc
    else if t = 'f32.nearest'         then inst := inst_f32_nearest
    else if t = 'f32.sqrt'            then inst := inst_f32_sqrt
    else if t = 'f32.add'             then inst := inst_f32_add
    else if t = 'f32.sub'             then inst := inst_f32_sub
    else if t = 'f32.mul'             then inst := inst_f32_mul
    else if t = 'f32.div'             then inst := inst_f32_div
    else if t = 'f32.min'             then inst := inst_f32_min
    else if t = 'f32.max'             then inst := inst_f32_max
    else if t = 'f32.copysign'        then inst := inst_f32_copysign
    else if t = 'f32.convert_i32_s'   then inst := inst_f32_convert_i32_s
    else if t = 'f32.convert_s/i32'   then inst := inst_f32_convert_i32_s
    else if t = 'f32.convert_i32_u'   then inst := inst_f32_convert_i32_u
    else if t = 'f32.convert_u/i32'   then inst := inst_f32_convert_i32_u
    else if t = 'f32.convert_i64_s'   then inst := inst_f32_convert_i64_s
    else if t = 'f32.convert_s/i64'   then inst := inst_f32_convert_i64_s
    else if t = 'f32.convert_i64_u'   then inst := inst_f32_convert_i64_u
    else if t = 'f32.convert_u/i64'   then inst := inst_f32_convert_i64_u
    else if t = 'f32.demote_f64'      then inst := inst_f32_demote_f64
    else if t = 'f32.demote/f64'      then inst := inst_f32_demote_f64
    else if t = 'f32.reinterpret_i32' then inst := inst_f32_reinterpret_i32
    else if t = 'f32.reinterpret/i32' then inst := inst_f32_reinterpret_i32
    else Result := false;

  end else if (t[2]='6') and (t[3]='4') then begin
         if t = 'f64.load'            then inst := inst_f64_load
    else if t = 'f64.store'           then inst := inst_f64_store
    else if t = 'f64.const'           then inst := inst_f64_const
    else if t = 'f64.eq'              then inst := inst_f64_eq
    else if t = 'f64.ne'              then inst := inst_f64_ne
    else if t = 'f64.lt'              then inst := inst_f64_lt
    else if t = 'f64.gt'              then inst := inst_f64_gt
    else if t = 'f64.le'              then inst := inst_f64_le
    else if t = 'f64.ge'              then inst := inst_f64_ge
    else if t = 'f64.abs'             then inst := inst_f64_abs
    else if t = 'f64.neg'             then inst := inst_f64_neg
    else if t = 'f64.ceil'            then inst := inst_f64_ceil
    else if t = 'f64.floor'           then inst := inst_f64_floor
    else if t = 'f64.trunc'           then inst := inst_f64_trunc
    else if t = 'f64.nearest'         then inst := inst_f64_nearest
    else if t = 'f64.sqrt'            then inst := inst_f64_sqrt
    else if t = 'f64.add'             then inst := inst_f64_add
    else if t = 'f64.sub'             then inst := inst_f64_sub
    else if t = 'f64.mul'             then inst := inst_f64_mul
    else if t = 'f64.div'             then inst := inst_f64_div
    else if t = 'f64.min'             then inst := inst_f64_min
    else if t = 'f64.max'             then inst := inst_f64_max
    else if t = 'f64.copysign'        then inst := inst_f64_copysign
    else if t = 'f64.convert_i32_s'   then inst := inst_f64_convert_i32_s
    else if t = 'f64.convert_s/i32'   then inst := inst_f64_convert_i32_s
    else if t = 'f64.convert_i32_u'   then inst := inst_f64_convert_i32_u
    else if t = 'f64.convert_u/i32'   then inst := inst_f64_convert_i32_u
    else if t = 'f64.convert_i64_s'   then inst := inst_f64_convert_i64_s
    else if t = 'f64.convert_s/i64'   then inst := inst_f64_convert_i64_s
    else if t = 'f64.convert_i64_u'   then inst := inst_f64_convert_i64_u
    else if t = 'f64.convert_u/i64'   then inst := inst_f64_convert_i64_u
    else if t = 'f64.promote_f32'     then inst := inst_f64_promote_f32
    else if t = 'f64.promote/f32'     then inst := inst_f64_promote_f32
    else if t = 'f64.reinterpret_i64' then inst := inst_f64_reinterpret_i64
    else if t = 'f64.reinterpret/i64' then inst := inst_f64_reinterpret_i64
    else Result := false;
  end;
end;

function intTextToInst(const t: string; var inst: byte): Boolean;
begin
  Result := length(t)>4;
  if not Result then Exit;

  if (t[2]='3') and (t[3]='2') then begin
         if t = 'i32.load'            then inst := inst_i32_load
    else if t = 'i32.load8_s'         then inst := inst_i32_load8_s
    else if t = 'i32.load8_u'         then inst := inst_i32_load8_u
    else if t = 'i32.load16_s'        then inst := inst_i32_load16_s
    else if t = 'i32.load16_u'        then inst := inst_i32_load16_u
    else if t = 'i32.store'           then inst := inst_i32_store
    else if t = 'i32.store8'          then inst := inst_i32_store8
    else if t = 'i32.store16'         then inst := inst_i32_store16
    else if t = 'i32.const'           then inst := inst_i32_const
    else if t = 'i32.eqz'             then inst := inst_i32_eqz
    else if t = 'i32.eq'              then inst := inst_i32_eq
    else if t = 'i32.ne'              then inst := inst_i32_ne
    else if t = 'i32.lt_s'            then inst := inst_i32_lt_s
    else if t = 'i32.lt_u'            then inst := inst_i32_lt_u
    else if t = 'i32.gt_s'            then inst := inst_i32_gt_s
    else if t = 'i32.gt_u'            then inst := inst_i32_gt_u
    else if t = 'i32.le_s'            then inst := inst_i32_le_s
    else if t = 'i32.le_u'            then inst := inst_i32_le_u
    else if t = 'i32.ge_s'            then inst := inst_i32_ge_s
    else if t = 'i32.ge_u'            then inst := inst_i32_ge_u
    else if t = 'i32.clz'             then inst := inst_i32_clz
    else if t = 'i32.ctz'             then inst := inst_i32_ctz
    else if t = 'i32.popcnt'          then inst := inst_i32_popcnt
    else if t = 'i32.add'             then inst := inst_i32_add
    else if t = 'i32.sub'             then inst := inst_i32_sub
    else if t = 'i32.mul'             then inst := inst_i32_mul
    else if t = 'i32.div_s'           then inst := inst_i32_div_s
    else if t = 'i32.div_u'           then inst := inst_i32_div_u
    else if t = 'i32.rem_s'           then inst := inst_i32_rem_s
    else if t = 'i32.rem_u'           then inst := inst_i32_rem_u
    else if t = 'i32.and'             then inst := inst_i32_and
    else if t = 'i32.or'              then inst := inst_i32_or
    else if t = 'i32.xor'             then inst := inst_i32_xor
    else if t = 'i32.shl'             then inst := inst_i32_shl
    else if t = 'i32.shr_s'           then inst := inst_i32_shr_s
    else if t = 'i32.shr_u'           then inst := inst_i32_shr_u
    else if t = 'i32.rotl'            then inst := inst_i32_rotl
    else if t = 'i32.rotr'            then inst := inst_i32_rotr
    else if t = 'i32.wrap_i64'        then inst := inst_i32_wrap_i64
    else if t = 'i32.wrap/i64'        then inst := inst_i32_wrap_i64
    else if t = 'i32.trunc_f32_s'     then inst := inst_i32_trunc_f32_s
    else if t = 'i32.trunc_s/f32'     then inst := inst_i32_trunc_f32_s
    else if t = 'i32.trunc_f32_u'     then inst := inst_i32_trunc_f32_u
    else if t = 'i32.trunc_u/f32'     then inst := inst_i32_trunc_f32_u
    else if t = 'i32.trunc_f64_s'     then inst := inst_i32_trunc_f64_s
    else if t = 'i32.trunc_s/f64'     then inst := inst_i32_trunc_f64_s
    else if t = 'i32.trunc_f64_u'     then inst := inst_i32_trunc_f64_u
    else if t = 'i32.trunc_u/f64'     then inst := inst_i32_trunc_f64_u
    else if t = 'i32.reinterpret_f32' then inst := inst_i32_reinterpret_f32
    else if t = 'i32.reinterpret/f32' then inst := inst_i32_reinterpret_f32
    else Result := false;
  end else if (t[2]='6') and (t[3]='4') then begin
         if t = 'i64.load8_s'         then inst := inst_i64_load8_s
    else if t = 'i64.load8_u'         then inst := inst_i64_load8_u
    else if t = 'i64.load16_s'        then inst := inst_i64_load16_s
    else if t = 'i64.load16_u'        then inst := inst_i64_load16_u
    else if t = 'i64.load32_s'        then inst := inst_i64_load32_s
    else if t = 'i64.load32_u'        then inst := inst_i64_load32_u
    else if t = 'i64.store'           then inst := inst_i64_store
    else if t = 'i64.store8'          then inst := inst_i64_store8
    else if t = 'i64.store16'         then inst := inst_i64_store16
    else if t = 'i64.store32'         then inst := inst_i64_store32
    else if t = 'i64.const'           then inst := inst_i64_const
    else if t = 'i64.eqz'             then inst := inst_i64_eqz
    else if t = 'i64.eq'              then inst := inst_i64_eq
    else if t = 'i64.ne'              then inst := inst_i64_ne
    else if t = 'i64.lt_s'            then inst := inst_i64_lt_s
    else if t = 'i64.lt_u'            then inst := inst_i64_lt_u
    else if t = 'i64.gt_s'            then inst := inst_i64_gt_s
    else if t = 'i64.gt_u'            then inst := inst_i64_gt_u
    else if t = 'i64.le_s'            then inst := inst_i64_le_s
    else if t = 'i64.le_u'            then inst := inst_i64_le_u
    else if t = 'i64.ge_s'            then inst := inst_i64_ge_s
    else if t = 'i64.ge_u'            then inst := inst_i64_ge_u
    else if t = 'i64.clz'             then inst := inst_i64_clz
    else if t = 'i64.ctz'             then inst := inst_i64_ctz
    else if t = 'i64.popcnt'          then inst := inst_i64_popcnt
    else if t = 'i64.add'             then inst := inst_i64_add
    else if t = 'i64.sub'             then inst := inst_i64_sub
    else if t = 'i64.mul'             then inst := inst_i64_mul
    else if t = 'i64.div_s'           then inst := inst_i64_div_s
    else if t = 'i64.div_u'           then inst := inst_i64_div_u
    else if t = 'i64.rem_s'           then inst := inst_i64_rem_s
    else if t = 'i64.rem_u'           then inst := inst_i64_rem_u
    else if t = 'i64.and'             then inst := inst_i64_and
    else if t = 'i64.or'              then inst := inst_i64_or
    else if t = 'i64.xor'             then inst := inst_i64_xor
    else if t = 'i64.shl'             then inst := inst_i64_shl
    else if t = 'i64.shr_s'           then inst := inst_i64_shr_s
    else if t = 'i64.shr_u'           then inst := inst_i64_shr_u
    else if t = 'i64.rotl'            then inst := inst_i64_rotl
    else if t = 'i64.rotr'            then inst := inst_i64_rotr
    else if t = 'i64.extend_i32_s'    then inst := inst_i64_extend_i32_s
    else if t = 'i64.extend_s/i32'    then inst := inst_i64_extend_i32_s
    else if t = 'i64.extend_i32_u'    then inst := inst_i64_extend_i32_u
    else if t = 'i64.extend_u/i32'    then inst := inst_i64_extend_i32_u
    else if t = 'i64.trunc_f32_s'     then inst := inst_i64_trunc_f32_s
    else if t = 'i64.trunc_s/f32'     then inst := inst_i64_trunc_f32_s
    else if t = 'i64.trunc_f32_u'     then inst := inst_i64_trunc_f32_u
    else if t = 'i64.trunc_u/f32'     then inst := inst_i64_trunc_f32_u
    else if t = 'i64.trunc_f64_s'     then inst := inst_i64_trunc_f64_s
    else if t = 'i64.trunc_s/f64'     then inst := inst_i64_trunc_f64_s
    else if t = 'i64.trunc_f64_u'     then inst := inst_i64_trunc_f64_u
    else if t = 'i64.trunc_u/f64'     then inst := inst_i64_trunc_f64_u
    else if t = 'i64.load'            then inst := inst_i64_load
    else if t = 'i64.reinterpret_f64' then inst := inst_i64_reinterpret_f64
    else if t = 'i64.reinterpret/f64' then inst := inst_i64_reinterpret_f64
    else Result := false;
  end else
    Result := false;
end;

function TextToInst(const t: string; out inst: byte): Boolean;
begin
  inst:=0;
  Result := length(t)>0;
  if not Result then Exit;

  case t[1] of
  'b':
    if t = 'block'    then inst := inst_block
    else if t = 'br'       then inst := inst_br
    else if t = 'br_if'    then inst := inst_br_if
    else if t = 'br_table' then inst := inst_br_table
    else Result := false;
  'c':
    if t = 'call'          then inst := inst_call
    else if t = 'call_indirect' then inst := inst_call_indirect
    else if t = 'current_memory' then inst := INST_memory_size
    else Result := false;
  'd':
    if t = 'drop' then inst := inst_drop
    else Result := false;
  'e':
    if t = 'else' then inst := inst_else
    else if t = 'end'  then inst := inst_end
    else Result := false;
  'f':
    Result := floatTextToInst(t, inst);
  'g':
    if t = 'global.get' then inst := inst_global_get
    else if t = 'global.set' then inst := inst_global_set
    // wabt
    else if t = 'get_local' then inst := INST_local_get
    else if t = 'get_global' then inst := INST_global_get
    else if t = 'grow_memory' then inst := inst_memory_grow
    else Result := false;
  'i':
    if t = 'if' then inst := inst_if
    else Result := intTextToInst(t, inst);
  'l':
    if t = 'local.get' then inst := inst_local_get
    else if t = 'local.set' then inst := inst_local_set
    else if t = 'local.tee' then inst := inst_local_tee
    else if t = 'loop'      then inst := inst_loop
    else Result := false;
  'm':
    if t = 'memory.size' then inst := inst_memory_size
    else if t = 'memory.grow' then inst := inst_memory_grow
    else Result := false;
  'n':
    if t = 'nop' then inst := inst_nop
    else Result := false;
  'r':
    if t = 'return' then inst := inst_return
    else Result := false;
  's':
    if t = 'select' then inst := inst_select
    // wabt
    else if t = 'set_local' then inst := INST_local_set
    else if t = 'set_global' then inst := INST_global_set
    else Result := false;
  't':
    if t = 'tee_local' then inst := INST_local_tee;
  'u':
    if t ='unreachable' then inst := inst_unreachable
    else Result := false;
  else
    Result := false;
  end;
end;

end.
