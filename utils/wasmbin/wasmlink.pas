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

unit wasmlink;
// The unit covers the WebAssembly static linking convention
// as described at https://github.com/WebAssembly/tool-conventions/blob/master/Linking.md

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, lebutils, wasmbin, wasmbincode;

const
  SectionName_Linking = 'linking';
  SectionNamePfx_Reloc = 'reloc.';

type
  TRelocationSection = record
    section : UInt32; // the index of the target section
    count   : Uint32; // count of entries to follow
  end;

  TRelocationEntry = record
    sec     : Uint8;  // section to be relocated at
    reltype : UInt8;  // the relocation type (see R_WASM constants)
    offset  : UInt32; // offset of the value to rewrite
    index   : Uint32; // the index of the symbol used (or, for R_WASM_TYPE_INDEX_LEB relocations, the index of the type)
  end;
  TRelocationEntryEx = record
    entry  : TRelocationEntry;
    addend : UInt32;
  end;

const
  // A relocation type can be one of the following:
  R_WASM_FUNCTION_INDEX_LEB  = 0;  // a function index encoded as a 5-byte varuint32. Used for the immediate argument of a call instruction.
  R_WASM_TABLE_INDEX_SLEB    = 1;  // a function table index encoded as a 5-byte varint32. Used to refer to the immediate argument of a i32.const instruction, e.g. taking the address of a function.
  R_WASM_TABLE_INDEX_I32     = 2;  // a function table index encoded as a uint32, e.g. taking the address of a function in a static data initializer.
  R_WASM_MEMORY_ADDR_LEB     = 3;  // a linear memory index encoded as a 5-byte varuint32. Used for the immediate argument of a load or store instruction, e.g. directly loading from or storing to a C++ global.
  R_WASM_MEMORY_ADDR_SLEB    = 4;  // a linear memory index encoded as a 5-byte varint32. Used for the immediate argument of a i32.const instruction, e.g. taking the address of a C++ global.
  R_WASM_MEMORY_ADDR_I32     = 5;  // a linear memory index encoded as a uint32, e.g. taking the address of a C++ global in a static data initializer.
  R_WASM_TYPE_INDEX_LEB      = 6;  // a type table index encoded as a 5-byte varuint32, e.g. the type immediate in a call_indirect.
  R_WASM_GLOBAL_INDEX_LEB    = 7;  // a global index encoded as a 5-byte varuint32, e.g. the index immediate in a get_global.
  R_WASM_FUNCTION_OFFSET_I32 = 8;  // a byte offset within code section for the specic function encoded as a uint32. The offsets start at the actual function code excluding its size field.
  R_WASM_SECTION_OFFSET_I32  = 9;  // an byte offset from start of the specified section encoded as a uint32.
  R_WASM_EVENT_INDEX_LEB     = 10; // an event index encoded as a 5-byte varuint32. Used for the immediate argument of a throw and if_except instruction.
  R_WASM_TABLE_NUMBER_LEB    = 13; // a table number encoded as a 5-byte varuint32. Used for the table immediate argument in the table.* instructions.

type
  TLinkingMetadata = record
    version : UInt32; // the version of linking metadata contained in this section. Currently: 2
  end;

  TLinkingSubSection = record
    sectype : UInt8;   // code identifying type of subsection
    length  : UInt32;  // size of this subsection in bytes
  end;

const
  LINKING_VERSION = 2;

  // The current list of valid TLinkinSubSection.sectype codes are:
  WASM_SEGMENT_INFO = 5; // Extra metadata about the data segments.
  WASM_INIT_FUNCS   = 6; // Specifies a list of constructor functions to be called at startup.
                         // These constructors will be called in priority order after memory
                         // has been initialized.
  WASM_COMDAT_INFO  = 7; // Specifies the COMDAT groups of associated linking objects,
                         // which are linked only once and all together.
  WASM_SYMBOL_TABLE = 8; // Specifies extra information about the symbols present in the module


type
  TSymInfo = record
    kind        : UInt8;
    flags       : UInt32;

    hasSymIndex : Boolean; // always true for Kind non Data
                           // for Data it's true for defined symbols (see flags);
    symindex    : UInt32;  // either symbol or data symbol offset
    hasSymName  : Boolean;
    symname     : string;
    dataofs     : integer; // only if Kind is Data and hasSymIndex = true;
    datasize    : integer;

  end;

// The symbol type. One of:
const
  SYMTAB_FUNCTION = 0;
  SYMTAB_DATA     = 1;
  SYMTAB_GLOBAL   = 2;
  SYMTAB_SECTION  = 3;
  SYMTAB_EVENT    = 4;
  SYMTAB_TABLE    = 5;

//  The current set of valid flags for symbols are:
const
  // Indicating that this is a weak symbol.  When linking multiple modules
  // defining the same symbol, all weak definitions are discarded if
  // any strong definitions exist; then if multiple weak definitions
  // exist all but one (unspecified) are discarded; and finally it is an error
  // if more than one definition remains.
  WASM_SYM_BINDING_WEAK      = $01;

  // Indicating that this is a local symbol (this is exclusive
  // with WASM_SYM_BINDING_WEAK). Local symbols are not to be exported,
  // or linked to other modules/sections. The names of all non-local
  // symbols must be unique, but the names of local symbols are
  // not considered for uniqueness. A local function or global
  // symbol cannot reference an import.
  WASM_SYM_BINDING_LOCAL     = $02;

  // Indicating that this is a hidden symbol. Hidden symbols are not to be
  // exported when performing the final link, but may be linked to other modules.
  WASM_SYM_VISIBILITY_HIDDEN = $04;

  // Indicating that this symbol is not defined. For non-data symbols,
  // this must match whether the symbol is an import or is defined;
  // for data symbols, determines whether a segment is specified.
  WASM_SYM_UNDEFINED         = $10;
  WASM_SYM_IMPORTED          = WASM_SYM_UNDEFINED;

  // The symbol is intended to be exported from the wasm module to the host
  // environment. This differs from the visibility flags in that it effects
  // the static linker.
  WASM_SYM_EXPORTED          = $20;

  // The symbol uses an explicit symbol name, rather than reusing the name
  // from a wasm import. This allows it to remap imports from foreign WebAssembly
  // modules into local symbols with different names.
  WASM_SYM_EXPLICIT_NAME     = $40;

  // The symbol is intended to be included in the linker output,
  // regardless of whether it is used by the program.
  WASM_SYM_NO_STRIP          = $80;

function ReadMetaData(st: TStream; out m:TLinkingMetadata): Boolean;
function ReadLinkSubSect(st: TStream; out m: TLinkingSubSection): Boolean;
function ReadSymInfo(st: TStream; out m: TSymInfo): Boolean;

procedure WriteSymInfo(st: TStream; const m: TSymInfo);

// dumps linking information. Note: that the name of the "Linking" section
// must have already been read
procedure DumpLinking(st: TStream; secsize: integer);

function SubSecTypeToStr(b: Byte): string;
function SymKindToStr(b: Byte): string;

type
  TLinkingSection = record
    metadata: TLinkingMetadata;
    symbols : array of TSymInfo;
  end;

// the stream should be set at the beggining of the section
// after name and size values
procedure ReadLinkingSection(st: TStream; size: integer; var sc: TLinkingSection);
procedure WriteLinkingSection(st: TStream; const sc: TLinkingSection);

type
  TInstRelocFlag = record
    doReloc: Boolean;
    relocType: byte;
  end;

const
  INST_RELOC_FLAGS : array [MIN_INST..MAX_INST] of TInstRelocFlag = (
    (doReloc: false; relocType: $FF)  // 00 trap (unreachable)
   ,(doReloc: false; relocType: $FF)  // 01 nop
   ,(doReloc: false; relocType: $FF)  // 02 block
   ,(doReloc: false; relocType: $FF)  // 03 lock
   ,(doReloc: false; relocType: $FF)  // 04 if
   ,(doReloc: false; relocType: $FF)  // 05
   ,(doReloc: false; relocType: $FF)  // 06
   ,(doReloc: false; relocType: $FF)  // 07
   ,(doReloc: false; relocType: $FF)  // 08
   ,(doReloc: false; relocType: $FF)  // 09
   ,(doReloc: false; relocType: $FF)  // 0A
   ,(doReloc: false; relocType: $FF)  // 0B  end
   ,(doReloc: false; relocType: $FF)  // 0C  br
   ,(doReloc: false; relocType: $FF)  // 0D  br_if
   ,(doReloc: false; relocType: $FF)  // 0E  br_table
   ,(doReloc: false; relocType: $FF)  // 0F  return
   ,(doReloc: true;  relocType: R_WASM_FUNCTION_INDEX_LEB) // 10  call
   ,(doReloc: true;  relocType: R_WASM_TYPE_INDEX_LEB)     // 11  call_indirect
   ,(doReloc: false; relocType: $FF)  // 12
   ,(doReloc: false; relocType: $FF)  // 13
   ,(doReloc: false; relocType: $FF)  // 14
   ,(doReloc: false; relocType: $FF)  // 15
   ,(doReloc: false; relocType: $FF)  // 16
   ,(doReloc: false; relocType: $FF)  // 17
   ,(doReloc: false; relocType: $FF)  // 18
   ,(doReloc: false; relocType: $FF)  // 19
   ,(doReloc: false; relocType: $FF)  // 1A  drop
   ,(doReloc: false; relocType: $FF)  // 1B  select
   ,(doReloc: false; relocType: $FF)  // 1C
   ,(doReloc: false; relocType: $FF)  // 1D
   ,(doReloc: false; relocType: $FF)  // 1E
   ,(doReloc: false; relocType: $FF)  // 1F
   ,(doReloc: false; relocType: $FF)  // 20  local.get
   ,(doReloc: false; relocType: $FF)  // 21  local.set
   ,(doReloc: false; relocType: $FF)  // 22  local.tee
   ,(doReloc: true;  relocType: R_WASM_GLOBAL_INDEX_LEB)  // 23  global.get
   ,(doReloc: true;  relocType: R_WASM_GLOBAL_INDEX_LEB)  // 24  global.set
   ,(doReloc: false; relocType: $FF)  // 25
   ,(doReloc: false; relocType: $FF)  // 26
   ,(doReloc: false; relocType: $FF)  // 27
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 28  i32.load
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 29  i64_load
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 2A  f32_load
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 2B  f64_load
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 2C  i32_load8_s
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 2D  i32_load8_u
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 2E  i32_load16_s
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 2F  i32_load16_u
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 30  i64_load8_s
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 31  i64_load8_u
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 32  i64_load16_s
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 33  i64_load16_u
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 34  i64.load32_s
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 35  i64.load32_u
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 36  i32_store
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 37  i64_store
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 38  f32_store
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 39  f64_store
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 3A  i32_store8
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 3B  i32_store16
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 3C  i64_store8
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 3D  i64_store16
   ,(doReloc: true;  relocType: R_WASM_MEMORY_ADDR_LEB)  // 3E  i64_store32
   ,(doReloc: false; relocType: $FF)  // 3F  memory_size
   ,(doReloc: false; relocType: $FF)  // 40  memory_grow

   ,(doReloc: true;  relocType: R_WASM_TABLE_INDEX_SLEB)  // 41  i32_const // TODO: special case for function address

   ,(doReloc: false; relocType: $FF)  // 42  i64_const
   ,(doReloc: false; relocType: $FF)  // 43  f32_const
   ,(doReloc: false; relocType: $FF)  // 44  f64_const
   ,(doReloc: false; relocType: $FF)  // 45  i32_eqz
   ,(doReloc: false; relocType: $FF)  // 46  i32_eq
   ,(doReloc: false; relocType: $FF)  // 47  i32_ne
   ,(doReloc: false; relocType: $FF)  // 48  i32_lt_s
   ,(doReloc: false; relocType: $FF)  // 49  i32_lt_u
   ,(doReloc: false; relocType: $FF)  // 4A  i32_gt_s
   ,(doReloc: false; relocType: $FF)  // 4B  i32_gt_u
   ,(doReloc: false; relocType: $FF)  // 4C  i32_le_s
   ,(doReloc: false; relocType: $FF)  // 4D  i32_le_u
   ,(doReloc: false; relocType: $FF)  // 4E  i32_ge_s
   ,(doReloc: false; relocType: $FF)  // 4F  i32_ge_u
   ,(doReloc: false; relocType: $FF)  // 50  i64_eqz
   ,(doReloc: false; relocType: $FF)  // 51  i64_eq
   ,(doReloc: false; relocType: $FF)  // 52  i64_ne
   ,(doReloc: false; relocType: $FF)  // 53  i64_lt_s
   ,(doReloc: false; relocType: $FF)  // 54  i64_lt_u
   ,(doReloc: false; relocType: $FF)  // 55  i64_gt_s
   ,(doReloc: false; relocType: $FF)  // 56  i64_gt_u
   ,(doReloc: false; relocType: $FF)  // 57  i64_le_s
   ,(doReloc: false; relocType: $FF)  // 58  i64_le_u
   ,(doReloc: false; relocType: $FF)  // 59  i64_ge_s
   ,(doReloc: false; relocType: $FF)  // 5A  i64_ge_u
   ,(doReloc: false; relocType: $FF)  // 5B  f32_eq
   ,(doReloc: false; relocType: $FF)  // 5C  f32_ne
   ,(doReloc: false; relocType: $FF)  // 5D  f32_lt
   ,(doReloc: false; relocType: $FF)  // 5E  f32_gt
   ,(doReloc: false; relocType: $FF)  // 5F  f32_le
   ,(doReloc: false; relocType: $FF)  // 60  f32_ge
   ,(doReloc: false; relocType: $FF)  // 61  f64_eq
   ,(doReloc: false; relocType: $FF)  // 62  f64_ne
   ,(doReloc: false; relocType: $FF)  // 63  f64_lt
   ,(doReloc: false; relocType: $FF)  // 64  f64_gt
   ,(doReloc: false; relocType: $FF)  // 65  f64_le
   ,(doReloc: false; relocType: $FF)  // 66  f64_ge
   ,(doReloc: false; relocType: $FF)  // 67  i32_clz
   ,(doReloc: false; relocType: $FF)  // 68  i32_ctz
   ,(doReloc: false; relocType: $FF)  // 69  i32_popcnt
   ,(doReloc: false; relocType: $FF)  // 6A  i32_add
   ,(doReloc: false; relocType: $FF)  // 6B  i32_sub
   ,(doReloc: false; relocType: $FF)  // 6C  i32_mul
   ,(doReloc: false; relocType: $FF)  // 6D  i32_div_s
   ,(doReloc: false; relocType: $FF)  // 6E  i32_div_u
   ,(doReloc: false; relocType: $FF)  // 6F  i32_rem_s
   ,(doReloc: false; relocType: $FF)  // 70  i32_rem_u
   ,(doReloc: false; relocType: $FF)  // 71  i32_and
   ,(doReloc: false; relocType: $FF)  // 72  i32_or
   ,(doReloc: false; relocType: $FF)  // 73  i32_xor
   ,(doReloc: false; relocType: $FF)  // 74  i32_shl
   ,(doReloc: false; relocType: $FF)  // 75  i32_shr_s
   ,(doReloc: false; relocType: $FF)  // 76  i32_shr_u
   ,(doReloc: false; relocType: $FF)  // 77  i32_rotl
   ,(doReloc: false; relocType: $FF)  // 78  i32_rotr
   ,(doReloc: false; relocType: $FF)  // 79  i64_clz
   ,(doReloc: false; relocType: $FF)  // 7A  i64_ctz
   ,(doReloc: false; relocType: $FF)  // 7B  i64_popcnt
   ,(doReloc: false; relocType: $FF)  // 7C  i64_add
   ,(doReloc: false; relocType: $FF)  // 7D  i64_sub
   ,(doReloc: false; relocType: $FF)  // 7E  i64_mul
   ,(doReloc: false; relocType: $FF)  // 7F  i64_div_s
   ,(doReloc: false; relocType: $FF)  // 80  i64_div_u
   ,(doReloc: false; relocType: $FF)  // 81  i64_rem_s
   ,(doReloc: false; relocType: $FF)  // 82  i64_rem_u
   ,(doReloc: false; relocType: $FF)  // 83  i64_and
   ,(doReloc: false; relocType: $FF)  // 84  i64_or
   ,(doReloc: false; relocType: $FF)  // 85  i64_xor
   ,(doReloc: false; relocType: $FF)  // 86  i64_shl
   ,(doReloc: false; relocType: $FF)  // 87  i64_shr_s
   ,(doReloc: false; relocType: $FF)  // 88  i64_shr_u
   ,(doReloc: false; relocType: $FF)  // 89  i64_rotl
   ,(doReloc: false; relocType: $FF)  // 8A  i64_rotr
   ,(doReloc: false; relocType: $FF)  // 8B  f32_abs
   ,(doReloc: false; relocType: $FF)  // 8C  f32_neg
   ,(doReloc: false; relocType: $FF)  // 8D  f32_ceil
   ,(doReloc: false; relocType: $FF)  // 8E  f32_floor
   ,(doReloc: false; relocType: $FF)  // 8F  f32_trunc
   ,(doReloc: false; relocType: $FF)  // 90  f32_nearest
   ,(doReloc: false; relocType: $FF)  // 91  f32_sqrt
   ,(doReloc: false; relocType: $FF)  // 92  f32_add
   ,(doReloc: false; relocType: $FF)  // 93  f32_sub
   ,(doReloc: false; relocType: $FF)  // 94  f32_mul
   ,(doReloc: false; relocType: $FF)  // 95  f32_div
   ,(doReloc: false; relocType: $FF)  // 96  f32_min
   ,(doReloc: false; relocType: $FF)  // 97  f32_max
   ,(doReloc: false; relocType: $FF)  // 98  f32_copysign
   ,(doReloc: false; relocType: $FF)  // 99  f64_abs
   ,(doReloc: false; relocType: $FF)  // 9A  f64_neg
   ,(doReloc: false; relocType: $FF)  // 9B  f64_ceil
   ,(doReloc: false; relocType: $FF)  // 9C  f64_floor
   ,(doReloc: false; relocType: $FF)  // 9D  f64_trunc
   ,(doReloc: false; relocType: $FF)  // 9E  f64_nearest
   ,(doReloc: false; relocType: $FF)  // 9F  f64_sqrt
   ,(doReloc: false; relocType: $FF)  // A0  f64_add
   ,(doReloc: false; relocType: $FF)  // A1  f64_sub
   ,(doReloc: false; relocType: $FF)  // A2  f64_mul
   ,(doReloc: false; relocType: $FF)  // A3  f64_div
   ,(doReloc: false; relocType: $FF)  // A4  f64_min
   ,(doReloc: false; relocType: $FF)  // A5  f64_max
   ,(doReloc: false; relocType: $FF)  // A6  f64_copysign
   ,(doReloc: false; relocType: $FF)  // A7  i32_wrap_i64
   ,(doReloc: false; relocType: $FF)  // A8  i32_trunc_f32_s
   ,(doReloc: false; relocType: $FF)  // A9  i32_trunc_f32_u
   ,(doReloc: false; relocType: $FF)  // AA  i32_trunc_f64_s
   ,(doReloc: false; relocType: $FF)  // AB  i32_trunc_f64_u
   ,(doReloc: false; relocType: $FF)  // AC  i64_extend_i32_s
   ,(doReloc: false; relocType: $FF)  // AD  i64_extend_i32_u
   ,(doReloc: false; relocType: $FF)  // AE  i64_trunc_f32_s
   ,(doReloc: false; relocType: $FF)  // AF  i64_trunc_f32_u
   ,(doReloc: false; relocType: $FF)  // B0  i64_trunc_f64_s
   ,(doReloc: false; relocType: $FF)  // B1  i64_trunc_f64_u
   ,(doReloc: false; relocType: $FF)  // B2  f32_convert_i32_s
   ,(doReloc: false; relocType: $FF)  // B3  f32_convert_i32_u
   ,(doReloc: false; relocType: $FF)  // B4  f32_convert_i64_s
   ,(doReloc: false; relocType: $FF)  // B5  f32_convert_i64_u
   ,(doReloc: false; relocType: $FF)  // B6  f32_demote_f64
   ,(doReloc: false; relocType: $FF)  // B7  f64_convert_i32_s
   ,(doReloc: false; relocType: $FF)  // B8  f64_convert_i32_u
   ,(doReloc: false; relocType: $FF)  // B9  f64_convert_i64_s
   ,(doReloc: false; relocType: $FF)  // BA  f64_convert_i64_u
   ,(doReloc: false; relocType: $FF)  // BB  f64_promote_f32
   ,(doReloc: false; relocType: $FF)  // BC  i32_reinterpret_f32
   ,(doReloc: false; relocType: $FF)  // BD  i64_reinterpret_f64
   ,(doReloc: false; relocType: $FF)  // BE  f32_reinterpret_i32
   ,(doReloc: false; relocType: $FF)  // BF  f64_reinterpret_i64
  );


implementation

function ReadMetaData(st: TStream; out m:TLinkingMetadata): Boolean;
begin
  FillChar(m, sizeof(m), 0);
  m.version := st.ReadByte;
  Result:=true;
end;

function ReadLinkSubSect(st: TStream; out m: TLinkingSubSection): Boolean;
begin
  FillChar(m, sizeof(m), 0);
  m.sectype := st.ReadByte; //ReadU(st);
  m.length := ReadU(st);
  Result := true;
end;

function ReadSymInfo(st: TStream; out m: TSymInfo): Boolean;
begin
  FillChar(m, sizeof(m), 0);
  m.kind := st.ReadByte;
  m.flags := ReadU(st);

  if m.kind = SYMTAB_DATA then begin
    m.hasSymName := true; // always exist
    m.symname := ReadName(st);

    m.hasSymIndex := (m.flags and WASM_SYM_UNDEFINED)=0;
    if m.hasSymIndex then begin
      m.symindex := ReadU(st);
      m.dataofs := ReadU(st);
      m.datasize := ReadU(st);
    end;

  end else begin
    m.hasSymIndex := true; // always exists
    m.symindex := ReadU(st);

    m.hasSymName := ((m.flags and WASM_SYM_IMPORTED) = 0) or ((m.flags and WASM_SYM_EXPLICIT_NAME) > 0); // imported
    if m.hasSymName then
      m.symname:=ReadName(st);
  end;

  Result := true;
end;

procedure WriteSymInfo(st: TStream; const m: TSymInfo);
begin
  st.WriteByte(m.kind);
  WriteU32(st, m.flags);

  if m.kind = SYMTAB_DATA then begin
    WriteName(st, m.symname);
    if (m.flags and WASM_SYM_UNDEFINED)=0 then begin
      WriteU32(st, m.symindex);
      WriteU32(st, m.dataofs);
      WriteU32(st, m.datasize);
    end;

  end else begin
    WriteU32(st, m.symindex);
    if ((m.flags and WASM_SYM_IMPORTED) = 0) or ((m.flags and WASM_SYM_EXPLICIT_NAME) > 0) then
      WriteName(st, m.symname);
  end;
end;

procedure DumpLinking(st: TStream; secsize: integer);
var
  mt  : TLinkingMetadata;
  en  : Int64;
  sub : TLinkingSubSection;
  cnt : LongWord;
  nx  : Int64;
  i   : integer;
  si  : TSymInfo;
begin
  en := st.Position+secsize;
  ReadMetadata(st, mt);
  writeln('version: ', mt.version);
  while st.Position<en do begin
    ReadLinkSubSect(st, sub);
    nx := st.Position+sub.length;

    writeln('subsec=',SubSecTypeToStr(sub.sectype),' ',sub.sectype);
    cnt := ReadU(st);
    writeln('- symbol table [count=', cnt,']');
    for i:=0 to cnt-1 do begin
      write('  - ',i,' ');
      ReadSymInfo(st, si);
      write(SymKindToStr(si.kind),' ',IntToHex(si.flags,8));
      if si.hasSymName then write(' ',si.symname);
      writeln;
      //writeln(si.symname);
    end;

    st.Position:=nx;
  end;
end;

function SubSecTypeToStr(b: Byte): string;
begin
  case b of
    WASM_SEGMENT_INFO: Result := 'WASM_SEGMENT_INFO';
    WASM_INIT_FUNCS:   Result := 'WASM_INIT_FUNCS';
    WASM_COMDAT_INFO:  Result := 'WASM_COMDAT_INFO';
    WASM_SYMBOL_TABLE: Result := 'WASM_SYMBOL_TABLE';
  else
    Result := Format('UNKNOWN %d',[b]);
  end;
end;

function SymKindToStr(b: Byte): string;
begin
  case b of
    SYMTAB_FUNCTION: Result := 'F';
    SYMTAB_DATA:     Result := 'D';
    SYMTAB_GLOBAL:   Result := 'G';
    SYMTAB_SECTION:  Result := 'S';
    SYMTAB_EVENT:    Result := 'E';
    SYMTAB_TABLE:    Result := 'T';
  else
    Result := 'U'+IntToStR(b);
  end;
end;

procedure ReadLinkingSection(st: TStream; size: integer; var sc: TLinkingSection);
var
  eofs : int64;
  sub  : TLinkingSubSection;
  cnt  : integer;
  i    : integer;
  nx   : int64;
begin
  eofs := st.Position+size;
  ReadMetadata(st, sc.metadata);
  while st.Position < eofs do begin
    ReadLinkSubSect(st, sub);
    nx := st.Position+sub.length;
    case sub.sectype of
      //todo: others!
      WASM_SYMBOL_TABLE: begin
        cnt := ReadU(st);
        SetLength(sc.symbols, cnt);
        for i:=0 to cnt-1 do
          ReadSymInfo(st, sc.symbols[i]);
      end;
    end;
    st.Position:=nx;
  end;
end;

procedure WriteLinkingSection(st: TStream; const sc: TLinkingSection);
var
  mem : TMemoryStream;
  i   : integer;
begin
  st.WriteByte(sc.metadata.version);

  mem:=TMemoryStream.Create;
  try
    WriteU32(mem, length(sc.symbols));
    for i:=0 to length(sc.symbols)-1 do
      WriteSymInfo(mem, sc.symbols[i]);

    st.WriteByte(WASM_SYMBOL_TABLE);
    WriteU32(st, mem.Size);

    mem.Position:=0;
    st.CopyFrom(mem, mem.Size);
  finally
    mem.Free;
  end;

  // todo: other sub setions are possible
end;

end.
