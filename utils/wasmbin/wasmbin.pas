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

unit wasmbin;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, lebutils, wasmbincode;

const
  valtype_i32   = $7f;
  valtype_i64   = $7e;
  valtype_f32   = $7d;
  valtype_f64   = $7C;

  block_type    = $40;
  func_type     = $60;
  elem_type     = $70;

  global_const  = $00;
  global_mut    = $01;

  limit_min_inf = $00; // minimum - to infinity
  limit_min_max = $01; // minimum - maximum

const
  WasmId = #0'asm';
  WasmId_Int = $6D736100;
  Wasm_Version1 = 1;
var
  WasmId_Buf  : array [0..3] of char = (#0, 'a','s','m');

type
  TLimit = record
    limitType : byte;
    min       : UInt32;
    max       : UInt32;
  end;
  TMemoryType = TLimit;

  TTableType = record
    elemType : Byte;   // see "elem_type"
    limits   : TLimit;
  end;

  TGlobalType = record
    valtype  : Byte;   // see "valtype_" consts
    mut      : Byte;   // see "global_" consts
  end;

const
  SECT_CUSTOM   = 0;	// custom section
  SECT_TYPE     = 1;	// type section
  SECT_IMPORT   = 2;	// import section
  SECT_FUNCTION = 3;	// function section
  SECT_TABLE    = 4;	// table section
  SECT_MEMORY   = 5;	// memory section
  SECT_GLOBAL   = 6;	// global section
  SECT_EXPORT   = 7;	// export section
  SECT_START    = 8;	// start section
  SECT_ELEMENT  = 9;	// element section
  SECT_CODE     = 10;	// code section
  SECT_DATA     = 11;	// data section

type
  TSection = packed record
    id   : byte;
    size : LongWord; // it is Leb128 encoded in the file, thus cannot be read directly
  end;

  TFuncType = record
    param  : array of byte;
    result : array of byte;
  end;

  TFuncTypeArray = record
    funTypes : array of TFuncType;
  end;

  TCodeLocalEntry = record
    count  : LongWord;
    valtyp : Byte;
  end;

  TCodeInstr = record
    inst : byte;
    idxArr: array of LongWord;
    case byte of
      0: (align, offset : LongWord);
      1: (index: LongWord);
      2: (i32: LongWord);
      3: (i64: UInt64);
      4: (f32: single);
      5: (f64: double);
      // for labels
      6: (idxCount: integer;
          idxDef  :LongWord);
      7: (returnType: byte);
  end;

  TCodeEntry = record
    locals    : array of TCodeLocalEntry;
    instBuf   : array of byte;
  end;

  TCodeSection = record
    entries : array of TCodeEntry;
  end;

const
  IMPDESC_FUNC   = $00;
  IMPDESC_TABLE  = $01;
  IMPDESC_MEM    = $02;
  IMPDESC_GLOBAL = $03;

type
  TImportEntry = record
    module   : string;
    name     : string;
    case desc: byte of
    IMPDESC_FUNC : (
      fnType : UInt32;
    );
    IMPDESC_TABLE: (
      tblType  : TTableType;
    );
    IMPDESC_TABLE: (
      memType  : TMemoryType;
    );
    IMPDESC_GLOBAL: (
      glbType  : TGlobalType;
    );
  end;

  TImportSection = record
    entries : array of TImportEntry;
  end;

const
  EXPDESC_FUNC   = $00;
  EXPDESC_TABLE  = $01;
  EXPDESC_MEM    = $02;
  EXPDESC_GLOBAL = $03;


type
  TExportEntry = record
    name    : string;
    desc    : byte;
    index   : UInt32;
  end;

  TExportSection = record
    entries : array of TExportEntry;
  end;

const
  ELEMTYPE_FUNC = $70;

type
  TElementEntry = record
    table : Uint32;
    expr  : array of byte;   // instructions
    funcs : array of UInt32;
  end;

  TElementSection = record
    entries : array of TElementEntry;
  end;

function SectionIdToStr(id: integer): string;
function ValTypeToStr(id: integer): string;

// reads the name from the input stream
// the name consists of
//    size - in butes Leb128
//    bytes - in utf8 format
function ReadName(st: TStream): string;
procedure WriteName(st: TStream; const str: string);
function GetName(sr: TStream): string;

// reads
function GetU32(sr: TStream): UInt32;

// reads the code entry into TCodeEntry structure
procedure ReadCodeEntry(src: TStream; var en: TCodeEntry);
// reads the code entry into TCodeEntry structure
procedure ReadCodeSection(src: TStream; var sc: TCodeSection);

function isUnreachable(const cd: TCodeEntry): Boolean;

procedure ReadExportEntry(src: TStream; var ex: TExportEntry);
// reads the export entry
procedure ReadExport(src: TStream; var ex: TExportSection);
procedure WriteExport(const ex: TExportSection; dst: TStream);

function isWasmStream(st: TStream): Boolean;
function isWasmFile(const fn: string): Boolean;

procedure ReadElementEntry(st: TStream; var en: TElementEntry);
procedure ReadElementSection(st: TStream; var sc: TelementSection);

procedure ReadLimit(st: TStream; var lm: TLimit);
procedure ReadTableType(st: TStream; var tb: TTableType);
procedure ReadGlobalType(st: TStream; var gb: TGlobalType);

function ReadImportEntry(st: TStream; var imp: TImportEntry): Boolean;
function ReadImportSection(st: TStream; var imp: TImportSection): Boolean;

implementation

procedure ReadLimit(st: TStream; var lm: TLimit);
begin
  lm.limitType := st.ReadByte;
  lm.min := ReadU(st);
  if lm.limitType <> limit_min_inf then
    lm.max := ReadU(st)
  else
    lm.max := 0;
end;

procedure ReadTableType(st: TStream; var tb: TTableType);
begin
  tb.elemType := st.ReadByte;
  ReadLimit(st, tb.limits);
end;

procedure ReadGlobalType(st: TStream; var gb: TGlobalType);
begin
  gb.valtype := st.ReadByte;
  gb.mut := st.ReadByte;
end;

function ValTypeToStr(id: integer): string;
begin
  case id of
    valtype_i32 : Result := 'i32';
    valtype_i64 : Result := 'i64';
    valtype_f32 : Result := 'f32';
    valtype_f64 : Result := 'f64';
  else
    Str(id, Result);
    Result := 'iUnk'+Result;
  end
end;

function SectionIdToStr(id: integer): string;
begin
  case id of
    sect_custom   : Result := 'custom';	// custom section
    sect_type     : Result := 'type';	// type section
    sect_import   : Result := 'import';	// import section
    sect_function : Result := 'function';	// function section
    sect_table    : Result := 'table';	// table section
    sect_memory   : Result := 'memory';	// memory section
    sect_global   : Result := 'global';	// global section
    sect_export   : Result := 'export';	// export section
    sect_start    : Result := 'start';	// start section
    sect_element  : Result := 'element';	// element section
    sect_code     : Result := 'code';	// code section
    sect_data     : Result := 'data';	// data section
  else
    Str(id, Result);
    Result := 'sect_unknown'+Result;
  end;

end;

function ReadName(st: TStream): string;
var
  ln : LongWord;
begin
  ln := ReadU(st);
  SetLength(result, ln);
  if ln>0 then st.Read(result[1], ln);
end;

procedure WriteName(st: TStream; const str: string);
begin
  WriteU32(st, length(str));
  if length(str)>0 then
    st.Write(str[1], length(str));
end;

function GetName(sr: TStream): string;
begin
  Result := ReadName(sr);
end;

function GetU32(sr: TStream): UInt32;
begin
  Result := UInt32(ReadU(sr));
end;

procedure ReadCodeEntry(src: TStream; var en: TCodeEntry);
var
  sz  : integer; // size in bytes
  //pos : int64;
  cnt : Integer;
  i   : integer;
  eofs : Int64;
begin
  sz := ReadU(src);
  eofs := src.Position+sz;

  cnt := ReadU(src);
  SetLength(en.locals, cnt);
  for i:=0 to cnt-1 do begin
    en.locals[i].count := ReadU(src);
    en.locals[i].valtyp := src.ReadByte;
  end;
  SetLength(en.instBuf, eofs-src.Position);
  if (length(en.instBuf)>0) then
    src.Read(en.instBuf[0], length(en.instBuf));

end;

procedure ReadCodeSection(src: TStream; var sc: TCodeSection);
var
  cnt : integer;
  i   : integer;
begin
  cnt := ReadU(src);
  SetLength(sc.entries, cnt);
  for i:= 0 to cnt-1 do
    ReadCodeEntry(src, sc.entries[i]);
end;

function isUnreachable(const cd: TCodeEntry): Boolean;
begin
  Result:=(length(cd.instBuf)>0) and (cd.instBuf[0]=INST_TRAP);
end;

procedure ReadExportEntry(src: TStream; var ex: TExportEntry);
begin
  ex.name := ReadName(src);
  ex.desc := src.ReadByte;
  ex.index := ReadU(src);
end;

procedure ReadExport(src: TStream; var ex: TExportSection);
var
  cnt : integer;
  i   : integer;
begin
  cnt := ReadU(src);
  SetLength(ex.entries, cnt);
  for i:=0 to cnt-1 do
    ReadExportEntry(src, ex.entries[i]);
end;

procedure WriteExport(const ex: TExportSection; dst: TStream);
var
  i : integer;
begin
  WriteU32(dst, length(ex.entries));
  for i:=0 to length(ex.entries)-1 do begin
    WriteName(dst, ex.entries[i].name);
    dst.WriteByte(ex.entries[i].desc);
    WriteU32(dst, ex.entries[i].index);
  end;
end;

function isWasmStream(st: TStream): Boolean;
var
  pos : Int64;
begin
  try
    pos:=st.Position;
    try
      Result := st.ReadDWord = WasmId_Int;
    finally
      st.Position:=pos;
    end;
  except
    Result:=false;
  end;
end;

function isWasmFile(const fn: string): Boolean;
var
  fs: TFileStream;
begin
  try
    fs:=TFileStream.Create(fn, fmOpenRead or fmShareDenyNone);
    try
      Result:=isWasmStream(fs);
    finally
      fs.Free;
    end;
  except
    Result := false;
  end;
end;

procedure ReadElementEntry(st: TStream; var en: TElementEntry);
var
  ln : integer;
  i  : integer;
begin
  en.table := ReadU(st);
  ln:=InstLen(st);
  if ln<0 then Exit;
  SetLength(en.expr, ln);
  if ln>0 then st.Read(en.expr[0], ln);

  ln:=ReadU(st);
  SetLength(en.funcs, ln);
  for i:=0 to ln-1 do
    en.funcs[i]:=ReadU(st);
end;

procedure ReadElementSection(st: TStream; var sc: TelementSection);
var
  cnt : integer;
  i   : integer;
begin
  cnt := ReadU(st);
  SetLength(sc.entries, cnt);
  for i:=0 to cnt-1 do
    ReadElementEntry(st, sc.entries[i]);
end;

function ReadImportEntry(st: TStream; var imp: TImportEntry): Boolean;
begin
  Result := true;
  imp.module := ReadName(st);
  imp.name := ReadName(st);
  imp.desc := st.ReadByte;
  case imp.desc of
    IMPDESC_FUNC :  imp.fnType := ReadU(st);
    IMPDESC_TABLE:  ReadTableType(st, imp.tblType);
    IMPDESC_MEM:    ReadLimit(st, imp.memType);
    IMPDESC_GLOBAL: ReadGlobalType(st, imp.glbType);
  else
    Result := false;
  end;
end;

function ReadImportSection(st: TStream; var imp: TImportSection): Boolean;
var
  cnt : integer;
  i   : integer;
begin
  cnt := ReadU(st);
  SetLength(imp.entries, cnt);
  Result := true;
  if cnt>0 then
    for i:=0 to cnt-1 do
      if not ReadImportEntry(st, imp.entries[i]) then begin
        Result := false;
        break;
      end;
end;

end.

