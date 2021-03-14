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

unit wasmtoolutils;

{$mode objfpc}{$H+}

interface

uses
  Classes,SysUtils, wasmbin, lebutils,
  //wasmbindebug,
  wasmlink, wasmlinkchange;

function ChangeSymbolFlagStream(st: TStream; syms: TStrings): Boolean;
procedure ChangeSymbolFlag(const fn, symfn: string);
function PredictSymbolsFromLink(const wasmfn: string; weakList: TStrings; doVerbose: Boolean = false): Boolean;

procedure MatchExportNameToSymName(const x: TExportSection; const l: TLinkingSection; dst: TStrings);
function ExportRenameSym(var x: TExportSection; syms: TStrings): Integer;
function ExportRenameProcess(st, dst: TStream; syms: TStrings; doVerbose: Boolean): Boolean;
function ExportNameGather(const wasmfile: string; syms: TStrings; doVerbose: Boolean = false): Boolean;
procedure ExportRename(const fn, symfn: string; doVerbose: Boolean);

implementation

function ChangeSymbolFlagStream(st: TStream; syms: TStrings): Boolean;
var
  dw  : LongWord;
  ofs : int64;
  sc  : TSection;
  ps  : int64;
  nm  : string;
begin
  dw := st.ReadDWord;
  Result := dw = WasmId_Int;
  if not Result then Exit;

  dw := st.ReadDWord;
  while st.Position<st.Size do begin
    ofs := st.Position;
    sc.id := st.ReadByte;
    sc.Size := ReadU(st);

    ps := st.Position+sc.size;
    if sc.id=0 then begin
      nm := GetName(st);
      if nm = SectionName_Linking then begin
        ProcessLinkingSection(st, syms);
        break;
      end;
        //DumpLinking(st, sc.size - (st.Position - ofs));
    end;
    //if sc.id= 1 then DumpTypes(st);

    if st.Position <> ps then
    begin
      //writeln('adjust stream targ=',ps,' actual: ', st.position);
      st.Position := ps;
    end;
  end;
end;

// Assumption is made, there's only 1 table in the file!
// if a function is a stub function (the only code is "unreachable"), the status given
//  "weak" (it's a reference function elsewhere)
// if a function is located in the function table, then the status given is
//  "hidden" (do not add to the final linked executable)
// if a function is not located in the function table, the status given is:
//  "hidden"+"local" (local means the function can be used only in this object file)
procedure MatchExportNameToSymFlag(
  const imp: TImportSection;
  const c: TCodeSection;
  const e: TElementSection;
  const x: TExportSection;
  var l: TLinkingSection;
  weakList: TStrings; // do known set of globals weak
  doVerbose: Boolean);
type
  TFuncType = (ftImpl = 0, ftIntf, ftStub, ftExport);

  TFuncInfo = record
    hasSymbol : Boolean;
    fnType    :  TFuncType;
  end;

var
  i  : integer;
  j  : integer;
  idx : integer;
  fn : array of TFuncInfo;
  codeofs: integer;
begin
  idx := -1;
  for i:=0 to length(l.symbols)-1 do
    if l.symbols[i].kind = SYMTAB_FUNCTION then begin
      if l.symbols[i].symindex>idx then begin
        idx:= l.symbols[i].symindex;
      end;
    end;

  SetLength(fn, idx+1);
  for i:=0 to length(l.symbols)-1 do
    if l.symbols[i].kind = SYMTAB_FUNCTION then begin
      idx := l.symbols[i].symindex;
      fn[idx].hasSymbol:=true;
    end;

  for i:=0 to length(e.entries)-1 do
    for j:=0 to length(e.entries[i].funcs)-1 do begin
      idx := e.entries[i].funcs[j];
      fn[idx].fnType:=ftIntf;
    end;

  codeofs:=0;
  for i:=0 to length(imp.entries)-1 do
    if imp.entries[i].desc = IMPDESC_FUNC then
      inc(codeofs);

  for i:=codeofs to length(fn)-1 do begin
    if not fn[i].hasSymbol then begin
      Continue;
    end;

    if (fn[i].fnType=ftImpl) and (isUnreachable(c.entries[i-codeofs])) then begin
      fn[i].fnType:=ftStub;
    end;
  end;

  for i:=0 to length(x.entries)-1 do begin
    if x.entries[i].desc = EXPDESC_FUNC then begin
      idx := x.entries[i].index;
      if fn[idx].fnType<>ftStub then
        fn[idx].fnType:=ftExport;
    end;
  end;

  for i:=0 to length(l.symbols)-1 do begin
    if l.symbols[i].kind = SYMTAB_FUNCTION then begin
      j := l.symbols[i].symindex;

      if j>=codeofs then // not imported
        case fn[j].fnType of
          ftImpl:
            l.symbols[i].flags := l.symbols[i].flags or WASM_SYM_VISIBILITY_HIDDEN or WASM_SYM_BINDING_LOCAL;
          ftIntf:
            l.symbols[i].flags := l.symbols[i].flags or WASM_SYM_VISIBILITY_HIDDEN;
          ftStub:
            l.symbols[i].flags := l.symbols[i].flags or WASM_SYM_BINDING_WEAK or WASM_SYM_VISIBILITY_HIDDEN;
          ftExport:
            //l.symbols[i].flags := l.symbols[i].flags or WASM_SYM_VISIBILITY_HIDDEN or WASM_SYM_BINDING_WEAK;
            ;
        end;

      if DoVerbose then begin
        write('func ');
        if l.symbols[i].hasSymName then
          write(l.symbols[i].symname)
        else
          write('#',j);
        write(' ', fn[j].fnType);
        writeln;
      end;
      //if l.symbols[i].symindex>mx then mx := ;
    end else if (l.symbols[i].kind = SYMTAB_GLOBAL) and Assigned(weakList) then begin
      if l.symbols[i].hasSymName and (weakList.IndexOf(l.symbols[i].symname)>=0) then begin
        if doVerbose then
          writeln('weakining: ',l.symbols[i].symname);
        l.symbols[i].flags := l.symbols[i].flags or WASM_SYM_BINDING_WEAK or WASM_SYM_VISIBILITY_HIDDEN;
      end;
    end;
  end;


end;

function PredictSymbolsFromLink(const wasmfn: string; weakList: TStrings; doVerbose: Boolean = false): Boolean;
var
  st : TFileStream;
  dw : LongWord;
  foundCode    : Boolean;
  foundElement : Boolean;
  foundLink    : Boolean;
  foundExport  : Boolean;
  foundImport  : Boolean;
  ofs : Int64;
  ps  : Int64;
  sc  : TSection;
  c   : TCodeSection;
  imp : TImportSection;
  l   : TLinkingSection;
  e   : TElementSection;
  x   : TExportSection;
  nm  : string;
  lofs : Int64;
  lsize : Int64;
  mem   : TMemoryStream;
  mem2  : TMemoryStream;
begin
  st := TFileStream.Create(wasmfn, fmOpenReadWrite or fmShareDenyNone);
  try
    dw := st.ReadDWord;
    Result := dw = WasmId_Int;
    if not Result then Exit;

    dw := st.ReadDWord;

    foundElement := false;
    foundCode := false;
    foundLink := false;
    foundExport := false;
    foundImport := false;
    Result := false;
    while st.Position<st.Size do begin
      ofs := st.Position;
      sc.id := st.ReadByte;
      sc.Size := ReadU(st);

      ps := st.Position+sc.size;

      case sc.id of
        SECT_IMPORT: begin
          ReadImportSection(st, imp);
          foundImport := true;
        end;
        SECT_EXPORT: begin
          ReadExport(st, x);
          foundExport := true;
        end;
        SECT_ELEMENT: begin
          ReadElementSection(st, e);
          foundElement := true;
        end;
        SECT_CODE: begin
          ReadCodeSection(st, c);
          foundCode := true;
        end;
        SECT_CUSTOM: begin
          nm := ReadName(st);
          if nm = SectionName_Linking then begin
            lofs:=ofs;
            ReadLinkingSection(st, sc.size, l);
            foundLink := true;
            lsize := ps-lofs;
          end;
        end;
      end;

      if st.Position <> ps then begin
        st.Position := ps;
      end;

      Result := foundLink and foundCode and foundElement;
      if Result then break;
    end;

    if not foundExport then SetLength(x.entries,0);
    if not foundImport then SetLength(imp.entries, 0);

    if Result then begin
      if doVerbose then writeln('detecting symbols');
      MatchExportNameToSymFlag(imp, c, e, x, l, weakList, doVerbose);
      mem:=TMemoryStream.Create;
      mem2:=TMemoryStream.Create;
      try
        st.Position:=lofs+lsize;
        mem2.CopyFrom(st, st.Size - st.Position);

        st.Position:=lofs;
        WriteName(mem, SectionName_Linking);
        WriteLinkingSection(mem, l);

        st.WriteByte(SECT_CUSTOM);
        if doVerbose then writeln('section size: ', mem.Size);
        WriteU32(st, mem.Size);

        mem.Position:=0;
        if doVerbose then writeln('copying from mem');
        st.CopyFrom(mem, mem.Size);
        mem2.Position:=0;
        if doVerbose then writeln('copying from mem2');
        st.CopyFrom(mem2, mem2.Size);
        st.Size:=st.Position;
      finally
        mem.Free;
        mem2.Free;
      end;
      if doVerbose then writeln('written: ', st.Position-lofs,' bytes');
    end else
      writeln('failed. section find status. Likning: ', foundLink,'; Code: ', foundCode,'; Element: ', foundElement);
  finally
    st.Free;
  end;
end;

procedure ChangeSymbolFlag(const fn, symfn: string);
var
  fs :TFileStream;
  syms:  TStringList;
begin
  syms:=TStringList.Create;
  fs := TFileStream.Create(fn, fmOpenReadWrite or fmShareDenyNone);
  try
    if (symfn<>'') then begin
      ReadSymbolsConf(symfn, syms);
      ChangeSymbolFlagStream(fs, syms);
    end;
  finally
    fs.Free;
    syms.Free;
  end;
end;

function ExportRenameSym(var x: TExportSection; syms: TStrings): integer;
var
  i : integer;
  v : string;
begin
  Result := 0;
  for i:=0 to length(x.entries)-1 do begin
    v := syms.Values[x.entries[i].name];
    if v <> '' then begin
      x.entries[i].name := v;
      inc(Result);
    end;
  end;
end;

function ExportRenameProcess(st, dst: TStream; syms: TStrings; doVerbose: Boolean): Boolean;
var
  dw  : LongWord;
  ofs : int64;
  sc  : TSection;
  ps  : int64;
  x   : TExportSection;
  mem : TMemoryStream;
  cnt : integer;
begin
  dw := st.ReadDWord;
  Result := dw = WasmId_Int;
  if not Result then begin
    Exit;
  end;
  dw := st.ReadDWord;
  while st.Position<st.Size do begin
    ofs := st.Position;
    sc.id := st.ReadByte;
    sc.Size := ReadU(st);

    ps := st.Position+sc.size;

    if sc.id = SECT_EXPORT then begin
      if doVerbose then writeln(' export section found');
      ReadExport(st, x);
      cnt := ExportRenameSym(x, syms);

      st.Position:=0;
      dst.CopyFrom(st, ofs);
      st.Position:=ps;

      mem := TMemoryStream.Create;
      WriteExport(x, mem);
      mem.Position:=0;

      dst.WriteByte(SECT_EXPORT);
      WriteU32(dst, mem.Size);
      dst.CopyFrom(mem, mem.Size);

      dst.CopyFrom(st, st.Size-st.Position);
      break;
    end;

    if st.Position <> ps then
      st.Position := ps;
  end;
end;

// match between exported function names and symbol names
procedure MatchExportNameToSymName(const x: TExportSection; const l: TLinkingSection;  dst: TStrings);
var
  expname : string;
  i,j     : integer;
begin
  for i:=0 to length(x.entries)-1 do begin
    // gathering only function names for now
    if x.entries[i].desc <> EXPDESC_FUNC then continue;
    expname := x.entries[i].name;
    for j:=0 to length(l.symbols)-1 do begin
      if (l.symbols[j].kind = SYMTAB_FUNCTION)
        and (l.symbols[j].symindex = x.entries[i].index)
        and (l.symbols[j].hasSymName)
      then
        dst.Values[ l.symbols[j].symname ] := expname;
    end;
  end;

end;

function ExportNameGather(const wasmfile: string; syms: TStrings; doVerbose: Boolean = false): Boolean;
var
  dw  : LongWord;
  ofs : int64;
  sc  : TSection;
  ps  : int64;
  mem : TMemoryStream;
  cnt : integer;
  st  : TFileStream;
  nm  : string;
  x   : TExportSection;
  foundExport: Boolean;
  l   : TLinkingSection;
  foundLink: Boolean;
begin
  st := TFileStream.Create(wasmfile, fmOpenRead or fmShareDenyNone);
  try
    dw := st.ReadDWord;
    Result := dw = WasmId_Int;
    if not Result then begin
      Exit;
    end;
    dw := st.ReadDWord;

    foundExport:=false;
    foundLink:=false;
    while st.Position<st.Size do begin
      ofs := st.Position;
      sc.id := st.ReadByte;
      sc.Size := ReadU(st);

      ps := st.Position+sc.size;

      if sc.id = SECT_EXPORT then begin
        if doVerbose then writeln(' export section found');
        ReadExport(st, x);
        cnt := ExportRenameSym(x, syms);
        foundExport:=true;
      end else if sc.id = SECT_CUSTOM then begin
        nm := ReadName(st);
        if nm = SectionName_Linking then begin
          foundLink := true;
          ReadLinkingSection(st, sc.size, l);
        end;
      end;

      if st.Position <> ps then
        st.Position := ps;
    end;

    Result := foundLink and foundExport;
    if Result then
      MatchExportNameToSymName(x, l, syms);
  finally
    st.Free;
  end;
end;

procedure ExportRename(const fn, symfn: string; doVerbose: Boolean);
var
  fs    : TFileStream;
  syms  : TStringList;
  dst   : TMemoryStream;
begin
  if doVerbose then writeln('Export symbols renaming');
  syms:=TStringList.Create;
  fs := TFileStream.Create(fn, fmOpenReadWrite or fmShareDenyNone);
  dst := TMemoryStream.Create;
  try
    if (symfn <> '') and fileExists(symfn) then
    begin
      if doVerbose then writeln('reading symbols: ', symfn);

      if isWasmFile(symfn) then
        ExportNameGather(symfn, syms, doVerbose)
      else
        syms.LoadFromFile(symfn);
      if doVerbose then write(syms.Text);
    end;

    ExportRenameProcess(fs, dst, syms, doVerbose);

    fs.Position:=0;
    dst.Position:=0;
    fs.CopyFrom(dst, dst.Size);
    fs.Size:=dst.Size;

  finally
    dst.Free;
    fs.Free;
    syms.Free;
  end;
end;

end.
