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

unit wasmbinwriter;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, AVL_Tree,
  wasmmodule, wasmbin, lebutils, wasmbincode
  ,wasmlink;

type
  TSectionRec = record
    secpos    : int64;
    szpos     : int64;
    datapos   : int64;
    endofdata : int64;
  end;

  TSymbolObject = class(TObject)
    idx     : Integer;
    syminfo : TSymInfo;
    next    : TSymbolObject;
    wasmObj : TObject;
  end;

  { TBinWriter }

  TBinWriter = class
  protected
    dst  : TStream;
    org  : TStream;
    strm : TList;

    module : TWasmModule;

    // the list of relocations per module
    writeSec   : Byte;
    reloc      : array of TRelocationEntry;
    relocCount : integer;

    symHead    : TSymbolObject;
    symTail    : TSymbolObject;
    syms       : TAVLTree;
    symCount   : Integer;
    function AddSymbolObject(obj: TObject): TSymbolObject;
    procedure AddRelocWithIndex(relocType: byte; secOfs: int64; index: UInt32);
    procedure AddRelocToObj(relocType: byte; secOfs: int64; wasmObj: TObject);

    procedure WriteRelocU32(u: longword);
    procedure WriteString(const s: string);
    procedure SectionBegin(secId: byte; out secRec: TSectionRec; secsize: longWord=0);
    function SectionEnd(var secRec: TSectionRec): Boolean;

    procedure WriteInstList(list: TWasmInstrList; ofsAddition: LongWord);

    procedure WriteImportSect;
    procedure WriteFuncTypeSect;
    procedure WriteTableSect;
    procedure WriteMemorySect;
    procedure WriteGlobalSect;
    procedure WriteFuncSect;
    procedure WriteExportSect;
    procedure WriteCodeSect;
    procedure WriteElemSect;

    procedure PrepareLinkSym(m: TWasmModule);
    procedure WriteLinkingSect;
    procedure WriteRelocSect;

    procedure pushStream(st: TStream);
    function popStream: TStream;
  public
    keepLeb128 : Boolean; // keep leb128 at 4 offset relocatable
    writeReloc : Boolean; // writting relocation (linking) information
    constructor Create;
    destructor Destroy; override;
    function Write(m: TWasmModule; adst: TStream): Boolean;
  end;

function WriteModule(m: TWasmModule; dst: TStream;
  awriteLEB128: Boolean = false;
  awriteReloc: Boolean = false): Boolean;

type
  TLocalsInfo = record
    count : Integer;
    tp    : byte;
  end;
  TLocalInfoArray = array of TLocalsInfo;

// returns the list of local arrays
procedure GetLocalInfo(func: TWasmFunc; out loc: TLocalInfoArray);

procedure WriteLimit(dst: TStream; amin, amax: LongWord);

implementation


function ComparePtrUInt(p1,p2: PtrUInt): Integer; inline;
begin
  if p1<p2 then Result:=-1
  else if p1=p2 then Result:=0
  else Result:=1;
end;

function CompareSymObjs(Item1, Item2: Pointer): Integer;
var
  s1, s2: TSymbolObject;
begin
  s1:=TSymbolObject(Item1);
  s2:=TSymbolObject(Item2);
  Result:=ComparePtrUInt(PtrUInt(s1.wasmObj), PtrUInt(s2.wasmObj));
end;

function CompareWasmToSymObj(Item1, Item2: Pointer): Integer;
var
  s2: TSymbolObject;
begin
  s2:=TSymbolObject(Item2);
  Result:=ComparePtrUInt(PtrUInt(Item1), PtrUInt(s2.wasmObj));
end;

procedure WriteLimit(dst: TStream; amin, amax: LongWord);
begin
  if not Assigned(dst) then Exit;
  if amax>0 then begin
    dst.WriteByte(1);
    WriteU32(dst, amin);
    WriteU32(dst, amax);
  end else begin
    dst.WriteByte(0);
    WriteU32(dst, amin);
  end;
end;

procedure GetLocalInfo(func: TWasmFunc; out loc: TLocalInfoArray);
var
  i   : integer;
  cnt : integer;
  tp  : byte;
  nt  : byte;
  j   : integer;

  procedure Push;
  begin
    if j=length(loc) then begin
      if j=0 then SetLength(loc, 1)
      else SetLength(loc, j*2);
    end;
    loc[j].tp:=tp;
    loc[j].count:=cnt;
    inc(j);
  end;

begin
  SetLength(Loc, 0);
  if func.LocalsCount = 0 then Exit;
  cnt:=1;
  tp:=func.GetLocal(0).tp;
  j:=0;
  for i:=1 to func.LocalsCount-1 do begin
    nt := func.GetLocal(i).tp;
    if nt<>tp then begin
      Push;
      tp:=nt;
      cnt:=1;
    end else
      inc(cnt);
  end;
  Push;
  SetLength(loc, j);
end;

function WriteModule(m: TWasmModule; dst: TStream;
  awriteLEB128, awriteReloc: Boolean): Boolean;
var
  bw : TBinWriter;
begin
  bw := TBinWriter.Create;
  try
    bw.keepLeb128:=awriteLEB128;
    bw.writeReloc:=awriteReloc;
    Result := bw.Write(m, dst);
  finally
    bw.Free;
  end;
end;

{ TBinWriter }

function GetLinkName(const linkInfo: TLinkInfo; const id: string): string;
begin
  if linkInfo.Name<>'' then Result:=linkInfo.Name
  else Result:=id;
end;

function TBinWriter.AddSymbolObject(obj: TObject): TSymbolObject;
var
  so : TSymbolObject;
  t  : TAVLTreeNode;
begin
  t := syms.FindKey(obj, @CompareWasmToSymObj);
  if Assigned(t) then begin
    Result:=TSymbolObject(t.Data);
    Exit;
  end;
  so := TSymbolObject.Create;
  if not Assigned(symHead) then symHead:=so;
  if Assigned(symTail) then symTail.Next:=so;
  so.idx:=symCount;
  so.wasmObj:=obj;
  symTail:=so;
  inc(symCount);
  Result:=so;

  if (obj is TWasmFunc) then begin
    so.syminfo.kind:=SYMTAB_FUNCTION;
    so.syminfo.symindex:=TWasmFunc(obj).idNum;
  end else if (obj is TWasmGlobal) then begin
    so.syminfo.kind:=SYMTAB_GLOBAL;
    so.syminfo.symindex:=TWasmGlobal(obj).id.idNum;
    so.syminfo.symname:=GetLinkName(TWasmGlobal(obj).LinkInfo, TWasmGlobal(obj).id.id);  //todo: use symbolic name
  end else if (obj is TWasmTable) then begin
    so.syminfo.kind:=SYMTAB_TABLE;
    so.syminfo.symindex:=TWasmTable(obj).id.idNum;
  end;

  syms.Add(so);
end;

procedure TBinWriter.AddRelocWithIndex(relocType: byte; secOfs: int64; index: UInt32);
var
  i : integer;
  f : TWasmFunc;
  //so : TSymbolObject;
begin
  if relocCount=length(reloc) then begin
    if relocCount=0 then SetLength(reloc, 16)
    else SetLength(reloc, relocCount*2);
  end;
  i:=relocCount;
  reloc[i].sec:=writeSec;
  reloc[i].reltype:=relocType;
  reloc[i].offset:=secOfs;
  reloc[i].index:=index;
  inc(relocCount);
end;

procedure TBinWriter.AddRelocToObj(relocType: byte; secOfs: int64; wasmObj: TObject);
var
  idx : integer;
begin
  if not Assigned(wasmObj) then Exit;

  idx:=AddSymbolObject(wasmObj).idx;
  AddRelocWithIndex(relocType, secOfs, idx);
end;

procedure TBinWriter.WriteRelocU32(u: longword);
begin
  WriteU(dst, u, sizeof(u)*8, keepLeb128);
end;

procedure TBinWriter.WriteString(const s: string);
begin
  WriteU32(dst, length(s));
  if length(s)>0 then
    dst.Write(s[1], length(s));
end;

function TBinWriter.Write(m: TWasmModule; adst: TStream): Boolean;
var
  l : Longword;
begin
  if not Assigned(m) or not Assigned(adst) then begin
    Result:=false;
    Exit;
  end;
  keepLeb128:=keepLeb128 or writeReloc; // use 128, if relocation has been requested

  module:=m;
  dst:=adst;
  org:=adst;

  dst.Write(WasmId_Buf, length(WasmId_Buf));
  l:=NtoLE(Wasm_Version1);
  dst.Write(l, sizeof(l));

  writeSec:=0;
  // 01 function type section
  if m.TypesCount>0 then begin
    WriteFuncTypeSect;
    inc(writeSec);
  end;

  // 02 import section
  if m.ImportCount>0 then begin
    WriteImportSect;
    inc(writeSec);
  end;

  // 03 function section
  if m.FuncCount>0 then begin
    WriteFuncSect;
    inc(writeSec);
  end;

  // 04 tables section
  if m.TableCount>0 then begin
    WriteTableSect;
    inc(writeSec);
  end;

  // 05 memory section
  if m.MemoryCount>0 then begin
    WriteMemorySect;
    inc(writeSec);
  end;

  // 06 globals section
  if m.GlobalCount>0 then begin
    WriteGlobalSect;
    inc(writeSec);
  end;

  // 07 export section
  if m.ExportCount>0 then begin
    WriteExportSect;
    inc(writeSec);
  end;

  // 09 - element sections
  if m.ElementCount>0 then begin
    WriteElemSect;
    inc(writeSec);
  end;

  // 10 code section
  if m.FuncCount>0 then begin
    WriteCodeSect;
    inc(writeSec);
  end;

  if writeReloc then begin
    PrepareLinkSym(m);
    WriteLinkingSect;
    WriteRelocSect;
  end;

  Result:=true;
end;

procedure TBinWriter.SectionBegin(secId: byte; out secRec: TSectionRec; secsize: longWord=0);
begin
  secRec.secpos:=dst.Position;
  dst.WriteByte(secId);
  secRec.szpos:=dst.Position;
  WriteRelocU32(secsize);
  secRec.datapos:=dst.Position;
  secRec.endofdata:=dst.Position+secsize;
end;

function TBinWriter.SectionEnd(var secRec: TSectionRec): Boolean;
var
  sz: LongWord;
begin
  secRec.endofdata:=dst.Position;
  dst.Position:=secRec.szpos;
  sz := secRec.endofdata - secRec.datapos;
  WriteRelocU32(sz);
  dst.Position:=secRec.endofdata;
  Result := true;
end;

procedure TBinWriter.WriteFuncTypeSect;
var
  sc : TSectionRec;
  i  : integer;
  j  : integer;
  tp : TWasmFuncType;
begin
  SectionBegin(SECT_TYPE, sc);

  WriteU32(dst, module.TypesCount);
  for i:=0 to module.TypesCount-1 do begin
    tp:=module.GetType(i);
    dst.WriteByte(func_type);

    WriteU32(dst, tp.ParamCount);
    for j:=0 to tp.ParamCount-1 do
      dst.WriteByte(tp.GetParam(j).tp);

    WriteU32(dst, tp.ResultCount);
    for j:=0 to tp.ResultCount-1 do
      dst.WriteByte(tp.GetResult(j).tp);
  end;
  SectionEnd(sc);
end;

procedure TBinWriter.WriteTableSect;
var
  sc  : TSectionRec;
  i   : integer;
  t   : TWasmTable;
begin
  SectionBegin(SECT_TABLE, sc);
  WriteU32(dst, module.TableCount);
  for i:=0 to module.TableCount-1 do begin
    t:=module.GetTable(i);
    dst.WriteByte(t.elemsType);
    WriteLimit(dst, t.min, t.max);
  end;
  SectionEnd(sc);
end;

procedure TBinWriter.WriteMemorySect;
var
  sc : TSectionRec;
  i  : integer;
  m  : TWasmMemory;
begin
  SectionBegin(SECT_MEMORY, sc);

  WriteU32(dst, module.MemoryCount);
  for i:=0 to module.MemoryCount-1 do begin
    m := module.GetMemory(i);
    WriteLimit(dst, m.min, m.max);
  end;

  SectionEnd(sc);
end;

procedure TBinWriter.WriteGlobalSect;
var
  sc : TSectionRec;
  i  : integer;
  g  : TWasmGlobal;
begin
  SectionBegin(SECT_GLOBAL, sc);

  WriteU32(dst, module.GlobalCount);
  for i:=0 to module.GlobalCount-1 do begin
    g := module.GetGlobal(i);
    dst.WriteByte(g.tp);
    if g.isMutable then dst.WriteByte(1)
    else dst.WriteByte(0);
    WriteInstList(g.StartValue, sc.datapos);
  end;

  SectionEnd(sc);
end;

procedure TBinWriter.WriteFuncSect;
var
  sc : TSectionRec;
  i  : integer;
begin
  SectionBegin(SECT_FUNCTION, sc);

  WriteU32(dst, module.FuncCount);
  for i:=0 to module.FuncCount-1 do
    // wat2test doesn't write the function section as relocatable
    // WriteRelocU32(m.GetFunc(i).functype.typeNum);
    WriteU32(dst, module.GetFunc(i).functype.typeNum);

  SectionEnd(sc);
end;

procedure TBinWriter.WriteExportSect;
var
  sc : TSectionRec;
  i  : integer;
  x  : TWasmExport;
begin
  SectionBegin(SECT_EXPORT, sc);

  WriteU32(dst, module.ExportCount);
  for i:=0 to module.ExportCount-1 do begin
    x:=module.GetExport(i);
    WriteU32(dst, length(x.name));
    if length(x.name)>0 then
      dst.Write(x.name[1], length(x.name));
    dst.WriteByte(x.exportType);

    //wat2wasm doesn't write relocate the information
    //WriteRelocU32(x.exportNum);
    WriteU32(dst, x.exportNum);
  end;

  SectionEnd(sc);
end;


procedure TBinWriter.WriteCodeSect;
var
  sc    : TSectionRec;
  i, j  : integer;
  sz    : int64;
  mem   : TMemoryStream;
  la    : TLocalInfoArray;
  f     : TWasmFunc;
  dofs  : Int64;
  fnofs : Int64; // function offset in the data of "code section"
  main  : TMemoryStream;
begin
  //  for the use of leb128, the header can be written ahead of the body
  //  as the size of the section would always take 5 bytes.
  //  for not forcing leb128, the size of the body must be known ahead of time

  if keepLeb128 then begin
    SectionBegin(SECT_CODE, sc);
    dofs := dst.Position;
  end else
    dofs := 0; // we don't really care. dofs only matters for relocation+keepLeb128

  main:=TMemoryStream.Create;
  mem:=TMemoryStream.Create;
  try
    pushStream(main);
    WriteU32(dst, module.FuncCount);
    for i :=0 to module.FuncCount-1 do begin
      f:=module.GetFunc(i);

      GetLocalInfo(f, la);

      mem.Position:=0;
      fnofs := dofs + main.Position + 5; // "la" will be written after, 5 is for the writeSize. +5 is for WriteRelocU32(sz)
      pushStream(mem);

      WriteU32(dst, length(la));
      for j:=0 to length(la)-1 do begin
        WriteU32(dst, la[j].count);
        dst.WriteByte(la[j].tp);
      end;
      WriteInstList(f.instr, LongWord(fnofs-sc.datapos));
      popStream;

      sz:=mem.Position;
      mem.Position:=0;

      WriteRelocU32(sz);
      dst.CopyFrom(mem, sz);
    end;
    popStream;

    if not keepLeb128 then
      SectionBegin(SECT_CODE, sc, main.Size);

    main.Position:=0;
    dst.CopyFrom(main, main.Size);
  finally
    mem.Free;
    main.Free;
  end;


  SectionEnd(sc);
end;

procedure TBinWriter.WriteElemSect;
var
  sc : TSectionRec;
  el : TWasmElement;
  i  : Integer;
  j  : Integer;
begin
  SectionBegin(SECT_ELEMENT, sc);

  WriteU32(dst, module.ElementCount);
  for i:=0 to module.ElementCount-1 do begin
    el := module.GetElement(i);
    WriteU32(dst, el.tableId.idNum);
    WriteInstList(el.offset, sc.datapos);
    WriteU32(dst, el.funcCount);

    if writeReloc then begin
      for j:=0 to el.funcCount-1 do begin
        AddRelocToObj(R_WASM_FUNCTION_INDEX_LEB, dst.Position - sc.datapos,
          GetFuncByNum(module, el.funcs[j].idNum));
        WriteRelocU32(el.funcs[j].idNum);
      end;
    end else
      for j:=0 to el.funcCount-1 do
        WriteU32(dst, el.funcs[j].idNum);
  end;

  SectionEnd(sc);
end;

procedure TBinWriter.WriteLinkingSect;
var
  sc : TSectionRec;
  mem : TMemoryStream;
  so  : TSymbolObject;
begin
  SectionBegin(SECT_CUSTOM, sc);
  WriteString(SectionName_Linking);

  WriteU32(dst, LINKING_VERSION);
  if symCount>0 then begin
    dst.WriteByte(WASM_SYMBOL_TABLE);
    mem := TMemoryStream.Create;
    try
      pushStream(mem);
      //WriteU32(dst, symCount);
      WriteRelocU32(symCount);
      so:=symHead;
      while Assigned(so) do begin
        dst.WriteByte(so.syminfo.kind);
        WriteU32(dst, so.syminfo.flags);
        WriteU32(dst, so.syminfo.symindex);
        //if ((so.syminfo.flags and WASM_SYM_EXPLICIT_NAME)>0) then begin
          WriteU32(dst, length(so.syminfo.symname));
          dst.Write(so.syminfo.symname[1], length(so.syminfo.symname));
        //end;
        so:=so.next;
      end;
      popStream;

      mem.Position:=0;
      WriteU32(dst, mem.Size);
      dst.CopyFrom(mem, mem.size);
    finally
      mem.Free;
    end;
  end;
  // todo: fill out subsections

  SectionEnd(sc);
end;

procedure TBinWriter.WriteRelocSect;
var
  i  : integer;
  j  : integer;
  si : Byte;
  cnt: integer;
  sc : TSectionRec;
begin
  si:=0;
  i:=0;

  while (si<writeSec) and (i<relocCount) do begin
    if reloc[i].sec=si then begin
      SectionBegin(SECT_CUSTOM, sc);
      WriteString(SectionNamePfx_Reloc+'Code');
      j:=i;
      cnt:=0;
      while (i<relocCount) and (reloc[i].sec=si) do begin
        inc(cnt);
        inc(i);
      end;
      WriteU32(dst, reloc[j].sec);
      WriteU32(dst, cnt);
      for j:=j to i-1 do begin
        dst.WriteByte(reloc[j].reltype);
        WriteU32(dst, reloc[j].offset);
        WriteU32(dst, reloc[j].index);
      end;
      SectionEnd(sc);
    end;
    inc(si);
  end;
end;

procedure TBinWriter.WriteInstList(list: TWasmInstrList; ofsAddition: LongWord);
var
  i   : integer;
  j   : integer;
  ci  : TWasmInstr;
  rt  : Byte;
begin
  for i:=0 to list.Count-1 do begin
    ci :=list[i];
    dst.WriteByte(ci.code);

    if ci.hasRelocIdx then begin
      rt := ci.relocType;
      AddRelocToObj(rt, dst.Position+ofsAddition, ci.relocObj);
    end;

    case INST_FLAGS[ci.code].Param of
      ipi32: WriteS(dst, ci.operand1.s32, sizeof(ci.operand1.s32));
      ipi64: WriteS64(dst, ci.operand1.s64);
      ipu32: WriteU32(dst, ci.operand1.u32);
      ipu64: WriteS64(dst, Int64(ci.operand1.u64));
      ipf32: dst.Write(ci.operand1.f32, sizeof(ci.operand1.f32));
      ipf64: dst.Write(ci.operand1.f64, sizeof(ci.operand1.f64));

      ipi32OrFunc: begin
        if ci.hasRelocIdx then
          // should have been populated with Normalize
          WriteRelocU32(LongWord(ci.operand1.u32)) // todo!
        else
          WriteS(dst, ci.operand1.s32, sizeof(ci.operand1.s32));
      end;
      //ipf32,     // float point single
      //ipf64,     // float point double

      ipLeb:
      begin
        if ci.hasRelocIdx then
          WriteRelocU32(ci.operandNum)
        else
          WriteU32(dst, ci.operandNum);
      end;

      ipCallType:
      begin
        if Assigned(ci.insttype) then begin
          if ci.hasRelocIdx
            then WriteRelocU32(ci.insttype.typeNum)
            else WriteU32(dst, ci.insttype.typeNum);
        end else
          WriteU32(dst, LongWord(-1)); // this is an error.

        // table index reference
        WriteU32(dst, ci.operandNum);
      end;

      ipJumpVec: begin
        writeU32(dst, ci.vecTableCount);
        for j:=0 to ci.vecTableCount-1 do
          WriteU32(dst, ci.vecTable[j].idNum);
        WriteU32(dst, ci.operandNum);
      end;

      ipResType:
        dst.WriteByte(byte(ci.operandNum));

      ipZero:
        dst.WriteByte(0);

      ipOfsAlign: begin
        // align
        WriteU32(dst, ci.operand2.u32);
        // offset
        WriteU32(dst, ci.operand1.u32);
      end;
    end;
  end;
end;

procedure TBinWriter.WriteImportSect;
var
  sc : TSectionRec;
  i  : integer;
  im : TWasmImport;
const
  isMutableFlag : array [boolean] of byte = (global_const, global_mut);
begin
  SectionBegin(SECT_IMPORT, sc);

  WriteU32(dst, module.ImportCount);
  for i:=0 to module.ImportCount-1 do begin
    im:=module.GetImport(i);

    WriteString(im.module);
    WriteString(im.name);
    if Assigned(im.fn) then begin
      dst.WriteByte(IMPDESC_FUNC);
      WriteU32(dst, im.fn.functype.typeNum);
    end else if Assigned(im.mem) then begin
      dst.WriteByte(IMPDESC_MEM);
      WriteLimit(dst, im.mem.min, im.mem.max)
    end else if Assigned(im.table) then begin
      dst.WriteByte(IMPDESC_TABLE);
      dst.WriteByte(im.table.elemsType);
      WriteLimit(dst, im.table.min, im.table.max);
    end else if Assigned(im.glob) then begin
      dst.WriteByte(IMPDESC_GLOBAL);
      dst.WriteByte(im.glob.tp);
      dst.WriteByte(isMutableFlag[im.glob.isMutable]);
    end;
  end;
  SectionEnd(sc);
end;

procedure TBinWriter.pushStream(st: TStream);
begin
  if st=nil then Exit;
  strm.Add(st);
  dst:=st;
end;

function TBinWriter.popStream: TStream;
begin
  if strm.Count=0 then
    Result:=nil
  else begin
    Result:=TStream(strm[strm.Count-1]);
    strm.Delete(strm.Count-1);
  end;
  if strm.Count=0 then dst:=org
  else dst:=TStream(strm[strm.Count-1]);
end;

constructor TBinWriter.Create;
begin
  inherited Create;
  strm:=TList.Create;
  syms:=TAVLTree.Create(@CompareSymObjs);
end;

destructor TBinWriter.Destroy;
begin
  syms.Free;
  strm.Free;
  inherited Destroy;
end;

function isFuncLinkSym(const l: TLinkInfo): boolean;
begin
  Result:=(l.Binding<>lbUndefined)
    or l.isHidden
    or l.isUndefined
    or l.NoStrip;
end;

procedure LinkInfoToBin(const src: TLinkInfo; var dst: TSymInfo; ASymTab: byte; objFnIdx: longword);
begin
  dst.kind := ASymTab;
  dst.flags := 0;
  case src.Binding of
    lbWeak: dst.flags := dst.flags or WASM_SYM_BINDING_WEAK;
    lbLocal: dst.flags := dst.flags or WASM_SYM_BINDING_LOCAL;
    lbForHost: dst.flags := dst.flags or WASM_SYM_EXPORTED;
  end;
  if src.isHidden then dst.flags := dst.flags or WASM_SYM_VISIBILITY_HIDDEN;
  if src.isUndefined then dst.flags := dst.flags or  WASM_SYM_UNDEFINED;
  if src.NoStrip then dst.flags := dst.flags or WASM_SYM_NO_STRIP;
  dst.symindex := objFnIdx;
  dst.hasSymIndex := ASymTab<>SYMTAB_DATA;
  dst.hasSymName := src.Name<>'';
  if (dst.hasSymName) then begin
    dst.flags := dst.flags or WASM_SYM_EXPLICIT_NAME;
    dst.symname := src.Name;
  end;
end;

procedure TBinWriter.PrepareLinkSym(m: TWasmModule);
var
  i   : integer;
  f   : TWasmFunc;
  so  : TSymbolObject;
begin
  for i:=0 to m.FuncCount-1 do begin
    f := m.GetFunc(i);
    if isFuncLinkSym(f.LinkInfo) then begin
      if f.LinkInfo.Name ='' then f.LinkInfo.Name := f.id;
      so:=AddSymbolObject(f);
      LinkInfoToBin(f.linkInfo, so.syminfo, SYMTAB_FUNCTION, f.idNum);
    end;
  end;
end;

end.

