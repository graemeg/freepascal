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

unit wasmmodule;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, wasmbin, wasmbincode, wasmlink;

type
  TLinkBind = (lbUndefined = 0
               ,lbWeak
               ,lbLocal
               ,lbForHost
               );

  TLinkInfo = record
    Name        : string;
    Binding     : TLinkBind;
    isHidden    : Boolean;
    isUndefined : Boolean;
    NoStrip     : Boolean;
  end;

  TExportInfo = record
    isExport : Boolean;
    name     : string;
  end;

  { TWasmId }

  TWasmId = record
    idNum : integer;
    id    : string;
  end;

  { TWasmParam }

  TWasmParam = class(TObject)
    id : string;
    tp : byte;
    procedure CopyTo(d: TWasmParam);
  end;

  { TWasmType }

  // function signature

  { TWasmFuncType }

  TWasmFuncType = class(TObject)
  public
    params  : TList;
    results : TList;
    typeNum : Integer; // if Idx < 0 then type is declared from typeDef
    typeIdx : string;  // if typeID='' then type is declared from typeDef

    // linking information
    constructor Create;
    destructor Destroy; override;
    function AddResult(tp: byte = 0): TWasmParam;
    function AddParam(tp: byte = 0; const id: string = ''): TWasmParam;
    function GetParam(i: integer): TWasmParam;
    function GetResult(i: integer): TWasmParam; overload;
    function GetResult: TWasmParam;  overload;
    function ResultCount: Integer;
    function ParamCount: Integer;

    function isNumOrIdx: Boolean;

    procedure CopyTo(t: TWasmFuncType);
  end;


  TWasmInstrOperandType = (
     otNotused,
     otText,
     otSInt32,
     otUInt32,
     otSInt64,
     otUInt64,
     otFloat32,
     otFloat64
  );

  TWasmInstrOperand = record
     textVal : string;
     case tp : TWasmInstrOperandType of
       otSInt32: (s32: Int32);
       otUInt32: (u32: UInt32);
       otSInt64: (s64: Int64);
       otUInt64: (u64: UInt64);
       otFloat32: (f32: single);
       otFloat64: (f64: double);
  end;

  { TWasmInstr }

  TWasmInstr = class(TObject)
    code        : byte;
    operandIdx  : string;
    operandNum  : integer;            // for "call_indirect" this is table index
                                      // for "if", "loop", "block" - it's type
    operand1    : TWasmInstrOperand;  // it's "offset" for load operations
    operand2    : TWasmInstrOperand;  // it's "align" for load operations
    insttype    : TWasmFuncType;      // used by call_indirect only

    jumplabel   : string;   // the label is used only for "loop", "block" and "if"

    hasRelocIdx : Boolean;
    //relocIdx    : integer;
    relocType   : Byte;
    relocObj    : TObject; //

    vecTableCount : Integer;
    vecTable      : array of TWasmId;

    function addInstType: TWasmFuncType;
    constructor Create;
    destructor Destroy; override;
    procedure SetReloc(ARelocType: byte; ARelocObj: TObject);

    property offsetText : TWasmInstrOperand read operand1 write operand1;
    property alignText  : TWasmInstrOperand read operand2 write operand2;
  end;

  { TWasmInstrList }

  TWasmInstrList = class(TObject)
  private
    items: TList;
    function GetItem(i: integer): TWasmInstr;
  public
    constructor Create;
    destructor Destroy; override;
    function AddInstr(acode: byte = 0): TWasmInstr;
    function Count: Integer;
    property Item[i: integer]: TWasmInstr read GetItem; default;
  end;

  { TWasmGlobal }

  TWasmGlobal = class(TObject)
  public
    id         : TWasmId;
    tp         : byte;    // byte;
    isMutable  : Boolean; // is mutable
    value      : TWasmInstrList;
    LinkInfo   : TLinkInfo;
    ExportInfo : TExportInfo;
    function StartValue: TWasmInstrList;
    destructor Destroy; override;
  end;

  { TWasmFunc }

  TWasmFunc = class(TObject)
  public
    locals   : TList;
    LinkInfo : TLinkInfo;
    id       : string;
    idNum    : Integer;     // reference number (after Normalization)
    instr    : TWasmInstrList;
    functype : TWasmFuncType;
    constructor Create;
    destructor Destroy; override;
    function AddLocal: TWasmParam;
    function GetLocal(i: integer): TWasmParam;
    function LocalsCount: integer;
  end;

  { TWasmElement }

  TWasmElement = class(TObject)
    tableId   : TWasmId;
    offset    : TWasmInstrList; // offset expression
    funcCount : Integer;
    funcs     : array of TWasmId;
    function AddFunc(idx: integer): integer;
    function AddFuncId(const idx: TWasmID): integer;
    function AddOffset: TWasmInstrList;
    constructor Create;
    destructor Destroy; override;
  end;

  { TWasmExport }

  TWasmExport = class(TObject)
    name       : string;
    exportType : byte;
    exportNum  : integer;
    exportIdx  : string;
    constructor Create;
  end;

  { TWasmTable }

  TWasmTable = class(TObject)
    id        : TWasmId;
    elemsType : Byte; // type of elements
    min       : LongWord;
    max       : LongWord;
    elem      : TWasmElement;
    function AddElem: TWasmElement;
    procedure RemoveElem;
    destructor Destroy; override;
  end;

  { TWasmData }

  TWasmData = class(TObject)
    id       : TWasmId;
    offset   : TWasmInstrList;
    databuf  : array of byte;
    function StartOffset: TWasmInstrList;
    destructor Destroy; override;
  end;

  { TWasmMemory }

  TWasmMemory = class(TObject)
    id    : TWasmId;
    min   : LongWord;
    max   : LongWord; // limit
    LinkInfo   : TLinkInfo;
    exportInfo : TExportInfo;
  end;

  { TWasmImport }

  TWasmImport = class(TObject)
    LinkInfo : TLinkInfo;
    module   : string;
    name     : string;
    fn       : TWasmFunc;
    mem      : TWasmMemory;
    glob     : TWasmGlobal;
    table    : TWasmTable;
    destructor Destroy; override;
    function AddFunc: TWasmFunc;
    function AddMemory: TWasmMemory;
    function AddGlobal: TWasmGlobal;
    function AddTable: TWasmTable;
  end;

  { TWasmModule }

  TWasmModule = class(TObject)
  private
    globals : TList;
    memes   : TList;
    imports : TList;
    types   : TList;
    funcs   : TList;
    exp     : TList;
    tables  : TList;
    elems   : TList;
    data    : TList;
  public
    constructor Create;
    destructor Destroy; override;

    function AddMemory: TWasmMemory;
    function GetMemory(i: integer): TWasmMemory;
    function MemoryCount: Integer;

    function AddTable: TWasmTable;
    function GetTable(i: integer): TWasmTable;
    function TableCount: Integer;

    function AddImport: TWasmImport;
    function GetImport(i: integer): TWasmImport;
    function ImportCount: Integer;

    function AddFunc: TWasmFunc;
    function GetFunc(i: integer): TWasmFunc;
    function FuncCount: integer;

    function AddType: TWasmFuncType;
    function GetType(i: integer): TWasmFuncType;
    function TypesCount: integer;

    function AddExport: TWasmExport;
    function GetExport(i: integer): TWasmExport;
    function ExportCount: integer;

    function AddElement: TWasmElement;
    function GetElement(i: integer): TWasmElement;
    function ElementCount: Integer;

    function AddData: TWasmData;
    function GetData(i: integer): TWasmData;
    function DataCount: Integer;

    function AddGlobal: TWasmGlobal;
    function GetGlobal(i: integer): TWasmGlobal;
    function GlobalCount: Integer;
  end;

// making binary friendly. finding proper "nums" for each symbol "index"
// used or implicit type declartions
function WasmBasTypeToChar(b: byte): Char;
function WasmFuncTypeDescr(t: TWasmFuncType): string;

function FindGlobal(m: TWasmModule; const globIdx: string): integer;
function FindFunc(m: TWasmModule; const funcIdx: string): integer;
function FindParam(l: TList; const idx: string): Integer;

function FindFuncType(m: TWasmModule; const typeIdx: string): integer;

// tries to register a function in the module
// the returned value is the offset of the element within the TABLE.
function RegisterFuncIdxInElem(m: TWasmModule; const func: Integer): integer;
function RegisterFuncInElem(m: TWasmModule; const funcId: string): integer;

function RegisterFuncType(m: TWasmModule; funcType: TWasmFuncType): integer;

// tries to get a constant value from instruction list
// right now, it only pulls the first i32_const expression and tries
// to get the value out of it.
// todo: it should be more sophistacated
//
// returns false, if instruction "l" is invalid, or no i32 instruction
function InstrGetConsti32Value(l: TWasmInstrList; var vl: Integer): Boolean;

procedure OperandSetType(var op: TWasmInstrOperand; tp: TWasmInstrOperandType); inline;
procedure OperandSetInt32(var op: TWasmInstrOperand; i32: Int32); inline;
procedure OperandSetText(var op: TWasmInstrOperand; const txt: string); inline;

// should be used after normalization
// todo: what about imported functions?
function GetFuncByNum(m: TWasmModule; const idNum: Integer): TWasmFunc;
function GetGlobalByNum(m: TWasmModule; const idNum: Integer): TWasmGlobal;
function GetMemByNum(m: TWasmModule; const idNum: Integer): TWasmMemory;

implementation

procedure OperandSetType(var op: TWasmInstrOperand; tp: TWasmInstrOperandType); inline;
begin
  if op.tp<>tp then op.tp:=tp;
end;

procedure OperandSetInt32(var op: TWasmInstrOperand; i32: Int32);
begin
  OperandSetType(op, otSInt32);
  op.s32:=i32;
end;

procedure OperandSetText(var op: TWasmInstrOperand; const txt: string); inline;
begin
  OperandSetType(op, otText);
  op.textVal := txt;
end;

// returing a basic wasm basic type to a character
// i32 = i
// i64 = I
// f32 = f
// f64 = F
function WasmBasTypeToChar(b: byte): Char;
begin
  case b of
    valtype_i32: Result:='i';
    valtype_i64: Result:='I';
    valtype_f32: Result:='f';
    valtype_f64: Result:='F';
  else
    Result:='.';
  end;
end;

// converting function type to the type string
// result and params are separated by ":"
// iI:i  (param i32)(param i32) (result i32)
// :f    (result f32)
// FF    (param f64)(param(64)
function WasmFuncTypeDescr(t: TWasmFuncType): string;
var
  cnt   : integer;
  i : integer;
  j : integer;
begin
  cnt:=t.ParamCount;
  if t.Resultcount>0 then inc(cnt, t.ResultCount+1);
  SetLength(Result, cnt);
  if cnt=0 then Exit;

  j:=1;
  for i:=0 to t.ParamCount-1 do begin
    Result[j]:=WasmBasTypeToChar(t.GetParam(i).tp);
    inc(j);
  end;

  if t.ResultCount=0 then Exit;

  Result[j]:=':';
  inc(j);
  for i:=0 to t.ResultCount-1 do begin
    Result[j]:=WasmBasTypeToChar(t.GetResult(i).tp);
    inc(j);
  end;
end;

// deleting objects from the list and clearing the list
procedure ClearList(l: TList);
var
  i : integer;
begin
  for i:=0 to l.Count-1 do
    TObject(l[i]).Free;
  l.Clear;
end;

{ TWasmGlobal }

function TWasmGlobal.StartValue: TWasmInstrList;
begin
  if not Assigned(value) then
    value:=TWasmInstrList.Create;
  Result:=value;
end;

destructor TWasmGlobal.Destroy;
begin
  value.Free;
  inherited Destroy;
end;

{ TWasmTable }

function TWasmTable.AddElem: TWasmElement;
begin
  if not Assigned(elem) then
    elem:= TWasmElement.Create;
  Result := elem;
end;

procedure TWasmTable.RemoveElem;
begin
  elem.Free;
  elem:=nil;
end;

destructor TWasmTable.Destroy;
begin
  RemoveElem;
  inherited Destroy;
end;

{ TWasmData }

function TWasmData.StartOffset: TWasmInstrList;
begin
  if not Assigned(offset) then
    offset := TWasmInstrList.Create;
  Result:=offset;
end;

destructor TWasmData.Destroy;
begin
  if Assigned(offset) then offset.Free;
  inherited Destroy;
end;

{ TWasmElement }

function TWasmElement.AddFunc(idx: integer): integer;
var
  w : TWasmId;
begin
  w.id:='';
  w.idNum:=idx;
  Result := AddFuncId(w);
end;

function TWasmElement.AddFuncId(const idx: TWasmID): integer;
begin
  if funcCount = length(funcs) then begin
    if funcCount=0 then SetLength(funcs, 4)
    else SetLength(funcs, funcCount*2);
  end;
  Result:=funcCount;
  funcs[funcCount] := idx;
  inc(funcCount);
end;

function TWasmElement.AddOffset: TWasmInstrList;
begin
  if not Assigned(offset) then offset:=TWasmInstrList.Create;
  Result := offset;
end;

constructor TWasmElement.Create;
begin
  inherited Create;
  offset := TWasmInstrList.Create;
end;

destructor TWasmElement.Destroy;
begin
  offset.Free;
  inherited Destroy;
end;

{ TWasmImport }

destructor TWasmImport.Destroy;
begin
  mem.Free;
  fn.Free;
  glob.Free;
  table.Free;
  inherited Destroy;
end;

function TWasmImport.AddFunc: TWasmFunc;
begin
  if not Assigned(fn) then fn:= TWasmFunc.Create;
  Result:=fn;
end;

function TWasmImport.AddMemory: TWasmMemory;
begin
  if not Assigned(mem) then
    mem := TWasmMemory.Create;
  Result := mem;
end;

function TWasmImport.AddGlobal: TWasmGlobal;
begin
  if not Assigned(glob) then
    glob := TWasmGlobal.Create;
  Result := glob;
end;

function TWasmImport.AddTable: TWasmTable;
begin
  if not Assigned(table) then
    table := TWasmTable.Create;
  Result := table;
end;

{ TWasmExport }

constructor TWasmExport.Create;
begin
  inherited Create;
  exportNum:=-1;
end;

{ TWasmParam }

procedure TWasmParam.CopyTo(d: TWasmParam);
begin
  d.tp:=tp;
end;

{ TWasmInstr }

function TWasmInstr.addInstType: TWasmFuncType;
begin
  if insttype=nil then insttype := TWasmFuncType.Create;
  result:=insttype;
end;

constructor TWasmInstr.Create;
begin
  operandNum:=-1;
end;

destructor TWasmInstr.Destroy;
begin
  insttype.Free;
  inherited Destroy;
end;

procedure TWasmInstr.SetReloc(ARelocType: byte; ARelocObj: TObject);
begin
  hasRelocIdx := true;
  relocType := ARelocType;
  relocObj := ARelocObj;
end;

{ TWasmInstrList }

function TWasmInstrList.GetItem(i: integer): TWasmInstr;
begin
  if (i>=0) and (i < items.Count) then
    Result:=TWasmInstr(items[i])
  else
    Result:=nil;
end;

constructor TWasmInstrList.Create;
begin
  inherited Create;
  items:=TList.Create;
end;

destructor TWasmInstrList.Destroy;
begin
  ClearList(items);
  items.Free;
  inherited Destroy;
end;

function TWasmInstrList.AddInstr(acode: byte = 0): TWasmInstr;
begin
  Result:=TWasmInstr.Create;
  Result.code:=acode;
  items.Add(Result);
end;

function TWasmInstrList.Count: Integer;
begin
  Result:=items.Count;
end;

{ TWasmFuncType }

constructor TWasmFuncType.Create;
begin
  inherited Create;
  typeNum:=-1;
  params:=Tlist.Create;
  results:=Tlist.Create;
end;

destructor TWasmFuncType.Destroy;
begin
  ClearList(params);
  ClearList(results);
  params.free;
  results.free;
  inherited Destroy;
end;

function TWasmFuncType.AddResult(tp: byte): TWasmParam;
begin
  Result:=TWasmParam.Create;
  Result.tp:=tp;
  results.Add(Result);
end;

function TWasmFuncType.AddParam(tp: byte; const id: string): TWasmParam;
begin
  Result:=TWasmParam.Create;
  Result.tp:=tp;
  Result.id:=id;
  params.Add(Result);
end;

function TWasmFuncType.GetParam(i: integer): TWasmParam;
begin
  if (i>=0) and (i<params.Count) then
    Result:=TWasmParam(params[i])
  else
    Result:=nil;
end;

function TWasmFuncType.GetResult(i: integer): TWasmParam;
begin
  if (i>=0) and (i<results.Count) then
    Result:=TWasmParam(results[i])
  else
    Result:=nil;
end;

function TWasmFuncType.GetResult: TWasmParam;
begin
  Result:=GetResult(0);
end;

function TWasmFuncType.ResultCount: Integer;
begin
  Result:=results.Count;
end;

function TWasmFuncType.ParamCount: Integer;
begin
  Result:=params.Count;
end;

function TWasmFuncType.isNumOrIdx: Boolean;
begin
  Result:=(typeIdx<>'') or (typeNum>=0);
end;

procedure TWasmFuncType.CopyTo(t: TWasmFuncType);
var
  i : integer;
  s : TWasmParam;
  d : TWasmParam;
begin
  for i:=0 to ParamCount-1 do begin
    d := t.AddParam;
    s := GetParam(i);
    s.CopyTo(d);
  end;

  for i:=0 to ResultCount-1 do begin
    d := t.AddResult;
    s := GetResult(i);
    s.CopyTo(d);
  end;
end;

{ TWasmModule }

constructor TWasmModule.Create;
begin
  inherited Create;
  globals := TList.Create;
  types := TList.Create;
  funcs := TList.Create;
  exp := TList.Create;
  imports := TList.Create;
  tables := TList.Create;
  elems := TList.Create;
  memes := TList.Create;
  data := TList.Create;
end;

destructor TWasmModule.Destroy;
begin
  ClearList(data);
  data.Free;
  ClearList(memes);
  memes.Free;
  ClearList(elems);
  elems.Free;
  ClearList(tables);
  tables.Free;
  ClearList(imports);
  imports.Free;
  ClearList(exp);
  exp.Free;
  ClearList(types);
  types.Free;
  ClearList(funcs);
  funcs.Free;
  ClearList(globals);
  globals.Free;
  inherited Destroy;
end;

function TWasmModule.AddMemory: TWasmMemory;
begin
  Result:=TWasmMemory.Create;
  memes.Add(result);
end;

function TWasmModule.GetMemory(i: integer): TWasmMemory;
begin
  if (i>=0) and (i<memes.Count) then
    Result:=TWasmMemory(memes[i])
  else
    Result:=nil;
end;

function TWasmModule.MemoryCount: Integer;
begin
  Result:=memes.Count;
end;

function TWasmModule.AddTable: TWasmTable;
begin
  Result:=TWasmTable.Create;
  tables.Add(Result);
end;

function TWasmModule.GetTable(i: integer): TWasmTable;
begin
  if (i>=0) and (i<tables.Count) then
    Result:=TWasmTable(tables[i])
  else
    Result:=nil;
end;

function TWasmModule.TableCount: Integer;
begin
  Result:=tables.Count;
end;

function TWasmModule.AddImport: TWasmImport;
begin
  Result:=TWasmImport.Create;
  imports.Add(Result);
end;

function TWasmModule.GetImport(i: integer): TWasmImport;
begin
  if (i>=0) and (i<imports.Count) then
    Result:=TWasmImport(imports[i])
  else
    Result:=nil;
end;

function TWasmModule.ImportCount: Integer;
begin
  Result:=imports.Count;
end;

function TWasmModule.AddFunc: TWasmFunc;
begin
  Result:=TWasmFunc.Create;
  funcs.Add(Result);
end;

function TWasmModule.AddType: TWasmFuncType;
begin
  Result:=TWasmFuncType.Create;
  types.Add(Result);
end;

function TWasmModule.GetFunc(i: integer): TWasmFunc;
begin
  if (i>=0) and (i<funcs.Count) then
    Result:=TWasmFunc(funcs[i])
  else
    Result:=nil;
end;

function TWasmModule.FuncCount: integer;
begin
  Result:=funcs.Count;
end;

function TWasmModule.GetType(i: integer): TWasmFuncType;
begin
  if (i>=0) and (i<types.Count) then
    Result:=TWasmFuncType(types[i])
  else
    Result:=nil;
end;

function TWasmModule.TypesCount: integer;
begin
  Result:=types.Count;
end;

function TWasmModule.AddExport: TWasmExport;
begin
  Result:=TWasmExport.Create;
  exp.add(Result);
end;

function TWasmModule.GetExport(i: integer): TWasmExport;
begin
  if (i>=0) and (i<exp.Count) then
    Result:=TWasmExport(exp[i])
  else
    Result:=nil;
end;

function TWasmModule.ExportCount: integer;
begin
  Result:=exp.Count;
end;

function TWasmModule.AddElement: TWasmElement;
begin
  Result:=TWasmElement.Create;
  elems.add(Result);
end;

function TWasmModule.GetElement(i: integer): TWasmElement;
begin
  if (i>=0) and (i<elems.Count) then
    Result:=TWasmElement(elems[i])
  else
    Result:=nil;
end;

function TWasmModule.ElementCount: Integer;
begin
  Result := elems.Count;
end;

function TWasmModule.AddData: TWasmData;
begin
  Result:=TWasmData.Create;
  data.Add(Result);
end;

function TWasmModule.GetData(i: integer): TWasmData;
begin
  if (i>=0) and (i<data.Count) then
    Result:=TWasmData(data[i])
  else
    Result:=nil;
end;

function TWasmModule.DataCount: Integer;
begin
  Result:=data.Count;
end;

function TWasmModule.AddGlobal: TWasmGlobal;
begin
  Result:=TWasmGlobal.Create;
  globals.Add(Result);
end;

function TWasmModule.GetGlobal(i: integer): TWasmGlobal;
begin
  if (i>=0) and (i<globals.Count) then
    Result:=TWasmGlobal(globals[i])
  else
    Result:=nil;
end;

function TWasmModule.GlobalCount: Integer;
begin
  Result:=globals.Count;
end;

{ TWasmFunc }

constructor TWasmFunc.Create;
begin
  inherited;
  locals:=TList.Create;
  instr:=TWasmInstrList.Create;
  functype:=TWasmFuncType.Create;
  idNum:=-1;
end;

destructor TWasmFunc.Destroy;
begin
  ClearList(locals);
  locals.Free;
  functype.Free;
  instr.Free;
  inherited Destroy;
end;

function TWasmFunc.AddLocal: TWasmParam;
begin
  Result:=TWasmParam.Create;
  locals.AdD(Result);
end;

function TWasmFunc.GetLocal(i: integer): TWasmParam;
begin
  if (i>=0) and (i<locals.Count) then
    Result:=TWasmParam(locals[i])
  else
    Result:=nil;
end;

function TWasmFunc.LocalsCount: integer;
begin
  result:=locals.Count;
end;

// registering new or finding the existing type for a function type
// it's assumed the function type is explicitly types
function RegisterFuncType(m: TWasmModule; funcType: TWasmFuncType): integer;
var
  i   : integer;
  trg : string;
  d   : string;
begin
  trg := WasmFuncTypeDescr(funcType);
  for i:=0 to m.TypesCount-1 do begin
    d := WasmFuncTypeDescr(m.GetType(i));
    if trg = d then begin
      Result:= i;
      Exit;
    end;
  end;
  Result:=m.TypesCount;
  funcType.CopyTo(m.AddType);
end;

// searching through TWasmParam list for the specified index-by-name
function FindParam(l: TList; const idx: string): Integer;
var
  i : integer;
begin
  if not Assigned(l) then begin
    Result:=-1;
    Exit;
  end;
  for i:=0 to l.Count-1 do
    if TWasmParam(l[i]).id=idx then begin
      Result:=i;
      Exit;
    end;
  Result:=-1;
end;

// finding functions by funcIdx
function FindFunc(m: TWasmModule; const funcIdx: string): integer;
var
  i  : integer;
  im : TWasmImport;
begin
  Result:=-1;
  for i:=0 to m.ImportCount-1 do begin
    im:=m.GetImport(i);
    if Assigned(im.fn) and (im.fn.id = funcIdx) then begin
      Result:=im.fn.idNum;
      Exit;
    end;
  end;

  for i:=0 to m.FuncCount-1 do
    if m.GetFunc(i).id = funcIdx then begin
      Result:=m.GetFunc(i).idNum;
      Exit;
    end;
end;

function FindGlobal(m: TWasmModule; const globIdx: string): integer;
var
  i  : integer;
  im : TWasmImport;
begin
  Result:=-1;
  {for i:=0 to m.ImportCount-1 do begin
    im:=m.GetImport(i);
    if Assigned(im.fn) and (im.fn.id = funcIdx) then begin
      Result:=im.fn.idNum;
      Exit;
    end;
  end;}

  for i:=0 to m.GlobalCount-1 do
    if m.GetGlobal(i).id.id = globIdx then begin
      Result:=m.GetGlobal(i).id.idNum;
      Exit;
    end;
end;

function GetFuncByNum(m: TWasmMOdule; const idNum: Integer): TWasmFunc;
var
  i : integer;
begin
  for i:=0 to m.FuncCount-1 do begin
    Result := m.GetFunc(i);
    if Assigned(Result) and (Result.idNum = idNum) then
      Exit;
  end;
  Result:=nil;
end;

function GetGlobalByNum(m: TWasmModule; const idNum: Integer): TWasmGlobal;
var
  i : integer;
begin
  for i:=0 to m.GlobalCount-1 do begin
    Result := m.GetGlobal(i);
    if Assigned(Result) and (Result.id.idNum = idNum) then
      Exit;
  end;
  Result:=nil;
end;

function GetMemByNum(m: TWasmModule; const idNum: Integer): TWasmMemory;
var
  i : integer;
begin
  for i:=0 to m.MemoryCount-1 do begin
    Result := m.GetMemory(i);
    if Assigned(Result) and (Result.id.idNum = idNum) then
      Exit;
  end;
  Result:=nil;
end;

// only looking up for the by the type index name
function FindFuncType(m: TWasmModule; const typeIdx: string): integer;
var
  i : integer;
begin
  Result:=-1;
  for i:=0 to m.TypesCount-1 do
    if m.GetType(i).typeIdx = typeIdx then begin
      Result:=i;
      Exit;
    end;
end;

function RegisterFuncIdxInElem(m: TWasmModule; const func: Integer): integer;
var
  el : TWasmElement;
  i  : Integer;
  ofs : Integer;
const
  NON_ZEROFFSET = 1; // being compliant with Linking convention
  // The output table elements shall begin at a non-zero offset within
  // the table, so that a call_indirect 0 instruction is guaranteed to fail.
  // Finally, when processing table relocations for symbols which
  // have neither an import nor a definition (namely, weakly-undefined
  // function symbols), the value 0 is written out as the value of the relocation.
  NON_ZEROFFSET_STR = '1';
begin
  if m.ElementCount=0 then begin
    el := m.AddElement;
    OperandSetInt32( el.offset.AddInstr(INST_i32_const).operand1, NON_ZEROFFSET);
    el.offset.AddInstr(INST_END);
  end else
    el := m.GetElement(0);

  if not InstrGetConsti32Value(el.offset, ofs) then ofs := 0;

  Result:=-1;
  for i:=0 to el.funcCount-1 do begin
    if el.funcs[i].idNum = func then
      Result:=i;
  end;
  if Result<0 then
    Result := el.AddFunc(func);

  Result := Result + ofs;
end;

function RegisterFuncInElem(m: TWasmModule; const funcId: string): integer;
var
  fnidx : integer;
begin
  fnidx := FindFunc(m, funcId);
  if fnidx>=0 then
    Result := RegisterFuncIdxInElem(m, fnidx)
  else
    Result := -1;
end;

function InstrGetConsti32Value(l: TWasmInstrList; var vl: Integer): Boolean;
var
  err : integer;
begin
  //todo: it must be more complicated than that
  Result:=Assigned(l) and (l.Count>0) and (l.Item[0].code = INST_i32_const);
  if not Result then Exit;

  vl := l.Item[0].operand1.s32; // todo: check the type
end;

end.
