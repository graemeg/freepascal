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

unit watparser;

{$mode delphi}{$H+}

interface

uses
  SysUtils, Classes, wasmtext, wasmmodule, watscanner, wasmbincode, wasmbin;

type
  TParseResult = record
    error  : string;
    line   : integer;
    pos    : integer;
    offset : integer;
  end;

const
  TokenStr : array[TWatToken] of string = (
     'uknown', 'error',
     'index',
     'string', 'number', '(', ')',
     'assembler symbol',

     'instruction',
     'func',
     'param', 'result',
     'module', 'mut', 'funcref',
     'i32', 'i64',
     'f32', 'f64',
     'type',
     'import', 'global', 'table', 'memory', 'local', 'export',
     'elem', 'data', 'offset','align','='
     );

  WasmTypeTokens = [wei32, wei64, wef32, wef64];

//function ConsumeToken(sc: TWatScanner; tk: TWatToken): Boolean;
function ParseModule(sc: TWatScanner; dst: TWasmModule; var errMsg: string): Boolean; overload;
function ParseModule(sc: TWatScanner; dst: TWasmModule; out err: TParseResult): Boolean; overload;

// castType can be otNotused, if no explicit type cast is expected
function ParseOperand(sc: TWatScanner; var op: TWasmInstrOperand; castType: TWasmInstrOperandType): Boolean;

implementation

type
  // used to stop the recursive parsing

  { EParserError }

  EParserError = class(Exception)
    offset : integer;
    constructor Create(const amsg: string; aofs: integer);
  end;

  TAsmSym = record
    name  : string;
    value : string;
  end;

  { TAsmSymList }

  TAsmSymList = class(TObject)
    syms  : array of TAsmSym;
    count : integer;
    procedure Push(const AName, AValue: string);
    procedure Clear;
    procedure ToLinkInfo(var AInfo: TLinkInfo);
  end;

const
  WAT_DEFTYPES='iN or fN';

procedure ParseError(sc: TWatScanner; const errMsg: string);
begin
  raise EParserError.Create(errMsg, sc.ofs);
end;

procedure ErrorExpectButFound(sc: TWatScanner; const expected: string; const butfound: string  =''); overload;
var
  r : string;
begin
  if butfound = '' then r := sc.resText
  else r := butfound;
  ParseError(sc, 'expected "'+expected+'", but "'+r+'" found');
end;

procedure ErrorUnexpectedEof(sc: TWatScanner);
begin
  ParseError(sc, 'unexpected end of file');
end;

procedure ConsumeAnyOpenToken(sc: TWatScanner; out tk: TWatToken;
  out hadOpenBrace: Boolean); overload;
begin
  hadOpenBrace := sc.token = weOpenBrace;
  if hadOpenBrace then sc.Next;
  tk:=sc.token;
end;

procedure ConsumeAnyOpenToken(sc: TWatScanner); overload;
var
  tk: TWatToken;
  op: Boolean;
begin
  ConsumeAnyOpenToken(sc, tk, op);
end;

procedure ConsumeAnyOpenToken(sc: TWatScanner; out tk: TWatToken); overload;
var
  op: Boolean;
begin
  ConsumeAnyOpenToken(sc, tk, op);
end;


function ConsumeOpenToken(sc: TWatScanner; tk: TWatToken): Boolean;
begin
  sc.Next;
  Result := (sc.token=weOpenBrace) or (sc.Token=tk);
  if Result and (sc.token=weOpenBrace) then begin
    sc.Next;
    Result := (sc.Token=tk);
  end;
end;

function ConsumeToken(sc: TWatScanner; tk: TWatToken): Boolean;
begin
  Result:=sc.token =tk;
  if not Result then
    ErrorExpectButFound(sc,TokenStr[tk])
  else
    sc.Next;
end;

function ParseNumOfId(sc: TWatScanner; out num: integer; out id: string): Boolean;
begin
  num:=-1;
  id:='';
  case sc.token of
    weNumber: num:=sc.resInt32;
    weIdent: id:=sc.resText;
  else
    ErrorExpectButFound(sc, 'index', TokenStr[sc.token]);
    Result := false;
  end;
  Result := true;
  if Result then sc.Next;
end;

function ParseId(sc: TWatScanner; var id: TWasmId): boolean;
begin
  Result := ParseNumOfId(sc, id.idNum, id.id);
end;

function TokenTypeToValType(t: TWatToken; out tp: byte): Boolean;
begin
  Result:=true;
  case t of
    wei32: tp:=valtype_i32;
    wei64: tp:=valtype_i64;
    wef32: tp:=valtype_f32;
    wef64: tp:=valtype_f64;
  else
    tp:=0;
    Result:=false;
  end;
end;

procedure ParseParam(sc: TWatScanner; out id: string; out tp: byte; allowIdent: Boolean = true; consumeClose: Boolean = true);
begin
  tp:=0;
  id:='';

  if allowIdent and (sc.token=weIdent) then begin
    id:=sc.resText;
    sc.Next;
  end;

  if not TokenTypeToValType(sc.token, tp) then
    ErrorExpectButFound(sc, WAT_DEFTYPES, TokenStr[sc.token]);

  sc.Next;

  if consumeClose then
    ConsumeToken(sc, weCloseBrace);
end;

procedure ParseNumOrIdx(sc: TWatScanner; out num: integer; out idx: string);
begin
  if sc.token = weIdent then begin
    idx := sc.resText;
    num := -1;
  end else if sc.token = weNumber then begin
    idx := '';
    num := sc.resInt32;
  end else
    ErrorExpectButFound(sc, 'number');
  sc.Next;
end;

// lookForRefId (if true) parses for the case of (type 0)
// (if false) just looks for (param i32) (result i32)
procedure ParseTypeUse(sc: TWatScanner; dst: TWasmFuncType; lookForRefId: Boolean);
var
  tk  : TWatToken;
  nm  : integer;
  id  : string;
  p   : TWasmParam;
begin
  tk := sc.token;
  if lookForRefId and (tk = weType) then begin
    sc.Next;
    if not ParseNumOfId(sc, nm, id) then
      Exit;

    if nm>=0 then dst.typeNum:=nm
    else dst.typeIdx:=id;
    ConsumeAnyOpenToken(sc, tk);
  end;

  while tk = weParam do begin
    p:=dst.AddParam;
    sc.Next;

    ParseParam(sc, p.id, p.tp, true, false);

    // Text format specification:
    // Abbreviations
    // Multiple anonymous parameters or results may be combined into a single declaration
    if (p.id = '') and (sc.token in [wei32, wei64, wef32, wef64]) then begin
      while (sc.token in [wei32, wei64, wef32, wef64]) do begin
        p:=dst.AddParam;
        TokenTypeToValType(sc.token, p.tp);
        sc.Next;
      end;
    end;
    ConsumeToken(sc, weCloseBrace);

    ConsumeAnyOpenToken(sc, tk);
  end;

  while tk = weResult do begin
    p:=dst.AddResult;
    sc.Next;
    ParseParam(sc, p.id, p.tp, false);
    ConsumeAnyOpenToken(sc, tk);
  end;
end;

function IntToAlign(aint: Integer): integer;
begin
  Result:=0;
  aint := aint shr 1;
  while aint>0 do begin
    aint := aint shr 1;
    inc(Result);
  end;
end;

procedure ParseInstrList(sc: TWatScanner; dst: TWasmInstrList);
var
  ci  : TWasmInstr;
  ft  : TWasmFuncType;
const
  ParamTypeToOpType : array [TInstParamType] of TWasmInstrOperandType = (
      otNotused, // ipNone
      otUInt32,  // ipLeb,        // label index or function index
      otUInt32,  // ipOfsAlign,   // memory arguments, ask for offset + align
      otSInt32,  // ipi32,        // signed Leb of maximum 4 bytes
      otNotused,  // ipi64,        // signed Leb of maximum 8 bytes
      otUInt32,  // ipu32,        // signed Leb of maximum 4 bytes
      otNotused,  // ipu64,        // signed Leb of maximum 8 bytes
      otFloat32,  // ipf32,       // float point single
      otFloat64,  // ipf64,       // float point double
      otNotused,  // ipJumpVec,   // an array of jump labels used for br_table only
      otNotused,  // ipResType,   // result type used for blocks, such as If, block or loop
      otNotused,  // ipCallType,  // used for call_indirect
      otNotused,  // ipi32OrFunc, // use for i32.const. Either a numeric OR function id is accepted.
      otUInt32    // ipZero       // followed by a single byte zero
  );
begin
  while sc.token=weInstr do begin
    ci := dst.AddInstr(sc.instrCode);
    sc.Next;

    case INST_FLAGS[ci.code].Param of
      ipNone:; // do nothing

      ipLeb:
        ParseNumOrIdx(sc, ci.operandNum, ci.operandIdx);

      ipOfsAlign: begin
        if sc.token = weOffset then begin
          sc.Next;
          ConsumeToken(sc, weEqual);
          if sc.token<>weNumber then ErrorExpectButFound(sc, 'number');
          //todo: fail on invalid value
          OperandSetInt32(ci.operand1, sc.resInt32(0));
          sc.Next;
        end;

        if sc.token = weAlign then begin
          sc.Next;
          ConsumeToken(sc, weEqual);
          if sc.token<>weNumber then ErrorExpectButFound(sc, 'number');
          OperandSetInt32(ci.operand2, IntToAlign(sc.resInt32(0)));
          sc.Next;
        end else
          OperandSetInt32(ci.operand2, INST_FLAGS[ci.code].align);
      end;

      ipi32,ipi64,ipu32,ipu64,ipf32,ipf64,ipi32OrFunc:
      begin
        if (INST_FLAGS[ci.code].Param = ipi32OrFunc) and (sc.token = weIdent) then
          OperandSetText(ci.operand1, sc.resText)
        else if sc.token<>weNumber then begin
          ErrorExpectButFound(sc, 'number');
          Exit;
        end else
          ParseOperand(sc, ci.operand1, ParamTypeToOpType[INST_FLAGS[ci.code].Param]);
        sc.Next;
      end;

      ipCallType:
      begin
        // call_indirect operator consists of 2 parameters
        //  1 - type call
        //  2 - table reference index. Which should always be zero.
        ConsumeToken(sc, weOpenBrace);
        ft := ci.addInstType;
        ParseTypeUse(sc, ft, true);
        ci.operandNum := 0; // table reference index
      end;

      //ip2Leb,  // memory arguments, ask for offset + align
      ipJumpVec: 
      begin
        while (sc.token in [weNumber, weIdent]) do begin
          if (ci.vecTableCount = length(ci.vecTable)) then begin
            if (ci.vecTableCount = 0) then SetLength(ci.vecTable, 2)
            else SetLength(ci.vecTable, ci.vecTableCount * 2);
          end;
          ParseId(sc, ci.vecTable[ci.vecTableCount]);
          inc(ci.vecTableCount);
        end;

        if (ci.vecTableCount<2) then begin
          ErrorExpectButFound(sc, 'label');
          Exit;
        end;

        dec(ci.vecTableCount);
        ci.operandIdx := ci.vecTable[ci.vecTableCount].id;
        ci.operandNum := ci.vecTable[ci.vecTableCount].idNum;
      end;

      ipResType:  // result type used for blocks, such as If, block or loop
      begin
        if sc.token = weIdent then begin
          ci.jumplabel := sc.resText;
          sc.Next;
        end;

        if (sc.token = weOpenBrace) then begin
          ConsumeToken(sc, weOpenBrace);
          ConsumeToken(sc, weResult);
          case sc.token of
            wei32: ci.operandNum := valtype_i32;
            wei64: ci.operandNum := valtype_i64;
            wef32: ci.operandNum := valtype_f32;
            wef64: ci.operandNum := valtype_f64;
          else
            ErrorExpectButFound(sc, 'i32');
          end;
          sc.Next;
          ConsumeToken(sc, weCloseBrace);
        end else
          ci.operandNum := block_type; // no value type
      end;

    end;
  end;

end;


procedure ParseFunc(sc: TWatScanner; dst: TWasmFunc);
var
  tk  : TWatToken;
  p   : TWasmParam;
begin
  if sc.token=weFunc then sc.Next;

  if sc.token=weIdent then begin
    dst.id:=sc.resText;
    sc.Next;
  end;

  ConsumeAnyOpenToken(sc, tk);

  if tk in [weType, weParam, weResult] then begin
    ParseTypeUse(sc, dst.functype, true);
    ConsumeAnyOpenToken(sc, tk);
  end;

  while tk = weLocal do begin
    p:=dst.AddLocal;
    sc.Next;
    ParseParam(sc, p.id, p.tp, true, false);
    if p.id = '' then begin
      while sc.token in WasmTypeTokens do begin
        p:=dst.AddLocal;
        TokenTypeToValType(sc.token, p.tp);
        sc.Next;
      end;
    end;
    if sc.token=weCloseBrace then sc.Next;

    ConsumeAnyOpenToken(sc, tk);
  end;

  if not (sc.token in [weInstr, weCloseBrace]) then
    ErrorExpectButFound(sc, 'identifier');

  ParseInstrList(sc, dst.instr);
  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseTypeDef(sc: TWatScanner; dst: TWasmFuncType);
begin
  if sc.token=weType then sc.Next;

  if (sc.token in [weNumber, weIdent]) then
    ParseNumOfId(sc, dst.typeNum, dst.typeIdx);

  ConsumeToken(sc, weOpenBrace);
  ConsumeToken(sc, weFunc);

  if (sc.token = weOpenBrace) then begin
    sc.Next;
    ParseTypeUse(sc, dst, false);
    ConsumeToken(sc, weCloseBrace);
  end;

  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseGlobal(sc: TWatScanner; dst: TWasmGlobal);
var
  allowValue: Boolean;
begin
  if sc.token = weGlobal then sc.Next;

  allowValue := true;
  // parsing id
  if (sc.token in [weIdent, weNumber]) then ParseId(sc, dst.id);

  // import or export
  if (sc.token=weOpenBrace) then begin
    sc.Next;
    if sc.token=weImport then begin
      // import
      allowValue := false;
    end else if sc.token=weExport then begin
     // export
    end;
  end;

  // parsing type. Global can be mutable type (mut i32)

  if (sc.token=weOpenBrace) then sc.Next;
  if sc.token = weMut then begin
    dst.isMutable := true;
    sc.Next;
  end;

  if (sc.token in WasmTypeTokens) then begin
    TokenTypeToValType(sc.token, dst.tp);
    sc.Next;
  end else
    ErrorExpectButFound(sc, 'type');

  if dst.isMutable then ConsumeToken(sc, weCloseBrace);

  if allowValue and (sc.token = weOpenBrace) then begin
    sc.Next;
    ParseInstrList(sc, dst.StartValue);
    ConsumeToken(sc, weCloseBrace);
  end;
  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseData(sc: TWatScanner; dst: TWasmData);
var
  l : integer;
begin
  if sc.token=weData then sc.Next;

  //id (if not exists, should be zero)
  if sc.token in [weIdent, weNumber] then
    ParseId(sc, dst.id);

  // offset (if not exist, should be zero)
  if (sc.token = weOpenBrace) then begin
    sc.Next;
    ParseInstrList(sc, dst.StartOffset);
    ConsumeToken(sc, weCloseBrace);
  end;

  // data (if not exist, then blank)
  if sc.token = weString then begin
    l := length(sc.resText);
    SetLength(dst.databuf, l);
    if l>0 then
      Move(sc.resText[1], dst.databuf[0], l);
    sc.Next;
  end;

  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseMemory(sc: TWatScanner; dst: TWasmMemory);
begin
  if sc.token=weMemory then sc.Next;

  if sc.token in [weIdent] then
    ParseId(sc, dst.id);

  // todo: parsing of Import / Export

  if sc.token in [weNumber] then begin
    dst.min := sc.resInt32(0);
    sc.Next;
  end;

  if sc.token in [weNumber] then begin
    dst.max := sc.resInt32(0);
    sc.Next;
  end;

  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseExport(sc: TWatScanner; dst: TWasmExport);
begin
  if sc.token=weExport then
    sc.Next;

  if sc.token<>weString then
    ErrorExpectButFound(sc, 'string');

  dst.name := sc.resWasmString;
  sc.Next;

  ConsumeAnyOpenToken(sc);
  case sc.token of
    weFunc:  dst.exportType:=EXPDESC_FUNC;
    weTable: dst.exportType:=EXPDESC_TABLE;
    weMemory: dst.exportType:=EXPDESC_MEM;
    weGlobal: dst.exportType:=EXPDESC_GLOBAL;
  else
    ErrorExpectButFound(sc, 'func');
  end;

  sc.Next;
  case sc.token of
    weNumber:
      dst.exportNum := sc.resInt32;
    weIdent:
      dst.exportIdx := sc.resText;
  else
    ErrorExpectButFound(sc, 'index');
  end;
  sc.Next;
  ConsumeToken(sc, weCloseBrace);
  ConsumeToken(sc, weCloseBrace);
end;

procedure ConsumeAsmSym(sc: TWatScanner; dst: TAsmSymList);
begin
  dst.Push(sc.asmCmd, sc.resText);
  sc.Next;
end;

// parseIdOffset - should only be used for elems declareted at module leve
// if elems declared in a table, parseIdOffset should be set to false
procedure ParseElem(sc: TWatScanner; dst: TWasmElement; parseIdOffset: Boolean);
var
  vid : TWasmId;
begin
  if sc.token = weElem then sc.Next;

  if parseIdOffset then begin
    ParseId(sc, dst.tableId);
    if (sc.token = weOpenBrace) then begin
      sc.Next;
      ParseInstrList(sc, dst.AddOffset);
      ConsumeToken(sc, weCloseBrace);
    end;
  end;

  while sc.token in [weIdent, weNumber] do begin
    ParseId(sc, vid);
    dst.AddFuncId(vid);
  end;
  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseTable(sc: TWatScanner; dst: TWasmTable);
begin
  if sc.token = weTable then sc.Next;

  // table ident can be missing? If missing, then it's zero
  if (sc.token in [weIdent, weNumber]) then
    ParseId(sc, dst.id);

  ConsumeToken(sc, weFuncRef);
  dst.elemsType := elem_type;

  // consuming elements
  if (sc.token = weOpenBrace) then begin
    sc.Next;
    ParseElem(sc, dst.AddElem, false);
  end;

  ConsumeToken(sc, weCloseBrace);
end;

procedure ParseImport(sc: TWatScanner; dst: TWasmImport);
var
  tk      : TWatToken;
begin
  if sc.token=weImport then
    sc.Next;

  if sc.token<>weString then
    ErrorExpectButFound(sc, 'string');
  dst.module := sc.resWasmString;
  sc.Next;

  if sc.token<>weString then
    ErrorExpectButFound(sc, 'string');
  dst.name := sc.resWasmString;
  sc.Next;

  ConsumeAnyOpenToken(sc, tk);
  case tk of
    weAsmSymbol: ;
    weFunc: begin
      ParseFunc(sc, dst.AddFunc);
    end;
    weMemory: begin
      ParseMemory(sc, dst.AddMemory);
    end;
    weTable: begin
      ParseTable(sc, dst.AddTable);
    end;
    weGlobal: begin
      ParseGlobal(sc, dst.AddGlobal);
    end;
  else
    ErrorExpectButFound(sc, 'importdesc', TokenStr[sc.token]);
  end;
  ConsumeToken(sc, weCloseBrace);
end;


procedure ParseModuleInt(sc: TWatScanner; dst: TWasmModule);
var
  tk      : TWatToken;
  symlist : TAsmSymList;
  f       : TWasmFunc;
  imp     : TWasmImport;
  m       : TWasmMemory;
  g       : TWasmGlobal;
  e       : TWasmElement;
begin
  if not ConsumeOpenToken(sc, weModule) then
    ErrorExpectButFound(sc, 'module');

  symlist := TAsmSymList.Create;
  try
    sc.Next;
    ConsumeAnyOpenToken(sc, tk);
    while tk <> weCloseBrace do begin
      case tk of
        weAsmSymbol:
          ConsumeAsmSym(sc, symlist);
        weImport: begin
          imp:=dst.AddImport;
          symlist.ToLinkInfo(imp.LinkInfo);
          ParseImport(sc, imp);
          symlist.Clear;
        end;
        weTable: begin
          ParseTable(sc, dst.AddTable)
        end;
        weFunc: begin
          f:=dst.AddFunc;
          symlist.ToLinkInfo(f.LinkInfo);
          symlist.Clear;
          ParseFunc(sc, f);
        end;
        weMemory:
        begin
          m:=dst.AddMemory;
          symlist.ToLinkInfo(f.LinkInfo);
          symlist.Clear;
          ParseMemory(sc, m);
        end;
        weExport:
        begin
          ParseExport(sc, dst.AddExport);
          symlist.Clear;
        end;
        weData:begin
          ParseData(sc, dst.AddData);
          symlist.Clear;
        end;
        weType: begin
          symlist.Clear;
          ParseTypeDef(sc, dst.AddType);
        end;
        weGlobal: begin
          g:=dst.AddGlobal;
          symlist.ToLinkInfo(g.LinkInfo);
          symlist.Clear;
          ParseGlobal(sc, g);
        end;
        weElem: begin
          e:=dst.AddElement;
          symlist.Clear;
          ParseElem(sc, e, true);
        end;
      else
        ErrorExpectButFound(sc, 'func', TokenStr[sc.token]);
      end;
      ConsumeAnyOpenToken(sc, tk);
    end;
    ConsumeToken(sc, weCloseBrace);
  finally
    symlist.Free;
  end;
end;

function ParseModule(sc: TWatScanner; dst: TWasmModule; var errMsg: string): Boolean;
var
  res : TParseResult;
begin
  Result := ParseModule(sc, dst, res);
  if not Result then begin
    errMsg:=Format('line: %d, pos: %d, %s', [res.line, res.pos, res.error]);
  end else
    errMsg:='';
end;

procedure GetLineAndPos(const buf: string; ofs: integer; out line, pos: integer);
var
  i: integer;
  ll: integer;
begin
  i:=1;
  line:=1;
  ll:=1;
  while (i<=length(buf)) and (i<ofs) do begin
    if (buf[i]=#13) or (buf[i]=#10) then begin
      inc(line);
      if (i<=length(buf)) and (i<ofs) and ((buf[i]=#13) or (buf[i]=#10)) and (buf[i] <> buf[i-1]) then
        inc(i);
      ll:=i;
    end;
    inc(i);
  end;
  pos:=ofs - ll;
end;

function ParseModule(sc: TWatScanner; dst: TWasmModule; out err: TParseResult): Boolean;
begin
  try
    err.error:='';
    err.pos:=0;
    err.line:=0;
    err.offset:=0;
    ParseModuleInt(sc, dst);
    Result:=true;
  except
    on x: EParserError do begin
      err.error := x.Message;
      err.offset := x.offset;
      GetLineAndPos(sc.buf, x.offset, err.line, err.pos);
      Result:=false;
    end;
  end;
end;

{ TAsmSymList }

procedure TAsmSymList.Push(const AName, AValue: string);
var
  i : integer;
begin
  for i:=0 to count-1 do
    if syms[i].name = Aname then begin
      syms[i].value := AValue;
      Exit;
    end;

  if count=length(syms) then begin
    if count=0 then SetLength(syms, 4)
    else SetLength(syms, count*2);
  end;
  syms[count].name:=AName;
  syms[count].value:=Avalue;
  inc(count);
end;

procedure TAsmSymList.Clear;
begin
  count:=0;
end;

procedure TAsmSymList.ToLinkInfo(var AInfo: TLinkInfo);
var
  i : integer;
begin
  for i:=0 to count-1 do begin
    if syms[i].name = 'name' then
      AInfo.Name := syms[i].value
    else if syms[i].name = 'weak' then
      AInfo.Binding := lbWeak
    else if syms[i].name = 'local' then
      AInfo.Binding := lbLocal
    else if syms[i].name = 'hidden' then
      Ainfo.isHidden := true
    else if syms[i].name = 'undef' then
      AInfo.isUndefined := true
    else if syms[i].name = 'nostrip'  then
      AInfo.NoStrip := true
    else if syms[i].name = 'forhost' then
      AInfo.Binding := lbForHost;
  end;
end;

{ EParserError }

constructor EParserError.Create(const amsg: string; aofs: integer);
begin
  inherited Create(amsg);
  offset:=aofs;
end;

const
  INT64MASK = $FFFFFFFF00000000;
  maxInt64  = 9223372036854775807;

function Int64ToOperand(v: Int64; var op: TWasmInstrOperand; castType: TWasmInstrOperandType): Boolean;
begin
  Result := true;
  case castType of
    otNotused:
      if (v and INT64MASK = 0) then begin
        if (v<0) or (v < maxLongint) then begin
          op.tp := otSInt32;
          op.s32 := v;
        end else begin
          op.tp := otUInt32;
          op.u32 := v;
        end
      end else begin
        op.tp := otSInt64;
        op.s64 := v;
      end;

    otSInt32:
    begin
      Result := (v and INT64MASK = 0);
      if not Result then Exit;
      op.tp := otSInt32;
      op.s32 := v;
    end;

    otUInt32:
    begin
      Result := (v and INT64MASK = 0) and (v>=0);
      if not Result then Exit;
      op.tp := otUInt32;
      op.u32 := v;
    end;

    otSInt64: begin
      op.tp := otSInt32;
      op.s64 := v;
    end;

    otUInt64: begin
      Result := (v>=0);
      op.tp := otUInt64;
      op.u64 := v;
    end;

    otFloat32: begin
      op.tp := otFloat32;
      op.f32 := v;
    end;

    otFloat64: begin
      op.tp := otFloat64;
      op.f64 := v;
    end;
  end
end;

function UInt64ToOperand(v: UInt64; var op: TWasmInstrOperand; castType: TWasmInstrOperandType): Boolean;
begin
  Result := true;
  case castType of
    otNotused:
      if (v and INT64MASK = 0) then begin
        if (v < maxLongint) then begin
          op.tp := otSInt32;
          op.s32 := v;
        end else begin
          op.tp := otUInt32;
          op.u32 := v;
        end
      end else begin
        if v < maxInt64 then begin
          op.tp := otSInt64;
          op.s64 := v;
        end else begin
          op.tp := otUInt64;
          op.u64 := v;
        end;
      end;

    otSInt32:
    begin
      Result := (v and INT64MASK = 0) and (v < maxLongint);
      if not Result then Exit;
      op.tp := otSInt32;
      op.s32 := v;
    end;

    otUInt32:
    begin
      Result := (v and INT64MASK = 0);
      if not Result then Exit;
      op.tp := otUInt32;
      op.u32 := v;
    end;

    otSInt64: begin
      Result := (v <= maxInt64);
      if not Result then Exit;
      op.tp := otSInt32;
      op.s64 := v;
    end;

    otUInt64: begin
      op.tp := otUInt64;
      op.u64 := v;
    end;

    otFloat32: begin
      op.tp := otFloat32;
      op.f32 := v;
    end;

    otFloat64: begin
      op.tp := otFloat64;
      op.f64 := v;
    end;
  end
end;

function TextToFloat32(const txt: string; var vl: Single): Boolean;
var
  err : integer;
  l   : LongWord;
  i   : Integer;
  hx  : string;
  hl  : LongWord;
const
  BINARY_INF    = $7f800000;
  BINARY_NEGINF = $ff800000;
  BINARY_NEGMAN = $00400000;
begin
  // val() doesn't handle "nan" in wasm compatible
  // any "nan" is always returned as "negative nan" (in Wasm terms)
  // "inf" works just fine
  Result := true;
  if (Pos('nan', txt)>0) then begin
    if txt[1]='-' then l := NtoLE(BINARY_NEGINF)
    else l := NtoLE(BINARY_INF);

    i:=Pos(':', txt);
    if i>0 then begin
      hx := '$'+Copy(txt, i+3, length(txt)); // skipping '0x'
      Val(hx, hl, err);
    end else
      hl:=BINARY_NEGMAN; // nan
    l:=l or hl;
    vl := PSingle(@l)^;
  end else begin
    vl:=0;
    err:=0;
    if (Pos('0x', txt)>0) then
      vl := HexFloatStrToSingle(txt)
    else begin
      Val(txt, vl, err);
      Result := err = 0;
    end;
  end;
end;

function TextToFloat64(const txt: string; var vl: Double): Boolean;
var
  err : integer;
  l   : QWord;
  i   : Integer;
  hx  : string;
  hl  : QWord;
const
  BINARY_INF    = QWord($7ff0000000000000);
  BINARY_NEGINF = QWord($fff0000000000000);
  BINARY_NEGMAN = QWord($0008000000000000);
begin
  Result := true;
  if (Pos('nan', txt)>0) then begin
    if txt[1]='-' then l := NtoLE(BINARY_NEGINF)
    else l := NtoLE(BINARY_INF);

    i:=Pos(':', txt);
    if i>0 then begin
      hx := '$'+Copy(txt, i+3, length(txt)); // skipping '0x'
      Val(hx, hl, err);
    end else
      hl:=BINARY_NEGMAN; // nan
    l:=l or hl;
    vl := PDouble(@l)^;
  end else begin
    vl:=0;
    if (Pos('0x', txt)>0) then
      vl := HexFloatStrToDouble(txt)
    else begin
      Val(txt, vl, err);
      Result := err = 0;
    end;
  end;
end;

function ParseOperand(sc: TWatScanner; var op: TWasmInstrOperand; castType: TWasmInstrOperandType): Boolean;
var
  i64 : Int64;
  u64 : UInt64;
  err : Integer;
begin
  Result := Assigned(sc);
  if not Result then Exit;

  if (castType = otText) or (sc.token <> weNumber) then begin
    OperandSetText(op, sc.resText);
    Exit;
  end;

  case sc.numformat of
    wnfInteger, wnfHex: begin
      val(sc.resText, i64, err);
      if err = 0 then
        Result := Int64ToOperand(i64, op, castType)
      else begin
        Val(sc.resText, u64, err);
        Result := err = 0;
        if Result then
          Result := UInt64ToOperand(u64, op, castType);
      end;
    end;
    wnfFloat, wnfFloatHex: begin   // 0.000
      Result := castType in [otNotused, otFloat32, otFloat64];
      if not Result then Exit;
      if castType = otFloat32 then begin
        op.tp := otFloat32;
        TextToFloat32(sc.resText, op.f32);
      end else begin
        op.tp := otFloat64;
        TextToFloat64(sc.resText, op.f64);
      end;
    end;
  end;
end;

end.
