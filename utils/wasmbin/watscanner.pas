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

unit watscanner;

{$mode delphi}{$H+}

interface

uses
  SysUtils, Classes, parseutils, wasmtext;

type
  TWatToken = (weNone, weError,
     weIdent,
     weString, weNumber, weOpenBrace, weCloseBrace,
     weAsmSymbol,

     weInstr,
     weFunc,
     weParam, weResult,
     weModule, weMut, weFuncRef,
     wei32, wei64,
     wef32, wef64,
     weType,
     weImport, weGlobal, weTable, weMemory, weLocal, weExport,
     weElem, weData, weOffset, weAlign, weEqual
   );

  // used only for weNumber
  TWatNumberFormat = (
     wnfNo,      // other than number
     wnfInteger, // 00
     wnfHex,     // 0xABC
     wnfFloat,   // 0.000
     wnfFloatHex // 0x000.bced
  );

  THexStr = record
    num   : QWord;
    frac  : QWord;
    exp   : integer;
    isNeg : Boolean;
  end;

  { TWatScanner }

  TWatScanner = class(TObject)
  protected
    procedure DoComment(ofs: Integer; const cmt: string); virtual;
    function CommentIsSymbol(const cmt: string): Boolean;
  public
    buf       : string;
    idx       : integer;

    instrCode : byte;
    ofs       : integer;
    token     : TWatToken;
    numformat : TWatNumberFormat;
    resText   : string;
    asmCmd    : string;

    skipAsmSym : Boolean;
    procedure SetSource(const abuf: string);
    function Next: Boolean;

    function resInt32(const def: integer=-1): Integer;
    function resWasmString: string;
  end;

const
  // see Identifiers of Textual format
  IdStart  = '$';
  IdBody   = AlphaNumChars
             + [ '!' ,'#' ,'$' ,'%' ,'&' ,'''' ,'*'
                ,'+' ,'-' ,'.' ,'/' ,':' ,'<' ,'='
                ,'>' ,'?' ,'@' ,'\' ,'^' ,'_' ,'`'
                ,'|' ,'~'];
  GrammarChars  = AlphaNumChars+['.','_'
                    ,'/' // some old instructions are like that: "f32.reinterpret/i32"
                  ];

procedure GetGrammar(const txt: string; out entity: TWatToken; out instByte: byte);

const
  KEY_MODULE = 'module';
  KEY_FUNC   = 'func';
  KEY_FUNCREF = 'funcref';
  KEY_I32    = 'i32';
  KEY_I64    = 'i64';
  KEY_F32    = 'f32';
  KEY_F64    = 'f64';
  KEY_PARAM  = 'param';
  KEY_RESULT = 'result';
  KEY_MUT    = 'mut';
  KEY_TYPE   = 'type';

  KEY_IMPORT = 'import';
  KEY_GLOBAL = 'global';
  KEY_TABLE  = 'table';
  KEY_MEMORY = 'memory';
  KEY_LOCAL  = 'local';
  KEY_EXPORT = 'export';
  KEY_ELEM   = 'elem';
  KEY_DATA   = 'data';
  KEY_OFFSET = 'offset';

function ScanString(const buf: string; var idx: integer): string;

function HexFloatStrToHexStr(const t: string; out hexStr: THexStr): Boolean;
function HexFracToSingle(const num, frac: QWord; exp: Integer; isNeg: Boolean): Single;
function HexFloatStrToSingle(const hexstr: string): Single;
function HexFracToDouble(const num, frac: QWord; exp: Integer; neg: Boolean): Double;
function HexFloatStrToDouble(const hexstr: string): double;

implementation

procedure GetGrammar(const txt: string; out entity: TWatToken; out instByte: byte);
begin
  instByte:=0;
  entity:=weError;
  if txt='' then Exit;
  case txt[1] of
    'a':
      if txt='anyfunc' then entity:=weFuncRef
      else if txt = 'align' then entity:=weAlign
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'd':
      if txt=KEY_DATA then entity:=weData
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'e':
      if txt=KEY_EXPORT then entity:=weExport
      else if txt=KEY_ELEM then entity:=weElem
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'i':
      if txt=KEY_I32 then entity:=wei32
      else if txt=KEY_I64 then entity:=wei64
      else if txt=KEY_IMPORT then entity:=weImport
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'g':
      if txt=KEY_GLOBAL then entity:=weGlobal
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'f':
      if txt=KEY_FUNC then entity:=weFunc
      else if txt=KEY_FUNCREF then entity:=weFuncRef
      else if txt=KEY_F32 then entity:=wef32
      else if txt=KEY_F64 then entity:=wef64
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'l':
      if txt=KEY_LOCAL then entity:=weLocal
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'm':
      if txt=KEY_MODULE then entity:=weModule
      else if txt = KEY_MUT then entity:=weMut
      else if txt = KEY_MEMORY then entity:=weMemory
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'o':
      if txt=KEY_OFFSET then entity:=weOffset
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'p':
      if txt=KEY_PARAM then entity:=weParam
      else if TextToInst(txt, instByte) then entity:=weInstr;
    'r':
      if txt=KEY_RESULT then entity:=weResult
      else if TextToInst(txt, instByte) then entity:=weInstr;
    't':
      if txt=KEY_TYPE then entity:=weType
      else if txt=KEY_TABLE then entity:=weTable
      else if TextToInst(txt, instByte) then entity:=weInstr;
  else
    if TextToInst(txt, instByte) then entity:=weInstr;
  end;
end;

{ TWatScanner }

procedure TWatScanner.DoComment(ofs: Integer; const cmt: string);
begin

end;

function TWatScanner.CommentIsSymbol(const cmt: string): Boolean;
var
  i: integer;
  t: string;
  v: string;
begin
  Result := false;
  if (Pos(';;',cmt)<>1) then Exit;
  i:=3;
  ScanWhile(cmt, i, SpaceChars);
  if (i>length(cmt)) or (cmt[i]<>'.') then Exit;

  inc(i);
  t := AnsiLowerCase(ScanTo(cmt, i, SpaceChars));
  ScanWhile(cmt, i, SpaceChars);
  v := ScanTo(cmt, i, SpaceChars);

  asmCmd := t;
  resText := v;
  Result := true;
end;

procedure TWatScanner.SetSource(const abuf: string);
begin
  buf:=abuf;
  idx:=1;
end;

function ScanString(const buf: string; var idx: integer): string;
var
  j : integer;
begin
  if buf[idx]<>'"' then begin
    Result:='';
    Exit;
  end;
  j:=idx;
  inc(idx);
  while (buf[idx]<>'"') and (idx<length(buf)) do begin
    if buf[idx]='\' then inc(idx);
    inc(idx);
  end;
  inc(idx);
  Result:=Copy(buf, j, idx-j);
end;

function TWatScanner.Next: Boolean;
var
  cmt : string;
  done: boolean;
  fmt : TCNumberFormat;
  si  : integer;
begin
  numformat := wnfNo;

  Result := idx<=length(buf);
  if not Result then Exit;

  done:=false;
  resText:='';
  while not done do begin
    ScanWhile(buf, idx, SpaceEolnChars);
    Result := idx<=length(buf);
    if not Result then Exit;
    ofs:=idx;
    if (idx<length(buf)) and (buf[idx] in [';','(']) and (buf[idx+1]=';') then begin
      if (buf[idx]=';') then begin
        // comment until the end of the line
        cmt := ScanTo(buf, idx, EoLnChars);
        ScanWhile(buf, idx, EoLnChars);
      end else
        // comment until the ;)
        cmt := ScanToSubstr(buf, idx, ';)');

      if not skipAsmSym and CommentIsSymbol(cmt) then begin
        token:=weAsmSymbol;
        done:=true;
      end else
        DoComment(ofs, cmt);
    end else begin
      done:=true;
      if buf[idx] = '(' then begin
        token:=weOpenBrace;
        inc(idx);
      end else if buf[idx]=')' then begin
        token:=weCloseBrace;
        inc(idx);
      end else if buf[idx]='=' then begin
        token:=weEqual;
        inc(idx);
      end else if buf[idx]='"' then begin
        token:=weString;
        resText:=ScanString(buf, idx);
      end else if buf[idx] = IdStart then begin
        token:=weIdent;
        resText:=ScanWhile(buf, idx, IdBody);
      end else if buf[idx] in SignNumericChars then begin
        fmt := ScanNumberC(buf, idx, resText);
        if fmt = nfError then begin
          token := weError;
          Exit;
        end else
          token:=weNumber;
        case fmt of
          nfFloat: numformat := wnfFloat;
          nfFloatHex: numFormat := wnfFloatHex;
          nfHex: numformat := wnfHex;
        else
          numformat := wnfInteger;
        end;
      end else if buf[idx] in GrammarChars then begin
        si := idx;
        resText:=ScanWhile(buf, idx, GrammarChars);

        // second try for  the number
        if (resText = 'nan') or (resText = 'inf') then begin
          idx := si;
          fmt := ScanNumberC(buf, idx, resText);
          if fmt = nfError then begin
            token := weError;
            Exit;
          end else
            token:=weNumber;
          case fmt of
            nfFloat: numformat := wnfFloat;
            nfHex: numformat := wnfHex;
          else
            numformat := wnfInteger;
          end;
        end else
          GetGrammar(resText, token, instrCode);
        done:=true;
      end else begin
        token:=weError;
        inc(idx);
        done:=true;
      end;
    end;
  end;

  if resText='' then
    resText := Copy(buf, ofs, idx-ofs);
end;

function TWatScanner.resInt32(const def: integer=-1): Integer;
var
  err: integer;
begin
  Val(resText, Result, err);
  if err<>0 then Result:=def;
end;

function TWatScanner.resWasmString: string;
var
  i : integer;
  j : integer;
begin
  if token<>weString then begin
    Result:='';
    Exit;
  end;
  Result:=Copy(resText, 2, length(resText)-2);
  if Result='' then Exit;

  i:=1;
  j:=1;
  while i<=length(Result) do begin
    if Result[i]='\' then begin
      inc(i);
      if i<=length(Result) then
        case Result[i] of
          'r': Result[j]:=#13;
          'n': Result[j]:=#10;
          '\': Result[j]:='\';
          '"': Result[j]:='"';
        end;
    end else
      if (j<i) then Result[j]:=Result[i];
    inc(j);
    inc(i);
  end;
  SetLength(Result, j-1);
end;


function HexFloatStrToHexStr(const t: string; out hexStr: THexStr): Boolean;
var
  i : integer;
  j : integer;
  err : Integer;
const
  HexChars = ['0'..'9','a'..'f','A'..'F'];
begin
  hexStr.isNeg:=false;
  hexStr.num:=0;
  hexStr.frac:=0;
  hexStr.exp:=0;
  if (t='') then begin
    Result:=true;
    Exit;
  end;

  i:=1;
  hexStr.isNeg:=t[i]='-';
  if (hexStr.isNeg) then inc(i);
  inc(i,2); // skipping '0x'

  j:=i;
  while (i<=length(t)) and (t[i] in HexChars) do inc(i);
  Val('$'+Copy(t, j, i-j), hexStr.num, err);
  Result:=err=0;
  if not Result then Exit;

  if (t[i]='.') then begin
    inc(i);
    j:=i;
    while (i<=length(t)) and (t[i] in HexChars) do inc(i);
    Val('$'+Copy(t, j, i-j), hexStr.frac, err);
    Result:=err=0;
    if not Result then Exit;
  end;

  Result := (i<=length(t)) and (t[i] = 'p') or (t[i]='P');
  inc(i);
  Val(Copy(t, i, length(t)), hexStr.exp, err);
  Result:=err=0;
end;

function HexFracToSingle(const num, frac: QWord; exp: Integer; isNeg: Boolean): Single;
var
  x      : QWord;
  nm     : QWord;
  adjexp : integer;
  sr     : TSingleRec;
begin
  nm := num;
  x := frac;
  adjexp := -1;
  while (nm > 0) do begin
    x:=(x shr 1) or ((nm and 1) shl 23);
    nm := nm shr 1;
    inc(adjexp);
  end;
  sr.Exp:=127 + exp + adjexp;
  sr.Frac:=x;
  sr.Sign:=isNeg;
  Result := sr.Value;
end;

function HexFloatStrToSingle(const hexstr: string): Single;
var
  st : THexStr;
begin
  HexFloatStrToHexStr(hexstr, st);
  Result:=HexFracToSingle(st.num, st.frac, st.exp, st.isNeg);
end;

function HexFracToDouble(const num, frac: QWord; exp: Integer; neg: Boolean): Double;
var
  x      : QWord;
  nm     : QWord;
  adjexp : integer;
  sr     : TDoubleRec;
begin
  nm := num;
  x := frac;
  adjexp := 0;
  while (nm > 1) do begin
    x:=(x shr 1) or ((nm and 1) shl 52);
    nm := nm shr 1;
    inc(adjexp);
  end;
  sr.Exp:=1023 + exp + adjexp;
  sr.Frac:=x;
  sr.Sign:=neg;
  Result := sr.Value;
end;

function HexFloatStrToDouble(const hexstr: string): double;
var
  st : THexStr;
begin
  HexFloatStrToHexStr(hexstr, st);
  Result:=HexFracToDouble(st.num, st.frac, st.exp, st.isNeg);
end;

end.
