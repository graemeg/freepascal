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

unit wasmlinkchange;

{$mode objfpc}{$H+}

interface

uses Classes, SysUtils, wasmlink, wasmbin, lebutils;

type
  TSymbolType = (
    st_Nochange,
    st_Local,     // 'L'
    st_Weak,      // 'W'
    st_VisHidden, // 'H' visibility-local
    st_VisDefault // 'D' visibility default (not-hidden)
  );
  TSymbolTypes = set of TSymbolType;

  TSymbolConfigure = class(TObject)
    symname  : string;
    needtype : TSymbolType;
  end;

procedure ReadSymbolsConf(const fn: string; dst: TStrings);
procedure ReadSymbolsConf(src: TStream; dst: TStrings);

procedure ProcessLinkingSection(st: TStream; syms: TStrings);

implementation

procedure ReadSymbolsConf(const fn: string; dst: TStrings);
var
  fs: TFileStream;
begin
  fs:=TFileStream.Create(fn, fmOpenRead or fmShareDenyNone);
  try
    ReadSymbolsConf(fs, dst);
  finally
    fs.Free;
  end;
end;

function StrToSymType(const s: string): TSymbolType;
begin
  if length(s)=0 then
    Result:=st_Nochange
  else
  case upCase(s[1]) of
    'L': Result:=st_Local;
    'H': Result:=st_VisHidden;
    'W': Result:=st_Weak;
    'D': Result:=st_VisDefault;
  else
    Result:=st_Nochange;
  end;
end;

function StrToSymTypes(const s: string): TSymbolTypes;
var
  i: integer;
  tt : TSymbolType;
begin
  Result := [];
  for i:=1 to length(s) do begin
    tt := StrToSymType(s[i]);
    if tt <> st_Nochange then
      Include(Result, tt);
  end;
end;


procedure ReadSymbolsConf(src: TStream; dst: TStrings);
begin
  dst.LoadFromStream(src);
end;

procedure ProcessLinkingSection(st: TStream; syms: TStrings);
var
  mt  : TLinkingMetadata;
  //en  : Int64;
  sub : TLinkingSubSection;
  cnt : LongWord;
  nx  : Int64;
  i   : integer;
  si  : TSymInfo;
  ofs : Int64;
  v   : string;
  tt  : TSymbolType;
  tts : TSymbolTypes;
  fl  : LongWord;
begin
  //en := st.Position+secsize;
  ReadMetadata(st, mt);
  writeln('version: ', mt.version);
  //while st.Position<en do begin
    ReadLinkSubSect(st, sub);
    nx := st.Position+sub.length;

    writeln('subsec=',SubSecTypeToStr(sub.sectype),' ',sub.sectype);
    cnt := ReadU(st);
    writeln('- symbol table [count=', cnt,']');
    for i:=0 to cnt-1 do begin
      write('  - ',i,' ');

      ofs := st.Position;
      ReadSymInfo(st, si);
      //write(SymKindToStr(si.kind),' ',IntToHex(si.flags,8));
      //if si.hasSymName then write(' ',si.symname);
      //writeln;

      if si.hasSymName then begin
        v := syms.Values[si.symname];
        fl := si.flags;

        tts := StrToSymTypes(v);
        for tt:=Low(TSymbolType) to High(TSymbolType) do begin
          if not (tt in tts) then continue;
          writeln(tt);
          case tt of
            st_Local:
              si.flags := (si.flags or WASM_SYM_BINDING_LOCAL) and (not WASM_SYM_BINDING_WEAK) and (not WASM_SYM_UNDEFINED);
            st_VisDefault:
              si.flags := (si.flags and not WASM_SYM_VISIBILITY_HIDDEN);
            st_VisHidden:
              si.flags := (si.flags or WASM_SYM_VISIBILITY_HIDDEN);
            st_Weak:
              si.flags := (si.flags or WASM_SYM_BINDING_WEAK) and (not WASM_SYM_BINDING_LOCAL)
          end;
        end;

        if fl <> si.flags then begin
          st.Position := ofs;
          WriteSymInfo(st, si);
        end;
      end;

      //writeln(si.symname);
    end;

    st.Position:=nx;
  //end;
end;

end.
