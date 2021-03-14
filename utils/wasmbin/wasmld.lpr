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

program wasmld;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  { you can add units after this }
  Classes, SysUtils, wasmbin, lebutils, wasmbindebug, wasmlink, wasmlinkchange;

function ReadStream(st: TStream): Boolean;
var
  dw  : LongWord;
  ofs : int64;
  sc  : TSection;
  ps  : int64;
  nm  : string;
begin
  dw := st.ReadDWord;
  Result := dw = WasmId_Int;
  if not Result then begin
    writeln('not a wasm file');
    Exit;
  end;
  dw := st.ReadDWord;
  writeln('version: ', dw);
  while st.Position<st.Size do begin
    ofs := st.Position;
    sc.id := st.ReadByte;
    sc.Size := ReadU(st);
    writeln(ofs,': id=', sc.id,'(', SectionIdToStr(sc.id),') sz=', sc.size);

    ps := st.Position+sc.size;
    if sc.id=0 then begin
      nm := GetName(st);
      writeln(nm);
      if nm = SectionName_Linking then
        DumpLinking(st, sc.size - (st.Position - ofs));
    end;
    //if sc.id= 1 then DumpTypes(st);

    if st.Position <> ps then
    begin
      //writeln('adjust stream targ=',ps,' actual: ', st.position);
      st.Position := ps;
    end;
  end;

end;

procedure ReadWasmFile(const fn: string);
var
  fs :TFileStream;
begin
  fs := TFileStream.Create(fn, fmOpenRead or fmShareDenyNone);
  try
    ReadStream(fs);
  finally
    fs.Free;
  end;
end;


function WriteStream(st: TStream; syms: TStrings): Boolean;
var
  dw  : LongWord;
  ofs : int64;
  sc  : TSection;
  ps  : int64;
  nm  : string;
begin
  writeln('read: ');
  dw := st.ReadDWord;
  writeln('dw: ' ,dw);
  Result := dw = WasmId_Int;
  if not Result then begin
    writeln('not a wasm file');
    Exit;
  end;
  dw := st.ReadDWord;
  writeln('version: ', dw);
  while st.Position<st.Size do begin
    ofs := st.Position;
    sc.id := st.ReadByte;
    sc.Size := ReadU(st);
    writeln(ofs,': id=', sc.id,'(', SectionIdToStr(sc.id),') sz=', sc.size);

    ps := st.Position+sc.size;
    if sc.id=0 then begin
      nm := GetName(st);
      writeln(nm);
      if nm = SectionName_Linking then begin
        writeln('rewriting linking...');
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

procedure ProcessWasmFile(const fn, symfn: string);
var
  fs :TFileStream;
  syms:  TStringList;
begin
  writeln('proc: ', fn);
  syms:=TStringList.Create;
  fs := TFileStream.Create(fn, fmOpenReadWrite or fmShareDenyNone);
  try
    if (symfn<>'') then
      ReadSymbolsConf(symfn, syms);
    writeln('size: ', fs.size);
    WriteStream(fs, syms);
  finally
    fs.Free;
    syms.Free;
  end;
end;

procedure RenameExport(var x: TExportSection; syms: TStrings);
var
  i : integer;
  v : string;
begin
  for i:=0 to length(x.entries)-1 do begin
    v := syms.Values[x.entries[i].name];
    if v <> '' then
      x.entries[i].name := v;
  end;
end;

procedure ParseCode(st: TStream);
var
  cnt : integer;
  cs  : TCodeSection;
  i : integer;
begin
  ReadCodeSection(st, cs);
  writeln('code=',length(cs.entries));
  for i:=0 to length(cs.entries)-1 do begin
    writelN('code[', i,'] ', isUnreachable(cs.entries[i]));
    writeln(' locals= ', length(cs.entries[i].locals));
    writeln(' code  = ', length(cs.entries[i].instBuf));
    writeln(' inst  = ', IntToHex(cs.entries[i].instBuf[0],2),' ',IntToHex(cs.entries[i].instBuf[1],2));
  end;
end;

procedure ParseTable(st: TStream);
begin
end;

procedure ParseElems(st: TStream);
var
  s : TElementSection;
  i : integer;
  j : integer;
begin
  ReadElementSection(st, s);
  writelN('entries = ', length(s.entries));
  for i:=0 to length(s.entries)-1 do begin
    writeln('  ',i,' functions = ', length(s.entries[i].funcs));
    for j:=0 to length(s.entries[i].funcs) - 1 do
      writeln('    ',s.entries[i].funcs[j]);
  end;
end;

function ProcessSections(st, dst: TStream; syms: TStrings): Boolean;
var
  dw  : LongWord;
  ofs : int64;
  sc  : TSection;
  ps  : int64;
  x   : TExportSection;
  i   : integer;
  mem : TMemoryStream;
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
    writeln(ofs,': id=', sc.id,'(', SectionIdToStr(sc.id),') sz=', sc.size);

    ps := st.Position+sc.size;

    if sc.id=SECT_ELEMENT then begin
      ParseElems(st);
    end else if sc.id=SECT_TABLE then begin
      ParseTable(st);
    end else if sc.id=SECT_CODE then begin
      ParseCode(st);
    (*end else if sc.id = SECT_EXPORT then begin
      ReadExport(st, x);
      RenameExport(x, syms);

      st.Position:=0;
      dst.CopyFrom(st, ofs);
      st.Position:=ps;

      mem := TMemoryStream.Create;
      WriteExport(x, mem);
      mem.Position:=0;

      dst.WriteByte(SECT_EXPORT);
      WriteU32(dst, mem.Size);
      dst.CopyFrom(mem, mem.Size);

      writeln('entries = ', length(x.entries));
      for i:=0 to length(x.entries)-1 do begin
        writeln(x.entries[i].desc,' ', x.entries[i].name)
      end;

      dst.CopyFrom(st, st.Size-st.Position);
      break; // done
      *)
    end;
    {if sc.id=0 then begin
      nm := GetName(st);
      writeln(nm);
      if nm = SectionName_Linking then begin
        writeln('rewriting linking...');
        ProcessLinkingSection(st, syms);
        break;
      end;
        //DumpLinking(st, sc.size - (st.Position - ofs));
    end;}
    //if sc.id= 1 then DumpTypes(st);

    if st.Position <> ps then
    begin
      //writeln('adjust stream targ=',ps,' actual: ', st.position);
      st.Position := ps;
    end;
  end;
end;


procedure ProcessWasmSection(const fn, {%H-}symfn: string);
var
  fs    : TFileStream;
  syms  : TStringList;
  dst   : TMemoryStream;
begin
  writeln('proc: ', fn);
  syms:=TStringList.Create;
  fs := TFileStream.Create(fn, fmOpenReadWrite or fmShareDenyNone);
  dst := TMemoryStream.Create;
  try
    if (symfn <> '') and fileExists(symfn) then
      syms.LoadFromFile(symfn);

    ProcessSections(fs, dst, syms);
    if dst.Size>0 then begin
      fs.Position:=0;
      dst.Position:=0;
      fs.CopyFrom(dst, dst.Size);
      fs.Size:=dst.Size;
    end;

  finally
    dst.Free;
    fs.Free;
    syms.Free;
  end;
end;

var
  symfn : string;
begin
  if ParamCount=0 then begin
    writeln('please specify .wasm file');
    exit;
  end;
  symfn:='';
  if ParamCount>1 then symfn:=ParamStr(2);

  //ReadWasmFile(ParamStr(1));
  //ProcessWasmFile(ParamStr(1), symfn);
  ProcessWasmSection(ParamStr(1), symfn);
end.

