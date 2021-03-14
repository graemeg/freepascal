{
    Copyright (c) 2019 by Dmitry Boyarintsev

    This unit implements support import,export,link routines
    for the WASI target

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}

unit t_wasi;

{$i fpcdefs.inc}

interface

uses
  systems,

  globtype, globals,
  aasmbase,
  cfileutl, cutils, cclasses,

  import, export, aasmdata, aasmcpu,
  fmodule, ogbase,

  symsym, symdef,

  link,

  i_wasi, tgcpu;

type

  { texportlibwasi }

  texportlibwasi=class(texportlib)
      procedure preparelib(const s : string);override;
      procedure exportprocedure(hp : texported_item);override;
      procedure exportvar(hp : texported_item);override;
      procedure generatelib;override;
    end;

  { timportlibwasi }
  timportlibwasi = class(timportlib)
      procedure generatelib;override;
    end;

  { tlinkerwasi }

  tlinkerwasi=class(texternallinker)
  public
    constructor Create;override;
    procedure SetDefaultInfo;override;

    procedure InitSysInitUnitName;override;

    function  MakeExecutable:boolean;override;
    function  MakeSharedLibrary:boolean;override;
  end;


implementation

uses
  SysUtils,
  verbose;

{ timportlibwasi }

  procedure timportlibwasi.generatelib;
    begin
    end;

{ tlinkerwasi }

constructor tlinkerwasi.Create;
begin
  inherited Create;
end;

procedure tlinkerwasi.SetDefaultInfo;
begin
  Info.DllCmd[1] := 'wasm-ld $SONAME $GCSECTIONS $MAP -o $EXE';
  //Info.DllCmd[2] := 'wasmtool --exportrename $INPUT $EXE';
end;

procedure tlinkerwasi.InitSysInitUnitName;
begin
  sysinitunit:='si_prc';
end;

function tlinkerwasi.MakeExecutable:boolean;
var
  GCSectionsStr  : ansistring;
  binstr, cmdstr : Tcmdstr;
  InitStr,
  FiniStr,
  SoNameStr      : string[80];
  mapstr,ltostr  : TCmdStr;
  success        : Boolean;

  tmp : TCmdStrListItem;
  tempFileName : ansistring;
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
    Message1(exec_i_linking,current_module.exefilename);

  { Create some replacements }
  mapstr:='';
  if (cs_link_map in current_settings.globalswitches) then
    mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
  if (cs_link_smart in current_settings.globalswitches) and
     create_smartlink_sections then
   GCSectionsStr:='--gc-sections'
  else
    GCSectionsStr:='';

  SoNameStr:='';
  SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));

  tmp := TCmdStrListItem(ObjectFiles.First);
  while Assigned(tmp) do begin
    cmdstr := tmp.Str+ ' ' + cmdstr;
    tmp := TCmdStrListItem(tmp.Next);
  end;

//  if HasExports then
//    cmdstr := cmdstr + ' --export-dynamic'; //' --export-dynamic';

  cmdstr := cmdstr + ' --no-entry';

  if (cs_link_strip in current_settings.globalswitches) then
   begin
     { only remove non global symbols and debugging info for a library }
     cmdstr := cmdstr + ' --strip-all';
   end;

  //Replace(cmdstr,'$OPT',Info.ExtraOptions);
  //Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  //Replace(cmdstr,'$INIT',InitStr);
  //Replace(cmdstr,'$FINI',FiniStr);
  Replace(cmdstr,'$SONAME',SoNameStr);
  Replace(cmdstr,'$MAP',mapstr);
  //Replace(cmdstr,'$LTO',ltostr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,false);

  //SplitBinCmd(Info.DllCmd[2],binstr,cmdstr);
  //Replace(cmdstr,'$INPUT',current_module.objfilename );
  //Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  //DoExec(FindUtil(utilsprefix+binstr),cmdstr,false,false);

  MakeExecutable:=success;
end;

function tlinkerwasi.MakeSharedLibrary: boolean;
var
  GCSectionsStr  : ansistring;
  binstr, cmdstr : Tcmdstr;
  InitStr,
  FiniStr,
  SoNameStr      : string[80];
  mapstr,ltostr  : TCmdStr;
  success        : Boolean;

  tmp : TCmdStrListItem;
  tempFileName : ansistring;
begin
  //Result := true;
  //Result:=inherited MakeSharedLibrary;
  if (cs_link_smart in current_settings.globalswitches) and
     create_smartlink_sections then
   GCSectionsStr:='--gc-sections'
  else
    GCSectionsStr:='';

  SoNameStr:='';
  SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));

  tmp := TCmdStrListItem(ObjectFiles.First);
  while Assigned(tmp) do begin
    cmdstr := tmp.Str+ ' ' + cmdstr;
    tmp := TCmdStrListItem(tmp.Next);
  end;

  if HasExports then
    cmdstr := cmdstr + ' --export-dynamic'; //' --export-dynamic';

  cmdstr := cmdstr + ' --no-entry --allow-undefined';

  if (cs_link_strip in current_settings.globalswitches) then
   begin
     { only remove non global symbols and debugging info for a library }
     cmdstr := cmdstr + ' --strip-all';
   end;

  //Replace(cmdstr,'$OPT',Info.ExtraOptions);
  //Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  //Replace(cmdstr,'$INIT',InitStr);
  //Replace(cmdstr,'$FINI',FiniStr);
  Replace(cmdstr,'$SONAME',SoNameStr);
  //Replace(cmdstr,'$MAP',mapstr);
  //Replace(cmdstr,'$LTO',ltostr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  writeln(utilsprefix+binstr,' ',cmdstr);
  success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,false);

  //SplitBinCmd(Info.DllCmd[2],binstr,cmdstr);
  //Replace(cmdstr,'$INPUT',current_module.objfilename );
  //Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  //DoExec(FindUtil(utilsprefix+binstr),cmdstr,false,false);

  MakeSharedLibrary:=success;
end;

{ texportlibwasi }

procedure texportlibwasi.preparelib(const s: string);
begin
  //nothing to happen. wasm files are modules
end;

procedure texportlibwasi.exportprocedure(hp: texported_item);
var
  nm : TSymStr;
begin
  nm := tprocdef(tprocsym(hp.sym).ProcdefList[0]).mangledname;
  current_asmdata.asmlists[al_exports].Concat(tai_impexp.create(hp.name^, nm, ie_Func));
end;

procedure texportlibwasi.exportvar(hp: texported_item);
begin
  //inherited exportvar(hp);
end;

procedure texportlibwasi.generatelib;
begin
  //inherited generatelib;
end;

initialization
  RegisterTarget(system_wasm32_wasi_info);
  RegisterImport(system_wasm32_wasi, timportlibwasi);
  RegisterExport(system_wasm32_wasi, texportlibwasi);
  RegisterLinker(ld_wasi, tlinkerwasi);

end.
