{
    Copyright (c) 2004-2006 by Free Pascal Development Team

    This unit implements support import, export, link routines
    for the Amiga targets (AmigaOS/m68k, AmigaOS/PPC)

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
unit t_amiga;

{$i fpcdefs.inc}

interface

    uses
      rescmn, comprsrc, link;


type
  PLinkerAmiga = ^TLinkerAmiga;
  TLinkerAmiga = class(texternallinker)
    private
      UseVLink: boolean;
      function WriteResponseFile(isdll: boolean): boolean;
      procedure SetAmiga68kInfo;
      procedure SetAmigaPPCInfo;
      function MakeAmiga68kExe: boolean;
      function MakeAmigaPPCExe: boolean;
    public
      constructor Create; override;
      procedure SetDefaultInfo; override;
      procedure InitSysInitUnitName; override;
      function  MakeExecutable: boolean; override;
  end;


implementation

    uses
       SysUtils,
       cutils,cfileutl,cclasses,aasmbase,
       globtype,globals,systems,verbose,cscript,fmodule,i_amiga;



{****************************************************************************
                               TLinkerAmiga
****************************************************************************}

constructor TLinkerAmiga.Create;
begin
  UseVLink:=(cs_link_vlink in current_settings.globalswitches);

  Inherited Create;
  { allow duplicated libs (PM) }
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkerAmiga.SetAmiga68kInfo;
begin
  with Info do
   begin
    if not UseVLink then
     begin
      ExeCmd[1]:='ld $DYNLINK $OPT -d -n -o $EXE $RES';
     end
    else
     begin
      ExeCmd[1]:='vlink -b amigahunk $GCSECTIONS $OPT $STRIP -o $EXE -T $RES';
     end;
   end;
end;

procedure TLinkerAmiga.SetAmigaPPCInfo;
begin
  with Info do 
   begin
    if not UseVLink then
     begin
      ExeCmd[1]:='ld $DYNLINK $OPT -defsym=__amigaos4__=1 -d -q -N -o $EXE $RES';
     end
    else
     begin
      ExeCmd[1]:='vlink -q -n -b elf32amigaos -P_start -P__amigaos4__ -nostdlib $GCSECTIONS $OPT $STRIP -o $EXE -T $RES';
     end;
  end;
end;

procedure TLinkerAmiga.SetDefaultInfo;
begin
  case (target_info.system) of
    system_m68k_amiga:      SetAmiga68kInfo;
    system_powerpc_amiga:   SetAmigaPPCInfo;
    else
      internalerror(2019050949);
  end;
end;


Procedure TLinkerAmiga.InitSysInitUnitName;
begin
  sysinitunit:='si_prc';
end;


function TLinkerAmiga.WriteResponseFile(isdll: boolean): boolean;
var
  linkres  : TLinkRes;
  i        : longint;
  HPath    : TCmdStrListItem;
  s        : string;
  linklibc : boolean;
begin
  WriteResponseFile:=False;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if (cs_link_on_target in current_settings.globalswitches) then
     s:=ScriptFixFileName(s);
    LinkRes.Add('-L'+s);
    HPath:=TCmdStrListItem(HPath.Next);
   end;
  HPath:=TCmdStrListItem(LibrarySearchPath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if s<>'' then
     LinkRes.Add('SEARCH_DIR("'+Unix2AmigaPath(s)+'")');
    HPath:=TCmdStrListItem(HPath.Next);
   end;

  LinkRes.Add('INPUT (');
  { add objectfiles, start with prt0 always }
  if not (target_info.system in systems_internal_sysinit) then
    begin
      s:=FindObjectFile('prt0','',false);
      LinkRes.AddFileName(Unix2AmigaPath(maybequoted(s)));
    end;
  while not ObjectFiles.Empty do
   begin
    s:=ObjectFiles.GetFirst;
    if s<>'' then
     begin
      { vlink doesn't use SEARCH_DIR for object files }
      if UseVLink then
       s:=FindObjectFile(s,'',false);
      LinkRes.AddFileName(Unix2AmigaPath(maybequoted(s)));
     end;
   end;

  { Write staticlibraries }
  if not StaticLibFiles.Empty then
   begin
     { vlink doesn't need, and doesn't support GROUP }
    if not UseVLink then
     begin
      LinkRes.Add(')');
      LinkRes.Add('GROUP(');
     end;
    while not StaticLibFiles.Empty do
     begin
      S:=StaticLibFiles.GetFirst;
      LinkRes.AddFileName(Unix2AmigaPath(maybequoted(s)));
     end;
   end;

  if not UseVLink then
   begin
    LinkRes.Add(')');

    { Write sharedlibraries like -l<lib>, also add the needed dynamic linker
      here to be sure that it gets linked this is needed for glibc2 systems (PFV) }
    linklibc:=false;
    while not SharedLibFiles.Empty do
     begin
      S:=SharedLibFiles.GetFirst;
      if s<>'c' then
       begin
        i:=Pos(target_info.sharedlibext,S);
        if i>0 then
         Delete(S,i,255);
        LinkRes.Add('-l'+s);
       end
      else
       begin
        LinkRes.Add('-l'+s);
        linklibc:=true;
       end;
     end;
    { be sure that libc&libgcc is the last lib }
    if linklibc then
     begin
      LinkRes.Add('-lc');
      LinkRes.Add('-lgcc');
     end;
   end
  else
   begin
    while not SharedLibFiles.Empty do
     begin
      S:=SharedLibFiles.GetFirst;
      LinkRes.Add('lib'+s+target_info.staticlibext);
     end;
    LinkRes.Add(')');
   end;

  if (target_info.system = system_powerpc_amiga) and UseVLink then
   begin
    with linkres do
     begin
      { AmigaOS4-specific linker script from VBCC for VLink, with modifications,
        courtesy of Frank Wille, used with his permission }
      Add('SECTIONS');
      Add('{');
      Add('  . = 0x01000000 + SIZEOF_HEADERS;');
      Add('  /* Read-only sections, merged into text segment: */');
      Add('  .interp         : { *(.interp) }');
      Add('  .hash           : { *(.hash) }');
      Add('  .dynsym         : { *(.dynsym) }');
      Add('  .dynstr         : { *(.dynstr) }');
      Add('  .fpc            : { *(.fpc) }');
      Add('  .gnu.version    : { *(.gnu.version) }');
      Add('  .gnu.version_d  : { *(.gnu.version_d) }');
      Add('  .gnu.version_r  : { *(.gnu.version_r) }');
      Add('  .rela.dyn       : { *(.rela.dyn) }');
      Add('  .rela.plt       : { *(.rela.plt) }');
      Add('  .init           : { *(.init) }');
      Add('  .text           : { *(.text .text.* .gnu.linkonce.t.*) }');
      Add('  .fini           : { *(.fini) }');
      Add('  .code68k        : { *(CODE text code) }');
      Add('');
      Add('  .rodata         : { *(.rodata .rodata.* .gnu.linkonce.r.*) }');
      Add('  .sdata2         : { *(.sdata2 .sdata2.* .gnu.linkonce.s2.*) }');
      Add('  .sbss2          : { *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*) }');
      Add('');
      Add('  /* data segment: */');
      Add('  . = ALIGN(16) + 0x10000;');
      Add('');
      Add('  .dynamic        : { *(.dynamic) }');
      Add('  .data           : {');
      Add('    PROVIDE(_DATA_BASE_ = .);');
      Add('    *(.data .data.* .gnu.linkonce.d.*)');
      Add('    VBCC_CONSTRUCTORS_ELF');
      Add('  }');
      Add('  .ctors          : { *(.ctors .ctors.*) }');
      Add('  .dtors          : { *(.dtors .dtors.*) }');
      Add('  .data68k        : { *(DATA data) }');
      Add('  .got            : { *(.got.plt) *(.got) }');
      Add('  .sdata          : {');
      Add('    PROVIDE(_SDATA_BASE_ = .);');
      Add('    _LinkerDB = . + 0x8000;');
      Add('    _SDA_BASE_ = . + 0x8000;');
      Add('    *(.sdata .sdata.* .tocd .gnu.linkonce.s.*)');
      Add('  }');
      Add('  .sdata68k       : { *(__MERGED) }');
      Add('');
      Add('  /*');
      Add('  PROVIDE(_edata = .);');
      Add('  PROVIDE(edata = .);');
      Add('  PROVIDE(__bss_start = .);');
      Add('  */');
      Add('');
      Add('  .sbss           : {');
      Add('    PROVIDE(__sbss_start = .);');
      Add('    PROVIDE(___sbss_start = .);');
      Add('    *(.sbss .sbss.* .gnu.linkonce.sb.*)');
      Add('    *(.scommon)');
      Add('    PROVIDE(__sbss_end = .);');
      Add('    PROVIDE(___sbss_end = .);');
      Add('  }');
      Add('  .plt            : { *(.plt) }');
      Add('  .bss            : {');
      Add('    *(.bss .bss.* .gnu.linkonce.b.*)');
      Add('    *(COMMON)');
      Add('  }');
      Add('  .bss68k         : { *(BSS bss) }');
      Add('');
      Add('  . = ALIGN(16);');
      Add('  PROVIDE(_end = .);');
      Add('  PROVIDE(end = .);');
      Add('');
      Add('  .comment      0 : { *(.comment) }');
      Add('');
      { Do not provide the __amigaos4__ symbol for now. It's provided by our prt0.o,
        sadly various linkers for OS4 either provide it or not, which might or might
        not work with our prt0.o unmodified. }
      {Add('  __amigaos4__ = 1;');
      Add('');}
      Add('  /* DWARF debug sections.');
      Add('     Symbols in the DWARF debugging sections are relative to the beginning');
      Add('     of the section so we begin them at 0.  */');
      Add('  /* DWARF 1 */');
      Add('  .debug          0 : { *(.debug) }');
      Add('  .line           0 : { *(.line) }');
      Add('  /* GNU DWARF 1 extensions */');
      Add('  .debug_srcinfo  0 : { *(.debug_srcinfo) }');
      Add('  .debug_sfnames  0 : { *(.debug_sfnames) }');
      Add('  /* DWARF 1.1 and DWARF 2 */');
      Add('  .debug_aranges  0 : { *(.debug_aranges) }');
      Add('  .debug_pubnames 0 : { *(.debug_pubnames) }');
      Add('  /* DWARF 2 */');
      Add('  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }');
      Add('  .debug_abbrev   0 : { *(.debug_abbrev) }');
      Add('  .debug_line     0 : { *(.debug_line) }');
      Add('  .debug_frame    0 : { *(.debug_frame) }');
      Add('  .debug_str      0 : { *(.debug_str) }');
      Add('  .debug_loc      0 : { *(.debug_loc) }');
      Add('  .debug_macinfo  0 : { *(.debug_macinfo) }');
      Add('  /* DWARF 2.1 */');
      Add('  .debug_ranges   0 : { *(.debug_ranges) }');
      Add('}');
     end;
   end;

{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;
end;


function TLinkerAmiga.MakeAmiga68kExe: boolean;
var
  BinStr,
  CmdStr  : TCmdStr;
  StripStr: string[40];
  DynLinkStr : string;
  GCSectionsStr : string;
begin
  StripStr:='';
  GCSectionsStr:='';
  DynLinkStr:='';

  if (cs_link_strip in current_settings.globalswitches) then
    StripStr:='-s';
  if rlinkpath<>'' Then
    DynLinkStr:='--rpath-link '+rlinkpath;
  if UseVLink then
    begin
      if create_smartlink_sections then
        GCSectionsStr:='-gc-all -mtype';
    end;

  { Call linker }
  SplitBinCmd(Info.ExeCmd[1],BinStr,CmdStr);
  binstr:=FindUtil(utilsprefix+BinStr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$EXE',Unix2AmigaPath(maybequoted(ScriptFixFileName(current_module.exefilename))));
  Replace(cmdstr,'$RES',Unix2AmigaPath(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);

  MakeAmiga68kExe:=DoExec(BinStr,CmdStr,true,false);
end;


function TLinkerAmiga.MakeAmigaPPCExe: boolean;
var
  BinStr,
  CmdStr  : TCmdStr;
  StripStr: string[40];
  DynLinkStr : string;
  GCSectionsStr : string;
begin
  StripStr:='';
  GCSectionsStr:='';
  DynLinkStr:='';

  if (cs_link_strip in current_settings.globalswitches) then
    StripStr:='-s';
  if rlinkpath<>'' Then
    DynLinkStr:='--rpath-link '+rlinkpath;
  if UseVLink then
    begin
      if create_smartlink_sections then
        GCSectionsStr:='-gc-all -sc -sd';
    end;

  { Call linker }
  SplitBinCmd(Info.ExeCmd[1],BinStr,CmdStr);
  binstr:=FindUtil(utilsprefix+BinStr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$EXE',Unix2AmigaPath(maybequoted(ScriptFixFileName(current_module.exefilename))));
  Replace(cmdstr,'$RES',Unix2AmigaPath(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);

  MakeAmigaPPCExe:=DoExec(BinStr,CmdStr,true,false);
end;


function TLinkerAmiga.MakeExecutable:boolean;
var
  success : boolean;
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
    Message1(exec_i_linking,current_module.exefilename);

  { Write used files and libraries }
  WriteResponseFile(false);

  success:=false;
  case (target_info.system) of
    system_m68k_amiga:      success:=MakeAmiga68kExe;
    system_powerpc_amiga:   success:=MakeAmigaPPCExe;
    else
      internalerror(2019050948);
  end;

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    DeleteFile(outputexedir+Info.ResName);

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
{$ifdef m68k}
  RegisterLinker(ld_amiga,TLinkerAmiga);
  RegisterTarget(system_m68k_Amiga_info);
  RegisterRes(res_ext_info, TWinLikeResourceFile);
{$endif m68k}
{$ifdef powerpc}
  RegisterLinker(ld_amiga,TLinkerAmiga);
  RegisterTarget(system_powerpc_Amiga_info);
{$endif powerpc}
end.
