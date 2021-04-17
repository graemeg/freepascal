{
    Copyright (c) 2004 by Free Pascal Development Team

    This unit implements support import, export, link routines
    for the MorphOS (PowerPC) target

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
unit t_morph;

{$i fpcdefs.inc}

interface


implementation

    uses
       SysUtils,
       cutils,cfileutl,cclasses,rescmn,comprsrc,aasmbase,
       globtype,globals,systems,verbose,cscript,fmodule,i_morph,link;

    type
       PlinkerMorphOS=^TlinkerMorphOS;
       TlinkerMorphOS=class(texternallinker)
       private
          UseVLink: Boolean;
          Function  WriteResponseFile(isdll:boolean) : Boolean;
       public
          constructor Create; override;
          procedure SetDefaultInfo; override;
          procedure InitSysInitUnitName; override;
          function  MakeExecutable:boolean; override;
       end;


{****************************************************************************
                               TLinkerMorphOS
****************************************************************************}

Constructor TLinkerMorphOS.Create;
begin
  UseVLink:=(cs_link_vlink in current_settings.globalswitches);

  Inherited Create;
  { allow duplicated libs (PM) }
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkerMorphOS.SetDefaultInfo;
begin
  with Info do
   begin
    if not UseVLink then
     begin
      ExeCmd[1]:='ld $OPT $GCSECTIONS -o $EXE $RES';
      ExeCmd[2]:='strip --strip-unneeded --remove-section .comment $EXE';
     end
    else
     begin
      ExeCmd[1]:='vlink -b elf32amiga $OPT $STRIP $GCSECTIONS -o $EXE -T $RES';
     end;
   end;
end;


Procedure TLinkerMorphOS.InitSysInitUnitName;
begin
  sysinitunit:='si_prc';
end;


Function TLinkerMorphOS.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres  : TLinkRes;
  i        : longint;
  HPath    : TCmdStrListItem;
  s        : string;
  linklibc : boolean;
begin
  WriteResponseFile:=False;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);
  if UseVLink and (source_info.dirsep <> '/') then
    LinkRes.fForceUseForwardSlash:=true;

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if not (cs_link_on_target in current_settings.globalswitches) then
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


{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;

end;


function TLinkerMorphOS.MakeExecutable:boolean;
var
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  GCSectionsStr: string;
  StripStr: string[40];
begin
  StripStr:='';
  GCSectionsStr:='';

  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

  if UseVLink then
   begin
    if (cs_link_strip in current_settings.globalswitches) then
     StripStr:='-s -P __abox__';
    if create_smartlink_sections then
     GCSectionsStr:='-gc-all -sc -sd';
   end
  else
   begin
    if create_smartlink_sections then
     GCSectionsStr:='--gc-sections -e _start';
   end;

{ Write used files and libraries }
  WriteResponseFile(false);

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  if UseVLink then
   begin
    Replace(cmdstr,'$EXE',Unix2AmigaPath(maybequoted(ScriptFixFileName(current_module.exefilename))));
    Replace(cmdstr,'$RES',Unix2AmigaPath(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
    Replace(cmdstr,'$STRIP',StripStr);
    Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
   end
  else
   begin
    Replace(cmdstr,'$EXE',maybequoted(ScriptFixFileName(current_module.exefilename)));
    Replace(cmdstr,'$RES',maybequoted(ScriptFixFileName(outputexedir+Info.ResName)));
    Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
   end;
  success:=DoExec(FindUtil(utilsprefix+BinStr),cmdstr,true,false);

{ Stripping Enabled? }
  { For MorphOS ld a separate strip command is needed, to avoid stripping }
  { __abox__ symbol, which is required to be present in current MorphOS }
  { executables. }
  if not UseVLink then
   begin
    if success and (cs_link_strip in current_settings.globalswitches) then
     begin
      SplitBinCmd(Info.ExeCmd[2],binstr,cmdstr);
      Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
      success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,false);
     end;
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
  RegisterLinker(ld_morphos,TLinkerMorphOS);
  RegisterTarget(system_powerpc_morphos_info);
  RegisterRes(res_elf_info, TWinLikeResourceFile);
end.
