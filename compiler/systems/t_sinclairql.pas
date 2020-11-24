{
    Copyright (c) 2020 by Free Pascal Development Team

    This unit implements support import, export, link routines
    for the m68k Sinclair QL target

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
unit t_sinclairql;

{$i fpcdefs.inc}

interface

    uses
      rescmn, comprsrc, link;

type
  PLinkerSinclairQL = ^TLinkerSinclairQL;
  TLinkerSinclairQL = class(texternallinker)
    private
      Origin: DWord;
      UseVLink: boolean;
      ExeLength: longint;
      function WriteResponseFile(isdll: boolean): boolean;
      procedure SetSinclairQLInfo;
      function MakeSinclairQLExe: boolean;
    public
      constructor Create; override;
      procedure SetDefaultInfo; override;
      procedure InitSysInitUnitName; override;
      function  MakeExecutable: boolean; override;
  end;


implementation

    uses
       sysutils,cutils,cfileutl,cclasses,aasmbase,
       globtype,globals,systems,verbose,cscript,fmodule,i_sinclairql;


    const
       DefaultOrigin = $0;
       ProgramHeaderName = 'main';


constructor TLinkerSinclairQL.Create;
begin
  UseVLink:=(cs_link_vlink in current_settings.globalswitches);

  Inherited Create;
  { allow duplicated libs (PM) }
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkerSinclairQL.SetSinclairQLInfo;
begin
  if ImageBaseSetExplicity then
    Origin:=ImageBase
  else
    Origin:=DefaultOrigin;

  with Info do
   begin
    if not UseVLink then
     begin
      ExeCmd[1]:='ld $DYNLINK $OPT -d -n -o $EXE $RES';
     end
    else
     begin
      ExeCmd[1]:='vlink -b rawseg -q $FLAGS $GCSECTIONS $OPT $STRIP $MAP -o $EXE -T $RES';
     end;
   end;
end;


procedure TLinkerSinclairQL.SetDefaultInfo;
begin
  if target_info.system = system_m68k_sinclairql then
    SetSinclairQLInfo;
end;


procedure TLinkerSinclairQL.InitSysInitUnitName;
begin
  sysinitunit:='si_prc';
end;


function TLinkerSinclairQL.WriteResponseFile(isdll: boolean): boolean;
var
  linkres  : TLinkRes;
  HPath    : TCmdStrListItem;
  s        : string;
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
        LinkRes.Add('SEARCH_DIR("'+s+'")');
      HPath:=TCmdStrListItem(HPath.Next);
    end;

  LinkRes.Add('INPUT (');
  { add objectfiles, start with prt0 always }
  if not (target_info.system in systems_internal_sysinit) then
    begin
      s:=FindObjectFile('prt0','',false);
      LinkRes.AddFileName(maybequoted(s));
    end;
  while not ObjectFiles.Empty do
    begin
      s:=ObjectFiles.GetFirst;
      if s<>'' then
        begin
          { vlink doesn't use SEARCH_DIR for object files }
          if UseVLink then
             s:=FindObjectFile(s,'',false);
          LinkRes.AddFileName(maybequoted(s));
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
          LinkRes.AddFileName(maybequoted(s));
        end;
    end;

  LinkRes.Add(')');

  with LinkRes do
    begin
      Add('');
      Add('PHDRS {');
      Add('  '+ProgramHeaderName+' PT_LOAD;');
      Add('}');
      Add('SECTIONS');
      Add('{');
      Add('  . = 0x'+hexstr(Origin,8)+';');
      Add('  .text : {');
      Add('      _stext = .;');
      Add('      *(.text .text.* )');
      Add('      *(.data .data.* .rodata .rodata.* .fpc.* )');
      Add('      *(.stack .stack.*)');
      { force the end of section to be word aligned }
      Add('      . = ALIGN(2); SHORT(0x514C);');
      Add('      _etext = .;');
      Add('  } :'+ProgramHeaderName);
      Add('  .bss (NOLOAD): {');
      Add('      _sbss = .;');
      Add('      *(.bss .bss.*)');
      Add('      _ebss = .;');
      Add('  } :'+ProgramHeaderName);
      Add('}');
    end;

{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;
end;


function TLinkerSinclairQL.MakeSinclairQLExe: boolean;
var
  BinStr,
  CmdStr  : TCmdStr;
  StripStr: string[40];
  DynLinkStr : string;
  GCSectionsStr : string;
  FlagsStr : string;
  MapStr : string;
  ExeName: string;
  fd,fs: file;
  fhdr: text;
  buf: pointer;
  bufread,bufsize: longint;
  HdrName: string;
  HeaderLine: string;
  HeaderSize: longint;
  code: word;
begin
  StripStr:='';
  GCSectionsStr:='';
  DynLinkStr:='';
  FlagsStr:='';
  MapStr:='';

  if (cs_link_map in current_settings.globalswitches) then
    MapStr:='-M'+maybequoted(ScriptFixFilename(current_module.mapfilename));
  if (cs_link_strip in current_settings.globalswitches) then
    StripStr:='-s';
  if rlinkpath<>'' then
    DynLinkStr:='--rpath-link '+rlinkpath;
  if UseVLink then
    begin
      if create_smartlink_sections then
        GCSectionsStr:='-gc-all';
    end;

  ExeName:=current_module.exefilename;
  HdrName:=ExeName+'.hdr';

  { Call linker }
  SplitBinCmd(Info.ExeCmd[1],BinStr,CmdStr);
  binstr:=FindUtil(utilsprefix+BinStr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$EXE',maybequoted(ScriptFixFileName(ExeName)));
  Replace(cmdstr,'$RES',maybequoted(ScriptFixFileName(outputexedir+Info.ResName)));
  Replace(cmdstr,'$MAP',MapStr);
  Replace(cmdstr,'$FLAGS',FlagsStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);

  MakeSinclairQLExe:=DoExec(BinStr,CmdStr,true,false);

  { Kludge:
      With the above linker script, vlink will produce two files. The main binary 
      and the relocation info. Here we copy the two together. (KB) }
  if MakeSinclairQLExe then
    begin
      ExeLength:=0;
      bufsize:=16384;
{$push}
{$i-}
      { Rename vlink's output file into the header file it is, then parse the 
        expected length from it. Later we use either this size or the final binary
        size in the BASIC loader, depending on which one is bigger. (KB) }
      RenameFile(ExeName,HdrName);
      assign(fhdr,HdrName);
      reset(fhdr);
      readln(fhdr,HeaderLine);
      Val(Copy(HeaderLine,RPos('0x',HeaderLine),Length(HeaderLine)),HeaderSize,code);
      close(fhdr);

      buf:=GetMem(bufsize);
      assign(fd,ExeName);
      rewrite(fd,1);

      assign(fs,ExeName+'.'+ProgramHeaderName);
      reset(fs,1);
      repeat
        blockread(fs,buf^,bufsize,bufread);
        blockwrite(fd,buf^,bufread);
      until eof(fs);
      close(fs);
      // erase(fs);

      assign(fs,ExeName+'.'+ProgramHeaderName+'.rel'+ProgramHeaderName);
      reset(fs,1);
      repeat
        blockread(fs,buf^,bufsize,bufread);
        blockwrite(fd,buf^,bufread);
      until eof(fs);
      close(fs);
      // erase(fs);

      ExeLength:=FileSize(fd);
      close(fd);
{$pop}
      FreeMem(buf);
      if HeaderSize > ExeLength then
        ExeLength:=HeaderSize;
      MakeSinclairQLExe:=(code = 0) and not (ExeLength = 0);
    end;
end;


function TLinkerSinclairQL.MakeExecutable:boolean;
const
  DefaultBootString = '10 $SYM=RESPR($BINSIZE):LBYTES"win1_$EXENAME",$SYM:CALL $SYM';
var
  success : boolean;
  bootfile : TScript;
  ExeName: String;
  BootStr: String;
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
    Message1(exec_i_linking,current_module.exefilename);

  { Write used files and libraries }
  WriteResponseFile(false);

  success:=MakeSinclairQLExe;

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    DeleteFile(outputexedir+Info.ResName);

  if (success) then
    begin
      ExeName:=current_module.exefilename;
      BootStr:=DefaultBootString;

      Replace(BootStr,'$BINSIZE',tostr(ExeLength));
      Replace(BootStr,'$EXENAME',ExeName);

      Replace(ExeName,target_info.exeext,'');
      Replace(BootStr,'$SYM',ExeName);

      { Write bootfile }
      bootfile:=TScript.Create(outputexedir+ExeName);
      bootfile.Add(BootStr);
      bootfile.writetodisk;
      bootfile.Free;
    end;

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;




{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_sinclairql,TLinkerSinclairQL);
  RegisterTarget(system_m68k_sinclairql_info);
end.
