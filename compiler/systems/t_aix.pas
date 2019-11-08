{
    Copyright (c) 2011 by Jonas Maebe

    This unit implements support import,export,link routines
    for the AIX target

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
unit t_aix;

{$i fpcdefs.inc}

interface

  uses
    aasmdata,
    symsym,symdef,ppu,
    import,export,expunix,link;

  type
    timportlibaix=class(timportlib)
      procedure generatelib;override;
    end;

    texportlibaix=class(texportlibunix)
      procedure setfininame(list: TAsmList; const s: string); override;
    end;

    TLinkerAIX=class(texternallinker)
    private
      prtobj  : string[80];
      assumebinutils,use_gld : boolean;
      Function  WriteResponseFile(isdll:boolean) : Boolean;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      function  MakeExecutable:boolean;override;
      function  MakeSharedLibrary:boolean;override;
    end;

implementation

  uses
    SysUtils,
    cutils,cfileutl,cclasses,
    verbose,systems,globtype,globals,
    symconst,cscript,
    fmodule,
    aasmbase,aasmtai,aasmcpu,cpubase,
    cgbase,cgobj,cgutils,ogbase,ncgutil,
    comprsrc,
    rescmn, i_aix
    ;

{*****************************************************************************
                               timportlibaix
*****************************************************************************}

    procedure timportlibaix.generatelib;
      var
        i : longint;
        ImportLibrary : TImportLibrary;
      begin
        for i:=0 to current_module.ImportLibraryList.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(current_module.ImportLibraryList[i]);
            current_module.linkothersharedlibs.add(ImportLibrary.Name,link_always);
          end;
      end;


{*****************************************************************************
                               texportlibaix
*****************************************************************************}

    procedure texportlibaix.setfininame(list: TAsmList; const s: string);
      begin
        inherited setfininame(list,s);
      end;

{*****************************************************************************
                                  TLinkerAIX
*****************************************************************************}

Constructor TLinkerAIX.Create;
begin
  Inherited Create;
  if not Dontlinkstdlibpath then
    if not(cs_profile in current_settings.moduleswitches) then
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib;=/usr/X11R6/lib;=/opt/freeware/lib',true)
    else
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/profiled;=/usr/X11R6/lib;=/opt/freeware/lib',true)
end;


procedure TLinkerAIX.SetDefaultInfo;
begin
  with Info do
   begin
     { the -bpT and -bpD options set the base addresses for text and data
       sections. They are required because otherwise they are both 0 and hence
       overlap, which confuses gdb. GCC uses those same options. -btextro makes
       sure that the binary does not contain any relocations in the text
       section (otherwise you get an error at load time instead of at link time
       in case something is wrong) }
     ExeCmd[1]:='$LDBIN -bpT:0x10000000 -bpD:0x20000000 -btextro $OPT $STRIP -L. -o $EXE $CATRES' {$ifdef powerpc64}+' -b64'{$endif};
     DllCmd[1]:='$LDBIN -bpT:0x10000000 -bpD:0x20000000 -btextro $OPT $INITFINI $STRIP -G -L. -o $EXE $CATRES' {$ifdef powerpc64}+' -b64'{$endif};
     if cs_debuginfo in current_settings.moduleswitches then
       begin
         { debugging helpers }
         ExeCmd[1]:=ExeCmd[1]+' -lg -bexport:/usr/lib/libg.exp';
         DllCmd[1]:=DllCmd[1]+' -lg -bexport:/usr/lib/libg.exp';
       end;
   end;
{$if defined(powerpc)}
   if not(cs_profile in current_settings.moduleswitches) then
     prtobj:=sysrootpath+'/lib/crt0.o'
   else
     prtobj:=sysrootpath+'/lib/gcrt0.o';
{$elseif defined(powerpc64)}
  if not(cs_profile in current_settings.moduleswitches) then
    prtobj:=sysrootpath+'/lib/crt0_64.o'
  else
    prtobj:=sysrootpath+'/lib/gcrt0_64.o';
{$else}
{$error unsupported AIX architecture}
{$endif}
  assumebinutils:=
    not(cs_link_native in current_settings.globalswitches) or
    (not(cs_link_on_target in current_settings.globalswitches) and
     not(source_info.system in systems_aix)) ;
  use_gld:=assumebinutils and (source_info.system in systems_aix)
end;


Function TLinkerAIX.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres      : TLinkRes;
  i            : longint;
  HPath        : TCmdStrListItem;
  s,s1         : TCmdStr;
begin
  result:=False;
  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,assumebinutils);
  with linkres do
    begin
      { Write path to search libraries }
      HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
      while assigned(HPath) do
       begin
         if assumebinutils then
           Add('SEARCH_DIR("'+HPath.Str+'")')
         else
           Add('-L'+HPath.Str);
         HPath:=TCmdStrListItem(HPath.Next);
       end;
      HPath:=TCmdStrListItem(LibrarySearchPath.First);
      while assigned(HPath) do
       begin
         if assumebinutils then
           Add('SEARCH_DIR("'+HPath.Str+'")')
         else
           Add('-L'+HPath.Str);
         HPath:=TCmdStrListItem(HPath.Next);
       end;

       if assumebinutils then
        StartSection('INPUT(');
      { add objectfiles, start with prt0 always }
       if assumebinutils then
         AddFileName(maybequoted(FindObjectFile(prtobj,'',false)))
       else
         AddFileName(FindObjectFile(prtobj,'',false));
      { main objectfiles }
      while not ObjectFiles.Empty do
       begin
         s:=ObjectFiles.GetFirst;
         if s<>'' then
          if assumebinutils then
            AddFileName(maybequoted(s))
          else
            AddFileName(s)
       end;

      { Write staticlibraries }
      if not StaticLibFiles.Empty then
       begin
         While not StaticLibFiles.Empty do
          begin
            S:=StaticLibFiles.GetFirst;
            if assumebinutils then
              AddFileName(maybequoted(s))
            else
              AddFileName(s);
          end;
       end;

      { Write sharedlibraries like -l<lib> }
      While not SharedLibFiles.Empty do
        begin
          S:=SharedLibFiles.GetFirst;
          i:=Pos(target_info.sharedlibext,S);
          if i>0 then
            Delete(S,i,255);
          Add('-l'+s);
        end;
       { when we have -static for the linker the we also need libgcc }
       if (cs_link_staticflag in current_settings.globalswitches) then
         begin
           Add('-lgcc');
           if librarysearchpath.FindFile('libgcc_eh.a',false,s1) then
             Add('-lgcc_eh');
         end;
       if assumebinutils then
         EndSection(')');

      { Write and Close response }
      writetodisk;
      Free;
    end;

  WriteResponseFile:=True;
end;


function TLinkerAIX.MakeExecutable:boolean;
var
  linkscript: TAsmScript;
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  StripStr   : string[40];
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

  linkscript:=nil;
{ Create some replacements }
  StripStr:='';
  if (cs_link_strip in current_settings.globalswitches) and
     not(cs_link_separate_dbg_file in current_settings.globalswitches) then
   StripStr:='-s';
  if (cs_link_map in current_settings.globalswitches) then
   StripStr:='-bmap:'+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
{ Write used files and libraries }
  WriteResponseFile(false);

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  if assumebinutils and (source_info.system in systems_aix) then
    Replace(binstr,'$LDBIN','gld')
  else
    Replace(binstr,'$LDBIN','ld');
  binstr:=FindUtil(utilsprefix+BinStr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  { the native AIX linker does not support linkres files, so we need
    CatFileContent(). The binutils cross-linker does support such files, so
    use them when cross-compiling to avoid overflowing the Windows maximum
    command line length
  }
  if not(cs_link_on_target in current_settings.globalswitches) and
     not(source_info.system in systems_aix) then
    Replace(cmdstr,'$CATRES',outputexedir+Info.ResName)
  else
    Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  Replace(cmdstr,'$STRIP',StripStr);

  { create dynamic symbol table? }
  if HasExports then
    cmdstr:=cmdstr+' -E';

  { custom sysroot? (for cross-compiling -> assume gnu ld) }
  if sysrootpath<>'' then
    cmdstr:=cmdstr+' --sysroot='+sysrootpath;

  if not(cs_link_nolink in current_settings.globalswitches) and
     not(tf_no_backquote_support in source_info.flags) then
    begin
      { we have to use a script to use the IFS hack }
      linkscript:=GenerateScript(outputexedir+'ppaslink');
      linkscript.AddLinkCommand(binstr,CmdStr,'');
//      if (extdbgbinstr<>'') then
//        linkscript.AddLinkCommand(extdbgbinstr,extdbgcmdstr,'');
      linkscript.WriteToDisk;
      BinStr:=linkscript.fn;
      if not path_absolute(BinStr) then
        if cs_link_on_target in current_settings.globalswitches then
          BinStr:='.'+target_info.dirsep+BinStr
        else
          BinStr:='.'+source_info.dirsep+BinStr;
      CmdStr:='';
    end;

  success:=DoExec(BinStr,cmdstr,true,true);

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    begin
      DeleteFile(outputexedir+Info.ResName);
      if assigned(linkscript) then
        DeleteFile(linkscript.fn);
    end;
  linkscript.free;

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


Function TLinkerAIX.MakeSharedLibrary:boolean;
const
{$ifdef cpu64bitaddr}
  libobj = 'shr_64.o';
{$else}
  libobj = 'shr.o';
{$endif}
var
  linkscript  : TAsmScript;
  exportedsyms: text;
  InitFiniStr : string[80];
  StripStr,
  binstr,
  cmdstr,
  libfn: TCmdStr;
  success : boolean;
begin
  MakeSharedLibrary:=false;
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.sharedlibfilename);

{ Write used files and libraries }
  WriteResponseFile(true);

 { Create some replacements }
  InitFiniStr:='-binitfini:'+exportlib.initname+':'+exportlib.fininame;
  Replace(InitFiniStr,'$','.');
  if cs_link_strip in current_settings.globalswitches then
    StripStr:='-s'
  else
    StripStr:='';

{ Call linker }
  SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
  if assumebinutils and (source_info.system in systems_aix) then
    Replace(binstr,'$LDBIN','gld')
  else
    Replace(binstr,'$LDBIN','ld');
  binstr:=FindUtil(utilsprefix+BinStr);
  { on AIX, shared libraries are special object files that are stored inside
    an archive. In that archive, the 32 bit version of the library is called
    shr.o and the 64 bit version shr_64.o }
  Replace(cmdstr,'$EXE',maybequoted(libobj));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  if not(cs_link_on_target in current_settings.globalswitches) and
     not(source_info.system in systems_aix) then
    Replace(cmdstr,'$CATRES',outputexedir+Info.ResName)
  else
    Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  Replace(cmdstr,'$INITFINI',InitFiniStr);
  Replace(cmdstr,'$STRIP',StripStr);
  { exported symbols }
  if not texportlibunix(exportlib).exportedsymnames.empty then
    begin
      assign(exportedsyms,outputexedir+'linksyms.fpc');
      rewrite(exportedsyms);
      repeat
        writeln(exportedsyms,texportlibunix(exportlib).exportedsymnames.getfirst);
      until texportlibunix(exportlib).exportedsymnames.empty;
      close(exportedsyms);
      cmdstr:=cmdstr+' -bE:'+maybequoted(outputexedir)+'linksyms.fpc';
    end;

  libfn:=maybequoted(current_module.sharedlibfilename);
  { we have to use a script to use the IFS hack }
  linkscript:=GenerateScript(outputexedir+'ppaslink');
  linkscript.AddLinkCommand(binstr,CmdStr,'');
  { delete the target static library containing the dynamic object file in
    case it already existed }
  if FileExists(libfn,true) then
    linkscript.AddDeleteCommand(libfn);
  { and create the new one }
  linkscript.AddLinkCommand(FindUtil(utilsprefix+'ar'),' -X32_64 -q '+libfn+(' '+libobj),'');
  linkscript.WriteToDisk;
  BinStr:=linkscript.fn;
  if not path_absolute(BinStr) then
    if cs_link_on_target in current_settings.globalswitches then
      BinStr:='.'+target_info.dirsep+BinStr
    else
      BinStr:='.'+source_info.dirsep+BinStr;
  CmdStr:='';

  success:=DoExec(BinStr,cmdstr,true,true);

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    begin
      DeleteFile(libobj);
      DeleteFile(outputexedir+Info.ResName);
      DeleteFile(outputexedir+'linksyms.fpc');
      DeleteFile(linkscript.fn);
    end;
  linkscript.free;
  MakeSharedLibrary:=success;   { otherwise a recursive call to link method }
end;

{*****************************************************************************
                                  Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_aix,TLinkerAIX);
{$ifdef powerpc}
  RegisterImport(system_powerpc_aix,timportlibaix);
  RegisterExport(system_powerpc_aix,texportlibaix);
  RegisterTarget(system_powerpc_aix_info);
{$endif powerpc}
{$ifdef powerpc64}
  RegisterImport(system_powerpc64_aix,timportlibaix);
  RegisterExport(system_powerpc64_aix,texportlibaix);
  RegisterTarget(system_powerpc64_aix_info);
{$endif powerpc64}
  RegisterRes(res_xcoff_info,TWinLikeResourceFile);
end.
