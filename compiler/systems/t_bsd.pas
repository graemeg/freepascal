{
    Copyright (c) 1998-2002 by Peter Vreman (original Linux)
              (c) 2000      by Marco van de Voort (FreeBSD mods)

    This unit implements support import,export,link routines
    for the (i386)FreeBSD target

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
unit t_bsd;

{$i fpcdefs.inc}

interface


implementation

  uses
    sysutils,
    cutils,cfileutl,cclasses,
    verbose,systems,globtype,globals,
    symconst,cscript,
    fmodule,aasmbase,aasmtai,aasmdata,aasmcpu,cpubase,symsym,symdef,
    import,export,link,comprsrc,rescmn,i_bsd,expunix,
    cgutils,cgbase,cgobj,cpuinfo,ogbase;

  type
    timportlibbsd=class(timportlib)
      procedure generatelib;override;
    end;

    texportlibbsd=class(texportlibunix)
    end;

    tlinkerbsd=class(texternallinker)
    private
      LdSupportsNoResponseFile : boolean;
      LibrarySuffix : Char;
      prtobj : string[80];
      ReOrder : Boolean;
      linklibc : boolean;
      Function  WriteResponseFile(isdll:boolean) : Boolean;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      function  MakeExecutable:boolean;override;
      function  MakeSharedLibrary:boolean;override;
      procedure LoadPredefinedLibraryOrder; override;
      procedure InitSysInitUnitName; override;
    end;


function ModulesLinkToLibc:boolean;
var
  hp: tmodule;
begin
  { This is called very early, ImportLibraryList is not yet merged into linkothersharedlibs.
    The former contains library names qualified with prefix and suffix (coming from
    "external 'c' name 'foo' declarations), the latter contains raw names (from "$linklib c"
    directives). }
  hp:=tmodule(loaded_units.first);
  while assigned(hp) do
    begin
      result:=Assigned(hp.ImportLibraryList.find(target_info.sharedClibprefix+'c'+target_info.sharedClibext));
      if result then break;
      result:=hp.linkothersharedlibs.find(target_info.sharedClibprefix+'c'+target_info.sharedClibext);
      if result then break;
      result:=hp.linkothersharedlibs.find('c');
      if result then break;
      hp:=tmodule(hp.next);
    end;
end;


{*****************************************************************************
                               TIMPORTLIBBSD
*****************************************************************************}

    procedure timportlibbsd.generatelib;
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
                                  TLINKERBSD
*****************************************************************************}

Constructor TLinkerBSD.Create;
begin
  Inherited Create;
  if not Dontlinkstdlibpath Then
   if target_info.system in systems_openbsd then
     LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib;=$OPENBSD_X11BASE/lib;=$OPENBSD_LOCALBASE/lib',true)
   else
     LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib;=/usr/lib;=/usr/X11R6/lib',true);
end;


procedure TLinkerBSD.SetDefaultInfo;
{
  This will also detect which libc version will be used
}
var
  LdProgram: string='ld';
begin
  if target_info.system in (systems_openbsd+[system_x86_64_dragonfly]) then
    LdProgram:='ld.bfd';
  LibrarySuffix:=' ';
  LdSupportsNoResponseFile := (target_info.system in ([system_m68k_netbsd]+systems_darwin));
  with Info do
   begin
     if LdSupportsNoResponseFile then
       begin
         ExeCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -L. -o $EXE $CATRES $FILELIST';
         DllCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $MAP $LTO $ORDERSYMS -shared -L. -o $EXE $CATRES $FILELIST'
       end
     else
       begin
         ExeCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -L. -o $EXE $RES';
         DllCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $INIT $FINI $SONAME $MAP $LTO $ORDERSYMS -shared -L. -o $EXE $RES';
       end;
     DllCmd[2]:='strip --strip-unneeded $EXE';
     { OpenBSD seems to use a wrong dynamic linker by default }
     if target_info.system in systems_openbsd then
      DynamicLinker:='/usr/libexec/ld.so'
     else if target_info.system in systems_netbsd then
      DynamicLinker:='/usr/libexec/ld.elf_so'
     else if target_info.system=system_x86_64_dragonfly then
      DynamicLinker:='/libexec/ld-elf.so.2'
     else
       DynamicLinker:='';
   end;
end;

procedure TLinkerBSD.LoadPredefinedLibraryOrder;
// put your linkorder/linkalias overrides here.
// Note: assumes only called when reordering/aliasing is used.
Begin
  if (target_info.system =system_i386_freebsd) and
     not (cs_link_no_default_lib_order in  current_settings.globalswitches) Then
    Begin
      LinkLibraryOrder.add('gcc','',15);
      LinkLibraryOrder.add('c','',50);		     // c and c_p mutual. excl?
      LinkLibraryOrder.add('c_p','',55);
      LinkLibraryOrder.add('pthread','',75);	     // pthread and c_r should be mutually exclusive
      LinkLibraryOrder.add('c_r','',76);
      LinkLibraryOrder.add('kvm','',80);		     // must be before ncurses
      if (cs_link_pthread in current_settings.globalswitches) Then     // convert libpthread to libc_r.
        LinkLibraryAliases.add('pthread','c_r');
    end;
End;


procedure TLinkerBSD.InitSysInitUnitName;
var
  cprtobj,
  gprtobj,
  si_cprt,
  si_gprt : string[80];
begin
  linklibc:=ModulesLinkToLibc;
  if current_module.islibrary and
     (target_info.system in systems_bsd) then
    begin
      prtobj:='dllprt0';
      cprtobj:='dllprt0';
      gprtobj:='dllprt0';
      SysInitUnit:='si_dll';
      si_cprt:='si_dll';
      si_gprt:='si_dll';
    end
  else
    begin
      prtobj:='prt0';
      cprtobj:='cprt0';
      gprtobj:='gprt0';
      SysInitUnit:='si_prc';
      si_cprt:='si_c';
      si_gprt:='si_g';
    end;
  // this one is a bit complex.
  // Only reorder for now if -XL or -XO params are given
  // or when -Xf.
  reorder:= linklibc and
            (
              ReorderEntries
               or
              (cs_link_pthread in current_settings.globalswitches));
  if cs_profile in current_settings.moduleswitches then
   begin
     prtobj:=gprtobj;
     SysInitUnit:=si_gprt;
     AddSharedLibrary('c');
     LibrarySuffix:='p';
     linklibc:=true;
   end
  else
   begin
     if linklibc then
       begin
         prtobj:=cprtobj;
         SysInitUnit:=si_cprt;
       end;
   end;
end;


Function TLinkerBSD.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres      : TLinkRes;
  FilesList    : TLinkRes;
  i            : longint;
  HPath        : TCmdStrListItem;
  s,s1,s2      : TCmdStr;
  linkdynamic  : boolean;
  Fl1,Fl2      : Boolean;

begin
  WriteResponseFile:=False;
  ReOrder:=False;
  linkdynamic:=False;
  { set special options for some targets }
  linkdynamic:=not(SharedLibFiles.empty);
  // after this point addition of shared libs not allowed.

  if reorder Then
     ExpandAndApplyOrder(SharedLibFiles);

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,not LdSupportsNoResponseFile);

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
     if LdSupportsNoResponseFile then
       LinkRes.Add('-L'+HPath.Str)
     else
       LinkRes.Add('SEARCH_DIR("'+HPath.Str+'")');
     HPath:=TCmdStrListItem(HPath.Next);
   end;
  HPath:=TCmdStrListItem(LibrarySearchPath.First);
  while assigned(HPath) do
   begin
     if LdSupportsNoResponseFile then
       LinkRes.Add('-L'+HPath.Str)
     else
       LinkRes.Add('SEARCH_DIR("'+HPath.Str+'")');
     HPath:=TCmdStrListItem(HPath.Next);
   end;

  { force local symbol resolution (i.e., inside the shared }
  { library itself) for all non-exorted symbols, otherwise }
  { several RTL symbols of FPC-compiled shared libraries   }
  { will be bound to those of a single shared library or   }
  { to the main program                                    }
  if (isdll) and (target_info.system in systems_bsd) then
    begin
      LinkRes.add('VERSION');
      LinkRes.add('{');
      LinkRes.add('  {');
      if not texportlibunix(exportlib).exportedsymnames.empty then
        begin
          LinkRes.add('    global:');
          repeat
            LinkRes.add('      '+texportlibunix(exportlib).exportedsymnames.getfirst+';');
          until texportlibunix(exportlib).exportedsymnames.empty;
        end;
      LinkRes.add('    local:');
      LinkRes.add('      *;');
      LinkRes.add('  };');
      LinkRes.add('}');
    end;

  if not LdSupportsNoResponseFile then
    LinkRes.Add('INPUT(');
  { add objectfiles, start with prt0 always }
  if not (target_info.system in systems_internal_sysinit) and (prtobj<>'') then
   LinkRes.AddFileName(FindObjectFile(prtobj,'',false));
  { try to add crti and crtbegin if linking to C }
  if linklibc then
   begin
     if librarysearchpath.FindFile('crti.o',false,s) then
      LinkRes.AddFileName(s);
     if ((cs_create_pic in current_settings.moduleswitches) and
         not (target_info.system in systems_openbsd)) or
        (current_module.islibrary and
         (target_info.system in systems_openbsd)) then
       begin
         if librarysearchpath.FindFile('crtbeginS.o',false,s) then
           LinkRes.AddFileName(s);
       end
       else
         if (cs_link_staticflag in current_settings.globalswitches) and
           librarysearchpath.FindFile('crtbeginT.o',false,s) then
             LinkRes.AddFileName(s)
         else if librarysearchpath.FindFile('crtbegin.o',false,s) then
             LinkRes.AddFileName(s);
   end;

  { main objectfiles }
   while not ObjectFiles.Empty do
    begin
      s:=ObjectFiles.GetFirst;
      if s<>'' then
        if LdSupportsNoResponseFile then
          LinkRes.AddFileName(s)
        else
          LinkRes.AddFileName(maybequoted(s));
    end;

  if not LdSupportsNoResponseFile then
   LinkRes.Add(')');

  { Write staticlibraries }
  if not StaticLibFiles.Empty then
   begin
     if not LdSupportsNoResponseFile then
       LinkRes.Add('GROUP(');
     While not StaticLibFiles.Empty do
      begin
        S:=StaticLibFiles.GetFirst;
        if LdSupportsNoResponseFile then
          LinkRes.AddFileName(s)
        else
          LinkRes.AddFileName(maybequoted(s))
      end;
     if not LdSupportsNoResponseFile then
       LinkRes.Add(')');
   end;

  { Write sharedlibraries like -l<lib>, also add the needed dynamic linker
    here to be sure that it gets linked this is needed for glibc2 systems (PFV) }
  if not SharedLibFiles.Empty then
   begin
     if not LdSupportsNoResponseFile then
       LinkRes.Add('INPUT(');
     While not SharedLibFiles.Empty do
      begin
        S:=SharedLibFiles.GetFirst;
        if (s<>'c') or reorder then
         begin
           i:=Pos(target_info.sharedlibext,S);
           if i>0 then
            Delete(S,i,255);
           LinkRes.Add('-l'+s);
         end
        else
         begin
           linklibc:=true;
           linkdynamic:=false; { libc will include the ld-* for us }
         end;
      end;
     { be sure that libc is the last lib }
     if linklibc and not reorder then
       Begin
         If LibrarySuffix=' ' Then
          LinkRes.Add('-lc')
         else
          LinkRes.Add('-lc_'+LibrarySuffix);
         If LibrarySuffix='r' Then
             LinkRes.Add('-lc');
       end;
     { when we have -static for the linker the we also need libgcc }
     if (cs_link_staticflag in current_settings.globalswitches) then
      LinkRes.Add('-lgcc');
     if linkdynamic and (Info.DynamicLinker<>'') and
        not(target_info.system in systems_openbsd) then
      LinkRes.AddFileName(Info.DynamicLinker);
     if not LdSupportsNoResponseFile then
       LinkRes.Add(')');
   end;

  { objects which must be at the end }
  if linklibc then
   begin
     if ((cs_create_pic in current_settings.moduleswitches) and
         not (target_info.system in systems_openbsd)) or
        (current_module.islibrary and
         (target_info.system in systems_openbsd)) then
       Fl1:=librarysearchpath.FindFile('crtendS.o',false,s1)
     else
       Fl1:=librarysearchpath.FindFile('crtend.o',false,s1);
     Fl2:=librarysearchpath.FindFile('crtn.o',false,s2);
     if Fl1 or Fl2 then
      begin
        LinkRes.Add('INPUT(');
         If Fl1 Then
        LinkRes.AddFileName(s1);
        If Fl2 Then
         LinkRes.AddFileName(s2);
        LinkRes.Add(')');
      end;
   end;
{ Write and Close response }
  linkres.writetodisk;
  linkres.Free;

  WriteResponseFile:=True;
end;


function TLinkerBSD.MakeExecutable:boolean;
var
  binstr,
  cmdstr,
  mapstr,
  targetstr,
  emulstr,
  ltostr,
  ordersymfile: TCmdStr;
  linkscript: TAsmScript;
  DynLinkStr : string[60];
  GCSectionsStr,
  StaticStr,
  StripStr   : string[63];
  success,
  useshell : boolean;
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Create some replacements }
  StaticStr:='';
  StripStr:='';
  DynLinkStr:='';
  GCSectionsStr:='';
  linkscript:=nil;
  mapstr:='';
  ltostr:='';
  if (cs_link_map in current_settings.globalswitches) then
    mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
  { i386_freebsd needs -b elf32-i386-freebsd and -m elf_i386_fbsd
    to avoid creation of a i386:x86_64 arch binary }

  if target_info.system=system_i386_freebsd then
    begin
      targetstr:='-b elf32-i386-freebsd';
      emulstr:='-m elf_i386_fbsd';
    end
  else
    begin
      targetstr:='';
      emulstr:='';
    end;

  if (cs_link_staticflag in current_settings.globalswitches) then
    begin
      if (target_info.system=system_m68k_netbsd) and
         ((cs_link_on_target in current_settings.globalswitches) or
          (target_info.system=source_info.system)) then
        StaticStr:='-Bstatic'
      else
        StaticStr:='-static';
    end;
  if (cs_link_strip in current_settings.globalswitches) then
    StripStr:='-s';

  if (cs_link_smart in current_settings.globalswitches) and
     (tf_smartlink_sections in target_info.flags) then
    GCSectionsStr:='--gc-sections';

   if(cs_profile in current_settings.moduleswitches) or
     ((Info.DynamicLinker<>'') and
      ((not SharedLibFiles.Empty) or
       (target_info.system in systems_openbsd))) then
   DynLinkStr:='-dynamic-linker='+Info.DynamicLinker;

  if CShared Then
   begin
      DynLinKStr:=DynLinkStr+' --shared'
   end;

{ Use -nopie on OpenBSD if PIC support is turned off }
  if (target_info.system in systems_openbsd) and
     not(cs_create_pic in current_settings.moduleswitches) then
    Info.ExtraOptions:=Info.ExtraOptions+' -nopie';

{ -N seems to be needed on NetBSD/earm }
  if (target_info.system in [system_arm_netbsd]) then
    Info.ExtraOptions:=Info.ExtraOptions+' -N';

{ Write used files and libraries }
  WriteResponseFile(false);

{ Write symbol order file }
  ordersymfile:=WriteSymbolOrderFile;

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$TARGET',targetstr);
  Replace(cmdstr,'$EMUL',EmulStr);
  Replace(cmdstr,'$MAP',mapstr);
  Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$LTO',ltostr);
  if ordersymfile<>'' then
    Replace(cmdstr,'$ORDERSYMS','--symbol-ordering-file '+maybequoted(ordersymfile))
  else
    Replace(cmdstr,'$ORDERSYMS','');

  Replace(cmdstr,'$FILELIST','');
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  BinStr:=FindUtil(utilsprefix+BinStr);

  if (LdSupportsNoResponseFile) and
     not(cs_link_nolink in current_settings.globalswitches) then
    begin
      { we have to use a script to use the IFS hack }
      linkscript:=GenerateScript(outputexedir+'ppaslink');
      linkscript.AddLinkCommand(BinStr,CmdStr,'');
      linkscript.WriteToDisk;
      BinStr:=linkscript.fn;
      if not path_absolute(BinStr) then
        if cs_link_on_target in current_settings.globalswitches then
          BinStr:='.'+target_info.dirsep+BinStr
        else
          BinStr:='.'+source_info.dirsep+BinStr;
      CmdStr:='';
    end;

  useshell:=not (tf_no_backquote_support in source_info.flags);
  success:=DoExec(BinStr,CmdStr,true,LdSupportsNoResponseFile or useshell);

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   begin
     DeleteFile(outputexedir+Info.ResName);
     if ordersymfile<>'' then
       DeleteFile(ordersymfile);
     if LdSupportsNoResponseFile Then
       begin
         DeleteFile(linkscript.fn);
         linkscript.free
       end;
   end;

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


Function TLinkerBSD.MakeSharedLibrary:boolean;
var
  InitStr,
  FiniStr,
  SoNameStr : string[80];
  linkscript: TAsmScript;
  binstr,
  cmdstr,
  mapstr,
  ltostr,
  ordersymfile,
  targetstr,
  emulstr       : TCmdStr;
  GCSectionsStr : string[63];
  exportedsyms: text;
  success : boolean;
begin
  MakeSharedLibrary:=false;
  GCSectionsStr:='';
  mapstr:='';
  ltostr:='';
  linkscript:=nil;
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.sharedlibfilename);

{ Write used files and libraries }
  WriteResponseFile(true);

{ Write symbol order file }
  ordersymfile:=WriteSymbolOrderFile;

  if (cs_link_smart in current_settings.globalswitches) and
     (tf_smartlink_sections in target_info.flags) then
     { disabled because not tested
      GCSectionsStr:='--gc-sections' }
    ;

  if (cs_link_map in current_settings.globalswitches) then
    mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.sharedlibfilename,'.map'));

  { i386_freebsd needs -b elf32-i386-freebsd and -m elf_i386_fbsd
    to avoid creation of a i386:x86_64 arch binary }

  if target_info.system=system_i386_freebsd then
    begin
      targetstr:='-b elf32-i386-freebsd';
      emulstr:='-m elf_i386_fbsd';
    end
  else
    begin
      targetstr:='';
      emulstr:='';
    end;

  InitStr:='-init FPC_LIB_START';
  FiniStr:='-fini FPC_LIB_EXIT';
  SoNameStr:='-soname '+ExtractFileName(current_module.sharedlibfilename);

{ Call linker }
  SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$TARGET',targetstr);
  Replace(cmdstr,'$EMUL',EmulStr);
  Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  Replace(cmdstr,'$FILELIST','');
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$INIT',InitStr);
  Replace(cmdstr,'$FINI',FiniStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$SONAME',SoNameStr);
  Replace(cmdstr,'$MAP',mapstr);
  Replace(cmdstr,'$LTO',ltostr);
  if ordersymfile<>'' then
    Replace(cmdstr,'$ORDERSYMS','--symbol-ordering-file '+maybequoted(ordersymfile))
  else
    Replace(cmdstr,'$ORDERSYMS','');
  BinStr:=FindUtil(utilsprefix+BinStr);

  if (LdSupportsNoResponseFile) and
     not(cs_link_nolink in current_settings.globalswitches) then
    begin
      { we have to use a script to use the IFS hack }
      linkscript:=GenerateScript(outputexedir+'ppaslink');
      linkscript.AddLinkCommand(BinStr,CmdStr,'');
      linkscript.WriteToDisk;
      BinStr:=linkscript.fn;
      if not path_absolute(BinStr) then
        if cs_link_on_target in current_settings.globalswitches then
          BinStr:='.'+target_info.dirsep+BinStr
        else
          BinStr:='.'+source_info.dirsep+BinStr;
      CmdStr:='';
    end;

  success:=DoExec(BinStr,cmdstr,true,LdSupportsNoResponseFile);

{ Strip the library ? }
  if success and (cs_link_strip in current_settings.globalswitches) then
   begin
     SplitBinCmd(Info.DllCmd[2],binstr,cmdstr);
     Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
     success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,false,false);
   end;

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    begin
      DeleteFile(outputexedir+Info.ResName);
      if ordersymfile<>'' then
        DeleteFile(ordersymfile);
      if LdSupportsNoResponseFile Then
        begin
          DeleteFile(linkscript.fn);
          linkscript.free
        end;
    end;

  MakeSharedLibrary:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_bsd,TLinkerBSD);
{$ifdef x86_64}
  RegisterImport(system_x86_64_dragonfly,timportlibbsd);
  RegisterExport(system_x86_64_dragonfly,texportlibbsd);
  RegisterTarget(system_x86_64_dragonfly_info);
  RegisterImport(system_x86_64_freebsd,timportlibbsd);
  RegisterExport(system_x86_64_freebsd,texportlibbsd);
  RegisterTarget(system_x86_64_freebsd_info);
  RegisterImport(system_x86_64_openbsd,timportlibbsd);
  RegisterExport(system_x86_64_openbsd,texportlibbsd);
  RegisterTarget(system_x86_64_openbsd_info);
  RegisterImport(system_x86_64_netbsd,timportlibbsd);
  RegisterExport(system_x86_64_netbsd,texportlibbsd);
  RegisterTarget(system_x86_64_netbsd_info);
{$endif}
{$ifdef i386}
  RegisterImport(system_i386_freebsd,timportlibbsd);
  RegisterExport(system_i386_freebsd,texportlibbsd);
  RegisterTarget(system_i386_freebsd_info);
  RegisterImport(system_i386_netbsd,timportlibbsd);
  RegisterExport(system_i386_netbsd,texportlibbsd);
  RegisterTarget(system_i386_netbsd_info);
  RegisterImport(system_i386_openbsd,timportlibbsd);
  RegisterExport(system_i386_openbsd,texportlibbsd);
  RegisterTarget(system_i386_openbsd_info);
{$endif i386}
{$ifdef m68k}
  RegisterImport(system_m68k_netbsd,timportlibbsd);
  RegisterExport(system_m68k_netbsd,texportlibbsd);
  RegisterTarget(system_m68k_netbsd_info);
{$endif m68k}
{$ifdef powerpc}
  RegisterImport(system_powerpc_netbsd,timportlibbsd);
  RegisterExport(system_powerpc_netbsd,texportlibbsd);
  RegisterTarget(system_powerpc_netbsd_info);
{$endif powerpc}
{$ifdef arm}
  RegisterImport(system_arm_netbsd,timportlibbsd);
  RegisterExport(system_arm_netbsd,texportlibbsd);
  RegisterTarget(system_arm_netbsd_info);
{$endif arm}

  RegisterRes(res_elf_info,TWinLikeResourceFile);
end.
