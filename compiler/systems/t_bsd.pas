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
    timportlibdarwin=class(timportlib)
      procedure generatelib;override;
    end;

    timportlibbsd=class(timportlib)
      procedure generatelib;override;
    end;

    texportlibbsd=class(texportlibunix)
    end;

    texportlibdarwin=class(texportlibbsd)
      procedure setinitname(list: TAsmList; const s: string); override;
      procedure setfininame(list: TAsmList; const s: string); override;
    end;

    tlinkerbsd=class(texternallinker)
    private
      LdSupportsNoResponseFile : boolean;
      LibrarySuffix : Char;
      prtobj : string[80];
      ReOrder : Boolean;
      linklibc : boolean;
      Function  WriteResponseFile(isdll:boolean) : Boolean;
      function GetDarwinCrt1ObjName(isdll: boolean): TCmdStr;
      Function GetDarwinPrtobjName(isdll: boolean): TCmdStr;
      Function WriteSymbolOrderFile: TCmdStr;
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
                             TIMPORTLIBDARWIN
*****************************************************************************}

    procedure timportlibdarwin.generatelib;
      begin
      end;


{*****************************************************************************
                             TEXPORTLIBDARWIN
*****************************************************************************}

    procedure texportlibdarwin.setinitname(list: TAsmList; const s: string);
      begin
        new_section(list,sec_init_func,'',sizeof(pint));
        list.concat(Tai_const.Createname(s,0));
      end;


    procedure texportlibdarwin.setfininame(list: TAsmList; const s: string);
      begin
        new_section(list,sec_term_func,'',sizeof(pint));
        list.concat(Tai_const.Createname(s,0));
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
   if target_info.system in systems_darwin then
     { Mac OS X doesn't have a /lib }
     LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib',true)
   else if target_info.system in systems_openbsd then
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
  if target_info.system in systems_openbsd then
    LdProgram:='ld.bfd';
  LibrarySuffix:=' ';
  LdSupportsNoResponseFile := (target_info.system in ([system_m68k_netbsd]+systems_darwin));
  with Info do
   begin
     if LdSupportsNoResponseFile then
       begin
         if not(target_info.system in systems_darwin) then
           begin
             ExeCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -L. -o $EXE $CATRES $FILELIST';
             DllCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $MAP $LTO $ORDERSYMS -shared -L. -o $EXE $CATRES $FILELIST'
           end
         else
           begin
{$ifndef cpu64bitaddr}
             { Set the size of the page at address zero to 64kb, so nothing
               is loaded below that address. This avoids problems with the
               strange Windows-compatible resource handling that assumes
               that addresses below 64kb do not exist.

               On 64bit systems, page zero is 4GB by default, so no problems
               there.
             }
             { In case of valgrind, don't do that, because it cannot deal with
               a custom pagezero size -- in general, this should not cause any
               problems because the resources are added at the end and most
               programs with problems that require Valgrind will have more
               than 60KB of data (first 4KB of address space is always invalid)
             }
             ExeCmd[1]:=LdProgram+' $PRTOBJ $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST';
             if not(cs_gdb_valgrind in current_settings.globalswitches) then
               ExeCmd[1]:=ExeCmd[1]+' -pagezero_size 0x10000';
{$else ndef cpu64bitaddr}
             ExeCmd[1]:=LdProgram+' $PRTOBJ $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST';
{$endif ndef cpu64bitaddr}
             if (apptype<>app_bundle) then
               DllCmd[1]:=LdProgram+' $PRTOBJ $TARGET $EMUL $OPT $GCSECTIONS $MAP $LTO $ORDERSYMS -dynamic -dylib -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST'
             else
               DllCmd[1]:=LdProgram+' $PRTOBJ $TARGET $EMUL $OPT $GCSECTIONS $MAP $LTO $ORDERSYMS -dynamic -bundle -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST'
           end
       end
     else
       begin
         ExeCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -L. -o $EXE $RES';
         DllCmd[1]:=LdProgram+' $TARGET $EMUL $OPT $INIT $FINI $SONAME $MAP $LTO $ORDERSYMS -shared -L. -o $EXE $RES';
       end;
     if not(target_info.system in systems_darwin) then
       DllCmd[2]:='strip --strip-unneeded $EXE'
     else
       DllCmd[2]:='strip -x $EXE';
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
  if not(target_info.system in systems_darwin) then
    begin
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
    end
else
    begin
          LinkLibraryOrder.add('gcc','',15);
          LinkLibraryOrder.add('c','',50);
   end;
End;


procedure TLinkerBSD.InitSysInitUnitName;
var
  cprtobj,
  gprtobj,
  si_cprt,
  si_gprt : string[80];
begin
  if target_info.system in systems_darwin then
    begin
      { for darwin: always link dynamically against libc }
      linklibc := true;
      reorder:=reorderentries;
      prtobj:='';
      SysInitUnit:='sysinit';
    end
  else
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
end;


function TLinkerBSD.GetDarwinCrt1ObjName(isdll: boolean): TCmdStr;
begin
  if not isdll then
    begin
      if not(cs_profile in current_settings.moduleswitches) then
        begin
          case target_info.system of
            system_powerpc_darwin,
            system_powerpc64_darwin,
            system_i386_darwin,
            system_x86_64_darwin:
              begin
                { 10.8 and later: no crt1.* }
                if CompareVersionStrings(MacOSXVersionMin,'10.8')>=0 then
                  exit('');
                { x86: crt1.10.6.o -> crt1.10.5.o -> crt1.o }
                { others: crt1.10.5 -> crt1.o }
                if (target_info.system in [system_i386_darwin,system_x86_64_darwin]) and
                   (CompareVersionStrings(MacOSXVersionMin,'10.6')>=0) then
                  exit('crt1.10.6.o');
                if CompareVersionStrings(MacOSXVersionMin,'10.5')>=0 then
                  exit('crt1.10.5.o');
              end;
            system_arm_darwin:
              begin
                { iOS:
                    iOS 6 and later: nothing
                    iOS 3.1 - 5.x: crt1.3.1.o
                    pre-iOS 3.1: crt1.o
                }
                if (CompareVersionStrings(iPhoneOSVersionMin,'6.0')>=0) then
                  exit('');
                if (CompareVersionStrings(iPhoneOSVersionMin,'3.1')>=0) then
                  exit('crt1.3.1.o');
              end;
            system_i386_iphonesim,
            system_x86_64_iphonesim:
              begin
                { "recent versions" must not use anything (https://github.com/llvm-mirror/clang/commit/e6d04f3d152a22077022cf9287d4c538a0918ab0 )
                  What those recent versions could be, is anyone's guess. It
                  still seems to work with 8.1 and no longer with 8.3, so use
                  8.1 as a cut-off point }
                if (CompareVersionStrings(iPhoneOSVersionMin,'8.1')>0) then
                  exit('');
              end;
            system_aarch64_darwin:
              { never anything }
              exit('');
            else
              Internalerror(2019050709);
          end;
          { nothing special -> default }
          result:='crt1.o';
        end
      else
        begin
          result:='gcrt1.o';
          { 10.8 and later: tell the linker to use 'start' instead of "_main"
            as entry point }
          if CompareVersionStrings(MacOSXVersionMin,'10.8')>=0 then
            Info.ExeCmd[1]:=Info.ExeCmd[1]+' -no_new_main';
        end;
    end
  else
    begin
      if (apptype=app_bundle) then
        begin
          case target_info.system of
            system_powerpc_darwin,
            system_powerpc64_darwin,
            system_i386_darwin,
            system_x86_64_darwin:
              begin
                { < 10.6: bundle1.o
                  >= 10.6: nothing }
                if CompareVersionStrings(MacOSXVersionMin,'10.6')>=0 then
                  exit('');
              end;
            system_arm_darwin,
            system_aarch64_darwin:
              begin
                { iOS: < 3.1: bundle1.o
                       >= 3.1: nothing }
                if (CompareVersionStrings(iPhoneOSVersionMin,'3.1')>=0) then
                  exit('');
              end;
            system_i386_iphonesim,
            system_x86_64_iphonesim:
              begin
                { see rule for crt1.o }
                if (CompareVersionStrings(iPhoneOSVersionMin,'8.1')>0) then
                  exit('');
              end;
            else
              Internalerror(2019050710);
          end;
          result:='bundle1.o';
        end
      else
        begin
          case target_info.system of
            system_powerpc_darwin,
            system_powerpc64_darwin,
            system_i386_darwin,
            system_x86_64_darwin:
              begin
                { >= 10.6: nothing
                  = 10.5: dylib1.10.5.o
                  < 10.5: dylib1.o
                }
                if CompareVersionStrings(MacOSXVersionMin,'10.6')>=0 then
                  exit('');
                if CompareVersionStrings(MacOSXVersionMin,'10.5')>=0 then
                  exit('dylib1.10.5.o');
              end;
            system_arm_darwin,
            system_aarch64_darwin:
              begin
                { iOS: < 3.1: dylib1.o
                       >= 3.1: nothing }
                if (CompareVersionStrings(iPhoneOSVersionMin,'3.1')>=0) then
                  exit('');
              end;
            system_i386_iphonesim,
            system_x86_64_iphonesim:
              begin
                { see rule for crt1.o }
                if (CompareVersionStrings(iPhoneOSVersionMin,'8.1')>0) then
                  exit('');
              end;
            else
              Internalerror(2019050711);
          end;
          result:='dylib1.o';
        end;
    end;
end;


Function TLinkerBSD.GetDarwinPrtobjName(isdll: boolean): TCmdStr;
var
  startupfile: TCmdStr;
begin
  result:='';

  startupfile:=GetDarwinCrt1ObjName(isdll);
  if startupfile<>'' then
    begin
     if not librarysearchpath.FindFile(startupfile,false,result) then
       result:='/usr/lib/'+startupfile;
    end;
  result:=maybequoted(result);
end;


function tlinkerbsd.WriteSymbolOrderFile: TCmdStr;
  var
    item: TCmdStrListItem;
    symfile: TScript;
  begin
    result:='';
    { only for darwin for now; can also enable for other platforms when using
      the LLVM linker }
    if (OrderedSymbols.Empty) or
       not(tf_supports_symbolorderfile in target_info.flags) then
      exit;
    symfile:=TScript.Create(outputexedir+'symbol_order.fpc');
    item:=TCmdStrListItem(OrderedSymbols.First);
    while assigned(item) do
      begin
        symfile.add(item.str);
        item:=TCmdStrListItem(item.next);
      end;
    symfile.WriteToDisk;
    result:=symfile.fn;
    symfile.Free;
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
  IsDarwin     : Boolean;

begin
  WriteResponseFile:=False;
  ReOrder:=False;
  linkdynamic:=False;
  IsDarwin:=target_info.system in systems_darwin;
{ set special options for some targets }
  if not IsDarwin Then
    begin
      linkdynamic:=not(SharedLibFiles.empty);
      // after this point addition of shared libs not allowed.
    end
  else
    begin
{$ifdef MACOSX104ORHIGHER}
      { not sure what this is for, but gcc always links against it }
      if not(cs_profile in current_settings.moduleswitches) then
        AddSharedLibrary('SystemStubs')
      else
        AddSharedLibrary('SystemStubs_profile');
{$endif MACOSX104ORHIGHER}
    end;

  if reorder Then
     ExpandAndApplyOrder(SharedLibFiles);

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,not LdSupportsNoResponseFile);

  if (target_info.system in systems_darwin) and
     (sysrootpath<>'') then
    begin
      LinkRes.Add('-syslibroot');
      LinkRes.Add(sysrootpath);
    end;

  if (target_info.system in systems_darwin) then
    begin
      LinkRes.Add('-arch');
      case target_info.system of
        system_powerpc_darwin:
          LinkRes.Add('ppc');
        system_i386_darwin,
        system_i386_iphonesim:
          LinkRes.Add('i386');
        system_powerpc64_darwin:
          LinkRes.Add('ppc64');
        system_x86_64_darwin,
        system_x86_64_iphonesim:
          LinkRes.Add('x86_64');
        system_arm_darwin:
          { current versions of the linker require the sub-architecture type
            to be specified }
          LinkRes.Add(lower(cputypestr[current_settings.cputype]));
        system_aarch64_darwin:
          LinkRes.Add('arm64');
        else
          internalerror(2014121801);
      end;
      if MacOSXVersionMin<>'' then
        begin
          LinkRes.Add('-macosx_version_min');
          LinkRes.Add(MacOSXVersionMin);
        end
      else if iPhoneOSVersionMin<>'' then
        begin
          LinkRes.Add('-iphoneos_version_min');
          LinkRes.Add(iPhoneOSVersionMin);
        end;
    end;
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

  if (target_info.system in systems_darwin) then
    begin
      HPath:=TCmdStrListItem(current_module.localframeworksearchpath.First);
      while assigned(HPath) do
       begin
         LinkRes.Add('-F'+HPath.Str);
         HPath:=TCmdStrListItem(HPath.Next);
       end;
      HPath:=TCmdStrListItem(FrameworkSearchPath.First);
      while assigned(HPath) do
       begin
         LinkRes.Add('-F'+HPath.Str);
         HPath:=TCmdStrListItem(HPath.Next);
       end;
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
  if linklibc and
     not IsDarwin Then
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

  { Generate linkfiles.res file if needed }
  { Only needed on Windows, due to the limitation of 8196 characters for command line }
  if (LdSupportsNoResponseFile) and
     (source_info.system in systems_all_windows) then
   begin
     FilesList:=TLinkRes.Create(outputexedir+'linkfiles.res',false);
     while not ObjectFiles.Empty do
      begin
        s:=ObjectFiles.GetFirst;
        if s<>'' then
         begin
           repeat
             i:=Pos(source_info.dirsep,s);
             if i>0 then 
               s[i]:=target_info.dirsep;
           until i=0;
           FilesList.Add(s);
         end;
      end;
     FilesList.writetodisk;
     FilesList.Free;
   end
  else
   begin
     while not ObjectFiles.Empty do
      begin
        s:=ObjectFiles.GetFirst;
        if s<>'' then
          if LdSupportsNoResponseFile then
            LinkRes.AddFileName(s)
          else
            LinkRes.AddFileName(maybequoted(s));
      end;
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

  { frameworks for Darwin }
  if IsDarwin then
    while not FrameworkFiles.empty do
      begin
        LinkRes.Add('-framework');
        LinkRes.Add(FrameworkFiles.GetFirst);
      end;

  { objects which must be at the end }
  if linklibc and
     not IsDarwin Then
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
  extdbgbinstr,
  extdbgcmdstr,
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
    if (target_info.system in systems_darwin) then
      StripStr:='-x'
    else
      StripStr:='-s';

  if (cs_link_smart in current_settings.globalswitches) and
     (tf_smartlink_sections in target_info.flags) then
    if not(target_info.system in systems_darwin) then
      GCSectionsStr:='--gc-sections'
    else
      GCSectionsStr:='-dead_strip -no_dead_strip_inits_and_terms';

   if(not(target_info.system in systems_darwin) and
      (cs_profile in current_settings.moduleswitches)) or
     ((Info.DynamicLinker<>'') and
      ((not SharedLibFiles.Empty) or
       (target_info.system in systems_openbsd))) then
   DynLinkStr:='-dynamic-linker='+Info.DynamicLinker;

  if CShared Then
   begin
   if not(target_info.system in systems_darwin) then
     DynLinKStr:=DynLinkStr+' --shared'
    else
     DynLinKStr:=DynLinkStr+' -dynamic'; // one dash!
   end;

  { add custom LTO library if using custom clang }
  if (target_info.system in systems_darwin) and
     (cs_lto in current_settings.moduleswitches) and
     not(cs_link_on_target in current_settings.globalswitches) and
     (utilsdirectory<>'') and
     FileExists(utilsdirectory+'/../lib/libLTO.dylib',true) then
    begin
      ltostr:='-lto_library '+maybequoted(utilsdirectory+'/../lib/libLTO.dylib');
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
    begin
      if target_info.system in systems_darwin then
        Replace(cmdstr,'$ORDERSYMS','-order_file '+maybequoted(ordersymfile))
      else
        Replace(cmdstr,'$ORDERSYMS','--symbol-ordering-file '+maybequoted(ordersymfile))
    end
  else
    Replace(cmdstr,'$ORDERSYMS','');

  if (LdSupportsNoResponseFile) and (source_info.system in systems_all_windows) then
    Replace(cmdstr,'$FILELIST','-filelist '+maybequoted(outputexedir+'linkfiles.res'))
  else
    Replace(cmdstr,'$FILELIST','');
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  if (target_info.system in systems_darwin) then
    Replace(cmdstr,'$PRTOBJ',GetDarwinPrtobjName(false));
  BinStr:=FindUtil(utilsprefix+BinStr);

  { create dsym file? }
  extdbgbinstr:='';
  extdbgcmdstr:='';
  if (target_info.system in systems_darwin) and
     (target_dbg.id in [dbg_dwarf2,dbg_dwarf3]) and
     (cs_link_separate_dbg_file in current_settings.globalswitches) then
    begin
      extdbgbinstr:=FindUtil(utilsprefix+'dsymutil');
      extdbgcmdstr:=maybequoted(current_module.exefilename);
    end;

  if (LdSupportsNoResponseFile) and
     not(cs_link_nolink in current_settings.globalswitches) then
    begin
      { we have to use a script to use the IFS hack }
      linkscript:=GenerateScript(outputexedir+'ppaslink');
      linkscript.AddLinkCommand(BinStr,CmdStr,'');
      if (extdbgcmdstr<>'') then
        linkscript.AddLinkCommand(extdbgbinstr,extdbgcmdstr,'');
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
  if (success and
      (extdbgbinstr<>'') and
      (cs_link_nolink in current_settings.globalswitches)) then
    success:=DoExec(extdbgbinstr,extdbgcmdstr,false,LdSupportsNoResponseFile);

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

  { Remove linkfiles.res }
  if (success) and (LdSupportsNoResponseFile) and (source_info.system in systems_all_windows) then
    DeleteFile(outputexedir+'linkfiles.res');

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
  emulstr,
  extdbgbinstr,
  extdbgcmdstr  : TCmdStr;
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
    if not(target_info.system in systems_darwin) then
     { disabled because not tested
      GCSectionsStr:='--gc-sections' }
    else
      GCSectionsStr:='-dead_strip -no_dead_strip_inits_and_terms';

  if (cs_link_map in current_settings.globalswitches) then
    mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.sharedlibfilename,'.map'));

  { add custom LTO library if using custom clang }
  if (target_info.system in systems_darwin) and
     (cs_lto in current_settings.moduleswitches) and
     not(cs_link_on_target in current_settings.globalswitches) and
     (utilsdirectory<>'') and
     FileExists(utilsdirectory+'/../lib/libLTO.dylib',true) then
    begin
      ltostr:='-lto_library '+maybequoted(utilsdirectory+'/../lib/libLTO.dylib');
    end;

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
{$ifndef darwin}
  Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
{$else darwin}
  Replace(cmdstr,'$EXE',maybequoted(ExpandFileName(current_module.sharedlibfilename)));
{$endif darwin}
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$TARGET',targetstr);
  Replace(cmdstr,'$EMUL',EmulStr);
  Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  if (LdSupportsNoResponseFile) and (source_info.system in systems_all_windows) then
    Replace(cmdstr,'$FILELIST','-filelist '+maybequoted(outputexedir+'linkfiles.res'))
  else
    Replace(cmdstr,'$FILELIST','');
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$INIT',InitStr);
  Replace(cmdstr,'$FINI',FiniStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$SONAME',SoNameStr);
  Replace(cmdstr,'$MAP',mapstr);
  Replace(cmdstr,'$LTO',ltostr);
  if ordersymfile<>'' then
    begin
      if target_info.system in systems_darwin then
        Replace(cmdstr,'$ORDERSYMS','-order_file '+maybequoted(ordersymfile))
      else
        Replace(cmdstr,'$ORDERSYMS','--symbol-ordering-file '+maybequoted(ordersymfile))
    end
  else
    Replace(cmdstr,'$ORDERSYMS','');
  if (target_info.system in systems_darwin) then
    Replace(cmdstr,'$PRTOBJ',GetDarwinPrtobjName(true));
  BinStr:=FindUtil(utilsprefix+BinStr);

  { create dsym file? }
  extdbgbinstr:='';
  extdbgcmdstr:='';
  if (target_info.system in systems_darwin) and
     (target_dbg.id in [dbg_dwarf2,dbg_dwarf3]) and
     (cs_link_separate_dbg_file in current_settings.globalswitches) then
    begin
      extdbgbinstr:=FindUtil(utilsprefix+'dsymutil');
      extdbgcmdstr:=maybequoted(current_module.sharedlibfilename);
    end;

  if (target_info.system in systems_darwin) then
    begin
      { exported symbols for darwin }
      if not texportlibunix(exportlib).exportedsymnames.empty then
        begin
          assign(exportedsyms,outputexedir+'linksyms.fpc');
          rewrite(exportedsyms);
          repeat
            writeln(exportedsyms,texportlibunix(exportlib).exportedsymnames.getfirst);
          until texportlibunix(exportlib).exportedsymnames.empty;
          close(exportedsyms);
          cmdstr:=cmdstr+' -exported_symbols_list '+maybequoted(outputexedir)+'linksyms.fpc';
        end;
    end;

  if (LdSupportsNoResponseFile) and
     not(cs_link_nolink in current_settings.globalswitches) then
    begin
      { we have to use a script to use the IFS hack }
      linkscript:=GenerateScript(outputexedir+'ppaslink');
      linkscript.AddLinkCommand(BinStr,CmdStr,'');
      if (extdbgbinstr<>'') then
        linkscript.AddLinkCommand(extdbgbinstr,extdbgcmdstr,'');
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
  if (success and
      (extdbgbinstr<>'') and
      (cs_link_nolink in current_settings.globalswitches)) then
    success:=DoExec(extdbgbinstr,extdbgcmdstr,false,LdSupportsNoResponseFile);

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
      if (target_info.system in systems_darwin) then
        DeleteFile(outputexedir+'linksyms.fpc');
    end;

  { Remove linkfiles.res }
  if (success) and (LdSupportsNoResponseFile) and (source_info.system in systems_all_windows) then
    DeleteFile(outputexedir+'linkfiles.res');

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

  RegisterImport(system_x86_64_darwin,timportlibdarwin);
  RegisterExport(system_x86_64_darwin,texportlibdarwin);
  RegisterTarget(system_x86_64_darwin_info);
  RegisterImport(system_x86_64_iphonesim,timportlibdarwin);
  RegisterExport(system_x86_64_iphonesim,texportlibdarwin);
  RegisterTarget(system_x86_64_iphonesim_info);
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
  RegisterImport(system_i386_darwin,timportlibdarwin);
  RegisterExport(system_i386_darwin,texportlibdarwin);
  RegisterTarget(system_i386_darwin_info);
  RegisterImport(system_i386_iphonesim,timportlibdarwin);
  RegisterExport(system_i386_iphonesim,texportlibdarwin);
  RegisterTarget(system_i386_iphonesim_info);
{$endif i386}
{$ifdef m68k}
  RegisterImport(system_m68k_netbsd,timportlibbsd);
  RegisterExport(system_m68k_netbsd,texportlibbsd);
  RegisterTarget(system_m68k_netbsd_info);
{$endif m68k}
{$ifdef powerpc}
  RegisterImport(system_powerpc_darwin,timportlibdarwin);
  RegisterExport(system_powerpc_darwin,texportlibdarwin);
  RegisterTarget(system_powerpc_darwin_info);

  RegisterImport(system_powerpc_netbsd,timportlibbsd);
  RegisterExport(system_powerpc_netbsd,texportlibbsd);
  RegisterTarget(system_powerpc_netbsd_info);
{$endif powerpc}
{$ifdef powerpc64}
  RegisterImport(system_powerpc64_darwin,timportlibdarwin);
  RegisterExport(system_powerpc64_darwin,texportlibdarwin);
  RegisterTarget(system_powerpc64_darwin_info);
{$endif powerpc64}
{$ifdef arm}
  RegisterImport(system_arm_darwin,timportlibdarwin);
  RegisterExport(system_arm_darwin,texportlibdarwin);
  RegisterTarget(system_arm_darwin_info);

  RegisterImport(system_arm_netbsd,timportlibbsd);
  RegisterExport(system_arm_netbsd,texportlibbsd);
  RegisterTarget(system_arm_netbsd_info);
{$endif arm}
{$ifdef aarch64}
  RegisterImport(system_aarch64_darwin,timportlibdarwin);
  RegisterExport(system_aarch64_darwin,texportlibdarwin);
  RegisterTarget(system_aarch64_darwin_info);
{$endif aarch64}

  RegisterRes(res_elf_info,TWinLikeResourceFile);
  RegisterRes(res_macho_info,TWinLikeResourceFile);
end.
