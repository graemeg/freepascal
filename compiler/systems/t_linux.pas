{
    Copyright (c) 1998-2008 by Peter Vreman

    This unit implements support import,export,link routines
    for the (i386) Linux target

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
unit t_linux;

{$i fpcdefs.inc}

interface

  uses
    aasmdata,
    symsym,
    import,export,expunix,link;

  type
    timportliblinux=class(timportlib)
      procedure generatelib;override;
    end;

    texportliblinux=class(texportlibunix)
      procedure setfininame(list: TAsmList; const s: string); override;
    end;

    TLibcType=(libc5,glibc2,glibc21,uclibc);

    tlinkerlinux=class(texternallinker)
    private
      libctype: TLibcType;
      prtobj  : string[80];
      reorder : boolean;
      linklibc: boolean;
      Function  WriteResponseFile(isdll:boolean) : Boolean;
      function postprocessexecutable(const fn: string; isdll: boolean): boolean;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      procedure InitSysInitUnitName;override;
      function  MakeExecutable:boolean;override;
      function  MakeSharedLibrary:boolean;override;
      procedure LoadPredefinedLibraryOrder; override;
    end;

    TInternalLinkerLinux=class(TInternalLinker)
    private
      libctype: TLibcType;
      reorder: boolean;
      linklibc: boolean;
      prtobj: string[20];
      dynlinker: string[100];
    public
      constructor Create;override;
      procedure DefaultLinkScript;override;
      procedure InitSysInitUnitName;override;
    end;

implementation

  uses
    SysUtils,
    cutils,cfileutl,cclasses,
    verbose,systems,globtype,globals,
    cscript,
    fmodule,
    aasmbase,aasmtai,aasmcpu,cpubase,
    cgbase,ogbase,
    comprsrc,
    ogelf,owar,
    rescmn, i_linux
    ;

{*****************************************************************************
                               TIMPORTLIBLINUX
*****************************************************************************}

    procedure timportliblinux.generatelib;
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
                               TEXPORTLIBLINUX
*****************************************************************************}

    procedure texportliblinux.setfininame(list: TAsmList; const s: string);
      begin
        { the problem with not having a .fini section is that a finalization
          routine in regular code can get "smart" linked away -> reference it
          just like the debug info }
        new_section(list,sec_fpc,'links',0);
        list.concat(Tai_const.Createname(s,0));
        inherited setfininame(list,s);
      end;

{*****************************************************************************
                                  TLINKERLINUX
*****************************************************************************}

procedure SetupLibrarySearchPath;
begin
  if not Dontlinkstdlibpath Then
    begin
{$ifdef x86_64}
      { some linuxes might not have the lib64 variants (Arch, LFS }
      { don't use PathExists checks, as we need to take sysroots and
        cross-compiling into account }
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/X11R6/lib',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/X11R6/lib64',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib64',true);
      { /lib64 should be the really first, so add it before everything else }
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib64',true);
{$else}
{$ifdef powerpc64}
      if target_info.abi<>abi_powerpc_elfv2 then
        LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib64;=/usr/lib64;=/usr/X11R6/lib64',true)
      else
        LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib64;=/usr/lib/powerpc64le-linux-gnu;=/usr/X11R6/powerpc64le-linux-gnu',true);
{$else powerpc64}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib;=/usr/lib;=/usr/X11R6/lib',true);
{$endif powerpc64}
{$endif x86_64}

{$ifdef arm}
  { some newer Debian have the crt*.o files at uncommon locations,
    for other arm flavours, this cannot hurt }
    if target_info.abi=abi_eabihf then
      begin
        LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/arm-linux-gnueabihf',true);
        LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/arm-linux-gnueabihf',true);
      end;
    if target_info.abi=abi_eabi then
      begin
        LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/arm-linux-gnueabi',true);
        LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/arm-linux-gnueabi',true);
      end;
{$endif arm}
{$ifdef x86_64}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/x86_64-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/x86_64-linux-gnu',true);
{$endif x86_64}
{$ifdef i386}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/i386-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/i386-linux-gnu',true);
{$endif i386}
{$ifdef aarch64}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib64',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/aarch64-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/aarch64-linux-gnu',true);
{$endif aarch64}
{$ifdef powerpc}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/powerpc-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/powerpc-linux-gnu',true);
{$endif powerpc}
{$ifdef m68k}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/m68k-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/m68k-linux-gnu',true);
{$endif m68k}
{$ifdef mipsel}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/mipsel-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/mipsel-linux-gnu',true);
{$endif mipsel}
{$ifdef mips}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/mips-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/mips-linux-gnu',true);
{$endif mips}
{$ifdef sparc64}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/sparc64-linux-gnu',true);
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib/sparc64-linux-gnu',true);
{$endif sparc64}
{$ifdef riscv32}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/riscv32-linux-gnu',true);
{$endif riscv32}
{$ifdef riscv64}
      LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib/riscv64-linux-gnu',true);
{$endif riscv64}
    end;
end;

{$ifdef m68k}
  const defdynlinker='/lib/ld.so.1';
{$endif m68k}

{$ifdef i386}
  const defdynlinker='/lib/ld-linux.so.2';
{$endif}

{$ifdef x86_64}
  const defdynlinker='/lib64/ld-linux-x86-64.so.2';
{$endif x86_64}

{$ifdef sparc}
  const defdynlinker='/lib/ld-linux.so.2';
{$endif sparc}

{$ifdef powerpc}
  const defdynlinker='/lib/ld.so.1';
{$endif powerpc}

{$ifdef powerpc64}
  const defdynlinkerv1='/lib64/ld64.so.1';
  const defdynlinkerv2='/lib64/ld64.so.2';
  var defdynlinker: string;
{$endif powerpc64}

{$ifdef arm}
{$ifdef FPC_ARMHF}
  const defdynlinker='/lib/ld-linux-armhf.so.3';
{$else FPC_ARMHF}
{$ifdef FPC_ARMEL}
  const defdynlinker='/lib/ld-linux.so.3';
{$else FPC_ARMEL}
  const defdynlinker='/lib/ld-linux.so.2';
{$endif FPC_ARMEL}
{$endif FPC_ARMHF}
{$endif arm}

{$ifdef aarch64}
const defdynlinker='/lib/ld-linux-aarch64.so.1';
{$endif aarch64}

{$ifdef mips}
  const defdynlinker='/lib/ld.so.1';
{$endif mips}

{$ifdef sparc64}
  const defdynlinker='/lib64/ld-linux.so.2';
{$endif sparc64}

{$ifdef riscv32}
  const defdynlinker='/lib32/ld.so.1';
{$endif riscv32}

{$ifdef riscv64}
  const defdynlinker='/lib/ld-linux-riscv64-lp64d.so.1';
{$endif riscv64}

{$ifdef xtensa}
  const defdynlinker='/lib/ld.so.1';
{$endif xtensa}


procedure SetupDynlinker(out DynamicLinker:string;out libctype:TLibcType);
begin
{$ifdef powerpc64}
  if defdynlinker='' then
    if target_info.abi=abi_powerpc_sysv then
      defdynlinker:=defdynlinkerv1
    else
      defdynlinker:=defdynlinkerv2;
{$endif powerpc64}
  {
    Search order:
    glibc 2.1+
    uclibc
    glibc 2.0
    If none is found (e.g. when cross compiling) glibc21 is assumed
  }
  if fileexists(sysrootpath+defdynlinker,false) then
    begin
      DynamicLinker:=defdynlinker;
{$ifdef i386}
      libctype:=glibc21;
{$else i386}
      libctype:=glibc2;
{$endif i386}
    end
  else if fileexists(sysrootpath+'/lib/ld-uClibc.so.0',false) then
    begin
      DynamicLinker:='/lib/ld-uClibc.so.0';
      libctype:=uclibc;
    end
{$ifdef i386}
  else if FileExists(sysrootpath+'/lib/ld-linux.so.1',false) then
    begin
      DynamicLinker:='/lib/ld-linux.so.1';
      libctype:=glibc2;
    end
{$endif i386}
  else
    begin
      { when no dyn. linker is found, we are probably
        cross compiling, so use the default dyn. linker }
      DynamicLinker:=defdynlinker;
      {
        the default c startup script is gcrt0.as on all platforms
        except i386
      }
{$ifdef i386}
      libctype:=glibc21;
{$else i386}
      libctype:=glibc2;
{$endif i386}
    end;
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

Constructor TLinkerLinux.Create;
begin
  Inherited Create;
  SetupLibrarySearchPath;
end;

procedure TLinkerLinux.SetDefaultInfo;
{
  This will also detect which libc version will be used
}

const
{$ifdef i386}      platform_select='-b elf32-i386 -m elf_i386';{$endif}
{$ifdef x86_64}    platform_select='-b elf64-x86-64 -m elf_x86_64';{$endif}
{$ifdef powerpc}   platform_select='-b elf32-powerpc -m elf32ppclinux';{$endif}
{$ifdef POWERPC64} platform_select='';{$endif}
{$ifdef sparc}     platform_select='-b elf32-sparc -m elf32_sparc';{$endif}
{$ifdef sparc64}   platform_select='-b elf64-sparc -m elf64_sparc';{$endif}
{$ifdef arm}       platform_select='';{$endif} {unknown :( }
{$ifdef aarch64}   platform_select='';{$endif} {unknown :( }
{$ifdef m68k}      platform_select='';{$endif} {unknown :( }
{$ifdef mips}
  {$ifdef mipsel}
                   platform_select='-EL';
  {$else}
                   platform_select='-EB';
  {$endif}
{$endif}
{$ifdef riscv32}   platform_select='-m elf32lriscv';{$endif}
{$ifdef riscv64}   platform_select='-m elf64lriscv';{$endif}
{$ifdef xtensa}    platform_select='';{$endif}

var
  platformopt: string;
begin
  platformopt:='';
{$ifdef powerpc64}
  if (target_info.abi=abi_powerpc_elfv2) and
     (target_info.endian=endian_little) then
    platformopt:=' -b elf64-powerpcle -m elf64lppc'
  else
    platformopt:=' -b elf64-powerpc -m elf64ppc';
{$endif powerpc64}
{$ifdef arm}
  platformopt:=' -z noexecstack';
{$endif arm}

  with Info do
   begin
     ExeCmd[1]:='ld '+platform_select+platformopt+' $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO -L. -o $EXE';
     DllCmd[1]:='ld '+platform_select+platformopt+' $OPT $INIT $FINI $SONAME $MAP $LTO -shared $GCSECTIONS -L. -o $EXE';
     { when we want to cross-link we need to override default library paths;
       when targeting binutils 2.19 or later, we use the "INSERT" command to
       augment the default linkerscript, which also requires -T (normally that
       option means "completely replace the default linkerscript) }
     if not(cs_link_pre_binutils_2_19 in current_settings.globalswitches) or
       (length(sysrootpath)>0) then
       begin
         ExeCmd[1]:=ExeCmd[1]+' -T';
         DllCmd[1]:=DllCmd[1]+' -T';
       end;
     ExeCmd[1]:=ExeCmd[1]+' $RES';
     DllCmd[1]:=DllCmd[1]+' $RES';
     DllCmd[2]:='strip --strip-unneeded $EXE';
     ExtDbgCmd[1]:='objcopy --only-keep-debug $EXE $DBG';
     ExtDbgCmd[2]:='objcopy "--add-gnu-debuglink=$DBGX" $EXE';
     ExtDbgCmd[3]:='strip --strip-unneeded $EXE';

     SetupDynlinker(DynamicLinker,libctype);
   end;
end;


procedure TLinkerLinux.LoadPredefinedLibraryOrder;
// put your linkorder/linkalias overrides here.
// Note: assumes only called when reordering/aliasing is used.
Begin
   if not (cs_link_no_default_lib_order in  current_settings.globalswitches) Then
        Begin
          LinkLibraryOrder.add('gcc','',15);
          LinkLibraryOrder.add('c','',100);
          LinkLibraryOrder.add('gmon','',120);
          LinkLibraryOrder.add('dl','',140);
          LinkLibraryOrder.add('pthread','',160);
         end;
End;

type
  tlibcnames=array [TLibcType] of string[8];

const                     { libc5    glibc2   glibc21   uclibc }
  cprtnames: tlibcnames = ('cprt0', 'cprt0', 'cprt21', 'ucprt0');
  csinames: tlibcnames  = ('si_c',  'si_c',  'si_c21', 'si_uc');
  gprtnames: tlibcnames = ('gprt0', 'gprt0', 'gprt21', 'ugprt0');
  gsinames: tlibcnames  = ('si_g',  'si_g',  'si_c21g','si_ucg');

  defprtnames: array[boolean] of string[8] = ('prt0',  'dllprt0');
  defsinames: array[boolean] of string[8]  = ('si_prc','si_dll');

{ uclibc and glibc21 are not available on x86_64! si_g is also absent. }
Procedure TLinkerLinux.InitSysInitUnitName;
begin
  linklibc:=ModulesLinkToLibc;
  reorder:=linklibc and ReOrderEntries;
  sysinitunit:=defsinames[current_module.islibrary];
  prtobj:=defprtnames[current_module.islibrary];

  if current_module.islibrary then
    exit;
  if cs_profile in current_settings.moduleswitches then
    begin
      prtobj:=gprtnames[libctype];
      sysinitunit:=gsinames[libctype];
      linklibc:=true;
    end
  else if linklibc then
    begin
      prtobj:=cprtnames[libctype];
      sysinitunit:=csinames[libctype];
    end;
end;

Function TLinkerLinux.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres      : TLinkRes;
  i            : longint;
  HPath        : TCmdStrListItem;
  s,s1,s2      : TCmdStr;
  found1,
  found2       : boolean;
  linksToSharedLibFiles, libraryadded: boolean;
begin
  result:=False;
{ set special options for some targets }
  if cs_profile in current_settings.moduleswitches then
   begin
     if not(libctype in [glibc2,glibc21]) then
       AddSharedLibrary('gmon');
     AddSharedLibrary('c');
   end;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);
  with linkres do
    begin
      { Write path to search libraries }
      HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
      while assigned(HPath) do
       begin
         Add('SEARCH_DIR("'+HPath.Str+'")');
         HPath:=TCmdStrListItem(HPath.Next);
       end;
      HPath:=TCmdStrListItem(LibrarySearchPath.First);
      while assigned(HPath) do
       begin
         Add('SEARCH_DIR("'+HPath.Str+'")');
         HPath:=TCmdStrListItem(HPath.Next);
       end;

      { force local symbol resolution (i.e., inside the shared }
      { library itself) for all non-exorted symbols, otherwise }
      { several RTL symbols of FPC-compiled shared libraries   }
      { will be bound to those of a single shared library or   }
      { to the main program                                    }
      if (isdll) then
        begin
          add('VERSION');
          add('{');
          add('  {');
          if not texportlibunix(exportlib).exportedsymnames.empty then
            begin
              add('    global:');
              repeat
                add('      '+texportlibunix(exportlib).exportedsymnames.getfirst+';');
              until texportlibunix(exportlib).exportedsymnames.empty;
            end;
          add('    local:');
          add('      *;');
          add('  };');
          add('}');
        end;

      StartSection('INPUT(');
      { add objectfiles, start with prt0 always }
      if not (target_info.system in systems_internal_sysinit) and (prtobj<>'') then
       AddFileName(maybequoted(FindObjectFile(prtobj,'',false)));
      { try to add crti and crtbegin if linking to C }
      if linklibc and (libctype<>uclibc) then
       begin
         { crti.o must come first }
         if librarysearchpath.FindFile('crti.o',false,s) then
           AddFileName(s)
         else
           Message1(exec_w_init_file_not_found,'crti.o');

         { then the crtbegin* }
         if cs_create_pic in current_settings.moduleswitches then
           begin
             if librarysearchpath.FindFile('crtbeginS.o',false,s) then
               AddFileName(s)
             else
               Message1(exec_w_init_file_not_found,'crtbeginS.o');
           end
         else
           if (cs_link_staticflag in current_settings.globalswitches) then
             begin
               if librarysearchpath.FindFile('crtbeginT.o',false,s) then
                 AddFileName(s)
               else
                 Message1(exec_w_init_file_not_found,'crtbeginT.o');
             end
           else if librarysearchpath.FindFile('crtbegin.o',false,s) then
             AddFileName(s)
           else
             Message1(exec_w_init_file_not_found,'crtbegin.o');
       end;
      { main objectfiles }
      while not ObjectFiles.Empty do
       begin
         s:=ObjectFiles.GetFirst;
         if s<>'' then
          AddFileName(maybequoted(s));
       end;
      EndSection(')');

      { Write staticlibraries }
      if not StaticLibFiles.Empty then
       begin
         Add('GROUP(');
         While not StaticLibFiles.Empty do
          begin
            S:=StaticLibFiles.GetFirst;
            AddFileName(maybequoted(s))
          end;
         Add(')');
       end;

      // we must reorder here because the result could empty sharedlibfiles
      if reorder Then
        ExpandAndApplyOrder(SharedLibFiles);
      // after this point addition of shared libs not allowed.

      { Write sharedlibraries like -l<lib>, also add the needed dynamic linker
        here to be sure that it gets linked this is needed for glibc2 systems (PFV) }
      if isdll and not linklibc then
       begin
         Add('INPUT(');
         Add(sysrootpath+info.DynamicLinker);
         Add(')');
       end;
      linksToSharedLibFiles := not SharedLibFiles.Empty;

      if not SharedLibFiles.Empty then
       begin

         if (SharedLibFiles.Count<>1) or
            (TCmdStrListItem(SharedLibFiles.First).Str<>'c') or
            reorder then
           begin
             libraryadded:=false;
             Add('INPUT(');
             While not SharedLibFiles.Empty do
               begin
                 S:=SharedLibFiles.GetFirst;
                 if (s<>'c') or reorder then
                  begin
                    i:=Pos(target_info.sharedlibext,S);
                    if i>0 then
                     Delete(S,i,255);
                    Add('-l'+s);
                    libraryadded:=true;
                  end
                 else
                   linklibc:=true;
               end;
             { link explicitly against the dyn. linker in case we are using section threadvars and
               if we link against any other library. We need __tls_get_addr from the dyn. linker in this case.
               This does not hurt as in case we use a dyn. library we depend on the dyn. linker anyways.

               All this does not apply if we link anyways against libc }
             if libraryadded and not(linklibc) and not(isdll) and (tf_section_threadvars in target_info.flags) then
               Add('-l:'+ExtractFileName(defdynlinker));
             Add(')');
           end
         else
           linklibc:=true;
         if (cs_link_staticflag in current_settings.globalswitches) or
            (linklibc and not reorder) then
           begin
             Add('GROUP(');
             { when we have -static for the linker the we also need libgcc }
             if (cs_link_staticflag in current_settings.globalswitches) then
               begin
                 Add('-lgcc');
                 if librarysearchpath.FindFile('libgcc_eh.a',false,s1) then
                   Add('-lgcc_eh');
               end;
             { be sure that libc is the last lib }
             if linklibc and not reorder then
               Add('-lc');
             Add(')');
           end;
       end;

      { objects which must be at the end }
      if linklibc and (libctype<>uclibc) then
       begin
         if cs_create_pic in current_settings.moduleswitches then
           begin
             found1:=librarysearchpath.FindFile('crtendS.o',false,s1);
             if not(found1) then
               Message1(exec_w_init_file_not_found,'crtendS.o');
           end
         else
           begin
             found1:=librarysearchpath.FindFile('crtend.o',false,s1);
             if not(found1) then
               Message1(exec_w_init_file_not_found,'crtend.o');
           end;

         found2:=librarysearchpath.FindFile('crtn.o',false,s2);
         if not(found2) then
           Message1(exec_w_init_file_not_found,'crtn.o');
         if found1 or found2 then
          begin
            Add('INPUT(');
            if found1 then
             AddFileName(s1);
            if found2 then
             AddFileName(s2);
            Add(')');
          end;
       end;

      { Entry point. Only needed for executables, as for shared lubraries we use
        the -init command line option instead

       The "ENTRY" linkerscript command does not have any effect when augmenting
       a linker script, so use the command line parameter instead }
      if (not isdll) then
        if (linksToSharedLibFiles and not linklibc) then
          info.ExeCmd[1]:=info.ExeCmd[1]+' -e _dynamic_start'
        else
          info.ExeCmd[1]:=info.ExeCmd[1]+' -e _start';

      add('SECTIONS');
      add('{');
      if not(cs_link_pre_binutils_2_19 in current_settings.globalswitches) then
        { we can't use ".data", as that would hide the .data from the
          original linker script in combination with the INSERT at the end }
        add('  .fpcdata           :')
      else
        add('  .data           :');
      add('  {');
      add('    KEEP (*(.fpc .fpc.n_version .fpc.n_links))');
      add('  }');
      add('  .threadvar : { *(.threadvar .threadvar.* .gnu.linkonce.tv.*) }');
      add('}');
      { this "INSERT" means "merge into the original linker script, even if
        -T is used" }
      if not(cs_link_pre_binutils_2_19 in current_settings.globalswitches) then
        add('INSERT AFTER .data;');
      { Write and Close response }
      writetodisk;
      Free;
    end;

  WriteResponseFile:=True;
end;


function TLinkerLinux.MakeExecutable:boolean;
var
  i : longint;
  binstr,
  cmdstr,
  mapstr,
  ltostr  : TCmdStr;
  success : boolean;
  DynLinkStr : string;
  GCSectionsStr,
  StaticStr,
  StripStr   : string[40];
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Create some replacements }
  StaticStr:='';
  StripStr:='';
  GCSectionsStr:='';
  DynLinkStr:='';
  mapstr:='';
  ltostr:='';
  if (cs_link_staticflag in current_settings.globalswitches) then
   StaticStr:='-static';
  if (cs_link_strip in current_settings.globalswitches) and
     not(cs_link_separate_dbg_file in current_settings.globalswitches) then
   StripStr:='-s';
  if (cs_link_map in current_settings.globalswitches) then
   mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
  if (cs_link_smart in current_settings.globalswitches) and
     create_smartlink_sections then
   GCSectionsStr:='--gc-sections';
  If (cs_profile in current_settings.moduleswitches) or
     ((Info.DynamicLinker<>'') and (not SharedLibFiles.Empty)) then
   begin
     DynLinkStr:='--dynamic-linker='+Info.DynamicLinker;
     if cshared then
       DynLinkStr:=DynLinkStr+' --shared ';
     if rlinkpath<>'' then
       DynLinkStr:=DynLinkStr+' --rpath-link '+rlinkpath;
   End;

  { add custom LTO library if using custom clang }
  if (cs_lto in current_settings.moduleswitches) and
     not(cs_link_on_target in current_settings.globalswitches) and
     (utilsdirectory<>'') and
     FileExists(utilsdirectory+'/../lib/LLVMgold.so',true) then
    begin
      ltostr:='-plugin '+maybequoted(utilsdirectory+'/../lib/LLVMgold.so ');
    end;

{ Write used files and libraries }
  WriteResponseFile(false);

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  Replace(cmdstr,'$MAP',mapstr);
  Replace(cmdstr,'$LTO',ltostr);

  { create dynamic symbol table? }
  if HasExports then
    cmdstr:=cmdstr+' -E';

  { create eh_frame_hdr section? }
  if tf_use_psabieh in target_info.flags then
    cmdstr:=cmdstr+ ' --eh-frame-hdr';

  if cs_large in current_settings.globalswitches then
    cmdstr:=cmdstr+' --no-relax';

  success:=DoExec(FindUtil(utilsprefix+BinStr),CmdStr,true,false);

  { Create external .dbg file with debuginfo }
  if success and (cs_link_separate_dbg_file in current_settings.globalswitches) then
    begin
      for i:=1 to 3 do
        begin
          SplitBinCmd(Info.ExtDbgCmd[i],binstr,cmdstr);
          Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
          Replace(cmdstr,'$DBGFN',maybequoted(extractfilename(current_module.dbgfilename)));
          Replace(cmdstr,'$DBGX',current_module.dbgfilename);
          Replace(cmdstr,'$DBG',maybequoted(current_module.dbgfilename));
          success:=DoExec(FindUtil(utilsprefix+BinStr),CmdStr,true,false);
          if not success then
            break;
        end;
    end;

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

  { Post process,
    as it only writes sections sizes so far, do this only if V_Info is set }
  if success and CheckVerbosity(V_Info) and not(cs_link_nolink in current_settings.globalswitches) then
    success:=PostProcessExecutable(current_module.exefilename,false);

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


Function TLinkerLinux.MakeSharedLibrary:boolean;
var
  InitStr,
  FiniStr,
  GCSectionsStr,
  SoNameStr : string[80];
  binstr,
  cmdstr,
  mapstr,
  ltostr : TCmdStr;
  success : boolean;
begin
  MakeSharedLibrary:=false;
  mapstr:='';
  ltostr:='';
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.sharedlibfilename);
  if (cs_link_smart in current_settings.globalswitches) and
     create_smartlink_sections then
   GCSectionsStr:='--gc-sections'
  else
    GCSectionsStr:='';

{ Write used files and libraries }
  WriteResponseFile(true);

 { Create some replacements }
 { note: linux does not use exportlib.initname/fininame due to the custom startup code }
  InitStr:='-init FPC_SHARED_LIB_START';
  FiniStr:='-fini FPC_LIB_EXIT';
  SoNameStr:='-soname '+ExtractFileName(current_module.sharedlibfilename);
  if (cs_link_map in current_settings.globalswitches) then
     mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.sharedlibfilename,'.map'));

  { add custom LTO library if using custom clang }
  if (cs_lto in current_settings.moduleswitches) and
     not(cs_link_on_target in current_settings.globalswitches) and
     (utilsdirectory<>'') and
     FileExists(utilsdirectory+'/../lib/LLVMgold.so',true) then
    begin
      ltostr:='-plugin '+maybequoted(utilsdirectory+'/../lib/LLVMgold.so ');
    end;

{ Call linker }
  SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$INIT',InitStr);
  Replace(cmdstr,'$FINI',FiniStr);
  Replace(cmdstr,'$SONAME',SoNameStr);
  Replace(cmdstr,'$MAP',mapstr);
  Replace(cmdstr,'$LTO',ltostr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,false);

{ Strip the library ? }
  if success and (cs_link_strip in current_settings.globalswitches) then
   begin
     { only remove non global symbols and debugging info for a library }
     Info.DllCmd[2]:='strip --discard-all --strip-debug $EXE';
     SplitBinCmd(Info.DllCmd[2],binstr,cmdstr);
     Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
     success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,false);
   end;

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

  MakeSharedLibrary:=success;   { otherwise a recursive call to link method }
end;


function TLinkerLinux.postprocessexecutable(const fn : string;isdll:boolean):boolean;
  begin
    Result:=PostProcessELFExecutable(fn,isdll);
  end;

{*****************************************************************************
                              TINTERNALLINKERLINUX
*****************************************************************************}

constructor TInternalLinkerLinux.Create;
begin
  inherited Create;
  SetupLibrarySearchPath;
  SetupDynlinker(dynlinker,libctype);

  CArObjectReader:=TArObjectReader;
  CExeOutput:=ElfExeOutputClass;
  CObjInput:=TElfObjInput;

end;

procedure TInternalLinkerLinux.InitSysInitUnitName;
begin
  linklibc:=ModulesLinkToLibc;
  reorder:=linklibc and ReOrderEntries;
  sysinitunit:=defsinames[current_module.islibrary];
  prtobj:=defprtnames[current_module.islibrary];

  if cs_profile in current_settings.moduleswitches then
    begin
      prtobj:=gprtnames[libctype];
      sysinitunit:=gsinames[libctype];
      linklibc:=true;
    end
  else if linklibc then
    begin
      prtobj:=cprtnames[libctype];
      sysinitunit:=csinames[libctype];
    end;
end;


const
  relsec_prefix:array[boolean] of TCmdStr = ('rel','rela');

procedure TInternalLinkerLinux.DefaultLinkScript;
var
  s,s1,s2,relprefix:TCmdStr;
  found1,found2:boolean;
  linkToSharedLibs:boolean;

  procedure AddLibraryStatement(const s:TCmdStr);
    var
      i:longint;
      s1,s2:TCmdStr;
    begin
      i:=pos(target_info.sharedClibext+'.',s);
      if (i>0) then
        s1:=target_info.sharedClibprefix+S
      else
        s1:=target_info.sharedClibprefix+S+target_info.sharedClibext;
      { TODO: to be compatible with ld search algorithm, each found file
        must be tested for target compatibility, incompatible ones should be skipped. }
      { TODO: shall we search library without suffix if one with suffix is not found? }
      if (not(cs_link_staticflag in current_settings.globalswitches)) and
         FindLibraryFile(s1,'','',s2) then
        LinkScript.Concat('READSTATICLIBRARY '+maybequoted(s2))
      { TODO: static libraries never have numeric suffix in their names }
      else if FindLibraryFile(s,target_info.staticClibprefix,target_info.staticClibext,s2) then
        LinkScript.Concat('READSTATICLIBRARY '+maybequoted(s2))
      else
        Comment(V_Error,'Import library not found for '+S);
    end;

begin
  if cs_profile in current_settings.moduleswitches then
    begin
      if not(libctype in [glibc2,glibc21]) then
        AddSharedLibrary('gmon');
      AddSharedLibrary('c');
    end;

  TElfExeOutput(exeoutput).interpreter:=stringdup(dynlinker);

  { add objectfiles, start with prt0 always }
  if not (target_info.system in systems_internal_sysinit) and (prtobj<>'') then
    LinkScript.Concat('READOBJECT '+ maybequoted(FindObjectFile(prtobj,'',false)));

  { try to add crti and crtbegin if linking to C }
  if linklibc and (libctype<>uclibc) then
    begin
      { crti.o must come first }
      if librarysearchpath.FindFile('crti.o',false,s) then
        LinkScript.Concat('READOBJECT '+maybequoted(s));
      { then the crtbegin* }
      if cs_create_pic in current_settings.moduleswitches then
        begin
          if librarysearchpath.FindFile('crtbeginS.o',false,s) then
            LinkScript.Concat('READOBJECT '+maybequoted(s));
        end
      else
        if (cs_link_staticflag in current_settings.globalswitches) and
          librarysearchpath.FindFile('crtbeginT.o',false,s) then
          LinkScript.Concat('READOBJECT '+maybequoted(s))
        else if librarysearchpath.FindFile('crtbegin.o',false,s) then
          LinkScript.Concat('READOBJECT '+maybequoted(s));
    end;

  ScriptAddSourceStatements(false);
  { we must reorder here because the result could empty sharedlibfiles }
  if reorder then
    ExpandAndApplyOrder(SharedLibFiles);

  { See tw9089*.pp: if more than one pure-Pascal shared libs are loaded,
    and none have rtld in their DT_NEEDED, then rtld cannot finalize correctly.  }
  if IsSharedLibrary then
    LinkScript.Concat('READSTATICLIBRARY '+maybequoted(sysrootpath+dynlinker));

  linkToSharedLibs:=(not SharedLibFiles.Empty);

  { Symbols declared as "external 'libx.so'" are added to ImportLibraryList, library
    prefix/extension *not* stripped. TImportLibLinux copies these to SharedLibFiles,
    stripping prefixes and extensions.
    However extension won't be stripped if library is specified with numeric suffix
    (like "libpango-1.0.so.0")
    Libraries specified with $LINKLIB directive are directly added to SharedLibFiles
    and won't be present in ImportLibraryList. }
  while not SharedLibFiles.Empty do
    begin
      S:=SharedLibFiles.GetFirst;
      if (S<>'c') or reorder then
        AddLibraryStatement(S);
    end;

  if (cs_link_staticflag in current_settings.globalswitches) or
    (linklibc and not reorder) then
    begin
      LinkScript.Concat('GROUP');
      if (cs_link_staticflag in current_settings.globalswitches) then
        begin
          AddLibraryStatement('gcc');
          AddLibraryStatement('gcc_eh');
        end;
      if linklibc and not reorder then
        AddLibraryStatement('c');
      LinkScript.Concat('ENDGROUP');
    end;

  { objects which must be at the end }
  if linklibc and (libctype<>uclibc) then
    begin
      if cs_create_pic in current_settings.moduleswitches then
        found1:=librarysearchpath.FindFile('crtendS.o',false,s1)
      else
        found1:=librarysearchpath.FindFile('crtend.o',false,s1);
      found2:=librarysearchpath.FindFile('crtn.o',false,s2);
      if found1 then
        LinkScript.Concat('READOBJECT '+maybequoted(s1));
      if found2 then
        LinkScript.Concat('READOBJECT '+maybequoted(s2));
    end;

   if (not IsSharedLibrary) then
     if (linkToSharedLibs and not linklibc) then
       LinkScript.Concat('ENTRYNAME _dynamic_start')
     else
       LinkScript.Concat('ENTRYNAME _start')
   else
     LinkScript.Concat('ISSHAREDLIBRARY');

  relprefix:=relsec_prefix[ElfTarget.relocs_use_addend];

  with LinkScript do
    begin
      Concat('HEADER');
      Concat('EXESECTION .interp');
      Concat('  OBJSECTION .interp');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .note.ABI-tag');
      Concat('  OBJSECTION .note.ABI-tag');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .note.gnu.build-id');
      Concat('  OBJSECTION .note.gnu.build-id');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .hash');
      Concat('  OBJSECTION .hash');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .dynsym');
      Concat('  OBJSECTION .dynsym');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .dynstr');
      Concat('  OBJSECTION .dynstr');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .gnu.version');
      Concat('  OBJSECTION .gnu.version');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .gnu.version_d');
      Concat('  OBJSECTION .gnu.version_d');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .gnu.version_r');
      Concat('  OBJSECTION .gnu.version_r');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .'+relprefix+'.dyn');
      Concat('  OBJSECTION .'+relprefix+'.dyn');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .'+relprefix+'.plt');
      Concat('  OBJSECTION .'+relprefix+'.plt');
      Concat('  PROVIDE __'+relprefix+'_iplt_start');
      Concat('  OBJSECTION .'+relprefix+'.iplt');
      Concat('  PROVIDE __'+relprefix+'_iplt_end');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .init');
      Concat('  OBJSECTION .init');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .plt');
      Concat('  OBJSECTION .plt');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .text');
      Concat('  OBJSECTION .text*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .fini');
      Concat('  OBJSECTION .fini');
      Concat('  PROVIDE __etext');
      Concat('  PROVIDE _etext');
      Concat('  PROVIDE etext');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .rodata');
      Concat('  OBJSECTION .rodata*');
      Concat('ENDEXESECTION');
{$ifdef arm}
      Concat('EXESECTION .ARM.extab');
      Concat('  OBJSECTION .ARM.extab*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .ARM.exidx');
      Concat('  SYMBOL __exidx_start');
      Concat('  OBJSECTION .ARM.exidx*');
      Concat('  SYMBOL __exidx_end');
      Concat('ENDEXESECTION');
{$endif}
      Concat('EXESECTION .eh_frame');
      Concat('  OBJSECTION .eh_frame');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .gcc_except_table');
      Concat('  OBJSECTION .gcc_except_table');
      Concat('  OBJSECTION .gcc_except_table.*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .tdata');
      Concat('  OBJSECTION .tdata');
      Concat('  OBJSECTION .tdata.*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .tbss');
      Concat('  OBJSECTION .tbss');
      Concat('  OBJSECTION .tbss.*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .preinit_array');
      Concat('  PROVIDE __preinit_array_start');
      Concat('  OBJSECTION .preinit_array');
      Concat('  PROVIDE __preinit_array_end');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .init_array');
      Concat('  PROVIDE __init_array_start');
      { why the hell .ctors are both here and exesection .ctors below?? }
      //  KEEP ( *(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      Concat('  OBJSECTION .init_array');
      //  KEEP ( *(EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
      Concat('PROVIDE __init_array_end');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .fini_array');
      Concat('  PROVIDE __fini_array_start');
      //  KEEP ( *(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
      Concat('  OBJSECTION .fini_array');
      //  KEEP ( *(EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
      Concat('  PROVIDE __fini_array_end');
      Concat('ENDEXESECTION');

      Concat('EXESECTION .ctors');
      Concat('  OBJSECTION .ctors*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .dtors');
      Concat('  OBJSECTION .dtors*');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .jcr');
      Concat('  OBJSECTION .jcr');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .dynamic');
      Concat('  OBJSECTION .dynamic');
      Concat('ENDEXESECTION');
{$ifndef mips}
      Concat('EXESECTION .got');
{$ifdef arm}
      Concat('  OBJSECTION .got.plt');
{$endif arm}
      Concat('  OBJSECTION .got');
      Concat('ENDEXESECTION');
{$endif mips}
{$ifndef arm}
      Concat('EXESECTION .got.plt');
      Concat('  OBJSECTION .got.plt');
      Concat('ENDEXESECTION');
{$endif arm}
      Concat('EXESECTION .data');
      Concat('  OBJSECTION .data*');
      Concat('  OBJSECTION .fpc*');
      Concat('  OBJSECTION fpc.resources');
      Concat('  PROVIDE _edata');
      Concat('  PROVIDE edata');
      Concat('ENDEXESECTION');
{$ifdef mips}
      Concat('EXESECTION .got');
      Concat('  OBJSECTION .got');
      Concat('ENDEXESECTION');
{$endif mips}
      Concat('EXESECTION .bss');
      Concat('  OBJSECTION .dynbss');
      Concat('  OBJSECTION .bss*');
      Concat('  OBJSECTION fpc.reshandles');
      Concat('  PROVIDE end');
      Concat('  SYMBOL _end');
      Concat('ENDEXESECTION');

      ScriptAddGenericSections('.debug_aranges,.debug_pubnames,.debug_info,'+
         '.debug_abbrev,.debug_line,.debug_frame,.debug_str,.debug_loc,'+
         '.debug_macinfo,.debug_weaknames,.debug_funcnames,.debug_typenames,.debug_varnames,.debug_ranges');
      Concat('EXESECTION .stab');
      Concat('  OBJSECTION .stab');
      Concat('ENDEXESECTION');
      Concat('EXESECTION .stabstr');
      Concat('  OBJSECTION .stabstr');
      Concat('ENDEXESECTION');
    end;
end;

{*****************************************************************************
                                  Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_linux,TLinkerLinux);
  RegisterLinker(ld_int_linux,TInternalLinkerLinux);
{$ifdef i386}
  RegisterImport(system_i386_linux,timportliblinux);
  RegisterExport(system_i386_linux,texportliblinux);
  RegisterTarget(system_i386_linux_info);
{$endif i386}
{$ifdef m68k}
  RegisterImport(system_m68k_linux,timportliblinux);
  RegisterExport(system_m68k_linux,texportliblinux);
  RegisterTarget(system_m68k_linux_info);
{$endif m68k}
{$ifdef powerpc}
  RegisterImport(system_powerpc_linux,timportliblinux);
  RegisterExport(system_powerpc_linux,texportliblinux);
  RegisterTarget(system_powerpc_linux_info);
{$endif powerpc}
{$ifdef powerpc64}
  { default to little endian either when compiling with -dppc64le, or when
    compiling on a little endian ppc64 platform }
 {$if defined(ppc64le) or (defined(cpupowerpc64) and defined(FPC_LITTLE_ENDIAN))}
  system_powerpc64_linux_info.endian:=endian_little;
  system_powerpc64_linux_info.abi:=abi_powerpc_elfv2;
 {$endif}
  RegisterImport(system_powerpc64_linux,timportliblinux);
  RegisterExport(system_powerpc64_linux,texportliblinux);
  RegisterTarget(system_powerpc64_linux_info);
{$endif powerpc64}
{$ifdef x86_64}
  RegisterImport(system_x86_64_linux,timportliblinux);
  RegisterExport(system_x86_64_linux,texportliblinux);
  RegisterTarget(system_x86_64_linux_info);
  RegisterTarget(system_x86_6432_linux_info);
{$endif x86_64}
{$ifdef SPARC}
  RegisterImport(system_SPARC_linux,timportliblinux);
  RegisterExport(system_SPARC_linux,texportliblinux);
  RegisterTarget(system_SPARC_linux_info);
{$endif SPARC}
{$ifdef SPARC64}
  RegisterImport(system_SPARC64_linux,timportliblinux);
  RegisterExport(system_SPARC64_linux,texportliblinux);
  RegisterTarget(system_SPARC64_linux_info);
{$endif SPARC64}
{$ifdef ARM}
  RegisterImport(system_arm_linux,timportliblinux);
  RegisterExport(system_arm_linux,texportliblinux);
  RegisterTarget(system_arm_linux_info);
{$endif ARM}
{$ifdef aarch64}
  RegisterImport(system_aarch64_linux,timportliblinux);
  RegisterExport(system_aarch64_linux,texportliblinux);
  RegisterTarget(system_aarch64_linux_info);
{$endif aarch64}
{$ifdef MIPS}
{$ifdef MIPSEL}
  RegisterImport(system_mipsel_linux,timportliblinux);
  RegisterExport(system_mipsel_linux,texportliblinux);
  RegisterTarget(system_mipsel_linux_info);
{$else MIPS}
  RegisterImport(system_mipseb_linux,timportliblinux);
  RegisterExport(system_mipseb_linux,texportliblinux);
  RegisterTarget(system_mipseb_linux_info);
{$endif MIPSEL}
{$endif MIPS}
{$ifdef riscv32}
  RegisterImport(system_riscv32_linux,timportliblinux);
  RegisterExport(system_riscv32_linux,texportliblinux);
  RegisterTarget(system_riscv32_linux_info);
{$endif riscv32}
{$ifdef riscv64}
  RegisterImport(system_riscv64_linux,timportliblinux);
  RegisterExport(system_riscv64_linux,texportliblinux);
  RegisterTarget(system_riscv64_linux_info);
{$endif riscv64}
{$ifdef xtensa}
  RegisterImport(system_xtensa_linux,timportliblinux);
  RegisterExport(system_xtensa_linux,texportliblinux);
  RegisterTarget(system_xtensa_linux_info);
{$endif xtensa}
  RegisterRes(res_elf_info,TWinLikeResourceFile);
end.

