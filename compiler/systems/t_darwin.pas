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

unit t_darwin;

{$i fpcdefs.inc}

interface

implementation

  uses
    sysutils,
    cutils,cfileutl,cclasses,
    verbose,systems,globtype,globals,
    symconst,cscript,
    fmodule,aasmbase,aasmtai,aasmdata,aasmcpu,cpubase,symsym,symdef,
    import,export,link,comprsrc,rescmn,i_darwin,expunix,
    cgutils,cgbase,cgobj,cpuinfo,ogbase;

  type
    timportlibdarwin=class(timportlib)
      procedure generatelib;override;
    end;

    texportlibdarwin=class(texportlibunix)
      procedure setinitname(list: TAsmList; const s: string); override;
      procedure setfininame(list: TAsmList; const s: string); override;
    end;

    tlinkerdarwin=class(texternallinker)
    private
      LinkSymsFileName   : TCmdStr;
      function WriteFileList: TCmdStr;
      function GetDarwinCrt1ObjName(isdll: boolean): TCmdStr;
      Function GetDarwinPrtobjName(isdll: boolean): TCmdStr;
      function GetLinkArch: TCmdStr;
      function GetLinkVersion: TCmdStr;
      function GetSysroot: TCmdStr;
      function GetLibSearchPath: TCmdStr;
      function GetLibraries: TCmdStr;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      function  MakeExecutable:boolean;override;
      function  MakeSharedLibrary:boolean;override;
      procedure LoadPredefinedLibraryOrder; override;
      procedure InitSysInitUnitName; override;
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
                                  TLINKERBSD
*****************************************************************************}

    constructor tlinkerdarwin.Create;
      begin
        inherited;
        if not Dontlinkstdlibpath Then
          LibrarySearchPath.AddLibraryPath(sysrootpath,'=/usr/lib',true)
      end;


    procedure tlinkerdarwin.SetDefaultInfo;
      begin
        with Info do
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
           ExeCmd[1]:='ld $PRTOBJ $TARGET $OPT $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -L. -o $EXE $ARCH $VERSION $SYSROOT $LIBSEARCHPATH $FILELIST $LIBRARIES';
           if not(cs_gdb_valgrind in current_settings.globalswitches) then
             ExeCmd[1]:=ExeCmd[1]+' -pagezero_size 0x10000';
  {$else ndef cpu64bitaddr}
           ExeCmd[1]:='ld $PRTOBJ $TARGET $OPT $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -L. -o $EXE $ARCH $VERSION $SYSROOT $LIBSEARCHPATH $FILELIST $LIBRARIES';
  {$endif ndef cpu64bitaddr}
           if (apptype<>app_bundle) then
             DllCmd[1]:='ld $PRTOBJ $TARGET $OPT $GCSECTIONS $MAP $LTO $ORDERSYMS -dynamic -dylib -L. -o $EXE $ARCH $VERSION $SYSROOT $LIBSEARCHPATH $FILELIST $LIBRARIES'
           else
             DllCmd[1]:='ld $PRTOBJ $TARGET $OPT $GCSECTIONS $MAP $LTO $ORDERSYMS -dynamic -bundle -L. -o $EXE $ARCH $VERSION $SYRSROOT $LIBSEARCHPATH $FILELIST $LIBRARIES';
           DllCmd[2]:='strip -x $EXE';
           DynamicLinker:='';
         end;
      end;


    procedure tlinkerdarwin.LoadPredefinedLibraryOrder;
      begin
        // put your linkorder/linkalias overrides here.
        // Note: assumes only called when reordering/aliasing is used.
        LinkLibraryOrder.add('gcc','',15);
        LinkLibraryOrder.add('c','',50);
      end;


    procedure tlinkerdarwin.InitSysInitUnitName;
      begin
        SysInitUnit:='sysinit';
      end;


    function tlinkerdarwin.GetDarwinCrt1ObjName(isdll: boolean): TCmdStr;
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
                  system_arm_ios:
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
                  system_aarch64_ios,
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
                  system_arm_ios,
                  system_aarch64_ios:
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
                  system_aarch64_darwin:
                    exit('');
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
                  system_arm_ios,
                  system_aarch64_ios:
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
                  system_aarch64_darwin:
                    exit('');
                  else
                    Internalerror(2019050711);
                end;
                result:='dylib1.o';
              end;
          end;
      end;


    function tlinkerdarwin.GetDarwinPrtobjName(isdll: boolean): TCmdStr;
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


    function tlinkerdarwin.GetLinkArch: TCmdStr;
      begin
        case target_info.system of
          system_powerpc_darwin:
            result:='-arch ppc';
          system_i386_darwin,
          system_i386_iphonesim:
            result:='-arch i386';
          system_powerpc64_darwin:
            result:='-arch ppc64';
          system_x86_64_darwin,
          system_x86_64_iphonesim:
            result:='-arch x86_64';
          system_arm_ios:
            { current versions of the linker require the sub-architecture type
              to be specified }
            result:='-arch '+lower(cputypestr[current_settings.cputype]);
          system_aarch64_ios,
          system_aarch64_darwin:
            result:='-arch arm64';
          else
            internalerror(2014121801);
        end;
      end;


    function tlinkerdarwin.GetLinkVersion: TCmdStr;
      begin
        if MacOSXVersionMin<>'' then
          begin
            result:='-macosx_version_min '+MacOSXVersionMin;
          end
        else if iPhoneOSVersionMin<>'' then
          begin
            result:='-iphoneos_version_min '+iPhoneOSVersionMin;
          end
        else
          begin
            result:='';
          end;
      end;


    function tlinkerdarwin.GetSysroot: TCmdStr;
      begin
        if sysrootpath<>'' then
          begin
            result:='-syslibroot '+maybequoted(sysrootpath);
          end
        else
          begin
            result:='';
          end;
      end;


    function tlinkerdarwin.GetLibSearchPath: TCmdStr;
      var
        HPath: TCmdStrListItem;
      begin
        result:='';
        HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
        while assigned(HPath) do
         begin
           result:=result+' '+maybequoted('-L'+HPath.Str);
           HPath:=TCmdStrListItem(HPath.Next);
         end;
        HPath:=TCmdStrListItem(LibrarySearchPath.First);
        while assigned(HPath) do
         begin
           result:=result+' '+maybequoted('-L'+HPath.Str);
           HPath:=TCmdStrListItem(HPath.Next);
         end;

        HPath:=TCmdStrListItem(current_module.localframeworksearchpath.First);
        while assigned(HPath) do
         begin
           result:=result+' '+maybequoted('-F'+HPath.Str);
           HPath:=TCmdStrListItem(HPath.Next);
         end;
        HPath:=TCmdStrListItem(FrameworkSearchPath.First);
        while assigned(HPath) do
         begin
           result:=result+' '+maybequoted('-F'+HPath.Str);
           HPath:=TCmdStrListItem(HPath.Next);
         end;
      end;


    function tlinkerdarwin.GetLibraries: TCmdStr;
      var
        s: TCmdStr;
        i: longint;
      begin
        result:='';
        while not SharedLibFiles.Empty do
          begin
            s:=SharedLibFiles.GetFirst;
            if (s<>'c') or ReOrderEntries then
              begin
                i:=Pos(target_info.sharedlibext,s);
                if i>0 then
                  Delete(s,i,length(s));
                result:=result+' '+maybequoted('-l'+s);
              end;
           { be sure that libc is the last lib }
           if not ReOrderEntries then
             begin
               result:=result+' -lc'
             end;
         end;

        while not FrameworkFiles.empty do
          begin
            result:=result+' -framework '+maybequoted(FrameworkFiles.GetFirst);
          end;
      end;


    Function tlinkerdarwin.WriteFileList: TCmdStr;
    Var
      FilesList    : TScript;
      s            : TCmdStr;
    begin
      if ReOrderEntries Then
         ExpandAndApplyOrder(SharedLibFiles);

      { main objectfiles and static libraries: in filelist file to avoid overflowing command line limit }
      result:=UniqueName(outputexedir+'linkfiles')+'.res';
      FilesList:=TScript.Create(result);
      while not ObjectFiles.Empty do
        begin
          s:=ObjectFiles.GetFirst;
          if s<>'' then
            begin
              s:=TargetFixFileName(s);
              FilesList.Add(s);
            end;
          { Write staticlibraries }
          while not StaticLibFiles.Empty do
            begin
              s:=StaticLibFiles.GetFirst;
              FilesList.Add(s)
            end;
        end;
      FilesList.writetodisk;
      FilesList.Free;
    end;


    function tlinkerdarwin.MakeExecutable:boolean;
    var
      binstr,
      cmdstr,
      mapstr,
      targetstr,
      extdbgbinstr,
      extdbgcmdstr,
      ltostr,
      ordersymfile,
      linkfiles: TCmdStr;
      GCSectionsStr,
      StaticStr,
      StripStr   : TCmdStr;
      success : boolean;
    begin
      if not(cs_link_nolink in current_settings.globalswitches) then
       Message1(exec_i_linking,current_module.exefilename);

    { Create some replacements }
      StaticStr:='';
      StripStr:='';
      GCSectionsStr:='';
      mapstr:='';
      targetstr:='';
      ltostr:='';

      if (cs_link_map in current_settings.globalswitches) then
        mapstr:='-map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));

      if (cs_link_staticflag in current_settings.globalswitches) then
        StaticStr:='-static';
      if (cs_link_strip in current_settings.globalswitches) then
        StripStr:='-x';

      if (cs_link_smart in current_settings.globalswitches) then
        GCSectionsStr:='-dead_strip -no_dead_strip_inits_and_terms';

      { add custom LTO library if using custom clang }
      if (cs_lto in current_settings.moduleswitches) and
         not(cs_link_on_target in current_settings.globalswitches) and
         (utilsdirectory<>'') and
         FileExists(utilsdirectory+'/../lib/libLTO.dylib',true) then
        begin
          ltostr:='-lto_library '+maybequoted(utilsdirectory+'/../lib/libLTO.dylib');
        end;

      { Write list of files to link }
      linkfiles:=WriteFileList;

      { Write symbol order file }
      ordersymfile:=WriteSymbolOrderFile;

    { Call linker }
      SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
      Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
      Replace(cmdstr,'$OPT',Info.ExtraOptions);
      Replace(cmdstr,'$TARGET',targetstr);
      Replace(cmdstr,'$MAP',mapstr);
      Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
      Replace(cmdstr,'$LTO',ltostr);
      if ordersymfile<>'' then
        Replace(cmdstr,'$ORDERSYMS','-order_file '+maybequoted(ordersymfile))
      else
        Replace(cmdstr,'$ORDERSYMS','');

      Replace(cmdstr,'$STATIC',StaticStr);
      Replace(cmdstr,'$STRIP',StripStr);
      Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
      Replace(cmdstr,'$PRTOBJ',GetDarwinPrtobjName(false));
      Replace(cmdstr,'$ARCH', GetLinkArch);
      Replace(cmdstr,'$VERSION',GetLinkVersion);
      Replace(cmdstr,'$SYSROOT',GetSysroot);
      Replace(cmdstr,'$LIBSEARCHPATH',GetLibSearchPath);
      Replace(cmdstr,'$FILELIST','-filelist '+maybequoted(linkfiles));
      Replace(cmdstr,'$LIBRARIES',GetLibraries);
      BinStr:=FindUtil(utilsprefix+BinStr);

      { create dsym file? }
      extdbgbinstr:='';
      extdbgcmdstr:='';
      if (target_dbg.id in [dbg_dwarf2,dbg_dwarf3]) and
         (cs_link_separate_dbg_file in current_settings.globalswitches) then
        begin
          extdbgbinstr:=FindUtil(utilsprefix+'dsymutil');
          extdbgcmdstr:=maybequoted(current_module.exefilename);
        end;

      success:=DoExec(BinStr,CmdStr,true,false);
      if success and
         (extdbgbinstr<>'') then
        success:=DoExec(extdbgbinstr,extdbgcmdstr,false,false);

    { Remove ReponseFile }
      if (success) and not(cs_link_nolink in current_settings.globalswitches) then
       begin
         DeleteFile(outputexedir+Info.ResName);
         if ordersymfile<>'' then
           DeleteFile(ordersymfile);
         DeleteFile(linkfiles);
       end;

      MakeExecutable:=success;   { otherwise a recursive call to link method }
    end;


    Function tlinkerdarwin.MakeSharedLibrary:boolean;
    var
      InitStr,
      FiniStr,
      binstr,
      cmdstr,
      mapstr,
      ltostr,
      ordersymfile,
      targetstr,
      extdbgbinstr,
      extdbgcmdstr,
      linkfiles,
      GCSectionsStr : TCmdStr;
      exportedsyms: text;
      success : boolean;
    begin
      MakeSharedLibrary:=false;
      GCSectionsStr:='';
      mapstr:='';
      ltostr:='';
      if not(cs_link_nolink in current_settings.globalswitches) then
       Message1(exec_i_linking,current_module.sharedlibfilename);

      { Write list of files to link }
      linkfiles:=WriteFileList;

      { Write symbol order file }
      ordersymfile:=WriteSymbolOrderFile;

      if (cs_link_smart in current_settings.globalswitches) then
        GCSectionsStr:='-dead_strip -no_dead_strip_inits_and_terms';

      if (cs_link_map in current_settings.globalswitches) then
        mapstr:='-map '+maybequoted(ChangeFileExt(current_module.sharedlibfilename,'.map'));

      { add custom LTO library if using custom clang }
      if (cs_lto in current_settings.moduleswitches) and
         not(cs_link_on_target in current_settings.globalswitches) and
         (utilsdirectory<>'') and
         FileExists(utilsdirectory+'/../lib/libLTO.dylib',true) then
        begin
          ltostr:='-lto_library '+maybequoted(utilsdirectory+'/../lib/libLTO.dylib');
        end;

      targetstr:='';

      InitStr:='-init FPC_LIB_START';
      FiniStr:='-fini FPC_LIB_EXIT';

      { Call linker }
      SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
      Replace(cmdstr,'$EXE',maybequoted(ExpandFileName(current_module.sharedlibfilename)));
      Replace(cmdstr,'$OPT',Info.ExtraOptions);
      Replace(cmdstr,'$TARGET',targetstr);
      Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
      Replace(cmdstr,'$INIT',InitStr);
      Replace(cmdstr,'$FINI',FiniStr);
      Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
      Replace(cmdstr,'$MAP',mapstr);
      Replace(cmdstr,'$LTO',ltostr);
      if ordersymfile<>'' then
        Replace(cmdstr,'$ORDERSYMS','-order_file '+maybequoted(ordersymfile))
      else
        Replace(cmdstr,'$ORDERSYMS','');
      Replace(cmdstr,'$PRTOBJ',GetDarwinPrtobjName(true));
      Replace(cmdstr,'$ARCH', GetLinkArch);
      Replace(cmdstr,'$VERSION',GetLinkVersion);
      Replace(cmdstr,'$SYSROOT',GetSysroot);
      Replace(cmdstr,'$LIBSEARCHPATH',GetLibSearchPath);
      Replace(cmdstr,'$FILELIST','-filelist '+maybequoted(linkfiles));
      Replace(cmdstr,'$LIBRARIES',GetLibraries);
      BinStr:=FindUtil(utilsprefix+BinStr);

      { create dsym file? }
      extdbgbinstr:='';
      extdbgcmdstr:='';
      if (target_dbg.id in [dbg_dwarf2,dbg_dwarf3]) and
         (cs_link_separate_dbg_file in current_settings.globalswitches) then
        begin
          extdbgbinstr:=FindUtil(utilsprefix+'dsymutil');
          extdbgcmdstr:=maybequoted(current_module.sharedlibfilename);
        end;

      LinkSymsFileName:='';
      if not texportlibunix(exportlib).exportedsymnames.empty then
        begin
          LinkSymsFileName:=UniqueName('linksyms')+'.fpc';
          assign(exportedsyms,outputexedir+LinkSymsFileName);
          rewrite(exportedsyms);
          repeat
            writeln(exportedsyms,texportlibunix(exportlib).exportedsymnames.getfirst);
          until texportlibunix(exportlib).exportedsymnames.empty;
          close(exportedsyms);
          cmdstr:=cmdstr+' -exported_symbols_list '+maybequoted(outputexedir+LinkSymsFileName);
        end;

      success:=DoExec(BinStr,cmdstr,true,false);
      if (success and
          (extdbgbinstr<>'') and
          (cs_link_nolink in current_settings.globalswitches)) then
        success:=DoExec(extdbgbinstr,extdbgcmdstr,false,false);

    { Strip the library ? }
      if success and (cs_link_strip in current_settings.globalswitches) then
       begin
         SplitBinCmd(Info.DllCmd[2],binstr,cmdstr);
         Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
         success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,false,false);
       end;

    { Remove temporary files }
      if (success) and not(cs_link_nolink in current_settings.globalswitches) then
        begin
          DeleteFile(outputexedir+Info.ResName);
          if ordersymfile<>'' then
            DeleteFile(ordersymfile);
           if LinkSymsFileName<>'' then
             DeleteFile(outputexedir+LinkSymsFileName);
           DeleteFile(linkfiles);
        end;

      MakeSharedLibrary:=success;   { otherwise a recursive call to link method }
    end;



{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_darwin,tlinkerdarwin);
{$ifdef x86_64}
  RegisterImport(system_x86_64_darwin,timportlibdarwin);
  RegisterExport(system_x86_64_darwin,texportlibdarwin);
  RegisterTarget(system_x86_64_darwin_info);
  RegisterImport(system_x86_64_iphonesim,timportlibdarwin);
  RegisterExport(system_x86_64_iphonesim,texportlibdarwin);
  RegisterTarget(system_x86_64_iphonesim_info);
{$endif}
{$ifdef i386}
  RegisterImport(system_i386_darwin,timportlibdarwin);
  RegisterExport(system_i386_darwin,texportlibdarwin);
  RegisterTarget(system_i386_darwin_info);
  RegisterImport(system_i386_iphonesim,timportlibdarwin);
  RegisterExport(system_i386_iphonesim,texportlibdarwin);
  RegisterTarget(system_i386_iphonesim_info);
{$endif i386}
{$ifdef powerpc}
  RegisterImport(system_powerpc_darwin,timportlibdarwin);
  RegisterExport(system_powerpc_darwin,texportlibdarwin);
  RegisterTarget(system_powerpc_darwin_info);
{$endif powerpc}
{$ifdef powerpc64}
  RegisterImport(system_powerpc64_darwin,timportlibdarwin);
  RegisterExport(system_powerpc64_darwin,texportlibdarwin);
  RegisterTarget(system_powerpc64_darwin_info);
{$endif powerpc64}
{$ifdef arm}
  RegisterImport(system_arm_ios,timportlibdarwin);
  RegisterExport(system_arm_ios,texportlibdarwin);
  RegisterTarget(system_arm_ios_info);
{$endif arm}
{$ifdef aarch64}
  RegisterImport(system_aarch64_ios,timportlibdarwin);
  RegisterExport(system_aarch64_ios,texportlibdarwin);
  RegisterTarget(system_aarch64_ios_info);
  RegisterImport(system_aarch64_darwin,timportlibdarwin);
  RegisterExport(system_aarch64_darwin,texportlibdarwin);
  RegisterTarget(system_aarch64_darwin_info);
{$endif aarch64}

  RegisterRes(res_macho_info,TWinLikeResourceFile);
  RegisterRes(res_macosx_ext_info,TWinLikeResourceFile);
end.

