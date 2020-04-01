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
      LinkFilesFileName,
      LinkSymsFileName   : TCmdStr;
      Function  WriteResponseFile(isdll:boolean) : Boolean;
      function GetDarwinCrt1ObjName(isdll: boolean): TCmdStr;
      Function GetDarwinPrtobjName(isdll: boolean): TCmdStr;
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
           ExeCmd[1]:='ld $PRTOBJ $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST';
           if not(cs_gdb_valgrind in current_settings.globalswitches) then
             ExeCmd[1]:=ExeCmd[1]+' -pagezero_size 0x10000';
  {$else ndef cpu64bitaddr}
           ExeCmd[1]:='ld $PRTOBJ $TARGET $EMUL $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP $MAP $LTO $ORDERSYMS -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST';
  {$endif ndef cpu64bitaddr}
           if (apptype<>app_bundle) then
             DllCmd[1]:='ld $PRTOBJ $TARGET $EMUL $OPT $GCSECTIONS $MAP $LTO $ORDERSYMS -dynamic -dylib -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST'
           else
             DllCmd[1]:='ld $PRTOBJ $TARGET $EMUL $OPT $GCSECTIONS $MAP $LTO $ORDERSYMS -dynamic -bundle -multiply_defined suppress -L. -o $EXE $CATRES $FILELIST';
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


    Function tlinkerdarwin.GetDarwinPrtobjName(isdll: boolean): TCmdStr;
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


    Function tlinkerdarwin.WriteResponseFile(isdll:boolean) : Boolean;
    Var
      linkres      : TLinkRes;
      FilesList    : TLinkRes;
      i            : longint;
      HPath        : TCmdStrListItem;
      s,s1,s2      : TCmdStr;
      Fl1,Fl2      : Boolean;
    begin
      WriteResponseFile:=False;
      if ReOrderEntries Then
         ExpandAndApplyOrder(SharedLibFiles);

      { Open link.res file }
      LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,false);

      if sysrootpath<>'' then
        begin
          LinkRes.Add('-syslibroot');
          LinkRes.Add(sysrootpath);
        end;

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

      { Write path to search libraries }
      HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
      while assigned(HPath) do
       begin
         LinkRes.Add('-L'+HPath.Str);
         HPath:=TCmdStrListItem(HPath.Next);
       end;
      HPath:=TCmdStrListItem(LibrarySearchPath.First);
      while assigned(HPath) do
       begin
         LinkRes.Add('-L'+HPath.Str);
         HPath:=TCmdStrListItem(HPath.Next);
       end;

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

      { main objectfiles }
      LinkFilesFileName:=UniqueName('linkfiles')+'.res';
      FilesList:=TLinkRes.Create(outputexedir+LinkFilesFileName,false);
      while not ObjectFiles.Empty do
        begin
          s:=ObjectFiles.GetFirst;
          if s<>'' then
            begin
              s:=TargetFixFileName(s);
              FilesList.Add(s);
            end;
        end;
      FilesList.writetodisk;
      FilesList.Free;

      { Write staticlibraries }
      while not StaticLibFiles.Empty do
        begin
          S:=StaticLibFiles.GetFirst;
          LinkRes.AddFileName(s)
        end;

      { Write sharedlibraries like -l<lib> }
      while not SharedLibFiles.Empty do
        begin
          S:=SharedLibFiles.GetFirst;
          if (s<>'c') or ReOrderEntries then
            begin
              i:=Pos(target_info.sharedlibext,S);
              if i>0 then
                Delete(S,i,255);
              LinkRes.Add('-l'+s);
            end;
         { be sure that libc is the last lib }
         if not ReOrderEntries then
           begin
             LinkRes.Add('-lc')
           end;
       end;

      while not FrameworkFiles.empty do
        begin
          LinkRes.Add('-framework');
          LinkRes.Add(FrameworkFiles.GetFirst);
        end;

    { Write and Close response }
      linkres.writetodisk;
      linkres.Free;

      WriteResponseFile:=True;
    end;


    function tlinkerdarwin.MakeExecutable:boolean;
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
      success : boolean;
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
      targetstr:='';
      emulstr:='';
      ltostr:='';
      if (cs_link_map in current_settings.globalswitches) then
        mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));

      if (cs_link_staticflag in current_settings.globalswitches) then
        StaticStr:='-static';
      if (cs_link_strip in current_settings.globalswitches) then
        StripStr:='-x';

      if (cs_link_smart in current_settings.globalswitches) then
        GCSectionsStr:='-dead_strip -no_dead_strip_inits_and_terms';

      if CShared Then
       begin
         DynLinKStr:=DynLinkStr+' -dynamic'; // one dash!
       end;

      { add custom LTO library if using custom clang }
      if (cs_lto in current_settings.moduleswitches) and
         not(cs_link_on_target in current_settings.globalswitches) and
         (utilsdirectory<>'') and
         FileExists(utilsdirectory+'/../lib/libLTO.dylib',true) then
        begin
          ltostr:='-lto_library '+maybequoted(utilsdirectory+'/../lib/libLTO.dylib');
        end;

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
        Replace(cmdstr,'$ORDERSYMS','-order_file '+maybequoted(ordersymfile))
      else
        Replace(cmdstr,'$ORDERSYMS','');

      Replace(cmdstr,'$FILELIST','-filelist '+maybequoted(outputexedir+LinkFilesFileName));
      Replace(cmdstr,'$STATIC',StaticStr);
      Replace(cmdstr,'$STRIP',StripStr);
      Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
      Replace(cmdstr,'$DYNLINK',DynLinkStr);
      Replace(cmdstr,'$PRTOBJ',GetDarwinPrtobjName(false));
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

      if not(cs_link_nolink in current_settings.globalswitches) then
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

      success:=DoExec(BinStr,CmdStr,true,true);
      if (success and
          (extdbgbinstr<>'') and
          (cs_link_nolink in current_settings.globalswitches)) then
        success:=DoExec(extdbgbinstr,extdbgcmdstr,false,true);

    { Remove ReponseFile }
      if (success) and not(cs_link_nolink in current_settings.globalswitches) then
       begin
         DeleteFile(outputexedir+Info.ResName);
         if ordersymfile<>'' then
           DeleteFile(ordersymfile);
         DeleteFile(linkscript.fn);
         linkscript.free;
         DeleteFile(outputexedir+LinkFilesFileName);
       end;

      MakeExecutable:=success;   { otherwise a recursive call to link method }
    end;


    Function tlinkerdarwin.MakeSharedLibrary:boolean;
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

      if (cs_link_smart in current_settings.globalswitches) then
        GCSectionsStr:='-dead_strip -no_dead_strip_inits_and_terms';

      if (cs_link_map in current_settings.globalswitches) then
        mapstr:='-Map '+maybequoted(ChangeFileExt(current_module.sharedlibfilename,'.map'));

      { add custom LTO library if using custom clang }
      if (cs_lto in current_settings.moduleswitches) and
         not(cs_link_on_target in current_settings.globalswitches) and
         (utilsdirectory<>'') and
         FileExists(utilsdirectory+'/../lib/libLTO.dylib',true) then
        begin
          ltostr:='-lto_library '+maybequoted(utilsdirectory+'/../lib/libLTO.dylib');
        end;

      targetstr:='';
      emulstr:='';

      InitStr:='-init FPC_LIB_START';
      FiniStr:='-fini FPC_LIB_EXIT';
      SoNameStr:='-soname '+ExtractFileName(current_module.sharedlibfilename);

      { Call linker }
      SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
      Replace(cmdstr,'$EXE',maybequoted(ExpandFileName(current_module.sharedlibfilename)));
      Replace(cmdstr,'$OPT',Info.ExtraOptions);
      Replace(cmdstr,'$TARGET',targetstr);
      Replace(cmdstr,'$EMUL',EmulStr);
      Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
      Replace(cmdstr,'$FILELIST','-filelist '+maybequoted(outputexedir+LinkFilesFileName));
      Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
      Replace(cmdstr,'$INIT',InitStr);
      Replace(cmdstr,'$FINI',FiniStr);
      Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
      Replace(cmdstr,'$SONAME',SoNameStr);
      Replace(cmdstr,'$MAP',mapstr);
      Replace(cmdstr,'$LTO',ltostr);
      if ordersymfile<>'' then
        Replace(cmdstr,'$ORDERSYMS','-order_file '+maybequoted(ordersymfile))
      else
        Replace(cmdstr,'$ORDERSYMS','');
      Replace(cmdstr,'$PRTOBJ',GetDarwinPrtobjName(true));
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

      if not texportlibunix(exportlib).exportedsymnames.empty then
        begin
          LinkSymsFileName:=UniqueName('linksyms')+'.fpc';
          assign(exportedsyms,outputexedir+LinkSymsFileName);
          rewrite(exportedsyms);
          repeat
            writeln(exportedsyms,texportlibunix(exportlib).exportedsymnames.getfirst);
          until texportlibunix(exportlib).exportedsymnames.empty;
          close(exportedsyms);
          cmdstr:=cmdstr+' -exported_symbols_list '+maybequoted(outputexedir)+LinkSymsFileName;
        end;

      if not(cs_link_nolink in current_settings.globalswitches) then
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

      success:=DoExec(BinStr,cmdstr,true,true);
      if (success and
          (extdbgbinstr<>'') and
          (cs_link_nolink in current_settings.globalswitches)) then
        success:=DoExec(extdbgbinstr,extdbgcmdstr,false,true);

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
          DeleteFile(linkscript.fn);
          linkscript.free;
           if not texportlibunix(exportlib).exportedsymnames.empty then
             DeleteFile(outputexedir+LinkSymsFileName);
           DeleteFile(outputexedir+LinkFilesFileName);
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
  RegisterImport(system_arm_darwin,timportlibdarwin);
  RegisterExport(system_arm_darwin,texportlibdarwin);
  RegisterTarget(system_arm_darwin_info);
{$endif arm}
{$ifdef aarch64}
  RegisterImport(system_aarch64_darwin,timportlibdarwin);
  RegisterExport(system_aarch64_darwin,texportlibdarwin);
  RegisterTarget(system_aarch64_darwin_info);
{$endif aarch64}

  RegisterRes(res_macho_info,TWinLikeResourceFile);
  RegisterRes(res_macosx_ext_info,TWinLikeResourceFile);
end.

