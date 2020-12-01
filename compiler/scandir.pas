{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit implements directive parsing for the scanner

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
unit scandir;

{$i fpcdefs.inc}

  interface

    uses
      globtype,
      systems;

    const
      switchesstatestackmax = 20;

    type
      tsavedswitchesstate = record
        localsw: tlocalswitches;
        verbosity: longint;
        pmessage : pmessagestaterecord;
        alignment : talignmentinfo;
        setalloc,
        packenum,
        packrecords : shortint;
      end;

    type
      tswitchesstatestack = array[0..switchesstatestackmax] of tsavedswitchesstate;

    var
      switchesstatestack:tswitchesstatestack;
      switchesstatestackpos: Integer;

    procedure InitScannerDirectives;

  implementation

    uses
      SysUtils,
      cutils,cfileutl,
      globals,widestr,cpuinfo,
      verbose,comphook,ppu,
      scanner,switches,
      fmodule,
      defutil,
      dirparse,link,
      syscinfo,
      symconst,symtable,symbase,symtype,symsym,
      rabase;

{*****************************************************************************
                                    Helpers
*****************************************************************************}

    procedure do_delphiswitch(sw:char);
      var
        state : char;
      begin
      { c contains the next char, a + or - would be fine }
        state:=current_scanner.readstate;
        if state in ['-','+'] then
          HandleSwitch(sw,state);
      end;


    procedure do_setverbose(flag:char);
      var
        state : char;
      begin
      { support ON/OFF }
        state:=current_scanner.ReadState;
        recordpendingverbosityswitch(flag,state);
      end;


    procedure do_moduleswitch(sw:tmoduleswitch);
      var
        state : char;
      begin
        state:=current_scanner.readstate;
        if (sw<>cs_modulenone) and (state in ['-','+']) then
         begin
           if state='-' then
            exclude(current_settings.moduleswitches,sw)
           else
            include(current_settings.moduleswitches,sw);
         end;
      end;


    procedure do_localswitch(sw:tlocalswitch);
      var
        state : char;
      begin
        state:=current_scanner.readstate;
        if (sw<>cs_localnone) and (state in ['-','+']) then
          recordpendinglocalswitch(sw,state);
      end;

    function do_localswitchdefault(sw:tlocalswitch): char;
      begin
        result:=current_scanner.readstatedefault;
        if (sw<>cs_localnone) and (result in ['-','+','*']) then
          recordpendinglocalswitch(sw,result);
      end;


    procedure do_moduleflagswitch(flag:tmoduleflag;optional:boolean);
      var
        state : char;
      begin
        if optional then
          state:=current_scanner.readoptionalstate('+')
        else
          state:=current_scanner.readstate;
        if state='-' then
          exclude(current_module.moduleflags,flag)
        else
          include(current_module.moduleflags,flag);
      end;


    procedure do_message(w:integer);
      begin
        current_scanner.skipspace;
        Message1(w,current_scanner.readcomment);
      end;


    procedure do_version(out major, minor, revision: word; out verstr: string; allowrevision: boolean; out isset: boolean);
      var
        majorl,
        minorl,
        revisionl,
        error : longint;
      begin
        { change description global var in all cases }
        { it not used but in win32, os2 and netware }
        current_scanner.skipspace;
        { we should only accept Major.Minor format for win32 and os2 }
        current_scanner.readnumber;
        major:=0;
        minor:=0;
        revision:=0;
        verstr:='';
        isset:=false;
        majorl:=0;
        minorl:=0;
        revisionl:=0;
        val(pattern,majorl,error);
        if (error<>0) or (majorl > high(word)) or (majorl < 0) then
          begin
            Message1(scan_w_wrong_version_ignored,pattern);
            exit;
          end;
        isset:=true;
        if c='.' then
          begin
            current_scanner.readchar;
            current_scanner.readnumber;
            val(pattern,minorl,error);
            if (error<>0) or (minorl > high(word)) or (minorl < 0) then
              begin
                Message1(scan_w_wrong_version_ignored,tostr(majorl)+'.'+pattern);
                exit;
              end;
            if (c='.') and
               allowrevision then
              begin
                 current_scanner.readchar;
                 current_scanner.readnumber;
                 val(pattern,revisionl,error);
                 if (error<>0) or (revisionl > high(word)) or (revisionl < 0) then
                   begin
                      Message1(scan_w_wrong_version_ignored,tostr(majorl)+'.'+tostr(minorl)+'.'+pattern);
                      exit;
                   end;
                 major:=word(majorl);
                 minor:=word(minorl);
                 revision:=word(revisionl);
                 verstr:=tostr(major)+','+tostr(minor)+','+tostr(revision);
              end
            else
              begin
                 major:=word(majorl);
                 minor:=word(minorl);
                 verstr:=tostr(major)+'.'+tostr(minor);
              end;
          end
        else
          begin
            major:=word(majorl);
            verstr:=tostr(major);
          end;
      end;


{*****************************************************************************
                              Directive Callbacks
*****************************************************************************}

    procedure dir_align;
      var
        hs : string;
        b : byte;
      begin
        current_scanner.skipspace;
        if not(c in ['0'..'9']) then
         begin
           { Support also the ON and OFF as switch }
           hs:=current_scanner.readid;
           if (hs='ON') then
            current_settings.packrecords:=4
           else if (hs='OFF') then
             current_settings.packrecords:=1
           else if m_mac in current_settings.modeswitches then
             begin
               { Support switches used in Apples Universal Interfaces}
               if (hs='MAC68K') then
                 current_settings.packrecords:=mac68k_alignment
               { "power" alignment is the default C packrecords setting on
                 Mac OS X }
               else if (hs='POWER') or (hs='POWERPC') then
                 current_settings.packrecords:=C_alignment
               else if (hs='RESET') then
                 current_settings.packrecords:=default_settings.packrecords
               else
                 Message1(scan_e_illegal_pack_records,hs);
             end
           else
             Message1(scan_e_illegal_pack_records,hs);
         end
        else
         begin
           b:=current_scanner.readval;
           case b of
             1 : current_settings.packrecords:=1;
             2 : current_settings.packrecords:=2;
             4 : current_settings.packrecords:=4;
             8 : current_settings.packrecords:=8;
            16 : current_settings.packrecords:=16;
            32 : current_settings.packrecords:=32;
           else
            Message1(scan_e_illegal_pack_records,tostr(b));
           end;
         end;
      end;

    procedure dir_a1;
      begin
        current_settings.packrecords:=1;
      end;

    procedure dir_a2;
      begin
        current_settings.packrecords:=2;
      end;

    procedure dir_a4;
      begin
        current_settings.packrecords:=4;
      end;

    procedure dir_a8;
      begin
        current_settings.packrecords:=8;
      end;

    procedure dir_asmcpu;
      var
        s : string;
        cpu: tcputype;
        found: Boolean;
      begin
        current_scanner.skipspace;
        s:=current_scanner.readid;
        If Inside_asm_statement then
          Message1(scan_w_no_asm_reader_switch_inside_asm,s);
        if s='ANY' then
          current_settings.asmcputype:=cpu_none
        else if s='CURRENT' then
          current_settings.asmcputype:=current_settings.cputype
        else
          begin
            found:=false;
            for cpu:=succ(low(tcputype)) to high(tcputype) do
              if s=cputypestr[cpu] then
                begin
                  found:=true;
                  current_settings.asmcputype:=cpu;
                  break;
                end;
            if not found then
              Message1(scan_e_illegal_asmcpu_specifier,s);
          end;
      end;

    procedure dir_asmmode;
      var
        s : string;
      begin
        current_scanner.skipspace;
        s:=current_scanner.readid;
        If Inside_asm_statement then
          Message1(scan_w_no_asm_reader_switch_inside_asm,s);
        if s='DEFAULT' then
         current_settings.asmmode:=init_settings.asmmode
        else
         if not SetAsmReadMode(s,current_settings.asmmode) then
           Message1(scan_e_illegal_asmmode_specifier,s);
      end;

{$if defined(m68k) or defined(arm)}
    procedure dir_appid;
      begin
        if target_info.system<>system_m68k_palmos then
          Message(scan_w_appid_not_support);
        { change description global var in all cases }
        { it not used but in win32 and os2 }
        current_scanner.skipspace;
        palmos_applicationid:=current_scanner.readcomment;
      end;

    procedure dir_appname;
      begin
        if target_info.system<>system_m68k_palmos then
          Message(scan_w_appname_not_support);
        { change description global var in all cases }
        { it not used but in win32 and os2 }
        current_scanner.skipspace;
        palmos_applicationname:=current_scanner.readcomment;
      end;
{$endif defined(m68k) or defined(arm)}

    procedure dir_apptype;
      var
         hs : string;
      begin
        if not (target_info.system in systems_all_windows + [system_i386_os2,
                                       system_i386_emx, system_powerpc_macosclassic,
                                       system_arm_nds, system_i8086_msdos,
                                       system_i8086_embedded, system_m68k_atari] +
                                       systems_nativent) then
          begin
            if m_delphi in current_settings.modeswitches then
              Message(scan_n_app_type_not_support)
            else
              Message(scan_w_app_type_not_support);
          end
        else
          begin
            if not current_module.in_global then
              Message(scan_w_switch_is_global)
            else
              begin
                 current_scanner.skipspace;
                 hs:=current_scanner.readid;
                 if (hs='GUI') and not (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
                   SetApptype(app_gui)
                 else if (hs='CONSOLE') and not (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
                   SetApptype(app_cui)
                 else if (hs='NATIVE') and (target_info.system in systems_windows + systems_nativent) then
                   SetApptype(app_native)
                 else if (hs='FS') and (target_info.system in [system_i386_os2,
                                                             system_i386_emx]) then
                   SetApptype(app_fs)
                 else if (hs='TOOL') and (target_info.system in [system_powerpc_macosclassic]) then
                   SetApptype(app_tool)
                 else if (hs='ARM9') and (target_info.system in [system_arm_nds]) then
                   SetApptype(app_arm9)
                 else if (hs='ARM7') and (target_info.system in [system_arm_nds]) then
                   SetApptype(app_arm7)
                 else if (hs='COM') and (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
                   SetApptype(app_com)
                 else if (hs='EXE') and (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
                   SetApptype(app_cui)
                 else
                   Message1(scan_w_unsupported_app_type,hs);
              end;
          end;
      end;


    procedure dir_calling;
      var
         hs : string;
      begin
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        if (hs='') then
          Message(parser_e_proc_directive_expected)
        else
          recordpendingcallingswitch(hs);
      end;


    procedure dir_checklowaddrloads;
      begin
        do_localswitchdefault(cs_check_low_addr_load);
      end;


    procedure dir_checkpointer;
      var
        switch: char;
      begin
        switch:=do_localswitchdefault(cs_checkpointer);
        if (switch='+') and
           not(target_info.system in systems_support_checkpointer) then
          Message1(scan_e_unsupported_switch,'CHECKPOINTER+');
      end;


    procedure dir_excessprecision;
      begin
        do_localswitch(cs_excessprecision);
      end;


    procedure dir_checkcasecoverage;
      begin
        do_localswitch(cs_check_all_case_coverage);
      end;


    procedure dir_checkfpuexceptions;
      begin
        do_localswitch(cs_check_fpu_exceptions);
      end;


    procedure dir_objectchecks;
      begin
        do_localswitch(cs_check_object);
      end;


    procedure dir_ieeeerrors;
      begin
        do_localswitch(cs_ieee_errors);
      end;


    procedure dir_assertions;
      begin
        do_delphiswitch('C');
      end;


    procedure dir_booleval;
      begin
        do_delphiswitch('B');
      end;

    procedure dir_debuginfo;
      begin
        do_delphiswitch('D');
      end;

    procedure dir_denypackageunit;
      begin
        do_moduleflagswitch(mf_package_deny,true);
      end;

    procedure dir_description;
      begin
        if not (target_info.system in systems_all_windows+[system_i386_os2,system_i386_emx,
                 system_i386_netware,system_i386_wdosx,system_i386_netwlibc,system_i8086_win16]) then
          Message(scan_w_description_not_support);
        { change description global var in all cases }
        { it not used but in win32, os2 and netware }
        current_scanner.skipspace;
        description:=current_scanner.readcomment;
        DescriptionSetExplicity:=true;
      end;

    procedure dir_screenname; {ad}
      begin
        if not (target_info.system in [system_i386_netware,system_i386_netwlibc]) then
          {Message(scan_w_decription_not_support);}
          comment (V_Warning,'Screenname only supported for target netware');
        current_scanner.skipspace;
        nwscreenname:=current_scanner.readcomment;
      end;

      procedure dir_threadname; {ad}
      begin
        if not (target_info.system in [system_i386_netware,system_i386_netwlibc]) then
          {Message(scan_w_decription_not_support);}
          comment (V_Warning,'Threadname only supported for target netware');
        current_scanner.skipspace;
        nwthreadname:=current_scanner.readcomment;
      end;

      procedure dir_copyright; {ad}
      begin
        if not (target_info.system in [system_i386_netware,system_i386_netwlibc]) then
          {Message(scan_w_decription_not_support);}
          comment (V_Warning,'Copyright only supported for target netware');
        current_scanner.skipspace;
        nwcopyright:=current_scanner.readcomment;
      end;

    procedure dir_error;
      begin
        do_message(scan_e_user_defined);
      end;

    procedure dir_extendedsyntax;
      begin
        do_delphiswitch('X');
      end;

    procedure dir_forcefarcalls;
      begin
        if not (target_info.system in [system_i8086_msdos,system_i8086_embedded])
{$ifdef i8086}
           or (current_settings.x86memorymodel in x86_near_code_models)
{$endif i8086}
            then
          begin
            Message1(scan_n_ignored_switch,pattern);
            exit;
          end;
        do_localswitch(cs_force_far_calls);
      end;

    procedure dir_fatal;
      begin
        do_message(scan_f_user_defined);
      end;

    procedure dir_fputype;
      begin
        current_scanner.skipspace;
        undef_system_macro('FPU'+fputypestr[current_settings.fputype]);
        if not(SetFPUType(upper(current_scanner.readcomment),current_settings.fputype)) then
          comment(V_Error,'Illegal FPU type');
        def_system_macro('FPU'+fputypestr[current_settings.fputype]);
     end;

    procedure dir_frameworkpath;
      begin
        if not current_module.in_global then
         Message(scan_w_switch_is_global)
        else if not(target_info.system in systems_darwin) then
          begin
            Message(scan_w_frameworks_darwin_only);
            current_scanner.skipspace;
            current_scanner.readcomment
          end
        else
          begin
            current_scanner.skipspace;
            current_module.localframeworksearchpath.AddPath(current_scanner.readcomment,false);
          end;
      end;

    procedure dir_goto;
      begin
        do_moduleswitch(cs_support_goto);
      end;

    procedure dir_hint;
      begin
        do_message(scan_h_user_defined);
      end;

    procedure dir_hints;
      begin
        do_setverbose('H');
      end;

    procedure dir_imagebase;
      begin
        if not (target_info.system in (systems_windows+systems_wince)) then
          Message(scan_w_imagebase_not_support);
        current_scanner.skipspace;
        imagebase:=current_scanner.readval;
        ImageBaseSetExplicity:=true
      end;

    procedure dir_implicitexceptions;
      begin
        do_moduleswitch(cs_implicit_exceptions);
      end;

    procedure dir_importeddata;
      begin
        do_delphiswitch('G');
      end;

    procedure dir_includepath;
      begin
        if not current_module.in_global then
         Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            current_module.localincludesearchpath.AddPath(current_scanner.readcomment,false);
          end;
      end;

    procedure dir_info;
      begin
        do_message(scan_i_user_defined);
      end;

    procedure dir_inline;
      begin
        do_localswitch(cs_do_inline);
      end;

    procedure dir_interfaces;
      var
        hs : string;
      begin
        {corba/com/default}
        current_scanner.skipspace;
        hs:=current_scanner.readid;
{$ifndef jvm}
        if (hs='CORBA') then
          current_settings.interfacetype:=it_interfacecorba
        else if (hs='COM') then
          current_settings.interfacetype:=it_interfacecom
        else
{$endif jvm}
             if (hs='DEFAULT') then
          current_settings.interfacetype:=init_settings.interfacetype
        else
          Message(scan_e_invalid_interface_type);
      end;

    procedure dir_iochecks;
      begin
        do_delphiswitch('I');
      end;

    procedure dir_libexport;
      begin
        {not implemented}
      end;

    procedure dir_librarypath;
      begin
        if not current_module.in_global then
         Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            current_module.locallibrarysearchpath.AddPath(current_scanner.readcomment,false);
          end;
      end;

    procedure dir_link;
      var
        s : string;
      begin
        current_scanner.skipspace;
        if scanner.c = '''' then
          begin
            s:= current_scanner.readquotedstring;
            current_scanner.readcomment
          end
        else
          s:= trimspace(current_scanner.readcomment);
        s:=FixFileName(s);
        if ExtractFileExt(s)='' then
          s:=ChangeFileExt(s,target_info.objext);
        current_module.linkotherofiles.add(s,link_always);
      end;

    procedure dir_linkframework;
      var
        s : string;
      begin
        current_scanner.skipspace;
        if scanner.c = '''' then
          begin
            s:= current_scanner.readquotedstring;
            current_scanner.readcomment
          end
        else
          s:= trimspace(current_scanner.readcomment);
        s:=FixFileName(s);
        if (target_info.system in systems_darwin) then
          current_module.linkotherframeworks.add(s,link_always)
        else
          Message(scan_w_frameworks_darwin_only);
      end;

    procedure dir_linklib;
      type
        tLinkMode=(lm_shared,lm_static);
      var
        s : string;
        quote : char;
        libext,
        libname,
        linkmodestr : string;
        p : longint;
        linkMode : tLinkMode;
      begin
        current_scanner.skipspace;
        if scanner.c = '''' then
          begin
            libname:= current_scanner.readquotedstring;
            s:= current_scanner.readcomment;
            p:=pos(',',s);
          end
        else
          begin
            s:= current_scanner.readcomment;
            p:=pos(',',s);
            if p=0 then
              libname:=TrimSpace(s)
            else
              libname:=TrimSpace(copy(s,1,p-1));
          end;
        if p=0 then
          linkmodeStr:=''
        else
          linkmodeStr:=Upper(TrimSpace(copy(s,p+1,255)));


        if (libname='') or (libname='''''') or (libname='""') then
         exit;
        { create library name }
        if libname[1] in ['''','"'] then
         begin
           quote:=libname[1];
           Delete(libname,1,1);
           p:=pos(quote,libname);
           if p>0 then
            Delete(libname,p,1);
         end;
        libname:=FixFileName(libname);

        { get linkmode, default is to check the extension for
          the static library, otherwise shared linking is assumed }
        linkmode:=lm_shared;
        if linkModeStr='' then
         begin
           libext:=ExtractFileExt(libname);
           if libext=target_info.staticClibext then
             linkMode:=lm_static;
         end
        else if linkModeStr='STATIC' then
         linkmode:=lm_static
        else if (LinkModeStr='SHARED') or (LinkModeStr='') then
         linkmode:=lm_shared
        else
         Comment(V_Error,'Wrong link mode specified: "'+Linkmodestr+'"');

        { add to the list of other libraries }
        if linkMode=lm_static then
         current_module.linkOtherStaticLibs.add(libname,link_always)
        else
         current_module.linkOtherSharedLibs.add(libname,link_always);
      end;

    procedure dir_localsymbols;
      begin
        do_delphiswitch('L');
      end;

    procedure dir_longstrings;
      begin
        do_delphiswitch('H');
      end;

    procedure dir_macro;
      begin
        do_moduleswitch(cs_support_macro);
      end;

    procedure dir_pascalmainname;
      var
        s: string;
      begin
        current_scanner.skipspace;
        s:=trimspace(current_scanner.readcomment);
        if assigned(current_module.mainname) and
           (s<>current_module.mainname^) then
          begin
            Message1(scan_w_multiple_main_name_overrides,current_module.mainname^);
            stringdispose(current_module.mainname)
          end
        else if (mainaliasname<>defaultmainaliasname) and
                (mainaliasname<>s) then
          Message1(scan_w_multiple_main_name_overrides,mainaliasname);
        mainaliasname:=s;
        if (mainaliasname<>defaultmainaliasname) then
          current_module.mainname:=stringdup(mainaliasname);
      end;

    procedure dir_maxfpuregisters;
      var
         l  : integer;
         hs : string;
      begin
         current_scanner.skipspace;
         if not(c in ['0'..'9']) then
           begin
              hs:=current_scanner.readid;
              if (hs='NORMAL') or (hs='DEFAULT') then
                current_settings.maxfpuregisters:=-1
              else
                Message(scan_e_invalid_maxfpureg_value);
           end
         else
           begin
              l:=current_scanner.readval;
              case l of
                 0..8:
                   current_settings.maxfpuregisters:=l;
                 else
                   Message(scan_e_invalid_maxfpureg_value);
              end;
           end;
      end;

    procedure dir_maxstacksize;
      begin
        if not (target_info.system in (systems_windows+systems_wince)) then
          Message(scan_w_maxstacksize_not_support);
        current_scanner.skipspace;
        maxstacksize:=current_scanner.readval;
        MaxStackSizeSetExplicity:=true;
      end;

    procedure dir_memory;
      var
        l : longint;
        heapsize_limit: longint;
        maxheapsize_limit: longint;
      begin
{$if defined(i8086)}
        if target_info.system=system_i8086_win16 then
          begin
            heapsize_limit:=65520;
            maxheapsize_limit:=65520;
          end
        else if current_settings.x86memorymodel in x86_far_data_models then
          begin
            heapsize_limit:=655360;
            maxheapsize_limit:=655360;
          end
        else
          begin
            heapsize_limit:=65520;
            maxheapsize_limit:=65520;
          end;
{$elseif defined(cpu16bitaddr)}
        heapsize_limit:=65520;
        maxheapsize_limit:=65520;
{$else}
        heapsize_limit:=high(heapsize);
        maxheapsize_limit:=high(maxheapsize);
{$endif}
        current_scanner.skipspace;
        l:=current_scanner.readval;
        if (l>=1024)
{$ifdef cpu16bitaddr}
          and (l<=65521) { TP7's $M directive allows specifying a stack size of
                           65521, but it actually sets the stack size to 65520 }
{$else cpu16bitaddr}
          and (l<67107840)
{$endif cpu16bitaddr}
        then
          stacksize:=min(l,{$ifdef cpu16bitaddr}65520{$else}67107839{$endif})
        else
          Message(scan_w_invalid_stacksize);
        if c=',' then
          begin
            current_scanner.readchar;
            current_scanner.skipspace;
            l:=current_scanner.readval;
            if l>=1024 then
              heapsize:=min(l,heapsize_limit);
            if c=',' then
              begin
                current_scanner.readchar;
                current_scanner.skipspace;
                l:=current_scanner.readval;
                if l>=heapsize then
                  maxheapsize:=min(l,maxheapsize_limit)
                else
                  Message(scan_w_heapmax_lessthan_heapmin);
              end;
          end;
      end;


    procedure dir_message;
      var
        hs : string;
        w  : longint;
      begin
        w:=0;
        current_scanner.skipspace;
        { Message level specified? }
        if c='''' then
          w:=scan_n_user_defined
        else
          begin
            hs:=current_scanner.readid;
            if (hs='WARN') or (hs='WARNING') then
              w:=scan_w_user_defined
            else
              if (hs='ERROR') then
                w:=scan_e_user_defined
            else
              if (hs='FATAL') then
                w:=scan_f_user_defined
            else
              if (hs='HINT') then
                w:=scan_h_user_defined
            else
              if (hs='NOTE') then
                w:=scan_n_user_defined
            else
              if (hs='INFO') then
                w:=scan_i_user_defined
            else
              Message1(scan_w_illegal_directive,hs);
          end;
        { Only print message when there was no error }
        if w<>0 then
          begin
            current_scanner.skipspace;
            if c='''' then
              hs:=current_scanner.readquotedstring
            else
              hs:=current_scanner.readcomment;
            Message1(w,hs);
          end
        else
          current_scanner.readcomment;
      end;


    procedure dir_minstacksize;
      begin
        if not (target_info.system in (systems_windows+systems_wince)) then
          Message(scan_w_minstacksize_not_support);
        current_scanner.skipspace;
        minstacksize:=current_scanner.readval;
        MinStackSizeSetExplicity:=true;
      end;


    procedure dir_mode;

    begin
      if not current_module.in_global then
        Message(scan_w_switch_is_global)
      else
        begin
          current_scanner.skipspace;
          current_scanner.readstring;
          if not current_module.mode_switch_allowed and
              not ((m_mac in current_settings.modeswitches) and (pattern='MACPAS')) then
            Message1(scan_e_mode_switch_not_allowed,pattern)
          else if not SetCompileMode(pattern,false) then
            Message1(scan_w_illegal_switch,pattern)
        end;
      current_module.mode_switch_allowed:= false;
    end;


    procedure dir_modeswitch;
      var
        s : string;
      begin
        if not current_module.in_global then
          Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            current_scanner.readstring;
            s:=pattern;
            { don't combine the assignments to s as the method call will be
              done before "pattern" is assigned to s and the method changes
              "pattern" }
            s:=s+current_scanner.readoptionalstate('+');
            if not SetCompileModeSwitch(s,false) then
              Message1(scan_w_illegal_switch,s)
          end;
      end;


    procedure dir_namespace;
      var
        s : string;
      begin
        { used to define Java package names for all types declared in the
          current unit }
        if not current_module.in_global then
          Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            current_scanner.readstring;
            s:=orgpattern;
            while c='.' do
              begin
                current_scanner.readchar;
                current_scanner.readstring;
                s:=s+'.'+orgpattern;
              end;
            disposestr(current_module.namespace);
            current_module.namespace:=stringdup(s);
          end;
      end;


    procedure dir_mmx;
      begin
        do_localswitch(cs_mmx);
      end;

    procedure dir_note;
      begin
        do_message(scan_n_user_defined);
      end;

    procedure dir_notes;
      begin
        do_setverbose('N');
      end;

    procedure dir_objectpath;
      begin
        if not current_module.in_global then
         Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            current_module.localobjectsearchpath.AddPath(current_scanner.readcomment,false);
          end;
      end;

    procedure dir_openstrings;
      begin
        do_delphiswitch('P');
      end;

    procedure dir_optimization;
      var
        hs : string;
      begin
        current_scanner.skipspace;
        { Support also the ON and OFF as switch }
        hs:=current_scanner.readid;
        if (hs='ON') then
          current_settings.optimizerswitches:=level2optimizerswitches
        else if (hs='OFF') then
          current_settings.optimizerswitches:=[]
        else if (hs='DEFAULT') then
          current_settings.optimizerswitches:=init_settings.optimizerswitches
        else
          begin
            if not UpdateOptimizerStr(hs,current_settings.optimizerswitches) then
              Message1(scan_e_illegal_optimization_specifier,hs);
          end;
      end;

    procedure dir_overflowchecks;
      begin
        do_delphiswitch('Q');
      end;

    procedure dir_packenum;
      var
        hs : string;
      begin
        current_scanner.skipspace;
        if not(c in ['0'..'9']) then
         begin
           hs:=current_scanner.readid;
           if (hs='NORMAL') or (hs='DEFAULT') then
            current_settings.packenum:=4
           else
            Message1(scan_e_illegal_pack_enum, hs);
         end
        else
         begin
           case current_scanner.readval of
            1 : current_settings.packenum:=1;
            2 : current_settings.packenum:=2;
            4 : current_settings.packenum:=4;
           else
            Message1(scan_e_illegal_pack_enum, pattern);
           end;
         end;
      end;


    procedure dir_minfpconstprec;
      begin
        current_scanner.skipspace;
        if not SetMinFPConstPrec(current_scanner.readid,current_settings.minfpconstprec) then
          Message1(scan_e_illegal_minfpconstprec, pattern);
      end;


    procedure dir_packrecords;
      var
        hs : string;
      begin
        { can't change packrecords setting on managed vm targets }
        if target_info.system in systems_managed_vm then
          Message1(scanner_w_directive_ignored_on_target, 'PACKRECORDS');
        current_scanner.skipspace;
        if not(c in ['0'..'9']) then
         begin
           hs:=current_scanner.readid;
           { C has the special recordalignmax of C_alignment }
           if (hs='C') then
            current_settings.packrecords:=C_alignment
           else
            if (hs='NORMAL') or (hs='DEFAULT') then
             current_settings.packrecords:=default_settings.packrecords
           else
            Message1(scan_e_illegal_pack_records,hs);
         end
        else
         begin
           case current_scanner.readval of
             1 : current_settings.packrecords:=1;
             2 : current_settings.packrecords:=2;
             4 : current_settings.packrecords:=4;
             8 : current_settings.packrecords:=8;
            16 : current_settings.packrecords:=16;
            32 : current_settings.packrecords:=32;
           else
            Message1(scan_e_illegal_pack_records,pattern);
           end;
         end;
      end;


    procedure dir_packset;
      var
        hs : string;
      begin
        current_scanner.skipspace;
        if not(c in ['1','2','4','8']) then
         begin
           hs:=current_scanner.readid;
           if (hs='FIXED') or (hs='DEFAULT') OR (hs='NORMAL') then
            current_settings.setalloc:=0               {Fixed mode, sets are 4 or 32 bytes}
           else
            Message(scan_e_only_packset);
         end
        else
         begin
           case current_scanner.readval of
            1 : current_settings.setalloc:=1;
            2 : current_settings.setalloc:=2;
            4 : current_settings.setalloc:=4;
            8 : current_settings.setalloc:=8;
           else
            Message(scan_e_only_packset);
           end;
         end;
      end;


    procedure dir_pic;
      begin
        { windows doesn't need/support pic }
        if tf_no_pic_supported in target_info.flags then
          message(scan_w_pic_ignored)
        else
          do_moduleswitch(cs_create_pic);
      end;

    procedure dir_pop;

    begin
      if switchesstatestackpos < 1 then
        Message(scan_e_too_many_pop);

      Dec(switchesstatestackpos);
      recordpendinglocalfullswitch(switchesstatestack[switchesstatestackpos].localsw);
      recordpendingverbosityfullswitch(switchesstatestack[switchesstatestackpos].verbosity);
      recordpendingalignmentfullswitch(switchesstatestack[switchesstatestackpos].alignment);
      recordpendingpackenum(switchesstatestack[switchesstatestackpos].packenum);
      recordpendingpackrecords(switchesstatestack[switchesstatestackpos].packrecords);
      recordpendingsetalloc(switchesstatestack[switchesstatestackpos].setalloc);
      pendingstate.nextmessagerecord:=switchesstatestack[switchesstatestackpos].pmessage;
      { Reset verbosity and forget previous pmeesage }
      RestoreLocalVerbosity(nil);
      current_settings.pmessage:=nil;
      { Do not yet activate these changes, as otherwise
        you get problem idf you put a $pop just right after
        a addition for instance fro which you explicitly truned the overflow check
        out by using $Q- after a $push PM 2012-08-29 }
      // flushpendingswitchesstate;
    end;

    procedure dir_pointermath;
      begin
        do_localswitch(cs_pointermath);
      end;

    procedure dir_profile;
      begin
        do_moduleswitch(cs_profile);
        { defined/undefine FPC_PROFILE }
        if cs_profile in current_settings.moduleswitches then
          def_system_macro('FPC_PROFILE')
        else
          undef_system_macro('FPC_PROFILE');
      end;

    procedure dir_push;

    begin
      if switchesstatestackpos > switchesstatestackmax then
        Message(scan_e_too_many_push);

      { do not flush here as we might have read directives which shall not be active yet,
        see e.g. tests/webtbs/tw22744b.pp }
      if psf_alignment_changed in pendingstate.flags then
        switchesstatestack[switchesstatestackpos].alignment:=pendingstate.nextalignment
      else
        switchesstatestack[switchesstatestackpos].alignment:=current_settings.alignment;

      if psf_verbosity_full_switched in pendingstate.flags then
        switchesstatestack[switchesstatestackpos].verbosity:=pendingstate.nextverbosityfullswitch
      else
        switchesstatestack[switchesstatestackpos].verbosity:=status.verbosity;

      if psf_local_switches_changed in pendingstate.flags then
        switchesstatestack[switchesstatestackpos].localsw:=pendingstate.nextlocalswitches
      else
        switchesstatestack[switchesstatestackpos].localsw:=current_settings.localswitches;

      if psf_packenum_changed in pendingstate.flags then
        switchesstatestack[switchesstatestackpos].packenum:=pendingstate.nextpackenum
      else
        switchesstatestack[switchesstatestackpos].packenum:=current_settings.packenum;

      if psf_packrecords_changed in pendingstate.flags then
        switchesstatestack[switchesstatestackpos].packrecords:=pendingstate.nextpackrecords
      else
        switchesstatestack[switchesstatestackpos].packrecords:=current_settings.packrecords;

      if psf_setalloc_changed in pendingstate.flags then
        switchesstatestack[switchesstatestackpos].setalloc:=pendingstate.nextsetalloc
      else
        switchesstatestack[switchesstatestackpos].setalloc:=current_settings.setalloc;

      switchesstatestack[switchesstatestackpos].pmessage:=pendingstate.nextmessagerecord;
      Inc(switchesstatestackpos);
    end;

    procedure dir_rangechecks;
      begin
        do_delphiswitch('R');
      end;

    procedure dir_referenceinfo;
      begin
        do_delphiswitch('Y');
      end;

    procedure dir_resource;
      var
        s : string;
      begin
        current_scanner.skipspace;
        if scanner.c = '''' then
          begin
            s:= current_scanner.readquotedstring;
            current_scanner.readcomment
          end
        else
          s:= trimspace(current_scanner.readcomment);

        { replace * with the name of the main source.
          This should always be defined. }
        if s[1]='*' then
          if Assigned(Current_Module) then
            begin
              delete(S,1,1);
              insert(ChangeFileExt(ExtractFileName(current_module.mainsource),''),S,1 );
            end;
        s:=FixFileName(s);
        if ExtractFileExt(s)='' then
          s:=ChangeFileExt(s,target_info.resext);
        if target_info.res<>res_none then
          begin
            include(current_module.moduleflags,mf_has_resourcefiles);
            if (res_single_file in target_res.resflags) and
                                   not (Current_module.ResourceFiles.Empty) then
              Message(scan_w_only_one_resourcefile_supported)
            else
              current_module.resourcefiles.insert(FixFileName(s));
          end
        else
          Message(scan_e_resourcefiles_not_supported);
      end;

    procedure dir_saturation;
      begin
        do_localswitch(cs_mmx_saturation);
      end;

    procedure dir_safefpuexceptions;
      begin
        do_localswitch(cs_fpu_fwait);
      end;

    procedure dir_scopedenums;
      begin
        do_localswitch(cs_scopedenums);
      end;

    function get_peflag_const(const ident:string;error:longint):longint;
      var
        srsym : tsym;
        srsymtable : tsymtable;
      begin
        result:=0;
        if searchsym(ident,srsym,srsymtable) then
          if (srsym.typ=constsym) and
              (tconstsym(srsym).consttyp=constord) and
              is_integer(tconstsym(srsym).constdef) then
            with tconstsym(srsym).value.valueord do
              if signed then
                result:=tconstsym(srsym).value.valueord.svalue
              else
                result:=tconstsym(srsym).value.valueord.uvalue
          else
            message(error)
        else
          message1(sym_e_id_not_found,ident);
      end;

    procedure dir_setpeflags;
      var
        ident : string;
        flags : int64;
      begin
        if not (target_info.system in (systems_all_windows)) then
          Message(scan_w_setpeflags_not_support);
        if current_scanner.readpreprocint(flags,'SETPEFLAGS') then
          begin
            if flags>$ffff then
              message(scan_e_illegal_peflag);
            peflags:=peflags or uint16(flags);
          end;
        SetPEFlagsSetExplicity:=true;
      end;

    procedure dir_setpeoptflags;
      var
        ident : string;
        flags : int64;
      begin
        if not (target_info.system in (systems_all_windows)) then
          Message(scan_w_setpeoptflags_not_support);
        if current_scanner.readpreprocint(flags,'SETPEOPTFLAGS') then
          begin
            if flags>$ffff then
              message(scan_e_illegal_peoptflag);
            peoptflags:=peoptflags or uint16(flags);
          end;
        SetPEOptFlagsSetExplicity:=true;
      end;

    procedure dir_setpeuserversion;
      var
        dummystr : string;
        dummyrev : word;
      begin
        if not (target_info.system in systems_all_windows) then
          Message(scan_w_setpeuserversion_not_support);
        if (compile_level<>1) then
          Message(scan_n_only_exe_version)
        else
          do_version(peuserversionmajor,peuserversionminor,dummyrev,dummystr,false,SetPEUserVersionSetExplicitely);
      end;

    procedure dir_setpeosversion;
      var
        dummystr : string;
        dummyrev : word;
      begin
        if not (target_info.system in systems_all_windows) then
          Message(scan_w_setpeosversion_not_support);
        if (compile_level<>1) then
          Message(scan_n_only_exe_version)
        else
          do_version(peosversionmajor,peosversionminor,dummyrev,dummystr,false,SetPEOSVersionSetExplicitely);
      end;

    procedure dir_setpesubsysversion;
      var
        dummystr : string;
        dummyrev : word;
      begin
        if not (target_info.system in systems_all_windows) then
          Message(scan_w_setpesubsysversion_not_support);
        if (compile_level<>1) then
          Message(scan_n_only_exe_version)
        else
          do_version(pesubsysversionmajor,pesubsysversionminor,dummyrev,dummystr,false,SetPESubSysVersionSetExplicitely);
      end;

    procedure dir_smartlink;
      begin
        do_moduleswitch(cs_create_smart);
        if (target_dbg.id in [dbg_dwarf2,dbg_dwarf3]) and
            not(target_info.system in (systems_darwin+[system_i8086_msdos,system_i8086_embedded])) and
            { smart linking does not yet work with DWARF debug info on most targets }
            (cs_create_smart in current_settings.moduleswitches) and
            not (af_outputbinary in target_asm.flags) then
        begin
          Message(option_dwarf_smart_linking);
          Exclude(current_settings.moduleswitches,cs_create_smart);
        end;
        { Also create a smartlinked version, on an assembler that
          does not support smartlink sections like nasm?
          This is not compatible with using internal linker. }
       if ((cs_link_smart in current_settings.globalswitches) or
           (cs_create_smart in current_settings.moduleswitches)) and
          (af_needar in target_asm.flags) and
          not (af_smartlink_sections in target_asm.flags) and
          not (cs_link_extern in current_settings.globalswitches) then
         begin
           DoneLinker;
           Message(option_smart_link_requires_external_linker);
           include(current_settings.globalswitches,cs_link_extern);
           InitLinker;
         end
      end;

    procedure dir_stackframes;
      begin
        do_delphiswitch('W');
      end;

    procedure dir_stop;
      begin
        do_message(scan_f_user_defined);
      end;

    procedure dir_stringchecks;
      begin
        // Delphi adds checks that ansistring and unicodestring are correct in
        // different places. Skip it for now.
      end;

    procedure dir_syscall;
      var
        sctype : string;
        syscall : psyscallinfo;
      begin
        current_scanner.skipspace;
        sctype:=current_scanner.readid;

        syscall:=get_syscall_by_name(sctype);
        if assigned(syscall) then
          begin
            if not (target_info.system in syscall^.validon) then
              Message(scan_w_syscall_convention_not_useable_on_target)
            else
              set_default_syscall(syscall^.procoption);
            exit;
          end;
        Message(scan_w_syscall_convention_invalid);
      end;

    procedure dir_targetswitch;
      var
        name, value: string;
      begin
        { note: *not* recorded in the tokenstream, so not replayed for generics }
        current_scanner.skipspace;
        name:=current_scanner.readid;
        if c='=' then
          begin
            current_scanner.readchar;
            current_scanner.readid;
            value:=orgpattern;
            UpdateTargetSwitchStr(name+'='+value,current_settings.targetswitches,current_module.in_global);
          end
        else if c='-' then
          begin
            current_scanner.readchar;
            UpdateTargetSwitchStr(name+'-',current_settings.targetswitches,current_module.in_global);
          end
        else
          UpdateTargetSwitchStr(name,current_settings.targetswitches,current_module.in_global);
      end;

    procedure dir_typedaddress;
      begin
        do_delphiswitch('T');
      end;

    procedure dir_typeinfo;
      begin
        do_delphiswitch('M');
      end;

    procedure dir_unitpath;
      var
        unitpath: TPathStr;
      begin
        if not current_module.in_global then
         Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            unitpath:=current_scanner.readcomment;
            if (current_module.path<>'') and
               not path_absolute(unitpath) then
             unitpath:=current_module.path+source_info.DirSep+unitpath;
            current_module.localunitsearchpath.AddPath(unitpath,false);
          end;
      end;

    procedure dir_varparacopyoutcheck;
      begin
        if not(target_info.system in systems_jvm) then
          begin
            Message1(scan_w_illegal_switch,pattern);
            exit;
          end;
        do_localswitch(cs_check_var_copyout);
      end;

    procedure dir_varpropsetter;
      begin
        do_localswitch(cs_varpropsetter);
      end;

    procedure dir_varstringchecks;
      begin
        do_delphiswitch('V');
      end;

    procedure dir_version;
      var
        major, minor, revision : longint;
        error : integer;
      begin
        if not (target_info.system in systems_all_windows+[system_i386_os2,system_i386_emx,
                 system_i386_netware,system_i386_wdosx,
                 system_i386_netwlibc]) then
          begin
            Message(scan_n_version_not_support);
            exit;
          end;
        if (compile_level<>1) then
          Message(scan_n_only_exe_version)
        else
          begin
            { change description global var in all cases }
            { it not used but in win32, os2 and netware }
            current_scanner.skipspace;
            { we should only accept Major.Minor format for win32 and os2 }
            current_scanner.readnumber;
            major:=0;
            minor:=0;
            revision:=0;
            val(pattern,major,error);
            if (error<>0) or (major > high(word)) or (major < 0) then
              begin
                Message1(scan_w_wrong_version_ignored,pattern);
                exit;
              end;
            if c='.' then
              begin
                current_scanner.readchar;
                current_scanner.readnumber;
                val(pattern,minor,error);
                if (error<>0) or (minor > high(word)) or (minor < 0) then
                  begin
                    Message1(scan_w_wrong_version_ignored,tostr(major)+'.'+pattern);
                    exit;
                  end;
                if (c='.') and
                   (target_info.system in [system_i386_netware,system_i386_netwlibc]) then
                  begin
                     current_scanner.readchar;
                     current_scanner.readnumber;
                     val(pattern,revision,error);
                     if (error<>0) or (revision > high(word)) or (revision < 0) then
                       begin
                          Message1(scan_w_wrong_version_ignored,tostr(revision)+'.'+pattern);
                          exit;
                       end;
                     dllmajor:=word(major);
                     dllminor:=word(minor);
                     dllrevision:=word(revision);
                     dllversion:=tostr(major)+','+tostr(minor)+','+tostr(revision);
                  end
                else
                  begin
                     dllmajor:=word(major);
                     dllminor:=word(minor);
                     dllversion:=tostr(major)+'.'+tostr(minor);
                  end;
              end
            else
              dllversion:=tostr(major);
          end;
      end;

    procedure dir_wait;
      var
        had_info : boolean;
      begin
        had_info:=(status.verbosity and V_Info)<>0;
        { this message should allways appear !! }
        status.verbosity:=status.verbosity or V_Info;
        Message(scan_i_press_enter);
        readln;
        If not(had_info) then
          status.verbosity:=status.verbosity and (not V_Info);
      end;

    { delphi compatible warn directive:
      $warn <identifier> on
      $warn <identifier> off
      $warn <identifier> error
    }
    procedure dir_warn;
      var
        ident : string;
        state : string;
        msgstate : tmsgstate;
        i : integer;
      begin
        current_scanner.skipspace;
        ident:=current_scanner.readid;
        current_scanner.skipspace;
        state:=current_scanner.readid;

        { support both delphi and fpc switches }
        { use local ms_on/off/error tmsgstate values }
        if (state='ON') or (state='+') then
          msgstate:=ms_on
        else
        if (state='OFF') or (state='-') then
          msgstate:=ms_off
        else
        if (state='ERROR') then
          msgstate:=ms_error
        else
        begin
          Message1(scanner_e_illegal_warn_state,state);
          exit;
        end;

        if ident='CONSTRUCTING_ABSTRACT' then
          begin
            recordpendingmessagestate(type_w_instance_with_abstract, msgstate);
            recordpendingmessagestate(type_w_instance_abstract_class, msgstate);
          end
        else
        if ident='IMPLICIT_VARIANTS' then
          recordpendingmessagestate(parser_w_implicit_uses_of_variants_unit, msgstate)
        else
        if ident='NO_RETVAL' then
          recordpendingmessagestate(sym_w_function_result_not_set, msgstate)
        else
        if ident='SYMBOL_DEPRECATED' then
          begin
            recordpendingmessagestate(sym_w_deprecated_symbol, msgstate);
            recordpendingmessagestate(sym_w_deprecated_symbol_with_msg, msgstate);
          end
        else
        if ident='SYMBOL_EXPERIMENTAL' then
          recordpendingmessagestate(sym_w_experimental_symbol, msgstate)
        else
        if ident='SYMBOL_LIBRARY' then
          recordpendingmessagestate(sym_w_library_symbol, msgstate)
        else
        if ident='SYMBOL_PLATFORM' then
          recordpendingmessagestate(sym_w_non_portable_symbol, msgstate)
        else
        if ident='SYMBOL_UNIMPLEMENTED' then
          recordpendingmessagestate(sym_w_non_implemented_symbol, msgstate)
        else
        if ident='UNIT_DEPRECATED' then
          begin
            recordpendingmessagestate(sym_w_deprecated_unit, msgstate);
            recordpendingmessagestate(sym_w_deprecated_unit_with_msg, msgstate);
          end
        else
        if ident='UNIT_EXPERIMENTAL' then
          recordpendingmessagestate(sym_w_experimental_unit, msgstate)
        else
        if ident='UNIT_LIBRARY' then
          recordpendingmessagestate(sym_w_library_unit, msgstate)
        else
        if ident='UNIT_PLATFORM' then
          recordpendingmessagestate(sym_w_non_portable_unit, msgstate)
        else
        if ident='UNIT_UNIMPLEMENTED' then
          recordpendingmessagestate(sym_w_non_implemented_unit, msgstate)
        else
        if ident='ZERO_NIL_COMPAT' then
          recordpendingmessagestate(type_w_zero_to_nil, msgstate)
        else
        if ident='IMPLICIT_STRING_CAST' then
          recordpendingmessagestate(type_w_implicit_string_cast, msgstate)
        else
        if ident='IMPLICIT_STRING_CAST_LOSS' then
          recordpendingmessagestate(type_w_implicit_string_cast_loss, msgstate)
        else
        if ident='EXPLICIT_STRING_CAST' then
          recordpendingmessagestate(type_w_explicit_string_cast, msgstate)
        else
        if ident='EXPLICIT_STRING_CAST_LOSS' then
          recordpendingmessagestate(type_w_explicit_string_cast_loss, msgstate)
        else
        if ident='CVT_NARROWING_STRING_LOST' then
          recordpendingmessagestate(type_w_unicode_data_loss, msgstate)
        else
        if ident='INTF_RAISE_VISIBILITY' then
          recordpendingmessagestate(type_w_interface_lower_visibility, msgstate)
        else
          begin
            i:=0;
            if not ChangeMessageVerbosity(ident,i,msgstate) then
              Message1(scanner_w_illegal_warn_identifier,ident);
          end;
      end;

    procedure dir_warning;
      begin
        do_message(scan_w_user_defined);
      end;

    procedure dir_warnings;
      begin
        do_setverbose('W');
      end;

    procedure dir_weakpackageunit;
      begin
        { old Delphi versions seem to use merely $WEAKPACKAGEUNIT while newer
          Delphis have $WEAPACKAGEUNIT ON... :/ }
        do_moduleflagswitch(mf_package_weak, true);
      end;

    procedure dir_writeableconst;
      begin
        do_delphiswitch('J');
      end;

    procedure dir_z1;
      begin
        current_settings.packenum:=1;
      end;

    procedure dir_z2;
      begin
        current_settings.packenum:=2;
      end;

    procedure dir_z4;
      begin
        current_settings.packenum:=4;
      end;

    procedure dir_externalsym;
      begin
      end;

    procedure dir_nodefine;
      begin
      end;

    procedure dir_hppemit;
      begin
      end;

    procedure dir_hugecode;
      begin
        if not (target_info.system in [system_i8086_msdos,system_i8086_embedded])
{$ifdef i8086}
           or (current_settings.x86memorymodel in x86_near_code_models)
{$endif i8086}
            then
          begin
            Message1(scan_n_ignored_switch,pattern);
            exit;
          end;
        do_moduleswitch(cs_huge_code);
      end;

    procedure dir_hugepointernormalization;
      var
        hs : string;
      begin
        if not (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
          begin
            Message1(scanner_w_directive_ignored_on_target, 'HUGEPOINTERNORMALIZATION');
            exit;
          end;
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        case hs of
          'BORLANDC':
             begin
               recordpendinglocalswitch(cs_hugeptr_arithmetic_normalization,'+');
               recordpendinglocalswitch(cs_hugeptr_comparison_normalization,'+');
             end;
          'MICROSOFTC':
             begin
               recordpendinglocalswitch(cs_hugeptr_arithmetic_normalization,'-');
               recordpendinglocalswitch(cs_hugeptr_comparison_normalization,'-');
             end;
          'WATCOMC':
             begin
               recordpendinglocalswitch(cs_hugeptr_arithmetic_normalization,'-');
               recordpendinglocalswitch(cs_hugeptr_comparison_normalization,'+');
             end;
          else
            Message(scan_e_illegal_hugepointernormalization);
        end;
      end;

    procedure dir_hugepointerarithmeticnormalization;
      begin
        if not (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
          begin
            Message1(scanner_w_directive_ignored_on_target, 'HUGEPOINTERARITHMETICNORMALIZATION');
            exit;
          end;
        do_localswitch(cs_hugeptr_arithmetic_normalization);
      end;

    procedure dir_hugepointercomparisonnormalization;
      begin
        if not (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
          begin
            Message1(scanner_w_directive_ignored_on_target, 'HUGEPOINTERCOMPARISONNORMALIZATION');
            exit;
          end;
        do_localswitch(cs_hugeptr_comparison_normalization);
      end;

    procedure dir_codealign;
      var
        s : string;
      begin
        current_scanner.skipspace;
        s:=current_scanner.readcomment;
        if not(UpdateAlignmentStr(s,current_settings.alignment)) then
          message(scanner_e_illegal_alignment_directive);
      end;

    procedure dir_codepage;
      var
         s : string;
      begin
        if not current_module.in_global then
          Message(scan_w_switch_is_global)
        else
          begin
            current_scanner.skipspace;
            s:=current_scanner.readcomment;
            if (upper(s)='UTF8') or (upper(s)='UTF-8') then
              current_settings.sourcecodepage:=CP_UTF8
            else if not cpavailable(s) then
              Message1(option_code_page_not_available,s)
            else
              current_settings.sourcecodepage:=codepagebyname(s);
            { we're not using the system code page now }
            exclude(current_settings.modeswitches,m_systemcodepage);
            exclude(current_settings.moduleswitches,cs_system_codepage);
            include(current_settings.moduleswitches,cs_explicit_codepage);
          end;
      end;

    procedure dir_coperators;
      begin
        do_moduleswitch(cs_support_c_operators);
      end;


    procedure dir_bitpacking;
      begin
        do_localswitch(cs_bitpacking);
      end;

    procedure dir_region;
      begin
      end;

    procedure dir_endregion;
      begin
      end;

    procedure dir_zerobasesstrings;
      begin
        do_localswitch(cs_zerobasedstrings);
      end;


{****************************************************************************
                         Initialize Directives
****************************************************************************}

    procedure InitScannerDirectives;
      begin
        AddDirective('A1',directive_all, @dir_a1);
        AddDirective('A2',directive_all, @dir_a2);
        AddDirective('A4',directive_all, @dir_a4);
        AddDirective('A8',directive_all, @dir_a8);
        AddDirective('ALIGN',directive_all, @dir_align);
{$ifdef m68k}
        AddDirective('APPID',directive_all, @dir_appid);
        AddDirective('APPNAME',directive_all, @dir_appname);
{$endif m68k}
        AddDirective('APPTYPE',directive_all, @dir_apptype);
        AddDirective('ASMCPU',directive_all, @dir_asmcpu);
        AddDirective('ASMMODE',directive_all, @dir_asmmode);
        AddDirective('ASSERTIONS',directive_all, @dir_assertions);
        AddDirective('BOOLEVAL',directive_all, @dir_booleval);
        AddDirective('BITPACKING',directive_all, @dir_bitpacking);
        AddDirective('CALLING',directive_all, @dir_calling);
        AddDirective('CHECKCASECOVERAGE',directive_all, @dir_checkcasecoverage);
        AddDirective('CHECKFPUEXCEPTIONS',directive_all, @dir_checkfpuexceptions);
        AddDirective('CHECKLOWADDRLOADS',directive_all, @dir_checklowaddrloads);
        AddDirective('CHECKPOINTER',directive_all, @dir_checkpointer);
        AddDirective('CODEALIGN',directive_all, @dir_codealign);
        AddDirective('CODEPAGE',directive_all, @dir_codepage);
        AddDirective('COPERATORS',directive_all, @dir_coperators);
        AddDirective('COPYRIGHT',directive_all, @dir_copyright);
        AddDirective('D',directive_all, @dir_description);
        AddDirective('DEBUGINFO',directive_all, @dir_debuginfo);
        AddDirective('DENYPACKAGEUNIT',directive_all,@dir_denypackageunit);
        AddDirective('DESCRIPTION',directive_all, @dir_description);
        AddDirective('ENDREGION',directive_all, @dir_endregion);
        AddDirective('ERROR',directive_all, @dir_error);
        AddDirective('ERRORC',directive_mac, @dir_error);
        AddDirective('EXCESSPRECISION',directive_all, @dir_excessprecision);
        AddDirective('EXTENDEDSYNTAX',directive_all, @dir_extendedsyntax);
        AddDirective('EXTERNALSYM',directive_all, @dir_externalsym);
        AddDirective('F',directive_all, @dir_forcefarcalls);
        AddDirective('FATAL',directive_all, @dir_fatal);
        AddDirective('FPUTYPE',directive_all, @dir_fputype);
        AddDirective('FRAMEWORKPATH',directive_all, @dir_frameworkpath);
        AddDirective('GOTO',directive_all, @dir_goto);
        AddDirective('HINT',directive_all, @dir_hint);
        AddDirective('HINTS',directive_all, @dir_hints);
        AddDirective('HPPEMIT',directive_all, @dir_hppemit);
        AddDirective('HUGECODE',directive_all, @dir_hugecode);
        AddDirective('HUGEPOINTERNORMALIZATION',directive_all,@dir_hugepointernormalization);
        AddDirective('HUGEPOINTERARITHMETICNORMALIZATION',directive_all,@dir_hugepointerarithmeticnormalization);
        AddDirective('HUGEPOINTERCOMPARISONNORMALIZATION',directive_all,@dir_hugepointercomparisonnormalization);
        AddDirective('IEEEERRORS',directive_all,@dir_ieeeerrors);
        AddDirective('IOCHECKS',directive_all, @dir_iochecks);
        AddDirective('IMAGEBASE',directive_all, @dir_imagebase);
        AddDirective('IMPLICITEXCEPTIONS',directive_all, @dir_implicitexceptions);
        AddDirective('IMPORTEDDATA',directive_all, @dir_importeddata);
        AddDirective('INCLUDEPATH',directive_all, @dir_includepath);
        AddDirective('INFO',directive_all, @dir_info);
        AddDirective('INLINE',directive_all, @dir_inline);
        AddDirective('INTERFACES',directive_all, @dir_interfaces);
        AddDirective('L',directive_all, @dir_link);
        AddDirective('LIBEXPORT',directive_mac, @dir_libexport);
        AddDirective('LIBRARYPATH',directive_all, @dir_librarypath);
        AddDirective('LINK',directive_all, @dir_link);
        AddDirective('LINKFRAMEWORK',directive_all, @dir_linkframework);
        AddDirective('LINKLIB',directive_all, @dir_linklib);
        AddDirective('LOCALSYMBOLS',directive_all, @dir_localsymbols);
        AddDirective('LONGSTRINGS',directive_all, @dir_longstrings);
        AddDirective('M',directive_all, @dir_memory);
        AddDirective('MACRO',directive_all, @dir_macro);
        AddDirective('MAXFPUREGISTERS',directive_all, @dir_maxfpuregisters);
        AddDirective('MAXSTACKSIZE',directive_all, @dir_maxstacksize);
        AddDirective('MEMORY',directive_all, @dir_memory);
        AddDirective('MESSAGE',directive_all, @dir_message);
        AddDirective('MINENUMSIZE',directive_all, @dir_packenum);
        AddDirective('MINFPCONSTPREC',directive_all, @dir_minfpconstprec);
        AddDirective('MINSTACKSIZE',directive_all, @dir_minstacksize);
        AddDirective('MMX',directive_all, @dir_mmx);
        AddDirective('MODE',directive_all, @dir_mode);
        AddDirective('MODESWITCH',directive_all, @dir_modeswitch);
        AddDirective('NAMESPACE',directive_all, @dir_namespace);
        AddDirective('NODEFINE',directive_all, @dir_nodefine);
        AddDirective('NOTE',directive_all, @dir_note);
        AddDirective('NOTES',directive_all, @dir_notes);
        AddDirective('OBJECTCHECKS',directive_all, @dir_objectchecks);
        AddDirective('OBJECTPATH',directive_all, @dir_objectpath);
        AddDirective('OPENSTRINGS',directive_all, @dir_openstrings);
        AddDirective('OPTIMIZATION',directive_all, @dir_optimization);
        AddDirective('OV',directive_mac, @dir_overflowchecks);
        AddDirective('OVERFLOWCHECKS',directive_all, @dir_overflowchecks);
        AddDirective('PACKENUM',directive_all, @dir_packenum);
        AddDirective('PACKRECORDS',directive_all, @dir_packrecords);
        AddDirective('PACKSET',directive_all, @dir_packset);
        AddDirective('PASCALMAINNAME',directive_all, @dir_pascalmainname);
        AddDirective('PIC',directive_all, @dir_pic);
        AddDirective('POINTERMATH',directive_all, @dir_pointermath);
        AddDirective('POP',directive_all, @dir_pop);
        AddDirective('PROFILE',directive_all, @dir_profile);
        AddDirective('PUSH',directive_all, @dir_push);
        AddDirective('R',directive_all, @dir_resource);
        AddDirective('RANGECHECKS',directive_all, @dir_rangechecks);
        AddDirective('REFERENCEINFO',directive_all, @dir_referenceinfo);
        AddDirective('REGION',directive_all, @dir_region);
        AddDirective('RESOURCE',directive_all, @dir_resource);
        AddDirective('SATURATION',directive_all, @dir_saturation);
        AddDirective('SAFEFPUEXCEPTIONS',directive_all, @dir_safefpuexceptions);
        AddDirective('SCOPEDENUMS',directive_all, @dir_scopedenums);
        AddDirective('SETPEFLAGS', directive_all, @dir_setpeflags);
        AddDirective('SETPEOPTFLAGS', directive_all, @dir_setpeoptflags);
        AddDirective('SETPEOSVERSION', directive_all, @dir_setpeosversion);
        AddDirective('SETPEUSERVERSION', directive_all, @dir_setpeuserversion);
        AddDirective('SETPESUBSYSVERSION', directive_all, @dir_setpesubsysversion);
        AddDirective('SCREENNAME',directive_all, @dir_screenname);
        AddDirective('SMARTLINK',directive_all, @dir_smartlink);
        AddDirective('STACKFRAMES',directive_all, @dir_stackframes);
        AddDirective('STOP',directive_all, @dir_stop);
        AddDirective('STRINGCHECKS', directive_all, @dir_stringchecks);
        AddDirective('SYSCALL',directive_all, @dir_syscall);
        AddDirective('TARGETSWITCH',directive_all, @dir_targetswitch);
        AddDirective('THREADNAME',directive_all, @dir_threadname);
        AddDirective('TYPEDADDRESS',directive_all, @dir_typedaddress);
        AddDirective('TYPEINFO',directive_all, @dir_typeinfo);
        AddDirective('UNITPATH',directive_all, @dir_unitpath);
        AddDirective('VARPARACOPYOUTCHECK',directive_all, @dir_varparacopyoutcheck);
        AddDirective('VARPROPSETTER',directive_all, @dir_varpropsetter);
        AddDirective('VARSTRINGCHECKS',directive_all, @dir_varstringchecks);
        AddDirective('VERSION',directive_all, @dir_version);
        AddDirective('WAIT',directive_all, @dir_wait);
        AddDirective('WARN',directive_all, @dir_warn);
        AddDirective('WARNING',directive_all, @dir_warning);
        AddDirective('WARNINGS',directive_all, @dir_warnings);
        AddDirective('WEAKPACKAGEUNIT',directive_all, @dir_weakpackageunit);
        AddDirective('WRITEABLECONST',directive_all, @dir_writeableconst);
        AddDirective('Z1',directive_all, @dir_z1);
        AddDirective('Z2',directive_all, @dir_z2);
        AddDirective('Z4',directive_all, @dir_z4);
        AddDirective('ZEROBASEDSTRINGS',directive_all, @dir_zerobasesstrings);
      end;

end.
