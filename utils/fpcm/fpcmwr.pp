{
    Copyright (c) 2001 by Peter Vreman

    FPCMake - Makefile writer

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$ifdef fpc}{$mode objfpc}{$endif}
{$H+}
unit fpcmwr;
interface

    uses
      sysutils,classes,
      fpcmmain;

    type
      tsections=(sec_none,
        sec_units,sec_exes,sec_loaders,sec_examples,sec_rsts,
        sec_compile,sec_install,
        sec_distinstall,sec_zipinstall,sec_clean,sec_shared,
        sec_command,sec_exts,sec_dirs,sec_tools,sec_info,sec_makefile,
        sec_fpmake
      );

      trules=(
        r_all,r_debug,r_smart,r_release,r_units,
        r_examples,
        r_shared,
        r_install,r_sourceinstall,r_exampleinstall,r_distinstall,
        r_zipinstall,r_zipsourceinstall,r_zipexampleinstall,r_zipdistinstall,
        r_clean,r_distclean,r_cleanall,
        r_info,r_makefiles
      );


    const
      rule2str : array[trules] of string=(
        'all','debug','smart','release','units',
        'examples',
        'shared',
        'install','sourceinstall','exampleinstall','distinstall',
        'zipinstall','zipsourceinstall','zipexampleinstall','zipdistinstall',
        'clean','distclean','cleanall',
        'info','makefiles'
      );

      rule2sec : array[trules] of tsections=(
        sec_compile,sec_compile,sec_compile,sec_compile,sec_compile,
        sec_examples,
        sec_shared,
        sec_install,sec_install,sec_install,sec_distinstall,
        sec_zipinstall,sec_zipinstall,sec_zipinstall,sec_zipinstall,
        sec_clean,sec_clean,sec_clean,
        sec_info,sec_makefile
      );



    type

      { TMakefileWriter }

      TMakefileWriter=class
      private
        FFileName : string;
        FIni    : TFPCMake;
        FInput  : TFPCMake;
        FOutput : TStringList;
        FPhony  : string;
        FHasSection : array[tsections] of boolean;
        procedure LoadFPCMakeIni;
        procedure AddIniSection(const s:string);
        procedure AddCustomSection(const s:string);
        procedure AddTargetVariable(const inivar:string);
        procedure AddVariable(const inivar:string);
        function  AddTargetDefines(const inivar,prefix:string):string;
        procedure AddMainPackage(const pack:string);
        procedure AddRequiredPackages;
        procedure AddTool(const varname,exename,altexename:string);
        procedure AddTools(const inivar:string);
        procedure AddRules;
        procedure AddPhony(const s:string);
        procedure WritePhony;
        procedure AddTargetDirs(const inivar:string);
        procedure AddDefaultTools;
        procedure AddMakefileTargets;
        procedure OptimizeSections;
      public
        constructor Create(AFPCMake:TFPCMake;const AFileName:string);
        destructor  Destroy;override;
        procedure WriteGenericMakefile;
      end;


implementation

{$i fpcmake.inc}

    type
      TMyMemoryStream=class(TMemoryStream)
      public
        constructor Create(p:pointer;mysize:integer);
      end;


{*****************************************************************************
                               Helpers
*****************************************************************************}

    function FixVariable(s:string):string;
      var
        i : integer;
      begin
        Result:=UpperCase(s);
        i:=pos('.',Result);
        if i>0 then
         Result[i]:='_';
      end;


    function VarName(const s:string):string;
      var
        i,j : longint;
      begin
        i:=0;
        result:=s;
        while i<length(result) do
         begin
           inc(i);
           case result[i] of
             '{' :
               begin
                 { this are pkgs which are hold the dirs between the accolades }
                 j:=PosIdx('}',result,i);
                 if j>0 then
                  Delete(result,i,j-i+1)
                 else
                  Delete(result,i,1);
                 dec(i);
               end;
             '$','(',')' :
               begin
                 Delete(result,i,1);
                 dec(i);
               end;
             'a'..'z' :
               result[i]:=chr(ord(result[i])-32);
           end;
         end;
      end;


    procedure FixTab(sl:TStringList);
      var
        i,j,k : integer;
        s,s2  : string;
      begin
        i:=0;
        while (i<sl.Count) do
         begin
           if (sl[i]<>'') and (sl[i][1] in [' ',#9]) then
            begin
              s:=sl[i];
              k:=0;
              j:=0;
              repeat
                inc(j);
                case s[j] of
                  ' ' :
                    inc(k);
                  #9 :
                    k:=(k+7) and not(7);
                  else
                    break;
                end;
              until (j=length(s));
              if k>7 then
               begin
                 s2:='';
                 Delete(s,1,j-1);
                 while (k>7) do
                  begin
                    s2:=s2+#9;
                    dec(k,8);
                  end;
                 while (k>0) do
                  begin
                    s2:=s2+' ';
                    dec(k);
                  end;
                 sl[i]:=s2+s;
               end;
            end;
           inc(i);
         end;
      end;


{*****************************************************************************
                               TMyMemoryStream
*****************************************************************************}

    constructor TMyMemoryStream.Create(p:pointer;mysize:integer);
      begin
        inherited Create;
        SetPointer(p,mysize);
      end;


{*****************************************************************************
                            TMakefileWriter
*****************************************************************************}

    constructor TMakefileWriter.Create(AFPCMake:TFPCMake;const AFileName:string);
      begin
        FInput:=AFPCMake;
        FFileName:=AFileName;
        FOutput:=TStringList.Create;
        FPhony:='';
        FillChar(FHasSection,sizeof(FHasSection),1);
        LoadFPCMakeIni;
      end;


    destructor TMakefileWriter.Destroy;
      begin
        FOutput.Free;
        FIni.Free;
      end;


    procedure TMakefileWriter.LoadFPCMakeIni;
      var
        IniStream : TStream;
      begin
        try
          IniStream:=TMyMemoryStream.Create(@fpcmakeini,sizeof(fpcmakeini));
          FIni:=TFPCMake.CreateFromStream(IniStream,'fpcmake.ini');
          { Leave the '#' comments in the output }
//          FIni.CommentChars:=[';'];
          FIni.LoadSections;
        finally
          IniStream.Destroy;
        end;
      end;


    procedure TMakefileWriter.AddIniSection(const s:string);
      var
        Sec : TFPCMakeSection;
      begin
        Sec:=TFPCMakeSection(FIni[s]);
        if assigned(Sec) then
         FOutput.AddStrings(Sec.List)
        else
         Raise Exception.Create(Format('Section "%s" doesn''t exists in fpcmake.ini',[s]));
      end;


    procedure TMakefileWriter.AddCustomSection(const s:string);
      var
        Sec : TFPCMakeSection;
      begin
        Sec:=TFPCMakeSection(FInput[s]);
        if assigned(Sec) then
         begin
           Sec.BuildMakefile;
           FOutput.AddStrings(Sec.List);
         end;
      end;


    procedure TMakefileWriter.AddTargetVariable(const inivar:string);
      var
        s : string;
        T : TOs;
        C : TCpu;
      begin
        for c:=low(TCpu) to high(TCpu) do
          for t:=low(TOS) to high(TOS) do
            if FInput.IncludeTargets[c,t] then
              begin
                s:=FInput.GetTargetVariable(c,t,IniVar,false);
                if s<>'' then
                  begin
                    FOutput.Add('ifeq ($(FULL_TARGET),'+CPUStr[c]+'-'+OSStr[t]+')');
                    FOutput.Add('override '+FixVariable(IniVar)+'+='+s);
                    FOutput.Add('endif');
                  end;
              end;
      end;


    procedure TMakefileWriter.AddVariable(const inivar:string);
      var
        s : string;
      begin
        s:=FInput.GetVariable(IniVar,false);
        if s<>'' then
         FOutput.Add('override '+FixVariable(IniVar)+'='+s)
      end;


    function TMakefileWriter.AddTargetDefines(const inivar,prefix:string):string;

        procedure addtokens(s:string);
        var
          name : string;
          k1,k2 : integer;
        begin
          repeat
            Name:=GetToken(s,' ');
            if Name='' then
             break;
            { Remove (..) }
            k1:=pos('(',name);
            if k1>0 then
             begin
               k2:=PosIdx(')',name,k1);
               if k2=0 then
                k2:=length(name)+1;
               Delete(Name,k1,k2);
             end;
            FOutput.Add(prefix+VarName(name)+'=1');
            { add to the list of dirs without duplicates }
            AddTokenNoDup(result,name,' ');
          until false;
        end;

      var
        s : string;
        T : TOs;
        C : TCpu;
      begin
        result:='';
        for c:=low(TCpu) to high(TCpu) do
          for t:=low(TOS) to high(TOS) do
            if FInput.IncludeTargets[c,t] then
              begin
                s:=FInput.GetTargetVariable(c,t,IniVar,false);
                if s<>'' then
                  begin
                    FOutput.Add('ifeq ($(FULL_TARGET),'+CpuStr[c]+'-'+OSStr[t]+')');
                    AddTokens(s);
                    FOutput.Add('endif');
                  end;
              end;
      end;


    procedure TMakefileWriter.AddTool(const varname,exename,altexename:string);
      begin
        with FOutput do
         begin
           Add('ifndef '+varname);
           Add(varname+':=$(strip $(wildcard $(addsuffix /'+exename+'$(SRCEXEEXT),$(SEARCHPATH))))');
           if altexename<>'' then
            begin
              Add('ifeq ($('+varname+'),)');
              Add(varname+':=$(strip $(wildcard $(addsuffix /'+altexename+'$(SRCEXEEXT),$(SEARCHPATH))))');
            end;
           Add('ifeq ($('+varname+'),)');
           Add(varname+'= __missing_command_'+varname); {This is to be shure make stops,
              if the command is not found. Otherwise if the command was set to the
              empty string, options to the command would be interpreted as command,
              and because options is preceeded by a "-", make will ignore the error
              that the command is not found.}
           Add('else');
           Add(varname+':=$(firstword $('+varname+'))');
           Add('endif');
           if altexename<>'' then
            begin
              Add('else');
              Add(varname+':=$(firstword $('+varname+'))');
              Add('endif');
            end;
           Add('endif');
           Add('export '+varname);
         end;
      end;


    procedure TMakefileWriter.AddTools(const inivar:string);
      var
        hs,tool : string;
      begin
        hs:=FInput.GetVariable(inivar,false);
        repeat
          Tool:=GetToken(hs,' ');
          if Tool='' then
           break;
          AddTool(FixVariable(Tool),Tool,'');
        until false;
      end;


    procedure TMakefileWriter.AddRules;

        procedure AddRule(rule:trules);
        var
          i : integer;
          hs : string;
          Sec : TFPCMakeSection;
          Rules : TStringList;
        begin
          Sec:=TFPCMakeSection(FInput['rules']);
          if assigned(Sec) then
           begin
             Rules:=Sec.List;
             for i:=0 to Rules.Count-1 do
              begin
                if (length(rules[i])>length(rule2str[rule])) and
                   (rules[i][1]=rule2str[rule][1]) and
                   ((rules[i][length(rule2str[rule])+1]=':') or
                    ((length(rules[i])>length(rule2str[rule])+1) and
                     (rules[i][length(rule2str[rule])+2]=':'))) and
                   (Copy(rules[i],1,length(rule2str[rule]))=rule2str[rule]) then
                  exit;
              end;
           end;
          hs:='';
          if FHasSection[Rule2Sec[rule]] then
           hs:=hs+' fpc_'+rule2str[rule];
          { include target dirs, but not for info and targets that
            call other targets with a only extra settings, if the
            section was not included, then still process the targets }
          if FInput.HasTargetVariable('target_dirs') and
             (not(rule in [r_info,r_shared,r_smart,r_debug,r_release,r_zipdistinstall,r_distinstall]) or
              not FHasSection[Rule2Sec[rule]]) then
           begin
             if FInput.HasVariable('default_dir') then
              hs:=hs+' $(addsuffix _'+rule2str[rule]+',$(DEFAULT_DIR))'
             else
              if not(rule in [r_sourceinstall,r_zipinstall,r_zipsourceinstall,
                              r_makefiles]) or
                 not(FInput.HasVariable('package_name')) then
               hs:=hs+' $(addsuffix _'+rule2str[rule]+',$(TARGET_DIRS))';
           end;
          { include cleaning of example dirs }
          if (rule=r_clean) and
             FInput.HasTargetVariable('target_exampledirs') then
           hs:=hs+' $(addsuffix _'+rule2str[rule]+',$(TARGET_EXAMPLEDIRS))';
          { Add the rule }
          AddPhony(Rule2Str[Rule]);
          FOutput.Add(rule2str[rule]+':'+hs);
        end;

      var
        rule : trules;
      begin
        for rule:=low(trules) to high(trules) do
         AddRule(rule);
        WritePhony;
      end;

    procedure TMakefileWriter.AddPhony(const s:string);
      begin
        FPhony:=FPhony+' '+s;
      end;


    procedure TMakefileWriter.WritePhony;
      begin
        if FPhony<>'' then
         begin
           FOutput.Add('.PHONY:'+FPhony);
           FPhony:='';
         end;
      end;


    procedure TMakefileWriter.AddTargetDirs(const inivar:string);

        procedure AddTargetDir(const s,defpref:string);
        var
          j  : trules;
        begin
          FOutput.Add('ifdef '+defpref+VarName(s));
          for j:=low(trules) to high(trules) do
           begin
             FOutput.Add(s+'_'+rule2str[j]+':');
             FOutput.Add(#9+'$(MAKE) -C '+s+' '+rule2str[j]);
             AddPhony(s+'_'+rule2str[j]);
           end;
          FOutput.Add(s+':');
          FOutput.Add(#9+'$(MAKE) -C '+s+' all');
          AddPhony(s);
          WritePhony;
          FOutput.Add('endif');
        end;

      var
        hs,dir : string;
        prefix : string;
      begin
        prefix:=FixVariable(inivar)+'_';
        hs:=AddTargetDefines(inivar,prefix);
        repeat
          Dir:=GetToken(hs,' ');
          if Dir='' then
           break;
          AddTargetDir(Dir,prefix);
        until false;
      end;


    procedure TMakefileWriter.AddMainPackage(const pack:string);
      var
        packdirvar : string;
      begin
        { create needed variables }
        packdirvar:='PACKAGEDIR_MAIN';
        { Search packagedir by looking for Makefile.fpc }
        FOutput.Add(packdirvar+':=$(firstword $(subst /Makefile.fpc,,$(strip $(wildcard $(addsuffix /'+pack+'/Makefile.fpc,$(PACKAGESDIR))))))');
      end;


    procedure TMakefileWriter.AddRequiredPackages;

        procedure AddPackage(const pack,prefix:string);
        var
          packdirvar,unitdirvar,unitfpmakedirvar : string;
          fpcmadedirvar : string;
        begin
          FOutput.Add('ifdef '+Prefix+VarName(pack));
          { create needed variables }
          packdirvar:='PACKAGEDIR_'+VarName(pack);
          unitdirvar:='UNITDIR_'+VarName(pack);
          unitfpmakedirvar:='UNITDIR_FPMAKE_'+VarName(pack);
          { Search packagedir by looking for Makefile.fpc }
          FOutput.Add(packdirvar+':=$(firstword $(subst /Makefile.fpc,,$(strip $(wildcard $(addsuffix /'+pack+'/Makefile.fpc,$(PACKAGESDIR))))))');
          FOutput.Add('ifneq ($('+packdirvar+'),)');
          { Create unit dir, check if os dependent dir exists }
          FOutput.Add('ifneq ($(wildcard $('+packdirvar+')/units/$(TARGETSUFFIX)),)');
          FOutput.Add(unitdirvar+'=$('+packdirvar+')/units/$(TARGETSUFFIX)');
          FOutput.Add('else');
          FOutput.Add(unitdirvar+'=$('+packdirvar+')');
          FOutput.Add('endif');

          FOutput.Add('ifneq ($(wildcard $('+packdirvar+')/units/$(SOURCESUFFIX)),)');
          FOutput.Add(unitfpmakedirvar+'=$('+packdirvar+')/units/$(SOURCESUFFIX)');
          FOutput.Add('else');

          FOutput.Add('ifneq ($(wildcard $('+packdirvar+')/units_bs/$(SOURCESUFFIX)),)');
          FOutput.Add(unitfpmakedirvar+'=$('+packdirvar+')/units_bs/$(SOURCESUFFIX)');
          FOutput.Add('else');
          FOutput.Add(unitfpmakedirvar+'=$('+packdirvar+')');
          FOutput.Add('endif');

          FOutput.Add('endif');

          FOutput.Add('ifdef CHECKDEPEND');
          { rtl needs special handling for FPCMADE }
          if pack='rtl' then
            fpcmadedirvar:='/$(OS_TARGET)'
          else
            fpcmadedirvar:='';
          FOutput.Add('$('+packdirvar+')'+fpcmadedirvar+'/$(FPCMADE):');
          FOutput.Add(#9'$(MAKE) -C $('+packdirvar+')'+fpcmadedirvar+' $(FPCMADE)');
          FOutput.Add('override ALLDEPENDENCIES+=$('+packdirvar+')'+fpcmadedirvar+'/$(FPCMADE)');
          FOutput.Add('endif');
          { Package dir doesn't exists, check unit dir }
          FOutput.Add('else');
          FOutput.Add(packdirvar+'=');
          FOutput.Add(unitdirvar+':=$(subst /Package.fpc,,$(strip $(wildcard $(addsuffix /'+pack+'/Package.fpc,$(UNITSDIR)))))');
          FOutput.Add('ifneq ($('+unitdirvar+'),)');
          FOutput.Add(unitdirvar+':=$(firstword $('+unitdirvar+'))');
          FOutput.Add('else');
          FOutput.Add(unitdirvar+'=');
          FOutput.Add('endif');
          FOutput.Add('endif');
          { Add Unit dir to the command line -Fu }
          FOutput.Add('ifdef '+unitdirvar);
          FOutput.Add('override COMPILER_UNITDIR+=$('+unitdirvar+')');
          FOutput.Add('endif');

          FOutput.Add('ifdef '+unitfpmakedirvar);
          FOutput.Add('override COMPILER_FPMAKE_UNITDIR+=$('+unitfpmakedirvar+')');
          FOutput.Add('endif');

          { endif for package }
          FOutput.Add('endif');
        end;

      var
        i  : integer;
        reqs,req,prefix : string;
        t : TOS;
        c : TCpu;
        sl : TStringList;
      begin
        prefix:='REQUIRE_PACKAGES_';
        reqs:='';
        { Add target defines }
        for c:=low(tcpu) to high(tcpu) do
          for t:=low(tos) to high(tos) do
            if FInput.IncludeTargets[c,t] then
              begin
                sl:=FInput.GetTargetRequires(c,t);
                { show info }
                FInput.Verbose(FPCMakeInfo,CpuStr[c]+'-'+OSStr[t]+' requires: '+sl.CommaText);
                if sl.count>0 then
                 begin
                   FOutput.Add('ifeq ($(FULL_TARGET),'+CPUStr[c]+'-'+OSStr[t]+')');
                   for i:=0 to sl.count-1 do
                    begin
                      FOutput.Add(prefix+VarName(sl[i])+'=1');
                      AddTokenNoDup(reqs,sl[i],' ');
                    end;
                   FOutput.Add('endif');
                 end;
                sl.Free;
              end;
        { Add all require packages }
        repeat
          req:=GetToken(reqs,' ');
          if Req='' then
           break;
          AddPackage(req,prefix);
        until false;
        WritePhony;
      end;


    procedure TMakefileWriter.AddDefaultTools;
      begin
        AddTool('ECHO','gecho','echo');
        AddTool('DATE','gdate','date');
        AddTool('GINSTALL','ginstall','install');
        AddTool('CPPROG','cp','');
        AddTool('RMPROG','rm','');
        AddTool('MVPROG','mv','');
        AddTool('MKDIRPROG','gmkdir','mkdir');
        AddIniSection('shelltools');
        AddTool('PPUMOVE','ppumove','');
        AddTool('FPCMAKE','fpcmake','');
        AddTool('ZIPPROG','zip','');
        AddTool('TARPROG','gtar','tar');
        AddIniSection('defaulttools');
      end;

    procedure TMakefileWriter.AddMakefileTargets;
      var
        s : string;
        c : TCpu;
        t : Tos;
      begin
        s:='';
        for c:=low(tcpu) to high(tcpu) do
         for t:=low(tos) to high(tos) do
          if FInput.IncludeTargets[c,t] then
           AddToken(s,CpuStr[c]+'-'+OSStr[t],' ');
        FOutput.Add('MAKEFILETARGETS='+s);
      end;

    procedure TMakefileWriter.OptimizeSections;
      var
        SkippedSecs :integer;
      begin
        { Turn some sections off, depending if files are
          provided }
        if not FInput.IsPackage then
         begin
           FHasSection[sec_zipinstall]:=false;
           FHasSection[sec_distinstall]:=false;
         end;

        { Remove unused sections for targets }
        SkippedSecs:=0;
        if (not FInput.HasTargetVariable('target_units')) and (not FInput.HasTargetVariable('target_implicitunits')) then
         begin
           inc(SkippedSecs);
           FHasSection[sec_units]:=false;
         end;
        if (not FInput.HasTargetVariable('target_programs')) then
         begin
           inc(SkippedSecs);
           FHasSection[sec_exes]:=false;
         end;
        if (not FInput.HasTargetVariable('target_examples')) then
         begin
           inc(SkippedSecs);
           { example dirs also requires the fpc_examples target, because
             it also depends on the all target }
           if (not FInput.HasTargetVariable('target_exampledirs')) then
            FHasSection[sec_examples]:=false;
         end;
        if (not FInput.HasTargetVariable('target_loaders')) then
         begin
           inc(SkippedSecs);
           FHasSection[sec_loaders]:=false;
         end;
        if (not FInput.HasTargetVariable('target_fpmake')) then
         begin
           inc(SkippedSecs);
           FHasSection[sec_fpmake]:=false;
         end;
        { if all 5 sections are not available we can skip also the
          generic compile rules }
        if SkippedSecs=5 then
         begin
           FHasSection[sec_shared]:=false;
           FHasSection[sec_compile]:=false;
           if (not FInput.HasTargetVariable('package_name')) and
              (not FInput.HasTargetVariable('install_units')) and
              (not FInput.HasTargetVariable('install_files')) and
              (not FInput.HasTargetVariable('install_createpackagefpc')) then
            FHasSection[sec_install]:=false;
           { Package.fpc also needs to be cleaned }
           if (not FInput.HasTargetVariable('clean_units')) and
              (not FInput.HasTargetVariable('clean_files')) and
              (not FInput.HasTargetVariable('install_createpackagefpc')) then
            FHasSection[sec_clean]:=false;
         end;
      end;


    procedure TMakefileWriter.WriteGenericMakefile;
      begin
        { Remove unused sections }
        OptimizeSections;
        { Generate Output }
        with FOutput do
         begin
           { Header }
           Add('#');
           Add('# Don''t edit, this file is generated by '+TitleNoDate);
           Add('#');
           if FInput.HasVariable('default_rule') then
            Add('default: '+FInput.GetVariable('default_rule',false))
           else
            Add('default: all');
           { Supported targets by this Makefile }
           AddMakefileTargets;
           { Add misc defines }
           AddIniSection('defines');
           { Add automatic detect sections }
           AddIniSection('osdetect');
           { Forced target }
           if FInput.HasVariable('require_target') then
            Add('override OS_TARGET='+FInput.GetVariable('require_target',false))
           else if FInput.HasVariable('default_target') then
            Add('override OS_TARGET_DEFAULT='+FInput.GetVariable('default_target',false));
           if FInput.HasVariable('require_cpu') then
            Add('override CPU_TARGET='+FInput.GetVariable('require_cpu',false))
           else if FInput.HasVariable('default_cpu') then
            Add('override CPU_TARGET_DEFAULT='+FInput.GetVariable('default_cpu',false));
           { FPC Detection }
           AddVariable('default_fpcdir');
           AddIniSection('fpcdetect');
           AddIniSection('fpcdircheckenv');
           AddIniSection('fpcdirdetect');
           AddIniSection('fpmakefpcdetect');
           { Package }
           AddVariable('package_name');
           AddVariable('package_version');
           AddVariable('package_targets');
           { Directory of main package }
           if FInput.HasVariable('package_main') then
             AddMainPackage(FInput.GetVariable('package_main',false));
           { LCL rules }
           if FInput.UsesLCL then
            begin
              AddVariable('default_lcldir');
              AddVariable('lcl_platform');
              AddIniSection('lclrules');
            end;
           { First add the required packages sections }
//           for i:=0 to FInput.RequireList.Count-1 do
//            AddCustomSection(FInput.Requirelist[i]);
           { prerules section }
           if assigned(FInput['prerules']) then
            AddStrings(TFPCMakeSection(FInput['prerules']).List);
           if FHasSection[sec_fpmake] then
            AddIniSection('fpmakeprerules');
           { Default }
           AddVariable('default_dir');
           { Targets }
           AddTargetVariable('target_dirs');
           AddTargetVariable('target_programs');
           AddTargetVariable('target_units');
           AddTargetVariable('target_implicitunits');
           AddTargetVariable('target_loaders');
           AddTargetVariable('target_rsts');
           AddTargetVariable('target_examples');
           AddTargetVariable('target_exampledirs');
           AddTargetVariable('target_fpmake');
           { Clean }
           AddTargetVariable('clean_units');
           AddTargetVariable('clean_files');
           AddTargetVariable('clean_programs');
           { Install }
           AddTargetVariable('install_units');
           AddTargetVariable('install_files');
           AddVariable('install_buildunit');
           AddVariable('install_prefix');
           AddVariable('install_basedir');
           AddVariable('install_datadir');
           AddVariable('install_fpcpackage');
           AddVariable('install_fpcsubdir');
           AddVariable('install_createpackagefpc');
           { Dist }
           AddVariable('dist_destdir');
           AddVariable('dist_zipname');
           AddVariable('dist_ziptarget');
           { Compiler }
           AddTargetVariable('compiler_options');
           AddTargetVariable('compiler_version');
           AddTargetVariable('compiler_includedir');
           AddTargetVariable('compiler_unitdir');
           AddTargetVariable('compiler_sourcedir');
           AddTargetVariable('compiler_objectdir');
           AddTargetVariable('compiler_librarydir');
           AddTargetVariable('compiler_targetdir');
           AddTargetVariable('compiler_unittargetdir');
           { shared }
           AddVariable('shared_build');
           AddVariable('shared_libname');
           AddVariable('shared_libversion');
           AddVariable('shared_libunits');
           AddVariable('shared_build');
           { default Dirs and extensions }
           AddIniSection('defaultdirs');
           if FInput.CheckLibcRequire then
            AddIniSection('dirlibc');
           AddIniSection('extensions');
           { Add default tools }
           AddDefaultTools;
           { Required packages }
           AddVariable('require_packages');
           AddRequiredPackages;
           { commandline }
           AddIniSection('command_begin');
           if FInput.CheckLibcRequire then
            AddIniSection('command_libc');
           AddIniSection('command_end');
           { compile }
           if FHasSection[sec_fpmake] then
            AddIniSection('fpmakerules');
           if FHasSection[sec_loaders] then
            AddIniSection('loaderrules');
           if FHasSection[sec_units] then
            AddIniSection('unitrules');
           if FHasSection[sec_exes] then
            AddIniSection('exerules');
           if FHasSection[sec_rsts] then
            AddIniSection('rstrules');
           if FHasSection[sec_examples] then
            AddIniSection('examplerules');
           if FHasSection[sec_compile] then
            AddIniSection('compilerules');
           if FHasSection[sec_shared] then
             AddIniSection('sharedrules');
           { install }
           if FHasSection[sec_install] then
            AddIniSection('installrules');
           if FHasSection[sec_distinstall] then
            AddIniSection('distinstallrules');
           if FHasSection[sec_zipinstall] then
            AddIniSection('zipinstallrules');
           { clean }
           AddIniSection('cleanrules');
           { info }
           AddIniSection('baseinforules');
           if FInput.UsesLCL then
            AddIniSection('lclinforules');
           AddIniSection('inforules');
           { info }
           AddIniSection('makefilerules');
           { Subdirs }
           AddTargetDirs('target_dirs');
           AddTargetDirs('target_exampledirs');
           { Tools }
           AddTools('require_tools');
           { Rules }
           AddRules;
           { Users own rules }
           AddIniSection('localmakefile');
           AddIniSection('userrules');
           if assigned(FInput['rules']) then
            AddStrings(TFPCMakeSection(FInput['rules']).List);
         end;
        { write to disk }
        FInput.Verbose(FPCMakeInfo,'Writing Makefile');
        Fixtab(FOutput);
        FOutput.SaveToFile(FFileName);
      end;

end.
