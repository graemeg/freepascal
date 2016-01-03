program fppkg;

{$mode objfpc}{$H+}

{$if defined(VER2_2) and (FPC_PATCH<1)}
  {$fatal At least FPC 2.2.1 is required to compile fppkg}
{$endif}

uses
  // General
{$ifdef unix}
  baseunix, cthreads,
{$endif}
  Classes, SysUtils, TypInfo, custapp,
  // Repository handler objects
  fprepos, fpxmlrep,
  pkgmessages, pkgglobals, pkgoptions, pkgrepos,
  // Package Handler components
  pkghandler,pkgmkconv, pkgdownload,
<<<<<<< HEAD
<<<<<<< HEAD
  pkgfpmake, pkgcommands,
  fpmkunit
=======
  pkgfpmake, pkgcommands
>>>>>>> graemeg/fixes_2_2
=======
  pkgfpmake, pkgcommands
>>>>>>> origin/fixes_2_2
  // Downloaders
{$if (defined(unix) and not defined(android)) or defined(windows)}
  ,pkgwget
  ,pkglnet
{$endif}
  ;

Type
  { TMakeTool }

  TMakeTool = Class(TCustomApplication)
  Private
    ParaAction   : string;
    ParaPackages : TStringList;
    procedure MaybeCreateLocalDirs;
    procedure ShowUsage;
  Public
    Constructor Create;
    Destructor Destroy;override;
    Procedure LoadGlobalDefaults;
<<<<<<< HEAD
=======
    Procedure LoadCompilerDefaults;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Procedure ProcessCommandLine(FirstPass: boolean);
    Procedure DoRun; Override;
  end;

  EMakeToolError = Class(Exception);


{ TMakeTool }

<<<<<<< HEAD
=======
function TMakeTool.GetConfigFileName: String;
begin
  if HasOption('C','config-file') then
    Result:=GetOptionValue('C','config-file')
  else
    Result:=GetAppConfigFile(IsSuperUser,False);
end;


>>>>>>> graemeg/fixes_2_2
procedure TMakeTool.LoadGlobalDefaults;
var
  i : integer;
  cfgfile : String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  GeneratedConfig,
  UseGlobalConfig : boolean;
>>>>>>> graemeg/cpstrnew
=======
  GeneratedConfig,
  UseGlobalConfig : boolean;
>>>>>>> graemeg/cpstrnew
=======
  GeneratedConfig,
  UseGlobalConfig : boolean;
>>>>>>> graemeg/cpstrnew
=======
  GeneratedConfig,
  UseGlobalConfig : boolean;
>>>>>>> origin/cpstrnew
begin
  // Default verbosity
  LogLevels:=DefaultLogLevels;
  for i:=1 to ParamCount do
    if (ParamStr(i)='-d') or (ParamStr(i)='--debug') then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        LogLevels:=AllLogLevels+[llDebug];
        break;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // First try config file from command line
  if HasOption('C','config-file') then
    cfgfile:=GetOptionValue('C','config-file')
  else
    cfgfile:='';
  pkgoptions.LoadGlobalDefaults(cfgfile);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  GeneratedConfig:=false;
  UseGlobalConfig:=false;
  // First try config file from command line
  if HasOption('C','config-file') then
    begin
      cfgfile:=GetOptionValue('C','config-file');
      if not FileExists(cfgfile) then
        Error(SErrNoSuchFile,[cfgfile]);
    end
  else
    begin
      // Now try if a local config-file exists
      cfgfile:=GetAppConfigFile(False,False);
      if not FileExists(cfgfile) then
        begin
          // If not, try to find a global configuration file
          cfgfile:=GetAppConfigFile(True,False);
          if FileExists(cfgfile) then
            UseGlobalConfig := true
          else
            begin
              // Create a new configuration file
              if not IsSuperUser then // Make a local, not global, configuration file
                cfgfile:=GetAppConfigFile(False,False);
              ForceDirectories(ExtractFilePath(cfgfile));
              GlobalOptions.SaveGlobalToFile(cfgfile);
              GeneratedConfig:=true;
            end;
        end;
    end;
  // Load file or create new default configuration
  if not GeneratedConfig then
    begin
      GlobalOptions.LoadGlobalFromFile(cfgfile);
      if GlobalOptions.SaveInifileChanges and (not UseGlobalConfig or IsSuperUser) then
        GlobalOptions.SaveGlobalToFile(cfgfile);
    end;
  GlobalOptions.CompilerConfig:=GlobalOptions.DefaultCompilerConfig;
  // Tracing of what we've done above, need to be done after the verbosity is set
  if GeneratedConfig then
    pkgglobals.Log(vlDebug,SLogGeneratingGlobalConfig,[cfgfile])
  else
    pkgglobals.Log(vlDebug,SLogLoadingGlobalConfig,[cfgfile]);
  // Log configuration
  GlobalOptions.LogValues(vlDebug);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
        LogLevels:=AllLogLevels+[vlDebug];
        break;
      end;
  // Load file or create new default configuration
  cfgfile:=GetConfigFileName;
  GeneratedConfig:=false;
=======
        LogLevels:=AllLogLevels+[vlDebug];
        break;
      end;
  // Load file or create new default configuration
  cfgfile:=GetConfigFileName;
  GeneratedConfig:=false;
>>>>>>> origin/fixes_2_2
  if FileExists(cfgfile) then
    begin
      GlobalOptions.LoadGlobalFromFile(cfgfile);
      if GlobalOptions.Dirty then
        GlobalOptions.SaveGlobalToFile(cfgfile);
    end
  else
    begin
      ForceDirectories(ExtractFilePath(cfgfile));
      GlobalOptions.SaveGlobalToFile(cfgfile);
      GeneratedConfig:=true;
    end;
  GlobalOptions.CompilerConfig:=GlobalOptions.DefaultCompilerConfig;
  // Tracing of what we've done above, need to be done after the verbosity is set
  if GeneratedConfig then
    Log(vlDebug,SLogGeneratingGlobalConfig,[cfgfile])
  else
    Log(vlDebug,SLogLoadingGlobalConfig,[cfgfile]);
  // Log configuration
  GlobalOptions.LogValues;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TMakeTool.MaybeCreateLocalDirs;
begin
  ForceDirectories(GlobalOptions.BuildDir);
  ForceDirectories(GlobalOptions.ArchivesDir);
  ForceDirectories(GlobalOptions.CompilerConfigDir);
end;


<<<<<<< HEAD
=======
procedure TMakeTool.LoadCompilerDefaults;
var
  S : String;
begin
  // Load default compiler config
  S:=GlobalOptions.CompilerConfigDir+GlobalOptions.CompilerConfig;
<<<<<<< HEAD
<<<<<<< HEAD
  CompilerOptions.UpdateLocalRepositoryOption;
  if FileExists(S) then
    begin
      pkgglobals.Log(vlDebug,SLogLoadingCompilerConfig,[S]);
=======
  if FileExists(S) then
    begin
      Log(vlDebug,SLogLoadingCompilerConfig,[S]);
>>>>>>> graemeg/fixes_2_2
=======
  if FileExists(S) then
    begin
      Log(vlDebug,SLogLoadingCompilerConfig,[S]);
>>>>>>> origin/fixes_2_2
      CompilerOptions.LoadCompilerFromFile(S)
    end
  else
    begin
      // Generate a default configuration if it doesn't exists
      if GlobalOptions.CompilerConfig='default' then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
          pkgglobals.Log(vlDebug,SLogGeneratingCompilerConfig,[S]);
          CompilerOptions.InitCompilerDefaults;
          CompilerOptions.SaveCompilerToFile(S);
          if CompilerOptions.SaveInifileChanges then
=======
=======
>>>>>>> origin/fixes_2_2
          Log(vlDebug,SLogGeneratingCompilerConfig,[S]);
          CompilerOptions.InitCompilerDefaults;
          CompilerOptions.SaveCompilerToFile(S);
          if CompilerOptions.Dirty then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            CompilerOptions.SaveCompilerToFile(S);
        end
      else
        Error(SErrMissingCompilerConfig,[S]);
    end;
  // Log compiler configuration
<<<<<<< HEAD
<<<<<<< HEAD
  CompilerOptions.LogValues(vlDebug,'');
  // Load FPMake compiler config, this is normally the same config as above
  S:=GlobalOptions.CompilerConfigDir+GlobalOptions.FPMakeCompilerConfig;
  FPMakeCompilerOptions.UpdateLocalRepositoryOption;
  if FileExists(S) then
    begin
      pkgglobals.Log(vlDebug,SLogLoadingFPMakeCompilerConfig,[S]);
      FPMakeCompilerOptions.LoadCompilerFromFile(S);
      if FPMakeCompilerOptions.SaveInifileChanges then
=======
=======
>>>>>>> origin/fixes_2_2
  CompilerOptions.LogValues('');
  // Load FPMake compiler config, this is normally the same config as above
  S:=GlobalOptions.CompilerConfigDir+GlobalOptions.FPMakeCompilerConfig;
  if FileExists(S) then
    begin
      Log(vlDebug,SLogLoadingFPMakeCompilerConfig,[S]);
      FPMakeCompilerOptions.LoadCompilerFromFile(S);
      if FPMakeCompilerOptions.Dirty then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        FPMakeCompilerOptions.SaveCompilerToFile(S);
    end
  else
    Error(SErrMissingCompilerConfig,[S]);
  // Log compiler configuration
<<<<<<< HEAD
<<<<<<< HEAD
  FPMakeCompilerOptions.LogValues(vlDebug,'fpmake-building ');
=======
  FPMakeCompilerOptions.LogValues('fpmake-building ');
>>>>>>> graemeg/fixes_2_2
=======
  FPMakeCompilerOptions.LogValues('fpmake-building ');
>>>>>>> origin/fixes_2_2
end;


>>>>>>> graemeg/cpstrnew
procedure TMakeTool.ShowUsage;
begin
  Writeln('Usage: ',Paramstr(0),' [options] <action> <package>');
  Writeln('Options:');
  Writeln('  -C --config-file   Specify the configuration file to use');
  Writeln('  -c --config        Set compiler configuration to use');
  Writeln('  -h --help          This help');
  Writeln('  -v --verbose       Show more information');
  Writeln('  -d --debug         Show debugging information');
  Writeln('  -g --global        Force installation to global (system-wide) directory');
  Writeln('  -f --force         Force installation also if the package is already installed');
  Writeln('  -r --recovery      Recovery mode, use always internal fpmkunit');
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln('  -b --broken        Do not stop on broken packages');
  Writeln('  -l --showlocation  Show if the packages are installed globally or locally');
  Writeln('  -o --options=value Pass extra options to the compiler');
  Writeln('  -n                 Do not read the default configuration files');
  Writeln('  -p --prefix=value  Specify the prefix');
  Writeln('  -s --skipbroken    Skip the rebuild of depending packages after installation');
  Writeln('  --compiler=value   Specify the compiler-executable');
  Writeln('  --cpu=value        Specify the target cpu to compile for');
  Writeln('  --os=value         Specify the target operating system to compile for');
  Writeln('Actions:');
  Writeln('  update            Update packages list');
  Writeln('  list              List available and installed packages');
  Writeln('  build             Build package');
  Writeln('  compile           Compile package');
  Writeln('  install           Install package');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln('  uninstall         Uninstall package');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Writeln('  clean             Clean package');
  Writeln('  archive           Create archive of package');
  Writeln('  download          Download package');
  Writeln('  convertmk         Convert Makefile.fpc to fpmake.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln('  info              Show more information about a package');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Writeln('  fixbroken         Recompile all (broken) packages with changed dependencies');
  Writeln('  listsettings      Show the values for all fppkg settings');
=======
=======
>>>>>>> origin/fixes_2_2
  Writeln('Actions:');
  Writeln('  update             Update packages list');
  Writeln('  list               List available and installed packages');
  Writeln('  build              Build package');
  Writeln('  compile            Compile package');
  Writeln('  install            Install package');
  Writeln('  clean              Clean package');
  Writeln('  archive            Create archive of package');
  Writeln('  download           Download package');
  Writeln('  convertmk          Convert Makefile.fpc to fpmake.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//  Writeln('  addconfig          Add a compiler configuration for the supplied compiler');
  Halt(0);
end;

Constructor TMakeTool.Create;
begin
  inherited Create(nil);
  ParaPackages:=TStringList.Create;
end;


Destructor TMakeTool.Destroy;
begin
  FreeAndNil(ParaPackages);
  inherited Destroy;
end;


procedure TMakeTool.ProcessCommandLine(FirstPass: boolean);

  Function CheckOption(Index : Integer;Short,Long : String): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O='--'+long) or (copy(O,1,Length(Long)+3)=('--'+long+'='));
  end;

  Function OptionArg(Var Index : Integer) : String;
  Var
    P : Integer;
  begin
    if (Length(ParamStr(Index))>1) and (Paramstr(Index)[2]<>'-') then
      begin
        If Index<ParamCount then
          begin
            Inc(Index);
            Result:=Paramstr(Index);
          end
        else
          Error(SErrNeedArgument,[Index,ParamStr(Index)]);
      end
    else If length(ParamStr(Index))>2 then
      begin
        P:=Pos('=',Paramstr(Index));
        If (P=0) then
          Error(SErrNeedArgument,[Index,ParamStr(Index)])
        else
          begin
            Result:=Paramstr(Index);
            Delete(Result,1,P);
          end;
      end;
  end;

  function SplitSpaces(var SplitString: string) : string;
  var i : integer;
  begin
    i := pos(' ',SplitString);
    if i > 0 then
      begin
        result := copy(SplitString,1,i-1);
        delete(SplitString,1,i);
      end
    else
      begin
        result := SplitString;
        SplitString:='';
      end;
  end;

Var
  I : Integer;
  HasAction : Boolean;
  OptString : String;
begin
  I:=0;
  HasAction:=false;
  // We can't use the TCustomApplication option handling,
  // because they cannot handle [general opts] [command] [cmd-opts] [args]
  While (I<ParamCount) do
    begin
      Inc(I);
      // Check options.
<<<<<<< HEAD
      if CheckOption(I,'C','config-file') then
        begin
          // Do nothing, the config-file has already been read.
          OptionArg(I);
        end
      else if CheckOption(I,'c','config') then
=======
      if CheckOption(I,'c','config') then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        GlobalOptions.CompilerConfig:=OptionArg(I)
      else if CheckOption(I,'v','verbose') then
        LogLevels:=AllLogLevels
      else if CheckOption(I,'d','debug') then
<<<<<<< HEAD
<<<<<<< HEAD
        LogLevels:=AllLogLevels+[llDebug]
=======
        LogLevels:=AllLogLevels+[vlDebug]
>>>>>>> graemeg/fixes_2_2
=======
        LogLevels:=AllLogLevels+[vlDebug]
>>>>>>> origin/fixes_2_2
      else if CheckOption(I,'g','global') then
        GlobalOptions.InstallGlobal:=true
      else if CheckOption(I,'r','recovery') then
        GlobalOptions.RecoveryMode:=true
<<<<<<< HEAD
<<<<<<< HEAD
      else if CheckOption(I,'n','') then
        GlobalOptions.SkipConfigurationFiles:=true
      else if CheckOption(I,'b','broken') then
        GlobalOptions.AllowBroken:=true
      else if CheckOption(I,'l','showlocation') then
        GlobalOptions.ShowLocation:=true
      else if CheckOption(I,'s','skipbroken') then
        GlobalOptions.SkipFixBrokenAfterInstall:=true
      else if CheckOption(I,'o','options') and FirstPass then
        begin
          OptString := OptionArg(I);
          while OptString <> '' do
            CompilerOptions.Options.Add(SplitSpaces(OptString));
        end
      else if CheckOption(I,'p','prefix') then
        begin
          CompilerOptions.GlobalPrefix := OptionArg(I);
          CompilerOptions.LocalPrefix := OptionArg(I);
          FPMakeCompilerOptions.GlobalPrefix := OptionArg(I);
          FPMakeCompilerOptions.LocalPrefix := OptionArg(I);
        end
      else if CheckOption(I,'','compiler') then
        begin
          CompilerOptions.Compiler := OptionArg(I);
          FPMakeCompilerOptions.Compiler := OptionArg(I);
        end
      else if CheckOption(I,'','os') then
        CompilerOptions.CompilerOS := StringToOS(OptionArg(I))
      else if CheckOption(I,'','cpu') then
        CompilerOptions.CompilerCPU := StringToCPU(OptionArg(I))
=======
      else if CheckOption(I,'b','broken') then
        GlobalOptions.AllowBroken:=true
>>>>>>> graemeg/fixes_2_2
=======
      else if CheckOption(I,'b','broken') then
        GlobalOptions.AllowBroken:=true
>>>>>>> origin/fixes_2_2
      else if CheckOption(I,'h','help') then
        begin
          ShowUsage;
          halt(0);
        end
      else if (Length(Paramstr(i))>0) and (Paramstr(I)[1]='-') then
        begin
          if FirstPass then
            Raise EMakeToolError.CreateFmt(SErrInvalidArgument,[I,ParamStr(i)])
        end
      else
      // It's a command or target.
        begin
          if HasAction then
            begin
              if FirstPass then
                ParaPackages.Add(Paramstr(i))
            end
          else
            begin
              ParaAction:=Paramstr(i);
              HasAction:=true;
            end;
        end;
    end;
  if not HasAction then
    ShowUsage;
end;


procedure TMakeTool.DoRun;
var
  ActionPackage : TFPPackage;
  OldCurrDir : String;
  i      : Integer;
  SL     : TStringList;
begin
  OldCurrDir:=GetCurrentDir;
  Try
    LoadGlobalDefaults;
<<<<<<< HEAD
<<<<<<< HEAD
    ProcessCommandLine(true);
=======
    ProcessCommandLine;
>>>>>>> graemeg/fixes_2_2
=======
    ProcessCommandLine;
>>>>>>> origin/fixes_2_2

    // Scan is special, it doesn't need a valid local setup
    if (ParaAction='scan') then
      begin
        RebuildRemoteRepository;
        ListRemoteRepository;
        SaveRemoteRepository;
        halt(0);
      end;

    MaybeCreateLocalDirs;
<<<<<<< HEAD
    if not GlobalOptions.SkipConfigurationFiles then
      LoadCompilerDefaults
    else
      begin
        FPMakeCompilerOptions.InitCompilerDefaults;
        CompilerOptions.InitCompilerDefaults;
      end;

    // The command-line is parsed for the second time, to make it possible
    // to override the values in the compiler-configuration file. (like prefix)
    ProcessCommandLine(false);

    // If CompilerVersion, CompilerOS or CompilerCPU is still empty, use the
    // compiler-executable to get them
    FPMakeCompilerOptions.CheckCompilerValues;
    CompilerOptions.CheckCompilerValues;

=======
    LoadCompilerDefaults;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    LoadLocalAvailableMirrors;

    // Load local repository, update first if this is a new installation
    // errors will only be reported as warning. The user can be bootstrapping
    // and do an update later
    if not FileExists(GlobalOptions.LocalPackagesFile) then
      begin
        try
          pkghandler.ExecuteAction('','update');
        except
          on E: Exception do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            pkgglobals.Log(llWarning,E.Message);
=======
            pkgglobals.Log(vlWarning,E.Message);
>>>>>>> graemeg/cpstrnew
=======
            pkgglobals.Log(vlWarning,E.Message);
>>>>>>> graemeg/cpstrnew
=======
            pkgglobals.Log(vlWarning,E.Message);
>>>>>>> graemeg/cpstrnew
=======
            pkgglobals.Log(vlWarning,E.Message);
>>>>>>> origin/cpstrnew
=======
            Log(vlWarning,E.Message);
>>>>>>> graemeg/fixes_2_2
=======
            Log(vlWarning,E.Message);
>>>>>>> origin/fixes_2_2
        end;
      end;
    LoadLocalAvailableRepository;
    FindInstalledPackages(FPMakeCompilerOptions,true);
    CheckFPMakeDependencies;
    // We only need to reload the status when we use a different
<<<<<<< HEAD
<<<<<<< HEAD
    // configuration for compiling fpmake or when the CPU, OS or compiler
    // are set in the command-line
    if (GlobalOptions.CompilerConfig<>GlobalOptions.FPMakeCompilerConfig) or
       (CompilerOptions.CompilerCPU<>FPMakeCompilerOptions.CompilerCPU) or
       (CompilerOptions.CompilerOS<>FPMakeCompilerOptions.CompilerOS) or
       (CompilerOptions.Compiler<>FPMakeCompilerOptions.Compiler) then
=======
    // configuration for compiling fpmake
    if GlobalOptions.CompilerConfig<>GlobalOptions.FPMakeCompilerConfig then
>>>>>>> graemeg/fixes_2_2
=======
    // configuration for compiling fpmake
    if GlobalOptions.CompilerConfig<>GlobalOptions.FPMakeCompilerConfig then
>>>>>>> origin/fixes_2_2
      FindInstalledPackages(CompilerOptions,true);

    // Check for broken dependencies
    if not GlobalOptions.AllowBroken and
<<<<<<< HEAD
<<<<<<< HEAD
       (((ParaAction='fixbroken') and (ParaPackages.Count>0)) or
        (ParaAction='compile') or
        (ParaAction='build') or
        (ParaAction='install') or
        (ParaAction='archive')) then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pkgglobals.Log(llDebug,SLogCheckBrokenDependenvies);
=======
        pkgglobals.Log(vlDebug,SLogCheckBrokenDependenvies);
>>>>>>> graemeg/cpstrnew
=======
        pkgglobals.Log(vlDebug,SLogCheckBrokenDependenvies);
>>>>>>> graemeg/cpstrnew
=======
        pkgglobals.Log(vlDebug,SLogCheckBrokenDependenvies);
>>>>>>> graemeg/cpstrnew
=======
        pkgglobals.Log(vlDebug,SLogCheckBrokenDependenvies);
>>>>>>> origin/cpstrnew
=======
       not((ParaPackages.Count=0) and (ParaAction='fixbroken')) then
      begin
>>>>>>> graemeg/fixes_2_2
=======
       not((ParaPackages.Count=0) and (ParaAction='fixbroken')) then
      begin
>>>>>>> origin/fixes_2_2
        SL:=TStringList.Create;
        if FindBrokenPackages(SL) then
          Error(SErrBrokenPackagesFound);
        FreeAndNil(SL);
      end;

    if ParaPackages.Count=0 then
      begin
        ActionPackage:=AvailableRepository.AddPackage(CurrentDirPackageName);
        pkghandler.ExecuteAction(CurrentDirPackageName,ParaAction);
      end
    else
      begin
        // Process packages
        for i:=0 to ParaPackages.Count-1 do
          begin
<<<<<<< HEAD
<<<<<<< HEAD
            if sametext(ExtractFileExt(ParaPackages[i]),'.zip') and FileExists(ParaPackages[i]) then
=======
            if FileExists(ParaPackages[i]) then
>>>>>>> graemeg/fixes_2_2
=======
            if FileExists(ParaPackages[i]) then
>>>>>>> origin/fixes_2_2
              begin
                ActionPackage:=AvailableRepository.AddPackage(CmdLinePackageName);
                ActionPackage.LocalFileName:=ExpandFileName(ParaPackages[i]);
                pkghandler.ExecuteAction(CmdLinePackageName,ParaAction);
              end
            else
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                pkgglobals.Log(llDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
=======
                pkgglobals.Log(vlDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
>>>>>>> graemeg/cpstrnew
=======
                pkgglobals.Log(vlDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
>>>>>>> graemeg/cpstrnew
=======
                pkgglobals.Log(vlDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
>>>>>>> graemeg/cpstrnew
=======
                pkgglobals.Log(vlDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
>>>>>>> origin/cpstrnew
=======
                Log(vlDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
>>>>>>> graemeg/fixes_2_2
=======
                Log(vlDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
>>>>>>> origin/fixes_2_2
                pkghandler.ExecuteAction(ParaPackages[i],ParaAction);
              end;
          end;
      end;

    // Recompile all packages dependent on this package
<<<<<<< HEAD
<<<<<<< HEAD
    if (ParaAction='install') and not GlobalOptions.SkipFixBrokenAfterInstall then
=======
    if (ParaAction='install') then
>>>>>>> graemeg/fixes_2_2
=======
    if (ParaAction='install') then
>>>>>>> origin/fixes_2_2
      pkghandler.ExecuteAction('','fixbroken');

    Terminate;

  except
    On E : Exception do
      begin
        Writeln(StdErr,SErrException);
        Writeln(StdErr,E.Message);
        Halt(1);
      end;
  end;
  SetCurrentDir(OldCurrDir);
end;


begin
  With TMakeTool.Create do
    try
      run;
    finally
      Free;
    end;
end.

